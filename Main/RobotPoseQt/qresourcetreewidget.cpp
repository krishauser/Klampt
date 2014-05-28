#include "qresourcetreewidget.h"
#include <QMessageBox>
#include <QApplication>
#include <QHeaderView>
#include <QDebug>
#include <QStyledItemDelegate>

class NoEditDelegate: public QStyledItemDelegate {
    public:
      NoEditDelegate(QObject* parent=0): QStyledItemDelegate(parent) {}
      virtual QWidget* createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const {
        return 0;
      }
    };

QResourceTreeWidget::QResourceTreeWidget(QWidget* parent)
  :QTreeWidget(parent)
{
  setSelectionMode(QAbstractItemView::SingleSelection);
  setDragEnabled(true);
  viewport()->setAcceptDrops(true);
  setDropIndicatorShown(true);
  setDragDropMode(QAbstractItemView::InternalMove);
  header()->resizeSection ( 1, 80 );
  header()->resizeSection ( 2, 20 );
  //disallow editing for columns 1 and 2
  setItemDelegateForColumn(1,new NoEditDelegate);
  setItemDelegateForColumn(2,new NoEditDelegate);

  dragNode = NULL;
}

void QResourceTreeWidget::setManager(ResourceManager* _manager)
{
  manager = _manager;
  refresh();
}

void QResourceTreeWidget::refresh()
{
  while(topLevelItemCount() > 0)
    delete topLevelItem(0);
  for(size_t i=0;i<manager->topLevel.size();i++) {
    QResourceTreeItem* res=new QResourceTreeItem(manager->topLevel[i]);
    addTopLevelItem(res);
  }
}

void QResourceTreeWidget::addNotify(ResourceNodePtr node)
{
  //parse the path
  vector<ResourceNode*> path;
  ResourceNode* n=node;
  while(n != NULL) {
    path.push_back(n);
    n = n->parent;
  }
  reverse(path.begin(),path.end());
  if(path.size()==1)
    //add to top-level items
    insertTopLevelItem(manager->ChildIndex(node),new QResourceTreeItem(node));
  else {
    QTreeWidgetItem* item = invisibleRootItem();
    for(size_t i=0;i+1<path.size();i++) {
      int row = manager->ChildIndex(path[i]);
      if(row < 0) {
	fprintf(stderr,"Error finding %d'th path item %s\n",i,path[i]->Name());
	return;
      }
      item = item->child(row);
    }
    item->insertChild(manager->ChildIndex(node),new QResourceTreeItem(node));
  }
}

void QResourceTreeWidget::onAdd(ResourcePtr r)
{
  ResourceNodePtr n=manager->Add(r);
  addNotify(n);
}

void QResourceTreeWidget::onDeletePressed()
{
  QTreeWidgetItem* item = currentItem();
  if(item) {
    onDelete(item);
  }
}

void QResourceTreeWidget::onDelete(QTreeWidgetItem* n)
{
  assert(n!=NULL);
  QResourceTreeItem* rn = dynamic_cast<QResourceTreeItem*>(n);
  assert(rn!=NULL);
  manager->Delete(rn->resource);
  if(n->parent() != NULL) {
    QTreeWidgetItem* p = n->parent();
    QResourceTreeItem* rp = dynamic_cast<QResourceTreeItem*>(p);
    assert(rp!=NULL);
    rp->updateDecorator();
  }
  delete rn;
}

void QResourceTreeWidget::onExpand(QTreeWidgetItem* n)
{
  QResourceTreeItem* rn = dynamic_cast<QResourceTreeItem*>(n);
  assert(n!=NULL);
  if(rn->resource && !rn->resource->IsExpanded()) {
    rn->resource->Expand();
    for(size_t i=0;i<rn->resource->children.size();i++)
      n->addChild(new QResourceTreeItem(rn->resource->children[i]));
  }
}

void QResourceTreeWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        startPos = event->pos();
    }
 
    QTreeWidget::mousePressEvent(event);
}
 
void QResourceTreeWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (event->buttons() & Qt::LeftButton) {
        int distance = (event->pos() - startPos).manhattanLength();
        if (distance >= QApplication::startDragDistance()) {
	  QTreeWidgetItem *item = currentItem();
	  dragNode = dynamic_cast<QResourceTreeItem*>(item)->resource;
	}
    }
 
    QTreeWidget::mouseMoveEvent(event);
}
 
void QResourceTreeWidget::dragEnterEvent(QDragEnterEvent *event)
{
  QTreeWidget::dragEnterEvent(event);
}
 
void QResourceTreeWidget::dragMoveEvent(QDragMoveEvent *event)
{
  QTreeWidget::dragMoveEvent(event);
}
 
void QResourceTreeWidget::dragLeaveEvent(QDragLeaveEvent *event)
{
  QTreeWidget::dragLeaveEvent(event);
}

QResourceTreeItem* QResourceTreeWidget::nodeToItem(ResourceNode* n)
{
  vector<ResourceNode*> path;
  while(n != NULL) {
    path.push_back(n);
    n = n->parent;
  }
  reverse(path.begin(),path.end());
  QTreeWidgetItem* item = invisibleRootItem();
  for(size_t i=0;i<path.size();i++) {
    int row = manager->ChildIndex(path[i]);
    if(row < 0 || row >= item->childCount()) {
      fprintf(stderr,"Error finding %d'th path item %s\n",i,path[i]->Name());
      return NULL;
    }
    item = item->child(row);
  }
  return dynamic_cast<QResourceTreeItem*>(item);
}

ResourceNodePtr QResourceTreeWidget::itemToNode(QTreeWidgetItem* node)
{
  QResourceTreeItem* qnode = dynamic_cast<QResourceTreeItem*>(node);
  if(!qnode) return NULL;
  return qnode->resource;
}

void updateDecoratorsRecurse(QTreeWidgetItem* node)
{
  QResourceTreeItem* qnode = dynamic_cast<QResourceTreeItem*>(node);
  if(qnode) qnode->updateDecorator();
  else printf("Unable to cast to QResourceTreeItem\n");
  for(int i=0;i<node->childCount();i++)
    updateDecoratorsRecurse(node->child(i));
}

void QResourceTreeWidget::updateAllDecorators()
{
  for(int i=0;i<topLevelItemCount();i++)
    updateDecoratorsRecurse(topLevelItem(i));
}

void QResourceTreeWidget::dropEvent(QDropEvent *event)
{
    //MimeData: application/x-qabstractitemmodeldatalist"
 
    assert(dragNode != NULL);
    cout<<"Dragged item: "<<dragNode->Identifier()<<endl;

    QResourceTreeItem* dragParent = NULL;
    if(dragNode->parent) dragParent = nodeToItem(dragNode->parent);

    QTreeWidgetItem* target = this->itemAt(event->pos());
    ResourceNodePtr targetNode;
    if(target) {
      QResourceTreeItem* qtarget = dynamic_cast<QResourceTreeItem*>(target);
      assert(qtarget != NULL);
      targetNode = qtarget->resource;
    }
    if(event->dropAction() == Qt::MoveAction) {
      QTreeWidgetItem* targetParent = target;
      ResourceNode* targetNodeParent = targetNode;
      int insertIndex = -1;  //default: insert at end
      if(dropIndicatorPosition() == OnItem) {
	if(!target) {
	  targetParent = invisibleRootItem();
	  targetNodeParent = NULL;
	}
      }
      else if(dropIndicatorPosition() == AboveItem) {
	targetParent = target->parent();
	targetNodeParent = targetNode->parent;
	insertIndex = manager->ChildIndex(targetNode);
      }
      else if(dropIndicatorPosition() == BelowItem) {
	targetParent = target->parent();
	targetNodeParent = targetNode->parent;
	insertIndex = manager->ChildIndex(targetNode)+1;
      }
      else if(dropIndicatorPosition() == OnViewport) {
	targetParent = invisibleRootItem();
	targetNodeParent = NULL;
      }
      else {
	printf("Invalid dropIndicatorPosition? %d\n",(int)dropIndicatorPosition());
	return;
      }
      cout<<"Dragged parent: "<<(dragNode->parent ? dragNode->parent->Name() : "Root" )<<endl;
      cout<<"Target parent: "<<(targetNodeParent ? targetNodeParent->Name() : "Root")<<endl;
      cout<<"Insert position: "<<insertIndex<<endl;

      Assert(dragNode != targetNodeParent);
      printf("Deleting from drag parent\n");
      //detatch dragNode from parent and put it before, after, or in targetNode
      manager->Delete(dragNode);
      if(dragParent) {
	printf("Updating drag parent decorator\n");
	dragParent->updateDecorator();
      }

      printf("Adding to target parent\n");
      if(targetNodeParent) {
	if(!targetNodeParent->IsExpandable()) return;
	if(!targetNodeParent->IsExpanded())
	  targetNodeParent->Expand();
	dragNode->parent = targetNodeParent;
	if(insertIndex < 0)
	  targetNodeParent->children.push_back(dragNode);
	else
	  targetNodeParent->children.insert(targetNodeParent->children.begin()+insertIndex,dragNode);
	targetNodeParent->SetChildrenChanged();
	dynamic_cast<QResourceTreeItem*>(targetParent)->updateDecorator();
      }
      else {
	dragNode->parent = NULL;
	if(insertIndex < 0) {
	  manager->topLevel.push_back(dragNode);
	  manager->library.Add(dragNode->resource);
	}
	else {
	  manager->topLevel.insert(manager->topLevel.begin()+insertIndex,dragNode);
	  manager->library.Add(dragNode->resource);
	}
      }
    }
    else {
      printf("Unable to copy yet\n");
      return;
    }
    cout<<"After drag: new resource tree: "<<endl;
    manager->Print();
    cout<<endl;

    QTreeWidget::dropEvent(event);

    //for some reason QTreeWidget takes away the child indicator
    //when all children are deleted
    if(dragParent) dragParent->setChildIndicatorPolicy(QTreeWidgetItem::ShowIndicator);

    //select the dropped item
    if(dropIndicatorPosition() == AboveItem) {
      setCurrentItem(itemAbove(target));
    }
    else if(dropIndicatorPosition() == BelowItem) {
      setCurrentItem(itemBelow(target));
    }
    else if(dropIndicatorPosition() == OnViewport) {
      setCurrentItem(topLevelItem(topLevelItemCount()-1));
    }
    /*
    const QMimeData* qMimeData = event->mimeData();
 
    QByteArray encoded = qMimeData->data("application/x-qabstractitemmodeldatalist");
    QDataStream stream(&encoded, QIODevice::ReadOnly);
 
    while (!stream.atEnd())
    {
        int row, col;
        QMap<int,  QVariant> roleDataMap;
 
        stream >> row >> col >> roleDataMap;
 
	if(col == NAMECOL) {
	  cout<<"RoleDataMap:"<<endl;
	  QMapIterator<int, QVariant> i(roleDataMap);
	  while (i.hasNext()) {
	    i.next();
	    QString value = i.value().toString();
	    cout << "  " << i.key() << ": " << value.toStdString() << endl;
	  }
	  QString dropped = roleDataMap[0].toString();

	  QString into = (target ? target->text(NAMECOL) : "root");
	  
	  QMessageBox::information(this, "", "DROPPING:" + dropped + "\nINTO:" +
				   into);
	}
    }
    */
}


void QResourceTreeWidget::keyPressEvent(QKeyEvent * event)
{
  int key = event->key();
  if(key == Qt::Key_Delete || Qt::Key_Backspace)
    onDeletePressed();
  else
    QTreeWidget::keyPressEvent(event);
}
