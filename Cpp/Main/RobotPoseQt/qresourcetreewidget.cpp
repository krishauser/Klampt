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
  header()->resizeSection ( 1, 300 );
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
    QTreeWidgetItem* res=makeItem(manager->topLevel[i].get());
    addTopLevelItem(res);
  }
}

QTreeWidgetItem* QResourceTreeWidget::makeItem(ResourceNode* rt)
{
  QTreeWidgetItem* item = new QTreeWidgetItem;
  item->setData(NAMECOL,Qt::UserRole,qVariantFromValue((void*)rt));
    string name=rt->Name();
    item->setText(NAMECOL,QString::fromStdString(name));
    string type = rt->resource->Type();
    if(!type.empty())
        item->setText(TYPECOL,QString::fromStdString(rt->resource->Type()));

    updateProperties(item);
    updateDecorator(item);

    for(size_t i=0;i<rt->children.size();i++) {
      QTreeWidgetItem* c = makeItem(rt->children[i].get());
      item->addChild(c);
    }
    return item;
}

void QResourceTreeWidget::updateProperties(QTreeWidgetItem* item)
{
  ResourceNode* resource = itemToNode(item);
  if(resource->IsExpandable()) {
    item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable | Qt::ItemIsSelectable | Qt::ItemIsDropEnabled | Qt::ItemIsDragEnabled);
    item->setChildIndicatorPolicy(QTreeWidgetItem::ShowIndicator);
  }
  else {
    item->setChildIndicatorPolicy(QTreeWidgetItem::DontShowIndicator);
    item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable | Qt::ItemIsSelectable | Qt::ItemIsDragEnabled);
  }
}

void QResourceTreeWidget::updateDecorator(QTreeWidgetItem* item)
{
  if(!item) return;
  ResourceNode* resource = itemToNode(item);
  item->setText(DECORATORCOL,QString(resource->Decorator()));
}


void QResourceTreeWidget::addNotify(ResourceNode* node)
{
  //parse the path
  vector<ResourceNode*> path;
  ResourceNode* n=node;
  while(n != NULL) {
    path.push_back(n);
    n = n->parent;
  }
  reverse(path.begin(),path.end());

  QTreeWidgetItem* newitem = makeItem(node);
  if(path.size()==1) {
    //add to top-level items
    insertTopLevelItem(manager->ChildIndex(node),newitem);
  }
  else {
    QTreeWidgetItem* item = invisibleRootItem();
    for(size_t i=0;i+1<path.size();i++) {
      int row = manager->ChildIndex(path[i]);
      if(row < 0) {
        fprintf(stderr,"Error finding %d'th path item %s\n",i,path[i]->Name());
        delete newitem;
        return;
      }
      item = item->child(row);
    }
    item->insertChild(manager->ChildIndex(node),newitem);
  }
}

void QResourceTreeWidget::onAdd(ResourcePtr r)
{
  ResourceNodePtr n=manager->Add(r);
  addNotify(n.get());
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
  ResourceNode* r = itemToNode(n);
  assert(r!=NULL);
  assert(r->resource!=NULL);
  manager->Delete(r);
  if(n->parent() != NULL) {
    QTreeWidgetItem* p = n->parent();
    updateDecorator(p);
  }
  delete n;
}

void QResourceTreeWidget::onExpand(QTreeWidgetItem* n)
{
  ResourceNode* r = itemToNode(n);
  assert(r!=NULL);
  if(!r->IsExpanded()) {
    r->Expand();
    for(size_t i=0;i<r->children.size();i++)
      n->addChild(makeItem(r->children[i].get()));
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
          //need to convert raw pointer to a smart pointer
          ResourceNode * nodePtr = itemToNode(item);
          int index = manager->ChildIndex(nodePtr);
          if(nodePtr->parent)
            dragNode = nodePtr->parent->children[index];
          else
            dragNode = manager->topLevel[index];
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

QTreeWidgetItem* QResourceTreeWidget::nodeToItem(ResourceNode* n)
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
  return item;
}

ResourceNode* QResourceTreeWidget::itemToNode(QTreeWidgetItem* node)
{
  if(!node) return NULL;
  QVariant data = node->data(NAMECOL,Qt::UserRole);
  return (ResourceNode*)data.value<void*>();
}

void updateDecoratorsRecurse(QResourceTreeWidget* w,QTreeWidgetItem* node)
{
  w->updateDecorator(node);
  for(int i=0;i<node->childCount();i++)
    updateDecoratorsRecurse(w,node->child(i));
}

void QResourceTreeWidget::updateAllDecorators()
{
  for(int i=0;i<topLevelItemCount();i++)
    updateDecoratorsRecurse(this,topLevelItem(i));
}

void QResourceTreeWidget::dropEvent(QDropEvent *event)
{
    //MimeData: application/x-qabstractitemmodeldatalist"
 
    assert(dragNode != NULL);
    cout<<"Dragged item: "<<dragNode->Identifier()<<endl;

    QTreeWidgetItem* dragParent = NULL;
    if(dragNode->parent) dragParent = nodeToItem(dragNode->parent);

    QTreeWidgetItem* target = this->itemAt(event->pos());
    ResourceNode* targetNode;
    if(target) {
      targetNode = itemToNode(target);
    }
    if(event->dropAction() == Qt::MoveAction || event->dropAction() == Qt::CopyAction) {
      QTreeWidgetItem* targetParent = target;
      ResourceNode* targetNodeParent = targetNode;
      int dragIndex = manager->ChildIndex(dragNode.get());
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

      Assert(dragNode.get() != targetNodeParent);
      if(event->dropAction() == Qt::MoveAction) {
        printf("Deleting from drag parent\n");
        //detatch dragNode from parent and put it before, after, or in targetNode
        manager->Delete(dragNode.get());
        if(dragParent) {
          printf("Updating drag parent decorator\n");
          updateDecorator(dragParent);
        }
        if(dragParent==targetParent) {
          //need to adjust insertion index
          if(dragIndex < insertIndex)
            insertIndex--;
        }
      }
      else {
        //copying -- make a copy
        printf("Copying resource\n");
        ResourcePtr rcopy(dragNode->resource->Copy());
        rcopy->name = dragNode->resource->name;
        rcopy->fileName = dragNode->resource->fileName;
        dragNode = make_shared<ResourceNode>(rcopy);
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
        updateDecorator(targetParent);
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

    if(event->dropAction() == Qt::CopyAction) {
      //item can't load data pointer type, we need to set new pointer for the 
      //item
      addNotify(dragNode.get());
    }
    else 
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
  if(key == Qt::Key_Delete || key == Qt::Key_Backspace)
    onDeletePressed();
  else
    QTreeWidget::keyPressEvent(event);
}
