#ifndef QRESOURCETREEWIDGET_H
#define QRESOURCETREEWIDGET_H

#include <QTreeWidget>
#include <QModelIndex>
#include <QDropEvent>
#include "resourcemanager.h"

#define NAMECOL 0
#define TYPECOL 1
#define DECORATORCOL 2

//enum { TypeCol=0,NameCol=1,DecoratorCol=2 };

class QResourceTreeWidget : public QTreeWidget
{
  Q_OBJECT

public:
  ResourceManager* manager;
  explicit QResourceTreeWidget(QWidget* parent=NULL);
  void setManager(ResourceManager* manager);
  void refresh();
  void addNotify(ResourceNode* node);
  void onAdd(ResourcePtr r);
  void onDeletePressed();
  void onDelete(QTreeWidgetItem* n);
  void onExpand(QTreeWidgetItem* n);
  QTreeWidgetItem* nodeToItem(ResourceNode* node);
  ResourceNode* itemToNode(QTreeWidgetItem* node);
  void updateAllDecorators();
  void updateDecorator(QTreeWidgetItem* item);
  void updateProperties(QTreeWidgetItem* item);

private:
  //for drag and drop
  QPoint startPos;
  ResourceNodePtr dragNode;
  virtual void mousePressEvent(QMouseEvent *event);
  virtual void mouseMoveEvent(QMouseEvent *event);
  virtual void dragEnterEvent(QDragEnterEvent *event);
  virtual void dragMoveEvent(QDragMoveEvent *event);
  virtual void dragLeaveEvent(QDragLeaveEvent *event);
  virtual void dropEvent(QDropEvent *event);

  //for delete
  virtual void keyPressEvent(QKeyEvent * event);

  //helpers
  QTreeWidgetItem* makeItem(ResourceNode* node);
};

#endif
