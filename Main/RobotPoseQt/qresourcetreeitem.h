#ifndef QRESOURCETREEITEM_H
#define QRESOURCETREEITEM_H

#include <QTreeWidgetItem>

#include "Modeling/Resources.h"
#include "resourcemanager.h"

#define NAMECOL 0
#define TYPECOL 1
#define DECORATORCOL 2

class QResourceTreeItem : public QTreeWidgetItem
{
public:
  ResourceNodePtr resource;

    QResourceTreeItem(ResourceNodePtr rt);
    void AddChildren(const vector<ResourceNodePtr>& resources);
    void addChild(ResourceNodePtr r);
    void updateDecorator();
};
#endif // QRESOURCETREEITEM_H
