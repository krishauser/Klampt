#ifndef QRESOURCETREEITEM_H
#define QRESOURCETREEITEM_H

#include <QTreeWidgetItem>

#include "Modeling/Resources.h"
#include "resourcemanager.h"

#define NAMECOL 0
#define TYPECOL 1

class QResourceTreeItem : public QTreeWidgetItem
{
public:

    int dirty;

    int show_delete;
    int show_load;
    int show_togui;
    int show_expand;
    string name;

    ResourceNode resource;
    Math::VectorTemplate<double> *config;

    QString type;

    QResourceTreeItem();

    QResourceTreeItem(ResourceNode rt);
    void AddChildren(vector<ResourceNode> resources);
    void addChild(ResourceNode r);
public slots:
    virtual void ToGUI();
    virtual void FromGUI();
    virtual void DeleteItem();

};
#endif // QRESOURCETREEITEM_H
