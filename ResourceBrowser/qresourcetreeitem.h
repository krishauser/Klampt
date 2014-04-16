#ifndef QRESOURCETREEITEM_H
#define QRESOURCETREEITEM_H

#include <QTreeWidgetItem>

#include "Modeling/Resources.h"

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

    ResourcePtr resource;

    QString type;

    QResourceTreeItem();

    QResourceTreeItem(ResourcePtr p);
    void AddChildren(vector<ResourcePtr> ptrs);
public slots:
    virtual void ToGUI();
    virtual void FromGUI();
    virtual void DeleteItem();
    virtual void Expand();

};
#endif // QRESOURCETREEITEM_H
