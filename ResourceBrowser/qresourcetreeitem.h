#ifndef QRESOURCETREEITEM_H
#define QRESOURCETREEITEM_H

#include <QTreeWidgetItem>

#include "Modeling/Resources.h"
#include "RobotPoseTestQt/resourcemanager.h"

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

    ResourceTracker* resource;
    Math::VectorTemplate<double> *config;

    QString type;

    QResourceTreeItem();

    QResourceTreeItem(ResourcePtr p);
    void AddChildren(vector<ResourcePtr> ptrs);
public slots:
    virtual void ToGUI();
    virtual void FromGUI();
    virtual void DeleteItem();

};
#endif // QRESOURCETREEITEM_H
