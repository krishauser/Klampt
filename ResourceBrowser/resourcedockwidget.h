#ifndef RESOURCEDOCKWIDGET_H
#define RESOURCEDOCKWIDGET_H

#include <QDockWidget>
#include <QListWidget>
#include <QMainWindow>
#include <QTreeWidgetItem>

#include <Interface/GenericGUI.h>

namespace Ui {
class ResourceDockWidget;
}

class ResourceDockWidget : public QDockWidget
{
    Q_OBJECT
    
public:
    GenericGUIBase* gui;
    explicit ResourceDockWidget(int *valid = 0,QWidget *parent=NULL);
    ~ResourceDockWidget();
    
    QMainWindow* master_dock;
    QTreeWidgetItem* treeItem;

    QListWidget *AddContentList(QString name);
public slots:
    virtual void ToGUI();
    virtual void FromGUI();
    virtual void DeleteItem();
    virtual void Expand();
protected:
    Ui::ResourceDockWidget *ui;
};

#endif // RESOURCEDOCKWIDGET_H
