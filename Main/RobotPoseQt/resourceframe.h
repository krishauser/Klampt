#ifndef RESOURCEFRAME_H
#define RESOURCEFRAME_H

#include <QFrame>
#include <QFileDialog>
#include <QTreeWidgetItem>

#include <Interface/GenericGUI.h>
#include <Modeling/Resources.h>
#include <qresourcetreeitem.h>

#include "resourcemanager.h"
#include "Interface/GenericGUI.h"

namespace Ui {
  class ResourceFrame;
}

class ResourceFrame : public QFrame
{
    Q_OBJECT
    
public:
    explicit ResourceFrame(QWidget *parent = 0);
    ResourceManager* manager;
    GenericGUIBase *gui;
    Robot* robot;

    ~ResourceFrame();
    
    void updateNewResource(string name);
public slots:
    void OpenFile(QString filename);
    virtual void OpenFile(){OpenFile(QString());}
    void ChangeSelectedItem(QTreeWidgetItem *_it);
    void PressedDelete();
    void PressedExpand();
    void ToGUI();
    void FromGUI();

private:
    Ui::ResourceFrame *ui;
};

#endif // RESOURCEFRAME_H
