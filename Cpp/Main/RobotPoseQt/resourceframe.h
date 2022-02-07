#ifndef RESOURCEFRAME_H
#define RESOURCEFRAME_H

#include <QFrame>
#include <QFileDialog>
#include <QTreeWidgetItem>

#include <Modeling/Resources.h>
//#include <qresourcetreemodel.h>

#include "resourcemanager.h"
#include "Interface/GenericGUI.h"
using namespace Klampt;

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
    //QResourceTreeModel* resourceTreeModel;
    map<string,int> resourceToStackWidgetIndex;

    ~ResourceFrame();
    void SetManager(ResourceManager* manager);
    void updateNewResource(string id);
    void refreshResources();
    void updateConvertBox(ResourcePtr resource);
    void updateToFromPoser(ResourcePtr resource);
    void updateSelectedResourcePane(ResourcePtr resource);
    bool doBackup(QTreeWidgetItem* it);
    void onResourceEdit();
public slots:
    void OpenFile(QString filename);
    virtual void OpenFile(){OpenFile(QString());}
    void SaveResource();
    void SaveAllResources();
    void ChangeSelectedItem(QTreeWidgetItem *_it);
    void ItemExpanded(QTreeWidgetItem*);
    void ItemChanged(QTreeWidgetItem*,int);
    void PressedDelete();
    void ToGUI();
    void AddNew(QString type);
    void ConvertTo(QString type);
    void FromGUI();
    void SendPathTime(double t);

private:
    Ui::ResourceFrame *ui;
};

#endif // RESOURCEFRAME_H
