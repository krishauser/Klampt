#include "stancedock.h"
#include "holddock.h"
#include "ui_resourcedockwidget.h"

#include <QLabel>
#include <Modeling/MultiPath.h>
#include <QListWidgetItem>
#include <QMainWindow>

#include <boost/foreach.hpp>

void StanceDock::Initialize(StanceResource* s)
    {
    setWindowTitle("Stance " + QString::fromStdString(s->name));
    lst_holds = AddContentList("Holds");
    vector<ResourcePtr> holds = ExtractResources(s,"Hold");
    for(int i=0;i<holds.size();i++){
        map_holds[i] = holds[i];
        QListWidgetItem* it = new QListWidgetItem(QString::fromStdString(holds[i]->name));
        it->setData(Qt::UserRole,i);
        lst_holds->addItem(it);
    }

    treeItem->setText(0,QString::fromStdString(s->name));
    treeItem->setText(1,"stance");
}

StanceDock::StanceDock(QString filename, int *valid, QWidget *parent):
    ResourceDockWidget(valid,parent)
{
    //map_holds = new std::map<int,ResourcePtr>;
    StanceResource* s =new StanceResource();
    //s->Load()
    std::istringstream stream(filename.toStdString());
    *valid=s->Load(filename.toStdString());
    if(!*valid) return;
    s->name=filename.toStdString();
    Initialize(s);
    }
    /*
    for(Stance::const_iterator i=stance.begin();i!=stance.end();i++){
        QListWidgetItem* it=new QListWidgetItem(QString::fromStdString(l->name) + "[" + QString::number(i) + "]");
        it->setData(Qt::UserRole,i);
        it->setData(Qt::UserRole,l->milestones[i]);
        cout<<l->times[i]<<"\t"<<l->milestones[i]<<endl;
        lst_times->addItem(it);
    }*/

StanceDock::StanceDock(ResourcePtr& ptr, QWidget *parent):
    ResourceDockWidget(NULL,parent)
{
    StanceResource *s = dynamic_cast<StanceResource*>((ResourceBase*) ptr);
    Initialize(s);
}

void StanceDock::ToGUI(){
    if(gui != NULL){
        //gui->SendCommand("command_config",lst_times->selectedItems().at(0)->data(Qt::UserRole + 1).toString().toStdString());
    }
    else
        cout<<"GUI does not exist\n";
}

void StanceDock::Expand(){
    BOOST_FOREACH(QListWidgetItem* it,lst_holds->selectedItems()){
        master_dock->addDockWidget(Qt::RightDockWidgetArea, new HoldDock(dynamic_cast<HoldResource*>((ResourceBase*)map_holds[it->data(Qt::UserRole).toInt()])));
    }
}

StanceDock::~StanceDock()
{
    delete lst_holds;
}
