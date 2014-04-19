#include "linearpathdock.h"
#include "ui_resourcedockwidget.h"
#include <QLabel>
#include <Modeling/MultiPath.h>
#include <Modeling/Resources.h>
#include <QListWidgetItem>

LinearPathDock::LinearPathDock(QString filename, int *valid, QWidget *parent):
    ResourceDockWidget(valid,parent)
{
    LinearPathResource* l =new LinearPathResource();
    *valid=l->Load(filename.toStdString());
    if(!*valid) return;
    l->name=filename.toStdString();
    LinearPathDock::Initialize(l);
}


LinearPathDock::LinearPathDock(ResourcePtr& ptr, QWidget *parent):
    ResourceDockWidget(NULL,parent)
{
    LinearPathResource *s = dynamic_cast<LinearPathResource*>((ResourceBase*) ptr);
    Initialize(s);
}

void LinearPathDock::Initialize(LinearPathResource* r){
    setWindowTitle("Linear Path: " + QString::fromStdString(r->name));
    lst_times = AddContentList("Configs");
    for(int i=0;i<r->times.size();i++){
        QListWidgetItem* it=new QListWidgetItem(QString::fromStdString(r->name) + "[" + QString::number(i) + "]");
        it->setData(Qt::UserRole,i);
        //it->setData(Qt::UserRole,l->milestones[i]);
        cout<<r->times[i]<<"\t"<<r->milestones[i]<<endl;
        lst_times->addItem(it);
    }
}

void LinearPathDock::ToGUI(){
    if(gui != NULL){
        gui->SendCommand("command_config",lst_times->selectedItems().at(0)->data(Qt::UserRole + 1).toString().toStdString());
    }
    else
        cout<<"GUI does not exist\n";
}

LinearPathDock::~LinearPathDock()
{
    delete lst_times;
    delete ui;
}
