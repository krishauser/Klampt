#include "holddock.h"
#include "ui_resourcedockwidget.h"
#include <QLabel>
#include <Modeling/MultiPath.h>
#include <Modeling/Resources.h>
#include <QListWidgetItem>

HoldDock::HoldDock(HoldResource* h, QWidget *parent):
    ResourceDockWidget(0,parent)
{
    this->setWindowTitle("Hold " + QString::fromStdString(h->name));
    QString name="IK Constraints";
    lst_ik = AddContentList(name);
    QListWidgetItem* it=new QListWidgetItem("IK Constraint");
    it->setData(Qt::UserRole,-1);
    lst_ik->addItem(it);
    lst_contacts = AddContentList("Contacts");
    for(int i=0;i<h->data.contacts.size();i++){
        QListWidgetItem* it=new QListWidgetItem(QString::number(i));
        it->setData(Qt::UserRole,i);
        //it->setData(Qt::UserRole,l->milestones[i]);
        //cout<<l->times[i]<<"\t"<<l->milestones[i]<<endl;
        lst_contacts->addItem(it);
    }
}

void HoldDock::ToGUI(){
    if(gui != NULL){
        //gui->SendCommand("command_config",lst_times->selectedItems().at(0)->data(Qt::UserRole + 1).toString().toStdString());
    }
    else
        cout<<"GUI does not exist\n";
}

void HoldDock::Expand(){
    if(lst_ik->isActiveWindow()){

    }
}

HoldDock::~HoldDock()
{
    delete lst_contacts;
    delete ui;
}
