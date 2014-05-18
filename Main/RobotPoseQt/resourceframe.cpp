#include "resourceframe.h"
#include "ui_resourceframe.h"

#include <QSettings>
#include <QCoreApplication>
#include <QInputDialog>

#include <boost/foreach.hpp>

ResourceFrame::ResourceFrame(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::ResourceFrame)
{
    ui->setupUi(this);
}

ResourceFrame::~ResourceFrame()
{
    delete ui;
}

void ResourceFrame::OpenFile(QString filename){
    QSettings ini(QSettings::IniFormat, QSettings::UserScope,
          QCoreApplication::organizationName(),
          QCoreApplication::applicationName());  if(filename.isEmpty()){
    QFileDialog f;
    QString openDir = ini.value("last_open_resource_directory",".").toString();
    filename = f.getOpenFileName(0,"Open File",openDir,"");
  }
  if(!filename.isNull()){
    ini.setValue("last_open_resource_directory",QFileInfo(filename).absolutePath());
    //gui->SendCommand("load_resource",filename.toStdString());
    //todo send message
    ResourceNode r = manager->LoadResource(filename.toStdString());
    QResourceTreeItem* it=new QResourceTreeItem(r);
    ui->treeWidget->addTopLevelItem(it);
    ui->treeWidget->setItemSelected(it,1);
    gui->SendCommand("set_resource",r->Name());
    }
}

void ResourceFrame::ChangeSelectedItem(QTreeWidgetItem* it){
    if(it == NULL) return;
    /*
    if(it)
        manager->ChangeSelected(it->text(0).toStdString());
    else
        manager->ChangeSelected("");
        */
    QResourceTreeItem* rti=(QResourceTreeItem*)it;
    printf("recovering %s\n",rti->name.c_str());
    gui->SendCommand("set_resource",rti->name);
    ui->addNewBox->clear();
}

//todo make this message passing like adding a resource
void ResourceFrame::PressedDelete(){
    if(manager->DeleteSelected()){
        QList<QTreeWidgetItem*> sel = ui->treeWidget->selectedItems();
        BOOST_FOREACH(QTreeWidgetItem* it, sel){
            if(it != NULL){
                if(it->parent() == NULL){
                    delete it;
                }
                else{
                    it->parent()->removeChild(it);
                    delete it;
                }

            }
        }
    }
    ui->treeWidget->clearSelection();
}

void ResourceFrame::PressedExpand(){
    if(ui->treeWidget->selectedItems().isEmpty()) return;
    QResourceTreeItem* sel=(QResourceTreeItem*)(ui->treeWidget->selectedItems()[0]);
    sel->AddChildren(manager->ExpandSelected());
}

void ResourceFrame::DiscretizePath(){
    int value = QInputDialog::getInteger(this,"Discrete Segmants","enter the number of segments output",10,1);
    if(value > 0){
        gui->SendCommand("discretize_path",value);
    }
}

void ResourceFrame::OptimizePath(){

}

void ResourceFrame::ToGUI(){
    gui->SendCommand("resource_to_poser");
}

void ResourceFrame::FromGUI(QString type){
    if(type == "Add New") return;
    gui->SendCommand("poser_to_resource",type.toStdString());
    ui->addNewBox->setCurrentIndex(0);
}

void ResourceFrame::updateNewResource(string name){
    ResourceNode node = manager->itemsByName[name];
    if(node){
        if(ui->treeWidget->selectedItems().isEmpty()){
            ui->treeWidget->addTopLevelItem(new QResourceTreeItem(node));
        }
        else{
        QResourceTreeItem *parent = (QResourceTreeItem*)(ui->treeWidget->selectedItems()[0]);
        parent->addChild(node);
        }
    }
    else{
       printf("No node of name %s found in tree",name.c_str());
    }
}

void ResourceFrame::SendPathTime(double t){
    gui->SendCommand("set_path_time",t);
}

//void ResourceFrame::selectionChanged(){

//}
