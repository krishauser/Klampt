#include "resourceframe.h"
#include "ui_resourceframe.h"

#include <QSettings>
#include <QCoreApplication>

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
    filename = f.getOpenFileName(0,"Open File",QDir::home().absolutePath(),"");
  }
  if(!filename.isNull()){
    ini.setValue("last_open_resource_directory",QFileInfo(filename).absolutePath());
    //gui->SendCommand("load_resource",filename.toStdString());

    ResourceNode r = manager->LoadResource(filename.toStdString());
    QResourceTreeItem* it=new QResourceTreeItem(r);
    ui->treeWidget->addTopLevelItem(it);

    }
}

void ResourceFrame::ChangeSelectedItem(QTreeWidgetItem* it){
    if(it)
        manager->ChangeSelected(it->text(0).toStdString());
    else
        manager->ChangeSelected("");
}

void ResourceFrame::PressedDelete(){
    if(manager->DeleteSelected())
        BOOST_FOREACH(QTreeWidgetItem* it, ui->treeWidget->selectedItems()){
            delete it;
        }
}

void ResourceFrame::PressedExpand(){
    if(ui->treeWidget->selectedItems().isEmpty()) return;
    QResourceTreeItem* sel=(QResourceTreeItem*)(ui->treeWidget->selectedItems()[0]);
    sel->AddChildren(manager->ExpandSelected());
}

void ResourceFrame::ToGUI(){
    gui->SendCommand("resource_to_poser");
    //gui->backend->RenderCurrentResource();
  /*
    if(0 == strcmp(selected->resource->Type(),"Config")){
        ConfigResource* config = dynamic_cast<ConfigResource*>((ResourceBase*)selected->resource);
            stringstream ss;
            ss<<config->data;
          gui->SendCommand("set_q",ss.str());
    }
  */
}

void ResourceFrame::FromGUI(){
    gui->SendCommand("poser_to_resource","Config");
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

//void ResourceFrame::selectionChanged(){

//}
