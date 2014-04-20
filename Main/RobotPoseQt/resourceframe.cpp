#include "resourceframe.h"
#include "ui_resourceframe.h"

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
  if(filename.isEmpty()){
    QFileDialog f;
    filename = f.getOpenFileName(0,"Open File",QDir::home().absolutePath(),"");
  }
  if(!filename.isNull()){
    int valid;
//      LinearPathDock* widget = new LinearPathDock(filename,&valid);
      ResourceTracker *r = manager->LoadResource(filename.toStdString());
      //ResourceDockWidget* widget;
      QResourceTreeItem* it=new QResourceTreeItem(r);
      ui->treeWidget->addTopLevelItem(it);
    }
}
void ResourceFrame::ChangeSelectedItem(QTreeWidgetItem* it){
    manager->ChangeSelected(((QResourceTreeItem*)it)->resource);
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
    BOOST_FOREACH(ResourceTracker* rt,manager->ExpandSelected()){
        sel->addChild(new QResourceTreeItem(rt));
    }
}

void ResourceFrame::ToGUI(){
    gui->backend->RenderCurrentResource();
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
}
