#include "resourceframe.h"
#include "ui_resourceframe.h"

ResourceFrame::ResourceFrame(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::ResourceFrame)
{
    MakeRobotResourceLibrary(library);
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
      ResourcePtr r = library.LoadItem(filename.toStdString());
      //ResourceDockWidget* widget;
      QResourceTreeItem* it=new QResourceTreeItem(r);
      ui->treeWidget->addTopLevelItem(it);
    }
}
void ResourceFrame::ChangeSelectedItem(QTreeWidgetItem* _it){
    selected=(QResourceTreeItem*)_it;
}

void ResourceFrame::PressedDelete(){
    if(selected)
        selected->DeleteItem();
}

void ResourceFrame::PressedExpand(){
    if(selected)
        selected->Expand();
}

void ResourceFrame::ToGUI(){
    if(0 == strcmp(selected->resource->Type(),"Config")){
        ConfigResource* config = dynamic_cast<ConfigResource*>((ResourceBase*)selected->resource);
            stringstream ss;
            ss<<config->data;
          gui->SendCommand("set_q",ss.str());
    }
}

void ResourceFrame::FromGUI(){
}
