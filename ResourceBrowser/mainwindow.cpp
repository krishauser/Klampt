#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QErrorMessage>
#include <QTreeWidgetItem>

#include "multipathdock.h"
#include "linearpathdock.h"
#include "stancedock.h"
#include "qresourcetreeitem.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    gui=NULL;
    MakeRobotResourceLibrary(library);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::OpenFile(QString filename){
  if(filename.isEmpty()){
    QFileDialog f;
    filename = f.getOpenFileName(0,"Open File",QDir::home().absolutePath(),"");
  }
  if(!filename.isNull()){
    int valid;
//      LinearPathDock* widget = new LinearPathDock(filename,&valid);
      ResourcePtr r = library.LoadItem(filename.toStdString());
      ResourceDockWidget* widget;
      /*
      string str=r->Type();
      if(0==strcmp("LinearPath",r->Type())){
          widget = new LinearPathDock(r);
      }
      else if(0==strcmp("Stance",r->Type())){
         widget = new StanceDock(r);
      }
      else if(0==strcmp("MultiPath",r->Type())){
          //widget = new MultiPathDock(r);
      }
    //MultiPathDock* widget = new MultiPathDock(filename,&valid);
      else{
        QMessageBox(QMessageBox::NoIcon,QString("Error"),QString("File is not a valid resource")).exec();
        delete widget;
        return;
      }
      widget->gui=gui;
      widget->master_dock = this;
      //this->addDockWidget(Qt::LeftDockWidgetArea,widget);
      //this->ui->treeWidget->addTopLevelItem(widget->treeItem);
      */
      QResourceTreeItem* it=new QResourceTreeItem(r);
      ui->treeWidget->addTopLevelItem(it);
    }
}

void MainWindow::ChangeSelectedItem(QTreeWidgetItem* _it){
    selected=(QResourceTreeItem*)_it;
}

void MainWindow::PressedDelete(){
    if(selected)
        selected->DeleteItem();
}

void MainWindow::PressedExpand(){
    if(selected)
        selected->Expand();
}
