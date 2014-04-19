#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QErrorMessage>
#include <QTreeWidgetItem>

#include "multipathdock.h"
#include "linearpathdock.h"
#include "stancedock.h"
#include "qresourcetreeitem.h"


ResourceBrowser::MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    gui=NULL;
    MakeRobotResourceLibrary(library);
    RobotPose();
}

ResourceBrowser::MainWindow::~MainWindow()
{
    delete ui;
}

void ResourceBrowser::MainWindow::RobotPose(){
    RobotTest::MainWindow main;
    main.show();
}

void ResourceBrowser::MainWindow::OpenFile(QString filename){
  if(filename.isEmpty()){
    QFileDialog f;
    filename = f.getOpenFileName(0,"Open File",QDir::home().absolutePath(),"");
  }
  if(!filename.isNull()){
    int valid;
//      LinearPathDock* widget = new LinearPathDock(filename,&valid);
      ResourcePtr r = library.LoadItem(filename.toStdString());
      ResourceDockWidget* widget;
      QResourceTreeItem* it=new QResourceTreeItem(r);
      ui->treeWidget->addTopLevelItem(it);
    }
}

void ResourceBrowser::MainWindow::ChangeSelectedItem(QTreeWidgetItem* _it){
    selected=(QResourceTreeItem*)_it;
}

void ResourceBrowser::MainWindow::PressedDelete(){
    if(selected)
        selected->DeleteItem();
}

void ResourceBrowser::MainWindow::PressedExpand(){
    if(selected)
        selected->Expand();
}
