#include "resourcedockwidget.h"
#include "ui_resourcedockwidget.h"
#include <QLabel>

ResourceDockWidget::ResourceDockWidget(int *valid, QWidget *parent):
    QDockWidget(parent),
    ui(new Ui::ResourceDockWidget)
{
    ui->setupUi(this);
    treeItem=new QTreeWidgetItem();
}

ResourceDockWidget::~ResourceDockWidget()
{
    delete ui;
}

QListWidget* ResourceDockWidget::AddContentList(QString name){
    QLabel* lbl = new QLabel(name);
    ui->contents->addWidget(lbl);
    QListWidget* items=new QListWidget();
    ui->contents->addWidget(items);
    return items;
}

void ResourceDockWidget::ToGUI()
{printf("virtual class");}
void ResourceDockWidget::FromGUI()
{printf("virtual class");}

void ResourceDockWidget::DeleteItem()
{printf("virtual class");}

void ResourceDockWidget::Expand()
{printf("virtual class");}
