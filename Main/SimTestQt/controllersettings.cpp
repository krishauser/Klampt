#include "controllersettings.h"
#include "ui_controllersettings.h"

#include <iostream>

ControllerSettings::ControllerSettings(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ControllerSettings)
{
    ui->setupUi(this);
}

ControllerSettings::~ControllerSettings()
{
    delete ui;
    refreshing=0;
}

void ControllerSettings::Refresh(){
    refreshing=1;
    ui->tableWidget->clear();
    ui->tableWidget->setRowCount(settings.size());
    ui->tableWidget->setColumnCount(1);
    int j=0;
    for(map<string,string>::iterator i=settings.begin();i!=settings.end();i++,j++){
        ui->tableWidget->setItem(j,0,new QTableWidgetItem());
        ui->tableWidget->setVerticalHeaderItem(j,new QTableWidgetItem(QString::fromStdString(i->first)));
        ui->tableWidget->item(j,0)->setText(QString::fromStdString(i->second));
    }
    refreshing=0;
}

void ControllerSettings::OnCellEdited(int row,int col){
    if(refreshing) return;
    string key=ui->tableWidget->verticalHeaderItem(row)->text().toStdString();
    edited.insert(key);
    settings[key]=ui->tableWidget->item(row,col)->text().toStdString();
}

void ControllerSettings::OnApply(){
    BOOST_FOREACH(string str,edited){
        emit SendControllerSetting(str,settings[str]);
    }
    edited.clear();
}
