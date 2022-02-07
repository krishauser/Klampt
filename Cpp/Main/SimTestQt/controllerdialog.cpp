#include "controllerdialog.h"
#include "ui_controllerdialog.h"
#include "Modeling/World.h"
#include "Simulation/Simulator.h"
#include <iostream>
//#include <boost/foreach.hpp>

string toStdString(const QString& s);

ControllerDialog::ControllerDialog(Simulator* _sim,QWidget *parent) :
  QDialog(parent),sim(_sim),world(_sim ? _sim->world : NULL),
    ui(new Ui::ControllerDialog)
{
    ui->setupUi(this);

   ui->cmb_robot->clear();
   if(world==NULL) return;
   for(size_t i=0;i<world->robots.size();i++){
     ui->cmb_robot->addItem(QString(world->robots[i]->name.c_str()),(int)i);
   }
   ui->tableWidget->verticalHeader()->setDefaultSectionSize(20); 
   Refresh();
}

ControllerDialog::~ControllerDialog()
{
    delete ui;
    refreshing=0;
}

void ControllerDialog::OnRobotChange(int robot)
{
  if(sim==NULL) return;
  if(sim->robotControllers.empty()) return;

  blockSignals(true);
    refreshing=1;
    settings=sim->robotControllers[robot]->Settings();
    ui->tableWidget->clear();
    ui->tableWidget->setRowCount(settings.size());
    ui->tableWidget->setColumnCount(1);
    int j=0;
    for(map<string,string>::iterator i=settings.begin();i!=settings.end();i++,j++){
        ui->tableWidget->setItem(j,0,new QTableWidgetItem());
        ui->tableWidget->setVerticalHeaderItem(j,new QTableWidgetItem(QString::fromStdString(i->first)));
        ui->tableWidget->item(j,0)->setText(QString::fromStdString(i->second));
    }
    commands=sim->robotControllers[robot]->Commands();
    //commands is a vector of strings
    //BOOST_FOREACH(string str,commands){
    for(vector<string>::iterator it=commands.begin(); it!= commands.end(); it++){
      ui->comboBox->addItem(QString::fromStdString(*it));     
    }
    //}
    refreshing=0;
  blockSignals(false);
}

void ControllerDialog::Refresh(){
    refreshing=1;
    int index = ui->cmb_robot->itemData(ui->cmb_robot->currentIndex()).toInt();
    OnRobotChange(index);
    refreshing=0;
}

void ControllerDialog::OnCellEdited(QTableWidgetItem* item) {
    if(refreshing) return;
    string key= toStdString(ui->tableWidget->verticalHeaderItem(item->row())->text());
    settings[key]= toStdString(item->text());
    int robot = ui->cmb_robot->itemData(ui->cmb_robot->currentIndex()).toInt();
    emit SendControllerSetting(robot,key,settings[key]);    
}

void ControllerDialog::OnSendCommand()
{
  int robot = ui->cmb_robot->itemData(ui->cmb_robot->currentIndex()).toInt();
  if(ui->lineEdit->text().isEmpty()) ui->lineEdit->setFocus();
  else emit ControllerCommand(robot, toStdString(ui->comboBox->currentText()), toStdString(ui->lineEdit->text()));
}



void ControllerDialog::OnConnectSerial()
{
  int robot = ui->cmb_robot->itemData(ui->cmb_robot->currentIndex()).toInt();
    emit MakeConnect(robot,
                     QString("localhost"),ui->spn_port->value(),ui->spn_rate->value());
}
