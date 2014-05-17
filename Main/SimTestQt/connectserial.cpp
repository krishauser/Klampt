#include "connectserial.h"
#include "ui_connectserial.h"
#include "Modeling/World.h"

#include <QDialogButtonBox>
#include <QPushButton>

ConnectSerial::ConnectSerial(RobotWorld* world,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ConnectSerial)
{
    ui->setupUi(this);
    SetNumRobots(world);
}

ConnectSerial::~ConnectSerial()
{
    delete ui;
}

void ConnectSerial::accept()
{
    emit MakeConnect(ui->cmb_robot->itemData(ui->cmb_robot->currentIndex()).toInt(),
                     QString("localhost"),ui->spn_port->value(),ui->spn_rate->value());
    QDialog::accepted();
}

void ConnectSerial::SetNumRobots(RobotWorld* world){
   ui->cmb_robot->clear();
   if(world==NULL) return;
   for(size_t i=0;i<world->robots.size();i++){
     ui->cmb_robot->addItem(QString(world->robots[i].name.c_str()),i);
   }
}
