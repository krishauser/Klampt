#include "driveredit.h"
#include "ui_driveredit.h"

DriverEdit::DriverEdit(RobotWorld* _world,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DriverEdit),
    world(_world),current(0)
{
    ui->setupUi(this);
    if(world->robots.size() > 0) {
      Robot* robot=world->robots[0];
      addDrivers(robot->driverNames);
    }
    RequestDriverParameters();
    ui->slider->setValue(500);
}

DriverEdit::~DriverEdit()
{
    delete ui;
}

void DriverEdit::RequestDriverValue(){
    if(world->robots.size() > 0) {
      Robot* robot=world->robots[0];
      value=robot->GetDriverValue(current);
      ui->doubleSpinBox->setValue(value);
      HandleSpinBox(value);
    }
}

void DriverEdit::RequestDriverParameters(){
   //emit GetDriverValues(index);
    if(world->robots.size() > 0) {
      Robot* robot=world->robots[0];
      Vector2 limits=robot->GetDriverLimits(current);
      GetDriverParameters(limits[0],limits[1],robot->GetDriverValue(current));
    }
    //current=index;
}

void DriverEdit::NewSelection(int _current){
    current=_current;
    RequestDriverParameters();
}

void DriverEdit::AddDriver(QString name){
   ui->comboBox->addItem(name);
}

void DriverEdit::addDrivers(vector<string> drivers){
    for(int i=0;i<drivers.size();i++)
        ui->comboBox->addItem(QString::fromStdString(drivers[i]));
}

void DriverEdit::GetDriverParameters(float _min,float _max,float _value){
    min=_min;
    max=_max;
    value=_value;
    ui->lbl_max->setText(QString::number(max));
    ui->lbl_min->setText(QString::number(min));
    ui->doubleSpinBox->setMaximum(max);
    ui->doubleSpinBox->setMinimum(min);
    ui->doubleSpinBox->setValue(value);
    HandleSpinBox(value);
}

void DriverEdit::SendDriverValue(){
    float value=ui->doubleSpinBox->value();
    emit SetDriverValue(current,value);
}

void DriverEdit::HandleSlider(int pos){

    float inc=(max-min)/1000;
    value=min+inc*pos;
    ui->doubleSpinBox->setValue(value);
    SendDriverValue();
}

void DriverEdit::HandleSpinBox(double _value){
    if(value==_value)return;//looping
    value=_value;
    float range=max-min;
    float relative=value-min;
    ui->slider->setValue(relative/range*1000);
    SendDriverValue();
}
