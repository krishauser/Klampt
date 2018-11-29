#include "driveredit.h"
#include "ui_driveredit.h"
#include <QFileDialog>

string toStdString(const QString& s);

DriverEdit::DriverEdit(RobotWorld* _world,WorldSimulation* _sim,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DriverEdit),
    world(_world),sim(_sim),robot(NULL),current(0)
{
    ui->setupUi(this);
    if(world->robots.size() > 0) {
      robot=world->robots[0].get();
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
    if(robot) {
      value=robot->GetDriverValue(current);
      ui->doubleSpinBox->setValue(value);
      HandleSpinBox(value);
    }
}

void DriverEdit::RequestDriverParameters(){
   //emit GetDriverValues(index);
    if(robot) {
      Vector2 limits=robot->GetDriverLimits(current);
      min=limits[0];
      max=limits[1];
      value=robot->GetDriverValue(current);
      ui->lbl_max->setText(QString::number(max));
      ui->lbl_min->setText(QString::number(min));
      ui->doubleSpinBox->setMaximum(max);
      ui->doubleSpinBox->setMinimum(min);
      ui->doubleSpinBox->setValue(value);
      ui->kPSpinBox->setValue(robot->drivers[current].servoP);
      ui->kPSpinBox->setMaximum(Max(robot->drivers[current].servoP*2.0,100.0));
      ui->kDSpinBox->setValue(robot->drivers[current].servoD);
      ui->kDSpinBox->setMaximum(Max(robot->drivers[current].servoD*2.0,100.0));
      ui->kISpinBox->setValue(robot->drivers[current].servoI);
      ui->kISpinBox->setMaximum(Max(robot->drivers[current].servoI*2.0,100.0));
      ui->dryFrictionSpinBox->setValue(robot->drivers[current].dryFriction);
      ui->viscousFrictionSpinBox->setValue(robot->drivers[current].viscousFriction);
      ui->vmaxSpinBox->setValue(robot->drivers[current].vmax);
      ui->vmaxSpinBox->setMaximum(Max(robot->drivers[current].vmax*2,10.0));
      ui->amaxSpinBox->setValue(robot->drivers[current].amax);
      ui->amaxSpinBox->setMaximum(Max(robot->drivers[current].amax*2,100.0));
      ui->tmaxSpinBox->setValue(robot->drivers[current].tmax);
      ui->tmaxSpinBox->setMaximum(Max(robot->drivers[current].tmax*2,100.0));
      HandleSpinBox(value);
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

void DriverEdit::HandleKP(double _value){
    if(robot==NULL) return;
    robot->drivers[current].servoP = _value;
    if(sim)
      sim->controlSimulators[0].command.actuators[current].kP = _value;
    ui->kPSpinBox->setMaximum(Max(robot->drivers[current].servoP*2.0,100.0));
}

void DriverEdit::HandleKD(double _value){
    if(robot==NULL) return;
    robot->drivers[current].servoD = _value;
    if(sim)
      sim->controlSimulators[0].command.actuators[current].kD = _value;
    ui->kDSpinBox->setMaximum(Max(robot->drivers[current].servoD*2.0,100.0));
}

void DriverEdit::HandleKI(double _value){
    if(robot==NULL) return;
    robot->drivers[current].servoI = _value;
    if(sim)
      sim->controlSimulators[0].command.actuators[current].kI = _value;
    ui->kISpinBox->setMaximum(Max(robot->drivers[current].servoI*2.0,100.0));
}

void DriverEdit::HandleDryFriction(double _value){
    if(robot==NULL) return;
    robot->drivers[current].dryFriction = _value;
}

void DriverEdit::HandleViscousFriction(double _value){
    if(robot==NULL) return;
    robot->drivers[current].viscousFriction = _value;
}

void DriverEdit::HandleVmax(double _value){
    if(robot==NULL) return;
  robot->drivers[current].vmax = _value;
  robot->drivers[current].vmin = -_value;
  ui->vmaxSpinBox->setMaximum(Max(robot->drivers[current].vmax*2,10.0));
}

void DriverEdit::HandleAmax(double _value){
    if(robot==NULL) return;
  robot->drivers[current].amax = _value;
  robot->drivers[current].amin = -_value;
  ui->amaxSpinBox->setMaximum(Max(robot->drivers[current].amax*2,100.0));
}

void DriverEdit::HandleTmax(double _value){
    if(robot==NULL) return;
  robot->drivers[current].tmax = _value;
  robot->drivers[current].tmin = -_value;
  ui->tmaxSpinBox->setMaximum(Max(robot->drivers[current].tmax*2,100.0));
}

void DriverEdit::SaveSettings()
{
    if(robot==NULL) return;
    QString filename = QFileDialog::getSaveFileName(0, "Open Scenario", QString(), "Robot (*.rob *.urdf);;All Files (*)");
    if (filename.isNull()) return;
    robot->Save(toStdString(filename).c_str());
}