#include "driveredit.h"
#include "ui_driveredit.h"
#include <QFileDialog>
using namespace Klampt;

#define SPIN_BOX_MAX_SCALE 10.0

#define SPIN_BOX_MAX_SCALE 10.0

string toStdString(const QString& s);

DriverEdit::DriverEdit(WorldModel* _world,Simulator* _sim,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DriverEdit),
    world(_world),sim(_sim),robot(NULL),current(0)
{
    ui->setupUi(this);
    if(world->robots.size() > 0) {
      robot=world->robots[0].get();
      addDrivers(robot->driverNames);
    }
    if(!robot || robot->drivers.size()==0)
      current = -1;
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
    if(!robot) return;
    if(current < 0) return;
    Assert(current >= 0 && current < (int)robot->drivers.size());
    Vector2 limits=robot->GetDriverLimits(current);
    min=limits[0];
    max=limits[1];
    value=robot->GetDriverValue(current);
    ui->lbl_max->setText(QString::number(max));
    ui->lbl_min->setText(QString::number(min));
    ui->doubleSpinBox->setMaximum(max);
    ui->doubleSpinBox->setMinimum(min);
    ui->doubleSpinBox->setValue(value);

    double kP = robot->drivers[current].servoP, kI = robot->drivers[current].servoI, kD = robot->drivers[current].servoD;
    double dryFriction = robot->drivers[current].dryFriction;
    double viscousFriction = robot->drivers[current].viscousFriction;
    ui->kPSpinBox->setMaximum(Max(kP*SPIN_BOX_MAX_SCALE,100.0));  /// warning: will clamp the current value
    ui->kPSpinBox->setValue(kP);
    ui->kDSpinBox->setMaximum(Max(kD*SPIN_BOX_MAX_SCALE,100.0));  /// warning: will clamp the current value
    ui->kDSpinBox->setValue(kD);
    ui->kISpinBox->setMaximum(Max(kI*SPIN_BOX_MAX_SCALE,100.0));  /// warning: will clamp the current value
    ui->kISpinBox->setValue(kI);
    ui->dryFrictionSpinBox->setMaximum(Max(dryFriction*SPIN_BOX_MAX_SCALE,10.0));  /// warning: will clamp the current value
    ui->dryFrictionSpinBox->setValue(dryFriction);
    ui->viscousFrictionSpinBox->setMaximum(Max(viscousFriction*SPIN_BOX_MAX_SCALE,10.0));  /// warning: will clamp the current value
    ui->viscousFrictionSpinBox->setValue(viscousFriction);
    if(!IsFinite(robot->drivers[current].vmax)) {
      ui->vmaxSpinBox->setMaximum(10000);
      ui->vmaxSpinBox->setValue(10000);
    }
    else {
      double vmax = robot->drivers[current].vmax;
      ui->vmaxSpinBox->setMaximum(Max(vmax*SPIN_BOX_MAX_SCALE,10.0));
      ui->vmaxSpinBox->setValue(vmax);
    }
    if(!IsFinite(robot->drivers[current].amax)) {
      ui->amaxSpinBox->setMaximum(10000);
      ui->amaxSpinBox->setValue(10000);
    }
    else {
      double amax = robot->drivers[current].amax;
      ui->amaxSpinBox->setMaximum(Max(amax*SPIN_BOX_MAX_SCALE,100.0));
      ui->amaxSpinBox->setValue(amax);
    }
    if(!IsFinite(robot->drivers[current].tmax)) {
      ui->tmaxSpinBox->setMaximum(10000);
      ui->tmaxSpinBox->setValue(10000);
    }
    else {
      double tmax = robot->drivers[current].tmax;
      ui->tmaxSpinBox->setMaximum(Max(tmax*SPIN_BOX_MAX_SCALE,100.0));
      ui->tmaxSpinBox->setValue(tmax);
    }
    HandleSpinBox(value);
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
    ui->kPSpinBox->setMaximum(Max(ui->kPSpinBox->maximum(),robot->drivers[current].servoP*SPIN_BOX_MAX_SCALE,100.0));
}

void DriverEdit::HandleKD(double _value){
    if(robot==NULL) return;
    robot->drivers[current].servoD = _value;
    if(sim)
      sim->controlSimulators[0].command.actuators[current].kD = _value;
    ui->kDSpinBox->setMaximum(Max(ui->kDSpinBox->maximum(),robot->drivers[current].servoD*SPIN_BOX_MAX_SCALE,100.0));
}

void DriverEdit::HandleKI(double _value){
    if(robot==NULL) return;
    robot->drivers[current].servoI = _value;
    if(sim)
      sim->controlSimulators[0].command.actuators[current].kI = _value;
    ui->kISpinBox->setMaximum(Max(ui->kISpinBox->maximum(),robot->drivers[current].servoI*SPIN_BOX_MAX_SCALE,100.0));
}

void DriverEdit::HandleDryFriction(double _value){
    if(robot==NULL) return;
    robot->drivers[current].dryFriction = _value;
    ui->dryFrictionSpinBox->setMaximum(Max(ui->dryFrictionSpinBox->maximum(),robot->drivers[current].dryFriction*SPIN_BOX_MAX_SCALE,10.0));
}

void DriverEdit::HandleViscousFriction(double _value){
    if(robot==NULL) return;
    robot->drivers[current].viscousFriction = _value;
    ui->viscousFrictionSpinBox->setMaximum(Max(ui->viscousFrictionSpinBox->maximum(),robot->drivers[current].viscousFriction*SPIN_BOX_MAX_SCALE,10.0));
}

void DriverEdit::HandleVmax(double _value){
    if(robot==NULL) return;
  robot->drivers[current].vmax = _value;
  robot->drivers[current].vmin = -_value;
  ui->vmaxSpinBox->setMaximum(Max(ui->vmaxSpinBox->maximum(),robot->drivers[current].vmax*SPIN_BOX_MAX_SCALE,10.0));
}

void DriverEdit::HandleAmax(double _value){
    if(robot==NULL) return;
  robot->drivers[current].amax = _value;
  robot->drivers[current].amin = -_value;
  ui->amaxSpinBox->setMaximum(Max(ui->amaxSpinBox->maximum(),robot->drivers[current].amax*SPIN_BOX_MAX_SCALE,100.0));
}

void DriverEdit::HandleTmax(double _value){
    if(robot==NULL) return;
  robot->drivers[current].tmax = _value;
  robot->drivers[current].tmin = -_value;
  ui->tmaxSpinBox->setMaximum(Max(ui->tmaxSpinBox->maximum(),robot->drivers[current].tmax*SPIN_BOX_MAX_SCALE,100.0));
}

void DriverEdit::SaveSettings()
{
    if(robot==NULL) return;
    QString filename = QFileDialog::getSaveFileName(0, "Modified robot file", QString(), "Robot (*.rob *.urdf);;All Files (*)");
    if (filename.isNull()) return;
    robot->Save(toStdString(filename).c_str());
}