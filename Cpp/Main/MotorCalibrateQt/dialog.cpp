#include "dialog.h"
#include "ui_dialog.h"
#include "Modeling/Robot.h"
#include <KrisLibrary/utils/AnyCollection.h>
#include <Main/motorcalibrate.h>
#include "showtext.h"
using namespace Klampt;

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);
    popup=new ShowText();
}

Dialog::~Dialog()
{
    delete ui;
  delete popup;
}

int Dialog::LoadRobot(){
    robotFilename = f.getOpenFileName(0,"Open Robot","","*.rob *.urdf");
    if(!robotFilename.isEmpty()){
        RobotModel r;
        r.Load(robotFilename.toStdString().c_str());
        ui->lbl_robot->setText(QFileInfo(robotFilename).baseName());
        for(int i=0;i<r.links.size();i++){
            QListWidgetItem* it=new QListWidgetItem();
            it->setText(QString::fromStdString(r.linkNames[i]));
            it->setData(Qt::UserRole,i);
            ui->list_free->addItem(it);
        }
        for(int i=0;i<r.drivers.size();i++){
            QListWidgetItem* it=new QListWidgetItem(QString::fromStdString(r.driverNames[i]));
            it->setData(Qt::UserRole,i);
            ui->list_drivers->addItem(it);
        }
        return 1;
    }
    return 0;
}

int Dialog::AddPaths(){
    QString commanded = f.getOpenFileName(0,"Select Commanded Path","","*.path");
    if(!commanded.isEmpty()){
        QString sensed = f.getOpenFileName(0,"Select Sensed Path",QFileInfo(commanded).filePath(),"*.path");
        if(!sensed.isEmpty()){
            ui->list_commanded->addItem(commanded);
            ui->list_sensed->addItem(sensed);
            return 1;
        }
    }
    return 0;
}

int Dialog::SetPathIndex(int i){
    ui->list_commanded->setCurrentRow(i);
    ui->list_sensed->setCurrentRow(i);
    return 1;
}

int Dialog::DeletePaths(){
    int i=ui->list_commanded->currentRow();
    delete ui->list_commanded->takeItem(i);
    delete ui->list_sensed->takeItem(i);
    return 1;
}

int Dialog::DoCalibrate(){
  if(robotFilename.isEmpty()){
      return 0;
  }
  AnyCollection settings;
  settings["robot"]=robotFilename.toStdString();
  settings["numIters"]=ui->spn_numiters->value();
//  settings["maxMilestones"]=gMaxMilestones;
    settings["maxMilestones"]=100000;
  settings["dt"]=ui->line_dt->text().toDouble();
  settings["velocityErrorWeight"]=Real(ui->line_error->text().toDouble());
  vector<int> drivers=vector<int>();
  for(int row = 0; row < ui->list_estimated->count(); row++)
    drivers.push_back(ui->list_estimated->item(row)->data(Qt::UserRole).toInt());
  settings["drivers"]=drivers;

  vector<int> fixed=vector<int>();
  for(int row = 0; row < ui->list_fixed->count(); row++)
    fixed.push_back(ui->list_fixed->item(row)->data(Qt::UserRole).toInt());

  vector<string> commanded=vector<string>();
  vector<string> sensed=vector<string>();

  for(int row = 0; row < ui->list_commanded->count(); row++)
   commanded.push_back(ui->list_commanded->item(row)->text().toStdString());
  for(int row = 0; row < ui->list_sensed->count(); row++)
    sensed.push_back(ui->list_sensed->item(row)->text().toStdString());

  settings["fixedLinks"]=fixed;
  settings["commandedPaths"]=commanded;
  settings["sensedPaths"]=sensed;
  QString newtext=QString::fromStdString(motorcalibrate(settings));
  popup->SetText(newtext);
  popup->rob=robotFilename;
  popup->show();

  cout<<endl;
  cout<<"To re-run from command line, copy the following lines to \n\
motorcalibrate.settings and run \n\
  ./MotorCalibrate motorcalibrate.settings"<<endl;
  cout<<settings<<endl;
}
