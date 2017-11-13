#include "stdio.h"
//#include <boost/foreach.hpp>

#include "logoptions.h"
#include "ui_logoptions.h"

#include "QCheckBox"
#include "QStandardItemModel"
#include "QLabel"
#include "QAction"


LogOptions::LogOptions(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::LogOptions),
    selected_measurement(0)
{
    ui->setupUi(this);
    currentLayout=new QFormLayout();
    ChangeSensor(0);
}

LogOptions::~LogOptions()
{
    delete ui;
}

void LogOptions::GetSensors(){
    sensorPlotted.resize(robotsensors.sensors.size(),false);
    sensorRendered.resize(robotsensors.sensors.size(),false);
    sensorMeasurementPlotted.resize(robotsensors.sensors.size());
    for(int i=0;i<robotsensors.sensors.size();i++){
        AddSensor(robotsensors.sensors[i]->name.c_str());
        vector<string> names;
        robotsensors.sensors[i]->MeasurementNames(names);
        sensorMeasurementPlotted[i].resize(names.size(),true);
    }
}

void LogOptions::ChangeSelectedIndex(int index){
    selected_measurement=index;
}

void LogOptions::AddSensor(QString name){
    ui->comboBox->addItem(name);
}

void LogOptions::addSensors(vector<string> sensors){
    for(int i=0;i<sensors.size();i++)
        ui->comboBox->addItem(QString::fromStdString(sensors[i]));
}

void LogOptions::AddMeasurement(QString name){
    ui->list_show->addItem(name);
}

void LogOptions::AddMeasurements(vector<string> names){
       //BOOST_FOREACH(string measurement,names){
    for(vector<string>::iterator it=names.begin();it != names.end();it++){
        AddMeasurement(QString::fromStdString(*it));
    }
}

void LogOptions::ChangeSensor(int _sensor){
    ui->list_show->clear();
    selected_sensor=_sensor;
    emit SyncSensorMeasurements(_sensor);
    vector<string> names;
    if(robotsensors.sensors.size())
        robotsensors.sensors[selected_sensor]->MeasurementNames(names);
    ui->list_show->clear();
    AddMeasurements(names);
    if(selected_sensor < (int)sensorPlotted.size()){
        ui->chk_plot->setChecked(sensorPlotted[selected_sensor]);
        ui->chk_render->setChecked(sensorRendered[selected_sensor]);
        for(int i=0;i<sensorMeasurementPlotted[selected_sensor].size();i++){
            if(sensorMeasurementPlotted[selected_sensor][i]){
                ui->list_show->item(i)->setBackgroundColor(Qt::white);
            }
            else{
                ui->list_show->item(i)->setBackgroundColor(Qt::gray);
            }
        }
    }
}

void LogOptions::HideItem(int index){
    ui->list_show->item(index)->setBackgroundColor(Qt::gray);
    sensorMeasurementPlotted[selected_sensor][index]=false;
    emit toggle_measurement(selected_sensor,index,false);
    //gui.SendCommand("hide_sensor_measurement",selected_sensor,index);
}

void LogOptions::ShowItem(int index){
    ui->list_show->item(index)->setBackgroundColor(Qt::white);
    sensorMeasurementPlotted[selected_sensor][index]=true;
    emit toggle_measurement(selected_sensor,index,true);
}

void LogOptions::ShowItem(){
    ShowItem(selected_measurement);
}

void LogOptions::HideItem(){
    HideItem(selected_measurement);
}

void LogOptions::ShowAll(){
    for(int i=0;i<ui->list_show->count();i++){
        ShowItem(i);
    }
}

void LogOptions::Isolate(){
    int is=selected_measurement;
    for(int i=0;i<ui->list_show->count();i++){
        HideItem(i);
    }
    ShowItem(is);
}

void LogOptions::TogglePlot(bool status){
    //emit toggle_plot(status);
    if(selected_sensor < (int)sensorPlotted.size())
        sensorPlotted[selected_sensor]=status;
    emit ShowSensor(selected_sensor,status);
}

void LogOptions::ToggleRender(bool status){
    if(selected_sensor < (int)sensorPlotted.size())
        sensorRendered[selected_sensor]=status;
    emit RenderSensor(selected_sensor,status);
}


void LogOptions::ToggleLogging(bool status){
    emit toggle_logging(status);
}
