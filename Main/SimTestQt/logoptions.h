#ifndef LOGOPTIONS_H
#define LOGOPTIONS_H

#include <QDialog>
#include <QFormLayout>

#include <Control/Sensor.h>

using namespace std;
namespace Ui {
class LogOptions;
}

class LogOptions : public QDialog
{
    Q_OBJECT
    
public:
    explicit LogOptions(QWidget *parent=0);
    ~LogOptions();
    
    void addrows(std::vector<bool> drawMeasurement);
    int selected_measurement;
    int selected_sensor;
    RobotSensors robotsensors;
    vector<bool> sensorDrawn;
    vector<vector<bool> > sensorMeasurementDrawn;

    void GetSensors();
public slots:
    void AddSensor(QString name);
    void addSensors(vector<string> sensors);
    void AddMeasurement(QString name);
    void ChangeSensor(int sensor);
    void Isolate();
    void ShowAll();
    void ShowItem(int index);
    void ShowItem();
    void HideItem(int index);
    void HideItem();
    void ChangeSelectedIndex(int index);
    void TogglePlot(bool status);
    void ToggleLogging(bool status);
    void AddMeasurements(vector<string> names);
signals:
    void SyncSensorMeasurements(int sensor);
    void toggle_measurement(int sensor,int measurement,bool status);
    void toggle_plot(bool status);
    void toggle_logging(bool status);
    void ShowSensor(int);
    void HideSensor(int);
private:
    Ui::LogOptions *ui;
    QFormLayout *currentLayout;
};

#endif // LOGOPTIONS_H
