#ifndef QTGUIBASE_H
#define QTGUIBASE_H

#include <Interface/GenericGUI.h>
#include "Modeling/World.h"
#include <Interface/SimTestGUI.h>

#include <QMouseEvent>
#include <QObject>
#include <QTimer>
#include <QFileDialog>
#include <driveredit.h>
#include <logoptions.h>
#include <controllersettings.h>

class QtGUIBase : public QObject,public GenericGUIBase
{
    Q_OBJECT

public:
    RobotWorld *world;
    WorldSimulation* sim;
    QTimer* idle_timer;
    DriverEdit *driver_tool;
    LogOptions *log_options;
    ControllerSettings *controller_settings;
    QtGUIBase(GenericBackendBase* _backend,RobotWorld* _world);
    QString old_filename;
    int constrain_mode,delete_mode;
    bool OnCommand(const string &cmd, const string &args);
    void UpdateGUI();
    ~QtGUIBase();
    void UpdateMeasurements();
    void LoadFile(QString filename=QString());
    void SaveScenario(QString filename=QString());
    void SaveLastScenario();
public slots:
    void SendMouseMove(QMouseEvent *e);
    void SendMouseWheel(QWheelEvent *e);
    void SendMousePress(QMouseEvent *e);
    void SendMouseRelease(QMouseEvent *e);
    void SendKeyDown(QKeyEvent *e);
    void SendKeyUp(QKeyEvent *e);
    void SendIdle();
    void SendDriverValue(int index, float value);
    void ShowSensor(int sensor);
    void HideSensor(int sensor);
    void SendMeasurement(int sensor, int measurement, bool status);
    void SendControllerSetting(string setting, string value);
signals:
    void  EndDelete();
    void EndConstrain();
};

#endif // QTGUIBASE_H
