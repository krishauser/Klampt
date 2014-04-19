#ifndef QSIMTESTGUI_H
#define QSIMTESTGUI_H

#include "KlamptQt/qtguibase.h"

#include <Interface/GenericGUI.h>
#include "Modeling/World.h"
#include <Interface/SimTestGUI.h>

#include <QMouseEvent>
#include <QObject>
#include <QTimer>
#include <QFileDialog>
#include <QMessageBox>

#include <driveredit.h>
#include <logoptions.h>
#include <controllersettings.h>
#include <controllercommanddialog.h>

class QSimTestGUI : public QtGUIBase
{
    Q_OBJECT

public:
    WorldSimulation* sim;
    DriverEdit *driver_tool;
    LogOptions *log_options;
    ControllerSettings *controller_settings;
    ControllerCommandDialog *command_dialog;
    QSimTestGUI(GenericBackendBase* _backend,RobotWorld* _world);
    QString old_filename;
    int constrain_mode,constrain_point_mode,delete_mode;
    bool OnCommand(const string &cmd, const string &args);
    void UpdateGUI();
    ~QSimTestGUI();
    void UpdateMeasurements();
    void LoadFile(QString filename=QString());
    void SaveScenario(QString filename=QString());
    void SaveLastScenario();
    void ShowHelp();
    void ShowAbout();
public slots:
    void SendMousePress(QMouseEvent *e);
    void SendDriverValue(int index, float value);
    void ShowSensor(int sensor);
    void HideSensor(int sensor);
    void SendMeasurement(int sensor, int measurement, bool status);
    void SendControllerSetting(string setting, string value);
    void SendControllerCommand(string setting, string value);

signals:
    void  EndDelete();
    void EndConstrain();
};

#endif // QTGUIBASE_H
