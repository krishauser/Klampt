#ifndef QSIMTESTGUI_H
#define QSIMTESTGUI_H

#include "KlamptQt/qklamptguibase.h"

#include <Interface/GenericGUI.h>
#include "Modeling/World.h"
#include <Interface/SimTestGUI.h>

#include <QMouseEvent>
#include <QObject>
#include <QFileDialog>
#include <QMessageBox>
#include <QSettings>

#include <driveredit.h>
#include <logoptions.h>
#include <controllerdialog.h>

class QSimTestGUI : public QKlamptGUIBase
{
    Q_OBJECT

public:
    QSimTestGUI(QKlamptDisplay* display,SimTestBackend* _backend);
    virtual ~QSimTestGUI();
    Simulator* sim;
    DriverEdit *driver_tool;
    LogOptions *log_options;
    ControllerDialog *controller_dialog;
    QString old_filename;
    QSettings* ini;

    virtual bool OnCommand(const string& cmd,const string& args);
    void UpdateGUI();
    void UpdateMeasurements();
    void LoadFile(QString filename=QString());
    void LoadFilePrompt(QString directory_key=".",QString filter="*");
    void SaveScenario(QString filename=QString());
    void SaveLastScenario();
    void ShowHelp();
    void ShowAbout();
public slots:
    void SendDriverValue(int index, float value);
    void ShowSensor(int sensor,bool shown);
    void RenderSensor(int sensor,bool rendered);
    void SendMeasurement(int sensor, int measurement, bool status);
    void SendControllerSetting(int robot,string setting, string value);
    void SendControllerCommand(int robot,string setting, string value);
    void SendConnection(int robot,QString host,int port,int rate);
};

#endif // QTGUIBASE_H
