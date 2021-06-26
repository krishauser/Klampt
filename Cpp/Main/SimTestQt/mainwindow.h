#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDir>
#include <QSettings>
#include <QTimer>

#include "Modeling/World.h"
#include "Interface/SimTestGUI.h"
#include "qsimtestgui.h"

using namespace Klampt;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QSettings* ini;
    WorldModel world;
    shared_ptr<SimTestBackend> backend;
    shared_ptr<QSimTestGUI> gui;
    bool Initialize(int _argc, const char **_argv);
    void StatusConnect();
private:
    Ui::MainWindow *ui;
public slots:
    //all checkboxes
  void SetWrenches(bool status);
  void SetDesired(bool status);
  void SetDrawTime(bool status);
  void SetEstimated(bool status);
  void SetBBoxes(bool status);
  void SetContacts(bool status);
  void SetPoser(bool status);
  void SetPoseObjects(bool status);
  void SetExpanded(bool status);
  void SetPoseIK(bool status);
  void SetSensorPlot(bool status);
  void SetLogCheck(bool status);
  void SetSimulate(bool status);
  void SetRecord(bool status);
  void SetROS(bool status);

  void SendMilestone();
  void SetMode(int);
  void DoFreeMode(){SetMode(0);}
  void DoIKMode(){SetMode(1);}
  void DoForceMode(){SetMode(2);}
  void LoadResource();
  void LoadPath();
  void LoadState();
  void SaveScenario();
  void SaveLastScenario();
  void DefaultViewport();
  void LoadViewport();
  void SaveViewport();

  //Menus
  void ShowPlotOptions();
  void ShowDriverEdit();
  void ShowController();
  void ShowHelp();
  void ShowAbout();

  //logging
  void LogSimulation(bool status);
  void LogContactState(bool status);
  void LogContactWrenches(bool status);
  void LogSensedPath(bool status);
  void LogCommandedPath(bool status);
  void ChangeSimulationLogFile();
  void ChangeContactStateLogFile();
  void ChangeContactWrenchesLogFile();
  void ChangeSensedPathLogFile();
  void ChangeCommandedPathLogFile();

  //IK Mode
  void IKConstrain();
  void IKConstrainPoint();
  void IKDelete();

  //reset
  void Reset();

  //recording
  void ChangeRecordFile();
  void ChangeResolution(int w,int h);
  void Resolution640x480(){ChangeResolution(640,480);}
  void Resolution800x600(){ChangeResolution(800,600);}
  void Resolution1280x768(){ChangeResolution(1280,768);}
  void Resolution1920x1080(){ChangeResolution(1920,1080);}
  void FlexibleResolution();
  void Shrink(){this->resize(0,0);}
  void DoMultipleParts(bool status);
  void Encode();
  void ChangeEncoderCommand();

};

#endif // MAINWINDOW_H
