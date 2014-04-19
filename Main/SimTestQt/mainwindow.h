#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDir>

#include "Modeling/World.h"
#include "Interface/SimTestGUI.h"
#include "qsimtestgui.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QDir directory;
    QString filename;
    RobotWorld *world;
    QSimTestGUI *gui;
    QTimer* refresh_timer;
    int argc;
    const char** argv;
    void Initialize(int _argc, const char **_argv);
    void StatusConnect();
private:
    Ui::MainWindow *ui;
public slots:
    //all checkboxes
  void SetWrenches(bool status);
  void SetDesired(bool status);
  void SetEstimated(bool status);
  void SetBBoxes(bool status);
  void SetContacts(bool status);
  void SetPoser(bool status);
  void SetExpanded(bool status);
  void SetPoseIK(bool status);
  void SetSensorPlot(bool status);
  void SetLogCheck(bool status);
  void SetSimulate(bool status);
  void SetRecord(bool status);


  void SendMilestone();
  void SetMode(int);
  void DoFreeMode(){SetMode(0);}
  void DoIKMode(){SetMode(1);}
  void DoForceMode(){SetMode(2);}
  void LoadResource();
  void SaveScenario();
  void SaveLastScenario();
  void DefaultViewport();
  void LoadViewport();
  void SaveViewport();

  //Menus
  void ShowPlotOptions();
  void ShowDriverEdit();
  void ShowOptions();
  void ShowCommand();
  void ShowHelp();
  void ShowAbout();

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
};

#endif // MAINWINDOW_H
