#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDir>
#include <QSettings>

#include "Modeling/World.h"
#include "Interface/RobotPoseGUI.h"
#include "qrobotposegui.h"

#include <collisionoutput.h>
#include <resourcemanager.h>

namespace Ui {
class MainWindow;
}

namespace RobotTest {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    RobotWorld world;
    SmartPointer<QRobotPoseGUI> gui;
    SmartPointer<RobotPoseBackend> backend;
    SmartPointer<ResourceManager> manager;
    int mode;
    void Initialize(int _argc, const char **_argv);
    void StatusConnect();
private:
    Ui::MainWindow *ui;
public slots:
    void SetGeometry(bool status);
    void SetPoser(bool status);
    void SetBboxes(bool status);
    void SetCOM(bool status);
    void SetFrame(bool status);
    void SetFree();
    void SetIK();
    void IKConstrain();
    void IKConstrainPoint();
    void IKDelete();

    void SetDriver(int index);
    void UpdateDriverValue();
    void UpdateDriverParameters();

    void SetLink(int index);
    void UpdateLinkValue();
    void UpdateLinkParameters();

    void LinkMode();
    void DriverMode();
    void SliderLinkAngle(int ticks);
    void SliderDriverAngle(int ticks);
    void UpdateLinkSlider(double value);
    void UpdateDriverSlider(double value);
    void PrintCollisions();

    void LoadFile();
};

#endif // MAINWINDOW_H
