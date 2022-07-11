#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDir>
#include "Modeling/World.h"
#include "Interface/RobotTestGUI.h"
#include "qrobottestgui.h"
#include "collisionoutput.h"

using namespace Klampt;

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
    QDir directory;
    QString filename;
    WorldModel world;
    shared_ptr<RobotTestBackend> backend;
    shared_ptr<QRobotTestGUI> gui;
    int mode;
    int argc;
    const char** argv;
    bool Initialize(int _argc, const char **_argv);
    void StatusConnect();
private:
    Ui::MainWindow *ui;
public slots:
    void SetGeometry(bool status);
    void SetBboxes(bool status);
    void SetCOM(bool status);
    void SetFrame(bool status);
    void SetExpanded(bool status);
    void SetCollisions(bool status);
    void SetSensors(bool status);
    void SetIK(bool status);
    void SetROS(bool status);

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
    void PrintConfig();

    void LoadFile();
    void ReloadFile();

};

#endif // MAINWINDOW_H
