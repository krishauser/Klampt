#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDir>

#include "Modeling/World.h"
#include "Interface/SimTestGUI.h"
#include "qrobottestguibase.h"

#include <collisionoutput.h>

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
    RobotWorld *world;
    QRobotTestGUIBase *gui;
    QTimer* refresh_timer;
    Robot* rob;
    int mode;
    int argc;
    const char** argv;
    void Initialize(int _argc, const char **_argv);
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
    void SetIK(bool status);

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
