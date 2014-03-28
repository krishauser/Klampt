#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDir>

#include "Modeling/World.h"
#include "Interface/SimTestGUI.h"
#include "qtguibase.h"

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
    QtGUIBase *gui;
    QTimer* refresh_timer;
    int argc;
    const char** argv;
    void Initialize(int _argc, const char **_argv);
    void StatusConnect();
private:
    Ui::MainWindow *ui;
public slots:
};

#endif // MAINWINDOW_H
