#ifndef QTGUIBASE_H
#define QTGUIBASE_H

#include <Interface/GenericGUI.h>
#include "Modeling/World.h"

#include <QMouseEvent>
#include <QObject>
#include <QTimer>
#include <QFileDialog>
#include <QMessageBox>
using namespace Klampt;

class QtGUIBase : public QObject,public GenericGUIBase
{
    Q_OBJECT

public:
    QTimer* idle_timer;
    QtGUIBase(GenericBackendBase* _backend);
    ~QtGUIBase();
    void ShowHelp();
    void ShowAbout();
public slots:
    void SendMouseMove(QMouseEvent *e);
    void SendMouseWheel(QWheelEvent *e);
    void SendMousePress(QMouseEvent *e);
    void SendMouseRelease(QMouseEvent *e);
    void SendKeyDown(QKeyEvent *e);
    void SendKeyUp(QKeyEvent *e);
    void SendIdle();

signals:

};

#endif // QTGUIBASE_H
