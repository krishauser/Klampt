#ifndef QROBOTTESTGUIBASE_H
#define QROBOTTESTGUIBASE_H

#include "qtguibase.h"

#include <qrobottestbackend.h>
#include <collisionoutput.h>
class QRobotTestGUI : public QtGUIBase
{
    Q_OBJECT
public:
    explicit QRobotTestGUI(GenericBackendBase* _backend,RobotWorld* _world);
    void SetDriver(int index);
    void SetLink(int index);
    void LoadFile(QString filename=QString());
    bool OnCommand(const string &cmd, const string &args);
    void UpdateGUI();

    QRobotTestBackend* backend;
    CollisionOutput* col_out;

    int driver_index;
    int link_index;
public slots:
    void SetDriverValue(double val);
    void SetLinkValue(double val);

signals:
    void UpdateDriverValue();
    void UpdateDriverParameters();

    void UpdateLinkValue();
    void UpdateLinkParameters();
};

#endif // QROBOTTESTGUIBASE_H
