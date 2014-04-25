#ifndef QROBOTTESTGUIBASE_H
#define QROBOTTESTGUIBASE_H

#include "../KlamptQt/qtguibase.h"

#include <qrobotposebackend.h>
#include <collisionoutput.h>
class QRobotPoseGUI : public QtGUIBase
{
    Q_OBJECT
public:
    explicit QRobotPoseGUI(GenericBackendBase* _backend,RobotWorld* _world);
    void SetDriver(int index);
    void SetLink(int index);
    void LoadFile(QString filename=QString());
    void LoadFilePrompt(QString directory=".", QString filter="*");
    bool OnCommand(const string &cmd, const string &args);
    void UpdateGUI();

    QRobotPoseBackend* backend;

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
