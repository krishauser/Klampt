#ifndef QROBOTTESTGUIBASE_H
#define QROBOTTESTGUIBASE_H

#include "KlamptQt/qtguibase.h"
#include "KlamptQt/qklamptdisplay.h"

#include "Interface/RobotPoseGUI.h"
#include <collisionoutput.h>
#include <resourceframe.h>
#include <playresourceframe.h>

class QRobotPoseGUI : public QtGUIBase
{
    Q_OBJECT
public:
    explicit QRobotPoseGUI(QKlamptDisplay* display,RobotPoseBackend* _backend);
    void SetDriver(int index);
    void SetLink(int index);
    void LoadFile(QString filename=QString());
    void LoadFilePrompt(QString directory_key="", QString filter="*");
    virtual bool OnCommand(const string &cmd, const string &args);
    virtual bool OnRefresh();
    void UpdateGUI();

    QKlamptDisplay* display;
    ResourceFrame* resource_frame;

    int driver_index;
    int link_index;
public slots:
    void SetDriverValue(double val);
    void SetLinkValue(double val);
    void SetRecord(bool status);

signals:
    void UpdateDriverValue();
    void UpdateDriverParameters();

    void UpdateLinkValue();
    void UpdateLinkParameters();
};

#endif // QROBOTTESTGUIBASE_H
