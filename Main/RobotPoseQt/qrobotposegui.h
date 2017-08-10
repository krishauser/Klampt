#ifndef QROBOTTESTGUIBASE_H
#define QROBOTTESTGUIBASE_H

#include "KlamptQt/qklamptguibase.h"

#include "Interface/RobotPoseGUI.h"
#include <collisionoutput.h>
#include <resourceframe.h>
#include <playresourceframe.h>

class QRobotPoseGUI : public QKlamptGUIBase
{
    Q_OBJECT
public:
    explicit QRobotPoseGUI(QKlamptDisplay* display,RobotPoseBackend* _backend);
    void SetDriver(int index);
    void SetLink(int index);
    void LoadFile(QString filename=QString());
    void LoadFilePrompt(QString directory_key="", QString filter="*");
    void SaveFilePrompt(QString directory_key="");
    virtual bool OnCommand(const string &cmd, const string &args);
    void UpdateGUI();

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
