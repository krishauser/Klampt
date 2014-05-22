#ifndef QROBOTTESTGUIBASE_H
#define QROBOTTESTGUIBASE_H

#include "KlamptQt/qtguibase.h"
#include "KlamptQt/qklamptdisplay.h"
#include "Interface/RobotTestGUI.h"
#include "collisionoutput.h"

class QRobotTestGUI : public QtGUIBase
{
    Q_OBJECT
public:
  explicit QRobotTestGUI(GenericBackendBase* _backend,QKlamptDisplay* display);
    virtual ~QRobotTestGUI();
    void SetDriver(int index);
    void SetLink(int index);
    void LoadFile(QString filename=QString());
    virtual bool OnCommand(const string &cmd, const string &args);
    virtual bool OnRefresh();
    void UpdateGUI();

    QKlamptDisplay* display;
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
