#ifndef QROBOTTESTGUIBASE_H
#define QROBOTTESTGUIBASE_H

#include "KlamptQt/qklamptguibase.h"
#include "Interface/RobotTestGUI.h"
#include "collisionoutput.h"

class QRobotTestGUI : public QKlamptGUIBase
{
    Q_OBJECT
public:
  explicit QRobotTestGUI(QKlamptDisplay* display,GenericBackendBase* _backend,RobotModel* robot);
    virtual ~QRobotTestGUI();
    void SetDriver(int index);
    void SetLink(int index);
    void LoadFile(QString filename=QString());
    void ReloadFile();
    virtual bool OnCommand(const string &cmd, const string &args);
    void UpdateGUI();

    RobotModel* robot;
    CollisionOutput* col_out;
    QString opened_file;

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
