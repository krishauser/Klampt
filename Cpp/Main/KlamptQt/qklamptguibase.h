#ifndef QKLAMPTGUIBASE_H
#define QKLAMPTGUIBASE_H

#include "KlamptQt/qtguibase.h"
#include "KlamptQt/qklamptdisplay.h"
#include <Interface/GenericGUI.h>
#include <QTimer>

class QKlamptGUIBase : public QtGUIBase
{
    Q_OBJECT

public:
    QKlamptGUIBase(QKlamptDisplay* display,GenericBackendBase* _backend);
    virtual ~QKlamptGUIBase();
    QKlamptDisplay* display;
    QTimer idle_timer;
    
    virtual bool OnRefresh();
    virtual bool OnPauseIdle(double secs);
    virtual bool OnDrawText(int x, int y, const std::string& str, int height);
    virtual bool OnDrawText(double x, double y, double z, const std::string& str, int height);

public slots:
    void OnIdleTimer();
};

#endif // QKLAMPTGUIBASE_H
