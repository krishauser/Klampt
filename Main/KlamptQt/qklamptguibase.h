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
    QTimer display_timer;

    virtual bool OnRefresh();
    virtual bool OnPauseIdle(double secs);

public slots:
    void OnIdleTimer();
    void OnDisplayTimer();
};

#endif // QKLAMPTGUIBASE_H
