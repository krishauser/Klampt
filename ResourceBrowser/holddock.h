#ifndef HOLDDOCK_H
#define HOLDDOCK_H

#include <QDockWidget>
#include <QListWidget>

#include "resourcedockwidget.h"
#include <Modeling/Resources.h>

class HoldDock : public ResourceDockWidget
{
    Q_OBJECT
    
public:
    QListWidget* lst_contacts;
    QListWidget* lst_ik;

    explicit HoldDock(HoldResource* h,QWidget *parent=NULL);
    ~HoldDock();
public slots:
    void ToGUI();
    void Expand();
private:
    Ui::ResourceDockWidget *ui;
};

#endif // HoldDock_H
