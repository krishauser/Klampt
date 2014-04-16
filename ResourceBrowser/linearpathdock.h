#ifndef LINEARPATHDOCK_H
#define LINEARPATHDOCK_H

#include <QDockWidget>
#include <QListWidget>

#include <Modeling/Resources.h>

#include "resourcedockwidget.h"

class LinearPathDock : public ResourceDockWidget
{
    Q_OBJECT
    
public:
    QListWidget* lst_times;

    explicit LinearPathDock(QString filename=QString(),int* valid=NULL,QWidget *parent = 0);
    ~LinearPathDock();
    LinearPathDock(ResourcePtr &ptr, QWidget *parent = 0);
    void Initialize(LinearPathResource *r);
public slots:
    void ToGUI();
private:
    Ui::ResourceDockWidget *ui;
};

#endif // LinearPathDock_H
