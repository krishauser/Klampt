#ifndef MULTIPATHDOCK_H
#define MULTIPATHDOCK_H

#include <QDockWidget>
#include <QListWidget>

#include <Modeling/Resources.h>


#include "resourcedockwidget.h"


class MultiPathDock : public ResourceDockWidget
{
    Q_OBJECT
    
public:
    QListWidget* lst_times;

    explicit MultiPathDock(QString filename=QString(),int* valid=NULL,QWidget *parent = 0);
    explicit MultiPathDock(ResourcePtr&, QWidget *parent=0);
    ~MultiPathDock();

    void Initialize(MultiPathResource*);
public slots:
    void Expand();
private:
    Ui::ResourceDockWidget *ui;
};

#endif // MultiPathDock_H
