#ifndef StanceDock_H
#define StanceDock_H

#include <QDockWidget>
#include <QListWidget>

#include <Modeling/Resources.h>

#include "resourcedockwidget.h"

class StanceDock : public ResourceDockWidget
{
    Q_OBJECT
    
public:
    QListWidget* lst_holds;
    std::map<int,ResourcePtr> map_holds;

    explicit StanceDock(QString filename=QString(),int* valid=NULL,QWidget *parent = 0);
    ~StanceDock();
    StanceDock(ResourcePtr &ptr, QWidget *parent = 0);
    void Initialize(StanceResource *s);

public slots:
    void ToGUI();
    void Expand();
private:
    //Ui::ResourceDockWidget *ui;
};

#endif // StanceDock_H
