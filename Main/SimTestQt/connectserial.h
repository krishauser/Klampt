#ifndef CONNECTSERIAL_H
#define CONNECTSERIAL_H

#include <QDialog>

namespace Ui {
class ConnectSerial;
}

class RobotWorld;

class ConnectSerial : public QDialog
{
    Q_OBJECT
    
public:
    explicit ConnectSerial(RobotWorld* world=NULL, QWidget *parent = 0);
    ~ConnectSerial();
    void SetNumRobots(RobotWorld* world);
public slots:
    void accept();
signals:
    void MakeConnect(int,QString,int,int);
private:
    Ui::ConnectSerial *ui;
};

#endif // CONNECTSERIAL_H
