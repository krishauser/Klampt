#ifndef CONNECTSERIAL_H
#define CONNECTSERIAL_H

#include <QDialog>

namespace Ui {
class ConnectSerial;
}

class ConnectSerial : public QDialog
{
    Q_OBJECT
    
public:
    explicit ConnectSerial(int robots = 0, QWidget *parent = 0);
    ~ConnectSerial();
    void SetNumRobots(int n);
public slots:
    void OnTextEdit();
    void accept();
signals:
    void MakeConnect(int,QString,int,int);
private:
    Ui::ConnectSerial *ui;
};

#endif // CONNECTSERIAL_H
