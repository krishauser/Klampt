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
    explicit ConnectSerial(QWidget *parent = 0);
    ~ConnectSerial();
public slots:
    void accepted();
private:
    Ui::ConnectSerial *ui;
};

#endif // CONNECTSERIAL_H
