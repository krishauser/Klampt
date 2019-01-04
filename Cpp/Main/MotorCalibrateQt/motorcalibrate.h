#ifndef MOTORCALIBRATE_H
#define MOTORCALIBRATE_H

#include <QMainWindow>

namespace Ui {
class MotorCalibrate;
}

class MotorCalibrate : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MotorCalibrate(QWidget *parent = 0);
    ~MotorCalibrate();
    
private:
    Ui::MotorCalibrate *ui;
};

#endif // MOTORCALIBRATE_H
