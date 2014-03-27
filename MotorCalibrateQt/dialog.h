#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QFileDialog>
#include <QListWidgetItem>
#include "showtext.h"
namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit Dialog(QWidget *parent = 0);
  ~Dialog();
public slots:
    int LoadRobot();
    int AddPaths();
    int SetPathIndex(int i);
    int DeletePaths();
    int DoCalibrate();

private:
    Ui::Dialog *ui;
    QFileDialog f;
    QString robotFilename;
    ShowText *popup;

};

#endif // DIALOG_H
