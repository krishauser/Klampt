#ifndef CONTROLLERCOMMANDDIALOG_H
#define CONTROLLERCOMMANDDIALOG_H

#include <QDialog>
#include <vector>
using namespace std;

namespace Ui {
class ControllerCommandDialog;
}

class ControllerCommandDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit ControllerCommandDialog(QWidget *parent = 0);
    ~ControllerCommandDialog();

    vector<string> commands;

    void Refresh();
public slots:
    void SendCommand();
signals:
    void ControllerCommand(string,string);
private:
    Ui::ControllerCommandDialog *ui;
};

#endif // CONTROLLERCOMMANDDIALOG_H
