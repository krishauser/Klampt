#ifndef CONTROLLERDIALOG_H
#define CONTROLLERDIALOG_H

#include <QDialog>
#include <QTableWidget>
#include <map>
#include <set>

using namespace std;
namespace Ui {
class ControllerDialog;
}

class RobotWorld;
class WorldSimulation;

class ControllerDialog : public QDialog
{
    Q_OBJECT
    
public:
  explicit ControllerDialog(WorldSimulation* sim=NULL,QWidget *parent = 0);
    ~ControllerDialog();

    WorldSimulation* sim;
    RobotWorld* world;
    std::map<string,string> settings;
    vector<string> commands;
    int refreshing;

    void Refresh();
public slots:
    void OnRobotChange(int);
    void OnSendCommand();
    void OnCellEdited(QTableWidgetItem*);
    void OnConnectSerial();
signals:
    void SendControllerSetting(int,string setting, string value);
    void ControllerCommand(int,string,string);
    void MakeConnect(int,QString,int,int);
private:
    Ui::ControllerDialog *ui;

};

#endif // CONTROLLERDIALOG_H
