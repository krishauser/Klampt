#ifndef CONTROLLERSETTINGS_H
#define CONTROLLERSETTINGS_H

#include <QDialog>
#include <map>
#include <set>
#include <boost/foreach.hpp>

using namespace std;
namespace Ui {
class ControllerSettings;
}

class ControllerSettings : public QDialog
{
    Q_OBJECT
    
public:
    explicit ControllerSettings(QWidget *parent = 0);
    ~ControllerSettings();
    std::map<string,string> settings;
    void Refresh();
    set<string> edited;
    int refreshing;
public slots:
    void OnCellEdited(int row, int col);
    void OnApply();
signals:
    void SendControllerSetting(string setting, string value);
private:
    Ui::ControllerSettings *ui;

};

#endif // CONTROLLERSETTINGS_H
