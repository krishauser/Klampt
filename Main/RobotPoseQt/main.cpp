#include <QtGui/QApplication>

#include <QFileDialog>
#include <QSettings>

#include "mainwindow.h"
#include "glut.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    glutInit(&argc,argv);
    QString filename;
    //inithome directory
    QDir directory=QDir::home();

    //load settings from qsetings ini
    QCoreApplication::setOrganizationName("Klampt");
    QCoreApplication::setOrganizationDomain("klampt.org");
    QCoreApplication::setApplicationName("RobotTest");
    QCoreApplication::setApplicationVersion("0.6");
    QSettings ini(QSettings::IniFormat, QSettings::UserScope,
          QCoreApplication::organizationName(),
          QCoreApplication::applicationName());
    QString dir = QFileInfo(ini.fileName()).absolutePath();

    if(argc==1){
        QFileDialog f;
        QString openDir = ini.value("last_open_scenario_directory",".").toString();
        filename = f.getOpenFileName(0,"Open Scenario",openDir,"*.rob");
        if(filename.isNull()) return 0;
        ini.setValue("last_open_scenario_directory",QFileInfo(filename).absolutePath());
      }
      MainWindow w;
      if(argc==1){
          string s = filename.toStdString();
          const char* c = s.c_str();
          const char* args[3] = {"RobotTest",c,""};
          w.Initialize(2,(const char**)args);
      }
      else
          w.Initialize(argc,(const char**)argv);
      w.directory=directory;
    w.show();
    return a.exec();
}
