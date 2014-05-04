#include <QtGui/QApplication>

#include <QFileDialog>
#include "mainwindow.h"
#include <GL/glut.h>
#include "QDebug"
#include <QSettings>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    glutInit(&argc,argv);
    QString filename;
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
        QString openDir = ini.value("last_open_robot_directory",".").toString();
        filename = f.getOpenFileName(0,"Open Robot",openDir,"Robot (*.rob);;Scenario (*.xml);;All Files (*)");
        if(filename.isNull()) return 0;
        ini.setValue("last_open_robot_directory",QFileInfo(filename).absolutePath());
      }
      MainWindow w;
      if(argc==1){
          //do not simplify the proceeding lines, this makes it work somehow
          string s = filename.toStdString();
          const char* c = s.c_str();
          const char* args[3] = {"RobotTest",c,""};
          w.Initialize(2,(const char**)args);
      }
      else{
          string s = argv[1];
          w.Initialize(argc,(const char**)argv);
      }
      w.directory=dir;
    w.show();
    return a.exec();
}
