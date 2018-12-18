#include <QApplication>

#include <QFileDialog>
#include <QSettings>

#include "mainwindow.h"


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QString filename;

    //load settings from qsetings ini
    QCoreApplication::setOrganizationName("Klampt");
    QCoreApplication::setOrganizationDomain("klampt.org");
    QCoreApplication::setApplicationName("RobotPose");
    QCoreApplication::setApplicationVersion("0.8.0");
    QSettings ini(QSettings::IniFormat, QSettings::UserScope,
          QCoreApplication::organizationName(),
          QCoreApplication::applicationName());

    QGLFormat glf = QGLFormat::defaultFormat();
    glf.setSampleBuffers(true);
    glf.setSamples(4);
    QGLFormat::setDefaultFormat(glf);

    QString dir = QFileInfo(ini.fileName()).absolutePath();
    if(argc==1){
        QString openDir = ini.value("last_open_scenario_directory",".").toString();
        filename = QFileDialog::getOpenFileName(0,"Open Robot",openDir,"Robot (*.rob *.urdf);;Rigid Object (*.obj);;Scenario (*.xml);;All Files (*)");
        if(filename.isNull()) return 0;
        ini.setValue("last_open_scenario_directory",QFileInfo(filename).absolutePath());
      }
      MainWindow w;
      if(argc==1){
          QByteArray arr = filename.toUtf8();
		  string s(arr.data());
          const char* args[3] = {"RobotPose",s.c_str(),""};
          w.Initialize(2,(const char**)args);
      }
      else
          w.Initialize(argc,(const char**)argv);
    w.show();
    return a.exec();
}
