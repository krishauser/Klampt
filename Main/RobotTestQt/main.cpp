#include <QtGui/QApplication>
#include <QFileDialog>
#include "mainwindow.h"
#include "QDebug"
#include <QSettings>
/*
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
*/

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    //needed for GLUTBitmapCharacter
    //glutInit(&argc,argv);
    QString filename;
    //load settings from qsetings ini
    QCoreApplication::setOrganizationName("Klampt");
    QCoreApplication::setOrganizationDomain("klampt.org");
    QCoreApplication::setApplicationName("RobotTest");
    QCoreApplication::setApplicationVersion("0.6.2");
    QSettings ini(QSettings::IniFormat, QSettings::UserScope,
		  QCoreApplication::organizationName(),
		  QCoreApplication::applicationName());
    QString dir = QFileInfo(ini.fileName()).absolutePath();

    QGLFormat glf = QGLFormat::defaultFormat();
    glf.setSampleBuffers(true);
    glf.setSamples(4);
    QGLFormat::setDefaultFormat(glf);

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
      if(!w.Initialize(2,(const char**)args)) {
	printf("Failed to initialize\n");
	return 1;
      }
    }
    else{
      if(!w.Initialize(argc,(const char**)argv)) {
	printf("Failed to initialize\n");
	return 1;
      }
    }
    w.directory=dir;
    w.show();
    return a.exec();
}
