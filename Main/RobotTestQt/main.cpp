#include <QtGui/QApplication>

#include <QFileDialog>
#include "mainwindow.h"
#include "glut.h"
#include "QDebug"
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    glutInit(&argc,argv);
    QString filename;
    //inithome directory
    QDir directory=QDir::home();
  #ifdef WIN32
    directory.cd("Application Data");
    directory.mkdir("klampt");
    directory.cd("klampt");
  #else
    directory.mkdir(".klampt");
    directory.cd(".klampt");
  #endif
    directory.mkdir("states");
    directory.mkdir("linearpath");
    directory.mkdir("milestonepath");
    directory.mkdir("multipath");
    directory.mkdir("commandlog");

    if(argc==1){
        QFileDialog f;
        filename = f.getOpenFileName(0,"Open Robot","../../data/robots","*.rob");
        if(filename.isNull()) return 0;
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
      w.directory=directory;
    w.show();
    return a.exec();
}
