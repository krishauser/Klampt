#include <QApplication>
#include <QFileDialog>
#include "mainwindow.h"
#include "QDebug"
#include <QSettings>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QString filename;
    //load settings from qsetings ini
    QCoreApplication::setOrganizationName("Klampt");
    QCoreApplication::setOrganizationDomain("klampt.org");
    QCoreApplication::setApplicationName("RobotTest");
    QCoreApplication::setApplicationVersion("0.8");
    QSettings ini(QSettings::IniFormat, QSettings::UserScope,
          QCoreApplication::organizationName(),
          QCoreApplication::applicationName());
    QString dir = QFileInfo(ini.fileName()).absolutePath();

    QGLFormat glf = QGLFormat::defaultFormat();
    glf.setSampleBuffers(true);
    glf.setSamples(4);
    QGLFormat::setDefaultFormat(glf);

    MainWindow w;
    if(argc==1){
        a.setQuitOnLastWindowClosed(false);
        QString openDir = ini.value("last_open_robot_directory", ".").toString();
        filename = QFileDialog::getOpenFileName(0, "Open Robot", openDir, "Robot (*.rob *.urdf);;Scenario (*.xml);;All Files (*)");
        if (filename.isNull()) return 0;
        ini.setValue("last_open_robot_directory", QFileInfo(filename).absolutePath());

        //workaround for Qt 4.8.x crash on Windows
        QByteArray arr = filename.toUtf8();
        string s (arr.data());
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
    a.setQuitOnLastWindowClosed(true);
    w.directory=dir;
    w.show();
    return a.exec();
}
