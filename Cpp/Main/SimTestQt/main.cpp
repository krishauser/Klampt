#include <QApplication>

#include <QFileDialog>
#include "mainwindow.h"
#include "QDebug"
#include <QSettings>

string toStdString(const QString& s)
{
	QByteArray arr = s.toUtf8();
	return string(arr.data());
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QString filename;
    //inithome directory
    //load settings from qsetings ini
    QCoreApplication::setOrganizationName("Klampt");
    QCoreApplication::setOrganizationDomain("klampt.org");
    QCoreApplication::setApplicationName("SimTest");
    QCoreApplication::setApplicationVersion("0.8.0");
    QSettings ini(QSettings::IniFormat, QSettings::UserScope,
		  QCoreApplication::organizationName(),
		  QCoreApplication::applicationName());
    QString dir = QFileInfo(ini.fileName()).absolutePath();

    QGLFormat glf = QGLFormat::defaultFormat();
    glf.setSampleBuffers(true);
    glf.setSamples(4);
    QGLFormat::setDefaultFormat(glf);

    MainWindow w;
    w.ini=&ini;
    if(argc==1){
		QString openDir = ini.value("last_open_scenario_directory", ".").toString();
		filename = QFileDialog::getOpenFileName(0, "Open Scenario", openDir, "Scenario (*.xml);;Robot (*.rob *.urdf);;Rigid Object (*.obj);;All Files (*)");
		if (filename.isNull()) return 0;
		ini.setValue("last_open_scenario_directory", QFileInfo(filename).absolutePath());
		//workaround for Qt 4.8.x crash on Windows
		string s = toStdString(filename);
      const char* args[3] = {"SimTest",s.c_str(),""};
      if(!w.Initialize(2,(const char**)args)) return 1;
    }
    else {
      if(!w.Initialize(argc,(const char**)argv)) return 1;
    }
    w.show();
    return a.exec();
}
