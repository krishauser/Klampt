#include <QApplication>
#include "dialog.h"
#include "Main/motorcalibrate.h"

int main(int argc, char *argv[])
{
  if(argc > 1){
      return main_shell(argc,argv);
  }
  Robot::disableGeometryLoading = true;

    QApplication a(argc, argv);
    Dialog w;
    w.show();
    
    return a.exec();
}
