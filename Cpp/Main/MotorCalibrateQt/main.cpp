#include <QApplication>
#include "dialog.h"
#include "Main/motorcalibrate.h"
using namespace Klampt;

int main(int argc, char *argv[])
{
  if(argc > 1){
      return main_shell(argc,argv);
  }
  RobotModel::disableGeometryLoading = true;

    QApplication a(argc, argv);
    Dialog w;
    w.show();
    
    return a.exec();
}
