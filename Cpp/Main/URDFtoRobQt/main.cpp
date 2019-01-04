#include <QApplication>
#include "dialog.h"
#include "Main/urdftorob.h"

int main(int argc, char *argv[])
{
    if(argc >= 2){
        return main_shell(argc,argv);
    }
    QApplication a(argc, argv);
    Dialog w;
    w.show();
    
    return a.exec();
}
