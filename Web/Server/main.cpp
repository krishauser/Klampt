#include <QCoreApplication>
#include "WebsocketServer.h"
//#include "simutilSimulation.h"
//#include "simtestSimulation.h"
#include "PythonWrapper.h"
#include <QThread>
#include <QObject>
#include <QString>
#include <signal.h>
#include <unistd.h>

WebsocketServer *server;

void handler(int sig) //allow us to get control-c 
{
   printf("\nquit the application (user request signal = %d).\n", sig);
   delete server;
   QCoreApplication::quit();
}

void error_handler(QString e)
{
   printf("received error string: %s\n",e.toStdString().c_str());
   handler(-1);
}

void catch_signals() //TODO: linux specific, should be more cross platform
{
   signal(SIGQUIT, handler);
   signal(SIGINT, handler);
   signal(SIGTERM, handler);
   signal(SIGHUP, handler);
}

int main(int argc, char **argv)
{
   QCoreApplication a(argc, argv);
   catch_signals();

   server=new WebsocketServer(1234);

   //According to some people, proper way is NOT to subclass QThread. see article:
   //https://mayaposch.wordpress.com/2011/11/01/how-to-really-truly-use-qthreads-the-full-explanation/

   QThread* thread = new QThread;
   PythonWrapper *simlulation = new PythonWrapper(argc,argv);

   simlulation->moveToThread(thread);

   QObject::connect(simlulation, &PythonWrapper::error, error_handler);
   QObject::connect(simlulation, SIGNAL(finished()),        thread,           SLOT(quit()));
   QObject::connect(simlulation, SIGNAL(finished()),        simlulation,      SLOT(deleteLater()));
   QObject::connect(simlulation, SIGNAL(sendData(QString)), server,           SLOT(sendToAll(QString)));  
   QObject::connect(thread,      SIGNAL(finished()),        thread,           SLOT(deleteLater()));
   QObject::connect(server,      SIGNAL(incomingMessage(QString)), simlulation, SLOT(incomingMessage(QString)));
   QObject::connect(server,      SIGNAL(newConnection()),   simlulation,       SLOT(initialize())); 

   thread->start();

   return a.exec();
}
