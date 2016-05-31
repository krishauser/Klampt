#ifndef DJZ_PYTHON_WRAPPER
#define DJZ_PYTHON_WRAPPER

#include <QObject>
#include <Python.h>

class PythonWrapper : public QObject {
    Q_OBJECT
 
public:
    PythonWrapper(int argc, char **argv);
    ~PythonWrapper();

    void send(QString data);

public slots:
    void initialize();
    void incomingMessage(QString message);
   
signals:
    void finished();
    void sendData(QString data);
    void error(QString err);
 
private:
    // add your variables here
    int argc;
    char **argv;
};

#endif
