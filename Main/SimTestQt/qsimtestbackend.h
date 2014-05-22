#ifndef QSIMTESTBACKEND_H
#define QSIMTESTBACKEND_H

#include <Interface/SimTestGUI.h>

class QSimTestBackend : public SimTestBackend
{
public:
    typedef SimTestBackend BaseT;
    
    QSimTestBackend();
    virtual ~QSimTestBackend();
    virtual bool OnGLRender();
    virtual bool OnMouseMove(int mx, int my);
    virtual bool OnMouseWheel(int dwheel);
    virtual bool OnCommand(const string &cmd, const string &args);   
};

#endif // QSIMTESTBACKEND_H
