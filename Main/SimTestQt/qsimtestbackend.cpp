#include "qsimtestbackend.h"
#include "GLdraw/GLView.h"

/** @brief QSimTestBackend.
 *
 * Messages are defined as follows.
 */

QSimTestBackend::QSimTestBackend() :
    SimTestBackend(new RobotWorld())
{
    MapButtonToggle("simulate",&simulate);
}

QSimTestBackend::~QSimTestBackend()
{
  delete world;
}


bool QSimTestBackend::OnCommand(const string &cmd, const string &args){
  return BaseT::OnCommand(cmd,args);
}

//widget specific function
bool QSimTestBackend::OnGLRender(){
    glShadeModel(GL_SMOOTH);
    glEnable(GL_MULTISAMPLE);

    BaseT::OnGLRender();
    return true;
}

bool QSimTestBackend::OnMouseMove(int mx, int my){
    BaseT::OnMouseMove(mx,my);
}

bool QSimTestBackend::OnMouseWheel(int dwheel){
  return SimTestBackend::OnMouseWheel(dwheel/10);
}


