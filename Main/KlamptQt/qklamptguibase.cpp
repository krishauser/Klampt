#include "qklamptguibase.h"

QKlamptGUIBase::QKlamptGUIBase(QKlamptDisplay* _display,GenericBackendBase *_backend) :
  QtGUIBase(_backend), display(_display)
{
  assert(_backend != NULL);
  _backend->gui = this;
  display->gui = this;

  connect(&idle_timer, SIGNAL(timeout()),this,SLOT(OnIdleTimer()));
  idle_timer.start(0);
  connect(&display_timer, SIGNAL(timeout()),this,SLOT(OnDisplayTimer()));
}

void QKlamptGUIBase::OnIdleTimer()
{
  SendIdle();
  idle_timer.start(0);
}

void QKlamptGUIBase::OnDisplayTimer()
{
  display->updateGL();
  display_timer.stop();
}

bool QKlamptGUIBase::OnPauseIdle(double secs) 
{
  if(secs > 10000000)
    idle_timer.stop();
  else
    idle_timer.start(int(secs*1000));
  return true;
}

bool QKlamptGUIBase::OnRefresh()
{
  if(!display->painted) return true;
  display->painted = false;
  display_timer.start(0);
  return true;
}

QKlamptGUIBase::~QKlamptGUIBase(){
}

