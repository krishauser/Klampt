#include "qklamptguibase.h"

QKlamptGUIBase::QKlamptGUIBase(QKlamptDisplay* _display,GenericBackendBase *_backend) :
  QtGUIBase(_backend), display(_display)
{
  assert(_backend != NULL);
  _backend->gui = this;
  display->gui = this;

  connect(&idle_timer, SIGNAL(timeout()),this,SLOT(OnIdleTimer()));
  idle_timer.start(0);
}

void QKlamptGUIBase::OnIdleTimer()
{
  //printf("Idle.\n");
  idle_timer.start(0);
  SendIdle();
}


bool QKlamptGUIBase::OnPauseIdle(double secs) 
{
  if(secs > 10000000) {
    idle_timer.stop();
    //printf("Pause idle.\n",secs);
  }
  else {
    idle_timer.start(int(secs*1000));
    //printf("Pause idle %g.\n",secs);
  }
  return true;
}

bool QKlamptGUIBase::OnRefresh()
{
  if(!display->painted) {
    //printf("Skipping refresh, not painted yet\n");
    return true;
  }
  display->painted = false;
  display->update();
  return true;
}

bool QKlamptGUIBase::OnDrawText(double x, double y, double z, const std::string &text, int height)
{
  QFont font;
  font.setPointSize(height);
  display->renderText(x, y, z, QString(text.c_str()), font);
  return true;
}

bool QKlamptGUIBase::OnDrawText(int x, int y, const std::string &text, int height)
{
  QFont font;
  font.setPointSize(height);
  display->renderText(x, y, QString(text.c_str()), font);
  return true;
}

QKlamptGUIBase::~QKlamptGUIBase(){
}

