#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "GLUTGUI.h"
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <limits.h>
#include <assert.h>
#include <iostream>
#include <stdio.h>

const char* special_map(int k) {
  switch(k){
  case GLUT_KEY_F1: return "f1";
  case GLUT_KEY_F2: return "f2";
  case GLUT_KEY_F3: return "f3";
  case GLUT_KEY_F4: return "f4";
  case GLUT_KEY_F5: return "f5";
  case GLUT_KEY_F6: return "f6";
  case GLUT_KEY_F7: return "f7";
  case GLUT_KEY_F8: return "f8";
  case GLUT_KEY_F9: return "f9";
  case GLUT_KEY_F10: return "f10";
  case GLUT_KEY_F11: return "f11";
  case GLUT_KEY_F12: return "f12";
  case GLUT_KEY_LEFT: return "left";
  case GLUT_KEY_UP: return "up";
  case GLUT_KEY_RIGHT: return "right";
  case GLUT_KEY_DOWN: return "down";
  case GLUT_KEY_PAGE_UP: return "pageup";
  case GLUT_KEY_PAGE_DOWN: return "pagedown";
  case GLUT_KEY_HOME: return "home";
  case GLUT_KEY_END: return "end";
  case GLUT_KEY_INSERT: return "insert";
  default:
    return "unknown_key";
  }
}

GLUTGUI::GLUTGUI(GenericBackendBase* backend,int w,int h)
  :GenericGUIBase(backend),GLUTProgramBase(w,h),window_title("OpenGL Viewer")
{}

void GLUTGUI::Handle_Display()
{
  SendGLRender();
  glutSwapBuffers();
}

void GLUTGUI::Handle_Reshape(int w,int h)
{
  width=w; height=h;
  SendGLViewport(0,0,w,h);
}
void GLUTGUI::Handle_Keypress(unsigned char key,int x,int y)
{
  SendKeyDown(string(1,key));
}
void GLUTGUI::Handle_KeypressUp(unsigned char key,int x,int y)
{
  SendKeyUp(string(1,key));
}
void GLUTGUI::Handle_Special(int key,int x,int y) 
{
  SendKeyDown(special_map(key)); 
}
void GLUTGUI::Handle_SpecialUp(int key,int x,int y) 
{
  SendKeyUp(special_map(key)); 
}
void GLUTGUI::Handle_Click(int button,int state,int x,int y)
{
  int clickModifiers = glutGetModifiers();
  if(clickModifiers & GLUT_ACTIVE_CTRL) SendKeyDown("control");
  else SendKeyUp("control");
  if(clickModifiers & GLUT_ACTIVE_SHIFT) SendKeyDown("shift");
  else SendKeyUp("shift");
  if(clickModifiers & GLUT_ACTIVE_ALT) SendKeyDown("alt");
  else SendKeyUp("alt");
  SendMouseClick(button,1-state,x,y); 
}

void GLUTGUI::Handle_Drag(int x,int y)
{
  SendMouseMove(x,y);
}

void GLUTGUI::Handle_Motion(int x,int y)
{
  SendMouseMove(x,y);
}

void GLUTGUI::Handle_Idle()
{
  SendIdle();
}

void GLUTGUI::Run()
{
  GLUTProgramBase::Run(window_title.c_str(),displayMode);
}

bool GLUTGUI::Initialize()
{
  backend->Start();
  glEnable (GL_DEPTH_TEST);
  glEnable (GL_CULL_FACE);
  return true;
}

bool GLUTGUI::OnRefresh()
{
  glutPostRedisplay();
  return true;
}

bool GLUTGUI::OnQuit()
{
  exit(0);
  return true;
}

bool GLUTGUI::OnNotify(const string& text,const string& msglevel)
{
  LOG4CXX_INFO(KrisLibrary::logger(),"GLUTGUI::OnNotify:"<<text<<"\n");
  return true;
}

bool GLUTGUI::OnPauseIdle(double secs)
{
  if(secs > INT_MAX/1000)
    SleepIdleCallback(INT_MAX);
  else
    SleepIdleCallback(int(secs*1000));
  return true;
}


