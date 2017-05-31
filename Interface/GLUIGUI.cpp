#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "GLUIGUI.h"
#include <limits.h>
#include <sstream>
#include <assert.h>
#include <stdio.h>
#include <iostream>

#include <GL/glui.h>

//defined in GLUTGUI.cpp
const char* special_map(int k);


GLUIGUI::GLUIGUI(GenericBackendBase* backend,int w,int h)
  :GenericGUIBase(backend),window_title("OpenGL Viewer"),displayMode(0)
{}

void GLUIGUI::Handle_Display()
{
  SendGLRender();
  glutSwapBuffers();
}

void GLUIGUI::Handle_Reshape(int w,int h)
{
  width=w; height=h;
  int x,y;
  GLUI_Master.get_viewport_area(&x,&y,&w,&h);
  glViewport(x,y,w,h);
  SendGLViewport(x,y,w,h);
}
void GLUIGUI::Handle_Keypress(unsigned char key,int x,int y)
{
  SendKeyDown(string(1,key));
}
void GLUIGUI::Handle_KeypressUp(unsigned char key,int x,int y)
{
  SendKeyUp(string(1,key));
}
void GLUIGUI::Handle_Special(int key,int x,int y) 
{
  SendKeyDown(special_map(key)); 
}
void GLUIGUI::Handle_SpecialUp(int key,int x,int y) 
{
  SendKeyUp(special_map(key)); 
}
void GLUIGUI::Handle_Click(int button,int state,int x,int y)
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

void GLUIGUI::Handle_Drag(int x,int y)
{
  SendMouseMove(x,y);
}

void GLUIGUI::Handle_Motion(int x,int y)
{
  SendMouseMove(x,y);
}

void GLUIGUI::Handle_Idle()
{
  SendIdle();
}

void GLUIGUI::Run()
{
  GLUIProgramBase::Run(window_title.c_str(),displayMode);
}

int GLUIGUI::AddControl(GLUI_Control* control,const char* name)
{
  long id = (long)controls.size();
  controls.push_back(control);
  if(name == NULL)
    controlNames.push_back(control->name);
  else
    controlNames.push_back(name);
  control->callback = GLUI_CB(ControlFunc);
  control->user_id = id;
  if(dynamic_cast<GLUI_Spinner*>(control) != NULL) {
    GLUI_EditText* partner = dynamic_cast<GLUI_Spinner*>(control)->edittext;
    partner->callback = GLUI_CB(ControlFunc);
    partner->user_id = id;
  }
  return id;
}

bool GLUIGUI::Initialize()
{
  backend->Start();
  glEnable (GL_DEPTH_TEST);
  glEnable (GL_CULL_FACE);
  return true;
}

void GLUIGUI::Handle_Control(int id)
{
  if(id < 0 || id >= (int)controls.size())
    LOG4CXX_ERROR(KrisLibrary::logger(),"GLUIGUI: invalid control"<<"\n");
  if(controlNames[id]=="") return;
  if(dynamic_cast<GLUI_Button*>(controls[id]) != NULL) 
    SendButtonPress(controlNames[id]);
  else if(dynamic_cast<GLUI_Checkbox*>(controls[id]) != NULL) {
    SendButtonToggle(controlNames[id],controls[id]->get_int_val());
  }
  else if(dynamic_cast<GLUI_EditText*>(controls[id]) != NULL ||
	  dynamic_cast<GLUI_TextBox*>(controls[id]) != NULL) {
    SendWidgetValue(controlNames[id],controls[id]->text);
  }
  else if(dynamic_cast<GLUI_RadioGroup*>(controls[id]) != NULL ||
	  dynamic_cast<GLUI_Scrollbar*>(controls[id]) != NULL ||
	  dynamic_cast<GLUI_List*>(controls[id]) != NULL ||
	  dynamic_cast<GLUI_Listbox*>(controls[id]) != NULL) {
    stringstream ss;
    ss<<controls[id]->get_int_val();
    SendWidgetValue(controlNames[id],ss.str());
  }
  else if(dynamic_cast<GLUI_Spinner*>(controls[id]) != NULL) {
    stringstream ss;
    ss<<controls[id]->get_float_val();
    SendWidgetValue(controlNames[id],ss.str());
  }
  else {
    LOG4CXX_ERROR(KrisLibrary::logger(),"GLUIGUI: Unknown control type, name "<<controlNames[id]<<"\n");
    SendWidgetValue(controlNames[id],controls[id]->text);
  }
}

bool GLUIGUI::OnRefresh()
{
  glutPostRedisplay();
  return true;
}

bool GLUIGUI::OnResize(int w,int h)
{
  glutReshapeWindow(w,h);
  return true;
}

bool GLUIGUI::OnQuit()
{
  exit(0);
  return true;
}

bool GLUIGUI::OnNotify(const string& text,const string& msglevel)
{
  LOG4CXX_INFO(KrisLibrary::logger(),"GLUIGUI::OnNotify:"<<text<<"\n");
  return true;
}

bool GLUIGUI::OnPauseIdle(double secs)
{
  if(secs > INT_MAX/1000)
    SleepIdleCallback(INT_MAX);
  else
    SleepIdleCallback(int(secs*1000));
  return true;
}


