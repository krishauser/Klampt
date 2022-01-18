#if HAVE_GLUI

#include <KrisLibrary/Logger.h>
#include <KrisLibrary/GLdraw/GLUTString.h>
#include "GLUIGUI.h"
#include <limits.h>
#include <sstream>
#include <assert.h>
#include <stdio.h>
#include <iostream>

using namespace Klampt;

#if defined (__APPLE__) || defined (MACOSX)
#include <GL/glui.h>
#else
#include <GL/glui.h>
#endif //__APPLE__ || MACOSX

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
    cerr<<"GLUIGUI: invalid control"<<endl;
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
    cerr<<"GLUIGUI: Unknown control type, name "<<controlNames[id]<<endl;
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
  cout<<"GLUIGUI::OnNotify:"<<text<<endl;
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

bool GLUIGUI::OnDrawText(double x, double y, double z, const std::string &text, int height)
{
  void* fontface;
  if (height <= 10)
    fontface = GLUT_BITMAP_HELVETICA_10;
  else if (height <= 12)
    fontface = GLUT_BITMAP_HELVETICA_12;
  else
    fontface = GLUT_BITMAP_HELVETICA_18;

  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);

  glRasterPos3d(x, y, z);
  glutBitmapString(fontface, text.c_str());

  glEnable(GL_DEPTH_TEST);
}
bool GLUIGUI::OnDrawText(int x, int y, const std::string &text, int height)
{
  void* fontface;
  if (height <= 10)
    fontface = GLUT_BITMAP_HELVETICA_10;
  else if (height <= 12)
    fontface = GLUT_BITMAP_HELVETICA_12;
  else
    fontface = GLUT_BITMAP_HELVETICA_18;

  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);

  glRasterPos2i(x, y);
  glutBitmapString(fontface, text.c_str());

  glEnable(GL_DEPTH_TEST);
}


#endif //HAVE_GLUI
