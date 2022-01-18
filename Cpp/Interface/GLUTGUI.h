#ifndef INTERFACE_GLUT_GUI_H
#define INTERFACE_GLUT_GUI_H

#include <KrisLibrary/GLdraw/GLUTProgram.h>
#include "GenericGUI.h"

namespace Klampt {

class GLUTGUI : public GenericGUIBase, public GLUTProgramBase
{
 public:
  GLUTGUI(GenericBackendBase* backend=NULL,int w=800,int h=600);
  virtual ~GLUTGUI() {}
  //if you want to set up the window title, it must be called before Run()
  void SetWindowTitle(const char* title) { window_title = title; }
  //if you want to set up the display mode, it must be called before Run()
  void SetDisplayMode(int mode) { displayMode = mode; }

  ///Subclasses: put GL initialization code here
  virtual bool Initialize();
  virtual void Handle_Display();
  virtual void Handle_Reshape(int w,int h);
  virtual void Handle_Keypress(unsigned char key,int x,int y);
  virtual void Handle_KeypressUp(unsigned char key,int x,int y);
  virtual void Handle_Special(int key,int x,int y);
  virtual void Handle_SpecialUp(int key,int x,int y);
  virtual void Handle_Click(int button,int state,int x,int y);
  virtual void Handle_Drag(int x,int y);
  virtual void Handle_Motion(int x,int y);
  virtual void Handle_Idle();

  virtual void Run();
  virtual bool OnQuit();
  virtual bool OnNotify(const string& text,const string& msglevel);
  virtual bool OnPauseIdle(double secs);
  virtual bool OnRefresh();
  virtual bool OnDrawText(double x, double y, double z, const std::string &text, int height);
  virtual bool OnDrawText(int x, int y, const std::string &text, int height);

  string window_title;
  unsigned int displayMode;
};

} //namespace Klampt

#endif
