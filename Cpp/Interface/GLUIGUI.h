#ifndef INTERFACE_GLUI_GUI_H
#define INTERFACE_GLUI_GUI_H

#include "GenericGUI.h"
#include <KrisLibrary/GLdraw/GLUIProgram.h>

class GLUI_Control;

namespace Klampt {

/** @brief A base class for GLUI GUIs.
 *
 * GLUI controls should not be setup with hooks and live variables -- instead,
 * pass them to AddControl, and this will automatically send status changes to
 * the backend.
 *
 * To add more complex functionality, you can override the Handle_Control
 * method with the ID returned by AddControl.
 */
class GLUIGUI : public GenericGUIBase, public GLUIProgramBase
{
 public:
  GLUIGUI(GenericBackendBase* backend=NULL,int w=800,int h=600);
  virtual ~GLUIGUI() {}
  //if you want to set up the window title, it must be called before Run()
  void SetWindowTitle(const char* title) { window_title = title; }
  //if you want to set up the display mode, it must be called before Run()
  void SetDisplayMode(int mode) { displayMode = mode; }

  ///Subclasses: put GL initialization code here
  virtual bool Initialize();
  ///Subclasses: may override this to translate IDs to messages sent to
  ///the backend.  Default sends the value of the control to the controlNames
  ///as set up via AddControl.
  virtual void Handle_Control(int id);
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

  ///Subclasses should call this when adding a new GUI element
  ///(No need to create and ID and pass ControlFunc to the element constructor)
  ///Returns the GLUI ID of the control
  ///
  ///If name = NULL, then the button name is passed directly to the backend.
  ///If name = "", then the signal is not sent to the backend (used to bind
  ///controls to Handle_Control).
  int AddControl(GLUI_Control*,const char* name=NULL);

  virtual void Run();
  virtual bool OnQuit();
  virtual bool OnNotify(const string& text,const string& msglevel);
  virtual bool OnPauseIdle(double secs);
  virtual bool OnRefresh();
  virtual bool OnResize(int w,int h);
  virtual bool OnDrawText(double x, double y, double z, const std::string &text, int height);
  virtual bool OnDrawText(int x, int y, const std::string &text, int height);

  string window_title;
  unsigned int displayMode;
  vector<GLUI_Control*> controls;
  vector<string> controlNames;
};

} // namespace Klampt

#endif
