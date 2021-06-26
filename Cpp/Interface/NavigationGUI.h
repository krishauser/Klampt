#ifndef INTERFACE_NAVIGATION_GUI_H
#define INTERFACE_NAVIGATION_GUI_H

#include "GenericGUI.h"
#include <KrisLibrary/math3d/geometry3d.h>
#include <KrisLibrary/camera/viewport.h>
#include <KrisLibrary/Timer.h>

namespace Klampt {

/** @brief A backend that processes mouse motion calls into dragging
 * callbacks.  Makes it a bit easier to determine free-dragging,
 * control-dragging, shift-dragging, etc.
 */
class MouseDragBackend : public GenericBackendBase
{
 public:
  MouseDragBackend();
  virtual ~MouseDragBackend() {}
  virtual bool OnMouseClick(int button,int state,int mx,int my);
  virtual bool OnMouseMove(int mx,int my);
  virtual bool OnKeyDown(const string& key);
  virtual bool OnKeyUp(const string& key);

  virtual void BeginDrag(int x,int y,int button,int modifiers) {}
  virtual void DoDrag(int dx,int dy,int button,int modifiers);
  virtual void EndDrag(int x,int y,int button,int modifiers) {}
  ///Overload this for regular (non-modified) dragging
  virtual void DoFreeDrag(int dx,int dy,int button) {}
  ///Overload this for control-dragging
  virtual void DoCtrlDrag(int dx,int dy,int button) {}
  ///Overload this for alt-dragging
  virtual void DoAltDrag(int dx,int dy,int button) {}
  ///Overload this for shift-dragging
  virtual void DoShiftDrag(int dx,int dy,int button) {}
  ///Overload this to handle plain, non-dragging mouse motion events
  virtual void DoPassiveMouseMove(int x,int y) {}

  int oldmousex,oldmousey;
  int clickButton, clickModifiers;
};

/** @brief A backend that manages a camera and performs OpenGL scene
 * management.
 * 
 * The World is rendered from the perspective of the mouse-controlled camera.
 *
 * The Screen is rendered with pixel coordinates (0,0) in the upper left,
 * with x axis pointing right and y axis pointing down.
 */
class GLNavigationBackend : public MouseDragBackend
{
 public:
  typedef MouseDragBackend BaseT;

  GLNavigationBackend();
  virtual ~GLNavigationBackend() {}
  //overrideable
  virtual void SetWorldLights() {}
  virtual void RenderWorld() {}
  virtual void RenderScreen() {}

  virtual void Start();
  virtual bool OnIdle();
  virtual bool OnGLRender();
  virtual bool OnGLViewport(int x,int y,int w,int h);
  virtual bool OnMouseWheel(int dwheel);
  virtual bool OnCommand(const string& cmd,const string& args);

  virtual void BeginDrag(int x,int y,int button,int modifiers);
  virtual void DoFreeDrag(int dx,int dy,int button);
  virtual void DoCtrlDrag(int dx,int dy,int button);
  virtual void DoAltDrag(int dx,int dy,int button);
  virtual void DoShiftDrag(int dx,int dy,int button);

  void DragPan(int dx,int dy);
  void DragRotate(int dx,int dy);
  void DragZoom(int dx,int dy);
  void DragTruck(int dx,int dy);

  void Set2DMode(bool mode=true);
  void DisplayCameraTarget();
  void CenterCameraOn(const Math3D::AABB3D& bbox);
  void ClickRay(int x,int y,Math3D::Vector3& src,Math3D::Vector3& dir) const;

  void WriteDisplaySettings(std::ostream& out) const;
  void ReadDisplaySettings(std::istream& in);
  
  Camera::Viewport viewport;
  Camera::CameraController_Orbit camera;
  bool stereo_mode;
  float stereo_offset;
  Timer timer;
  int show_view_target; float t_hide_view_target;
  float frames_per_second; bool show_frames_per_second;
  int frames_rendered;
  bool mode_2d;
};

} //namespace Klampt

#endif //INTERFACE_NAVIGATION_GUI_H
