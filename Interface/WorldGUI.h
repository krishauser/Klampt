#ifndef INTERFACE_WORLD_VIEW_GUI_H
#define INTERFACE_WORLD_VIEW_GUI_H

#include "Modeling/World.h"
#include "NavigationGUI.h"
#include <math3d/Ray3D.h>
using namespace Math3D;

class WorldGUIBackend : public GLNavigationBackend
{
public:
  WorldGUIBackend(RobotWorld* world);
  virtual ~WorldGUIBackend() {}

  bool LoadCommandLine(int argc,const char** argv);
  bool LoadFile(const char* fn);
  //backend overloads
  virtual void Start();
  virtual bool OnCommand(const string& cmd,const string& args);
  //GLNavigationBackend overloads
  virtual void SetWorldLights() { world->SetGLLights(); }
  RobotInfo* ClickRobot(int x,int y,int& body,Vector3& localpt) const;
  RobotInfo* ClickRobot(const Ray3D& r,int& body,Vector3& localpt) const;
  RigidObjectInfo* ClickObject(int x,int y,Vector3& localpt) const;
  RigidObjectInfo* ClickObject(const Ray3D& r,Vector3& localpt) const;
  virtual void RefreshIdle() { SendPauseIdle(0); }
  virtual void RenderWorld();

  virtual void DoFreeDrag(int dx,int dy,int button)
  {
    if(button == 0)  DragRotate(dx,dy);
  }

  virtual void DoCtrlDrag(int dx,int dy,int button)
  {
    if(button == 0)  DragPan(dx,dy);
  }
  
  virtual void DoAltDrag(int dx,int dy,int button)
  {
    if(button == 0)  DragZoom(dx,dy);
  }

  virtual void DoShiftDrag(int dx,int dy,int button)
  {
    if(button == 0) { camera.dist *= (1 + 0.01*Real(dy)); SendRefresh(); }
  }

  virtual bool OnMouseWheel(int dwheel) {
    camera.dist *= (1 + 0.01*Real(dwheel));
    return true;
  }

  RobotWorld* world;
};



#endif
