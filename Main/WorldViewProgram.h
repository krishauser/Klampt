#ifndef WORLD_VIEW_PROGRAM_H
#define WORLD_VIEW_PROGRAM_H

#include <math3d/Ray3D.h>
#include "Modeling/World.h"
#include <GLdraw/GLUINavigationProgram.h>
#include <GLdraw/GLScreenshotProgram.h>
#include <GLdraw/GL.h>
#include <GLdraw/drawextra.h>
#include <GLdraw/Widget.h>
#include <glui.h>
using namespace Math3D;
using namespace GLDraw;

class WorldViewProgram : public GLScreenshotProgram<GLUINavigationProgram>
{
public:
  WorldViewProgram(RobotWorld* world);
  virtual ~WorldViewProgram() {}

  virtual bool Initialize();
  virtual void SetWorldLights()
  {
    world->SetGLLights();
  }
  void ClickRay(int x,int y,Ray3D& r) const;
  RobotInfo* ClickRobot(const Ray3D& r,int& body,Vector3& localpt) const;
  RigidObjectInfo* ClickObject(const Ray3D& r,Vector3& localpt) const;
  virtual void RefreshIdle() { SleepIdleCallback(0); }
  virtual void RenderWorld()
  {
    glDisable(GL_LIGHTING);
    drawCoords(0.1);
    glEnable(GL_LIGHTING);
    world->DrawGL();
  }

  virtual void DoFreeDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON)  DragRotate(dx,dy);
  }

  virtual void DoCtrlDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON)  DragPan(dx,dy);
  }
  
  virtual void DoAltDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON)  DragZoom(dx,dy);
  }

  virtual void DoShiftDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON) { camera.dist *= (1 + 0.01*Real(dy)); }
  }

  RobotWorld* world;
};


class WorldViewWidget : public Widget
{
 public:
  RobotWorld* world;
  //click information
  RobotInfo* clickedRobot;
  RigidObjectInfo* clickedObject;
  int body;
  Vector3 localpt;

  WorldViewWidget(RobotWorld* _world)
    :world(_world),clickedRobot(NULL),clickedObject(NULL)
  {}
  virtual bool Hover(int x,int y,Camera::Viewport& viewport,double& distance) { 
    Ray3D r;
    viewport.getClickSource(x,y,r.source);
    viewport.getClickVector(x,y,r.direction);
    clickedRobot = world->ClickRobot(r,body,localpt);
    if(clickedRobot) {
      Vector3 worldpt = clickedRobot->robot->links[body].T_World*localpt;
      distance = r.direction.dot(worldpt-r.source);
    }
    Vector3 localpt2;
    clickedObject = world->ClickObject(r,localpt2);
    if(clickedObject) {
      Vector3 worldpt2 = clickedObject->object->T*localpt2;
      Real distance2 = r.direction.dot(worldpt2-r.source);      
      if(clickedRobot && distance < distance2) {
	//robot is closest
	clickedObject = NULL;
      }
      else {
	clickedRobot = NULL;
	localpt = localpt2;
	distance = distance2;
      }
    }
    return clickedRobot || clickedObject; 
  }
  //these currently don't do anything, but can be overloaded to do something with the current robot
  virtual bool BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance) { 
    bool res=Hover(x,y,viewport,distance);
    return false;
  }
  virtual void Drag(int dx,int dy,Camera::Viewport& viewport) {}
  virtual void EndDrag() {}

  virtual void DrawGL(Camera::Viewport& viewport) {
    glEnable(GL_LIGHTING);
    world->DrawGL();
  }
};


WorldViewProgram::WorldViewProgram(RobotWorld* _world)
  :world(_world)
{
}

bool WorldViewProgram::Initialize()
{
  //TODO: read in the camera from the world?
  camera.dist = 6;
  viewport.n = 0.1;
  viewport.f = 100;
  viewport.setLensAngle(DtoR(30.0));

  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glClearColor(world->background.rgba[0],world->background.rgba[1],world->background.rgba[2],world->background.rgba[3]);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  return GLUINavigationProgram::Initialize();
}


void WorldViewProgram::ClickRay(int x,int y,Ray3D& r) const
{
  viewport.getClickSource(x,viewport.y+viewport.h-y,r.source);
  viewport.getClickVector(x,viewport.y+viewport.h-y,r.direction);
  //cout<<"Ray "<<r.source<<" -> "<<r.direction<<endl;
}


RobotInfo* WorldViewProgram::ClickRobot(const Ray3D& r,int& body,Vector3& localpt) const
{
  return world->ClickRobot(r,body,localpt);
}

RigidObjectInfo* WorldViewProgram::ClickObject(const Ray3D& r,Vector3& localpt) const
{
  return world->ClickObject(r,localpt);
}

#endif
