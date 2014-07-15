#ifndef ROBOT_VIEW_PROGRAM_H
#define ROBOT_VIEW_PROGRAM_H

#include "Modeling/Robot.h"
#include "Modeling/Environment.h"
#include "View/ViewEnvironment.h"
#include "View/ViewRobot.h"
#include <math3d/Ray3D.h>
#include <GLdraw/GLUINavigationProgram.h>
#include <GLdraw/drawextra.h>
#include <GLdraw/GL.h>
#include <GLdraw/GLLight.h>
#include <GL/glui.h>

class RobotViewProgram : public GLUINavigationProgram
{
public:
  RobotViewProgram(Robot* robot,Environment* env);
  virtual ~RobotViewProgram() {}

  virtual bool Initialize();
  virtual void SetWorldLights();
  void ClickRay(int x,int y,Ray3D& r) const;
  bool ClickRobot(const Ray3D& r,int& body,Vector3& localpt) const;
  virtual void RefreshIdle() { SleepIdleCallback(0); }
  virtual void RenderWorld()
  {
    glEnable(GL_LIGHTING);
    viewEnv.Draw(env);
    glDisable(GL_LIGHTING);
    drawCoords(0.1);
    viewRobot.DrawLinkFrames();
    viewRobot.Draw();
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

  Robot* robot;
  Environment* env;
  ViewEnvironment viewEnv;
  ViewRobot viewRobot;
};

RobotViewProgram::RobotViewProgram(Robot* _robot,Environment* _env)
  :robot(_robot),env(_env),viewRobot(_robot)
{
}


bool RobotViewProgram::Initialize()
{
  camera.dist = 10;
  viewport.n = 0.1;
  viewport.f = 100;
  viewport.setLensAngle(DtoR(30.0));

  
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glClearColor(0.4,0.4,1,0);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  return GLUINavigationProgram::Initialize();
}


void RobotViewProgram::SetWorldLights()
{
  GLLight light;
  light.setColor(GLColor(1,1,1));
  //light.setDirectionalLight(Vector3(0,0.4,1));
  light.setDirectionalLight(Vector3(0.2,-0.4,1));
  light.setCurrentGL(0);
  
  /*
  light.setColor(GLColor(0.3,0.3,0.3));
  light.setPointLight(Vector3(0,0,3));
  light.setCurrentGL(1);
  light.att1 = 0.01;
  light.att0 = 1;
  */
  /*
  light.setColor(GLColor(0.3,0.3,0.3));
  light.setDirectionalLight(Vector3(1,-2,-0.4));
  light.setCurrentGL(2);
  */
}

void RobotViewProgram::ClickRay(int x,int y,Ray3D& r) const
{
  viewport.getClickSource(x,viewport.y+viewport.h-y,r.source);
  viewport.getClickVector(x,viewport.y+viewport.h-y,r.direction);
  //cout<<"Ray "<<r.source<<" -> "<<r.direction<<endl;
}

bool RobotViewProgram::ClickRobot(const Ray3D& r,int& body,Vector3& localpt) const
{
  Real closestDist = Inf;
  Vector3 closestPoint;
  int closestBody = -1;
  Vector3 worldpt;
  robot->UpdateGeometry();
  for(size_t i=0;i<robot->links.size();i++) {
    if(robot->geometry[i].Empty()) continue;
    Real dist;
    if(robot->geometry[i].RayCast(r,&dist)) {
      if(dist < closestDist) {
	closestDist = dist;
	closestPoint = r.source+dist*r.direction;
	closestBody = i;
      }
    }
  }
  if(closestBody >= 0)
    robot->links[closestBody].T_World.mulInverse(closestPoint,localpt);
  body = closestBody;
  return (closestBody != -1);
}


#endif
