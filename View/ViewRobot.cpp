#include "ViewRobot.h"
#include <GLdraw/drawextra.h>

const static GLColor grey(0.5,0.5,0.5);
const static GLColor red(1,0,0);
const static GLColor orange(1,0.5,0);
const static GLColor yellow(1,1,0);
const static GLColor purple(0.5,0,1);
const static GLColor lightpurple(0.75,0.5,1);
const static GLColor blue(0,0,1);


struct GLCheckeredSphere
{
  GLCheckeredSphere();
  void Draw();

  Real radius;
  Vector3 center;
  GLColor c1,c2;
  int numSlices,numStacks;
};

GLCheckeredSphere::GLCheckeredSphere()
  :radius(1),center(Zero),numSlices(16),numStacks(8)
{
  c1.set(1,1,1);
  c2.set(0,0,0);
}

//theta is the vertical range, phi is the rotational range
void DrawSphereArc(Real r,Real theta0,Real theta1,Real phi0,Real phi1,int numSlices,int numStacks)
{
  Real thetaInc = (theta1-theta0)/Real(numStacks);
  Real phiInc = (phi1-phi0)/Real(numSlices);
  Real phi=phi0;
  Real theta;
  for(int i=0;i<numSlices;i++,phi+=phiInc) {
    Real x1=Cos(phi);
    Real x2=Cos(phi+phiInc);
    Real y1=Sin(phi);
    Real y2=Sin(phi+phiInc);
    theta=theta0;
    glBegin(GL_TRIANGLE_STRIP);
    for(int j=0;j<=numStacks;j++,theta+=thetaInc) {
      Real cz=Cos(theta);
      Real sz=Sin(theta);
      glNormal3f(x2*sz,y2*sz,cz);
      glVertex3f(r*x2*sz,r*y2*sz,r*cz);
      glNormal3f(x1*sz,y1*sz,cz);
      glVertex3f(r*x1*sz,r*y1*sz,r*cz);
    }
    glEnd();
  }
}

void GLCheckeredSphere::Draw()
{
  glEnable(GL_LIGHTING);
  glPushMatrix();
  {
    glTranslate(center);
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,c1.rgba); 
    DrawSphereArc(radius, 0,Pi_2,  0,Pi_2,    numSlices/4,numStacks/2);
    DrawSphereArc(radius, 0,Pi_2,  Pi,3*Pi_2, numSlices/4,numStacks/2);
    DrawSphereArc(radius, Pi_2,Pi, Pi_2,Pi,   numSlices/4,numStacks/2);
    DrawSphereArc(radius, Pi_2,Pi, 3*Pi_2,TwoPi,numSlices/4,numStacks/2);
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,c2.rgba); 
    DrawSphereArc(radius, 0,Pi_2,  Pi_2,Pi,   numSlices/4,numStacks/2);
    DrawSphereArc(radius, 0,Pi_2,  3*Pi_2,TwoPi,numSlices/4,numStacks/2);
    DrawSphereArc(radius, Pi_2,Pi, 0,Pi_2,    numSlices/4,numStacks/2);
    DrawSphereArc(radius, Pi_2,Pi, Pi,3*Pi_2, numSlices/4,numStacks/2);
  }
  glPopMatrix();
}




void InitDisplayLists(Robot& robot,vector<GLDisplayList>& displayLists)
{
  displayLists.resize(robot.links.size());
  for(size_t i=0;i<robot.links.size();i++) {
    if(!displayLists[i].isCompiled()) {
      displayLists[i].beginCompile();
      glBegin(GL_POINTS);
      glVertex3f(0,0,0);
      glEnd();
      robot.DrawLinkGL(i);
      displayLists[i].endCompile();
    }
  }
}

ViewRobot::ViewRobot(Robot* _robot)
  :robot(_robot)
{
  if(robot) {
    linkAppearance.resize(robot->links.size());
    for(size_t i=0;i<linkAppearance.size();i++) {
      linkAppearance[i].Set(robot->geometry[i]);
      linkAppearance[i].faceColor = grey;
    }
  }
}

ViewRobot::~ViewRobot()
{
}

void ViewRobot::Draw(Robot* _robot) 
{
  robot = _robot;
  if(!robot) return;
  SetGrey();
  Draw();
}

void ViewRobot::SetColors(const GLColor& c)
{
  if(robot) {
    if(linkAppearance.empty()) {
      linkAppearance.resize(robot->links.size());
      for(size_t i=0;i<linkAppearance.size();i++) 
	linkAppearance[i].Set(robot->geometry[i]);
    }
    for(size_t i=0;i<linkAppearance.size();i++) {
      linkAppearance[i].faceColor = c;
      linkAppearance[i].vertexColor = c;
    }
  }
}

void ViewRobot::SetColor(int link,const GLColor& c)
{
  if(linkAppearance.empty()) {
    linkAppearance.resize(robot->links.size());
    for(size_t i=0;i<linkAppearance.size();i++) {
      linkAppearance[i].Set(robot->geometry[i]);
      linkAppearance[i].faceColor = grey;
      linkAppearance[i].vertexColor = grey;
    }
  }
  linkAppearance[link].faceColor = c;
  linkAppearance[link].vertexColor = c;
}

void ViewRobot::SetGrey() { SetColors(grey); }

void ViewRobot::Draw() 
{
  if(!robot) return;

  for(size_t i=0;i<robot->links.size();i++) {
    Matrix4 mat = robot->links[i].T_World;
    glPushMatrix();
    glMultMatrix(mat);
    linkAppearance[i].DrawGL();
    glPopMatrix();
  }
}

void ViewRobot::DrawLink_Local(int i)
{
  if(!robot) return;
  linkAppearance[i].DrawGL();
}

void ViewRobot::DrawLink_World(int i)
{
  if(!robot) return;
  Matrix4 mat = robot->links[i].T_World;
  glPushMatrix();
  glMultMatrix(mat);
  DrawLink_Local(i);
  glPopMatrix();
}


void ViewRobot::DrawCenterOfMass(Real radius)
{
  if(!robot) return;
  Vector3 com = robot->GetCOM();

  GLCheckeredSphere sph;
  sph.center = com;
  sph.radius = radius;
  sph.c1.set(1,0,0);
  sph.c2.set(0,0,1);
  sph.Draw();

  glDisable(GL_LIGHTING);  
  glColor3f(0,0,1);
  glBegin(GL_LINES);
  glVertex3f(com.x,com.y,com.z);
  glVertex3f(com.x,com.y,0);
  glEnd();
}

void ViewRobot::DrawLinkCenterOfMass(int i,Real radius)
{
  Vector3 com = robot->links[i].T_World*robot->links[i].com;
  GLCheckeredSphere sph;
  sph.center = com;
  sph.radius = radius;
  sph.c1.set(1,0,0);
  sph.c2.set(0,0,1);
  sph.Draw();
}

void ViewRobot::DrawLinkFrames()
{
  if(!robot) return;
  glDisable(GL_LIGHTING);
  for(int i=0;i<robot->links.size();i++) {
    glPushMatrix();
    glMultMatrix((Matrix4)robot->links[i].T_World);
    drawCoords(0.1);
    glPopMatrix();
  }
}

/*void ViewRobot::DrawTorques(const Vector& T)
{
  SetTorqueColors(T);
  Draw();
}
*/

inline void GetTorqueColor(Real t,GLColor& color)
{
  if(t < 0.5) color.set(0.5+t,0.5+t,0.5-t);
  else if(t < 0.75) color.set(1,2-2*t,0);
  else if(t < 1) color.set(1,2-2*t,0);
  else color.set(0.5,0,0);
}

inline bool EqualColor(const GLColor& a,const GLColor& b)
{
  for(int i=0;i<4;i++)
    if(a.rgba[i] != b.rgba[i]) return false;
  return true;
}

/*

void ViewRobot::SetTorqueColors(const Vector& T)
{
  if(T.isEmpty()) {
    SetColors(GLColor(1,0,1));
    return;
  }
  //draw torques
  Assert(T.n == (int)robot->links.size());
  for(int i=0;i<T.n;i++) {
    GetTorqueColor(T[i],linkAppearance[i].faceColor);
  }
}


*/
