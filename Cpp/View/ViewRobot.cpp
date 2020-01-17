#include "ViewRobot.h"
#include <KrisLibrary/GLdraw/drawextra.h>
using namespace GLDraw;

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


ViewRobot::ViewRobot(Robot* _robot)
  :robot(_robot)
{
}

ViewRobot::~ViewRobot()
{
}

void ViewRobot::Draw(Robot* _robot) 
{
  robot = _robot;
  if(!robot) return;
  Draw();
}

void ViewRobot::SetColors(const GLColor& c)
{
  if(robot) {
    for(size_t i=0;i<robot->links.size();i++) {
      GLDraw::GeometryAppearance& a = Appearance(i);
      a.SetColor(c);
    }
  }
}

void ViewRobot::SetColor(int link,const GLColor& c)
{
  if(robot) {
    GLDraw::GeometryAppearance& a = Appearance(link);
    a.SetColor(c);
  }
}

void ViewRobot::SetGrey() { SetColors(grey); }

void ViewRobot::Draw() 
{
  if(!robot) return;

  for(size_t i=0;i<robot->links.size();i++) {
    if(robot->IsGeometryEmpty(i)) continue;
    Matrix4 mat = robot->links[i].T_World;
    glPushMatrix();
    glMultMatrix(mat);
    GLDraw::GeometryAppearance& a = Appearance(i);
    if(a.geom != robot->geometry[i].get()) {
      a.Set(*robot->geometry[i]);
    }
    a.DrawGL();
    glPopMatrix();
  }
}

void ViewRobot::DrawLink_Local(int i,bool keepAppearance)
{
  if(!robot || robot->IsGeometryEmpty(i)) return;
  if(keepAppearance) {
    GLDraw::GeometryAppearance& a = Appearance(i);
    if(a.geom != robot->geometry[i].get())
      a.Set(*robot->geometry[i]);
    a.DrawGL();
  }
  else 
    draw(*robot->geometry[i]);
}

void ViewRobot::DrawLink_World(int i,bool keepAppearance)
{
  if(!robot) return;
  Matrix4 mat = robot->links[i].T_World;
  glPushMatrix();
  glMultMatrix(mat);
  DrawLink_Local(i,keepAppearance);
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

void ViewRobot::DrawLinkFrames(Real size)
{
  if(!robot) return;
  glDisable(GL_LIGHTING);
  for(size_t i=0;i<robot->links.size();i++) {
    glPushMatrix();
    glMultMatrix((Matrix4)robot->links[i].T_World);
    drawCoords(size);
    glPopMatrix();
  }
}

void ViewRobot::DrawLinkSkeleton()
{
  if(!robot) return;
  glDisable(GL_LIGHTING);
  glColor3f(1,0.5,0);
  glLineWidth(3.0);
  glBegin(GL_LINES);
  for(size_t i=0;i<robot->links.size();i++) {
    if(robot->parents[i] >= 0) {
      glVertex3v(robot->links[robot->parents[i]].T_World.t);
      glVertex3v(robot->links[i].T_World.t);
    }
  }
  glEnd();
  glLineWidth(1.0);
}

void ViewRobot::DrawTorques(const Vector& T)
{
  SetTorqueColors(T);
  Draw();
}

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

void ViewRobot::SetTorqueColors(const Vector& T)
{
  if(T.empty()) {
    SetColors(GLColor(1,0,1));
    return;
  }
  //draw torques
  Assert(T.n == (int)robot->links.size() || T.n == (int)robot->drivers.size());
  if(T.n == (int)robot->links.size()) {
    for(int i=0;i<T.n;i++) {
      GetTorqueColor(T[i],Appearance(i).faceColor);
    }
  }
  else {
    for(int i=0;i<T.n;i++) {
      GLColor c;
      GetTorqueColor(T[i],c);
      for(size_t k=0;k<robot->drivers[i].linkIndices.size();k++)
	Appearance(robot->drivers[i].linkIndices[k]).faceColor = c;
    }
  }
}

GLDraw::GeometryAppearance& ViewRobot::Appearance(int link)
{
  Assert(robot!=NULL);
  if(appearanceStack.empty()) {
    if(robot->geomManagers[link].IsAppearanceShared()) 
      robot->geomManagers[link].SetUniqueAppearance();
    return *robot->geomManagers[link].Appearance();
  }
  return appearanceStack.back()[link];
}

void ViewRobot::PushAppearance()
{
  if(robot==NULL) return;
  vector<GLDraw::GeometryAppearance> app(robot->links.size());
  for(size_t i=0;i<robot->links.size();i++) {
    app[i] = Appearance(i);
    if(Appearance(i).faceDisplayList)
      Assert(app[i].faceDisplayList);
  }
  appearanceStack.push_back(app);
}

void ViewRobot::PopAppearance()
{
  if(!appearanceStack.empty()) {
    //may want to copy back face display lists up the stack
    if(appearanceStack.size()==1) {
      for(size_t i=0;i<robot->links.size();i++) {
        if(!robot->geomManagers[i].Appearance()->faceDisplayList)
          robot->geomManagers[i].Appearance()->faceDisplayList = appearanceStack.back()[i].faceDisplayList;
        if(!robot->geomManagers[i].Appearance()->edgeDisplayList)
          robot->geomManagers[i].Appearance()->edgeDisplayList = appearanceStack.back()[i].edgeDisplayList;
        if(!robot->geomManagers[i].Appearance()->vertexDisplayList)
          robot->geomManagers[i].Appearance()->vertexDisplayList = appearanceStack.back()[i].vertexDisplayList;
        if(!robot->geomManagers[i].Appearance()->silhouetteDisplayList)
          robot->geomManagers[i].Appearance()->silhouetteDisplayList = appearanceStack.back()[i].silhouetteDisplayList;
        if(!robot->geomManagers[i].Appearance()->tempMesh)
          robot->geomManagers[i].Appearance()->tempMesh = appearanceStack.back()[i].tempMesh;
        if(!robot->geomManagers[i].Appearance()->tempMesh2)
          robot->geomManagers[i].Appearance()->tempMesh2 = appearanceStack.back()[i].tempMesh2;
      }
    }
    else {
      size_t n=appearanceStack.size()-2;
      for(size_t i=0;i<robot->links.size();i++) {
        if(!appearanceStack[n][i].faceDisplayList)
          appearanceStack[n][i].faceDisplayList = appearanceStack.back()[i].faceDisplayList;
        if(!appearanceStack[n][i].edgeDisplayList)
          appearanceStack[n][i].edgeDisplayList = appearanceStack.back()[i].edgeDisplayList;
        if(!appearanceStack[n][i].vertexDisplayList)
          appearanceStack[n][i].vertexDisplayList = appearanceStack.back()[i].vertexDisplayList;
        if(!appearanceStack[n][i].silhouetteDisplayList)
          appearanceStack[n][i].silhouetteDisplayList = appearanceStack.back()[i].silhouetteDisplayList;
        if(!appearanceStack[n][i].tempMesh)
          appearanceStack[n][i].tempMesh = appearanceStack.back()[i].tempMesh;
        if(!appearanceStack[n][i].tempMesh2)
          appearanceStack[n][i].tempMesh2 = appearanceStack.back()[i].tempMesh2;
      }
    }
    appearanceStack.resize(appearanceStack.size()-1);
  }
}
void ViewRobot::RestoreAppearance()

{
  appearanceStack.resize(0);
}

vector<GLDraw::GeometryAppearance> ViewRobot::GetAppearance()
{
  vector<GLDraw::GeometryAppearance> res;
  if(!robot) return res;
  res.resize(robot->links.size());
  for(size_t i=0;i<res.size();i++)
    res[i] = Appearance(i);
  return res;
}

void ViewRobot::SetAppearance(const vector<GLDraw::GeometryAppearance>& app)
{
  if(!robot) return;
  Assert(app.size()==robot->links.size());
  for(size_t i=0;i<app.size();i++) {
    Appearance(i) = app[i];
  }
}
