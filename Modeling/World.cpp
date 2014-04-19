#include "World.h"
#include <utils/stringutils.h>
#include <GLdraw/GL.h>
#include <GLdraw/GLError.h>
#include <string.h>
#include <meshing/IO.h>

RobotWorld::RobotWorld()
{
  background.set(0.4,0.4,1,0);
}

int RobotWorld::NumIDs() const
{
  size_t n=terrains.size()+rigidObjects.size()+robots.size();
  for(size_t i=0;i<robots.size();i++)
    n += robots[i].robot->links.size();
  return n;
}

int RobotWorld::GetID(const string& name,int link) const
{
  for(size_t i=0;i<terrains.size();i++)
    if(terrains[i].name == name)
      return TerrainID(i);
  for(size_t i=0;i<rigidObjects.size();i++)
    if(rigidObjects[i].name == name) 
      return RigidObjectID(i);
  for(size_t i=0;i<robots.size();i++)
    if(robots[i].name == name) {
      if(link < 0) return RobotID(i);
      else return RobotLinkID(i,link);
    }
  return -1;
}

string RobotWorld::GetName(int id) const
{
  int i = IsRigidObject(id);
  if(i >= 0) return rigidObjects[i].name;
  i = IsTerrain(id);
  if(i >= 0) return terrains[i].name;
  i = IsRobot(id);
  if(i >= 0) return robots[i].name;
  pair<int,int> j = IsRobotLink(id);
  if(j.first >= 0) return robots[j.first].name+"["+robots[j.first].robot->linkNames[j.second]+"]";
  return "";
}

int RobotWorld::IsRobot(int id) const
{
  if(id >= (int)(rigidObjects.size()+terrains.size())) {
    id -= (int)(rigidObjects.size()+terrains.size());
    int index=0;
    for(size_t i=0;i<robots.size();i++) {
      if(index == id) return (int)i;
      index += 1 + robots[i].robot->links.size();
    }
    return -1;
  }
  else {
    return -1;
  }
}

pair<int,int> RobotWorld::IsRobotLink(int id) const
{
  if(id >= (int)(rigidObjects.size()+terrains.size())) {
    id -= (int)(rigidObjects.size()+terrains.size());
    int index=0;
    for(size_t i=0;i<robots.size();i++) {
      if(index == id) return pair<int,int>(-1,-1);
      index++;
      if(id < index+(int)robots[i].robot->links.size())
	return pair<int,int>(i,id-index);
      index += robots[i].robot->links.size();
    }
    return pair<int,int>(-1,-1);
  }
  else {
    return pair<int,int>(-1,-1);
  }
}

int RobotWorld::IsTerrain(int id) const
{
  if(id >= 0 && id < (int)terrains.size()) return id;
  return -1;
}

int RobotWorld::IsRigidObject(int id) const
{
  if(id >= (int)terrains.size() && id < (int)(terrains.size()+rigidObjects.size())) return id-(int)terrains.size();
  return -1;
}
int RobotWorld::RobotID(int index) const
{
  int id=(int)(terrains.size()+rigidObjects.size());
  for(int i=0;i<index;i++)
    id += (int)robots[i].robot->links.size()+1;
  return id;
}

int RobotWorld::RobotLinkID(int index,int link) const
{
  int id=(int)(terrains.size()+rigidObjects.size());
  for(int i=0;i<index;i++)
    id += (int)robots[i].robot->links.size()+1;
  return id+1+link;
}

int RobotWorld::TerrainID(int index) const
{
  return index;
}

int RobotWorld::RigidObjectID(int index) const
{
  return index+(int)terrains.size();
}

Geometry::AnyCollisionGeometry3D& RobotWorld::GetGeometry(int id)
{
  int terrain = IsTerrain(id);
  if(terrain >= 0)
    return terrains[terrain].terrain->geometry;
  int rigidObject = IsRigidObject(id);
  if(rigidObject >= 0)
    return rigidObjects[rigidObject].object->geometry;
  pair<int,int> robotLink = IsRobotLink(id);
  if(robotLink.first >= 0) {
    return robots[robotLink.first].robot->geometry[robotLink.second];
  }
  FatalError("GetGeometry: Invalid ID: %d\n",id);
  return robots[0].robot->geometry[0];
}

const Geometry::AnyCollisionGeometry3D& RobotWorld::GetGeometry(int id) const
{
  int terrain = IsTerrain(id);
  if(terrain >= 0)
    return terrains[terrain].terrain->geometry;
  int rigidObject = IsRigidObject(id);
  if(rigidObject >= 0)
    return rigidObjects[rigidObject].object->geometry;
  pair<int,int> robotLink = IsRobotLink(id);
  if(robotLink.first >= 0) {
    return robots[robotLink.first].robot->geometry[robotLink.second];
  }
  FatalError("GetGeometry: Invalid ID: %d\n",id);
  return robots[0].robot->geometry[0];
}

GLDraw::GeometryAppearance& RobotWorld::GetAppearance(int id)
{
  int terrain = IsTerrain(id);
  if(terrain >= 0)
    return terrains[terrain].view.appearance;
  int rigidObject = IsRigidObject(id);
  if(rigidObject >= 0)
    return rigidObjects[rigidObject].view.appearance;
  pair<int,int> robotLink = IsRobotLink(id);
  if(robotLink.first >= 0) {
    return robots[robotLink.first].view.linkAppearance[robotLink.second];
  }
  FatalError("GetApperance: Invalid ID: %d\n",id);
  return robots[0].view.linkAppearance[0];
}

RigidTransform RobotWorld::GetTransform(int id) const
{
  RigidTransform T;
  int terrain = IsTerrain(id);
  if(terrain >= 0) {
    T.setIdentity();
    return T;
  }
  int rigidObject = IsRigidObject(id);
  if(rigidObject >= 0)
    return rigidObjects[rigidObject].object->T;
  pair<int,int> robotLink = IsRobotLink(id);
  if(robotLink.first >= 0) {
    return robots[robotLink.first].robot->links[robotLink.second].T_World;
  }
  FatalError("GetTransform: Invalid ID: %d\n",id);
  return RigidTransform();
}

void RobotWorld::SetTransform(int id,const RigidTransform& T)
{
  int terrain = IsTerrain(id);
  if(terrain >= 0) {
    FatalError("SetTransform: cannot set transform of a terrain");
    return;
  }
  int rigidObject = IsRigidObject(id);
  if(rigidObject >= 0) {
    rigidObjects[rigidObject].object->T = T;
    return;
  }
  int robot = IsRobot(id);
  if(robot >= 0) {
    if(robots[robot].robot->joints[0].type == RobotJoint::Floating) 
      robots[robot].robot->SetJointByTransform(0,5,T);
    else 
      robots[robot].robot->links[0].T0_Parent = T;
    robots[robot].robot->UpdateFrames();
    return;
  }
  pair<int,int> robotLink = IsRobotLink(id);
  if(robotLink.first >= 0) {
    if(robots[robotLink.first].robot->joints[0].type == RobotJoint::Floating) {
      if(robotLink.second != 5) FatalError("SetTransform: cannot set transforms of arbitrary robot links");
      robots[robotLink.first].robot->SetJointByTransform(0,robotLink.second,T);
    }
    else
      FatalError("SetTransform: cannot set transforms of arbitrary robot links");
  }
  FatalError("SetTransform: Invalid ID: %d\n",id);
}

void RobotWorld::UpdateGeometry()
{
  for(size_t i=0;i<robots.size();i++) {
    robots[i].robot->UpdateGeometry();
  }
  for(size_t i=0;i<rigidObjects.size();i++) {
    rigidObjects[i].object->UpdateGeometry();
  }
}

void RobotWorld::SetGLLights()
{
  for(size_t i=0;i<lights.size();i++) 
    lights[i].setCurrentGL(i);
  DEBUG_GL_ERRORS()
    /*
  for(int i=(int)lights.size();i<GL_MAX_LIGHTS;i++) {
    printf("disable %d\n",i);
    glDisable(GL_LIGHT0+i);
    DEBUG_GL_ERRORS()
      }
    */
}

void RobotWorld::DrawGL()
{
  for(size_t i=0;i<robots.size();i++)
    robots[i].view.Draw();
  for(size_t i=0;i<terrains.size();i++)
    terrains[i].view.Draw();
  for(size_t i=0;i<rigidObjects.size();i++)
    rigidObjects[i].view.Draw();
}

int RobotWorld::LoadRobot(const string& fn)
{
  Robot* robot = new Robot;
  if(!robot->Load(fn.c_str())) {
    delete robot;
    return -1;
  }
  const char* justfn = GetFileName(fn.c_str());
  char* buf = new char[strlen(justfn)+1];
  strcpy(buf,justfn);
  StripExtension(buf);
  string name=buf;
  delete [] buf;
  int i = AddRobot(name,robot);
  return i;
}

int RobotWorld::AddRobot(const string& name,Robot* robot)
{
  robots.resize(robots.size()+1);
  robots.back().name = name;
  robots.back().robot = robot;
  if(robot) {
    robots.back().view.robot = robot;
    robots.back().view.SetGrey();
  }
  return (int)robots.size()-1;
}

void RobotWorld::DeleteRobot(const string& name)
{
  for(size_t i=0;i<robots.size();i++) {
    if(robots[i].name == name) {
      robots.erase(robots.begin()+i);
      i--;
    }
  }
}

Robot* RobotWorld::GetRobot(const string& name)
{
  for(size_t i=0;i<robots.size();i++) 
    if(robots[i].name == name) return robots[i].robot;
  return NULL;
}

ViewRobot* RobotWorld::GetRobotView(const string& name)
{
  for(size_t i=0;i<robots.size();i++) 
    if(robots[i].name == name) return &robots[i].view;
  return NULL;
}




int RobotWorld::LoadTerrain(const string& fn)
{
  Environment* t = new Environment;
  if(!t->Load(fn.c_str())) {
    delete t;
    return -1;
  }
  AABB3D bb = t->geometry.GetAABB();
  //printf("Environment %s bounding box [%g,%g]x[%g,%g]x[%g,%g]\n",fn.c_str(),bb.bmin.x,bb.bmax.x,bb.bmin.y,bb.bmax.y,bb.bmin.z,bb.bmax.z);
  const char* justfn = GetFileName(fn.c_str());
  char* buf = new char[strlen(justfn)+1];
  strcpy(buf,justfn);
  StripExtension(buf);
  string name=buf;
  delete [] buf;
  int i = AddTerrain(name,t);
  return i;
}

int RobotWorld::AddTerrain(const string& name,Environment* t)
{
  terrains.resize(terrains.size()+1);
  terrains.back().name = name;
  terrains.back().terrain = t;
  terrains.back().view.env = terrains.back().terrain;
  return (int)terrains.size()-1;
}

void RobotWorld::DeleteTerrain(const string& name)
{
  for(size_t i=0;i<terrains.size();i++) {
    if(terrains[i].name == name) {
      terrains.erase(terrains.begin()+i);
      i--;
    }
  }  
}

Environment* RobotWorld::GetTerrain(const string& name)
{
  for(size_t i=0;i<terrains.size();i++) 
    if(terrains[i].name == name) return terrains[i].terrain;
  return NULL;
}

ViewEnvironment* RobotWorld::GetTerrainView(const string& name)
{
  for(size_t i=0;i<terrains.size();i++) 
    if(terrains[i].name == name) return &terrains[i].view;
  return NULL;
}




int RobotWorld::LoadRigidObject(const string& fn)
{
  RigidObject* t = new RigidObject;
  if(!t->Load(fn.c_str())) {
    delete t;
    return -1;
  }
  const char* justfn = GetFileName(fn.c_str());
  char* buf = new char[strlen(justfn)+1];
  strcpy(buf,justfn);
  StripExtension(buf);
  string name=buf;
  delete [] buf;
  int i = AddRigidObject(name,t);
  return i;
}

int RobotWorld::AddRigidObject(const string& name,RigidObject* t)
{
  rigidObjects.resize(rigidObjects.size()+1);
  rigidObjects.back().name = name;
  rigidObjects.back().object = t;
  rigidObjects.back().view.obj = rigidObjects.back().object;
  return (int)rigidObjects.size()-1;
}

void RobotWorld::DeleteRigidObject(const string& name)
{
  for(size_t i=0;i<rigidObjects.size();i++) {
    if(rigidObjects[i].name == name) {
      rigidObjects.erase(rigidObjects.begin()+i);
      i--;
    }
  }  
}

RigidObject* RobotWorld::GetRigidObject(const string& name)
{
  for(size_t i=0;i<rigidObjects.size();i++) 
    if(rigidObjects[i].name == name) return rigidObjects[i].object;
  return NULL;
}

ViewRigidObject* RobotWorld::GetRigidObjectView(const string& name)
{
  for(size_t i=0;i<rigidObjects.size();i++) 
    if(rigidObjects[i].name == name) return &rigidObjects[i].view;
  return NULL;
}





RobotInfo* RobotWorld::ClickRobot(const Ray3D& r,int& body,Vector3& localpt)
{
  RobotInfo* closestRobot=NULL;
  Real closestDist = Inf;
  Vector3 closestPoint;
  int closestBody = -1;
  Vector3 worldpt;
  for(size_t j=0;j<robots.size();j++) {
    Robot* robot = robots[j].robot;
    robot->UpdateGeometry();
    for(size_t i=0;i<robot->links.size();i++) {
      if(robot->geometry[i].Empty()) continue;
      Real dist;
      if(robot->geometry[i].RayCast(r,&dist)) {
	if(dist < closestDist) {
	  closestDist = dist;
	  closestPoint = r.source + dist*r.direction;
	  closestBody = i;
	  closestRobot = &robots[j];
	}
      }
    }
  }
  if(closestRobot)
    closestRobot->robot->links[closestBody].T_World.mulInverse(closestPoint,localpt);
  body = closestBody;
  return closestRobot;
}

RigidObjectInfo* RobotWorld::ClickObject(const Ray3D& r,Vector3& localpt)
{
  RigidObjectInfo* closest=NULL;
  Real closestDist = Inf;
  Vector3 closestPoint;
  Vector3 worldpt;
  for(size_t j=0;j<rigidObjects.size();j++) {
    RigidObject* obj = rigidObjects[j].object;
    obj->geometry.SetTransform(obj->T);
    Real dist;
    if(obj->geometry.RayCast(r,&dist)) {
      if(dist < closestDist) {
	closestDist = dist;
	closestPoint = r.source+dist*r.direction;
	closest = &rigidObjects[j];
      }
    }
  }
  if(closest)
    closest->object->T.mulInverse(closestPoint,localpt);
  return closest;
}

void CopyWorld(const RobotWorld& a,RobotWorld& b)
{
  b.camera=a.camera;
  b.viewport=a.viewport;
  b.lights=a.lights;

  b.robots.resize(a.robots.size());
  b.terrains.resize(a.terrains.size());
  b.rigidObjects.resize(a.rigidObjects.size());
  for(size_t i=0;i<b.robots.size();i++) {
    b.robots[i].name = a.robots[i].name;
    b.robots[i].view = a.robots[i].view;
    b.robots[i].robot = new Robot;
    *b.robots[i].robot = *a.robots[i].robot;
    b.robots[i].view.robot = b.robots[i].robot;
  }
  for(size_t i=0;i<b.terrains.size();i++) {
    b.terrains[i].name = a.terrains[i].name;
    b.terrains[i].view = a.terrains[i].view;
    b.terrains[i].terrain = new Environment;
    *b.terrains[i].terrain = *a.terrains[i].terrain;
    b.terrains[i].view.env = b.terrains[i].terrain;
  }
  for(size_t i=0;i<b.rigidObjects.size();i++) {
    b.rigidObjects[i].name = a.rigidObjects[i].name;
    b.rigidObjects[i].view = a.rigidObjects[i].view;
    b.rigidObjects[i].object = new RigidObject;
    *b.rigidObjects[i].object = *a.rigidObjects[i].object;
    b.rigidObjects[i].view.obj = b.rigidObjects[i].object;
  }

}

int RobotWorld::LoadElement(const string& sfn)
{
  const char* fn = sfn.c_str();
  const char* ext=FileExtension(fn);
  if(0==strcmp(ext,"rob") || 0==strcmp(ext,"urdf")) {
    int res=LoadRobot(fn);
    if(res<0) {
      printf("Error loading robot file %s\n",fn);
      return -1;
    }
    return RobotID(res);
  }
  else if(0==strcmp(ext,"obj")) {
    int res=LoadRigidObject(fn);
    if(res<0) {
      if(Geometry::AnyGeometry3D::CanLoadExt(ext)) { //try loading as OBJ file
	res = LoadTerrain(fn);
	if(res >= 0) return TerrainID(res);
      }
      printf("Error loading rigid object file %s\n",fn);
      return -1;
    }
    return RigidObjectID(res);
  }
  else if(0==strcmp(ext,"env") || Geometry::AnyGeometry3D::CanLoadExt(ext)) {
    int res=LoadTerrain(fn);
    if(res < 0) {
      printf("Error loading terrain file %s\n",fn);
      return -1;
    }
    return TerrainID(res);
  }
  else {
    printf("Unknown file extension %s on file %s\n",ext,fn);
    return -1;
  }
}

