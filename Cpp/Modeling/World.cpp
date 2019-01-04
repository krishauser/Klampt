#include "World.h"
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/GLError.h>
#include <string.h>
#include <KrisLibrary/meshing/IO.h>
#include "IO/XmlWorld.h"

RobotWorld::RobotWorld()
{
  background.set(0.4,0.4,1,0);
}

bool RobotWorld::LoadXML(const char* fn)
{
  XmlWorld xmlWorld;
  if(!xmlWorld.Load(fn)) {
    printf("RobotWorld::LoadXML: Error loading world file %s\n",fn);
    return false;
  }
  if(!xmlWorld.GetWorld(*this)) {
    printf("RobotWorld::LoadXML: Error extracting world data from %s\n",fn);
    return false;
  }
  return true;
}

bool RobotWorld::SaveXML(const char* fn,const char* elementDir)
{
  XmlWorld xmlWorld;
  if(elementDir)
    return xmlWorld.Save(*this,fn,elementDir);
  else
    return xmlWorld.Save(*this,fn);
}

int RobotWorld::NumIDs() const
{
  size_t n=terrains.size()+rigidObjects.size()+robots.size();
  for(size_t i=0;i<robots.size();i++)
    n += robots[i]->links.size();
  return n;
}

int RobotWorld::GetID(const string& name,int link) const
{
  for(size_t i=0;i<terrains.size();i++)
    if(terrains[i]->name == name)
      return TerrainID(i);
  for(size_t i=0;i<rigidObjects.size();i++)
    if(rigidObjects[i]->name == name) 
      return RigidObjectID(i);
  for(size_t i=0;i<robots.size();i++)
    if(robots[i]->name == name) {
      if(link < 0) return RobotID(i);
      else return RobotLinkID(i,link);
    }
  return -1;
}

string RobotWorld::GetName(int id) const
{
  int i = IsRigidObject(id);
  if(i >= 0) return rigidObjects[i]->name;
  i = IsTerrain(id);
  if(i >= 0) return terrains[i]->name;
  i = IsRobot(id);
  if(i >= 0) return robots[i]->name;
  pair<int,int> j = IsRobotLink(id);
  if(j.first >= 0) return robots[j.first]->name+"["+robots[j.first]->linkNames[j.second]+"]";
  return "";
}

int RobotWorld::IsRobot(int id) const
{
  if(id >= (int)(rigidObjects.size()+terrains.size())) {
    id -= (int)(rigidObjects.size()+terrains.size());
    int index=0;
    for(size_t i=0;i<robots.size();i++) {
      if(index == id) return (int)i;
      index += 1 + robots[i]->links.size();
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
      if(id < index+(int)robots[i]->links.size())
	return pair<int,int>(i,id-index);
      index += robots[i]->links.size();
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
    id += (int)robots[i]->links.size()+1;
  return id;
}

int RobotWorld::RobotLinkID(int index,int link) const
{
  int id=(int)(terrains.size()+rigidObjects.size());
  for(int i=0;i<index;i++)
    id += (int)robots[i]->links.size()+1;
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

RobotWorld::GeometryPtr RobotWorld::GetGeometry(int id)
{
  int terrain = IsTerrain(id);
  if(terrain >= 0)
    return terrains[terrain]->geometry;
  int rigidObject = IsRigidObject(id);
  if(rigidObject >= 0)
    return rigidObjects[rigidObject]->geometry;
  pair<int,int> robotLink = IsRobotLink(id);
  if(robotLink.first >= 0) {
    return robots[robotLink.first]->geometry[robotLink.second];
  }
  fprintf(stderr,"RobotWorld::GetGeometry: Invalid ID: %d\n",id);
  return NULL;
}

RobotWorld::AppearancePtr RobotWorld::GetAppearance(int id)
{
  int terrain = IsTerrain(id);
  if(terrain >= 0)
    return terrains[terrain]->geometry.Appearance();
  int rigidObject = IsRigidObject(id);
  if(rigidObject >= 0)
    return rigidObjects[rigidObject]->geometry.Appearance();
  pair<int,int> robotLink = IsRobotLink(id);
  if(robotLink.first >= 0) {
    return robots[robotLink.first]->geomManagers[robotLink.second].Appearance();
  }
  fprintf(stderr,"RobotWorld::GetAppearance: Invalid ID: %d\n",id);
  return NULL;
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
    return rigidObjects[rigidObject]->T;
  pair<int,int> robotLink = IsRobotLink(id);
  if(robotLink.first >= 0) {
    return robots[robotLink.first]->links[robotLink.second].T_World;
  }
  fprintf(stderr,"GetTransform: Invalid ID: %d\n",id);
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
    rigidObjects[rigidObject]->T = T;
    return;
  }
  int robot = IsRobot(id);
  if(robot >= 0) {
    if(robots[robot]->joints[0].type == RobotJoint::Floating) 
      robots[robot]->SetJointByTransform(0,5,T);
    else 
      robots[robot]->links[0].T0_Parent = T;
    robots[robot]->UpdateFrames();
    return;
  }
  pair<int,int> robotLink = IsRobotLink(id);
  if(robotLink.first >= 0) {
    if(robots[robotLink.first]->joints[0].type == RobotJoint::Floating) {
      if(robotLink.second != 5) FatalError("SetTransform: cannot set transforms of arbitrary robot links");
      robots[robotLink.first]->SetJointByTransform(0,robotLink.second,T);
    }
    else
      FatalError("SetTransform: cannot set transforms of arbitrary robot links");
  }
  FatalError("SetTransform: Invalid ID: %d\n",id);
}

void RobotWorld::InitCollisions()
{
  for(size_t j=0;j<robots.size();j++) 
    robots[j]->InitCollisions();
  for(size_t j=0;j<rigidObjects.size();j++) 
    rigidObjects[j]->InitCollisions();
  for(size_t j=0;j<terrains.size();j++) 
    terrains[j]->InitCollisions();
}

void RobotWorld::UpdateGeometry()
{
  for(size_t i=0;i<robots.size();i++) {
    robots[i]->UpdateGeometry();
  }
  for(size_t i=0;i<rigidObjects.size();i++) {
    rigidObjects[i]->UpdateGeometry();
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
    robotViews[i].Draw();
  for(size_t i=0;i<terrains.size();i++)
    terrains[i]->DrawGL();
  for(size_t i=0;i<rigidObjects.size();i++)
    rigidObjects[i]->DrawGL();
}

int RobotWorld::LoadRobot(const string& fn)
{
  Robot* robot = new Robot;
  printf("RobotWorld::LoadRobot: %s\n",fn.c_str());
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
  robots.back().reset(robot);
  robotViews.resize(robots.size());
  if(robot) {
    robot->name = name;
    robotViews.back() = ViewRobot(robot);
  }
  return (int)robots.size()-1;
}

void RobotWorld::DeleteRobot(const string& name)
{
  for(size_t i=0;i<robots.size();i++) {
    if(robots[i]->name == name) {
      robots.erase(robots.begin()+i);
      robotViews.erase(robotViews.begin()+i);
      i--;
    }
  }
}

Robot* RobotWorld::GetRobot(const string& name)
{
  for(size_t i=0;i<robots.size();i++) 
    if(robots[i]->name == name) return robots[i].get();
  return NULL;
}

ViewRobot* RobotWorld::GetRobotView(const string& name)
{
  for(size_t i=0;i<robots.size();i++) 
    if(robots[i]->name == name) return &robotViews[i];
  return NULL;
}




int RobotWorld::LoadTerrain(const string& fn)
{
  Terrain* t = new Terrain;
  if(!t->Load(fn.c_str())) {
    delete t;
    return -1;
  }
  //AABB3D bb = t->geometry.GetAABB();
  //printf("Terrain %s bounding box [%g,%g]x[%g,%g]x[%g,%g]\n",fn.c_str(),bb.bmin.x,bb.bmax.x,bb.bmin.y,bb.bmax.y,bb.bmin.z,bb.bmax.z);
  const char* justfn = GetFileName(fn.c_str());
  char* buf = new char[strlen(justfn)+1];
  strcpy(buf,justfn);
  StripExtension(buf);
  string name=buf;
  delete [] buf;
  int i = AddTerrain(name,t);
  return i;
}

int RobotWorld::AddTerrain(const string& name,Terrain* t)
{
  terrains.resize(terrains.size()+1);
  terrains.back().reset(t);
  if(t) t->name = name;
  return (int)terrains.size()-1;
}

void RobotWorld::DeleteTerrain(const string& name)
{
  for(size_t i=0;i<terrains.size();i++) {
    if(terrains[i]->name == name) {
      terrains.erase(terrains.begin()+i);
      i--;
    }
  }  
}

Terrain* RobotWorld::GetTerrain(const string& name)
{
  for(size_t i=0;i<terrains.size();i++) 
    if(terrains[i]->name == name) return terrains[i].get();
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
  if(t) t->name = name;
  rigidObjects.resize(rigidObjects.size()+1);
  rigidObjects.back().reset(t);
  return (int)rigidObjects.size()-1;
}

void RobotWorld::DeleteRigidObject(const string& name)
{
  for(size_t i=0;i<rigidObjects.size();i++) {
    if(rigidObjects[i]->name == name) {
      rigidObjects.erase(rigidObjects.begin()+i);
      i--;
    }
  }  
}

RigidObject* RobotWorld::GetRigidObject(const string& name)
{
  for(size_t i=0;i<rigidObjects.size();i++) 
    if(rigidObjects[i]->name == name) return rigidObjects[i].get();
  return NULL;
}



int RobotWorld::RayCast(const Ray3D& r,Vector3& worldpt)
{
  for(size_t j=0;j<robots.size();j++) 
    robots[j]->InitCollisions();
  for(size_t j=0;j<rigidObjects.size();j++) 
    rigidObjects[j]->InitCollisions();
  for(size_t j=0;j<terrains.size();j++) 
    terrains[j]->InitCollisions();
  int closestBody = -1;
  Real closestDist = Inf;
  Vector3 closestPoint;
  for(size_t j=0;j<robots.size();j++) {
    Robot* robot = robots[j].get();
    robot->UpdateGeometry();
    for(size_t i=0;i<robot->links.size();i++) {
      if(robot->IsGeometryEmpty(i)) continue;
      Real dist;
      if(robot->geometry[i]->RayCast(r,&dist)) {
        if(dist < closestDist) {
          closestDist = dist;
          closestPoint = r.source + dist*r.direction;
          closestBody = RobotLinkID(j,i);
        }
      }
    }
  }
  for(size_t j=0;j<rigidObjects.size();j++) {
    RigidObject* obj = rigidObjects[j].get();
    obj->geometry->SetTransform(obj->T);
    Real dist;
    if(obj->geometry->RayCast(r,&dist)) {
      if(dist < closestDist) {
        closestDist = dist;
        closestPoint = r.source+dist*r.direction;
        closestBody = RigidObjectID(j);
      }
    }
  }
  for(size_t j=0;j<terrains.size();j++) {
    Terrain* ter = terrains[j].get();
    Real dist;
    if(ter->geometry->RayCast(r,&dist)) {
      if(dist < closestDist) {
        closestDist = dist;
        closestPoint = r.source+dist*r.direction;
        closestBody = TerrainID(j);
      }
    }
  }
  worldpt = closestPoint;
  return closestBody;
}

Robot* RobotWorld::RayCastRobot(const Ray3D& r,int& body,Vector3& localpt)
{
  //doing it this way rather than dynamic initialization gives better 
  //debug printing info
  for(size_t j=0;j<robots.size();j++) {
    robots[j]->InitCollisions();
  }

  Robot* closestRobot=NULL;
  Real closestDist = Inf;
  Vector3 closestPoint;
  int closestBody = -1;
  Vector3 worldpt;
  for(size_t j=0;j<robots.size();j++) {
    Robot* robot = robots[j].get();
    robot->UpdateGeometry();
    for(size_t i=0;i<robot->links.size();i++) {
      if(robot->IsGeometryEmpty(i)) continue;
      Real dist;
      if(robot->geometry[i]->RayCast(r,&dist)) {
	if(dist < closestDist) {
	  closestDist = dist;
	  closestPoint = r.source + dist*r.direction;
	  closestBody = i;
	  closestRobot = robots[j].get();
	}
      }
    }
  }
  if(closestRobot)
    closestRobot->links[closestBody].T_World.mulInverse(closestPoint,localpt);
  body = closestBody;
  return closestRobot;
}

RigidObject* RobotWorld::RayCastObject(const Ray3D& r,Vector3& localpt)
{
  //doing it this way rather than dynamic initialization gives better 
  //debug printing info
  for(size_t j=0;j<rigidObjects.size();j++) {
    rigidObjects[j]->InitCollisions();
  }

  RigidObject* closest=NULL;
  Real closestDist = Inf;
  Vector3 closestPoint;
  Vector3 worldpt;
  for(size_t j=0;j<rigidObjects.size();j++) {
    RigidObject* obj = rigidObjects[j].get();
    obj->geometry->SetTransform(obj->T);
    Real dist;
    if(obj->geometry->RayCast(r,&dist)) {
      if(dist < closestDist) {
	closestDist = dist;
	closestPoint = r.source+dist*r.direction;
	closest = rigidObjects[j].get();
      }
    }
  }
  if(closest)
    closest->T.mulInverse(closestPoint,localpt);
  return closest;
}

void CopyWorld(const RobotWorld& a,RobotWorld& b)
{
  b.camera=a.camera;
  b.viewport=a.viewport;
  b.lights=a.lights;

  b.robots.resize(a.robots.size());
  b.robotViews.resize(a.robots.size());
  b.terrains.resize(a.terrains.size());
  b.rigidObjects.resize(a.rigidObjects.size());
  for(size_t i=0;i<b.robots.size();i++) {
    b.robots[i] = make_shared<Robot>();
    *b.robots[i] = *a.robots[i];
    b.robotViews[i] = a.robotViews[i];
    b.robotViews[i].robot = b.robots[i].get();
  }
  for(size_t i=0;i<b.terrains.size();i++) {
    b.terrains[i] = make_shared<Terrain>();
    *b.terrains[i] = *a.terrains[i];
  }
  for(size_t i=0;i<b.rigidObjects.size();i++) {
    b.rigidObjects[i] = make_shared<RigidObject>();
    *b.rigidObjects[i] = *a.rigidObjects[i];
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
    printf("RobotWorld::Load: Unknown file extension %s on file %s\n",ext,fn);
    return -1;
  }
}

bool RobotWorld::CanLoadElementExt(const char* ext) const
{
  if(!ext) return false;
  if(0==strcmp(ext,"rob") || 0==strcmp(ext,"urdf")) {
    return true;
  }
  if(0==strcmp(ext,"obj")) {
    return true;
  }
  if(0==strcmp(ext,"env") || Geometry::AnyGeometry3D::CanLoadExt(ext)) {
    return true;
  }
  return false;
}
