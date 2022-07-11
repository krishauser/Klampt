#include "World.h"
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/GLError.h>
#include <string.h>
#include <KrisLibrary/meshing/IO.h>
#include "IO/XmlWorld.h"

namespace Klampt {

WorldModel::WorldModel()
{
  background.set(0.4f,0.4f,1,0);
}

bool WorldModel::LoadXML(const char* fn)
{
  XmlWorld xmlWorld;
  if(!xmlWorld.Load(fn)) {
    printf("WorldModel::LoadXML: Error loading world file %s\n",fn);
    return false;
  }
  if(!xmlWorld.GetWorld(*this)) {
    printf("WorldModel::LoadXML: Error extracting world data from %s\n",fn);
    return false;
  }
  return true;
}

bool WorldModel::SaveXML(const char* fn,const char* elementDir)
{
  XmlWorld xmlWorld;
  if(elementDir)
    return xmlWorld.Save(*this,fn,elementDir);
  else
    return xmlWorld.Save(*this,fn);
}

int WorldModel::NumIDs() const
{
  size_t n=terrains.size()+rigidObjects.size()+robots.size();
  for(size_t i=0;i<robots.size();i++)
    n += robots[i]->links.size();
  return n;
}

int WorldModel::GetID(const string& name,int link) const
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

string WorldModel::GetName(int id) const
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

int WorldModel::IsRobot(int id) const
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

pair<int,int> WorldModel::IsRobotLink(int id) const
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

int WorldModel::IsTerrain(int id) const
{
  if(id >= 0 && id < (int)terrains.size()) return id;
  return -1;
}

int WorldModel::IsRigidObject(int id) const
{
  if(id >= (int)terrains.size() && id < (int)(terrains.size()+rigidObjects.size())) return id-(int)terrains.size();
  return -1;
}
int WorldModel::RobotID(int index) const
{
  int id=(int)(terrains.size()+rigidObjects.size());
  for(int i=0;i<index;i++)
    id += (int)robots[i]->links.size()+1;
  return id;
}

int WorldModel::RobotLinkID(int index,int link) const
{
  int id=(int)(terrains.size()+rigidObjects.size());
  for(int i=0;i<index;i++)
    id += (int)robots[i]->links.size()+1;
  return id+1+link;
}

int WorldModel::TerrainID(int index) const
{
  return index;
}

int WorldModel::RigidObjectID(int index) const
{
  return index+(int)terrains.size();
}

WorldModel::GeometryPtr WorldModel::GetGeometry(int id)
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
  fprintf(stderr,"WorldModel::GetGeometry: Invalid ID: %d\n",id);
  return NULL;
}

WorldModel::AppearancePtr WorldModel::GetAppearance(int id)
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
  fprintf(stderr,"WorldModel::GetAppearance: Invalid ID: %d\n",id);
  return NULL;
}

RigidTransform WorldModel::GetTransform(int id) const
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

void WorldModel::SetTransform(int id,const RigidTransform& T)
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
    if(robots[robot]->joints[0].type == RobotModelJoint::Floating) 
      robots[robot]->SetJointByTransform(0,5,T);
    else 
      robots[robot]->links[0].T0_Parent = T;
    robots[robot]->UpdateFrames();
    return;
  }
  pair<int,int> robotLink = IsRobotLink(id);
  if(robotLink.first >= 0) {
    if(robots[robotLink.first]->joints[0].type == RobotModelJoint::Floating) {
      if(robotLink.second != 5) FatalError("SetTransform: cannot set transforms of arbitrary robot links");
      robots[robotLink.first]->SetJointByTransform(0,robotLink.second,T);
    }
    else
      FatalError("SetTransform: cannot set transforms of arbitrary robot links");
  }
  FatalError("SetTransform: Invalid ID: %d\n",id);
}

void WorldModel::InitCollisions()
{
  for(size_t j=0;j<robots.size();j++) 
    robots[j]->InitCollisions();
  for(size_t j=0;j<rigidObjects.size();j++) 
    rigidObjects[j]->InitCollisions();
  for(size_t j=0;j<terrains.size();j++) 
    terrains[j]->InitCollisions();
}

void WorldModel::UpdateGeometry()
{
  for(size_t i=0;i<robots.size();i++) {
    robots[i]->UpdateGeometry();
  }
  for(size_t i=0;i<rigidObjects.size();i++) {
    rigidObjects[i]->UpdateGeometry();
  }
}

void WorldModel::SetGLLights()
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

void WorldModel::DrawGL()
{
  for(size_t i=0;i<robots.size();i++)
    robotViews[i].DrawOpaque(true);
  for(size_t i=0;i<terrains.size();i++)
    terrains[i]->DrawGLOpaque(true);
  for(size_t i=0;i<rigidObjects.size();i++)
    rigidObjects[i]->DrawGLOpaque(true);
  for(size_t i=0;i<robots.size();i++)
    robotViews[i].DrawOpaque(false);
  for(size_t i=0;i<terrains.size();i++)
    terrains[i]->DrawGLOpaque(false);
  for(size_t i=0;i<rigidObjects.size();i++)
    rigidObjects[i]->DrawGLOpaque(false);
}

int WorldModel::LoadRobot(const string& fn)
{
  RobotModel* robot = new RobotModel;
  printf("WorldModel::LoadRobot: %s\n",fn.c_str());
  if(!robot->Load(fn.c_str())) {
    delete robot;
    return -1;
  }
  const char* justfn = GetFileName(fn.c_str());
  char* buf = new char[strlen(justfn)+1];
  strcpy(buf, justfn);
  StripExtension(buf);
  string name=buf;
  delete [] buf;
  int i = AddRobot(name,robot);
  return i;
}

int WorldModel::AddRobot(const string& name,RobotModel* robot)
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

void WorldModel::DeleteRobot(const string& name)
{
  for(size_t i=0;i<robots.size();i++) {
    if(robots[i]->name == name) {
      robots.erase(robots.begin()+i);
      robotViews.erase(robotViews.begin()+i);
      i--;
    }
  }
}

RobotModel* WorldModel::GetRobot(const string& name)
{
  for(size_t i=0;i<robots.size();i++) 
    if(robots[i]->name == name) return robots[i].get();
  return NULL;
}

ViewRobot* WorldModel::GetRobotView(const string& name)
{
  for(size_t i=0;i<robots.size();i++) 
    if(robots[i]->name == name) return &robotViews[i];
  return NULL;
}




int WorldModel::LoadTerrain(const string& fn)
{
  TerrainModel* t = new TerrainModel;
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

int WorldModel::AddTerrain(const string& name,TerrainModel* t)
{
  terrains.resize(terrains.size()+1);
  terrains.back().reset(t);
  if(t) t->name = name;
  return (int)terrains.size()-1;
}

void WorldModel::DeleteTerrain(const string& name)
{
  for(size_t i=0;i<terrains.size();i++) {
    if(terrains[i]->name == name) {
      terrains.erase(terrains.begin()+i);
      i--;
    }
  }  
}

TerrainModel* WorldModel::GetTerrain(const string& name)
{
  for(size_t i=0;i<terrains.size();i++) 
    if(terrains[i]->name == name) return terrains[i].get();
  return NULL;
}




int WorldModel::LoadRigidObject(const string& fn)
{
  RigidObjectModel* t = new RigidObjectModel;
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

int WorldModel::AddRigidObject(const string& name,RigidObjectModel* t)
{
  if(t) t->name = name;
  rigidObjects.resize(rigidObjects.size()+1);
  rigidObjects.back().reset(t);
  return (int)rigidObjects.size()-1;
}

void WorldModel::DeleteRigidObject(const string& name)
{
  for(size_t i=0;i<rigidObjects.size();i++) {
    if(rigidObjects[i]->name == name) {
      rigidObjects.erase(rigidObjects.begin()+i);
      i--;
    }
  }  
}

RigidObjectModel* WorldModel::GetRigidObject(const string& name)
{
  for(size_t i=0;i<rigidObjects.size();i++) 
    if(rigidObjects[i]->name == name) return rigidObjects[i].get();
  return NULL;
}



int WorldModel::RayCast(const Ray3D& r,Vector3& worldpt)
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
    RobotModel* robot = robots[j].get();
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
    RigidObjectModel* obj = rigidObjects[j].get();
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
    TerrainModel* ter = terrains[j].get();
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

int WorldModel::RayCastIgnore(const Ray3D& r,const vector<int>& ignoreIDList,Vector3& worldpt)
{
  vector<bool> ignoreIDs(NumIDs(),false);
  for(auto i:ignoreIDList)
    ignoreIDs[i] = true;
  int idBase = 0;
  for(size_t j=0;j<robots.size();j++) {
    if(ignoreIDs[RobotID((int)j)]) continue;
    robots[j]->InitCollisions();
  }
  idBase = RigidObjectID(0);
  for(size_t j=0;j<rigidObjects.size();j++) {
    if(ignoreIDs[idBase+(int)j]) continue;
    rigidObjects[j]->InitCollisions();
  }
  idBase = TerrainID(0);
  for(size_t j=0;j<terrains.size();j++) {
    if(ignoreIDs[idBase+(int)j]) continue;
    terrains[j]->InitCollisions();
  }
  int closestBody = -1;
  Real closestDist = Inf;
  Vector3 closestPoint;
  for(size_t j=0;j<robots.size();j++) {
    if(ignoreIDs[RobotID((int)j)]) continue;
    RobotModel* robot = robots[j].get();
    robot->UpdateGeometry();
    idBase = RobotLinkID((int)j,0);
    for(size_t i=0;i<robot->links.size();i++) {
      if(ignoreIDs[idBase+(int)i]) continue;
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
  idBase = RigidObjectID(0);
  for(size_t j=0;j<rigidObjects.size();j++) {
    if(ignoreIDs[idBase+(int)j]) continue;
    RigidObjectModel* obj = rigidObjects[j].get();
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
  idBase = TerrainID(0);
  for(size_t j=0;j<terrains.size();j++) {
    if(ignoreIDs[idBase+(int)j]) continue;
    TerrainModel* ter = terrains[j].get();
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

int WorldModel::RayCastSelected(const Ray3D& r,const vector<int>& selectIDList,Vector3& worldpt)
{
  vector<bool> ignoreIDs(NumIDs(),true);
  for(auto i:selectIDList)
    ignoreIDs[i] = false;
  int idBase = 0;
  for(size_t j=0;j<robots.size();j++) {
    if(ignoreIDs[RobotID((int)j)]) continue;
    robots[j]->InitCollisions();
  }
  idBase = RigidObjectID(0);
  for(size_t j=0;j<rigidObjects.size();j++) {
    if(ignoreIDs[idBase+(int)j]) continue;
    rigidObjects[j]->InitCollisions();
  }
  idBase = TerrainID(0);
  for(size_t j=0;j<terrains.size();j++) {
    if(ignoreIDs[idBase+(int)j]) continue;
    terrains[j]->InitCollisions();
  }
  int closestBody = -1;
  Real closestDist = Inf;
  Vector3 closestPoint;
  for(size_t j=0;j<robots.size();j++) {
    if(ignoreIDs[RobotID((int)j)]) continue;
    RobotModel* robot = robots[j].get();
    robot->UpdateGeometry();
    idBase = RobotLinkID((int)j,0);
    for(size_t i=0;i<robot->links.size();i++) {
      if(ignoreIDs[idBase+(int)i]) continue;
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
  idBase = RigidObjectID(0);
  for(size_t j=0;j<rigidObjects.size();j++) {
    if(ignoreIDs[idBase+(int)j]) continue;
    RigidObjectModel* obj = rigidObjects[j].get();
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
  idBase = TerrainID(0);
  for(size_t j=0;j<terrains.size();j++) {
    if(ignoreIDs[idBase+(int)j]) continue;
    TerrainModel* ter = terrains[j].get();
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

RobotModel* WorldModel::RayCastRobot(const Ray3D& r,int& body,Vector3& localpt)
{
  //doing it this way rather than dynamic initialization gives better 
  //debug printing info
  for(size_t j=0;j<robots.size();j++) {
    robots[j]->InitCollisions();
  }

  RobotModel* closestRobot=NULL;
  Real closestDist = Inf;
  Vector3 closestPoint;
  int closestBody = -1;
  Vector3 worldpt;
  for(size_t j=0;j<robots.size();j++) {
    RobotModel* robot = robots[j].get();
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

RigidObjectModel* WorldModel::RayCastObject(const Ray3D& r,Vector3& localpt)
{
  //doing it this way rather than dynamic initialization gives better 
  //debug printing info
  for(size_t j=0;j<rigidObjects.size();j++) {
    rigidObjects[j]->InitCollisions();
  }

  RigidObjectModel* closest=NULL;
  Real closestDist = Inf;
  Vector3 closestPoint;
  Vector3 worldpt;
  for(size_t j=0;j<rigidObjects.size();j++) {
    RigidObjectModel* obj = rigidObjects[j].get();
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

void WorldModel::Copy(const WorldModel& a)
{
  this->camera=a.camera;
  this->viewport=a.viewport;
  this->lights=a.lights;

  this->robots.resize(a.robots.size());
  this->robotViews.resize(a.robots.size());
  this->terrains.resize(a.terrains.size());
  this->rigidObjects.resize(a.rigidObjects.size());
  for(size_t i=0;i<this->robots.size();i++) {
    this->robots[i] = make_shared<RobotModel>();
    *this->robots[i] = *a.robots[i];
    this->robotViews[i] = a.robotViews[i];
    this->robotViews[i].robot = this->robots[i].get();
  }
  for(size_t i=0;i<this->terrains.size();i++) {
    this->terrains[i] = make_shared<TerrainModel>();
    *this->terrains[i] = *a.terrains[i];
  }
  for(size_t i=0;i<this->rigidObjects.size();i++) {
    this->rigidObjects[i] = make_shared<RigidObjectModel>();
    *this->rigidObjects[i] = *a.rigidObjects[i];
  }

}

int WorldModel::LoadElement(const string& sfn)
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
    printf("WorldModel::Load: Unknown file extension %s on file %s\n",ext,fn);
    return -1;
  }
}

bool WorldModel::CanLoadElementExt(const char* ext) const
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

} //namespace Klampt