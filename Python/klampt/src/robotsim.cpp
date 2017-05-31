#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include <vector>
#include <string>
#include "robotsim.h"
#include "widget.h"
#include "Control/Command.h"
#include "Control/PathController.h"
#include "Control/FeedforwardController.h"
#include "Control/LoggingController.h"
#include "Planning/RobotCSpace.h"
#include "Simulation/WorldSimulation.h"
#include "Modeling/Interpolate.h"
#include "Planning/RobotCSpace.h"
#include "IO/XmlWorld.h"
#include "IO/XmlODE.h"
#include "IO/ROS.h"
#include "IO/three.js.h" 
#include <KrisLibrary/robotics/NewtonEuler.h>
#include <KrisLibrary/robotics/Stability.h>
#include <KrisLibrary/robotics/TorqueSolver.h>
#include <KrisLibrary/meshing/PointCloud.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/drawMesh.h>
#include <KrisLibrary/GLdraw/Widget.h>
#include <KrisLibrary/GLdraw/TransformWidget.h>
#include "View/ObjectPoseWidget.h"
#include "View/RobotPoseWidget.h"
#include <KrisLibrary/utils/AnyCollection.h>
#include <KrisLibrary/utils/stringutils.h>
#include <ode/ode.h>
#include "pyerr.h"
#include "pyconvert.h"
#include <fstream>
#ifndef WIN32
#include <unistd.h>
#endif //WIN32

/***************************  GLOBALS: REFERENCING TO KLAMPT C++ CODE ***************************************/

/// Internally used.
struct WorldData
{
  RobotWorld* world;
  bool worldExternal;
  XmlWorld xmlWorld;
  int refCount;
};

/// Internally used.
struct SimData
{
  WorldSimulation sim;
};


/// Internally used.
struct WidgetData
{
  SmartPointer<GLDraw::Widget> widget;
  int refCount;
};


static vector<SmartPointer<WorldData> > worlds;
static list<int> worldDeleteList;

static vector<SmartPointer<SimData> > sims;
static list<int> simDeleteList;

static vector<WidgetData> widgets;
static list<int> widgetDeleteList;

static bool gEnableCollisionInitialization = false;

static int gStabilityNumFCEdges = 4;

int createWorld(RobotWorld* ptr=NULL)
{
  if(worldDeleteList.empty()) {
    worlds.push_back(new WorldData);
    if(ptr) {
      worlds.back()->world = ptr;
      worlds.back()->worldExternal = true;
    }
    else {
      worlds.back()->world = new RobotWorld;
      worlds.back()->worldExternal = false;
    }
    worlds.back()->refCount = 1;
    return (int)(worlds.size()-1);
  }
  else {
    int index = worldDeleteList.front();
    worldDeleteList.erase(worldDeleteList.begin());
    worlds[index] = new WorldData;
    if(ptr) {
      worlds[index]->world = ptr;
      worlds[index]->worldExternal = true;
    }
    else {
      worlds[index]->world = new RobotWorld;
      worlds[index]->worldExternal = false;
    }
    worlds[index]->refCount = 1;
    return index;
  }
}

void derefWorld(int index)
{
  if(worlds.empty()) {
    //may have already cleared module...
    return;
  }
  if(index < 0 || index >= (int)worlds.size())
    throw PyException("Invalid world index");
  if(!worlds[index])
    throw PyException("Invalid dereference");
  if(worlds[index]->refCount <= 0)
    throw PyException("Invalid dereference");

  worlds[index]->refCount--;
  //LOG4CXX_INFO(KrisLibrary::logger(),"Deref world "<<index<<": count "<<worlds[index]->refCount);
  if(worlds[index]->refCount == 0) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Deleting world "<<index);
    if(!worlds[index]->worldExternal)
      delete worlds[index]->world;
    worlds[index] = NULL;
    worldDeleteList.push_back(index);
  }
}

void refWorld(int index)
{
  if(index < 0 || index >= (int)worlds.size())
    throw PyException("Invalid world index");
  if(!worlds[index])
    throw PyException("Invalid dereference");
  worlds[index]->refCount++;
  //LOG4CXX_INFO(KrisLibrary::logger(),"Ref world "<<index<<": count "<<worlds[index]->refCount);
}

int createSim()
{
  if(simDeleteList.empty()) {
    sims.push_back(new SimData);
    return (int)(sims.size()-1);
  }
  else {
    int index = simDeleteList.front();
    simDeleteList.erase(simDeleteList.begin());
    sims[index] = new SimData;
    return index;
  }
}

void destroySim(int index)
{
  if(worlds.empty()) {
    //may have already cleared module...
    return;
  }
  if(index < 0 || index >= (int)sims.size())
    throw PyException("Invalid sim index");
  if(!sims[index])
    throw PyException("Invalid sim index");

  sims[index] = NULL;
  simDeleteList.push_back(index);
}

int createWidget()
{
  if(widgetDeleteList.empty()) {
    widgets.resize(widgets.size()+1);
    widgets.back().refCount = 1;
    //LOG4CXX_INFO(KrisLibrary::logger(),"Creating widget "<<widgets.size()-1<<", ref count "<<1);
    return (int)(widgets.size()-1);
  }
  else {
    int index = widgetDeleteList.front();
    widgetDeleteList.erase(widgetDeleteList.begin());
    widgets[index].widget = NULL;
    widgets[index].refCount = 1;
    //LOG4CXX_INFO(KrisLibrary::logger(),"Creating widget "<<index<<", ref count "<<1);
    return index;
  }
}

void derefWidget(int index)
{
  if(index < 0 || index >= (int)widgets.size())
    throw PyException("Invalid widget index");
  if(widgets[index].refCount <= 0)
    throw PyException("Invalid dereference");

  widgets[index].refCount--;
  //LOG4CXX_INFO(KrisLibrary::logger(),"Deref widget "<<index<<": count "<<widgets[index].refCount);
  if(widgets[index].refCount == 0) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Deleting widget "<<index);
    widgets[index].widget = NULL;
    widgetDeleteList.push_back(index);
  }
}

void refWidget(int index)
{
  if(index < 0 || index >= (int)widgets.size())
    throw PyException("Invalid widget index");
  widgets[index].refCount++;
  //LOG4CXX_INFO(KrisLibrary::logger(),"Ref widget "<<index<<": count "<<widgets[index].refCount);
}

//cleans up internal data structures
void destroy()
{
  for(size_t i=0;i<sims.size();i++)
    sims[i] = NULL;
  for(size_t i=0;i<worlds.size();i++)
    worlds[i] = NULL;
  simDeleteList.clear();
  worldDeleteList.clear();
  sims.resize(0);
  worlds.resize(0);
  ManagedGeometry::manager.Clear();
}

void setRandomSeed(int seed)
{
  Math::Srand(seed);
}


/***************************  GEOMETRY CODE ***************************************/

ManagedGeometry& GetManagedGeometry(RobotWorld& world,int id)
{
  if(id < 0) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"GetManagedGeometry(): Invalid ID: "<<id);
    return world.robots[0]->geomManagers[0];
  }
  int terrain = world.IsTerrain(id);
  if(terrain >= 0)
    return world.terrains[terrain]->geometry;
  int rigidObject = world.IsRigidObject(id);
  if(rigidObject >= 0)
    return world.rigidObjects[rigidObject]->geometry;
  pair<int,int> robotLink = world.IsRobotLink(id);
  if(robotLink.first >= 0) {
    return world.robots[robotLink.first]->geomManagers[robotLink.second];
  }
    LOG4CXX_ERROR(KrisLibrary::logger(),"GetManagedGeometry(): Invalid ID: "<<id);
  return world.robots[0]->geomManagers[0];
}


class ManualOverrideController : public RobotController
{
 public:
  ManualOverrideController(Robot& robot,const SmartPointer<RobotController>& _base)
    :RobotController(robot),base(_base),override(false)
  {}
  virtual const char* Type() const { return "ManualOverrideController"; }
  virtual void Update(Real dt);

  virtual bool ReadState(File& f);
  virtual bool WriteState(File& f) const;

  //getters/setters
  virtual map<string,string> Settings() const { return base->Settings(); }
  virtual bool GetSetting(const string& name,string& str) const { return base->GetSetting(name,str); }
  virtual bool SetSetting(const string& name,const string& str) { return base->SetSetting(name,str); }

  virtual vector<string> Commands() const { return base->Commands(); }
  virtual bool SendCommand(const string& name,const string& str) { return base->SendCommand(name,str); }

  SmartPointer<RobotController> base;
  bool override;
};

bool ManualOverrideController::ReadState(File& f)
{
  if(!ReadFile(f,override)) return false;
  if(!override) return base->ReadState(f);
  return RobotController::ReadState(f);
}

bool ManualOverrideController::WriteState(File& f) const
{
  if(!WriteFile(f,override)) return false;
  if(!override) return base->WriteState(f);
  return RobotController::WriteState(f);
}

void ManualOverrideController::Update(Real dt)
{
  if(!override) {
    base->time = time;
    base->command = command;
    base->sensors = sensors;
    base->Update(dt);
    return;
  }
  else {
    //keep existing commands
  }
  RobotController::Update(dt);
}


typedef ManualOverrideController MyController;
inline MyController* MakeController(Robot* robot)
{
  ManualOverrideController* lc=new ManualOverrideController(*robot,MakeDefaultController(robot));
  return lc;
}
inline PolynomialPathController* GetPathController(RobotController* controller)
{
  MyController* mc=dynamic_cast<MyController*>(controller);
  if(!mc) {
    throw PyException("Not using the default manual override controller");
  }
  LoggingController* lc=dynamic_cast<LoggingController*>((RobotController*)mc->base);
  if(!lc) {
    throw PyException("Not using the default robot controller");
  }
  FeedforwardController* ffc=dynamic_cast<FeedforwardController*>((RobotController*)lc->base);
  PolynomialPathController* pc=dynamic_cast<PolynomialPathController*>((RobotController*)ffc->base);
  return pc;
}
inline PolynomialMotionQueue* GetMotionQueue(RobotController* controller)
{
  return GetPathController(controller);
}


void GetMesh(const Geometry::AnyCollisionGeometry3D& geom,TriangleMesh& tmesh)
{
  Assert(geom.type == Geometry::AnyGeometry3D::TriangleMesh);
  const Meshing::TriMesh& mesh = geom.AsTriangleMesh();
  tmesh.indices.resize(mesh.tris.size()*3);
  tmesh.vertices.resize(mesh.verts.size()*3);
  for(size_t i=0;i<mesh.tris.size();i++) {
    tmesh.indices[i*3] = mesh.tris[i].a;
    tmesh.indices[i*3+1] = mesh.tris[i].b;
    tmesh.indices[i*3+2] = mesh.tris[i].c;
  }
  for(size_t i=0;i<mesh.verts.size();i++) {
    mesh.verts[i].get(tmesh.vertices[i*3],tmesh.vertices[i*3+1],tmesh.vertices[i*3+2]);
  }
}

void GetMesh(const TriangleMesh& tmesh,Geometry::AnyCollisionGeometry3D& geom)
{
  Meshing::TriMesh mesh;
  mesh.tris.resize(tmesh.indices.size()/3);
  mesh.verts.resize(tmesh.vertices.size()/3);
  for(size_t i=0;i<mesh.tris.size();i++)
    mesh.tris[i].set(tmesh.indices[i*3],tmesh.indices[i*3+1],tmesh.indices[i*3+2]);
  for(size_t i=0;i<mesh.verts.size();i++)
    mesh.verts[i].set(tmesh.vertices[i*3],tmesh.vertices[i*3+1],tmesh.vertices[i*3+2]);
  geom = mesh;
  geom.ClearCollisionData();
}


void GetPointCloud(const Geometry::AnyCollisionGeometry3D& geom,PointCloud& pc)
{
  Assert(geom.type == Geometry::AnyGeometry3D::PointCloud);
  const Meshing::PointCloud3D& gpc = geom.AsPointCloud();
  pc.vertices.resize(gpc.points.size()*3);
  pc.propertyNames = gpc.propertyNames;
  pc.properties.resize(gpc.points.size()*gpc.propertyNames.size());
  for(size_t i=0;i<gpc.points.size();i++) {
    gpc.points[i].get(pc.vertices[i*3],pc.vertices[i*3+1],pc.vertices[i*3+2]);
    gpc.properties[i].getCopy(&pc.properties[i*gpc.propertyNames.size()]);
  }
  pc.settings = gpc.settings;
}

void GetPointCloud(const PointCloud& pc,Geometry::AnyCollisionGeometry3D& geom)
{
  Meshing::PointCloud3D gpc;
  gpc.points.resize(pc.vertices.size()/3);
  for(size_t i=0;i<gpc.points.size();i++)
    gpc.points[i].set(pc.vertices[i*3],pc.vertices[i*3+1],pc.vertices[i*3+2]);
  gpc.propertyNames = pc.propertyNames;
  if(pc.propertyNames.size() > 0) {
    gpc.properties.resize(pc.properties.size() / pc.propertyNames.size());
    for(size_t i=0;i<gpc.properties.size();i++) {
      gpc.properties[i].resize(pc.propertyNames.size());
      gpc.properties[i].copy(&pc.properties[i*pc.propertyNames.size()]);
    }
  }
  gpc.settings = pc.settings;
  //LOG4CXX_INFO(KrisLibrary::logger(),"Copying PointCloud to geometry, "<<(int)gpc.points.size());
  geom = gpc;
  geom.ClearCollisionData();
}

void GeometricPrimitive::setPoint(const double pt[3])
{
  type = "Point";
  properties.resize(3);
  copy(pt,pt+3,properties.begin());
}

void GeometricPrimitive::setSphere(const double c[3],double r)
{
  type = "Sphere";
  properties.resize(4);
  copy(c,c+3,properties.begin());
}

void GeometricPrimitive::setSegment(const double a[3],const double b[3])
{
  type = "Segment";
  properties.resize(6);
  copy(a,a+3,properties.begin());
  copy(b,b+3,properties.begin()+3);
}

void GeometricPrimitive::setAABB(const double bmin[3],const double bmax[3])
{
  type = "AABB";
  properties.resize(6);
  copy(bmin,bmin+3,properties.begin());
  copy(bmax,bmax+3,properties.begin()+3);
}

bool GeometricPrimitive::loadString(const char* str)
{
  vector<string> items = Split(str," \t\n");
  type = items[0];
  properties.resize(items.size()-1);
  for(size_t i=1;i<items.size();i++)
    if(!LexicalCast<double>(items[i],properties[i-1])) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"GeometricPrimitive::loadString: could not parse item "<<(int)i<<": \""<<items[i].c_str());
      return false;
    }
  return true;
}

std::string GeometricPrimitive::saveString() const
{
  stringstream ss;
  ss<<type<<" ";
  for(size_t i=0;i<properties.size();i++)
    ss<<properties[i]<<" ";
  return ss.str();
}


Geometry3D::Geometry3D()
  :world(-1),id(-1),geomPtr(NULL)
{
  geomPtr = new SmartPointer<AnyCollisionGeometry3D>();
}

Geometry3D::Geometry3D(const Geometry3D& rhs)
  :world(rhs.world),id(rhs.id),geomPtr(NULL)
{
  SmartPointer<AnyCollisionGeometry3D>* geom = reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(rhs.geomPtr);
  if(*geom != NULL)
    geomPtr = new SmartPointer<AnyCollisionGeometry3D>(*geom);
  else
    geomPtr = new SmartPointer<AnyCollisionGeometry3D>();
}

Geometry3D::Geometry3D(const GeometricPrimitive& rhs)
  :world(-1),id(-1),geomPtr(NULL)
{
  geomPtr = new SmartPointer<AnyCollisionGeometry3D>();
  setGeometricPrimitive(rhs);
}

Geometry3D::Geometry3D(const TriangleMesh& rhs)
  :world(-1),id(-1),geomPtr(NULL)
{
  geomPtr = new SmartPointer<AnyCollisionGeometry3D>();
  setTriangleMesh(rhs);
}

Geometry3D::Geometry3D(const PointCloud& rhs)
  :world(-1),id(-1),geomPtr(NULL)
{
  geomPtr = new SmartPointer<AnyCollisionGeometry3D>();
  setPointCloud(rhs);
}

Geometry3D::~Geometry3D()
{
  free();
  SmartPointer<AnyCollisionGeometry3D>* geom = reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  delete geom;
}

const Geometry3D& Geometry3D::operator = (const Geometry3D& rhs)
{
  free();
  world = rhs.world;
  id = rhs.id;
  SmartPointer<AnyCollisionGeometry3D>* geom = reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  SmartPointer<AnyCollisionGeometry3D>* geom2 = reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(rhs.geomPtr);
  *geom = *geom2;
  return *this;
} 

bool Geometry3D::isStandalone()
{
  return (world < 0);
}

Geometry3D Geometry3D::clone()
{
  Geometry3D res;
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  SmartPointer<AnyCollisionGeometry3D>& resgeom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(res.geomPtr);
  if(geom != NULL) {
    resgeom = new AnyCollisionGeometry3D(*geom);
  }
  return res;
}

void Geometry3D::set(const Geometry3D& g)
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  const SmartPointer<AnyCollisionGeometry3D>& ggeom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(g.geomPtr);
  ManagedGeometry* mgeom = NULL;
  if(!isStandalone()) {
    RobotWorld& world = *worlds[this->world]->world;
    mgeom = &GetManagedGeometry(world,id);
  }
  if(geom == NULL) {
    if(mgeom) {
      geom = mgeom->CreateEmpty();
    }
    else
      geom = new AnyCollisionGeometry3D();
  }
  else
    Assert(&*geom == &*mgeom);
  *geom = *ggeom;
  //geom->ClearCollisionData();
  if(mgeom) {
    //update the display list / cache
    mgeom->OnGeometryChange();
    mgeom->RemoveFromCache();
  }
}

void Geometry3D::free()
{
  SmartPointer<AnyCollisionGeometry3D>* geom = reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);  
  if(isStandalone() && *geom) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Geometry3D(): Freeing standalone geometry\n");
    *geom = NULL;
  }
  world = -1;
  id = -1;

  delete geom;
  geomPtr = new SmartPointer<AnyCollisionGeometry3D>;
}

string Geometry3D::type()
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) return "";
  if(geom->Empty()) return "";
  return geom->TypeName();
}

bool Geometry3D::empty()
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);  
  if(!geom) return true;
  if(geom->Empty()) return true;
  return false;
}

TriangleMesh Geometry3D::getTriangleMesh()
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  TriangleMesh mesh;
  if(geom) {
    GetMesh(*geom,mesh);
  }
  return mesh;
}


GeometricPrimitive Geometry3D::getGeometricPrimitive()
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) return GeometricPrimitive();
  stringstream ss;
  ss<<geom->AsPrimitive();
  GeometricPrimitive prim;
  bool res=prim.loadString(ss.str().c_str());
  if(!res) {
    throw PyException("Internal error, geometric primitive conversion");
  }
  return prim;
}


void Geometry3D::setTriangleMesh(const TriangleMesh& mesh)
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  ManagedGeometry* mgeom = NULL;
  if(!isStandalone()) {
    RobotWorld& world = *worlds[this->world]->world;
    mgeom = &GetManagedGeometry(world,id);
  }
  if(geom == NULL) {
    if(mgeom) 
      geom = mgeom->CreateEmpty();
    else
      geom = new AnyCollisionGeometry3D();
  }
  GetMesh(mesh,*geom);
  if(mgeom) {
    //update the display list / cache
    mgeom->OnGeometryChange();
    mgeom->RemoveFromCache();
  }
}

void Geometry3D::setGroup()
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  ManagedGeometry* mgeom = NULL;
  if(!isStandalone()) {
    RobotWorld& world = *worlds[this->world]->world;
    mgeom = &GetManagedGeometry(world,id);
  }
  if(geom == NULL) {
    if(mgeom) 
      geom = mgeom->CreateEmpty();
    else
      geom = new AnyCollisionGeometry3D();
  }
  *geom = AnyCollisionGeometry3D(vector<Geometry::AnyGeometry3D>());
  geom->ReinitCollisionData();
}

Geometry3D Geometry3D::getElement(int element)
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) 
    throw PyException("Geometry is empty");
  if(geom->type != AnyCollisionGeometry3D::Group)
    throw PyException("Not a group geometry");
  vector<AnyCollisionGeometry3D>& data = geom->GroupCollisionData();
  if(element < 0 || element >= (int)data.size())
    throw PyException("Invalid element specified");
  Geometry3D res;
  SmartPointer<AnyCollisionGeometry3D>& rgeom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(res.geomPtr);
  *rgeom = data[element];
  return res;
}

void Geometry3D::setElement(int element,const Geometry3D& rhs)
{
  SmartPointer<AnyCollisionGeometry3D>& rgeom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(rhs.geomPtr);
  if(rgeom == NULL) 
    throw PyException("Setting an element to an empty geometry?");
  rgeom->InitCollisionData();

  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) 
    throw PyException("Geometry is empty");
  if(geom->type != AnyCollisionGeometry3D::Group)
    throw PyException("Not a group geometry");
  vector<AnyGeometry3D>& data = geom->AsGroup();
  if(element < 0 || element > (int)data.size())
    throw PyException("Invalid element specified");
  Assert(rhs.geomPtr != NULL);
  vector<AnyCollisionGeometry3D>& cdata = geom->GroupCollisionData();
  
  if(element == (int)data.size()) {
    data.push_back(*rgeom);
    cdata.push_back(*rgeom);
  }
  else {
    data[element] = *rgeom;
    cdata[element] = *rgeom;
  }
}

int Geometry3D::numElements()
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) 
    throw PyException("Geometry is empty");
  if(geom->type != AnyCollisionGeometry3D::Group)
    throw PyException("Not a group geometry");
  vector<AnyGeometry3D>& data = geom->AsGroup();
  return (int)data.size();
}


PointCloud Geometry3D::getPointCloud()
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  PointCloud pc;
  if(geom) {
    GetPointCloud(*geom,pc);
  }
  return pc;
}

void Geometry3D::setPointCloud(const PointCloud& pc)
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  ManagedGeometry* mgeom = NULL;
  if(!isStandalone()) {
    RobotWorld& world = *worlds[this->world]->world;
    mgeom = &GetManagedGeometry(world,id);
  }
  if(geom == NULL) {
    if(mgeom) {
      geom = mgeom->CreateEmpty();
    }
    else
      geom = new AnyCollisionGeometry3D();
  }
  GetPointCloud(pc,*geom);
  //this is already called
  //geom->ClearCollisionData();
  if(mgeom) {
    //update the display list / cache
    mgeom->OnGeometryChange();
    mgeom->RemoveFromCache();
  }
}

void Geometry3D::setGeometricPrimitive(const GeometricPrimitive& prim)
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);  
  ManagedGeometry* mgeom = NULL;
  if(!isStandalone()) {
    RobotWorld& world = *worlds[this->world]->world;
    mgeom = &GetManagedGeometry(world,id);
  }
  if(geom == NULL) {
    if(mgeom) {
      geom = mgeom->CreateEmpty();
    }
    else
      geom = new AnyCollisionGeometry3D();
  }
  stringstream ss(prim.saveString());
  GeometricPrimitive3D g;
  ss>>g;
  if(!ss) {
    throw PyException("Internal error, can't read geometric primitive?");
  }
  *geom = g;
  geom->ClearCollisionData();
  if(mgeom) {
    //update the display list / cache
    mgeom->OnGeometryChange();
    mgeom->RemoveFromCache();
  }
}


bool Geometry3D::loadFile(const char* fn)
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  if(isStandalone()) {
    if(!geom) {
      geom = new AnyCollisionGeometry3D();
    }
    if(!geom->Load(fn)) return false;
    return true;
  }
  else {
    assert(id >= 0);
    //use the manager, this will automatically figure out caching and
    //appearance stuff
    RobotWorld& world = *worlds[this->world]->world;
    ManagedGeometry* mgeom = NULL;
    mgeom = &GetManagedGeometry(world,id);
    if(mgeom->Load(fn)) {
      geom = SmartPointer<Geometry::AnyCollisionGeometry3D>(*mgeom);
      return true;
    }
    return false;
  }
}

bool Geometry3D::saveFile(const char* fn)
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) return false;
  return geom->Save(fn);
}


void Geometry3D::setCurrentTransform(const double R[9],const double t[3])
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) return;
  RigidTransform T;
  T.R.set(R);
  T.t.set(t);
  geom->SetTransform(T);
}

void Geometry3D::getCurrentTransform(double out[9],double out2[3])
{
  RigidTransform T;
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) {
    T.setIdentity();
    return;
  }
  else
    T = geom->GetTransform();
  T.R.get(out);
  T.t.get(out2);
}

void Geometry3D::scale(double s)
{
  scale(s,s,s);
}

void Geometry3D::scale(double sx,double sy,double sz)
{
  Matrix3 R;
  R.setZero();
  R(0,0) = sx;
  R(1,1) = sy;
  R(2,2) = sz;
  const double t[3]={0,0,0};
  transform(R,t);
}

void Geometry3D::rotate(const double R[9])
{
  const double t[3]={0,0,0};
  transform(R,t);
}

void Geometry3D::translate(const double t[3])
{
  Matrix3 R;
  R.setIdentity();
  transform(R,t);
}

void Geometry3D::transform(const double R[9],const double t[3])
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  RigidTransform T;
  T.R.set(R);
  T.t.set(t);
  geom->Transform(T);
  geom->ClearCollisionData();

  if(!isStandalone()) {
    //update the display list / cache
    RobotWorld& world=*worlds[this->world]->world;
    ManagedGeometry* mgeom = &GetManagedGeometry(world,id);
    mgeom->OnGeometryChange();
    mgeom->RemoveFromCache();
  }
}

void Geometry3D::setCollisionMargin(double margin)
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) return;
  geom->margin = margin;
}

double Geometry3D::getCollisionMargin()
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) return 0;
  return geom->margin;
}

void Geometry3D::getBB(double out[3],double out2[3])
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) {
    out[0] = out[1] = out[2] = Inf;
    out2[0] = out2[1] = out2[2] = -Inf;
    return;
  }
  AABB3D bb = geom->GetAABB();
  bb.bmin.get(out);
  bb.bmax.get(out2);
}

bool Geometry3D::collides(const Geometry3D& other)
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  SmartPointer<AnyCollisionGeometry3D>& geom2 = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(other.geomPtr);
  if(!geom || !geom2) return false;
  return geom->Collides(*geom2);
}

bool Geometry3D::withinDistance(const Geometry3D& other,double tol)
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  SmartPointer<AnyCollisionGeometry3D>& geom2 = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(other.geomPtr);
  if(!geom || !geom2) return false;
  return geom->WithinDistance(*geom2,tol);
}

double Geometry3D::distance(const Geometry3D& other,double relErr,double absErr)
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  SmartPointer<AnyCollisionGeometry3D>& geom2 = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(other.geomPtr);
  if(!geom || !geom2) return 0;
  AnyCollisionQuery q(*geom,*geom2);
  return q.Distance(relErr,absErr);
}

bool Geometry3D::closestPoint(const double pt[3],double out[3])
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) return false;
  Vector3 vout;
  Real d = geom->Distance(Vector3(pt),vout);
  if(IsInf(d)) return false;
  vout.get(out);
  return true;
}

bool Geometry3D::rayCast(const double s[3],const double d[3],double out[3])
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) return false;
  Ray3D r;
  r.source.set(s);
  r.direction.set(d);
  Real distance;
  if(geom->RayCast(r,&distance)) {
    Vector3 pt = r.source + r.direction*distance;
    pt.get(out);
    return true;
  }
  return false;
}

//KH: note: pointer gymnastics necessary to allow appearances to refer to temporary appearances as well as references to world, while also
//exposing an opaque pointer in appearance.h
Appearance::Appearance()
  :world(-1),id(-1),appearancePtr(NULL)
{
  appearancePtr = new SmartPointer<GLDraw::GeometryAppearance>;
}

Appearance::Appearance(const Appearance& rhs)
  :world(rhs.world),id(rhs.id),appearancePtr(NULL)
{
  SmartPointer<GLDraw::GeometryAppearance>* geom = reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(rhs.appearancePtr);
  appearancePtr = new SmartPointer<GLDraw::GeometryAppearance>(*geom);
}

Appearance::~Appearance()
{
  free();
  SmartPointer<GLDraw::GeometryAppearance>* app = reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(appearancePtr);
  delete app;
}

const Appearance& Appearance::operator = (const Appearance& rhs)
{
  free();
  world = rhs.world;
  id = rhs.id;
  SmartPointer<GLDraw::GeometryAppearance>* geom = reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(appearancePtr);
  SmartPointer<GLDraw::GeometryAppearance>* geom2 = reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(rhs.appearancePtr);
  *geom = *geom2;
  return *this;
} 


void Appearance::refresh(bool deep)
{
  SmartPointer<GLDraw::GeometryAppearance>& app = *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return;
  if(!isStandalone()) {
    ManagedGeometry& geom = GetManagedGeometry(*worlds[this->world]->world,id);
    if(geom.IsDynamicGeometry()) {
      if(geom.DynamicGeometryUpdate()) return;
      return;
    }
  }
  if(deep && app->geom != NULL) 
    app->Set(*app->geom);
  else
    app->Refresh();
}

bool Appearance::isStandalone()
{
  return (world < 0);
}

Appearance Appearance::clone()
{
  Appearance res;
  SmartPointer<GLDraw::GeometryAppearance>& app = *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(appearancePtr);
  SmartPointer<GLDraw::GeometryAppearance>& resapp = *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(res.appearancePtr);
  if(app != NULL) {
    resapp = new GLDraw::GeometryAppearance(*app);
  }
  return res;
}

void Appearance::set(const Appearance& g)
{
  SmartPointer<GLDraw::GeometryAppearance>& app = *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(appearancePtr);
  SmartPointer<GLDraw::GeometryAppearance>& gapp = *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(g.appearancePtr);

  if(!isStandalone()) {
    //need to detach from other geometries that might be sharing this appearance
    RobotWorld& world=*worlds[this->world]->world;
    GetManagedGeometry(world,id).SetUniqueAppearance();
    app = GetManagedGeometry(world,id).Appearance();
  }
  if(app == NULL) {
    app = new GLDraw::GeometryAppearance(*gapp);
  }
  else {
    app->CopyMaterial(*gapp);
  }
}

void Appearance::free()
{
  SmartPointer<GLDraw::GeometryAppearance>* app = reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(appearancePtr);

  if(isStandalone() && *app) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Appearance(): Freeing standalone appearance for "<<this);
    *app = NULL;
  }
  else if(*app)
    //LOG4CXX_INFO(KrisLibrary::logger(),"Appearance(): Releasing reference to world appearance "<<world<<" "<<id<<" for "<<this);
    ;
    
  world = -1;
  id = -1;
  *app = NULL;
}

void Appearance::setDraw(bool draw)
{
  SmartPointer<GLDraw::GeometryAppearance>& app = *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return;
  if(!isStandalone()) {
    RobotWorld& world=*worlds[this->world]->world;
    GetManagedGeometry(world,id).SetUniqueAppearance();
    app = GetManagedGeometry(world,id).Appearance();
  }
  if(draw) {
    app->drawFaces = true;
    app->drawVertices = false;
    app->drawEdges = false;
  }
  else {
    app->drawFaces = false;
    app->drawVertices = false;
    app->drawEdges = false;
  }
}
void Appearance::setDraw(int primitive,bool draw)
{
  SmartPointer<GLDraw::GeometryAppearance>& app = *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return;
  if(!isStandalone()) {
    RobotWorld& world=*worlds[this->world]->world;
    GetManagedGeometry(world,id).SetUniqueAppearance();
    app = GetManagedGeometry(world,id).Appearance();
  }
  switch(primitive) {
  case ALL: app->drawFaces = app->drawVertices = app->drawEdges = draw; break;
  case VERTICES: app->drawVertices = draw; break;
  case EDGES: app->drawEdges = draw; break;
  case FACES: app->drawFaces = draw; break;
  }
}

bool Appearance::getDraw()
{
  SmartPointer<GLDraw::GeometryAppearance>& app = *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return false;
  return app->drawFaces || app->drawVertices || app->drawEdges;
}

bool Appearance::getDraw(int primitive)
{
  SmartPointer<GLDraw::GeometryAppearance>& app = *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return false;
  switch(primitive) {
  case ALL: return app->drawFaces || app->drawVertices || app->drawEdges;
  case VERTICES: return app->drawVertices;
  case EDGES: return app->drawEdges;
  case FACES: return app->drawFaces;
  }
  return false;
}

void Appearance::setColor(float r,float g,float b,float a)
{
  SmartPointer<GLDraw::GeometryAppearance>& app = *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return;
  if(!isStandalone()) {
    RobotWorld& world=*worlds[this->world]->world;
    GetManagedGeometry(world,id).SetUniqueAppearance();
    app = GetManagedGeometry(world,id).Appearance();
  }
  app->SetColor(r,g,b,a);
}

void Appearance::setColor(int primitive,float r,float g,float b,float a)
{
  SmartPointer<GLDraw::GeometryAppearance>& app = *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return;
  if(!isStandalone()) {
    RobotWorld& world=*worlds[this->world]->world;
    GetManagedGeometry(world,id).SetUniqueAppearance();
    app = GetManagedGeometry(world,id).Appearance();
  }
  switch(primitive) {
  case ALL:
    app->SetColor(r,g,b,a);
    break;
  case VERTICES:
    app->vertexColor.set(r,g,b,a);
    if(!app->vertexColors.empty()) {
      app->vertexColors.clear();
      app->Refresh();
    }
    break;
  case EDGES:
    app->edgeColor.set(r,g,b,a); 
    break;
  case FACES:
    app->faceColor.set(r,g,b,a);
    if(!app->faceColors.empty()) {
      app->faceColors.clear();
      app->Refresh();
    }
    break;
  }
}

void Appearance::getColor(float out[4])
{
  SmartPointer<GLDraw::GeometryAppearance>& app = *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) throw PyException("Invalid appearance");
  for(int i=0;i<4;i++) out[i] = app->faceColor.rgba[i];
}
void Appearance::getColor(int primitive,float out[4])
{
  SmartPointer<GLDraw::GeometryAppearance>& app = *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) throw PyException("Invalid appearance");
  GLDraw::GLColor c;
  switch(primitive) {
  case ALL:
  case FACES:
    c = app->faceColor;
    break;
  case VERTICES:
    c = app->vertexColor;
    break;
  case EDGES:
    c = app->edgeColor;
    break;
  default:
    throw PyException("Invalid primitive");
  }
  for(int i=0;i<4;i++) out[i] = c.rgba[i];
}
void Appearance::setColors(int primitive,const std::vector<float>& colors,bool alpha)
{
  FatalError("Not implemented yet");
}
void Appearance::setTexture1D(int w,const char* format,const std::vector<unsigned char>& bytes)
{
  FatalError("Not implemented yet");
}

void Appearance::setTexture2D(int w,int h,const char* format,const std::vector<unsigned char>& bytes)
{
  FatalError("Not implemented yet");
}

void Appearance::setTexcoords(const std::vector<double>& uvs)
{
  FatalError("Not implemented yet");
}

void Appearance::setPointSize(float size)
{
  SmartPointer<GLDraw::GeometryAppearance>& app = *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return;
  if(!isStandalone()) {
    RobotWorld& world=*worlds[this->world]->world;
    GetManagedGeometry(world,id).SetUniqueAppearance();
    app = GetManagedGeometry(world,id).Appearance();
  }
  app->vertexSize = size;
}

void Appearance::drawGL()
{
  SmartPointer<GLDraw::GeometryAppearance>& app = *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return;
  if(!app->geom) return;
  app->DrawGL();
}

void Appearance::drawWorldGL(Geometry3D& g)
{
  SmartPointer<GLDraw::GeometryAppearance>& app = *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(appearancePtr);
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(g.geomPtr);
  if(!geom) return;
  if(!app) {
    app = new GLDraw::GeometryAppearance();
  }
  if(app->geom) {
    if(app->geom != geom) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Appearance::drawGL(): performance warning, setting to a different geometry\n");
      app->Set(*geom);
    }
  }
  else {
    app->Set(*geom);
  }

  glPushMatrix();
  GLDraw::glMultMatrix(Matrix4(geom->GetTransform()));
  app->DrawGL();
  glPopMatrix();
}

void Appearance::drawGL(Geometry3D& g)
{
  SmartPointer<GLDraw::GeometryAppearance>& app = *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(appearancePtr);
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(g.geomPtr);
  if(!geom) return;
  if(!app) {
    app = new GLDraw::GeometryAppearance();
  }
  if(app->geom) {
    if(app->geom != geom) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Appearance::drawGL(): performance warning, setting to a different geometry\n");
      app->Set(*geom);
    }
  }
  else {
    app->Set(*geom);
  }

  app->DrawGL();
}




void copy(const Vector& vec,vector<double>& v)
{
  v.resize(vec.n);
  for(int i=0;i<vec.n;i++)
    v[i]=vec(i);
}

void copy(const vector<double>& vec,Vector& v)
{
  v.resize(vec.size());
  for(size_t i=0;i<vec.size();i++)
    v(i)=vec[i];
}


void copy(const Matrix& mat,vector<double>& v)
{
  v.resize(mat.m*mat.n);
  int k=0;
  for(int i=0;i<mat.m;i++)
    for(int j=0;j<mat.m;j++)
      v[k++] = mat(i,j);
}

void copy(const Matrix& mat,vector<vector<double> >& v)
{
  v.resize(mat.m);
  for(int i=0;i<mat.m;i++) {
    v[i].resize(mat.n);
    for(int j=0;j<mat.n;j++)
      v[i][j] = mat(i,j);
  }
}

void TriangleMesh::translate(const double t[3])
{
  for(size_t i=0;i<vertices.size();i+=3) {
    vertices[i] += t[0];
    vertices[i+1] += t[1];
    vertices[i+2] += t[2];
  }
}

void TriangleMesh::transform(const double R[9],const double t[3])
{
  RigidTransform T;
  T.R.set(R);
  T.t.set(t);
  for(size_t i=0;i<vertices.size();i+=3) {
    Vector3 v(vertices[i],vertices[i+1],vertices[i+2]);
    v = T*v;
    v.get(vertices[i],vertices[i+1],vertices[i+2]);
  }
}

int PointCloud::numPoints() const { return vertices.size()/3; }
int PointCloud::numProperties() const { return propertyNames.size(); }
void PointCloud::setPoints(int num,const vector<double>& plist)
{
  vertices.resize(num*3);
  assert(plist.size() >= num+3);
  copy(plist.begin(),plist.begin()+num*3,&vertices[0]);
  properties.resize(num*propertyNames.size());
  fill(properties.begin(),properties.end(),0.0);
}

int PointCloud::addPoint(const double p[3])
{
  int ofs = (int)vertices.size();

  vertices.push_back(p[0]);
  vertices.push_back(p[1]);
  vertices.push_back(p[2]);
  properties.resize(properties.size()+propertyNames.size(),0.0);
  return ofs/3;
}

void PointCloud::setPoint(int index,const double p[3])
{
  if(index < 0 || index*3 >= (int)vertices.size())
    throw PyException("Invalid point index");  
  vertices[index*3] = p[0];
  vertices[index*3+1] = p[1];
  vertices[index*3+2] = p[2];
}

void PointCloud::getPoint(int index,double out[3]) const
{
  if(index < 0 || index*3 >= (int)vertices.size())
    throw PyException("Invalid point index");  
  out[0] = vertices[index*3];
  out[1] = vertices[index*3+1];
  out[2] = vertices[index*3+2];
}

void PointCloud::addProperty(const std::string& pname)
{
  int n = numPoints();
  vector<double> values(n,0.0);
  addProperty(pname,values);
}

void PointCloud::addProperty(const std::string& pname,const std::vector<double> & values)
{
  int n = numPoints();
  if(values.size() != n) {
    throw PyException("Invalid size of properties list, must have size #points");
  }
  assert(values.size() == n);
  size_t m=propertyNames.size();
  assert(properties.size() == n*m);
  propertyNames.push_back(pname);
  vector<double> newprops(n*(m+1));
  for(int i=0;i<n;i++) {
    assert ((i+1)*m < (int)properties.size()); 
    assert (i*(m+1) + m < (int)newprops.size()); 
    std::copy(properties.begin()+i*m,properties.begin()+(i+1)*m,newprops.begin()+i*(m+1));
    newprops[i*(m+1) + m] = values[i];
  }
  std::swap(newprops,properties);
  assert(properties.size() == (n*(m+1)));
}

void PointCloud::setProperties(const vector<double>& vproperties)
{
  int n = numPoints();
  size_t m=propertyNames.size();
  if(vproperties.size() < n*m) {
    throw PyException("Invalid size of properties list, must have size at least #points * #properties");
  }
  assert(properties.size() == n*m);
  copy(vproperties.begin(),vproperties.begin()+m*n,properties.begin());
}

void PointCloud::setProperties(int pindex,const vector<double>& vproperties)
{
  if(pindex < 0 || pindex >= (int)propertyNames.size())
    throw PyException("Invalid property index"); 
  int n = numPoints();
  assert((int)vproperties.size() >= n);
  for(int i=0;i<n;i++)
    properties[i*propertyNames.size()+pindex] = vproperties[i];
}

void PointCloud::setProperty(int index,int pindex,double value)
{
  if(index < 0 || index*3 >= (int)vertices.size())
    throw PyException("Invalid point index");  
  if(pindex < 0 || pindex >= (int)propertyNames.size())
    throw PyException("Invalid property index");  
  properties[index*propertyNames.size()+pindex] = value;
}

void PointCloud::setProperty(int index,const std::string& pname,double value)
{
  int pindex = -1;
  for(size_t i=0;i<propertyNames.size();i++)
    if(propertyNames[i] == pname) {
      pindex = (int)i;
      break;
    }
  if(pindex < 0)
    throw PyException("Invalid property name");  
  setProperty(index,pindex,value);
}

double PointCloud::getProperty(int index,int pindex) const
{
  if(index < 0 || index*3 >= (int)vertices.size())
    throw PyException("Invalid point index");  
  if(pindex < 0 || pindex >= (int)propertyNames.size())
    throw PyException("Invalid property index");  
  return properties[index*propertyNames.size()+pindex];
}

double PointCloud::getProperty(int index,const std::string& pname) const
{
  int pindex = -1;
  for(size_t i=0;i<propertyNames.size();i++)
    if(propertyNames[i] == pname) {
      pindex = (int)i;
      break;
    }
  if(pindex < 0)
    throw PyException("Invalid property name");  
  return getProperty(index,pindex);
}

void PointCloud::join(const PointCloud& pc)
{
  if(propertyNames != pc.propertyNames) 
    throw PyException("PointCloud::join can't join two PCs with dissimilar property names");
  vertices.insert(vertices.end(),pc.vertices.begin(),pc.vertices.end());
  properties.insert(properties.end(),pc.properties.begin(),pc.properties.end());
}

void PointCloud::setSetting(const std::string& key,const std::string& value)
{
  settings[key] = value;
}

std::string PointCloud::getSetting(const std::string& key) const
{
  if(settings.count(key) == 0)
    throw PyException("PointCloud::getSetting(): key does not exist in settings map");
  return settings.find(key)->second;
}

void PointCloud::translate(const double t[3])
{
  for(size_t i=0;i<vertices.size();i+=3) {
    vertices[i] += t[0];
    vertices[i+1] += t[1];
    vertices[i+2] += t[2];
  }
}

void PointCloud::transform(const double R[9],const double t[3])
{
  RigidTransform T;
  T.R.set(R);
  T.t.set(t);
  for(size_t i=0;i<vertices.size();i+=3) {
    Vector3 v(vertices[i],vertices[i+1],vertices[i+2]);
    v = T*v;
    v.get(vertices[i],vertices[i+1],vertices[i+2]);
  }
}
















/***************************  ROBOT / WORLD MODELING CODE ***************************************/

WorldModel::WorldModel()
{
  index = createWorld();
}

/*
WorldModel::WorldModel(int _index)
{
  index = _index;
  refWorld(index);
}
*/

WorldModel::WorldModel(const WorldModel& w)
{
  index = w.index;
  refWorld(index);
}

WorldModel::WorldModel(void* ptrRobotWorld)
{
  index = createWorld((RobotWorld*)ptrRobotWorld);
}


const WorldModel& WorldModel::operator = (const WorldModel& w)
{
  if(index >= 0) 
    derefWorld(index);
  index = w.index;
  refWorld(index);
  return *this;
}

WorldModel WorldModel::copy()
{
  WorldModel res;
  RobotWorld& myworld = *worlds[index]->world;
  RobotWorld& otherworld = *worlds[res.index]->world;
  otherworld = myworld;
  //world occupants -- copy everything but geometry
  for(size_t i=0;i<otherworld.robots.size();i++) {
    otherworld.robots[i] = new Robot;
    *otherworld.robots[i] = *myworld.robots[i];
    otherworld.robotViews[i].robot = otherworld.robots[i];
  }
  for(size_t i=0;i<otherworld.terrains.size();i++) {
    otherworld.terrains[i] = new Terrain;
    *otherworld.terrains[i] = *myworld.terrains[i];
  }
  for(size_t i=0;i<otherworld.rigidObjects.size();i++) {
    otherworld.rigidObjects[i] = new RigidObject;
    *otherworld.rigidObjects[i] = *myworld.rigidObjects[i];
  }
  return res;
}

WorldModel::~WorldModel()
{
  if(index >= 0) {
    derefWorld(index);
    index = -1;
  }
}

bool WorldModel::loadFile(const char* fn)
{
  return readFile(fn);
}

bool WorldModel::saveFile(const char* fn,const char* elementPath)
{
  RobotWorld& world = *worlds[index]->world;
  return world.SaveXML(fn,elementPath);
}

bool WorldModel::readFile(const char* fn)
{
  RobotWorld& world = *worlds[index]->world;

  const char* ext=FileExtension(fn);
  if(0==strcmp(ext,"rob") || 0==strcmp(ext,"urdf")) {
    if(world.LoadRobot(fn)<0) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Error loading robot file "<<fn);
      return false;
    }
    if(gEnableCollisionInitialization) 
      world.robots.back()->InitCollisions();
    world.robots.back()->UpdateGeometry();
  }
  else if(0==strcmp(ext,"env") || 0==strcmp(ext,"tri") || 0==strcmp(ext,"pcd")) {
    if(world.LoadTerrain(fn)<0) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Error loading terrain file "<<fn);
      return false;
    }
    if(gEnableCollisionInitialization) world.terrains.back()->InitCollisions();
  }
  else if(0==strcmp(ext,"obj")) {
    if(world.LoadRigidObject(fn)<0) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Error loading rigid object file "<<fn);
      return false;
    }
    if(gEnableCollisionInitialization) 
      world.rigidObjects.back()->InitCollisions();
    world.rigidObjects.back()->UpdateGeometry();
  }
  else if(0==strcmp(ext,"xml")) {
    /*
    char* path = new char[strlen(fn)+1];
    GetFilePath(fn,path);
    char* oldwd = new char[1024];
    char* res=getcwd(oldwd,1024);
    Assert(res != NULL);
    if(strlen(path) != 0) {
      chdir(path);
    }
    */
    bool result = false;
    //if(worlds[index]->xmlWorld.Load(GetFileName(fn))) {
    if(worlds[index]->xmlWorld.Load(fn)) {
      if(worlds[index]->xmlWorld.GetWorld(world)) {
    result = true;
      }
    }
    /*
    chdir(oldwd);
    delete [] oldwd;
    delete [] path;
    */
    if(!result) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Error opening or parsing world file "<<fn);
      return false;
    }
    if(gEnableCollisionInitialization) 
      world.InitCollisions();
    world.UpdateGeometry();
    return true;
  }
  else {
    LOG4CXX_INFO(KrisLibrary::logger(),"Unknown file extension "<<ext<<" on file "<<fn);
    return false;
  }
  return true;
}

int WorldModel::numRobots()
{
  RobotWorld& world = *worlds[index]->world;
  return world.robots.size();
}

int WorldModel::numRobotLinks(int robot)
{
  RobotWorld& world = *worlds[index]->world;
  return world.robots[robot]->links.size();
}

int WorldModel::numRigidObjects()
{
  RobotWorld& world = *worlds[index]->world;
  return world.rigidObjects.size();
}

int WorldModel::numTerrains()
{
  RobotWorld& world = *worlds[index]->world;
  return world.terrains.size();
}

int WorldModel::numIDs()
{
  RobotWorld& world = *worlds[index]->world;
  return world.NumIDs();
}

RobotModel WorldModel::robot(int robot)
{
  if(robot < 0  || robot >= (int)worlds[index]->world->robots.size())
    throw PyException("Invalid robot index");
  RobotModel r;
  r.world = index;
  r.index = robot;
  r.robot = worlds[index]->world->robots[robot];
  return r;
}

RobotModel WorldModel::robot(const char* robot)
{
  RobotWorld& world = *worlds[index]->world;
  RobotModel r;
  r.world = index;
  for(size_t i=0;i<world.robots.size();i++)
    if(world.robots[i]->name == robot) {
      r.index = (int)i;
      r.robot = world.robots[i];
      return r;
    }
  throw PyException("Invalid robot name");
  return r;
}

RobotModelLink WorldModel::robotLink(int robot,int link)
{
  if(robot < 0  || robot >= (int)worlds[index]->world->robots.size())
    throw PyException("Invalid robot index");
  RobotModelLink r;
  r.world = index;
  r.robotIndex = robot;
  r.robotPtr = worlds[index]->world->robots[robot];
  r.index = link;
  return r;
}

RobotModelLink WorldModel::robotLink(const char* robotname,const char* link)
{
  RobotModelLink r;
  RobotModel rob=robot(robotname);
  r.world = index;
  r.robotIndex = rob.index;
  r.robotPtr = rob.robot;
  if(rob.index < 0) 
    return r;
  r.index = -1;
  for(size_t i=0;i<rob.robot->links.size();i++)
    if(rob.robot->linkNames[i] == link) {
      r.index = (int)i;
      return r;
    }
  throw PyException("Invalid link name");
  return r;
}

RigidObjectModel WorldModel::rigidObject(int object)
{
  if(object < 0  || object >= (int)worlds[index]->world->rigidObjects.size())
    throw PyException("Invalid rigid object index");
  RigidObjectModel obj;
  obj.world = index;
  obj.index = object;
  obj.object = worlds[index]->world->rigidObjects[object];
  return obj;
}

RigidObjectModel WorldModel::rigidObject(const char* object)
{
  RobotWorld& world = *worlds[index]->world;
  RigidObjectModel obj;
  obj.world = index;
  for(size_t i=0;i<world.rigidObjects.size();i++)
    if(world.rigidObjects[i]->name == object) {
      obj.index = (int)i;
      obj.object = world.rigidObjects[i];
      return obj;
    }
  throw PyException("Invalid rigid object name");
  return obj;
}

TerrainModel WorldModel::terrain(int terrain)
{
  if(terrain < 0  || terrain >= (int)worlds[index]->world->terrains.size())
    throw PyException("Invalid terrain index");

  TerrainModel t;
  t.world = index;
  t.index = terrain;
  t.terrain = worlds[index]->world->terrains[terrain];
  return t;
}

TerrainModel WorldModel::terrain(const char* terrain)
{
  TerrainModel t;
  t.world = index;
  RobotWorld& world = *worlds[index]->world;
  for(size_t i=0;i<world.terrains.size();i++)
    if(world.terrains[i]->name == terrain) {
      t.index = (int)i;
      t.terrain = world.terrains[i];
      return t;
    }
  return t;
}

RobotModel WorldModel::makeRobot(const char* name)
{
  RobotWorld& world = *worlds[index]->world;
  RobotModel robot;
  robot.world = index;
  robot.index = (int)world.robots.size();
  world.AddRobot(name,new Robot());
  robot.robot = world.robots.back();
  return robot;
}

RigidObjectModel WorldModel::makeRigidObject(const char* name)
{
  RobotWorld& world = *worlds[index]->world;
  RigidObjectModel object;
  object.world = index;
  object.index = (int)world.rigidObjects.size();
  world.AddRigidObject(name,new RigidObject());
  object.object = world.rigidObjects.back();
  object.object->geometry.CreateEmpty();
  return object;
}

TerrainModel WorldModel::makeTerrain(const char* name)
{
  RobotWorld& world = *worlds[index]->world;
  TerrainModel terrain;
  terrain.world = index;
  terrain.index = world.terrains.size();
  world.AddTerrain(name,new Terrain());
  terrain.terrain = world.terrains.back();
  terrain.terrain->geometry.CreateEmpty();
  return terrain;
}

RobotModel WorldModel::loadRobot(const char* fn)
{
  RobotWorld& world = *worlds[index]->world;
  int oindex=world.LoadRobot(fn);
  if(oindex < 0) return RobotModel();
  RobotModel robot;
  robot.world = index;
  robot.index = oindex;
  robot.robot = world.robots.back();
  if(gEnableCollisionInitialization) 
    world.robots.back()->InitCollisions();
  world.robots.back()->UpdateGeometry();
  return robot;
}

RigidObjectModel WorldModel::loadRigidObject(const char* fn)
{
  RobotWorld& world = *worlds[index]->world;
  int oindex=world.LoadRigidObject(fn);
  if(oindex < 0) return RigidObjectModel();
  RigidObjectModel obj;
  obj.world = index;
  obj.index = oindex;
  obj.object = world.rigidObjects.back();
  if(gEnableCollisionInitialization) 
    world.rigidObjects.back()->InitCollisions();
  world.rigidObjects.back()->UpdateGeometry();
  return obj;
}

TerrainModel WorldModel::loadTerrain(const char* fn)
{
  RobotWorld& world = *worlds[index]->world;
  int oindex=world.LoadTerrain(fn);
  if(oindex < 0) return TerrainModel();
  TerrainModel obj;
  obj.world = index;
  obj.index = oindex;
  obj.terrain = world.terrains.back();
  if(gEnableCollisionInitialization) world.terrains.back()->InitCollisions();
  return obj;
}

int WorldModel::loadElement(const char* fn)
{
  RobotWorld& world = *worlds[index]->world;
  int id = world.LoadElement(fn);
  return id;
}

RobotModel WorldModel::add(const char* name,const RobotModel& robot)
{
  if(robot.robot == NULL)
    throw PyException("add(RobotModel): robot refers to NULL object");
  RobotWorld& world = *worlds[index]->world;
  world.robots.push_back(new Robot);
  *world.robots.back() = *robot.robot;
  return this->robot((int)world.robots.size()-1);
}

RigidObjectModel WorldModel::add(const char* name,const RigidObjectModel& obj)
{
  if(obj.object == NULL)
    throw PyException("add(RigidObjectModel): obj refers to NULL object");
  RobotWorld& world = *worlds[index]->world;
  world.rigidObjects.push_back(new RigidObject);
  *world.rigidObjects.back() = *obj.object;
  return this->rigidObject((int)world.rigidObjects.size()-1);
}
 
TerrainModel WorldModel::add(const char* name,const TerrainModel& terrain)
{
  if(terrain.terrain == NULL)
    throw PyException("add(TerrianModel): terrain refers to NULL object");
  RobotWorld& world = *worlds[index]->world;
  world.terrains.push_back(new Terrain);
  *world.terrains.back() = *terrain.terrain;
  return this->terrain((int)world.terrains.size()-1);
}

void WorldModel::remove(const RobotModel& obj)
{
  if(obj.world != index) 
    throw PyException("Robot does not belong to this world");
  RobotWorld& world = *worlds[index]->world;
  world.robots.erase(world.robots.begin()+obj.index);
}

void WorldModel::remove(const RigidObjectModel& obj)
{
  if(obj.world != index) 
    throw PyException("Rigid object does not belong to this world");
  RobotWorld& world = *worlds[index]->world;
  world.rigidObjects.erase(world.rigidObjects.begin()+obj.index);
}

void WorldModel::remove(const TerrainModel& obj)
{
  if(obj.world != index) 
    throw PyException("Rigid object does not belong to this world");
  RobotWorld& world = *worlds[index]->world;
  world.terrains.erase(world.terrains.begin()+obj.index);
}


void WorldModel::drawGL()
{
  RobotWorld& world = *worlds[index]->world;
  world.DrawGL();
}

void WorldModel::enableGeometryLoading(bool enabled)
{
  Robot::disableGeometryLoading = !enabled;
}

void WorldModel::enableInitCollisions(bool enabled)
{
  gEnableCollisionInitialization = !enabled;
  if(enabled) {
    worlds[index]->world->InitCollisions();
    worlds[index]->world->UpdateGeometry();
  }
}


std::string WorldModel::getName(int id)
{
  RobotWorld& world = *worlds[index]->world;
  return world.GetName(id);
}

Geometry3D WorldModel::geometry(int id)
{
  RobotWorld& world = *worlds[index]->world;
  if(world.IsTerrain(id)>=0 || world.IsRigidObject(id)>=0 || world.IsRobotLink(id).first>=0) {
    Geometry3D geom;
    geom.world = index;
    geom.id = id;
    *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geom.geomPtr) = world.GetGeometry(id);
    return geom;
  }
  Geometry3D geom;
  geom.world = -1;
  geom.id = -1;
  return geom;
}

Appearance WorldModel::appearance(int id)
{
  RobotWorld& world = *worlds[index]->world;
  if(world.IsTerrain(id)>=0 || world.IsRigidObject(id)>=0 || world.IsRobotLink(id).first>=0) {
    Appearance geom;
    geom.world = index;
    geom.id = id;
    *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(geom.appearancePtr) = world.GetAppearance(id);
    return geom;
  }
  Appearance geom;
  geom.world = -1;
  geom.id = -1;
  return geom;
}



RobotModelLink::RobotModelLink()
  :world(-1),robotIndex(-1),robotPtr(NULL),index(-1)
{}

RobotModel RobotModelLink::robot()
{
  RobotModel r;
  r.world = world;
  r.index = robotIndex;
  r.robot = robotPtr;
  return r;
}

const char* RobotModelLink::getName() const
{
  if(index < 0) return "";
  return robotPtr->linkNames[index].c_str();
}

void RobotModelLink::setName(const char* name)
{
  if(index < 0) {
    throw PyException("Cannot set the name of an empty link");
  }
  robotPtr->linkNames[index] = name;
}

int RobotModelLink::getIndex()
{
  return index;
}

int RobotModelLink::getParent()
{
  return robotPtr->parents[index];
}

RobotModelLink RobotModelLink::parent()
{
  if(robotPtr->parents[index] < 0) return RobotModelLink();
  else {
    RobotModelLink res;
    res.world = world;
    res.robotIndex = robotIndex;
    res.robotPtr = robotPtr;
    res.index = robotPtr->parents[index];
    return res;
  }
}

void RobotModelLink::setParent(int p)
{
  if(p < 0 || p >= (int)robotPtr->links.size())
    throw PyException("Invalid parent index");

  //TODO: check for circular references
  robotPtr->parents[index] = p;
}

void RobotModelLink::setParent(const RobotModelLink& link)
{
  if(link.robotPtr == NULL)
    setParent(-1);
  else {
    if(link.robotPtr != robotPtr)
      throw PyException("Can't set a link to have a parent on a different robot");
    setParent(link.index);
  }
}

int RobotModelLink::getID() const
{
  RobotWorld& world = *worlds[this->world]->world;
  return world.RobotLinkID(robotIndex,index);
}

Geometry3D RobotModelLink::geometry()
{
  Geometry3D res;
  res.world = world;
  res.id = getID();
  assert(res.id >= 0);
  *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(res.geomPtr) = worlds[world]->world->GetGeometry(res.id); 
  return res;
}

Appearance RobotModelLink::appearance()
{
  Appearance res;
  res.world = world;
  res.id = getID();
  assert(res.id >= 0);
  *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(res.appearancePtr) = worlds[world]->world->GetAppearance(res.id);
  return res;
}

Mass RobotModelLink::getMass()
{
  Mass mass;
  const RobotLink3D& link=robotPtr->links[index];
  mass.mass=link.mass;
  mass.com.resize(3);
  mass.inertia.resize(9);
  link.com.get(&mass.com[0]);
  link.inertia.get(&mass.inertia[0]);
  return mass;
}

void RobotModelLink::setMass(const Mass& mass)
{
  if(mass.com.size()!=3) {
    throw PyException("Mass com does not have length 3");
  }
  if(mass.inertia.size()!=9 && mass.inertia.size()!=3) {
    throw PyException("Mass inertia does not have length 3 or 9");
  }
  Assert(mass.com.size()==3);
  RobotLink3D& link=robotPtr->links[index];
  link.mass=mass.mass;
  link.com.set(&mass.com[0]);
  if(mass.inertia.size()==3) {
    link.inertia.setZero();
    link.inertia(0,0) = mass.inertia[0];
    link.inertia(1,1) = mass.inertia[1];
    link.inertia(2,2) = mass.inertia[2];
  }
  else {
    link.inertia.set(&mass.inertia[0]);
  }
}

void RobotModelLink::getWorldPosition(const double plocal[3],double pworld[3])
{
  RobotLink3D& link=robotPtr->links[index];
  (link.T_World*Vector3(plocal)).get(pworld);
}

void RobotModelLink::getWorldDirection(const double vlocal[3],double vworld[3])
{
  RobotLink3D& link=robotPtr->links[index];
  (link.T_World.R*Vector3(vlocal)).get(vworld);
}

void RobotModelLink::getLocalPosition(const double pworld[3],double plocal[3])
{
  RobotLink3D& link=robotPtr->links[index];
  Vector3 temp;
  link.T_World.mulInverse(Vector3(pworld),temp);
  temp.get(plocal);
}

void RobotModelLink::getLocalDirection(const double vworld[3],double vlocal[3])
{
  RobotLink3D& link=robotPtr->links[index];
  Vector3 temp;
  link.T_World.R.mulTranspose(Vector3(vworld),temp);
  temp.get(vlocal);
}


void RobotModelLink::getTransform(double R[9],double t[3])
{
  const RobotLink3D& link=robotPtr->links[index];
  link.T_World.R.get(R);
  link.T_World.t.get(t);
}

void RobotModelLink::setTransform(const double R[9],const double t[3])
{
  RobotLink3D& link=robotPtr->links[index];
  link.T_World.R.set(R);
  link.T_World.t.set(t);
  robotPtr->geometry[index]->SetTransform(link.T_World);
}

void RobotModelLink::getParentTransform(double R[9],double t[3])
{
  const RobotLink3D& link=robotPtr->links[index];
  link.T0_Parent.R.get(R);
  link.T0_Parent.t.get(t);
}

void RobotModelLink::setParentTransform(const double R[9],const double t[3])
{
  RobotLink3D& link=robotPtr->links[index];
  link.T0_Parent.R.set(R);
  link.T0_Parent.t.set(t);
}

void RobotModelLink::getAxis(double axis[3])
{
  const RobotLink3D& link=robotPtr->links[index];
  link.w.get(axis);
}

void RobotModelLink::setAxis(const double axis[3])
{
  RobotLink3D& link=robotPtr->links[index];
  link.w.set(axis);
}

void RobotModelLink::getJacobian(const double p[3],vector<vector<double> >& J)
{
  Matrix Jmat;
  robotPtr->GetFullJacobian(Vector3(p),index,Jmat);
  copy(Jmat,J);
}

void RobotModelLink::getPositionJacobian(const double p[3],vector<vector<double> >& J)
{
  Matrix Jmat;
  robotPtr->GetPositionJacobian(Vector3(p),index,Jmat);
  copy(Jmat,J);
}

void RobotModelLink::getOrientationJacobian(vector<vector<double> >& J)
{
  Matrix Jmat;
  Jmat.resize(3,robotPtr->links.size(),Zero);
  int j=index;
  while(j!=-1) {
    Vector3 w;
    robotPtr->GetOrientationJacobian(index,j,w);
    Jmat(0,j)=w.x; Jmat(1,j)=w.y; Jmat(2,j)=w.z;
    j=robotPtr->parents[j];
  }
  copy(Jmat,J);
}

void RobotModelLink::getVelocity(double out[3])
{
  Vector3 v;
  robotPtr->GetWorldVelocity(Vector3(Zero),index,robotPtr->dq,v);
  v.get(out);
}

void RobotModelLink::getAngularVelocity(double out[3])
{
  Vector3 v;
  robotPtr->GetWorldAngularVelocity(index,robotPtr->dq,v);
  v.get(out);
}

void RobotModelLink::getPointVelocity(const double plocal[3],double out[3])
{
  Vector3 v;
  robotPtr->GetWorldVelocity(Vector3(plocal),index,robotPtr->dq,v);
  v.get(out);
}

void RobotModelLink::drawLocalGL(bool keepAppearance)
{
  RobotWorld& world = *worlds[this->world]->world;
  if(keepAppearance) {
    world.robotViews[robotIndex].DrawLink_Local(index);
  }
  else
    world.robots[robotIndex]->DrawLinkGL(index);
}

void RobotModelLink::drawWorldGL(bool keepAppearance)
{
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  GLDraw::glMultMatrix(Matrix4(robotPtr->links[index].T_World));
  drawLocalGL(keepAppearance);
  glPopMatrix();
}

RobotModelDriver::RobotModelDriver()
  :world(-1),robotIndex(-1),robotPtr(NULL),index(-1)
{}

RobotModel RobotModelDriver::robot()
{
  RobotModel r;
  r.world = world;
  r.index = robotIndex;
  r.robot = robotPtr;
  return r;
}

const char* RobotModelDriver::getName() const
{
  if(index < 0) return "";
  return robotPtr->driverNames[index].c_str();
}

/*
void RobotModelDriver::setName(const char* name)
{
  if(index < 0) {
    throw PyException("Cannot set the name of an empty driver");
  }
  robotPtr->driverNames[index] = name;
}
*/


const char* RobotModelDriver::getType()
{
  if(index < 0) return "";
  switch(robotPtr->drivers[index].type) {
  case RobotJointDriver::Normal: return "normal";
  case RobotJointDriver::Affine: return "affine";
  case RobotJointDriver::Translation: return "translation";
  case RobotJointDriver::Rotation: return "rotation";
  case RobotJointDriver::Custom: return "custom";
  default: return "error";
  }
}
int RobotModelDriver::getAffectedLink()
{
  if(index < 0) return -1;
  return robotPtr->drivers[index].linkIndices[0];
}

void RobotModelDriver::getAffectedLinks(std::vector<int>& links)
{
  if(index < 0) links.resize(0); 
  else links = robotPtr->drivers[index].linkIndices;
}

void RobotModelDriver::getAffineCoeffs(std::vector<double>& scale,std::vector<double>& offset)
{
  if(index < 0) {
    scale.resize(0);
    offset.resize(0);
  }
  else {
    scale = robotPtr->drivers[index].affScaling;
    offset = robotPtr->drivers[index].affOffset;
  }
}

void RobotModelDriver::setValue(double val)
{
  robotPtr->SetDriverValue(index,val);
}

double RobotModelDriver::getValue()
{
  return robotPtr->GetDriverValue(index);
}
void RobotModelDriver::setVelocity(double val)
{
  robotPtr->SetDriverVelocity(index,val);
}

double RobotModelDriver::getVelocity()
{
  return robotPtr->GetDriverVelocity(index);
}




RobotModel::RobotModel()
  :world(-1),index(-1),robot(NULL)
{}

const char* RobotModel::getName() const
{
  if(index < 0) throw PyException("Robot is empty");
  RobotWorld& world = *worlds[this->world]->world;
  return world.robots[index]->name.c_str();
}

void RobotModel::setName(const char* name)
{
  if(index < 0) {
    throw PyException("Cannot set the name of an empty robot");
  }
  RobotWorld& world = *worlds[this->world]->world;
  world.robots[index]->name = name;
}


int RobotModel::getID() const
{
  if(index < 0) return -1;
  RobotWorld& world = *worlds[this->world]->world;
  return world.RobotID(index);
}

int RobotModel::numLinks()
{
  if(index < 0) return -1;
  return robot->links.size();
}

RobotModelLink RobotModel::link(int linkindex)
{
  RobotModelLink link;
  link.world = world;
  link.robotIndex = index;
  link.robotPtr = robot;
  link.index = linkindex;
  return link;
}

RobotModelLink RobotModel::link(const char* name)
{
  string sname(name);
  for(size_t i=0;i<robot->linkNames.size();i++)
    if(sname == robot->linkNames[i]) {
      return link((int)i);
    }
  RobotModelLink link;
  link.world = this->world;
  link.robotPtr = robot;
  link.robotIndex = index;
  link.index = -1;
  return link;
}


int RobotModel::numDrivers()
{
  if(index < 0) return -1;
  return robot->drivers.size();
}

RobotModelDriver RobotModel::driver(int driverindex)
{
  RobotModelDriver link;
  link.world = world;
  link.robotIndex = index;
  link.robotPtr = robot;
  link.index = driverindex;
  return link;
}

RobotModelDriver RobotModel::driver(const char* name)
{
  string sname(name);
  for(size_t i=0;i<robot->driverNames.size();i++)
    if(sname == robot->driverNames[i]) {
      return driver((int)i);
    }
  RobotModelDriver link;
  link.world = this->world;
  link.robotPtr = robot;
  link.robotIndex = index;
  link.index = -1;
  return link;
}

const char* RobotModel::getJointType(int dofIndex)
{
  if(index < 0) throw PyException("Empty robot");
  for(size_t i=0;i<robot->joints.size();i++) {
    if(robot->DoesJointAffect((int)i,dofIndex)) {
      switch(robot->joints[i].type) {
      case RobotJoint::Weld: return "weld";
      case RobotJoint::Normal: return "normal";
      case RobotJoint::Spin: return "spin";
      case RobotJoint::Floating: return "floating";
      case RobotJoint::FloatingPlanar: return "floatingplanar";
      case RobotJoint::BallAndSocket: return "ballandsocket";
      default:
        return "invalid joint type?";
      }
    }
  }
  throw PyException("DOF is not affected by any joint definition?");
}

const char* RobotModel::getJointType(const char* name)
{
  RobotModelLink l = link(name);
  if(l.index < 0) throw PyException("Invalid DOF named");
  return getJointType(l.index);
}


void RobotModel::getConfig(vector<double>& q)
{
  q.resize(robot->q.n);
  robot->q.getCopy(&q[0]);
}

void RobotModel::getVelocity(vector<double>& dq)
{
  dq.resize(robot->dq.n);
  robot->dq.getCopy(&dq[0]);
}

void RobotModel::setConfig(const vector<double>& q)
{
  if(robot->links.size() != q.size()) {
    throw PyException("Invalid size of configuration");
  }
  robot->q.copy(&q[0]);
  robot->UpdateFrames();
  robot->UpdateGeometry();
}

void RobotModel::setVelocity(const vector<double>& dq)
{
  if(robot->links.size() != dq.size()) {
    throw PyException("Invalid size of velocity");
  }
  robot->dq.copy(&dq[0]);
}

void RobotModel::getJointLimits(vector<double>& qmin,vector<double>& qmax)
{
  qmin.resize(robot->q.n);
  qmax.resize(robot->q.n);
  robot->qMin.getCopy(&qmin[0]);
  robot->qMax.getCopy(&qmax[0]);
}

void RobotModel::setJointLimits(const vector<double>& qmin,const vector<double>& qmax)
{
  if(robot->links.size() != qmin.size()) {
    throw PyException("Invalid size of joint limit");
  }
  if(robot->links.size() != qmax.size()) {
    throw PyException("Invalid size of joint limit");
  }
  robot->qMin.copy(&qmin[0]);
  robot->qMax.copy(&qmax[0]);

  for(unsigned int i = 0; i < robot->drivers.size(); ++i)
  {
      int link_index = robot->drivers[i].linkIndices[0];
      robot->drivers[i].qmin = qmin[link_index];
      robot->drivers[i].qmax = qmax[link_index];
  }
}

void RobotModel::getVelocityLimits(vector<double>& vmax)
{
  vmax.resize(robot->q.n);
  robot->velMax.getCopy(&vmax[0]);
}

void RobotModel::setVelocityLimits(const vector<double>& vmax)
{
  if(robot->links.size() != vmax.size()) {
    throw PyException("Invalid size of velocity limit");
  }
  robot->velMax.copy(&vmax[0]);

  for(unsigned int i = 0; i < robot->drivers.size(); ++i)
  {
      int link_index = robot->drivers[i].linkIndices[0];
      robot->drivers[i].vmin = -vmax[link_index];
      robot->drivers[i].vmax = vmax[link_index];
  }
}

void RobotModel::getAccelerationLimits(vector<double>& amax)
{
  amax.resize(robot->q.n);
  robot->accMax.getCopy(&amax[0]);
}

void RobotModel::setAccelerationLimits(const vector<double>& amax)
{
  if(robot->links.size() != amax.size()) {
    throw PyException("Invalid size of acceleration limit");
  }
  robot->accMax.copy(&amax[0]);

  for(unsigned int i = 0; i < robot->drivers.size(); ++i)
  {
      int link_index = robot->drivers[i].linkIndices[0];
      robot->drivers[i].amin = -amax[link_index];
      robot->drivers[i].amax = amax[link_index];
  }
}

void RobotModel::getTorqueLimits(vector<double>& tmax)
{
  tmax.resize(robot->q.n);
  robot->torqueMax.getCopy(&tmax[0]);
}

void RobotModel::setTorqueLimits(const vector<double>& tmax)
{
  if(robot->links.size() != tmax.size()) {
    throw PyException("Invalid size of torque limits");
  }
  robot->torqueMax.copy(&tmax[0]);

  for(unsigned int i = 0; i < robot->drivers.size(); ++i)
  {
      int link_index = robot->drivers[i].linkIndices[0];
      robot->drivers[i].tmin = -tmax[link_index];
      robot->drivers[i].tmax = tmax[link_index];
  }
}

void RobotModel::setDOFPosition(int i,double qi)
{
  if(i < 0 || i >= (int)robot->links.size()) {
    throw PyException("Invalid joint index");
  }
  robot->q(i) = qi;
  robot->UpdateFrames();
}

void RobotModel::setDOFPosition(const char* name,double qi)
{
  string sname(name);
  for(size_t i=0;i<robot->linkNames.size();i++)
    if(sname == robot->linkNames[i]) {
      robot->q(i) = qi;
      robot->UpdateFrames();
      return;
    }
  throw PyException("Invalid link name");
}

double RobotModel::getDOFPosition(int i)
{
  return robot->q(i);
}

double RobotModel::getDOFPosition(const char* name)
{
  string sname(name);
  for(size_t i=0;i<robot->linkNames.size();i++)
    if(sname == robot->linkNames[i]) {
      return robot->q(i);
    }
  throw PyException("Invalid link name");
  return 0;
}


void RobotModel::interpolate(const std::vector<double>& a,const std::vector<double>& b,double u,std::vector<double>& out)
{
  Vector va(a),vb(b),vout;
  Interpolate(*robot,va,vb,u,vout);
  out = vout;
}

double RobotModel::distance(const std::vector<double>& a,const std::vector<double>& b)
{
  if(robot->links.size() != a.size()) 
    throw PyException("Invalid size of configuration");
  if(robot->links.size() != b.size()) 
    throw PyException("Invalid size of configuration");
  Vector va(a),vb(b);
  return Distance(*robot,va,vb,Inf);
}

void RobotModel::interpolateDeriv(const std::vector<double>& a,const std::vector<double>& b,std::vector<double>& dout)
{
  if(robot->links.size() != a.size()) 
    throw PyException("Invalid size of configuration");
  if(robot->links.size() != b.size()) 
    throw PyException("Invalid size of configuration");
  Vector va(a),vb(b),vout;
  InterpolateDerivative(*robot,va,vb,vout);
  dout = vout;
}

bool RobotModel::selfCollisionEnabled(int link1,int link2)
{
  if (link1 > link2) swap(link1,link2);
  return (robot->selfCollisions(link1,link2) != NULL);
}

void RobotModel::enableSelfCollision(int link1,int link2,bool value)
{
  if (link1 > link2) swap(link1,link2);
  if(value) {
    if(!robot->selfCollisions(link1,link2))
      robot->InitSelfCollisionPair(link1,link2);
  }
  else {
    if(robot->selfCollisions(link1,link2))
      SafeDelete(robot->selfCollisions(link1,link2));
  }
}

bool RobotModel::selfCollides()
{
  /* old version
  for(size_t i=0;i<robot->links.size();i++)
    for(size_t j=0;j<robot->links.size();j++)
      if(robot->SelfCollision(i,j)) return true;
  return false;
  */
  return robot->SelfCollision();
}

void RobotModel::randomizeConfig(double unboundedStdDeviation)
{
  RobotCSpace space(*robot);
  space.Sample(robot->q);
  for(size_t i=0;i<robot->joints.size();i++)
    if(robot->joints[i].type == RobotJoint::Floating) {
      int base = robot->joints[i].baseIndex;
      robot->q[base] *= unboundedStdDeviation;
      robot->q[base+1] *= unboundedStdDeviation;
      robot->q[base+2] *= unboundedStdDeviation;
    }
    else if(robot->joints[i].type == RobotJoint::FloatingPlanar) {
      int base = robot->joints[i].baseIndex;
      robot->q[base] *= unboundedStdDeviation;
      robot->q[base+1] *= unboundedStdDeviation;
    }
  robot->UpdateFrames();
  robot->UpdateGeometry();
}

void RobotModel::drawGL(bool keepAppearance)
{
  RobotWorld& world = *worlds[this->world]->world;
  if(keepAppearance) {
    world.robotViews[index].Draw();
  }
  else {
    for(size_t i=0;i<robot->links.size();i++)
      world.robotViews[index].DrawLink_World(i,false);
  }
}

void RobotModel::getCom(double out[3])
{
  Vector3 com = robot->GetCOM();
  com.get(out);
}

void RobotModel::getComJacobian(std::vector<std::vector<double> >& out)
{
  Matrix J;
  robot->GetCOMJacobian(J);
  copy(J,out);
}

void RobotModel::getMassMatrix(std::vector<std::vector<double> >& B)
{
  Matrix Bmat;
  robot->UpdateDynamics();
  robot->GetKineticEnergyMatrix(Bmat);
  copy(Bmat,B);
}

void RobotModel::getMassMatrixInv(std::vector<std::vector<double> >& Binv)
{
  Matrix Bmat;
  //robot->UpdateDynamics();
  NewtonEulerSolver ne(*robot);
  ne.CalcKineticEnergyMatrixInverse(Bmat);
  copy(Bmat,Binv);
}

void RobotModel::getCoriolisForceMatrix(std::vector<std::vector<double> >& C)
{
  Matrix Cmat;
  robot->UpdateDynamics();
  robot->GetCoriolisForceMatrix(Cmat);
  copy(Cmat,C);
}

void RobotModel::getCoriolisForces(std::vector<double>& C)
{
  Vector Cvec;
  if(robot->links.size() > 6) {
    NewtonEulerSolver ne(*robot);
    ne.CalcResidualTorques(Cvec);
  }
  else {
    robot->UpdateDynamics();
    robot->GetCoriolisForces(Cvec);
  }
  copy(Cvec,C);
}

void RobotModel::getGravityForces(const double g[3],std::vector<double>& G)
{
  Vector Gvec;
  robot->GetGravityTorques(Vector3(g),Gvec);
  copy(Gvec,G);
}

void RobotModel::torquesFromAccel(const std::vector<double>& ddq,std::vector<double>& out)
{
  Vector ddqvec,tvec;
  copy(ddq,ddqvec);
  if(robot->links.size() > 6) {
    NewtonEulerSolver ne(*robot);
    ne.CalcTorques(ddqvec,tvec);
  }
  else {
    robot->UpdateDynamics();
    robot->CalcTorques(ddqvec,tvec);
  }
  copy(tvec,out);
}

void RobotModel::accelFromTorques(const std::vector<double>& t,std::vector<double>& out)
{
  Vector ddqvec,tvec;
  copy(t,tvec);
  if(robot->links.size() > 6) {
    NewtonEulerSolver ne(*robot);
    ne.CalcAccel(tvec,ddqvec);
  }
  else {
    robot->UpdateDynamics();
    robot->CalcAcceleration(ddqvec,tvec);
  }
  copy(ddqvec,out);
}


RigidObjectModel::RigidObjectModel()
  :world(-1),index(-1),object(NULL)
{}

const char* RigidObjectModel::getName() const
{
  RobotWorld& world = *worlds[this->world]->world;
  return world.rigidObjects[index]->name.c_str();
}

void RigidObjectModel::setName(const char* name)
{
  if(index < 0) {
    throw PyException("Cannot set the name of an empty rigid object");
  }
  RobotWorld& world = *worlds[this->world]->world;
  world.rigidObjects[index]->name = name;
}

int RigidObjectModel::getID() const
{
  RobotWorld& world = *worlds[this->world]->world;
  return world.RigidObjectID(index);
}

Geometry3D RigidObjectModel::geometry()
{
  Geometry3D res;
  res.world = world;
  res.id = getID();
  assert(res.id >= 0);
  *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(res.geomPtr) = worlds[world]->world->GetGeometry(res.id); 
  return res;
}

Appearance RigidObjectModel::appearance()
{
  Appearance res;
  res.world = world;
  res.id = getID();
  assert(res.id >= 0);
  *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(res.appearancePtr) = worlds[world]->world->GetAppearance(res.id);
  return res;
}

Mass RigidObjectModel::getMass()
{
  Mass mass;
  RigidObject* obj=object;
  mass.mass = obj->mass;
  mass.com.resize(3);
  mass.inertia.resize(9);
  obj->com.get(&mass.com[0]);
  obj->inertia.get(&mass.inertia[0]);
  return mass;
}

void RigidObjectModel::setMass(const Mass& mass)
{
  if(mass.com.size()!=3) {
    throw PyException("Mass com does not have length 3");
  }
  if(mass.inertia.size()!=9 && mass.inertia.size()!=3) {
    throw PyException("Mass inertia does not have length 3 or 9");
  }
  RigidObject* obj=object;
  obj->mass = mass.mass;
  obj->com.set(&mass.com[0]);
  if(mass.inertia.size()==3) {
    obj->inertia.setZero();
    obj->inertia(0,0) = mass.inertia[0];
    obj->inertia(1,1) = mass.inertia[1];
    obj->inertia(2,2) = mass.inertia[2];
  }
  else {
    obj->inertia.set(&mass.inertia[0]);
  }
}

ContactParameters RigidObjectModel::getContactParameters()
{
  ContactParameters params;
  RigidObject* obj=object;
  params.kFriction = obj->kFriction;
  params.kRestitution = obj->kRestitution;
  params.kStiffness = obj->kStiffness;
  params.kDamping = obj->kDamping;
  return params;
}

void RigidObjectModel::setContactParameters(const ContactParameters& params)
{
  RigidObject* obj=object;
  obj->kFriction = params.kFriction;
  obj->kRestitution = params.kRestitution;
  obj->kStiffness = params.kStiffness;
  obj->kDamping = params.kDamping;
}

void RigidObjectModel::getTransform(double R[9],double t[3])
{
  RigidObject* obj=object;
  obj->T.R.get(R);
  obj->T.t.get(t);
}

void RigidObjectModel::setTransform(const double R[9],const double t[3])
{
  RigidObject* obj=object;
  obj->T.R.set(R);
  obj->T.t.set(t);
  obj->geometry->SetTransform(obj->T);
}

void RigidObjectModel::getVelocity(double out[3],double out2[3])
{
  RigidObject* obj=object;
  obj->w.get(out);
  obj->v.get(out2);
}

void RigidObjectModel::setVelocity(const double angularVelocity[3],const double velocity[3])
{
  RigidObject* obj=object;
  obj->w.set(angularVelocity);
  obj->v.set(velocity);
}

void RigidObjectModel::drawGL(bool keepAppearance)
{
  RobotWorld& world = *worlds[this->world]->world;
  if(keepAppearance) {
    world.rigidObjects[index]->DrawGL();
  }
  else {
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    GLDraw::glMultMatrix(Matrix4(object->T));
    GLDraw::draw(*object->geometry);
    glPopMatrix();
  }
}


TerrainModel::TerrainModel()
  :world(-1),index(-1),terrain(NULL)
{
}

const char* TerrainModel::getName() const
{
  RobotWorld& world = *worlds[this->world]->world;
  return world.terrains[index]->name.c_str();
}

void TerrainModel::setName(const char* name)
{
  if(index < 0) {
    throw PyException("Cannot set the name of an empty rigid object");
  }
  RobotWorld& world = *worlds[this->world]->world;
  world.terrains[index]->name = name;
}


int TerrainModel::getID() const
{
  RobotWorld& world = *worlds[this->world]->world;
  return world.TerrainID(index);
}

Geometry3D TerrainModel::geometry()
{
  Geometry3D res;
  res.world = world;
  res.id = getID();
  assert(res.id >= 0);
  *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(res.geomPtr) = worlds[world]->world->GetGeometry(res.id); 
  return res;
}


Appearance TerrainModel::appearance()
{
  Appearance res;
  res.world = world;
  res.id = getID();
  assert(res.id >= 0);
  *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(res.appearancePtr) = worlds[world]->world->GetAppearance(res.id);
  return res;
}

void TerrainModel::setFriction(double friction)
{
  terrain->SetUniformFriction(friction);
}

void TerrainModel::drawGL(bool keepAppearance)
{
  RobotWorld& world = *worlds[this->world]->world;
  if(keepAppearance) {
    world.terrains[index]->DrawGL();
  }
  else {
    GLDraw::draw(*terrain->geometry);
  }
}








/***************************  SIMULATION CODE ***************************************/


Simulator::Simulator(const WorldModel& model)
{
#ifdef dDOUBLE
  if(dCheckConfiguration("ODE_double_precision")!=1) {
    FatalError("ODE is compiled with single precision but Klamp't Python API is compiled with double, please check that -DdSINGLE is defined in the KLAMPT_DEFINITIONS variable in CMake and recompile");
  }
#else
  if(dCheckConfiguration("ODE_single_precision")!=1) {
    FatalError("ODE is compiled with double precision but Klamp't Python API is compiled with single, please check that -DdDOUBLE is defined in the KLAMPT_DEFINITIONS variable in CMake and recompile");
  }
#endif


  index = createSim();
  world = model;
  sim = &sims[index]->sim;

  //initialize simulation
  LOG4CXX_INFO(KrisLibrary::logger(),"Initializing simulation...\n");
  RobotWorld& rworld=*worlds[model.index]->world;
  sim->Init(&rworld);

  //setup controllers
  sim->robotControllers.resize(rworld.robots.size());
  for(size_t i=0;i<sim->robotControllers.size();i++) {
    Robot* robot=rworld.robots[i];
    sim->SetController(i,MakeController(robot));

    sim->controlSimulators[i].sensors.MakeDefault(robot);
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"Done\n");


  //setup ODE settings, if any
  TiXmlElement* e=worlds[world.index]->xmlWorld.GetElement("simulation");
  if(e) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Reading simulation settings...\n");
    XmlSimulationSettings s(e);
    if(!s.GetSettings(*sim)) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, simulation settings not read correctly\n");
    }
    LOG4CXX_INFO(KrisLibrary::logger(),"Done\n");
  }

  //TEMP: play around with auto disable of rigid objects
  //for(size_t i=0;i<sim->odesim.numObjects();i++)
  //    dBodySetAutoDisableFlag(sim->odesim.object(i)->body(),1);

  sim->WriteState(initialState);
}

Simulator::~Simulator()
{
  destroySim(index);
}

void Simulator::reset()
{
  sim->ReadState(initialState);
}

int Simulator::getStatus()
{
  return sim->worstStatus;
}

std::string Simulator::getStatusString(int s)
{
  if(s < 0) s = getStatus();
  switch(s) {
    case 0: return "normal";
    case 1: return "adaptive time stepping";
    case 2: return "contact unreliable";
    case 3: return "unstable";
    default: return "error";
  }
}

string Simulator::getState()
{
  string str;
  sim->WriteState(str);
  return ToBase64(str);
}

void Simulator::setState(const string& str)
{
  sim->ReadState(FromBase64(str));
}

void Simulator::simulate(double t)
{
  sim->Advance(t);
  sim->UpdateModel();
}

void Simulator::fakeSimulate(double t)
{
  sim->AdvanceFake(t);
  sim->UpdateModel();
}

double Simulator::getTime()
{
  return sim->time;
}

void Simulator::updateWorld()
{
  sim->UpdateModel();
}

void Simulator::getActualConfig(int robot,std::vector<double>& out)
{
  Vector qv;
  sim->controlSimulators[robot].GetSimulatedConfig(qv);
  out = qv;
}

void Simulator::getActualVelocity(int robot,std::vector<double>& out)
{
  Vector qv;
  sim->controlSimulators[robot].GetSimulatedVelocity(qv);
  out = qv;
}

void Simulator::getActualTorques(int robot,std::vector<double>& out)
{
  Vector t;
  sim->controlSimulators[robot].GetActuatorTorques(t);
  out = t;
}

bool Simulator::inContact(int aid,int bid)
{
  return sim->InContact(aid,bid);
}

void Simulator::contactForce(int aid,int bid,double res[3])
{
  sim->MeanContactForce(aid,bid).get(res);
}

void Simulator::contactTorque(int aid,int bid,double res[3])
{
  sim->MeanContactTorque(aid,bid).get(res);
}


void Simulator::getContacts(int aid,int bid,std::vector<std::vector<double> >& out)
{
  ODEContactList* c=sim->GetContactList(aid,bid);
  if(!c) {
    out.resize(0);
    return;
  }
  out.resize(c->points.size());
  for(size_t i=0;i<c->points.size();i++) {
    out[i].resize(7);
    c->points[i].x.get(out[i][0],out[i][1],out[i][2]);
    c->points[i].n.get(out[i][3],out[i][4],out[i][5]);
    out[i][6] = c->points[i].kFriction;
    if(bid < aid) {
      out[i][3] = -out[i][3];
      out[i][4] = -out[i][4];
      out[i][5] = -out[i][5];
    }
  }
}

void Simulator::getContactForces(int aid,int bid,std::vector<std::vector<double> >& out)
{
  ODEContactList* c=sim->GetContactList(aid,bid);
  if(!c) {
    out.resize(0);
    return;
  }
  out.resize(c->forces.size());
  for(size_t i=0;i<c->forces.size();i++) {
    out[i].resize(3);
    c->forces[i].get(out[i][0],out[i][1],out[i][2]);
    if(bid < aid) {
      out[i][0] = -out[i][0];
      out[i][1] = -out[i][1];
      out[i][2] = -out[i][2];
    }
  }
}

bool Simulator::hadContact(int aid,int bid)
{
  return sim->HadContact(aid,bid);
}

bool Simulator::hadSeparation(int aid,int bid)
{
  return sim->HadSeparation(aid,bid);
}

bool Simulator::hadPenetration(int aid,int bid)
{
  return sim->HadPenetration(aid,bid);
}

void Simulator::meanContactForce(int aid,int bid,double out[3])
{
  sim->MeanContactForce(aid,bid).get(out);
}

void Simulator::enableContactFeedback(int obj1,int obj2)
{
  sim->EnableContactFeedback(obj1,obj2);
}

void Simulator::enableContactFeedbackAll()
{
  //setup feedback
  RobotWorld& rworld=*worlds[world.index]->world;
  //world-object
  const ODESimulatorSettings& settings = sim->odesim.GetSettings();
  if(settings.rigidObjectCollisions) {
    for(size_t i=0;i<rworld.rigidObjects.size();i++) {
      int objid = rworld.RigidObjectID(i);
      for(size_t j=0;j<rworld.terrains.size();j++)
    sim->EnableContactFeedback(objid,rworld.TerrainID(j));
    }
  }
  for(size_t r=0;r<rworld.robots.size();r++) {
    for(size_t j=0;j<rworld.robots[r]->links.size();j++) {
      int linkid = rworld.RobotLinkID(r,j);
      //robot-world
      for(size_t i=0;i<rworld.rigidObjects.size();i++) {
    sim->EnableContactFeedback(rworld.RigidObjectID(i),linkid);
      }
      //robot-object
      for(size_t i=0;i<rworld.terrains.size();i++) {
    sim->EnableContactFeedback(rworld.TerrainID(i),linkid);
      }
      //robot-self
      if(settings.robotSelfCollisions) {
    for(size_t k=0;k<rworld.robots[r]->links.size();k++) {
      if(rworld.robots[r]->selfCollisions(j,k)) {
        sim->EnableContactFeedback(rworld.RobotLinkID(r,k),linkid);
      }
    }
      }
      //robot-robot
      if(settings.robotRobotCollisions) {
    for(size_t i=0;i<rworld.robots.size();i++) {
      if(i==r) continue;
      for(size_t k=0;k<rworld.robots[i]->links.size();k++) {
        sim->EnableContactFeedback(rworld.RobotLinkID(i,k),linkid);
      }
    }
      }
    }
  }
}

void Simulator::setGravity(const double g[3])
{
  sim->odesim.SetGravity(Vector3(g));
}

void Simulator::setSimStep(double dt)
{
  sim->simStep = dt;
}

  /// Retreives some simulation setting.  Valid names are gravity,
  /// simStep, boundaryLayerCollisions, rigidObjectCollisions, robotSelfCollisions,
  /// robotRobotCollisions, adaptiveTimeStepping, maxContacts,
  /// clusterNormalScale, errorReductionParameter, and dampedLeastSquaresParameter
std::string Simulator::getSetting(const std::string& name)
{
  ODESimulatorSettings& settings = sim->odesim.GetSettings();
  stringstream ss;
  if(name == "gravity") ss << Vector3(settings.gravity);
  else if(name == "simStep") ss << sim->simStep;
  else if(name == "boundaryLayerCollisions") ss << settings.boundaryLayerCollisions;
  else if(name == "rigidObjectCollisions") ss << settings.rigidObjectCollisions;
  else if(name == "robotSelfCollisions") ss << settings.robotSelfCollisions;
  else if(name == "robotRobotCollisions") ss << settings.robotRobotCollisions;
  else if(name == "adaptiveTimeStepping") ss << settings.adaptiveTimeStepping;
  else if(name == "minimumAdaptiveTimeStep") ss << settings.minimumAdaptiveTimeStep;
  else if(name == "maxContacts") ss << settings.maxContacts;
  else if(name == "clusterNormalScale") ss << settings.clusterNormalScale;
  else if(name == "errorReductionParameter") ss << settings.errorReductionParameter;
  else if(name == "dampedLeastSquaresParameter") ss << settings.dampedLeastSquaresParameter;
  else if(name == "instabilityConstantEnergyThreshold") ss << settings.instabilityConstantEnergyThreshold;
  else if(name == "instabilityLinearEnergyThreshold") ss << settings.instabilityLinearEnergyThreshold;
  else if(name == "instabilityMaxEnergyThreshold") ss << settings.instabilityMaxEnergyThreshold;
  else if(name == "instabilityPostCorrectionEnergy") ss << settings.instabilityPostCorrectionEnergy;
  else throw PyException("Invalid setting queried in Simulator.getSetting()");
  return ss.str();
}

void Simulator::setSetting(const std::string& name,const std::string& value)
{
  ODESimulatorSettings& settings = sim->odesim.GetSettings();
  stringstream ss(value);
  if(name == "gravity") { Vector3 g; ss >> g; sim->odesim.SetGravity(settings.gravity); }
  else if(name == "simStep") ss >> sim->simStep;
  else if(name == "boundaryLayerCollisions") ss >> settings.boundaryLayerCollisions;
  else if(name == "rigidObjectCollisions") ss >> settings.rigidObjectCollisions;
  else if(name == "robotSelfCollisions") ss >> settings.robotSelfCollisions;
  else if(name == "robotRobotCollisions") ss >> settings.robotRobotCollisions;
  else if(name == "adaptiveTimeStepping") ss >> settings.adaptiveTimeStepping;
  else if(name == "minimumAdaptiveTimeStep") ss >> settings.minimumAdaptiveTimeStep;
  else if(name == "maxContacts") ss >> settings.maxContacts;
  else if(name == "clusterNormalScale") ss >> settings.clusterNormalScale;
  else if(name == "errorReductionParameter") { ss >> settings.errorReductionParameter; sim->odesim.SetERP(settings.errorReductionParameter); }
  else if(name == "dampedLeastSquaresParameter") { ss >> settings.dampedLeastSquaresParameter; sim->odesim.SetCFM(settings.dampedLeastSquaresParameter); }
  else if(name == "instabilityConstantEnergyThreshold") ss >> settings.instabilityConstantEnergyThreshold;
  else if(name == "instabilityLinearEnergyThreshold") ss >> settings.instabilityLinearEnergyThreshold;
  else if(name == "instabilityMaxEnergyThreshold") ss >> settings.instabilityMaxEnergyThreshold;
  else if(name == "instabilityPostCorrectionEnergy") ss >> settings.instabilityPostCorrectionEnergy;
  else throw PyException("Invalid setting queried in Simulator.setSetting()");
  if(ss.bad()) throw PyException("Invalid value string argument in Simulator.setSetting()");
}



SimRobotController Simulator::controller(int robot)
{
  SimRobotController c;
  c.sim = this;
  c.controller = &sim->controlSimulators[robot];
  c.index = robot;
  return c;
}


SimRobotController Simulator::controller(const RobotModel& robot)
{
  SimRobotController c;
  c.sim = this;
  c.controller = &sim->controlSimulators[robot.index];
  c.index = robot.index;
  return c;
}

int SimBody::getID() const
{
  return objectID;
}

void SimBody::enable(bool enabled)
{
  if(!body) return;
  if(!enabled) dBodyDisable(body);
  else dBodyEnable(body);
}

bool SimBody::isEnabled()
{
  if(!body) return false;
  return dBodyIsEnabled(body) != 0;
}

void SimBody::enableDynamics(bool enabled)
{
  if(!body) return;
  if(!enabled) dBodySetKinematic(body);
  else dBodySetDynamic(body);
}

bool SimBody::isDynamicsEnabled()
{
  if(!body) return false;
  return dBodyIsKinematic(body) == 0;
}


void SimBody::applyWrench(const double f[3],const double t[3])
{
  if(!body) return;
  sim->sim->hooks.push_back(new WrenchHook(body,Vector3(f),Vector3(t)));
  sim->sim->hooks.back()->autokill = true;
  //dBodyAddForce(body,f[0],f[1],f[2]);
  //dBodyAddTorque(body,t[0],t[1],t[2]);
}

void SimBody::applyForceAtPoint(const double f[3],const double pworld[3])
{
  if(!body) return;
  sim->sim->hooks.push_back(new ForceHook(body,Vector3(pworld),Vector3(f)));
  sim->sim->hooks.back()->autokill = true;
  //dBodyAddForceAtPos(body,f[0],f[1],f[2],pworld[0],pworld[1],pworld[2]);
}

void SimBody::applyForceAtLocalPoint(const double f[3],const double plocal[3])
{
  if(!body) return;
  sim->sim->hooks.push_back(new LocalForceHook(body,Vector3(plocal),Vector3(f)));
  sim->sim->hooks.back()->autokill = true;
  //dBodyAddForceAtRelPos(body,f[0],f[1],f[2],plocal[0],plocal[1],plocal[2]);
}

void SimBody::setVelocity(const double w[3],const double v[3])
{
  if(!body) return;
  dBodySetLinearVel(body,v[0],v[1],v[2]);
  dBodySetAngularVel(body,w[1],w[1],w[2]);

}

void SimBody::getVelocity(double out[3],double out2[3])
{
  if(!body) {
    out[0] = out[1] = out[2] = out2[0] = out2[1] = out2[2] = 0;
    return;
  }
  const dReal* v=dBodyGetLinearVel(body);
  const dReal* w=dBodyGetAngularVel(body);
  for(int i=0;i<3;i++) out[i] = w[i];
  for(int i=0;i<3;i++) out2[i] = v[i];
}

void SimBody::setTransform(const double R[9],const double t[3])
{
  //out matrix is 3x3 column major, ODE matrices are 4x4 row major
  if(!body) return;
  dBodySetPosition(body,t[0],t[1],t[2]);
  dMatrix3 rot;
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      rot[i*4+j] = R[i+j*3];
  dBodySetRotation(body,rot);
}

void SimBody::getTransform(double out[9],double out2[3])
{
  //out matrix is 3x3 column major, ODE matrices are 4x4 row major
  if(!body) {
    for(int i=0;i<9;i++) out[i]=0;
    for(int i=0;i<3;i++) out2[i]=0;
    out[0] = out[4] = out[8] = 1.0;
    return;
  }
  const dReal* t=dBodyGetPosition(body);
  const dReal* R=dBodyGetRotation(body);
  for(int i=0;i<3;i++) out2[i] = t[i];
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      out[i+j*3] = R[i*4+j];
}

void SimBody::setObjectTransform(const double R[9],const double t[3])
{
  ODEObjectID id = sim->sim->WorldToODEID(objectID);
  if(id.IsRigidObject()) sim->sim->odesim.object(id.index)->SetTransform(RigidTransform(Matrix3(R),Vector3(t)));
  else if(id.IsRobot()) sim->sim->odesim.robot(id.index)->SetLinkTransform(id.bodyIndex,RigidTransform(Matrix3(R),Vector3(t)));
  else setTransform(R,t);
}

void SimBody::getObjectTransform(double out[9],double out2[3])
{
  ODEObjectID id = sim->sim->WorldToODEID(objectID);
  if(id.IsRigidObject()) {
    RigidTransform T;
    sim->sim->odesim.object(id.index)->GetTransform(T);
    T.R.get(out);
    T.t.get(out2);
  }
  else if(id.IsRobot()) {
    RigidTransform T;
    sim->sim->odesim.robot(id.index)->GetLinkTransform(id.bodyIndex,T);
    T.R.get(out);
    T.t.get(out2);
  }
  else getTransform(out,out2);
}

void SimBody::setCollisionPadding(double padding)
{
  if(!geometry) return;
  geometry->SetPadding(padding);
}

double SimBody::getCollisionPadding()
{
  if(!geometry) return 0;
  return geometry->GetPadding();
}

void SimBody::setCollisionPreshrink(bool shrinkVisualization)
{
  if(!geometry) return;
  geometry->SetPaddingWithPreshrink(geometry->GetPadding(),shrinkVisualization);
}

ContactParameters SimBody::getSurface()
{
  ContactParameters res;
  if(!geometry) {
    res.kFriction=res.kRestitution=res.kStiffness=res.kDamping=0;
  }
  else {
    ODESurfaceProperties* params = &geometry->surf();
    res.kFriction=params->kFriction;
    res.kRestitution=params->kRestitution;
    res.kStiffness=params->kStiffness;
    res.kDamping=params->kDamping;
  }
  return res;
}

void SimBody::setSurface(const ContactParameters& res)
{
  if(!geometry) return;
  ODESurfaceProperties* params = &geometry->surf();
  params->kFriction=res.kFriction;
  params->kRestitution=res.kRestitution;
  params->kStiffness=res.kStiffness;
  params->kDamping=res.kDamping;
}


SimBody Simulator::body(const RobotModelLink& link)
{
  SimBody b;
  b.sim = this;
  b.objectID = link.getID();
  b.body = sim->odesim.robot(link.robotIndex)->body(link.index);
  b.geometry = sim->odesim.robot(link.robotIndex)->triMesh(link.index);
  return b;
}

SimBody Simulator::body(const RigidObjectModel& object)
{
  SimBody b;
  b.sim = this;
  b.objectID = object.getID();
  b.body = sim->odesim.object(object.index)->body();
  b.geometry = sim->odesim.object(object.index)->triMesh();
  return b; 
}

SimBody Simulator::body(const TerrainModel& terrain)
{
  SimBody b;
  b.sim = this;
  b.objectID = terrain.getID();
  b.body = NULL;
  b.geometry = sim->odesim.terrainGeom(terrain.index);
  return b;
}

void Simulator::getJointForces(const RobotModelLink& link,double out[6])
{
  ODERobot* oderobot = sim->odesim.robot(link.robotIndex);
  dJointFeedback fb = oderobot->feedback(link.index);
  Vector3 fw(fb.f1[0],fb.f1[1],fb.f1[2]);
  RigidTransform T;
  oderobot->GetLinkTransform(link.index,T);
  Vector3 mcomw = Vector3(fb.t1[0],fb.t1[1],fb.t1[2]);
  //convert moment about link's com to moment about localpos
  //mp_w = (p-com) x f_w + mcom_w 
  Vector3 comw = T*link.robotPtr->links[link.index].com;
  Vector3 mw = cross(comw,fw) + mcomw;
  //convert to local frame
  Vector3 f,m;
  T.R.mulTranspose(fw,f);
  T.R.mulTranspose(mw,m);
  f.get(&out[0]);
  m.get(&out[3]);
}


SimRobotController::SimRobotController()
:index(-1),sim(NULL),controller(NULL)
{}

SimRobotController::~SimRobotController()
{}

RobotModel SimRobotController::model()
{
  if(!sim) return RobotModel();
  return sim->world.robot(index);
}

void SimRobotController::setRate(double dt)
{
  controller->controlTimeStep = dt;
}

double SimRobotController::getRate()
{
  return controller->controlTimeStep;
}

void SimRobotController::getCommandedConfig(vector<double>& q)
{
  Vector qv;
  controller->GetCommandedConfig(qv);
  q.resize(qv.n);
  qv.getCopy(&q[0]);
}

void SimRobotController::getCommandedVelocity(vector<double>& dq)
{
  Vector qv;
  controller->GetCommandedVelocity(qv);
  dq.resize(qv.n);
  qv.getCopy(&dq[0]);
}

void SimRobotController::getSensedConfig(vector<double>& q)
{
  Vector qv;
  controller->GetSensedConfig(qv);
  if(!qv.empty()) {
    q.resize(qv.n);
    qv.getCopy(&q[0]);
  }
}

void SimRobotController::getSensedVelocity(vector<double>& dq)
{
  Vector qv;
  controller->GetSensedVelocity(qv);
  if(!qv.empty()) {
    dq.resize(qv.n);
    qv.getCopy(&dq[0]);
  }
}

SimRobotSensor::SimRobotSensor(Robot* _robot,SensorBase* _sensor)
  :robot(_robot),sensor(_sensor)
{}

std::string SimRobotSensor::name()
{
  if(!sensor) return std::string();
  return sensor->name;
}
std::string SimRobotSensor::type()
{
  if(!sensor) return std::string();
  return sensor->Type();
}

std::vector<std::string> SimRobotSensor::measurementNames()
{
  std::vector<std::string> res;
  if(!sensor) return res;
  sensor->MeasurementNames(res);
  return res;
}

void SimRobotSensor::getMeasurements(std::vector<double>& out)
{
  out.resize(0);
  if(!sensor) return;
  sensor->GetMeasurements(out);
}

std::string SimRobotSensor::getSetting(const std::string& name)
{
  if(!sensor) return std::string();
  std::string val;
  if(!sensor->GetSetting(name,val)) throw PyException("Setting "+name+" not supported");
  return val;
}

void SimRobotSensor::setSetting(const std::string& name,const std::string& val)
{
  if(!sensor) return;
  if(!sensor->SetSetting(name,val)) throw PyException("Setting "+name+" not supported or value not formatted correctly");
}

void SimRobotSensor::drawGL()
{
  vector<double> measurements;
  drawGL(measurements);
}

void SimRobotSensor::drawGL(const std::vector<double>& measurements)
{
  if(!sensor) return;
  sensor->DrawGL(*robot,measurements);
}

void SimRobotSensor::kinematicSimulate(WorldModel& world,double dt)
{
  if(!sensor) return;
  sensor->SimulateKinematic(*robot,*worlds[world.index]->world);
  sensor->Advance(dt);
}

void SimRobotSensor::kinematicReset()
{
  if(!sensor) return;
  sensor->Reset();
}


SimRobotSensor SimRobotController::sensor(int sensorIndex)
{
  RobotSensors& sensors = controller->sensors;
  if(sensorIndex < 0 || sensorIndex >= (int)sensors.sensors.size())
    return SimRobotSensor(NULL,NULL);
  return SimRobotSensor(controller->robot,sensors.sensors[sensorIndex]);
}

SimRobotSensor SimRobotController::sensor(const char* name)
{
  RobotSensors& sensors = controller->sensors;
  SmartPointer<SensorBase> sensor = sensors.GetNamedSensor(name);
  if(sensor==NULL) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, sensor "<<name);
  }
  return SimRobotSensor(controller->robot,sensor);
}

std::vector<std::string> SimRobotController::commands()
{
  return controller->controller->Commands();
}

void SimRobotController::setManualMode(bool enabled)
{
  RobotController* c=sim->sim->robotControllers[index];
  MyController* mc=reinterpret_cast<MyController*>(c);
  if(mc)
    mc->override = enabled;
  else {
    if(enabled)
      throw PyException("Cannot enable manual mode, controller type incorrect");
  }
}

std::string SimRobotController::getControlType()
{
  std::vector<int> res;
  typedef std::vector<ActuatorCommand>::iterator it_ac;
  RobotMotorCommand& command = controller->command;
  int mode = -1;
  for(it_ac it = command.actuators.begin();
            it != command.actuators.end();
          ++it)
      if(mode == -1)
          mode = it->mode;
      else if(mode != it->mode)
          mode = -2;
  switch(mode)
  {
  case ActuatorCommand::OFF:
      return "off";
  case ActuatorCommand::TORQUE:
      return "torque";
  case ActuatorCommand::PID:
      return "PID";
  case ActuatorCommand::LOCKED_VELOCITY:
      return "locked_velocity";
  default:
      return "unknown";
  }
}


bool SimRobotController::sendCommand(const std::string& name,const std::string& args)
{
  return controller->controller->SendCommand(name,args);
}

std::string SimRobotController::getSetting(const std::string& name)
{
  std::string val;
  if(!controller->controller->GetSetting(name,val)) return "";
  return val;
}

bool SimRobotController::setSetting(const std::string& name,const std::string& val)
{
  return controller->controller->SetSetting(name,val);
}

void EnablePathControl(RobotController* c)
{
  PolynomialPathController* pc = GetPathController(c);
  MyController* mc=dynamic_cast<MyController*>(c);
  if(pc->path.elements.empty() || mc->override) {
    Config q;
    if(mc->GetCommandedConfig(q)) {
      pc->SetConstant(q);
    }
    else {
      if(mc->GetSensedConfig(q)) {
        pc->SetConstant(q);
      }
      else {
                LOG4CXX_ERROR(KrisLibrary::logger(),"First simulation cycle: the path controller needs to read from the encoders before motion commands can be issued\n");
      }
    }
  }
  mc->override = false;
}

void SimRobotController::setMilestone(const vector<double>& q)
{
  if(controller->robot->links.size() != q.size()) {
    throw PyException("Invalid size of configuration");
  }
  EnablePathControl(sim->sim->robotControllers[index]);
  Config qv(controller->robot->links.size(),&q[0]);
  stringstream ss;
  ss<<qv;
  controller->controller->SendCommand("set_q",ss.str());
}

void SimRobotController::setMilestone(const vector<double>& q,const vector<double>& dq)
{
  if(controller->robot->links.size() != q.size()) {
    throw PyException("Invalid size of configuration");
  }
  if(controller->robot->links.size() != dq.size()) {
    throw PyException("Invalid size of velocity");
  }
  EnablePathControl(sim->sim->robotControllers[index]);
  Config qv(controller->robot->links.size(),&q[0]);
  Config dqv(controller->robot->links.size(),&dq[0]);
  stringstream ss;
  ss<<qv<<"\t"<<dqv;
  controller->controller->SendCommand("set_qv",ss.str());
}

void SimRobotController::addMilestone(const vector<double>& q)
{
  if(controller->robot->links.size() != q.size()) {
    throw PyException("Invalid size of configuration");
  }
  EnablePathControl(sim->sim->robotControllers[index]);
  Config qv(controller->robot->links.size(),&q[0]);
  stringstream ss;
  ss<<qv;
  controller->controller->SendCommand("append_q",ss.str());
}

void SimRobotController::addMilestoneLinear(const vector<double>& q)
{
  if(controller->robot->links.size() != q.size()) {
    throw PyException("Invalid size of configuration");
  }
  EnablePathControl(sim->sim->robotControllers[index]);
  Config qv(controller->robot->links.size(),&q[0]);
  stringstream ss;
  ss<<qv;
  controller->controller->SendCommand("append_q_linear",ss.str());
}

void SimRobotController::setLinear(const std::vector<double>& q,double dt)
{
  if(controller->robot->links.size() != q.size()) {
    throw PyException("Invalid size of configuration");
  }
  EnablePathControl(sim->sim->robotControllers[index]);
  PolynomialMotionQueue* mq = GetMotionQueue(controller->controller);
  mq->Cut(0);
  mq->AppendLinear(q,dt);
}
void SimRobotController::setCubic(const std::vector<double>& q,const std::vector<double>& v,double dt)
{
  if(controller->robot->links.size() != q.size()) {
    throw PyException("Invalid size of configuration");
  }
  if(controller->robot->links.size() != v.size()) {
    throw PyException("Invalid size of velocity");
  }
  EnablePathControl(sim->sim->robotControllers[index]);
  PolynomialMotionQueue* mq = GetMotionQueue(controller->controller);
  mq->Cut(0);
  mq->AppendCubic(q,v,dt);
}
void SimRobotController::addLinear(const std::vector<double>& q,double dt)
{
  if(controller->robot->links.size() != q.size()) {
    throw PyException("Invalid size of configuration");
  }
  EnablePathControl(sim->sim->robotControllers[index]);
  PolynomialMotionQueue* mq = GetMotionQueue(controller->controller);
  mq->AppendLinear(q,dt);
}

void SimRobotController::addCubic(const std::vector<double>& q,const std::vector<double>& v,double dt)
{
  if(controller->robot->links.size() != q.size()) {
    throw PyException("Invalid size of configuration");
  }
  if(controller->robot->links.size() != v.size()) {
    throw PyException("Invalid size of velocity");
  }
  EnablePathControl(sim->sim->robotControllers[index]);
  PolynomialMotionQueue* mq = GetMotionQueue(controller->controller);
  mq->AppendCubic(q,v,dt);
}

void SimRobotController::addMilestone(const vector<double>& q,const vector<double>& dq)
{
  if(controller->robot->links.size() != q.size()) {
    throw PyException("Invalid size of configuration");
  }
  if(controller->robot->links.size() != dq.size()) {
    throw PyException("Invalid size of velocity");
  }
  EnablePathControl(sim->sim->robotControllers[index]);
  Config qv(controller->robot->links.size(),&q[0]);
  Config dqv(controller->robot->links.size(),&dq[0]);
  stringstream ss;
  ss<<qv<<"\t"<<dqv;
  controller->controller->SendCommand("append_qv",ss.str());
}

void SimRobotController::setVelocity(const vector<double>& dq,double dt)
{
  if(controller->robot->links.size() != dq.size()) {
    throw PyException("Invalid size of velocity");
  }
  EnablePathControl(sim->sim->robotControllers[index]);
  Config qv(controller->robot->links.size(),&dq[0]);
  stringstream ss;
  ss<<dt<<"\t"<<qv;
  controller->controller->SendCommand("set_tv",ss.str());
}

double SimRobotController::remainingTime() const
{
  PolynomialMotionQueue* mq = GetMotionQueue(controller->controller);
  return mq->TimeRemaining();
}


void SimRobotController::setTorque(const std::vector<double>& t)
{
  RobotMotorCommand& command = controller->command;
  if(t.size() != command.actuators.size()) {
    throw PyException("Invalid command size, must be equal to driver size");
  }
  for(size_t i=0;i<command.actuators.size();i++) {
    command.actuators[i].SetTorque(t[i]);
  }
  RobotController* c=sim->sim->robotControllers[index];
  MyController* mc=dynamic_cast<MyController*>(c);
  if(!mc) {
    throw PyException("Not using the default manual override controller");
  }
  mc->override = true;
}

void SimRobotController::setPIDCommand(const std::vector<double>& qdes,const std::vector<double>& dqdes)
{
  RobotMotorCommand& command = controller->command;
  Robot* robot=controller->robot;
  if(qdes.size() != command.actuators.size() || dqdes.size() != command.actuators.size()) {
    if(qdes.size() != robot->links.size() || dqdes.size() != robot->links.size())
      throw PyException("Invalid command sizes");
    for(size_t i=0;i<qdes.size();i++) {
      robot->q(i) = qdes[i];
      robot->dq(i) = dqdes[i];
    }
    for(size_t i=0;i<command.actuators.size();i++) {
      command.actuators[i].SetPID(robot->GetDriverValue(i),robot->GetDriverVelocity(i),command.actuators[i].iterm);
    }
  }
  else {
    for(size_t i=0;i<command.actuators.size();i++) {
      command.actuators[i].SetPID(qdes[i],dqdes[i],command.actuators[i].iterm);
    }
  }
  RobotController* c=sim->sim->robotControllers[index];
  MyController* mc=dynamic_cast<MyController*>(c);
  if(!mc) {
    throw PyException("Not using the default manual override controller");
  }
  mc->override = true;
}

void SimRobotController::setPIDCommand(const std::vector<double>& qdes,const std::vector<double>& dqdes,const std::vector<double>& tfeedforward)
{
  setPIDCommand(qdes,dqdes);
  RobotMotorCommand& command = controller->command;
  //Robot* robot=sim->sim->controlSimulators[index];
  if(tfeedforward.size() != command.actuators.size())
     throw PyException("Invalid command sizes");
  for(size_t i=0;i<command.actuators.size();i++) {
    command.actuators[i].torque = tfeedforward[i];
  }
}



void SimRobotController::setPIDGains(const std::vector<double>& kP,const std::vector<double>& kI,const std::vector<double>& kD)
{
  RobotMotorCommand& command = controller->command;
  if(kP.size() != command.actuators.size() || kI.size() != command.actuators.size() || kD.size() != command.actuators.size()) {
    throw PyException("Invalid gain sizes");
  }
  for(size_t i=0;i<kP.size();i++) {
    command.actuators[i].kP = kP[i];
    command.actuators[i].kI = kI[i];
    command.actuators[i].kD = kD[i];
  }
}

void SimRobotController::getPIDGains(std::vector<double>& kPout,std::vector<double>& kIout,std::vector<double>& kDout)
{
  RobotMotorCommand& command = controller->command;
  int size = command.actuators.size();
  kPout.resize(size, 0.0);
  kIout.resize(size, 0.0);
  kDout.resize(size, 0.0);
  for(size_t i=0;i<command.actuators.size();i++) {
    kPout[i] = command.actuators[i].kP;
    kIout[i] = command.actuators[i].kI;
    kDout[i] = command.actuators[i].kD;
  }
}








/***************************  VISUALIZATION CODE ***************************************/


bool Viewport::fromJson(const std::string& str)
{
  AnyCollection coll;
  std::stringstream ss(str);
  ss>>coll;
  if(!ss) return false;
  if(!coll["perspective"].as(perspective)) return false;
  if(!coll["scale"].as(scale)) return false;
  if(!coll["x"].as(x)) return false;
  if(!coll["y"].as(y)) return false;
  if(!coll["w"].as(w)) return false;
  if(!coll["h"].as(h)) return false;
  if(!coll["n"].as(n)) return false;
  if(!coll["f"].as(f)) return false;
  if(!coll["xform"].asvector(xform)) return false;
  if(xform.size() != 16) return false;
  return true;
}

void Viewport::setModelviewMatrix(const double M[16])
{
  xform.resize(16);
  copy(&M[0],&M[0]+16,xform.begin());
}

void Viewport::setRigidTransform(const double R[9],const double t[3])
{
  RigidTransform T;
  T.R.set(R);
  T.t.set(t);
  Matrix4 m(T);
  xform.resize(16);
  m.get(&xform[0]);
}

void Viewport::getRigidTransform(double out[9],double out2[3])
{
  Matrix4 m;
  m.set(&xform[0]);
  RigidTransform T(m);
  T.R.get(out);
  T.t.get(out2);
}


std::string Viewport::toJson() const
{
  AnyCollection coll;
  coll["perspective"] = perspective;
  coll["scale"] = scale;
  coll["x"] = x;
  coll["y"] = y;
  coll["w"] = w;
  coll["h"] = h;
  coll["n"] = n;
  coll["f"] = f;
  coll["xform"].resize(16);
  for(int i=0;i<16;i++)
    coll["xform"][i] = xform[i];
  std::stringstream ss;
  ss<<coll;
  return ss.str();
}

Widget::Widget()
{
  index = createWidget();
}

Widget::~Widget()
{
  derefWidget(index);
}

Camera::Viewport GetCameraViewport(const Viewport& viewport)
{
  Camera::Viewport vp;
  vp.x = viewport.x;
  vp.y = viewport.y;
  vp.w = viewport.w;
  vp.h = viewport.h;
  vp.n = viewport.n;
  vp.f = viewport.f;
  vp.perspective = viewport.perspective;
  vp.scale = viewport.scale;
  Assert(viewport.xform.size()==16);
  vp.xform.set(Matrix4(&viewport.xform[0]));
  return vp;
}

bool Widget::hover(int x,int y,const Viewport& viewport)
{
  double distance = Inf;
  Camera::Viewport vp = GetCameraViewport(viewport);
  Assert(widgets[index].widget != NULL);
  bool res=widgets[index].widget->Hover(x,y,vp,distance);
  if(res) widgets[index].widget->SetHighlight(true);
  else widgets[index].widget->SetHighlight(false);
  return res;
}

bool Widget::beginDrag(int x,int y,const Viewport& viewport)
{
  double distance = Inf;
  Camera::Viewport vp = GetCameraViewport(viewport);
  bool res=widgets[index].widget->BeginDrag(x,y,vp,distance);
  if(res) widgets[index].widget->SetFocus(true);
  else widgets[index].widget->SetFocus(false);
  return res;
}

void Widget::drag(int dx,int dy,const Viewport& viewport)
{
  Camera::Viewport vp = GetCameraViewport(viewport);
  widgets[index].widget->Drag(dx,dy,vp);
}

void Widget::endDrag()
{
  widgets[index].widget->EndDrag();
  widgets[index].widget->SetFocus(false);
}

void Widget::keypress(char c)
{
  widgets[index].widget->Keypress(c);
}

void Widget::drawGL(const Viewport& viewport)
{
  Camera::Viewport vp = GetCameraViewport(viewport);
  widgets[index].widget->DrawGL(vp);
  widgets[index].widget->requestRedraw = false;
}

void Widget::idle()
{
  widgets[index].widget->Idle();
}

bool Widget::wantsRedraw()
{
  return widgets[index].widget->requestRedraw;
}

bool Widget::hasHighlight()
{
  return widgets[index].widget->hasHighlight;
}

bool Widget::hasFocus()
{
  return widgets[index].widget->hasFocus;
}

WidgetSet::WidgetSet()
  :Widget()
{
  widgets[index].widget = new GLDraw::WidgetSet;
}

void WidgetSet::add(const Widget& subwidget)
{
  GLDraw::WidgetSet* ws=dynamic_cast<GLDraw::WidgetSet*>(&*widgets[index].widget);
  ws->widgets.push_back(widgets[subwidget.index].widget);
  ws->widgetEnabled.push_back(true);
  refWidget(subwidget.index);
}
void WidgetSet::remove(const Widget& subwidget)
{
  GLDraw::WidgetSet* ws=dynamic_cast<GLDraw::WidgetSet*>(&*widgets[index].widget);
  for(size_t i=0;i<ws->widgets.size();i++)
    if(ws->widgets[i] == widgets[subwidget.index].widget) {
      //delete it
      ws->widgets.erase(ws->widgets.begin()+i);
      ws->widgetEnabled.erase(ws->widgetEnabled.begin()+i);
      if(ws->activeWidget == widgets[subwidget.index].widget)
	ws->activeWidget = NULL;
      if(ws->closestWidget == widgets[subwidget.index].widget)
	ws->closestWidget = NULL;
      derefWidget(subwidget.index);
      if(widgets[subwidget.index].widget == NULL) 
	return;
      i--;
    }
}

void WidgetSet::enable(const Widget& subwidget,bool enabled)
{
  GLDraw::WidgetSet* ws=dynamic_cast<GLDraw::WidgetSet*>(&*widgets[index].widget);
  for(size_t i=0;i<ws->widgets.size();i++)
    if(ws->widgets[i] == widgets[subwidget.index].widget) {
      if(ws->activeWidget == widgets[subwidget.index].widget)
	ws->activeWidget = NULL;
      if(ws->closestWidget == widgets[subwidget.index].widget)
	ws->closestWidget = NULL;
      ws->widgetEnabled[i] = enabled;
    }
}

PointPoser::PointPoser()
  :Widget()
{
  widgets[index].widget = new GLDraw::TransformWidget;
  GLDraw::TransformWidget* tw=dynamic_cast<GLDraw::TransformWidget*>(&*widgets[index].widget);
  tw->enableRotation = false;
}

void PointPoser::set(const double t[3])
{
  GLDraw::TransformWidget* tw=dynamic_cast<GLDraw::TransformWidget*>(&*widgets[index].widget);
  tw->T.t.set(t);
}

void PointPoser::get(double out[3])
{
  GLDraw::TransformWidget* tw=dynamic_cast<GLDraw::TransformWidget*>(&*widgets[index].widget);
  tw->T.t.get(out);
}


void PointPoser::setAxes(const double R[9])
{
  GLDraw::TransformWidget* tw=dynamic_cast<GLDraw::TransformWidget*>(&*widgets[index].widget);
  tw->T.R.set(R);
}


TransformPoser::TransformPoser()
  :Widget()
{
  widgets[index].widget = new GLDraw::TransformWidget;
}

void TransformPoser::set(const double R[9],const double t[3])
{
  GLDraw::TransformWidget* tw=dynamic_cast<GLDraw::TransformWidget*>(&*widgets[index].widget);
  tw->T.R.set(R);
  tw->T.t.set(t);
}

void TransformPoser::get(double out[9],double out2[3])
{
  GLDraw::TransformWidget* tw=dynamic_cast<GLDraw::TransformWidget*>(&*widgets[index].widget);
  tw->T.R.get(out);
  tw->T.t.get(out2);
}

void TransformPoser::enableTranslation(bool enable)
{
  GLDraw::TransformWidget* tw=dynamic_cast<GLDraw::TransformWidget*>(&*widgets[index].widget);
  tw->enableTranslation = enable;
}

void TransformPoser::enableRotation(bool enable)
{
  GLDraw::TransformWidget* tw=dynamic_cast<GLDraw::TransformWidget*>(&*widgets[index].widget);
  tw->enableRotation = enable;
}

ObjectPoser::ObjectPoser(RigidObjectModel& object)
  :Widget()
{
  RobotWorld& world = *worlds[object.world]->world;
  RigidObject* obj = world.rigidObjects[object.index];
  widgets[index].widget = new RigidObjectPoseWidget(obj);
}

void ObjectPoser::set(const double R[9],const double t[3])
{
  RigidObjectPoseWidget* tw=dynamic_cast<RigidObjectPoseWidget*>(&*widgets[index].widget);
  RigidTransform T;
  T.R.set(R);
  T.t.set(t);
  tw->SetPose(T);
}

void ObjectPoser::get(double out[9],double out2[3])
{
  RigidObjectPoseWidget* tw=dynamic_cast<RigidObjectPoseWidget*>(&*widgets[index].widget);
  RigidTransform T = tw->Pose();
  T.R.get(out);
  T.t.get(out2);
}

RobotPoser::RobotPoser(RobotModel& robot)
{
  Assert(worlds[robot.world]->world != NULL);
  RobotWorld& world = *worlds[robot.world]->world;
  Assert(robot.index >= 0 && robot.index < world.robots.size());
  Robot* rob = world.robots[robot.index];
  ViewRobot* view = &world.robotViews[robot.index];
  Assert(rob != NULL);
  Assert(view != NULL);
  widgets[index].widget = new RobotPoseWidget(rob,view);
}

void RobotPoser::setActiveDofs(const std::vector<int>& dofs)
{
  RobotPoseWidget* tw=dynamic_cast<RobotPoseWidget*>(&*widgets[index].widget);
  tw->SetActiveDofs(dofs);
}

void RobotPoser::set(const std::vector<double>& q)
{
  RobotPoseWidget* tw=dynamic_cast<RobotPoseWidget*>(&*widgets[index].widget);
  tw->SetPose(Config(q));
}

void RobotPoser::get(std::vector<double>& out)
{
  RobotPoseWidget* tw=dynamic_cast<RobotPoseWidget*>(&*widgets[index].widget);
  out.resize(tw->Pose().size());
  tw->Pose().getCopy(&out[0]);
}

void RobotPoser::getConditioned(const std::vector<double>& qref,std::vector<double>& out)
{
  RobotPoseWidget* tw=dynamic_cast<RobotPoseWidget*>(&*widgets[index].widget);
  out.resize(tw->Pose().size());
  tw->Pose_Conditioned(Config(qref)).getCopy(&out[0]);
}







/***************************  STABILITY TESTING CODE ***************************************/

void setFrictionConeApproximationEdges(int numEdges)
{
  if(numEdges < 3) throw PyException("Invalid number of friction cone approximation edges, must be at least 3");
  gStabilityNumFCEdges = numEdges;
}

void Convert(const std::vector<std::vector<double > >& contacts,vector<ContactPoint>& cps)
{
  cps.resize(contacts.size());
  for(size_t i=0;i<contacts.size();i++) {
    if(contacts[i].size() != 7) throw PyException("Invalid size of contact point, must be in the format (x,y,z,nx,ny,nz,kFriction)");
    if(contacts[i][6] < 0) throw PyException("Invalid contact point, negative friction coefficient");
    cps[i].x.set(contacts[i][0],contacts[i][1],contacts[i][2]);
    cps[i].n.set(contacts[i][3],contacts[i][4],contacts[i][5]);
    cps[i].kFriction = contacts[i][6];
  }
}

void Convert(const std::vector<std::vector<double > >& contacts,vector<ContactPoint2D>& cps)
{
  cps.resize(contacts.size());
  for(size_t i=0;i<contacts.size();i++) {
    if(contacts[i].size() != 4) throw PyException("Invalid size of contact point, must be in the format (x,y,angle,kFriction)");
    if(contacts[i][3] < 0) throw PyException("Invalid contact point, negative friction coefficient");
    cps[i].x.set(contacts[i][0],contacts[i][1]);
    cps[i].n.set(Cos(contacts[i][2]),Sin(contacts[i][2]));
    cps[i].kFriction = contacts[i][3];
  }
}

void Convert(const std::vector<std::vector<double > >& contactPositions,const std::vector<std::vector<double > >& frictionCones,vector<CustomContactPoint>& cps)
{
  cps.resize(contactPositions.size());
  for(size_t i=0;i<contactPositions.size();i++) {
    if(contactPositions[i].size() != 3) throw PyException("contactPositions must be a list of 3-lists");
    if(frictionCones[i].size() % 4 != 0) throw PyException("frictionCones elements must be a multiple of 4");
    cps[i].x.set(contactPositions[i][0],contactPositions[i][1],contactPositions[i][2]);
    cps[i].kFriction = 0.0;
    cps[i].forceMatrix.resize(frictionCones[i].size()/4,3);
    cps[i].forceOffset.resize(frictionCones[i].size()/4);
    int k=0;
    for(int j=0;j<cps[i].forceMatrix.m;j++,k+=4) {
      cps[i].forceMatrix(j,0) = frictionCones[i][k];
      cps[i].forceMatrix(j,1) = frictionCones[i][k+1];
      cps[i].forceMatrix(j,2) = frictionCones[i][k+2];
      cps[i].forceOffset[j] = frictionCones[i][k+3];
    }
  }
}

void Convert(const std::vector<std::vector<double > >& contactPositions,const std::vector<std::vector<double > >& frictionCones,vector<CustomContactPoint2D>& cps)
{
  cps.resize(contactPositions.size());
  for(size_t i=0;i<contactPositions.size();i++) {
    if(contactPositions[i].size() != 2) throw PyException("contactPositions must be a list of 2-lists");
    if(frictionCones[i].size() % 3 != 0) throw PyException("frictionCones elements must be a multiple of 3");
    cps[i].x.set(contactPositions[i][0],contactPositions[i][1]);
    cps[i].kFriction = 0.0;
    cps[i].forceMatrix.resize(frictionCones[i].size()/3,2);
    cps[i].forceOffset.resize(frictionCones[i].size()/3);
    int k=0;
    for(int j=0;j<cps[i].forceMatrix.m;j++,k+=3) {
      cps[i].forceMatrix(j,0) = frictionCones[i][k];
      cps[i].forceMatrix(j,1) = frictionCones[i][k+1];
      cps[i].forceOffset[j] = frictionCones[i][k+2];
    }
  }
}

bool forceClosure(const std::vector<std::vector<double > >& contacts)
{
  vector<ContactPoint> cps;
  Convert(contacts,cps);
  return TestForceClosure(cps,gStabilityNumFCEdges);
}

bool forceClosure(const std::vector<std::vector<double> >& contactPositions,const std::vector<std::vector<double > >& frictionCones)
{
  vector<CustomContactPoint> cps;
  Convert(contactPositions,frictionCones,cps);
  return TestForceClosure(cps);
}

bool forceClosure2D(const std::vector<std::vector<double > >& contacts)
{
  vector<ContactPoint2D> cps;
  Convert(contacts,cps);
  return TestForceClosure(cps);
}

bool forceClosure2D(const std::vector<std::vector<double > >& contactPositions,const std::vector<std::vector<double> >& frictionCones)
{
  vector<CustomContactPoint2D> cps;
  Convert(contactPositions,frictionCones,cps);
  return TestForceClosure(cps);
}

PyObject* ToPy2(const vector<Vector3>& x)
{
  PyObject* ls = PyList_New(x.size());
  PyObject* pItem;
  if(ls == NULL) {
    goto fail;
  }

  for(size_t i = 0; i < x.size(); i++) {
    pItem = ::ToPy(x[i]);
    if(pItem == NULL)
      goto fail;
    PyList_SetItem(ls, i, pItem);
  }

  return ls;

  fail:
  Py_XDECREF(ls);
  throw PyException("Failure during ToPy");
  return NULL;
}

PyObject* ToPy2(const vector<Vector2>& x)
{
  PyObject* ls = PyList_New(x.size());
  PyObject* pItem;
  if(ls == NULL) {
    goto fail;
  }

  for(size_t i = 0; i < x.size(); i++) {
    pItem = ::ToPy(x[i]);
    if(pItem == NULL)
      goto fail;
    PyList_SetItem(ls, i, pItem);
  }

  return ls;

  fail:
  Py_XDECREF(ls);
  throw PyException("Failure during ToPy");
  return NULL;
}


PyObject* comEquilibrium(const std::vector<std::vector<double> >& contacts,const vector<double>& fext,PyObject* com)
{
  if(fext.size() != 3) throw PyException("Invalid external force, must be a 3-list");
  vector<ContactPoint> cps;
  Convert(contacts,cps);
  if(com == Py_None) {
    //test all 
    bool res=TestAnyCOMEquilibrium(cps,Vector3(fext[0],fext[1],fext[2]),gStabilityNumFCEdges);
    if(res) 
      Py_RETURN_TRUE;
    else
      Py_RETURN_FALSE;
  }
  Vector3 vcom;
  if(!FromPy(com,vcom)) throw PyException("Could not convert COM to a 3-list of floats");
  vector<Vector3> forces(cps.size());
  if(TestCOMEquilibrium(cps,Vector3(fext[0],fext[1],fext[2]),gStabilityNumFCEdges,vcom,forces)) {
    return ToPy2(forces);
  }
  Py_RETURN_NONE;
}

PyObject* comEquilibrium(const std::vector<std::vector<double> >& contactPositions,const std::vector<std::vector<double> >& frictionCones,const vector<double>& fext,PyObject* com)
{
  if(fext.size() != 3) throw PyException("Invalid external force, must be a 3-list");
  vector<CustomContactPoint> cps;
  Convert(contactPositions,frictionCones,cps);
  if(com == Py_None) {
    //test all 
    bool res=TestAnyCOMEquilibrium(cps,Vector3(fext[0],fext[1],fext[2]));
    if(res) 
      Py_RETURN_TRUE;
    else
      Py_RETURN_FALSE;
  }
  Vector3 vcom;
  if(!FromPy(com,vcom)) throw PyException("Could not convert COM to a 3-list of floats");
  vector<Vector3> forces(cps.size());
  if(TestCOMEquilibrium(cps,Vector3(fext[0],fext[1],fext[2]),vcom,forces)) {
    return ToPy2(forces);
  }
  Py_RETURN_NONE;
}


PyObject* comEquilibrium2D(const std::vector<std::vector<double> >& contacts,const vector<double>& fext,PyObject* com)
{
  if(fext.size() != 2) throw PyException("Invalid external force, must be a 2-list");
  vector<ContactPoint2D> cps;
  Convert(contacts,cps);
  if(com == Py_None) {
    //test all 
    bool res=TestAnyCOMEquilibrium(cps,Vector2(fext[0],fext[1]));
    if(res) 
      Py_RETURN_TRUE;
    else
      Py_RETURN_FALSE;
  }
  Vector2 vcom;
  if(!FromPy(com,vcom)) throw PyException("Could not convert COM to a 2-list of floats");
  vector<Vector2> forces(cps.size());
  if(TestCOMEquilibrium(cps,Vector2(fext[0],fext[1]),vcom,forces)) {
    return ToPy2(forces);
  }
  Py_RETURN_NONE;
}

PyObject* comEquilibrium2D(const std::vector<std::vector<double> >& contactPositions,const std::vector<std::vector<double> >& frictionCones,const vector<double>& fext,PyObject* com)
{
  if(fext.size() != 2) throw PyException("Invalid external force, must be a 2-list");
  vector<CustomContactPoint2D> cps;
  Convert(contactPositions,frictionCones,cps);
  if(com == Py_None) {
    //test all 
    bool res=TestAnyCOMEquilibrium(cps,Vector2(fext[0],fext[1]));
    if(res) 
      Py_RETURN_TRUE;
    else
      Py_RETURN_FALSE;
  }
  Vector2 vcom;
  if(!FromPy(com,vcom)) throw PyException("Could not convert COM to a 2-list of floats");
  vector<Vector2> forces(cps.size());
  if(TestCOMEquilibrium(cps,Vector2(fext[0],fext[1]),vcom,forces)) {
    return ToPy2(forces);
  }
  Py_RETURN_NONE;
}


PyObject* supportPolygon(const std::vector<std::vector<double> >& contacts)
{
  vector<ContactPoint> cps;
  Convert(contacts,cps);
  SupportPolygon sp;
  if(!sp.Set(cps,Vector3(0,0,-1),gStabilityNumFCEdges)) throw PyException("Numerical problem calculating support polygon?");
  if(sp.vertices.empty()) {
    //empty support polygon
    PyObject* res = PyList_New(1);
    PyObject* invalid = Py_BuildValue("[fff]",0.0,0.0,-1.0);
    PyList_SetItem(res,0,invalid);
    return res;
  }
  PyObject* res = PyList_New(sp.planes.size());
  for(size_t i=0;i<sp.planes.size();i++) {
    PyObject* plane = Py_BuildValue("[fff]",sp.planes[i].normal.x,sp.planes[i].normal.y,sp.planes[i].offset);
    PyList_SetItem(res,i,plane);
  }
  return res;
}

/// A fancy version of the normal supportPolygon test.
/// contactPositions is a list of 3-lists giving the contact point positions. 
/// The i'th element in the list frictionCones has length (k*4), and gives the contact
/// force constraints (ax,ay,az,b) where ax*fx+ay*fy+az*fz <= b limits the contact force
/// (fx,fy,fz) at the i'th contact.  Each of the k 4-tuples is laid out sequentially per-contact.
/// 
/// The return value is a list of 3-tuples giving the sorted plane boundaries of the polygon.
/// The format of a plane is (nx,ny,ofs) where (nx,ny) are the outward facing normals, and
/// ofs is the offset from 0.  In other words to test stability of a com [x,y], you can test
/// whether dot([nx,ny],[x,y]) <= ofs  for all planes.
PyObject* supportPolygon(const std::vector<std::vector<double> >& contactPositions,const std::vector<std::vector<double> >& frictionCones)
{
  vector<CustomContactPoint> cps;
  Convert(contactPositions,frictionCones,cps);
  SupportPolygon sp;
  if(!sp.Set(cps,Vector3(0,0,-1))) throw PyException("Numerical problem calculating support polygon?");
  if(sp.vertices.empty()) {
    //empty support polygon
    PyObject* res = PyList_New(1);
    PyObject* invalid = Py_BuildValue("[fff]",0.0,0.0,-1.0);
    PyList_SetItem(res,0,invalid);
    return res;
  }
  PyObject* res = PyList_New(sp.planes.size());
  for(size_t i=0;i<sp.planes.size();i++) {
    PyObject* plane = Py_BuildValue("[fff]",sp.planes[i].normal.x,sp.planes[i].normal.y,sp.planes[i].offset);
    PyList_SetItem(res,i,plane);
  }
  return res;
}


/// Calculates the support polygon (interval)  for a given set of contacts and a downward
/// external force (0,-g). A contact point is given by a list of 4 floats, [x,y,theta,k] as usual.
/// 
/// The return value is a 2-tuple giving the min / max extents of the support polygon.
/// If they are both infinite, the support polygon is empty.
PyObject* supportPolygon2D(const std::vector<std::vector<double> >& contacts)
{
  throw PyException("2D support polygons not implemented yet");
}

PyObject* supportPolygon2D(const std::vector<std::vector<double> >& contacts,const std::vector<std::vector<double> >& frictionCones)
{
  throw PyException("2D support polygons not implemented yet");
}

PyObject* equilibriumTorques(const RobotModel& robot,const std::vector<std::vector<double> >& contacts,const std::vector<int>& links,const std::vector<double>& fext,const std::vector<double>& internalTorques,double norm)
{
  if(robot.robot == NULL) throw PyException("Called with empty robot");
  if(fext.size() != 3) throw PyException("Invalid external force, must be a 3-list");
  if(!internalTorques.empty()) {
    if(internalTorques.size() != robot.robot->links.size())
      throw PyException("Invalid number of internal torques specified");
  }
  vector<ContactPoint> cps;
  CustomContactFormation formation;
  Convert(contacts,cps);
  formation.links = links;
  formation.contacts.resize(cps.size());
  for(size_t i=0;i<cps.size();i++)
    formation.contacts[i].set(cps[i],gStabilityNumFCEdges);
  TorqueSolver ts(*robot.robot,formation);
  ts.SetGravity(Vector3(fext[0],fext[1],fext[2]));
  ts.SetNorm((norm == 0? Inf : norm));
  bool weighted = true;
  ts.Init(weighted);
  if(!internalTorques.empty())
    ts.internalForces.copy(internalTorques);
  if(!ts.Solve()) {
    Py_RETURN_NONE;
  }
  return Py_BuildValue("(NN)",ToPy(ts.t),ToPy(ts.f));
}

PyObject* equilibriumTorques(const RobotModel& robot,const std::vector<std::vector<double> >& contacts,const std::vector<int>& links,const vector<double>& fext,double norm)
{
  vector<double> internalTorques;
  return ::equilibriumTorques(robot,contacts,links,fext,internalTorques,norm);
}




/*************************** IO CODE ***************************************/

bool SubscribeToStream(Geometry3D& g,const char* protocol,const char* name,const char* type)
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(g.geomPtr);
  if(0==strcmp(protocol,"ros")) {
    if(0==strcmp(type,""))
      type = "PointCloud";
    if(0 == strcmp(type,"PointCloud")) {
      if(!g.isStandalone()) {
  RobotWorld& world=*worlds[g.world]->world;
  GetManagedGeometry(world,g.id).RemoveFromCache();
  return GetManagedGeometry(world,g.id).Load((string("ros:PointCloud2//")+string(name)).c_str());
      }
      LOG4CXX_WARN(KrisLibrary::logger(),"Warning, attaching to a ROS stream without a ManagedGeometry.\n");
      LOG4CXX_INFO(KrisLibrary::logger(),"You will not be able to automatically get updates from ROS.\n");
      if(!geom) 
        geom = new AnyCollisionGeometry3D();
      (*geom) = AnyCollisionGeometry3D(Meshing::PointCloud3D());
      return ROSSubscribePointCloud(geom->AsPointCloud(),name);
      //TODO: update ROS, update the appearance every time the point cloud changes
    }
    else {
      throw PyException("AttachToStream(Geometry3D): Unsupported type argument");
      return false;
    }
  }
  else {
    throw PyException("AttachToStream(Geometry3D): Unsupported protocol argument");
    return false;
  }
}

bool DetachFromStream(const char* protocol,const char* name)
{
  if(0==strcmp(protocol,"ros")) {
    return ROSDetach(name);
  }
  else {
    throw PyException("DetachFromStream: Unsupported protocol argument");
    return false;
  }
}

bool ProcessStreams(const char* protocol)
{
  if((0==strcmp(protocol,"all")&&ROSInitialized()) || 0==strcmp(protocol,"ros"))
    if(ROSSubscribeUpdate()) return true;
  return false;
}

bool WaitForStream(const char* protocol,const char* name,double timeout)
{
  if(0==strcmp(protocol,"ros")) {
    return ROSWaitForUpdate(name,timeout);
  }
  return false;
}

/*
bool PublishToStream(const Vector& x,const char* protocol,const char* name,const char* type);
bool PublishToStream(const RobotModel& robot,const char* protocol,const char* name,const char* type);
bool PublishToStream(const WorldModel& world,const char* protocol,const char* name,const char* type);
bool PublishToStream(const Geometry3D& g,const char* protocol,const char* name,const char* type);
*/

///Exports the WorldModel to a JSON string ready for use in Three.js
std::string ThreeJSGetScene(const WorldModel& w)
{
  if(w.index < 0) return "{}";
  RobotWorld& world = *worlds[w.index]->world;

  AnyCollection obj;
  ThreeJSExport(world,obj);
  std::ostringstream stream;
  stream<<obj;
  return stream.str();
}

///Exports the WorldModel to a JSON string ready for use in Three.js
std::string ThreeJSGetTransforms(const WorldModel& w)
{
  if(w.index < 0) return "{}";
   RobotWorld& world = *worlds[w.index]->world;

   AnyCollection obj;
   ThreeJSExportTransforms(world,obj);
   std::ostringstream stream;
   stream<<obj;
   return stream.str();
}

