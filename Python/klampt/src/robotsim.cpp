#include <vector>
#include "pyerr.h"
#include "robotsim.h"
#include "widget.h"
#include "Control/Command.h"
#include "Control/PathController.h"
#include "Control/FeedforwardController.h"
#include "Control/LoggingController.h"
#include "Simulation/WorldSimulation.h"
#include "Modeling/Interpolate.h"
#include "IO/XmlWorld.h"
#include "IO/XmlODE.h"
#include "IO/ROS.h"
#include <KrisLibrary/robotics/NewtonEuler.h>
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
#include <fstream>
#ifndef WIN32
#include <unistd.h>
#endif //WIN32

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
  if(index < 0 || index >= (int)worlds.size())
    throw PyException("Invalid world index");
  if(!worlds[index])
    throw PyException("Invalid dereference");
  if(worlds[index]->refCount <= 0)
    throw PyException("Invalid dereference");

  worlds[index]->refCount--;
  //printf("Deref world %d: count %d\n",index,worlds[index]->refCount);
  if(worlds[index]->refCount == 0) {
    //printf("Deleting world %d\n",index);
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
  //printf("Ref world %d: count %d\n",index,worlds[index]->refCount);
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
    //printf("Creating widget %d, ref count %d\n",widgets.size()-1,1);
    return (int)(widgets.size()-1);
  }
  else {
    int index = widgetDeleteList.front();
    widgetDeleteList.erase(widgetDeleteList.begin());
    widgets[index].widget = NULL;
    widgets[index].refCount = 1;
    //printf("Creating widget %d, ref count %d\n",index,1);
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
  //printf("Deref widget %d: count %d\n",index,widgets[index].refCount);
  if(widgets[index].refCount == 0) {
    //printf("Deleting widget %d\n",index);
    widgets[index].widget = NULL;
    widgetDeleteList.push_back(index);
  }
}

void refWidget(int index)
{
  if(index < 0 || index >= (int)widgets.size())
    throw PyException("Invalid widget index");
  widgets[index].refCount++;
  //printf("Ref widget %d: count %d\n",index,widgets[index].refCount);
}


ManagedGeometry& GetManagedGeometry(RobotWorld& world,int id)
{
  if(id < 0) {
    fprintf(stderr,"GetManagedGeometry(): Invalid ID: %d\n",id);
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
  fprintf(stderr,"GetManagedGeometry(): Invalid ID: %d\n",id);
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
inline PolynomialMotionQueue* GetMotionQueue(RobotController* controller)
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
  //printf("Copying PointCloud to geometry, %d points\n",(int)gpc.points.size());
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
    if(!LexicalCast<double>(items[i],properties[i-1])) return false;
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
  geomPtr = new SmartPointer<AnyCollisionGeometry3D>(*geom);
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
  *geom = *ggeom;
  geom->ClearCollisionData();
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
    printf("Geometry3D(): Freeing standalone geometry\n");
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
  SmartPointer<AnyCollisionGeometry3D> geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
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
  SmartPointer<AnyCollisionGeometry3D> geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);  
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

bool Geometry3D::attachToStream(const char* protocol,const char* name,const char* type)
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  if(0==strcmp(protocol,"ros")) {
    if(0==strcmp(type,""))
      type = "PointCloud";
    if(0 == strcmp(type,"PointCloud")) {
      if(!isStandalone()) {
	RobotWorld& world=*worlds[this->world]->world;
	GetManagedGeometry(world,id).RemoveFromCache();
	return GetManagedGeometry(world,id).Load((string("ros:PointCloud2//")+string(name)).c_str());
      }
      printf("Warning, attaching to a ROS stream without a ManagedGeometry.\n");
      printf("You will not be able to automatically get updates from ROS.\n");
      if(!geom) 
        geom = new AnyCollisionGeometry3D();
      (*geom) = AnyCollisionGeometry3D(Meshing::PointCloud3D());
      return ROSSubscribePointCloud(geom->AsPointCloud(),name);
      //TODO: update ROS, update the appearance every time the point cloud changes
    }
    else {
      throw PyException("Geometry3D::attachToStream: Unsupported type argument");
      return false;
    }
  }
  else {
    throw PyException("Geometry3D::attachToStream: Unsupported protocol argument");
    return false;
  }
}

bool Geometry3D::detachFromStream(const char* protocol,const char* name)
{
  if(0==strcmp(protocol,"ros")) {
    return ROSDetach(name);
  }
  else {
    throw PyException("Geometry3D::detachFromStream: Unsupported protocol argument");
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

void Geometry3D::translate(const double t[3])
{
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) return;
  RigidTransform T;
  T.R.setIdentity();
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
    //printf("Appearance(): Freeing standalone appearance for %p\n",this);
    *app = NULL;
  }
  else if(*app)
    //printf("Appearance(): Releasing reference to world appearance %d %d for %p\n",world,id,this);
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
  FatalError("Not implemented yet");
}
void Appearance::getColor(int primitive,float out[4])
{
  FatalError("Not implemented yet");
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

void Appearance::drawGL(Geometry3D& g)
{
  SmartPointer<GLDraw::GeometryAppearance>& app = *reinterpret_cast<SmartPointer<GLDraw::GeometryAppearance>*>(appearancePtr);
  SmartPointer<AnyCollisionGeometry3D>& geom = *reinterpret_cast<SmartPointer<AnyCollisionGeometry3D>*>(g.geomPtr);
  if(!app) return;
  if(!geom) return;
  if(app->geom) {
    if(app->geom != geom) {
      fprintf(stderr,"Appearance::drawGL(): performance warning, setting to a different geometry\n");
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

void PointCloud::setProperties(const vector<double>& vproperties)
{
  int n = numPoints();
  assert(vproperties.size() >= n*propertyNames.size());
  copy(vproperties.begin(),vproperties.begin()+propertyNames.size()*n,properties.begin());
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


WorldModel::WorldModel()
{
  index = createWorld();
}

WorldModel::WorldModel(int _index)
{
  index = _index;
  refWorld(index);
}

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

bool WorldModel::readFile(const char* fn)
{
  RobotWorld& world = *worlds[index]->world;

  const char* ext=FileExtension(fn);
  if(0==strcmp(ext,"rob") || 0==strcmp(ext,"urdf")) {
    if(world.LoadRobot(fn)<0) {
      printf("Error loading robot file %s\n",fn);
      return false;
    }
    if(gEnableCollisionInitialization) {
      world.robots.back()->InitCollisions();
      world.robots.back()->UpdateGeometry();
    }
  }
  else if(0==strcmp(ext,"env") || 0==strcmp(ext,"tri") || 0==strcmp(ext,"pcd")) {
    if(world.LoadTerrain(fn)<0) {
      printf("Error loading terrain file %s\n",fn);
      return false;
    }
    if(gEnableCollisionInitialization) world.terrains.back()->InitCollisions();
  }
  else if(0==strcmp(ext,"obj")) {
    if(world.LoadRigidObject(fn)<0) {
      printf("Error loading rigid object file %s\n",fn);
      return false;
    }
    if(gEnableCollisionInitialization) {
      world.rigidObjects.back()->InitCollisions();
      world.rigidObjects.back()->UpdateGeometry();
    }
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
      printf("Error opening or parsing world file %s\n",fn);
      return false;
    }
    if(gEnableCollisionInitialization) {
      world.InitCollisions();
      world.UpdateGeometry();
    }
    return true;
  }
  else {
    printf("Unknown file extension %s on file %s\n",ext,fn);
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
  if(gEnableCollisionInitialization) {
    world.robots.back()->InitCollisions();
    world.robots.back()->UpdateGeometry();
  }
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
  if(gEnableCollisionInitialization) {
    world.rigidObjects.back()->InitCollisions();
    world.rigidObjects.back()->UpdateGeometry();
  }
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

RobotModel RobotModelLink::getRobot()
{
  static bool warned = false;
  if(!warned) {
    fprintf(stderr,"RobotModelLink::getRobot() will be deprecated, please use robot() instead\n");
    warned = true;
  }
  return robot();
}

RobotModel RobotModelLink::robot()
{
  RobotModel r;
  r.world = world;
  r.index = robotIndex;
  r.robot = robotPtr;
  return r;
}

const char* RobotModelLink::getName()
{
  if(index < 0) return "";
  return robotPtr->linkNames[index].c_str();
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

int RobotModelLink::getID()
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

RobotModel RobotModelDriver::getRobot()
{
  static bool warned = false;
  if(!warned) {
    fprintf(stderr,"RobotModelDriver::getRobot() will be deprecated, please use robot() instead\n");
    warned = true;
  }
  return robot();
}

RobotModel RobotModelDriver::robot()
{
  RobotModel r;
  r.world = world;
  r.index = robotIndex;
  r.robot = robotPtr;
  return r;
}

const char* RobotModelDriver::getName()
{
  if(index < 0) return "";
  return robotPtr->driverNames[index].c_str();
}

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

const char* RobotModel::getName()
{
  RobotWorld& world = *worlds[this->world]->world;
  return world.robots[index]->name.c_str();
}

int RobotModel::getID()
{
  RobotWorld& world = *worlds[this->world]->world;
  return world.RobotID(index);
}

int RobotModel::numLinks()
{
  return robot->links.size();
}

RobotModelLink RobotModel::getLink(int linkindex)
{
  static bool warned = false;
  if(!warned) {
    fprintf(stderr,"RobotModel::getLink() will be deprecated, please use link() instead\n");
    warned = true;
  }
  return link(linkindex);
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

RobotModelLink RobotModel::getLink(const char* name)
{
  static bool warned = false;
  if(!warned) {
    fprintf(stderr,"RobotModel::getLink() will be deprecated, please use link() instead\n");
    warned = true;
  }
  return link(name);
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

RobotModelDriver RobotModel::getDriver(int driverindex)
{
  static bool warned = false;
  if(!warned) {
    fprintf(stderr,"RobotModel::getDriver() will be deprecated, please use driver() instead\n");
    warned = true;
  }
  return driver(driverindex);
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

RobotModelDriver RobotModel::getDriver(const char* name)
{
  static bool warned = false;
  if(!warned) {
    fprintf(stderr,"RobotModel::getDriver() will be deprecated, please use driver() instead\n");
    warned = true;
  }
  return driver(name);
}

RobotModelDriver RobotModel::driver(const char* name)
{
  string sname(name);
  for(size_t i=0;i<robot->driverNames.size();i++)
    if(sname == robot->driverNames[i]) {
      return getDriver((int)i);
    }
  RobotModelDriver link;
  link.world = this->world;
  link.robotPtr = robot;
  link.robotIndex = index;
  link.index = -1;
  return link;
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
  robot->q.copy(&q[0]);
  robot->UpdateFrames();
  robot->UpdateGeometry();
}

void RobotModel::setVelocity(const vector<double>& dq)
{
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
  robot->qMin.copy(&qmin[0]);
  robot->qMax.copy(&qmax[0]);
}

void RobotModel::getVelocityLimits(vector<double>& vmax)
{
  vmax.resize(robot->q.n);
  robot->velMax.getCopy(&vmax[0]);
}

void RobotModel::setVelocityLimits(const vector<double>& vmax)
{
  robot->velMax.copy(&vmax[0]);
}

void RobotModel::getAccelerationLimits(vector<double>& amax)
{
  amax.resize(robot->q.n);
  robot->accMax.getCopy(&amax[0]);
}

void RobotModel::setAccelerationLimits(const vector<double>& amax)
{
  robot->accMax.copy(&amax[0]);
}

void RobotModel::getTorqueLimits(vector<double>& tmax)
{
  tmax.resize(robot->q.n);
  robot->torqueMax.getCopy(&tmax[0]);
}

void RobotModel::setTorqueLimits(const vector<double>& tmax)
{
  robot->torqueMax.copy(&tmax[0]);
}

void RobotModel::setDOFPosition(int i,double qi)
{
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
  Vector va(a),vb(b);
  return Distance(*robot,va,vb,Inf);
}

void RobotModel::interpolate_deriv(const std::vector<double>& a,const std::vector<double>& b,std::vector<double>& dout)
{
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

const char* RigidObjectModel::getName()
{
  RobotWorld& world = *worlds[this->world]->world;
  return world.rigidObjects[index]->name.c_str();
}

int RigidObjectModel::getID()
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
  printf("Copying ptr to rigid object appearance to %p\n",&res);
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

const char* TerrainModel::getName()
{
  RobotWorld& world = *worlds[this->world]->world;
  return world.terrains[index]->name.c_str();
}

int TerrainModel::getID()
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
  printf("Copying ptr to terrain appearance to %p\n",&res);
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


Simulator::Simulator(const WorldModel& model,const char* settings)
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
  if(settings && 0==strcmp(settings,"no_blem")) {
    printf("Turning off boundary layer collisions\n");
    sim->odesim.GetSettings().boundaryLayerCollisions = false;
  }

  //initialize simulation
  printf("Initializing simulation...\n");
  RobotWorld& rworld=*worlds[model.index]->world;
  sim->Init(&rworld);

  //setup controllers
  sim->robotControllers.resize(rworld.robots.size());
  for(size_t i=0;i<sim->robotControllers.size();i++) {
    Robot* robot=rworld.robots[i];
    sim->SetController(i,MakeController(robot));

    sim->controlSimulators[i].sensors.MakeDefault(robot);
  }
  printf("Done\n");


  //setup ODE settings, if any
  TiXmlElement* e=worlds[world.index]->xmlWorld.GetElement("simulation");
  if(e) {
    printf("Reading simulation settings...\n");
    XmlSimulationSettings s(e);
    if(!s.GetSettings(*sim)) {
      fprintf(stderr,"Warning, simulation settings not read correctly\n");
    }
    printf("Done\n");
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

WorldModel Simulator::getWorld() const
{
  static bool warned = false;
  if(!warned) {
    fprintf(stderr,"Simulator::getWorld() will be deprecated, please use world instead\n");
    warned = true;
  }
  return world;
}

void Simulator::reset()
{
  sim->ReadState(initialState);
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

SimRobotController Simulator::getController(int robot)
{
  static bool warned = false;
  if(!warned) {
    fprintf(stderr,"Simulator::getController() will be deprecated, please use controller() instead\n");
    warned = true;
  }
  return controller(robot);
}


SimRobotController Simulator::controller(int robot)
{
  SimRobotController c;
  c.sim = this;
  c.controller = &sim->controlSimulators[robot];
  c.index = robot;
  return c;
}


SimRobotController Simulator::getController(const RobotModel& robot)
{
  static bool warned = false;
  if(!warned) {
    fprintf(stderr,"Simulator::getController() will be deprecated, please use controller() instead\n");
    warned = true;
  }
  return controller(robot);
}

SimRobotController Simulator::controller(const RobotModel& robot)
{
  SimRobotController c;
  c.sim = this;
  c.controller = &sim->controlSimulators[robot.index];
  c.index = robot.index;
  return c;
}

void SimBody::enable(bool enabled)
{
  if(!enabled) dBodyDisable(body);
  else dBodyEnable(body);
}

bool SimBody::isEnabled()
{
  return dBodyIsEnabled(body) != 0;
}

void SimBody::enableDynamics(bool enabled)
{
  if(!enabled) dBodySetKinematic(body);
  else dBodySetDynamic(body);
}

bool SimBody::isDynamicsEnabled()
{
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
  sim->sim->hooks.push_back(new ForceHook(body,Vector3(f),Vector3(pworld)));
  sim->sim->hooks.back()->autokill = true;
  //dBodyAddForceAtPos(body,f[0],f[1],f[2],pworld[0],pworld[1],pworld[2]);
}

void SimBody::applyForceAtLocalPoint(const double f[3],const double plocal[3])
{
  if(!body) return;
  sim->sim->hooks.push_back(new LocalForceHook(body,Vector3(f),Vector3(plocal)));
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
  if(!body) return;
  const dReal* t=dBodyGetPosition(body);
  const dReal* R=dBodyGetRotation(body);
  for(int i=0;i<3;i++) out2[i] = t[i];
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      out[i+j*3] = R[i*4+j];
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


SimBody Simulator::getBody(const RobotModelLink& link)
{
  static bool warned = false;
  if(!warned) {
    fprintf(stderr,"Simulator::getBody() will be deprecated, please use body() instead\n");
    warned = true;
  }
  return body(link);
}

SimBody Simulator::getBody(const RigidObjectModel& object)
{
  static bool warned = false;
  if(!warned) {
    fprintf(stderr,"Simulator::getBody() will be deprecated, please use body() instead\n");
    warned = true;
  }
  return body(object);
}

SimBody Simulator::getBody(const TerrainModel& terrain)
{
  static bool warned = false;
  if(!warned) {
    fprintf(stderr,"Simulator::getBody() will be deprecated, please use body() instead\n");
    warned = true;
  }
  return body(terrain);
}

SimBody Simulator::body(const RobotModelLink& link)
{
  SimBody b;
  b.sim = this;
  b.body = sim->odesim.robot(link.robotIndex)->body(link.index);
  b.geometry = sim->odesim.robot(link.robotIndex)->triMesh(link.index);
  return b;
}

SimBody Simulator::body(const RigidObjectModel& object)
{
  SimBody b;
  b.sim = this;
  b.body = sim->odesim.object(object.index)->body();
  b.geometry = sim->odesim.object(object.index)->triMesh();
  return b; 
}

SimBody Simulator::body(const TerrainModel& terrain)
{
  SimBody b;
  b.sim = this;
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

SimRobotSensor::SimRobotSensor(SensorBase* _sensor)
  :sensor(_sensor)
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

SimRobotSensor SimRobotController::getSensor(int sensorIndex)
{
  static bool warned = false;
  if(!warned) {
    fprintf(stderr,"SimRobotController::getSensor() will be deprecated, please use sensor() instead\n");
    warned = true;
  }
  return sensor(sensorIndex);
}

SimRobotSensor SimRobotController::getNamedSensor(const std::string& name)
{
  static bool warned = false;
  if(!warned) {
    fprintf(stderr,"SimRobotController::getNamedSensor() will be deprecated, please use sensor() instead\n");
    warned = true;
  }
  return sensor(name.c_str());
}
SimRobotSensor SimRobotController::sensor(int sensorIndex)
{
  RobotSensors& sensors = controller->sensors;
  if(sensorIndex < 0 || sensorIndex >= (int)sensors.sensors.size())
    return SimRobotSensor(NULL);
  return SimRobotSensor(sensors.sensors[sensorIndex]);
}

SimRobotSensor SimRobotController::sensor(const char* name)
{
  RobotSensors& sensors = controller->sensors;
  SmartPointer<SensorBase> sensor = sensors.GetNamedSensor(name);
  if(sensor==NULL) {
    fprintf(stderr,"Warning, sensor %s does not exist\n",name);
  }
  return SimRobotSensor(sensor);
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

void SimRobotController::setMilestone(const vector<double>& q)
{
  Config qv(controller->robot->links.size(),&q[0]);
  stringstream ss;
  ss<<qv;
  controller->controller->SendCommand("set_q",ss.str());
}

void SimRobotController::setMilestone(const vector<double>& q,const vector<double>& dq)
{
  Config qv(controller->robot->links.size(),&q[0]);
  Config dqv(controller->robot->links.size(),&dq[0]);
  stringstream ss;
  ss<<qv<<"\t"<<dqv;
  controller->controller->SendCommand("set_qv",ss.str());
}


void SimRobotController::addMilestone(const vector<double>& q)
{
  Config qv(controller->robot->links.size(),&q[0]);
  stringstream ss;
  ss<<qv;
  controller->controller->SendCommand("append_q",ss.str());
}

void SimRobotController::addMilestoneLinear(const vector<double>& q)
{
  Config qv(controller->robot->links.size(),&q[0]);
  stringstream ss;
  ss<<qv;
  controller->controller->SendCommand("append_q_linear",ss.str());
}

void SimRobotController::setLinear(const std::vector<double>& q,double dt)
{
  PolynomialMotionQueue* mq = GetMotionQueue(controller->controller);
  mq->Cut(0);
  mq->AppendLinear(q,dt);
}
void SimRobotController::setCubic(const std::vector<double>& q,const std::vector<double>& v,double dt)
{
  PolynomialMotionQueue* mq = GetMotionQueue(controller->controller);
  mq->Cut(0);
  mq->AppendCubic(q,v,dt);
}
void SimRobotController::appendLinear(const std::vector<double>& q,double dt)
{
  PolynomialMotionQueue* mq = GetMotionQueue(controller->controller);
  mq->AppendLinear(q,dt);
}
void SimRobotController::addCubic(const std::vector<double>& q,const std::vector<double>& v,double dt)
{
  PolynomialMotionQueue* mq = GetMotionQueue(controller->controller);
  mq->AppendCubic(q,v,dt);
}

void SimRobotController::addMilestone(const vector<double>& q,const vector<double>& dq)
{
  Config qv(controller->robot->links.size(),&q[0]);
  Config dqv(controller->robot->links.size(),&dq[0]);
  stringstream ss;
  ss<<qv<<"\t"<<dqv;
  controller->controller->SendCommand("append_qv",ss.str());
}

void SimRobotController::setVelocity(const vector<double>& dq,double dt)
{
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
