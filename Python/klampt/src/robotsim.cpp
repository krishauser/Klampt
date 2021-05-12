#if defined (__APPLE__) || defined (MACOSX)
  #include "mac_fixes.h"
#endif //Mac fixes 

#include <vector>
#include <string>
#include "robotsim.h"
#include "widget.h"
#include <Klampt/Control/Command.h>
#include <Klampt/Control/PathController.h>
#include <Klampt/Control/FeedforwardController.h>
#include <Klampt/Control/LoggingController.h>
#include <Klampt/Sensing/JointSensors.h>
#include <Klampt/Planning/RobotCSpace.h>
#include <Klampt/Simulation/WorldSimulation.h>
#include <Klampt/Modeling/Interpolate.h>
#include <Klampt/Modeling/Mass.h>
#include <Klampt/Planning/RobotCSpace.h>
#include <Klampt/IO/XmlWorld.h>
#include <Klampt/IO/XmlODE.h>
#include <Klampt/IO/ROS.h>
#include <Klampt/IO/three.js.h> 
#include <KrisLibrary/robotics/NewtonEuler.h>
#include <KrisLibrary/robotics/Stability.h>
#include <KrisLibrary/robotics/TorqueSolver.h>
#include <KrisLibrary/meshing/PointCloud.h>
#include <KrisLibrary/meshing/VolumeGrid.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/drawMesh.h>
#include <KrisLibrary/GLdraw/Widget.h>
#include <KrisLibrary/GLdraw/TransformWidget.h>
#include <KrisLibrary/GLdraw/BoxWidget.h>
#include <KrisLibrary/GLdraw/SphereWidget.h>
#include <KrisLibrary/geometry/ConvexHull3D.h>
#include <Klampt/View/ObjectPoseWidget.h>
#include <Klampt/View/RobotPoseWidget.h>
#include <KrisLibrary/utils/AnyCollection.h>
#include <KrisLibrary/utils/stringutils.h>
#include <ode/ode.h>
#include "pyerr.h"
#include "pyconvert.h"
#include "robotik.h"
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
  vector<shared_ptr<RobotSensors> > robotSensors;
};

/// Internally used.
struct SimData
{
  WorldSimulation sim;
};


/// Internally used.
struct WidgetData
{
  shared_ptr<GLDraw::Widget> widget;
  int refCount;
};


static vector<shared_ptr<WorldData> > worlds;
static list<int> worldDeleteList;

static vector<shared_ptr<SimData> > sims;
static list<int> simDeleteList;

static vector<WidgetData> widgets;
static list<int> widgetDeleteList;

static bool gEnableCollisionInitialization = false;

static int gStabilityNumFCEdges = 4;

int createWorld(RobotWorld* ptr=NULL)
{
  if(worldDeleteList.empty()) {
    worlds.push_back(make_shared<WorldData>());
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
    worlds[index] = make_shared<WorldData>();
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
  //printf("Deref world %d: count %d\n",index,worlds[index]->refCount);
  if(worlds[index]->refCount == 0) {
    //printf("Deleting world %d\n",index);
    if(!worlds[index]->worldExternal)
      delete worlds[index]->world;
    worlds[index].reset();
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
    sims.push_back(make_shared<SimData>());
    return (int)(sims.size()-1);
  }
  else {
    int index = simDeleteList.front();
    simDeleteList.erase(simDeleteList.begin());
    sims[index] = make_shared<SimData>();
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

  sims[index].reset();
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
    widgets[index].widget.reset();
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
    widgets[index].widget.reset();
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

//cleans up internal data structures
void destroy()
{
  for(size_t i=0;i<sims.size();i++)
    sims[i].reset();
  for(size_t i=0;i<worlds.size();i++)
    worlds[i].reset();
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
  ManualOverrideController(Robot& robot,const shared_ptr<RobotController>& _base)
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

  shared_ptr<RobotController> base;
  bool override;
};

bool ManualOverrideController::ReadState(File& f)
{
  if(!ReadFile(f,override)) {
    printf("Unable to read override bit\n");
    return false;
  }
  if(!override) {
    if(!base->ReadState(f)) {
      printf("Unable to read base controller\n");
      return false;
    }
    return true;
  }
  if(!RobotController::ReadState(f)) {
    printf("Unable to read RobotController\n");
    return false;
  }
  return true;
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
inline shared_ptr<MyController> MakeController(Robot* robot)
{
  ManualOverrideController* lc=new ManualOverrideController(*robot,MakeDefaultController(robot));
  return shared_ptr<MyController>(lc);
}
inline PolynomialPathController* GetPathController(RobotController* controller)
{
  MyController* mc=dynamic_cast<MyController*>(controller);
  if(!mc) {
    throw PyException("Not using the default manual override controller");
  }
  LoggingController* lc=dynamic_cast<LoggingController*>(mc->base.get());
  if(!lc) {
    throw PyException("Not using the default robot controller");
  }
  FeedforwardController* ffc=dynamic_cast<FeedforwardController*>(lc->base.get());
  PolynomialPathController* pc=dynamic_cast<PolynomialPathController*>(ffc->base.get());
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
  geom = Meshing::TriMesh();
  Meshing::TriMesh& mesh = geom.AsTriangleMesh();
  mesh.tris.resize(tmesh.indices.size()/3);
  mesh.verts.resize(tmesh.vertices.size()/3);
  for(size_t i=0;i<mesh.tris.size();i++)
    mesh.tris[i].set(tmesh.indices[i*3],tmesh.indices[i*3+1],tmesh.indices[i*3+2]);
  for(size_t i=0;i<mesh.verts.size();i++)
    mesh.verts[i].set(tmesh.vertices[i*3],tmesh.vertices[i*3+1],tmesh.vertices[i*3+2]);
  geom.ClearCollisionData();
}


void GetPointCloud(const Geometry::AnyCollisionGeometry3D& geom,PointCloud& pc)
{
  Assert(geom.type == Geometry::AnyGeometry3D::PointCloud);
  const Meshing::PointCloud3D& gpc = geom.AsPointCloud();
  pc.vertices.resize(gpc.points.size()*3);
  pc.propertyNames = gpc.propertyNames;
  pc.properties.resize(gpc.points.size()*gpc.propertyNames.size());
  for(size_t i=0;i<gpc.points.size();i++) 
    gpc.points[i].get(pc.vertices[i*3],pc.vertices[i*3+1],pc.vertices[i*3+2]);
  if(!gpc.propertyNames.empty()) {
    if(gpc.properties.size() != gpc.points.size()) {
      throw PyException("GetPointCloud: Internal error, invalid # of properties");
    }
    for(size_t i=0;i<gpc.points.size();i++) {
      gpc.properties[i].getCopy(&pc.properties[i*gpc.propertyNames.size()]);
    }
  }
  pc.settings = gpc.settings;
}

void GetPointCloud(const PointCloud& pc,Geometry::AnyCollisionGeometry3D& geom)
{
  geom = Meshing::PointCloud3D();
  Meshing::PointCloud3D& gpc = geom.AsPointCloud();
  gpc.settings = pc.settings;
  gpc.points.resize(pc.vertices.size()/3);
  for(size_t i=0;i<gpc.points.size();i++)
    gpc.points[i].set(pc.vertices[i*3],pc.vertices[i*3+1],pc.vertices[i*3+2]);
  gpc.propertyNames = pc.propertyNames;
  if(pc.propertyNames.size() > 0) {
    if(pc.properties.size() != gpc.points.size()*pc.propertyNames.size()) {
      printf("Expected %d = %d*%d properties, got %d\n",(int)gpc.points.size(),(int)pc.propertyNames.size(),(int)gpc.points.size()*pc.propertyNames.size(),(int)pc.properties.size());
      throw PyException("GetPointCloud: Invalid number of properties in PointCloud");
    }
    gpc.properties.resize(pc.properties.size() / pc.propertyNames.size());
    gpc.properties[0].resize(pc.properties.size());
    gpc.properties[0].copy(&pc.properties[0]);
    int m=(int)pc.propertyNames.size();
    for(size_t i=0;i<gpc.properties.size();i++) {
      //gpc.properties[i].resize(pc.propertyNames.size());
      //gpc.properties[i].copy(&pc.properties[i*pc.propertyNames.size()]);
      if(i != 0)
        gpc.properties[i].setRef(gpc.properties[0],i*m,1,m);
    }
    gpc.properties[0].n = m;
  }
  geom.ClearCollisionData();
}

void GetVolumeGrid(const Geometry::AnyCollisionGeometry3D& geom,VolumeGrid& grid)
{
  Assert(geom.type == Geometry::AnyGeometry3D::ImplicitSurface);
  const Meshing::VolumeGrid& gvg = geom.AsImplicitSurface();
  grid.dims.resize(3);
  grid.dims[0] = gvg.value.m;
  grid.dims[1] = gvg.value.n;
  grid.dims[2] = gvg.value.p;
  grid.bbox.resize(6);
  grid.bbox[0] = gvg.bb.bmin.x;
  grid.bbox[1] = gvg.bb.bmin.y;
  grid.bbox[2] = gvg.bb.bmin.z;
  grid.bbox[3] = gvg.bb.bmax.x;
  grid.bbox[4] = gvg.bb.bmax.y;
  grid.bbox[5] = gvg.bb.bmax.z;
  grid.values.resize(gvg.value.m*gvg.value.n*gvg.value.p);
  int k=0;
  for(Array3D<Real>::iterator i=gvg.value.begin();i!=gvg.value.end();++i,k++) {
    grid.values[k] = *i;
  }
}

void GetVolumeGrid(const VolumeGrid& grid,Geometry::AnyCollisionGeometry3D& geom)
{
  geom = Meshing::VolumeGrid();
  Meshing::VolumeGrid& gvg = geom.AsImplicitSurface();
  Assert(grid.dims.size()==3);
  Assert(grid.bbox.size()==6);
  gvg.Resize(grid.dims[0],grid.dims[1],grid.dims[2]);
  gvg.bb.bmin.set(grid.bbox[0],grid.bbox[1],grid.bbox[2]);
  gvg.bb.bmax.set(grid.bbox[3],grid.bbox[4],grid.bbox[5]);
  Assert(grid.values.size() == grid.dims[0]*grid.dims[1]*grid.dims[2]);
  int k=0;
  for(Array3D<Real>::iterator i=gvg.value.begin();i!=gvg.value.end();++i,k++) {
    *i = grid.values[k];
  }
  geom.ClearCollisionData();
}

GeometricPrimitive::GeometricPrimitive()
{}

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
  properties[3] = r;
}

void GeometricPrimitive::setSegment(const double a[3],const double b[3])
{
  type = "Segment";
  properties.resize(6);
  copy(a,a+3,properties.begin());
  copy(b,b+3,properties.begin()+3);
}

void GeometricPrimitive::setTriangle(const double a[3],const double b[3],const double c[3])
{
  type = "Triangle";
  properties.resize(9);
  copy(a,a+3,properties.begin());
  copy(b,b+3,properties.begin()+3);
  copy(c,c+3,properties.begin()+6);
}

void GeometricPrimitive::setPolygon(const std::vector<double>& verts)
{
  if(verts.size() % 3 != 0)
    throw PyException("setPolygon requires a list of concatenated 3D vertices");
  if(verts.size() < 9)
    throw PyException("setPolygon requires at least 3 vertices (9 elements in list)");
  type = "Polygon";
  properties.resize(verts.size()+1);
  properties[0] = verts.size()/3;
  copy(verts.begin(),verts.end(),properties.begin()+1);
}

void GeometricPrimitive::setAABB(const double bmin[3],const double bmax[3])
{
  type = "AABB";
  properties.resize(6);
  copy(bmin,bmin+3,properties.begin());
  copy(bmax,bmax+3,properties.begin()+3);
}

void GeometricPrimitive::setBox(const double ori[3],const double R[9],const double dims[3])
{
  type = "Box";
  properties.resize(15);
  copy(ori,ori+3,properties.begin());
  copy(R,R+9,properties.begin()+3);
  copy(dims,dims+3,properties.begin()+12);
}

bool GeometricPrimitive::loadString(const char* str)
{
  vector<string> items = Split(str," \t\n");
  if(items.size() == 0) {
    type = "";
    properties.resize(0);
    return true;
  }
  type = items[0];
  properties.resize(items.size()-1);
  for(size_t i=1;i<items.size();i++)
    if(!LexicalCast<double>(items[i],properties[i-1])) {
      fprintf(stderr,"GeometricPrimitive::loadString: could not parse item %d: \"%s\"\n",(int)i,items[i].c_str());
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
  geomPtr = new shared_ptr<AnyCollisionGeometry3D>();
}

Geometry3D::Geometry3D(const Geometry3D& rhs)
  :world(rhs.world),id(rhs.id),geomPtr(NULL)
{
  shared_ptr<AnyCollisionGeometry3D>* geom = reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(rhs.geomPtr);
  if(*geom != NULL)
    geomPtr = new shared_ptr<AnyCollisionGeometry3D>(*geom);
  else
    geomPtr = new shared_ptr<AnyCollisionGeometry3D>();
}

Geometry3D::Geometry3D(const GeometricPrimitive& rhs)
  :world(-1),id(-1),geomPtr(NULL)
{
  geomPtr = new shared_ptr<AnyCollisionGeometry3D>();
  setGeometricPrimitive(rhs);
}

Geometry3D::Geometry3D(const ConvexHull& rhs)
  :world(-1),id(-1),geomPtr(NULL)
{
  geomPtr = new shared_ptr<AnyCollisionGeometry3D>();
  setConvexHull(rhs);
}


Geometry3D::Geometry3D(const TriangleMesh& rhs)
  :world(-1),id(-1),geomPtr(NULL)
{
  geomPtr = new shared_ptr<AnyCollisionGeometry3D>();
  setTriangleMesh(rhs);
}

Geometry3D::Geometry3D(const PointCloud& rhs)
  :world(-1),id(-1),geomPtr(NULL)
{
  geomPtr = new shared_ptr<AnyCollisionGeometry3D>();
  setPointCloud(rhs);
}

Geometry3D::Geometry3D(const VolumeGrid& rhs)
:world(-1),id(-1),geomPtr(NULL)
{
  geomPtr = new shared_ptr<AnyCollisionGeometry3D>();
  setVolumeGrid(rhs);
}

Geometry3D::~Geometry3D()
{
  free();
  shared_ptr<AnyCollisionGeometry3D>* geom = reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  delete geom;
}

const Geometry3D& Geometry3D::operator = (const Geometry3D& rhs)
{
  free();
  world = rhs.world;
  id = rhs.id;
  shared_ptr<AnyCollisionGeometry3D>* geom = reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  shared_ptr<AnyCollisionGeometry3D>* geom2 = reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(rhs.geomPtr);
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
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  shared_ptr<AnyCollisionGeometry3D>& resgeom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(res.geomPtr);
  if(geom != NULL) {
    resgeom = make_shared<AnyCollisionGeometry3D>(*geom);
  }
  return res;
}

void Geometry3D::set(const Geometry3D& g)
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  const shared_ptr<AnyCollisionGeometry3D>& ggeom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(g.geomPtr);
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
      geom = make_shared<AnyCollisionGeometry3D>();
  }
  else
    Assert(&*geom == &*(*mgeom));
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
  shared_ptr<AnyCollisionGeometry3D>* geom = reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);  
  if(isStandalone() && *geom) {
    //printf("Geometry3D(): Freeing standalone geometry\n");
    geom->reset();
  }
  world = -1;
  id = -1;

  delete geom;
  geomPtr = new shared_ptr<AnyCollisionGeometry3D>;
}

string Geometry3D::type()
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) return "";
  if(geom->Empty()) return "";
  string res = geom->TypeName();
  if(res == "Primitive") return "GeometricPrimitive";
  if(res == "ImplicitSurface") return "VolumeGrid";
  return res;
}

bool Geometry3D::empty()
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);  
  if(!geom) return true;
  if(geom->Empty()) return true;
  return false;
}

TriangleMesh Geometry3D::getTriangleMesh()
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  TriangleMesh mesh;
  if(geom) {
    GetMesh(*geom,mesh);
  }
  return mesh;
}


GeometricPrimitive Geometry3D::getGeometricPrimitive()
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
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

ConvexHull Geometry3D::getConvexHull()
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) return ConvexHull();
  Assert(geom->type == Geometry::AnyGeometry3D::ConvexHull);
  if(geom->type != Geometry::AnyGeometry3D::ConvexHull) {
    ConvexHull chull;
    return chull;
  }
  const Geometry::ConvexHull3D& hull = geom->AsConvexHull();
  if(hull.type != Geometry::ConvexHull3D::Polytope) {
    throw PyException("Can't get ConvexHull object from ConvexHull groups");
  }
  const auto& pts = hull.AsPolytope();
  ConvexHull chull;
  chull.points = pts;
  return chull;
}

void Geometry3D::setTriangleMesh(const TriangleMesh& mesh)
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  ManagedGeometry* mgeom = NULL;
  if(!isStandalone()) {
    RobotWorld& world = *worlds[this->world]->world;
    mgeom = &GetManagedGeometry(world,id);
  }
  if(geom == NULL) {
    if(mgeom) 
      geom = mgeom->CreateEmpty();
    else
      geom = make_shared<AnyCollisionGeometry3D>();
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
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  ManagedGeometry* mgeom = NULL;
  if(!isStandalone()) {
    RobotWorld& world = *worlds[this->world]->world;
    mgeom = &GetManagedGeometry(world,id);
  }
  if(geom == NULL) {
    if(mgeom) 
      geom = mgeom->CreateEmpty();
    else
      geom = make_shared<AnyCollisionGeometry3D>();
  }
  *geom = AnyCollisionGeometry3D(vector<Geometry::AnyGeometry3D>());
  geom->ReinitCollisionData();
}

Geometry3D Geometry3D::getElement(int element)
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) 
    throw PyException("Geometry is empty");
  if(geom->type == AnyCollisionGeometry3D::Group) {
    vector<AnyCollisionGeometry3D>& data = geom->GroupCollisionData();
    if(element < 0 || element >= (int)data.size())
      throw PyException("Invalid element specified");
    Geometry3D res;
    shared_ptr<AnyCollisionGeometry3D>& rgeom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(res.geomPtr);
    rgeom = make_shared<AnyCollisionGeometry3D>(data[element]);
    return res;
  }
  else if(geom->type == AnyCollisionGeometry3D::TriangleMesh) {
    CollisionMesh& data = geom->TriangleMeshCollisionData();
    if(element < 0 || element >= (int)data.tris.size())
      throw PyException("Invalid element specified");
    Math3D::Triangle3D tri;
    data.GetTriangle(element,tri);
    Geometry3D res;
    shared_ptr<AnyCollisionGeometry3D>& rgeom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(res.geomPtr);
    rgeom = make_shared<AnyCollisionGeometry3D>(Math3D::GeometricPrimitive3D(tri));
    return res;
  }
  else if(geom->type == AnyCollisionGeometry3D::TriangleMesh) {
    Meshing::PointCloud3D& data = geom->AsPointCloud();
    if(element < 0 || element >= (int)data.points.size())
      throw PyException("Invalid element specified");
    Geometry3D res;
    shared_ptr<AnyCollisionGeometry3D>& rgeom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(res.geomPtr);
    Assert(rgeom.get() != NULL);
    rgeom = make_shared<AnyCollisionGeometry3D>(Math3D::GeometricPrimitive3D(data.points[element]));
    return res;
  }
  else {
    throw PyException("Geometry type does not have sub-elements");
  }
}

void Geometry3D::setElement(int element,const Geometry3D& rhs)
{
  shared_ptr<AnyCollisionGeometry3D>& rgeom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(rhs.geomPtr);
  if(rgeom == NULL) 
    throw PyException("Setting an element to an empty geometry?");
  rgeom->InitCollisionData();

  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
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
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) 
    throw PyException("Geometry is empty");
  switch(geom->type) {
  case AnyCollisionGeometry3D::Group:
    return (int)geom->AsGroup().size();
  case AnyCollisionGeometry3D::PointCloud:
    return (int)geom->AsPointCloud().points.size();
  case AnyCollisionGeometry3D::TriangleMesh:
    return (int)geom->AsTriangleMesh().tris.size();
  default:
    return 0;
  }
}


PointCloud Geometry3D::getPointCloud()
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  PointCloud pc;
  if(geom) {
    GetPointCloud(*geom,pc);
  }
  return pc;
}

VolumeGrid Geometry3D::getVolumeGrid()
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  VolumeGrid grid;
  if(geom) {
    GetVolumeGrid(*geom,grid);
  }
  return grid;
}

void Geometry3D::setPointCloud(const PointCloud& pc)
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
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
      geom = make_shared<AnyCollisionGeometry3D>();
  }
  RigidTransform T = geom->GetTransform();
  GetPointCloud(pc,*geom);
  geom->SetTransform(T);
  //this is already called
  //geom->ClearCollisionData();
  if(mgeom) {
    //update the display list / cache
    mgeom->OnGeometryChange();
    mgeom->RemoveFromCache();
  }
}

void Geometry3D::setVolumeGrid(const VolumeGrid& vg)
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
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
      geom = make_shared<AnyCollisionGeometry3D>();
  }
  RigidTransform T = geom->GetTransform();
  GetVolumeGrid(vg,*geom);
  geom->SetTransform(T);
  //this is already called
  //geom->ClearCollisionData();
  if(mgeom) {
    //update the display list / cache
    mgeom->OnGeometryChange();
    mgeom->RemoveFromCache();
  }
}

void Geometry3D::setConvexHull(const ConvexHull& hull)
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
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
      geom = make_shared<AnyCollisionGeometry3D>();
  }
  ConvexHull3D chull;  
  chull.SetPoints(hull.points);

  RigidTransform T = geom->GetTransform();
  *geom = chull;
  geom->ClearCollisionData();
  geom->SetTransform(T);

  if(mgeom) {
    //update the display list / cache
    mgeom->OnGeometryChange();
    mgeom->RemoveFromCache();
  }
}

void Geometry3D::setConvexHullGroup(const Geometry3D& geom1, const Geometry3D & geom2)
{
  // make sure both geometry is convexhull
  shared_ptr<AnyCollisionGeometry3D>& ingeom1 = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geom1.geomPtr);
  Assert(ingeom1->type == AnyGeometry3D::ConvexHull);
  shared_ptr<AnyCollisionGeometry3D>& ingeom2 = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geom2.geomPtr);
  Assert(ingeom2->type == AnyGeometry3D::ConvexHull);
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
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
      geom = make_shared<AnyCollisionGeometry3D>();
  }
  // create collision data from its constructor
  RigidTransform T1 = ingeom1->GetTransform();
  RigidTransform T2 = ingeom2->GetTransform();
  RigidTransform TRel;
  TRel.mulInverseA(T1,T2);
  Geometry::ConvexHull3D hull;
  hull.SetHull(ingeom1->AsConvexHull(), ingeom2->AsConvexHull());
  *geom = AnyCollisionGeometry3D(hull);
  geom->InitCollisionData();
  geom->ConvexHullCollisionData().UpdateHullSecondRelativeTransform(TRel);
  geom->SetTransform(T1);

  if(mgeom) {
    //update the display list / cache
    mgeom->OnGeometryChange();
    mgeom->RemoveFromCache();
  }
}

void Geometry3D::setGeometricPrimitive(const GeometricPrimitive& prim)
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);  
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
      geom = make_shared<AnyCollisionGeometry3D>();
  }
  stringstream ss(prim.saveString());
  GeometricPrimitive3D g;
  ss>>g;
  if(!ss) {
    throw PyException("Internal error, can't read geometric primitive?");
  }
  RigidTransform T = geom->GetTransform();
  *geom = g;
  geom->ClearCollisionData();
  geom->SetTransform(T);
  if(mgeom) {
    //update the display list / cache
    mgeom->OnGeometryChange();
    mgeom->RemoveFromCache();
  }
}


bool Geometry3D::loadFile(const char* fn)
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  if(isStandalone()) {
    if(!geom) {
      geom = make_shared<AnyCollisionGeometry3D>();
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
      geom = shared_ptr<Geometry::AnyCollisionGeometry3D>(*mgeom);
      return true;
    }
    return false;
  }
}

bool Geometry3D::saveFile(const char* fn)
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) return false;
  return geom->Save(fn);
}

void Geometry3D::setCurrentTransform(const double R[9],const double t[3])
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) return;
  RigidTransform T;
  T.R.set(R);
  T.t.set(t);
  geom->SetTransform(T);
}

void Geometry3D::getCurrentTransform(double out[9],double out2[3])
{
  RigidTransform T;
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
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
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  RigidTransform T;
  T.R.set(R);
  T.t.set(t);
  if(isStandalone()) {
    geom->Transform(T);
    geom->ClearCollisionData();
  }
  else {
    //update the display list / cache
    RobotWorld& world=*worlds[this->world]->world;
    ManagedGeometry* mgeom = &GetManagedGeometry(world,id);
    mgeom->TransformGeometry(Matrix4(T));
    //mgeom->OnGeometryChange();
    //mgeom->RemoveFromCache();
  }
}

void Geometry3D::setCollisionMargin(double margin)
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) return;
  geom->margin = margin;
}

double Geometry3D::getCollisionMargin()
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) return 0;
  return geom->margin;
}

void Geometry3D::getBB(double out[3],double out2[3])
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) {
    out[0] = out[1] = out[2] = Inf;
    out2[0] = out2[1] = out2[2] = -Inf;
    return;
  }
  AABB3D bb = geom->GetAABB();
  bb.bmin.get(out);
  bb.bmax.get(out2);
}

void Geometry3D::getBBTight(double out[3],double out2[3])
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) {
    out[0] = out[1] = out[2] = Inf;
    out2[0] = out2[1] = out2[2] = -Inf;
    return;
  }
  geom->InitCollisionData();
  AABB3D bb = geom->GetAABBTight();
  bb.bmin.get(out);
  bb.bmax.get(out2);
}

Geometry3D Geometry3D::convert(const char* destype,double param)
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) throw PyException("Geometry3D is empty, cannot convert");
  AnyGeometry3D::Type srctype = geom->type;
  AnyGeometry3D::Type destype2;
  if(0==strcmp(destype,"TriangleMesh")) 
    destype2 = AnyGeometry3D::TriangleMesh;
  else if(0==strcmp(destype,"PointCloud")) 
    destype2 = AnyGeometry3D::PointCloud;
  else if(0==strcmp(destype,"VolumeGrid")) 
    destype2 = AnyGeometry3D::ImplicitSurface;
  else if(0==strcmp(destype,"GeometricPrimitive")) 
    destype2 = AnyGeometry3D::Primitive;
  else if(0==strcmp(destype,"ConvexHull")) 
    destype2 = AnyGeometry3D::ConvexHull;
  else
    throw PyException("Invalid desired type specified, must be TriangleMesh, PointCloud, or VolumeGrid or ConvexHull");

  if(param < 0 && srctype != AnyGeometry3D::ImplicitSurface) throw PyException("Invalid conversion parameter, must be nonnegative");

  //do the conversion
  Geometry3D res;
  shared_ptr<AnyCollisionGeometry3D>& resgeom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(res.geomPtr);
  resgeom = make_shared<AnyCollisionGeometry3D>();
  if(srctype == destype2 && param > 0)  {
    if(!geom->Remesh(param,*resgeom)) {
      stringstream ss;
      ss<<"Cannot perform the geometry remeshiing "<<geom->TypeName()<<" at res "<<param;
      throw PyException(ss.str().c_str());
    }
    return res;
  }
  if(!geom->Convert(destype2,*resgeom,param)) {
    stringstream ss;
    ss<<"Cannot perform the geometry conversion "<<geom->TypeName()<<" -> "<<destype;
    throw PyException(ss.str().c_str());
  }
  return res;
}

bool Geometry3D::collides(const Geometry3D& other)
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  shared_ptr<AnyCollisionGeometry3D>& geom2 = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(other.geomPtr);
  if(!geom || !geom2) return false;
  return geom->Collides(*geom2);
}

bool Geometry3D::withinDistance(const Geometry3D& other,double tol)
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  shared_ptr<AnyCollisionGeometry3D>& geom2 = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(other.geomPtr);
  if(!geom || !geom2) return false;
  return geom->WithinDistance(*geom2,tol);
}

double Geometry3D::distance_simple(const Geometry3D& other,double relErr,double absErr)
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  shared_ptr<AnyCollisionGeometry3D>& geom2 = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(other.geomPtr);
  if(!geom || !geom2) return 0;
  AnyCollisionQuery q(*geom,*geom2);
  return q.Distance(relErr,absErr);
}

DistanceQuerySettings::DistanceQuerySettings()
:relErr(0),absErr(0),upperBound(Inf)
{}

DistanceQueryResult::DistanceQueryResult()
{}

ContactQueryResult::ContactQueryResult()
{}

DistanceQueryResult Geometry3D::distance_point(const double pt[3])
{
  return distance_point_ext(pt,DistanceQuerySettings());
}

DistanceQueryResult Geometry3D::distance_point_ext(const double pt[3],const DistanceQuerySettings& settings)
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) throw PyException("Geometry3D.distance_point: Geometry is empty");
  AnyDistanceQuerySettings gsettings;
  gsettings.relErr = settings.relErr;
  gsettings.absErr = settings.absErr;
  gsettings.upperBound = settings.upperBound;
  AnyDistanceQueryResult gres = geom->Distance(Vector3(pt),gsettings);
  if(IsInf(gres.d)) {
    throw PyException("Distance queries not implemented yet for that type of geometry");
  }
  DistanceQueryResult result;
  result.d = gres.d;
  result.hasClosestPoints = gres.hasClosestPoints;
  if(result.hasClosestPoints) {
    result.cp1.resize(3);
    result.cp2.resize(3);
    gres.cp1.get(&result.cp1[0]);
    gres.cp2.get(&result.cp2[0]);
    result.elem1 = gres.elem1;
    result.elem2 = gres.elem2;
  }
  else {
    result.elem1 = -1;
    result.elem2 = -1;
  }
  result.hasGradients = gres.hasDirections;
  if(result.hasGradients) {
    result.grad1.resize(3);
    result.grad2.resize(3);
    gres.dir1.get(&result.grad2[0]);
    gres.dir2.get(&result.grad1[0]);
  }
  return result;
}

DistanceQueryResult Geometry3D::distance(const Geometry3D& other)
{
  return distance_ext(other,DistanceQuerySettings());
}

DistanceQueryResult Geometry3D::distance_ext(const Geometry3D& other,const DistanceQuerySettings& settings)
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  shared_ptr<AnyCollisionGeometry3D>& geom2 = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(other.geomPtr);
  if(!geom) throw PyException("Geometry3D.distance: Geometry is empty");
  if(!geom2) throw PyException("Geometry3D.distance: Other geometry is empty");
  AnyDistanceQuerySettings gsettings;
  gsettings.relErr = settings.relErr;
  gsettings.absErr = settings.absErr;
  gsettings.upperBound = settings.upperBound;
  //std::cout << "call dist\n";
  AnyDistanceQueryResult gres = geom->Distance(*geom2,gsettings);
  if(IsInf(gres.d)) {
    throw PyException("Distance queries not implemented yet for those types of geometry, or geometries are content-free?");
  }
  DistanceQueryResult result;
  result.d = gres.d;
  result.hasClosestPoints = gres.hasClosestPoints;
  if(result.hasClosestPoints) {
    result.cp1.resize(3);
    result.cp2.resize(3);
    gres.cp1.get(&result.cp1[0]);
    gres.cp2.get(&result.cp2[0]);
    result.elem1 = gres.elem1;
    result.elem2 = gres.elem2;
  }
  else {
    result.elem1 = -1;
    result.elem2 = -1;
  }
  result.hasGradients = gres.hasDirections;
  if(result.hasGradients) {
    result.grad1.resize(3);
    result.grad2.resize(3);
    gres.dir1.get(&result.grad2[0]);
    gres.dir2.get(&result.grad1[0]);
  }
  return result;
}

bool Geometry3D::rayCast(const double s[3],const double d[3],double out[3])
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
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

int Geometry3D::rayCast_ext(const double s[3],const double d[3],double out[3])
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) return false;
  Ray3D r;
  r.source.set(s);
  r.direction.set(d);
  Real distance;
  int element=-1;
  if(geom->RayCast(r,&distance,&element)) {
    Vector3 pt = r.source + r.direction*distance;
    pt.get(out);
    return element;
  }
  return -1;
}

ContactQueryResult Geometry3D::contacts(const Geometry3D& other,double padding1,double padding2,int maxContacts)
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  shared_ptr<AnyCollisionGeometry3D>& geom2 = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(other.geomPtr);
  if(!geom) throw PyException("Geometry3D.contacts: Geometry is empty");
  if(!geom2) throw PyException("Geometry3D.contacts: Other geometry is empty");
  AnyContactsQuerySettings settings;
  settings.padding1 = padding1;
  settings.padding2 = padding2;
  if(maxContacts > 0) {
    settings.maxcontacts = maxContacts;
    settings.cluster = true;
  }
  AnyContactsQueryResult res = geom->Contacts(*geom2,settings);
  ContactQueryResult out;
  out.depths.resize(res.contacts.size());
  out.points1.resize(res.contacts.size());
  out.points2.resize(res.contacts.size());
  out.normals.resize(res.contacts.size());
  out.elems1.resize(res.contacts.size());
  out.elems2.resize(res.contacts.size());
  for(size_t i=0;i<res.contacts.size();i++) {
    out.depths[i] = res.contacts[i].depth;
    out.points1[i].resize(3);
    res.contacts[i].p1.get(&out.points1[i][0]);
    out.points2[i].resize(3);
    res.contacts[i].p2.get(&out.points2[i][0]);
    out.normals[i].resize(3);
    res.contacts[i].n.get(&out.normals[i][0]);
    out.elems1[i] = res.contacts[i].elem1;
    out.elems2[i] = res.contacts[i].elem2;
  }
  return out;
}


void Geometry3D::support(const double dir[3], double out[3])
{
  shared_ptr<AnyCollisionGeometry3D>& ingeom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(this->geomPtr);
  if(ingeom->type != AnyGeometry3D::ConvexHull)
    throw PyException("Only the ConvexHull type supports the support() method");
  const auto& ch = ingeom->ConvexHullCollisionData();
  Vector3 res = ch.FindSupport(Vector3(dir));
  res.get(out);
}

//KH: note: pointer gymnastics necessary to allow appearances to refer to temporary appearances as well as references to world, while also
//exposing an opaque pointer in appearance.h

//defined in Cpp/Modeling/ManagedGeometry.cpp
void SetupDefaultAppearance(GLDraw::GeometryAppearance& app);

Appearance::Appearance()
  :world(-1),id(-1),appearancePtr(NULL)
{
  auto ptr = new shared_ptr<GLDraw::GeometryAppearance>;
  ptr->reset(new GLDraw::GeometryAppearance());
  SetupDefaultAppearance(**ptr);
  appearancePtr = ptr;
}

Appearance::Appearance(const Appearance& rhs)
  :world(rhs.world),id(rhs.id),appearancePtr(NULL)
{
  shared_ptr<GLDraw::GeometryAppearance>* geom = reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(rhs.appearancePtr);
  appearancePtr = new shared_ptr<GLDraw::GeometryAppearance>(*geom);
}

Appearance::~Appearance()
{
  free();
  shared_ptr<GLDraw::GeometryAppearance>* app = reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  delete app;
}

const Appearance& Appearance::operator = (const Appearance& rhs)
{
  free();
  world = rhs.world;
  id = rhs.id;
  shared_ptr<GLDraw::GeometryAppearance>* geom = reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  shared_ptr<GLDraw::GeometryAppearance>* geom2 = reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(rhs.appearancePtr);
  *geom = *geom2;
  return *this;
} 


void Appearance::refresh(bool deep)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
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
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  shared_ptr<GLDraw::GeometryAppearance>& resapp = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(res.appearancePtr);
  if(app != NULL) {
    resapp = make_shared<GLDraw::GeometryAppearance>(*app);
  }
  return res;
}

void Appearance::set(const Appearance& g)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  shared_ptr<GLDraw::GeometryAppearance>& gapp = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(g.appearancePtr);

  if(!isStandalone()) {
    //need to detach from other geometries that might be sharing this appearance
    RobotWorld& world=*worlds[this->world]->world;
    ManagedGeometry& geom = GetManagedGeometry(world,id);
    if(geom.IsAppearanceShared()) {
      geom.SetUniqueAppearance();
      app = geom.Appearance();
    }
  }
  if(app == NULL) {
    app = make_shared<GLDraw::GeometryAppearance>(*gapp);
  }
  else {
    app->CopyMaterial(*gapp);
  }
}

void Appearance::free()
{
  shared_ptr<GLDraw::GeometryAppearance>* app = reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);

  if(isStandalone() && *app) {
    //printf("Appearance(): Freeing standalone appearance for %p\n",this);
    app->reset();
  }
  else if(*app)
    //printf("Appearance(): Releasing reference to world appearance %d %d for %p\n",world,id,this);
    ;
    
  world = -1;
  id = -1;
  app->reset();
}

void Appearance::setDraw(bool draw)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return;
  if(!isStandalone()) {
    RobotWorld& world=*worlds[this->world]->world;
    ManagedGeometry& geom = GetManagedGeometry(world,id);
    if(geom.IsAppearanceShared()) {
      geom.SetUniqueAppearance();
      app = geom.Appearance();
    }
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
void Appearance::setDraw(int feature,bool draw)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return;
  if(!isStandalone()) {
    RobotWorld& world=*worlds[this->world]->world;
    ManagedGeometry& geom = GetManagedGeometry(world,id);
    if(geom.IsAppearanceShared()) {
      geom.SetUniqueAppearance();
      app = geom.Appearance();
    }
  }
  switch(feature) {
  case ALL: app->drawFaces = app->drawVertices = app->drawEdges = draw; break;
  case VERTICES: app->drawVertices = draw; break;
  case EDGES: app->drawEdges = draw; break;
  case FACES: app->drawFaces = draw; break;
  }
}

bool Appearance::getDraw()
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return false;
  return app->drawFaces || app->drawVertices || app->drawEdges;
}

bool Appearance::getDraw(int feature)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return false;
  switch(feature) {
  case ALL: return app->drawFaces || app->drawVertices || app->drawEdges;
  case VERTICES: return app->drawVertices;
  case EDGES: return app->drawEdges;
  case FACES: return app->drawFaces;
  }
  return false;
}

void Appearance::setColor(float r,float g,float b,float a)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return;
  if(!isStandalone()) {
    RobotWorld& world=*worlds[this->world]->world;
    ManagedGeometry& geom = GetManagedGeometry(world,id);
    if(geom.IsAppearanceShared()) {
      geom.SetUniqueAppearance();
      app = geom.Appearance();
    }
  }
  app->SetColor(r,g,b,a);
}

void Appearance::setShininess(float shininess,float strength)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return;
  if(!isStandalone()) {
    RobotWorld& world=*worlds[this->world]->world;
    ManagedGeometry& geom = GetManagedGeometry(world,id);
    if(geom.IsAppearanceShared()) {
      geom.SetUniqueAppearance();
      app = geom.Appearance();
    }
  }
  app->shininess = shininess;
  if(strength >= 0)
    app->specularColor.set(strength,strength,strength);
}

float Appearance::getShininess()
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) throw PyException("Invalid appearance");
  return app->shininess;
}

void Appearance::setColor(int feature,float r,float g,float b,float a)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return;
  if(!isStandalone()) {
    RobotWorld& world=*worlds[this->world]->world;
    ManagedGeometry& geom = GetManagedGeometry(world,id);
    if(geom.IsAppearanceShared()) {
      geom.SetUniqueAppearance();
      app = geom.Appearance();
    }
  }
  switch(feature) {
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
  case EMISSIVE:
    app->emissiveColor.set(r,g,b,a);
    break;
  case SPECULAR:
    app->specularColor.set(r,g,b,a);
    break;
  default:
    throw PyException("Invalid feature");
  }
}

void Appearance::getColor(float out[4])
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) throw PyException("Invalid appearance");
  for(int i=0;i<4;i++) out[i] = app->faceColor.rgba[i];
}
void Appearance::getColor(int feature,float out[4])
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) throw PyException("Invalid appearance");
  GLDraw::GLColor c;
  switch(feature) {
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
  case EMISSIVE:
    c = app->emissiveColor;
    break;
  case SPECULAR:
    c = app->specularColor;
    break;
  default:
    throw PyException("Invalid feature");
  }
  for(int i=0;i<4;i++) out[i] = c.rgba[i];
}
void Appearance::setColors(int feature,const std::vector<float>& colors,bool alpha)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) throw PyException("Invalid appearance");
  size_t nchannels = 3;
  if(alpha) nchannels = 4;
  if(colors.size()%nchannels != 0) 
    throw PyException("An invalid number of color channels is specified, must be a multiple of 3 or 4 (depending on value of alpha)");
  size_t n = colors.size()/nchannels;
  switch(feature) {
  case VERTICES:
    {
      app->vertexColors.resize(n,app->vertexColor);
      for(size_t i=0;i<n;i++) {
        for(size_t k=0;k<nchannels;k++)
          app->vertexColors[i].rgba[k] = colors[i*nchannels+k];
      }
    }
    break;
  case FACES:
    {
      app->faceColors.resize(n,app->faceColor);
      for(size_t i=0;i<n;i++) {
        for(size_t k=0;k<nchannels;k++)
          app->faceColors[i].rgba[k] = colors[i*nchannels+k];
      }
    }
    break;
  default:
    throw PyException("Invalid feature, can only do per-element colors for VERTICES or FACES");
  }
}
void Appearance::setElementColor(int feature,int element,float r,float g,float b,float a)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) throw PyException("Invalid appearance");
  if(element < 0) throw PyException("Invalid negative element");
  switch(feature) {
  case VERTICES:
    {
      if(element >= (int)app->vertexColors.size()) {
        if(app->geom == NULL) {
          app->vertexColors.resize(element+1,app->vertexColor);
        }
        else if(app->vertexColors.empty()) {
          throw PyException("TODO: resize vertex colors to geometry size");
        }
        else {
          throw PyException("Invalid element specified"); 
        }
      }
      app->vertexColors[element].set(r,g,b,a);
    }
    break;
  case FACES:
    {
      if(element >= (int)app->faceColors.size()) {
        if(app->geom == NULL) {
          app->faceColors.resize(element+1,app->faceColor);
        }
        else if(app->faceColors.empty()) {
          throw PyException("TODO: resize face colors to geometry size");
        }
        else {
          throw PyException("Invalid element specified"); 
        }
      }
      app->faceColors[element].set(r,g,b,a);
    }
    break;
  default:
    throw PyException("Invalid feature, can only do per-element colors for VERTICES or FACES");
  }
}

void Appearance::getElementColor(int feature,int element,float out[4])
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) throw PyException("Invalid appearance");
  switch(feature) {
  case VERTICES:
    {
      if(app->vertexColors.empty())
        for(int i=0;i<4;i++) out[i] = app->vertexColor.rgba[i];
      else {
        if(element < 0 || element >= (int)app->vertexColors.size()) throw PyException("Invalid element specified");
        for(int i=0;i<4;i++) out[i] = app->vertexColors[element].rgba[i];
      }
    }
    break;
  case FACES:
    {
      if(app->faceColors.empty())
        for(int i=0;i<4;i++) out[i] = app->faceColor.rgba[i];
      else {
        if(element < 0 || element >= (int)app->faceColors.size()) throw PyException("Invalid element specified");
        for(int i=0;i<4;i++) out[i] = app->faceColors[element].rgba[i];
      }
    }
    break;
  default:
    throw PyException("Invalid feature, can only do per-element colors for VERTICES or FACES");
  }
}
void Appearance::setTexture1D(int w,const char* format,const std::vector<unsigned char>& bytes)
{
  throw PyException("Python API for textures not implemented yet");
}

void Appearance::setTexture2D(int w,int h,const char* format,const std::vector<unsigned char>& bytes,bool topdown)
{
 throw PyException("Python API for textures not implemented yet");
}

void Appearance::setTexcoords(const std::vector<double>& uvs)
{
  throw PyException("Python API for textures not implemented yet");
}

void Appearance::setPointSize(float size)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return;
  if(!isStandalone()) {
    RobotWorld& world=*worlds[this->world]->world;
    ManagedGeometry& geom = GetManagedGeometry(world,id);
    if(geom.IsAppearanceShared()) {
      geom.SetUniqueAppearance();
      app = geom.Appearance();
    }
  }
  app->vertexSize = size;
}

void Appearance::setCreaseAngle(float creaseAngleRads)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return;
  if(!isStandalone()) {
    RobotWorld& world=*worlds[this->world]->world;
    ManagedGeometry& geom = GetManagedGeometry(world,id);
    if(geom.IsAppearanceShared()) {
      geom.SetUniqueAppearance();
      app = geom.Appearance();
    }
  }
  app->creaseAngle = creaseAngleRads;
}

void Appearance::setSilhouette(float radius,float r,float g,float b,float a)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return;
  if(!isStandalone()) {
    RobotWorld& world=*worlds[this->world]->world;
    ManagedGeometry& geom = GetManagedGeometry(world,id);
    if(geom.IsAppearanceShared()) {
      geom.SetUniqueAppearance();
      app = geom.Appearance();
    }
  }
  app->silhouetteRadius = radius;
  app->silhouetteColor.rgba[0] = r;
  app->silhouetteColor.rgba[1] = g;
  app->silhouetteColor.rgba[2] = b;
  app->silhouetteColor.rgba[3] = a;
}

void Appearance::drawGL()
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return;
  if(!app->geom) return;
  app->DrawGL();
}

void Appearance::drawWorldGL(Geometry3D& g)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(g.geomPtr);
  if(!geom) return;
  if(!app) {
    app = make_shared<GLDraw::GeometryAppearance>();
    SetupDefaultAppearance(*app);
  }
  if(app->geom) {
    if(app->geom != geom.get()) {
      fprintf(stderr,"Appearance::drawGL(): performance warning, setting to a different geometry\n");
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
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(g.geomPtr);
  if(!geom) return;
  if(!app) {
    app = make_shared<GLDraw::GeometryAppearance>();
    SetupDefaultAppearance(*app);
  }
  if(app->geom) {
    if(app->geom != geom.get()) {
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

TriangleMesh::TriangleMesh()
{}

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

ConvexHull::ConvexHull()
{}


int ConvexHull::numPoints() const
{
  return points.size()/3;
}

void ConvexHull::addPoint(const double pt[3])
{
  points.push_back(pt[0]);
  points.push_back(pt[1]);
  points.push_back(pt[2]);
}

void ConvexHull::getPoint(int index,double out[3]) const
{
  int i=index*3;
  if(i<0 || i >= (int)points.size())
    throw PyException("Invalid point index");
  out[0] = points[i];
  out[1] = points[i+1];
  out[2] = points[i+2];
}

void ConvexHull::translate(const double t[3])
{
  for(size_t i=0;i<points.size();i+=3) {
    points[i] += t[0];
    points[i+1] += t[1];
    points[i+2] += t[2];
  }
}

void ConvexHull::transform(const double R[9],const double t[3])
{
  RigidTransform T;
  T.R.set(R);
  T.t.set(t);
  std::vector<double> &vertices = points;
  for(size_t i=0;i<vertices.size();i+=3) {
    Vector3 v(vertices[i],vertices[i+1],vertices[i+2]);
    v = T*v;
    //v.get(vertices[i],vertices[i+1],vertices[i+2]);
    vertices[i] = v[0];
    vertices[i + 1] = v[1];
    vertices[i + 2] = v[2];
  }
}

PointCloud::PointCloud()
{}
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
  if(int(values.size()) != n) {
    throw PyException("Invalid size of properties list, must have size #points");
  }
  assert(values.size() == n);
  size_t m=propertyNames.size();
  assert(properties.size() == n*m);
  propertyNames.push_back(pname);
  vector<double> newprops(n*(m+1));
  for(int i=0;i<n;i++) {
    assert (i*(m+1) + m < (int)newprops.size()); 
    if(m > 0) {
      assert ((i+1)*m < (int)properties.size()); 
      std::copy(properties.begin()+i*m,properties.begin()+(i+1)*m,newprops.begin()+i*(m+1));
    }
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
  if(properties.size() != n*m) {
    throw PyException("Internal error, properties doesn't contain #points * #properties items?"); 
  }
  copy(vproperties.begin(),vproperties.begin()+m*n,properties.begin());
}

void PointCloud::setProperties(int pindex,const vector<double>& vproperties)
{
  if(pindex < 0 || pindex >= (int)propertyNames.size())
    throw PyException("Invalid property index"); 
  int n = numPoints();
  if((int)vproperties.size() < n) {
    throw PyException("Invalid size of properties vector, needs to have size #points"); 
  }
  size_t k=pindex;
  for(int i=0;i<n;i++,k+=propertyNames.size())
    properties[k] = vproperties[i];
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

void PointCloud::getProperties(int pindex,std::vector<double>& out) const
{
  if(pindex < 0 || pindex >= (int)propertyNames.size())
    throw PyException("Invalid property index");  
  int n=numPoints();
  out.resize(n);
  size_t k=pindex;
  for(int i=0;i<n;i++,k+=propertyNames.size())
    out[i] = properties[k];
}

void PointCloud::getProperties(const std::string& pname,std::vector<double>& out) const
{
  int pindex = -1;
  for(size_t i=0;i<propertyNames.size();i++)
    if(propertyNames[i] == pname) {
      pindex = (int)i;
      break;
    }
  if(pindex < 0)
    throw PyException("Invalid property name");  
  return getProperties(pindex,out); 
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

  //transform viewpoint, if available
  if(settings.count("viewpoint") > 0) {
    stringstream ss(settings["viewpoint"]);
    RigidTransform vpOld;
    QuaternionRotation q;
    ss>>vpOld.t>>q;
    q.getMatrix(vpOld.R);

    RigidTransform vpNew = T*vpOld;

    q.setMatrix(vpNew.R);
    stringstream ss2;
    ss2 << vpNew.t <<" "<<q;
    settings["viewpoint"] = ss2.str();
  }

  //transform normals
  int nx = -1, ny = -1, nz = -1;
  for(size_t i=0;i<propertyNames.size();i++)
    if(propertyNames[i] == "normal_x") {
      nx = (int)i;
      break;
    }
  if(nx < 0) return;
  for(size_t i=0;i<propertyNames.size();i++)
    if(propertyNames[i] == "normal_y") {
      ny = (int)i;
      break;
    }
  if(ny < 0) return;
  for(size_t i=0;i<propertyNames.size();i++)
    if(propertyNames[i] == "normal_z") {
      nz = (int)i;
      break;
    }
  if(nz < 0) return;
  for(size_t i=0,base=0;i<vertices.size();i++,base+=propertyNames.size()) {
    Vector3 n(properties[base+nx],properties[base+ny],properties[base+nz]);
    n = T.R*n;
    n.get(properties[base+nx],properties[base+ny],properties[base+nz]);
  }
}


VolumeGrid::VolumeGrid()
{}

void VolumeGrid::setBounds(const double bmin[3],const double bmax[3])
{
  bbox.resize(6);
  bbox[0] = bmin[0];
  bbox[1] = bmin[1];
  bbox[2] = bmin[2];
  bbox[3] = bmax[0];
  bbox[4] = bmax[1];
  bbox[5] = bmax[2];
}

void VolumeGrid::resize(int sx,int sy,int sz)
{
  Assert(sx >= 0 && sy >= 0 && sz >= 0);
  dims.resize(3);
  dims[0] = sx;
  dims[1] = sy;
  dims[2] = sz;
  values.resize(sx*sy*sz);
}

void VolumeGrid::set(double value)
{
  std::fill(values.begin(),values.end(),value);
}

void VolumeGrid::set(int i,int j,int k,double value)
{
  if(dims.empty()) throw PyException("VolumeGrid was not initialized yet");
  if(i < 0 || i >= (int)dims[0]) throw PyException("First index out of range");
  if(j < 0 || j >= (int)dims[1]) throw PyException("Second index out of range");
  if(k < 0 || k >= (int)dims[2]) throw PyException("Third index out of range");
  int ind = i*dims[1]*dims[2] + j*dims[2] + k;
  values[ind] = value;
}

double VolumeGrid::get(int i,int j,int k)
{
  if(dims.empty()) throw PyException("VolumeGrid was not initialized yet");
  if(i < 0 || i >= (int)dims[0]) throw PyException("First index out of range");
  if(j < 0 || j >= (int)dims[1]) throw PyException("Second index out of range");
  if(k < 0 || k >= (int)dims[2]) throw PyException("Third index out of range");
  int ind = i*dims[1]*dims[2] + j*dims[2] + k;
  return values[ind];
}

void VolumeGrid::shift(double dv)
{
  for(vector<double>::iterator i=values.begin();i!=values.end();i++)
    *i += dv;
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

WorldModel::WorldModel(const char* fn)
{
  index = createWorld();
  if(!loadFile(fn)) {
    stringstream ss;
    ss << "Error loading world XML file " << fn;
    throw PyException(ss.str().c_str());
  }
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
    otherworld.robots[i] = make_shared<Robot>();
    *otherworld.robots[i] = *myworld.robots[i];
    otherworld.robotViews[i].robot = otherworld.robots[i].get();
  }
  for(size_t i=0;i<otherworld.terrains.size();i++) {
    otherworld.terrains[i] = make_shared<Terrain>();
    *otherworld.terrains[i] = *myworld.terrains[i];
  }
  for(size_t i=0;i<otherworld.rigidObjects.size();i++) {
    otherworld.rigidObjects[i] = make_shared<RigidObject>();
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
      printf("Error loading robot file %s\n",fn);
      return false;
    }
    if(gEnableCollisionInitialization) 
      world.robots.back()->InitCollisions();
    world.robots.back()->UpdateGeometry();
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
      printf("Error opening or parsing world file %s\n",fn);
      return false;
    }
    if(gEnableCollisionInitialization) 
      world.InitCollisions();
    world.UpdateGeometry();
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
  r.robot = worlds[index]->world->robots[robot].get();
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
      r.robot = world.robots[i].get();
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
  r.robotPtr = worlds[index]->world->robots[robot].get();
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
  obj.object = worlds[index]->world->rigidObjects[object].get();
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
      obj.object = world.rigidObjects[i].get();
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
  t.terrain = worlds[index]->world->terrains[terrain].get();
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
      t.terrain = world.terrains[i].get();
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
  robot.robot = world.robots.back().get();
  return robot;
}

RigidObjectModel WorldModel::makeRigidObject(const char* name)
{
  RobotWorld& world = *worlds[index]->world;
  RigidObjectModel object;
  object.world = index;
  object.index = (int)world.rigidObjects.size();
  world.AddRigidObject(name,new RigidObject());
  object.object = world.rigidObjects.back().get();
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
  terrain.terrain = world.terrains.back().get();
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
  robot.robot = world.robots.back().get();
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
  obj.object = world.rigidObjects.back().get();
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
  obj.terrain = world.terrains.back().get();
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
  world.AddRobot(name,new Robot());
  *world.robots.back() = *robot.robot;
  return this->robot((int)world.robots.size()-1);
}

RigidObjectModel WorldModel::add(const char* name,const RigidObjectModel& obj)
{
  if(obj.object == NULL)
    throw PyException("add(RigidObjectModel): obj refers to NULL object");
  RobotWorld& world = *worlds[index]->world;
  world.AddRigidObject(name,new RigidObject());
  *world.rigidObjects.back() = *obj.object;
  return this->rigidObject((int)world.rigidObjects.size()-1);
}
 
TerrainModel WorldModel::add(const char* name,const TerrainModel& terrain)
{
  if(terrain.terrain == NULL)
    throw PyException("add(TerrianModel): terrain refers to NULL object");
  RobotWorld& world = *worlds[index]->world;
  world.AddTerrain(name,new Terrain());
  *world.terrains.back() = *terrain.terrain;
  return this->terrain((int)world.terrains.size()-1);
}

void WorldModel::remove(const RobotModel& obj)
{
  if(obj.world != index) 
    throw PyException("Robot does not belong to this world");
  RobotWorld& world = *worlds[index]->world;
  if(obj.index < 0 || obj.index >= (int)world.robots.size())
    throw PyException("Invalid robot index");
  world.robots.erase(world.robots.begin()+obj.index);
}

void WorldModel::remove(const RigidObjectModel& obj)
{
  if(obj.world != index) 
    throw PyException("Rigid object does not belong to this world");
  RobotWorld& world = *worlds[index]->world;
  if(obj.index < 0 || obj.index >= (int)world.rigidObjects.size())
    throw PyException("Invalid rigid object index");
  world.rigidObjects.erase(world.rigidObjects.begin()+obj.index);
}

void WorldModel::remove(const TerrainModel& obj)
{
  if(obj.world != index) 
    throw PyException("Terrain does not belong to this world");
  RobotWorld& world = *worlds[index]->world;
  if(obj.index < 0 || obj.index >= (int)world.terrains.size())
    throw PyException("Invalid terrain index");
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
    *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geom.geomPtr) = world.GetGeometry(id);
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
    *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(geom.appearancePtr) = world.GetAppearance(id);
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
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  robotPtr->linkNames[index] = name;
}

int RobotModelLink::getIndex()
{
  return index;
}

int RobotModelLink::getParent()
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  return robotPtr->parents[index];
}

RobotModelLink RobotModelLink::parent()
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
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
  if(p < -1 || p >= (int)robotPtr->links.size())
    throw PyException("Invalid parent index");
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
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
  if(index < 0) return -1;
  RobotWorld& world = *worlds[this->world]->world;
  return world.RobotLinkID(robotIndex,index);
}

Geometry3D RobotModelLink::geometry()
{
  Geometry3D res;
  res.world = world;
  res.id = getID();
  if(res.id < 0)
    throw PyException("RobotModelLink is invalid");
  *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(res.geomPtr) = worlds[world]->world->GetGeometry(res.id); 
  return res;
}

Appearance RobotModelLink::appearance()
{
  Appearance res;
  res.world = world;
  res.id = getID();
  if(res.id < 0)
    throw PyException("RobotModelLink is invalid");
  *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(res.appearancePtr) = worlds[world]->world->GetAppearance(res.id);
  return res;
}


Mass::Mass()
: mass(1),com(3,0.0),inertia(3,1.0)
{}

void Mass::estimate(const Geometry3D& g,double mass,double surfaceFraction)
{
  shared_ptr<AnyCollisionGeometry3D>* gp = reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(g.geomPtr);
  Vector3 com = CenterOfMass(**gp,surfaceFraction);
  Matrix3 H = Inertia(**gp,com,mass,surfaceFraction);
  this->mass = mass;
  com.get(&this->com[0]);
  this->inertia.resize(9);
  H.get(&this->inertia[0]);
}

ContactParameters::ContactParameters()
: kFriction(0.5),kRestitution(0),kStiffness(Inf),kDamping(Inf)
{}

Mass RobotModelLink::getMass()
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
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
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
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
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  RobotLink3D& link=robotPtr->links[index];
  (link.T_World*Vector3(plocal)).get(pworld);
}

void RobotModelLink::getWorldDirection(const double vlocal[3],double vworld[3])
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  RobotLink3D& link=robotPtr->links[index];
  (link.T_World.R*Vector3(vlocal)).get(vworld);
}

void RobotModelLink::getLocalPosition(const double pworld[3],double plocal[3])
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  RobotLink3D& link=robotPtr->links[index];
  Vector3 temp;
  link.T_World.mulInverse(Vector3(pworld),temp);
  temp.get(plocal);
}

void RobotModelLink::getLocalDirection(const double vworld[3],double vlocal[3])
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  RobotLink3D& link=robotPtr->links[index];
  Vector3 temp;
  link.T_World.R.mulTranspose(Vector3(vworld),temp);
  temp.get(vlocal);
}


void RobotModelLink::getTransform(double R[9],double t[3])
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  const RobotLink3D& link=robotPtr->links[index];
  link.T_World.R.get(R);
  link.T_World.t.get(t);
}

void RobotModelLink::setTransform(const double R[9],const double t[3])
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  RobotLink3D& link=robotPtr->links[index];
  link.T_World.R.set(R);
  link.T_World.t.set(t);
  if (robotPtr->geometry[index])
    robotPtr->geometry[index]->SetTransform(link.T_World);
}

void RobotModelLink::getParentTransform(double R[9],double t[3])
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  const RobotLink3D& link=robotPtr->links[index];
  link.T0_Parent.R.get(R);
  link.T0_Parent.t.get(t);
}

void RobotModelLink::setParentTransform(const double R[9],const double t[3])
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  RobotLink3D& link=robotPtr->links[index];
  link.T0_Parent.R.set(R);
  link.T0_Parent.t.set(t);
}

void RobotModelLink::getAxis(double axis[3])
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  const RobotLink3D& link=robotPtr->links[index];
  link.w.get(axis);
}

void RobotModelLink::setAxis(const double axis[3])
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  RobotLink3D& link=robotPtr->links[index];
  link.w.set(axis);
}

bool RobotModelLink::isPrismatic()
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  RobotLink3D& link=robotPtr->links[index];
  return link.type == RobotLink3D::Prismatic;
}

bool RobotModelLink::isRevolute()
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  RobotLink3D& link=robotPtr->links[index];
  return link.type == RobotLink3D::Revolute;
}

void RobotModelLink::setPrismatic(bool prismatic)
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  RobotLink3D& link=robotPtr->links[index];
  link.type = (prismatic ? RobotLink3D::Prismatic : RobotLink3D::Revolute);
}

void RobotModelLink::getJacobian(const double p[3],vector<vector<double> >& J)
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  Matrix Jmat;
  robotPtr->GetFullJacobian(Vector3(p),index,Jmat);
  copy(Jmat,J);
}

void RobotModelLink::getPositionJacobian(const double p[3],vector<vector<double> >& J)
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  Matrix Jmat;
  robotPtr->GetPositionJacobian(Vector3(p),index,Jmat);
  copy(Jmat,J);
}

void RobotModelLink::getOrientationJacobian(vector<vector<double> >& J)
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
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
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  Vector3 v;
  robotPtr->GetWorldVelocity(Vector3(Zero),index,robotPtr->dq,v);
  v.get(out);
}

void RobotModelLink::getAngularVelocity(double out[3])
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  Vector3 v;
  robotPtr->GetWorldAngularVelocity(index,robotPtr->dq,v);
  v.get(out);
}

void RobotModelLink::getPointVelocity(const double plocal[3],double out[3])
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  Vector3 v;
  robotPtr->GetWorldVelocity(Vector3(plocal),index,robotPtr->dq,v);
  v.get(out);
}

void RobotModelLink::getAcceleration(const std::vector<double>& ddq,double out[3])
{
  double zero[3] = {0.0,0.0,0.0};
  getPointAcceleration(zero,ddq,out);
}

void RobotModelLink::getPointAcceleration(const double plocal[3],const std::vector<double>& ddq,double out[3])
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  Vector3 dw,dv;
  if(ddq.empty()) {
    robotPtr->GetResidualAcceleration(Vector3(plocal),index,dw,dv);
  }
  else{
    if((int)ddq.size() != robotPtr->q.n) 
      throw PyException("Invalid size of ddq");
    robotPtr->GetWorldAcceleration(Vector3(plocal),index,Vector((int)ddq.size(),&ddq[0]),dw,dv);
  }
  dv.get(out);
}

void RobotModelLink::getAngularAcceleration(const std::vector<double>& ddq,double out[3])
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  Vector3 dw,dv;
  if(ddq.empty()) {
    robotPtr->GetResidualAcceleration(Vector3(0.0),index,dw,dv);
  }
  else{
    if((int)ddq.size() != robotPtr->q.n) 
      throw PyException("Invalid size of ddq");
    robotPtr->GetWorldAcceleration(Vector3(0.0),index,Vector((int)ddq.size(),&ddq[0]),dw,dv);
  }
  dw.get(out);
}

void RobotModelLink::getPositionHessian(const double p[3],std::vector<std::vector<double> >& out,std::vector<std::vector<double> >& out2,std::vector<std::vector<double> >& out3)
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  Matrix Hx,Hy,Hz;
  Matrix* H[3] = {&Hx,&Hy,&Hz};
  robotPtr->GetPositionHessian(Vector3(p),index,H);
  copy(Hx,out);
  copy(Hy,out2);
  copy(Hz,out3);
}

void RobotModelLink::getOrientationHessian(std::vector<std::vector<double> >& out,std::vector<std::vector<double> >& out2,std::vector<std::vector<double> >& out3)
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  Matrix Hx,Hy,Hz;
  Matrix* H[3] = {&Hx,&Hy,&Hz};
  Matrix Hwx,Hwy,Hwz;
  Matrix* Hw[3] = {&Hwx,&Hwy,&Hwz};
  robotPtr->GetJacobianDeriv(Vector3(0.0),index,Hw,H);
  copy(Hwx,out);
  copy(Hwy,out2);
  copy(Hwz,out3);
}

void RobotModelLink::drawLocalGL(bool keepAppearance)
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  RobotWorld& world = *worlds[this->world]->world;
  if(keepAppearance) {
    world.robotViews[robotIndex].DrawLink_Local(index);
  }
  else
    world.robots[robotIndex]->DrawLinkGL(index);
}

void RobotModelLink::drawWorldGL(bool keepAppearance)
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
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
  :world(-1),index(-1),robot(NULL),dirty_dynamics(true)
{}

bool RobotModel::loadFile(const char* fn)
{
  if(!robot) throw PyException("RobotModel is empty");
  return robot->Load(fn);
}

bool RobotModel::saveFile(const char* fn,const char* geometryPrefix)
{
  if(!robot) throw PyException("RobotModel is empty");
  if(!robot->Save(fn)) return false;
  if(geometryPrefix) {
    for(size_t i=0;i<robot->links.size();i++) {
      if(!robot->IsGeometryEmpty(i) && robot->geomFiles[i].empty()) {
        robot->geomFiles[i] = robot->linkNames[i]+".off";
      }
    }
    if(!robot->SaveGeometry(geometryPrefix)) return false;
  }
  return true;
}

const char* RobotModel::getName() const
{
  if(!robot) throw PyException("RobotModel is empty");
  RobotWorld& world = *worlds[this->world]->world;
  return world.robots[index]->name.c_str();
}

void RobotModel::setName(const char* name)
{
  if(!robot) throw PyException("RobotModel is empty");
  RobotWorld& world = *worlds[this->world]->world;
  world.robots[index]->name = name;
}


int RobotModel::getID() const
{
  if(!robot) return -1;
  RobotWorld& world = *worlds[this->world]->world;
  return world.RobotID(index);
}

int RobotModel::numLinks()
{
  if(!robot) return -1;
  return robot->links.size();
}

RobotModelLink RobotModel::link(int linkindex)
{
  if(!robot) throw PyException("RobotModel is empty");
  RobotModelLink link;
  link.world = world;
  link.robotIndex = index;
  link.robotPtr = robot;
  link.index = linkindex;
  return link;
}

RobotModelLink RobotModel::link(const char* name)
{
  if(!robot) throw PyException("RobotModel is empty");
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
  if(!robot) return -1;
  return robot->drivers.size();
}

RobotModelDriver RobotModel::driver(int driverindex)
{
  if(!robot) throw PyException("RobotModel is empty");
  RobotModelDriver link;
  link.world = world;
  link.robotIndex = index;
  link.robotPtr = robot;
  link.index = driverindex;
  return link;
}

RobotModelDriver RobotModel::driver(const char* name)
{
  if(!robot) throw PyException("RobotModel is empty");
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
  if(!robot) throw PyException("RobotModel is empty");
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
  if(!robot) throw PyException("RobotModel is empty");
  RobotModelLink l = link(name);
  if(l.index < 0) throw PyException("Invalid DOF named");
  return getJointType(l.index);
}


void RobotModel::getConfig(vector<double>& q)
{
  if(!robot) throw PyException("RobotModel is empty");
  q.resize(robot->q.n);
  robot->q.getCopy(&q[0]);
}

void RobotModel::getVelocity(vector<double>& dq)
{
  if(!robot) throw PyException("RobotModel is empty");
  dq.resize(robot->dq.n);
  robot->dq.getCopy(&dq[0]);
}

void RobotModel::setConfig(const vector<double>& q)
{
  if(!robot) throw PyException("RobotModel is empty");
  if(robot->links.size() != q.size()) {
    throw PyException("Invalid size of configuration");
  }
  robot->q.copy(&q[0]);
  robot->UpdateFrames();
  robot->UpdateGeometry();
  dirty_dynamics = true;
}

void RobotModel::setVelocity(const vector<double>& dq)
{
  if(!robot) throw PyException("RobotModel is empty");
  if(robot->links.size() != dq.size()) {
    throw PyException("Invalid size of velocity");
  }
  robot->dq.copy(&dq[0]);
  dirty_dynamics = true;
}

void RobotModel::getJointLimits(vector<double>& qmin,vector<double>& qmax)
{
  if(!robot) throw PyException("RobotModel is empty");
  qmin.resize(robot->q.n);
  qmax.resize(robot->q.n);
  robot->qMin.getCopy(&qmin[0]);
  robot->qMax.getCopy(&qmax[0]);
}

void RobotModel::setJointLimits(const vector<double>& qmin,const vector<double>& qmax)
{
  if(!robot) throw PyException("RobotModel is empty");
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
  if(!robot) throw PyException("RobotModel is empty");
  vmax.resize(robot->q.n);
  robot->velMax.getCopy(&vmax[0]);
}

void RobotModel::setVelocityLimits(const vector<double>& vmax)
{
  if(!robot) throw PyException("RobotModel is empty");
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
  if(!robot) throw PyException("RobotModel is empty");
  amax.resize(robot->q.n);
  robot->accMax.getCopy(&amax[0]);
}

void RobotModel::setAccelerationLimits(const vector<double>& amax)
{
  if(!robot) throw PyException("RobotModel is empty");
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
  if(!robot) throw PyException("RobotModel is empty");
  tmax.resize(robot->q.n);
  robot->torqueMax.getCopy(&tmax[0]);
}

void RobotModel::setTorqueLimits(const vector<double>& tmax)
{
  if(!robot) throw PyException("RobotModel is empty");
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
  if(!robot) throw PyException("RobotModel is empty");
  if(i < 0 || i >= (int)robot->links.size()) {
    throw PyException("Invalid joint index");
  }
  robot->q(i) = qi;
  robot->UpdateFrames();
}

void RobotModel::setDOFPosition(const char* name,double qi)
{
  if(!robot) throw PyException("RobotModel is empty");
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
  if(!robot) throw PyException("RobotModel is empty");
  return robot->q(i);
}

double RobotModel::getDOFPosition(const char* name)
{
  if(!robot) throw PyException("RobotModel is empty");
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
  if(!robot) throw PyException("RobotModel is empty");
  Vector va(a),vb(b),vout;
  Interpolate(*robot,va,vb,u,vout);
  out = vout;
}

double RobotModel::distance(const std::vector<double>& a,const std::vector<double>& b)
{
  if(!robot) throw PyException("RobotModel is empty");
  if(robot->links.size() != a.size()) 
    throw PyException("Invalid size of configuration");
  if(robot->links.size() != b.size()) 
    throw PyException("Invalid size of configuration");
  Vector va(a),vb(b);
  return Distance(*robot,va,vb,2);
}

void RobotModel::interpolateDeriv(const std::vector<double>& a,const std::vector<double>& b,std::vector<double>& dout)
{
  if(!robot) throw PyException("RobotModel is empty");
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
  if(!robot) throw PyException("RobotModel is empty");
  if(link1 > link2) swap(link1,link2);
  if(link1 < 0 || link2 >= (int)robot->links.size())
    throw PyException("Invalid link(s) specified");
  return (robot->selfCollisions(link1,link2) != NULL);
}

void RobotModel::enableSelfCollision(int link1,int link2,bool value)
{
  if(!robot) throw PyException("RobotModel is empty");
  if (link1 > link2) swap(link1,link2);
  if(link1 < 0 || link2 >= (int)robot->links.size())
    throw PyException("Invalid link(s) specified");
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
  if(!robot) throw PyException("RobotModel is empty");
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
  if(!robot) throw PyException("RobotModel is empty");
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

void RobotModel::configToDrivers(const std::vector<double>& config,std::vector<double>& out)
{
  if(!robot) throw PyException("RobotModel is empty");
  if(config.size() != robot->links.size()) throw PyException("Invalid size of configuration");
  Config oldq = robot->q;
  robot->q.copy(&config[0]);
  out.resize(robot->drivers.size());
  for(size_t i=0;i<robot->drivers.size();i++) 
    out[i] = robot->GetDriverValue(i);
  robot->q = oldq;
}

void RobotModel::velocityToDrivers(const std::vector<double>& velocities,std::vector<double>& out)
{
  if(!robot) throw PyException("RobotModel is empty");
  if(velocities.size() != robot->links.size()) throw PyException("Invalid size of configuration");
  Config oldq = robot->dq;
  robot->dq.copy(&velocities[0]);
  out.resize(robot->drivers.size());
  for(size_t i=0;i<robot->drivers.size();i++) 
    out[i] = robot->GetDriverVelocity(i);
  robot->dq = oldq;
}

void RobotModel::configFromDrivers(const std::vector<double>& driverValues,std::vector<double>& out)
{
  if(!robot) throw PyException("RobotModel is empty");
  if(driverValues.size() != robot->drivers.size()) throw PyException("Invalid size of driver value vector");
  Config oldq = robot->q;
  for(size_t i=0;i<robot->drivers.size();i++) 
    robot->SetDriverValue(i,driverValues[i]);
  out.resize(robot->q.n);
  robot->q.getCopy(&out[0]);
  robot->q = oldq;
}

void RobotModel::velocityFromDrivers(const std::vector<double>& driverVelocities,std::vector<double>& out)
{
  if(!robot) throw PyException("RobotModel is empty");
  if(driverVelocities.size() != robot->drivers.size()) throw PyException("Invalid size of driver velocity vector");
  Config oldq = robot->dq;
  for(size_t i=0;i<robot->drivers.size();i++) 
    robot->SetDriverVelocity(i,driverVelocities[i]);
  out.resize(robot->q.n);
  robot->dq.getCopy(&out[0]);
  robot->dq = oldq;
}

void RobotModel::drawGL(bool keepAppearance)
{
  if(!robot) throw PyException("RobotModel is empty");
  if(!worlds[this->world]) throw PyException("RobotModel is associated with a deleted world");
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
  if(!robot) throw PyException("RobotModel is empty");
  Vector3 com = robot->GetCOM();
  com.get(out);
}

void RobotModel::getComVelocity(double out[3])
{
  if(!robot) throw PyException("RobotModel is empty");
  Vector3 h = robot->GetLinearMomentum();
  Vector3 dcm = h / robot->GetTotalMass();
  dcm.get(out);
}

void RobotModel::getComJacobian(std::vector<std::vector<double> >& out)
{
  if(!robot) throw PyException("RobotModel is empty");
  Matrix J;
  robot->GetCOMJacobian(J);
  copy(J,out);
}

void RobotModel::getLinearMomentum(double out[3])
{
  if(!robot) throw PyException("RobotModel is empty");
  Vector3 h = robot->GetLinearMomentum();
  h.get(out);
}

void RobotModel::getAngularMomentum(double out[3])
{
  if(!robot) throw PyException("RobotModel is empty");
  Vector3 k = robot->GetAngularMomentum();
  k.get(out);
}

double RobotModel::getKineticEnergy()
{
  if(!robot) throw PyException("RobotModel is empty");
  return robot->GetKineticEnergy();
}

void RobotModel::getTotalInertia(std::vector<std::vector<double> >& out)
{
  if(!robot) throw PyException("RobotModel is empty");
  Matrix3 H = robot->GetTotalInertia();
  out.resize(3);
  for(int i=0;i<3;i++) {
    out[i].resize(3);
    for(int j=0;j<3;j++) 
      out[i][j] = H(i,j);
  }
}

void RobotModel::getMassMatrix(std::vector<std::vector<double> >& B)
{
  if(!robot) throw PyException("RobotModel is empty");
  Matrix Bmat;
  /*
  if(dirty_dynamics) {
    robot->UpdateDynamics();
    dirty_dynamics = false;
  }
  robot->GetKineticEnergyMatrix(Bmat);
  */
  NewtonEulerSolver ne(*robot);
  ne.CalcKineticEnergyMatrix(Bmat);
  copy(Bmat,B);
}

void RobotModel::getMassMatrixInv(std::vector<std::vector<double> >& Binv)
{
  if(!robot) throw PyException("RobotModel is empty");
  Matrix Bmatinv;
  /*
  if(dirty_dynamics) {
    robot->UpdateDynamics();
    dirty_dynamics = false;
  }
  Matrix Bmat;
  robot->GetKineticEnergyMatrix(Bmat);
  LDLDecomposition<Real> ldl;
  ldl.set(Bmat);
  ldl.getInverse(Bmatinv);
  */
  NewtonEulerSolver ne(*robot);
  ne.CalcKineticEnergyMatrixInverse(Bmatinv);
  copy(Bmatinv,Binv);
}

void RobotModel::getMassMatrixDeriv(int i,std::vector<std::vector<double> >& out)
{
  if(!robot) throw PyException("RobotModel is empty");
  Matrix Bmat;
  if(dirty_dynamics) {
    robot->UpdateDynamics();
    dirty_dynamics = false;
  }
  robot->GetKineticEnergyMatrixDeriv(i,Bmat);
  copy(Bmat,out);
}

void RobotModel::getMassMatrixTimeDeriv(std::vector<std::vector<double> >& out)
{
  if(!robot) throw PyException("RobotModel is empty");
  Matrix Bmat;
  if(dirty_dynamics) {
    robot->UpdateDynamics();
    dirty_dynamics = false;
  }
  robot->GetKineticEnergyMatrixTimeDeriv(Bmat);
  copy(Bmat,out);
}

void RobotModel::getCoriolisForceMatrix(std::vector<std::vector<double> >& C)
{
  if(!robot) throw PyException("RobotModel is empty");
  Matrix Cmat;
  robot->UpdateDynamics();
  robot->GetCoriolisForceMatrix(Cmat);
  copy(Cmat,C);
}

void RobotModel::getCoriolisForces(std::vector<double>& C)
{
  if(!robot) throw PyException("RobotModel is empty");
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
  if(!robot) throw PyException("RobotModel is empty");
  Vector Gvec;
  robot->GetGravityTorques(Vector3(g),Gvec);
  copy(Gvec,G);
}

void RobotModel::torquesFromAccel(const std::vector<double>& ddq,std::vector<double>& out)
{
  if(!robot) throw PyException("RobotModel is empty");
  Vector ddqvec,tvec;
  copy(ddq,ddqvec);
  if(robot->links.size() > 6) {
    NewtonEulerSolver ne(*robot);
    ne.CalcTorques(ddqvec,tvec);
  }
  else {
    if(dirty_dynamics) {
      robot->UpdateDynamics();
      dirty_dynamics = false;
    }
    robot->CalcTorques(ddqvec,tvec);
  }
  copy(tvec,out);
}

void RobotModel::accelFromTorques(const std::vector<double>& t,std::vector<double>& out)
{
  if(!robot) throw PyException("RobotModel is empty");
  Vector ddqvec,tvec;
  if(robot->links.size() > 6) {
    copy(t,tvec);
    NewtonEulerSolver ne(*robot);
    ne.CalcAccel(tvec,ddqvec);
    copy(ddqvec,out);
  }
  else {
    copy(t,tvec);
    if(dirty_dynamics) {
      robot->UpdateDynamics();
      dirty_dynamics = false;
    }
    robot->CalcAcceleration(ddqvec,tvec);
    copy(ddqvec,out);
  }
}

void RobotModel::reduce(const RobotModel& fullRobot,std::vector<int>& out)
{
  if(!robot) throw PyException("RobotModel is empty");
  fullRobot.robot->Reduce(*robot,out);
}

void RobotModel::mount(int link,const RobotModel& subRobot,const double R[9],const double t[3])
{
  if(!robot) throw PyException("RobotModel is empty");
  RigidTransform T;
  T.R.set(R);
  T.t.set(t);
  const char* name = subRobot.getName();
  if(strlen(name)==0)
    robot->Mount(link,*subRobot.robot,T,NULL);
  else {
    robot->Mount(link,*subRobot.robot,T,name);
  }
}

SimRobotSensor RobotModel::sensor(int sensorIndex)
{
  if(!robot) throw PyException("RobotModel is empty");
  shared_ptr<WorldData> worldData = worlds[this->world];
  if(index >= (int)worldData->robotSensors.size())
    worldData->robotSensors.resize(index+1);
  if(!worldData->robotSensors[index]) {
    worldData->robotSensors[index].reset(new RobotSensors);
    worldData->robotSensors[index]->MakeDefault(robot);
  }
  RobotSensors* sensors = worldData->robotSensors[index].get();
  Assert(sensors != NULL);
  if(sensorIndex < 0 || sensorIndex >= (int)sensors->sensors.size()) 
    return SimRobotSensor(*this,NULL);
  return SimRobotSensor(*this,sensors->sensors[sensorIndex].get());
}

SimRobotSensor RobotModel::sensor(const char* name)
{
  if(!robot) throw PyException("RobotModel is empty");
  shared_ptr<WorldData> worldData = worlds[this->world];
  if(index >= (int)worldData->robotSensors.size())
    worldData->robotSensors.resize(index+1);
  if(!worldData->robotSensors[index]) {
    worldData->robotSensors[index].reset(new RobotSensors);
    worldData->robotSensors[index]->MakeDefault(robot);
  }
  RobotSensors* sensors = worldData->robotSensors[index].get();
  Assert(sensors != NULL);
  shared_ptr<SensorBase> sensor = sensors->GetNamedSensor(name);
  if(sensor==NULL) {
    fprintf(stderr,"Warning, sensor %s does not exist\n",name);
  }
  return SimRobotSensor(*this,sensor.get());
}

RigidObjectModel::RigidObjectModel()
  :world(-1),index(-1),object(NULL)
{}

bool RigidObjectModel::loadFile(const char* fn)
{
  if(!object) {
    throw PyException("Cannot load an empty rigid object, this needs to be part of a world");
  }
  return object->Load(fn);
}

bool RigidObjectModel::saveFile(const char* fn,const char* geometryName)
{
  if(!object) throw PyException("RigidObjectModel is invalid");
  if(!object->Save(fn)) return false;
  if(geometryName)
    if(!object->geometry->Save(geometryName)) return false;
  return true;
}

const char* RigidObjectModel::getName() const
{
  if(!object) return "";
  if(!worlds[this->world]) throw PyException("RigidObjectModel is associated with a deleted world");
  RobotWorld& world = *worlds[this->world]->world;
  return world.rigidObjects[index]->name.c_str();
}

void RigidObjectModel::setName(const char* name)
{
  if(!object) throw PyException("RigidObjectModel is invalid");
  if(!worlds[this->world]) throw PyException("RigidObjectModel is associated with a deleted world");
  RobotWorld& world = *worlds[this->world]->world;
  world.rigidObjects[index]->name = name;
}

int RigidObjectModel::getID() const
{
  if(!object) throw PyException("RigidObjectModel is invalid");
  if(!worlds[this->world]) throw PyException("RigidObjectModel is associated with a deleted world");
  RobotWorld& world = *worlds[this->world]->world;
  return world.RigidObjectID(index);
}

Geometry3D RigidObjectModel::geometry()
{
  if(!object) throw PyException("RigidObjectModel is invalid");
  Geometry3D res;
  res.world = world;
  res.id = getID();
  assert(res.id >= 0);
  *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(res.geomPtr) = worlds[world]->world->GetGeometry(res.id); 
  return res;
}

Appearance RigidObjectModel::appearance()
{
  if(!object) throw PyException("RigidObjectModel is invalid");
  Appearance res;
  res.world = world;
  res.id = getID();
  assert(res.id >= 0);
  *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(res.appearancePtr) = worlds[world]->world->GetAppearance(res.id);
  return res;
}

Mass RigidObjectModel::getMass()
{
  if(!object) throw PyException("RigidObjectModel is invalid");
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
  if(!object) throw PyException("RigidObjectModel is invalid");
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
  if(!object) throw PyException("RigidObjectModel is invalid");
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
  if(!object) throw PyException("RigidObjectModel is invalid");
  RigidObject* obj=object;
  obj->kFriction = params.kFriction;
  obj->kRestitution = params.kRestitution;
  obj->kStiffness = params.kStiffness;
  obj->kDamping = params.kDamping;
}

void RigidObjectModel::getTransform(double R[9],double t[3])
{
  if(!object) throw PyException("RigidObjectModel is invalid");
  RigidObject* obj=object;
  obj->T.R.get(R);
  obj->T.t.get(t);
}

void RigidObjectModel::setTransform(const double R[9],const double t[3])
{
  if(!object) throw PyException("RigidObjectModel is invalid");
  RigidObject* obj=object;
  obj->T.R.set(R);
  obj->T.t.set(t);
  obj->geometry->SetTransform(obj->T);
}

void RigidObjectModel::getVelocity(double out[3],double out2[3])
{
  if(!object) throw PyException("RigidObjectModel is invalid");
  RigidObject* obj=object;
  obj->w.get(out);
  obj->v.get(out2);
}

void RigidObjectModel::setVelocity(const double angularVelocity[3],const double velocity[3])
{
  if(!object) throw PyException("RigidObjectModel is invalid");
  RigidObject* obj=object;
  obj->w.set(angularVelocity);
  obj->v.set(velocity);
}

void RigidObjectModel::drawGL(bool keepAppearance)
{
  if(!object) throw PyException("RigidObjectModel is invalid");
  if(!worlds[this->world]) throw PyException("RigidObjectModel is associated with a deleted world");
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


bool TerrainModel::loadFile(const char* fn)
{
  if(!terrain) {
    throw PyException("Cannot load an empty TerrainModel; it must be part of a world.");
  } 
  return terrain->Load(fn);
}

bool TerrainModel::saveFile(const char* fn,const char* geometryName)
{
  if(!terrain) throw PyException("TerrainModel is invalid");
  if(!terrain->Save(fn)) return false;
  if(geometryName)
    if(!terrain->geometry->Save(geometryName)) return false;
  return true;
}

const char* TerrainModel::getName() const
{
  if(!terrain) throw PyException("TerrainModel is invalid");
  if(!worlds[this->world]) throw PyException("TerrainModel is associated with a deleted world");
  RobotWorld& world = *worlds[this->world]->world;
  return world.terrains[index]->name.c_str();
}

void TerrainModel::setName(const char* name)
{
  if(!terrain) throw PyException("TerrainModel is invalid");
  if(!worlds[this->world]) throw PyException("TerrainModel is associated with a deleted world");
  RobotWorld& world = *worlds[this->world]->world;
  world.terrains[index]->name = name;
}


int TerrainModel::getID() const
{
  if(!terrain) throw PyException("TerrainModel is invalid");
  if(!worlds[this->world]) throw PyException("TerrainModel is associated with a deleted world");
  RobotWorld& world = *worlds[this->world]->world;
  return world.TerrainID(index);
}

Geometry3D TerrainModel::geometry()
{
  if(!terrain) throw PyException("TerrainModel is invalid");
  Geometry3D res;
  res.world = world;
  res.id = getID();
  assert(res.id >= 0);
  *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(res.geomPtr) = worlds[world]->world->GetGeometry(res.id); 
  return res;
}


Appearance TerrainModel::appearance()
{
  if(!terrain) throw PyException("TerrainModel is invalid");
  Appearance res;
  res.world = world;
  res.id = getID();
  assert(res.id >= 0);
  *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(res.appearancePtr) = worlds[world]->world->GetAppearance(res.id);
  return res;
}

void TerrainModel::setFriction(double friction)
{
  if(!terrain) throw PyException("TerrainModel is invalid");
  terrain->SetUniformFriction(friction);
}

void TerrainModel::drawGL(bool keepAppearance)
{
  if(!terrain) throw PyException("TerrainModel is invalid");
  if(!worlds[this->world]) throw PyException("TerrainModel is associated with a deleted world");
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
  printf("Initializing simulation...\n");
  RobotWorld& rworld=*worlds[model.index]->world;
  sim->Init(&rworld);

  //setup controllers
  sim->robotControllers.resize(rworld.robots.size());
  for(size_t i=0;i<sim->robotControllers.size();i++) {
    Robot* robot=rworld.robots[i].get();
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

void Simulator::checkObjectOverlap(std::vector<int>& out,std::vector<int>& out2)
{
  vector<pair<ODEObjectID,ODEObjectID> > overlaps;
  sim->odesim.CheckObjectOverlap(overlaps);
  out.resize(overlaps.size());
  out2.resize(overlaps.size());
  for(size_t i=0;i<overlaps.size();i++) {
    out[i]=sim->ODEToWorldID(overlaps[i].first);
    out2[i]=sim->ODEToWorldID(overlaps[i].second);
  }
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
  if(robot < 0 || robot>= (int)sim->controlSimulators.size()) 
    throw PyException("Invalid robot index, out of bounds");
  Vector qv;
  sim->controlSimulators[robot].GetSimulatedConfig(qv);
  out = qv;
}

void Simulator::getActualVelocity(int robot,std::vector<double>& out)
{
  if(robot < 0 || robot>= (int)sim->controlSimulators.size()) 
    throw PyException("Invalid robot index, out of bounds");
  Vector qv;
  sim->controlSimulators[robot].GetSimulatedVelocity(qv);
  out = qv;
}

void Simulator::getActualTorque(int robot,std::vector<double>& out)
{
  if(robot < 0 || robot>= (int)sim->controlSimulators.size()) 
    throw PyException("Invalid robot index, out of bounds");
  Vector t;
  sim->controlSimulators[robot].GetActuatorTorques(t);
  out = t;
}

void Simulator::getActualTorques(int robot,std::vector<double>& out)
{
  if(robot < 0 || robot>= (int)sim->controlSimulators.size()) 
    throw PyException("Invalid robot index, out of bounds");
  static bool warned=false;
  if(!warned) {
    fprintf(stderr,"Warning: Simulator.getActualTorques will be deprecated. Use getActualTorque instead\n");
    warned = true;
  }
  getActualTorque(robot,out);
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

std::vector<std::string> Simulator::settings()
{
  std::vector<std::string> res; res.reserve(17);
  res.push_back("gravity");
  res.push_back("autoDisable");
  res.push_back("boundaryLayerCollisions");
  res.push_back("rigidObjectCollisions");
  res.push_back("robotSelfCollisions");
  res.push_back("robotRobotCollisions");
  res.push_back("adaptiveTimeStepping");
  res.push_back("minimumAdaptiveTimeStep");
  res.push_back("maxContacts");
  res.push_back("clusterNormalScale");
  res.push_back("errorReductionParameter");
  res.push_back("dampedLeastSquaresParameter");
  res.push_back("instabilityConstantEnergyThreshold");
  res.push_back("instabilityLinearEnergyThreshold");
  res.push_back("instabilityMaxEnergyThreshold");
  res.push_back("instabilityPostCorrectionEnergy");
  return res;
}

std::string Simulator::getSetting(const std::string& name)
{
  ODESimulatorSettings& settings = sim->odesim.GetSettings();
  stringstream ss;
  if(name == "gravity") ss << Vector3(settings.gravity);
  else if(name == "simStep") ss << sim->simStep;
  else if(name == "autoDisable") ss >> settings.autoDisable;
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
  if(name == "gravity") { Vector3 g; ss >> g; sim->odesim.SetGravity(g); }
  else if(name == "simStep") ss >> sim->simStep;
  else if(name == "autoDisable") { ss >> settings.autoDisable; sim->odesim.SetAutoDisable(settings.autoDisable); }
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
  if(robot < 0 || robot >= (int)sim->controlSimulators.size())
    throw PyException("Invalid robot index");
  SimRobotController c;
  c.sim = this;
  c.controller = &sim->controlSimulators[robot];
  c.index = robot;
  return c;
}


SimRobotController Simulator::controller(const RobotModel& robot)
{
  if(robot.index < 0 || robot.index >= (int)sim->controlSimulators.size())
    throw PyException("Invalid robot index");
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
  sim->sim->hooks.push_back(make_shared<WrenchHook>(body,Vector3(f),Vector3(t)));
  sim->sim->hooks.back()->autokill = true;
  //dBodyAddForce(body,f[0],f[1],f[2]);
  //dBodyAddTorque(body,t[0],t[1],t[2]);
}

void SimBody::applyForceAtPoint(const double f[3],const double pworld[3])
{
  if(!body) return;
  sim->sim->hooks.push_back(make_shared<ForceHook>(body,Vector3(pworld),Vector3(f)));
  sim->sim->hooks.back()->autokill = true;
  //dBodyAddForceAtPos(body,f[0],f[1],f[2],pworld[0],pworld[1],pworld[2]);
}

void SimBody::applyForceAtLocalPoint(const double f[3],const double plocal[3])
{
  if(!body) return;
  sim->sim->hooks.push_back(make_shared<LocalForceHook>(body,Vector3(plocal),Vector3(f)));
  sim->sim->hooks.back()->autokill = true;
  //dBodyAddForceAtRelPos(body,f[0],f[1],f[2],plocal[0],plocal[1],plocal[2]);
}

void SimBody::setVelocity(const double w[3],const double v[3])
{
  if(!body) return;
  dBodySetLinearVel(body,v[0],v[1],v[2]);
  dBodySetAngularVel(body,w[1],w[1],w[2]);
  ODEObjectID id = sim->sim->WorldToODEID(objectID);
  sim->sim->odesim.DisableInstabilityCorrection(id);
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

SimJoint::SimJoint()
:type(0),a(NULL),b(NULL),joint(0)
{}

SimJoint::~SimJoint()
{
  destroy();
}

void SimJoint::makeHinge(const SimBody& a,const SimBody& b,const double pt[3],const double axis[3])
{
  if(a.sim != b.sim)
    throw PyException("The two bodies are not part of the same simulation");
  destroy();
  type = 1;
  this->a = &a;
  this->b = &b;
  joint = dJointCreateHinge(a.sim->sim->odesim.world(),0);
  dJointAttach(joint,a.body,b.body);
  dJointSetHingeAnchor(joint,pt[0],pt[1],pt[2]);
  dJointSetHingeAxis(joint,axis[0],axis[1],axis[2]);
  dJointSetHingeParam(joint,dParamBounce,0);
  dJointSetHingeParam(joint,dParamFMax,0);
}

void SimJoint::makeHinge(const SimBody& a,const double pt[3],const double axis[3])
{
  destroy();
  type = 1;
  this->a = &a;
  this->b = NULL;
  joint = dJointCreateHinge(a.sim->sim->odesim.world(),0);
  dJointAttach(joint,a.body,0);
  dJointSetHingeAnchor(joint,pt[0],pt[1],pt[2]);
  dJointSetHingeAxis(joint,axis[0],axis[1],axis[2]);
  dJointSetHingeParam(joint,dParamBounce,0);
  dJointSetHingeParam(joint,dParamFMax,0);
}

void SimJoint::makeSlider(const SimBody& a,const SimBody& b,const double axis[3])
{
  if(a.sim != b.sim)
    throw PyException("The two bodies are not part of the same simulation");
  destroy();
  type = 2;
  this->a = &a;
  this->b = &b;
  joint = dJointCreateSlider(a.sim->sim->odesim.world(),0);
  dJointAttach(joint,a.body,b.body);
  dJointSetSliderAxis(joint,axis[0],axis[1],axis[2]);
  dJointSetSliderParam(joint,dParamBounce,0);
  dJointSetSliderParam(joint,dParamFMax,0);
}

void SimJoint::makeSlider(const SimBody& a,const double axis[3])
{
  destroy();
  type = 2;
  this->a = &a;
  this->b = NULL;
  joint = dJointCreateSlider(a.sim->sim->odesim.world(),0);
  dJointAttach(joint,a.body,NULL);
  dJointSetSliderAxis(joint,axis[0],axis[1],axis[2]);
  dJointSetSliderParam(joint,dParamBounce,0);
  dJointSetSliderParam(joint,dParamFMax,0);
}

void SimJoint::makeFixed(const SimBody& a,const SimBody& b)
{
  if(a.sim != b.sim)
    throw PyException("The two bodies are not part of the same simulation");
  destroy();
  type = 2;
  this->a = &a;
  this->b = &b;
  joint = dJointCreateFixed(a.sim->sim->odesim.world(),0);
  dJointAttach(joint,a.body,b.body);
  dJointSetFixed(joint);
}

void SimJoint::destroy()
{
  if(joint) {
    dJointDestroy(joint);
    joint = 0;
  }
  a = NULL;
  b = NULL;
}

void SimJoint::setLimits(double min,double max)
{
  if(!joint) throw PyException("Joint has not yet been made, call makeX before setX");
  if(type == 1) {
    dJointSetHingeParam(joint,dParamLoStop,min);
    dJointSetHingeParam(joint,dParamHiStop,max);
  }
  else if(type == 2) {
    dJointSetSliderParam(joint,dParamLoStop,min);
    dJointSetSliderParam(joint,dParamHiStop,max); 
  }
}

void SimJoint::setFriction(double friction)
{
  setVelocity(0,friction);
}

void SimJoint::setVelocity(double vel,double fmax)
{
  if(!joint) throw PyException("Joint has not yet been made, call makeX before setX");
  if(type == 1) {
    dJointSetHingeParam(joint,dParamVel,vel);
    dJointSetHingeParam(joint,dParamFMax,fmax);
  }
  else if(type == 2) {
    dJointSetSliderParam(joint,dParamVel,vel);
    dJointSetSliderParam(joint,dParamFMax,fmax);
  }
}

void SimJoint::addForce(double force)
{
  if(!joint) throw PyException("Joint has not yet been made, call makeX before addForce");
  if(type == 1) 
    dJointAddHingeTorque(joint,force);
  else if(type == 2)
    dJointAddSliderForce(joint,force);
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
  if(!controller) throw PyException("Invalid SimRobotController");
  controller->controlTimeStep = dt;
}

double SimRobotController::getRate()
{
  if(!controller) throw PyException("Invalid SimRobotController");
  return controller->controlTimeStep;
}

void SimRobotController::getCommandedConfig(vector<double>& q)
{
  if(!controller) throw PyException("Invalid SimRobotController");
  Vector qv;
  controller->GetCommandedConfig(qv);
  q.resize(qv.n);
  qv.getCopy(&q[0]);
}

void SimRobotController::getCommandedVelocity(vector<double>& dq)
{
  if(!controller) throw PyException("Invalid SimRobotController");
  Vector qv;
  controller->GetCommandedVelocity(qv);
  dq.resize(qv.n);
  qv.getCopy(&dq[0]);
}

void SimRobotController::getCommandedTorque(std::vector<double>& t)
{
  if(!controller) throw PyException("Invalid SimRobotController");
  RobotMotorCommand& command = controller->command;
  //Robot* robot=sim->sim->controlSimulators[index];
  t.resize(command.actuators.size());
  for(size_t i=0;i<command.actuators.size();i++) 
    t[i] = command.actuators[i].torque;
}

void SimRobotController::getSensedConfig(vector<double>& q)
{
  if(!controller) throw PyException("Invalid SimRobotController");
  Vector qv;
  controller->GetSensedConfig(qv);
  if(!qv.empty()) {
    q.resize(qv.n);
    qv.getCopy(&q[0]);
  }
}

void SimRobotController::getSensedVelocity(vector<double>& dq)
{
  if(!controller) throw PyException("Invalid SimRobotController");
  Vector qv;
  controller->GetSensedVelocity(qv);
  if(!qv.empty()) {
    dq.resize(qv.n);
    qv.getCopy(&dq[0]);
  }
}

void SimRobotController::getSensedTorque(std::vector<double>& t)
{
  if(!controller) throw PyException("Invalid SimRobotController");
  DriverTorqueSensor* s = controller->sensors.GetTypedSensor<DriverTorqueSensor>();
  if(s==NULL){
      throw PyException("Robot has no torque sensor");
  }
  else {
    //resize to the correct size
    if(s->indices.empty() || s->t.empty())
      t = s->t;
    else {
      t.resize(controller->robot->q.n);
      fill(t.begin(),t.end(),0.0);
      for(size_t i=0;i<s->indices.size();i++)
        t[s->indices[i]] = s->t[i];
    }
  }
}

SimRobotSensor::SimRobotSensor(const RobotModel& _robot,SensorBase* _sensor)
  :robotModel(_robot),sensor(_sensor)
{}

SimRobotSensor::SimRobotSensor(SimRobotController& _controller,const char* name,const char* type)
  :sensor(NULL)
{
  robotModel = _controller.model();
  shared_ptr<SensorBase> newsensor = _controller.controller->sensors.CreateByType(type);
  if(!newsensor) {
    throw PyException("Invalid sensor type specified");
  }
  if(_controller.controller->sensors.GetNamedSensor(name)) {
    throw PyException("Sensor name already exists");
  }
  newsensor->name = name;
  _controller.controller->sensors.sensors.push_back(newsensor);
  _controller.controller->nextSenseTime.push_back(_controller.controller->curTime);
  sensor = _controller.controller->sensors.sensors.back().get();
}

RobotModel SimRobotSensor::robot()
{
  return robotModel;
}

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

std::vector<std::string> SimRobotSensor::settings()
{
  std::vector<std::string> res;
  if(!sensor) return res;
  std::map<std::string,std::string> s = sensor->Settings();
  for(auto& i:s)
    res.push_back(i.first);
  return res;
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
  sensor->DrawGL(*robotModel.robot,measurements);
}

void SimRobotSensor::kinematicSimulate(double dt)
{
  if(!sensor) return;
  sensor->SimulateKinematic(*robotModel.robot,*worlds[robotModel.world]->world);
  sensor->Advance(dt);
}

void SimRobotSensor::kinematicSimulate(WorldModel& world,double dt)
{
  if(!sensor) return;
  sensor->SimulateKinematic(*robotModel.robot,*worlds[world.index]->world);
  sensor->Advance(dt);
}

void SimRobotSensor::kinematicReset()
{
  if(!sensor) return;
  sensor->Reset();
}


SimRobotSensor SimRobotController::sensor(int sensorIndex)
{
  if(!controller) throw PyException("Invalid SimRobotController");
  RobotSensors& sensors = controller->sensors;
  if(sensorIndex < 0 || sensorIndex >= (int)sensors.sensors.size()) 
    return SimRobotSensor(RobotModel(),NULL);
  return SimRobotSensor(model(),sensors.sensors[sensorIndex].get());
}

SimRobotSensor SimRobotController::sensor(const char* name)
{
  if(!controller) throw PyException("Invalid SimRobotController");
  RobotSensors& sensors = controller->sensors;
  shared_ptr<SensorBase> sensor = sensors.GetNamedSensor(name);
  if(sensor==NULL) {
    fprintf(stderr,"Warning, sensor %s does not exist\n",name);
  }
  return SimRobotSensor(model(),sensor.get());
}

std::vector<std::string> SimRobotController::commands()
{
  if(!controller) throw PyException("Invalid SimRobotController");
  return controller->controller->Commands();
}

void SimRobotController::setManualMode(bool enabled)
{
  if(!controller) throw PyException("Invalid SimRobotController");
  RobotController* c=sim->sim->robotControllers[index].get();
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
  if(!controller) throw PyException("Invalid SimRobotController");
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

std::vector<std::string> SimRobotController::settings()
{
  std::vector<std::string> res;
  if(!controller) return res;
  std::map<std::string,std::string> s = controller->controller->Settings();
  for(auto& i:s)
    res.push_back(i.first);
  return res;
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
        fprintf(stderr,"First simulation cycle: the path controller needs to read from the encoders before motion commands can be issued\n");
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
  EnablePathControl(sim->sim->robotControllers[index].get());
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
  EnablePathControl(sim->sim->robotControllers[index].get());
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
  EnablePathControl(sim->sim->robotControllers[index].get());
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
  EnablePathControl(sim->sim->robotControllers[index].get());
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
  EnablePathControl(sim->sim->robotControllers[index].get());
  PolynomialMotionQueue* mq = GetMotionQueue(controller->controller);
  mq->Cut(0);
  mq->AppendLinear(Vector(q),dt);
}
void SimRobotController::setCubic(const std::vector<double>& q,const std::vector<double>& v,double dt)
{
  if(controller->robot->links.size() != q.size()) {
    throw PyException("Invalid size of configuration");
  }
  if(controller->robot->links.size() != v.size()) {
    throw PyException("Invalid size of velocity");
  }
  EnablePathControl(sim->sim->robotControllers[index].get());
  PolynomialMotionQueue* mq = GetMotionQueue(controller->controller);
  mq->Cut(0);
  mq->AppendCubic(Vector(q),Vector(v),dt);
}
void SimRobotController::addLinear(const std::vector<double>& q,double dt)
{
  if(controller->robot->links.size() != q.size()) {
    throw PyException("Invalid size of configuration");
  }
  EnablePathControl(sim->sim->robotControllers[index].get());
  PolynomialMotionQueue* mq = GetMotionQueue(controller->controller);
  mq->AppendLinear(Vector(q),dt);
}

void SimRobotController::addCubic(const std::vector<double>& q,const std::vector<double>& v,double dt)
{
  if(controller->robot->links.size() != q.size()) {
    throw PyException("Invalid size of configuration");
  }
  if(controller->robot->links.size() != v.size()) {
    throw PyException("Invalid size of velocity");
  }
  EnablePathControl(sim->sim->robotControllers[index].get());
  PolynomialMotionQueue* mq = GetMotionQueue(controller->controller);
  mq->AppendCubic(Vector(q),Vector(v),dt);
}

void SimRobotController::addMilestone(const vector<double>& q,const vector<double>& dq)
{
  if(controller->robot->links.size() != q.size()) {
    throw PyException("Invalid size of configuration");
  }
  if(controller->robot->links.size() != dq.size()) {
    throw PyException("Invalid size of velocity");
  }
  EnablePathControl(sim->sim->robotControllers[index].get());
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
  if(dt < 0) {
    throw PyException("Negative dt");
  }
  EnablePathControl(sim->sim->robotControllers[index].get());
  PolynomialMotionQueue* mq = GetMotionQueue(controller->controller);
  Config qv(controller->robot->links.size(),&dq[0]);
  stringstream ss;
  ss<<mq->CurTime()+dt<<"\t"<<qv;
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
  RobotController* c=sim->sim->robotControllers[index].get();
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
  RobotController* c=sim->sim->robotControllers[index].get();
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
  widgets[index].widget = make_shared<GLDraw::WidgetSet>();
}

void WidgetSet::add(const Widget& subwidget)
{
  GLDraw::WidgetSet* ws=dynamic_cast<GLDraw::WidgetSet*>(widgets[index].widget.get());
  ws->widgets.push_back(widgets[subwidget.index].widget.get());
  ws->widgetEnabled.push_back(true);
  refWidget(subwidget.index);
}
void WidgetSet::remove(const Widget& subwidget)
{
  GLDraw::WidgetSet* ws=dynamic_cast<GLDraw::WidgetSet*>(widgets[index].widget.get());
  GLDraw::Widget* subwidget_ptr = widgets[subwidget.index].widget.get();
  for(size_t i=0;i<ws->widgets.size();i++)
    if(ws->widgets[i] == subwidget_ptr) {
      //delete it
      ws->widgets.erase(ws->widgets.begin()+i);
      ws->widgetEnabled.erase(ws->widgetEnabled.begin()+i);
      if(ws->activeWidget == subwidget_ptr)
        ws->activeWidget = NULL;
      if(ws->closestWidget == subwidget_ptr)
        ws->closestWidget = NULL;
      derefWidget(subwidget.index);
      if(widgets[subwidget.index].widget == NULL) 
        return;
      i--;
    }
}

void WidgetSet::enable(const Widget& subwidget,bool enabled)
{
  GLDraw::WidgetSet* ws=dynamic_cast<GLDraw::WidgetSet*>(widgets[index].widget.get());
  GLDraw::Widget* subwidget_ptr = widgets[subwidget.index].widget.get();
  for(size_t i=0;i<ws->widgets.size();i++)
    if(ws->widgets[i] == subwidget_ptr) {
      if(ws->activeWidget == subwidget_ptr)
        ws->activeWidget = NULL;
      if(ws->closestWidget == subwidget_ptr)
        ws->closestWidget = NULL;
      ws->widgetEnabled[i] = enabled;
    }
}

PointPoser::PointPoser()
  :Widget()
{
  widgets[index].widget = make_shared<GLDraw::TransformWidget>();
  GLDraw::TransformWidget* tw=dynamic_cast<GLDraw::TransformWidget*>(widgets[index].widget.get());
  tw->enableRotation = false;
}

void PointPoser::set(const double t[3])
{
  GLDraw::TransformWidget* tw=dynamic_cast<GLDraw::TransformWidget*>(widgets[index].widget.get());
  tw->T.t.set(t);
}

void PointPoser::get(double out[3])
{
  GLDraw::TransformWidget* tw=dynamic_cast<GLDraw::TransformWidget*>(widgets[index].widget.get());
  tw->T.t.get(out);
}


void PointPoser::setAxes(const double R[9])
{
  GLDraw::TransformWidget* tw=dynamic_cast<GLDraw::TransformWidget*>(widgets[index].widget.get());
  tw->T.R.set(R);
}

void PointPoser::enableAxes(bool x,bool y,bool z)
{
  GLDraw::TransformWidget* tw=dynamic_cast<GLDraw::TransformWidget*>(widgets[index].widget.get());
  tw->enableTranslationAxes[0]=x;
  tw->enableTranslationAxes[1]=y;
  tw->enableTranslationAxes[2]=z;
}


TransformPoser::TransformPoser()
  :Widget()
{
  widgets[index].widget = make_shared<GLDraw::TransformWidget>();
}

void TransformPoser::set(const double R[9],const double t[3])
{
  GLDraw::TransformWidget* tw=dynamic_cast<GLDraw::TransformWidget*>(widgets[index].widget.get());
  tw->T.R.set(R);
  tw->T.t.set(t);
}

void TransformPoser::get(double out[9],double out2[3])
{
  GLDraw::TransformWidget* tw=dynamic_cast<GLDraw::TransformWidget*>(widgets[index].widget.get());
  tw->T.R.get(out);
  tw->T.t.get(out2);
}

void TransformPoser::enableTranslation(bool enable)
{
  GLDraw::TransformWidget* tw=dynamic_cast<GLDraw::TransformWidget*>(widgets[index].widget.get());
  tw->enableTranslation = enable;
}

void TransformPoser::enableRotation(bool enable)
{
  GLDraw::TransformWidget* tw=dynamic_cast<GLDraw::TransformWidget*>(widgets[index].widget.get());
  tw->enableRotation = enable;
}

ObjectPoser::ObjectPoser(RigidObjectModel& object)
  :Widget()
{
  RobotWorld& world = *worlds[object.world]->world;
  RigidObject* obj = world.rigidObjects[object.index].get();
  widgets[index].widget = make_shared<RigidObjectPoseWidget>(obj);
}

void ObjectPoser::set(const double R[9],const double t[3])
{
  RigidObjectPoseWidget* tw=dynamic_cast<RigidObjectPoseWidget*>(widgets[index].widget.get());
  RigidTransform T;
  T.R.set(R);
  T.t.set(t);
  tw->SetPose(T);
}

void ObjectPoser::get(double out[9],double out2[3])
{
  RigidObjectPoseWidget* tw=dynamic_cast<RigidObjectPoseWidget*>(widgets[index].widget.get());
  RigidTransform T = tw->Pose();
  T.R.get(out);
  T.t.get(out2);
}

RobotPoser::RobotPoser(RobotModel& robot)
{
  Assert(worlds[robot.world]->world != NULL);
  RobotWorld& world = *worlds[robot.world]->world;
  Assert(robot.index >= 0 && robot.index < world.robots.size());
  Robot* rob = world.robots[robot.index].get();
  ViewRobot* view = &world.robotViews[robot.index];
  Assert(rob != NULL);
  Assert(view != NULL);
  widgets[index].widget = make_shared<RobotPoseWidget>(rob,view);
}

void RobotPoser::setActiveDofs(const std::vector<int>& dofs)
{
  RobotPoseWidget* tw=dynamic_cast<RobotPoseWidget*>(widgets[index].widget.get());
  tw->SetActiveDofs(dofs);
}

void RobotPoser::set(const std::vector<double>& q)
{
  RobotPoseWidget* tw=dynamic_cast<RobotPoseWidget*>(widgets[index].widget.get());
  tw->SetPose(Config(q));
}

void RobotPoser::get(std::vector<double>& out)
{
  RobotPoseWidget* tw=dynamic_cast<RobotPoseWidget*>(widgets[index].widget.get());
  out.resize(tw->Pose().size());
  tw->Pose().getCopy(&out[0]);
}

void RobotPoser::getConditioned(const std::vector<double>& qref,std::vector<double>& out)
{
  RobotPoseWidget* tw=dynamic_cast<RobotPoseWidget*>(widgets[index].widget.get());
  out.resize(tw->Pose().size());
  tw->Pose_Conditioned(Config(qref)).getCopy(&out[0]);
}

void RobotPoser::addIKConstraint(const IKObjective& obj)
{
  RobotPoseWidget* tw=dynamic_cast<RobotPoseWidget*>(widgets[index].widget.get());
  tw->ikPoser.ClearLink(obj.goal.link);
  tw->ikPoser.Add(obj.goal);
  tw->ikPoser.Enable(&tw->ikPoser.poseWidgets.back(),false);
}

void RobotPoser::clearIKConstraints()
{
  RobotPoseWidget* tw=dynamic_cast<RobotPoseWidget*>(widgets[index].widget.get());
  tw->ikPoser.poseGoals.clear();
  tw->ikPoser.poseWidgets.clear();
}

AABBPoser::AABBPoser()
  :Widget()
{
  AABB3D bb;
  bb.bmin.set(0,0,0);
  bb.bmax.set(1,1,1);
  widgets[index].widget = make_shared<GLDraw::BoxWidget>(bb);
}

void AABBPoser::setFrame(const double R[9],const double t[3])
{
  GLDraw::BoxWidget* tw=dynamic_cast<GLDraw::BoxWidget*>(widgets[index].widget.get());
  tw->T.R.set(R);
  tw->T.t.set(t);
  tw->transformWidget.T.R.set(R);
  tw->transformWidget.T.t = 0.5*(tw->T*(tw->bb.bmin+tw->bb.bmax));
}

void AABBPoser::set(const double bmin[3],const double bmax[3])
{
  GLDraw::BoxWidget* tw=dynamic_cast<GLDraw::BoxWidget*>(widgets[index].widget.get());
  tw->bb.bmin.set(bmin);
  tw->bb.bmax.set(bmax);
}

void AABBPoser::get(double out[3],double out2[3])
{
  GLDraw::BoxWidget* tw=dynamic_cast<GLDraw::BoxWidget*>(widgets[index].widget.get());
  tw->bb.bmin.get(out);
  tw->bb.bmax.get(out2);
}

SpherePoser::SpherePoser()
  :Widget()
{
  Sphere3D s;
  s.center.set(0,0,0);
  s.radius = 1;
  widgets[index].widget = make_shared<GLDraw::SphereWidget>(s);
}

void SpherePoser::set(const double cr[4])
{
  GLDraw::SphereWidget* tw=dynamic_cast<GLDraw::SphereWidget*>(widgets[index].widget.get());
  tw->transformWidget.T.t.set(cr);
  tw->radius = cr[3];
}

void SpherePoser::get(double out[4])
{
  GLDraw::SphereWidget* tw=dynamic_cast<GLDraw::SphereWidget*>(widgets[index].widget.get());
  tw->transformWidget.T.t.get(out);
  out[3] = tw->radius;
}

BoxPoser::BoxPoser()
  :Widget()
{
  Box3D bb;
  bb.origin.setZero();
  bb.xbasis.set(1,0,0);
  bb.ybasis.set(0,1,0);
  bb.zbasis.set(0,0,1);
  bb.dims.set(1,1,1);
  widgets[index].widget = make_shared<GLDraw::BoxWidget>(bb);
}

void BoxPoser::set(const double R[9],const double t[3],const double dims[3])
{
  GLDraw::BoxWidget* tw=dynamic_cast<GLDraw::BoxWidget*>(widgets[index].widget.get());
  tw->T.R.set(R);
  tw->T.t.set(t);
  tw->bb.bmin.setZero();
  tw->bb.bmax.set(dims);
  tw->transformWidget.T.R.set(R);
  tw->transformWidget.T.t = tw->T*(0.5*(tw->bb.bmin+tw->bb.bmax));
}

void BoxPoser::setTransform(const double R[9],const double t[3])
{
  GLDraw::BoxWidget* tw=dynamic_cast<GLDraw::BoxWidget*>(widgets[index].widget.get());
  tw->T.R.set(R);
  tw->T.t.set(t);
  tw->transformWidget.T.R.set(R);
  tw->transformWidget.T.t = tw->T*(0.5*(tw->bb.bmin+tw->bb.bmax));
}

void BoxPoser::setDims(const double dims[3])
{
  GLDraw::BoxWidget* tw=dynamic_cast<GLDraw::BoxWidget*>(widgets[index].widget.get());
  tw->bb.bmin.setZero();
  tw->bb.bmax = tw->bb.bmin + Vector3(dims);
  tw->transformWidget.T.t = tw->T*(0.5*(tw->bb.bmin+tw->bb.bmax));
}

void BoxPoser::getTransform(double out[9],double out2[3])
{
  GLDraw::BoxWidget* tw=dynamic_cast<GLDraw::BoxWidget*>(widgets[index].widget.get());
  tw->transformWidget.T.R.get(out);
  Vector3 temp = tw->transformWidget.T * tw->bb.bmin;
  temp.get(out2);
}

void BoxPoser::getDims(double out[3])
{
  GLDraw::BoxWidget* tw=dynamic_cast<GLDraw::BoxWidget*>(widgets[index].widget.get());
  Vector3 temp = tw->bb.bmax - tw->bb.bmin;
  temp.get(out);
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
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(g.geomPtr);
  if(0==strcmp(protocol,"ros")) {
    if(0==strcmp(type,""))
      type = "PointCloud";
    if(0 == strcmp(type,"PointCloud")) {
      if(!g.isStandalone()) {
  RobotWorld& world=*worlds[g.world]->world;
  GetManagedGeometry(world,g.id).RemoveFromCache();
  return GetManagedGeometry(world,g.id).Load((string("ros:PointCloud2//")+string(name)).c_str());
      }
      printf("Warning, attaching to a ROS stream without a ManagedGeometry.\n");
      printf("You will not be able to automatically get updates from ROS.\n");
      if(!geom) 
        geom.reset(new AnyCollisionGeometry3D());
      (*geom) = AnyCollisionGeometry3D(Meshing::PointCloud3D());
      return ROSSubscribePointCloud(geom->AsPointCloud(),name);
      //TODO: update ROS, update the appearance every time the point cloud changes
    }
    else {
      throw PyException("SubscribeToStream(Geometry3D): Unsupported type argument");
      return false;
    }
  }
  else {
    throw PyException("SubscribeToStream(Geometry3D): Unsupported protocol argument");
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

