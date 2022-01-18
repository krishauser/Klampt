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
#include <Klampt/Simulation/Simulator.h>
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
#include <KrisLibrary/Timer.h>
#include <ode/ode.h>
#include "pyerr.h"
#include "pyconvert.h"
#include "robotik.h"
#include <fstream>
#ifndef WIN32
#include <unistd.h>
#endif //WIN32
using namespace Geometry;

inline void MakeNumpyArray(double** out,int* m,int* n,int _m,int _n,Matrix& ref)
{
  *m = _m;
  *n = _n;
  *out = (double*)malloc(_m*_n*sizeof(double));
  ref.setRef(*out,_m*_n,0,_n,1,_m,_n);
}

/***************************  GLOBALS: REFERENCING TO KLAMPT C++ CODE ***************************************/

/// Internally used.
struct WorldData
{
  Klampt::WorldModel* world;
  bool worldExternal;
  Klampt::XmlWorld xmlWorld;
  int refCount;
  vector<shared_ptr<Klampt::RobotSensors> > robotSensors;
};

/// Internally used.
struct SimData
{
  Klampt::Simulator sim;
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

int createWorld(Klampt::WorldModel* ptr=NULL)
{
  if(worldDeleteList.empty()) {
    worlds.push_back(make_shared<WorldData>());
    if(ptr) {
      worlds.back()->world = ptr;
      worlds.back()->worldExternal = true;
    }
    else {
      worlds.back()->world = new Klampt::WorldModel;
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
      worlds[index]->world = new Klampt::WorldModel;
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
  Klampt::ManagedGeometry::manager.Clear();
}

void set_random_seed(int seed)
{
  Math::Srand(seed);
}


/***************************  GEOMETRY CODE ***************************************/

Klampt::ManagedGeometry& GetManagedGeometry(Klampt::WorldModel& world,int id)
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


class ManualOverrideController : public Klampt::RobotController
{
 public:
  ManualOverrideController(Klampt::RobotModel& robot,const shared_ptr<Klampt::RobotController>& _base)
    :Klampt::RobotController(robot),base(_base),override(false)
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

  shared_ptr<Klampt::RobotController> base;
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
  if(!Klampt::RobotController::ReadState(f)) {
    printf("Unable to read Klampt::RobotController\n");
    return false;
  }
  return true;
}

bool ManualOverrideController::WriteState(File& f) const
{
  if(!WriteFile(f,override)) return false;
  if(!override) return base->WriteState(f);
  return Klampt::RobotController::WriteState(f);
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
  Klampt::RobotController::Update(dt);
}


typedef ManualOverrideController MyController;
inline shared_ptr<MyController> MakeController(Klampt::RobotModel* robot)
{
  ManualOverrideController* lc=new ManualOverrideController(*robot,MakeDefaultController(robot));
  return shared_ptr<MyController>(lc);
}
inline Klampt::PolynomialPathController* GetPathController(Klampt::RobotController* controller)
{
  MyController* mc=dynamic_cast<MyController*>(controller);
  if(!mc) {
    throw PyException("Not using the default manual override controller");
  }
  Klampt::LoggingController* lc=dynamic_cast<Klampt::LoggingController*>(mc->base.get());
  if(!lc) {
    throw PyException("Not using the default robot controller");
  }
  Klampt::FeedforwardController* ffc=dynamic_cast<Klampt::FeedforwardController*>(lc->base.get());
  Klampt::PolynomialPathController* pc=dynamic_cast<Klampt::PolynomialPathController*>(ffc->base.get());
  return pc;
}
inline Klampt::PolynomialMotionQueue* GetMotionQueue(Klampt::RobotController* controller)
{
  return GetPathController(controller);
}


void GetMesh(const AnyCollisionGeometry3D& geom,TriangleMesh& tmesh)
{
  Assert(geom.type == AnyGeometry3D::TriangleMesh);
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

void GetMesh(const TriangleMesh& tmesh,AnyCollisionGeometry3D& geom)
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


void GetPointCloud(const AnyCollisionGeometry3D& geom,PointCloud& pc)
{
  Assert(geom.type == AnyGeometry3D::PointCloud);
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

void GetPointCloud(const PointCloud& pc,AnyCollisionGeometry3D& geom)
{
  geom = Meshing::PointCloud3D();
  Meshing::PointCloud3D& gpc = geom.AsPointCloud();
  gpc.settings = pc.settings;
  gpc.points.resize(pc.vertices.size()/3);
  int k=0;
  //const double* v=&pc.vertices[0];
  //for(size_t i=0;i<gpc.points.size();i++,k+=3)
  //  gpc.points[i].set(v[k],v[k+1],v[k+2]);
  assert(sizeof(Vector3) == sizeof(double)*3);
  memcpy(&gpc.points[0],&pc.vertices[0],sizeof(double)*3*gpc.points.size());
  gpc.propertyNames = pc.propertyNames;
  if(pc.propertyNames.size() > 0) {
    if(pc.properties.size() != gpc.points.size()*pc.propertyNames.size()) {
      printf("Expected %d = %d*%d properties, got %d\n",(int)gpc.points.size(),(int)pc.propertyNames.size(),(int)gpc.points.size()*pc.propertyNames.size(),(int)pc.properties.size());
      throw PyException("GetPointCloud: Invalid number of properties in PointCloud");
    }
    gpc.properties.resize(pc.properties.size() / pc.propertyNames.size());
    gpc.properties[0].resize(pc.properties.size());
    //gpc.properties[0].copy(&pc.properties[0]);
    memcpy(&gpc.properties[0][0],&pc.properties[0],sizeof(double)*pc.properties.size());
    
    int m=(int)pc.propertyNames.size();
    int k=0;
    for(size_t i=1;i<gpc.properties.size();i++,k+=m) {
      //gpc.properties[i].resize(pc.propertyNames.size());
      //gpc.properties[i].copy(&pc.properties[i*pc.propertyNames.size()]);
      //if(i != 0)
        gpc.properties[i].setRef(gpc.properties[0],k,1,m);
    }
    gpc.properties[0].n = m;
  }
  geom.ClearCollisionData();
}

void GetVolumeGrid(const AnyCollisionGeometry3D& geom,VolumeGrid& grid)
{
  Assert(geom.type == AnyGeometry3D::ImplicitSurface);
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

void GetVolumeGrid(const VolumeGrid& grid,AnyCollisionGeometry3D& geom)
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
  static bool warned = false;
  if(!warned) {
    fprintf(stderr,"Warning: Geometry3D.clone will be deprecated. Use copy instead\n");
    warned = true;
  }
  return copy();
}

Geometry3D Geometry3D::copy()
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
  Klampt::ManagedGeometry* mgeom = NULL;
  if(!isStandalone()) {
    Klampt::WorldModel& world = *worlds[this->world]->world;
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
  Assert(geom->type == AnyGeometry3D::ConvexHull);
  if(geom->type != AnyGeometry3D::ConvexHull) {
    ConvexHull chull;
    return chull;
  }
  const ConvexHull3D& hull = geom->AsConvexHull();
  if(hull.type != ConvexHull3D::Polytope) {
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
  Klampt::ManagedGeometry* mgeom = NULL;
  if(!isStandalone()) {
    Klampt::WorldModel& world = *worlds[this->world]->world;
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
  Klampt::ManagedGeometry* mgeom = NULL;
  if(!isStandalone()) {
    Klampt::WorldModel& world = *worlds[this->world]->world;
    mgeom = &GetManagedGeometry(world,id);
  }
  if(geom == NULL) {
    if(mgeom) 
      geom = mgeom->CreateEmpty();
    else
      geom = make_shared<AnyCollisionGeometry3D>();
  }
  *geom = AnyCollisionGeometry3D(vector<AnyGeometry3D>());
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
  Klampt::ManagedGeometry* mgeom = NULL;
  if(!isStandalone()) {
    Klampt::WorldModel& world = *worlds[this->world]->world;
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
  Klampt::ManagedGeometry* mgeom = NULL;
  if(!isStandalone()) {
    Klampt::WorldModel& world = *worlds[this->world]->world;
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
  Klampt::ManagedGeometry* mgeom = NULL;
  if(!isStandalone()) {
    Klampt::WorldModel& world = *worlds[this->world]->world;
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
  Klampt::ManagedGeometry* mgeom = NULL;
  if(!isStandalone()) {
    Klampt::WorldModel& world = *worlds[this->world]->world;
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
  ConvexHull3D hull;
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
  Klampt::ManagedGeometry* mgeom = NULL;
  if(!isStandalone()) {
    Klampt::WorldModel& world = *worlds[this->world]->world;
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
    Klampt::WorldModel& world = *worlds[this->world]->world;
    Klampt::ManagedGeometry* mgeom = NULL;
    mgeom = &GetManagedGeometry(world,id);
    if(mgeom->Load(fn)) {
      geom = shared_ptr<AnyCollisionGeometry3D>(*mgeom);
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
  double R[9]={sx,0,0,0,sy,0,0,0,sz};
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
  const double R[9]={1,0,0,0,1,0,0,0,1};
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
    Klampt::WorldModel& world=*worlds[this->world]->world;
    Klampt::ManagedGeometry* mgeom = &GetManagedGeometry(world,id);
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
      throw PyException(ss.str());
    }
    return res;
  }
  if(!geom->Convert(destype2,*resgeom,param)) {
    stringstream ss;
    ss<<"Cannot perform the geometry conversion "<<geom->TypeName()<<" -> "<<destype;
    throw PyException(ss.str());
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


Geometry3D Geometry3D::slice(const double R[9],const double t[3],double tol)
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) throw PyException("Geometry3D is empty, cannot slice");
  if(geom->type == AnyGeometry3D::PointCloud && tol == 0)
     throw PyException("Geometry3D is a point cloud and tolerance is 0, slice will get no points");
  
  Geometry3D res;
  shared_ptr<AnyCollisionGeometry3D>& resgeom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(res.geomPtr);
  resgeom = make_shared<AnyCollisionGeometry3D>();
  RigidTransform T;
  T.R.set(R);
  T.t.set(t);
  if(!geom->Slice(T,*resgeom,tol))
    throw PyException("Slice is not supported for that type of geometry");
  return res;
}

Geometry3D Geometry3D::roi(const char* query,const double bmin[3],const double bmax[3])
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(geomPtr);
  if(!geom) throw PyException("Geometry3D is empty, cannot perform ROI");
  Geometry3D res;
  shared_ptr<AnyCollisionGeometry3D>& resgeom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(res.geomPtr);
  resgeom = make_shared<AnyCollisionGeometry3D>();
  int flag=0;
  if(query[0]=='~') {
    flag |= ExtractROIFlagInvert;
    query = &query[1];
  }
  if(0==strcmp(query,"intersect")) flag|=ExtractROIFlagIntersection;
  else if(0==strcmp(query,"within")) flag|=ExtractROIFlagWithin;
  else if(0==strcmp(query,"touching")) flag|=ExtractROIFlagTouching;
  else
    throw PyException("Invalid query, must be intersect, within, or touching");
  AABB3D bb;
  bb.bmin.set(bmin);
  bb.bmax.set(bmax);
  if(!geom->ExtractROI(bb,*resgeom,flag))
    throw PyException("ROI is not supported for that type of geometry");
  return res;
}

//KH: note: pointer gymnastics necessary to allow appearances to refer to temporary appearances as well as references to world, while also
//exposing an opaque pointer in appearance.h

//defined in Cpp/Modeling/Klampt::ManagedGeometry.cpp

namespace Klampt {
  void SetupDefaultAppearance(GLDraw::GeometryAppearance& app);
}

Appearance::Appearance()
  :world(-1),id(-1),appearancePtr(NULL)
{
  auto ptr = new shared_ptr<GLDraw::GeometryAppearance>;
  ptr->reset(new GLDraw::GeometryAppearance());
  Klampt::SetupDefaultAppearance(**ptr);
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
    Klampt::ManagedGeometry& geom = GetManagedGeometry(*worlds[this->world]->world,id);
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
    Klampt::WorldModel& world=*worlds[this->world]->world;
    Klampt::ManagedGeometry& geom = GetManagedGeometry(world,id);
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
    Klampt::WorldModel& world=*worlds[this->world]->world;
    Klampt::ManagedGeometry& geom = GetManagedGeometry(world,id);
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
    Klampt::WorldModel& world=*worlds[this->world]->world;
    Klampt::ManagedGeometry& geom = GetManagedGeometry(world,id);
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
    Klampt::WorldModel& world=*worlds[this->world]->world;
    Klampt::ManagedGeometry& geom = GetManagedGeometry(world,id);
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
    Klampt::WorldModel& world=*worlds[this->world]->world;
    Klampt::ManagedGeometry& geom = GetManagedGeometry(world,id);
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
    Klampt::WorldModel& world=*worlds[this->world]->world;
    Klampt::ManagedGeometry& geom = GetManagedGeometry(world,id);
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
void Appearance::setColors(int feature,float* colors,int m,int n)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) throw PyException("Invalid appearance");
  if(n != 3 && n != 4) throw PyException("Color array must have size N x 3 or N x 4");
  switch(feature) {
  case VERTICES:
    {
      app->vertexColors.resize(m,app->vertexColor);
      for(int i=0;i<m;i++) {
        for(int k=0;k<n;k++)
          app->vertexColors[i].rgba[k] = colors[i*n+k];
      }
    }
    break;
  case FACES:
    {
      app->faceColors.resize(m,app->faceColor);
      for(int i=0;i<m;i++) {
        for(int k=0;k<n;k++)
          app->faceColors[i].rgba[k] = colors[i*n+k];
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

Image::PixelFormat StringToImageFormat(const char* format)
{
  if(0==strcmp(format,"rgb8")) {
    return Image::R8G8B8;
  }
  else if(0==strcmp(format,"bgr8")) {
    return Image::B8G8R8;
  }
  else if(0==strcmp(format,"rgba8")) {
    return Image::R8G8B8A8;
  }
  else if(0==strcmp(format,"bgra8")) {
    return Image::B8G8R8A8;
  }
  else if(0==strcmp(format,"l8")) {
    return Image::A8;
  }
  else
    throw PyException("Invalid format string, must be rgb8, bgr8, rgba8, bgr8, or l8");
  return Image::None;
}

void Appearance::setTexture1D_b(const char* format,unsigned char* bytes,int m)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) throw PyException("Invalid appearance");
  app->tex1D.reset();
  app->tex2D.reset();
  app->textureObject.cleanup();
  if(strlen(format)==0) {
    return;
  }
  app->tex1D = make_shared<Image>();
  Image::PixelFormat fmt = StringToImageFormat(format);
  int bpp = Image::pixelFormatSize(fmt);
  app->tex1D->initialize(1,m/bpp,fmt);
  memcpy(app->tex1D->data,bytes,m);
}

void Appearance::setTexture1D_i(const char* format,unsigned int* bytes,int m)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) throw PyException("Invalid appearance");
  app->tex1D.reset();
  app->tex2D.reset();
  app->textureObject.cleanup();
  if(strlen(format)==0) {
    return;
  }
  app->tex1D = make_shared<Image>();
  Image::PixelFormat fmt = StringToImageFormat(format);
  int bpp = Image::pixelFormatSize(fmt);
  if(bpp != 4) throw PyException("Provided uint32 type to texture, but not a 32-bit format");
  app->tex1D->initialize(1,m,fmt);
  memcpy(app->tex1D->data,bytes,m*sizeof(unsigned int));
}

void Appearance::setTexture1D_channels(const char* format,unsigned char* bytes,int m,int n)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) throw PyException("Invalid appearance");
  app->tex1D.reset();
  app->tex2D.reset();
  app->textureObject.cleanup();
  if(strlen(format)==0) {
    return;
  }
  app->tex1D = make_shared<Image>();
  Image::PixelFormat fmt = StringToImageFormat(format);
  int bpp = Image::pixelFormatSize(fmt);
  if(bpp != n) {
    stringstream ss;
    ss<<"Provided "<<n<<"channels to texture, but format is a "<<bpp<<"-byte format";
    throw PyException(ss.str());
  }
  app->tex1D->initialize(1,m,fmt);
  memcpy(app->tex1D->data,bytes,m*n);
}

void Appearance::setTexture2D_b(const char* format,unsigned char* bytes,int m,int n,bool topdown)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) throw PyException("Invalid appearance");
  app->tex1D.reset();
  app->tex2D.reset();
  app->textureObject.cleanup();
  if(strlen(format)==0) {
    return;
  }
  app->tex2D = make_shared<Image>();
  Image::PixelFormat fmt = StringToImageFormat(format);
  int bpp = Image::pixelFormatSize(fmt);
  if(bpp != 1) throw PyException("Provided uint8 type to texture, but not an 8-bit format");
  app->tex2D->initialize(n,m,fmt);
  if(topdown) {
    memcpy(app->tex2D->data,bytes,m*n);
  }
  else {
    int stride = n;
    for(int i=0;i<m;i++) {
      int iflip = m-1-i;
      memcpy(&app->tex2D->data[iflip*stride],&bytes[i*stride],stride);
    }
  }
}

void Appearance::setTexture2D_i(const char* format,unsigned int* bytes,int m,int n,bool topdown)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) throw PyException("Invalid appearance");
  app->tex1D.reset();
  app->tex2D.reset();
  app->textureObject.cleanup();
  if(strlen(format)==0) {
    return;
  }
  app->tex2D = make_shared<Image>();
  Image::PixelFormat fmt = StringToImageFormat(format);
  int bpp = Image::pixelFormatSize(fmt);
  if(bpp != 4) throw PyException("Provided uint32 type to texture, but not a 32-bit format");
  app->tex2D->initialize(n,m,fmt);
  if(topdown) {
    memcpy(app->tex2D->data,bytes,m*n*sizeof(unsigned int));
  }
  else {
    int stride = n*sizeof(unsigned int);
    for(int i=0;i<m;i++) {
      int iflip = m-1-i;
      memcpy(&app->tex2D->data[iflip*stride],&bytes[i*stride],stride);
    }
  }
}

void Appearance::setTexture2D_channels(const char* format,unsigned char* bytes,int m,int n,int p,bool topdown)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) throw PyException("Invalid appearance");
  app->tex1D.reset();
  app->tex2D.reset();
  app->textureObject.cleanup();
  if(strlen(format)==0) {
    return;
  }
  app->tex2D = make_shared<Image>();
  Image::PixelFormat fmt = StringToImageFormat(format);
  int bpp = Image::pixelFormatSize(fmt);
  if(bpp != p) {
    stringstream ss;
    ss<<"Provided "<<p<<"channels to texture, but format is a "<<bpp<<"-byte format";
    throw PyException(ss.str());
  }
  app->tex2D->initialize(n,m,fmt);
  if(topdown) {
    memcpy(app->tex2D->data,bytes,m*n*p);
  }
  else {
    int stride = n*p;
    for(int i=0;i<m;i++) {
      int iflip = m-1-i;
      memcpy(&app->tex2D->data[iflip*stride],&bytes[i*stride],stride);
    }
  }
}

void Appearance::setTexcoords1D(double* np_array,int m)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) throw PyException("Invalid appearance");
  app->texcoords.resize(m);
  for(int i=0;i<m;i++)
    app->texcoords[i].x = np_array[i];
}

void Appearance::setTexcoords2D(double* np_array2,int m,int n)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) throw PyException("Invalid appearance");
  if(n != 2) throw PyException("Must provide exactly 2 columns");
  app->texcoords.resize(m);
  for(int i=0;i<m;i++) {
    app->texcoords[i].x = np_array2[i*2];
    app->texcoords[i].y = np_array2[i*2+1];
  }
}

void Appearance::setTexgen(double* np_array2,int m,int n,bool worldcoordinates)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) throw PyException("Invalid appearance");
  if(m==0) {
    app->texgen.resize(0);
    return;
  }
  if(n != 4) throw PyException("Texgen array must have exactly 4 columns");
  if(worldcoordinates) {
    RigidTransform Tident; Tident.setIdentity();
    app->texgenEyeTransform = make_shared<RigidTransform>(Tident);
  }
  else {
    app->texgenEyeTransform.reset();
  }
  app->texgen.resize(m);
  for(int i=0;i<m;i++)
    app->texgen[i].set(&np_array2[i*4]);
}

void Appearance::setTexWrap(bool wrap)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) throw PyException("Invalid appearance");
  app->texWrap = wrap;
}

void Appearance::setPointSize(float size)
{
  shared_ptr<GLDraw::GeometryAppearance>& app = *reinterpret_cast<shared_ptr<GLDraw::GeometryAppearance>*>(appearancePtr);
  if(!app) return;
  if(!isStandalone()) {
    Klampt::WorldModel& world=*worlds[this->world]->world;
    Klampt::ManagedGeometry& geom = GetManagedGeometry(world,id);
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
    Klampt::WorldModel& world=*worlds[this->world]->world;
    Klampt::ManagedGeometry& geom = GetManagedGeometry(world,id);
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
    Klampt::WorldModel& world=*worlds[this->world]->world;
    Klampt::ManagedGeometry& geom = GetManagedGeometry(world,id);
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
    Klampt::SetupDefaultAppearance(*app);
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
    Klampt::SetupDefaultAppearance(*app);
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

void TriangleMesh::getVertices(double** np_view2, int* m, int* n)
{
  if(vertices.empty()) {
    *np_view2 = 0;
    *m = 0;
    *n = 0;
    return;
  }
  *np_view2 = &vertices[0];
  *m = vertices.size()/3;
  *n = 3;
}

void TriangleMesh::setVertices(double* np_array2, int m, int n)
{
  if(n != 3) throw PyException("Vertex array must be nx3");
  vertices.resize(m*n);
  copy(np_array2,np_array2+m*n,&vertices[0]);
}

void TriangleMesh::getIndices(int** np_view2, int* m, int* n)
{
  if(indices.empty()) {
    *np_view2 = 0;
    *m = 0;
    *n = 0;
    return;
  }
  *np_view2 = &indices[0];
  *m = indices.size()/3;
  *n = 3;
}

void TriangleMesh::setIndices(int* np_array2, int m, int n)
{
  if(n != 3) throw PyException("Index array must be nx3");
  indices.resize(m*n);
  copy(np_array2,np_array2+m*n,&indices[0]);
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

ConvexHull::ConvexHull()
{}


int ConvexHull::numPoints() const
{
  return points.size()/3;
}

void ConvexHull::getPoints(double** np_view2, int* m, int* n)
{
  if(points.empty()) {
    *np_view2 = 0;
    *m = 0;
    *n = 0;
    return;
  }
  *np_view2 = &points[0];
  *m = points.size()/3;
  *n = 3;
}

void ConvexHull::setPoints(double* np_array2, int m, int n)
{
  if(n != 3) throw PyException("Vertex array must be nx3");
  points.resize(m*n);
  copy(np_array2,np_array2+m*n,&points[0]);
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
void PointCloud::setPoints(double* parray,int m,int n)
{
  if(n!=3) throw PyException("Array must be size nx3");
  int num = m;
  bool resized = ((int)vertices.size() != num*3);
  vertices.resize(num*3);
  copy(parray,parray+num*3,&vertices[0]);
  if(resized) {
    properties.resize(num*propertyNames.size());
    fill(properties.begin(),properties.end(),0.0);
  }
  else {
    properties.resize(num*propertyNames.size(),0.0);
  }
}
void PointCloud::getPoints(double** pview,int* m,int *n)
{
  if(vertices.size()==0) {
    *m=0;
    *n=0;
    *pview=NULL;
    return;
  }
  *m = vertices.size()/3;
  *n = 3;
  *pview = &vertices[0];
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
  addProperty(pname,&values[0],n);
}

void PointCloud::addProperty(const std::string& pname,double* values,int numvals)
{
  int n = numPoints();
  if(numvals != n) {
    stringstream ss;
    ss<<"Invalid size "<<numvals<<" of properties list, must have size #points = "<<n;
    throw PyException(ss.str());
  }
  assert(numvals == n);
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

void PointCloud::setProperties(double* np_array2, int m, int n)
{
  int npt = numPoints();
  size_t nprop=propertyNames.size();

  if(m != npt) {
    throw PyException("Invalid size of properties array, must have #points rows");
  }
  if((int)nprop != n) {
    propertyNames.resize(n);
    properties.resize(m*n);
    for(int i=(int)nprop;i<n;i++) {
      stringstream ss;
      ss<<"Property "<<i;
      propertyNames[i] = ss.str();
    }
  }
  //if(n != nprop) {
  //  throw PyException("Invalid size of properties array, must have #properties columns");
  //}

  copy(np_array2,np_array2+m*n,properties.begin());
}

void PointCloud::setPointsAndProperties(double* np_array2, int m,int n)
{
  if(m==0) {
    vertices.resize(0);
    properties.resize(0);
    propertyNames.resize(0);
    return;
  }
  if(n < 3)
    throw PyException("Invalid size of array, must have >= 3 dimensions");

  size_t nprop=propertyNames.size();
  if((int)nprop != n-3) {
    propertyNames.resize(n-3);
    for(int i=(int)nprop;i<n-3;i++) {
      stringstream ss;
      ss<<"Property "<<i;
      propertyNames[i] = ss.str();
    }
  }
  vertices.resize(m*3);
  properties.resize(m*(n-3));
  if(n==3)
    copy(np_array2,np_array2+m*3,&vertices[0]);
  else {
    int kv=0;
    int kp=0;
    int k=0;
    for(int i=0;i<m;i++,k+=n,kv+=3,kp+=(n-3)) {
      for(int j=0;j<3;j++) vertices[kv+j]=np_array2[k+j];
      for(int j=0;j<n-3;j++) properties[kp+j]=np_array2[k+3+j];
    }
  }
  
}
 
void PointCloud::getAllProperties(double** np_view2, int* m, int* n)
{
  *m = numPoints();
  *n = (int)propertyNames.size();
  *np_view2 = (double*)malloc(properties.size()*sizeof(double));
  copy(properties.begin(),properties.end(),*np_view2);
}

void PointCloud::setProperties(int pindex,double* vproperties,int numvals)
{
  if(pindex < 0 || pindex >= (int)propertyNames.size())
    throw PyException("Invalid property index"); 
  int n = numPoints();
  if(numvals != n) {
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

void PointCloud::getProperties(int pindex,double** out, int* m) const
{
  if(pindex < 0 || pindex >= (int)propertyNames.size())
    throw PyException("Invalid property index");  
  int n=numPoints();
  *m = n;
  *out = (double*)malloc(n*sizeof(double));
  size_t k=pindex;
  for(int i=0;i<n;i++,k+=propertyNames.size())
    (*out)[i] = properties[k];
}

void PointCloud::getProperties(const std::string& pname,double** out,int* m) const
{
  int pindex = -1;
  for(size_t i=0;i<propertyNames.size();i++)
    if(propertyNames[i] == pname) {
      pindex = (int)i;
      break;
    }
  if(pindex < 0)
    throw PyException("Invalid property name");  
  return getProperties(pindex,out,m); 
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

void PointCloud::setDepthImage_d(const double intrinsics[4],double* np_depth2,int m,int n,double depth_scale)
{
  double fx=intrinsics[0],fy=intrinsics[1],cx=intrinsics[2],cy=intrinsics[3];
  if(fx <= 0 || fy <= 0) throw PyException("Invalid intrinsics values");
  settings.clear();
  { stringstream ss; ss<<n; settings["width"] = ss.str(); }
  { stringstream ss; ss<<m; settings["height"] = ss.str(); }
  { settings["viewpoint"] = "0 0 0 1 0 0 0"; }
  double invfx = 1.0/fx;
  double invfy = 1.0/fy;
  propertyNames.resize(0);
  properties.resize(0);

  vertices.resize(m*n*3);
  int k=0;
  for(int i=0;i<m;i++) {
    double y=(i-cy)*invfy;
    for(int j=0;j<n;j++,k++) {
      double x=(j-cx)*invfx;
      double z=np_depth2[k]*depth_scale;
      vertices[k*3] = x*z;
      vertices[k*3+1] = y*z;
      vertices[k*3+2] = z;
    }
  }
}

void PointCloud::setDepthImage_f(const double intrinsics[4],float* np_depth2,int m,int n,double depth_scale)
{
  double fx=intrinsics[0],fy=intrinsics[1],cx=intrinsics[2],cy=intrinsics[3];
  if(fx <= 0 || fy <= 0) throw PyException("Invalid intrinsics values");
  settings.clear();
  { stringstream ss; ss<<n; settings["width"] = ss.str(); }
  { stringstream ss; ss<<m; settings["height"] = ss.str(); }
  { settings["viewpoint"] = "0 0 0 1 0 0 0"; }
  double invfx = 1.0/fx;
  double invfy = 1.0/fy;
  propertyNames.resize(0);
  properties.resize(0);

  vertices.resize(m*n*3);
  int k=0;
  for(int i=0;i<m;i++) {
    double y=(i-cy)*invfy;
    for(int j=0;j<n;j++,k++) {
      double x=(j-cx)*invfx;
      double z=np_depth2[k]*depth_scale;
      vertices[k*3] = x*z;
      vertices[k*3+1] = y*z;
      vertices[k*3+2] = z;
    }
  }
}

void PointCloud::setDepthImage_s(const double intrinsics[4],unsigned short* np_depth2,int m,int n,double depth_scale)
{
  double fx=intrinsics[0],fy=intrinsics[1],cx=intrinsics[2],cy=intrinsics[3];
  if(fx <= 0 || fy <= 0) throw PyException("Invalid intrinsics values");
  settings.clear();
  { stringstream ss; ss<<n; settings["width"] = ss.str(); }
  { stringstream ss; ss<<m; settings["height"] = ss.str(); }
  { settings["viewpoint"] = "0 0 0 1 0 0 0"; }
  double invfx = 1.0/fx;
  double invfy = 1.0/fy;
  propertyNames.resize(0);
  properties.resize(0);

  vertices.resize(m*n*3);
  int k=0;
  for(int i=0;i<m;i++) {
    double y=(i-cy)*invfy;
    for(int j=0;j<n;j++,k++) {
      double x=(j-cx)*invfx;
      double z=np_depth2[k]*depth_scale;
      vertices[k*3] = x*z;
      vertices[k*3+1] = y*z;
      vertices[k*3+2] = z;
    }
  }
}

void PointCloud::setRGBDImages_i_d(const double intrinsics[4],unsigned int* np_array2,int m,int n,double* np_depth2,int m2,int n2,double depth_scale)
{
  if(m != m2 || n != n2) throw PyException("Non-matching image sizes");
  double fx=intrinsics[0],fy=intrinsics[1],cx=intrinsics[2],cy=intrinsics[3];
  if(fx <= 0 || fy <= 0) throw PyException("Invalid intrinsics values");
  settings.clear();
  { stringstream ss; ss<<n; settings["width"] = ss.str(); }
  { stringstream ss; ss<<m; settings["height"] = ss.str(); }
  { settings["viewpoint"] = "0 0 0 1 0 0 0"; }
  double invfx = 1.0/fx;
  double invfy = 1.0/fy;
  propertyNames.resize(1);
  propertyNames[0] = "rgb";
  properties.resize(m*n);
  for(int i=0;i<m*n;i++)
    properties[i] = Real(np_array2[i]);  //encode uint as double

  vertices.resize(m*n*3);
  int k=0;
  for(int i=0;i<m;i++) {
    double y=(i-cy)*invfy;
    for(int j=0;j<n;j++,k++) {
      double x=(j-cx)*invfx;
      double z=np_depth2[k]*depth_scale;
      vertices[k*3] = x*z;
      vertices[k*3+1] = y*z;
      vertices[k*3+2] = z;
    }
  }
}

void PointCloud::setRGBDImages_i_f(const double intrinsics[4],unsigned int* np_array2,int m,int n,float* np_depth2,int m2,int n2,double depth_scale)
{
  if(m != m2 || n != n2) throw PyException("Non-matching image sizes");
  double fx=intrinsics[0],fy=intrinsics[1],cx=intrinsics[2],cy=intrinsics[3];
  if(fx <= 0 || fy <= 0) throw PyException("Invalid intrinsics values");
  settings.clear();
  { stringstream ss; ss<<n; settings["width"] = ss.str(); }
  { stringstream ss; ss<<m; settings["height"] = ss.str(); }
  { settings["viewpoint"] = "0 0 0 1 0 0 0"; }
  double invfx = 1.0/fx;
  double invfy = 1.0/fy;
  propertyNames.resize(1);
  propertyNames[0] = "rgb";
  properties.resize(m*n);
  for(int i=0;i<m*n;i++)
    properties[i] = Real(np_array2[i]);  //encode uint as double

  vertices.resize(m*n*3);
  int k=0;
  for(int i=0;i<m;i++) {
    double y=(i-cy)*invfy;
    for(int j=0;j<n;j++,k++) {
      double x=(j-cx)*invfx;
      double z=np_depth2[k]*depth_scale;
      vertices[k*3] = x*z;
      vertices[k*3+1] = y*z;
      vertices[k*3+2] = z;
    }
  }
}

void PointCloud::setRGBDImages_i_s(const double intrinsics[4],unsigned int* np_array2,int m,int n,unsigned short* np_depth2,int m2,int n2,double depth_scale)
{
  if(m != m2 || n != n2) throw PyException("Non-matching image sizes");
  double fx=intrinsics[0],fy=intrinsics[1],cx=intrinsics[2],cy=intrinsics[3];
  if(fx <= 0 || fy <= 0) throw PyException("Invalid intrinsics values");
  settings.clear();
  { stringstream ss; ss<<n; settings["width"] = ss.str(); }
  { stringstream ss; ss<<m; settings["height"] = ss.str(); }
  { settings["viewpoint"] = "0 0 0 1 0 0 0"; }
  double invfx = 1.0/fx;
  double invfy = 1.0/fy;
  propertyNames.resize(1);
  propertyNames[0] = "rgb";
  properties.resize(m*n);
  for(int i=0;i<m*n;i++)
    properties[i] = Real(np_array2[i]);  //encode uint as double

  vertices.resize(m*n*3);
  int k=0;
  for(int i=0;i<m;i++) {
    double y=(i-cy)*invfy;
    for(int j=0;j<n;j++,k++) {
      double x=(j-cx)*invfx;
      double z=np_depth2[k]*depth_scale;
      vertices[k*3] = x*z;
      vertices[k*3+1] = y*z;
      vertices[k*3+2] = z;
    }
  }
}

void PointCloud::setRGBDImages_b_d(const double intrinsics[4],unsigned char* np_array3,int m,int n,int p,double* np_depth2,int m2,int n2,double depth_scale)
{
  if(p != 3) throw PyException("Need 3 color channels");
  if(m != m2 || n != n2) throw PyException("Non-matching image sizes");
  double fx=intrinsics[0],fy=intrinsics[1],cx=intrinsics[2],cy=intrinsics[3];
  if(fx <= 0 || fy <= 0) throw PyException("Invalid intrinsics values");
  settings.clear();
  { stringstream ss; ss<<n; settings["width"] = ss.str(); }
  { stringstream ss; ss<<m; settings["height"] = ss.str(); }
  { settings["viewpoint"] = "0 0 0 1 0 0 0"; }
  double invfx = 1.0/fx;
  double invfy = 1.0/fy;
  propertyNames.resize(1);
  propertyNames[0] = "rgb";
  properties.resize(m*n);
  int j=0;
  for(int i=0;i<m*n;i++,j+=3) {
    unsigned int rgb = (((unsigned int)np_array3[j])<<16) | (((unsigned int)np_array3[j+1])<<8) | (np_array3[j+2]);
    properties[i] = Real(rgb);  //encode uint as double
  }

  vertices.resize(m*n*3);
  int k=0;
  for(int i=0;i<m;i++) {
    double y=(i-cy)*invfy;
    for(int j=0;j<n;j++,k++) {
      double x=(j-cx)*invfx;
      double z=np_depth2[k]*depth_scale;
      vertices[k*3] = x*z;
      vertices[k*3+1] = y*z;
      vertices[k*3+2] = z;
    }
  }
}

void PointCloud::setRGBDImages_b_f(const double intrinsics[4],unsigned char* np_array3,int m,int n,int p,float* np_depth2,int m2,int n2,double depth_scale)
{
  if(p != 3) throw PyException("Need 3 color channels");
  if(m != m2 || n != n2) throw PyException("Non-matching image sizes");
  double fx=intrinsics[0],fy=intrinsics[1],cx=intrinsics[2],cy=intrinsics[3];
  if(fx <= 0 || fy <= 0) throw PyException("Invalid intrinsics values");
  settings.clear();
  { stringstream ss; ss<<n; settings["width"] = ss.str(); }
  { stringstream ss; ss<<m; settings["height"] = ss.str(); }
  { settings["viewpoint"] = "0 0 0 1 0 0 0"; }
  double invfx = 1.0/fx;
  double invfy = 1.0/fy;
  propertyNames.resize(1);
  propertyNames[0] = "rgb";
  properties.resize(m*n);
  int j=0;
  for(int i=0;i<m*n;i++,j+=3) {
    unsigned int rgb = (((unsigned int)np_array3[j])<<16) | (((unsigned int)np_array3[j+1])<<8) | (np_array3[j+2]);
    properties[i] = Real(rgb);  //encode uint as double
  }

  vertices.resize(m*n*3);
  int k=0;
  for(int i=0;i<m;i++) {
    double y=(i-cy)*invfy;
    for(int j=0;j<n;j++,k++) {
      double x=(j-cx)*invfx;
      double z=np_depth2[k]*depth_scale;
      vertices[k*3] = x*z;
      vertices[k*3+1] = y*z;
      vertices[k*3+2] = z;
    }
  }
}

void PointCloud::setRGBDImages_b_s(const double intrinsics[4],unsigned char* np_array3,int m,int n,int p,unsigned short* np_depth2,int m2,int n2,double depth_scale)
{
  if(p != 3) throw PyException("Need 3 color channels");
  if(m != m2 || n != n2) throw PyException("Non-matching image sizes");
  double fx=intrinsics[0],fy=intrinsics[1],cx=intrinsics[2],cy=intrinsics[3];
  if(fx <= 0 || fy <= 0) throw PyException("Invalid intrinsics values");
  settings.clear();
  { stringstream ss; ss<<n; settings["width"] = ss.str(); }
  { stringstream ss; ss<<m; settings["height"] = ss.str(); }
  { settings["viewpoint"] = "0 0 0 1 0 0 0"; }
  double invfx = 1.0/fx;
  double invfy = 1.0/fy;
  propertyNames.resize(1);
  propertyNames[0] = "rgb";
  properties.resize(m*n);
  int j=0;
  for(int i=0;i<m*n;i++,j+=3) {
    unsigned int rgb = (((unsigned int)np_array3[j])<<16) | (((unsigned int)np_array3[j+1])<<8) | (np_array3[j+2]);
    properties[i] = Real(rgb);  //encode uint as double
  }

  vertices.resize(m*n*3);
  int k=0;
  for(int i=0;i<m;i++) {
    double y=(i-cy)*invfy;
    for(int j=0;j<n;j++,k++) {
      double x=(j-cx)*invfx;
      double z=np_depth2[k]*depth_scale;
      vertices[k*3] = x*z;
      vertices[k*3+1] = y*z;
      vertices[k*3+2] = z;
    }
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

void VolumeGrid::getValues(double** out, int* m, int* n, int* p)
{
  if(dims.empty()) throw PyException("VolumeGrid was not initialized yet");
  *m = dims[0];
  *n = dims[1];
  *p = dims[2];
  *out = &values[0];
}

void VolumeGrid::setValues(double* in, int m, int n, int p)
{
  resize(m,n,p);
  copy(in,in+m*n*p,&values[0]);
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
    throw PyException(ss.str());
  }
}

WorldModel::WorldModel(const WorldModel& w)
{
  index = w.index;
  refWorld(index);
}

WorldModel::WorldModel(void* ptr)
{
  index = createWorld((Klampt::WorldModel*)ptr);
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
  Klampt::WorldModel& myworld = *worlds[index]->world;
  Klampt::WorldModel& otherworld = *worlds[res.index]->world;
  otherworld = myworld;
  //world occupants -- copy everything but geometry
  for(size_t i=0;i<otherworld.robots.size();i++) {
    otherworld.robots[i] = make_shared<Klampt::RobotModel>();
    *otherworld.robots[i] = *myworld.robots[i];
    otherworld.robotViews[i].robot = otherworld.robots[i].get();
  }
  for(size_t i=0;i<otherworld.terrains.size();i++) {
    otherworld.terrains[i] = make_shared<Klampt::TerrainModel>();
    *otherworld.terrains[i] = *myworld.terrains[i];
  }
  for(size_t i=0;i<otherworld.rigidObjects.size();i++) {
    otherworld.rigidObjects[i] = make_shared<Klampt::RigidObjectModel>();
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
  Klampt::WorldModel& world = *worlds[index]->world;
  return world.SaveXML(fn,elementPath);
}

bool WorldModel::readFile(const char* fn)
{
  Klampt::WorldModel& world = *worlds[index]->world;

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
  Klampt::WorldModel& world = *worlds[index]->world;
  return world.robots.size();
}

int WorldModel::numRobotLinks(int robot)
{
  Klampt::WorldModel& world = *worlds[index]->world;
  return world.robots[robot]->links.size();
}

int WorldModel::numRigidObjects()
{
  Klampt::WorldModel& world = *worlds[index]->world;
  return world.rigidObjects.size();
}

int WorldModel::numTerrains()
{
  Klampt::WorldModel& world = *worlds[index]->world;
  return world.terrains.size();
}

int WorldModel::numIDs()
{
  Klampt::WorldModel& world = *worlds[index]->world;
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
  Klampt::WorldModel& world = *worlds[index]->world;
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
  Klampt::WorldModel& world = *worlds[index]->world;
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
  Klampt::WorldModel& world = *worlds[index]->world;
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
  Klampt::WorldModel& world = *worlds[index]->world;
  RobotModel robot;
  robot.world = index;
  robot.index = (int)world.robots.size();
  world.AddRobot(name,new Klampt::RobotModel());
  robot.robot = world.robots.back().get();
  return robot;
}

RigidObjectModel WorldModel::makeRigidObject(const char* name)
{
  Klampt::WorldModel& world = *worlds[index]->world;
  RigidObjectModel object;
  object.world = index;
  object.index = (int)world.rigidObjects.size();
  world.AddRigidObject(name,new Klampt::RigidObjectModel());
  object.object = world.rigidObjects.back().get();
  object.object->geometry.CreateEmpty();
  return object;
}

TerrainModel WorldModel::makeTerrain(const char* name)
{
  Klampt::WorldModel& world = *worlds[index]->world;
  TerrainModel terrain;
  terrain.world = index;
  terrain.index = world.terrains.size();
  world.AddTerrain(name,new Klampt::TerrainModel());
  terrain.terrain = world.terrains.back().get();
  terrain.terrain->geometry.CreateEmpty();
  return terrain;
}

RobotModel WorldModel::loadRobot(const char* fn)
{
  Klampt::WorldModel& world = *worlds[index]->world;
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
  Klampt::WorldModel& world = *worlds[index]->world;
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
  Klampt::WorldModel& world = *worlds[index]->world;
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
  Klampt::WorldModel& world = *worlds[index]->world;
  int id = world.LoadElement(fn);
  return id;
}

RobotModel WorldModel::add(const char* name,const RobotModel& robot)
{
  if(robot.robot == NULL)
    throw PyException("add(RobotModel): robot refers to NULL object");
  Klampt::WorldModel& world = *worlds[index]->world;
  world.AddRobot(name,new Klampt::RobotModel());
  *world.robots.back() = *robot.robot;
  return this->robot((int)world.robots.size()-1);
}

RigidObjectModel WorldModel::add(const char* name,const RigidObjectModel& obj)
{
  if(obj.object == NULL)
    throw PyException("add(RigidObjectModel): obj refers to NULL object");
  Klampt::WorldModel& world = *worlds[index]->world;
  world.AddRigidObject(name,new Klampt::RigidObjectModel());
  *world.rigidObjects.back() = *obj.object;
  return this->rigidObject((int)world.rigidObjects.size()-1);
}
 
TerrainModel WorldModel::add(const char* name,const TerrainModel& terrain)
{
  if(terrain.terrain == NULL)
    throw PyException("add(TerrianModel): terrain refers to NULL object");
  Klampt::WorldModel& world = *worlds[index]->world;
  world.AddTerrain(name,new Klampt::TerrainModel());
  *world.terrains.back() = *terrain.terrain;
  return this->terrain((int)world.terrains.size()-1);
}

void WorldModel::remove(const RobotModel& obj)
{
  if(obj.world != index) 
    throw PyException("Robot does not belong to this world");
  Klampt::WorldModel& world = *worlds[index]->world;
  if(obj.index < 0 || obj.index >= (int)world.robots.size())
    throw PyException("Invalid robot index");
  world.robots.erase(world.robots.begin()+obj.index);
}

void WorldModel::remove(const RigidObjectModel& obj)
{
  if(obj.world != index) 
    throw PyException("Rigid object does not belong to this world");
  Klampt::WorldModel& world = *worlds[index]->world;
  if(obj.index < 0 || obj.index >= (int)world.rigidObjects.size())
    throw PyException("Invalid rigid object index");
  world.rigidObjects.erase(world.rigidObjects.begin()+obj.index);
}

void WorldModel::remove(const TerrainModel& obj)
{
  if(obj.world != index) 
    throw PyException("Terrain does not belong to this world");
  Klampt::WorldModel& world = *worlds[index]->world;
  if(obj.index < 0 || obj.index >= (int)world.terrains.size())
    throw PyException("Invalid terrain index");
  world.terrains.erase(world.terrains.begin()+obj.index);
}


void WorldModel::drawGL()
{
  Klampt::WorldModel& world = *worlds[index]->world;
  world.DrawGL();
}

void WorldModel::enableGeometryLoading(bool enabled)
{
  Klampt::RobotModel::disableGeometryLoading = !enabled;
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
  Klampt::WorldModel& world = *worlds[index]->world;
  return world.GetName(id);
}

Geometry3D WorldModel::geometry(int id)
{
  Klampt::WorldModel& world = *worlds[index]->world;
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
  Klampt::WorldModel& world = *worlds[index]->world;
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
  Klampt::WorldModel& world = *worlds[this->world]->world;
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
  Vector3 com = Klampt::CenterOfMass(**gp,surfaceFraction);
  Matrix3 H = Klampt::Inertia(**gp,com,mass,surfaceFraction);
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

void RobotModelLink::getJacobian(const double p[3],double** np_out2,int* m,int* n)
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");

  Matrix Jmat;
  MakeNumpyArray(np_out2,m,n,6,int(robotPtr->links.size()),Jmat);
  robotPtr->GetFullJacobian(Vector3(p),index,Jmat);
}

void RobotModelLink::getPositionJacobian(const double p[3],double** np_out2,int* m,int* n)
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  Matrix Jmat;
  MakeNumpyArray(np_out2,m,n,3,int(robotPtr->links.size()),Jmat);
  robotPtr->GetPositionJacobian(Vector3(p),index,Jmat);
}

void RobotModelLink::getOrientationJacobian(double** np_out2,int* m,int* n)
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  Matrix Jmat;
  MakeNumpyArray(np_out2,m,n,3,int(robotPtr->links.size()),Jmat);
  Jmat.setZero();
  int j=index;
  while(j!=-1) {
    Vector3 w;
    robotPtr->links[j].GetOrientationJacobian(w);
    Jmat(0,j)=w.x; Jmat(1,j)=w.y; Jmat(2,j)=w.z;
    j=robotPtr->parents[j];
  }
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

void RobotModelLink::getPositionHessian(const double plocal[3],double** out,int* m, int* n, int* p)
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");

  *m = 3;
  *n = (int)robotPtr->links.size();
  *p = *n;
  *out = (double*)malloc(3*(*n)*(*p)*sizeof(double));
  Matrix Hx,Hy,Hz;
  Hx.setRef(*out,(*n)*(*p),0,*p,1,*n,*p);
  Hy.setRef(*out + (*n)*(*p),(*n)*(*p),0,*p,1,*n,*p);
  Hz.setRef(*out + 2*(*n)*(*p),(*n)*(*p),0,*p,1,*n,*p);
  Matrix* H[3] = {&Hx,&Hy,&Hz};
  robotPtr->GetPositionHessian(Vector3(plocal),index,H);
}

void RobotModelLink::getOrientationHessian(double** out,int* m, int* n, int* p)
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  Matrix Hx,Hy,Hz;
  Matrix* H[3] = {&Hx,&Hy,&Hz};
  Matrix Hwx,Hwy,Hwz;
  *m = 3;
  *n = (int)robotPtr->links.size();
  *p = *n;
  *out = (double*)malloc(3*(*n)*(*p)*sizeof(double));
  Hwx.setRef(*out,(*n)*(*p),0,*p,1,*n,*p);
  Hwy.setRef(*out + (*n)*(*p),(*n)*(*p),0,*p,1,*n,*p);
  Hwz.setRef(*out + 2*(*n)*(*p),(*n)*(*p),0,*p,1,*n,*p);
  Matrix* Hw[3] = {&Hwx,&Hwy,&Hwz};
  robotPtr->GetJacobianDeriv(Vector3(0.0),index,Hw,H);
}

void RobotModelLink::drawLocalGL(bool keepAppearance)
{
  if(index < 0)
    throw PyException("RobotModelLink is invalid");
  Klampt::WorldModel& world = *worlds[this->world]->world;
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

void RobotModelDriver::setName(const char* name)
{
  if(index < 0) {
    throw PyException("Cannot set the name of an empty driver");
  }
  robotPtr->driverNames[index] = name;
}

const char* RobotModelDriver::getType()
{
  if(index < 0) return "";
  switch(robotPtr->drivers[index].type) {
  case Klampt::RobotModelDriver::Normal: return "normal";
  case Klampt::RobotModelDriver::Affine: return "affine";
  case Klampt::RobotModelDriver::Translation: return "translation";
  case Klampt::RobotModelDriver::Rotation: return "rotation";
  case Klampt::RobotModelDriver::Custom: return "custom";
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
  if(index < 0 || index >= (int)robotPtr->drivers.size()) {
    throw PyException("Invalid driver index");
  }
  robotPtr->SetDriverValue(index,val);
}

double RobotModelDriver::getValue()
{
  if(index < 0 || index >= (int)robotPtr->drivers.size()) {
    throw PyException("Invalid driver index");
  }
  return robotPtr->GetDriverValue(index);
}
void RobotModelDriver::setVelocity(double val)
{
  if(index < 0 || index >= (int)robotPtr->drivers.size()) {
    throw PyException("Invalid driver index");
  }
  robotPtr->SetDriverVelocity(index,val);
}

double RobotModelDriver::getVelocity()
{
  if(index < 0 || index >= (int)robotPtr->drivers.size()) {
    throw PyException("Invalid driver index");
  }
  return robotPtr->GetDriverVelocity(index);
}

void RobotModelDriver::getLimits(double out[2])
{
  if(index < 0 || index >= (int)robotPtr->drivers.size()) {
    throw PyException("Invalid driver index");
  }
  out[0] = robotPtr->drivers[index].qmin;
  out[1] = robotPtr->drivers[index].qmax;
}

void RobotModelDriver::getVelocityLimits(double out[2])
{
  if(index < 0 || index >= (int)robotPtr->drivers.size()) {
    throw PyException("Invalid driver index");
  }
  out[0] = robotPtr->drivers[index].vmin;
  out[1] = robotPtr->drivers[index].vmax;
}

void RobotModelDriver::getAccelerationLimits(double out[2])
{
  if(index < 0 || index >= (int)robotPtr->drivers.size()) {
    throw PyException("Invalid driver index");
  }
  out[0] = robotPtr->drivers[index].amin;
  out[1] = robotPtr->drivers[index].amax;
}

void RobotModelDriver::getTorqueLimits(double out[2])
{
  if(index < 0 || index >= (int)robotPtr->drivers.size()) {
    throw PyException("Invalid driver index");
  }
  out[0] = robotPtr->drivers[index].tmin;
  out[1] = robotPtr->drivers[index].tmax;
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
  Klampt::WorldModel& world = *worlds[this->world]->world;
  return world.robots[index]->name.c_str();
}

void RobotModel::setName(const char* name)
{
  if(!robot) throw PyException("RobotModel is empty");
  Klampt::WorldModel& world = *worlds[this->world]->world;
  world.robots[index]->name = name;
}


int RobotModel::getID() const
{
  if(!robot) return -1;
  Klampt::WorldModel& world = *worlds[this->world]->world;
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
      case Klampt::RobotModelJoint::Weld: return "weld";
      case Klampt::RobotModelJoint::Normal: return "normal";
      case Klampt::RobotModelJoint::Spin: return "spin";
      case Klampt::RobotModelJoint::Floating: return "floating";
      case Klampt::RobotModelJoint::FloatingPlanar: return "floatingplanar";
      case Klampt::RobotModelJoint::BallAndSocket: return "ballandsocket";
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
  robot->UpdateDownstreamFrames(i);
}

void RobotModel::setDOFPosition(const char* name,double qi)
{
  if(!robot) throw PyException("RobotModel is empty");
  string sname(name);
  for(size_t i=0;i<robot->linkNames.size();i++)
    if(sname == robot->linkNames[i]) {
      robot->q(i) = qi;
      robot->UpdateDownstreamFrames(i);
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
  Klampt::RobotCSpace space(*robot);
  space.unboundedStdDeviation = unboundedStdDeviation;
  space.Sample(robot->q);
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
  Klampt::WorldModel& world = *worlds[this->world]->world;
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

void RobotModel::getComJacobian(double** out,int* m,int* n)
{
  if(!robot) throw PyException("RobotModel is empty");
  Matrix J;
  MakeNumpyArray(out,m,n,3,(int)robot->links.size(),J);
  robot->GetCOMJacobian(J);
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

void RobotModel::getTotalInertia(double** np_out2,int* m,int* n)
{
  Matrix Htemp;
  MakeNumpyArray(np_out2,m,n,3,3,Htemp);
  Matrix3 H = robot->GetTotalInertia();
  for(int i=0;i<3;i++) {
    for(int j=0;j<3;j++) 
      Htemp(i,j) = H(i,j);
  }
}

void RobotModel::getMassMatrix(double** B,int* m,int* n)
{
  if(!robot) throw PyException("RobotModel is empty");
  Matrix Bmat;
  MakeNumpyArray(B,m,n,(int)robot->links.size(),(int)robot->links.size(),Bmat);
  /*
  if(dirty_dynamics) {
    robot->UpdateDynamics();
    dirty_dynamics = false;
  }
  robot->GetKineticEnergyMatrix(Bmat);
  */
  NewtonEulerSolver ne(*robot);
  ne.CalcKineticEnergyMatrix(Bmat);
}

void RobotModel::getMassMatrixInv(double** Binv,int* m,int* n)
{
  if(!robot) throw PyException("RobotModel is empty");
  Matrix Bmatinv;
  MakeNumpyArray(Binv,m,n,(int)robot->links.size(),(int)robot->links.size(),Bmatinv);
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
}

void RobotModel::getMassMatrixDeriv(int i,double** dB,int* m,int* n)
{
  if(!robot) throw PyException("RobotModel is empty");
  Matrix Bmat;
  MakeNumpyArray(dB,m,n,(int)robot->links.size(),(int)robot->links.size(),Bmat);
  if(dirty_dynamics) {
    robot->UpdateDynamics();
    dirty_dynamics = false;
  }
  robot->GetKineticEnergyMatrixDeriv(i,Bmat);
}

void RobotModel::getMassMatrixTimeDeriv(double** dB,int* m,int* n)
{
  if(!robot) throw PyException("RobotModel is empty");
  Matrix Bmat;
  MakeNumpyArray(dB,m,n,(int)robot->links.size(),(int)robot->links.size(),Bmat);
  if(dirty_dynamics) {
    robot->UpdateDynamics();
    dirty_dynamics = false;
  }
  robot->GetKineticEnergyMatrixTimeDeriv(Bmat);
}

void RobotModel::getCoriolisForceMatrix(double** C,int* m,int* n)
{
  if(!robot) throw PyException("RobotModel is empty");
  Matrix Cmat;
  MakeNumpyArray(C,m,n,(int)robot->links.size(),(int)robot->links.size(),Cmat);
  robot->UpdateDynamics();
  robot->GetCoriolisForceMatrix(Cmat);
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
    worldData->robotSensors[index].reset(new Klampt::RobotSensors);
    worldData->robotSensors[index]->MakeDefault(robot);
  }
  Klampt::RobotSensors* sensors = worldData->robotSensors[index].get();
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
    worldData->robotSensors[index].reset(new Klampt::RobotSensors);
    worldData->robotSensors[index]->MakeDefault(robot);
  }
  Klampt::RobotSensors* sensors = worldData->robotSensors[index].get();
  Assert(sensors != NULL);
  shared_ptr<Klampt::SensorBase> sensor = sensors->GetNamedSensor(name);
  if(sensor==NULL) {
    fprintf(stderr,"RobotModel::sensor(): Warning, sensor %s does not exist\n",name);
  }
  return SimRobotSensor(*this,sensor.get());
}

SimRobotSensor RobotModel::addSensor(const char* name,const char* type)
{
  if(!robot) throw PyException("RobotModel is empty");
  shared_ptr<WorldData> worldData = worlds[world];
  if(index >= (int)worldData->robotSensors.size())
    worldData->robotSensors.resize(index+1);
  if(!worldData->robotSensors[index]) {
    worldData->robotSensors[index].reset(new Klampt::RobotSensors);
    worldData->robotSensors[index]->MakeDefault(robot);
  }
  Klampt::RobotSensors* sensors = worldData->robotSensors[index].get();
  Assert(sensors != NULL);
  if(sensors->GetNamedSensor(name)) {
    throw PyException("Sensor name already exists");
  }
  shared_ptr<Klampt::SensorBase> newsensor = sensors->CreateByType(type);
  if(!newsensor) {
    throw PyException("Invalid sensor type");
  }
  newsensor->name = name;
  worldData->robotSensors[index]->sensors.push_back(newsensor);
  return SimRobotSensor(*this,worldData->robotSensors[index]->sensors.back().get());
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
  Klampt::WorldModel& world = *worlds[this->world]->world;
  return world.rigidObjects[index]->name.c_str();
}

void RigidObjectModel::setName(const char* name)
{
  if(!object) throw PyException("RigidObjectModel is invalid");
  if(!worlds[this->world]) throw PyException("RigidObjectModel is associated with a deleted world");
  Klampt::WorldModel& world = *worlds[this->world]->world;
  world.rigidObjects[index]->name = name;
}

int RigidObjectModel::getID() const
{
  if(!object) throw PyException("RigidObjectModel is invalid");
  if(!worlds[this->world]) throw PyException("RigidObjectModel is associated with a deleted world");
  Klampt::WorldModel& world = *worlds[this->world]->world;
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
  Klampt::RigidObjectModel* obj=object;
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
  Klampt::RigidObjectModel* obj=object;
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
  Klampt::RigidObjectModel* obj=object;
  params.kFriction = obj->kFriction;
  params.kRestitution = obj->kRestitution;
  params.kStiffness = obj->kStiffness;
  params.kDamping = obj->kDamping;
  return params;
}

void RigidObjectModel::setContactParameters(const ContactParameters& params)
{
  if(!object) throw PyException("RigidObjectModel is invalid");
  Klampt::RigidObjectModel* obj=object;
  obj->kFriction = params.kFriction;
  obj->kRestitution = params.kRestitution;
  obj->kStiffness = params.kStiffness;
  obj->kDamping = params.kDamping;
}

void RigidObjectModel::getTransform(double R[9],double t[3])
{
  if(!object) throw PyException("RigidObjectModel is invalid");
  Klampt::RigidObjectModel* obj=object;
  obj->T.R.get(R);
  obj->T.t.get(t);
}

void RigidObjectModel::setTransform(const double R[9],const double t[3])
{
  if(!object) throw PyException("RigidObjectModel is invalid");
  Klampt::RigidObjectModel* obj=object;
  obj->T.R.set(R);
  obj->T.t.set(t);
  obj->geometry->SetTransform(obj->T);
}

void RigidObjectModel::getVelocity(double out[3],double out2[3])
{
  if(!object) throw PyException("RigidObjectModel is invalid");
  Klampt::RigidObjectModel* obj=object;
  obj->w.get(out);
  obj->v.get(out2);
}

void RigidObjectModel::setVelocity(const double angularVelocity[3],const double velocity[3])
{
  if(!object) throw PyException("RigidObjectModel is invalid");
  Klampt::RigidObjectModel* obj=object;
  obj->w.set(angularVelocity);
  obj->v.set(velocity);
}

void RigidObjectModel::drawGL(bool keepAppearance)
{
  if(!object) throw PyException("RigidObjectModel is invalid");
  if(!worlds[this->world]) throw PyException("RigidObjectModel is associated with a deleted world");
  Klampt::WorldModel& world = *worlds[this->world]->world;
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
  Klampt::WorldModel& world = *worlds[this->world]->world;
  return world.terrains[index]->name.c_str();
}

void TerrainModel::setName(const char* name)
{
  if(!terrain) throw PyException("TerrainModel is invalid");
  if(!worlds[this->world]) throw PyException("TerrainModel is associated with a deleted world");
  Klampt::WorldModel& world = *worlds[this->world]->world;
  world.terrains[index]->name = name;
}


int TerrainModel::getID() const
{
  if(!terrain) throw PyException("TerrainModel is invalid");
  if(!worlds[this->world]) throw PyException("TerrainModel is associated with a deleted world");
  Klampt::WorldModel& world = *worlds[this->world]->world;
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
  Klampt::WorldModel& world = *worlds[this->world]->world;
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
  Klampt::WorldModel& rworld=*worlds[model.index]->world;
  sim->Init(&rworld);

  //setup controllers
  sim->robotControllers.resize(rworld.robots.size());
  for(size_t i=0;i<sim->robotControllers.size();i++) {
    Klampt::RobotModel* robot=rworld.robots[i].get();
    sim->SetController(i,MakeController(robot));

    sim->controlSimulators[i].sensors.MakeDefault(robot);
  }
  printf("Done\n");


  //setup ODE settings, if any
  TiXmlElement* e=worlds[world.index]->xmlWorld.GetElement("simulation");
  if(e) {
    printf("Reading simulation settings...\n");
    Klampt::XmlSimulationSettings s(e);
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
  vector<pair<Klampt::ODEObjectID,Klampt::ODEObjectID> > overlaps;
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


void Simulator::getContacts(int aid,int bid,double** out,int* m,int* n)
{
  Klampt::ODEContactList* c=sim->GetContactList(aid,bid);
  if(!c) {
    *out = NULL;
    *m = 0;
    *n = 0;
    return;
  }
  Matrix temp;
  MakeNumpyArray(out,m,n,c->points.size(),7,temp);
  for(size_t i=0;i<c->points.size();i++) {
    c->points[i].x.get(temp(i,0),temp(i,1),temp(i,2));
    c->points[i].n.get(temp(i,3),temp(i,4),temp(i,5));
    temp(i,6) = c->points[i].kFriction;
    if(bid < aid) {
      temp(i,3) = -temp(i,3);
      temp(i,4) = -temp(i,4);
      temp(i,5) = -temp(i,5);
    }
  }
}

void Simulator::getContactForces(int aid,int bid,double** out,int* m,int* n)
{
  Klampt::ODEContactList* c=sim->GetContactList(aid,bid);
  if(!c) {
    *out = NULL;
    *m = 0;
    *n = 0;
    return;
  }
  Matrix temp;
  MakeNumpyArray(out,m,n,c->forces.size(),3,temp);
  for(size_t i=0;i<c->forces.size();i++) {
    c->forces[i].get(temp(i,0),temp(i,1),temp(i,2));
    if(bid < aid) {
      temp(i,0) = -temp(i,0);
      temp(i,1) = -temp(i,1);
      temp(i,2) = -temp(i,2);
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
  Klampt::WorldModel& rworld=*worlds[world.index]->world;
  //world-object
  const Klampt::ODESimulatorSettings& settings = sim->odesim.GetSettings();
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
  Klampt::ODESimulatorSettings& settings = sim->odesim.GetSettings();
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
  Klampt::ODESimulatorSettings& settings = sim->odesim.GetSettings();
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
  sim->sim->hooks.push_back(make_shared<Klampt::WrenchHook>(body,Vector3(f),Vector3(t)));
  sim->sim->hooks.back()->autokill = true;
  //dBodyAddForce(body,f[0],f[1],f[2]);
  //dBodyAddTorque(body,t[0],t[1],t[2]);
}

void SimBody::applyForceAtPoint(const double f[3],const double pworld[3])
{
  if(!body) return;
  sim->sim->hooks.push_back(make_shared<Klampt::ForceHook>(body,Vector3(pworld),Vector3(f)));
  sim->sim->hooks.back()->autokill = true;
  //dBodyAddForceAtPos(body,f[0],f[1],f[2],pworld[0],pworld[1],pworld[2]);
}

void SimBody::applyForceAtLocalPoint(const double f[3],const double plocal[3])
{
  if(!body) return;
  sim->sim->hooks.push_back(make_shared<Klampt::LocalForceHook>(body,Vector3(plocal),Vector3(f)));
  sim->sim->hooks.back()->autokill = true;
  //dBodyAddForceAtRelPos(body,f[0],f[1],f[2],plocal[0],plocal[1],plocal[2]);
}

void SimBody::setVelocity(const double w[3],const double v[3])
{
  if(!body) return;
  dBodySetLinearVel(body,v[0],v[1],v[2]);
  dBodySetAngularVel(body,w[1],w[1],w[2]);
  Klampt::ODEObjectID id = sim->sim->WorldToODEID(objectID);
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
  Klampt::ODEObjectID id = sim->sim->WorldToODEID(objectID);
  if(id.IsRigidObject()) sim->sim->odesim.object(id.index)->SetTransform(RigidTransform(Matrix3(R),Vector3(t)));
  else if(id.IsRobot()) sim->sim->odesim.robot(id.index)->SetLinkTransform(id.bodyIndex,RigidTransform(Matrix3(R),Vector3(t)));
  else setTransform(R,t);
}

void SimBody::getObjectTransform(double out[9],double out2[3])
{
  Klampt::ODEObjectID id = sim->sim->WorldToODEID(objectID);
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
    Klampt::ODESurfaceProperties* params = &geometry->surf();
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
  Klampt::ODESurfaceProperties* params = &geometry->surf();
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
  Klampt::ODERobot* oderobot = sim->odesim.robot(link.robotIndex);
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
  Klampt::RobotMotorCommand& command = controller->command;
  //Klampt::RobotModel* robot=sim->sim->controlSimulators[index];
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
  Klampt::DriverTorqueSensor* s = controller->sensors.GetTypedSensor<Klampt::DriverTorqueSensor>();
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

SimRobotSensor::SimRobotSensor(const RobotModel& _robot,Klampt::SensorBase* _sensor)
  :robotModel(_robot),sensor(_sensor)
{}



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

void SimRobotSensor::getMeasurements(double** out,int* m)
{
  if(!sensor) {
    *out = NULL;
    *m = 0;
    return;
  }
  vector<double> vout;
  sensor->GetMeasurements(vout);
  *m = (int)vout.size();
  *out = (double*)malloc((*m)*sizeof(double));
  copy(vout.begin(),vout.end(),*out);
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

bool SimRobotSensor::getEnabled()
{
  if(!sensor) return false;
  string enabled;
  if(!sensor->GetSetting("enabled",enabled)) return false;
  if(enabled=="0") return false;
  else return true;
}

void SimRobotSensor::setEnabled(bool enabled)
{
  if(!sensor) return;
  if(enabled) sensor->SetSetting("enabled","1");
  else sensor->SetSetting("enabled","0");
}

RobotModelLink SimRobotSensor::getLink()
{
  if(!sensor) return RobotModelLink();
  std::string val;
  if(!sensor->GetSetting("link",val)) {
    throw PyException("Sensor doesn't have link attribute");
  }
  stringstream ss(val);
  int index;
  ss>>index;
  if(!ss)
    return robotModel.link(val.c_str());
  else
    return robotModel.link(index);
}

void SimRobotSensor::setLink(const RobotModelLink& link)
{
  setLink(link.index);
}

void SimRobotSensor::setLink(int link)
{
  if(!sensor) return;
  string temp;
  if(!sensor->GetSetting("link",temp)) 
    throw PyException("Sensor doesn't have link attribute");
  stringstream ss;
  ss<<link;
  sensor->SetSetting("link",ss.str());
}

void SimRobotSensor::getTransform(double out[9],double out2[3])
{
  if(!sensor) return;
  RigidTransform T;
  string val;
  if(!sensor->GetSetting("Tsensor",val)) 
    throw PyException("Sensor doesn't have Tsensor attribute");
  stringstream ss(val);
  ss >> T;
  if(!ss) {
    stringstream ss2;
    ss2<<"Error parsing Tsensor attribute: "<<val;
    throw PyException(ss2.str().c_str());
  }
  T.R.get(out);
  T.t.get(out2);
}

void SimRobotSensor::getTransformWorld(double out[9],double out2[3])
{
  if(!sensor) return;
  RigidTransform Tlocal,Tlink;
  double R[9],t[3];
  getTransform(R,t);
  Tlocal.R.set(R);
  Tlocal.t.set(t);
  RobotModelLink link=getLink();
  if(link.index < 0) Tlink.setIdentity();
  else {
    link.getTransform(R,t);
    Tlink.R.set(R);
    Tlink.t.set(t);
  }
  RigidTransform Tworld=Tlink*Tlocal;
  Tworld.R.get(out);
  Tworld.t.get(out2);
}

void SimRobotSensor::setTransform(const double R[9],const double t[3])
{
  if(!sensor) return;
  string temp;
  if(!sensor->GetSetting("Tsensor",temp)) 
    throw PyException("Sensor doesn't have Tsensor attribute");
  RigidTransform T;
  T.R.set(R);
  T.t.set(t);
  stringstream ss;
  ss<<T;
  sensor->SetSetting("Tsensor",ss.str());
}

void SimRobotSensor::drawGL()
{
  if(!sensor) return;
  vector<double> measurements;
  sensor->DrawGL(*robotModel.robot,measurements);
}

void SimRobotSensor::drawGL(double* np_array,int m)
{
  if(!sensor) return;
  std::vector<double> measurements(m);
  copy(np_array,np_array+m,&measurements[0]);
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
  Klampt::RobotSensors& sensors = controller->sensors;
  if(sensorIndex < 0 || sensorIndex >= (int)sensors.sensors.size()) 
    return SimRobotSensor(RobotModel(),NULL);
  return SimRobotSensor(model(),sensors.sensors[sensorIndex].get());
}

SimRobotSensor SimRobotController::sensor(const char* name)
{
  if(!controller) throw PyException("Invalid SimRobotController");
  Klampt::RobotSensors& sensors = controller->sensors;
  shared_ptr<Klampt::SensorBase> sensor = sensors.GetNamedSensor(name);
  if(sensor==NULL) {
    fprintf(stderr,"Warning, sensor %s does not exist\n",name);
  }
  return SimRobotSensor(model(),sensor.get());
}

SimRobotSensor SimRobotController::addSensor(const char* name,const char* type)
{
  shared_ptr<Klampt::SensorBase> newsensor = controller->sensors.CreateByType(type);
  if(!newsensor) {
    throw PyException("Invalid sensor type specified");
  }
  if(controller->sensors.GetNamedSensor(name)) {
    throw PyException("Sensor name already exists");
  }
  newsensor->name = name;
  controller->sensors.sensors.push_back(newsensor);
  controller->nextSenseTime.push_back(controller->curTime);
  return SimRobotSensor(model(),controller->sensors.sensors.back().get());
}

std::vector<std::string> SimRobotController::commands()
{
  if(!controller) throw PyException("Invalid SimRobotController");
  return controller->controller->Commands();
}

void SimRobotController::setManualMode(bool enabled)
{
  if(!controller) throw PyException("Invalid SimRobotController");
  Klampt::RobotController* c=sim->sim->robotControllers[index].get();
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
  typedef std::vector<Klampt::ActuatorCommand>::iterator it_ac;
  Klampt::RobotMotorCommand& command = controller->command;
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
  case Klampt::ActuatorCommand::OFF:
      return "off";
  case Klampt::ActuatorCommand::TORQUE:
      return "torque";
  case Klampt::ActuatorCommand::PID:
      return "PID";
  case Klampt::ActuatorCommand::LOCKED_VELOCITY:
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

void EnablePathControl(Klampt::RobotController* c)
{
  Klampt::PolynomialPathController* pc = GetPathController(c);
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
  Klampt::PolynomialMotionQueue* mq = GetMotionQueue(controller->controller);
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
  Klampt::PolynomialMotionQueue* mq = GetMotionQueue(controller->controller);
  mq->Cut(0);
  mq->AppendCubic(Vector(q),Vector(v),dt);
}
void SimRobotController::addLinear(const std::vector<double>& q,double dt)
{
  if(controller->robot->links.size() != q.size()) {
    throw PyException("Invalid size of configuration");
  }
  EnablePathControl(sim->sim->robotControllers[index].get());
  Klampt::PolynomialMotionQueue* mq = GetMotionQueue(controller->controller);
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
  Klampt::PolynomialMotionQueue* mq = GetMotionQueue(controller->controller);
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
  Klampt::PolynomialMotionQueue* mq = GetMotionQueue(controller->controller);
  Config qv(controller->robot->links.size(),&dq[0]);
  stringstream ss;
  ss<<mq->CurTime()+dt<<"\t"<<qv;
  controller->controller->SendCommand("set_tv",ss.str());
}

double SimRobotController::remainingTime() const
{
  Klampt::PolynomialMotionQueue* mq = GetMotionQueue(controller->controller);
  return mq->TimeRemaining();
}


void SimRobotController::setTorque(const std::vector<double>& t)
{
  Klampt::RobotMotorCommand& command = controller->command;
  if(t.size() != command.actuators.size()) {
    throw PyException("Invalid command size, must be equal to driver size");
  }
  for(size_t i=0;i<command.actuators.size();i++) {
    command.actuators[i].SetTorque(t[i]);
  }
  Klampt::RobotController* c=sim->sim->robotControllers[index].get();
  MyController* mc=dynamic_cast<MyController*>(c);
  if(!mc) {
    throw PyException("Not using the default manual override controller");
  }
  mc->override = true;
}

void SimRobotController::setPIDCommand(const std::vector<double>& qdes,const std::vector<double>& dqdes)
{
  Klampt::RobotMotorCommand& command = controller->command;
  Klampt::RobotModel* robot=controller->robot;
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
  Klampt::RobotController* c=sim->sim->robotControllers[index].get();
  MyController* mc=dynamic_cast<MyController*>(c);
  if(!mc) {
    throw PyException("Not using the default manual override controller");
  }
  mc->override = true;
}

void SimRobotController::setPIDCommand(const std::vector<double>& qdes,const std::vector<double>& dqdes,const std::vector<double>& tfeedforward)
{
  setPIDCommand(qdes,dqdes);
  Klampt::RobotMotorCommand& command = controller->command;
  //Klampt::RobotModel* robot=sim->sim->controlSimulators[index];
  if(tfeedforward.size() != command.actuators.size())
     throw PyException("Invalid command sizes");
  for(size_t i=0;i<command.actuators.size();i++) {
    command.actuators[i].torque = tfeedforward[i];
  }
}



void SimRobotController::setPIDGains(const std::vector<double>& kP,const std::vector<double>& kI,const std::vector<double>& kD)
{
  Klampt::RobotMotorCommand& command = controller->command;
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
  Klampt::RobotMotorCommand& command = controller->command;
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

void TransformPoser::enableTranslationAxes(bool x,bool y,bool z)
{
  GLDraw::TransformWidget* tw=dynamic_cast<GLDraw::TransformWidget*>(widgets[index].widget.get());
  tw->enableTranslationAxes[0] = x;
  tw->enableTranslationAxes[1] = y;
  tw->enableTranslationAxes[2] = z;
}

void TransformPoser::enableRotationAxes(bool x,bool y,bool z)
{
  GLDraw::TransformWidget* tw=dynamic_cast<GLDraw::TransformWidget*>(widgets[index].widget.get());
  tw->enableRotationAxes[0] = x;
  tw->enableRotationAxes[1] = y;
  tw->enableRotationAxes[2] = z;
  tw->enableOuterRingRotation = (x && y && z);
}

ObjectPoser::ObjectPoser(RigidObjectModel& object)
  :Widget()
{
  Klampt::WorldModel& world = *worlds[object.world]->world;
  Klampt::RigidObjectModel* obj = world.rigidObjects[object.index].get();
  widgets[index].widget = make_shared<Klampt::RigidObjectPoseWidget>(obj);
}

void ObjectPoser::set(const double R[9],const double t[3])
{
  Klampt::RigidObjectPoseWidget* tw=dynamic_cast<Klampt::RigidObjectPoseWidget*>(widgets[index].widget.get());
  RigidTransform T;
  T.R.set(R);
  T.t.set(t);
  tw->SetPose(T);
}

void ObjectPoser::get(double out[9],double out2[3])
{
  Klampt::RigidObjectPoseWidget* tw=dynamic_cast<Klampt::RigidObjectPoseWidget*>(widgets[index].widget.get());
  RigidTransform T = tw->Pose();
  T.R.get(out);
  T.t.get(out2);
}

RobotPoser::RobotPoser(RobotModel& robot)
{
  Assert(worlds[robot.world]->world != NULL);
  Klampt::WorldModel& world = *worlds[robot.world]->world;
  Assert(robot.index >= 0 && robot.index < world.robots.size());
  Klampt::RobotModel* rob = world.robots[robot.index].get();
  Klampt::ViewRobot* view = &world.robotViews[robot.index];
  Assert(rob != NULL);
  Assert(view != NULL);
  widgets[index].widget = make_shared<Klampt::RobotPoseWidget>(rob,view);
}

void RobotPoser::setActiveDofs(const std::vector<int>& dofs)
{
  Klampt::RobotPoseWidget* tw=dynamic_cast<Klampt::RobotPoseWidget*>(widgets[index].widget.get());
  tw->SetActiveDofs(dofs);
}

void RobotPoser::set(const std::vector<double>& q)
{
  Klampt::RobotPoseWidget* tw=dynamic_cast<Klampt::RobotPoseWidget*>(widgets[index].widget.get());
  tw->SetPose(Config(q));
}

void RobotPoser::get(std::vector<double>& out)
{
  Klampt::RobotPoseWidget* tw=dynamic_cast<Klampt::RobotPoseWidget*>(widgets[index].widget.get());
  out.resize(tw->Pose().size());
  tw->Pose().getCopy(&out[0]);
}

void RobotPoser::getConditioned(const std::vector<double>& qref,std::vector<double>& out)
{
  Klampt::RobotPoseWidget* tw=dynamic_cast<Klampt::RobotPoseWidget*>(widgets[index].widget.get());
  out.resize(tw->Pose().size());
  tw->Pose_Conditioned(Config(qref)).getCopy(&out[0]);
}

void RobotPoser::addIKConstraint(const IKObjective& obj)
{
  Klampt::RobotPoseWidget* tw=dynamic_cast<Klampt::RobotPoseWidget*>(widgets[index].widget.get());
  tw->ikPoser.ClearLink(obj.goal.link);
  tw->ikPoser.Add(obj.goal);
  tw->ikPoser.Enable(&tw->ikPoser.poseWidgets.back(),false);
}

void RobotPoser::clearIKConstraints()
{
  Klampt::RobotPoseWidget* tw=dynamic_cast<Klampt::RobotPoseWidget*>(widgets[index].widget.get());
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

void set_friction_cone_approximation_edges(int numEdges)
{
  if(numEdges < 3) throw PyException("Invalid number of friction cone approximation edges, must be at least 3");
  gStabilityNumFCEdges = numEdges;
}

void Convert(const double* contacts,int m,int n,vector<ContactPoint>& cps)
{
  if(n != 7)  throw PyException("Invalid size of contact point, must be in the format (x,y,z,nx,ny,nz,kFriction)");
  cps.resize(m);
  for(int i=0;i<m;i+=n) {
    if(contacts[i+6] < 0) throw PyException("Invalid contact point, negative friction coefficient");
    cps[i].x.set(contacts[i+0],contacts[i+1],contacts[i+2]);
    cps[i].n.set(contacts[i+3],contacts[i+4],contacts[i+5]);
    if(!FuzzyEquals(cps[i].n.normSquared(),1.0,1e-5)) throw PyException("Invalid contact point, non-unit normal");
    cps[i].kFriction = contacts[i+6];
  }
}

void Convert(const double* contacts,int m,int n,vector<ContactPoint2D>& cps)
{
  if(n != 4) throw PyException("Invalid size of contact point, must be in the format (x,y,angle,kFriction)");
  cps.resize(m);
  for(int i=0;i<m;i+=n) {
    if(contacts[i+3] < 0) throw PyException("Invalid contact point, negative friction coefficient");
    cps[i].x.set(contacts[i+0],contacts[i+1]);
    cps[i].n.set(Cos(contacts[i+2]),Sin(contacts[i+2]));
    cps[i].kFriction = contacts[i+3];
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

bool force_closure(double* contacts,int m,int n)
{
  vector<ContactPoint> cps;
  Convert(contacts,m,n,cps);
  return TestForceClosure(cps,gStabilityNumFCEdges);
}

bool force_closure(const std::vector<std::vector<double> >& contactPositions,const std::vector<std::vector<double > >& frictionCones)
{
  vector<CustomContactPoint> cps;
  Convert(contactPositions,frictionCones,cps);
  return TestForceClosure(cps);
}

bool force_closure_2d(double* contacts,int m,int n)
{
  vector<ContactPoint2D> cps;
  Convert(contacts,m,n,cps);
  return TestForceClosure(cps);
}

bool force_closure_2d(const std::vector<std::vector<double > >& contactPositions,const std::vector<std::vector<double> >& frictionCones)
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


PyObject* com_equilibrium(double* contacts,int m,int n,const vector<double>& fext,PyObject* com)
{
  if(fext.size() != 3) throw PyException("Invalid external force, must be a 3-list");
  vector<ContactPoint> cps;
  Convert(contacts,m,n,cps);
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

PyObject* com_equilibrium(const std::vector<std::vector<double> >& contactPositions,const std::vector<std::vector<double> >& frictionCones,const vector<double>& fext,PyObject* com)
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


PyObject* com_equilibrium_2d(double* contacts,int m,int n,const vector<double>& fext,PyObject* com)
{
  if(fext.size() != 2) throw PyException("Invalid external force, must be a 2-list");
  vector<ContactPoint2D> cps;
  Convert(contacts,m,n,cps);
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

PyObject* com_equilibrium_2d(const std::vector<std::vector<double> >& contactPositions,const std::vector<std::vector<double> >& frictionCones,const vector<double>& fext,PyObject* com)
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


PyObject* support_polygon(double* contacts,int m,int n)
{
  vector<ContactPoint> cps;
  Convert(contacts,m,n,cps);
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
PyObject* support_polygon(const std::vector<std::vector<double> >& contactPositions,const std::vector<std::vector<double> >& frictionCones)
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
PyObject* support_polygon_2d(double* contacts,int m,int n)
{
  throw PyException("2D support polygons not implemented yet");
}

PyObject* support_polygon_2d(const std::vector<std::vector<double> >& contacts,const std::vector<std::vector<double> >& frictionCones)
{
  throw PyException("2D support polygons not implemented yet");
}

PyObject* equilibrium_torques(const RobotModel& robot,double* contacts,int m,int n,const std::vector<int>& links,const std::vector<double>& fext,const std::vector<double>& internalTorques,double norm)
{
  if(robot.robot == NULL) throw PyException("Called with empty robot");
  if(fext.size() != 3) throw PyException("Invalid external force, must be a 3-list");
  if(!internalTorques.empty()) {
    if(internalTorques.size() != robot.robot->links.size())
      throw PyException("Invalid number of internal torques specified");
  }
  vector<ContactPoint> cps;
  CustomContactFormation formation;
  Convert(contacts,m,n,cps);
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

PyObject* equilibrium_torques(const RobotModel& robot,double* contacts,int m,int n,const std::vector<int>& links,const vector<double>& fext,double norm)
{
  vector<double> internalTorques;
  return ::equilibrium_torques(robot,contacts,m,n,links,fext,internalTorques,norm);
}




/*************************** IO CODE ***************************************/

bool subscribe_to_stream(Geometry3D& g,const char* protocol,const char* name,const char* type)
{
  shared_ptr<AnyCollisionGeometry3D>& geom = *reinterpret_cast<shared_ptr<AnyCollisionGeometry3D>*>(g.geomPtr);
  if(0==strcmp(protocol,"ros")) {
    if(0==strcmp(type,""))
      type = "PointCloud";
    if(0 == strcmp(type,"PointCloud")) {
      if(!g.isStandalone()) {
  Klampt::WorldModel& world=*worlds[g.world]->world;
  GetManagedGeometry(world,g.id).RemoveFromCache();
  return GetManagedGeometry(world,g.id).Load((string("ros:PointCloud2//")+string(name)).c_str());
      }
      printf("Warning, attaching to a ROS stream without a Klampt::ManagedGeometry.\n");
      printf("You will not be able to automatically get updates from ROS.\n");
      if(!geom) 
        geom.reset(new AnyCollisionGeometry3D());
      (*geom) = AnyCollisionGeometry3D(Meshing::PointCloud3D());
      return Klampt::ROSSubscribePointCloud(geom->AsPointCloud(),name);
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

bool detach_from_stream(const char* protocol,const char* name)
{
  if(0==strcmp(protocol,"ros")) {
    return Klampt::ROSDetach(name);
  }
  else {
    throw PyException("DetachFromStream: Unsupported protocol argument");
    return false;
  }
}

bool process_streams(const char* protocol)
{
  if((0==strcmp(protocol,"all")&&Klampt::ROSInitialized()) || 0==strcmp(protocol,"ros"))
    if(Klampt::ROSSubscribeUpdate()) return true;
  return false;
}

bool wait_for_stream(const char* protocol,const char* name,double timeout)
{
  if(0==strcmp(protocol,"ros")) {
    return Klampt::ROSWaitForUpdate(name,timeout);
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
std::string threejs_get_scene(const WorldModel& w)
{
  if(w.index < 0) return "{}";
  Klampt::WorldModel& world = *worlds[w.index]->world;

  AnyCollection obj;
  Klampt::ThreeJSExport(world,obj);
  std::ostringstream stream;
  stream<<obj;
  return stream.str();
}

///Exports the WorldModel to a JSON string ready for use in Three.js
std::string threejs_get_transforms(const WorldModel& w)
{
  if(w.index < 0) return "{}";
   Klampt::WorldModel& world = *worlds[w.index]->world;

   AnyCollection obj;
   Klampt::ThreeJSExportTransforms(world,obj);
   std::ostringstream stream;
   stream<<obj;
   return stream.str();
}

