#include <vector>
#include "pyerr.h"
#include "robotsim.h"
#include "widget.h"
#include "Control/Command.h"
#include "Control/PathController.h"
#include "Control/FeedforwardController.h"
#include "Simulation/WorldSimulation.h"
#include "Modeling/Interpolate.h"
#include "IO/XmlWorld.h"
#include "IO/XmlODE.h"
#include <robotics/NewtonEuler.h>
#include <meshing/PointCloud.h>
#include <GLdraw/drawextra.h>
#include <GLdraw/drawMesh.h>
#include <GLdraw/Widget.h>
#include <GLdraw/TransformWidget.h>
#include "View/ObjectPoseWidget.h"
#include "View/RobotPoseWidget.h"
#include <utils/AnyCollection.h>
#include <utils/stringutils.h>
#include <ode/ode.h>
#include <fstream>
#ifndef WIN32
#include <unistd.h>
#endif //WIN32

/// Internally used.
struct WorldData
{
  RobotWorld world;
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

int createWorld()
{
  if(worldDeleteList.empty()) {
    worlds.push_back(new WorldData);
    worlds.back()->refCount = 1;
    return (int)(worlds.size()-1);
  }
  else {
    int index = worldDeleteList.front();
    worldDeleteList.erase(worldDeleteList.begin());
    worlds[index] = new WorldData;
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
    return (int)(widgets.size()-1);
  }
  else {
    int index = widgetDeleteList.front();
    widgetDeleteList.erase(widgetDeleteList.begin());
    widgets[index].widget = NULL;
    widgets[index].refCount = 1;
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
  //printf("Deref widget %d: count %d\n",index,widgets[index]->refCount);
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
  //printf("Ref widget %d: count %d\n",index,widgets[index]->refCount);
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
  PolynomialPathController* c = new PolynomialPathController(*robot);
  FeedforwardController* fc = new FeedforwardController(*robot,c);
  ManualOverrideController* lc=new ManualOverrideController(*robot,fc);
  //defaults -- gravity compensation is better off with free-floating robots
  if(robot->joints[0].type == RobotJoint::Floating)
    fc->enableGravityCompensation=false;  //feedforward capability
  else
    fc->enableGravityCompensation=true;  //feedforward capability
  fc->enableFeedforwardAcceleration=false;  //feedforward capability
  return lc;
}
inline PolynomialMotionQueue* GetMotionQueue(RobotController* controller)
{
  MyController* mc=dynamic_cast<MyController*>(controller);
  if(!mc) {
    throw PyException("Not using the default manual override controller");
  }
  FeedforwardController* ffc=dynamic_cast<FeedforwardController*>((RobotController*)mc->base);
  PolynomialPathController* pc=dynamic_cast<PolynomialPathController*>((RobotController*)ffc->base);
  return pc;
}
inline void MakeSensors(Robot* robot,RobotSensors& sensors)
{
  JointPositionSensor* jp = new JointPositionSensor;
  JointVelocitySensor* jv = new JointVelocitySensor;
  jp->name = "q";
  jv->name = "dq";
  jp->q.resize(robot->q.n,Zero);
  jv->dq.resize(robot->q.n,Zero);
  sensors.sensors.push_back(jp);
  sensors.sensors.push_back(jv);
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
  geom.InitCollisions();
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
  gpc.properties.resize(pc.properties.size() / pc.propertyNames.size());
  for(size_t i=0;i<gpc.properties.size();i++) {
    gpc.properties[i].resize(pc.propertyNames.size());
    gpc.properties[i].copy(&pc.properties[i*pc.propertyNames.size()]);
  }
  geom = gpc;
  geom.InitCollisions();
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
{}

Geometry3D::~Geometry3D()
{
  free();
}

bool Geometry3D::isStandalone()
{
  return(geomPtr != NULL && world < 0);
}

Geometry3D Geometry3D::clone()
{
  Geometry3D res;
  if(geomPtr != NULL) {
    AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
    res.geomPtr = new AnyCollisionGeometry3D(*geom);
  }
  return res;
}

void Geometry3D::set(const Geometry3D& g)
{
  AnyCollisionGeometry3D* ggeom = reinterpret_cast<AnyCollisionGeometry3D*>(g.geomPtr);
  if(geomPtr == NULL) {
    geomPtr = new AnyCollisionGeometry3D(*ggeom);
  }
  else {
    AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
    *geom = *ggeom;
  }
  AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
  geom->InitCollisions();
  if(!isStandalone()) {
    //update the display list
    RobotWorld& world=worlds[this->world]->world;
    world.GetAppearance(id).Set(*geom);
  }
}

void Geometry3D::free()
{
  if(isStandalone()) {
    printf("Geometry3D(): Freeing standalone geometry\n");
    AnyCollisionGeometry3D* ptr = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
    delete ptr;
  }
  world = -1;
  id = -1;
  geomPtr = NULL;
}

string Geometry3D::type()
{
  if(!geomPtr) return "";
  AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
  if(geom->Empty()) return "";
  return geom->TypeName();
}

bool Geometry3D::empty()
{
  if(!geomPtr) return true;
  AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
  if(geom->Empty()) return true;
  return false;
}

TriangleMesh Geometry3D::getTriangleMesh()
{
  TriangleMesh mesh;
  if(geomPtr) {
    AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
    GetMesh(*geom,mesh);
  }
  return mesh;
}


GeometricPrimitive Geometry3D::getGeometricPrimitive()
{
  if(!geomPtr) return GeometricPrimitive();
  AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
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
  if(!geomPtr) {
    geomPtr = new AnyCollisionGeometry3D();
  }
  AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
  GetMesh(mesh,*geom);
  if(!isStandalone()) {
    //update the display list
    RobotWorld& world=worlds[this->world]->world;
    world.GetAppearance(id).Set(*geom);
  }
}

PointCloud Geometry3D::getPointCloud()
{
  PointCloud pc;
  if(geomPtr) {
    AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
    GetPointCloud(*geom,pc);
  }
  return pc;
}

void Geometry3D::setPointCloud(const PointCloud& pc)
{
  if(!geomPtr) {
    geomPtr = new AnyCollisionGeometry3D();
  }
  AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
  GetPointCloud(pc,*geom);
  if(!isStandalone()) {
    //update the display list
    RobotWorld& world=worlds[this->world]->world;
    world.GetAppearance(id).Set(*geom);
  }
}

void Geometry3D::setGeometricPrimitive(const GeometricPrimitive& prim)
{
  if(!geomPtr) {
    geomPtr = new AnyCollisionGeometry3D();
  }
  AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
  stringstream ss(prim.saveString());
  GeometricPrimitive3D g;
  ss>>g;
  if(!ss) {
    throw PyException("Internal error");
  }
  *geom = g;
  geom->InitCollisions();
  if(!isStandalone()) {
    RobotWorld& world=worlds[this->world]->world;
    world.GetAppearance(id).Set(*geom);
  }
}


bool Geometry3D::loadFile(const char* fn)
{
  if(!geomPtr) {
    geomPtr = new AnyCollisionGeometry3D();
  }
  AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
  if(!geom->Load(fn)) return false;
  geom->InitCollisions();

  if(!isStandalone()) {
    //update the display list
    RobotWorld& world=worlds[this->world]->world;
    world.GetAppearance(id).Set(*geom);
  }
  return true;
}

bool Geometry3D::saveFile(const char* fn)
{
  if(!geomPtr) return false;
  AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
  return geom->Save(fn);
}


void Geometry3D::setCurrentTransform(const double R[9],const double t[3])
{
  if(!geomPtr) return;
  AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
  RigidTransform T;
  T.R.set(R);
  T.t.set(t);
  geom->SetTransform(T);
}

void Geometry3D::translate(const double t[3])
{
  if(!geomPtr) return;
  AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
  RigidTransform T;
  T.R.setIdentity();
  T.t.set(t);
  geom->Transform(T);
  geom->InitCollisions();

  if(!isStandalone()) {
    //update the display list
    RobotWorld& world=worlds[this->world]->world;
    world.GetAppearance(id).Set(*geom);
  }
}

void Geometry3D::transform(const double R[9],const double t[3])
{
  if(!geomPtr) return;
  AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
  RigidTransform T;
  T.R.set(R);
  T.t.set(t);
  geom->Transform(T);
  geom->InitCollisions();

  if(!isStandalone()) {
    //update the display list
    RobotWorld& world=worlds[this->world]->world;
    world.GetAppearance(id).Set(*geom);
  }
}

void Geometry3D::setCollisionMargin(double margin)
{
  if(!geomPtr) return;
  AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
  geom->margin = margin;
}

double Geometry3D::getCollisionMargin()
{
  if(!geomPtr) return 0;
  AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
  return geom->margin;
}

void Geometry3D::getBB(double out[3],double out2[3])
{
  if(!geomPtr) {
    out[0] = out[1] = out[2] = Inf;
    out2[0] = out2[1] = out2[2] = -Inf;
    return;
  }
  AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
  AABB3D bb = geom->GetAABB();
  bb.bmin.get(out);
  bb.bmax.get(out2);
}

bool Geometry3D::collides(const Geometry3D& other)
{
  if(!geomPtr || !other.geomPtr) return false;
  AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
  AnyCollisionGeometry3D* geom2 = reinterpret_cast<AnyCollisionGeometry3D*>(other.geomPtr);
  return geom->Collides(*geom2);
}

bool Geometry3D::withinDistance(const Geometry3D& other,double tol)
{
  if(!geomPtr || !other.geomPtr) return false;
  AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
  AnyCollisionGeometry3D* geom2 = reinterpret_cast<AnyCollisionGeometry3D*>(other.geomPtr);
  return geom->WithinDistance(*geom2,tol);
}

double Geometry3D::distance(const Geometry3D& other,double relErr,double absErr)
{
  if(!geomPtr || !other.geomPtr) return false;
  AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
  AnyCollisionGeometry3D* geom2 = reinterpret_cast<AnyCollisionGeometry3D*>(other.geomPtr);
  AnyCollisionQuery q(*geom,*geom2);
  return q.Distance(relErr,absErr);
}

bool Geometry3D::closestPoint(const double pt[3],double out[3])
{
  if(!geomPtr) return false;
  AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
  Vector3 vout;
  Real d = geom->Distance(Vector3(pt),vout);
  if(IsInf(d)) return false;
  vout.get(out);
  return true;
}

bool Geometry3D::rayCast(const double s[3],const double d[3],double out[3])
{
  if(!geomPtr) return false;
  AnyCollisionGeometry3D* geom = reinterpret_cast<AnyCollisionGeometry3D*>(geomPtr);
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

Appearance::Appearance()
  :world(-1),id(-1),appearancePtr(NULL)
{}

Appearance::~Appearance()
{
  free();
}

void Appearance::refresh()
{
  if(!appearancePtr) return;
  GLDraw::GeometryAppearance* app = reinterpret_cast<GLDraw::GeometryAppearance*>(appearancePtr);
  printf("Calling GeometryAppearance::Refresh()\n");
  app->Refresh();
}

bool Appearance::isStandalone()
{
  return(appearancePtr != NULL && world < 0);
}

Appearance Appearance::clone()
{
  Appearance res;
  if(appearancePtr != NULL) {
    GLDraw::GeometryAppearance* app = reinterpret_cast<GLDraw::GeometryAppearance*>(appearancePtr);
    res.appearancePtr = new GLDraw::GeometryAppearance(*app);
  }
  return res;
}

void Appearance::set(const Appearance& g)
{
  GLDraw::GeometryAppearance* gapp = reinterpret_cast<GLDraw::GeometryAppearance*>(appearancePtr);
  if(appearancePtr == NULL) {
    appearancePtr = new GLDraw::GeometryAppearance(*gapp);
  }
  else {
    GLDraw::GeometryAppearance* app = reinterpret_cast<GLDraw::GeometryAppearance*>(appearancePtr);
    *app = *gapp;
  }
}

void Appearance::free()
{
  if(isStandalone()) {
    printf("Appearance(): Freeing standalone appearance\n");
    GLDraw::GeometryAppearance* app = reinterpret_cast<GLDraw::GeometryAppearance*>(appearancePtr);
    delete app;
  }
  world = -1;
  id = -1;
  appearancePtr = NULL;
}

void Appearance::setDraw(bool draw)
{
  if(!appearancePtr) return;
  GLDraw::GeometryAppearance* app = reinterpret_cast<GLDraw::GeometryAppearance*>(appearancePtr);
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
  if(!appearancePtr) return;
  GLDraw::GeometryAppearance* app = reinterpret_cast<GLDraw::GeometryAppearance*>(appearancePtr);
  switch(primitive) {
  case ALL: app->drawFaces = app->drawVertices = app->drawEdges = draw; break;
  case VERTICES: app->drawVertices = draw; break;
  case EDGES: app->drawEdges = draw; break;
  case FACES: app->drawFaces = draw; break;
  }
}

bool Appearance::getDraw()
{
  if(!appearancePtr) return false;
  GLDraw::GeometryAppearance* app = reinterpret_cast<GLDraw::GeometryAppearance*>(appearancePtr);
  return app->drawFaces || app->drawVertices || app->drawEdges;
}

bool Appearance::getDraw(int primitive)
{
  if(!appearancePtr) return false;
  GLDraw::GeometryAppearance* app = reinterpret_cast<GLDraw::GeometryAppearance*>(appearancePtr);
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
  if(!appearancePtr) return;
  GLDraw::GeometryAppearance* app = reinterpret_cast<GLDraw::GeometryAppearance*>(appearancePtr);
  app->vertexColor.set(r,g,b,a);
  app->edgeColor.set(r,g,b,a);
  app->faceColor.set(r,g,b,a);
}

void Appearance::setColor(int primitive,float r,float g,float b,float a)
{
  if(!appearancePtr) return;
  GLDraw::GeometryAppearance* app = reinterpret_cast<GLDraw::GeometryAppearance*>(appearancePtr);
  switch(primitive) {
  case ALL:
    app->vertexColor.set(r,g,b,a);
    app->edgeColor.set(r,g,b,a);
    app->faceColor.set(r,g,b,a);
    break;
  case VERTICES: app->vertexColor.set(r,g,b,a); break;
  case EDGES: app->edgeColor.set(r,g,b,a);  break;
  case FACES: app->faceColor.set(r,g,b,a); break;
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

void Appearance::drawGL()
{
  if(!appearancePtr) return;
  GLDraw::GeometryAppearance* app = reinterpret_cast<GLDraw::GeometryAppearance*>(appearancePtr);
  if(!app->geom) return;
  app->DrawGL();
}

void Appearance::drawGL(Geometry3D& geom)
{
  if(!appearancePtr) return;
  if(!geom.geomPtr) return;
  GLDraw::GeometryAppearance* app = reinterpret_cast<GLDraw::GeometryAppearance*>(appearancePtr);
  if(app->geom) {
    if(app->geom != geom.geomPtr) {
      fprintf(stderr,"Appearance::drawGL(): performance warning, setting to a different geometry\n");
      app->Set(*reinterpret_cast<Geometry::AnyCollisionGeometry3D*>(geom.geomPtr));
    }
  }
  else {
    app->Set(*reinterpret_cast<Geometry::AnyCollisionGeometry3D*>(geom.geomPtr));
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

const WorldModel& WorldModel::operator = (const WorldModel& w)
{
  index = w.index;
  refWorld(index);
  return *this;
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
  RobotWorld& world = worlds[index]->world;

  const char* ext=FileExtension(fn);
  if(0==strcmp(ext,"rob")) {
    if(world.LoadRobot(fn)<0) {
      printf("Error loading robot file %s\n",fn);
      return false;
    }
  }
  else if(0==strcmp(ext,"env") || 0==strcmp(ext,"tri")) {
    if(world.LoadTerrain(fn)<0) {
      printf("Error loading terrain file %s\n",fn);
      return false;
    }
  }
  else if(0==strcmp(ext,"obj")) {
    if(world.LoadRigidObject(fn)<0) {
      printf("Error loading rigid object file %s\n",fn);
      return false;
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
  RobotWorld& world = worlds[index]->world;
  return world.robots.size();
}

int WorldModel::numRobotLinks(int robot)
{
  RobotWorld& world = worlds[index]->world;
  return world.robots[robot].robot->links.size();
}

int WorldModel::numRigidObjects()
{
  RobotWorld& world = worlds[index]->world;
  return world.rigidObjects.size();
}

int WorldModel::numTerrains()
{
  RobotWorld& world = worlds[index]->world;
  return world.terrains.size();
}

int WorldModel::numIDs()
{
  RobotWorld& world = worlds[index]->world;
  return world.NumIDs();
}

RobotModel WorldModel::robot(int robot)
{
  if(robot < 0  || robot >= (int)worlds[index]->world.robots.size())
    throw PyException("Invalid robot index");
  RobotModel r;
  r.world = index;
  r.index = robot;
  r.robot = worlds[index]->world.robots[robot].robot;
  return r;
}

RobotModel WorldModel::robot(const char* robot)
{
  RobotWorld& world = worlds[index]->world;
  RobotModel r;
  r.world = index;
  for(size_t i=0;i<world.robots.size();i++)
    if(world.robots[i].name == robot) {
      r.index = (int)i;
      r.robot = world.robots[i].robot;
      return r;
    }
  throw PyException("Invalid robot name");
  return r;
}

RobotModelLink WorldModel::robotLink(int robot,int link)
{
  if(robot < 0  || robot >= (int)worlds[index]->world.robots.size())
    throw PyException("Invalid robot index");
  RobotModelLink r;
  r.world = index;
  r.robotIndex = robot;
  r.robotPtr = worlds[index]->world.robots[robot].robot;
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
  if(object < 0  || object >= (int)worlds[index]->world.rigidObjects.size())
    throw PyException("Invalid rigid object index");
  RigidObjectModel obj;
  obj.world = index;
  obj.index = object;
  obj.object = worlds[index]->world.rigidObjects[object].object;
  return obj;
}

RigidObjectModel WorldModel::rigidObject(const char* object)
{
  RobotWorld& world = worlds[index]->world;
  RigidObjectModel obj;
  obj.world = index;
  for(size_t i=0;i<world.rigidObjects.size();i++)
    if(world.rigidObjects[i].name == object) {
      obj.index = (int)i;
      obj.object = worlds[index]->world.rigidObjects[i].object;
      return obj;
    }
  throw PyException("Invalid rigid object name");
  return obj;
}

TerrainModel WorldModel::terrain(int terrain)
{
  if(terrain < 0  || terrain >= (int)worlds[index]->world.terrains.size())
    throw PyException("Invalid terrain index");

  TerrainModel t;
  t.world = index;
  t.index = terrain;
  t.terrain = worlds[index]->world.terrains[terrain].terrain;
  return t;
}

TerrainModel WorldModel::terrain(const char* terrain)
{
  TerrainModel t;
  t.world = index;
  RobotWorld& world = worlds[index]->world;
  for(size_t i=0;i<world.terrains.size();i++)
    if(world.terrains[i].name == terrain) {
      t.index = (int)i;
      t.terrain = worlds[index]->world.terrains[i].terrain;
      return t;
    }
  return t;
}

RobotModel WorldModel::makeRobot(const char* name)
{
  RobotWorld& world = worlds[index]->world;
  RobotModel robot;
  robot.world = index;
  robot.index = (int)world.robots.size();
  world.AddRobot(name,new Robot());
  robot.robot = world.robots.back().robot;
  return robot;
}

RigidObjectModel WorldModel::makeRigidObject(const char* name)
{
  RobotWorld& world = worlds[index]->world;
  RigidObjectModel object;
  object.world = index;
  object.index = (int)world.rigidObjects.size();
  world.AddRigidObject(name,new RigidObject());
  object.object = world.rigidObjects.back().object;
  return object;
}

TerrainModel WorldModel::makeTerrain(const char* name)
{
  RobotWorld& world = worlds[index]->world;
  TerrainModel terrain;
  terrain.world = index;
  terrain.index = world.terrains.size();
  world.AddTerrain(name,new Environment());
  terrain.terrain = world.terrains.back().terrain;
  return terrain;
}

RobotModel WorldModel::loadRobot(const char* fn)
{
  RobotWorld& world = worlds[index]->world;
  int oindex=world.LoadRobot(fn);
  if(oindex < 0) return RobotModel();
  RobotModel robot;
  robot.world = index;
  robot.index = oindex;
  robot.robot = world.robots.back().robot;
  return robot;
}

RigidObjectModel WorldModel::loadRigidObject(const char* fn)
{
  RobotWorld& world = worlds[index]->world;
  int oindex=world.LoadRigidObject(fn);
  if(oindex < 0) return RigidObjectModel();
  RigidObjectModel obj;
  obj.world = index;
  obj.index = oindex;
  obj.object = world.rigidObjects.back().object;
  return obj;
}

TerrainModel WorldModel::loadTerrain(const char* fn)
{
  RobotWorld& world = worlds[index]->world;
  int oindex=world.LoadTerrain(fn);
  if(oindex < 0) return TerrainModel();
  TerrainModel obj;
  obj.world = index;
  obj.index = oindex;
  obj.terrain = world.terrains.back().terrain;
  return obj;
}

int WorldModel::loadElement(const char* fn)
{
  RobotWorld& world = worlds[index]->world;
  int id = world.LoadElement(fn);
  return id;
}

void WorldModel::drawGL()
{
  RobotWorld& world = worlds[index]->world;
  world.DrawGL();
}

void WorldModel::enableGeometryLoading(bool enabled)
{
  Robot::disableGeometryLoading = !enabled;
}

std::string WorldModel::getName(int id)
{
  RobotWorld& world = worlds[index]->world;
  return world.GetName(id);
}

Geometry3D WorldModel::geometry(int id)
{
  RobotWorld& world = worlds[index]->world;
  if(world.IsTerrain(id)>=0 || world.IsRigidObject(id)>=0 || world.IsRobotLink(id).first>=0) {
    Geometry3D geom;
    geom.world = index;
    geom.id = id;
    geom.geomPtr = &world.GetGeometry(id);
    return geom;
  }
  Geometry3D geom;
  geom.world = -1;
  geom.id = -1;
  return geom;
}

Appearance WorldModel::appearance(int id)
{
  RobotWorld& world = worlds[index]->world;
  if(world.IsTerrain(id)>=0 || world.IsRigidObject(id)>=0 || world.IsRobotLink(id).first>=0) {
    Appearance geom;
    geom.world = index;
    geom.id = id;
    geom.appearancePtr = &world.GetAppearance(id);
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
  fprintf(stderr,"RobotModelLink::getRobot() will be deprecated, please use robot() instead\n");
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

void RobotModelLink::setParent(int p)
{
  if(p < 0 || p >= (int)robotPtr->links.size())
    throw PyException("Invalid parent index");

  //TODO: check for circular references
  robotPtr->parents[index] = p;
}

int RobotModelLink::getID()
{
  RobotWorld& world = worlds[this->world]->world;
  return world.RobotLinkID(robotIndex,index);
}

Geometry3D RobotModelLink::geometry()
{
  Geometry3D res;
  res.world = world;
  res.id = getID();
  res.geomPtr = &worlds[world]->world.GetGeometry(res.id);
  return res;
}

Appearance RobotModelLink::appearance()
{
  Appearance res;
  res.world = world;
  res.id = getID();
  res.appearancePtr = &worlds[world]->world.GetAppearance(res.id);
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
  robotPtr->geometry[index].SetTransform(link.T_World);
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
  RobotWorld& world = worlds[this->world]->world;
  if(keepAppearance) {
    world.robots[robotIndex].view.DrawLink_Local(index);
  }
  else
    world.robots[robotIndex].robot->DrawLinkGL(index);
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
  fprintf(stderr,"RobotModelDriver::getRobot() will be deprecated, please use robot() instead\n");
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
  RobotWorld& world = worlds[this->world]->world;
  return world.robots[index].name.c_str();
}

int RobotModel::getID()
{
  RobotWorld& world = worlds[this->world]->world;
  return world.RobotID(index);
}

int RobotModel::numLinks()
{
  return robot->links.size();
}

RobotModelLink RobotModel::getLink(int linkindex)
{
  fprintf(stderr,"RobotModel::getLink() will be deprecated, please use link() instead\n");
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
  fprintf(stderr,"RobotModel::getLink() will be deprecated, please use link() instead\n");
  return link(name);
}

RobotModelLink RobotModel::link(const char* name)
{
  for(size_t i=0;i<robot->linkNames.size();i++)
    if(string(name) == robot->linkNames[i]) {
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
  fprintf(stderr,"RobotModel::getDriver() will be deprecated, please use driver() instead\n");
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
  fprintf(stderr,"RobotModel::getDriver() will be deprecated, please use driver() instead\n");
  return driver(name);
}

RobotModelDriver RobotModel::driver(const char* name)
{
  for(size_t i=0;i<robot->driverNames.size();i++)
    if(name == robot->driverNames[i]) {
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

void RobotModel::drawGL(bool keepAppearance)
{
  RobotWorld& world = worlds[this->world]->world;
  if(keepAppearance) {
    world.robots[index].view.Draw();
  }
  else {
    for(size_t i=0;i<robot->links.size();i++)
      world.robots[index].view.DrawLink_World(i,false);
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
  RobotWorld& world = worlds[this->world]->world;
  return world.rigidObjects[index].name.c_str();
}

int RigidObjectModel::getID()
{
  RobotWorld& world = worlds[this->world]->world;
  return world.RigidObjectID(index);
}

Geometry3D RigidObjectModel::geometry()
{
  Geometry3D res;
  res.world = world;
  res.id = getID();
  res.geomPtr = &worlds[world]->world.GetGeometry(res.id);
  return res;
}

Appearance RigidObjectModel::appearance()
{
  Appearance res;
  res.world = world;
  res.id = getID();
  res.appearancePtr = &worlds[world]->world.GetAppearance(res.id);
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
  obj->geometry.SetTransform(obj->T);
}

void RigidObjectModel::drawGL(bool keepAppearance)
{
  RobotWorld& world = worlds[this->world]->world;
  if(keepAppearance) {
    world.rigidObjects[index].view.Draw();
  }
  else {
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    GLDraw::glMultMatrix(Matrix4(object->T));
    GLDraw::draw(object->geometry);
    glPopMatrix();
  }
}


TerrainModel::TerrainModel()
  :world(-1),index(-1),terrain(NULL)
{
}

const char* TerrainModel::getName()
{
  RobotWorld& world = worlds[this->world]->world;
  return world.terrains[index].name.c_str();
}

int TerrainModel::getID()
{
  RobotWorld& world = worlds[this->world]->world;
  return world.TerrainID(index);
}

Geometry3D TerrainModel::geometry()
{
  Geometry3D res;
  res.world = world;
  res.id = getID();
  res.geomPtr = &worlds[world]->world.GetGeometry(res.id);
  return res;
}


Appearance TerrainModel::appearance()
{
  Appearance res;
  res.world = world;
  res.id = getID();
  res.appearancePtr = &worlds[world]->world.GetAppearance(res.id);
  return res;
}

void TerrainModel::setFriction(double friction)
{
  terrain->SetUniformFriction(friction);
}

void TerrainModel::drawGL(bool keepAppearance)
{
  RobotWorld& world = worlds[this->world]->world;
  if(keepAppearance) {
    world.terrains[index].view.Draw();
  }
  else {
    GLDraw::draw(terrain->geometry);
  }
}



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
  RobotWorld& rworld=worlds[model.index]->world;
  sim->Init(&rworld);

  //setup controllers
  sim->robotControllers.resize(rworld.robots.size());
  for(size_t i=0;i<sim->robotControllers.size();i++) {
    Robot* robot=rworld.robots[i].robot;
    sim->SetController(i,MakeController(robot));

    MakeSensors(robot,sim->controlSimulators[i].sensors);
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
  for(size_t i=0;i<sim->odesim.numObjects();i++)
    dBodySetAutoDisableFlag(sim->odesim.object(i)->body(),1);

  sim->WriteState(initialState);
}

Simulator::~Simulator()
{
  destroySim(index);
}

WorldModel Simulator::getWorld() const
{
  fprintf(stderr,"Simulator::getWorld() will be deprecated, please use world instead\n");
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
  RobotWorld& rworld=worlds[world.index]->world;
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
    for(size_t j=0;j<rworld.robots[r].robot->links.size();j++) {
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
    for(size_t k=0;k<rworld.robots[r].robot->links.size();k++) {
      if(rworld.robots[r].robot->selfCollisions(j,k)) {
        sim->EnableContactFeedback(rworld.RobotLinkID(r,k),linkid);
      }
    }
      }
      //robot-robot
      if(settings.robotRobotCollisions) {
    for(size_t i=0;i<rworld.robots.size();i++) {
      if(i==r) continue;
      for(size_t k=0;k<rworld.robots[i].robot->links.size();k++) {
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
  fprintf(stderr,"Simulator::getController() will be deprecated, please use controller() instead\n");
  return controller(robot);
}


SimRobotController Simulator::controller(int robot)
{
  SimRobotController c;
  c.sim = sim;
  c.index = robot;
  return c;
}


SimRobotController Simulator::getController(const RobotModel& robot)
{
  fprintf(stderr,"Simulator::getController() will be deprecated, please use controller() instead\n");
  return controller(robot);
}

SimRobotController Simulator::controller(const RobotModel& robot)
{
  SimRobotController c;
  c.sim = sim;
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

void SimBody::applyWrench(const double f[3],const double t[3])
{
  if(!body) return;
  dBodyAddForce(body,f[0],f[1],f[2]);
  dBodyAddTorque(body,t[0],t[1],t[2]);
}

void SimBody::applyForceAtPoint(const double f[3],const double pworld[3])
{
  if(!body) return;
  dBodyAddForceAtPos(body,f[0],f[1],f[2],pworld[0],pworld[1],pworld[2]);
}

void SimBody::applyForceAtLocalPoint(const double f[3],const double plocal[3])
{
  if(!body) return;
  dBodyAddForceAtRelPos(body,f[0],f[1],f[2],plocal[0],plocal[1],plocal[2]);
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
  fprintf(stderr,"Simulator::getBody() will be deprecated, please use body() instead\n");
  return body(link);
}

SimBody Simulator::getBody(const RigidObjectModel& object)
{
  fprintf(stderr,"Simulator::getBody() will be deprecated, please use body() instead\n");
  return body(object);
}

SimBody Simulator::getBody(const TerrainModel& terrain)
{
  fprintf(stderr,"Simulator::getBody() will be deprecated, please use body() instead\n");
  return body(terrain);
}

SimBody Simulator::body(const RobotModelLink& link)
{
  SimBody b;
  b.body = sim->odesim.robot(link.robotIndex)->body(link.index);
  b.geometry = sim->odesim.robot(link.robotIndex)->triMesh(link.index);
  return b;
}

SimBody Simulator::body(const RigidObjectModel& object)
{
  SimBody b;
  b.body = sim->odesim.object(object.index)->body();
  b.geometry = sim->odesim.object(object.index)->triMesh();
  return b; 
}

SimBody Simulator::body(const TerrainModel& terrain)
{
  SimBody b;
  b.body = NULL;
  b.geometry = sim->odesim.envGeom(terrain.index);
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
:index(-1),sim(NULL)
{}

SimRobotController::~SimRobotController()
{}

void SimRobotController::setRate(double dt)
{
  sim->controlSimulators[index].controlTimeStep = dt;
}

void SimRobotController::getCommandedConfig(vector<double>& q)
{
  Vector qv;
  sim->controlSimulators[index].GetCommandedConfig(qv);
  q.resize(qv.n);
  qv.getCopy(&q[0]);
}

void SimRobotController::getCommandedVelocity(vector<double>& dq)
{
  Vector qv;
  sim->controlSimulators[index].GetCommandedVelocity(qv);
  dq.resize(qv.n);
  qv.getCopy(&dq[0]);
}

void SimRobotController::getSensedConfig(vector<double>& q)
{
  Vector qv;
  sim->controlSimulators[index].GetSensedConfig(qv);
  if(!qv.empty()) {
    q.resize(qv.n);
    qv.getCopy(&q[0]);
  }
}

void SimRobotController::getSensedVelocity(vector<double>& dq)
{
  Vector qv;
  sim->controlSimulators[index].GetSensedVelocity(qv);
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
  fprintf(stderr,"SimRobotController::getSensor() will be deprecated, please use sensor() instead\n");
  return sensor(sensorIndex);
}

SimRobotSensor SimRobotController::getNamedSensor(const std::string& name)
{
  fprintf(stderr,"SimRobotController::getNamedSensor() will be deprecated, please use sensor() instead\n");
  return sensor(name.c_str());
}
SimRobotSensor SimRobotController::sensor(int sensorIndex)
{
  RobotSensors& sensors = sim->controlSimulators[index].sensors;
  if(sensorIndex < 0 || sensorIndex >= (int)sensors.sensors.size())
    return SimRobotSensor(NULL);
  return SimRobotSensor(sensors.sensors[sensorIndex]);
}

SimRobotSensor SimRobotController::sensor(const char* name)
{
  RobotSensors& sensors = sim->controlSimulators[index].sensors;
  SmartPointer<SensorBase> sensor = sensors.GetNamedSensor(name);
  if(sensor==NULL) {
    fprintf(stderr,"Warning, sensor %s does not exist\n",name);
  }
  return SimRobotSensor(sensor);
}

std::vector<std::string> SimRobotController::commands()
{
  return sim->controlSimulators[index].controller->Commands();
}

void SimRobotController::setManualMode(bool enabled)
{
  RobotController* c=sim->robotControllers[index];
  MyController* mc=dynamic_cast<MyController*>(c);
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
  RobotMotorCommand& command = sim->controlSimulators[index].command;
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
  return sim->controlSimulators[index].controller->SendCommand(name,args);
}

std::string SimRobotController::getSetting(const std::string& name)
{
  std::string val;
  if(!sim->controlSimulators[index].controller->GetSetting(name,val)) return "";
  return val;
}

bool SimRobotController::setSetting(const std::string& name,const std::string& val)
{
  return sim->controlSimulators[index].controller->SetSetting(name,val);
}

void SimRobotController::setMilestone(const vector<double>& q)
{
  Config qv(sim->controlSimulators[index].robot->links.size(),&q[0]);
  stringstream ss;
  ss<<qv;
  sim->controlSimulators[index].controller->SendCommand("set_q",ss.str());
}

void SimRobotController::setMilestone(const vector<double>& q,const vector<double>& dq)
{
  Config qv(sim->controlSimulators[index].robot->links.size(),&q[0]);
  Config dqv(sim->controlSimulators[index].robot->links.size(),&dq[0]);
  stringstream ss;
  ss<<qv<<"\t"<<dqv;
  sim->controlSimulators[index].controller->SendCommand("set_qv",ss.str());
}


void SimRobotController::addMilestone(const vector<double>& q)
{
  Config qv(sim->controlSimulators[index].robot->links.size(),&q[0]);
  stringstream ss;
  ss<<qv;
  sim->controlSimulators[index].controller->SendCommand("append_q",ss.str());
}

void SimRobotController::addMilestoneLinear(const vector<double>& q)
{
  Config qv(sim->controlSimulators[index].robot->links.size(),&q[0]);
  stringstream ss;
  ss<<qv;
  sim->controlSimulators[index].controller->SendCommand("append_q_linear",ss.str());
}

void SimRobotController::setLinear(const std::vector<double>& q,double dt)
{
  PolynomialMotionQueue* mq = GetMotionQueue(sim->controlSimulators[index].controller);
  mq->Cut(0);
  mq->AppendLinear(q,dt);
}
void SimRobotController::setCubic(const std::vector<double>& q,const std::vector<double>& v,double dt)
{
  PolynomialMotionQueue* mq = GetMotionQueue(sim->controlSimulators[index].controller);
  mq->Cut(0);
  mq->AppendCubic(q,v,dt);
}
void SimRobotController::appendLinear(const std::vector<double>& q,double dt)
{
  PolynomialMotionQueue* mq = GetMotionQueue(sim->controlSimulators[index].controller);
  mq->AppendLinear(q,dt);
}
void SimRobotController::addCubic(const std::vector<double>& q,const std::vector<double>& v,double dt)
{
  PolynomialMotionQueue* mq = GetMotionQueue(sim->controlSimulators[index].controller);
  mq->AppendCubic(q,v,dt);
}

void SimRobotController::addMilestone(const vector<double>& q,const vector<double>& dq)
{
  Config qv(sim->controlSimulators[index].robot->links.size(),&q[0]);
  Config dqv(sim->controlSimulators[index].robot->links.size(),&dq[0]);
  stringstream ss;
  ss<<qv<<"\t"<<dqv;
  sim->controlSimulators[index].controller->SendCommand("append_qv",ss.str());
}

void SimRobotController::setVelocity(const vector<double>& dq,double dt)
{
  Config qv(sim->controlSimulators[index].robot->links.size(),&dq[0]);
  stringstream ss;
  ss<<dt<<"\t"<<qv;
  sim->controlSimulators[index].controller->SendCommand("set_tv",ss.str());
}

double SimRobotController::remainingTime() const
{
  PolynomialMotionQueue* mq = GetMotionQueue(sim->controlSimulators[index].controller);
  return mq->TimeRemaining();
}


void SimRobotController::setTorque(const std::vector<double>& t)
{
  RobotMotorCommand& command = sim->controlSimulators[index].command;
  if(t.size() != command.actuators.size()) {
    throw PyException("Invalid command size, must be equal to driver size");
  }
  for(size_t i=0;i<command.actuators.size();i++) {
    command.actuators[i].SetTorque(t[i]);
  }
  RobotController* c=sim->robotControllers[index];
  MyController* mc=dynamic_cast<MyController*>(c);
  if(!mc) {
    throw PyException("Not using the default manual override controller");
  }
  mc->override = true;
}

void SimRobotController::setPIDCommand(const std::vector<double>& qdes,const std::vector<double>& dqdes)
{
  RobotMotorCommand& command = sim->controlSimulators[index].command;
  Robot* robot=sim->controlSimulators[index].robot;
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
  RobotController* c=sim->robotControllers[index];
  MyController* mc=dynamic_cast<MyController*>(c);
  if(!mc) {
    throw PyException("Not using the default manual override controller");
  }
  mc->override = true;
}

void SimRobotController::setPIDCommand(const std::vector<double>& qdes,const std::vector<double>& dqdes,const std::vector<double>& tfeedforward)
{
  setPIDCommand(qdes,dqdes);
  RobotMotorCommand& command = sim->controlSimulators[index].command;
  Robot* robot=sim->controlSimulators[index].robot;
  if(tfeedforward.size() != command.actuators.size())
     throw PyException("Invalid command sizes");
  for(size_t i=0;i<command.actuators.size();i++) {
    command.actuators[i].torque = tfeedforward[i];
  }
}



void SimRobotController::setPIDGains(const std::vector<double>& kP,const std::vector<double>& kI,const std::vector<double>& kD)
{
  RobotMotorCommand& command = sim->controlSimulators[index].command;
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
  double distance;
  Camera::Viewport vp = GetCameraViewport(viewport);
  bool res=widgets[index].widget->Hover(x,y,vp,distance);
  if(res) widgets[index].widget->SetHighlight(true);
  else widgets[index].widget->SetHighlight(false);
  return res;
}

bool Widget::beginDrag(int x,int y,const Viewport& viewport)
{
  double distance;
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
      derefWidget(subwidget.index);
      i--;
    }
}

void WidgetSet::enable(const Widget& subwidget,bool enabled)
{
  GLDraw::WidgetSet* ws=dynamic_cast<GLDraw::WidgetSet*>(&*widgets[index].widget);
  for(size_t i=0;i<ws->widgets.size();i++)
    if(ws->widgets[i] == widgets[subwidget.index].widget) {
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
  RobotWorld& world = worlds[object.world]->world;
  RigidObject* obj = world.rigidObjects[object.index].object;
  ViewRigidObject* view = &world.rigidObjects[object.index].view;
  widgets[index].widget = new RigidObjectPoseWidget(obj,view);
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
  RobotWorld& world = worlds[robot.world]->world;
  Robot* rob = world.robots[robot.index].robot;
  ViewRobot* view = &world.robots[robot.index].view;
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
