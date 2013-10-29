#include <vector>
#include "pyerr.h"
#include "robotsim.h"
#include "Control/PathController.h"
#include "Control/FeedforwardController.h"
#include "Simulation/WorldSimulation.h"
#include "Modeling/Interpolate.h"
#include "IO/XmlWorld.h"
#include "IO/XmlODE.h"
#include <robotics/NewtonEuler.h>
#include <GLdraw/drawextra.h>
#include <GLdraw/drawMesh.h>
#include <utils/stringutils.h>
#include <ode/ode.h>
#include <fstream>
#include <unistd.h>
using namespace GLDraw;

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

static vector<SmartPointer<WorldData> > worlds;
static list<int> worldDeleteList;

static vector<SmartPointer<SimData> > sims;
static list<int> simDeleteList;

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
typedef MilestonePathController MyMilestoneController;
inline MyController* MakeController(Robot* robot)
{
  MilestonePathController* c = new MilestonePathController(*robot);
  FeedforwardController* fc = new FeedforwardController(*robot,c);
  ManualOverrideController* lc=new ManualOverrideController(*robot,fc);
  //defaults -- gravity compensation is better off with free-floating robots
  if(robot->joints[0].type == RobotJoint::Floating)
    fc->enableGravityCompensation=false;  //feedforward capability
  else
    fc->enableGravityCompensation=true;  //feedforward capability
  fc->enableGravityCompensation=true;  //feedforward capability
  fc->enableFeedforwardAcceleration=false;  //feedforward capability
  return lc;
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

Geometry3D::Geometry3D()
  :world(-1),id(-1)
{}

string Geometry3D::type()
{
  return worlds[this->world]->world.GetGeometry(id).TypeName();
}

TriangleMesh Geometry3D::getTriangleMesh()
{
  TriangleMesh mesh;
  GetMesh(worlds[this->world]->world.GetGeometry(id),mesh);
  return mesh;
}

void Geometry3D::setTriangleMesh(const TriangleMesh& mesh)
{
  RobotWorld& world=worlds[this->world]->world;
  GetMesh(mesh,world.GetGeometry(id));
  //update the display list
  world.GetAppearance(id).vertexDisplayList.erase();
  world.GetAppearance(id).faceDisplayList.erase();
  world.GetAppearance(id).Set(world.GetGeometry(id));
}

PointCloud Geometry3D::getPointCloud()
{
  PointCloud pc;
  GetPointCloud(worlds[this->world]->world.GetGeometry(id),pc);
  return pc;
}

void Geometry3D::setPointCloud(const PointCloud& pc)
{
  RobotWorld& world=worlds[this->world]->world;
  GetPointCloud(pc,world.GetGeometry(id));
  //update the display list
  world.GetAppearance(id).vertexDisplayList.erase();
  world.GetAppearance(id).faceDisplayList.erase();
  world.GetAppearance(id).Set(world.GetGeometry(id));
}

void Geometry3D::translate(const double t[3])
{
  RobotWorld& world=worlds[this->world]->world;
  RigidTransform T;
  T.R.setIdentity();
  T.t.set(t);
  world.GetGeometry(id).Transform(T);
}

void Geometry3D::transform(const double R[9],const double t[3])
{
  RobotWorld& world=worlds[this->world]->world;
  RigidTransform T;
  T.R.set(R);
  T.t.set(t);
  world.GetGeometry(id).Transform(T);  
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
  if(robot < 0  || robot >= worlds[index]->world.robots.size())
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
      r.robot = worlds[index]->world.robots[i].robot;
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
  r.robot = worlds[index]->world.robots[robot].robot;
  r.index = link;
  return r;
}

RobotModelLink WorldModel::robotLink(const char* robotname,const char* link)
{
  RobotModelLink r;
  RobotModel rob=robot(robotname);
  r.world = index;
  r.robotIndex = rob.index;
  r.robot = rob.robot;
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
  world.AddRobot(name);
  world.robots.back().robot = new Robot();
  robot.robot = world.robots.back().robot;
  return robot;
}

RigidObjectModel WorldModel::makeRigidObject(const char* name)
{
  RobotWorld& world = worlds[index]->world;
  RigidObjectModel object;
  object.world = index;
  object.index = (int)world.rigidObjects.size();
  world.AddRigidObject(name);
  world.rigidObjects.back().object = new RigidObject();
  object.object = world.rigidObjects.back().object;
  return object;
}

TerrainModel WorldModel::makeTerrain(const char* name)
{
  RobotWorld& world = worlds[index]->world;
  TerrainModel terrain;
  terrain.world = index;
  terrain.index = world.terrains.size();
  world.AddTerrain(name);
  world.terrains.back().terrain = new Environment();  
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



RobotModelLink::RobotModelLink()
  :world(-1),robotIndex(-1),robot(NULL),index(-1)
{}

RobotModel RobotModelLink::getRobot()
{
  RobotModel r;
  r.world = world;
  r.index = robotIndex;
  r.robot = robot;
  return r;
}

const char* RobotModelLink::getName()
{
  return robot->linkNames[index].c_str();
}

int RobotModelLink::getParent()
{
  return robot->parents[index];
}

void RobotModelLink::setParent(int p)
{
  if(p < 0 || p >= (int)robot->links.size())
    throw PyException("Invalid parent index");

  //TODO: check for circular references
  robot->parents[index] = p;
}

int RobotModelLink::getID()
{
  RobotWorld& world = worlds[this->world]->world;
  return world.RobotLinkID(robotIndex,index);
}

Geometry3D RobotModelLink::getGeometry()
{
  Geometry3D res;
  res.world = world;
  res.id = getID();
  return res;
}

Mass RobotModelLink::getMass()
{
  Mass mass;
  const RobotLink3D& link=robot->links[index];
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
  RobotLink3D& link=robot->links[index];
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
  RobotLink3D& link=robot->links[index];
  (link.T_World*Vector3(plocal)).get(pworld);
}

void RobotModelLink::getWorldDirection(const double vlocal[3],double vworld[3])
{
  RobotLink3D& link=robot->links[index];
  (link.T_World.R*Vector3(vlocal)).get(vworld);
}

void RobotModelLink::getLocalPosition(const double pworld[3],double plocal[3])
{
  RobotLink3D& link=robot->links[index];
  Vector3 temp;
  link.T_World.mulInverse(Vector3(pworld),temp);
  temp.get(plocal);
}

void RobotModelLink::getLocalDirection(const double vworld[3],double vlocal[3])
{
  RobotLink3D& link=robot->links[index];
  Vector3 temp;
  link.T_World.R.mulTranspose(Vector3(vworld),temp);
  temp.get(vlocal);
}


void RobotModelLink::getTransform(double R[9],double t[3])
{
  const RobotLink3D& link=robot->links[index];
  link.T_World.R.get(R);
  link.T_World.t.get(t);
}

void RobotModelLink::setTransform(const double R[9],const double t[3])
{
  RobotLink3D& link=robot->links[index];
  link.T_World.R.set(R);
  link.T_World.t.set(t);
}

void RobotModelLink::getParentTransform(double R[9],double t[3])
{
  const RobotLink3D& link=robot->links[index];
  link.T0_Parent.R.get(R);
  link.T0_Parent.t.get(t);
}

void RobotModelLink::setParentTransform(const double R[9],const double t[3])
{
  RobotLink3D& link=robot->links[index];
  link.T0_Parent.R.set(R);
  link.T0_Parent.t.set(t);
}

void RobotModelLink::getAxis(double axis[3])
{
  const RobotLink3D& link=robot->links[index];
  link.w.get(axis);
}

void RobotModelLink::setAxis(const double axis[3])
{
  RobotLink3D& link=robot->links[index];
  link.w.set(axis);
}

void RobotModelLink::getJacobian(const double p[3],vector<vector<double> >& J)
{
  Matrix Jmat;
  robot->GetFullJacobian(Vector3(p),index,Jmat);
  copy(Jmat,J);
}

void RobotModelLink::getPositionJacobian(const double p[3],vector<vector<double> >& J)
{
  Matrix Jmat;
  robot->GetPositionJacobian(Vector3(p),index,Jmat);
  copy(Jmat,J);
}

void RobotModelLink::getOrientationJacobian(vector<vector<double> >& J)
{
  Matrix Jmat;
  Jmat.resize(3,robot->links.size(),Zero);
  int j=index;
  while(j!=-1) {
    Vector3 w;
    robot->GetOrientationJacobian(index,j,w);
    Jmat(0,j)=w.x; Jmat(1,j)=w.y; Jmat(2,j)=w.z;
    j=robot->parents[j];
  }
  copy(Jmat,J);
}

void RobotModelLink::getVelocity(double out[3])
{
  Vector3 v;
  robot->GetWorldVelocity(Vector3(Zero),index,robot->dq,v);
  v.get(out);
}

void RobotModelLink::getAngularVelocity(double out[3])
{
  Vector3 v;
  robot->GetWorldAngularVelocity(index,robot->dq,v);
  v.get(out);
}

void RobotModelLink::getPointVelocity(const double plocal[3],double out[3])
{
  Vector3 v;
  robot->GetWorldVelocity(Vector3(plocal),index,robot->dq,v);
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
  glMultMatrix(Matrix4(robot->links[index].T_World));
  drawLocalGL(keepAppearance);
  glPopMatrix();
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
  RobotModelLink link;
  link.world = world;
  link.robotIndex = index;
  link.robot = robot;
  link.index = linkindex;
  return link;
}

RobotModelLink RobotModel::getLink(const char* name)
{
  RobotModelLink link;
  link.world = this->world;
  link.robot = robot;
  link.robotIndex = index;
  link.index = -1;
  for(size_t i=0;i<robot->linkNames.size();i++)
    if(name == robot->linkNames[i]) {
      link.index = (int)i;
      return link;
    }
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
      world.robots[index].view.DrawLink_World(i);
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

Geometry3D RigidObjectModel::getGeometry()
{
  Geometry3D res;
  res.world = world;
  res.id = getID();
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
    glMultMatrix(Matrix4(object->T));
    draw(object->geometry);
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

Geometry3D TerrainModel::getGeometry()
{
  Geometry3D res;
  res.world = world;
  res.id = getID();
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
    draw(terrain->geometry);
  }
}



Simulator::Simulator(const WorldModel& model)
{
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

  sim->WriteState(initialState);
}

Simulator::~Simulator()
{
  destroySim(index);
}

WorldModel Simulator::getWorld() const
{
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
  sim->ContactForce(aid,bid).get(res);
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
  for(size_t i=0;i<rworld.rigidObjects.size();i++) 
    for(size_t j=0;j<rworld.terrains.size();j++) 
      sim->EnableContactFeedback(rworld.RigidObjectID(i),rworld.TerrainID(j));
  //robot-object
  for(size_t r=0;r<rworld.robots.size();r++) {
    for(size_t i=0;i<rworld.rigidObjects.size();i++) {
      for(size_t j=0;j<rworld.robots[r].robot->links.size();j++) {
	sim->EnableContactFeedback(rworld.RigidObjectID(i),rworld.RobotLinkID(r,j));
      }
    }
    //robot-terrain
    for(size_t i=0;i<rworld.terrains.size();i++) {
      for(size_t j=0;j<rworld.robots[r].robot->links.size();j++) {
	sim->EnableContactFeedback(rworld.TerrainID(i),rworld.RobotLinkID(r,j));
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
  SimRobotController c;
  c.sim = sim;
  c.index = robot;
  return c;
}

SimRobotController Simulator::getController(const RobotModel& robot)
{
  SimRobotController c;
  c.sim = sim;
  c.index = robot.index;
  return c;
}

void SimBody::applyWrench(const double f[3],const double t[3])
{
  if(!body) return;
  dBodyAddForce(body,f[0],f[1],f[2]);
  dBodyAddTorque(body,t[0],t[1],t[2]);
}

void SimBody::setVelocity(const double w[3],const double v[3])
{
  if(!body) return;
  dBodySetLinearVel(body,v[0],v[1],v[2]);
  dBodySetAngularVel(body,w[1],w[1],w[2]);

}

void SimBody::getVelocity(double out[3],double out2[3])
{
  if(!body) return;
  const dReal* v=dBodyGetLinearVel(body);
  const dReal* w=dBodyGetAngularVel(body);
  for(int i=0;i<3;i++) out[i] = w[i];
  for(int i=0;i<3;i++) out2[i] = v[i];
}

void SimBody::setTransform(const double R[9],double t[3])
{
  if(!body) return;
  dBodySetPosition(body,t[0],t[1],t[2]);
  dMatrix3 rot;
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      rot[i*4+j] = R[i*3+j];
  dBodySetRotation(body,rot);
}

void SimBody::getTransform(double out[9],double out2[3])
{
  if(!body) return;
  const dReal* t=dBodyGetPosition(body);
  const dReal* R=dBodyGetRotation(body);
  for(int i=0;i<3;i++) out2[i] = t[i];
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      out[i*3+j] = R[i*4+j];
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

ODESurfaceProperties* SimBody::surface()
{
  if(!geometry) return NULL;
  return &geometry->surf();
}



SimBody Simulator::getBody(const RobotModelLink& link)
{
  SimBody body;
  body.body = sim->odesim.robot(link.robotIndex)->body(link.index);
  body.geometry = sim->odesim.robot(link.robotIndex)->triMesh(link.index);
  return body;
}

SimBody Simulator::getBody(const RigidObjectModel& object)
{
  SimBody body;
  body.body = sim->odesim.object(object.index)->body();
  body.geometry = sim->odesim.object(object.index)->triMesh();
  return body;  
}

SimBody Simulator::getBody(const TerrainModel& terrain)
{
  SimBody body;
  body.body = NULL;
  body.geometry = sim->odesim.envGeom(terrain.index);
  return body;
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
  Vector3 comw = T*link.robot->links[link.index].com;
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
  RobotSensors& sensors = sim->controlSimulators[index].sensors;
  if(sensorIndex < 0 || sensorIndex >= (int)sensors.sensors.size())
    return SimRobotSensor(NULL);
  return SimRobotSensor(sensors.sensors[sensorIndex]);  
}

SimRobotSensor SimRobotController::getNamedSensor(const std::string& name)
{
  RobotSensors& sensors = sim->controlSimulators[index].sensors;
  SmartPointer<SensorBase> sensor = sensors.GetNamedSensor(name);
  if(sensor==NULL) {
    fprintf(stderr,"Warning, sensor %s does not exist\n",name.c_str());
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

double SimRobotController::remainingTime() const
{
  MyController* c = dynamic_cast<MyController*>((RobotController*)sim->controlSimulators[index].controller);
  if(!c) return 0;
  FeedforwardController* fc = dynamic_cast<FeedforwardController*>((RobotController*)c->base);
  if(!fc) return 0;
  MyMilestoneController* mc = dynamic_cast<MyMilestoneController*>((RobotController*)fc->base);
  if(!mc) return 0;
  return mc->TimeRemaining();
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

