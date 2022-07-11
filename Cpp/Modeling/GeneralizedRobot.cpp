#include "GeneralizedRobot.h"
#include "Interpolate.h"
#include <KrisLibrary/math3d/interpolate.h>

namespace Klampt {

void ConfigToTransform(const Vector& q,RigidTransform& T)
{
  T.t.set(q[0],q[1],q[2]);
  EulerAngleRotation ea(q[3],q[4],q[5]);
  ea.getMatrixZYX(T.R);
}

void TransformToConfig(const RigidTransform& T,Vector& q)
{
  q.resize(6);
  T.t.get(q[0],q[1],q[2]);
  EulerAngleRotation ea(q[3],q[4],q[5]);
  ea.setMatrixZYX(T.R);
  ea.get(q[3],q[4],q[5]);
}

void ObjectToRobot(const RigidObjectModel& object,RobotModel& robot)
{
  robot.InitializeRigidObject();
  robot.torqueMax.setZero();
  robot.powerMax.setZero();
  robot.geometry.resize(6);
  robot.geometry[5] = object.geometry;
  robot.geomFiles.resize(6);
  robot.geomFiles[5] = object.geomFile;
  robot.selfCollisions.resize(6,6,NULL);
  robot.envCollisions.resize(6,NULL);
  robot.links[5].mass = object.mass;
  robot.links[5].com = object.com;
  robot.links[5].inertia = object.inertia;
  TransformToConfig(object.T,robot.q);
  robot.accMax.resize(6,Inf);
  robot.joints.resize(1);
  robot.joints[0].type = RobotModelJoint::Floating;
  robot.joints[0].linkIndex = 5;
  robot.joints[0].baseIndex = -1;
  robot.drivers.resize(0);
  robot.linkNames.resize(6);
  robot.linkNames[0] = "x";
  robot.linkNames[1] = "y";
  robot.linkNames[2] = "z";
  robot.linkNames[3] = "rz";
  robot.linkNames[4] = "ry";
  robot.linkNames[5] = "rx";
  robot.driverNames.resize(0);
}

GeneralizedRobotModel::GeneralizedRobotModel()
{}

GeneralizedRobotModel::GeneralizedRobotModel(WorldModel& world)
{
  for(size_t i=0;i<world.robots.size();i++)
    Add(world.robots[i].get(),world.robots[i]->name.c_str());
  for(size_t i=0;i<world.rigidObjects.size();i++)
    Add(world.rigidObjects[i].get(),world.rigidObjects[i]->name.c_str());
}

int GeneralizedRobotModel::NumDof() const
{
  if(elements.empty()) return 0;
  return (--elements.end())->second.indexEnd;
}

int GeneralizedRobotModel::Add(RobotModel* robot,const char* name)
{
  Element e;
  e.robot = robot;
  e.object = NULL;
  if(name)
    e.name = name;
  else
    e.name = robot->name;
  int lastid = 0, lastend = 0;
  if(!elements.empty()) {
    lastid = (--elements.end())->first;
    lastend = NumDof();
  }
  e.indexStart = lastend;
  e.indexEnd = e.indexStart+robot->links.size();
  elements[lastid+1] = e;
  return lastid+1;
}

int GeneralizedRobotModel::Add(RigidObjectModel* object,const char* name)
{
  Element e;
  e.robot = NULL;
  e.object = object;
  if(name)
    e.name = name;
  else
    e.name = object->name;
  int lastid = 0, lastend = 0;
  if(!elements.empty()) {
    lastid = (--elements.end())->first;
    lastend = NumDof();
  }
  e.indexStart = lastend;
  e.indexEnd = e.indexStart+6;
  elements[lastid+1] = e;
  return lastid+1;
}

void GeneralizedRobotModel::Remove(int id)
{
  Assert(elements.count(id) != 0);
  int diff = elements[id].indexEnd - elements[id].indexStart;
  elements.erase(elements.find(id));
  for(map<int,Element>::iterator i=elements.begin();i!=elements.end();i++) {
    i->second.indexStart -= diff;
    i->second.indexEnd -= diff;
  }
}

int GeneralizedRobotModel::ID(const char* name) const
{
  for(map<int,Element>::const_iterator i=elements.begin();i!=elements.end();i++) 
    if(i->second.name == name)
      return i->first;
  return -1;
}


int GeneralizedRobotModel::ID(RobotModel* robot) const
{
  for(map<int,Element>::const_iterator i=elements.begin();i!=elements.end();i++) 
    if(i->second.robot == robot)
      return i->first;
  return -1;
}

int GeneralizedRobotModel::ID(RigidObjectModel* object) const
{
  for(map<int,Element>::const_iterator i=elements.begin();i!=elements.end();i++) 
    if(i->second.object == object)
      return i->first;
  return -1;
}

int GeneralizedRobotModel::DofToID(int index) const
{
  for(map<int,Element>::const_iterator i=elements.begin();i!=elements.end();i++) {
    if(index >= i->second.indexStart && index < i->second.indexEnd) 
      return i->first;
  }
  return -1;
}

string GeneralizedRobotModel::DofName(int index) const
{
  static const char* dofnames[6] = {"x","y","z","rz","ry","rx"};
  for(map<int,Element>::const_iterator i=elements.begin();i!=elements.end();i++) {
    if(index >= i->second.indexStart && index < i->second.indexEnd) {
      stringstream ss;
      if(i->second.name.empty())
	ss<<"element_"<<i->first;
      else
	ss<<i->second.name;
      ss<<"[";
      if(i->second.robot)
	ss<<i->second.robot->LinkName(index-i->second.indexStart);
      else 
	ss<<dofnames[index-i->second.indexStart];
      ss<<"]";
      return ss.str();
    }
  }
  return "invalid";
}

pair<int,int> GeneralizedRobotModel::Dofs(int id) const
{
  return pair<int,int>(elements.find(id)->second.indexStart,elements.find(id)->second.indexEnd);
}

void GeneralizedRobotModel::SetConfig(const Config& q)
{
  Assert(q.n == NumDof());
  for(map<int,Element>::const_iterator i=elements.begin();i!=elements.end();i++) {
    Config sub;
    sub.setRef(q,i->second.indexStart,1,i->second.indexEnd-i->second.indexStart);
    if(i->second.robot) {
      i->second.robot->UpdateConfig(sub);
    }
    else {
      ConfigToTransform(sub,i->second.object->T);
    }
  }
}

void GeneralizedRobotModel::SetVelocity(const Vector& v)
{
  Assert(v.n == NumDof());
  for(map<int,Element>::const_iterator i=elements.begin();i!=elements.end();i++) {
    Config sub;
    sub.setRef(v,i->second.indexStart,1,i->second.indexEnd-i->second.indexStart);
    if(i->second.robot) {
      i->second.robot->dq = sub;
    }
    else {
      //TODO: Rigid body velocities
    }
  }
}

void GeneralizedRobotModel::GetConfig(Config& q) const
{
  q.resize(NumDof());
  for(map<int,Element>::const_iterator i=elements.begin();i!=elements.end();i++) {
    Config sub;
    if(i->second.robot) {
      sub = i->second.robot->q;
    }
    else {
      const RigidTransform& T = i->second.object->T;
      TransformToConfig(T,sub);
    }
    q.copySubVector(i->second.indexStart,sub);
  }
}

void GeneralizedRobotModel::GetVelocity(Vector& v) const
{
  v.resize(NumDof());
  for(map<int,Element>::const_iterator i=elements.begin();i!=elements.end();i++) {
    Config sub;
    if(i->second.robot) {
      sub = i->second.robot->dq;
    }
    else {
      sub.resize(6,Zero);
    }
    v.copySubVector(i->second.indexStart,sub);
  }
}

void GeneralizedRobotModel::SplitRefs(const Config& q,vector<Config>& qsplit) const
{
  Assert(q.n == NumDof());
  qsplit.resize(elements.size());
  int k=0;
  for(map<int,Element>::const_iterator i=elements.begin();i!=elements.end();i++) {
    qsplit[k].setRef(q,i->second.indexStart,1,i->second.indexEnd-i->second.indexStart);
    k++;
  }
}


void GeneralizedRobotModel::Split(const Config& q,vector<Config>& qsplit) const
{
  Assert(q.n == NumDof());
  qsplit.resize(elements.size());
  int k=0;
  for(map<int,Element>::const_iterator i=elements.begin();i!=elements.end();i++) {
    
    qsplit[k].resize(i->second.indexEnd-i->second.indexStart);
    q.getSubVectorCopy(i->second.indexStart,qsplit[k]);
    k++;
  }
}

void GeneralizedRobotModel::Join(const vector<Config>& qsplit,Config& q) const
{
  int cnt = 0;
  for(size_t i=0;i<qsplit.size();i++) cnt += qsplit[i].n;
  q.resize(cnt);
  cnt = 0;
  for(size_t i=0;i<qsplit.size();i++){
    q.copySubVector(cnt,qsplit[i]);
    cnt += qsplit[i].n;
  }
}

void GeneralizedRobotModel::Interpolate(const Config& a,const Config& b,Real u,Config& out) const
{
  Assert(a.n == NumDof());
  Assert(b.n == NumDof());
  out.resize(NumDof());
  for(map<int,Element>::const_iterator i=elements.begin();i!=elements.end();i++) {
    Config asub,bsub,osub;
    asub.setRef(a,i->second.indexStart,1,i->second.indexEnd-i->second.indexStart);
    bsub.setRef(b,i->second.indexStart,1,i->second.indexEnd-i->second.indexStart);
    if(i->second.robot) {
      Klampt::Interpolate(*i->second.robot,asub,bsub,u,osub);
    }
    else {
      RigidTransform Ta,Tb,Tout;
      ConfigToTransform(asub,Ta);
      ConfigToTransform(bsub,Tb);
      interpolate(Ta,Tb,u,Tout);
      TransformToConfig(Tout,osub);
    }
    out.copySubVector(i->second.indexStart,osub);
  }
}
void GeneralizedRobotModel::InterpolateVelocity(const Config& a,const Config& b,Real u,Vector& dq) const
{
  FatalError("TODO");
}

void GeneralizedRobotModel::Integrate(const Config& q,const Vector& dq,Config& out) const
{
  FatalError("TODO");
}

Real GeneralizedRobotModel::Distance(const Config& a,const Config& b,Real floatingRotationWeight) const
{
  FatalError("TODO");
  return 0;
}

void GeneralizedRobotModel::GetJointLimits(Config& qmin,Config& qmax) const
{
  qmin.resize(NumDof());
  qmax.resize(NumDof());
  for(map<int,Element>::const_iterator i=elements.begin();i!=elements.end();i++) {
    Config submin,submax;
    if(i->second.robot) {
      submin = i->second.robot->qMin;
      submax = i->second.robot->qMax;
    }
    else {
      submin.resize(6,-Inf);
      submax.resize(6,Inf);
      submin[3] = submin[4] = submin[5] = -TwoPi;
      submax[3] = submax[4] = submax[5] = TwoPi;
    }
    qmin.copySubVector(i->second.indexStart,submin);
    qmax.copySubVector(i->second.indexStart,submax);
  }
}

Vector3 GeneralizedRobotModel::GetCOM() const
{
  Vector3 momentSum(0.0);
  Real massSum=0;
  for(map<int,Element>::const_iterator i=elements.begin();i!=elements.end();i++) {
    if(i->second.robot) {
      momentSum += i->second.robot->GetTotalMass()*i->second.robot->GetCOM();
      massSum += i->second.robot->GetTotalMass();
    }
    else {
      massSum += i->second.object->mass;
      momentSum += i->second.object->mass*(i->second.object->T*i->second.object->com);
    }
  }
  return momentSum*(1.0/massSum);
}

void GeneralizedRobotModel::GetMegaRobot(RobotModel& robot) const
{
  vector<string> prefix;
  vector<RobotModel*> robotsAndObjects;
  list<RobotModel> convertedObjects;
  for(map<int,Element>::const_iterator i=elements.begin();i!=elements.end();i++) {
    if(i->second.robot)
      robotsAndObjects.push_back(i->second.robot);
    else {
      convertedObjects.push_back(RobotModel());
      ObjectToRobot(*i->second.object,convertedObjects.back());
      robotsAndObjects.push_back(&convertedObjects.back());
    }
    if(i->second.name.empty()) {
      stringstream ss;
      ss<<"element_"<<i->first;
      prefix.push_back(ss.str());
    }
    else
      prefix.push_back(i->second.name);
  }
  robot.Merge(robotsAndObjects);
  //fix up the names
  size_t nl = 0;
  for(size_t i=0;i<robotsAndObjects.size();i++) {
    for(size_t j=0;j<robotsAndObjects[i]->links.size();j++)
      robot.linkNames[nl+j] = prefix[i]+"["+robot.linkNames[nl+j]+"]";
    nl += robotsAndObjects[i]->links.size();
  }
}

} // namespace Klampt