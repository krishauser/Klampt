#include "WorldSimulation.h"
#include <Timer.h>
#include <ode/ode.h>
#include "ODECommon.h"

#define TEST_READ_WRITE 0

template <class T>
bool TestReadWriteState(T& obj,const char* name="")
{
  File fwrite,fwritenew;
  fwrite.OpenData();
  if(!obj.WriteState(fwrite)) {
    fprintf(stderr,"WriteState %s failed\n",name);
    return false;
  }
  //HACK for File internal buffer length bug returning buffer capacity rather
  //than size
  int n1 = fwrite.Position();
  fwrite.Seek(0,FILESEEKSTART);
  if(!obj.ReadState(fwrite)) {
    fprintf(stderr,"ReadState %s failed\n",name);
    return false;
  }
  fwritenew.OpenData();
  if(!obj.WriteState(fwritenew)) {
    fprintf(stderr,"Second WriteState %s failed\n",name);
    return false;
  }
  //HACK for File internal buffer length bug returning buffer capacity rather
  //than size
  //int n2 = fwritenew.Length();
  int n2 = fwritenew.Position();
  char* d1 = (char*)fwrite.GetDataBuffer();
  char* d2 = (char*)fwritenew.GetDataBuffer();
  if(n1 != n2) {
    fprintf(stderr,"WriteState %s wrote different numbers of bytes: %d -> %d\n",name,n1,n2);
    return false;
  }
  for(int i=0;i<n1;i++) {
    if(d1[i] != d2[i]) {
      fprintf(stderr,"WriteState %s wrote different byte at position %d/%d: 0x%x vs 0x%x\n",name,i,n1,(int)d1[i],(int)d2[i]);
      return false;
    }
  }
  return true;
}


template <class T>
bool TestReadWrite(T& obj,const char* name="")
{
  File fwrite,fwritenew;
  fwrite.OpenData();
  if(!obj.WriteFile(fwrite)) {
    fprintf(stderr,"WriteFile %s failed\n",name);
    return false;
  }
  fwrite.Seek(0);
  if(!obj.ReadFile(fwrite)) {
    fprintf(stderr,"ReadFile %s failed\n",name);
    return false;
  }
  fwritenew.OpenData();
  if(!obj.WriteFile(fwritenew)) {
    fprintf(stderr,"Second WriteFile %s failed\n",name);
    return false;
  }
  int n1 = fwrite.Length(), n2 = fwritenew.Length();
  char* d1 = (char*)fwrite.GetDataBuffer();
  char* d2 = (char*)fwritenew.GetDataBuffer();
  if(n1 != n2) {
    fprintf(stderr,"WriteFile %s wrote different numbers of bytes: %d -> %d\n",name,n1,n2);
    return false;
  }
  for(int i=0;i<n1;i++) {
    if(d1[i] != d2[i]) {
      fprintf(stderr,"WriteFile %s wrote different byte at position %d: %c vs %c\n",name,i,d1[i],d2[i]);
      return false;
    }
  }
  return true;
}


void Reset(ContactFeedbackInfo& info)
{
  info.hadContact = false;
  info.hadSeparation = false;
  info.meanForce.setZero();
  info.meanPoint.setZero();
  info.times.clear();
  info.contactLists.clear();
}

template <class T>
bool WriteFile(File& f,const vector<T>& v)
{
  if(!WriteFile(f,v.size())) return false;
  if(!v.empty()) 
	if(!WriteArrayFile(f,&v[0],v.size())) return false;
  return true;
}


template <class T>
bool ReadFile(File& f,vector<T>& v)
{
  size_t n;
  if(!ReadFile(f,n)) return false;
  v.resize(n);
  if(n != 0)
	if(!ReadArrayFile(f,&v[0],n)) return false;
  return true;
}

bool WriteFile(File& f,const ContactPoint& cp)
{
  if(!WriteFile(f,cp.x)) return false;
  if(!WriteFile(f,cp.n)) return false;
  if(!WriteFile(f,cp.kFriction)) return false;
  return true;
}


bool ReadFile(File& f,ContactPoint& cp)
{
  if(!ReadFile(f,cp.x)) return false;
  if(!ReadFile(f,cp.n)) return false;
  if(!ReadFile(f,cp.kFriction)) return false;
  return true;
}

bool WriteFile(File& f,const ODEObjectID& obj)
{
  if(!WriteFile(f,obj.type)) return false;
  if(!WriteFile(f,obj.index)) return false;
  if(!WriteFile(f,obj.bodyIndex)) return false;
  return true;
}

bool ReadFile(File& f,ODEObjectID& obj)
{
  if(!ReadFile(f,obj.type)) return false;
  if(!ReadFile(f,obj.index)) return false;
  if(!ReadFile(f,obj.bodyIndex)) return false;
  return true;
}

bool WriteFile(File& f,const ODEContactList& list)
{
  if(!WriteFile(f,list.o1)) return false;
  if(!WriteFile(f,list.o2)) return false;
  if(!WriteFile(f,list.points)) return false;
  if(!WriteFile(f,list.forces)) return false;
  if(!WriteFile(f,list.feedbackIndices)) return false;
  return true;
}

bool ReadFile(File& f,ODEContactList& list)
{
  if(!ReadFile(f,list.o1)) return false;
  if(!ReadFile(f,list.o2)) return false;
  if(!ReadFile(f,list.points)) return false;
  if(!ReadFile(f,list.forces)) return false;
  if(!ReadFile(f,list.feedbackIndices)) return false;
  return true;
}

bool WriteFile(File& f,const ContactFeedbackInfo& info)
{
  if(!WriteFile(f,info.accum)) return false;
  if(!WriteFile(f,info.hadContact)) return false;
  if(!WriteFile(f,info.hadSeparation)) return false;
  if(!WriteFile(f,info.meanForce)) return false;
  if(!WriteFile(f,info.meanPoint)) return false;
  if(!WriteFile(f,info.accumFull)) return false;
  if(!WriteFile(f,info.times)) return false;
  if(!WriteFile(f,info.contactLists)) return false;
  return true;
}

bool ReadFile(File& f,ContactFeedbackInfo& info)
{
  if(!ReadFile(f,info.accum)) return false;
  if(!ReadFile(f,info.hadContact)) return false;
  if(!ReadFile(f,info.hadSeparation)) return false;
  if(!ReadFile(f,info.meanForce)) return false;
  if(!ReadFile(f,info.meanPoint)) return false;
  if(!ReadFile(f,info.accumFull)) return false;
  if(!ReadFile(f,info.times)) return false;
  if(!ReadFile(f,info.contactLists)) return false;
  return true;
}


WorldSimulation::WorldSimulation()
  :time(0),simStep(0.001),fakeSimulation(false)
{}

void WorldSimulation::Init(RobotWorld* _world)
{
  time = 0;
  world = _world;
  odesim.SetGravity(Vector3(0,0,-9.8));
  for(size_t i=0;i<world->terrains.size();i++)
    odesim.AddEnvironment(*world->terrains[i].terrain);
  for(size_t i=0;i<world->robots.size();i++)
    odesim.AddRobot(*world->robots[i].robot);
  for(size_t i=0;i<world->rigidObjects.size();i++) 
    odesim.AddObject(*world->rigidObjects[i].object);
  controlSimulators.resize(world->robots.size());

  //setup control simulators
  for(size_t i=0;i<controlSimulators.size();i++) {
    Robot* robot=world->robots[i].robot;
    RobotMotorCommand& command=controlSimulators[i].command;
    controlSimulators[i].Init(robot,odesim.robot(i),(i < robotControllers.size() ? robotControllers[i] : NULL));

    //RobotController* c=controlSimulators[i].controller;

    for(size_t j=0;j<robot->drivers.size();j++) {
      //setup dry friction
      if(robot->drivers[j].dryFriction != 0) {
	for(size_t k=0;k<robot->drivers[j].linkIndices.size();k++) {
	  int l=robot->drivers[j].linkIndices[k];
	  controlSimulators[i].oderobot->SetLinkDryFriction(l,robot->drivers[j].dryFriction);
	}
      }

      //printf("Setting up servo for joint %d\n",j);
      //setup actuator parameters
      if(robot->drivers[j].type == RobotJointDriver::Normal) {
	int k=robot->drivers[j].linkIndices[0];
	if(robot->links[k].type == RobotLink3D::Revolute) {
	  //ODE has problems with joint angles > 2pi
	  if(robot->qMax(k)-robot->qMin(k) >= TwoPi) {
	    command.actuators[j].measureAngleAbsolute=false;
	    printf("WorldSimulation: Link %d (%s) can make complete turn, using relative encoding\n",k,robot->LinkName(k).c_str());
	  }
	}
      }
      command.actuators[j].mode = ActuatorCommand::PID;
      command.actuators[j].kP = robot->drivers[j].servoP;
      command.actuators[j].kD = robot->drivers[j].servoD;
      command.actuators[j].kI = robot->drivers[j].servoI;
      command.actuators[j].qdes = robot->GetDriverValue(j);
    }
  }
}

void WorldSimulation::SetController(int index,SmartPointer<RobotController> c)
{
  if(robotControllers.empty()) {
    robotControllers.resize(world->robots.size());
  }
  robotControllers[index] = c;
  controlSimulators[index].controller = c;
  if(c) c->Reset();
}

void WorldSimulation::Advance(Real dt)
{
  if(fakeSimulation) {
    AdvanceFake(dt);
    return;
  }

  for(ContactFeedbackMap::iterator i=contactFeedback.begin();i!=contactFeedback.end();i++) {
    Reset(i->second);
  }
  Timer timer;
  Real timeLeft=dt;
  Real accumTime=0;
  int numSteps = 0;
  //printf("Advance %g -> %g\n",time,time+dt);
  while(timeLeft > 0.0) {
    Real step = Min(timeLeft,simStep);
    for(size_t i=0;i<controlSimulators.size();i++) 
      controlSimulators[i].Step(step);
    for(size_t i=0;i<hooks.size();i++)
      hooks[i]->Step(step);

    //update viscous friction approximation as dry friction from current velocity
    for(size_t i=0;i<controlSimulators.size();i++) {
      Robot* robot=world->robots[i].robot;
      for(size_t j=0;j<robot->drivers.size();j++) {
	//setup viscous friction
	if(robot->drivers[j].viscousFriction != 0) {
	  Real v=controlSimulators[i].oderobot->GetDriverVelocity(j);
	  for(size_t k=0;k<robot->drivers[j].linkIndices.size();k++) {
	    int l=robot->drivers[j].linkIndices[k];
	    controlSimulators[i].oderobot->SetLinkDryFriction(l,robot->drivers[j].dryFriction+robot->drivers[j].viscousFriction*Abs(v));
	  }
	}
      }
    }

    odesim.Step(step);
    accumTime += step;
    timeLeft -= step;
    numSteps++;

    //accumulate contact information
    for(ContactFeedbackMap::iterator i=contactFeedback.begin();i!=contactFeedback.end();i++) {
      if(i->second.accum || i->second.accumFull) {
	ODEContactList* list = odesim.GetContactFeedback(i->first.first,i->first.second);
	assert(list);
	if(i->second.accum) {
	  if(list->forces.empty()) i->second.hadSeparation = true;
	  else i->second.hadContact = true;
	  for(size_t k=0;k<list->forces.size();k++) {
	    i->second.meanForce += list->forces[k];
	    i->second.meanPoint += list->points[k].x*(1.0/list->forces.size());
	  }
	}
	if(i->second.accumFull) {
	  i->second.times.push_back(time + accumTime);
	  i->second.contactLists.push_back(*list);
	}
      }
    }
  }
  time += dt;
  UpdateModel();

  //convert sums to means
  for(ContactFeedbackMap::iterator i=contactFeedback.begin();i!=contactFeedback.end();i++) {
    if(i->second.accum) {
      i->second.meanForce /= numSteps;
      i->second.meanPoint /= numSteps;
    }
  }
  //printf("WorldSimulation: Sim step %gs, real step %gs\n",dt,timer.ElapsedTime());
}

void WorldSimulation::AdvanceFake(Real dt)
{
  bool oldFake = fakeSimulation;
  fakeSimulation = true;
  for(size_t i=0;i<controlSimulators.size();i++) 
    controlSimulators[i].Step(dt);
  for(size_t i=0;i<hooks.size();i++)
    hooks[i]->Step(dt);
  time += dt;
  UpdateModel();
  fakeSimulation = oldFake;
}

void WorldSimulation::UpdateModel()
{
  if(fakeSimulation) {
    for(size_t i=0;i<world->robots.size();i++) {
      Config q;
      controlSimulators[i].GetCommandedConfig(q);
      world->robots[i].robot->UpdateConfig(q);
      world->robots[i].robot->UpdateGeometry();
      odesim.robot(i)->SetConfig(q);
    }
  }
  else {
    for(size_t i=0;i<world->robots.size();i++) {
      odesim.robot(i)->GetConfig(world->robots[i].robot->q);
      world->robots[i].robot->UpdateFrames();
    }
    for(size_t i=0;i<world->rigidObjects.size();i++) {
      odesim.object(i)->GetTransform(world->rigidObjects[i].object->T);  
    }
    world->UpdateGeometry();
  }
}

void WorldSimulation::UpdateRobot(int i)
{
  if(fakeSimulation) {
    Config q;
    controlSimulators[i].GetCommandedConfig(q);
    world->robots[i].robot->UpdateConfig(q);
    world->robots[i].robot->UpdateGeometry();
    odesim.robot(i)->SetConfig(q);
  }
  else {
    odesim.robot(i)->GetConfig(world->robots[i].robot->q);
    world->robots[i].robot->UpdateFrames();
    world->robots[i].robot->UpdateGeometry();
  }
}

bool WorldSimulation::ReadState(File& f)
{
#if TEST_READ_WRITE
  TestReadWriteState(odesim,"odesim");
  for(size_t i=0;i<controlSimulators.size();i++) 
    TestReadWriteState(controlSimulators[i],"controller");
  for(size_t i=0;i<hooks.size();i++) 
    TestReadWriteState(*hooks[i],"hook");
#endif

  if(!ReadFile(f,time)) return false;
  if(!odesim.ReadState(f)) {
    fprintf(stderr,"WorldSimulation::ReadState: ODE sim failed to read\n");
    return false;
  }
  //controlSimulators will read the robotControllers' states
  for(size_t i=0;i<controlSimulators.size();i++) {
    if(!controlSimulators[i].ReadState(f)) {
      fprintf(stderr,"WorldSimulation::ReadState: Control simulator %d failed to read\n",i);
      return false;
    }
  }
  for(size_t i=0;i<hooks.size();i++) {
    if(!hooks[i]->ReadState(f)) {
      fprintf(stderr,"WorldSimulation::ReadState: Hook %d failed to read\n",i);
      return false;
    }
  }
  size_t n;
  if(!ReadFile(f,n)) return false;
  contactFeedback.clear();
  for(size_t i=0;i<n;i++) {
    pair<ODEObjectID,ODEObjectID> key;
    ContactFeedbackInfo info;
    if(!ReadFile(f,key.first)) return false;
    if(!ReadFile(f,key.second)) return false;
    if(!ReadFile(f,info)) return false;
    contactFeedback[key] = info;
  }
  UpdateModel();
  return true;
}

bool WorldSimulation::WriteState(File& f) const
{
  if(!WriteFile(f,time)) return false;
  if(!odesim.WriteState(f)) return false;
  //controlSimulators will write the robotControllers' states
  for(size_t i=0;i<controlSimulators.size();i++) {
    if(!controlSimulators[i].WriteState(f)) return false;
  }
  for(size_t i=0;i<hooks.size();i++) {
    if(!hooks[i]->WriteState(f)) {
      fprintf(stderr,"WorldSimulation::ReadState: Hook %d failed to write\n",i);
      return false;
    }
  }
  if(!WriteFile(f,contactFeedback.size())) return false;
  for(ContactFeedbackMap::const_iterator i=contactFeedback.begin();i!=contactFeedback.end();i++) {
    if(!WriteFile(f,i->first.first)) return false;
    if(!WriteFile(f,i->first.second)) return false;
    if(!WriteFile(f,i->second)) return false;
  }
  return true;
}

bool WorldSimulation::ReadState(const string& s)
{
  File f;
  if(!f.OpenData((void*)s.c_str(),s.length(),FILEREAD)) return false;
  return ReadState(f);
}

bool WorldSimulation::WriteState(string& s) const
{
  File f;
  if(!f.OpenData()) return false;
  if(!WriteState(f)) return false;
  const char* buf = (const char*)f.GetDataBuffer();
  //HACK for File internal buffer length bug returning buffer capacity rather
  //than size
  //int len = f.Length();
  int len = f.Position();
  s.resize(len);
  for(int i=0;i<len;i++) s[i] = buf[i];
  return true;
}

void WorldSimulation::EnableContactFeedback(int aid,int bid,bool accum,bool accumFull)
{
  ContactFeedbackInfo f;
  f.accum = accum;
  f.accumFull = accumFull;
  Reset(f);
  pair<ODEObjectID,ODEObjectID> index(WorldToODEID(aid),WorldToODEID(bid));
  if(index.second < index.first) 
    swap(index.second,index.first);
  contactFeedback[index] = f;
  odesim.EnableContactFeedback(index.first,index.second);
}

ContactFeedbackInfo* WorldSimulation::GetContactFeedback(int aid,int bid)
{
  pair<ODEObjectID,ODEObjectID> index(WorldToODEID(aid),WorldToODEID(bid));
  if(index.second < index.first) 
    swap(index.second,index.first);
  if(contactFeedback.count(index)==0) return NULL;
  return &contactFeedback[index];
}

ODEContactList* WorldSimulation::GetContactList(int aid,int bid)
{
  ODEObjectID a=WorldToODEID(aid);
  ODEObjectID b=WorldToODEID(bid);
  return odesim.GetContactFeedback(a,b);
}

bool WorldSimulation::InContact(int aid,int bid)
{
  ODEObjectID a=WorldToODEID(aid);
  if(bid < 0) return odesim.InContact(a);
  else {
    ODEObjectID b=WorldToODEID(bid);
    return odesim.InContact(a,b);
  }
}

bool WorldSimulation::HadContact(int aid,int bid)
{
  if(bid < 0) { 
    ODEObjectID a=WorldToODEID(aid);
    for(ContactFeedbackMap::iterator i=contactFeedback.begin();i!=contactFeedback.end();i++) {
      if(i->first.first == a || i->first.second == a) {
	if(i->second.hadContact) return true;
      }
    }
    return false;
  }
  else {
    ContactFeedbackInfo* info=GetContactFeedback(aid,bid);
    if(!info) return false;
    return info->hadContact;
  }
}

bool WorldSimulation::HadSeparation(int aid,int bid)
{
  if(bid < 0) { 
    ODEObjectID a=WorldToODEID(aid);
    for(ContactFeedbackMap::iterator i=contactFeedback.begin();i!=contactFeedback.end();i++) {
      if(i->first.first == a || i->first.second == a) {
	if(i->second.hadSeparation) return true;
      }
    }
    return false;
  }
  else {
    ContactFeedbackInfo* info=GetContactFeedback(aid,bid);
    if(!info) return false;
    return info->hadSeparation;
  }
}

Vector3 WorldSimulation::ContactForce(int aid,int bid)
{
  ODEObjectID a=WorldToODEID(aid);
  if(bid < 0) {
    Vector3 sum(Zero);
    for(ContactFeedbackMap::iterator i=contactFeedback.begin();i!=contactFeedback.end();i++) {
      ODEContactList* c=NULL;
      if(i->first.first == a) 
	c=odesim.GetContactFeedback(a,i->first.second);
      else if(i->first.second == a)
	c=odesim.GetContactFeedback(i->first.first,a);
      if(c) {
	Vector3 isum(Zero);
	for(size_t j=0;j<c->forces.size();j++)
	  isum += c->forces[j];
	
	//add to the accumulator
	if(a == i->first.first) sum+=isum;
	else sum-=isum;
      }
    }
    return sum;
  }
  else {
    ODEObjectID b=WorldToODEID(bid);
    ODEContactList* c=odesim.GetContactFeedback(a,b);
    Vector3 sum(Zero);
    if(!c) return sum;
    for(size_t i=0;i<c->forces.size();i++)
      sum += c->forces[i];
    if(a<b) return sum;
    else return -sum;
  }
}

Vector3 WorldSimulation::MeanContactForce(int aid,int bid)
{
  ODEObjectID a=WorldToODEID(aid);
  if(bid < 0) {
    Vector3 sum(Zero);
    for(ContactFeedbackMap::iterator i=contactFeedback.begin();i!=contactFeedback.end();i++) {
      if(a == i->first.first)
	sum += i->second.meanForce;
      else if(a == i->first.second)
	sum -= i->second.meanForce;
    }
    return sum;
  }
  else {
    ContactFeedbackInfo* info=GetContactFeedback(aid,bid);
    if(aid<bid) return info->meanForce;
    else return info->meanForce;
  }
}


int WorldSimulation::ODEToWorldID(const ODEObjectID& odeid) const
{
  switch(odeid.type) {
  case 0:  //terrain
    return world->TerrainID(odeid.index);
  case 1:  //robot
    if(odeid.bodyIndex < 0)
      return world->RobotID(odeid.index);
    else
      return world->RobotLinkID(odeid.index,odeid.bodyIndex);
  case 2:  //object
    return world->RigidObjectID(odeid.index);
  default: 
    FatalError("ODE object ID %d, %d not valid\n",odeid.type,odeid.index);
    return -1;
  }
}

ODEObjectID WorldSimulation::WorldToODEID(int id) const
{
  int i=world->IsRigidObject(id);
  if(i>=0) return ODEObjectID(2,i);
  i=world->IsTerrain(id);
  if(i>=0) return ODEObjectID(0,i);
  i=world->IsRobot(id);
  if(i>=0) return ODEObjectID(1,i);
  pair<int,int> res=world->IsRobotLink(id);
  if(res.first>=0) return ODEObjectID(1,res.first,res.second);
  FatalError("World ID %d not valid\n",id);
  return ODEObjectID();
}




ForceHook::ForceHook(dBodyID _body,const Vector3& _worldpt,const Vector3& _f)
  :body(_body),worldpt(_worldpt),f(_f)
{}

void ForceHook::Step(Real dt)
{
  dBodyAddForceAtPos(body,f.x,f.y,f.z,worldpt.x,worldpt.y,worldpt.z);
}

bool ForceHook::ReadState(File& f)
{
  FatalError("Not implemented yet");
  return false;
}

bool ForceHook::WriteState(File& f) const
{
  FatalError("Not implemented yet");
  return false;
}


SpringHook::SpringHook(dBodyID _body,const Vector3& _worldpt,const Vector3& _target,Real _k)
  :body(_body),target(_target),k(_k)
{
  Matrix3 R;
  Vector3 t;
  CopyVector(t,dBodyGetPosition(body));
  CopyMatrix(R,dBodyGetRotation(body));
  R.mulTranspose(_worldpt-t,localpt);
}

void SpringHook::Step(Real dt)
{
  Matrix3 R;
  Vector3 t;
  Vector3 wp,f;
  CopyVector(t,dBodyGetPosition(body));
  CopyMatrix(R,dBodyGetRotation(body));
  wp = R*localpt+t;
  f = k*(target-wp);
  //cout<<"Target "<<target<<", world point "<<wp<<", force "<<f<<endl;
  dBodyAddForceAtPos(body,f.x,f.y,f.z,wp.x,wp.y,wp.z);
}

bool SpringHook::ReadState(File& f)
{
  FatalError("Not implemented yet");
  return false;
}

bool SpringHook::WriteState(File& f) const
{
  FatalError("Not implemented yet");
  return false;
}
