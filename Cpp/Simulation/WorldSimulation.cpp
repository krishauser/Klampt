#include "WorldSimulation.h"
#include <KrisLibrary/Timer.h>
#include <ode/ode.h>
#include "ODECommon.h"
DEFINE_LOGGER(WorldSimulator)


#define READ_FILE_DEBUG(file,object,prefix)		\
  if(!ReadFile(file,object)) { \
    LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),prefix<<": ReadFile failed to read item "<<#object); \
    return false; \
  }

#define READ_ARRAY_FILE_DEBUG(file,object,count,prefix)	\
  if(!ReadArrayFile(file,object,count)) {					\
    LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),prefix<<": ReadArrayFile failed to read item "<<#object<<", size "<<count); \
    return false; \
  }

#define TEST_READ_WRITE 0

template <class T>
bool TestReadWriteState(T& obj,const char* name="")
{
  File fwrite,fwritenew;
  fwrite.OpenData();
  if(!obj.WriteState(fwrite)) {
    LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),"WriteState "<<name<<" failed");
    return false;
  }
  //HACK for File internal buffer length bug returning buffer capacity rather
  //than size
  //int n = fwrite.Length();
  int n1 = fwrite.Position();
  fwrite.Seek(0,FILESEEKSTART);
  if(!obj.ReadState(fwrite)) {
    LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),"ReadState "<<name<<" failed");
    return false;
  }
  fwritenew.OpenData();
  if(!obj.WriteState(fwritenew)) {
    LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),"Second WriteState "<<name<<" failed");
    return false;
  }
  //HACK for File internal buffer length bug returning buffer capacity rather
  //than size
  //int n2 = fwritenew.Length();
  int n2 = fwritenew.Position();
  char* d1 = (char*)fwrite.GetDataBuffer();
  char* d2 = (char*)fwritenew.GetDataBuffer();
  if(n1 != n2) {
    LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),"WriteState "<<name<<" wrote different numbers of bytes: "<<n1<<" -> "<<n2);
    return false;
  }
  for(int i=0;i<n1;i++) {
    if(d1[i] != d2[i]) {
      LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),"WriteState "<<name<<" wrote different byte at position "<<i<<"/"<<n1);
      //fprintf(stderr,"WriteState %s wrote different byte at position %d/%d: 0x%x vs 0x%x\n",name,i,n1,(int)d1[i],(int)d2[i]);
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
    LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),"WriteFile "<<name<<" failed");
    return false;
  }
  fwrite.Seek(0);
  if(!obj.ReadFile(fwrite)) {
    LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),"ReadFile "<<name<<" failed");
    return false;
  }
  fwritenew.OpenData();
  if(!obj.WriteFile(fwritenew)) {
    LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),"Second WriteFile "<<name<<" failed");
    return false;
  }
  int n1 = fwrite.Length(), n2 = fwritenew.Length();
  char* d1 = (char*)fwrite.GetDataBuffer();
  char* d2 = (char*)fwritenew.GetDataBuffer();
  if(n1 != n2) {
    LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),"WriteFile "<<name<<" wrote different numbers of bytes: "<<n1<<" -> "<<n2);
    return false;
  }
  for(int i=0;i<n1;i++) {
    if(d1[i] != d2[i]) {
      LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),"WriteFile "<<name<<" wrote different byte at position "<<i<<"/"<<n1);
      //fprintf(stderr,"WriteFile %s wrote different byte at position %d: %c vs %c\n",name,i,d1[i],d2[i]);
      return false;
    }
  }
  return true;
}


void Reset(ContactFeedbackInfo& info)
{
  info.contactCount = 0;
  info.separationCount = 0;
  info.penetrationCount = 0;
  info.inContact = false;
  info.penetrating = false;
  info.meanForce.setZero();
  info.meanTorque.setZero();
  info.meanPoint.setZero();
  info.times.clear();
  info.contactLists.clear();
}

template <class T>
bool WriteFile(File& f,const vector<T>& v)
{
  if(!WriteFile(f,int(v.size()))) return false;
  if(!v.empty()) 
	if(!WriteArrayFile(f,&v[0],v.size())) return false;
  return true;
}


template <class T>
bool ReadFile(File& f,vector<T>& v)
{
  int n;
  READ_FILE_DEBUG(f,n,"ReadFile(vector<T>)");
  if(n < 0) {
    LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),"ReadFile(vector<T>): invalid size "<<n);
    return false;
  }
  v.resize(n);
  if(n != 0)
    READ_ARRAY_FILE_DEBUG(f,&v[0],n,"ReadFile(vector<T>)")
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
  READ_FILE_DEBUG(f,cp.x,"ReadFile(ContactPoint)");
  READ_FILE_DEBUG(f,cp.n,"ReadFile(ContactPoint)");
  READ_FILE_DEBUG(f,cp.kFriction,"ReadFile(ContactPoint)");
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
  READ_FILE_DEBUG(f,obj.type,"ReadFile(ODEObjectID)");
  READ_FILE_DEBUG(f,obj.index,"ReadFile(ODEObjectID)");
  READ_FILE_DEBUG(f,obj.bodyIndex,"ReadFile(ODEObjectID)");
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
  READ_FILE_DEBUG(f,list.o1,"ReadFile(ODEContactList)");
  READ_FILE_DEBUG(f,list.o2,"ReadFile(ODEContactList)");
  READ_FILE_DEBUG(f,list.points,"ReadFile(ODEContactList)");
  READ_FILE_DEBUG(f,list.forces,"ReadFile(ODEContactList)");
  READ_FILE_DEBUG(f,list.feedbackIndices,"ReadFile(ODEContactList)");
  return true;
}

bool WriteFile(File& f,const ContactFeedbackInfo& info)
{
  if(!WriteFile(f,info.accum)) return false;
  if(!WriteFile(f,info.inContact)) return false;
  if(!WriteFile(f,info.contactCount)) return false;
  if(!WriteFile(f,info.separationCount)) return false;
  if(!WriteFile(f,info.penetrating)) return false;
  if(!WriteFile(f,info.penetrationCount)) return false;
  if(!WriteFile(f,info.meanForce)) return false;
  if(!WriteFile(f,info.meanTorque)) return false;
  if(!WriteFile(f,info.meanPoint)) return false;
  if(!WriteFile(f,info.accumFull)) return false;
  if(!WriteFile(f,info.times)) return false;
  if(!WriteFile(f,info.contactLists)) return false;
  return true;
}

bool ReadFile(File& f,ContactFeedbackInfo& info)
{
  READ_FILE_DEBUG(f,info.accum,"ReadFile(ContactFeedbackInfo)");
  READ_FILE_DEBUG(f,info.inContact,"ReadFile(ContactFeedbackInfo)");
  READ_FILE_DEBUG(f,info.contactCount,"ReadFile(ContactFeedbackInfo)");
  READ_FILE_DEBUG(f,info.separationCount,"ReadFile(ContactFeedbackInfo)");
  READ_FILE_DEBUG(f,info.penetrating,"ReadFile(ContactFeedbackInfo)");
  READ_FILE_DEBUG(f,info.penetrationCount,"ReadFile(ContactFeedbackInfo)");
  READ_FILE_DEBUG(f,info.meanForce,"ReadFile(ContactFeedbackInfo)");
  READ_FILE_DEBUG(f,info.meanTorque,"ReadFile(ContactFeedbackInfo)");
  READ_FILE_DEBUG(f,info.meanPoint,"ReadFile(ContactFeedbackInfo)");
  READ_FILE_DEBUG(f,info.accumFull,"ReadFile(ContactFeedbackInfo)");
  READ_FILE_DEBUG(f,info.times,"ReadFile(ContactFeedbackInfo)");
  READ_FILE_DEBUG(f,info.contactLists,"ReadFile(ContactFeedbackInfo)");
  return true;
}


WorldSimulation::WorldSimulation()
  :time(0),simStep(0.001),fakeSimulation(false),worstStatus(ODESimulator::StatusNormal)
{}

void WorldSimulation::Init(RobotWorld* _world)
{
  LOG4CXX_INFO(GET_LOGGER(WorldSimulator),"Creating WorldSimulation");
  time = 0;
  world = _world;
  odesim.SetGravity(Vector3(0,0,-9.8));
  for(size_t i=0;i<world->terrains.size();i++)
    odesim.AddTerrain(*world->terrains[i]);
  for(size_t i=0;i<world->robots.size();i++)
    odesim.AddRobot(*world->robots[i]);
  for(size_t i=0;i<world->rigidObjects.size();i++) 
    odesim.AddObject(*world->rigidObjects[i]);
  controlSimulators.resize(world->robots.size());

  //setup control simulators
  for(size_t i=0;i<controlSimulators.size();i++) {
    Robot* robot=world->robots[i].get();
    RobotMotorCommand& command=controlSimulators[i].command;
    controlSimulators[i].Init(robot,odesim.robot(i),(i < robotControllers.size() ? robotControllers[i].get() : NULL));

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
  command.actuators[j].qmin = robot->qMin(k);
  command.actuators[j].qmax = robot->qMax(k);
	if(robot->links[k].type == RobotLink3D::Revolute) {
    command.actuators[j].revolute = true;
	  //ODE has problems with joint angles > 2pi
	  if(!IsFinite(robot->qMax(k)-robot->qMin(k)) || robot->qMax(k)-robot->qMin(k) >= TwoPi) {
	    command.actuators[j].measureAngleAbsolute=false;
	    LOG4CXX_INFO(GET_LOGGER(WorldSimulator),"WorldSimulation: Link "<<k<<" ("<< robot->LinkName(k).c_str()<<") can make complete turn, using relative encoding");
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
  LOG4CXX_INFO(GET_LOGGER(WorldSimulator),"Done.");
}

void WorldSimulation::OnAddModel()
{
  for(size_t i=odesim.numTerrains();i<world->terrains.size();i++)
    odesim.AddTerrain(*world->terrains[i]);
  for(size_t i=odesim.numObjects();i<world->rigidObjects.size();i++) 
    odesim.AddObject(*world->rigidObjects[i]);
  for(size_t i=odesim.numRobots();i<world->robots.size();i++) {
    odesim.AddRobot(*world->robots[i]);

    //set up control simulator
    controlSimulators.resize(i+1);
    Robot* robot=world->robots[i].get();
    RobotMotorCommand& command=controlSimulators[i].command;
    controlSimulators[i].Init(robot,odesim.robot(i),(i < robotControllers.size() ? robotControllers[i].get() : NULL));

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
	    LOG4CXX_INFO(GET_LOGGER(WorldSimulator),"WorldSimulation: Link "<<k<<" ("<<robot->LinkName(k).c_str()<<") can make complete turn, using relative encoding");
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

void WorldSimulation::SetController(int index,shared_ptr<RobotController> c)
{
  if(robotControllers.empty()) {
    robotControllers.resize(world->robots.size());
  }
  robotControllers[index] = c;
  controlSimulators[index].controller = c.get();
  if(c) {
    c->sensors = &controlSimulators[index].sensors;
    c->command = &controlSimulators[index].command;
    c->Reset();
  }
}

void WorldSimulation::Advance(Real dt)
{
  worstStatus = ODESimulator::StatusNormal;
  if(fakeSimulation) {
    AdvanceFake(dt);
    return;
  }

  if(dt == 0) {
    //just update the control simulators and hooks
    for(size_t i=0;i<controlSimulators.size();i++) 
      controlSimulators[i].Step(0,this);
    for(size_t i=0;i<hooks.size();i++)
      hooks[i]->Step(0);
    return;
  }

  for(ContactFeedbackMap::iterator i=contactFeedback.begin();i!=contactFeedback.end();i++) {
    Reset(i->second);
  }
  //Timer timer;
  Real timeLeft=dt;
  Real accumTime=0;
  int numSteps = 0;
  //printf("Advance %g -> %g, simulation time step %g\n",time,time+dt,simStep);
  while(timeLeft > 0.0) {
    Real step = Min(timeLeft,simStep);
    for(size_t i=0;i<controlSimulators.size();i++) 
      controlSimulators[i].Step(step,this);
    for(size_t i=0;i<hooks.size();i++)
      hooks[i]->Step(step);

    //update viscous friction approximation as dry friction from current velocity
    for(size_t i=0;i<controlSimulators.size();i++) {
      Robot* robot=world->robots[i].get();
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
 
    if(odesim.GetStatus() > worstStatus) {
      worstStatus = odesim.GetStatus();
    }
 
    accumTime += step;
    timeLeft -= step;
    numSteps++;

    //accumulate contact information
    for(ContactFeedbackMap::iterator i=contactFeedback.begin();i!=contactFeedback.end();i++) {
      if(i->second.accum || i->second.accumFull) {
	ODEContactList* list = odesim.GetContactFeedback(i->first.first,i->first.second);
	if(!list) continue;
	if(i->second.accum) {
	  if(list->forces.empty()) i->second.separationCount++;
	  else i->second.contactCount++;
	  i->second.inContact = !list->forces.empty();
    i->second.penetrating = list->penetrating;
    if(list->penetrating) i->second.penetrationCount++;
	  Vector3 meanPoint(Zero),meanForce(Zero),meanTorque(Zero);
	  if(!list->forces.empty()) {
	    Real wsum = 0;
	    for(size_t k=0;k<list->forces.size();k++) {
	      Real w = list->forces[k].dot(list->points[k].n);
	      meanPoint += list->points[k].x*w;
	      wsum += w;
	    }
	    if(wsum == 0) {
	      meanPoint.setZero();
	      for(size_t k=0;k<list->forces.size();k++) 
		meanPoint += list->points[k].x;
	      meanPoint /= list->forces.size();
	    }
	    else 
	      meanPoint /= wsum;
	      //update average;
	    i->second.meanPoint += 1.0/i->second.contactCount*(meanPoint - i->second.meanPoint);
	  }
	  for(size_t k=0;k<list->forces.size();k++) {
	    meanForce += list->forces[k];
	    meanTorque += cross((list->points[k].x-meanPoint),list->forces[k]);
	  }
	  //update average

	  i->second.meanForce += 1.0/numSteps*(meanForce - i->second.meanForce);
	  i->second.meanTorque += 1.0/numSteps*(meanTorque - i->second.meanTorque);
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

  //kill any autokill hooks at end of timestep
  bool anyKilled = false;
  vector<shared_ptr<WorldSimulationHook> > newhooks;
  for(size_t i=0;i<hooks.size();i++) {
    if(hooks[i]->autokill) {
      if(!anyKilled) 
        newhooks.insert(newhooks.end(),hooks.begin(),hooks.begin()+i);
      anyKilled = true;
    }
    else if(anyKilled) newhooks.push_back(hooks[i]);
  }
  if(anyKilled) {
    swap(hooks,newhooks);
  }
  /*
  //convert sums to means
  for(ContactFeedbackMap::iterator i=contactFeedback.begin();i!=contactFeedback.end();i++) {
    if(i->second.accum) {
      i->second.meanForce /= numSteps;
      i->second.meanPoint /= numSteps;
      i->second.meanTorque /= numSteps;
    }
  }
  */
  //printf("WorldSimulation: Sim step %gs, real step %gs\n",dt,timer.ElapsedTime());
}

void WorldSimulation::AdvanceFake(Real dt)
{
  bool oldFake = fakeSimulation;
  fakeSimulation = true;
  for(size_t i=0;i<controlSimulators.size();i++) 
    controlSimulators[i].Step(dt,this);
  for(size_t i=0;i<hooks.size();i++)
    hooks[i]->Step(dt);
  time += dt;
  UpdateModel();
  fakeSimulation = oldFake;

  //kill any autokill hooks at end of timestep
  bool anyKilled = false;
  vector<shared_ptr<WorldSimulationHook> > newhooks;
  for(size_t i=0;i<hooks.size();i++) {
    if(hooks[i]->autokill) {
      if(!anyKilled) 
	newhooks.insert(newhooks.end(),hooks.begin(),hooks.begin()+i);
      else 
	anyKilled = true;
    }
    else if(anyKilled) newhooks.push_back(hooks[i]);
  }
  if(anyKilled)
    swap(hooks,newhooks);
}

void WorldSimulation::UpdateModel()
{
  if(fakeSimulation) {
    for(size_t i=0;i<world->robots.size();i++) {
      Config q;
      controlSimulators[i].GetCommandedConfig(q);
      world->robots[i]->UpdateConfig(q);
      world->robots[i]->UpdateGeometry();
      odesim.robot(i)->SetConfig(q);
      odesim.robot(i)->SetVelocities(q);
    }
  }
  else {
    for(size_t i=0;i<world->robots.size();i++) {
      odesim.robot(i)->GetConfig(world->robots[i]->q);
      odesim.robot(i)->GetVelocities(world->robots[i]->dq);
      world->robots[i]->UpdateFrames();
    }
    for(size_t i=0;i<world->rigidObjects.size();i++) {
      odesim.object(i)->GetTransform(world->rigidObjects[i]->T);  
      odesim.object(i)->GetVelocity(world->rigidObjects[i]->w,world->rigidObjects[i]->v);
    }
    world->UpdateGeometry();
  }
}

void WorldSimulation::UpdateRobot(int i)
{
  if(fakeSimulation) {
    Config q;
    controlSimulators[i].GetCommandedConfig(q);
    world->robots[i]->UpdateConfig(q);
    world->robots[i]->UpdateGeometry();
    odesim.robot(i)->SetConfig(q);
  }
  else {
    odesim.robot(i)->GetConfig(world->robots[i]->q);
    world->robots[i]->UpdateFrames();
    world->robots[i]->UpdateGeometry();
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
  //TODO: read this too?
  worstStatus = ODESimulator::StatusNormal;

  READ_FILE_DEBUG(f,time,"WorldSimulation::ReadState");
  if(!odesim.ReadState(f)) {
    LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),"WorldSimulation::ReadState: ODE sim failed to read");
    return false;
  }
  //controlSimulators will read the robotControllers' states
  for(size_t i=0;i<controlSimulators.size();i++) {
    if(!controlSimulators[i].ReadState(f)) {
      LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),"WorldSimulation::ReadState: Control simulator "<<i<<" failed to read");
      return false;
    }
  }
  for(size_t i=0;i<hooks.size();i++) {
    if(!hooks[i]->ReadState(f)) {
      LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),"WorldSimulation::ReadState: Hook "<<i<<" failed to read");
      return false;
    }
  }
  int n;
  READ_FILE_DEBUG(f,n,"WorldSimulation::ReadState: reading number of contactFeadback items");
  if(n < 0) {
    LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),"Invalid number "<<n<<" of contactFeedback items");
    return false;
  }
  contactFeedback.clear();
  for(int i=0;i<n;i++) {
    pair<ODEObjectID,ODEObjectID> key;
    ContactFeedbackInfo info;
    if(!ReadFile(f,key.first)) {
      LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),"Unable to read contact feedback "<<i<<" object 1");
      return false;
    }
    if(!ReadFile(f,key.second)) {
      LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),"Unable to read contact feedback "<<i<<" object 2");
      return false;
    }
    if(!ReadFile(f,info)) {
      LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),"Unable to read contact feedback "<<i<<" info");
      return false;
    }
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
      LOG4CXX_ERROR(GET_LOGGER(WorldSimulator),"WorldSimulation::ReadState: Hook "<<i<<" failed to write");
      return false;
    }
  }
  if(!WriteFile(f,int(contactFeedback.size()))) return false;
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
  //try finding this in the feedback map
  if(bid < 0) { 
    ODEObjectID a=WorldToODEID(aid);
    for(ContactFeedbackMap::iterator i=contactFeedback.begin();i!=contactFeedback.end();i++) {
      if(i->first.first == a || i->first.second == a) {
	if(i->second.inContact) return true;
      }
    }
  }
  else {
    ContactFeedbackInfo* info=GetContactFeedback(aid,bid);
    if(info) {
      return info->inContact;
    }
  }

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
	if(i->second.contactCount>0) return true;
      }
    }
    return false;
  }
  else {
    ContactFeedbackInfo* info=GetContactFeedback(aid,bid);
    if(!info) return false;
    return (info->contactCount>0);
  }
}

bool WorldSimulation::HadSeparation(int aid,int bid)
{
  if(bid < 0) { 
    ODEObjectID a=WorldToODEID(aid);
    for(ContactFeedbackMap::iterator i=contactFeedback.begin();i!=contactFeedback.end();i++) {
      if(i->first.first == a || i->first.second == a) {
	if(i->second.separationCount>0) return true;
      }
    }
    return false;
  }
  else {
    ContactFeedbackInfo* info=GetContactFeedback(aid,bid);
    if(!info) return false;
    return (info->separationCount>0);
  }
}


bool WorldSimulation::HadPenetration(int aid,int bid)
{
  if(aid < 0) {
    for(ContactFeedbackMap::iterator i=contactFeedback.begin();i!=contactFeedback.end();i++) {
  if(i->second.penetrationCount>0) return true;
    }
    return false;
  }
  else if(bid < 0) { 
    ODEObjectID a=WorldToODEID(aid);
    for(ContactFeedbackMap::iterator i=contactFeedback.begin();i!=contactFeedback.end();i++) {
      if(i->first.first == a || i->first.second == a) {
  if(i->second.penetrationCount>0) return true;
      }
    }
    return false;
  }
  else {
    ContactFeedbackInfo* info=GetContactFeedback(aid,bid);
    if(!info) return false;
    return (info->penetrationCount>0);
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
    if(!info) return Vector3(0.0);
    if(aid<bid) return info->meanForce;
    else return info->meanForce;
  }
}

Vector3 WorldSimulation::ContactTorque(int aid,int bid)
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
	  isum += cross(c->points[j].x,c->forces[j]);
	
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
      sum += cross(c->points[i].x,c->forces[i]);
    if(a<b) return sum;
    else return -sum;
  }
}

Vector3 WorldSimulation::MeanContactTorque(int aid,int bid)
{
  ODEObjectID a=WorldToODEID(aid);
  if(bid < 0) {
    Vector3 sum(Zero);
    for(ContactFeedbackMap::iterator i=contactFeedback.begin();i!=contactFeedback.end();i++) {
      if(a == i->first.first)
	sum += i->second.meanTorque;
      else if(a == i->first.second)
	sum -= i->second.meanTorque;
    }
    return sum;
  }
  else {
    ContactFeedbackInfo* info=GetContactFeedback(aid,bid);
    if(!info) return Vector3(0.0);
    if(aid<bid) return info->meanTorque;
    else return info->meanTorque;
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
    FatalError("ODE object ID %d, %d not valid",odeid.type,odeid.index);
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
  FatalError("World ID %d not valid",id);
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


LocalForceHook::LocalForceHook(dBodyID _body,const Vector3& _localpt,const Vector3& _f)
  :body(_body),localpt(_localpt),f(_f)
{}

void LocalForceHook::Step(Real dt)
{
  dBodyAddForceAtRelPos(body,f.x,f.y,f.z,localpt.x,localpt.y,localpt.z);
}

bool LocalForceHook::ReadState(File& f)
{
  FatalError("Not implemented yet");
  return false;
}

bool LocalForceHook::WriteState(File& f) const
{
  FatalError("Not implemented yet");
  return false;
}

WrenchHook::WrenchHook(dBodyID _body,const Vector3& _f,const Vector3& _m)
  :body(_body),f(_f),m(_m)
{}

void WrenchHook::Step(Real dt)
{
  dBodyAddForce(body,f.x,f.y,f.z);
  dBodyAddTorque(body,m.x,m.y,m.z);
}

bool WrenchHook::ReadState(File& f)
{
  FatalError("Not implemented yet");
  return false;
}

bool WrenchHook::WriteState(File& f) const
{
  FatalError("Not implemented yet");
  return false;
}



SpringHook::SpringHook(dBodyID _body,const Vector3& _worldpt,const Vector3& _target,Real _kP,Real _kD)
  :body(_body),target(_target),kP(_kP),kD(_kD)
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
  f = kP*(target-wp);
  if(kD >= 0) {
  }
  //LOG4CXX_INFO(GET_LOGGER(WorldSimulator),"Target "<<target<<", world point "<<wp<<", force "<<f<<"");
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

JointForceHook::JointForceHook(ODEJoint* _joint,Real _f)
:joint(_joint),f(_f)
{}

void JointForceHook::Step(Real dt)
{
  joint->AddForce(f);
}

bool JointForceHook::ReadState(File& f)
{
  FatalError("Not implemented yet");
  return false;
}

bool JointForceHook::WriteState(File& f) const
{
  FatalError("Not implemented yet");
  return false;
}

JointSpringHook::JointSpringHook(ODEJoint* _joint,Real _target,Real _kP,Real _kD)
:joint(_joint),target(_target),kP(_kP),kD(_kD)
{}
  
void JointSpringHook::Step(Real dt)
{
  Real p = joint->GetPosition();
  Real v = joint->GetVelocity();
  joint->AddForce(kP*(target-p) - kD*v);
}

bool JointSpringHook::ReadState(File& f)
{
  FatalError("Not implemented yet");
  return false;
}

bool JointSpringHook::WriteState(File& f) const
{
  FatalError("Not implemented yet");
  return false;
}