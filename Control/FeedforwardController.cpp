#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "FeedforwardController.h"
#include "JointSensors.h"
//#include "Modeling/ParabolicRamp.h"
#include <KrisLibrary/robotics/NewtonEuler.h>
#include <string>
#include <sstream>

FeedforwardController::FeedforwardController(Robot& _robot,SmartPointer<RobotController> _base)
  :RobotController(_robot),base(_base),stateEstimator(NULL),enableGravityCompensation(true),
   enableFeedforwardAcceleration(true),gravity(0,0,-9.8)
{
  if(base) 
    Assert(&robot == &base->robot);
  ZeroForces();
}

void FeedforwardController::Update(Real dt)
{
  if(!base) return;
  base->sensors = sensors;
  base->command = command;
  base->Update(dt);
  if(!enableGravityCompensation && !enableFeedforwardAcceleration) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"FF disabled"<<"\n");
    return;
  }

  if(stateEstimator) {
    stateEstimator->ReadSensors(*sensors);
    stateEstimator->UpdateModel();
  }
  else {
    if(!sensors->GetTypedSensor<JointPositionSensor>()) {
      LOG4CXX_INFO(KrisLibrary::logger(),"FeedforwardController: No joint positions, FF disabled\n");
      enableGravityCompensation = enableFeedforwardAcceleration = false;
      return;
    }
    Config& q = sensors->GetTypedSensor<JointPositionSensor>()->q;
    if(q.n != robot.q.n) {
      LOG4CXX_INFO(KrisLibrary::logger(),"FeedforwardController: joint encoders don't provide full state information, FF disabled\n");
      enableGravityCompensation = enableFeedforwardAcceleration = false;
      return;
    }    
    robot.UpdateConfig(q);
    if(!sensors->GetTypedSensor<JointVelocitySensor>()) 
      robot.dq.setZero();
    else {
      Vector& dq = sensors->GetTypedSensor<JointVelocitySensor>()->dq;
      if(dq.n != robot.dq.n) 
        robot.dq.setZero();
      else
        robot.dq = dq;
    }
  }

  Vector torques;
  SolveTorques(torques,dt);
  //LOG4CXX_INFO(KrisLibrary::logger(),"Estimated config "<<robot.q<<"\n");
  //LOG4CXX_INFO(KrisLibrary::logger(),"FF Torques: "<<torques<<"\n");
  for(size_t i=0;i<command->actuators.size();i++) {
    if(robot.drivers[i].type == RobotJointDriver::Normal) {
      command->actuators[i].torque = torques(robot.drivers[i].linkIndices[0]);
    }
    else {
      Vector J;
      robot.GetDriverJacobian(i,J);
      Real scale = 1.0/J.normSquared();
      command->actuators[i].torque=0;
      for(int j=0;j<J.n;j++)
	command->actuators[i].torque += J(j)*torques(j)*scale;
    }
  }
  if(stateEstimator) {
    stateEstimator->ReadCommand(*command);
    stateEstimator->Advance(dt);
  }
  RobotController::Update(dt);
}

void FeedforwardController::Reset()
{
  ZeroForces();
  if(base) {
    base->command = command;
    base->Reset();
    command = base->command;
  }
  if(stateEstimator) stateEstimator->Reset();
  RobotController::Reset();
}

bool FeedforwardController::ReadState(File& f)
{
  if(!RobotController::ReadState(f)) return false;
  if(base && !base->ReadState(f)) return false;
  if(!ReadFile(f,gravity)) return false;
  for(size_t i=0;i<wrenches.size();i++) {
    if(!ReadFile(f,wrenches[i].f)) return false;
    if(!ReadFile(f,wrenches[i].m)) return false;
  }
  return true;
}

bool FeedforwardController::WriteState(File& f) const
{
  if(!RobotController::WriteState(f)) return false;
  if(base && !base->WriteState(f)) return false;
  if(!WriteFile(f,gravity)) return false;
  for(size_t i=0;i<wrenches.size();i++) {
    if(!WriteFile(f,wrenches[i].f)) return false;
    if(!WriteFile(f,wrenches[i].m)) return false;
  }
  return true;
}


map<string,string> FeedforwardController::Settings() const
{
  map<string,string> res = base->Settings();
  FILL_CONTROLLER_SETTING(res,enableGravityCompensation);
  FILL_CONTROLLER_SETTING(res,enableFeedforwardAcceleration);
  FILL_CONTROLLER_SETTING(res,gravity);
  return res;
}

bool FeedforwardController::GetSetting(const string& name,string& str) const
{
  if(base->GetSetting(name,str)) return true;
  READ_CONTROLLER_SETTING(enableGravityCompensation)
  READ_CONTROLLER_SETTING(enableFeedforwardAcceleration)
  READ_CONTROLLER_SETTING(gravity)
  return false;
}

bool FeedforwardController::SetSetting(const string& name,const string& str)
{
  if(base->SetSetting(name,str)) return true;
  WRITE_CONTROLLER_SETTING(enableGravityCompensation)
  WRITE_CONTROLLER_SETTING(enableFeedforwardAcceleration)
  WRITE_CONTROLLER_SETTING(gravity)
  return false;
}


vector<string> FeedforwardController::Commands() const
{
  vector<string> res=base->Commands();
  res.push_back("add_ext_force");
  res.push_back("zero_ext_forces");
  return res;
}

bool FeedforwardController::SendCommand(const string& name,const string& str)
{
  if(base->SendCommand(name,str)) return true;
  if(name == "zero_ext_forces") {
    ZeroForces();
    return true;
  }
  else if(name == "add_ext_force") {
    int link;
    Vector3 f;
    Vector3 worldpt;
    stringstream ss(str);
    ss>>link>>f>>worldpt;
    if(!ss) return false;
    AddForce(link,f,worldpt);
    return true;
  }
  return false;
}

void FeedforwardController::SolveTorques(Vector& torques,Real dt)
{
  //assumes robot is updated from sensing
  NewtonEulerSolver ne(robot);
  if(enableGravityCompensation) ne.SetGravityWrenches(gravity);
  for(size_t i=0;i<wrenches.size();i++) {
    ne.externalWrenches[i].f += wrenches[i].f;
    ne.externalWrenches[i].m += wrenches[i].m;
    //LOG4CXX_INFO(KrisLibrary::logger(),"Total wrench "<<i<<": "<<ne.externalWrenches[i].m<<", "<<ne.externalWrenches[i].f<<"\n");
  }
  if(enableFeedforwardAcceleration) {
    Assert(dt > 0);
    Vector ddq(robot.links.size(),Zero);
    for(size_t i=0;i<command->actuators.size();i++) {
      if(robot.drivers[i].type == RobotJointDriver::Normal) {
	int link=robot.drivers[i].linkIndices[0];
	Assert(link >= 0 && link < (int)robot.links.size());
	//finite difference version
	ddq(link) = (command->actuators[i].dqdes-robot.dq(link))/dt;
	//Hacky PD-like version
	//Real kP=1.0,kD=10.0;
	//ddq(link) = kP*(command->actuators[i].qdes-robot.q(link))+kD*(command->actuators[i].dqdes-robot.dq(link));
	/*
	ParabolicRamp1D ramp;
	ramp.x0 = robot.q(link);
	ramp.x1 = command->actuators[i].qdes;
	ramp.dx0 = robot.dq(link);
	ramp.dx1 = command->actuators[i].dqdes;
	if(ramp.SolveMinTime(robot.accMax(link),robot.velMax(link))) 
	  ddq(link) = ramp.a1;
	//ddq(link) = 0;
	*/
      }
      else {
	//TODO: other types of drivers?
      }
    }
    ne.CalcTorques(ddq,torques);
    //LOG4CXX_INFO(KrisLibrary::logger(),"Desired accel: "<<ddq<<"\n");
    //LOG4CXX_INFO(KrisLibrary::logger(),"FF torques: "<<torques<<"\n");
  }
  else 
    ne.CalcResidualTorques(torques);
}


void FeedforwardController::ZeroForces()
{
  wrenches.resize(robot.links.size());
  Wrench zero;
  zero.f.setZero();
  zero.m.setZero();
  fill(wrenches.begin(),wrenches.end(),zero);
}

void FeedforwardController::AddForce(int link,const Vector3& f,const Vector3& worldpt)
{
  wrenches[link].f += f;
  wrenches[link].m += cross(f,worldpt-robot.links[link].T_World*robot.links[link].com);
}

