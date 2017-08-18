#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "ControlledSimulator.h"
#include "Control/JointSensors.h"

//Set these values to 0 to get all warnings

//const static double gTorqueLimitWarningThreshold = 0;
const static double gTorqueLimitWarningThreshold = Inf;

//const static double gJointLimitWarningThreshold = 0;
const static double gJointLimitWarningThreshold = Inf;


ControlledRobotSimulator::ControlledRobotSimulator()
  :robot(NULL),oderobot(NULL),controller(NULL)
{
  controlTimeStep = 0.01;
}

void ControlledRobotSimulator::Init(Robot* _robot,ODERobot* _oderobot,RobotController* _controller)
{
  robot=_robot;
  oderobot=_oderobot;
  controller=_controller;
  oderobot->SetConfig(robot->q);
  oderobot->SetVelocities(robot->dq);
  command.actuators.resize(robot->drivers.size());
  if(controller) {
    controller->Reset();
  }
  curTime = 0;
  nextControlTime = 0;
  nextSenseTime.resize(0);
}


void ControlledRobotSimulator::GetCommandedConfig(Config& q)
{
  Assert(command.actuators.size() == robot->drivers.size());
  bool warned=false;
  for(size_t i=0;i<command.actuators.size();i++) {
    RobotJointDriver& d=robot->drivers[i];
    if(command.actuators[i].mode == ActuatorCommand::PID)
      robot->SetDriverValue(i,command.actuators[i].qdes);
    else {
      if(!warned){
        LOG4CXX_ERROR(KrisLibrary::logger(),"ControlledRobotSimulator::GetCommandedConfig: Can't get commanded config for non-PID drivers\n");
      }
      warned = true;
      robot->SetDriverValue(i,0.0);
    }
  }
  q = robot->q;
}

void ControlledRobotSimulator::GetCommandedVelocity(Config& dq)
{
  Assert(command.actuators.size() == robot->drivers.size());
  bool warned=false;
  for(size_t i=0;i<command.actuators.size();i++) {
    RobotJointDriver& d=robot->drivers[i];
    if(command.actuators[i].mode == ActuatorCommand::PID)
      robot->SetDriverVelocity(i,command.actuators[i].dqdes);
    else {
      if(!warned){
        LOG4CXX_ERROR(KrisLibrary::logger(),"ControlledRobotSimulator::GetCommandedVelocity: Can't get commanded velocity for non-PID drivers\n");
      }warned = true;
      robot->SetDriverVelocity(i,0.0);
    }
  }
  dq = robot->dq;
}

void ControlledRobotSimulator::GetSensedConfig(Config& q)
{
  JointPositionSensor* s = sensors.GetTypedSensor<JointPositionSensor>();
  if(s==NULL){
        LOG4CXX_ERROR(KrisLibrary::logger(),"ControlledRobotSimulator::GetSensedConfig: Warning, robot has no joint position sensor\n");
  }else
    q = s->q;
}

void ControlledRobotSimulator::GetSensedVelocity(Config& dq)
{
  JointVelocitySensor* s=sensors.GetTypedSensor<JointVelocitySensor>();
  if(s==NULL){
    LOG4CXX_ERROR(KrisLibrary::logger(),"ControlledRobotSimulator::GetSensedVelocity: Warning, robot has no joint velocity sensor\n");
  }else
    dq = s->dq;
}

void ControlledRobotSimulator::GetSimulatedConfig(Config& q)
{
  oderobot->GetConfig(q);
}

void ControlledRobotSimulator::GetSimulatedVelocity(Config& dq)
{
  oderobot->GetVelocities(dq);
}

void ControlledRobotSimulator::GetLinkTorques(Vector& t) const
{
  Vector tact(robot->drivers.size());
  t.resize(robot->links.size());
  GetActuatorTorques(tact);
  for(size_t i=0;i<robot->drivers.size();i++)
    switch(robot->drivers[i].type) {
      case RobotJointDriver::Affine:
      {
        for (size_t j=0;j<robot->drivers[i].linkIndices.size();j++)
          t[robot->drivers[i].linkIndices[j]] = tact[i]*robot->drivers[i].affScaling[j];
        break;
      }
      default:
      {
        for (size_t j=0;j<robot->drivers[i].linkIndices.size();j++)
          t[robot->drivers[i].linkIndices[j]] = tact[i];
      }
    }
}

void ControlledRobotSimulator::GetActuatorTorques(Vector& t) const
{
  if(t.empty()) t.resize(robot->drivers.size());
  if(t.n != (int)robot->drivers.size()) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"ControlledRobotSimulator::GetActuatorTorques: Warning, vector isn't sized to the number of drivers "<<robot->drivers.size()<<" (got "<<t.n<<")\n");
    if(t.n == (int)robot->links.size()){
      LOG4CXX_ERROR(KrisLibrary::logger(),"  (Did you mean GetLinkTorques()?\n");
    }
  }
  Assert(command.actuators.size() == robot->drivers.size());
  t.resize(command.actuators.size());
  for(size_t i=0;i<command.actuators.size();i++) {
    const RobotJointDriver& d=robot->drivers[i];
    Real q=oderobot->GetDriverValue(i);
    Real dq=oderobot->GetDriverVelocity(i);
    int link = d.linkIndices[0];
    if(q < robot->qMin(link)) {
      if(q + TwoPi >= robot->qMin(link) && q + TwoPi <= robot->qMax(link))
    q += TwoPi;
    }
    else if(q > robot->qMax(link)) {
      if(q - TwoPi <= robot->qMax(link) && q - TwoPi >= robot->qMin(link))
    q -= TwoPi;
    }
    if(q < robot->qMin(link)-gJointLimitWarningThreshold || q > robot->qMax(link)+gJointLimitWarningThreshold) {
      LOG4CXX_WARN(KrisLibrary::logger(),"Warning: joint angle "<<robot->linkNames[link].c_str());
      LOG4CXX_INFO(KrisLibrary::logger(),"q="<<RtoD(q)<<", qmin="<<RtoD(robot->qMin(link))<<", qmax="<<RtoD(robot->qMax(link)));
      //KrisLibrary::loggerWait();
    }
    const ActuatorCommand& cmd=command.actuators[i];
    switch(cmd.mode) {
    case ActuatorCommand::OFF:
      LOG4CXX_WARN(KrisLibrary::logger(),"Warning: actuator off?\n");
      t(i) = 0;
      break;
    case ActuatorCommand::TORQUE:

      //LOG4CXX_WARN(KrisLibrary::logger(),"Warning: direct torque?\n");
      if(cmd.torque < d.tmin-gTorqueLimitWarningThreshold){
    LOG4CXX_INFO(KrisLibrary::logger(),"Actuator "<<robot->LinkName(robot->drivers[i].linkIndices[0]).c_str()<<" limit exceeded: "<<cmd.torque<<" < "<<d.tmin);
      }else if(cmd.torque > d.tmax+gTorqueLimitWarningThreshold){
    LOG4CXX_INFO(KrisLibrary::logger(),"Actuator "<<robot->LinkName(robot->drivers[i].linkIndices[0]).c_str()<<" limit exceeded: "<<cmd.torque<<" > "<<d.tmax);
      }
      t(i) = Clamp(cmd.torque,d.tmin,d.tmax);
      break;
    case ActuatorCommand::PID:
      {
    //TODO: simulate low level errors in the PID loop
    Real cmdtorque = cmd.GetPIDTorque(q,dq);
    if(cmdtorque < d.tmin-gTorqueLimitWarningThreshold){
      LOG4CXX_INFO(KrisLibrary::logger(),"Actuator "<<robot->LinkName(robot->drivers[i].linkIndices[0]).c_str()<<" limit exceeded: "<<cmdtorque<<" < "<<d.tmin);
    }
    else if(cmdtorque > d.tmax+gTorqueLimitWarningThreshold){
      LOG4CXX_INFO(KrisLibrary::logger(),"Actuator "<<robot->LinkName(robot->drivers[i].linkIndices[0]).c_str()<<" limit exceeded: "<<cmdtorque<<" > "<<d.tmax);
    }
    Real td=Clamp(cmdtorque,d.tmin,d.tmax);
    //LOG4CXX_INFO(KrisLibrary::logger(),""<<i<<": Current "<<q<<","<<dq<<", desired "<<cmd.qdes<<","<<cmd.dqdes<<", torque desired "<<cmd.GetPIDTorque(q<<", clamped "<<dq)    
    t(i) = td;
    break;
      }
    case ActuatorCommand::LOCKED_VELOCITY:
      t(i) = 0;
      break;
    }
  }
}

void ControlledRobotSimulator::Step(Real dt,WorldSimulation* sim)
{
  Real endOfTimeStep = curTime + dt;

  //process sensors, which don't operate on the same loop as the controller,
  //necessarily.
  if(nextSenseTime.empty()) {
    //make sure the sensors get updated
    nextSenseTime.resize(sensors.sensors.size(),0);
  }
  for(size_t i=0;i<sensors.sensors.size();i++) {
    Real delay = 0;
    if(sensors.sensors[i]->rate == 0)
      delay = controlTimeStep;
    else
      delay = 1.0/sensors.sensors[i]->rate;
    if(delay < dt) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Sensor "<<sensors.sensors[i]->name.c_str());
      LOG4CXX_INFO(KrisLibrary::logger(),"  ... Limiting sensor rate to "<<1.0/dt);
      sensors.sensors[i]->rate = 1.0/dt;
      //todo: handle numerical errors in inversion...
      delay = dt;
    }

    if(curTime >= nextSenseTime[i]) {
      //trigger a sensing action
      sensors.sensors[i]->Simulate(this,sim);
      sensors.sensors[i]->Advance(delay);
      nextSenseTime[i] += delay;
    }
  }

  if(controller) {
    //the controller update happens less often than the PID update loop
    if(nextControlTime <= endOfTimeStep) {
      //update controller
      controller->sensors = &sensors;
      controller->command = &command;
      controller->Update(controlTimeStep);
      nextControlTime += controlTimeStep;
    }

    //get torques
    Vector t;
    GetActuatorTorques(t);
    Assert(command.actuators.size() == robot->drivers.size());
    for(size_t i=0;i<command.actuators.size();i++) {
      RobotJointDriver& d=robot->drivers[i];
      ActuatorCommand& cmd=command.actuators[i];
      if(cmd.mode == ActuatorCommand::LOCKED_VELOCITY) {
    //TODO: clamp to braking velocitiy?
    oderobot->SetDriverFixedVelocity(i,cmd.desiredVelocity,cmd.torque);
      }
      else {
	if(d.type == RobotJointDriver::Normal || d.type == RobotJointDriver::Translation || d.type == RobotJointDriver::Rotation) {
	  oderobot->AddDriverTorque(i,t(i));
	}
	else if(d.type == RobotJointDriver::Affine) {
	  //figure out how the drive mechanism affects torques on the links
	  Real q=cmd.qdes;
	  Real dq=cmd.dqdes;
	  Vector tjoints(d.linkIndices.size());
	  Vector driverBasis(d.linkIndices.size());
	  //robot joints now have desired q and dq
	  robot->SetDriverValue(i,q);
	  robot->SetDriverVelocity(i,dq);
	  //TODO: don't hard-code these!  But how to encode arbitrary drive
	  //trains?
	  Real mechStiffness = 20;
	  Real mechDamping = 0.2;
	  Real mechMaxTorque = 2;
	  for(size_t j=0;j<d.linkIndices.size();j++) {
	    int link = d.linkIndices[j];
	    driverBasis[j] = d.affScaling[j]; //todo: should be a transmission parameter in the joint driver
	    tjoints[j] = mechStiffness*(robot->q(link)-oderobot->GetLinkAngle(link)) + mechDamping*(robot->dq(link)-oderobot->GetLinkVelocity(link));
	  }
	  tjoints.madd(driverBasis,-tjoints.dot(driverBasis)/driverBasis.normSquared());
	  if(tjoints.norm() > mechMaxTorque)
	    tjoints *= mechMaxTorque/tjoints.norm();
	  //LOG4CXX_INFO(KrisLibrary::logger(),"Stabilizing torques: "<<tjoints<<"\n");
	  tjoints.madd(driverBasis,t[i]);
	  //LOG4CXX_INFO(KrisLibrary::logger(),"Torques: "<<tjoints<<"\n");
	  for(size_t j=0;j<d.linkIndices.size();j++) 
	    oderobot->AddLinkTorque(d.linkIndices[j],tjoints[j]);
	}
	else {
	  FatalError("Unknown driver type");
	}
      }
      if(cmd.mode == ActuatorCommand::PID) {
    //advance PID controller
    Real q=oderobot->GetDriverValue(i);
    cmd.IntegratePID(q,dt);
    if(cmd.kI*cmd.iterm > d.tmax) { cmd.iterm = d.tmax/cmd.kI; }
    else if(cmd.kI*cmd.iterm < d.tmin) { cmd.iterm = d.tmin/cmd.kI; }
      }
    }
  }

  curTime = endOfTimeStep;
}

void ControlledRobotSimulator::UpdateRobot()
{
  oderobot->GetConfig(robot->q);
  oderobot->GetVelocities(robot->dq);
  robot->UpdateFrames();
}

bool ControlledRobotSimulator::ReadState(File& f)
{
  if(!ReadFile(f,curTime)) return false;
  if(!ReadFile(f,nextControlTime)) return false;
  if(!ReadFile(f,command)) return false;
  if(!sensors.ReadState(f)) return false;
  if(controller) {
    if(!controller->ReadState(f)) return false;
  }
  return true;
}

bool ControlledRobotSimulator::WriteState(File& f) const
{
  if(!WriteFile(f,curTime)) return false;
  if(!WriteFile(f,nextControlTime)) return false;
  if(!WriteFile(f,command)) return false;
  if(!sensors.WriteState(f)) return false;
  if(controller) {
    if(!controller->WriteState(f)) return false;
  }
  return true;
}
