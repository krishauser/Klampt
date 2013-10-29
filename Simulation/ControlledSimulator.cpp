#include "ControlledSimulator.h"

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
}

void ControlledRobotSimulator::SimulateSensors()
{
  if(!controller) return;

  for(size_t i=0;i<sensors.sensors.size();i++) {
    sensors.sensors[i]->Simulate(this);
    sensors.sensors[i]->Advance(controlTimeStep);
  }
}

void ControlledRobotSimulator::GetCommandedConfig(Config& q) 
{
  Assert(command.actuators.size() == robot->drivers.size());
  for(size_t i=0;i<command.actuators.size();i++) {
    RobotJointDriver& d=robot->drivers[i];
    if(command.actuators[i].mode == ActuatorCommand::PID)
      robot->SetDriverValue(i,command.actuators[i].qdes);
    else
      FatalError("Can't get commanded config for non-config drivers");
  }
  q = robot->q;
}

void ControlledRobotSimulator::GetCommandedVelocity(Config& dq)
{ 
  Assert(command.actuators.size() == robot->drivers.size());
  for(size_t i=0;i<command.actuators.size();i++) {
    RobotJointDriver& d=robot->drivers[i];
    if(command.actuators[i].mode == ActuatorCommand::PID)
      robot->SetDriverValue(i,command.actuators[i].dqdes);
    else
      FatalError("Can't get commanded config for non-config drivers");
  }
  dq = robot->dq;
}

void ControlledRobotSimulator::GetSensedConfig(Config& q)
{
  JointPositionSensor* s = sensors.GetTypedSensor<JointPositionSensor>();
  if(s==NULL) 
    fprintf(stderr,"Warning, robot has no joint position sensor\n");
  else
    q = s->q;
}

void ControlledRobotSimulator::GetSensedVelocity(Config& dq)
{
  JointVelocitySensor* s=sensors.GetTypedSensor<JointVelocitySensor>();
  if(s==NULL)
    fprintf(stderr,"Warning, robot has no joint velocity sensor\n");
  else
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

void ControlledRobotSimulator::GetActuatorTorques(Vector& t) const
{
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
      printf("Warning: joint angle %s out of bounds\n",robot->linkNames[link].c_str());
      printf("q=%g, qmin=%g, qmax=%g (deg)\n",RtoD(q),RtoD(robot->qMin(link)),RtoD(robot->qMax(link)));
      //getchar();
    }
    const ActuatorCommand& cmd=command.actuators[i];
    switch(cmd.mode) {
    case ActuatorCommand::OFF:
      printf("Warning: actuator off?\n");
      t(i) = 0;
      break;
    case ActuatorCommand::TORQUE:
      //printf("Warning: direct torque?\n");
      if(cmd.torque < d.tmin-gTorqueLimitWarningThreshold) 
	printf("Actuator %s limit exceeded: %g < %g\n",robot->LinkName(robot->drivers[i].linkIndices[0]).c_str(),cmd.torque,d.tmin);
      else if(cmd.torque > d.tmax+gTorqueLimitWarningThreshold) 
	printf("Actuator %s limit exceeded: %g > %g\n",robot->LinkName(robot->drivers[i].linkIndices[0]).c_str(),cmd.torque,d.tmax);
      t(i) = Clamp(cmd.torque,d.tmin,d.tmax);
      break;
    case ActuatorCommand::PID:
      {
	//TODO: simulate low level errors in the PID loop
	Real cmdtorque = cmd.GetPIDTorque(q,dq);
	if(cmdtorque < d.tmin-gTorqueLimitWarningThreshold) 
	  printf("Actuator %s limit exceeded: %g < %g\n",robot->LinkName(robot->drivers[i].linkIndices[0]).c_str(),cmdtorque,d.tmin);
	else if(cmdtorque > d.tmax+gTorqueLimitWarningThreshold) 
	  printf("Actuator %s limit exceeded: %g > %g\n",robot->LinkName(robot->drivers[i].linkIndices[0]).c_str(),cmdtorque,d.tmax);
	Real td=Clamp(cmdtorque,d.tmin,d.tmax);
	//printf("%d: Current %g,%g, desired %g,%g, torque desired %g, clamped %g\n",i,q,dq,cmd.qdes,cmd.dqdes,cmd.GetPIDTorque(q,dq),td);
	t(i) = td;
	break;
      }
    case ActuatorCommand::LOCKED_VELOCITY:
      t(i) = 0;
      break;
    }
  }
}

void ControlledRobotSimulator::Step(Real dt)
{
  Real endOfTimeStep = curTime + dt;
 
  if(controller) {
    //the controller update happens less often than the PID update loop
    if(nextControlTime < endOfTimeStep) {
      //simulate sensors
      SimulateSensors();
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
	  Real q=cmd.qdes;
	  Real dq=cmd.dqdes;
	  Vector tjoints(d.linkIndices.size());
	  Vector driverBasis(d.linkIndices.size());
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
	  //cout<<"Stabilizing torques: "<<tjoints<<endl;
	  tjoints.madd(driverBasis,t[i]);
	  //cout<<"Torques: "<<tjoints<<endl;
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
