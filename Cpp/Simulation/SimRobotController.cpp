#include "SimRobotController.h"
#include "Sensing/JointSensors.h"
#include "Sensing/Common_Internal.h"
#include <KrisLibrary/math/angle.h>
DEFINE_LOGGER(SimRobotController)

namespace Klampt {

//Set these values to 0 to get all warnings

//const static double gTorqueLimitWarningThreshold = 0;
const static double gTorqueLimitWarningThreshold = Inf;

//const static double gJointLimitWarningThreshold = 0;
const static double gJointLimitWarningThreshold = Inf;


SimRobotController::SimRobotController()
  :robot(NULL),oderobot(NULL),controller(NULL)
{
  controlTimeStep = 0.01;
}

void SimRobotController::Init(RobotModel* _robot,ODERobot* _oderobot,RobotController* _controller)
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


void SimRobotController::GetCommandedConfig(Config& q)
{
  Assert(command.actuators.size() == robot->drivers.size());
  robot->q.set(0.0);
  bool warned=false;
  for(size_t i=0;i<command.actuators.size();i++) {
    RobotModelDriver& d=robot->drivers[i];
    if(command.actuators[i].mode == ActuatorCommand::PID)
      robot->SetDriverValue(i,command.actuators[i].qdes);
    else {
      if(!warned)
        LOG4CXX_ERROR(GET_LOGGER(SimRobotController),"SimRobotController::GetCommandedConfig: Can't get commanded config for non-PID drivers");
      warned = true;
      //robot->SetDriverValue(i,0.0);
    }
  }
  q = robot->q;
}

void SimRobotController::GetCommandedVelocity(Config& dq)
{
  Assert(command.actuators.size() == robot->drivers.size());
  robot->dq.set(0.0);
  bool warned=false;
  for(size_t i=0;i<command.actuators.size();i++) {
    RobotModelDriver& d=robot->drivers[i];
    if(command.actuators[i].mode == ActuatorCommand::PID)
      robot->SetDriverVelocity(i,command.actuators[i].dqdes);
    else {
      if(!warned){
        LOG4CXX_ERROR(GET_LOGGER(SimRobotController),"SimRobotController::GetCommandedVelocity: Can't get commanded velocity for non-PID drivers");
      }
      warned = true;
      //robot->SetDriverVelocity(i,0.0);
    }
  }
  dq = robot->dq;
}

void SimRobotController::GetSensedConfig(Config& q)
{
  JointPositionSensor* s = sensors.GetTypedSensor<JointPositionSensor>();
  if(s==NULL){
        LOG4CXX_ERROR(GET_LOGGER(SimRobotController),"SimRobotController::GetSensedConfig: Warning, robot has no joint position sensor");
  }
  else {
    //resize to the correct size
    if(s->indices.empty() || s->q.empty())
      q = s->q;
    else {
      q.resize(robot->q.n);
      q.set(0.0);
      for(size_t i=0;i<s->indices.size();i++)
        q[s->indices[i]] = s->q[i];
    }
  }
}

void SimRobotController::GetSensedVelocity(Config& dq)
{
  JointVelocitySensor* s=sensors.GetTypedSensor<JointVelocitySensor>();
  if(s==NULL){
    LOG4CXX_ERROR(GET_LOGGER(SimRobotController),"SimRobotController::GetSensedVelocity: Warning, robot has no joint velocity sensor");
  }
  else {
    //resize to the correct size
    if(s->indices.empty() || s->dq.empty())
      dq = s->dq;
    else {
      dq.resize(robot->dq.n);
      dq.set(0.0);
      for(size_t i=0;i<s->indices.size();i++)
        dq[s->indices[i]] = s->dq[i];
    }
  }
}

void SimRobotController::GetSimulatedConfig(Config& q)
{
  oderobot->GetConfig(q);
}

void SimRobotController::GetSimulatedVelocity(Config& dq)
{
  oderobot->GetVelocities(dq);
}

void SimRobotController::GetLinkTorques(Vector& t) const
{
  Vector tact(robot->drivers.size());
  t.resize(robot->links.size());
  GetActuatorTorques(tact);
  for(size_t i=0;i<robot->drivers.size();i++)
    switch(robot->drivers[i].type) {
      case RobotModelDriver::Affine:
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

void SimRobotController::GetActuatorTorques(Vector& t) const
{
  if(t.empty()) t.resize(robot->drivers.size());
  if(t.n != (int)robot->drivers.size()) {
    LOG4CXX_WARN(GET_LOGGER(SimRobotController),"SimRobotController::GetActuatorTorques: Warning, vector isn't sized to the number of drivers "<<robot->drivers.size()<<" (got "<<t.n);
    if(t.n == (int)robot->links.size())
      LOG4CXX_WARN(GET_LOGGER(SimRobotController),"  (Did you mean GetLinkTorques()?");
  }
  Assert(command.actuators.size() == robot->drivers.size());
  t.resize(command.actuators.size());
  for(size_t i=0;i<command.actuators.size();i++) {
    const RobotModelDriver& d=robot->drivers[i];
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
      LOG4CXX_WARN(GET_LOGGER(SimRobotController),"Warning: joint angle "<<robot->linkNames[link].c_str()<<" out of bounds");
      LOG4CXX_WARN(GET_LOGGER(SimRobotController),"(q="<<RtoD(q)<<", qmin="<<RtoD(robot->qMin(link))<<", qmax="<<RtoD(robot->qMax(link))<<" (deg)");
      //getchar();
    }
    const ActuatorCommand& cmd=command.actuators[i];
    switch(cmd.mode) {
    case ActuatorCommand::OFF:
      LOG4CXX_WARN(GET_LOGGER(SimRobotController),"Warning: actuator off?");
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

void SimRobotController::Step(Real dt,Simulator* sim)
{
  Real endOfTimeStep = curTime + dt;

  //process sensors, which don't operate on the same loop as the controller,
  //necessarily.
  if(nextSenseTime.empty()) {
    //make sure the sensors get updated
    nextSenseTime.resize(sensors.sensors.size(),curTime);
  }
  for(size_t i=0;i<sensors.sensors.size();i++) {
    if(!sensors.sensors[i]->enabled)
      continue;
      
    Real delay = 0;
    if(sensors.sensors[i]->rate == 0)
      delay = controlTimeStep;
    else
      delay = 1.0/sensors.sensors[i]->rate;
    if(delay < dt) {
      LOG4CXX_WARN(GET_LOGGER(SimRobotController),"Sensor "<<sensors.sensors[i]->name<<" set to rate higher than internal simulation time step, limiting to "<<1.0/dt);
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
      RobotModelDriver& d=robot->drivers[i];
      ActuatorCommand& cmd=command.actuators[i];
      if(cmd.mode == ActuatorCommand::LOCKED_VELOCITY) {
        //TODO: clamp to braking velocitiy?
        oderobot->SetDriverFixedVelocity(i,cmd.desiredVelocity,cmd.torque);
      }
      else {
	if(d.type == RobotModelDriver::Normal || d.type == RobotModelDriver::Translation || d.type == RobotModelDriver::Rotation) {
	  oderobot->AddDriverTorque(i,t(i));
	}
	else if(d.type == RobotModelDriver::Affine) {
	  //figure out how the drive mechanism affects torques on the links
	  Real q=cmd.qdes;
	  Real dq=cmd.dqdes;
	  Vector tjoints(d.linkIndices.size());
	  Vector driverBasis(d.linkIndices.size());
	  //robot joints now have desired q and dq
	  robot->SetDriverValue(i,q);
	  robot->SetDriverVelocity(i,dq);
	  //TODO: don't hard-code these!  But how to encode arbitrarily geared / tendon-driven transmissions?
	  Real mechStiffness = 0.25*d.servoP;
	  Real mechDamping = 0.001*d.servoP;
          Real mechMaxTorque = 0;
          //printf("Affine joint errors: ");
	  for(size_t j=0;j<d.linkIndices.size();j++) {
	    int link = d.linkIndices[j];
            mechMaxTorque = Max(mechMaxTorque,robot->torqueMax(link)*10);  //10x stronger mechanism than the motor's torque limit?
	    driverBasis[j] = d.affScaling[j]; //todo: should be a transmission parameter in the joint driver
            Real q = oderobot->GetLinkAngle(link);
            Real qdes = robot->q(link);
            Real deltaq = qdes-q;
            if(Abs(deltaq) > Pi && robot->links[link].type == RobotLink3D::Revolute)
                deltaq = AngleDiff(qdes,q);
	    //printf("%f ",deltaq);
            tjoints[j] = mechStiffness*deltaq + mechDamping*(robot->dq(link)-oderobot->GetLinkVelocity(link));
	  }
          //printf("\n");
          //printf("Affine mechanism %d stiffness %f damping %f max %f\n",i,mechStiffness,mechDamping,mechMaxTorque);
          
          tjoints.madd(driverBasis,-tjoints.dot(driverBasis)/driverBasis.normSquared());
	  if(tjoints.norm() > mechMaxTorque)
	    tjoints *= mechMaxTorque/tjoints.norm();
          //cout<<"Affine driver corrective torques "<<tjoints<<endl;
	  tjoints.madd(driverBasis,t[i]);
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

void SimRobotController::UpdateRobot()
{
  oderobot->GetConfig(robot->q);
  oderobot->GetVelocities(robot->dq);
  robot->UpdateFrames();
}

bool SimRobotController::ReadState(File& f)
{
  if(!ReadFile(f,curTime)) {
    LOG4CXX_ERROR(GET_LOGGER(SimRobotController),"SimRobotController::ReadState: Unable to read curTime");
    return false;
  }
  if(!ReadFile(f,nextControlTime)) {
    LOG4CXX_ERROR(GET_LOGGER(SimRobotController),"SimRobotController::ReadState: Unable to read nextControlTime");
    return false;
  }
  if(!ReadFile(f,command)) {
    LOG4CXX_ERROR(GET_LOGGER(SimRobotController),"SimRobotController::ReadState: Unable to read command");
    return false;
  }
  if(!sensors.ReadState(f)) {
    LOG4CXX_ERROR(GET_LOGGER(SimRobotController),"SimRobotController::ReadState: Unable to read sensors");
    return false;
  }
  if(controller) {
    File cfile;
    if(!ReadFile(f,cfile)) {
      LOG4CXX_ERROR(GET_LOGGER(SimRobotController),"Unable to read controller file");
      return false;
    }
    if(!controller->ReadState(cfile)) {
      LOG4CXX_ERROR(GET_LOGGER(SimRobotController),"Unable to read controller");
      return false;
    }
  }
  return true;
}

bool SimRobotController::WriteState(File& f) const
{
  if(!WriteFile(f,curTime)) return false;
  if(!WriteFile(f,nextControlTime)) return false;
  if(!WriteFile(f,command)) return false;
  if(!sensors.WriteState(f)) return false;
  if(controller) {
    File cfile; cfile.OpenData();
    if(!controller->WriteState(cfile)) return false;
    if(!WriteFile(f,cfile)) return false;
  }
  return true;
}

} // namespace Klampt