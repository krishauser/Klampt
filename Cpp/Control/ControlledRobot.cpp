#include "ControlledRobot.h"
#include "Sensing/JointSensors.h"

using namespace Klampt;

ControlledRobot::ControlledRobot()
  :klamptRobotModel(NULL),klamptController(NULL)
{}

bool ControlledRobot::Init(RobotModel* _robot,RobotController* _controller)
{
  klamptRobotModel=_robot;
  klamptController=_controller;
  command.actuators.resize(_robot->drivers.size());
  if(klamptController) {
    klamptController->Reset();  
  }
  return true;
}

void ControlledRobot::Step(Real dt)
{
  ReadSensorData(sensors);
  if(klamptController) {
    klamptController->sensors = &sensors;
    klamptController->command = &command;
    klamptController->Update(dt);
  }
  WriteCommandData(command);
}

void ControlledRobot::GetCommandedConfig(Config& q) 
{
  Assert(command.actuators.size() == klamptRobotModel->drivers.size());
  for(size_t i=0;i<command.actuators.size();i++) {
    if(command.actuators[i].mode == ActuatorCommand::PID)
      klamptRobotModel->SetDriverValue(i,command.actuators[i].qdes);
    else
      FatalError("Can't get commanded config for non-config drivers");
  }
  q = klamptRobotModel->q;
}

void ControlledRobot::GetCommandedVelocity(Config& dq)
{ 
  Assert(command.actuators.size() == klamptRobotModel->drivers.size());
  for(size_t i=0;i<command.actuators.size();i++) {
    if(command.actuators[i].mode == ActuatorCommand::PID)
      klamptRobotModel->SetDriverVelocity(i,command.actuators[i].dqdes);
    else
      FatalError("Can't get commanded config for non-config drivers");
  }
  dq = klamptRobotModel->dq;
}

void ControlledRobot::GetSensedConfig(Config& q)
{
  JointPositionSensor* s = sensors.GetTypedSensor<JointPositionSensor>();
  if(s==NULL) 
    fprintf(stderr,"Warning, robot has no joint position sensor\n");
  else
    q = s->q;
}

void ControlledRobot::GetSensedVelocity(Config& dq)
{
  JointVelocitySensor* s=sensors.GetTypedSensor<JointVelocitySensor>();
  if(s==NULL)
    fprintf(stderr,"Warning, robot has no joint velocity sensor\n");
  else
    dq = s->dq;
}
