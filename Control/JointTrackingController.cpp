#include "JointTrackingController.h"
#include "Modeling/DynamicPath.h"
#include <sstream>

JointTrackingController::JointTrackingController(Robot& _robot)
  :RobotController(_robot)
{
  qdesDefault = _robot.q;
}
    
//subclasses should override this
void JointTrackingController::GetDesiredState(Config& q_des,Vector& dq_des)
{
  q_des = qdesDefault;
  dq_des.setZero();
}

void JointTrackingController::Update(Real dt)
{
  Assert(command != NULL);

  Config qdes(robot.links.size()),dqdes(robot.links.size());
  GetDesiredState(qdes,dqdes);
  robot.NormalizeAngles(qdes);
  for(size_t i=0;i<robot.drivers.size();i++) {
    if(robot.drivers[i].type == RobotJointDriver::Normal) {
      command->actuators[i].SetPID(qdes(robot.drivers[i].linkIndices[0]),dqdes(robot.drivers[i].linkIndices[0]),command->actuators[i].iterm);
    }
    else {
      robot.q = qdes;
      robot.dq = dqdes;
      //printf("Desired affine driver value %g, vel %g\n",robot.GetDriverValue(i),robot.GetDriverVelocity(i));
      command->actuators[i].SetPID(robot.GetDriverValue(i),robot.GetDriverVelocity(i),command->actuators[i].iterm);
    }
  }
  RobotController::Update(dt);
}

void JointTrackingController::Reset()
{
  RobotController::Reset();
}

vector<string> JointTrackingController::Commands() const
{
  vector<string> res;
  res.push_back("set_q");
  return res;
}

bool JointTrackingController::SendCommand(const string& name,const string& str)
{
  if(name == "set_q") {
    stringstream ss(str); 
    ss>>qdesDefault;
    return true;
  }
  return false;
}

