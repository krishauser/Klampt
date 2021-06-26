#include "JointTrackingController.h"
#include "Modeling/DynamicPath.h"
#include <sstream>

using namespace Klampt;

JointTrackingController::JointTrackingController(RobotModel& _robot)
  :RobotController(_robot)
{
}
    
//subclasses should override this
void JointTrackingController::GetDesiredState(Config& q_des,Vector& dq_des)
{
  q_des = qdesDefault;
  dq_des.setZero();
}

void JointTrackingController::Update(Real dt)
{
  if(qdesDefault.n == 0) {
    if(!GetSensedConfig(qdesDefault))
      return;
  }
  Assert(command != NULL);

  Config qdes(robot.links.size()),dqdes(robot.links.size());
  GetDesiredState(qdes,dqdes);
  SetPIDCommand(qdes,dqdes);
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

