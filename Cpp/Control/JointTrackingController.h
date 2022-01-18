#ifndef JOINT_TRACKING_CONTROLLER_H
#define JOINT_TRACKING_CONTROLLER_H

#include "Controller.h"

namespace Klampt {

/** @brief A controller base class that reads out a desired servo position
 * and velocity using the method GetDesiredState.
 */
class JointTrackingController : public RobotController
{
 public:
  JointTrackingController(RobotModel& robot);
  virtual ~JointTrackingController() {}  
  virtual const char* Type() const { return "JointTrackingController"; }
  virtual void Update(Real dt);
  virtual void Reset();
  virtual bool ReadState(File& f) {
    if(!RobotController::ReadState(f)) return false;
    if(!qdesDefault.Read(f)) return false;
    return true;
  }
  virtual bool WriteState(File& f) const {
    if(!RobotController::WriteState(f)) return false;
    if(!qdesDefault.Write(f)) return false;
    return true;
  }

  //commands
  virtual vector<string> Commands() const;
  virtual bool SendCommand(const string& name,const string& str);

  ///subclasses should override this
  virtual void GetDesiredState(Config& q_des,Vector& dq_des);

  Config qdesDefault;
};

} //namespace Klampt

#endif
