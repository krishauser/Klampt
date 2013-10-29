#ifndef FEEDFORWARD_CONTROLLER_H
#define FEEDFORWARD_CONTROLLER_H

#include "Controller.h"
#include "StateEstimator.h"
#include <robotics/Wrench.h>
#include <utils/SmartPointer.h>

/** @ingroup Control
 * @brief A class that adds a feedforward torque to the basic
 * control.  The necessary feedforward torque is estimated assuming the
 * robot is fixed-base.
 */
class FeedforwardController : public RobotController
{
 public:
  FeedforwardController(Robot& robot,SmartPointer<RobotController> base=NULL);
  virtual ~FeedforwardController() {}
  virtual const char* Type() const { return "FeedforwardController"; }
  virtual void Update(Real dt);
  virtual void Reset();
  virtual bool ReadState(File& f);
  virtual bool WriteState(File& f) const;

  //getters/setters
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  //commands
  void ZeroForces();
  void AddForce(int link,const Vector3& f,const Vector3& worldpt);
  virtual vector<string> Commands() const;
  virtual bool SendCommand(const string& name,const string& str);

  //helpers
  void SolveTorques(Vector& torques,Real dt);

  SmartPointer<RobotController> base;
  SmartPointer<RobotStateEstimator> stateEstimator;
  bool enableGravityCompensation,enableFeedforwardAcceleration;
  Vector3 gravity;
  //external forces and moments about the link origin
  vector<Wrench> wrenches;
};


#endif
