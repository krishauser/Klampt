#ifndef FEEDFORWARD_CONTROLLER_H
#define FEEDFORWARD_CONTROLLER_H

#include "Controller.h"
#include <Klampt/Sensing/StateEstimator.h>
#include <KrisLibrary/robotics/Wrench.h>

namespace Klampt {

/** @ingroup Control
 * @brief A class that adds a feedforward torque to the basic
 * control.  The necessary feedforward torque is estimated assuming the
 * robot is fixed-base.
 *
 * Settings:
 * - enableGravityCompensation (boolean, default true): true if gravity
 *   compensation should be turned on.
 * - enableFeedforwardAcceleration (boolean, default true): true if feedforward
 *   estimation of the inertial and coriolis terms, based on the finite
 *   differencing of the sensed velocity, should be turned on.
 * - gravity (Vector3, default (0,0,-9.8)) the gravity vector estimate for
 *   gravity compensation.
 */
class FeedforwardController : public RobotController
{
 public:
  FeedforwardController(RobotModel& robot,shared_ptr<RobotController> base);
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

  shared_ptr<RobotController> base;
  shared_ptr<RobotStateEstimator> stateEstimator;
  bool enableGravityCompensation,enableFeedforwardAcceleration;
  Vector3 gravity;
  //external forces and moments about the link origin
  vector<Wrench> wrenches;
};

} // namespace Klampt

#endif
