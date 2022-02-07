#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include <Klampt/Modeling/Robot.h>
#include <Klampt/Modeling/DynamicPath.h>
#include <Klampt/Control/PathController.h>

namespace Klampt {

/** A unified interface to control either a simulated or real robot, which
 * operates in a motion queue mode.
 * 
 * Subclasses must overload the following methods:
 * - HadExternalChange returns true if the queue changed for reasons outside
 *   of the caller's control.
 * - GetCurTime returns the current absolute time
 * - GetEndTime returns the absolute time of the end of the current motion 
 *   queue
 * - GetCurConfig/Velocity returns the current configuration/velocity
 * - GetEndConfig/Velocity returns the configuration/velocity at the end
 *   of the motion queue.
 * - GetConfig returns the configuration at some time >= current time.  If
 *   unavailable, x should be set to an empty config.
 * - SendMilestone appends a milestone to the back of the queue
 * - SendMilestoneImmediate breaks the motion queue at the current time and
 *   appends a milestone.
 * - SendPathImmediate breaks the current motion queue at absolute time tbreak.
 */
class MotionQueueInterface
{
 public:
  enum MotionResult { Success, InvalidParams, FailedCheck, TransmitError };

  MotionQueueInterface();
  virtual ~MotionQueueInterface() {}
  virtual bool HadExternalChange()=0;
  virtual Real GetCurTime()=0;
  virtual void GetCurConfig(Config& x)=0;
  virtual void GetCurVelocity(Config& dx)=0;
  virtual Real GetEndTime()=0;
  virtual void GetEndConfig(Config& x)=0;
  virtual void GetEndVelocity(Config& dx)=0;
  virtual void GetConfig(Real t,Config& x)=0;
  virtual MotionResult SendMilestone(const Config& x)=0;
  virtual MotionResult SendMilestoneImmediate(const Config& x)=0;
  virtual MotionResult SendPathImmediate(Real tbreak,const ParabolicRamp::DynamicPath& path)=0;
};


/** @brief A MotionQueueInterface that just sends to a
 * PolynomialMotionQueue.
 */
class DefaultMotionQueueInterface : public MotionQueueInterface
{
 public:
  PolynomialMotionQueue* queue;

  DefaultMotionQueueInterface(PolynomialMotionQueue* queue);
  virtual bool HadExternalChange();
  virtual Real GetCurTime();
  virtual void GetCurConfig(Config& x);
  virtual void GetCurVelocity(Config& dx);
  virtual Real GetEndTime();
  virtual void GetEndConfig(Config& x);
  virtual void GetEndVelocity(Config& dx);
  virtual void GetConfig(Real t,Config& x);
  virtual MotionResult SendMilestone(const Config& x);
  virtual MotionResult SendMilestoneImmediate(const Config& x);
  virtual MotionResult SendPathImmediate(Real tbreak,const ParabolicRamp::DynamicPath& path);
};

} //namespace Klampt

#endif
