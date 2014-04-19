#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include "Modeling/Robot.h"
#include "Modeling/DynamicPath.h"

/** A unified interface to control either a simulated or real robot, which
 * operates in a motion queue mode.
 * 
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

  virtual ~MotionQueueInterface() {}
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

#endif
