#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include "Modeling/Robot.h"
#include "Modeling/DynamicPath.h"

/** A unified interface to control either a simulated or real robot, which
 * operates in a motion queue mode.
 * 
 * In sim mode, currently only supports the SimViewProgram default controller.
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
  virtual MotionResult SendMilestone(const Config& x)=0;
  virtual MotionResult SendMilestoneImmediate(const Config& x)=0;
  virtual MotionResult SendPathImmediate(Real tbreak,const ParabolicRamp::DynamicPath& path)=0;
};

#endif
