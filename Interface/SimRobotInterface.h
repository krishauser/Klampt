#ifndef SIM_ROBOT_INTERFACE_H
#define SIM_ROBOT_INTERFACE_H

#include "RobotInterface.h"
#include "Simulation/WorldSimulation.h"

/** @brief A simulation mode RobotInterface.
 * Currently only supports the SimViewProgram default controller.
 */
class SimRobotInterface : public MotionQueueInterface
{
 public:
  WorldSimulation* sim;
  int index;

  SimRobotInterface(WorldSimulation* sim,int index=0);
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

#endif
