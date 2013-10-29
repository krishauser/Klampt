#ifndef REAL_TIME_PLANNER_H
#define REAL_TIME_PLANNER_H

#include "RobotCSpace.h"
#include "PlannerSettings.h"
#include "RampCSpace.h"
#include "PlannerObjective.h"
#include "Modeling/DynamicPath.h"
#include <utils/StatCollector.h>
#include <Timer.h>

/** @ingroup Planning
 * @brief A base class for a real-time planner.
 * Supports constant or adaptive-time-stepping
 */
class RealTimePlannerBase
{
public:
  enum { Failure=0, Success=1, Timeout=2 };

  RealTimePlannerBase();
  virtual ~RealTimePlannerBase();
  //takes ownership of the pointer
  virtual void Reset(PlannerObjectiveBase* newgoal);

  //default implementation: fix the split time, call PlanFrom
  //returns true if the path changed and planTime < splitTime
  virtual bool PlanUpdate(Real& splitTime,Real& planTime);

  //basic subclasses should override this.  Plan from the start of path
  //'path' (advanced from current config by time 'cutoff'), returns the
  //result in 'path'.
  //Returns Success if successful, Failure if planning fails.
  //Subroutines should stop and return Timeout if a timer exceeds cutoff.
  virtual int PlanFrom(ParabolicRamp::DynamicPath& path,Real cutoff)
  {
    return Failure;
  }

  //performs shortcutting up until the time limit
  int Shortcut(ParabolicRamp::DynamicPath& path,Real timeLimit) const;
  //performs shortcuts that reduce the objective function
  int SmartShortcut(Real tstart,ParabolicRamp::DynamicPath& path,Real timeLimit) const;

  bool GetMilestoneRamp(const Config& q0,const Vector& dq0,const Config& q1,ParabolicRamp::DynamicPath& ramp) const;
  bool GetMilestoneRamp(const ParabolicRamp::DynamicPath& curPath,const Config& q,ParabolicRamp::DynamicPath& ramp) const;

  //returns true if the ramp from the current config to q is collision free
  bool CheckMilestoneRamp(const ParabolicRamp::DynamicPath& curPath,const Config& q,ParabolicRamp::DynamicPath& ramp) const;

  //returns the cost of going straight to q (assuming no collision detection)
  Real EvaluateDestinationCost(const Config& q) const;

  //returns the cost of using the given path
  Real EvaluatePathCost(const ParabolicRamp::DynamicPath& path,Real tStart=0.0) const;

  //returns the current final destination
  const ParabolicRamp::Vector& CurrentDestination() const;

  //returns the cost for the current destination
  Real CurrentDestinationCost() const;

  //returns the cost for the current path
  Real CurrentPathCost() const;

  void SetConstantPath(const Config& q);

  //indicates that the previously planned path had too short of a padding
  void MarkLastFailure();

  Robot* robot;
  WorldPlannerSettings* settings;
  SingleRobotCSpace* cspace;
  ParabolicRamp::DynamicPath currentPath;
  PlannerObjectiveBase* goal;

  //use only in simulation: multiplies the effective computational power
  //of this machine (i.e., simulate a machine that's x times faster)
  Real cognitiveMultiplier;

  enum SplitUpdateProtocol { Constant, ExponentialBackoff, Learning };
  SplitUpdateProtocol protocol;
  Real currentSplitTime,currentPadding,currentExternalPadding;
  StatCollector planFailTimeStats,planSuccessTimeStats,planTimeoutTimeStats;
};

#endif
