#ifndef REAL_TIME_IK_PLANNER_H
#define REAL_TIME_IK_PLANNER_H

#include "RealTimePlanner.h"
#include <robotics/IK.h>

/** @brief A planner that uses numerical inverse kinematics to reach the
 * goal.  The goal must be of CartesianObjective or IKObjective type, or
 * a composite of several such objectives. (see PlannerObjectives.h)
 */
class RealTimeIKPlanner : public RealTimePlannerBase
{
public:
  virtual void Reset(PlannerObjectiveBase* newgoal);
  virtual int PlanFrom(ParabolicRamp::DynamicPath& path,Real cutoff);
};

/** @brief A planner that perturbs the current configuration to get an
 * improved path.
 */
class RealTimePerturbationPlanner : public RealTimePlannerBase
{
public:
  RealTimePerturbationPlanner();
  virtual void Reset(PlannerObjectiveBase* newgoal);
  virtual int PlanFrom(ParabolicRamp::DynamicPath& path,Real cutoff);

  //setting
  Real perturbationStep;
  int perturbLimit;

  //temporary state
  int iteration;
};

/** @brief A planner that perturbs the current configuration and uses
 * numerical IK to get an improved path.  All caveats of RealTimeIKPlanner
 * apply.
 */
class RealTimePerturbationIKPlanner : public RealTimePlannerBase
{
public:
  RealTimePerturbationIKPlanner();
  virtual void Reset(PlannerObjectiveBase* newgoal);
  virtual int PlanFrom(ParabolicRamp::DynamicPath& path,Real cutoff);

  //setting
  Real perturbationStep;
  int perturbLimit;

  //temporary state
  int iteration;
};

#endif
