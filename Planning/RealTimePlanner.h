#ifndef REAL_TIME_PLANNER_H
#define REAL_TIME_PLANNER_H

#include "RobotCSpace.h"
#include "PlannerSettings.h"
#include "PlannerObjective.h"
#include "RampCSpace.h"
#include "Modeling/DynamicPath.h"
#include <utils/StatCollector.h>
#include <Timer.h>


/** @brief A base class for the path sending callback.  Send is called by the
 * planner to update the path used by the execution thread.
 */
class SendPathCallbackBase
{
 public:
  virtual ~SendPathCallbackBase() {}
  /** Splice in path at time tcut, where tcut is relative to the time at
   * which planning was initiated, and tplanstart is the time the planning
   * started.
   *
   * If the update isn't received at a valid time < tplanstart + tcut, then
   * Send should return false. 
   */
  virtual bool Send(Real tplanstart,Real tcut,const ParabolicRamp::DynamicPath& path) =0;
};


/** @ingroup Planning
 * @brief A base class for a real-time planner.
 * Supports constant time-stepping or adaptive time-stepping
 *
 * Subclasses must override PlanMore.  Users must set up the planning
 * problem and may add in path sending hooks.
 * @sa RealTimeIKPlanner
 * @sa RealTimeTreePlanner
 *
 * Setup: Given a world and a robotIndex (typically 0), assuming the robot
 * model is set to its current configuration ...
 *
 * @code
 * MyRealTimePlanner planner; (initialize subclass as necessary)
 * SingleRobotCSpace cspace(...); (initialize cspace as necessary)
 * //set planning problem
 * planner.SetSpace(&cspace);
 * //set current path -- just stationary
 * planner.SetConstantPath(planner.robot->q);
 * //set objective
 * IKObjective* objective = new IKObjective(robot);
 * objective->ikGoal = ... // setup initial IK objective
 * planner.Reset(objective);
 * planner.sendPathCallback = new MySendPathCallback;
 * @endcode
 * 
 * Online: The planner should be launched in a parallel thread to the
 * execution thread. The planning thread should maintain a global timer,
 * synchronized with the execution thread. 
 *
 * @code
 * while(true) {
 *   Real t = currentGlobalTime();
 *   if(objectiveChanged) {
 *     //set new objective if needed
 *     IKObjective* objective = new IKObjective(robot);
 *     objective->ikGoal = ... // setup IK objective
 *     planner.Reset(objective);
 *   }
 * 
 *   Real splitTime, planTime;
 *   bool changed = planner.PlanUpdate(t,splitTime,planTime);
 *   //planner.sendPathCallback will be called if the planner succeeded
 * 
 *   if(changed) {
 *     printf("Path was successfully updated, split time %g, plan time %g\n",splitTime,planTime);
 *   }
 *   else {
 *     printf("Path update was unsuccessful, split time %g, plan time %g\n",splitTime,planTime);
 *   }
 * }
 * @endcode
 */
class RealTimePlannerBase
{
public:
  enum { Failure=0, Success=1, Timeout=2 };

  RealTimePlannerBase();
  virtual ~RealTimePlannerBase();

  ///Convenience fn: will set up the robot, space, settings pointers
  void SetSpace(SingleRobotCSpace* space);

  ///Should be called at the start to initialize the start configuration
  void SetConstantPath(const Config& q);
  ///If the robot's path has changed for a reason outside of the planner's
  ///control, call this before planning
  void SetCurrentPath(Real tglobal,const ParabolicRamp::DynamicPath& path);

  /// Set the objective function.  Takes ownership of the pointer
  virtual void Reset(PlannerObjectiveBase* newgoal);

  /** Performs planning, returns the splitting time and planning time.
   * tglobal is a global clock synchronized between the planning and
   * execution threads.
   * Default implementation: fix the split time, call PlanFrom
   * Returns true if the path changed and planTime < splitTime */
  virtual bool PlanUpdate(Real tglobal,Real& splitTime,Real& planTime);

  /** RealTimePlannerBase subclasses should override this. 
   * Plans from the start of 'path', and returns the result in 'path'.
   * The planning duration should not exceed 'cutoff', if possible.
   * 
   * Must return Success if successful, Failure if planning fails.
   *
   * Subroutines may maintain a timer, and stop and return Timeout
   * planning time exceeds the cutoff.  This is not strictly necessary;
   * PlanUpdate already checks whether the planning time exceeds the cutoff.
   * Timeout is mostly used for debugging; statistics are captured in
   * planTimeoutTimeStats.
   */
  virtual int PlanFrom(ParabolicRamp::DynamicPath& path,Real cutoff)
  {
    return Failure;
  }

  ///Performs shortcutting up until the time limit
  int Shortcut(ParabolicRamp::DynamicPath& path,Real timeLimit) const;
  ///Performs shortcuts that reduce the objective function, only on the 
  ///portion of the path after time tstart
  int SmartShortcut(Real tstart,ParabolicRamp::DynamicPath& path,Real timeLimit) const;

  ///Helper
  bool GetMilestoneRamp(const Config& q0,const Vector& dq0,const Config& q1,ParabolicRamp::DynamicPath& ramp) const;
  ///Helper
  bool GetMilestoneRamp(const ParabolicRamp::DynamicPath& curPath,const Config& q,ParabolicRamp::DynamicPath& ramp) const;

  //returns true if the ramp from the current config to q is collision free
  bool CheckMilestoneRamp(const ParabolicRamp::DynamicPath& curPath,const Config& q,ParabolicRamp::DynamicPath& ramp) const;

  ///returns the cost of going straight to q (assuming no collision detection)
  Real EvaluateDestinationCost(const Config& q) const;

  ///returns the cost of using the given path
  Real EvaluatePathCost(const ParabolicRamp::DynamicPath& path,Real tStart=0.0) const;

  ///returns the current final destination
  const ParabolicRamp::Vector& CurrentDestination() const;

  ///returns the cost for the current destination
  Real CurrentDestinationCost() const;

  ///returns the cost for the current path
  Real CurrentPathCost() const;

  /// Called whenever the sendPathCallback returned false on a planned path
  /// (e.g., the padding was too short)
  virtual void MarkSendFailure();

  /// Users must set these members before planning
  Robot* robot;
  WorldPlannerSettings* settings;
  SingleRobotCSpace* cspace;
  /// Set the current path before planning, using SetConstantPath or SetCurrentPath
  Real pathStartTime; 
  ParabolicRamp::DynamicPath currentPath;
  /// Set objective before planning, using Reset()
  PlannerObjectiveBase* goal;

  /// Users should set this up to capture the outputted path
  SmartPointer<SendPathCallbackBase> sendPathCallback;

  /// Use only in simulation: multiplies the effective computational power
  /// of this machine (i.e., simulate a machine that's x times faster)
  Real cognitiveMultiplier;

  /// Use only in simulation: set to true if you want to allow PlanFrom
  /// instances that overrun their alloted time to still update the path.
  bool acceptTimeOverruns;

  enum SplitUpdateProtocol { Constant, ExponentialBackoff, Learning };
  SplitUpdateProtocol protocol;
  Real currentSplitTime,currentPadding,currentExternalPadding;

  ///Statistics captured on planning times, depending on PlanMore output.
  StatCollector planFailTimeStats,planSuccessTimeStats,planTimeoutTimeStats;
};

#endif
