#ifndef REAL_TIME_PLANNER_H
#define REAL_TIME_PLANNER_H

#include "RobotCSpace.h"
#include "PlannerSettings.h"
#include "PlannerObjective.h"
#include "RampCSpace.h"
#include <Klampt/Modeling/DynamicPath.h>
#include <KrisLibrary/utils/StatCollector.h>
#include <KrisLibrary/utils/threadutils.h>
#include <KrisLibrary/Timer.h>

namespace Klampt {

class MotionQueueInterface;

/** @brief A base class for a motion planner that generates dynamic paths.
 * The output should always respect joint, velocity, and acceleration limits
 * and end in a zero-velocity terminal states.
 *
 * Important note: the objects pointed to by CSpace* space, Robot *robot,
 * and WorldPlannerSettings *settings must be owned by an outside source.
 * Make sure to pass a shared_ptr to SetGoal if you are going to use the
 * objective elsewhere.
 */
class DynamicMotionPlannerBase
{
 public:
  enum { Failure=0, Success=1, Timeout=2 };

  DynamicMotionPlannerBase();
  virtual ~DynamicMotionPlannerBase();
  virtual void Init(CSpace* space,RobotModel* robot,WorldPlannerSettings* settings);
  virtual void SetGoal(shared_ptr<PlannerObjectiveBase> newgoal);
  virtual void SetTime(Real tstart);
  virtual void SetDefaultLimits();
  virtual void SetLimits(Real qScale=1.0,Real vScale=1.0,Real aScale=1.0);

  bool LogBegin(const char* fn="realtimeplanner.log");
  bool LogEnd();


  /** DynamicMotionPlanner subclasses should override this. 
   * Plans from the start of 'path', and returns the result in 'path'.
   * The planning duration should not exceed 'cutoff', if possible.
   * 
   * Must return Success if successful, Failure if planning fails.
   *
   * Subroutines may maintain a timer, and stop and return Timeout
   * planning time exceeds the cutoff.  This is not strictly necessary;
   * the caller should check whether the planning time exceeds the cutoff.
   */
  virtual int PlanFrom(ParabolicRamp::DynamicPath& path,Real cutoff)
  {
    return Failure;
  }

  ///Plans from a start configuration
  int PlanFrom(const Config& qstart,Real cutoff,ParabolicRamp::DynamicPath& result)
  {
    result.ramps.resize(1);
    result.ramps[0].SetConstant(qstart);
    return PlanFrom(result,cutoff);
  }

  /** Tells the planner to stop.  Can be called from an external thread.
   * 
   * Subclasses should detect if stopPlanning is set to true and return
   * Timeout.
   */
  bool StopPlanning();

  ///Performs shortcutting up until the time limit
  int Shortcut(ParabolicRamp::DynamicPath& path,Real timeLimit);
  ///Performs shortcuts that reduce the objective function, only on the 
  ///portion of the path after time tstart
  int SmartShortcut(Real tstart,ParabolicRamp::DynamicPath& path,Real timeLimit);

  ///Helper
  bool GetMilestoneRamp(const Config& q0,const Vector& dq0,const Config& q1,ParabolicRamp::DynamicPath& ramp) const;
  ///Helper
  bool GetMilestoneRamp(const ParabolicRamp::DynamicPath& curPath,const Config& q,ParabolicRamp::DynamicPath& ramp) const;

  //returns true if the ramp from the current config to q is collision free
  bool CheckMilestoneRamp(const ParabolicRamp::DynamicPath& curPath,const Config& q,ParabolicRamp::DynamicPath& ramp) const;

  ///returns the cost of going straight to q (assuming no collision detection)
  Real EvaluateDirectPathCost(const ParabolicRamp::DynamicPath& curPath,const Config& q);

  ///returns the cost of using the given path.  If tStart is not given, this
  ///uses the previously set tstart
  Real EvaluatePathCost(const ParabolicRamp::DynamicPath& path,Real tStart=-1.0);

  ///returns the terminal cost for a path ending at q at time tEnd
  Real EvaluateTerminalCost(const Config& q,Real tEnd);

  RobotModel* robot;
  WorldPlannerSettings* settings;
  CSpace* cspace;
  //objective function
  shared_ptr<PlannerObjectiveBase> goal;
  //configuration, velocity, and acceleration limits
  ParabolicRamp::Vector qMin,qMax,velMax,accMax;

  //planning start time
  Real tstart;
  //flag: another thread may set this to true when the planner should
  //stop during a long planning cycle.  The subclass needs to continually
  //monitor this flag
  bool stopPlanning;

  //log file
  FILE* flog;
};


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
 * @brief A real-time planner. Supports constant time-stepping or
 * adaptive time-stepping
 *
 * Users must set up the underlying planner, the planning
 * problem and may add in path sending hooks.
 * @sa DynamicMotionPlannerBase
 * @sa SendPathCallbackBase
 * @sa RealTimePlanningThread
 *
 * Setup: Given a world and a robotIndex (typically 0), assuming the robot
 * model is set to its current configuration ...
 *
 * @code
 * RealTimePlanner planner; 
 * planner.planner = new MyDynamicMotionPlanner;
 *    (try, for example, DynamicIKPlanner, DynamicHybridTreePlanner)
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
class RealTimePlanner
{
public:
  RealTimePlanner();
  virtual ~RealTimePlanner();

  ///Convenience fn: will set up the planner's robot, space, settings pointers
  void SetSpace(SingleRobotCSpace* space);

  ///Should be called at the start to initialize the start configuration
  void SetConstantPath(const Config& q);
  ///If the robot's path has changed for a reason outside of the planner's
  ///control, call this before planning
  void SetCurrentPath(Real tglobal,const ParabolicRamp::DynamicPath& path);

  /// Set the objective function.
  virtual void Reset(shared_ptr<PlannerObjectiveBase> newgoal);

  /// Gets the objective function
  shared_ptr<PlannerObjectiveBase> Objective() const;

  /** Calls the planner, returns the splitting time and planning time.
   * tglobal is a global clock synchronized between the planning and
   * execution threads.
   * Default implementation: fix the split time, call planner->PlanFrom
   * Returns true if the path changed and planTime < splitTime 
   */
  virtual bool PlanUpdate(Real tglobal,Real& splitTime,Real& planTime);

  /// Tells the planner to stop, when called from an external thread.
  bool StopPlanning() { 
    if(!planner) return true;
    return planner->StopPlanning(); 
  }

  /// Called whenever the sendPathCallback returned false on a planned path
  /// (e.g., the padding was too short)
  virtual void MarkSendFailure();

  /// Users must set these members before planning

  /// The underlying planing algorithm
  shared_ptr<DynamicMotionPlannerBase> planner;

  /// Set the current path before planning, using SetConstantPath or SetCurrentPath
  Real pathStartTime; 
  ParabolicRamp::DynamicPath currentPath;

  /// Users should set this up to capture the outputted path
  shared_ptr<SendPathCallbackBase> sendPathCallback;

  /// Use only in simulation: multiplies the effective computational power
  /// of this machine (i.e., simulate a machine that's x times faster)
  Real cognitiveMultiplier;

  /// Use only in simulation: set to true if you want to allow PlanFrom
  /// instances that overrun their alloted time to still update the path.
  bool acceptTimeOverruns;

  enum SplitUpdateProtocol { Constant, ExponentialBackoff, Learning };
  SplitUpdateProtocol protocol;
  Real currentSplitTime,currentPadding,currentExternalPadding;
  Real maxPadding;

  ///Statistics captured on planning times, depending on PlanMore output.
  StatCollector planFailTimeStats,planSuccessTimeStats,planTimeoutTimeStats;
};

/** @brief An interface to a planning thread.
 * 
 * All methods are thread-safe and meant to be called by the execution
 * thread.
 *
 * Example code is as follows:
 *
 * MotionQueueInterface* queue = new MyMotionQueueInterface(robot);
 *
 * RealTimePlanningThread thread;
 * thread.SetPlanner(planner)
 * thread.SetStartConfig(qstart);
 * thread.Start();
 * while(want to continue planning) {
 *   if(newObjectiveAvailable()) {
 *     thread.BreakPlanning();   //optional, if this is not called the
 *                               //planner will finish an internal planning 
 *                               //cycle on the old objective
 *     thread.SetObjective(getObjective()); 
 *     //change the cspace or planner here if needed
 *   }
 *   if(thread.SendUpdate(queue)) {
 *     printf("Planner had an update\n")
 *   }
 *   ///advance the motion queue and do whatever else you need to do here...
 * }
 * thread.Stop()
 */
class RealTimePlanningThread
{
 public:
  RealTimePlanningThread();
  ~RealTimePlanningThread();

  /// Initializes the planning thread with a start configuration 
  void SetStartConfig(const Config& qstart);
  /// Initializes the planning thread with a CSpace
  void SetCSpace(SingleRobotCSpace* space);
  /// Sets the planner
  void SetPlanner(const shared_ptr<DynamicMotionPlannerBase>& planner);
  void SetPlanner(const shared_ptr<RealTimePlanner>& planner);
  /// Set the objective function.
  void SetObjective(shared_ptr<PlannerObjectiveBase> newgoal);
  /// Gets the objective function.  (You must not delete the pointer or assign
  /// it to a shared_ptr)
  PlannerObjectiveBase* GetObjective() const;
  ///If the robot's path has changed for a reason outside of the planner's
  ///control, call this
  void ResetCurrentPath(Real tglobal,const ParabolicRamp::DynamicPath& path);
  /// Starts planning. Returns false if there was some problem, e.g., the
  /// Initial config was not set.
  bool Start();
  /// Stops the planning thread. 
  void Stop();
  /// Returns true if a planning cycle is happening
  bool IsPlanning();
  /// Pauses planning after the current planning cycle
  void PausePlanning();
  /// Breaks an internal planning cycle on the existing objective
  void BreakPlanning();
  /// Stops planning.  Equivalent to break and pause
  void StopPlanning();
  /// Resumes planning after a pause or stop
  void ResumePlanning();
  /// Returns true if a trajectory update is available
  bool HasUpdate();
  /// Send the trajectory update, if one exists
  bool SendUpdate(MotionQueueInterface* interface);
  /// Returns the objective function value of the current trajectory
  /// update
  Real ObjectiveValue();

  void* internal;
  shared_ptr<RealTimePlanner> planner;
  Thread thread;
};

} // namespace Klampt

#endif
