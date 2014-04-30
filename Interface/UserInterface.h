#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include "Modeling/World.h"
#include "Modeling/DynamicPath.h"
#include <camera/viewport.h>
#include "Planning/RobotCSpace.h"
#include "Planning/PlannerSettings.h"
#include "Planning/RealTimePlanner.h"
#include "RobotInterface.h"
#include "InputProcessor.h"
#include <utils/threadutils.h>



/** @brief An abstract base class for a user interface.
 *
 * world, viewport, settings, and robotInterface must be initialized
 * before using the class.
 */
class RobotUserInterface 
{
 public:
  RobotUserInterface();
  virtual ~RobotUserInterface() { }
  Robot* GetRobot() const { return world->robots[0].robot; }
  void GetClickRay(int mx,int my,Ray3D& ray) const;

  //Subclasses overload these functions
  //
  virtual string Name() const { return "Unnamed"; }
  virtual string Description() const { return "Unnamed"; }
  virtual string Instructions() const { return ""; }
  virtual void DrawGL() { }
  //
  //The following callbacks are called respectively upon
  //activation/deactivation;
  //mouse input;
  //spaceball input;
  //keyboard input;
  //and idle update (here is where motions should be sent).
  //The return value is a string that gets logged to disk.
  virtual string ActivateEvent(bool enable) { return ""; }
  virtual string MouseInputEvent(int mx,int my,bool drag) { return ""; }
  virtual string SpaceballEvent(const RigidTransform& T) { return ""; }
  virtual string KeypressEvent(unsigned char c,int mx,int my) { return ""; }
  virtual string UpdateEvent() { return ""; }

  //settings
  RobotWorld* world;
  Camera::Viewport* viewport;
  //if set, the planner will plan in this world
  RobotWorld* planningWorld;
  WorldPlannerSettings* settings;
  MotionQueueInterface* robotInterface;
};

/** @brief An interface that allows the user to pose individual joints
 * using mouse dragging.
 */
class JointCommandInterface : public RobotUserInterface
{
 public:
  virtual string Name() const { return "JointPoser"; }
  virtual string Description() const { return "Joint poser"; }
  virtual string Instructions() const { return "Click and drag to pose individual joints"; }
  virtual string ActivateEvent(bool enabled);
  virtual string MouseInputEvent(int mx,int my,bool drag);
  virtual string UpdateEvent();

  int currentLink; 
  Config command;
  bool sendCommand;
};

/** @brief An interface that uses an InputProcessorBase subclass to
 * process input.  By default, it uses a StandardInputProcessor which lets
 * the user to pose points on the robot in Cartesian space by pointing and
 * dragging.
 */
class InputProcessingInterface : public RobotUserInterface
{
 public:
  InputProcessingInterface();
  void SetProcessor(SmartPointer<InputProcessorBase>& newProcessor);
  bool ObjectiveChanged();
  SmartPointer<PlannerObjectiveBase> GetObjective();
  CartesianObjective* GetCartesianObjective();

  virtual string Instructions() const { if(inputProcessor) return inputProcessor->Instructions(); else return ""; }
  virtual string ActivateEvent(bool enabled);
  virtual void DrawGL();
  virtual string MouseInputEvent(int mx,int my,bool drag);
  virtual string SpaceballEvent(const RigidTransform& T);
  virtual string UpdateEvent();

  SmartPointer<InputProcessorBase> inputProcessor;
  SmartPointer<PlannerObjectiveBase> currentObjective;
};

/** @brief An interface that uses numerical IK to solve for a Cartesian
 * objective function.  Assumes that IK is fast enough to be solved in
 * a single update step.
 */
class IKCommandInterface : public InputProcessingInterface
{
 public:
  virtual string Name() const { return "PointPoser"; }
  virtual string Description() const { return "Point poser"; }
  virtual string UpdateEvent();
};

/** @brief An interface that uses a real-time planner to solve for an
 * arbitrary objective function.  Subclasses must choose which type of
 * planner to use.
 */
class PlannerCommandInterface : public InputProcessingInterface
{
 public:
  PlannerCommandInterface();
  virtual ~PlannerCommandInterface();
  virtual string Name() const { return "UnnamedPlanner"; }
  virtual string Description() const { return "Unnamed planner interface"; }

  virtual string ActivateEvent(bool enabled);
  virtual string UpdateEvent();
  virtual string Instructions() const;

  RealTimePlannerBase* planner;
  double lastPlanTime;
  double nextPlanTime;

  ///The planner will not be called until the objective function is set below
  ///this threshold.  Infinity by default (i.e., the planner will start
  ///instantly)
  double startObjectiveThreshold;
  bool started;
};

/** @brief An interface uses safe IK as the real-time planner class to achieve
 * the user's objective.
 */
class IKPlannerCommandInterface : public PlannerCommandInterface
{
 public:
  virtual string Name() const { return "IKPointPoser"; }
  virtual string Description() const { return "Smart point poser"; }
  virtual string ActivateEvent(bool enabled);

  SmartPointer<SingleRobotCSpace> cspace;
};

/** @brief An interface that uses the real-time RRT motion planner to
 * achieve the user's objective.
 */
class RRTCommandInterface : public PlannerCommandInterface
{
 public:
  virtual string Name() const { return "RRTPointPoser"; }
  virtual string Description() const { return "Goal-based point poser"; }
  virtual string ActivateEvent(bool enabled);

  //TODO: draw the plan feedback?
  //GLuint planDisplayList;

  SmartPointer<SingleRobotCSpace> cspace;
};



/** @brief Shared data structure for a multithreaded real time planner.
 * The planner must have a RealTimePlannerDataSender as a sendPathCallback.
 */
struct RealTimePlannerData
{
  Mutex mutex;
  RealTimePlannerBase* planner;
  SmartPointer<PlannerObjectiveBase> objective;   //(in) the planning objective, can be NULL
  bool active;             //(in) set this to false to quit
  Real globalTime;         //(in) time measured in calling thread
  Real startPlanTime;       //(out) time of planning

  bool pathRefresh;         //(in/out) whether to refresh the path
  Real tcut;                //(out) the path cut time, relative to startPlanTime
  ParabolicRamp::DynamicPath path;  //(out) the path to splice in 
  bool pathRefreshSuccess;  //(in) whether the execution thread read in the path successfully
};

/** @brief For use with a multithreaded RealTimePlannerBase
 */
class RealTimePlannerDataSender : public SendPathCallbackBase
{
public:
  RealTimePlannerDataSender(RealTimePlannerData* data);
  virtual bool Send(Real tplanstart,Real tcut,const ParabolicRamp::DynamicPath& path);
  RealTimePlannerData* data;
};

/** @brief A base class for a multithreaded planning robot UI.
 * Subclasses must fill out planner.
 */
class MTPlannerCommandInterface: public InputProcessingInterface
{
public:
  RealTimePlannerBase* planner;
  Thread planningThread;
  RealTimePlannerData data;

  MTPlannerCommandInterface();
  virtual ~MTPlannerCommandInterface();
  virtual string Name() const { return "UnnamedPlanner"; }
  virtual string Description() const {  return "Unnamed planner interface";  }
  
  virtual string ActivateEvent(bool enabled);
  virtual string UpdateEvent();
};

class MTIKPlannerCommandInterface: public MTPlannerCommandInterface
{
 public:
  virtual string Name() const { return "IKPointPoser"; }
  virtual string Description() const { 	return "Smart point poser (MT)"; }
  virtual string ActivateEvent(bool enabled);

  SmartPointer<SingleRobotCSpace> cspace;

};

class MTRRTCommandInterface: public MTPlannerCommandInterface
{
 public:
  virtual string Name() const { return "RRTPointPoser"; }
  virtual string Description() const {  return "Goal-based point poser (MT)"; }
  //sets up the planner
  virtual string ActivateEvent(bool enabled);

  SmartPointer<SingleRobotCSpace> cspace;
};

#endif
