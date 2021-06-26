#ifndef INPUT_PROCESSOR_H
#define INPUT_PROCESSOR_H

#include <Klampt/Modeling/World.h>
#include <Klampt/Planning/PlannerObjective.h>
#include <KrisLibrary/utils/AsyncIO.h>
#include <KrisLibrary/Timer.h>

namespace Klampt {

/** @brief An abstract base class for processing user input through a 2D
 * mouse driven gui into PlannerObjectives used for planning.
 *
 * If a new objective is available, return true in HasUpdate(), and
 * MakeObjective should return a new PlannerObjectiveBase object.
 */
class InputProcessorBase
{
 public:
  InputProcessorBase();
  virtual ~InputProcessorBase() {}
  virtual string Instructions() const { return ""; }
  virtual void Activate(bool enabled) {}
  virtual bool HasUpdate() { return true; }
  virtual void Hover(int mx,int my) {}
  virtual void Drag(float dx,float dy) {}
  virtual void Spaceball(const RigidTransform& T) {}
  virtual void SetGlobalTime(Real time) { currentTime = time; }
  virtual void SetPredictionTime(Real splitTime) {}
  virtual PlannerObjectiveBase* MakeObjective(RobotModel* robot) { return NULL; }
  virtual void DrawGL() {}

  //helpers
  RobotModel* GetRobot() const;
  void GetClickRay(int mx, int my, Ray3D& ray) const;

  WorldModel* world;
  Camera::Viewport* viewport;
  Real currentTime;
};

/** @brief Translates click-and-drag input into an IKObjective.
 */
class StandardInputProcessor : public InputProcessorBase
{
 public:
  StandardInputProcessor();
  virtual ~StandardInputProcessor() {}
  virtual string Instructions() const { return "Click and drag to pose points on the robot"; }
  virtual void Activate(bool enabled);
  virtual bool HasUpdate() { return changed; }
  virtual void Hover(int mx,int my);
  virtual void Drag(float dx,float dy);
  virtual void Spaceball(const RigidTransform& T);
  virtual PlannerObjectiveBase* MakeObjective(RobotModel* robot);
  virtual void DrawGL();

  bool move, changed;
  int currentLink;
  Vector3 currentPoint;
  Vector3 currentDestination;
  bool useSpaceball;
  RigidTransform currentDesiredTransform;
  IKGoal goal;
  Real pathCost;
};

/** @brief Translates input and extrapolated velocity into a 
 * CartesianTrackingObjective. 
 */
class PredictiveExtrapolationInputProcessor : public StandardInputProcessor
{
 public:
  PredictiveExtrapolationInputProcessor();
  virtual void Activate(bool enabled);
  virtual bool HasUpdate() { return true; }
  virtual void Hover(int mx,int my);
  virtual void Drag(float mx,float my);
  virtual void SetPredictionTime(Real splitTime);
  virtual PlannerObjectiveBase* MakeObjective(RobotModel* robot);
  virtual void DrawGL();

  Real currentInputTime;
  int numInputs;
  Vector3 sumVelocity;
  Real alpha;  //decay term for velocity estimator
  Real weightDecay,speedDecay;
  Real predictionOffset;
  bool tracking;
  PlannerObjectiveBase* lastObjective;
};



/** @brief Reads an objective function from a reader thread.
 * 
 * See LoadPlannerObjective in Planning/PlannerObjective.h for details on 
 * the format that should be provided by the thread.
 */
class SerializedObjectiveProcessor : public InputProcessorBase
{
 public:
  SerializedObjectiveProcessor(AsyncReaderThread* reader=NULL);
  virtual ~SerializedObjectiveProcessor() {}
  virtual void Activate(bool enabled);
  virtual bool HasUpdate();
  virtual PlannerObjectiveBase* MakeObjective(RobotModel* robot);

  AsyncReaderThread* reader;
};

/** @brief Reads an objective function from a socket.
 * 
 * See LoadPlannerObjective in Planning/PlannerObjective.h for details on 
 * the format that should be provided by the socket.
 */
class SocketObjectiveProcessor : public SerializedObjectiveProcessor
{
 public:
  SocketObjectiveProcessor(const char* addr) 
    :SerializedObjectiveProcessor(&socketReader),socketReader(addr)
    {}
  virtual ~SocketObjectiveProcessor() {}
  virtual string Instructions() { return "Reading objectives over socket..."; }
  SocketReadWorker socketReader;
};

} // namespace Klampt

#endif
