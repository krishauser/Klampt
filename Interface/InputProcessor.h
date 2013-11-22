#ifndef INPUT_PROCESSOR_H
#define INPUT_PROCESSOR_H

#include "Modeling/World.h"
#include "Planning/PlannerObjective.h"

/** @brief An abstract base class for processing user input through a 2D
 * mouse driven gui into PlannerObjectives used for planning.
 */
struct InputProcessorBase
{
  InputProcessorBase();
  virtual ~InputProcessorBase() {}
  virtual void Reset() {}
  virtual bool HasUpdate() { return true; }
  virtual void Hover(int mx,int my) {}
  virtual void Drag(float dx,float dy) {}
  virtual void Spaceball(const RigidTransform& T) {}
  virtual void SetGlobalTime(Real time) { currentTime = time; }
  virtual void SetPredictionTime(Real splitTime) {}
  virtual PlannerObjectiveBase* MakeObjective(Robot* robot) { return NULL; }
  virtual void DrawGL() {}

  //helpers
  Robot* GetRobot() const;
  void GetClickRay(int mx, int my, Ray3D& ray) const;

  RobotWorld* world;
  Camera::Viewport* viewport;
  Real currentTime;
};

/** @brief Translates click-and-drag input into an IKObjective.
 */
struct StandardInputProcessor : public InputProcessorBase
{
  StandardInputProcessor();
  virtual void Reset();
  virtual bool HasUpdate() { return changed; }
  virtual void Hover(int mx,int my);
  virtual void Drag(float dx,float dy);
  virtual void Spaceball(const RigidTransform& T);
  virtual PlannerObjectiveBase* MakeObjective(Robot* robot);
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
struct PredictiveExtrapolationInputProcessor : public StandardInputProcessor
{
  PredictiveExtrapolationInputProcessor();
  virtual void Reset();
  virtual bool HasUpdate() { return true; }
  virtual void Hover(int mx,int my);
  virtual void Drag(float mx,float my);
  virtual void SetPredictionTime(Real splitTime);
  virtual PlannerObjectiveBase* MakeObjective(Robot* robot);
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




#endif
