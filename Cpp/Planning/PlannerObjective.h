#ifndef PLANNER_OBJECTIVES_H
#define PLANNER_OBJECTIVES_H

#include <Klampt/Modeling/Robot.h>
#include <Klampt/Modeling/ParabolicRamp.h>
#include <Klampt/Modeling/DynamicPath.h>
#include <KrisLibrary/robotics/IK.h>

class AnyCollection;

namespace Klampt {

/** @ingroup Planning
 * @brief A base class for objective functionals in time/config/velocity
 * space 
 */
class PlannerObjectiveBase
{
 public:
  PlannerObjectiveBase() {}
  virtual ~PlannerObjectiveBase() {}

  ///Subclasses: return an identifier for this goal type
  virtual const char* TypeString() { return NULL; }

  ///Subclasses: return a string for printing (optional)
  virtual string Description() { if(TypeString()) return TypeString(); return ""; }

  ///Subclasses: return the cost of a terminal state (qend,dqend) 
  ///reached at time tend
  virtual Real TerminalCost(Real tend,const Vector& qend,const Vector& dqend) { return 0.0; }

  ///Subclasses: return the differential cost of passing through state
  ///(q,dq) at time t
  virtual Real DifferentialCost(Real t,const Vector& q,const Vector& dq) { return 0.0; }

  ///Subclasses: planners may exploit time invariant costs for faster performance
  virtual bool DifferentialTimeInvariant() const { return false; }
  virtual bool TerminalTimeInvariant() const { return false; }
  ///Subclasses: planners may exploit path-invariant costs for faster performance
  virtual bool PathInvariant() const { return false; }

  ///Subclasses: return the incremental cost of undertaking the segment 'ramp'
  ///starting at time t.  This should be equal to the integral of the differential
  ///cost, or at least a good approximation
  virtual Real IncrementalCost(Real t,const ParabolicRamp::ParabolicRampND& ramp) { return 0.0; }
  virtual Real IncrementalCost(Real t,const ParabolicRamp::DynamicPath& path);

  ///Optional: return the cost of a candidate path starting at time tstart.
  ///This should be equal to the sum of the increment costs + the terminal cost
  virtual Real PathCost(const ParabolicRamp::DynamicPath& path,Real tstart=0);

  ///Subclasses: a similarity metric in for the amount of change between this
  ///objective and a prior goal.  Results >= 0.
  virtual Real Delta(PlannerObjectiveBase* priorGoal) { return Inf; }

  ///Subclasses: read and write to binary file (optional, not implemented yet)
  virtual bool Read(File& file) { return false; }
  virtual bool Write(File& file) { return false; }
};

/** @ingroup Planning
 * @brief An objective that measures path execution time.
 */
class TimeObjective : public PlannerObjectiveBase
{
 public:
  TimeObjective() {}
  virtual ~TimeObjective() {}
  virtual const char* TypeString() { return "time"; }
  virtual Real PathCost(const ParabolicRamp::DynamicPath& path,Real tstart=0);
  virtual Real DifferentialCost(Real t,const Vector& q,const Vector& dq) { return 1.0; }
  virtual Real IncrementalCost(Real t,const ParabolicRamp::ParabolicRampND& ramp);
  virtual Real Delta(PlannerObjectiveBase* priorGoal);
  virtual bool DifferentialTimeInvariant() const { return true; }
  virtual bool TerminalTimeInvariant() const { return true; }
};

/** @ingroup Planning
 * @brief A goal that measures absolute difference in terminal time (i.e.,
 * penalize stopping at a different time than specified). 
 */
class TerminalTimeObjective : public PlannerObjectiveBase
{
 public:
  TerminalTimeObjective(Real tgoal);
  virtual ~TerminalTimeObjective() {}
  virtual const char* TypeString() { return "term_time"; }
  virtual Real TerminalCost(Real tend,const Vector& qend,const Vector& dqend);
  virtual Real PathCost(const ParabolicRamp::DynamicPath& path,Real tstart=0);
  virtual Real Delta(PlannerObjectiveBase* priorGoal);
  virtual bool DifferentialTimeInvariant() const { return true; }

  Real tgoal;
};

/** @ingroup Planning
 * @brief A goal that measures distance to a goal configuration qgoal
 */
class ConfigObjective : public PlannerObjectiveBase
{
 public:
  ConfigObjective(const Config& qgoal);
  virtual ~ConfigObjective() {}
  virtual const char* TypeString() { return "config"; }
  virtual Real TerminalCost(Real tend,const Vector& qend,const Vector& dqend) {
    return qend.distance(qgoal);
  }
  virtual Real PathCost(const ParabolicRamp::DynamicPath& path,Real tstart=0) {
    return qgoal.distance(Vector(path.ramps.back().x1));
  }
  virtual Real Delta(PlannerObjectiveBase* priorGoal);
  virtual bool TerminalTimeInvariant() const { return true; }
  virtual bool DifferentialTimeInvariant() const { return true; }
  virtual bool PathInvariant() const { return true; }

  Vector qgoal;
};

/** @ingroup Planning
 * @brief A goal that measures distance to a goal velocity vgoal
 */
class VelocityObjective : public PlannerObjectiveBase
{
 public:
  VelocityObjective(const Vector& vgoal);
  virtual ~VelocityObjective() {}
  virtual const char* TypeString() { return "velocity"; }
  virtual Real TerminalCost(Real tend,const Vector& qend,const Vector& dqend) {
    return dqend.distance(vgoal);
  }
  virtual Real PathCost(const ParabolicRamp::DynamicPath& path,Real tstart=0) {
    return vgoal.distance(Vector(path.ramps.back().dx1));
  }
  virtual Real Delta(PlannerObjectiveBase* priorGoal);
  virtual bool TerminalTimeInvariant() const { return true; }
  virtual bool DifferentialTimeInvariant() const { return true; }
  virtual bool PathInvariant() const { return true; }

  Vector vgoal;
};

/** @ingroup Planning
 * @brief An objective that merges contributions from multiple other
 * objective functions.
 */
class CompositeObjective : public PlannerObjectiveBase
{
 public:
  CompositeObjective();
  ~CompositeObjective();

  ///Adds a new component.  Note: this takes ownership of the pointer.
  void Add(const shared_ptr<PlannerObjectiveBase>& obj,Real weight=1.0);

  virtual const char* TypeString() { return "composite"; }
  virtual string Description();

  virtual Real TerminalCost(Real tend,const Vector& qend,const Vector& dqend);
  virtual Real DifferentialCost(Real t,const Vector& q,const Vector& dq);
  virtual Real IncrementalCost(Real t,const ParabolicRamp::ParabolicRampND& ramp);
  virtual Real PathCost(const ParabolicRamp::DynamicPath& path,Real tstart=0);
  virtual Real Delta(PlannerObjectiveBase* priorGoal);
  virtual bool TerminalTimeInvariant() const;
  virtual bool DifferentialTimeInvariant() const;
  virtual bool PathInvariant() const;

  Real norm;
  vector<shared_ptr<PlannerObjectiveBase> > components;
  vector<Real> weights;
};


/** @ingroup Planning
 * @brief A goal that measures point-to-point distance.
 */
class CartesianObjective : public PlannerObjectiveBase
{
 public:
  CartesianObjective(RobotModel* robot);
  virtual ~CartesianObjective() {}

  virtual const char* TypeString() { return "cartesian"; }

  virtual Real TerminalCost(Real tend,const Vector& qend,const Vector& dqend);
  virtual Real PathCost(const ParabolicRamp::DynamicPath& path,Real tstart=0) {
    return TerminalCost(0,Vector(path.ramps.back().x1),Vector(path.ramps.back().dx1));
  }
  virtual Real Delta(PlannerObjectiveBase* priorGoal);
  virtual bool TerminalTimeInvariant() const { return true; }
  virtual bool DifferentialTimeInvariant() const { return true; }
  virtual bool PathInvariant() const { return true; }

  RobotModel* robot;
  IKGoal ikGoal;
};

/** @ingroup Planning
 * @brief A goal for an IK solution (including possibly rotation)
 */
class IKObjective : public PlannerObjectiveBase
{
 public:
  IKObjective(RobotModel* robot);
  virtual ~IKObjective() {}

  virtual const char* TypeString() { return "ik"; }

  virtual Real TerminalCost(Real tend,const Vector& qend,const Vector& dqend);

  virtual Real PathCost(const ParabolicRamp::DynamicPath& path,Real tstart=0) {
    return TerminalCost(0,Vector(path.ramps.back().x1),Vector(path.ramps.back().dx1));
  }

  virtual Real Delta(PlannerObjectiveBase* priorGoal);

  virtual bool TerminalTimeInvariant() const { return true; }
  virtual bool DifferentialTimeInvariant() const { return true; }
  virtual bool PathInvariant() const { return true; }

  RobotModel* robot;
  IKGoal ikGoal;
  Real posCoeff,oriCoeff;
};

/** @ingroup Planning
 * @brief Tracking a path in cartesian space
 */
class CartesianTrackingObjective : public PlannerObjectiveBase
{
 public:
  CartesianTrackingObjective(RobotModel* robot);
  virtual ~CartesianTrackingObjective() {}
  virtual const char* TypeString() { return "cartesian_tracking"; }

  virtual Real TerminalCost(Real tend,const Vector& qend,const Vector& dqend);
  virtual Real DifferentialCost(Real t,const Vector& q,const Vector& dq);
  virtual Real IncrementalCost(Real t,const ParabolicRamp::ParabolicRampND& ramp);
  virtual Real PathCost(const ParabolicRamp::DynamicPath& path,Real tstart=0);
  virtual Real Delta(PlannerObjectiveBase* priorGoal);

  ///Finds the index i such that times[i] <= t < times[i+1].
  ///If t < times[0], returns -1, or if t >= times.back(), returns times.size()
  int FindSegment(Real t) const;
  //integrates the time range [a,b] over the ramp starting at tstart
  Real IntegrateSegment(int index,Real a,Real b,Real tstart,const ParabolicRamp::ParabolicRampND& ramp);
  //computes desired 3-space position at time t
  Vector3 GetDesiredPosition(Real t) const;
  //differential cost at time t in the form (x-b)^T A (x-b)
  void GetDifferentialCostFunction(Real t,Matrix3& A,Vector3& b) const;
  Real DifferentialCost(Real t,const Vector& q,int index);

  RobotModel* robot;
  //track localPosition on this link 
  int link;
  Vector3 localPosition;
  //3-space trajectory to follow
  vector<Vector3> positions;
  vector<Real> times;
  //optional: scalar weight for each time
  vector<Real> weights;
  //optional: matrix weight e^T A e for each time
  vector<Matrix3> matWeights;
  //optional: terminal cost
  Real endTimeWeight,endPosWeight;
  Matrix3 endPosMatWeight;
};

/** @brief Reads and constructs an objective from a JSON message of the form
 * {type:[type],attr1:[value1],...,attrn:[valuen]}
 * where [type] is an objective type and the attribute-value
 * pairs are type-dependent.  If robot is provided, more error checking is
 * performed.
 *
 * Currently supported objective types (and their attributes include:
 * - config: sets a destination configuration.
 *   * data: the destination configuration.
 * - time: minimizes time.
 * - term_time: penalizes absolute difference to an ending time
 *   * data: the destination time.
 * - velocity: sets a destination velocity.
 *   * data: the destination velocity.
 * - composite: a composite 
 *   * norm (optional): the norm for weighting components (1, 2, or "inf")
 *   * components: a list of component objective functions.
 *   * weights (optional): a list of component weights.
 * - cartesian: sets a destination position / orientation.
 *   * link: link of the point on robot
 *   * plocal: local coordinates of the point on robot
 *   * pworld: target positions of the point on the world
 * - ik: sets an IK objective
 *   * data: serialized IKGoal.
 */
PlannerObjectiveBase* LoadPlannerObjective(istream& in,RobotModel* robot=NULL);

///Saves an objective in the format used by LoadPlannerObjective
bool SavePlannerObjective(PlannerObjectiveBase* obj,ostream& out);

///Same as the other LoadPlannerObjective, but just given a JSON structure.
PlannerObjectiveBase* LoadPlannerObjective(AnyCollection& msg,RobotModel* robot=NULL);
bool SavePlannerObjective(PlannerObjectiveBase* obj,AnyCollection& msg);

} //namespace Klampt

#endif

