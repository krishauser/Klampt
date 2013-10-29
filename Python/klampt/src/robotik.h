#ifndef ROBOTIK_H
#define ROBOTIK_H

#include "robotmodel.h"
#include <robotics/IK.h>

// Forward declaration of C-type PyObject
struct _object;
typedef _object PyObject;

/**
 * @brief A class defining an inverse kinematic target.  Either a link on a
 * robot can take on a fixed position/orientation in the world frame, or
 * a relative position/orientation to another frame.
 *
 * Currently only fixed-point constraints and fixed-transform constraints are
 * implemented in the Python API.
 */
class IKObjective
{
 public:
  IKObjective();
  ///The link that is constrained
  int link() const;
  ///The destination link, or -1 if fixed to the world
  int destLink() const;
  ///Returns the number of position dimensions constrained (0-3)
  int numPosDims() const;
  ///Returns the number of rotation dimensions constrained (0-3)
  int numRotDims() const;

  ///Sets a fixed-point constraint
  void setFixedPoint(int link,const double plocal[3],const double pworld[3]);
  ///Sets a multiple fixed-point constraint
  void setFixedPoints(int link,PyObject* plocals,PyObject* pworlds);
  ///Sets a fixed-transform constraint (R,t)
  void setFixedTransform(int link,const double R[9],const double t[3]);
  ///Sets a fixed-point constraint relative to link2
  void setRelativePoint(int link1,int link2,const double p1[3],const double p2[3]);
  ///Sets a multiple fixed-point constraint relative to link2
  void setRelativePoints(int link1,int link2,PyObject* p1s,PyObject* p2s);
  ///Sets a fixed-transform constraint (R,t) relative to linkTgt
  void setRelativeTransform(int link,int linkTgt,const double R[9],const double t[3]);
  ///Returns the local and global position of the position constraint
  void getPosition(double out[3],double out2[3]) const;
  ///For linear and planar constraints, returns the direction
  void getPositionDirection(double out[3]) const;
  ///For fixed rotation constraints, returns the orientation
  void getRotation(double out[9]) const;
  ///For axis rotation constraints, returns the local and global axes
  void getRotationAxis(double out[3],double out2[3]) const;
  ///For fixed-transform constraints, returns the transform (R,T)
  void getTransform(double out[9],double out2[3]) const;

  IKGoal goal;
};

/**
 * @brief An inverse kinematics solver based on the Newton-Raphson technique.
 *
 * Typical calling pattern is
 * s = IKSolver(robot)
 * s.add(objective1)
 * s.add(objective2)
 * (res,iters) = s.solve(100,1e-4)
 * if res:
 *    print "IK solution:",robot.getConfig(),"found in",iters,"iterations, residual",s.getResidual()
 * else:
 *    print "IK failed:",robot.getConfig(),"found in",iters,"iterations, residual",s.getResidual()
 *
 * sampleInitial() is a convenience routine.  More initial configurations can
 * be sampled in case the prior configs lead to local minima.
 */
class IKSolver
{
 public:
  IKSolver(const RobotModel& robot);
  IKSolver(const IKSolver& solver);

  /// Adds a new simultaneous objective
  void add(const IKObjective& objective);
  /// Sets the active degrees of freedom
  void setActiveDofs(const std::vector<int>& active);
  /// Gets the active degrees of freedom
  void getActiveDofs(std::vector<int>& out);

  /// Returns a vector describing the error of the objective
  void getResidual(std::vector<double>& out);
  /// Returns a matrix describing the instantaneous derivative of the objective
  /// with respect to the active Dofs
  void getJacobian(std::vector<std::vector<double> >& out);

  /** Tries to find a configuration that satifies all simultaneous objectives
   * up to the desired tolerance.
   * Returns (res,iters) where res indicates whether x converged.
   */
  PyObject* solve(int iters,double tol=1e-3);

  /// Samples an initial random configuration
  void sampleInitial();

  RobotModel robot;
  std::vector<IKObjective> objectives;
  bool useJointLimits;
  std::vector<int> activeDofs;
};


/**
 * @brief An inverse kinematics target for matching points between 
 * two robots and/or objects.
 *
 * The objects are chosen upon construction, so the following are valid:
 * - GeneralizedIKObjective(a) is an objective for object a to be constrained
 *   relative to the environment.
 * - GeneralizedIKObjective(a,b) is an objective for object a to be
 *   constrained relative to b.
 * Here a and b can be links on any robot or rigid objects.
 *
 * Once constructed, call setPoint, setPoints, or setTransform to specify
 * the nature of the constraint.
 */
class GeneralizedIKObjective
{
 public:
  GeneralizedIKObjective(const GeneralizedIKObjective& obj);
  GeneralizedIKObjective(const RobotModelLink& link);
  GeneralizedIKObjective(const RigidObjectModel& obj);
  GeneralizedIKObjective(const RobotModelLink& link,const RobotModelLink& link2);
  GeneralizedIKObjective(const RobotModelLink& link,const RigidObjectModel& obj2);
  GeneralizedIKObjective(const RigidObjectModel& obj,const RobotModelLink& link2);
  GeneralizedIKObjective(const RigidObjectModel& obj,const RigidObjectModel& obj2);
  void setPoint(const double p1[3],const double p2[3]);
  void setPoints(PyObject* p1s,PyObject* p2s);
  void setTransform(const double R[9],const double t[3]);

  RobotModelLink link1,link2;
  RigidObjectModel obj1,obj2;
  bool isObj1,isObj2;
  IKGoal goal;
};

/**
 * @brief An inverse kinematics solver between multiple robots and/or objects.
 */
class GeneralizedIKSolver
{
 public:
  GeneralizedIKSolver(const WorldModel& world);

  /// Adds a new simultaneous objective
  void add(const GeneralizedIKObjective& objective);

  /// Returns a vector describing the error of the objective
  void getResidual(std::vector<double>& out);
  /// Returns a matrix describing the instantaneous derivative of the objective
  /// with respect to the active parameters
  void getJacobian(std::vector<std::vector<double> >& out);

  /** Tries to find a configuration that satifies all simultaneous objectives
   * up to the desired tolerance.  
   * Returns (res,iters) where res indicates whether x converged.
   */
  PyObject* solve(int iters,double tol=1e-3);

  /// Samples an initial random configuration
  void sampleInitial();

  WorldModel world;
  std::vector<GeneralizedIKObjective> objectives;
  bool useJointLimits;
};

///Returns a transformation (R,t) from link to link2 sampled at random from
///the space of transforms that satisfies the objective.
void SampleTransform(const IKObjective& obj,double out[9],double out2[3]);
void SampleTransform(const GeneralizedIKObjective& obj,double out[9],double out2[3]);

#endif
