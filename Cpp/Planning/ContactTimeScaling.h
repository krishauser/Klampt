#ifndef CONTACT_TIME_SCALING_H
#define CONTACT_TIME_SCALING_H

#include "TimeScaling.h"
#include <Klampt/Modeling/MultiPath.h>
#include <Klampt/Modeling/Robot.h>
#include "RobotCSpace.h"
#include <KrisLibrary/math3d/Polygon2D.h>

namespace Klampt {

/** @brief A base class for a time scaling with colocation point constraints.
 * Subclasses should fill in dsmax, ds2ddsConstraintNormals, and
 * ds2ddsConstraintOffsets before Optimize is called.
 *
 * Output is given in traj.
 */
class CustomTimeScaling
{
 public:
  CustomTimeScaling(RobotModel& robot);
  ///Sets up the path
  void SetPath(const GeneralizedCubicBezierSpline& path,
	       const vector<Real>& colocationParams);
  ///Sets up the path -- multipath version
  void SetPath(const MultiPath& path,
	       const vector<Real>& colocationParams);
  ///Sets up the path to start and stop at zero velocity
  void SetStartStop();
  ///Sets up velocity and acceleration bounds
  void SetDefaultBounds();
  ///Runs the optimizer with the custom constraints
  bool Optimize();
  ///Returns true if the time scaling derivatives are feasible under the current constraints
  bool IsFeasible(const vector<Real>& ds) const;
  ///After running Optimize, prints out all the active constraints 
  void PrintActiveConstraints(ostream& out);

  RobotCSpace cspace;

  ///Output trajectory
  TimeScaledBezierCurve traj;
  ///colocation grid
  vector<Real> paramDivs;
  ///index from grid to section of multipath, if multipath is given
  vector<int> paramSections;
  ///colocation points
  vector<Vector> xs,dxs,ddxs;
  ///time scaling maxima.  Defaults to Inf
  vector<Real> dsmax;
  ///Constraint planes a^T (ds^2,dds) <= b in the
  ///(squared-rate, acceleration) plane
  vector<vector<Vector2> > ds2ddsConstraintNormals;
  vector<vector<Real> > ds2ddsConstraintOffsets;
  ///Names of each constraint plane
  bool saveConstraintNames;
  vector<vector<string> > ds2ddsConstraintNames;

  ///Whether the lagrange multipliers of a solution are requested
  bool computeLagrangeMultipliers;
  ///Lagrange multipliers for each velocity^2 variable's velocity^2 limit.
  ///Gives the amount by which the execution time would be reduced if the
  ///velocity^2 limit is increased.
  vector<Real> variableLagrangeMultipliers;
  ///Lagrange multipliers for each constraint plane.  Gives the amount by
  ///which the execution time would be reduced if the constraint plane is
  ///shifted outward.
  vector<vector<Real> > constraintLagrangeMultipliers;
};

/** @brief A time scaling with torque constraints |t| <= tmax.
 * Assuming fixed base manipulator.
 *
 * t = B(q)q'' + C(q,q') + G(q)
 * if v is the tangent, a is the curvature, then q' = ds*v, q'' = v*dds + a*ds^2
 * t = ds^2 (B(q)a+C(q,v)) + dds B(q)v + G(q)
 *
 * The torqueLimitShift and torqueLimitScale terms let you adjust the
 *  robot's torque limits to account for errors during execution.
 */
class TorqueTimeScaling : public CustomTimeScaling
{
 public:
  TorqueTimeScaling(RobotModel& robot);
  void SetParams(const MultiPath& path,const vector<Real>& colocationParams);

  Real torqueLimitShift;   ///< offsets the torque limits by a fixed amount (default 0)
  Real torqueLimitScale;   ///< from 0 to 1, scales the torque limits (default 1)
};

/** @brief A time scaling with Zero Moment Point constraints.
 * Each section of the multipath corresponds with one of the support polygons.
 * 
 * Assuming no change in angular momentum and no z acceleration of the CM,
 * the ZMP is given by
 *   Zx = Cx - Cz/g Cx''
 *   Zy = Cy - Cz/g Cy''
 * Where Cz is the height of the CM over the ground plane, and (Cx,Cy) are 
 * its x-y coordinates.
 * CM(q) => CM'(q) = dCM/dq(q)*q'
 *       => CM''(q) = q'^T ddCM/ddq(q)*q' + dCM/dq(q)*q''
 *
 * For motions with more complex contact formations, use ContactTimeScaling.
 */
class ZMPTimeScaling : public CustomTimeScaling
{
 public:
  ZMPTimeScaling(RobotModel& robot);
  ///Sets a multi-stance ZMP optimization problem
  void SetParams(const MultiPath& path,const vector<Real>& colocationParams,
     const vector<ConvexPolygon2D>& supportPolys,const vector<Real>& groundHeights);
  ///1-stance convenience version of the above
  void SetParams(const MultiPath& path,const vector<Real>& colocationParams,
		 const vector<Vector2>& supportPoly,Real groundHeight=0);

  ///double-check whether the solution is actually feasible
  bool Check(const MultiPath& path);

  vector<ConvexPolygon2D> supportPolys;
  vector<Real> groundHeights;
};

/** @brief A time scaling with torque/contact constraints
 *
 * Uses dynamic equation B(q)*q''+C(q,q')+G(q) = t+J^T(q)*f
 *   |t| <= tmax
 *   f \in FC
 * with f being the contact forces, J being the jacobian of the contact
 * points, tmax being the torque limits, and FC being the (polygonalized)
 * friction cones.
 *
 * Can shift/scale torque limits, reduce the friction coefficients, and
 * add a robustness margin to the friction cones.
 */
class ContactTimeScaling : public CustomTimeScaling
{
 public:
  ContactTimeScaling(RobotModel& robot);
  ///Uses the stances inside the multipath to determine the contacts.
  ///Discretizes friction cone into pyramid with numFCEdges edges.
  bool SetParams(const MultiPath& path,const vector<Real>& colocationParams,
		 int numFCEdges = 4);

  ///double-check whether the solution is actually feasible
  bool Check(const MultiPath& path);

  Real torqueLimitShift;   ///< offsets the torque limits by a fixed amount (default 0)
  Real torqueLimitScale;   ///< from 0 to 1, scales the torque limits (default 1)
  Real frictionRobustness;  ///< from 0 to 1, indicates the amount of increased robustness in friction cones (default 0)
  Real forceRobustness;   ///< >= 0, indicates the absolute margin for forces to be contained within the friction cone
};

} //namespace Klampt

#endif
