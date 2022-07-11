#ifndef ROBOT_TIME_SCALING_H
#define ROBOT_TIME_SCALING_H

#include <KrisLibrary/planning/GeneralizedBezierCurve.h>
#include <Klampt/Modeling/Robot.h>
#include <Klampt/Modeling/MultiPath.h>

namespace Klampt {

/** @ingroup Planning
 * @brief Optimizes a piecewise-linear path by first smoothing it, then
 * time-optimizing it with the given resolution dt.
 */
bool TimeOptimizePath(RobotModel& robot,const vector<Real>& oldtimes,const vector<Config>& oldconfigs,Real dt,vector<Real>& newtimes,vector<Config>& newconfigs);

/** @ingroup Planning
 * @brief Generates a constrained piecewise linear path between two
 * configurations while satisfying the constraints in ikGoals.
 */
bool InterpolateConstrainedPath(RobotModel& robot,const Config& a,const Config& b,const vector<IKGoal>& ikGoals,vector<Config>& path,Real xtol=1e-2);

/** @ingroup Planning
 * @brief Generates a constrained piecewise linear path between many
 * configurations while satisfying the constraints in ikGoals.
 */
bool InterpolateConstrainedPath(RobotModel& robot,const vector<Config>& milestones,const vector<IKGoal>& ikGoals,vector<Config>& path,Real xtol=1e-2);

/** @ingroup Planning
 * @brief Given a list of milestones oldconfigs, constructs a smooth interpolating
 * path and discretizes it uniformly with resolution n.  The output times
 * map to the range [0,1].
 */
void SmoothDiscretizePath(RobotModel& robot,const vector<Config>& oldconfigs,int n,vector<Real>& times,vector<Config>& configs);

/** @ingroup Planning
 * @brief Given a coarsely discretized multipath, produces a finely discretized,
 * smooth curve with appropriate tangents across contact states. 
 * 
 * Each segment must begin and end at transitions.  xtol is the discretization
 * resolution.
 * 
 * This can be applied to increase the resolution of already discretized paths,
 * or just to produce a set of splines interpolating the path.
 * It will check the path's "resolution" setting and discretize if empty
 * or the resolution is greater than xtol.
 * 
 * Warning: the space and manifold members of the bezier paths will be bogus
 * pointers.  If you need to use them, you will need to set them to appropriate
 * RobotCSpace and RobotGeodesicManifold objects (see code for
 * GenerateAndTimeOptimizeMultiPath in RobotTimeScaling.cpp for an example).
 */
bool InterpolateConstrainedMultiPath(RobotModel& robot,const MultiPath& path,vector<GeneralizedCubicBezierSpline>& paths,Real xtol=1e-2);

/** @ingroup Planning
 * @brief Given a coarsely discretized multipath, produces a finely discretized
 * multipath that satisfies contact constraints.
 *
 * @sa InterpolateConstrainedMultiPath
 */
bool DiscretizeConstrainedMultiPath(RobotModel& robot,const MultiPath& path,MultiPath& out,Real xtol=1e-2);

/** @ingroup Planning
 * @brief Given a multipath, time-scales it to minimize execution time given the robot's
 * velocity and acceleration bounds.
 *
 * The geometric path is discretized with resolution xtol, and the time scaling is
 * discretized with resolution dt.
 */
bool GenerateAndTimeOptimizeMultiPath(RobotModel& robot,MultiPath& multipath,Real xtol,Real dt);

/** @ingroup Planning
 * @brief Evaluate the multipath at time t with a smooth interpolator, possibly
 * solving intermediate contact constraints.  If the path has no resolution, or
 * resolution > xtol, an IK solver is invoked to solve for the path's contact
 * constraints.
 */
void EvaluateMultiPath(RobotModel& robot,const MultiPath& multipath,Real t,Config& q,Real xtol=0,Real contactol=1e-3,int numIKIters=100);

} //namespace Klampt

#endif
