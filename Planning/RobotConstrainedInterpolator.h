#ifndef ROBOT_CONSTRAINED_INTERPOLATOR_H
#define ROBOT_CONSTRAINED_INTERPOLATOR_H

#include "ConstrainedInterpolator.h"
#include "RobotCSpace.h"
#include <robotics/IKFunctions.h>


/** @ingroup Planning
 * @brief Just like a ConstrainedInterpolator but only projects the active
 * DOFs.  Much faster for high-DOF system with sparse constraints.
 */
class RobotConstrainedInterpolator : public ConstrainedInterpolator
{
public:
  RobotConstrainedInterpolator(Robot& robot,const vector<IKGoal>& goals);
  virtual void ConstraintValue(const Config& x,Vector& v);
  virtual bool Project(Config& x);

  RobotCSpace space;
  RobotGeodesicManifold manifold;
  RobotIKFunction f;
};

/** @ingroup Planning
 * @brief Just like a SmoothConstrainedInterpolator but only projects the
 * active DOFs.  Much faster for high-DOF system with sparse constraints.
 */
class RobotSmoothConstrainedInterpolator : public SmoothConstrainedInterpolator
{
public:
  RobotSmoothConstrainedInterpolator(Robot& robot,const vector<IKGoal>& goals);
  virtual void ConstraintValue(const Config& x,Vector& v);
  virtual bool Project(Config& x);
  virtual bool ProjectVelocity(const Config& x,Vector& v);

  RobotCSpace space;
  RobotGeodesicManifold manifold;
  RobotIKFunction f;
};

#endif
