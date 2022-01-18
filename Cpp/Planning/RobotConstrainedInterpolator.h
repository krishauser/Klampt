#ifndef ROBOT_CONSTRAINED_INTERPOLATOR_H
#define ROBOT_CONSTRAINED_INTERPOLATOR_H

#include "ConstrainedInterpolator.h"
#include "RobotCSpace.h"
#include <KrisLibrary/robotics/IKFunctions.h>

namespace Klampt {

/** @ingroup Planning
 * @brief Just like a ConstrainedInterpolator but only projects the active
 * DOFs.  Much faster for high-DOF system with sparse constraints.
 */
class RobotConstrainedInterpolator : public ConstrainedInterpolator
{
public:
  RobotConstrainedInterpolator(RobotModel& robot,const vector<IKGoal>& goals);
  virtual void ConstraintValue(const Config& x,Vector& v) override;
  virtual bool Project(Config& x) override;

  RobotCSpace space;
  RobotIKFunction f;
};

/** @ingroup Planning
 * @brief Just like a SmoothConstrainedInterpolator but only projects the
 * active DOFs.  Much faster for high-DOF system with sparse constraints.
 */
class RobotSmoothConstrainedInterpolator : public SmoothConstrainedInterpolator
{
public:
  RobotSmoothConstrainedInterpolator(RobotModel& robot,const vector<IKGoal>& goals);
  virtual void ConstraintValue(const Config& x,Vector& v) override;
  virtual bool Project(Config& x) override;
  virtual bool ProjectVelocity(const Config& x,Vector& v) override;

  RobotCSpace space;
  RobotIKFunction f;
};

} //namespace Klampt

#endif
