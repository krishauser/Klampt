#ifndef ROBOT_INTERPOLATE_H
#define ROBOT_INTERPOLATE_H

#include "Robot.h"

namespace Klampt {

/** @addtogroup Modeling */
/*@{*/

/** @brief Interpolates between the two configurations in geodesic fashion 
 * on the robot's underlying configuration space.
 */
void Interpolate(RobotModel& robot,const Config& x,const Config& y,Real u,Config& out);

/** @brief Returns the velocity vector at a that will move the robot from 
 * a to b in minimal time (i.e., derivative of the geodesic).
 */
void InterpolateDerivative(RobotModel& robot,const Config& a,const Config& b,Vector& dq);

/** @brief Returns the velocity vector that will move the robot from the
 * configuration that is u fraction of the way from a to b to b in minimal time. 
 */
void InterpolateDerivative(RobotModel& robot,const Config& a,const Config& b,Real u,Vector& dq);


/** @brief Integrates a velocity vector dq from q to obtain the configuration b
 */
void Integrate(RobotModel& robot,const Config& q,const Vector& dq,Config& b);

/** @brief Returns the geodesic distance between a and b.
 * Combines individual joint distances together via the L-k norm, 
 * where k = #norm.
 * 
 * For floating joints, the rotation error is multiplied by
 * floatingRotationWeight
 */
Real Distance(const RobotModel& robot,const Config& a,const Config& b,Real norm,Real floatingRotationWeight=1.0);

/** @brief Returns the geodesic distance between a and b.
 * Combines individual joint distances together via the weighted L-k norm, 
 * where k = #norm.
 * 
 * For floating joints, the rotation error is multiplied by
 * floatingRotationWeight
 */
Real Distance(const RobotModel& robot,const Config& a,const Config& b,Real norm,const vector<Real>& weights,Real floatingRotationWeight=1.0);

/*@}*/

} //namespace Klampt

#endif
