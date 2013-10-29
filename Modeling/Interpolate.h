#ifndef ROBOT_INTERPOLATE_H
#define ROBOT_INTERPOLATE_H

#include "Robot.h"

/** @brief Interpolates between the two configurations.
 */
void Interpolate(Robot& robot,const Config& x,const Config& y,Real u,Config& out);

/** @brief Returns the velocity vector that will move the robot from the
 * current configuration to 'dest' in minimal time. 
 */
void InterpolateDerivative(Robot& robot,const Config& a,const Config& b,Vector& dq);

/** @brief Integrates a velocity vector dq from q to obtain the configuration b
 */
void Integrate(Robot& robot,const Config& q,const Vector& dq,Config& b);

/** @brief Returns the geodesic distance between a and b.
 * Combines individual joint distances together via the L-k norm, 
 * where k = #norm.
 * 
 * For floating joints, the rotation error is multiplied by
 * floatingRotationWeight
 */
Real Distance(const Robot& robot,const Config& a,const Config& b,Real norm,Real floatingRotationWeight=1.0);

/** @brief Returns the geodesic distance between a and b.
 * Combines individual joint distances together via the weighted L-k norm, 
 * where k = #norm.
 * 
 * For floating joints, the rotation error is multiplied by
 * floatingRotationWeight
 */
Real Distance(const Robot& robot,const Config& a,const Config& b,Real norm,const vector<Real>& weights,Real floatingRotationWeight=1.0);

#endif
