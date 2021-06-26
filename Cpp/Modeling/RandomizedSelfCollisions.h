#ifndef RANDOMIZED_SELF_COLLISIONS_H
#define RANDOMIZED_SELF_COLLISIONS_H

#include <KrisLibrary/structs/array2d.h>
#include "Modeling/Robot.h"

namespace Klampt {

/** @file RandomizedSelfCollisions.h
 * @ingroup Modeling
 * @brief Helper functions for computing potentially colliding pairs of links
 * using random sampling.
 */

/** @addtogroup Modeling */
/*@{*/

/** @brief Calculates a bit-matrix of potential collision pairs using random
 * sampling.
 *
 * Sets collision(i,j) = true iff a collision between link i and j has been
 * detected within numSamples samples.
 */
void RandomizedSelfCollisionPairs(RobotWithGeometry& robot,Array2D<bool>& collision,int numSamples);


/** @brief Calculates the bit-matrix of potential independent collision pairs.
 *
 * Sets collision(i,j) = true iff a collision between link i and j has been
 * detected within numSamples samples.  And independent means they have been
 * detected to occur independently of any other pair.
 */
void RandomizedIndependentSelfCollisionPairs(RobotWithGeometry& robot,Array2D<bool>& collision,int numSamples);

/** @brief Calculates the min/max distance matrix of collision pairs.
 *
 * Sets min/maxDistance(i,j) to the min/max distance between bodies i and j
 * within numSamples samples.
 */
void RandomizedSelfCollisionDistances(RobotWithGeometry& robot,Array2D<Real>& minDistance,Array2D<Real>& maxDistance,int numSamples);

/*@}*/

} //namespace Klampt

#endif
