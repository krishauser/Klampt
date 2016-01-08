#ifndef GEOMETRY_DISTANCE_QUERY_H
#define GEOMETRY_DISTANCE_QUERY_H

#include <KrisLibrary/geometry/AnyGeometry.h>
using namespace Math3D;
using namespace Geometry;

/** @ingroup Planning
 * @brief A method for efficiently caching and updating distance
 * computations using temporal coherence.
 *
 * Suppose the transforms of the collision objects change.
 * Call NextStatus() to cycle the DistanceStatus.
 * UpdateQuery() returns the separation distance,
 *    (a negative number signifies penetration distance)
 * QueryClosestPoints() returns the closest points (or furthest
 *    penetrating points).  It will call UpdateQuery if needed.
 */
struct DistanceQuery
{
  enum Status { Far, Close, Contact, WasFar, WasClose, WasContact, Invalid };

  DistanceQuery();
  void NextCycle();
  Real UpdateQuery();
  bool ClosestPoints(Vector3& cp1, Vector3& cp2, Vector3& dir);

  Geometry::AnyCollisionQuery* query;
  Status s;

  Real distanceTolerance;
  Real distanceAbsErr;
  Real distanceRelErr;
};


#endif
