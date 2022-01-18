#ifndef POLYGON_SAMPLER_H
#define POLYGON_SAMPLER_H

#include <KrisLibrary/geometry/rayprimitives.h>
#include "TriangleSampler.h"

namespace Klampt {
  using Geometry::PointRay2D;

/** @ingroup Geometry
 * @brief Samples points in a convex polygon. 
 *
 * Allows unbounded rays, which are capped at some distance and the
 * resulting polygon is triangulated.
 */
class Polygon2DSampler : protected Triangle2DSampler
{
public:
  void Set(const vector<Vector2>& poly);
  void Set(const vector<PointRay2D>& poly,Real rayBound);
  void Sample(Vector2& x) const { return Triangle2DSampler::SamplePoint(x); }
  bool IsEmpty() const { return tris.size()==0; }
  void Clear() { Triangle2DSampler::Clear(); }
};

/** @ingroup Geometry
 * @brief Samples points in a convex polygon (in 3D). 
 */
class Polygon3DSampler : protected Triangle3DSampler
{
public:
  void Set(const vector<Vector3>& poly);
  void Sample(Vector3& x) const { return Triangle3DSampler::SamplePoint(x); }
  bool IsEmpty() const { return tris.size()==0; }
  void Clear() { Triangle3DSampler::Clear(); }
};

} //namespace Klampt

#endif
