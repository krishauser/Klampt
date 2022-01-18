#ifndef TRIANGLE_SAMPLER_H
#define TRIANGLE_SAMPLER_H

#include <KrisLibrary/math3d/Triangle2D.h>
#include <KrisLibrary/math3d/Triangle3D.h>
#include <vector>
#include <KrisLibrary/math/sample.h>

namespace Klampt {
  using namespace std;
  using namespace Math3D;

/** @ingroup Geometry
 * @brief Samples points in a list of 2d triangles.
 *
 * Call InitAreas() before sampling.
 */
struct Triangle2DSampler
{
  void InitAreas();
  void Clear() { tris.clear(); areas.clear(); sumAreas.clear(); }
  inline Real TotalArea() const { return sumAreas.back(); }
  inline int SampleTri() const { return CumulativeWeightedSample(sumAreas); }
  void SamplePointOnTri(int tri,Vector2& pt) const;
  void SamplePoint(Vector2& pt) const;
  void SamplePoints(int num,std::vector<Vector2>& pts) const;
  void SamplePoints(int num,std::vector<int>& tris,std::vector<Vector2>& pts) const;
  
  std::vector<Triangle2D> tris;
  std::vector<Real> areas;
  std::vector<Real> sumAreas;
};

/** @ingroup Geometry
 * @brief Samples points in a list of 3d triangles. 
 *
 * Call InitAreas() before sampling.
 */
struct Triangle3DSampler
{
  void InitAreas();
  void Clear() { tris.clear(); areas.clear(); sumAreas.clear(); }
  inline Real TotalArea() const { return sumAreas.back(); }
  inline int SampleTri() const { return CumulativeWeightedSample(sumAreas); }
  void SamplePointOnTri(int tri,Vector3& pt) const;
  void SamplePoint(Vector3& pt) const;
  void SamplePoints(int num,std::vector<Vector3>& pts) const;
  void SamplePoints(int num,std::vector<int>& tris,std::vector<Vector3>& pts) const;
  
  std::vector<Triangle3D> tris;
  std::vector<Real> areas;
  std::vector<Real> sumAreas;
};

} //namespace Klampt

#endif

