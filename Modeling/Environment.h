#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <geometry/AnyGeometry.h>
#include <vector>
using namespace std;
using namespace Math;

/** @ingroup Modeling
 * @brief A model of a static environment with known friction.
 */
struct Environment
{
  bool Load(const char* fn);
  bool Save(const char* fn);
  ///Can be called optionally to get better debug information about 
  ///long collision initialization times, rather than using dynamic
  ///initialization
  void InitCollisions();
  inline void SetUniformFriction(Real mu) {
    kFriction.resize(geometry.NumElements());
    fill(kFriction.begin(),kFriction.end(),mu);
  }

  string geomFile;
  Geometry::AnyCollisionGeometry3D geometry;
  vector<Real> kFriction;       //per element friction
};

#endif
