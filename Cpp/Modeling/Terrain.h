#ifndef TERRAIN_H
#define TERRAIN_H

#include "ManagedGeometry.h"
#include <vector>
#include <string>

namespace Klampt {
  using namespace std;
  using namespace Math;

/** @ingroup Modeling
 * @brief A model of a static terrain with known friction.
 */
class TerrainModel
{
public:
  ///Can support .env files, anything the AnyGeometry class uses, and
  ///also ROS PointCloud2 topics (use prefix ros://[topic_name] or
  ///ros:PointCloud2/[topic_name])
  bool Load(const char* fn);
  bool Save(const char* fn);
  ///Loads just the geometry of the terrain
  bool LoadGeometry(const char* fn);
  ///Can be called optionally to get better debug information about 
  ///long collision initialization times, rather than using dynamic
  ///initialization
  void InitCollisions();
  inline void SetUniformFriction(Real mu) {
    kFriction.resize(geometry->NumElements());
    fill(kFriction.begin(),kFriction.end(),mu);
  }
  ///Renders the terrain in OpenGL
  void DrawGL();
  void DrawGLOpaque(bool opaque);

  string name;
  string geomFile;
  ManagedGeometry geometry;
  vector<Real> kFriction;       //per element friction
};

} //namespace Klampt

#endif
