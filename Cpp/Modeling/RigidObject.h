#ifndef KLAMPT_RIGID_OBJECT_H
#define KLAMPT_RIGID_OBJECT_H

#include <KrisLibrary/geometry/AnyGeometry.h>
#include <string>
#include "ManagedGeometry.h"

namespace Klampt {
  using namespace std;
  using namespace Math3D;

/** @ingroup Modeling
 * @brief A (static) rigid object that may be manipulated.
 */
class RigidObjectModel
{
public:
  RigidObjectModel();
  ///Loads from a .obj file.  If libcurl is available, can load from a URL
  bool Load(const char* fn);
  bool Save(const char* fn);
  ///Supports anything the AnyGeometry class uses, and also
  ///ROS PointCloud2 topics (use prefix ros://[topic_name] or
  ///ros:PointCloud2/[topic_name]).
  ///
  ///If libcurl is available, can load from a URL
  bool LoadGeometry(const char* fn);
  void SetMassFromGeometry(Real totalMass,Real surfaceFraction=1.0);
  void SetMassFromBB(Real totalMass);
  void InitCollisions();
  void UpdateGeometry(); ///< Call this before collision detection if the transform is changed
  void DrawGL();
  void DrawGLOpaque(bool opaque);

  string name;
  string geomFile;
  ManagedGeometry geometry;
  RigidTransform T;    //current transform
  Vector3 w,v;         //current angular velocity and velocity
  Real mass;           //mass
  Vector3 com;         //center of mass in local frame
  Matrix3 inertia;     //inertia matrix about com
  Real kFriction;
  Real kRestitution;
  Real kStiffness,kDamping;
};

} //namespace Klampt

#endif
