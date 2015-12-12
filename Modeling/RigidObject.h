#ifndef RIGID_OBJECT_H
#define RIGID_OBJECT_H

#include <geometry/AnyGeometry.h>
using namespace Math3D;

/** @ingroup Modeling
 * @brief A (static) rigid object that may be manipulated.
 */
class RigidObject
{
public:
  RigidObject();
  bool Load(const char* fn);
  bool Save(const char* fn);
  ///Supports anything the AnyGeometry class uses, and also
  ///ROS PointCloud2 topics (use prefix ros://[topic_name] or
  ///ros://PointCloud/[topic_name])
  bool LoadGeometry(const char* fn);
  void SetMassFromGeometry(Real totalMass);
  void SetMassFromBB(Real totalMass);
  void InitCollisions();
  void UpdateGeometry(); ///< Call this before collision detection if the transform is changed

  std::string geomFile;
  Geometry::AnyCollisionGeometry3D geometry;
  RigidTransform T;    //current transform
  Real mass;           //mass
  Vector3 com;         //center of mass in local frame
  Matrix3 inertia;     //inertia matrix about com
  Real kFriction;
  Real kRestitution;
  Real kStiffness,kDamping;
};

#endif
