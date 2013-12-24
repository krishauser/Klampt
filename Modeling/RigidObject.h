#ifndef RIGID_OBJECT_H
#define RIGID_OBJECT_H

#include <geometry/AnyGeometry.h>
using namespace Math3D;

/** @ingroup Modeling
 * @brief A (static) rigid object that may be manipulated.
 */
struct RigidObject
{
  bool Load(const char* fn);
  bool Save(const char* fn);
  void SetMassFromGeometry(Real totalMass);
  void SetMassFromBB(Real totalMass);
  void UpdateGeometry(); ///< Call this before collision detection if the transform is changed

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
