#ifndef RIGID_OBJECT_H
#define RIGID_OBJECT_H

#include <KrisLibrary/geometry/AnyGeometry.h>
#include <string>
#include "ManagedGeometry.h"
using namespace std;
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
  ///ros:PointCloud2/[topic_name])
  bool LoadGeometry(const char* fn);
  void SetMassFromGeometry(Real totalMass);
  void SetMassFromBB(Real totalMass);
  void InitCollisions();
  void UpdateGeometry(); ///< Call this before collision detection if the transform is changed
  void DrawGL();

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

#endif
