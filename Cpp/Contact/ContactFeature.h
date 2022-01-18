#ifndef CONTACT_FEATURE_H
#define CONTACT_FEATURE_H

#include "Hold.h"
#include <KrisLibrary/math3d/Polygon3D.h>
#include <KrisLibrary/math3d/geometry3d.h>
#include <memory>

namespace Klampt {

struct Triangle3DSampler;

/** @defgroup Contact
 * Contact points, holds, stances, and grasps
 */

/** @file ContactFeature.h
 * @ingroup Contact
 * @brief Defines the ContactFeature structures and some
 *  routines for creating holds from features.
 */

/** @addtogroup Contact */
/*@{*/

/** @brief A feature on the robot that can be used for contact.
 *
 * Abstract base class. The GetType() function allows querying
 * the dynamic type for type conversion.  Be careful to cast
 * to the right type!
 */
struct ContactFeatureBase
{
  enum Type { Point, Edge, Face, MultipleFaces, Wheel };

  ContactFeatureBase() :link(-1),weight(1) {}
  virtual ~ContactFeatureBase() {}
  virtual Type GetType() const =0;

  int link;
  string name;
  Real weight;   ///< weight for picking this feature
};

///A single point contact feature.
struct PointContactFeature : public ContactFeatureBase
{
  virtual Type GetType() const { return Point; }

  /** @brief Makes the hold that matches p to x.
   *
   * Normals and friction are set to zero.
   */
  void GetHold(const Vector3& x,Hold&) const;

  Vector3 p;
};

///An edge contact feature.
struct EdgeContactFeature : public ContactFeatureBase
{
  virtual Type GetType() const { return Edge; }

  /** @brief Makes the hold that matches p1 to x, and
   * the segment (p1,p2) line up to axis.
   *
   * Normals and friction are set to zero.
   */
  void GetHold(const Vector3& x,const Vector3& axis,Hold&) const;

  /** @brief Makes the hold that matches p1 to x, and
   * orients the segment (p1,p2) to be orthogonal to n, and
   * then rotates around n by the angle theta.
   *
   * Normals are set to n.  Friction is set to zero.
   */
  void GetHold(const Vector3& x,const Vector3& n,Real theta,Hold&) const;

  Vector3 p1,p2;
};

/** @brief A (planar) face contact feature.
 *
 * Polygon is assumed to be planar and counter-clockwise oriented around
 * the normal direction (e.g. if the face faces up, the vertices are CCW
 * looking downward)
 */
struct FaceContactFeature : public ContactFeatureBase, Polygon3D
{
  virtual Type GetType() const { return Face; }

  ///normal is set to be pointing in the outward direction
  void GetPlane(Plane3D& p) const;

  void GetCentroid(Vector3& p) const;

  /** @brief Makes the hold that matches cp[0] to x.
   *
   * First orients the feature's plane to match up against
   * the normal n.  Then, rotates the feature by angle theta
   * around the normal.
   *
   * Normals are set to n, friction is set to 0.
   */
  void GetHold(const Vector3& x,const Vector3& n,Real theta,Hold&) const;
  ///Same as above, but matches localPos to x
  void GetHold(const Vector3& localPos,const Vector3& x,const Vector3& n,Real theta,Hold&) const;
};

///A wheel contact feature.
struct WheelContactFeature : public ContactFeatureBase
{
  virtual Type GetType() const { return Wheel; }

  /// Returns the center of the wheel
  Vector3 GetCenter() const { Vector3 center; axis.closestPoint(p,center); return center; }
  /// Returns the radius of the wheel
  Real GetRadius() const { return axis.distance(p); }

  /** @brief Makes the hold that matches the center of the wheel to x.
   *
   * First orients the wheel to be tangent to x at the normal n.
   * Then, rotates the wheel about angle theta around the normal.
   * Allows the wheel to roll about the axis
   *
   * Normals are set to n, friction is set to 0.
   */
  void GetHold(const Vector3& x,const Vector3& n,Real theta,Hold&) const;

  /** @brief Makes the fixed-orientation hold that matches the center 
   * of the wheel to x, and making an rolling angle of roll.
   *
   * First orients the wheel to be tangent to x at the normal n.
   * Then, rotates the wheel about angle theta around the normal.
   * Finally, fixes the wheel with the rolling angle roll.
   *
   * Normals are set to n, friction is set to 0.
   */
  void GetFixedHold(const Vector3& x,const Vector3& n,Real theta,Real roll,Hold&) const;

  Line3D axis;
  Vector3 p;
  Real width;   //wheel's extent along the axis 
};

/** @brief A contact feature consisting of multiple faces.
 *
 * Polygon is assumed to be planar and counter-clockwise oriented around
 * the normal direction (e.g. if the face faces up, the vertices are CCW
 * looking downward)
 */
struct FacesContactFeature : public ContactFeatureBase
{
  virtual Type GetType() const { return MultipleFaces; }

  //TODO: get the holds

  vector<Polygon3D> faces;
};

/// A smart pointer to a ContactFeatureBase -- use GetType() to cast
/// to the right subtype.
typedef std::shared_ptr<ContactFeatureBase> ContactFeature;

bool LoadContactFeatures(const char* fn,vector<ContactFeature>& ccs);
bool SaveContactFeatures(const char* fn,const vector<ContactFeature>& ccs);

void SampleHold(const Vector3& x,const Vector3& n,const ContactFeatureBase*,Hold&);
void SampleHold(const Vector3& x,const Vector3& n,const vector<ContactFeature>&,Hold&);
void SampleHold(Triangle3DSampler& smp,const ContactFeatureBase*,Hold&);
void SampleHold(Triangle3DSampler& smp,const vector<ContactFeature>&,Hold&);

/// Computes a hold that transforms the contact feature by T
void HoldFromTransform(const ContactFeatureBase* f,const RigidTransform& T,Hold& h);

/// Returns true if the ray intersects the contact feature
bool FeatureRayIntersection(const ContactFeatureBase* f,const Ray3D& ray,Real tol=0);

/// Returns the centroid of the feature contact region
Vector3 FeatureContactPoint(const ContactFeatureBase* f);
/// Returns the local normal that is mapped to the environment contact normal.
/// The zero vector is returned if there is no such normal defined
/// (e.g. point and edge features)
Vector3 FeatureContactNormal(const ContactFeatureBase* f);
/// Uniformly samples a contact point in the feature contact region
Vector3 SampleFeatureContactPoint(const ContactFeatureBase* f);

/*@}*/

} //namespace Klampt

#endif
