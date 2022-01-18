#ifndef HOLD_H
#define HOLD_H

#include <KrisLibrary/robotics/Contact.h>
#include <KrisLibrary/robotics/IK.h>

namespace Klampt {
  using namespace std;

/** @file Contact/Hold.h
 * @ingroup Contact
 * @brief Structures defining the finalized contacts used in stances.
 */

/** @addtogroup Contact */
/**@{*/


/** @brief A single contact between the robot and the environment.
 *
 * Consists of one or more contact points, along with the relative
 * position of the robot link with the environment.  Can be considered
 * a "finalized" version of a ContactFeatureMapping, where the
 * actual contact interface has been determined.
 * @sa ContactPoint
 * @sa ContactFeatureMapping
 */
struct Hold
{
  /// Transforms the hold by T
  void Transform(const RigidTransform& T);
  /// Returns the centroid of the contacts
  inline void GetCentroid(Vector3& c) const 
  {
    c.setZero();
    for(size_t i=0;i<contacts.size();i++)
      c += contacts[i].x;
    c/=(Real)contacts.size();
  }

  //Returns the average normal of the contacts
  inline void GetNormal(Vector3& n) const 
  {
    n.setZero();
    for(size_t j=0;j<contacts.size();j++)
      n += contacts[j].n;
    n.inplaceNormalize();
  }

  /// Same as above
  inline Vector3 GetCentroid() const {  Vector3 c; GetCentroid(c); return c; }
  inline Vector3 GetNormal() const { Vector3 n; GetNormal(n); return n; }

  /**
   * Given the current set of contact points, the IK constraint is defined
   * by the local position of the 0'th contact point and the
   * local-to-world rotation of the constrained link.
   * If 1 contact point, it's a free rotation about that point
   * 2, it's a constrained rotation around that axis
   * 3 or more, both position/rotation are constrained.
   *
   * @param localPos0 The local position of the 0'th contact point
   * @param rot A moment rotation @f$e^{[rot]}@f$ that represents the link's
   *        desired rotation.  Ignored if there is only 1 contact point.
   */
  void SetupIKConstraint(const Vector3& localPos0, const Vector3& rot);
  /**
   * Given the current set of contact points, the IK constraint is defined
   * by the local position of the 0'th contact point and the
   * local-to-world rotation of the constrained link.

   * @param localPos0 The local position of the 0'th contact point
   * @param rot The link's fixed rotation
   */
  void SetupFixedIKConstraint( const Vector3& localPos0, const Matrix3& rot);
  /**
   * Given the current set of contact points, the IK constraint is defined
   * by matching the local position of the 0'th contact point.

   * @param localPos0 The local position of the 0'th contact point
   */
  void SetupPointIKConstraint(const Vector3& localPos0);
  /**
   * Given the current set of contact points, the IK constraint is defined
   * by matching the local position of the 0'th contact point and the local
   * axis to the world axis (axis_w is usually parallel to
   * (contacts[1].x-contacts[0].x)

   * @param localPos0 The local position of the 0'th contact point
   * @param axis The local rotation axis
   * @param axis_w The world rotation axis
   */
  void SetupAxisIKConstraint( const Vector3& localPos0, const Vector3& axis, const Vector3& axis_w);

  int link;
  vector<ContactPoint> contacts;
  IKGoal ikConstraint;
};

bool operator == (const Hold& a,const Hold& b);
bool operator < (const Hold& a,const Hold& b);
bool operator > (const Hold& a,const Hold& b);
ostream& operator << (ostream&, const Hold&);
istream& operator >> (istream&, Hold&);

/**@}*/

} //namespace Klampt

#endif
