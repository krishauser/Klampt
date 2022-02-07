#ifndef CONTACT_FEATURE_MAPPING_H
#define CONTACT_FEATURE_MAPPING_H

#include "Hold.h"
#include "ContactFeature.h"

/** @file ContactFeatureMapping.h
 * @ingroup Contact
 * @brief Defines the ContactFeatureMapping structure, load/save functions.
 */

/** @addtogroup Contact */
/*@{*/

namespace Klampt {

/** @brief A mapping from a ContactFeature to a point on the environment.
 */
struct ContactFeatureMapping
{
  ContactFeatureMapping();

  /// Returns the hold that maps the feature to the contact point,
  /// with the local orientation given in angle (and wheelRoll)
  void GetHold(Hold& h) const;

  /// Returns a frame that maps the link to the world position.
  /// Used for drawing functions.  For non-fixed holds, returns
  /// one of the many possible transforms.
  void GetLocalFrame(RigidTransform& T) const;

  ContactFeature feature;
  ContactPoint contact;
  /// for non-point holds, this describes the orientation of the hold
  /// relative to the surface.
  Real angle;
  /// for wheel holds, this marks whether to fix the roll of the
  /// wheel or not.
  bool fixedWheel;
  /// for fixed wheels, this is the roll angle at which the wheel is fixed
  Real wheelRoll;
  /// for face and edge contacts, this is the amount that the contact
  /// point on the robot is offset from the contact region centroid
  Vector3 localOffset;
};

bool LoadContactFeatureMapping(istream& in,const vector<ContactFeature>& features,ContactFeatureMapping& m);
void SaveContactFeatureMapping(ostream& out,ContactFeatureMapping& m);

/** @brief Constructs a contact feature mapping from a hold by trying
 * to match contact features to the hold.
 *
 * The matching criterion is that the hold IK constraint matches the 
 * contact feature's, and the hold contact points are approximately
 * within in the contact feature.
 * Returns true if successful.
 */
bool FeatureMappingFromHold(const Hold& h,const vector<ContactFeature>& features, ContactFeatureMapping& m);

/// Returns the angle parameter that makes the feature map to Rdes
Real BestFeatureMappingAngle(const ContactFeatureMapping& feature,const Matrix3& Rdes);

/// Sets the angle parameters that make the feature map to Rdes
void SetFeatureMappingOrientation(ContactFeatureMapping& feature,const Matrix3& Rdes);

/*@}*/

} //namespace Klampt

#endif
