#ifndef GRASP_H
#define GRASP_H

#include <KrisLibrary/robotics/Contact.h>
#include <KrisLibrary/robotics/IK.h>
#include "Hold.h"
#include "Stance.h"
#include <vector>
class TiXmlElement;

namespace Klampt {

/** @ingroup Contact
 * @brief Slightly more sophisticated than a Stance, a Grasp allows
 * some of the robot's degrees of freedom to be fixed.
 */
class Grasp
{
 public:
  Grasp();

  ///Returns the number of constraints touching the object
  inline size_t NumConstraints() const { return constraints.size(); }

  ///A grasp is simple if it's a rigid attachment to a single robot link
  inline bool IsSimple() const
  {
    return constraints.size()==1 && IKGoal::NumDims(constraints[0].posConstraint)+IKGoal::NumDims(constraints[0].rotConstraint) == 6;
  }

  void Add(const Grasp& grasp);
  void SetHold(const Hold& h);
  void SetStance(const Stance& h);
  void GetHold(Hold& h) const;
  void GetStance(Stance& s) const;
  void GetContactFormation(ContactFormation& s) const;
  void Transform(const RigidTransform& T);
  void SetFixed(Vector& q) const;

  bool Load(TiXmlElement* e);
  bool Save(TiXmlElement* e);

  ///index of the robot and object -- object may be -1
  int objectIndex,robotIndex;

  ///describes constraints on robot configuration relative to object
  vector<IKGoal> constraints;
  vector<Real> fixedValues;
  vector<int> fixedDofs;

  ///optional: describes contact state
  vector<ContactPoint> contacts;
  vector<int> contactLinks;
  vector<Vector3> forces;
};

} //namespace Klampt

#endif
