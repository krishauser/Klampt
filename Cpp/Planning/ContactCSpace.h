#ifndef ROBOT_CONTACT_CSPACE_H
#define ROBOT_CONTACT_CSPACE_H

#include "RobotCSpace.h"
#include <Klampt/Modeling/GeneralizedRobot.h>
#include <Klampt/Contact/Stance.h>
#include <KrisLibrary/robotics/IK.h>

namespace Klampt {

/** @brief A SingleRobotCSpace for a robot maintaining contact.
 *
 * The loop closure constraints are solved for using the Newton Raphson method
 * during sampling and interpolation.
 *
 * Note: interpolation is NOT geodesic, so GeodesicCSpace methods should not be
 * used.
 */
class ContactCSpace : public SingleRobotCSpace
{
 public:
  ContactCSpace(WorldModel& world,int index,
		WorldPlannerSettings* settings);
  ContactCSpace(const SingleRobotCSpace& space);
  ContactCSpace(const ContactCSpace& space);

  virtual void Sample(Config& q) override;
  virtual void SampleNeighborhood(const Config& c,Real r,Config& q) override;
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out) override;
  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b) override;
  virtual void Properties(PropertyMap&) override;

  void AddContact(const IKGoal& goal);
  void AddContact(int link,const Vector3& localPos,const Vector3& worldPos);
  void AddContact(int link,const vector<Vector3>& localPos,const vector<Vector3>& worldPos);
  void RemoveContact(int link);
  bool SolveContact(int numIters=0,Real tol=0);
  ///Returns the max error on the contact constraints at the robot's current
  ///configuration.
  Real ContactDistance();
  ///Same as ContactDistance(). Note: sets the robot's configuration to q.
  Real ContactDistance(const Config& q);
  ///Checks whether the contact constraints are satisfied at the robot's
  //current configuration.  If dist is nonzero, this overrides the defaults
  ///in the WorldPlannerSettings
  bool CheckContact(Real dist=0);
  ///Same as CheckContact(Real). Note: sets the robot's configuration to q.
  bool CheckContact(const Config& q,Real dist=0);

  vector<IKGoal> contactIK;
  int numSolveContact;
  double solveContactTime;
};

/*
class MultiContactCSpace : public MultiRobotCSpace
{
 public:
  struct ContactPair
  {
    int id1,id2;            //world id of the given object
    vector<Vector3> c1,c2;  //cx = contacts in local frame of object x
    vector<Vector3> n1,n2;  //nx = normals in local frame  of object x
    vector<Real> kFriction;
  };

  MultiContactCSpace(WorldModel& world,
		     WorldPlannerSettings* settings);
  MultiContactCSpace(const MultiRobotCSpace&);
  MultiContactCSpace(const MultiContactCSpace&);
  virtual ~MultiContactCSpace() {}
  ///Initializes all internal structures
  void InitContactPairs(const vector<ContactPair>& pairs);
  ///Same as above, but with different structure.  Here all the indexes in the formation are World ids.
  void InitContactPairs(const ContactFormation& formation);

  virtual int NumDimensions() const override;
  virtual void Sample(Config& x) override;
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x) override;
  virtual bool IsFeasible(const Config&) override;
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out) override;
  virtual void Midpoint(const Config& x,const Config& y,Config& out) override;
  virtual void Properties(PropertyMap&) override;

  bool SolveContact(Config& x,int numIters=0,Real tol=0);
  Real ContactDistance();
  Real ContactDistance(const Config& x);
  bool CheckContact(Real dist=0);
  bool CheckContact(const Config& x,Real dist=0);

  vector<ContactPair> contactPairs;
  Robot aggregateRobot;
  vector<IKGoal> closedChainConstraints;
  Stance aggregateStance;

  //stats
  int numSolveContact,numIsFeasible;
  double solveContactTime,isFeasibleTime;
};
*/

} // namespace Klampt

#endif
