#ifndef ROBOT_CONTACT_CSPACE_H
#define ROBOT_CONTACT_CSPACE_H

#include "RobotCSpace.h"
#include <robotics/IK.h>

/** @brief A SingleRobotCSpace for a robot maintaining contact.
 *
 * The loop closure constraints are solved for using the Newton Raphson method.
 *
 * Note that the current implementation doesn't override any ExplicitCSpace
 * methods, so this should not be used in a planner that expects an
 * ExplicitCSpace.
 */
class ContactCSpace : public SingleRobotCSpace2
{
 public:
  ContactCSpace(RobotWorld& world,int index,
		WorldPlannerSettings* settings);
  ContactCSpace(const SingleRobotCSpace& space);
  ContactCSpace(const ContactCSpace& space);
  virtual ~ContactCSpace() {}
  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual bool IsFeasible(const Config&);

  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out);
  virtual void Midpoint(const Config& x,const Config& y,Config& out);

  void AddContact(const IKGoal& goal);
  void AddContact(int link,const Vector3& localPos,const Vector3& worldPos);
  void AddContact(int link,const vector<Vector3>& localPos,const vector<Vector3>& worldPos);
  void RemoveContact(int link);
  bool SolveContact(int numIters=0,Real tol=0);
  Real ContactDistance();
  bool CheckContact(Real dist=0);

  vector<IKGoal> contactIK;
};

class MultiContactCSpace : public MultiRobotCSpace
{
 public:
  struct ContactPair
  {
    int id1,id2;            //world id of the given object
    vector<Vector3> c1,c2;  //cx = contacts in local frame of object x
    vector<Vector3> n1,n2;  //nx = normals in local frame  of object x
  };

  MultiContactCSpace(RobotWorld& world,
		     WorldPlannerSettings* settings);
  MultiContactCSpace(const MultiRobotCSpace&);
  MultiContactCSpace(const MultiContactCSpace&);
  virtual ~MultiContactCSpace() {}
  void InitContactPairs(const vector<ContactPair>& pairs);

  virtual int NumDimensions() const;
  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x);
  virtual bool IsFeasible(const Config&);

  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out);
  virtual void Midpoint(const Config& x,const Config& y,Config& out);

  bool SolveContact(Config& x,int numIters=0,Real tol=0);
  Real ContactDistance(const Config& x);
  bool CheckContact(const Config& x,Real dist=0);

  void SplitRefs(const Config& x,vector<Config>& robotConfigs,vector<Config>& objectConfigs) const;
  void SetWorldConfig(const Config& x);
  void GetWorldConfig(Config& x);

  vector<ContactPair> contactPairs;
  vector<int> activeIDs;  //IDs of robots/objects used in the aggregate robot
  vector<bool> robotActive;   //true if the robot is active
  vector<bool> objectActive;  //true if the object is active

  RobotKinematics3D aggregateRobot;
  vector<IKGoal> closedChainConstraints;
};

#endif
