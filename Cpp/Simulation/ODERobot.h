#ifndef ODE_INTERFACE_ROBOT_H
#define ODE_INTERFACE_ROBOT_H

#include <Klampt/Modeling/Robot.h>
#include "ODEGeometry.h"
#include "ODESurface.h"
#include <ode/common.h>
#include <KrisLibrary/File.h>

namespace Klampt {
  using namespace Math;
  using namespace std;

/** @ingroup Simulation
 * @brief A robot simulated in an ODE "world"
 *
 * Collision detection
 * geom is a list of ODEGeometry objects.  The ODE "geom" of geom[i] contains
 *    user data = (void*)body_index.  This can be used for collision
 *    identification.
 * spaceID is a pointer to an ODE "space" containing all the robot's geometry.
 *
 * If self-collisions should be tested, the EnableSelfCollisions function
 * must be called first. It appears that the space has to be a
 * dSimpleSpace which is somewhat less efficient than the default dHashSpace.
 *
 * Note: if you manually set the robot's velocities, make sure to disable
 * instability correction in the simulator for that time step.
 */
class ODERobot
{
 public:
  static double defaultPadding;
  static ODESurfaceProperties defaultSurface;

  ODERobot(RobotModel& robot);
  ~ODERobot();
  void Create(int index,dWorldID worldID,bool useBoundaryLayer=true);
  void Clear();
  void EnableSelfCollisions(bool enabled);
  bool SelfCollisionsEnabled() const;
  void SetConfig(const Config& q);
  void GetConfig(Config& q) const;
  void SetVelocities(const Config& dq);
  void GetVelocities(Config& dq) const;
  void AddTorques(const Vector& t);
  Real GetJointAngle(int joint) const;
  Real GetJointVelocity(int joint) const;
  void AddJointTorque(int joint,Real t);
  void SetJointDryFriction(int joint,Real coeff);
  void SetJointFixedVelocity(int joint,Real vel,Real tmax);
  Real GetLinkAngle(int link) const;
  Real GetLinkVelocity(int link) const;
  void AddLinkTorque(int link,Real t);
  void SetLinkDryFriction(int link,Real coeff);
  void SetLinkFixedVelocity(int link,Real vel,Real tmax);
  Real GetDriverValue(int driver) const;
  Real GetDriverVelocity(int driver) const;
  void AddDriverTorques(const Vector& t);
  void AddDriverTorque(int driver,Real t);
  void SetDriverFixedVelocity(int driver,Real vel,Real tmax);

  void SetLinkTransform(int link,const RigidTransform& T);
  void GetLinkTransform(int link,RigidTransform& T) const;
  void SetLinkVelocity(int link,const Vector3& w,const Vector3& v);
  void GetLinkVelocity(int link,Vector3& w,Vector3& v) const;
  Real GetKineticEnergy() const;
  Real GetKineticEnergy(int link) const;
  bool ReadState(File& f);
  bool WriteState(File& f) const;

  inline dSpaceID space() const { return spaceID; }
  inline dBodyID body(int link) const { return bodyID[link]; }
  inline dGeomID geom(int link) const { return geometry[link]->geom(); }
  inline dJointID joint(int link) const { return jointID[link]; }
  inline ODEGeometry* triMesh(int link) const { return geometry[link]; }
  dBodyID baseBody(int link) const;  //for attached links, returns the base body for the link
  inline dJointFeedback feedback(int link) const { return jointFeedback[link]; }

  RobotModel& robot;

 private:
  vector<RigidTransform> T_bodyToLink;
  vector<dBodyID> bodyID;
  vector<ODEGeometry*> geometry;
  vector<dJointID> jointID;
  vector<dJointFeedback> jointFeedback;
  dJointGroupID jointGroupID;
  dSpaceID spaceID;
  vector<shared_ptr<RobotWithGeometry::CollisionGeometry> > tempGeometries;
};

} //namespace Klampt

#endif
