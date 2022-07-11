#ifndef MODELING_ROBOT_H
#define MODELING_ROBOT_H

#include <KrisLibrary/robotics/RobotWithGeometry.h>
#include <KrisLibrary/utils/PropertyMap.h>
#include "ManagedGeometry.h"

//forward declaration
class RobotIKFunction;

namespace Klampt {
  using namespace std;

/** @ingroup Modeling 
 * @brief Additional joint properties 
 */
struct RobotModelJoint
{
  /** Types:
    - Weld: completely fixed to the parent
    - Normal: regular fixed axis joint with finite range
    - Spin: infinitely spinnable rotational joint
    - Floating: free-floating base
    - FloatingPlanar: a free-floating 2D base
    - BallAndSocket: ball and socket joint
    - Closed: a closed chain loop
  */
  enum Type { Weld, Normal, Spin, Floating, FloatingPlanar, BallAndSocket, Closed };

  ///The type of this joint
  Type type;
  ///The affected link.  For Floating/BallAndSocket joints, the last
  ///link in the chain.  For Closed joints, the `free' link
  int linkIndex;
  ///For Floating/BallAndSocket joints, the first link in the chain. 
  ///For Closed joints, the attachment link 
  int baseIndex;
  ///For closed joints
  Vector3 localPt,attachmentPt;
};

/** @ingroup Modeling 
 * @brief Determines the effects of an actuator on the robot configuration.
 */
struct RobotModelDriver
{
  /** Types
  - Normal: normal 
  - Affine: scaling/offset of a single control to multiple outputs
     linkIndices stores the mapping
  - Translation: controls are a direct force applied to the body.  (not implemented yet)
     linkIndices[0] is the "driver" link, linkIndices[1] is the affected link
  - Rotation: controls are a direct moment applied to the body.  (not implemented yet)
     linkIndices[0] is the "driver" link, linkIndices[1] is the affected link
  - Custom: in the future will hold more sophisticated mappings
  */
  enum Type { Normal, Affine, Translation, Rotation, Custom };

  int NumControls() const;  //number of input controls
  int NumLinks() const;
  bool Affects(int link) const;

  Type type;
  vector<int> linkIndices;
  Real qmin,qmax;           //min/max values
  Real vmin,vmax;           //min/max velocities
  Real amin,amax;           //min/max accelerations
  Real tmin,tmax;           //min/max torques
  vector<Real> affScaling;  //for Affine joints
  vector<Real> affOffset;   //for Affine joints
  Real servoP,servoI,servoD;  //servo parameters
  Real dryFriction;           //constant friction coefficient
  Real viscousFriction;       //velocity-dependent friction coefficient
};

/** @ingroup Modeling
 * @brief The main robot type used in RobotSim.
 *
 * Inherits its main functionality from RobotWithGeometry, but adds extra
 * saving and loading to .rob files, and contains RobotModelJoint and
 * RobotModelDriver information.
 */
class RobotModel : public RobotWithGeometry
{
public:
  virtual std::string LinkName(int i) const;
  int LinkIndex(const char* name) const;
  bool Load(const char* fn);
  bool LoadRob(const char* fn);
  bool LoadURDF(const char* fn);
  bool Save(const char* fn);
  bool LoadGeometry(int i,const char* file);
  void SetGeomFiles(const char* geomPrefix="",const char* geomExt="off");  ///< Sets the geometry file names to geomPrefix+[linkName].[geomExt]
  void SetGeomFiles(const vector<string>& geomFiles);
  bool SaveGeometry(const char* prefix="");  
  void InitStandardJoints();
  bool CheckValid() const;
  //adds a geometry to the geometry of the given link
  void Mount(int link,const Geometry::AnyGeometry3D& geom,const RigidTransform& T);
  //adds a subchain as descendents of a given link
  void Mount(int link,const RobotModel& subchain,const RigidTransform& T,const char* prefix=NULL);
  ///Creates this into a mega-robot from several other robots
  void Merge(const std::vector<RobotModel*>& robots);
  ///The reduced robot is a robot for which all fixed DOFs are eliminated; this
  ///helps with Newton-Euler solving.  dofMap maps robot DOFs to reduced dofs, with
  ///-1 indicating the dof is eliminated.
  ///
  ///Note: all links fixed to the environment no longer have any geometry attached
  void Reduce(RobotModel& reducedRobot,vector<int>& dofMap);

  bool DoesJointAffect(int joint,int dof) const;
  void GetJointIndices(int joint,vector<int>& indices) const;
  ///Used for setting configuration of floating and ball-and-socket joints
  void SetJointByTransform(int joint,int link,const RigidTransform& T);
  ///Used for setting configuration of floating and ball-and-socket joints
  void SetJointByOrientation(int joint,int link,const Matrix3& R);
  ///Used for setting velocity of floating and ball-and-socket joints
  void SetJointVelocityByMoment(int joint,int link,const Vector3& w,const Vector3& v);

  ///Returns true if the given DOF does not have a driver attached to it
  bool IsPassiveDOF(int dof) const;
  bool DoesDriverAffect(int driver,int dof) const;
  void GetDriverIndices(int driver,vector<int>& indices) const;
  Vector2 GetDriverLimits(int driver) const;
  Real GetDriverValue(int driver) const;
  Real GetDriverVelocity(int driver) const;
  void SetDriverValue(int driver,Real value);
  void SetDriverVelocity(int driver,Real value);
  ///Returns a vector J such that the change in dof values is J*v where v is the change in the driver value
  void GetDriverJacobian(int driver,Vector& J);

  ///The lipschitz matrix (i,j) is a bound on the amount that the geometry at
  ///link j moves in the workspace in response to a unit change in q(i)
  ///It is used by exact collision checkers, and is uninitialized by default.
  void ComputeLipschitzMatrix();

  ///Configures an IK solver to respect driver constraints.  Only works for
  ///Affine constraints at the moment.
  void ConfigureDriverConstraints(RobotIKFunction& f);

  string name;
  vector<string> geomFiles;   ///< geometry file names (used in saving)
  vector<ManagedGeometry> geomManagers; ///< geometry loaders (speeds up loading)
  Vector accMax;   ///< conservative acceleration limits, used by DynamicPath
  vector<RobotModelJoint> joints;
  vector<RobotModelDriver> drivers;
  vector<string> linkNames;
  vector<string> driverNames;

  ///Any extra properties that might be useful.  Currently, we support the following:
  ///- sensors: an XML string defining the robot's sensors.  If an xml filename is given,
  ///     this string is loaded from disk.
  ///- controller: an XML string defining the robot's controller.  If an xml filename is
  ///     given, this string is loaded from disk.
  PropertyMap properties;

  ///A matrix of lipschitz constants (see ComputeLipschitzMatrix)
  Matrix lipschitzMatrix;

  ///Set this to true if you want to disable loading of geometry -- saves time
  ///for some utility programs.
  static bool disableGeometryLoading;
};

} //namespace Klampt

#endif
