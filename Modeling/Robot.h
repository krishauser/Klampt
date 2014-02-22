#ifndef MODELING_ROBOT_H
#define MODELING_ROBOT_H

#include <robotics/RobotWithGeometry.h>
using namespace std;

/** @ingroup Modeling 
 * @brief Additional joint properties 
 */
struct RobotJoint
{
  /* Types:
     Weld: completely fixed to the parent
     Normal: regular fixed axis joint with finite range
     Spin: infinitely spinnable rotational joint
     Floating: free-floating base
     FloatingPlanar: a free-floating 2D base
     BallAndSocket: ball and socket joint
     Closed: a closed chain loop
  */
  enum Type { Weld, Normal, Spin, Floating, FloatingPlanar, BallAndSocket, Closed };
//  UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED
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
struct RobotJointDriver
{
  /* Types
  Normal: normal 
  Affine: scaling/offset of a single control to multiple outputs
     linkIndices stores the mapping
  Translation: controls are a direct force applied to the body. 
     linkIndices[0] is the "driver" link, linkIndices[1] is the affected link
  Rotation: controls are a direct moment applied to the body
     linkIndices[0] is the "driver" link, linkIndices[1] is the affected link
  Custom: in the future will hold more sophisticated mappings
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
 * saving and loading to .rob files, and contains RobotJoint and
 * RobotJointDriver information.
 */
class Robot : public RobotWithGeometry
{
public:
  virtual std::string LinkName(int i) const;
  int LinkIndex(const char* name) const;
  bool Load(const char* fn);
  bool LoadRob(const char* fn);
  bool LoadURDF(const char* fn);
  bool Save(const char* fn,const char* geomPrefix="",const char* geomExt="tri");  ///< Saves the geometry line as geomPrefix+[linkName].[geomExt]
  bool Save(const char* fn,const vector<string>& geomFiles); 
  bool SaveGeometry(const char* geomPrefix="",const char* geomExt="tri");  ///< Saves the geometry files to geomPrefix+[linkName].[geomExt]
  bool SaveGeometry(const vector<string>& geomFiles); 
  void InitStandardJoints();
  bool CheckValid() const;
  //adds a geometry to the geometry of the given link
  void Mount(int link,const Geometry::AnyGeometry3D& geom,const RigidTransform& T);
  //adds a subchain as descendents of a given link
  void Mount(int link,const Robot& subchain,const RigidTransform& T);

  bool DoesJointAffect(int joint,int dof) const;
  void GetJointIndices(int joint,vector<int>& indices) const;
  //for floating joints
  void SetJointByTransform(int joint,int link,const RigidTransform& T);
  void SetJointByOrientation(int joint,int link,const Matrix3& R);
  void SetJointVelocityByMoment(int joint,int link,const Vector3& w,const Vector3& v);

  bool IsPassiveDOF(int dof) const;
  bool DoesDriverAffect(int driver,int dof) const;
  void GetDriverIndices(int driver,vector<int>& indices) const;
  Vector2 GetDriverLimits(int driver) const;
  Real GetDriverValue(int driver) const;
  Real GetDriverVelocity(int driver) const;
  void SetDriverValue(int driver,Real value);
  void SetDriverVelocity(int driver,Real value);
  void GetDriverJacobian(int driver,Vector& J);

  //The lipschitz matrix (i,j) is a bound on the amount that the geometry at
  //link j moves in the workspace in response to a unit change in q(i)
  //It is used by exact collision checkers, and is uninitialized by default.
  void ComputeLipschitzMatrix();

  Vector accMax;   //conservative acceleration limits, used by DynamicPath
  vector<RobotJoint> joints;
  vector<RobotJointDriver> drivers;
  vector<string> linkNames;
  vector<string> driverNames;
  Matrix lipschitzMatrix;
};

#endif
