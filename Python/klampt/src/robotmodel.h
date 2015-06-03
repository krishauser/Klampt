#ifndef _ROBOT_WORLD_H
#define _ROBOT_WORLD_H

/** @file robotmodel.h
 * @brief C++ bindings for robot/world modeling. */

#include "geometry.h"
#include "appearance.h"

//forward definitions for API objects
class WorldModel;
class RobotModel;
class RobotModelLink;
class RigidObjectModel;
class TerrainModel;

//forward definitions for pointers to internal objects
struct RigidObject;
struct Environment;
struct Robot;

/** @brief Stores mass information for a rigid body. */
struct Mass
{
  void setMass(double _mass) { mass=_mass; }
  double getMass() const { return mass; }
  void setCom(const std::vector<double>& _com) { com = _com; }
  void getCom(std::vector<double>& out) const { out = com; }
  void setInertia(const std::vector<double>& _inertia) { inertia = _inertia; }
  void getInertia(std::vector<double>& out) const { out=inertia; }

  double mass;        ///<mass
  std::vector<double> com;      ///<local center of mass, size 3
  std::vector<double> inertia;  ///<local inertia matrix, size 3 or 9
};

struct ContactParameters
{
  double kFriction;
  double kRestitution;
  double kStiffness,kDamping;
};

/** @brief A reference to a link of a RobotModel.
 *
 * Note that the mass is given local to the link frame, not about the COM.
 */
class RobotModelLink
{
 public:
  RobotModelLink();
  int getID();
  const char* getName();
  RobotModel robot();
  RobotModel getRobot();
  int getIndex();
  int getParent();
  void setParent(int p);
  Geometry3D geometry();
  Appearance appearance();
  Mass getMass();
  void setMass(const Mass& mass);
  ///Gets transformation (R,t) to the parent link
  void getParentTransform(double out[9],double out2[3]);
  void setParentTransform(const double R[9],const double t[3]);
  ///Gets the local rotational axis
  void getAxis(double out[3]);
  void setAxis(const double axis[3]);

  ///Converts point from local to world coordinates 
  void getWorldPosition(const double plocal[3],double out[3]);
  ///Converts direction from local to world coordinates 
  void getWorldDirection(const double vlocal[3],double out[3]);
  ///Converts point from world to local coordinates 
  void getLocalPosition(const double pworld[3],double out[3]);
  ///Converts direction from world to local coordinates 
  void getLocalDirection(const double vworld[3],double out[3]);
  ///Gets transformation (R,t) to the world frame
  void getTransform(double out[9],double out2[3]);
  ///Sets transformation (R,t) to the world frame.  Note: this does NOT
  ///perform inverse kinematics.  The transform is overwritten when the
  ///robot's setConfig() method is called.
  void setTransform(const double R[9],const double t[3]);
  ///Returns the total jacobian of the local point p (row-major matrix)
  ///(orientation jacobian is stacked on position jacobian)
  void getJacobian(const double p[3],std::vector<std::vector<double> >& out);
  ///Returns the jacobian of the local point p (row-major matrix)
  void getPositionJacobian(const double p[3],std::vector<std::vector<double> >& out);
  ///Returns the orientation jacobian of the link (row-major matrix)
  void getOrientationJacobian(std::vector<std::vector<double> >& out);
  ///Returns the velocity of the origin given the robot's current velocity
  void getVelocity(double out[3]);
  ///Returns the angular velocity given the robot's current velocity
  void getAngularVelocity(double out[3]);
  ///Returns the world velocity of the point given the robot's current velocity
  void getPointVelocity(const double plocal[3],double out[3]);
  void drawLocalGL(bool keepAppearance=true);
  void drawWorldGL(bool keepAppearance=true);

  int world;
  int robotIndex;
  Robot* robotPtr;
  int index;
};

/** @brief A reference to a driver of a RobotModel.
 */
class RobotModelDriver
{
 public:
  RobotModelDriver();
  const char* getName();
  RobotModel robot();
  RobotModel getRobot();
  ///Currently can be "normal", "affine", "rotation", "translation", or "custom"
  const char* getType();
  ///Returns the single affected link for "normal" links
  int getAffectedLink();
  ///Returns the driver's affected links
  void getAffectedLinks(std::vector<int>& links);
  ///For "affine" links, returns the scale and offset of the driver value mapped to the world
  void getAffineCoeffs(std::vector<double>& scale,std::vector<double>& offset);
  ///Sets the robot's config to correspond to the given driver value 
  void setValue(double val);
  ///Gets the current driver value from the robot's config
  double getValue();
  ///Sets the robot's velocity to correspond to the given driver velocity value 
  void setVelocity(double val);
  ///Gets the current driver velocity value from the robot's velocity
  double getVelocity();

  int world;
  int robotIndex;
  Robot* robotPtr;
  int index;
};

/** @brief A model of a dynamic and kinematic robot.
 *
 * It is important to understand that changing the configuration of the model
 * doesn't actually send a command to the robot.  In essence, this model
 * maintains temporary storage for performing kinematics and dynamics
 * computations.
 *
 * The robot maintains configuration/velocity/acceleration/torque bounds
 * which are not enforced by the model, but must rather be enforced by the
 * planner / simulator.
 *
 * The state of the robot is retrieved using getConfig/getVelocity calls, and
 * is set using setConfig/setVelocity.
 */
class RobotModel
{
 public:
  RobotModel();
  int getID();
  const char* getName();
  int numLinks();
  RobotModelLink link(int index);
  RobotModelLink link(const char* name);
  ///Old-style: will be deprecated
  RobotModelLink getLink(int index);
  ///Old-style: will be deprecated
  RobotModelLink getLink(const char* name);
  int numDrivers();
  RobotModelDriver driver(int index);
  RobotModelDriver driver(const char* name);
  ///Old-style: will be deprecated
  RobotModelDriver getDriver(int index);
  ///Old-style: will be deprecated
  RobotModelDriver getDriver(const char* name);

  //kinematic and dynamic properties
  void getConfig(std::vector<double>& out);
  void getVelocity(std::vector<double>& out);
  void setConfig(const std::vector<double>& q);
  void setVelocity(const std::vector<double>& dq);
  void getJointLimits(std::vector<double>& out,std::vector<double>& out2);
  void setJointLimits(const std::vector<double>& qmin,const std::vector<double>& qmax);
  void getVelocityLimits(std::vector<double>& out);
  void setVelocityLimits(const std::vector<double>& vmax);
  void getAccelerationLimits(std::vector<double>& out);
  void setAccelerationLimits(const std::vector<double>& amax);
  void getTorqueLimits(std::vector<double>& out);
  void setTorqueLimits(const std::vector<double>& tmax);

  //dynamics functions
  void getCom(double out[3]);
  void getComJacobian(std::vector<std::vector<double> >& out);
  void getMassMatrix(std::vector<std::vector<double> >& out);
  void getMassMatrixInv(std::vector<std::vector<double> >& out);
  void getCoriolisForceMatrix(std::vector<std::vector<double> >& out);
  void getCoriolisForces(std::vector<double>& out);
  void getGravityForces(const double g[3],std::vector<double>& out);
  void torquesFromAccel(const std::vector<double>& ddq,std::vector<double>& out);
  void accelFromTorques(const std::vector<double>& t,std::vector<double>& out);

  //interpolation functions
  void interpolate(const std::vector<double>& a,const std::vector<double>& b,double u,std::vector<double>& out);
  double distance(const std::vector<double>& a,const std::vector<double>& b);
  void interpolate_deriv(const std::vector<double>& a,const std::vector<double>& b,std::vector<double>& out);

  //geometry functions
  bool selfCollisionEnabled(int link1,int link2);
  void enableSelfCollision(int link1,int link2,bool value);
  void drawGL(bool keepAppearance=true);

  int world;
  int index;
  Robot* robot;
};

/** @brief A rigid movable object.
 *
 * State is retrieved/set using get/setTransform.  No velocities are stored.
 */
class RigidObjectModel
{
 public:
  RigidObjectModel();
  int getID();
  const char* getName();
  Geometry3D geometry();
  Appearance appearance();
  Mass getMass();
  void setMass(const Mass& mass);
  ContactParameters getContactParameters();
  void setContactParameters(const ContactParameters& params);
  void getTransform(double out[9],double out2[3]);
  void setTransform(const double R[9],const double t[3]);
  void drawGL(bool keepAppearance=true);

  int world;
  int index;
  RigidObject* object;
};

/** @brief Static environment geometry.
 */
class TerrainModel
{
 public:
  TerrainModel();
  int getID();
  const char* getName();
  Geometry3D geometry();
  Appearance appearance();
  void setFriction(double friction);
  void drawGL(bool keepAppearance=true);

  int world;
  int index;
  Environment* terrain;
};

/** @brief The main world class, containing robots, rigid objects, and static
 * environment geometry.
 *
 * Note that this is just a model and can be changed at will -- in fact 
 * planners and simulators will make use of a model to "display" computed
 *
 * Every robot/robot link/terrain/rigid object is given a unique ID in the
 * world.  This is potentially a source of confusion because some functions
 * take IDs and some take indices.  Only the WorldModel and Simulator
 * classes use IDs when the argument has 'id' as a suffix, e.g., geometry(),
 * appearance(), Simulator.inContact().
 * All other functions use indices, e.g. robot(0), terrain(0), etc.
 *
 * To get an object's ID, you can see the value returned by loadElement
 * and/or object.getID().  
 * states.
 *
 * To save/restore the state of the model, you must manually maintain copies of
 * the states of whichever objects you wish to save/restore.
 */
class WorldModel
{
 public:
  WorldModel();
  WorldModel(int index);
  WorldModel(const WorldModel& w);
  ~WorldModel();
  const WorldModel& operator = (const WorldModel& w);
  bool readFile(const char* fn);
  int numRobots();
  int numRobotLinks(int robot);
  int numRigidObjects();
  int numTerrains();
  int numIDs();
  RobotModel robot(int index);
  RobotModel robot(const char* name);
  RobotModelLink robotLink(int robot,int index);
  RobotModelLink robotLink(const char* robot,const char* name);
  RigidObjectModel rigidObject(int index);
  RigidObjectModel rigidObject(const char* name);
  TerrainModel terrain(int index);
  TerrainModel terrain(const char* name);
  RobotModel makeRobot(const char* name);
  RigidObjectModel makeRigidObject(const char* name);
  TerrainModel makeTerrain(const char* name);
  RobotModel loadRobot(const char* fn);
  RigidObjectModel loadRigidObject(const char* fn);
  TerrainModel loadTerrain(const char* fn);
  int loadElement(const char* fn);
  std::string getName(int id);
  Geometry3D geometry(int id);
  Appearance appearance(int id);
  void drawGL();
  void enableGeometryLoading(bool enabled);

  //WARNING: do not modify this member directly
  int index;
};

#endif
