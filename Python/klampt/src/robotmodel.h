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
class RigidObject;
class Terrain;
class Robot;

/** @brief Stores mass information for a rigid body or robot link. */
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

/** @brief Stores contact parameters for an entity.  Currently only
 * used for simulation, but could be used for contact mechanics in the
 * future. */
struct ContactParameters
{
  double kFriction;
  double kRestitution;
  double kStiffness,kDamping;
};

/** @brief A reference to a link of a RobotModel.
 *
 * 
 */
class RobotModelLink
{
 public:
  RobotModelLink();
  ///Returns the ID of the robot link in its world (Note: not the same as getIndex())
  int getID();
  ///Returns the name of the robot link
  const char* getName();
  ///Returns a reference to the link's robot.
  RobotModel robot();
  ///Old-style: will be deprecated
  RobotModel getRobot();
  ///Returns the index of the link (on its robot).
  int getIndex();
  ///Returns the index of the link's parent (on its robot).
  int getParent();
  ///Returns a reference to the link's parent, or a NULL link if it has no parent
  RobotModelLink parent();
  ///Sets the index of the link's parent (on its robot).
  void setParent(int p);
  ///Sets the link's parent (must be on the same robot).
  void setParent(const RobotModelLink& l);
  ///Returns a reference to the link's geometry
  Geometry3D geometry();
  ///Returns a reference to the link's appearance
  Appearance appearance();
  ///Retrieves the inertial properties of the link. (Note that the Mass is given with
  ///origin at the link frame, not about the COM.)
  Mass getMass();
  ///Sets the inertial proerties of the link. (Note that the Mass is given with origin
  ///at the link frame, not about the COM.)
  void setMass(const Mass& mass);
  ///Gets transformation (R,t) to the parent link
  void getParentTransform(double out[9],double out2[3]);
  void setParentTransform(const double R[9],const double t[3]);
  ///Gets the local rotational / translational axis
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
  ///Draws the link's geometry in its local frame.  If keepAppearance=true, the
  ///current Appearance is honored.  Otherwise, just the geometry is drawn.
  void drawLocalGL(bool keepAppearance=true);
  ///Draws the link's geometry in the world frame.  If keepAppearance=true, the
  ///current Appearance is honored.  Otherwise, just the geometry is drawn.
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
  ///Returns a reference to the driver's robot.
  RobotModel robot();
  ///Old-style: will be deprecated
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
  ///Returns the ID of the robot in its world (Note: not the same as the robot index)
  int getID();
  const char* getName();
  ///Returns the number of links = number of DOF's.
  int numLinks();
  ///Returns a reference to the indexed link
  RobotModelLink link(int index);
  ///Returns a reference to the named link
  RobotModelLink link(const char* name);
  ///Old-style: will be deprecated.  Returns a reference to the indexed link.
  RobotModelLink getLink(int index);
  ///Old-style: will be deprecated.  Returns a reference to the named link.
  RobotModelLink getLink(const char* name);
  ///Returns the number of drivers.
  int numDrivers();
  ///Returns a reference to the indexed driver.
  RobotModelDriver driver(int index);
  ///Returns a reference to the named driver.
  RobotModelDriver driver(const char* name);
  ///Old-style: will be deprecated. Returns a reference to the indexed driver.
  RobotModelDriver getDriver(int index);
  ///Old-style: will be deprecated. Returns a reference to a RobotModelDriver.
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
  ///Sets a single DOF's position.  Note: if you are setting several joints 
  ///at once, use setConfig because this function computes forward kinematics
  ///every time.
  void setDOFPosition(int i,double qi);
  void setDOFPosition(const char* name,double qi);
  ///Returns a single DOF's position
  double getDOFPosition(int i);
  double getDOFPosition(const char* name);

  //dynamics functions
  ///Returns the 3D center of mass at the current config
  void getCom(double out[3]);
  ///Returns the 3xn Jacobian matrix of the current center of mass
  void getComJacobian(std::vector<std::vector<double> >& out);
  ///Returns the nxn mass matrix B(q)
  void getMassMatrix(std::vector<std::vector<double> >& out);
  ///Returns the inverse of the nxn mass matrix B(q)^-1 (faster than inverting result of getMassMatrix)
  void getMassMatrixInv(std::vector<std::vector<double> >& out);
  ///Returns the Coriolis force matrix C(q,dq) for current config and velocity
  void getCoriolisForceMatrix(std::vector<std::vector<double> >& out);
  ///Returns the Coriolis forces C(q,dq)*dq for current config and velocity
  ///(faster than computing matrix and doing product). ("Forces" is somewhat
  ///of a misnomer; the result is a joint torque vector)
  void getCoriolisForces(std::vector<double>& out);
  ///Returns the generalized gravity vector G(q) for the given workspace
  ///gravity vector g (usually (0,0,-9.8)).  ("Forces" is somewhat of a
  ///misnomer; the result is a joint torque vector)
  void getGravityForces(const double g[3],std::vector<double>& out);
  ///Computes the inverse dynamics (using Recursive Newton Euler solver).
  ///Note: does not include gravity term G(q)
  void torquesFromAccel(const std::vector<double>& ddq,std::vector<double>& out);
  ///Computes the foward dynamics (using Recursive Newton Euler solver)
  ///Note: does not include gravity term G(q)
  void accelFromTorques(const std::vector<double>& t,std::vector<double>& out);

  //interpolation functions
  ///Interpolates smoothly between two configurations, properly taking into account nonstandard joints
  void interpolate(const std::vector<double>& a,const std::vector<double>& b,double u,std::vector<double>& out);
  ///Computes a distance between two configurations, properly taking into account nonstandard joints
  double distance(const std::vector<double>& a,const std::vector<double>& b);
  ///Returns the configuration derivative at a as you interpolate toward b at unit speed.
  void interpolateDeriv(const std::vector<double>& a,const std::vector<double>& b,std::vector<double>& out);

  ///Samples a random configuration and updates the robot's pose.  Properly
  ///handles non-normal joints and handles DOFs with infinite bounds
  ///using a centered Laplacian distribution with the given scaling term.
  ///(Note that the python random seeding does not affect the result.)
  void randomizeConfig(double unboundedScale=1.0);

  //geometry functions
  ///Queries whether self collisions between two links is enabled
  bool selfCollisionEnabled(int link1,int link2);
  ///Enables/disables self collisions between two links (depending on value)
  void enableSelfCollision(int link1,int link2,bool value);
  ///Returns true if the robot is in self collision (faster than manual testing)
  bool selfCollides();
  ///Draws the robot geometry. If keepAppearance=true, the current appearance is honored.
  ///Otherwise, only the raw geometry is drawn.  PERFORMANCE WARNING: if keepAppearance is
  ///false, then this does not properly reuse OpenGL display lists.  A better approach
  ///to changing the robot's appearances is to set the link Appearance's directly.
  void drawGL(bool keepAppearance=true);

  int world;
  int index;
  Robot* robot;
};

/** @brief A rigid movable object.
 *
 * State is retrieved/set using get/setTransform.  Note: no velocities are stored.
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
  ///Draws the object's geometry. If keepAppearance=true, the current appearance is honored.
  ///Otherwise, only the raw geometry is drawn.  PERFORMANCE WARNING: if keepAppearance is
  ///false, then this does not properly reuse OpenGL display lists.  A better approach
  ///to changing object's Appearance directly.
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
  ///Draws the object's geometry. If keepAppearance=true, the current appearance is honored.
  ///Otherwise, only the raw geometry is drawn.  PERFORMANCE WARNING: if keepAppearance is
  ///false, then this does not properly reuse OpenGL display lists.  A better approach
  ///to changing object's Appearance directly.
  void drawGL(bool keepAppearance=true);

  int world;
  int index;
  Terrain* terrain;
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
  ///Creates a WorldModel.  With no arguments, creates a new world.  With
  ///an integer or another WorldModel instance, creates a reference to an
  ///existing world.  (To create a copy, use the copy() method.)
  ///
  ///If passed a pointer to a C++ RobotWorld structure, a reference to that
  ///structure is returned. (This is used pretty much only when
  ///interfacing C++ and Python code)
  WorldModel();
  WorldModel(void* ptrRobotWorld);
  WorldModel(int index);
  WorldModel(const WorldModel& w);
  ~WorldModel();
  ///Sets this WorldModel to a reference to w
  const WorldModel& operator = (const WorldModel& w);
  ///Creates a copy of the world model.  Note that geometries and appearances
  ///are shared...
  WorldModel copy();
  ///Reads from a world XML file.
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
  ///Creates a new empty robot. (Not terribly useful now since you can't resize the number of links yet)
  RobotModel makeRobot(const char* name);
  ///Creates a new empty rigid object.
  RigidObjectModel makeRigidObject(const char* name);
  ///Creates a new empty terrain 
  TerrainModel makeTerrain(const char* name);
  ///Loads a robot from a .rob or .urdf file.  An empty robot is returned if loading fails.
  RobotModel loadRobot(const char* fn);
  ///Loads a rigid object from a .obj or a mesh file.  An empty rigid object is returned if loading fails.
  RigidObjectModel loadRigidObject(const char* fn);
  ///Loads a rigid object from a mesh file.  An empty terrain is returned if loading fails.
  TerrainModel loadTerrain(const char* fn);
  ///Loads some element from a file, automatically detecting its type.  Meshes are interpreted
  ///as terrains.  The ID is returned, or -1 if loading failed.
  int loadElement(const char* fn);
  ///Adds a copy of the given robot to this world, either from this WorldModel
  ///or another.
  RobotModel add(const char* name,const RobotModel& robot);
  ///Adds a copy of the given rigid object to this world, either from this
  ///WorldModel or another.
  RigidObjectModel add(const char* name,const RigidObjectModel& obj);
  ///Adds a copy of the given terrain to this world, either from this
  ///WorldModel or another.
  TerrainModel add(const char* name,const TerrainModel& terrain);
  ///Removes a robot.  It must be in this world or an exception is raised.
  ///IMPORTANT: all other references to robots will be invalidated.
  void remove(const RobotModel& robot);
  ///Removes a rigid object.  It must be in this world or an exception is
  ///raised.
  ///IMPORTANT: all other references to rigid objects will be invalidated.
  void remove(const RigidObjectModel& object);
  ///Removes a terrain.  It must be in this world or an exception is raised.
  ///IMPORTANT: all other references to terrains will be invalidated.
  void remove(const TerrainModel& terrain);
  ///Retrieves a name for a given element ID
  std::string getName(int id);
  ///Retrieves a geometry for a given element ID
  Geometry3D geometry(int id);
  ///Retrieves an appearance for a given element ID
  Appearance appearance(int id);
  ///Draws the entire world using OpenGL
  void drawGL();
  ///If geometry loading is set to false, then only the kinematics are loaded from
  ///disk, and no geometry / visualization / collision detection structures will be
  ///loaded.  Useful for quick scripts that just use kinematics / dynamics of a robot.
  void enableGeometryLoading(bool enabled);
  ///If collision detection is set to true, then collision acceleration data
  ///structures will be automatically initialized, with debugging information.
  ///Useful for scripts that do planning and for which collision
  ///initialization may take a long time.  Note that even when this flag
  ///is off, the collision acceleration data structures will indeed be
  ///initialized whenever geometry collision, distance, or ray-casting
  ///routines are called.
  void enableInitCollisions(bool enabled);

  //WARNING: do not modify this member directly
  int index;
};

#endif
