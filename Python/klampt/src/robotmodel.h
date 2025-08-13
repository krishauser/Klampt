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
class SensorModel;

//forward definitions for pointers to internal objects
namespace Klampt {

  class RigidObjectModel;
  class TerrainModel;
  class RobotModel;
  class SensorBase;

} //namespace Klampt

/** @brief Stores mass information for a rigid body or robot link.
 * 
 * .. note::
 * 
 *     Recommended to use the set/get functions rather than changing the members
 *     directly due to strangeness in SWIG's handling of vectors.
 * 
 * 
 * .. note:
 * 
 *     The inertia matrix is specified in the local frame of the object
 *     and centered at the center of mass. 
 * 
 * 
 * Attributes:
 *
 *     mass (float): the actual mass (typically in kg)
 *     com (list of 3 floats): the center of mass position, in
 *         local coordinates.
 *     inertia (list of 3 floats or 9 floats): the inertia matrix
 *         in local coordinates.  If 3 floats, this is a diagonal matrix.
 *         If 9 floats, this gives all entries of the 3x3 inertia matrix
 *         (in column major or row major order, it doesn't matter since
 *         inertia matrices are symmetric)
 */
class Mass
{
public:
  Mass();
  void setMass(double _mass) { mass=_mass; }
  double getMass() const { return mass; }
  void setCom(const std::vector<double>& _com) { com = _com; }
  ///Returns the COM 
  ///
  ///Return type: Vector3
  void getCom(std::vector<double>& out) const { out = com; }
  ///Sets an inertia matrix.
  void setInertia(const std::vector<double>& _inertia) { inertia = _inertia; }
  ///Returns the inertia matrix as a list of 3 floats or 9 floats
  /// 
  ///Return type: Vector
  void getInertia(std::vector<double>& out) const { out=inertia; }
  ///Estimates the com and inertia of a geometry, with a given total mass. 
  ///
  ///For TriangleMesh types, surfaceFraction dictates how much of the object's mass
  ///is concentrated at the surface rather than the interior.
  void estimate(const Geometry3D& g,double mass,double surfaceFraction=0);

  double mass;        //<mass
  std::vector<double> com;      //<local center of mass, size 3
  std::vector<double> inertia;  //<local inertia matrix, size 3 or 9
};

/** @brief Stores contact parameters for an entity.  Currently only
 * used for simulation, but could be used for contact mechanics in the
 * future.
 *
 * Attributes:
 *
 *     kFriction (float): The coefficient of (Coulomb) friction, in range
 *         [0,inf).
 *     kRestitution (float): The coefficient of restitution, in range [0,1].
 *     kStiffness (float): The stiffness of the material, in range (0,inf)
 *         (default inf, perfectly rigid).
 *     kDamping (float): The damping of the material, in range (0,inf)
 *         (default inf, perfectly rigid).
 */
class ContactParameters
{
public:
  ContactParameters();
  double kFriction;
  double kRestitution;
  double kStiffness,kDamping;
};

/** @brief A reference to a link of a RobotModel.
 *
 * The link stores many mostly-constant items (id, name, parent, geometry,
 * appearance, mass, joint axes).  There are two exceptions:
 *
 * - the link's current transform, which is affected by the RobotModel's
 *     current configuration, i.e., the last :meth:`RobotModel.setConfig` call.
 * - The various Jacobians of points on the link, accessed by
 *     :meth:`RobotModelLink.getJacobian` ,
 *     :meth:`RobotModelLink.getPositionJacobian` , and
 *     :meth:`RobotModelLink.getOrientationJacobian` ,
 *     which are configuration dependent.
 *
 * A RobotModelLink is not created by hand, but instead accessed using
 * :meth:`RobotModel.link` (index or name).
 */
class RobotModelLink
{
 public:
  RobotModelLink();
  ///Returns the ID of the robot link in its world
  ///
  ///.. note::
  ///
  ///    The world ID is not the same as the link's index, retrieved by
  ///    getIndex.
  ///
  int getID() const;
  ///Returns the name of the robot link
  const char* getName() const;
  ///Sets the name of the robot link
  void setName(const char* name);
  ///Returns a reference to the link's robot.
  RobotModel robot();
  ///Returns the index of the link (on its robot).
  int getIndex();
  ///Returns the index of the link's parent (on its robot). -1 indicates no parent.
  int getParentIndex();
  ///Sets the index of the link's parent (on its robot).
  void setParentIndex(int p);
  ///Returns a reference to the link's parent, or a NULL link if it has no parent
  RobotModelLink getParentLink();
  ///Sets the link's parent (must be on the same robot).
  void setParentLink(const RobotModelLink& l);
  ///Returns a reference to the link's geometry
  Geometry3D geometry();
  ///Returns a reference to the link's appearance
  Appearance appearance();
  ///Returns the inertial properties of the link. 
  ///
  ///.. note::
  ///
  ///    To change the mass properties, you should call ``m=link.getMass()``,
  ///    change the desired properties in m, and then ``link.setMass(m)``
  ///
  ///.. note::
  ///
  ///     The Mass object considers the inertia matrix origin as the link
  ///     frame, not about the COM.
  ///
  Mass getMass();
  ///Sets the inertial proerties of the link. (Note that the Mass is given with origin
  ///at the link frame, not about the COM.)
  void setMass(const Mass& mass);
  ///Gets the transformation (R,t) to the parent link
  ///
  ///Returns:
  ///
  ///    se3 object: a pair (R,t), with R a 9-list and t a 3-list of floats,
  ///    giving the local transform from this link to its parent, in the
  ///    reference (zero) configuration.
  ///     
  ///Return type: RigidTransform
  void getParentTransform(double out[9],double out2[3]);
  ///Sets transformation (R,t) to the parent link
  void setParentTransform(const double R[9],const double t[3]);
  ///Gets the local rotational / translational axis
  ///
  ///Return type: Vector3
  void getAxis(double out[3]);
  ///Sets the local rotational / translational axis
  void setAxis(const double axis[3]);
  ///Returns whether the joint is prismatic
  bool isPrismatic();
  ///Returns whether the joint is revolute
  bool isRevolute();
  ///Changes a link from revolute to prismatic or vice versa
  void setPrismatic(bool prismatic);

  ///Converts point from local to world coordinates 
  ///
  ///Returns:
  ///
  ///    list of 3 floats: the world coordinates of the local point plocal
  /// 
  ///Return type: Vector3
  void getWorldPosition(const double plocal[3],double out[3]);
  ///Converts direction from local to world coordinates 
  ///
  ///Returns:
  ///
  ///    list of 3 floats: the world coordinates of the local direction
  ///    vlocal
  /// 
  ///Return type: Vector3
  void getWorldDirection(const double vlocal[3],double out[3]);
  ///Converts point from world to local coordinates 
  ///
  ///Returns:
  ///
  ///    list of 3 floats: the local coordinates of the world point pworld
  /// 
  ///Return type: Vector3    
  void getLocalPosition(const double pworld[3],double out[3]);
  ///Converts direction from world to local coordinates 
  ///
  ///Returns:
  ///
  ///    list of 3 floats: the local coordinates of the world direction
  ///    vworld
  /// 
  ///Return type: Vector3
  void getLocalDirection(const double vworld[3],double out[3]);
  ///Gets the link's current transformation (R,t) to the world frame
  ///
  ///Returns:
  ///
  ///    se3 object: a pair (R,t), with R a 9-list and t a 3-list of floats.
  /// 
  ///Return type: RigidTransform
  void getTransform(double out[9],double out2[3]);
  ///Sets the link's current transformation (R,t) to the world frame. 
  ///
  ///.. note::
  ///
  ///    This does NOT perform inverse kinematics.  The transform is
  ///    overwritten when the robot's setConfig() method is called.
  ///
  void setTransform(const double R[9],const double t[3]);
  ///Computes the velocity of the link's origin given the robot's current joint
  ///configuration and velocities.  Equivalent to getPointVelocity([0,0,0]).
  ///
  ///Returns:
  ///  
  ///    list of 3 floats: the current velocity of the link's origin, in
  ///    world coordinates
  ///
  ///Return type: Vector3
  void getVelocity(double out[3]);
  ///Computes the angular velocity of the link given the robot's current joint
  ///configuration and velocities
  ///
  ///Returns:
  ///  
  ///    list of 3 floats: the current angular velocity of the link, in world
  ///    coordinates
  ///     
  ///Return type: Vector3
  void getAngularVelocity(double out[3]);
  ///Computes the world velocity of a point attached to the link, given the
  ///robot's current joint configuration and velocities
  ///
  ///Returns:
  /// 
  ///    list of 3 floats: the current velocity of the point, in
  ///    world coordinates.
  ///     
  ///Return type: Vector3
  void getPointVelocity(const double plocal[3],double out[3]);
  ///Computes the total jacobian of a point on this link w.r.t. the robot's
  ///configuration q.
  ///
  ///The orientation jacobian is given in the first 3 rows, and is stacked
  ///on the position jacobian, which is given in the last 3 rows.
  ///
  ///Returns:
  /// 
  ///    ndarray: the 6xn total Jacobian matrix of the
  ///    point given by local coordinates plocal.  
  ///
  ///Return type: np.ndarray
  void getJacobian(const double plocal[3],double** np_out2,int* m,int* n);
  ///Computes the position jacobian of a point on this link  w.r.t. the robot's
  ///configuration q.
  ///
  ///This matrix J gives the point's velocity (in world coordinates) via
  ///np.dot(J,dq), where dq is the robot's joint velocities.
  ///
  ///Returns:
  /// 
  ///    ndarray: the 3xn Jacobian matrix of the
  ///    point given by local coordinates plocal.  
  ///  
  ///Return type: np.ndarray
  void getPositionJacobian(const double plocal[3],double** np_out2,int* m,int* n);
  ///Computes the orientation jacobian of this link  w.r.t. the robot's
  ///configuration q.
  ///
  ///This matrix J gives the link's angular velocity (in world coordinates)
  ///via np.dot(J,dq), where dq is the robot's joint velocities.
  ///
  ///Returns:
  /// 
  ///    ndarray:: the 3xn orientation Jacobian matrix of the link.  
  ///     
  ///Return type: np.ndarray
  void getOrientationJacobian(double** np_out2,int* m,int* n);
  ///Returns the jacobian of a point on this link  w.r.t. specified entries of 
  ///the robot's configuration q given by `links`.
  ///
  ///The orientation jacobian is given in the first 3 rows, and is stacked
  ///on the position jacobian, which is given in the last 3 rows.
  ///
  ///Returns:
  ///
  ///    ndarray: the 6xlen(links) Jacobian matrix of the
  ///    point given by local coordinates plocal. 
  ///
  ///Return type: np.ndarray
  void getJacobianCols(const double plocal[3],const std::vector<int>& links,double** np_out2,int* m,int* n);
  ///Returns the position jacobian of a point on this link  w.r.t. specified entries of 
  ///the robot's configuration q given by `links`.
  ///
  ///This matrix J gives the point's velocity (in world coordinates) via
  ///np.dot(J,dqlinks), where dqlinks are the joint velocities of the links
  ///in `links`
  ///
  ///Returns:
  ///
  ///    ndarray: the 3xlen(links) position Jacobian matrix of the
  ///    point given by local coordinates plocal. 
  ///
  ///Return type: np.ndarray
  void getPositionJacobianCols(const double plocal[3],const std::vector<int>& links,double** np_out2,int* m,int* n);
  ///Returns the orientation jacobian this link  w.r.t. specified entries of 
  ///the robot's configuration q given by `links`.
  ///
  ///This matrix J gives the point's angular velocity (in world coordinates) via
  ///np.dot(J,dqlinks), where dqlinks are the joint velocities of the links
  ///in `links`
  ///
  ///Returns:
  ///
  ///    ndarray: the 3xlen(links) orientation Jacobian matrix of the
  ///    link.
  ///
  ///Return type: np.ndarray
  void getOrientationJacobianCols(const std::vector<int>& links,double** np_out2,int* m,int* n);

  ///Computes the acceleration of the link origin given the robot's current
  ///joint configuration and velocities, and the joint accelerations ddq.
  ///
  ///ddq can be empty, which calculates the acceleration with acceleration 0,
  ///and is a little faster than setting ddq to [0]*n
  ///
  ///Returns:
  /// 
  ///    list of 3 floats: the acceleration of the link's origin, in
  ///    world coordinates.
  ///
  ///Return type: Vector3
  void getAcceleration(const std::vector<double>& ddq,double out[3]);
  ///Computes the acceleration of the point given the robot's current
  ///joint configuration and velocities, and the joint accelerations ddq.
  ///
  ///Returns:
  /// 
  ///    list of 3 floats: the acceleration of the point, in
  ///    world coordinates.
  ///
  ///Return type: Vector3 
  void getPointAcceleration(const double plocal[3],const std::vector<double>& ddq,double out[3]);
  ///Computes the angular acceleration of the link given the robot's current
  ///joint configuration and velocities, and the joint accelerations ddq.
  ///
  ///Returns:
  /// 
  ///    list of 3 floats: the angular acceleration of the link, in
  ///    world coordinates.
  /// 
  ///Return type: Vector3
  void getAngularAcceleration(const std::vector<double>& ddq,double out[3]);
  ///Computes the Hessians of each component of the position p w.r.t the
  ///robot's configuration q.
  ///
  ///Returns:
  /// 
  ///    ndarray: a 3xnxn array with each of the elements in the first axis
  ///    corresponding respectively, to the (x,y,z) components of the Hessian.
  /// 
  ///Return type: np.ndarray
  void getPositionHessian(const double plocal[3],double** np_out3,int* m,int* n,int* p);
  ///Computes the pseudo-Hessians of each orientation component of the link 
  ///w.r.t the robot's configuration q.  The pseudo-Hessian is the derivative
  ///of the angular velocity of this link w.r.t. the joint velocities.
  ///
  ///Returns:
  /// 
  ///    ndarray: a 3xnxn array with each of the elements in the first axis
  ///    corresponding, respectively, to the (wx,wy,wz) components of the 
  ///    pseudo-Hessian.
  /// 
  ///Return type: np.ndarray
  void getOrientationHessian(double** np_out3,int* m,int* n,int* p);
  ///Draws the link's geometry in its local frame.  If keepAppearance=true, the
  ///current Appearance is honored.  Otherwise, just the geometry is drawn.
  void drawLocalGL(bool keepAppearance=true);
  ///Draws the link's geometry in the world frame.  If keepAppearance=true, the
  ///current Appearance is honored.  Otherwise, just the geometry is drawn.
  void drawWorldGL(bool keepAppearance=true);

  int world;
  int robotIndex;
  Klampt::RobotModel* robotPtr;
  int index;
};

/** @brief A reference to a driver of a RobotModel.
 * 
 * A driver corresponds to one of the robot's actuators and encodes how its
 * forces are transmitted to joints.
 *
 * A RobotModelDriver is not created by hand, but instead accessed using :meth:`RobotModel.driver` (index or name)
 */
class RobotModelDriver
{
 public:
  RobotModelDriver();
  const char* getName() const;
  ///Sets the name of the driver
  void setName(const char* name);
  ///Returns a reference to the driver's robot.
  RobotModel robot();
  ///Gets the type of the driver.
  ///
  ///Returns:
  ///
  ///    One of  "normal", "affine", "rotation", "translation", or "custom"
  ///
  const char* getType();
  ///Returns the single affected link for "normal" links
  int getAffectedLink();
  ///Returns the indices of the driver's affected links
  ///
  ///Return type: List[int]
  void getAffectedLinks(std::vector<int>& out);
  ///For "affine" links, returns the scale and offset of the driver value mapped
  ///to the world.
  ///
  ///Returns a pair (scale,offset), each of length len(getAffectedLinks()).
  ///
  ///Return type: Tuple[Vector,Vector]
  void getAffineCoeffs(std::vector<double>& out,std::vector<double>& out2);
  ///Sets the robot's config to correspond to the given driver value.
  ///
  ///.. note::
  ///
  ///    Does not update the links' forward kinematics.  Use
  ///    robot.setConfig(robot.getConfig()) to update the forward kinematics.
  ///
  void setValue(double val);
  ///Returns the current driver value from the robot's config
  double getValue();
  ///Sets the robot's velocity to correspond to the given driver velocity value 
  void setVelocity(double val);
  ///Returns the current driver velocity value from the robot's velocity
  double getVelocity();
  ///Returns value limits [xmin,xmax]
  void getLimits(double out[2]);
  ///Returns velocity limits [vmin,vmax]
  ///
  ///Return type: Tuple[float,float]
  void getVelocityLimits(double out[2]);
  ///Returns acceleration limits [amin,amax]
  ///
  ///Return type: Tuple[float,float]
  void getAccelerationLimits(double out[2]);
  ///Returns generalized torque limits [tmin,tmax]
  ///
  ///Return type: Tuple[float,float]
  void getTorqueLimits(double out[2]);

  int world;
  int robotIndex;
  Klampt::RobotModel* robotPtr;
  int index;
};

/** @brief A model of a dynamic and kinematic robot.
 *
 * Stores both constant information, like the reference placement of the links,
 * joint limits, velocity limits, etc, as well as a *current configuration*
 * and *current velocity* which are state-dependent.  Several functions depend
 * on the robot's current configuration and/or velocity.  To update that, use
 * the setConfig() and setVelocity() functions.  setConfig() also update's the
 * robot's link transforms via forward kinematics.  You may also use setDOFPosition
 * and setDOFVelocity for individual changes, but these are more expensive because
 * each call updates all of the affected the link transforms.
 *
 * It is important to understand that changing the configuration of the model
 * doesn't actually send a command to the physical / simulated robot.  Moreover,
 * the model does not automatically get updated when the physical / simulated
 * robot moves.  In essence, the model maintains temporary storage for performing
 * kinematics, dynamics, and planning computations, as well as for visualization.
 * 
 * The state of the robot is retrieved using getConfig/getVelocity calls, and
 * is set using setConfig/setVelocity.  Because many routines change the robot's
 * configuration, like IK and motion planning, a common design pattern is to
 * save/restore the configuration as follows::
 * 
 *     q = robot.getConfig()
 *     do some stuff that may touch the robot's configuration...
 *     robot.setConfig(q)
 *
 * 
 * The model maintains configuration/velocity/acceleration/torque limits.
 * However, these are not enforced by the model, so you can happily set
 * configurations outside the limits. Valid commands must rather be enforced 
 * by the planner / controller / simulator.
 * 
 * As elsewhere in Klampt, the mapping between links and drivers is not one-to
 * one. A driver is essentially an actuator and transmission, and for most links
 * a link is driven by a unique driver (e.g., a motor and gearbox). However,
 * there do exist certain cases in which a link is not driven at all (e.g., the
 * 6 virtual links of a floating-base robot), or multiple links are driven by
 * a single actuator (e.g., a parallel-bar mechanism or a compliant hand). 
 * There are also unusual drivers that introduce underactuated dynamics into
 * the system, such as a differential drive or Dubin's car mobile base.   Care
 * must be taken when sending commands to motor controllers (e.g., Klampt
 * Robot Interface Layer), which often work in the actuator space rather than
 * joint space.  (See :func:`configToDrivers`, :func:`configFromDrivers`,
 * :func:`velocityToDrivers`, :func:`velocityFromDrivers`).
 */
class RobotModel
{
 public:
  RobotModel();
  ///Loads the robot from the file fn
  ///
  ///Returns:
  ///
  ///    True if successful, False if failed.
  /// 
  bool loadFile(const char* fn);
  ///Saves the robot to the file fn.  Geometries may be saved as well.
  ///
  ///If ``geometryPrefix == None`` (default), the geometry is not saved. 
  ///Otherwise, the geometry of each link will be saved to files named
  ///``geometryPrefix+name``, where ``name`` is either the name of the
  ///geometry file that was loaded, or ``[link_name].off``
  bool saveFile(const char* fn,const char* geometryPrefix=NULL);
  ///Returns the ID of the robot in its world
  ///
  ///.. note::
  ///
  ///    The world ID is not the same as the robot index.
  ///
  int getID() const;
  ///Gets the name of the robot 
  const char* getName() const;
  ///Sets the name of the robot
  void setName(const char* name);
  ///Returns the number of links = number of DOF's.
  int numLinks();
  ///Returns a reference to the link by index or name
  RobotModelLink link(int index);
  //note: only the last overload docstring is added to the documentation
  ///Returns a reference to the link by index or name
  RobotModelLink link(const char* name);
  ///Returns the number of drivers.
  int numDrivers();
  ///Returns a reference to the driver by index or name
  RobotModelDriver driver(int index);
  //note: only the last overload docstring is added to the documentation
  ///Returns a reference to the driver by index or name
  RobotModelDriver driver(const char* name);
  ///Returns the joint type of the joint connecting the link to its parent,
  ///where the link is identified by index or by name
  const char* getJointType(int index);
  //note: only the last overload docstring is added to the documentation
  ///Returns the joint type of the joint connecting the link to its parent,
  ///where the link is identified by index or by name
  const char* getJointType(const char* name);

  //kinematic and dynamic properties
  ///Retrieves the current configuration of the robot model.
  ///
  ///Return type: Vector
  void getConfig(std::vector<double>& out);
  ///Retreives the current velocity of the robot model.
  ///
  ///Return type: Vector
  void getVelocity(std::vector<double>& out);
  ///Sets the current configuration of the robot.  Input q is a vector of length numLinks().  This also updates forward kinematics of all links.
  ///
  ///Again, it is important to realize that the RobotModel is not the same as a simulated robot, and this will not change the simulation world.
  ///Many functions such as IK and motion planning use the RobotModel configuration as a temporary variable, so if you need to keep the
  ///configuration through a robot-modifying function call, you should call ``q = robot.getConfig()`` before the call, and then ``robot.setConfig(q)``
  ///after it.
  void setConfig(const std::vector<double>& q);
  ///Sets the current velocity of the robot model.  Like the configuration, this is also essentially a temporary variable. 
  void setVelocity(const std::vector<double>& dq);
  ///Returns a pair (qmin,qmax) of min/max joint limit vectors
  ///
  ///Return type: Tuple[Vector,Vector]
  void getJointLimits(std::vector<double>& out,std::vector<double>& out2);
  ///Sets the min/max joint limit vectors (must have length numLinks())
  void setJointLimits(const std::vector<double>& qmin,const std::vector<double>& qmax);
  ///Returns the velocity limit vector vmax, the constraint is :math:`|dq[i]| \leq vmax[i]`
  ///
  ///Return type: Vector
  void getVelocityLimits(std::vector<double>& out);
  ///Sets the velocity limit vector vmax, the constraint is :math:`|dq[i]| \leq vmax[i]`
  void setVelocityLimits(const std::vector<double>& vmax);
  ///Returns the acceleration limit vector amax, the constraint is :math:`|ddq[i]| \leq amax[i]`
  ///
  ///Return type: Vector
  void getAccelerationLimits(std::vector<double>& out);
  ///Sets the acceleration limit vector amax, the constraint is :math:`|ddq[i]| \leq amax[i]`
  void setAccelerationLimits(const std::vector<double>& amax);
  ///Returns the torque limit vector tmax, the constraint is :math:`|torque[i]| \leq tmax[i]`
  ///
  ///Return type: Vector
  void getTorqueLimits(std::vector<double>& out);
  ///Sets the torque limit vector tmax, the constraint is :math:`|torque[i]| \leq tmax[i]`
  void setTorqueLimits(const std::vector<double>& tmax);
  ///Sets a single DOF's position (by index or by name).
  ///
  ///.. note::
  ///
  ///    If you are setting several joints at once, use setConfig because this
  ///   function computes forward kinematics for all descendant links each time 
  ///   it is called.
  ///
  void setDOFPosition(int i,double qi);
  ///Sets a single DOF's position (by index or by name).
  ///
  ///.. note::
  ///
  ///    If you are setting several joints at once, use setConfig because this
  ///    function computes forward kinematics for all descendant links each time 
  ///    it is called.
  ///
  void setDOFPosition(const char* name,double qi);
  ///Returns a single DOF's position
  double getDOFPosition(int i);
  ///Returns a single DOF's position (by name)
  double getDOFPosition(const char* name);

  //dynamics functions
  ///Returns the 3D center of mass at the current config
  ///
  ///Return type: Vector3
  void getCom(double out[3]);
  ///Returns the 3D velocity of the center of mass at the current config / velocity
  ///
  ///Return type: Vector3
  void getComVelocity(double out[3]);
  ///Computes the Jacobian matrix of the current center of mass
  ///
  ///Returns:
  /// 
  ///    ndarray: a 3xn matrix J such that np.dot(J,dq) gives the
  ///    COM velocity at the currene configuration
  /// 
  ///Return type: np.ndarray
  void getComJacobian(double** np_out2,int* m,int* n);
  ///Returns the Jacobian matrix of the current center of mass w.r.t. some
  ///links of the robot
  ///
  ///Returns:
  ///
  ///    ndarray: a 3xlen(links) matrix J such that np.dot(J,dqlinks)
  ///    gives the COM velocity at the current configuration, and dqlinks
  ///    is the array of velocities of the links given by `links`
  ///
  ///Return type: np.ndarray
  void getComJacobianCols(const std::vector<int>& links,double** np_out2,int* m,int* n);
  ///Computes the 3D linear momentum vector
  ///
  ///Return type: Vector3
  void getLinearMomentum(double out[3]);
  ///Computes the 3D angular momentum vector
  ///
  ///Return type: Vector3
  void getAngularMomentum(double out[3]);
  ///Computes the kinetic energy at the current config / velocity
  double getKineticEnergy();
  ///Computes the 3x3 total inertia matrix of the robot
  ///
  ///Return type: np.ndarray
  void getTotalInertia(double** np_out2,int* m,int* n);
  ///Computes the nxn mass matrix B(q).
  ///
  ///Takes O(n^2) time
  ///
  ///Return type: np.ndarray
  void getMassMatrix(double** np_out2,int* m,int* n);
  ///Computes the inverse of the nxn mass matrix B(q)^-1.
  ///
  ///Takes O(n^2) time, which is much faster than inverting the result of getMassMatrix
  ///
  ///Return type: np.ndarray
  void getMassMatrixInv(double** np_out2,int* m,int* n);
  ///Computes the derivative of the nxn mass matrix with respect to q_i.
  ///
  ///Takes O(n^3) time.
  ///
  ///Return type: np.ndarray
  void getMassMatrixDeriv(int i,double** np_out2,int* m,int* n);
  ///Computes the derivative of the nxn mass matrix with respect to t, given the
  ///robot's current velocity.
  ///
  ///Takes O(n^4) time.
  ///
  ///Return type: np.ndarray
  void getMassMatrixTimeDeriv(double** np_out2,int* m,int* n);
  ///Computes the Coriolis force matrix C(q,dq) for current config and velocity.
  ///
  ///Takes O(n^2) time.
  ///
  ///Return type: np.ndarray
  void getCoriolisForceMatrix(double** np_out2,int* m,int* n);
  ///Computes the Coriolis forces C(q,dq)*dq for current config and velocity.
  ///
  ///Takes O(n) time, which is faster than computing matrix and doing the product.
  ///
  ///("Forces" is somewhat of a misnomer; the result is a joint torque vector)
  ///
  ///Return type: Vector
  void getCoriolisForces(std::vector<double>& out);
  ///Computes the generalized gravity vector G(q) for the given workspace
  ///gravity vector g (usually (0,0,-9.8)). 
  ///
  ///.. note::
  ///
  ///    "Forces" is somewhat of a misnomer; the result is a vector of joint
  ///    torques.
  ///
  ///
  ///Returns:
  /// 
  ///    list of floats: the n-element generalized gravity vector at the
  ///    robot's current configuration.
  ///
  ///Return type: Vector
  void getGravityForces(const double g[3],std::vector<double>& out);
  ///Computes the inverse dynamics.  Uses Recursive Newton Euler solver and
  ///takes O(n) time.
  ///
  ///Specifically, solves for :math:`\tau` in the (partial) dynamics equation:
  ///
  ///.. math::
  ///
  ///    `B(q) \ddot{q} + C(q,\dot{q}) = \tau`
  ///
  ///
  ///.. note::
  ///
  ///    Does not include gravity term G(q).  getGravityForces(g) will 
  ///    need to be added to the result.
  ///
  ///
  ///Returns:
  ///
  ///    list of floats: the n-element torque vector that would produce
  ///    the joint accelerations ddq in the absence of external forces.
  ///
  ///Return type: Vector 
  void torquesFromAccel(const std::vector<double>& ddq,std::vector<double>& out);
  ///Computes the foward dynamics.  Uses Recursive Newton Euler solver and
  ///takes O(n) time.
  //
  ///Specifically, solves for :math:`\ddot{q}` in the (partial) dynamics
  ///equation:
  ///
  ///.. math::
  ///
  ///    `B(q) \ddot{q} + C(q,\dot{q}) = \tau`
  ///
  ///
  ///.. note::
  ///
  ///    Does not include gravity term G(q).  getGravityForces(g) will 
  ///    need to be subtracted from the argument t.
  ///
  ///
  ///Returns:
  /// 
  ///    list of floats: the n-element joint acceleration vector that would
  ///    result from joint torques t in the absence of external forces.
  ///
  ///Return type: Vector
  void accelFromTorques(const std::vector<double>& t,std::vector<double>& out);

  ///Interpolates smoothly between two configurations, properly taking into
  ///account nonstandard joints.
  ///
  ///Returns:
  /// 
  ///    The n-element configuration that is u fraction of the way from a to b.
  ///
  ///Return type: Vector
  void interpolate(const std::vector<double>& a,const std::vector<double>& b,double u,std::vector<double>& out);
  ///Computes a distance between two configurations, properly taking into account nonstandard joints
  double distance(const std::vector<double>& a,const std::vector<double>& b);
  ///Returns the configuration derivative at a as you interpolate toward
  ///b at unit speed.
  ///
  ///Return type: Vector
  void interpolateDeriv(const std::vector<double>& a,const std::vector<double>& b,std::vector<double>& out);

  ///Samples a random configuration and updates the robot's pose.  Properly
  ///handles non-normal joints and handles DOFs with infinite bounds
  ///using a centered Laplacian distribution with the given scaling term.
  ///
  ///.. note::
  ///
  ///    Python random module seeding does not affect the result.
  ///
  ///
  void randomizeConfig(double unboundedScale=1.0);

  ///Converts a full configuration (length numLinks()) to a list of driver values
  ///(length numDrivers()).
  ///
  ///Return type: Vector
  void configToDrivers(const std::vector<double>& config,std::vector<double>& out);
  ///Converts a full velocity vector (length numLinks()) to a list of driver
  ///velocities (length numDrivers()).
  ///
  ///Return type: Vector
  void velocityToDrivers(const std::vector<double>& velocities,std::vector<double>& out);
  ///Converts a list of driver values (length numDrivers()) to a full configuration
  ///(length numLinks()).
  ///
  ///Return type: Vector
  void configFromDrivers(const std::vector<double>& driverValues,std::vector<double>& out);
  ///Converts a list of driver velocities (length numDrivers()) to a full velocity
  ///vector (length numLinks()).
  ///
  ///Return type: Vector
  void velocityFromDrivers(const std::vector<double>& driverVelocities,std::vector<double>& out);

  //geometry functions
  ///Queries whether self collisions between two links is enabled
  bool selfCollisionEnabled(int link1,int link2);
  ///Enables/disables self collisions between two links (depending on value)
  void enableSelfCollision(int link1,int link2,bool value);
  ///Returns true if the robot is in self collision (faster than manual testing)
  /// 
  bool selfCollides();
  ///Draws the robot geometry. If keepAppearance=true, the current appearance is 
  ///honored. Otherwise, only the raw geometry is drawn.
  ///
  ///PERFORMANCE WARNING: if keepAppearance is false, then this does not properly
  ///reuse OpenGL display lists.  A better approach to changing the robot's
  ///appearances is to set the link Appearance's directly.
  void drawGL(bool keepAppearance=true);

  ///Sets self to a reduced version of robot, where all fixed DOFs are eliminated.
  ///The return value is a map from the original robot DOF indices to the reduced
  ///DOFs.
  ///
  ///Note that any geometries fixed to the world will disappear.
  ///
  ///Return type: List[int]
  void reduce(const RobotModel& robot,std::vector<int>& out);
  ///Mounts a sub-robot onto a link, with its origin at a given local transform (R,t).
  ///The sub-robot's links will be renamed to subRobot.getName() + ':' + link.getName()
  ///unless subRobot.getName() is '', in which case the link names are preserved.
  void mount(int link,const RobotModel& subRobot,const double R[9],const double t[3]);

  ///Returns the number of sensors
  int numSensors() const; 
  SensorModel _sensor(int index);
  SensorModel _sensor(const char* name);
  ///Adds a new sensor with a given name and type
  ///
  ///Returns:
  ///
  ///    The new sensor.
  ///
  SensorModel addSensor(const char* name,const char* type);

  int world;
  int index;
  Klampt::RobotModel* robot;
  bool dirty_dynamics;
};

/** @brief A sensor on a simulated robot.
 * 
 * Kinematic models of sensors are retrieved using :meth:`RobotModel.sensor`
 * and can be created using  :meth:`RobotModel.addSensor`.  
 * 
 * Physically-simulated sensors are retrieved from a controller using
 * :meth:`SimRobotController.sensor`, and can be created using
 * :meth:`SimRobotController.addSensor`.  
 *
 * Some types of sensors can be kinematically-simulated such that they make
 * sensible measurements.
 * To use kinematic simulation, you may arbitrarily set the robot's position,
 * call :meth:`kinematicReset`, and then call :meth:`kinematicSimulate`.
 * Subsequent calls assume the robot is being driven along a coherent trajectory
 * until the next :meth:`kinematicReset` is called.  This is necessary for
 * sensors that estimate accelerations, e.g., ForceTorqueSensor, Accelerometer
 *
 * Physically-simulated sensors are automatically updated through the
 * :meth:`Simulator.simulate` call.
 * 
 * Use  :meth:`getMeasurements` to get the currently simulated measurement
 * vector. You may get garbage measurements before kinematicSimulate /
 * Simulator.simulate are called.
 * 
 * LaserSensor, CameraSensor, TiltSensor, AccelerometerSensor, GyroSensor,
 * JointPositionSensor, JointVelocitySensor support kinematic simulation mode.
 * FilteredSensor and TimeDelayedSensor also work.  The force-related sensors 
 * (ContactSensor and ForceTorqueSensor) return 0's in kinematic simulation.
 *
 * To use get/setSetting, you will need to know the sensor attribute names
 * and types as described in `the Klampt sensor documentation <https://github.com/krishauser/Klampt/blob/master/Cpp/docs/Manual-Control.md#sensors>`_
 * (same as in the world or sensor XML file). Common settings include:
 * 
 * - rate (float): how frequently the sensor is simulated
 * - enabled (bool): whether the simulator simulates this sensor
 * - link (int): the link on which this sensor lies (-1 for world)
 * - Tsensor (se3 transform, serialized with loader.write_se3(T)): the transform
 *    of the sensor on the robot / world.
 * 
 */
class SensorModel
{
 public:
  SensorModel(const RobotModel& robot,Klampt::SensorBase* sensor);

  ///Returns the name of the sensor
  std::string getName() const;
  ///Sets the name of the sensor
  void setName(const std::string& name);
  ///Returns the type of the sensor
  std::string getType() const;
  ///Returns the model of the robot to which this belongs
  RobotModel robot();
  ///Returns a list of names for the measurements (one per measurement).
  std::vector<std::string> measurementNames();
  ///Returns an array of measurements from the previous simulation (or
  ///kinematicSimulate) timestep
  ///
  ///Return type: np.ndarray
  void getMeasurements(double** np_out,int* m);
  ///Returns all setting names
  ///
  std::vector<std::string> settings();
  ///Returns the value of the named setting (you will need to manually parse this)
  std::string getSetting(const std::string& name);
  ///Sets the value of the named setting (you will need to manually cast an int/float/etc to a str)
  void setSetting(const std::string& name,const std::string& val);
  ///Return whether the sensor is enabled during simulation (helper for getSetting)
  bool getEnabled();
  ///Sets whether the sensor is enabled in physics simulation.
  void setEnabled(bool enabled);
  ///Returns the link on which the sensor is mounted
  RobotModelLink _getLink();
  ///Sets the link on which the sensor is mounted (helper for setSetting)
  void _setLink(const RobotModelLink& link);
  ///Sets the link on which the sensor is mounted (helper for setSetting)
  void _setLink(int link);
  ///Returns the local transform of the sensor on the robot's link. 
  ///(helper for getSetting)
  ///
  ///If the sensor doesn't have a transform (such as a joint position or
  ///torque sensor) an exception will be raised.
  ///
  ///Return type: RigidTransform
  void getTransform(double out[9],double out2[3]);
  ///Returns the world transform of the sensor given the robot's current
  ///configuration. (helper for getSetting)
  ///
  ///If the sensor doesn't have a transform (such as a joint position or
  ///torque sensor) an exception will be raised.
  ///
  ///Return type: RigidTransform
  void getTransformWorld(double out[9],double out2[3]);
  ///Sets the local transform of the sensor on the robot's link.
  ///(helper for setSetting)
  ///
  ///If the sensor doesn't have a transform (such as a joint position or
  ///torque sensor) an exception will be raised.
  void setTransform(const double R[9],const double t[3]);
  //note: only the last overload docstring is added to the documentation
  ///Draws a sensor indicator using OpenGL.  If measurements are given,
  ///the indicator is drawn as though these are the latest measurements,
  ///otherwise only an indicator is drawn.
  void drawGL();
  //note: only the last overload docstring is added to the documentation
  ///Draws a sensor indicator using OpenGL.  If measurements are given,
  ///the indicator is drawn as though these are the latest measurements,
  ///otherwise only an indicator is drawn.
  void drawGL(double* np_array,int m);

  ///simulates / advances the kinematic simulation
  void kinematicSimulate(WorldModel& world,double dt);
  void kinematicSimulate(double dt);
  ///resets a kinematic simulation so that a new initial condition can be set
  void kinematicReset();

  RobotModel robotModel;
  Klampt::SensorBase* sensor;
};

/** @brief A rigid movable object.
 *
 * A rigid object has a name, geometry, appearance, mass, surface properties, and current
 * transform / velocity.
 *
 * State is retrieved/set using get/setTransform, and get/setVelocity
 */
class RigidObjectModel
{
 public:
  RigidObjectModel();
  ///Loads the object from the file fn
  bool loadFile(const char* fn);
  ///Saves the object to the file fn.  If geometryName is given, the geometry is saved to that file.
  bool saveFile(const char* fn,const char* geometryName=NULL);
  ///Returns the ID of the rigid object in its world 
  ///
  ///.. note::
  ///
  ///    The world ID is not the same as the rigid object index.
  ///
  ///
  int getID() const;
  const char* getName() const;
  void setName(const char* name);
  ///Returns a reference to the geometry associated with this object
  Geometry3D geometry();
  ///Returns a reference to the appearance associated with this object
  Appearance appearance();
  ///Returns a copy of the Mass of this rigid object. 
  ///
  ///.. note::
  ///
  ///    To change the mass properties, you should call ``m=object.getMass()``,
  ///    change the desired properties in m, and then ``object.setMass(m)``
  ///
  ///
  Mass getMass();
  void setMass(const Mass& mass);
  ///Returns a copy of the ContactParameters of this rigid object.
  ///
  ///.. note::
  ///
  ///    To change the contact parameters, you should call
  ///    ``p=object.getContactParameters()``, change the desired properties in
  ///    p, and then call ``object.setContactParameters(p)``
  ///
  ///
  ContactParameters getContactParameters();
  void setContactParameters(const ContactParameters& params);
  ///Retrieves the rotation / translation of the rigid object (R,t)
  ///
  ///Returns:
  /// 
  ///    se3 object: a pair (R,t), with R a 9-list and t a 3-list of floats,
  ///    giving the transform to world coordinates.
  /// 
  ///Return type: RigidTransform
  void getTransform(double out[9],double out2[3]);
  ///Sets the rotation / translation (R,t) of the rigid object
  void setTransform(const double R[9],const double t[3]);
  ///Retrieves the (angular velocity, velocity) of the rigid object.
  ///
  ///Returns:
  /// 
  ///    A pair of 3-lists (w,v) where w is the angular velocity
  ///    vector and v is the translational velocity vector (both in world
  ///    coordinates)
  /// 
  ///Return type: Tuple[Vector3,Vector3]
  void getVelocity(double out[3],double out2[3]);
  ///Sets the (angular velocity, velocity) of the rigid object.
  void setVelocity(const double angularVelocity[3],const double velocity[3]);
  ///Draws the object's geometry. If keepAppearance=true, the current appearance is honored.
  ///Otherwise, only the raw geometry is drawn.
  ///
  ///PERFORMANCE WARNING: if keepAppearance is false, then this does not properly reuse
  ///OpenGL display lists.  A better approach is to change the object's Appearance
  ///directly.
  void drawGL(bool keepAppearance=true);

  int world;
  int index;
  Klampt::RigidObjectModel* object;
};

/** @brief Static environment geometry.
 */
class TerrainModel
{
 public:
  TerrainModel();
  ///Loads the terrain from the file fn
  bool loadFile(const char* fn);
  ///Saves the terrain to the file fn.  If geometryName is given, the geometry is saved to that file.
  bool saveFile(const char* fn,const char* geometryName=NULL);
  ///Returns the ID of the terrain in its world
  ///
  ///.. note::
  ///
  ///    The world ID is not the same as the terrain index.
  ///
  int getID() const;
  const char* getName() const;
  void setName(const char* name);
  ///Returns a reference to the geometry associated with this object
  Geometry3D geometry();
  ///Returns a reference to the appearance associated with this object
  Appearance appearance();
  ///Changes the friction coefficient for this terrain
  void setFriction(double friction);
  ///Draws the object's geometry. If keepAppearance=true, the current appearance is honored.
  ///Otherwise, only the raw geometry is drawn. 
  ///
  ///PERFORMANCE WARNING: if keepAppearance is false, then this does not properly
  ///reuse OpenGL display lists.  A better approach is to change the object's Appearance
  ///directly.
  void drawGL(bool keepAppearance=true);

  int world;
  int index;
  Klampt::TerrainModel* terrain;
};

/** @brief The main world class, containing robots, rigid objects, and static
 * environment geometry.
 * 
 * Attributes:
 * 
 *     - robots: a list of RobotModel instances
 *     - rigidObjects: a list of RigidObjectModel instances
 *     - terrains: a list of TerrainModel instances
 *
 *
 * .. note:
 * 
 *     Although a WorldModel instance is typically called a "world" it is
 *     just a model and does not have to reflect the state of a physical world.
 *     The state of robots and objects in the world can be changed at will --
 *     in fact planners and simulators will query and modify the state of a
 *     WorldModel during their operation.
 * 
 *     To keep around some "authoritative" world, you can keep around a copy
 *     (use ``WorldModel.copy()``) or ``config.get_config(world)`` using the
 *     :mod:`klampt.model.config` module.
 *
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
 *
 * To save/restore the state of the model, you must manually maintain copies of
 * the states of whichever objects you wish to save/restore.
 */
class WorldModel
{
 public:
  WorldModel();
  WorldModel(void* ptrRobotWorld);
  WorldModel(const WorldModel& w);
  //Note: only the last docstring is added
  ///Creates a WorldModel. 
  ///
  ///- Given no arguments, creates a new world. 
  ///- Given another WorldModel instance, creates a reference to an
  ///  existing world.  (To create a copy, use the copy() method.)
  ///- Given a string, loads from a file. A PyException is raised on failure.
  ///- Given a pointer to a C++ RobotWorld structure, a reference to that
  ///  structure is returned. (This is advanced usage, seen only when
  ///  interfacing C++ and Python code)
  ///
  WorldModel(const char* fn); 
  ~WorldModel();
  ///Sets this WorldModel to a reference to w
  const WorldModel& operator = (const WorldModel& w);
  ///Creates a copy of the world model.  Note that geometries and appearances
  ///are shared, so this is very quick.
  WorldModel copy();
  ///Reads from a world XML file.
  ///
  ///Returns:
  ///
  ///    True if successful, False if failed.
  /// 
  bool readFile(const char* fn);
  ///Alias of readFile
  bool loadFile(const char* fn);
  ///Saves to a world XML file.  Elements in the world will be saved to a
  ///folder.
  ///
  ///If elementDir is provided, then robots, terrains, etc.
  ///will be saved there.  Otherwise they will be saved to a folder with the same base
  ///name as fn (without the trailing .xml)
  bool saveFile(const char* fn,const char* elementDir=NULL);
  ///Returns the number of robots
  int numRobots();
  ///Returns the number of links on the given robot
  int numRobotLinks(int robot);
  ///Returns the number of rigid objects
  int numRigidObjects();
  ///Returns the number of terrains
  int numTerrains();
  ///Returns the total number of world ids
  int numIDs();
  ///Returns a RobotModel in the world by index or name.
  RobotModel robot(int index);
  //note: only the last overload docstring is added to the documentation
  ///Returns a RobotModel in the world by index or name.
  RobotModel robot(const char* name);
  ///Returns a RobotModelLink of some RobotModel in the world by index or name.
  RobotModelLink robotLink(int robot,int index);
  ///Returns a RobotModelLink of some RobotModel in the world by index or name.
  RobotModelLink robotLink(const char* robot,const char* name);
  ///Returns a RigidObjectModel in the world by index or name.
  RigidObjectModel rigidObject(int index);
  //note: only the last overload docstring is added to the documentation
  ///Returns a RigidObjectModel in the world by index or name.
  RigidObjectModel rigidObject(const char* name);
  ///Returns a TerrainModel in the world by index or name.
  TerrainModel terrain(int index);
  //note: only the last overload docstring is added to the documentation
  ///Returns a TerrainModel in the world by index or name.
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
  ///as terrains. 
  ///
  ///Returns:
  ///
  ///    The element's ID, or -1 if loading failed.
  /// 
  int loadElement(const char* fn);
  ///Adds a copy of the given robot, rigid object, or terrain to this world, either from
  ///this WorldModel or another.
  RobotModel add(const char* name,const RobotModel& robot);
  ///Adds a copy of the given robot, rigid object, or terrain to this world, either from
  ///this WorldModel or another.
  RigidObjectModel add(const char* name,const RigidObjectModel& obj);
  //note: only the last overload docstring is added to the documentation
  ///Adds a copy of the given robot, rigid object, or terrain to this world, either from
  ///this WorldModel or another.
  TerrainModel add(const char* name,const TerrainModel& terrain);
  ///Removes a robot, rigid object, or terrain from the world.  It must be in this world or an exception
  ///is raised.
  ///
  ///IMPORTANT:
  ///
  ///    All other RobotModel, RigidObjectModel, or TerrainModel references
  ///    will be invalidated.
  ///
  void remove(const RobotModel& robot);
  ///Removes a robot, rigid object, or terrain from the world.  It must be in this world or an exception
  ///is raised.
  ///
  ///IMPORTANT:
  ///
  ///    All other RobotModel, RigidObjectModel, or TerrainModel references
  ///    will be invalidated.
  void remove(const RigidObjectModel& object);
  //note: only the last overload docstring is added to the documentation
  ///Removes a robot, rigid object, or terrain from the world.  It must be in this world or an exception
  ///is raised.
  ///
  ///IMPORTANT:
  ///
  ///    All other RobotModel, RigidObjectModel, or TerrainModel references
  ///    will be invalidated.
  void remove(const TerrainModel& terrain);
  /// Returns either 'robot', 'robotLink', 'rigidObject', or 'terrain' 
  std::string entityType(int id);
  ///Retrieves the name for a given element ID
  std::string getName(int id);
  /// Sets the name for a given element ID. 
  void setName(int id,const char* name);
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
  ///initialization may take a long time.
  ///
  ///Note that even when this flag is off, the collision acceleration data
  ///structures will indeed be initialized the first time that geometry collision,
  ///distance, or ray-casting routines are called.
  void enableInitCollisions(bool enabled);

  //WARNING: do not modify this member directly
  int index;
};

#endif
