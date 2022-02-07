#ifndef ROBOTSIM_H
#define ROBOTSIM_H

/** @file robotsim.h
 * @brief C++ bindings for robot/world simulation. */

#include <stddef.h>
#include "robotmodel.h"

//declarations for internal objects
namespace Klampt {
  class SensorBase;
  class Simulator;
  class SimRobotController;
  class ODEGeometry;
}
typedef struct dxBody *dBodyID;
typedef struct dxJoint *dJointID;

//forward declarations
class SimRobotSensor;
class SimRobotController;
class SimBody;
class Simulator;

using namespace std;

/** @brief A sensor on a simulated robot.  Retrieve one from the controller
 * using :meth:`SimRobotController.sensor`, or create a new one
 * using :meth:`SimRobotController.addSensor`.  You may also use
 * kinematically-simulated sensors using :meth:`RobotModel.sensor` or create
 * a new one using :meth:`RobotModel.addSensor`.
 *
 * Use  :meth:`getMeasurements` to get the currently simulated measurement
 * vector.
 *
 * Sensors are automatically updated through the :meth:`Simulator.simulate` call,
 * and :meth:`getMeasurements` retrieves the updated values.  As a result,
 * you may get garbage measurements before the first Simulator.simulate call is
 * made.
 * 
 * There is also a mode for doing kinematic simulation, which is supported
 * (i.e., makes sensible measurements) for some types of sensors when just 
 * a robot / world model is given. This is similar to Simulation.fakeSimulate
 * but the entire controller structure is bypassed.  You can arbitrarily set the
 * robot's position, call :meth:`kinematicReset`, and then call
 * :meth:`kinematicSimulate`.  Subsequent calls assume the robot is being
 * driven along a trajectory until the next :meth:`kinematicReset` is called.
 * 
 * LaserSensor, CameraSensor, TiltSensor, AccelerometerSensor, GyroSensor,
 * JointPositionSensor, JointVelocitySensor support kinematic simulation mode.
 * FilteredSensor and TimeDelayedSensor also work.  The force-related sensors 
 * (ContactSensor and ForceTorqueSensor) return 0's in kinematic simulation.
 *
 * To use get/setSetting, you will need to know the sensor attribute names
 * and types as described in `the Klampt sensor documentation <https://github.com/krishauser/Klampt/blob/master/Documentation/Manual-Control.md#sensors>`_
 * (same as in the world or sensor XML file). Common settings include:
 * 
 * - rate (float): how frequently the sensor is simulated
 * - enabled (bool): whether the simulator simulates this sensor
 * - link (int): the link on which this sensor lies (-1 for world)
 * - Tsensor (se3 transform, serialized with loader.write_se3(T)): the transform
 *    of the sensor on the robot / world.
 * 
 */
class SimRobotSensor
{
 public:
  SimRobotSensor(const RobotModel& robot,Klampt::SensorBase* sensor);

  ///Returns the name of the sensor
  std::string name();
  ///Returns the type of the sensor
  std::string type();
  ///Returns the model of the robot to which this belongs
  RobotModel robot();
  ///Returns a list of names for the measurements (one per measurement).
  std::vector<std::string> measurementNames();
  ///Returns an array of measurements from the previous simulation (or
  ///kinematicSimulate) timestep
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
  ///Sets whether the sensor is enabled (helper for setSetting)
  void setEnabled(bool enabled);
  ///Returns the link on which the sensor is mounted (helper for getSetting)
  RobotModelLink getLink();
  ///Sets the link on which the sensor is mounted (helper for setSetting)
  void setLink(const RobotModelLink& link);
  ///Sets the link on which the sensor is mounted (helper for setSetting)
  void setLink(int link);
  ///Returns the local transform of the sensor on the robot's link. 
  ///(helper for getSetting)
  ///
  ///If the sensor doesn't have a transform (such as a joint position or
  ///torque sensor) an exception will be raised.
  void getTransform(double out[9],double out2[3]);
  ///Returns the world transform of the sensor given the robot's current
  ///configuration. (helper for getSetting)
  ///
  ///If the sensor doesn't have a transform (such as a joint position or
  ///torque sensor) an exception will be raised.
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

/** @brief A controller for a simulated robot.
 *
 * By default a SimRobotController has three possible modes:
 * 
 * - Motion queue + PID mode: the controller has an internal trajectory
 *   queue that may be added to and modified.  This queue supports
 *   piecewise linear interpolation, cubic interpolation, and time-optimal
 *   move-to commands.
 * - PID mode: the user controls the motor's PID setpoints directly
 * - Torque control: the user controlls the motor torques directly.
 *
 * The "standard" way of using this is in move-to mode which accepts
 * a milestone (setMilestone) or list of milestones (repeated calls to
 * addMilestone) and interpolates dynamically from the current
 * configuration/velocity.  To handle disturbances, a PID loop is run
 * automatically at the controller's specified rate.
 *
 * To get finer-grained control over the motion queue's timing, you may
 * use the setLinear/setCubic/addLinear/addCubic functions.  In these functions
 * it is up to the user to respect velocity, acceleration, and torque limits.
 *  
 * Whether in motion queue or PID mode, the constants of the PID loop
 * are initially set in the robot file.  You can programmatically 
 * tune these via the setPIDGains function.
 *
 * Arbitrary trajectories can be tracked by using setVelocity over short time
 * steps.  Force controllers can be implemented using setTorque, again using
 * short time steps. 
 * 
 * If the setVelocity, setTorque, or setPID command are called, the motion queue 
 * behavior will be completely overridden.  To reset back to motion queue control, 
 * setManualMode(False) must be called first.
 *
 * Individual joints cannot be addressed with mixed motion queue mode and
 * torque/PID mode.  However, you can mix PID and torque mode between
 * different joints with a workaround::
 * 
 * <pre>
 *    \# setup by zeroing out PID constants for torque controlled joints
 *    pid_joint_indices = [...]
 *    torque_joint_indices = [...] # complement of pid_joint_indices
 *    kp,ki,kp = controller.getPIDGains()
 *    for i in torque_joint_indices:  #turn off PID gains here
 *       kp[i] = ki[i] = kp[i] = 0
 *    
 *    \# to send PID command (qcmd,dqcmd) and torque commands tcmd, use
 *    \# a PID command with feedforward torques.  First we build a whole-robot
 *    \# command:
 *    qcmd_whole = [0]*controller.model().numLinks()
 *    dqcmd_whole = [0]*controller.model().numLinks()
 *    tcmd_whole = [0]*controller.model().numLinks()
 *    for i,k in enumerate(pid_joint_indices):
 *        qcmd_whole[k],dqcmd_whole[i] = qcmd[i],dqcmd[i]
 *    for i,k in enumerate(torque_joint_indices):
 *        tcmd_whole[k] = tcmd[i]
 *    \# Then we send it to the controller
 *    controller.setPIDCommand(qcmd_whole,dqcmd_whole,tcmd_whole)
 *
 * </pre>
 */
class SimRobotController
{
 public:
  SimRobotController();
  ~SimRobotController();
  ///Retrieves the robot model associated with this controller
  RobotModel model();
  /// Sets the current feedback control rate, in s
  void setRate(double dt);
  /// Returns The current feedback control rate, in s
  ///
  double getRate();

  /// Returns The current commanded configuration (size model().numLinks())
  ///
  void getCommandedConfig(std::vector<double>& out);
  /// Returns The current commanded velocity (size model().numLinks())
  ///
  void getCommandedVelocity(std::vector<double>& out);
  /// Returns The current commanded (feedforward) torque
  ///     (size model().numDrivers())
  ///
  void getCommandedTorque(std::vector<double>& out);

  /// Returns The current "sensed" configuration from the simulator
  ///     (size model().numLinks())
  ///
  void getSensedConfig(std::vector<double>& out);
  /// Returns The current "sensed" velocity from the simulator
  ///     (size model().numLinks())
  ///
  void getSensedVelocity(std::vector<double>& out);
  /// Returns The current "sensed" (feedback) torque from the simulator. 
  ///     (size model().numDrivers())
  ///
  /// Note: a default robot doesn't have a torque sensor, so this will be 0
  void getSensedTorque(std::vector<double>& out);

  /// Returns a sensor by index or by name.  If out of bounds or unavailable,
  /// a null sensor is returned (i.e., SimRobotSensor.name() or
  /// SimRobotSensor.type()) will return the empty string.)
  SimRobotSensor sensor(int index);
  //note: only the last overload docstring is added to the documentation
  /// Returns a sensor by index or by name.  If out of bounds or unavailable,
  ///     a null sensor is returned (i.e., SimRobotSensor.name() or
  ///     SimRobotSensor.type()) will return the empty string.)
  SimRobotSensor sensor(const char* name);
  ///Adds a new sensor with a given name and type
  ///
  /// Returns:
  ///
  ///     The new sensor.
  ///
  SimRobotSensor addSensor(const char* name,const char* type);
  
  /// Returns a custom command list
  std::vector<std::string> commands();
  /// Sends a custom string command to the controller
  bool sendCommand(const std::string& name,const std::string& args);

  /// Returns all valid setting names
  std::vector<std::string> settings();
  /// Returns a setting of the controller
  std::string getSetting(const std::string& name);
  /// Sets a setting of the controller
  bool setSetting(const std::string& name,const std::string& val);

  /// Uses a dynamic interpolant to get from the current state to the
  /// desired milestone (with optional ending velocity).  This interpolant
  /// is time-optimal with respect to the velocity and acceleration bounds.
  ///
  /// Arguments have size model().numLinks().
  void setMilestone(const std::vector<double>& q);
  //note: only the last overload docstring is added to the documentation
  /// Uses a dynamic interpolant to get from the current state to the
  /// desired milestone (with optional ending velocity).  This interpolant
  /// is time-optimal with respect to the velocity and acceleration bounds.
  void setMilestone(const std::vector<double>& q,const std::vector<double>& dq);
  /// Same as setMilestone, but appends an interpolant onto an internal
  /// motion queue starting at the current queued end state.
  ///
  /// Arguments have size model().numLinks().
  void addMilestone(const std::vector<double>& q);
  //note: only the last overload docstring is added to the documentation
  /// Same as setMilestone, but appends an interpolant onto an internal
  /// motion queue starting at the current queued end state.
  void addMilestone(const std::vector<double>& q,const std::vector<double>& dq);
  /// Same as addMilestone, but enforces that the motion should move along
  /// a straight-line joint-space path
  void addMilestoneLinear(const std::vector<double>& q);
  /// Uses linear interpolation to get from the current configuration to the
  /// desired configuration after time dt
  ///
  /// q has size model().numLinks().  dt must be > 0.
  void setLinear(const std::vector<double>& q,double dt);
  /// Uses cubic (Hermite) interpolation to get from the current
  /// configuration/velocity to the desired configuration/velocity after time dt
  ///
  /// q and v have size model().numLinks().  dt must be > 0.
  void setCubic(const std::vector<double>& q,const std::vector<double>& v,double dt);
  /// Same as setLinear but appends an interpolant onto the motion queue
  void addLinear(const std::vector<double>& q,double dt);
  /// Same as setCubic but appends an interpolant onto the motion queue
  void addCubic(const std::vector<double>& q,const std::vector<double>& v,double dt);

  /// Returns the remaining duration of the motion queue
  double remainingTime() const;

  /// Sets a rate controller from the current commanded config to move at
  /// rate dq for time dt > 0.  dq has size model().numLinks()
  void setVelocity(const std::vector<double>& dq,double dt);
  /// Sets a torque command controller.  t can have size model().numDrivers() or
  /// model().numLinks().
  void setTorque(const std::vector<double>& t);
  /// Sets a PID command controller.  Arguments can have size model().numDrivers()
  /// or model().numLinks().
  void setPIDCommand(const std::vector<double>& qdes,const std::vector<double>& dqdes);
  //note: only the last overload docstring is added to the documentation
  /// Sets a PID command controller.  If tfeedforward is provided, it is the feedforward torque vector
  void setPIDCommand(const std::vector<double>& qdes,const std::vector<double>& dqdes,const std::vector<double>& tfeedforward);
  /// Turns on/off manual mode, if either the setTorque or setPID command were
  /// previously set.
  void setManualMode(bool enabled);

  /** @brief Returns the control type for the active controller.
   *
   * Returns:
   * 
   *     One of
   *
   *     - unknown
   *     - off
   *     - torque
   *     - PID
   *     - locked_velocity
   * 
   */
  std::string getControlType();

  /// Sets the PID gains.  Arguments have size model().numDrivers().
  void setPIDGains(const std::vector<double>& kP,const std::vector<double>& kI,const std::vector<double>& kD);
  /// Returns the PID gains for the PID controller
  void getPIDGains(std::vector<double>& kPout,std::vector<double>& kIout,std::vector<double>& kDout);

  int index;
  Simulator* sim;
  Klampt::SimRobotController* controller;
};

/** @brief A reference to a rigid body inside a Simulator (either a
 * RigidObjectModel, TerrainModel, or a link of a RobotModel).
 *
 * Can use this class to directly apply forces to or control positions
 * / velocities of objects in the simulation.  
 * 
 * .. note::
 * 
 *     All changes are applied in the current simulation substep, not the duration
 *     provided to Simulation.simulate().  If you need fine-grained control,
 *     make sure to call Simulation.simulate() with time steps equal to the value
 *     provided to Simulation.setSimStep() (this is 0.001s by default).  Or, use
 *     a hook from :class:`~klampt.sim.simulation.SimpleSimulator`.
 *
 * .. note::
 * 
 *     The transform of the body is centered at the *object's center of mass*
 *     rather than the object's reference frame given in the RobotModelLink or
 *     RigidObjectModel.
 * 
 */
class SimBody
{
 public:
  /// Returns the object ID that this body associated with
  int getID() const;
  /// Sets the simulation of this body on/off
  void enable(bool enabled=true);
  /// Returns true if this body is being simulated
  bool isEnabled();

  /// Turns dynamic simulation of the body on/off.  If false, velocities
  /// will simply be integrated forward, and forces will not affect velocity
  /// i.e., it will be pure kinematic simulation.
  void enableDynamics(bool enabled=true);
  bool isDynamicsEnabled();

  /// Applies a force and torque about the COM over the duration of the
  /// next Simulator.simulate(t) call.
  void applyWrench(const double f[3],const double t[3]);
  /// Applies a force at a given point (in world coordinates) over the
  /// duration of the next Simulator.simulate(t) call.
  void applyForceAtPoint(const double f[3],const double pworld[3]);
  /// Applies a force at a given point (in local center-of-mass-centered
  /// coordinates) over the duration of the next Simulator.simulate(t) call.
  void applyForceAtLocalPoint(const double f[3],const double plocal[3]);

  /// Sets the body's transformation at the current
  /// simulation time step (in center-of-mass centered coordinates).
  void setTransform(const double R[9],const double t[3]);
  /// Gets the body's transformation at the current
  /// simulation time step (in center-of-mass centered coordinates).
  void getTransform(double out[9],double out2[3]);

  /// Sets the body's transformation at the current
  /// simulation time step (in object-native coordinates)
  void setObjectTransform(const double R[9],const double t[3]);
  /// Gets the body's transformation at the current
  /// simulation time step (in object-native coordinates).
  void getObjectTransform(double out[9],double out2[3]);

  /// Sets the angular velocity and translational velocity at the current
  /// simulation time step.
  void setVelocity(const double w[3],const double v[3]);
  /// Returns the angular velocity and translational velocity
  void getVelocity(double out[3],double out2[3]);

  /// Sets the collision padding used for contact generation.  At 0 padding
  /// the simulation will be unstable for triangle mesh and point cloud
  /// geometries. A larger value is useful to maintain simulation stability
  /// for thin or soft objects.  Default is 0.0025.
  void setCollisionPadding(double padding);
  double getCollisionPadding();
  /// If set, preshrinks the geometry so that the padded geometry better matches
  /// the original mesh.  If shrinkVisualization=true, the underlying mesh is
  /// also shrunk (helps debug simulation artifacts due to preshrink)
  void setCollisionPreshrink(bool shrinkVisualization=false);

  /// Gets (a copy of) the surface properties
  ContactParameters getSurface();
  /// Sets the surface properties
  void setSurface(const ContactParameters& params);

  Simulator* sim;
  int objectID;
  Klampt::ODEGeometry* geometry;
  dBodyID body;
};

/** @brief An interface to ODE's hinge and slider joints. You may use this
 * to create custom objects, e.g., drawers, doors, cabinets, etc.  It can 
 * also be used to attach objects together, e.g., an object to a robot's
 * gripper.
 */
class SimJoint
{
public:
  SimJoint();
  ~SimJoint();
  ///Creates a hinge between ``a`` and ``b``, or ``a`` and the world.  The hinge 
  ///is located at point ``pt``, with axis ``axis``, both in world coordinates.
  void makeHinge(const SimBody& a,const SimBody& b,const double pt[3],const double axis[3]);
  void makeHinge(const SimBody& a,const double pt[3],const double axis[3]);
  ///Creates a slider between ``a`` and ``b``, or ``a`` and the world.  The slider
  ///restricts movement to axis ``axis``, given in world coordinates.
  void makeSlider(const SimBody& a,const SimBody& b,const double axis[3]);
  void makeSlider(const SimBody& a,const double axis[3]);
  ///Creates a fixed joint between ``a`` and ``b``.  (There's no method to fix a
  ///to the world; just call a.enableDynamics(False))
  void makeFixed(const SimBody& a,const SimBody& b);
  ///Removes the joint from the simulation.
  void destroy();
  ///Sets the joint limits, relative to the initial configuration of the bodies.
  ///Units are in radians for hinges and meters for sliders.
  void setLimits(double min,double max);
  ///Sets the (dry) friction of the joint
  void setFriction(double friction);
  ///Locks velocity of the joint, up to force fmax. Can't be used with setFriction.
  void setVelocity(double vel,double fmax);
  ///Adds a torque for the hinge joint and a force for a slider joint
  void addForce(double force);

  int type;
  const SimBody *a,*b;
  dJointID joint;
};

/** @brief A dynamics simulator for a WorldModel.
 */
class Simulator
{
 public:
  ///Simulation status flags
  enum { STATUS_NORMAL=0, STATUS_ADAPTIVE_TIME_STEPPING=1, STATUS_CONTACT_UNRELIABLE=2,
    STATUS_UNSTABLE=3, STATUS_ERROR=4 };

  /// Constructs the simulator from a WorldModel.  If the WorldModel was
  /// loaded from an XML file, then the simulation setup is loaded from it.
  Simulator(const WorldModel& model);
  ~Simulator();

  /// Resets to the initial state (same as setState(initialState))
  void reset();

  /// Returns an indicator code for the simulator status. 
  ///
  /// Returns:
  ///
  ///     One of the STATUS_X flags.  (Technically, this returns the *worst* status
  ///     over the last simulate() call)
  ///
  int getStatus();
  /// Returns a string indicating the simulator's status.  If s is provided and >= 0,
  /// this function maps the indicator code s to a string.
  std::string getStatusString(int s=-1);
  /// Checks if any objects are overlapping.
  ///
  /// Returns:
  ///
  ///     A pair of lists of integers, giving the pairs of object ids that
  ///     are overlapping.
  ///
  void checkObjectOverlap(std::vector<int>& out,std::vector<int>& out2);

  /// Gets the current simulation state, including controller parameters, etc.
  ///
  /// Returns:
  ///
  ///     A Base64 string representing the binary data for the state
  ///
  std::string getState();
  /// Sets the current simulation state from a Base64 string returned by
  /// a prior getState call.
  void setState(const std::string& str);

  /// Advances the simulation by time t, and updates the world model from the
  /// simulation state.
  void simulate(double t);
  /// Advances a faked simulation by time t, and updates the world model
  /// from the faked simulation state.
  void fakeSimulate(double t);
  /// Returns the simulation time
  double getTime();

  /// Updates the world model from the current simulation state.  This only
  /// needs to be called if you change the world model and want to revert
  /// back to the simulation state.
  void updateWorld();

  /// Returns the current actual configuration of the robot from the simulator
  void getActualConfig(int robot,std::vector<double>& out);
  /// Returns the current actual velocity of the robot from the simulator
  void getActualVelocity(int robot,std::vector<double>& out);
  /// Returns the current actual torques on the robot's drivers
  /// from the simulator
  void getActualTorque(int robot,std::vector<double>& out);
  /// Deprecated: renamed to getActualTorque to be consistent with
  /// SimRobotController methods
  void getActualTorques(int robot,std::vector<double>& out);

  /// Call this to enable contact feedback between the two objects
  /// (arguments are indexes returned by object.getID()).  Contact feedback
  /// has a small overhead so you may want to do this selectively.
  /// This must be called before using inContact, getContacts, getContactForces,
  /// contactForce, contactTorque, hadContact, hadSeparation, hadPenetration,
  /// and meanContactForce.
  void enableContactFeedback(int obj1,int obj2);
  /// Call this to enable contact feedback between all pairs of objects.
  /// Contact feedback has a small overhead so you may want to do this
  /// selectively.
  void enableContactFeedbackAll();
  /// Returns true if the objects (indexes returned by object.getID()) are in
  /// contact on the current time step.  You can set bid=-1 to tell if object 
  /// ``a`` is in contact with any object. 
  bool inContact(int aid,int bid);
  /// Returns the nx7 list of contacts (x,n,kFriction) at the last time step.
  /// Normals point into object ``a``.  Each contact point (x,n,kFriction) is 
  /// represented as a 7-element vector
  void getContacts(int aid,int bid,double** np_out2,int* m,int* n);
  /// Returns the list of contact forces on object a at the last time step. Result
  /// is an nx3 array.
  void getContactForces(int aid,int bid,double** np_out2,int* m,int* n);
  /// Returns the contact force on object a at the last time step.  You can set
  /// bid to -1 to get the overall contact force on object a.
  void contactForce(int aid,int bid,double out[3]);
  /// Returns the contact force on object ``a`` (about ``a``'s origin) at the last time step.
  /// You can set ``bid`` to -1 to get the overall contact force on object ``a``.
  void contactTorque(int aid,int bid,double out[3]);
  /// Returns true if the objects had contact over the last simulate() call.  You
  /// can set ``bid`` to -1 to determine if object ``a`` had contact with any other object.
  bool hadContact(int aid,int bid);
  /// Returns true if the objects had ever separated during the last
  /// simulate() call. You can set ``bid`` to -1 to determine if object ``a`` 
  /// had no contact with any other object.
  bool hadSeparation(int aid,int bid);
  /// Returns true if the objects interpenetrated during the last simulate()
  /// call.  If so, the simulation may lead to very inaccurate results or
  /// artifacts. 
  ///
  /// You can set ``bid`` to -1 to determine if object ``a`` penetrated
  /// any object, or you can set ```aid=bid=-1``` to determine whether any
  /// object is penetrating any other (indicating that the simulation will
  /// not be functioning properly in general).
  bool hadPenetration(int aid,int bid);
  /// Returns the average contact force on object a over the last simulate()
  /// call
  void meanContactForce(int aid,int bid,double out[3]);

  /// Returns a controller for the indicated robot, either by index or by RobotModel
  SimRobotController controller(int robot);
  //note: only the last overload docstring is added to the documentation
  /// Returns a controller for the indicated robot, either by index or by RobotModel
  SimRobotController controller(const RobotModel& robot);
  ///Returns the SimBody corresponding to the given link
  SimBody body(const RobotModelLink& link);
  ///Returns the SimBody corresponding to the given object
  SimBody body(const RigidObjectModel& object);
  //note: only the last overload docstring is added to the documentation
  ///Return the SimBody corresponding to the given link, rigid object, or terrain
  SimBody body(const TerrainModel& terrain);

  /// Returns the joint force and torque local to the link, as would be read
  /// by a force-torque sensor mounted at the given link's origin. 
  ///
  /// Returns:
  ///
  ///     6 entries of the wrench (fx,fy,fz,mx,my,mz)
  ///     
  void getJointForces(const RobotModelLink& link,double out[6]);

  /// Sets the overall gravity vector
  void setGravity(const double g[3]);
  /// Sets the internal simulation substep.  Values < 0.01 are recommended.
  void setSimStep(double dt);
  /// Returns all setting names
  std::vector<std::string> settings();
  /** @brief Retrieves some simulation setting. 
   * 
   * Valid names are:
   * 
   * - gravity: the gravity vector (default "0 0 -9.8")
   * - simStep: the internal simulation step (default "0.001")
   * - autoDisable: whether to disable bodies that don't move much between time
   *   steps (default "0", set to "1" for many static objects)
   * - boundaryLayerCollisions: whether to use the Klampt inflated boundaries
   *   for contact detection'(default "1", recommended)
   * - rigidObjectCollisions: whether rigid objects should collide (default "1")
   * - robotSelfCollisions: whether robots should self collide (default "0")
   * - robotRobotCollisions: whether robots should collide with other robots
   *   (default "1")
   * - adaptiveTimeStepping: whether adaptive time stepping should be used to
   *   improve stability.  Slower but more stable. (default "1")
   * - minimumAdaptiveTimeStep: the minimum size of an adaptive time step before
   *   giving up (default "1e-6")
   * - maxContacts: max # of clustered contacts between pairs of objects
   *   (default "20")
   * - clusterNormalScale: a parameter for clustering contacts (default "0.1")
   * - errorReductionParameter: see ODE docs on ERP (default "0.95")
   * - dampedLeastSquaresParameter: see ODE docs on CFM (default "1e-6")
   * - instabilityConstantEnergyThreshold: parameter c0 in instability correction
   *   (default "1")
   * - instabilityLinearEnergyThreshold: parameter c1 in instability correction
   *   (default "1.5")
   * - instabilityMaxEnergyThreshold: parameter cmax in instability correction
   *   (default "100000")
   * - instabilityPostCorrectionEnergy: kinetic energy scaling parameter if 
   *   instability is detected (default "0.8")
   *
   * Instability correction kicks in whenever the kinetic energy K(t) of an 
   * object exceeds min(c0*m + c1*K(t-dt),cmax).  m is the object's mass.
   * 
   * See `Klampt/Simulation/ODESimulator.h <http://motion.pratt.duke.edu/klampt/klampt_docs/ODESimulator_8h_source.html>`_
   * for detailed descriptions of these parameters.
   * 
   * Returns:
   *      
   *     A string encoding the data. This will need to be cast to int or
   *     float manually.
   * 
   */
  std::string getSetting(const std::string& name);
  /// Sets some simulation setting. Raises an exception if the name is
  /// unknown or the value is of improper format
  void setSetting(const std::string& name,const std::string& value);

  int index;
  WorldModel world;
  Klampt::Simulator* sim;
  std::string initialState;
};

/// Sets the random seed used by the configuration sampler
void set_random_seed(int seed);

///Cleans up all internal data structures.  Useful for multithreaded programs to make sure ODE errors
///aren't thrown on exit.  This is called for you on exit when importing the Python klampt module.
void destroy();

#endif
