#ifndef ODE_INTERFACE_SIMULATOR_H
#define ODE_INTERFACE_SIMULATOR_H

#include "ODERobot.h"
#include "ODERigidObject.h"
#include "ODESurface.h"
#include <Klampt/Modeling/Terrain.h>
#include <Klampt/Modeling/RigidObject.h>
#include <KrisLibrary/robotics/Contact.h>
#include <ode/contact.h>
#include <map>

namespace Klampt {

struct ODEObjectID;
struct ODEContactList;
struct ODEContactResult;
struct ODEJoint;

/** @ingroup Simulation
 * @brief Global simulator settings.
 */
struct ODESimulatorSettings
{
  ODESimulatorSettings();

  ///The gravity vector
  double gravity[3];
  ///Whether to use ODE's auto-disable functionality for non-moving objects
  bool autoDisable;
  ///The default collision padding for environments
  double defaultEnvPadding;
  ///The default surface property for environments
  ODESurfaceProperties defaultEnvSurface;

  //collision checking settings
  ///If true, uses boundary layers for collision detection (recommended true)
  bool boundaryLayerCollisions;
  ///If true, rigid objects collide with one another (default true)
  bool rigidObjectCollisions;
  ///If true, robot self-collisions are detected (default false)
  bool robotSelfCollisions;
  ///If true, robots can collide with other robots (default false)
  bool robotRobotCollisions;
  ///If true, difficult collision situations are resolved using new
  ///adaptive time stepping methods (default true)
  ///
  ///NOTE: adaptiveTimeStepping and autoDisable should not be on at the same
  ///time!  Disabling does weird stuff.
  bool adaptiveTimeStepping;
  ///The minimum time step used by adaptive time stepping (default 1e-6)
  double minimumAdaptiveTimeStep;

  //contact detection settings
  ///Maximum number of contacts between each pair of objects after clustering.
  ///Larger values slow down collision detection (default 20)
  int maxContacts;
  ///During contact clustering, the metric for determining "nearby" contacts
  ///uses this weight to scale distances in normal space.  Distance in position
  ///space have weight 1. (default 0.1)
  double clusterNormalScale;

  //ODE constants, mostly relevant to tightness of robot constraints
  ///ODE's global ERP parameter
  double errorReductionParameter;
  ///ODE's global DLS parameter
  double dampedLeastSquaresParameter;

  //Instability detection / correction parameters
  ///If kinetic energy grows too much from step to step, instability correction
  ///is triggered. Specifically it turns on if
  ///KE[t+1] > instabilityLinearEnergyThreshold*KE[t]+instabilityConstantEnergyThreshold
  ///KE[t+1] > instabilityMaxEnergyThreshold
  ///for any object.  Default values are 1, 1.5, and 100000.
  double instabilityConstantEnergyThreshold,instabilityLinearEnergyThreshold;
  double instabilityMaxEnergyThreshold;
  ///The kinetic energy threshold enforced for any body going unstable. 
  ///negative values -c means scale current kinetic energy by c.  0 means
  ///set velocity to 0.  Positive values are constants * threshold.  (default -0.9)
  double instabilityPostCorrectionEnergy;
};



/** @ingroup Simulation
 * @brief An interface to the ODE simulator.
 * 
 * Step() performs collision detection, sets up contact response,
 * calls StepDynamics(), and computes collision feedback.
 *
 * StepDynamics() integrates the dynamics without setting up collision
 * detection structures.  This probably should not be used externally.
 *
 * Read/WriteState can be used to serialize state to binary.
 *
 * To get contact force information from the simulator, use the
 * EnableContactFeedback() function to initialize feedback, and then call
 * GetContactFeedback() to get a pointer to the feedback data structure.
 * Contact forces are updated after Step().
 */
class ODESimulator
{
 public:
  /**Overall simulation status flag.  If a status is:
   * - Normal: everything is proceeding as normal.
   * - AdaptiveTimeStepping: everything is proceeding as normal, but simulation may be
   *   slower than usual because adaptive time stepping needs to be used.
   * - ContactUnreliable, then some objects have penetrated beyond the
   *   acceptable limits.  Contact response artifacts, like penetration or jittering,
   *   may occur. 
   * - Unstable: then some objects have gone unstable.  Usually this is due to
   *   poorly tuned motor PIDs or external forces/torques.  Klamp't will try to force
   *   things back into a stable state, but the results are not guaranteed.
   * - Error: some unknown error has occurred.  All further simulation will stop.
   */
  enum Status { StatusNormal=0, StatusAdaptiveTimeStepping=1, StatusContactUnreliable=2, StatusUnstable=3, StatusError=4 };

  ODESimulator();
  virtual ~ODESimulator();
  void SetGravity(const Vector3& g);
  void SetAutoDisable(bool autoDisable=true);  //global auto disable -- see ODE docs
  void SetERP(double erp);   //global error reduction  -- see ODE docs
  void SetCFM(double erp);   //global constraint force mixing -- see ODE docs
  ODESimulatorSettings& GetSettings() { return settings; }
  Status GetStatus() const; 
  void GetStatusHistory(vector<Status>& statuses,vector<Real>& statusChangeTimes) const;
  void AddTerrain(TerrainModel& terr);
  void AddRobot(RobotModel& robot);
  void AddObject(RigidObjectModel& object);
  ///Returns true if the current  state is in "reliable" status. Otherwise returns false
  ///and populates the list of overlapping object pairs. 
  bool CheckObjectOverlap(vector<pair<ODEObjectID,ODEObjectID> >& overlaps);
  void Step(Real dt);
  void StepDynamics(Real dt);
  bool ReadState(File& f);
  bool WriteState(File& f) const;

  size_t numTerrains() const { return terrains.size(); }
  size_t numRobots() const { return robots.size(); }
  size_t numObjects() const { return objects.size(); }
  inline dWorldID world() const { return worldID; }
  const TerrainModel* terrain(int i) const { return terrains[i]; }
  ODEGeometry* terrainGeom(int i) const { return terrainGeoms[i]; }
  ODERobot* robot(int i) const { return robots[i]; }
  ODERigidObject* object(int i) const { return objects[i]; }

  string ObjectName(const ODEObjectID& obj) const;
  dBodyID ObjectBody(const ODEObjectID& obj) const;
  dGeomID ObjectGeom(const ODEObjectID& obj) const;
  void EnableContactFeedback(const ODEObjectID& a,const ODEObjectID& b);
  ODEContactList* GetContactFeedback(const ODEObjectID& a,const ODEObjectID& b);
  void GetContactFeedback(const ODEObjectID& a,vector<ODEContactList*>& contacts);
  void ClearContactFeedback();
  bool InContact(const ODEObjectID& a) const;
  bool InContact(const ODEObjectID& a,const ODEObjectID& b) const;
  ///Disables instability correction for the next time step.  This should be done if you manually set several objects' velocities, for example.
  void DisableInstabilityCorrection();
  ///Disables instability correction for the given object on the next time step. This should be done if you manually set an object's velocities, for example.
  void DisableInstabilityCorrection(const ODEObjectID& obj);

  ///Adds a new joint between obj and the world. The returned ODEJoint must be set up using the MakeHinge, MakeSlider, or MakeFixed functions
  ODEJoint* AddJoint(const ODEObjectID& obj);
  ///Adds a new joint between a and b. The returned ODEJoint must be set up using the MakeHinge, MakeSlider, or MakeFixed functions
  ODEJoint* AddJoint(const ODEObjectID& a,const ODEObjectID& b);
  ///Removes a single joint
  void RemoveJoint(ODEJoint*);
  ///Removes all joints touching obj
  void RemoveJoints(const ODEObjectID& obj);
  ///Removes all joints between a and b
  void RemoveJoints(const ODEObjectID& a,const ODEObjectID& b);

  //used internally
  bool ReadState_Internal(File& f);
  bool WriteState_Internal(File& f) const;
  void DetectCollisions();
  void SetupContactResponse(); 
  void SetupContactResponse(const ODEObjectID& a,const ODEObjectID& b,int feedbackIndex,ODEContactResult& c);
  void ClearCollisions();
  bool InstabilityCorrection();
    
  //overload this to have custom parameters for surface pairs
  virtual void GetSurfaceParameters(const ODEObjectID& a,const ODEObjectID& b,dSurfaceParameters& surface) const;

 private:
  vector<pair<Status,Real> > statusHistory;
  ODESimulatorSettings settings;
  dWorldID worldID;
  dSpaceID envSpaceID;
  vector<ODEGeometry*> terrainGeoms;
  vector<const TerrainModel*> terrains;
  vector<ODERobot*> robots;
  vector<ODERigidObject*> objects;
  map<pair<ODEObjectID,ODEObjectID>,ODEContactList> contactList;
  dJointGroupID contactGroupID;
  Real timestep;
  Real simTime;
  map<ODEObjectID,Real> energies;

public:
  //for adaptive time stepping
  File lastState;
  Real lastStateTimestep;
  map<pair<ODEObjectID,ODEObjectID>,double> lastMarginsRemaining;
  //joints
  list<ODEJoint> joints;
};


/** @ingroup Simulation
 * @brief An index that identifies some ODE object in the world.
 * Environments, robots, robot bodies, or rigid objects are supported.
 */
struct ODEObjectID
{
  inline ODEObjectID(int _t=-1,int _i=-1,int _b=-1)
    :type(_t),index(_i),bodyIndex(_b) {}
  inline void SetEnv(int _index=0) { type = 0; index = _index; }
  inline void SetRobot(int _index=0) { type = 1; index = _index; bodyIndex = -1; }
  inline void SetRobotBody(int _index,int _bodyIndex=-1) { type = 1; index = _index; bodyIndex = _bodyIndex; }
  inline void SetRigidObject(int _index=0) { type = 2; index = _index; }
  inline bool IsEnv() const { return type == 0; }
  inline bool IsRobot() const { return type == 1; }
  inline bool IsRigidObject() const { return type == 2; }
  inline bool operator == (const ODEObjectID& rhs) const {
    if(type != rhs.type) return false;
    if(index != rhs.index) return false;
    if(type == 1 && bodyIndex!=rhs.bodyIndex) return false;
    return true;
  }
  inline bool operator < (const ODEObjectID& rhs) const {
    if(type < rhs.type) return true;
    else if(type > rhs.type) return false;
    if(index < rhs.index) return true;
    else if(index > rhs.index) return false;
    return (bodyIndex<rhs.bodyIndex);
  }

  int type;       //0: static environment, 1: robot, 2: movable object
  int index;      //index in world
  int bodyIndex;  //for robots, this identifies a link
};

/** @ingroup Simulation
 * @brief A list of contacts between two objects, returned as feedback 
 * from the simulation.
 *
 * Note: are o1 and o2 actually used?
 */
struct ODEContactList
{
  ODEObjectID o1,o2;
  //the contact points
  vector<ContactPoint> points;
  vector<Vector3> forces;
  //whether the contact detector found excessive penetration
  bool penetrating;   

  vector<int> feedbackIndices;           //internally used
};

/** @ingroup Simulation
 * @brief A joint between two objects.
 *
 * Should be set up by the simulation using AddJoint.  o2 may be left uninitialized.
 */
struct ODEJoint
{
  ODEJoint();
  ~ODEJoint();
  ///Initializes a fixed joint at the objects' current poses
  void MakeFixed();
  ///Initializes a hinge joint at the objects' current poses, with world-space axis passing through pt with direction axis
  void MakeHinge(const Vector3& pt,const Vector3& axis);
  ///Initializes a slider joint at the objects' current poses, with world-space axis dir
  void MakeSlider(const Vector3& dir);
  ///Deallocates this joint. It can be re-allocated using MakeX
  void Destroy();
  
  ///Returns the current position of this joint
  Real GetPosition();
  ///Returns the current velocity of this joint
  Real GetVelocity();

  ///Sets the joint limits
  void SetLimits(Real min,Real max);
  ///Sets the bounce parameter
  void SetBounce(Real coeff);
  ///Sets dry friction on this joint
  void SetFriction(Real coeff);
  ///Sets a fixed velocity motor with target velocity vel max force fmax
  void SetFixedVelocity(Real vel,Real fmax);
  ///Adds a force on this joint to be applied for the next time step
  void AddForce(Real f);
  ///Retrieves the solved constraint forces/torques from the prior time step.  These are all in world coordinates
  void GetConstraintForces(Vector3& f1,Vector3& t1,Vector3& f2,Vector3& t2);

  int type; //<-1 not initialized, 0 fixed, 1 hinge, 2 slider
  ODEObjectID o1,o2;
  ODESimulator* sim;
  dJointID joint;
  dJointFeedback feedback;
};

} //namespace Klampt

#endif
