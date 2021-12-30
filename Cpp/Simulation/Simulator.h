#ifndef WORLD_SIMULATION_H
#define WORLD_SIMULATION_H

#include <Klampt/Modeling/World.h>
#include "ODESimulator.h"
#include "SimRobotController.h"
#include <map>

namespace Klampt {

/** @defgroup Simulation
 * Klampt's rigid body physics simulation engine.
 */

/** @ingroup Simulation
 * @brief Container for information about contacts regarding a certain
 * object.  Can be set to accumulate a summary over sub-steps or detailed
 * data per-step.
 *
 * The reason why this is needed is that each outer Simulator.Advance
 * call may potential make several inner sub steps.  Rather than simply
 * reporting the data at the last point, this class will aggregate the
 * sub-step data.
 */
struct ContactFeedbackInfo
{
  //summary information
  bool accum;      ///< set this to true if you want to accumulate summary feedback over sub-steps
  int contactCount,separationCount; ///< number of sub-steps in which contact was made / object was separated during the outer simulation interval.
  bool inContact; ///< true if contact exists at the end of the outer simulation interval
  Vector3 meanForce,meanTorque,meanPoint;

  bool penetrating;  ///< true if the objects are currently penetrating
  int penetrationCount;   ///< the number of sub-steps in which the objects were penetrating during the outer simulation interval

  //full contact information over sub-steps
  bool accumFull;  //set to true if all ODEContactLists should be stored
  vector<double> times;
  vector<ODEContactList> contactLists;
};

/** @ingroup Simulation
 * @brief Any function that should be run per sub-step of the simulation
 * needs to be a SimulatorHook subclass and added to the
 * Simulator.hooks member.
 *
 * If the flag autokill = true, the hook is removed at the end of the
 * Advance() call.
 *
 * @sa ForceHook
 * @sa LocalForceHook
 * @sa WrenchHook
 * @sa SpringHook
 */
class SimulatorHook
{
 public:
  SimulatorHook() : autokill(false) {}
  virtual ~SimulatorHook() {}
  virtual void Step(Real dt) {}
  virtual bool ReadState(File& f) { return true; }
  virtual bool WriteState(File& f) const { return true; }
  bool autokill;
};

/** @ingroup Simulation
 * @brief A physical simulator for a WorldModel.
 */
class Simulator
{
public:
  Simulator();
  void Init(WorldModel* world);
  ///Updates the simulation with new items added to the world model.
  void OnAddModel();
  ///Sets the robot's controller 
  void SetController(int robot,shared_ptr<RobotController> c);
  ///Advance simulation time by dt (may take multiple sub-steps)
  void Advance(Real dt);
  ///Advance simulation time without actually performing ODE simulation
  void AdvanceFake(Real dt);
  ///Takes the simulation state and puts it in the world model
  void UpdateModel(); 
  ///Takes the simulation state for the robot and puts it in the world model
  void UpdateRobot(int index); 
  ///Load/save state
  ///Note: when reading state, the user must make sure that the controllers
  ///and hooks are *exactly* the same objects as when they were written!
  bool ReadState(File& f);
  bool WriteState(File& f) const;
  bool ReadState(const string& data);
  bool WriteState(string& data) const;

  //contact querying routines
  ///Enables contact feedback between the two objects.  This must be called
  ///before most of the contact querying functions work.
  ///The exception is InContact, which works no matter what.
  void EnableContactFeedback(int aid,int bid,bool accum=true,bool accumFull=false);
  ///Returns true if the objects were in contact on the prior time sub-step.
  ///Warning: if contact feedback is not set up, this does not distinguish
  ///between terrains.  That is, if either a is a terrain and b hits a
  ///different terrain, this will return true.
  bool InContact(int aid,int bid=-1);
  ///Returns true if the objects had contact during past Advance call
  bool HadContact(int aid,int bid=-1);
  ///Returns true if the objects had no contact during past Advance call
  bool HadSeparation(int aid,int bid=-1);
  ///Returns true if the objects had penetration during past Advance call.
  ///Penetration indicates that there may be simulation artifacts due to poor
  ///contact handling.  Can also set both aid=bid=-1 to determine whether the
  ///simulation is generally functioning properly.
  bool HadPenetration(int aid,int bid=-1);
  ///Returns the ContactFeedback structure for the two objects
  ContactFeedbackInfo* GetContactFeedback(int aid,int bid);
  ///Returns the contact list for the prior time step
  ODEContactList* GetContactList(int aid,int bid);
  ///Returns the resultant contact force (on object a) from the prior time step
  Vector3 ContactForce(int aid,int bid=-1);
  ///Returns the resultant contact torque (on object a, about its origin) from the prior time step
  Vector3 ContactTorque(int aid,int bid=-1);
  ///Returns the average contact force (on object a) from the past Advance call
  Vector3 MeanContactForce(int aid,int bid=-1);
  ///Returns the resultant contact torque (on object a, about its origin) from the past Advance call
  Vector3 MeanContactTorque(int aid,int bid=-1);

  //helpers to convert indexing schemes
  int ODEToWorldID(const ODEObjectID& odeid) const;
  ODEObjectID WorldToODEID(int id) const;

  WorldModel* world;
  ODESimulator odesim;
  Real time;
  Real simStep;
  bool fakeSimulation;
  vector<SimRobotController> controlSimulators;
  vector<shared_ptr<RobotController> > robotControllers;
  vector<shared_ptr<SimulatorHook> > hooks;
  typedef map<pair<ODEObjectID,ODEObjectID>,ContactFeedbackInfo> ContactFeedbackMap;
  ContactFeedbackMap contactFeedback;
  ///Worst simulation status over the last Advance() call.
  ODESimulator::Status worstStatus;
};

/** @ingroup Simulation
 * @brief A hook that adds a constant force to a body
 */
class ForceHook : public SimulatorHook
{
 public:
  ForceHook(dBodyID body,const Vector3& worldpt,const Vector3& f);
  virtual void Step(Real dt) override;
  virtual bool ReadState(File& f) override;
  virtual bool WriteState(File& f) const override;

  dBodyID body;
  Vector3 worldpt,f;
};

/** @ingroup Simulation
 * @brief A hook that adds a constant force in world coordinates to a point
 * on a body given in local coordinates
 */
class LocalForceHook : public SimulatorHook
{
 public:
  LocalForceHook(dBodyID body,const Vector3& localpt,const Vector3& f);
  virtual void Step(Real dt) override;
  virtual bool ReadState(File& f) override;
  virtual bool WriteState(File& f) const override;

  dBodyID body;
  Vector3 localpt,f;
};


/** @ingroup Simulation
 * @brief A hook that adds a constant wrench (force f and moment m) to
 * the body
 */
class WrenchHook : public SimulatorHook
{
 public:
  WrenchHook(dBodyID body,const Vector3& f,const Vector3& m);
  virtual void Step(Real dt) override;
  virtual bool ReadState(File& f) override;
  virtual bool WriteState(File& f) const override;

  dBodyID body;
  Vector3 f,m;
};

/** @ingroup Simulation
 * @brief A hook that acts as a Hookean (optionally damped) spring to a given fixed target point.
 */
class SpringHook : public SimulatorHook
{
 public:
  SpringHook(dBodyID body,const Vector3& worldpt,const Vector3& target,Real kP,Real kD=0);
  virtual void Step(Real dt) override;
  virtual bool ReadState(File& f) override;
  virtual bool WriteState(File& f) const override;

  dBodyID body;
  Vector3 localpt,target;
  Real kP,kD;
};

/** @ingroup Simulation
 * @brief A hook that adds a constant force to a joint
 */
class JointForceHook : public SimulatorHook
{
public:
  JointForceHook(ODEJoint* joint,Real f);
  virtual void Step(Real dt) override;
  virtual bool ReadState(File& f) override;
  virtual bool WriteState(File& f) const override;

  ODEJoint* joint;
  Real f;
};

/** @ingroup Simulation
 * @brief A hook that adds a Hookean (optionally damped) virtual spring to a joint
 */
class JointSpringHook : public SimulatorHook
{
public:
  JointSpringHook(ODEJoint* joint,Real target,Real kP,Real kD=0);
  virtual void Step(Real dt) override;
  virtual bool ReadState(File& f) override;
  virtual bool WriteState(File& f) const override;

  ODEJoint* joint;
  Real target;
  Real kP,kD;
};

} //namespace Klampt

#endif
