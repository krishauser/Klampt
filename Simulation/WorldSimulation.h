#ifndef WORLD_SIMULATION_H
#define WORLD_SIMULATION_H

#include "Modeling/World.h"
#include "ODESimulator.h"
#include "ControlledSimulator.h"
#include <map>

/** @brief Container for information about contacts regarding a certain
 * object.  Can be set to accumulate a summary over sub-steps or detailed
 * data per-step.
 *
 * The reason why this is needed is that each outer WorldSimulation.Advance
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

  //full contact information over sub-steps
  bool accumFull;  //set to true if all ODEContactLists should be stored
  vector<double> times;
  vector<ODEContactList> contactLists;
};

/** @brief Any function that should be run per sub-step of the simulation
 * needs to be a WorldSimulationHook subclass and added to the
 * WorldSimulation.hooks member.
 *
 * @sa ForceHook
 * @sa SpringHook
 */
class WorldSimulationHook
{
 public:
  virtual ~WorldSimulationHook() {}
  virtual void Step(Real dt) {}
  virtual bool ReadState(File& f) { return true; }
  virtual bool WriteState(File& f) const { return true; }
};

/** @brief A physical simulator for a RobotWorld.
 */
class WorldSimulation
{
public:
  WorldSimulation();
  void Init(RobotWorld* world);
  ///Updates the simulation with new items added to the world model.
  void OnAddModel();
  ///Sets the robot's controller 
  void SetController(int robot,SmartPointer<RobotController> c);
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
  ///Returns the ContactFeedback structure for the two objects
  ContactFeedbackInfo* GetContactFeedback(int aid,int bid);
  ///Returns the contact list for the prior time step
  ODEContactList* GetContactList(int aid,int bid);
  ///Returns the resultant contact force (on object a) from the prior time step
  Vector3 ContactForce(int aid,int bid=-1);
  ///Returns the average contact force (on object a) from the past Advance call
  Vector3 MeanContactForce(int aid,int bid=-1);

  //helpers to convert indexing schemes
  int ODEToWorldID(const ODEObjectID& odeid) const;
  ODEObjectID WorldToODEID(int id) const;

  RobotWorld* world;
  ODESimulator odesim;
  Real time;
  Real simStep;
  bool fakeSimulation;
  vector<ControlledRobotSimulator> controlSimulators;
  vector<SmartPointer<RobotController> > robotControllers;
  vector<SmartPointer<WorldSimulationHook> > hooks;
  typedef map<pair<ODEObjectID,ODEObjectID>,ContactFeedbackInfo> ContactFeedbackMap;
  ContactFeedbackMap contactFeedback;
};

/** @brief A hook that adds a constant force to a body
 */
class ForceHook : public WorldSimulationHook
{
 public:
  ForceHook(dBodyID body,const Vector3& worldpt,const Vector3& f);
  virtual void Step(Real dt);
  virtual bool ReadState(File& f);
  virtual bool WriteState(File& f) const;

  dBodyID body;
  Vector3 worldpt,f;
};

/** @brief A hook that acts as a Hookean spring to a given fixed target point.
 */
class SpringHook : public WorldSimulationHook
{
 public:
  SpringHook(dBodyID body,const Vector3& worldpt,const Vector3& target,Real k);
  virtual void Step(Real dt);
  virtual bool ReadState(File& f);
  virtual bool WriteState(File& f) const;

  dBodyID body;
  Vector3 localpt,target;
  Real k;
};

#endif
