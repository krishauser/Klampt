#ifndef SIMULATION_GUI_H
#define SIMULATION_GUI_H

#include <Klampt/Simulation/Simulator.h>
#include "WorldGUI.h"
#include <set>

namespace Klampt {

/** @brief Generic simulation program.
 *
 * To set up the world and simulation from a command line, call LoadAndInitSim().
 * To set up the simulation from a world, just InitSim().
 * To change the default controller, override the InitController(int i) method.
 *
 * Messages are defined as follows.
 * 
 * command:
 * - simulate(active): turns simulation on or off
 * - toggle_simulate(): toggles simulation activity value
 * - reset(): resets the simulation
 * - load_file(file): loads an entity (object,robot,mesh,path,state) into
 *   the simulation.
 * - load_path(file): loads a path (.path,.xml,.milestones).
 * - load_state(file): loads a simulation state from file
 * - save_state(file): saves a simulation state to file
 * - load_view(file): loads a previously saved view (inherited from GLNavigationProgram)
 * - save_view(file): saves a view to a file (inherited from GLNavigationProgram)
 * - connect_serial_controller(robot,port,rate): connects a robot (index) to a SerialController on the given
 *   port (listens for open TCP connections on localhost:port).  The controller will write at the indicated
 *   rate (in Hz).
 * - output_ros([prefix]): outputs the simulation information to ROS with the given prefix under the tf
 *   module, and the robot's commanded / sensed JointState under [prefix]/[robot name]/commanded_joint_state
 *   / sensed_joint_state.
 *
 * In the current format, elements in the world should not be added/deleted after initialization.
 */
class SimGUIBackend : public WorldGUIBackend
{
public:
  typedef GLNavigationBackend BaseT;
  int simulate;
  Simulator sim;
  string initialState;

  ///the contact state on the last DoContactStateLogging call
  set<pair<int,int> > inContact;

  SimGUIBackend(WorldModel* world)
    :WorldGUIBackend(world),simulate(0)
  {}

  virtual bool OnCommand(const string& cmd,const string& args);

  ///Loads from a world XML file
  bool LoadAndInitSim(const char* xmlFile);

  ///Loads from a command line
  bool LoadAndInitSim(int argc,const char** argv);

  ///Loads some file, figuring out the type from the extension
  bool LoadFile(const char* fn);

  ///Loads some file, figuring out the type from the extension
  bool LoadPath(const char* fn);

  ///Initializes simulation default controllers, sensors, and contact feedback
  virtual void InitSim();
  ///Initializes default controllers and sensors for the indicated robot
  virtual void InitController(int robot);
  ///Initializes all contact feedback
  virtual void InitContactFeedbackAll();

  ///Connects a robot to a SerialController listening for new connections on the given port
  void ConnectSerialController(int robot,int port=3456,Real writeRate=10);

  ///Returns the simulation to its initial state
  void ResetSim();

  ///Renders the state of the simulation
  virtual void RenderWorld();

  ///Sets the colors of robots to indicate force magnitudes
  void SetForceColors();

  ///Sets the colors of robots to indicate torque magnitudes
  void SetTorqueColors();

  ///Renders the simulation clock (when in screen mode)
  void DrawClock(int x,int y);

  ///Draws sensor readings on the world visualization.
  ///If robot < 0, draws all the robots/sensors.
  ///If sensor < 0, draws all the sensors on the given robot.
  void DrawSensor(int robot=-1,int sensor=-1);

  ///Draws contact points
  void DrawContacts(Real pointSize = 5.0, Real fscale = 0.01, Real nscale=0.05);
  
  ///Draws wrenches
  void DrawWrenches(Real fscale=-1);

  ///Loads and sends a milestone path file
  bool LoadMilestones(const char* fn);

  ///Loads and sends a linear path file
  bool LoadLinearPath(const char* fn);

  ///Loads a simulation state file
  bool LoadState(const char* fn);

  ///Loads a multipath file and possibly discretizes it into a fine-grained linear path before sending
  bool LoadMultiPath(const char* fn,bool constrainedInterpolate=true,Real interpolateTolerance=1e-2,Real durationScale=1.0);

  ///Sends a linear path to the controller.  The path starts pathDelay
  ///seconds after the current time
  bool SendLinearPath(const vector<Real>& times,const vector<Config>& milestones,Real pathDelay=0.1); 

  ///Outputs simulation data to ROS with the given prefix
  bool OutputROS(const char* prefix="klampt");

  ///Logs the state of all objects in the world to the given CSV file
  void DoLogging(const char* fn="simtest_log.csv");

  ///Logs the robot's commands to the given linear path file
  void DoCommandLogging_LinearPath(int robot,const char* fn="simtest_command_log.path");

  ///Logs the robot's sensed configuration to the given linear path file
  void DoSensorLogging_LinearPath(int robot,const char* fn="simtest_sensed_log.path");

  ///Logs the robot's simulation state to the given linear path file
  void DoStateLogging_LinearPath(int robot,const char* fn="simtest_state_log.path");

  ///Logs contact changes to the given CSV file
  void DoContactStateLogging(const char* fn="simtest_contact_log.csv");

  ///Logs contact wrenches to the given CSV file
  void DoContactWrenchLogging(const char* fn="simtest_wrench_log.csv");
};

} // namespace Klampt

#endif
