#ifndef ODE_CONTROLLED_SIMULATOR_H
#define ODE_CONTROLLED_SIMULATOR_H

#include <Klampt/Control/Controller.h>
#include "ODERobot.h"

namespace Klampt {

  class Simulator;

/** @ingroup Simulation
 * @brief A class containing information about an ODE-simulated and
 * controlled robot.
 *
 * Performs the controller update and sensor simulation for each robot in
 * the Simulator class.  Most of these functions won't need to be
 * modified or used except by GUIs to examine the simulator/control state.
 */
class SimRobotController
{
 public:
  SimRobotController();
  void Init(RobotModel* robot,ODERobot* oderobot,RobotController* controller=NULL);
  void Step(Real dt,Simulator* sim);
  void UpdateRobot();

  void GetCommandedConfig(Config& q);
  void GetCommandedVelocity(Config& dq);
  void GetSensedConfig(Config& q);
  void GetSensedVelocity(Config& dq);
  void GetSimulatedConfig(Config& q);
  void GetSimulatedVelocity(Config& dq);
  ///Returns a list of torques commanded at the actuator (driver) level
  void GetActuatorTorques(Vector& t) const;
  ///Returns a list of torques commanded at the link level
  void GetLinkTorques(Vector& t) const;
  RobotMotorCommand* GetCommands() { return &command; }

  bool ReadState(File& f);
  bool WriteState(File& f) const;

  //settings
  RobotModel* robot;
  ODERobot* oderobot;
  RobotController* controller;
  Real controlTimeStep;

  //state
  Real curTime;
  Real nextControlTime;
  RobotMotorCommand command;
  RobotSensors sensors;
  vector<Real> nextSenseTime;
};

} // namespace Klampt

#endif
