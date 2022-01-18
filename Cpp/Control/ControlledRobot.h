#ifndef CONTROLLED_ROBOT_H
#define CONTROLLED_ROBOT_H

#include "Controller.h"

namespace Klampt {

/** @brief An interface for a Klamp't controlled robot.  This should be
 * implemented if you wish to use Klamp't controllers to communicate directly
 * with a real robot's motor controller.
 *
 * Init should be called first.  Then, a loop should call the Step method
 * every dt seconds to:
 * 1. Write the physical robot's sensor data into the sensors structure.
 * 2. Call controller->Update(dt)
 * 3. Read in the command structure, and send it to the physical robot.
 *
 * The subclass is responsible for overloading ReadSensorData (step 1)
 * and WriteCommandData (step 3).
 */
class ControlledRobot
{
 public:
  ControlledRobot();
  virtual ~ControlledRobot() {}
  virtual bool Init(RobotModel* robot,RobotController* controller=NULL);
  virtual void Step(Real dt);
  virtual void ReadSensorData(RobotSensors& sensors)=0;
  virtual void WriteCommandData(const RobotMotorCommand& command)=0;

  //convenience functions for inspection
  void GetCommandedConfig(Config& q);
  void GetCommandedVelocity(Config& dq);
  void GetSensedConfig(Config& q);
  void GetSensedVelocity(Config& dq);

  //settings
  RobotModel* klamptRobotModel;
  RobotController* klamptController;

  //state
  RobotMotorCommand command;
  RobotSensors sensors;
};

} //namespace Klampt

#endif
