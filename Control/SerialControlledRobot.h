#ifndef SERIAL_CONTROLLED_ROBOT_H
#define SERIAL_CONTROLLED_ROBOT_H

#include "ControlledRobot.h"
#include <myfile.h>

/** @brief A Klamp't controlled robot that communicates to a robot (either
 * real or virtual) using the Klamp't controller serialization mechanism.
 * Acts as a client connecting to the given host.
 *
 * You usually use this if you want to set up a Klamp't C++ controller
 * running as a standalone program to communicate with SimTest.
 */
class SerialControlledRobot : public ControlledRobot
{
 public:
  SerialControlledRobot(const char* host);
  virtual ~SerialControlledRobot() {}
  ///This call will run the controller forever and never terminate
  void Run();
  virtual void ReadSensorData(RobotSensors& sensors);
  virtual void WriteCommandData(const RobotMotorCommand& command);
  
  File socketfile;
  Real timeStep;
};

#endif
