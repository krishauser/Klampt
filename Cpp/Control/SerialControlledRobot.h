#ifndef SERIAL_CONTROLLED_ROBOT_H
#define SERIAL_CONTROLLED_ROBOT_H

#include "ControlledRobot.h"
#include <KrisLibrary/utils/AsyncIO.h>

namespace Klampt {

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
  SerialControlledRobot(const char* host,double timeout=Inf);
  virtual ~SerialControlledRobot();
  ///call this first before calling Run
  virtual bool Init(RobotModel* robot,RobotController* controller);
  ///Call to process a single message
  bool Process(double timeout);
  ///This call will run the controller forever and never terminate unless
  ///an external thread calls Stop()
  bool Run();
  ///Called by an external thread to stop the Run() loop
  void Stop();
  //for multi-threaded applications -- this mutex locks access to the command / sensors / klamptController structures
  void SetMutex(Mutex* controllerMutex);
  virtual void ReadSensorData(RobotSensors& sensors);
  virtual void WriteCommandData(const RobotMotorCommand& command);
 
  string host;
  shared_ptr<SocketPipeWorker> controllerPipe;
  Real robotTime;
  Real timeStep;
  int numOverruns;
  bool stopFlag;
  Mutex* controllerMutex;
};

} //namespace Klampt

#endif
