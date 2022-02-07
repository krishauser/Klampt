#ifndef SERIAL_CONTROLLER_H
#define SERIAL_CONTROLLER_H

#include "Controller.h"
#include <KrisLibrary/utils/AsyncIO.h>

class AnyCollection;

namespace Klampt {

/** @ingroup Control
 * @brief A controller that writes sensor data to a socket and reads
 * robot commands from a socket.
 *
 * Sensor data is given by string messages, formatted in JSON, prepended
 * by 4 bytes of the string length.  Messages are of the form:
 * "{sensor1:[values1[0],...,values1[n1]],...,sensork:[valuesk[0],...valuesk[0]]}"
 *
 * Command data is formatted the same way in JSON, prepended by 4 bytes
 * of the string length.  The commands are of the form:
 *    {command1:value1,...commandk:valuek}
 * where command can be
 * - setting: change setting, value is a pair (setting_name,setting_value)
 * - qcmd: position command
 * - dqcmd: velocity command
 * - tcmd: duration, used in fixed-velocity command
 * - torquecmd: torque command
 *
 * Command data is read opportunistically.  Sensor data is written at a given
 * writeRate (in Hz)
 *
 * Settings include
 * - servAddr: socket address.  Set to "" for no connection.
 * - connected: 1 if connected (can only be gotten), 0 if disconnected
 * - writeRate: rate at which sensor data is written.
 */
class SerialController : public RobotController
{
public:
  SerialController(RobotModel& robot,const string& servAddr="",Real writeRate=10);
  virtual ~SerialController() {}
  virtual const char* Type() const { return "SerialController"; }
  virtual void Update(Real dt);
  virtual void Reset();
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  bool OpenConnection(const string& servaddr);
  bool CloseConnection();
  void PackSensorData(AnyCollection& data);

  string servAddr;
  Real writeRate;
  Real lastWriteTime;
  shared_ptr<SocketPipeWorker> controllerPipe;

  //for fixed-velocity commands, these are an accumulator that processes
  //the linearly increasing configuration
  Config vcmd;
  Real endVCmdTime;
};

} // namespace Klampt

#endif
