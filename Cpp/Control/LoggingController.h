#ifndef LOGGING_CONTROLLER_H
#define LOGGING_CONTROLLER_H

#include "Controller.h"

namespace Klampt {

/** @brief A controllre that saves/replays low-level commands from disk.
 *
 * Use the boolean flags 'save' and 'replay' to control the logging. 
 * 'Replay' overrides the base controller's normal update function.
 * Be careful not to interleave the replay mode with commands to the base
 * controller!
 *
 * If 'onlyJointCommands' is true, only the joint commands qdes, dqdes,
 * torque, and desiredVelocity are replayed.
 * The standard servo parameters are left untouched.
 */
class LoggingController : public RobotController
{
 public:
  LoggingController(RobotModel& robot,const shared_ptr<RobotController>& base);
  virtual const char* Type() const { return "LoggingController"; }
  virtual void Update(Real dt);
  virtual void Reset();
  virtual bool ReadState(File& f);
  virtual bool WriteState(File& f) const;
  bool SaveLog(const char* fn) const;
  bool LoadLog(const char* fn);

  //getters/setters
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  virtual vector<string> Commands() const;
  virtual bool SendCommand(const string& name,const string& str);

  //helpers that compress the logs
  bool EqualCommand(const ActuatorCommand& a,const ActuatorCommand& b) const;
  bool EqualCommand(const RobotMotorCommand& a,const RobotMotorCommand& b) const;
  void RemoveDelays(Real maxDelayTime);

  shared_ptr<RobotController> base;
  bool save,replay;
  bool onlyJointCommands; 
  vector<pair<Real,RobotMotorCommand> > trajectory;
  int replayIndex;
};

} // namespace Klampt

#endif
