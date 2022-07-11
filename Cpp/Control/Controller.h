#ifndef CONTROL_CONTROLLER_BASE_H
#define CONTROL_CONTROLLER_BASE_H

#include <Klampt/Modeling/Robot.h>
#include <Klampt/Sensing/Sensor.h>
#include "Command.h"
#include <KrisLibrary/File.h>
#include <map>

namespace Klampt {

/** @ingroup Control
 * @brief A base class for a robot controller.  The base class does nothing.
 *
 * The sensors and command pointer will be set up by the simulator/real robot.
 * In the Update function, the controller should read the sensor information
 * in sensors and write the desired joint commands to the command pointer.
 *
 * Reset is called when the user hits reset on the simulation.
 *
 * To support reading and writing controller states, fill in the ReadState
 * and WriteState functions.
 *
 * To support user-supplied configuration parameters, fill in the *Settings*()
 * methods.  These are currently used in SimTest but in the future we hope to
 * support more general methods for getting/setting controller settings, e.g.
 * in an XML file.
 */
class RobotController
{
public:
  RobotController(RobotModel& robot);
  virtual ~RobotController() {}
  //subclasses fill these out
  virtual const char* Type() const { return "RobotController"; }
  virtual void Update(Real dt) { time+=dt; }
  virtual void Reset() { time=0; } 
  virtual bool ReadState(File& f);
  virtual bool WriteState(File& f) const;
  //settings: can be set by external configuration files / functions
  virtual map<string,string> Settings() const { return map<string,string>(); }
  virtual bool GetSetting(const string& name,string& str) const { return false; }
  virtual bool SetSetting(const string& name,const string& str) { return false; }

  virtual vector<string> Commands() const { return vector<string>(); }
  virtual bool SendCommand(const string& name,const string& str) { return false; }

  //convenience functions
  void SetPIDCommand(const Config& qdes);
  void SetPIDCommand(const Config& qdes,const Config& dqdes);
  void SetFeedforwardPIDCommand(const Config& qdes,const Config& dqdes,const Vector&torques);
  void SetTorqueCommand(const Vector& torques);

  bool GetCommandedConfig(Config& q);
  bool GetCommandedVelocity(Config& dq);
  bool GetSensedConfig(Config& q);
  bool GetSensedVelocity(Config& dq);

  RobotModel& robot;
  Real time;
  Real nominalTimeStep;   ///< a "desired" time step, by default 0, which acts as a hint to the simulator.  Note that it doesn't have to abide the hint.

  RobotSensors* sensors;  ///<sensor input (filled in by simulator)
  RobotMotorCommand* command;  ///<motor command output (output to simulator)
};

///Makes a default controller used in all the Klamp't simulation apps.
///First, reads from the file given by robot->properties["controller"].
///If this fails, makes a Logging, Feedforward, PolynomialPath controller.
shared_ptr<RobotController> MakeDefaultController(RobotModel* robot);

/** @ingroup Control
 * @brief A class to simplify the loading of different controllers at run time.
 *
 * First a list of controllers is provided to Register(), or RegisterDefault()
 * can be called.  Then CreateByName() is called to retreive the named controller.
 * 
 * Note that Register takes ownership of the pointers.
 */
class RobotControllerFactory
{
 public:
  static void RegisterDefault(RobotModel& robot);
  static void Register(RobotController* controller);
  static void Register(const char* name,RobotController* controller);
  static shared_ptr<RobotController> CreateByName(const char* name);
  static shared_ptr<RobotController> CreateByName(const char* name,RobotModel& robot);
  static shared_ptr<RobotController> Load(const char* fn,RobotModel& robot);
  static bool Save(RobotController* controller,const char* fn);
  static shared_ptr<RobotController> Load(TiXmlElement* in,RobotModel& robot);
  static bool Save(RobotController* controller,TiXmlElement* out);

  static std::map<std::string,shared_ptr<RobotController> > controllers;
};

//these macros will help you read in / write out settings
#define FILL_CONTROLLER_SETTING(res,membername) \
  { \
    stringstream ss;  \
    ss<<membername;   \
    res[#membername] = ss.str();  \
  }
#define READ_CONTROLLER_SETTING(membername) \
  if(name == #membername) { \
    stringstream ss;   \
    ss << membername;  \
    str = ss.str();    \
    return true;       \
  }
#define WRITE_CONTROLLER_SETTING(membername) \
  if(name == #membername) { \
    stringstream ss(str);   \
    ss >> membername;       \
    return bool(ss);              \
  }

} //namespace Klampt

#endif
