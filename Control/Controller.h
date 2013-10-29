#ifndef CONTROL_CONTROLLER_BASE_H
#define CONTROL_CONTROLLER_BASE_H

#include "Modeling/Robot.h"
#include "Sensor.h"
#include "Command.h"
#include <myfile.h>
#include <map>

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
  RobotController(Robot& robot);
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

  Robot& robot;
  Real time;

  RobotSensors* sensors;  ///<sensor input (filled in by simulator)
  RobotMotorCommand* command;  ///<motor command output (output to simulator)
};

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
  static void RegisterDefault(Robot& robot);
  static void Register(RobotController* controller);
  static void Register(const char* name,RobotController* controller);
  static SmartPointer<RobotController> CreateByName(const char* name);
  static SmartPointer<RobotController> CreateByName(const char* name,Robot& robot);
  static SmartPointer<RobotController> Load(TiXmlElement* in,Robot& robot);
  static bool Save(RobotController* controller,TiXmlElement* out);

  static std::map<std::string,SmartPointer<RobotController> > controllers;
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
    return ss;              \
  }


#endif
