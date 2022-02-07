#ifndef CONTROL_PY_CONTROLLER_H
#define CONTROL_PY_CONTROLLER_H

#include "Controller.h"

#if HAVE_PYTHON
#include "Python.h"
#else
typedef void PyObject;
#endif //HAVE_PYTHON

namespace Klampt {

/** @brief A controller that interfaces with a python module.
 *
 * The module is required to define at least the "update" function.
 * This function takes in a dictionary of sensor data and outputs
 * a dictionary of actuator data.
 *
 * The sensor dictionary includes
 * - 't': current time
 * - 'dt': time step
 * - 'qcmd': current commanded configuration
 * - 'dqcmd': current commanded velocity
 * - for each sensor named 's', 's' is a list of sensor measurements.
 *
 * The following actuator keys are accepted:
 * - 'qcmd': desired configuration
 * - 'dqcmd': desired velocity
 * - 'torquecmd': a torque command or a feedforward torque.
 *
 * Other functions include
 * - "reset" which takes no arguments
 * - "getSettingsFunc" / "setSettingsFunc" which return / take a dictionary
 *   representing the tunable settings for the controller.
 * - "getState" / "setState" which return / take a string representing the
 *   controller's state.  These are only used in simulation mode.
 */
class PyController : public RobotController
{
 public:
  PyController(RobotModel& robot);
  ~PyController();
  bool Load(const string& moduleName);
  void Unload();

  virtual const char* Type() const { return "PyController"; }
  virtual void Update(Real dt);
  virtual void Reset(); 
  virtual bool ReadState(File& f);
  virtual bool WriteState(File& f) const;

  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  virtual vector<string> Commands() const;
  virtual bool SendCommand(const string& name,const string& str);

  string moduleName;
  PyObject *module, *updateFunc, *resetFunc, *getStateFunc, *setStateFunc, *getSettingsFunc, *setSettingsFunc;
  vector<string> commandFuncNames;
  vector<PyObject*> commandFuncs;
};

} //namespace Klampt

#endif
