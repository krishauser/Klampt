#ifndef CONTROL_SENSORS_H
#define CONTROL_SENSORS_H

#include <KrisLibrary/math/vector.h>
#include <map>
#include <vector>
#include <memory>
#include <string>
#include <typeinfo>

class TiXmlElement;

namespace Klampt {
  using namespace std;

/** @defgroup Sensing
 * Sensor configuration and simulation.
 */

class RobotModel;
class WorldModel;
class SimRobotController;
class Simulator;

/** @ingroup Sensing
 * @brief A sensor base class.  A SensorBase should allow a Controller to 
 * both connect to a simulation as well as a real sensor. 
 *
 * The mapping from simulation -> SensorBase is given by the Simulate
 * and Advance calls which are called immediately in succession.
 *
 * The mapping from a sensor to this structure can be given by the
 * SetMeasurements function.
 *
 * Default settings:
 * - rate: the number of time per second this should be called, in Hz.  If 0,
 *   the sensor is updated every time the controller is called (default)
 * - enabled: whether the sensor provides data. True by default.
 *
 * FOR IMPLEMENTERS: at a minimum, you must overload the Type(),
 * MeasurementNames and Get/SetMeasurements methods.  (Note: it is important
 * that GetMeasurements is idempotent and does not change internal state.)
 *
 * If the sensor simulator manages any internal state, such as a state
 * estimate, then you will also need to override
 * Get/SetInternalState in order for the simulator state to be properly saved and
 * loaded during rewind/replay.
 * If you want the simulator to be resettable, you will need to overload
 * Reset() to reset the state when necessary.
 *
 * If your sensor is reconfigurable, you will want to also override the
 * Settings and Get/SetSetting methods.  The macros FILL_SENSOR_SETTING,
 * GET_SENSOR_SETTING, and SET_SENSOR_SETTING are helpful for doing this.
 */
class SensorBase
{
 public:
  SensorBase();
  virtual ~SensorBase() {}
  virtual const char* Type() const { return "SensorBase"; }
  ///Called whenever the sensor is updated from the simulaton
  virtual void Simulate(SimRobotController* robot,Simulator* sim) {}
  ///Updates the sensor for a kinematic world.  Useful for non-simulation debugging.
  virtual void SimulateKinematic(RobotModel& robot,WorldModel& world) {}
  ///Advances to the next time step with duration dt elapsed
  virtual void Advance(double dt) {}
  ///Should be overridden if the sensor is stateful to reset to an initial state
  virtual void Reset() {}
  virtual bool ReadState(File& f);
  virtual bool WriteState(File& f) const;
  ///Must be overridden to produce a list of names of each measurement
  virtual void MeasurementNames(vector<string>& names) const { names.resize(0); }
  ///Must be overridden to returns a list of all measurements
  virtual void GetMeasurements(vector<double>& values) const { values.resize(0); }
  ///Updates the internal measurement vector.  Should be overridden to
  ///correctly restore state using ReadState(), or to visualize a physical
  ///robot's sensors.
  virtual void SetMeasurements(const vector<double>& values) { }
  ///Any other state besides measurements/settings that you might want to store.  Used in ReadState
  virtual void GetInternalState(vector<double>& state) const {  }
  ///Any other state besides measurements/settings that you might want to store.  Used in WriteState
  virtual void SetInternalState(const vector<double>& state) { }
  ///Returns a map of all current name-value pairs of the sensor's settings
  virtual map<string,string> Settings() const;
  ///Get a named setting.   Returns false if the name is not supported
  virtual bool GetSetting(const string& name,string& str) const;
  ///Set a named setting.  Returns false if the name is not supported, or the
  ///value is formatted incorrectly
  virtual bool SetSetting(const string& name,const string& str);
  ///If the sensor can be drawn, draw the sensor on the robot's current configuration,
  ///using these measurements, using OpenGL calls.
  virtual void DrawGL(const RobotModel& robot,const vector<double>& measurements) {}

  string name;
  double rate;
  bool enabled;
};




/** @ingroup Sensing
 * @brief A set of sensors for the robot.
 *
 * Accepts saving/loading to XML format.
 *
 * MakeDefault first looks in the robot->properties["sensors"]
 * element to load an XML file.  If this fails, then it will
 * add joint position and joint velocity sensors to the robot.
 */
class RobotSensors
{
 public:
  void MakeDefault(RobotModel* robot);
  bool LoadSettings(const char* fn);
  bool SaveSettings(const char* fn);
  bool LoadSettings(TiXmlElement* in);
  void SaveSettings(TiXmlElement* out);
  bool LoadMeasurements(TiXmlElement* in);
  void SaveMeasurements(TiXmlElement* out);
  bool ReadState(File& f);
  bool WriteState(File& f) const;
  shared_ptr<SensorBase> GetNamedSensor(const string& name);
  template <class T>
  void GetTypedSensors(vector<T*>& sensors);
  template <class T>
  T* GetTypedSensor(int index=0);
  shared_ptr<SensorBase> CreateByType(const char* type) const;

  vector<shared_ptr<SensorBase> > sensors;
};



template <class T>
void RobotSensors::GetTypedSensors(vector<T*>& _sensors)
{
  _sensors.resize(0);
  for(size_t i=0;i<sensors.size();i++)
    if(typeid(T) == typeid(*sensors[i])) _sensors.push_back(dynamic_cast<T*>(sensors[i].get()));
}


template <class T>
T* RobotSensors::GetTypedSensor(int index)
{
  for(size_t i=0;i<sensors.size();i++) {
    if(typeid(T) == typeid(*sensors[i])) {
      if(index==0) return dynamic_cast<T*>(sensors[i].get());
      index--;
    }
  }
  return NULL;
}



//these macros will help you read in / write out settings
#define FILL_SENSOR_SETTING(res,membername) \
  { \
    stringstream ss;  \
    ss<<membername;   \
    res[#membername] = ss.str();  \
  }
#define GET_SENSOR_SETTING(membername) \
  if(name == #membername) { \
    stringstream ss;   \
    ss << membername;  \
    str = ss.str();    \
    return true;       \
  }
#define SET_SENSOR_SETTING(membername) \
  if(name == #membername) { \
    stringstream ss(str);   \
    ss >> membername;       \
    return bool(ss);              \
  }

#define FILL_ARRAY_SENSOR_SETTING(res,membername,count)	\
  { \
    stringstream ss;                      \
    for(int _i=0;_i<count;_i++)             \
      ss<<membername[_i]<<" ";             \
    settings[#membername] = ss.str();     \
  }
#define GET_ARRAY_SENSOR_SETTING(membername,count)	\
  if(name == #membername) { \
    stringstream ss;   \
    for(int _i=0;_i<count;_i++)             \
      ss << membername[_i]<<" ";		  \
    str = ss.str();    \
    return true;       \
  }
#define SET_ARRAY_SENSOR_SETTING(membername,count)	\
  if(name == #membername) {	\
    stringstream ss(str);	  \
    for(int _i=0;_i<count;_i++)     \
      ss >> membername[_i];	  \
    return bool(ss);              \
  }

#define FILL_VECTOR_SENSOR_SETTING(res,membername) \
  { \
    stringstream ss;                      \
    for(size_t _i=0;_i<membername.size();_i++)             \
      ss<<membername[_i]<<" ";             \
    settings[#membername] = ss.str();     \
  }
#define GET_VECTOR_SENSOR_SETTING(membername)  \
  if(name == #membername) { \
    stringstream ss;   \
    for(size_t _i=0;_i<membername.size();_i++)            \
      ss << membername[_i]<<" ";      \
    str = ss.str();    \
    return true;       \
  }
#define SET_VECTOR_SENSOR_SETTING(membername)  \
  if(name == #membername) { \
    stringstream ss(str);   \
    membername.resize(0); \
    while(ss) {     \
      membername.resize(membername.size()+1); \
      ss >> membername.back();   \
      if(ss.fail()) { \
        membername.resize(membername.size()-1); \
        return true; \
      } \
      if(ss.bad()) return false; \
    } \
    return true;              \
  }

} //namespace Klampt

#endif
