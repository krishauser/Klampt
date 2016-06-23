#ifndef DYNAMICS_SENSORS_H
#define DYNAMICS_SENSORS_H

#include <KrisLibrary/math/vector.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/utils/SmartPointer.h>
#include <map>
#include <vector>
#include <deque>
#include <string>
#include <typeinfo>
using namespace Math3D;
using namespace std;

class Robot;
class ControlledRobotSimulator;
class WorldSimulation;
class TiXmlElement;

/** @ingroup Control
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
 *
 * FOR IMPLEMENTERS: at a minimum, you must overload the Type(),
 * MeasurementNames and Get/SetMeasurements methods. 
 *
 * If the sensor simulator manages any internal state, such as a state
 * estimate, then you will also need to override
 * Get/SetState in order for the simulator state to be properly saved and
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
  virtual void Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim) {}
  virtual void Advance(Real dt) {}
  virtual void Reset() {}
  virtual bool ReadState(File& f);
  virtual bool WriteState(File& f) const;
  virtual void MeasurementNames(vector<string>& names) const { names.resize(0); }
  virtual void GetMeasurements(vector<double>& values) const { values.resize(0); }
  virtual void SetMeasurements(const vector<double>& values) const { }
  //Any other state that you might want to store.  Used in ReadState
  virtual void GetState(vector<double>& state) const {  }
  //Any other state that you might want to store.  Used in WriteState
  virtual void SetState(const vector<double>& state) { }
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);
  virtual void DrawGL() {}

  string name;
  double rate;
};




/** @ingroup Control
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
  void MakeDefault(Robot* robot);
  bool LoadSettings(const char* fn);
  bool SaveSettings(const char* fn);
  bool LoadSettings(TiXmlElement* in);
  void SaveSettings(TiXmlElement* out);
  bool LoadMeasurements(TiXmlElement* in);
  void SaveMeasurements(TiXmlElement* out);
  bool ReadState(File& f);
  bool WriteState(File& f) const;
  SmartPointer<SensorBase> GetNamedSensor(const string& name);
  template <class T>
  void GetTypedSensors(vector<T*>& sensors);
  template <class T>
  T* GetTypedSensor(int index=0);

  vector<SmartPointer<SensorBase> > sensors;
};



template <class T>
void RobotSensors::GetTypedSensors(vector<T*>& _sensors)
{
  _sensors.resize(0);
  for(size_t i=0;i<sensors.size();i++)
    if(typeid(T) == typeid(*sensors[i])) _sensors.push_back(dynamic_cast<T*>((SensorBase*)sensors[i]));
}


template <class T>
T* RobotSensors::GetTypedSensor(int index)
{
  for(size_t i=0;i<sensors.size();i++) {
    if(typeid(T) == typeid(*sensors[i])) {
      if(index==0) return dynamic_cast<T*>((SensorBase*)sensors[i]);
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



#endif
