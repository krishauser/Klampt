#ifndef DYNAMICS_SENSORS_H
#define DYNAMICS_SENSORS_H

#include <math/vector.h>
#include <math3d/primitives.h>
#include <utils/SmartPointer.h>
#include <map>
#include <vector>
#include <string>
#include <typeinfo>
using namespace Math3D;
using namespace std;

class ControlledRobotSimulator;
class TiXmlElement;

/** @ingroup Control
 * @brief A sensor base class.
 *
 * To be XML read/writeable, override the Settings, Get/SetSetting methods.
 * To be loggable/replayable, override the MeasurementNames and
 * Get/SetMeasurements methods.
 */
class SensorBase
{
 public:
  SensorBase() {}
  virtual ~SensorBase() {}
  virtual const char* Type() const { return "SensorBase"; }
  virtual void Simulate(ControlledRobotSimulator* robot) {}
  virtual void Advance(Real dt) {}
  virtual void Reset() {}
  virtual bool ReadState(File& f);
  virtual bool WriteState(File& f) const;
  virtual void MeasurementNames(vector<string>& names) const { names.resize(0); }
  virtual void GetMeasurements(vector<double>& values) const { values.resize(0); }
  virtual void SetMeasurements(const vector<double>& values) { }
  virtual map<string,string> Settings() const { return map<string,string>(); }
  virtual bool GetSetting(const string& name,string& str) const { return false; }
  virtual bool SetSetting(const string& name,const string& str) { return false; }

  string name;
};

/** @ingroup Control
 * @brief Simulates a joint encoder.
 */
class JointPositionSensor : public SensorBase
{
 public:
  JointPositionSensor();
  virtual const char* Type() const { return "JointPositionSensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot);
  virtual void Reset();
  virtual void MeasurementNames(vector<string>& names) const;
  virtual void GetMeasurements(vector<double>& values) const;
  virtual void SetMeasurements(const vector<double>& values);
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  vector<int> indices;   ///< The indices on which the position sensors are located
  Vector qvariance;      ///< Estimated variance of the encoder values
  Vector qresolution;    ///< Estimate on the encoder resolution
  Vector q;              ///< Measurement: joint angles
};

/** @ingroup Control
 * @brief Simulates a "true" velocity sensor
 */
class JointVelocitySensor : public SensorBase
{
 public:
  JointVelocitySensor();
  virtual const char* Type() const { return "JointVelocitySensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot);
  virtual void Reset();
  virtual void MeasurementNames(vector<string>& names) const;
  virtual void GetMeasurements(vector<double>& values) const;
  virtual void SetMeasurements(const vector<double>& values);
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  vector<int> indices;   ///< The indices on which the velocity sensors are located
  Vector dqvariance;     ///< Estimated variance of the encoder values
  Vector dqresolution;   ///< Estimate on the encoder resolution
  Vector dq;             ///< Measurement: joint velocities
};

/** @ingroup Control
 * @brief Simulates a torque sensor.
 * 
 * Motors typically provide current/voltage information, and this assumes
 * that it is transformed into torque units.
 */
class DriverTorqueSensor : public SensorBase
{
 public:
  DriverTorqueSensor();
  virtual const char* Type() const { return "DriverTorqueSensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot);
  virtual void Reset();
  virtual void MeasurementNames(vector<string>& names) const;
  virtual void GetMeasurements(vector<double>& values) const;
  virtual void SetMeasurements(const vector<double>& values);
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  vector<int> indices;   ///< The indices on which the torque sensors are located
  Vector tvariance;     ///< Estimated variance of the torque values
  Vector tresolution;   ///< Estimate on the torque resolution
  Vector t;             ///< Measurement: joint torques
};

/** @ingroup Control
 * @brief Simulates a pressure sensor.  Not currently functional.
 */
class ContactSensor : public SensorBase
{
 public:
  ContactSensor();
  virtual const char* Type() const { return "ContactSensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot);
  virtual void Reset();
  virtual void MeasurementNames(vector<string>& names) const;
  virtual void GetMeasurements(vector<double>& values) const;
  virtual void SetMeasurements(const vector<double>& values);
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  int link;                ///< The link on which the sensor is located
  RigidTransform Tsensor;  ///< Local frame of the sensor (by convention, origin is at contact patch, z is normal to surface, into robot)
  Vector2 patchMin,patchMax;///< The 2D contact patch in the local frame of the sensor 
  Real patchTolerance;     ///< The deformation tolerance of the contact patch
  bool hasForce[3];        ///< If an element is true, that component of force is measured
  Vector3 fVariance;       ///< Estimated variance of the sensor

  bool contact;            ///< Measurement: true if contact has been made
  Vector3 force;           ///< Measurement: the force magnitude
};

/** @ingroup Control
 * @brief Simulates a force-torque sensor mounted between a link and its
 * parent.
 * Can be configured to be up to 6DOF.
 */
class ForceTorqueSensor : public SensorBase
{
 public:
  ForceTorqueSensor();
  virtual const char* Type() const { return "ForceTorqueSensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot);
  virtual void Reset();
  virtual void MeasurementNames(vector<string>& names) const;
  virtual void GetMeasurements(vector<double>& values) const;
  virtual void SetMeasurements(const vector<double>& values);
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  int link;                ///< The link on which the sensor is located (between link and parent)
  Vector3 localPos;        ///< The position of the sensor, in the local frame
  bool hasForce[3];        ///< true if force is measured along the given axis
  bool hasMoment[3];       ///< true if moment is measured along the given axis
  Vector3 fVariance, mVariance; ///< Estimated variance of the sensor

  Vector3 f,m;             ///< Measurement: the force/moment at the given position, on the link (negative on the parent link)
};

/** @ingroup Control
 * @brief Simulates an accelerometer.
 */
class Accelerometer : public SensorBase
{
 public:
  Accelerometer();
  virtual const char* Type() const { return "Accelerometer"; }
  virtual void Simulate(ControlledRobotSimulator* robot);
  virtual void Advance(Real dt);
  virtual void Reset();
  virtual bool ReadState(File& f);
  virtual bool WriteState(File& f) const;
  virtual void MeasurementNames(vector<string>& names) const;
  virtual void GetMeasurements(vector<double>& values) const;
  virtual void SetMeasurements(const vector<double>& values);
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  int link;
  RigidTransform Tsensor;  ///< Position of unit on link
  bool hasAxis[3];         ///< true if accel is measured along the given axis
  Vector3 accelVariance;   ///< Estimated variances of the sensor

  Vector3 accel;           ///< Measurement: acceleration value

  Real last_dt;            ///< Temporary: needed to derive accel from ODE
  Vector3 last_v;          ///< Temporary: needed to derive accel from ODE
};

/** @ingroup Control
 * @brief Simulates a tilt sensor that measures the angle of
 * a reference direction about certain axes.
 */
class TiltSensor : public SensorBase
{
 public:
  TiltSensor();
  virtual const char* Type() const { return "TiltSensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot);
  virtual void Advance(Real dt);
  virtual void Reset();
  virtual bool ReadState(File& f);
  virtual bool WriteState(File& f) const;
  virtual void MeasurementNames(vector<string>& names) const;
  virtual void GetMeasurements(vector<double>& values) const;
  virtual void SetMeasurements(const vector<double>& values);
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  int link;
  Vector3 referenceDir;
  Matrix3 Rsensor;  ///< Position of unit on link
  bool hasAxis[3];  ///< true if tilt is measured on the x, y, z axes
  Vector3 resolution,variance;
  bool hasVelocity;

  Vector3 alocal,wlocal;
};


/** @ingroup Control
 * @brief Simulates a gyroscope.
 */
class GyroSensor : public SensorBase
{
 public:
  GyroSensor();
  virtual const char* Type() const { return "GyroSensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot);
  virtual void Reset();
  virtual void Advance(Real dt);
  virtual bool ReadState(File& f);
  virtual bool WriteState(File& f) const;
  virtual void MeasurementNames(vector<string>& names) const;
  virtual void GetMeasurements(vector<double>& values) const;
  virtual void SetMeasurements(const vector<double>& values);
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  int link;                ///< The link on which the sensor is located
  bool hasAngAccel;        ///< True if angular accel is directly measured
  bool hasAngVel;          ///< True if angular velocity is directly measured
  bool hasRotation;        ///< True if rotation is directly measured
  Matrix3 angAccelVariance;///< The variance associated with the measurement
  Matrix3 angVelVariance;  ///< The variance associated with the measurement
  Matrix3 rotationVariance;///< The variance associated with the measurement

  Vector3 angAccel;        ///< Measurement: the angular accel reading
  Vector3 angVel;          ///< Measurement: the angular velocity reading
  Matrix3 rotation;        ///< Measurement: the rotation matrix reading

  Real last_dt;            ///< Temporary: needed to derive accel from ODE
  Vector3 last_w;          ///< Temporary: needed to derive accel from ODE
};

/** @ingroup Control
 * @brief An inertial measurement unit.  May provide all or some of a 
 * rigid body's state.
 */
class IMUSensor : public SensorBase
{
 public:
  IMUSensor();
  virtual const char* Type() const { return "IMUSensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot);
  virtual void Advance(Real dt);
  virtual void Reset();
  virtual void MeasurementNames(vector<string>& names) const;
  virtual void GetMeasurements(vector<double>& values) const;
  virtual void SetMeasurements(const vector<double>& values);
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  Accelerometer accelerometer;
  GyroSensor gyro;
  Vector3 accel,velocity,translation;
  Vector3 angAccel,angVel;
  Matrix3 rotation;
};

/** @ingroup Control
 * @brief An exponentially smoothed "piggyback" filter.
 */
class FilteredSensor : public SensorBase
{
 public:
  FilteredSensor();
  virtual const char* Type() const { return "FilteredSensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot);
  virtual void Advance(Real dt);
  virtual void Reset();
  virtual void MeasurementNames(vector<string>& names) const;
  virtual void GetMeasurements(vector<double>& values) const;
  virtual void SetMeasurements(const vector<double>& values);
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  SmartPointer<SensorBase> sensor;
  vector<double> measurements;
  Real smoothing;
};


/** @ingroup Control
 * @brief A set of sensors for the robot.
 *
 * Accepts saving/loading to XML format.
 */
class RobotSensors
{
 public:
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
    return ss;              \
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
    return ss;              \
  }



#endif
