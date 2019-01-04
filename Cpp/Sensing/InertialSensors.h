#ifndef CONTROL_INERTIAL_SENSORS_H
#define CONTROL_INERTIAL_SENSORS_H

#include "Sensor.h"
#include <KrisLibrary/math3d/primitives.h>
using namespace Math3D;

/** @ingroup Sensing
 * @brief Simulates an accelerometer.
 *
 * Configurable settings:
 * - link: int
 * - Tsensor: RigidTransform (default identity)
 * - hasAxis: bool [3]
 * - accelVariance: Vector3
 */
class Accelerometer : public SensorBase
{
 public:
  Accelerometer();
  virtual const char* Type() const { return "Accelerometer"; }
  virtual void Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim);
  virtual void SimulateKinematic(Robot& robot,RobotWorld& world);
  virtual void Advance(double dt);
  virtual void Reset();
  virtual void MeasurementNames(vector<string>& names) const;
  virtual void GetMeasurements(vector<double>& values) const;
  virtual void SetMeasurements(const vector<double>& values);
  virtual void GetInternalState(vector<double>& state) const;
  virtual void SetInternalState(const vector<double>& state);
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

/** @ingroup Sensing
 * @brief Simulates a tilt sensor that measures the angle of
 * a reference direction about certain axes.
 *
 * Configurable settings:
 * - link: int
 * - referenceDir: Vector3 (default 0,0,1)
 * - Rsensor: Matrix3 (default identity)
 * - hasAxis: bool [3]
 * - resolution: Vector3
 * - variance: Vector3
 * - hasVelocity: bool (default false)
 */
class TiltSensor : public SensorBase
{
 public:
  TiltSensor();
  virtual const char* Type() const { return "TiltSensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim);
  virtual void SimulateKinematic(Robot& robot,RobotWorld& world);
  virtual void Advance(double dt);
  virtual void Reset();
  virtual void MeasurementNames(vector<string>& names) const;
  virtual void GetMeasurements(vector<double>& values) const;
  virtual void SetMeasurements(const vector<double>& values);
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  int link;
  Vector3 referenceDir;
  Matrix3 Rsensor;  ///< Orientation of unit on link
  bool hasAxis[3];  ///< true if tilt is measured on the x, y, z axes
  Vector3 resolution,variance;
  bool hasVelocity;

  Vector3 alocal,wlocal;
};


/** @ingroup Sensing
 * @brief Simulates a gyroscope.
 *
 * Configurable settings:
 * - link: int
 * - hasAngAccel: bool (default 0)
 * - hasAngVel: bool (default 0)
 * - hasRotation: bool (default 0)
 * - angAccelVariance: Matrix3
 * - angVelVariance: Matrix3
 * - rotationVariance: Matrix3
 */
class GyroSensor : public SensorBase
{
 public:
  GyroSensor();
  virtual const char* Type() const { return "GyroSensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim);
  virtual void SimulateKinematic(Robot& robot,RobotWorld& world);
  virtual void Reset();
  virtual void Advance(Real dt);
  virtual void MeasurementNames(vector<string>& names) const;
  virtual void GetMeasurements(vector<double>& values) const;
  virtual void SetMeasurements(const vector<double>& values);
  virtual void GetInternalState(vector<double>& state) const;
  virtual void SetInternalState(const vector<double>& state);
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

/** @ingroup Sensing
 * @brief An inertial measurement unit.  May provide all or some of a 
 * rigid body's state.
 *
 * Configurable settings:
 * see Accelerometer and GyroSensor
 */
class IMUSensor : public SensorBase
{
 public:
  IMUSensor();
  virtual const char* Type() const { return "IMUSensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim);
  virtual void SimulateKinematic(Robot& robot,RobotWorld& world);
  virtual void Advance(Real dt);
  virtual void Reset();
  virtual void MeasurementNames(vector<string>& names) const;
  virtual void GetMeasurements(vector<double>& values) const;
  virtual void SetMeasurements(const vector<double>& values);
  virtual void GetInternalState(vector<double>& state) const;
  virtual void SetInternalState(const vector<double>& state);
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  Accelerometer accelerometer;
  GyroSensor gyro;
  Vector3 accel,velocity,translation;
  Vector3 angAccel,angVel;
  Matrix3 rotation;
};

#endif
