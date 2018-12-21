#ifndef CONTROL_JOINT_SENSORS_H
#define CONTROL_JOINT_SENSORS_H

#include "Sensor.h"
using namespace Math;

/** @ingroup Sensing
 * @brief Simulates a joint encoder.
 *
 * Configurable settings:
 * - indices: integer array
 * - qvariance: Vector
 * - qresolution: Vector
 */
class JointPositionSensor : public SensorBase
{
 public:
  JointPositionSensor();
  virtual ~JointPositionSensor() {}
  virtual const char* Type() const { return "JointPositionSensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim);
  virtual void SimulateKinematic(Robot& robot,RobotWorld& world);
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

/** @ingroup Sensing
 * @brief Simulates a velocity sensor.  (Does not perform differencing)
 *
 * Configurable settings:
 * - indices: integer array
 * - dqvariance: Vector
 * - dqresolution: Vector
 */
class JointVelocitySensor : public SensorBase
{
 public:
  JointVelocitySensor();
  virtual ~JointVelocitySensor() {}
  virtual const char* Type() const { return "JointVelocitySensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim);
  virtual void SimulateKinematic(Robot& robot,RobotWorld& world);
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

/** @ingroup Sensing
 * @brief Simulates a torque sensor.
 * 
 * Motors typically provide current/voltage information, and this assumes
 * that it is transformed into torque units.
 *
 * Configurable settings:
 * - indices: integer array
 * - tvariance: Vector
 * - tresolution: Vector
 */
class DriverTorqueSensor : public SensorBase
{
 public:
  DriverTorqueSensor();
  virtual ~DriverTorqueSensor() {}
  virtual const char* Type() const { return "DriverTorqueSensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim);
  virtual void SimulateKinematic(Robot& robot,RobotWorld& world);
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

#endif 
