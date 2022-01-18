#ifndef CONTROL_JOINT_SENSORS_H
#define CONTROL_JOINT_SENSORS_H

#include "Sensor.h"

namespace Klampt {
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
  virtual const char* Type() const override { return "JointPositionSensor"; }
  virtual void Simulate(SimRobotController* robot,Simulator* sim) override;
  virtual void SimulateKinematic(RobotModel& robot,WorldModel& world) override;
  virtual void Reset() override;
  virtual void MeasurementNames(vector<string>& names) const override;
  virtual void GetMeasurements(vector<double>& values) const override;
  virtual void SetMeasurements(const vector<double>& values) override;
  virtual map<string,string> Settings() const override;
  virtual bool GetSetting(const string& name,string& str) const override;
  virtual bool SetSetting(const string& name,const string& str) override;

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
  virtual const char* Type() const override { return "JointVelocitySensor"; }
  virtual void Simulate(SimRobotController* robot,Simulator* sim) override;
  virtual void SimulateKinematic(RobotModel& robot,WorldModel& world) override;
  virtual void Reset() override;
  virtual void MeasurementNames(vector<string>& names) const override;
  virtual void GetMeasurements(vector<double>& values) const override;
  virtual void SetMeasurements(const vector<double>& values) override;
  virtual map<string,string> Settings() const override;
  virtual bool GetSetting(const string& name,string& str) const override;
  virtual bool SetSetting(const string& name,const string& str) override;

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
  virtual const char* Type() const override { return "DriverTorqueSensor"; }
  virtual void Simulate(SimRobotController* robot,Simulator* sim) override;
  virtual void SimulateKinematic(RobotModel& robot,WorldModel& world) override;
  virtual void Reset() override;
  virtual void MeasurementNames(vector<string>& names) const override;
  virtual void GetMeasurements(vector<double>& values) const override;
  virtual void SetMeasurements(const vector<double>& values) override;
  virtual map<string,string> Settings() const override;
  virtual bool GetSetting(const string& name,string& str) const override;
  virtual bool SetSetting(const string& name,const string& str) override;

  vector<int> indices;   ///< The indices on which the torque sensors are located
  Vector tvariance;     ///< Estimated variance of the torque values
  Vector tresolution;   ///< Estimate on the torque resolution
  Vector t;             ///< Measurement: joint torques
};

} //namespace Klampt

#endif 
