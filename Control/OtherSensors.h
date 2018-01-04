#ifndef CONTROL_OTHER_SENSORS_H
#define CONTROL_OTHER_SENSORS_H

#include "Sensor.h"
#include <deque>

/** @ingroup Control
 * @brief A transformed "piggyback" sensor with a scale, bias, and minimum / maximum
 *
 * Configurable settings:
 * - scale: the scale of the sensor (either 1-D or n-D)
 * - bias: the bias of the sensor (either 1-D or n-D)
 * - minimum/maximum: the minimum/maximum of the sensor (either 1-D or n-D) applied after scaling and biasing.
 */
class TransformedSensor : public SensorBase
{
 public:
  TransformedSensor();
  virtual ~TransformedSensor() {}
  virtual const char* Type() const { return "TransformedSensor"; }
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
  virtual void DrawGL(const Robot& robot,const vector<double>& measurements);
  void DoTransform();

  SmartPointer<SensorBase> sensor;
  vector<double> scale;
  vector<double> bias;
  vector<double> minimum,maximum;

  vector<double> measurements;
};

/** @ingroup Control
 * @brief A "piggyback" sensor that corrupts readings with quantization error and gaussian noise
 *
 * Configurable settings:
 * - variance: the variance of the sensor (either 1-D or n-D)
 * - resolution: the minimum resolution of the sensor (either 1-D or n-D)
 */
class CorruptedSensor : public SensorBase
{
 public:
  CorruptedSensor();
  virtual ~CorruptedSensor() {}
  virtual const char* Type() const { return "CorruptedSensor"; }
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
  virtual void DrawGL(const Robot& robot,const vector<double>& measurements);
  void DoCorrupt();

  SmartPointer<SensorBase> sensor;
  vector<double> resolution;
  vector<double> variance;

  vector<double> measurements;
};

/** @ingroup Control
 * @brief An exponentially smoothed filter that acts as a "piggyback" sensor.
 *
 * Configurable settings:
 * - smoothing: the amount of exponential smoothing
 */
class FilteredSensor : public SensorBase
{
 public:
  FilteredSensor();
  virtual ~FilteredSensor() {}
  virtual const char* Type() const { return "FilteredSensor"; }
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
  virtual void DrawGL(const Robot& robot,const vector<double>& measurements);

  SmartPointer<SensorBase> sensor;
  vector<double> measurements;
  Real smoothing;
};

/** @ingroup Control
 * @brief An time delayed "piggyback" sensor.
 *
 * Configurable settings:
 * - delay: the amount of delay
 * - jitter: the amount of random jitter
 */
class TimeDelayedSensor : public SensorBase
{
 public:
  TimeDelayedSensor();
  virtual ~TimeDelayedSensor() {}
  virtual const char* Type() const { return "TimeDelayedSensor"; }
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
  virtual void DrawGL(const Robot& robot,const vector<double>& measurements);

  SmartPointer<SensorBase> sensor;
  deque<vector<double> > measurementsInTransit;
  deque<double> deliveryTimes;
  vector<double> arrivedMeasurement;
  double curTime;
  double delay,jitter;
};

#endif
