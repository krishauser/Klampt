#ifndef CONTROL_OTHER_SENSORS_H
#define CONTROL_OTHER_SENSORS_H

#include "Sensor.h"
#include <deque>

namespace Klampt {

/** @ingroup Sensing
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
  virtual const char* Type() const override { return "TransformedSensor"; }
  virtual void Simulate(SimRobotController* robot,Simulator* sim) override;
  virtual void SimulateKinematic(RobotModel& robot,WorldModel& world) override;
  virtual void Advance(double dt) override;
  virtual void Reset() override;
  virtual void MeasurementNames(vector<string>& names) const override;
  virtual void GetMeasurements(vector<double>& values) const override;
  virtual void SetMeasurements(const vector<double>& values) override;
  virtual void GetInternalState(vector<double>& state) const override;
  virtual void SetInternalState(const vector<double>& state) override;
  virtual map<string,string> Settings() const override;
  virtual bool GetSetting(const string& name,string& str) const override;
  virtual bool SetSetting(const string& name,const string& str) override;
  virtual void DrawGL(const RobotModel& robot,const vector<double>& measurements) override;
  void DoTransform();

  shared_ptr<SensorBase> sensor;
  vector<double> scale;
  vector<double> bias;
  vector<double> minimum,maximum;

  vector<double> measurements;
};

/** @ingroup Sensing
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
  virtual const char* Type() const override { return "CorruptedSensor"; }
  virtual void Simulate(SimRobotController* robot,Simulator* sim) override;
  virtual void SimulateKinematic(RobotModel& robot,WorldModel& world) override;
  virtual void Advance(double dt) override;
  virtual void Reset() override;
  virtual void MeasurementNames(vector<string>& names) const override;
  virtual void GetMeasurements(vector<double>& values) const override;
  virtual void SetMeasurements(const vector<double>& values) override;
  virtual void GetInternalState(vector<double>& state) const override;
  virtual void SetInternalState(const vector<double>& state) override;
  virtual map<string,string> Settings() const override;
  virtual bool GetSetting(const string& name,string& str) const override;
  virtual bool SetSetting(const string& name,const string& str) override;
  virtual void DrawGL(const RobotModel& robot,const vector<double>& measurements) override;
  void DoCorrupt();

  shared_ptr<SensorBase> sensor;
  vector<double> resolution;
  vector<double> variance;

  vector<double> measurements;
};

/** @ingroup Sensing
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
  virtual const char* Type() const override { return "FilteredSensor"; }
  virtual void Simulate(SimRobotController* robot,Simulator* sim) override;
  virtual void SimulateKinematic(RobotModel& robot,WorldModel& world) override;
  virtual void Advance(double dt) override;
  virtual void Reset() override;
  virtual void MeasurementNames(vector<string>& names) const override;
  virtual void GetMeasurements(vector<double>& values) const override;
  virtual void SetMeasurements(const vector<double>& values) override;
  virtual void GetInternalState(vector<double>& state) const override;
  virtual void SetInternalState(const vector<double>& state) override;
  virtual map<string,string> Settings() const override;
  virtual bool GetSetting(const string& name,string& str) const override;
  virtual bool SetSetting(const string& name,const string& str) override;
  virtual void DrawGL(const RobotModel& robot,const vector<double>& measurements) override;

  shared_ptr<SensorBase> sensor;
  vector<double> measurements;
  double smoothing;
};

/** @ingroup Sensing
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
  virtual const char* Type() const override { return "TimeDelayedSensor"; }
  virtual void Simulate(SimRobotController* robot,Simulator* sim) override;
  virtual void SimulateKinematic(RobotModel& robot,WorldModel& world) override;
  virtual void Advance(double dt) override;
  virtual void Reset() override;
  virtual void MeasurementNames(vector<string>& names) const override;
  virtual void GetMeasurements(vector<double>& values) const override;
  virtual void SetMeasurements(const vector<double>& values) override;
  virtual void GetInternalState(vector<double>& state) const override;
  virtual void SetInternalState(const vector<double>& state) override;
  virtual map<string,string> Settings() const override;
  virtual bool GetSetting(const string& name,string& str) const override;
  virtual bool SetSetting(const string& name,const string& str) override;
  virtual void DrawGL(const RobotModel& robot,const vector<double>& measurements) override;

  shared_ptr<SensorBase> sensor;
  deque<vector<double> > measurementsInTransit;
  deque<double> deliveryTimes;
  vector<double> arrivedMeasurement;
  double curTime;
  double delay,jitter;
};

} //namespace Klampt

#endif
