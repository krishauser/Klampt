#ifndef CONTROL_OTHER_SENSORS_H
#define CONTROL_OTHER_SENSORS_H

#include "Sensor.h"
#include <deque>

/** @ingroup Control
 * @brief An exponentially smoothed filter that acts as a "piggyback" sensor.
 */
class FilteredSensor : public SensorBase
{
 public:
  FilteredSensor();
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
 */
class TimeDelayedSensor : public SensorBase
{
 public:
  TimeDelayedSensor();
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
