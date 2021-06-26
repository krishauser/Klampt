#include "OtherSensors.h"
#include "Common_Internal.h"
#include <KrisLibrary/errors.h>

DECLARE_LOGGER(Sensing);
using namespace Klampt;

TransformedSensor::TransformedSensor()
{}

void TransformedSensor::SimulateKinematic(RobotModel& robot,WorldModel& world)
{
  if(!sensor) return;
  sensor->SimulateKinematic(robot,world);
  sensor->GetMeasurements(measurements);
  DoTransform();
}

void TransformedSensor::DoTransform()
{
  if(!scale.empty()) {
    if(scale.size() == 1) {
      for(size_t i=0;i<measurements.size();i++)
        measurements[i] *= scale[0];
    }
    else {
      if(measurements.size() != scale.size()) {
        LOG4CXX_ERROR(GET_LOGGER(Sensing),"TransformedSensor "<<sensor->name<<" has incorrect scale size ("<<measurements.size()<<" != "<<scale.size()<<")");
      }
      else {
        for(size_t i=0;i<measurements.size();i++)
          measurements[i] *= scale[i];
      }
    }
  }
  if(!bias.empty()) {
    if(bias.size() == 1) {
      for(size_t i=0;i<measurements.size();i++)
        measurements[i] += bias[0];
    }
    else {
      if(measurements.size() != bias.size()) {
        LOG4CXX_ERROR(GET_LOGGER(Sensing),"TransformedSensor "<<sensor->name<<" has incorrect bias size ("<<measurements.size()<<" != "<<bias.size()<<")");
      }
      else {
        for(size_t i=0;i<measurements.size();i++)
          measurements[i] += bias[i];
      }
    }
  }
  if(!minimum.empty()) {
    if(minimum.size() == 1) {
      for(size_t i=0;i<measurements.size();i++)
        measurements[i] = Max(minimum[0],measurements[i]);
    }
    else {
      if(measurements.size() != minimum.size()) {
        LOG4CXX_ERROR(GET_LOGGER(Sensing),"TransformedSensor "<<sensor->name<<" has incorrect minimum size ("<<measurements.size()<<" != "<<minimum.size()<<")");
      }
      else {
        for(size_t i=0;i<measurements.size();i++)
          measurements[i] = Max(minimum[i],measurements[i]);
      }
    }
  }
  if(!maximum.empty()) {
    if(maximum.size() == 1) {
      for(size_t i=0;i<measurements.size();i++)
        measurements[i] = Max(maximum[0],measurements[i]);
    }
    else {
      if(measurements.size() != maximum.size()) {
        LOG4CXX_ERROR(GET_LOGGER(Sensing),"TransformedSensor "<<sensor->name<<" has incorrect maximum size ("<<measurements.size()<<" != "<<maximum.size()<<")");
      }
      else {
        for(size_t i=0;i<measurements.size();i++)
          measurements[i] = Max(maximum[i],measurements[i]);
      }
    }
  }
}

void TransformedSensor::Simulate(SimRobotController* robot,Simulator* sim)
{
  if(!sensor) return;
  sensor->Simulate(robot,sim);
  sensor->GetMeasurements(measurements);
  DoTransform();
}

void TransformedSensor::Advance(Real dt)
{
  if(!sensor) return;
  sensor->Advance(dt);
}

void TransformedSensor::Reset()
{
  fill(measurements.begin(),measurements.end(),0.0);
  if(sensor) sensor->Reset();
}

void TransformedSensor::MeasurementNames(vector<string>& names) const
{
  if(sensor)
    sensor->MeasurementNames(names);
}

void TransformedSensor::GetMeasurements(vector<double>& values) const
{
  values = measurements;
}

void TransformedSensor::SetMeasurements(const vector<double>& values)
{
  measurements = values;
}

void TransformedSensor::GetInternalState(vector<double>& values) const
{
  if(sensor) {
    sensor->GetInternalState(values);
  }
}
void TransformedSensor::SetInternalState(const vector<double>& state)
{
  if(sensor) sensor->SetInternalState(state);
}

map<string,string> TransformedSensor::Settings() const
{
  map<string,string> settings = SensorBase::Settings();
  FILL_VECTOR_SENSOR_SETTING(settings,scale);
  FILL_VECTOR_SENSOR_SETTING(settings,bias);
  FILL_VECTOR_SENSOR_SETTING(settings,minimum);
  FILL_VECTOR_SENSOR_SETTING(settings,maximum);
  return settings;
}

bool TransformedSensor::GetSetting(const string& name,string& str) const
{
  if(SensorBase::GetSetting(name,str)) return true;
  GET_VECTOR_SENSOR_SETTING(scale);
  GET_VECTOR_SENSOR_SETTING(bias);
  GET_VECTOR_SENSOR_SETTING(minimum);
  GET_VECTOR_SENSOR_SETTING(maximum);
  return false;
}

bool TransformedSensor::SetSetting(const string& name,const string& str)
{
  if(SensorBase::SetSetting(name,str)) return true;
  SET_VECTOR_SENSOR_SETTING(scale);
  SET_VECTOR_SENSOR_SETTING(bias);
  SET_VECTOR_SENSOR_SETTING(minimum);
  SET_VECTOR_SENSOR_SETTING(maximum);
  return false;
}

void TransformedSensor::DrawGL(const RobotModel& robot,const vector<double>& measurements)
{
  if(sensor) sensor->DrawGL(robot,measurements);
}



CorruptedSensor::CorruptedSensor()
{}

void CorruptedSensor::SimulateKinematic(RobotModel& robot,WorldModel& world)
{
  if(!sensor) return;
  sensor->SimulateKinematic(robot,world);
  sensor->GetMeasurements(measurements);
  DoCorrupt();
}

void CorruptedSensor::DoCorrupt()
{
  if(!variance.empty()) {
    if(variance.size() == 1) {
      for(size_t i=0;i<measurements.size();i++)
        if(measurements[i]!=0.0)
          measurements[i] += RandGaussian()*Sqrt(variance[0]);
    }
    else {
      if(measurements.size() != variance.size()) {
        LOG4CXX_ERROR(GET_LOGGER(Sensing),"CorruptedSensor "<<sensor->name<<" has incorrect variance size ("<<measurements.size()<<" != "<<variance.size()<<")");
      }
      else {
        for(size_t i=0;i<measurements.size();i++)
          if(measurements[i]!=0.0)
            measurements[i] += RandGaussian()*Sqrt(variance[i]);
      }
    }
  }
  if(!resolution.empty()) {
    if(resolution.size() == 1) {
      for(size_t i=0;i<measurements.size();i++)
        measurements[i] = round(measurements[i]/resolution[0])*resolution[0];
    }
    else {
      if(measurements.size() != resolution.size()) {
        LOG4CXX_ERROR(GET_LOGGER(Sensing),"CorruptedSensor "<<sensor->name<<" has incorrect resolution size ("<<measurements.size()<<" != "<<resolution.size()<<")");
      }
      else {
        for(size_t i=0;i<measurements.size();i++)
          measurements[i] = round(measurements[i]/resolution[i])*resolution[i];
      }
    }
  }
}

void CorruptedSensor::Simulate(SimRobotController* robot,Simulator* sim)
{
  if(!sensor) return;
  sensor->Simulate(robot,sim);
  sensor->GetMeasurements(measurements);
  DoCorrupt();
}

void CorruptedSensor::Advance(Real dt)
{
  if(!sensor) return;
  sensor->Advance(dt);
}

void CorruptedSensor::Reset()
{
  fill(measurements.begin(),measurements.end(),0.0);
  if(sensor) sensor->Reset();
}

void CorruptedSensor::MeasurementNames(vector<string>& names) const
{
  if(sensor)
    sensor->MeasurementNames(names);
}

void CorruptedSensor::GetMeasurements(vector<double>& values) const
{
  values = measurements;
}

void CorruptedSensor::SetMeasurements(const vector<double>& values)
{
  measurements = values;
}

void CorruptedSensor::GetInternalState(vector<double>& values) const
{
  if(sensor) {
    sensor->GetInternalState(values);
  }
}
void CorruptedSensor::SetInternalState(const vector<double>& state)
{
  if(sensor) sensor->SetInternalState(state);
}

map<string,string> CorruptedSensor::Settings() const
{
  map<string,string> settings = SensorBase::Settings();
  FILL_VECTOR_SENSOR_SETTING(settings,variance);
  FILL_VECTOR_SENSOR_SETTING(settings,resolution);
  return settings;
}

bool CorruptedSensor::GetSetting(const string& name,string& str) const
{
  if(SensorBase::GetSetting(name,str)) return true;
  GET_VECTOR_SENSOR_SETTING(resolution);
  GET_VECTOR_SENSOR_SETTING(variance);
  return false;
}

bool CorruptedSensor::SetSetting(const string& name,const string& str)
{
  if(SensorBase::SetSetting(name,str)) return true;
  SET_VECTOR_SENSOR_SETTING(resolution);
  SET_VECTOR_SENSOR_SETTING(variance);
  return false;
}

void CorruptedSensor::DrawGL(const RobotModel& robot,const vector<double>& measurements)
{
  if(sensor) sensor->DrawGL(robot,measurements);
}





FilteredSensor::FilteredSensor()
  :smoothing(0)
{}

void FilteredSensor::SimulateKinematic(RobotModel& robot,WorldModel& world)
{
  if(!sensor) return;
  sensor->SimulateKinematic(robot,world);
}

void FilteredSensor::Simulate(SimRobotController* robot,Simulator* sim)
{
  if(!sensor) return;
  sensor->Simulate(robot,sim);
}

void FilteredSensor::Advance(Real dt)
{
  if(!sensor) return;
  vector<double> newMeasurements;
  sensor->GetMeasurements(newMeasurements);
  sensor->Advance(dt);
  if(measurements.empty()) {
    measurements.resize(newMeasurements.size(),0.0);
  }
  for(size_t i=0;i<measurements.size();i++) {
    measurements[i] = smoothing*measurements[i] + (1.0-smoothing)*newMeasurements[i];
  }
}

void FilteredSensor::Reset()
{
  fill(measurements.begin(),measurements.end(),0.0);
  if(sensor) sensor->Reset();
}

void FilteredSensor::MeasurementNames(vector<string>& names) const
{
  if(sensor)
    sensor->MeasurementNames(names);
}

void FilteredSensor::GetMeasurements(vector<double>& values) const
{
  values = measurements;
}

void FilteredSensor::SetMeasurements(const vector<double>& values)
{
  measurements = values;
}

void FilteredSensor::GetInternalState(vector<double>& values) const
{
  if(sensor) {
    sensor->GetInternalState(values);
  }
}
void FilteredSensor::SetInternalState(const vector<double>& state)
{
  if(sensor) sensor->SetInternalState(state);
}

map<string,string> FilteredSensor::Settings() const
{
  map<string,string> settings = SensorBase::Settings();
  FILL_SENSOR_SETTING(settings,smoothing);
  return settings;
}

bool FilteredSensor::GetSetting(const string& name,string& str) const
{
  if(SensorBase::GetSetting(name,str)) return true;
  GET_SENSOR_SETTING(smoothing);
  return false;
}

bool FilteredSensor::SetSetting(const string& name,const string& str)
{
  if(SensorBase::SetSetting(name,str)) return true;
  SET_SENSOR_SETTING(smoothing);
  return false;
}

void FilteredSensor::DrawGL(const RobotModel& robot,const vector<double>& measurements)
{
  if(sensor) sensor->DrawGL(robot,measurements);
}


TimeDelayedSensor::TimeDelayedSensor()
  :curTime(0),delay(0),jitter(0)
{}


void TimeDelayedSensor::SimulateKinematic(RobotModel& robot,WorldModel& world)
{
  if(!sensor) return;
  sensor->SimulateKinematic(robot,world);
  vector<double> newMeasurements;
  sensor->GetMeasurements(newMeasurements);
  double delivTime = curTime + delay + Rand(-jitter,jitter);
  measurementsInTransit.push_back(newMeasurements);
  deliveryTimes.push_back(delivTime);

  while(!deliveryTimes.empty() && deliveryTimes.front() <= curTime) {
    swap(arrivedMeasurement,measurementsInTransit.front());
    measurementsInTransit.pop_front();
    deliveryTimes.pop_front();
  }
}

void TimeDelayedSensor::Simulate(SimRobotController* robot,Simulator* sim)
{
  if(!sensor) return;
  sensor->Simulate(robot,sim);
  vector<double> newMeasurements;
  sensor->GetMeasurements(newMeasurements);
  double delivTime = curTime + delay + Rand(-jitter,jitter);
  measurementsInTransit.push_back(newMeasurements);
  deliveryTimes.push_back(delivTime);

  while(!deliveryTimes.empty() && deliveryTimes.front() <= curTime) {
    swap(arrivedMeasurement,measurementsInTransit.front());
    measurementsInTransit.pop_front();
    deliveryTimes.pop_front();
  }
}

void TimeDelayedSensor::Advance(Real dt)
{
  curTime += dt;
  if(!sensor) return;
  sensor->Advance(dt);
}

void TimeDelayedSensor::Reset()
{
  if(sensor) sensor->Reset();
  measurementsInTransit.clear();
  deliveryTimes.clear();
  arrivedMeasurement.clear();
  curTime = 0;
}

void TimeDelayedSensor::MeasurementNames(vector<string>& names) const
{
  if(sensor)
    sensor->MeasurementNames(names);
}

void TimeDelayedSensor::GetMeasurements(vector<double>& values) const
{
  values = arrivedMeasurement;
}


void TimeDelayedSensor::SetMeasurements(const vector<double>& values)
{
  arrivedMeasurement = values;
}

void TimeDelayedSensor::GetInternalState(vector<double>& state) const
{
  if(!sensor) return;
  vector<double> sstate;
  sensor->GetInternalState(sstate);
  size_t n = 0;
  if(!measurementsInTransit.empty()) n = measurementsInTransit.front().size();
  state = sstate;
  state.push_back(curTime);
  state.push_back(double(deliveryTimes.size()));
  state.push_back(double(n));
  for(deque<vector<double> >::const_iterator i=measurementsInTransit.begin();i!=measurementsInTransit.end();i++) {
    Assert(i->size()==n);
    for(size_t j=0;j<i->size();j++)
      state.push_back((*i)[j]);
  }
  for(deque<double>::const_iterator i=deliveryTimes.begin();i!=deliveryTimes.end();i++)
    state.push_back(*i);
}

void TimeDelayedSensor::SetInternalState(const vector<double>& state)
{
  if(!sensor) return;
  //just to get the size
  vector<double> sstate;
  sensor->GetInternalState(sstate);
  copy(state.begin(),state.begin()+sstate.size(),sstate.begin());
  sensor->SetInternalState(sstate);
  vector<double>::const_iterator readpos = state.begin()+sstate.size();
  curTime = *readpos; readpos++;
  int k = int(*readpos); readpos++;
  int n = int(*readpos); readpos++;
  Assert(k >= 0);
  Assert(n >= 0);
  deliveryTimes.resize(0);
  measurementsInTransit.resize(0);
  for(int i=0;i<k;i++) {
    vector<double> meas;
    for(int j=0;j<n;j++) {
      meas.push_back(double(*readpos));
      readpos++;
    }
    measurementsInTransit.push_back(meas);
  }
  for(int i=0;i<k;i++) {
    deliveryTimes.push_back(double(*readpos));
    readpos++;
  }
}

map<string,string> TimeDelayedSensor::Settings() const
{
  map<string,string> settings = SensorBase::Settings();
  FILL_SENSOR_SETTING(settings,delay);
  FILL_SENSOR_SETTING(settings,jitter);
  return settings;
}

bool TimeDelayedSensor::GetSetting(const string& name,string& str) const
{
  if(SensorBase::GetSetting(name,str)) return true;
  GET_SENSOR_SETTING(delay);
  GET_SENSOR_SETTING(jitter);
  return false;
}

bool TimeDelayedSensor::SetSetting(const string& name,const string& str)
{
  if(SensorBase::SetSetting(name,str)) return true;
  SET_SENSOR_SETTING(delay);
  SET_SENSOR_SETTING(jitter);
  return false;
}

void TimeDelayedSensor::DrawGL(const RobotModel& robot,const vector<double>& measurements)
{
  if(sensor) sensor->DrawGL(robot,measurements);
}

