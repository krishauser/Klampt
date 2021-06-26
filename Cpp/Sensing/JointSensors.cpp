#include "JointSensors.h"
#include "Common_Internal.h"
#include "Simulation/SimRobotController.h"
DECLARE_LOGGER(Sensing);
using namespace Klampt;

JointPositionSensor::JointPositionSensor()
{}

void JointPositionSensor::SimulateKinematic(RobotModel& robot,WorldModel& world)
{
  q = robot.q;
  if(!qvariance.empty()) {
    //cout<<"q: "<<qvariance<<endl;
    for(int i=0;i<q.n;i++)
      q(i) += RandGaussian()*Sqrt(qvariance(i));
  }
  if(!qresolution.empty()) {
    //cout<<"q: "<<qresolution<<endl;
    for(int i=0;i<q.n;i++) {
      if(qresolution(i) > 0) {
  q(i) = round(q(i)/qresolution(i))*qresolution(i);
      }
    }
  }
  if(!indices.empty()) {
    //only read a subset
    Vector qread(indices.size(),Zero);
    for(size_t i=0;i<indices.size();i++)
      qread(i) = q(indices[i]);
    swap(qread,q);
  }
}

void JointPositionSensor::Simulate(SimRobotController* robot,Simulator* sim)
{
  robot->oderobot->GetConfig(q);
  if(!qvariance.empty()) {
    //cout<<"q: "<<qvariance<<endl;
    for(int i=0;i<q.n;i++)
      q(i) += RandGaussian()*Sqrt(qvariance(i));
  }
  if(!qresolution.empty()) {
    //cout<<"q: "<<qresolution<<endl;
    for(int i=0;i<q.n;i++) {
      if(qresolution(i) > 0) {
        q(i) = round(q(i)/qresolution(i))*qresolution(i);
      }
    }
  }
  if(!indices.empty()) {
    //only read a subset
    Vector qread(indices.size(),Zero);
    for(size_t i=0;i<indices.size();i++)
      qread(i) = q(indices[i]);
    swap(qread,q);
  }
}

void JointPositionSensor::Reset()
{
  if(!indices.empty())
    q.resize(indices.size(),0.0);
  else
    q.setZero();
}

void JointPositionSensor::MeasurementNames(std::vector<std::string>& names) const
{
  if(!indices.empty()) {
    names.resize(indices.size());
    for(size_t i=0;i<indices.size();i++) {
      stringstream ss; ss<<"q["<<indices[i]<<"]";
      names[i] = ss.str();
    }
  }
  else {
    names.resize(q.n);
    for(size_t i=0;i<names.size();i++) {
      stringstream ss; ss<<"q["<<i<<"]";
      names[i] = ss.str();
    }
  }
}

void JointPositionSensor::GetMeasurements(std::vector<double>& values) const
{
  values = q;
}

void JointPositionSensor::SetMeasurements(const std::vector<double>& values)
{
  q.resize(0);
  q = values;
}

map<string,string> JointPositionSensor::Settings() const
{
  map<string,string> settings = SensorBase::Settings();
  FILL_SENSOR_SETTING(settings,qvariance);
  FILL_SENSOR_SETTING(settings,qresolution);
  GetSetting("indices",settings["indices"]);
  return settings;
}

bool JointPositionSensor::GetSetting(const string& name,string& str) const
{
  if(SensorBase::GetSetting(name,str)) return true;
  GET_SENSOR_SETTING(qvariance);
  GET_SENSOR_SETTING(qresolution);
  if(name == "indices") {
    stringstream ss;
    for(size_t i=0;i<indices.size();i++)
      ss<<indices[i]<<" ";
    str = ss.str();
    return true;
  }
  return false;
}

bool JointPositionSensor::SetSetting(const string& name,const string& str)
{
  if(SensorBase::SetSetting(name,str)) return true;
  SET_SENSOR_SETTING(qvariance);
  SET_SENSOR_SETTING(qresolution);
  if(name == "indices") {
    stringstream ss(str);
    indices.resize(0);
    while(ss) {
      int index;
      ss>>index;
      if(ss) indices.push_back(index);
    }
    return true;
  }
  return false;
}



JointVelocitySensor::JointVelocitySensor()
{}

void JointVelocitySensor::SimulateKinematic(RobotModel& robot,WorldModel& world)
{
  dq = robot.dq;
  if(!dqvariance.empty()) {
    //cout<<"dq: "<<qvariance<<endl;
    for(int i=0;i<dq.n;i++)
      dq(i) += RandGaussian()*Sqrt(dqvariance(i));
  }
  if(!dqresolution.empty()) {
    //cout<<"dq: "<<dqresolution<<endl;
    for(int i=0;i<dq.n;i++) {
      if(dqresolution(i) > 0) {
  dq(i) = int(dq(i)/dqresolution(i)+0.5)*dqresolution(i);
      }
    }
  }
  if(!indices.empty()) {
    //only read a subset
    Vector dqread(indices.size(),Zero);
    for(size_t i=0;i<indices.size();i++)
      dqread(i) = dq(indices[i]);
    swap(dqread,dq);
  }
}

void JointVelocitySensor::Simulate(SimRobotController* robot,Simulator* sim)
{
  robot->oderobot->GetVelocities(dq);
  if(!dqvariance.empty()) {
    //cout<<"dq: "<<qvariance<<endl;
    for(int i=0;i<dq.n;i++)
      dq(i) += RandGaussian()*Sqrt(dqvariance(i));
  }
  if(!dqresolution.empty()) {
    //cout<<"dq: "<<dqresolution<<endl;
    for(int i=0;i<dq.n;i++) {
      if(dqresolution(i) > 0) {
        dq(i) = int(dq(i)/dqresolution(i)+0.5)*dqresolution(i);
      }
    }
  }
  if(!indices.empty()) {
    //only read a subset
    Vector dqread(indices.size(),Zero);
    for(size_t i=0;i<indices.size();i++)
      dqread(i) = dq(indices[i]);
    swap(dqread,dq);
  }
}

void JointVelocitySensor::Reset()
{
  if(!indices.empty())
    dq.resize(indices.size(),0.0);
  else
    dq.setZero();
}

void JointVelocitySensor::MeasurementNames(std::vector<std::string>& names) const
{
  if(!indices.empty()) {
    names.resize(indices.size());
    for(size_t i=0;i<indices.size();i++) {
      stringstream ss; ss<<"dq["<<indices[i]<<"]";
      names[i] = ss.str();
    }
  }
  else {
    names.resize(dq.n);
    for(size_t i=0;i<names.size();i++) {
      stringstream ss; ss<<"dq["<<i<<"]";
      names[i] = ss.str();
    }
  }
}

void JointVelocitySensor::GetMeasurements(std::vector<double>& values) const
{
  values = dq;
}

void JointVelocitySensor::SetMeasurements(const std::vector<double>& values)
{
  dq.resize(0);
  dq = values;
}

map<string,string> JointVelocitySensor::Settings() const
{
  map<string,string> settings = SensorBase::Settings();
  FILL_SENSOR_SETTING(settings,dqvariance);
  FILL_SENSOR_SETTING(settings,dqresolution);
  GetSetting("indices",settings["indices"]);
  return settings;
}

bool JointVelocitySensor::GetSetting(const string& name,string& str) const
{
  if(SensorBase::GetSetting(name,str)) return true;
  GET_SENSOR_SETTING(dqvariance);
  GET_SENSOR_SETTING(dqresolution);
  if(name == "indices") {
    stringstream ss;
    for(size_t i=0;i<indices.size();i++)
      ss<<indices[i]<<" ";
    str = ss.str();
    return true;
  }
  return false;
}

bool JointVelocitySensor::SetSetting(const string& name,const string& str)
{
  if(SensorBase::SetSetting(name,str)) return true;
  SET_SENSOR_SETTING(dqvariance);
  SET_SENSOR_SETTING(dqresolution);
  if(name == "indices") {
    stringstream ss(str);
    indices.resize(0);
    while(ss) {
      int index;
      ss>>index;
      if(ss) indices.push_back(index);
    }
    return true;
  }
  return false;
}



DriverTorqueSensor::DriverTorqueSensor()
{}

void DriverTorqueSensor::SimulateKinematic(RobotModel& robot,WorldModel& world) 
{
  t.resize(robot.drivers.size(),0.0);
  if(!tvariance.empty()) {
    for(int i=0;i<t.n;i++)
      t(i) += RandGaussian()*Sqrt(tvariance(i));
  }
  if(!tresolution.empty()) {
    for(int i=0;i<t.n;i++) {
      if(tresolution(i) > 0) {
  t(i) = int(t(i)/tresolution(i)+0.5)*tresolution(i);
      }
    }
  }
  if(!indices.empty()) {
    //only read a subset
    Vector tread(t.n,Zero);
    for(size_t i=0;i<indices.size();i++) {
      assert(indices[i] < t.n && indices[i] >= 0);
      tread(indices[i]) = t(indices[i]);
    }
    swap(tread,t);
  }
}

void DriverTorqueSensor::Simulate(SimRobotController* robot,Simulator* sim)
{
  Assert(robot->command.actuators.size() == robot->robot->drivers.size());
  robot->GetActuatorTorques(t);
  //TODO: for fixed velocity motors, need to get joint feedback to obtain
  //ODE computed torques
  if(!tvariance.empty()) {
    for(int i=0;i<t.n;i++)
      t(i) += RandGaussian()*Sqrt(tvariance(i));
  }
  if(!tresolution.empty()) {
    for(int i=0;i<t.n;i++) {
      if(tresolution(i) > 0) {
        t(i) = int(t(i)/tresolution(i)+0.5)*tresolution(i);
      }
    }
  }
  if(!indices.empty()) {
    //only read a subset
    Vector tread(t.n,Zero);
    for(size_t i=0;i<indices.size();i++) {
      assert(indices[i] < t.n && indices[i] >= 0);
      tread(indices[i]) = t(indices[i]);
    }
    swap(tread,t);
  }
}

void DriverTorqueSensor::Reset()
{
  if(!indices.empty())
    t.resize(indices.size(),0.0);
  else
    t.setZero();
}

void DriverTorqueSensor::MeasurementNames(std::vector<std::string>& names) const
{
  if(!indices.empty()) {
    names.resize(indices.size());
    for(size_t i=0;i<indices.size();i++) {
      stringstream ss; ss<<"t["<<indices[i]<<"]";
      names[i] = ss.str();
    }
  }
  else {
    names.resize(t.n);
    for(size_t i=0;i<names.size();i++) {
      stringstream ss; ss<<"t["<<i<<"]";
      names[i] = ss.str();
    }
  }
}

void DriverTorqueSensor::GetMeasurements(std::vector<double>& values) const
{
  values = t;
}

void DriverTorqueSensor::SetMeasurements(const std::vector<double>& values)
{
  t.resize(0);
  t = values;
}

map<string,string> DriverTorqueSensor::Settings() const
{
  map<string,string> settings = SensorBase::Settings();
  FILL_SENSOR_SETTING(settings,tvariance);
  FILL_SENSOR_SETTING(settings,tresolution);
  GetSetting("indices",settings["indices"]);
  return settings;
}

bool DriverTorqueSensor::GetSetting(const string& name,string& str) const
{
  if(SensorBase::GetSetting(name,str)) return true;
  GET_SENSOR_SETTING(tvariance);
  GET_SENSOR_SETTING(tresolution);
  if(name == "indices") {
    stringstream ss;
    for(size_t i=0;i<indices.size();i++)
      ss<<indices[i]<<" ";
    str = ss.str();
    return true;
  }
  return false;
}

bool DriverTorqueSensor::SetSetting(const string& name,const string& str)
{
  if(SensorBase::SetSetting(name,str)) return true;
  SET_SENSOR_SETTING(tvariance);
  SET_SENSOR_SETTING(tresolution);
  if(name == "indices") {
    stringstream ss(str);
    indices.resize(0);
    while(ss) {
      int index;
      ss>>index;
      if(ss) indices.push_back(index);
    }
    return true;
  }
  return false;
}
