#include "Sensor.h"
#include "JointSensors.h"
#include "ForceSensors.h"
#include "InertialSensors.h"
#include "OtherSensors.h"
#include "Simulation/ControlledSimulator.h"
#include "Simulation/ODESimulator.h"
#include <KrisLibrary/utils/PropertyMap.h>
#include <KrisLibrary/math/random.h>
#include <tinyxml.h>
#include <sstream>

#ifdef WIN32
static inline double round(double val) { return floor(val + 0.5); }
#endif //WIN32

//defined in ODESimulator.cpp
bool HasContact(dBodyID a);

//defined in ODESimulator.cpp
///Will produce bogus o1 and o2 vectors
void GetContacts(dBodyID a,vector<ODEContactList>& contacts);

//emulates a process that discretizes a continuous value into a digital one
//with resolution resolution, and variance variance
Real Discretize(Real value,Real resolution,Real variance)
{
  if(variance>0)
    value += RandGaussian()*Sqrt(variance);
  if(resolution>0)
    value = round(value/resolution)*resolution;
  return value;
}

Vector3 Discretize(const Vector3& value,const Vector3& resolution,const Vector3& variance)
{
  Vector3 res;
  res.x = Discretize(value.x,resolution.x,variance.x);
  res.y = Discretize(value.y,resolution.y,variance.y);
  res.z = Discretize(value.z,resolution.z,variance.z);
  return res;
}


bool WriteFile(File& f,const string& s)
{
  size_t n=s.length();
  if(!WriteFile(f,n)) return false;
  if(n > 0)
    if(!WriteArrayFile(f,&s[0],s.length())) return false;
  return true;
}

bool ReadFile(File& f,string& s)
{
  size_t n;
  if(!ReadFile(f,n)) return false;
  s.resize(n);
  if(n > 0)
    if(!ReadArrayFile(f,&s[0],n)) return false;
  return true;
}

template <class T>
bool WriteFile(File& f,const vector<T>& v)
{
  if(!WriteFile(f,(int)v.size())) return false;
  if(!v.empty())
    if(!WriteArrayFile(f,&v[0],v.size())) return false;
  return true;
}

template <class T>
bool ReadFile(File& f,vector<T>& v)
{
  int n;
  if(!ReadFile(f,n)) return false;
  v.resize(0);
  if(n > 0) {
    v.resize(n);
    if(!ReadArrayFile(f,&v[0],n)) return false;
    return true;
  }
  return false;
}


SensorBase::SensorBase()
  :name("Unnamed sensor"),rate(0)
{}

bool SensorBase::ReadState(File& f)
{
  vector<double> values;
  if(!ReadFile(f,values)) return false;
  SetMeasurements(values);
  vector<double> state;
  if(!ReadFile(f,state)) return false;
  SetState(state);
  size_t n;
  if(!ReadFile(f,n)) return false;
  for(size_t i=0;i<n;i++) {
    string key,value;
    if(!ReadFile(f,key)) return false;
    if(!ReadFile(f,value)) return false;
    SetSetting(key,value);
  }
  return true;
}

bool SensorBase::WriteState(File& f) const
{
  vector<double> values;
  GetMeasurements(values);
  if(!WriteFile(f,values)) return false;
  vector<double> state;
  GetState(state);
  if(!WriteFile(f,state)) return false;
  map<string,string> settings;
  size_t n=settings.size();
  if(!WriteFile(f,n)) return false;
  for(map<string,string>::const_iterator i=settings.begin();i!=settings.end();i++) {
    if(!WriteFile(f,i->first)) return false;
    if(!WriteFile(f,i->second)) return false;
  }
  return true;
}

map<string,string> SensorBase::Settings() const
{
  map<string,string> settings;
  FILL_SENSOR_SETTING(settings,rate);
  return settings;
}
bool SensorBase::GetSetting(const string& name,string& str) const
{
  GET_SENSOR_SETTING(rate);
  return false;
}

bool SensorBase::SetSetting(const string& name,const string& str)
{
  SET_SENSOR_SETTING(rate);
  return false;
}

JointPositionSensor::JointPositionSensor()
{}

void JointPositionSensor::Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim)
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

void JointVelocitySensor::Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim)
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
    Vector dqread(dq.n,Zero);
    for(size_t i=0;i<indices.size();i++)
      dqread(indices[i]) = dq(indices[i]);
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

void DriverTorqueSensor::Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim)
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
    for(size_t i=0;i<indices.size();i++)
      tread(indices[i]) = t(indices[i]);
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


ContactSensor::ContactSensor()
  :link(0),patchMin(Zero),patchMax(Zero),patchTolerance(0),fVariance(Zero),contact(false),force(Zero)
{
  Tsensor.setIdentity();
  hasForce[0] = hasForce[1] = hasForce[2] = false;
}

void ContactSensor::Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim)
{
  RigidTransform Tlink;
  robot->oderobot->GetLinkTransform(link,Tlink);
  RigidTransform TsensorWorld  = Tlink*Tsensor;
  contact = false;
  force.setZero();
  dBodyID body = robot->oderobot->body(link);
  if(!body || !HasContact(body)) {
    return;
  }
  //look through contacts
  vector<ODEContactList> contacts;
  GetContacts(body,contacts);
  Vector3 xlocal,flocal;
  for(size_t i=0;i<contacts.size();i++) {
    for(size_t j=0;j<contacts[i].points.size();j++) {
      TsensorWorld.mulInverse(contacts[i].points[j].x,xlocal);
      if(patchMin.x <= xlocal.x && xlocal.x <= patchMax.x &&
	 patchMin.y <= xlocal.y && xlocal.y <= patchMax.y &&
	 Abs(xlocal.z) <= patchTolerance) {
	//contact!
	contact = true;
	Tsensor.R.mulTranspose(contacts[i].forces[j],flocal);
	force += flocal;
      }
    }
  }
}

void ContactSensor::Reset()
{
  contact = false;
  force.setZero();
}

void ContactSensor::MeasurementNames(std::vector<std::string>& names) const
{
  names.resize(1);
  names[0] = "contact";
  if(hasForce[0]) names.push_back("force_x");
  if(hasForce[1]) names.push_back("force_y");
  if(hasForce[2]) names.push_back("force_z");
}

void ContactSensor::GetMeasurements(std::vector<double>& values) const
{
  values.resize(1);
  values[0] = contact;
  if(hasForce[0]) values.push_back(force.x);
  if(hasForce[1]) values.push_back(force.y);
  if(hasForce[2]) values.push_back(force.z);
}

void ContactSensor::SetMeasurements(const std::vector<double>& values)
{
  Assert(values.size() >= 1);
  contact = (values[0] != 0.0);
  int i=1;
  if(hasForce[0]) { force.x = values[i]; i++; }
  if(hasForce[1]) { force.y = values[i]; i++; }
  if(hasForce[2]) { force.z = values[i]; i++; }
  Assert(i == (int)values.size());
}

map<string,string> ContactSensor::Settings() const
{
  map<string,string> settings = SensorBase::Settings();
  FILL_SENSOR_SETTING(settings,link);
  FILL_SENSOR_SETTING(settings,Tsensor);
  FILL_SENSOR_SETTING(settings,patchMin);
  FILL_SENSOR_SETTING(settings,patchMax);
  FILL_SENSOR_SETTING(settings,patchTolerance);
  FILL_ARRAY_SENSOR_SETTING(settings,hasForce,3);
  FILL_SENSOR_SETTING(settings,fVariance);
  return settings;
}

bool ContactSensor::GetSetting(const string& name,string& str) const
{
  if(SensorBase::GetSetting(name,str)) return true;
  GET_SENSOR_SETTING(link);
  GET_SENSOR_SETTING(Tsensor);
  GET_SENSOR_SETTING(patchMin);
  GET_SENSOR_SETTING(patchMax);
  GET_SENSOR_SETTING(patchTolerance);
  GET_ARRAY_SENSOR_SETTING(hasForce,3);
  GET_SENSOR_SETTING(fVariance);
  return false;
}

bool ContactSensor::SetSetting(const string& name,const string& str)
{
  if(SensorBase::SetSetting(name,str)) return true;
  SET_SENSOR_SETTING(link);
  SET_SENSOR_SETTING(Tsensor);
  SET_SENSOR_SETTING(patchMin);
  SET_SENSOR_SETTING(patchMax);
  SET_SENSOR_SETTING(patchTolerance);
  SET_ARRAY_SENSOR_SETTING(hasForce,3);
  SET_SENSOR_SETTING(fVariance);
  return false;
}

ForceTorqueSensor::ForceTorqueSensor()
  :link(0),localPos(Zero),fVariance(Zero),mVariance(Zero),f(Zero),m(Zero)
{
  hasForce[0] = hasForce[1] = hasForce[2] = false;
  hasMoment[0] = hasMoment[1] = hasMoment[2] = false;
}

void ForceTorqueSensor::Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim) 
{
  dJointFeedback fb = robot->oderobot->feedback(link);
  Vector3 fw(fb.f1[0],fb.f1[1],fb.f1[2]);
  RigidTransform T;
  robot->oderobot->GetLinkTransform(link,T);
  Vector3 mcomw = Vector3(fb.t1[0],fb.t1[1],fb.t1[2]);
  //convert moment about link's com to moment about localpos
  //mp_w = (p-com) x f_w + mcom_w 
  Vector3 comw = T*robot->robot->links[link].com;
  Vector3 pw = T*localPos;
  //TEMP: is the moment measured about the joint axis?
  //Vector3 mw = cross(comw-pw,fw) + mcomw;
  Vector3 mw = mcomw;
  //convert to local frame
  T.R.mulTranspose(fw,f);
  T.R.mulTranspose(mw,m);

  f = Discretize(f,Vector3(0.0),fVariance);
  m = Discretize(m,Vector3(0.0),mVariance);
  for(int i=0;i<3;i++)
    if(!hasForce[i]) f[i] = 0;
  for(int i=0;i<3;i++)
    if(!hasMoment[i]) m[i] = 0;
}

void ForceTorqueSensor::Reset()
{
  f.setZero();
  m.setZero();
}

void ForceTorqueSensor::MeasurementNames(vector<string>& names) const
{
  names.resize(6);
  names[0] = "force_x";
  names[1] = "force_y";
  names[2] = "force_z";
  names[3] = "moment_x";
  names[4] = "moment_y";
  names[5] = "moment_z";
}

void ForceTorqueSensor::GetMeasurements(vector<double>& values) const
{
  values.resize(6);
  f.get(values[0],values[1],values[2]);
  m.get(values[3],values[4],values[5]);
}

void ForceTorqueSensor::SetMeasurements(const vector<double>& values)
{
  Assert(values.size()==6);
  f.set(values[0],values[1],values[2]);
  m.set(values[3],values[4],values[5]);
}

map<string,string> ForceTorqueSensor::Settings() const
{
  map<string,string> settings = SensorBase::Settings();
  FILL_SENSOR_SETTING(settings,link);
  FILL_SENSOR_SETTING(settings,localPos);
  FILL_ARRAY_SENSOR_SETTING(settings,hasForce,3);
  FILL_ARRAY_SENSOR_SETTING(settings,hasMoment,3);
  FILL_SENSOR_SETTING(settings,fVariance);
  FILL_SENSOR_SETTING(settings,mVariance);
  return settings;
}

bool ForceTorqueSensor::GetSetting(const string& name,string& str) const
{
  if(SensorBase::GetSetting(name,str)) return true;
  GET_SENSOR_SETTING(link);
  GET_SENSOR_SETTING(localPos);
  GET_ARRAY_SENSOR_SETTING(hasForce,3);
  GET_ARRAY_SENSOR_SETTING(hasMoment,3);
  GET_SENSOR_SETTING(fVariance);
  GET_SENSOR_SETTING(mVariance);  
  return false;
}

bool ForceTorqueSensor::SetSetting(const string& name,const string& str)
{
  if(SensorBase::SetSetting(name,str)) return true;
  SET_SENSOR_SETTING(link);
  SET_SENSOR_SETTING(localPos);
  SET_ARRAY_SENSOR_SETTING(hasForce,3);
  SET_ARRAY_SENSOR_SETTING(hasMoment,3);
  SET_SENSOR_SETTING(fVariance);
  SET_SENSOR_SETTING(mVariance);  
  return false;
}

Accelerometer::Accelerometer()
  :link(0),accelVariance(Zero),accel(Zero)
{
  Tsensor.setIdentity();
  hasAxis[0] = hasAxis[1] = hasAxis[2] = false;
  last_dt = 0;
  last_v.setZero();
}

void Accelerometer::Advance(Real dt)
{
  last_dt = dt;
}

void Accelerometer::Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim)
{
  RigidTransform T;
  Vector3 w,v,vp;
  robot->oderobot->GetLinkTransform(link,T);
  robot->oderobot->GetLinkVelocity(link,w,v);
  vp = v+cross(w,T.R*Tsensor.t);
  if(last_dt==0) {
    accel.setZero();
  }
  else {
    accel = (vp - last_v)/last_dt;
  }
  last_v = vp;

  accel += Vector3(0,0,-9.8);
  accel.x += RandGaussian()*Sqrt(accelVariance.x);
  accel.y += RandGaussian()*Sqrt(accelVariance.y);
  accel.z += RandGaussian()*Sqrt(accelVariance.z);

  Vector3 accelw = accel;
  T.R.mulTranspose(accelw,accel);

  accel = Discretize(accel,Vector3(Zero),accelVariance);
  for(int i=0;i<3;i++)
    if(!hasAxis[i]) accel[i] = 0;
}

void Accelerometer::Reset()
{
  accel.setZero();
  last_dt = 0;
  last_v.setZero();
}

void Accelerometer::MeasurementNames(vector<string>& names) const
{
  names.resize(3);
  names[0] = "accel_x";
  names[1] = "accel_y";
  names[2] = "accel_z";
}

void Accelerometer::GetMeasurements(vector<double>& values) const
{
  values.resize(3);
  accel.get(values[0],values[1],values[2]);
}

void Accelerometer::SetMeasurements(const vector<double>& values)
{
  Assert(values.size()==3);
  accel.set(values[0],values[1],values[2]);
}

void Accelerometer::GetState(vector<double>& state) const
{
  state.resize(4);
  state[0] = last_dt;
  last_v.get(state[1],state[2],state[3]);
}

void Accelerometer::SetState(const vector<double>& state)
{
  Assert(state.size()==4);
  last_dt = state[0];
  last_v.set(state[1],state[2],state[3]);
}

map<string,string> Accelerometer::Settings() const
{
  map<string,string> settings = SensorBase::Settings();
  FILL_SENSOR_SETTING(settings,link);
  FILL_SENSOR_SETTING(settings,Tsensor);
  FILL_ARRAY_SENSOR_SETTING(settings,hasAxis,3);
  FILL_SENSOR_SETTING(settings,accelVariance);
  return settings;
}

bool Accelerometer::GetSetting(const string& name,string& str) const
{
  if(SensorBase::GetSetting(name,str)) return true;
  GET_SENSOR_SETTING(link);
  GET_SENSOR_SETTING(Tsensor);
  GET_ARRAY_SENSOR_SETTING(hasAxis,3);
  GET_SENSOR_SETTING(accelVariance);
  return false;
}

bool Accelerometer::SetSetting(const string& name,const string& str)
{
  if(SensorBase::SetSetting(name,str)) return true;
  SET_SENSOR_SETTING(link);
  SET_SENSOR_SETTING(Tsensor);
  SET_ARRAY_SENSOR_SETTING(hasAxis,3);
  SET_SENSOR_SETTING(accelVariance);
  return false;
}
 
TiltSensor::TiltSensor()
  :link(0),resolution(Zero),variance(Zero),hasVelocity(false)
{
  referenceDir.set(0,0,1);
  Rsensor.setIdentity();
  hasAxis[0] = hasAxis[1] = true;
  hasAxis[2] = false;
}

void TiltSensor::Advance(Real dt)
{
}

void TiltSensor::Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim)
{
  RigidTransform T;
  Vector3 w,v;
  robot->oderobot->GetLinkTransform(link,T);
  robot->oderobot->GetLinkVelocity(link,w,v);

  Vector3 dlocal,dsensor;
  T.R.mulTranspose(referenceDir,dlocal);
  Rsensor.mulTranspose(dlocal,dsensor);
  //x value = sin alpha
  //assume reference direction is z
  alocal.x = Asin(dsensor[1]);
  alocal.y = Asin(-dsensor[0]);
  alocal.z = Asin(dsensor[2]);

  //compute local angular velocity value
  wlocal = w;
  T.R.mulTranspose(w,wlocal);
  Rsensor.mulTranspose(wlocal,w);
  wlocal = w;

  alocal = Discretize(alocal,resolution,variance);
  wlocal = Discretize(wlocal,resolution,variance);

  for(int i=0;i<3;i++)
    if(!hasAxis[i]) alocal[i] = 0;
  for(int i=0;i<3;i++)
    if(!hasAxis[i]) wlocal[i] = 0;
}

void TiltSensor::Reset()
{
}

void TiltSensor::MeasurementNames(vector<string>& names) const
{
  names.resize(6);
  names[0] = "a_x";
  names[1] = "a_y";
  names[2] = "a_z";
  names[3] = "w_x";
  names[4] = "w_y";
  names[5] = "w_z";
}

void TiltSensor::GetMeasurements(vector<double>& values) const
{
  values.resize(6);
  values[0] = alocal.x;
  values[1] = alocal.y; 
  values[2] = alocal.z; 
  values[3] = wlocal.x;
  values[4] = wlocal.y;
  values[5] = wlocal.z;
}

void TiltSensor::SetMeasurements(const vector<double>& values)
{
  Assert(values.size()==6);
  alocal.set(values[0],values[1],values[2]);
  wlocal.set(values[3],values[4],values[5]);
}

map<string,string> TiltSensor::Settings() const
{
  map<string,string> settings=Settings();
  FILL_SENSOR_SETTING(settings,link);
  FILL_SENSOR_SETTING(settings,referenceDir);
  FILL_SENSOR_SETTING(settings,Rsensor);
  FILL_ARRAY_SENSOR_SETTING(settings,hasAxis,3);
  FILL_SENSOR_SETTING(settings,hasVelocity);
  FILL_SENSOR_SETTING(settings,resolution);
  FILL_SENSOR_SETTING(settings,variance);
  return settings;
}

bool TiltSensor::GetSetting(const string& name,string& str) const
{
  if(SensorBase::GetSetting(name,str)) return true;
  GET_SENSOR_SETTING(link);
  GET_SENSOR_SETTING(referenceDir);
  GET_SENSOR_SETTING(Rsensor);
  GET_ARRAY_SENSOR_SETTING(hasAxis,3);
  GET_SENSOR_SETTING(hasVelocity);
  GET_SENSOR_SETTING(resolution);
  GET_SENSOR_SETTING(variance);
  return false;
}

bool TiltSensor::SetSetting(const string& name,const string& str)
{
  if(SensorBase::SetSetting(name,str)) return true;
  SET_SENSOR_SETTING(link);
  SET_SENSOR_SETTING(referenceDir);
  SET_SENSOR_SETTING(Rsensor);
  SET_ARRAY_SENSOR_SETTING(hasAxis,3);
  SET_SENSOR_SETTING(hasVelocity);
  SET_SENSOR_SETTING(resolution);
  SET_SENSOR_SETTING(variance);
  return false;
}

GyroSensor::GyroSensor()
  :link(0),hasAngAccel(0),hasAngVel(0),hasRotation(0),angAccel(Zero),angVel(Zero)
{
  rotation.setIdentity();
  last_dt = 0;
  last_w.setZero();
}

void GyroSensor::Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim)
{
  RigidTransform T;
  Vector3 w,v;
  robot->oderobot->GetLinkTransform(link,T);
  robot->oderobot->GetLinkVelocity(link,w,v);
  if(hasAngAccel) {
    if(last_dt == 0) 
      angAccel.setZero();
    else {
      angAccel = (w-last_w)/last_dt;
    }
    last_w = w;
    angAccel.x += RandGaussian()*Sqrt(angAccelVariance(0,0));
    angAccel.y += RandGaussian()*Sqrt(angAccelVariance(1,1));
    angAccel.z += RandGaussian()*Sqrt(angAccelVariance(2,2));
  }
  if(hasAngVel) {
    angVel = w;
    angVel.x += RandGaussian()*Sqrt(angVelVariance(0,0));
    angVel.y += RandGaussian()*Sqrt(angVelVariance(1,1));
    angVel.z += RandGaussian()*Sqrt(angVelVariance(2,2));
  }
  if(hasRotation) {
    rotation = T.R;
    //TODO: variance
  }
}

void GyroSensor::Advance(Real dt)
{
  last_dt = dt;
}

void GyroSensor::Reset()
{
  last_dt = 0;
  last_w.setZero();
  angAccel.setZero();
  angVel.setZero();
  rotation.setIdentity();
}


void GyroSensor::MeasurementNames(vector<string>& names) const
{
  names.resize(15);
  names[0] = "R_xx";
  names[1] = "R_xy";
  names[2] = "R_xz";
  names[3] = "R_yx";
  names[4] = "R_yy";
  names[5] = "R_yz";
  names[6] = "R_zx";
  names[7] = "R_zy";
  names[8] = "R_zz";
  names[9] = "angVel_x";
  names[10] = "angVel_y";
  names[11] = "angVel_z";
  names[12] = "angAccel_x";
  names[13] = "angAccel_y";
  names[14] = "angAccel_z";
}

void GyroSensor::GetMeasurements(vector<double>& values) const
{
  values.resize(15);
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      values[i*3+j] = rotation(i,j);
  angVel.get(values[9],values[10],values[11]);
  angAccel.get(values[12],values[13],values[14]);
}

void GyroSensor::SetMeasurements(const vector<double>& values)
{
  Assert(values.size()==15);
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      rotation(i,j) = values[i*3+j];
  angVel.set(values[9],values[10],values[11]);
  angAccel.set(values[12],values[13],values[14]);
}

void GyroSensor::GetState(vector<double>& state) const
{
  state.resize(0);
  state.push_back(last_dt);
  state.push_back(last_w.x);
  state.push_back(last_w.y);
  state.push_back(last_w.z);
}

void GyroSensor::SetState(const vector<double>& state)
{
  Assert(state.size()==4);
  last_dt = state[0];
  last_w.set(state[1],state[2],state[3]);
}

map<string,string> GyroSensor::Settings() const
{
  map<string,string> settings = SensorBase::Settings();
  FILL_SENSOR_SETTING(settings,link);
  FILL_SENSOR_SETTING(settings,angAccelVariance);
  FILL_SENSOR_SETTING(settings,angVelVariance);
  FILL_SENSOR_SETTING(settings,rotationVariance);
  FILL_SENSOR_SETTING(settings,hasAngAccel);
  FILL_SENSOR_SETTING(settings,hasAngVel);
  FILL_SENSOR_SETTING(settings,hasRotation);
  return settings;
}

bool GyroSensor::GetSetting(const string& name,string& str) const
{
  if(SensorBase::GetSetting(name,str)) return true;
  GET_SENSOR_SETTING(link);
  GET_SENSOR_SETTING(angAccelVariance);
  GET_SENSOR_SETTING(angVelVariance);
  GET_SENSOR_SETTING(rotationVariance);
  GET_SENSOR_SETTING(hasAngAccel);
  GET_SENSOR_SETTING(hasAngVel);
  GET_SENSOR_SETTING(hasRotation);
  return false;
}

bool GyroSensor::SetSetting(const string& name,const string& str)
{
  if(SensorBase::SetSetting(name,str)) return true;
  SET_SENSOR_SETTING(link);
  SET_SENSOR_SETTING(angAccelVariance);
  SET_SENSOR_SETTING(angVelVariance);
  SET_SENSOR_SETTING(rotationVariance);
  SET_SENSOR_SETTING(hasAngAccel);
  SET_SENSOR_SETTING(hasAngVel);
  SET_SENSOR_SETTING(hasRotation);
  return false;
}

IMUSensor::IMUSensor()
{
  Reset();
}

void IMUSensor::Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim)
{
  accelerometer.Simulate(robot,sim);
  accel = accelerometer.accel;
  //translate to global frame and remove gravity from acceleration reading
  RigidTransform T;
  robot->oderobot->GetLinkTransform(accelerometer.link,T);
  accel = T.R*accel;
  accel += Vector3(0,0,9.8);
  //integrate velocity and position
  translation.madd(velocity,accelerometer.last_dt);
  translation.madd(accel,0.5*Sqr(accelerometer.last_dt));
  velocity.madd(accel,accelerometer.last_dt);

  gyro.Simulate(robot,sim);
  if(gyro.hasAngAccel) angAccel = gyro.angAccel;
  if(gyro.hasAngVel) {
    angVel = gyro.angAccel;
  }
  else {
    angVel.madd(angAccel,gyro.last_dt);
  }
  if(gyro.hasRotation) {
    rotation = gyro.rotation;
  }
  else {
    MomentRotation m(angVel*gyro.last_dt);
    Matrix3 R;
    m.getMatrix(R);
    rotation = rotation*R;
  }
}

void IMUSensor::Advance(Real dt)
{
  accelerometer.Advance(dt);
  gyro.Advance(dt);
}

void IMUSensor::Reset()
{
  accel.setZero();
  velocity.setZero();
  translation.setZero();
  angAccel.setZero();
  angVel.setZero();
  rotation.setIdentity();
  accelerometer.Reset();
  gyro.Reset();
}

void IMUSensor::MeasurementNames(vector<string>& names) const
{
  names.resize(9);
  names[0] = "accel_x";
  names[1] = "accel_y";
  names[2] = "accel_z";
  names[3] = "vel_x";
  names[4] = "vel_y";
  names[5] = "vel_z";
  names[6] = "trans_x";
  names[7] = "trans_y";
  names[8] = "trans_z";

  size_t ofs = names.size();
  names.resize(names.size()+15);
  names[ofs+0] = "angAccel_x";
  names[ofs+1] = "angAccel_y";
  names[ofs+2] = "angAccel_z";
  names[ofs+3] = "angVel_x";
  names[ofs+4] = "angVel_y";
  names[ofs+5] = "angVel_z";
  names[ofs+6] = "R_xx";
  names[ofs+7] = "R_xy";
  names[ofs+8] = "R_xz";
  names[ofs+9] = "R_yx";
  names[ofs+10] = "R_yy";
  names[ofs+11] = "R_yz";
  names[ofs+12] = "R_zx";
  names[ofs+13] = "R_zy";
  names[ofs+14] = "R_zz";
}

void IMUSensor::GetMeasurements(vector<double>& values) const
{
  values.resize(24);
  accel.get(values[0],values[1],values[2]);
  velocity.get(values[3],values[4],values[5]);
  translation.get(values[6],values[7],values[8]);
  angAccel.get(values[9],values[10],values[11]);
  angVel.get(values[12],values[13],values[14]);
  values[15] = rotation(0,0);
  values[16] = rotation(0,1);
  values[17] = rotation(0,2);
  values[18] = rotation(1,0);
  values[19] = rotation(1,1);
  values[20] = rotation(1,2);
  values[21] = rotation(2,0);
  values[22] = rotation(2,1);
  values[23] = rotation(2,2);
}

void IMUSensor::SetMeasurements(const vector<double>& values)
{
  Assert(values.size()==24);
  accel.set(values[0],values[1],values[2]);
  velocity.set(values[3],values[4],values[5]);
  translation.set(values[6],values[7],values[8]);
  angAccel.set(values[9],values[10],values[11]);
  angVel.set(values[12],values[13],values[14]);
  rotation(0,0) = values[15];
  rotation(0,1) = values[16];
  rotation(0,2) = values[17];
  rotation(1,0) = values[18];
  rotation(1,1) = values[19];
  rotation(1,2) = values[20];
  rotation(2,0) = values[21];
  rotation(2,1) = values[22];
  rotation(2,2) = values[23];
}

void IMUSensor::GetState(vector<double>& state) const
{
  vector<double> astate,gstate;
  accelerometer.GetState(astate);
  gyro.GetState(gstate);
  state.resize(0);
  state.insert(state.end(),astate.begin(),astate.end());
  state.insert(state.end(),gstate.begin(),gstate.end());
}

void IMUSensor::SetState(const vector<double>& state)
{
  Assert(state.size()==8);
  vector<double> astate,gstate;
  astate.resize(4);
  gstate.resize(4);
  copy(state.begin(),state.begin()+4,astate.begin());
  copy(state.begin()+4,state.end()+4,gstate.begin());
  accelerometer.SetState(astate);
  gyro.SetState(gstate);
}

map<string,string> IMUSensor::Settings() const
{
  map<string,string> s0 = SensorBase::Settings(),s1,s2;
  s1=accelerometer.Settings();
  s2=gyro.Settings();
  for(map<string,string>::iterator i=s1.begin();i!=s1.end();i++)
    s0[i->first] = i->second;
  for(map<string,string>::iterator i=s2.begin();i!=s2.end();i++)
    s0[i->first] = i->second;
  return s0;
}

bool IMUSensor::GetSetting(const string& name,string& str) const
{
  if(SensorBase::GetSetting(name,str)) return true;
  if(accelerometer.GetSetting(name,str)) return true;
  if(gyro.GetSetting(name,str)) return true;
  return false;
}

bool IMUSensor::SetSetting(const string& name,const string& str)
{
  if(SensorBase::SetSetting(name,str)) return true;
  if(accelerometer.SetSetting(name,str)) return true;
  if(gyro.SetSetting(name,str)) return true;
  return false;
}

FilteredSensor::FilteredSensor()
  :smoothing(0)
{}

void FilteredSensor::Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim)
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

void FilteredSensor::GetState(vector<double>& values) const
{
  if(sensor) {
    sensor->GetState(values);
  }
}
void FilteredSensor::SetState(const vector<double>& state)
{
  if(sensor) sensor->SetState(state);
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


TimeDelayedSensor::TimeDelayedSensor()
  :delay(0),jitter(0)
{}

void TimeDelayedSensor::Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim)
{
  if(!sensor) return;
  sensor->Simulate(robot,sim);
  vector<double> newMeasurements;
  sensor->GetMeasurements(newMeasurements);
  double delivTime = robot->curTime + delay + Rand(-jitter,jitter);
  measurementsInTransit.push_back(newMeasurements);
  deliveryTimes.push_back(delivTime);

  while(!deliveryTimes.empty() && deliveryTimes.front() <= robot->curTime) {
    swap(arrivedMeasurement,measurementsInTransit.front());
    measurementsInTransit.pop_front();
    deliveryTimes.pop_front();
  }
}

void TimeDelayedSensor::Advance(Real dt)
{
  if(!sensor) return;
  sensor->Advance(dt);
}

void TimeDelayedSensor::Reset()
{
  if(sensor) sensor->Reset();
  measurementsInTransit.clear();
  deliveryTimes.clear();
  arrivedMeasurement.clear();
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

void TimeDelayedSensor::GetState(vector<double>& state) const
{
  if(!sensor) return;
  vector<double> sstate;
  sensor->GetState(sstate);
  size_t n = 0;
  if(!measurementsInTransit.empty()) n = measurementsInTransit.front().size();
  state = sstate;
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

void TimeDelayedSensor::SetState(const vector<double>& state)
{
  if(!sensor) return;
  //just to get the size
  vector<double> sstate;
  sensor->GetState(sstate);
  copy(state.begin(),state.begin()+sstate.size(),sstate.begin());
  sensor->SetState(sstate);
  vector<double>::const_iterator readpos = state.begin()+sstate.size();
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

bool RobotSensors::LoadSettings(const char* fn)
{
  TiXmlDocument doc;
  if(!doc.LoadFile(fn)) return false;
  return LoadSettings(doc.RootElement());
}

bool RobotSensors::SaveSettings(const char* fn)
{
  TiXmlDocument doc;
  SaveSettings(doc.RootElement());
  return doc.SaveFile(fn);
}

bool RobotSensors::LoadSettings(TiXmlElement* node)
{
  if(0!=strcmp(node->Value(),"sensors")) return false;
  TiXmlElement* e=node->FirstChildElement();
  sensors.resize(0);
  while(e != NULL) {
    SensorBase* sensor = NULL;
    set<string> processedAttributes;
    if(0==strcmp(e->Value(),"JointPositionSensor")) {
      sensor = new JointPositionSensor;
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"JointVelocitySensor")) {
      sensor = new JointVelocitySensor;
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"DriverTorqueSensor")) {
      sensor = new DriverTorqueSensor;
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"GyroSensor")) {
      sensor = new GyroSensor;
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"Accelerometer")) {
      sensor = new Accelerometer;
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"TiltSensor")) {
      sensor = new TiltSensor;
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"IMUSensor")) {
      sensor = new IMUSensor;
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"ContactSensor")) {
      sensor = new ContactSensor;
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"ForceTorqueSensor")) {
      sensor = new ForceTorqueSensor;
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"FilteredSensor")) {
      FilteredSensor* fs = new FilteredSensor;
      if(!e->Attribute("sensor")) {
	fprintf(stderr,"Filtered sensor doesn't have a \"sensor\" attribute\n");
	return false;
      }
      fs->sensor = GetNamedSensor(e->Attribute("sensor"));
      if(fs->sensor == NULL) {
	fprintf(stderr,"Filtered sensor has unknown sensor type \"%s\"\n",e->Attribute("sensor"));
	return false;
      }
      sensor = fs;
      sensors.push_back(sensor);
      processedAttributes.insert("sensor");
    }
    else if(0==strcmp(e->Value(),"TimeDelayedSensor")) {
      TimeDelayedSensor* fs = new TimeDelayedSensor;
      if(!e->Attribute("sensor")) {
	fprintf(stderr,"Time-delayed sensor doesn't have a \"sensor\" attribute\n");
	return false;
      }
      fs->sensor = GetNamedSensor(e->Attribute("sensor"));
      if(fs->sensor == NULL) {
	fprintf(stderr,"Time-delayed sensor has unknown sensor type \"%s\"\n",e->Attribute("sensor"));
	return false;
      }
      sensor = fs;
      sensors.push_back(sensor);
      processedAttributes.insert("sensor");
    }
    else {
      printf("Unknown sensor type %s\n",e->Value());
      return false;
    }
    TiXmlAttribute* attr = e->FirstAttribute();
    while(attr != NULL) {
      if(processedAttributes.count(attr->Name())!=0) {
	//pass
      }
      else if(0==strcmp(attr->Name(),"name")) {
	sensor->name = attr->Value();
      }
      else if(!sensor->SetSetting(attr->Name(),attr->Value())) {
	fprintf(stderr,"Error setting sensor %s attribute %s, doesn't exist?\n",e->Value(),attr->Name());
	map<string,string> s = sensor->Settings();
	fprintf(stderr,"Candidates:\n");
	for(map<string,string>::const_iterator i=s.begin();i!=s.end();i++)
	  fprintf(stderr,"  %s : %s by default\n",i->first.c_str(),i->second.c_str());
	return false;
      }
      attr = attr->Next();
    }
    e = e->NextSiblingElement();
  }
  return true;
}

void RobotSensors::SaveSettings(TiXmlElement* node)
{
  node->SetValue("sensors");
  for(size_t i=0;i<sensors.size();i++) {
    TiXmlElement c(sensors[i]->Type());
    node->SetAttribute("name",sensors[i]->name.c_str());
    PropertyMap settings = sensors[i]->Settings();
    settings.Save(&c);
    node->InsertEndChild(c);
  }
}

bool RobotSensors::LoadMeasurements(TiXmlElement* node)
{
  if(0!=strcmp(node->Value(),"measurement")) return false;
  TiXmlElement* e=node->FirstChildElement();
  while(e != NULL) {
    SensorBase* s = GetNamedSensor(e->Value());
    if(!s) {
      fprintf(stderr,"No sensor named %s\n",e->Value());
      return false;
    }
    vector<string> measurementNames;
    s->MeasurementNames(measurementNames);
    vector<double> measurementValues(measurementNames.size(),0);
    PropertyMap attrs;
    attrs.Load(e);
    for(size_t i=0;i<measurementNames.size();i++) {
      if(attrs.count(measurementNames[i]) == 0) {
	fprintf(stderr,"No measurement of %s of name %s\n",e->Value(),measurementNames[i].c_str());
	return false;
      }
      stringstream ss(attrs[measurementNames[i]]);
      ss >> measurementValues[i];
    }
    s->SetState(measurementValues);

    e = e->NextSiblingElement();
  }
  return true;
}

void RobotSensors::SaveMeasurements(TiXmlElement* node)
{
  node->SetValue("measurement");
  for(size_t i=0;i<sensors.size();i++) {
    TiXmlElement c(sensors[i]->name.c_str());
    vector<string> measurementNames;
    sensors[i]->MeasurementNames(measurementNames);
    vector<double> measurementValues(measurementNames.size());
    sensors[i]->GetMeasurements(measurementValues);
    for(size_t i=0;i<measurementNames.size();i++) {
      stringstream ss; ss << measurementValues[i];
      c.SetAttribute(measurementNames[i].c_str(),ss.str().c_str());
    }
    node->InsertEndChild(c);
  }
}

bool RobotSensors::ReadState(File& f)
{
  for(size_t i=0;i<sensors.size();i++)
    if(!sensors[i]->ReadState(f)) return false;
  return true;
}

bool RobotSensors::WriteState(File& f) const
{
  for(size_t i=0;i<sensors.size();i++)
    if(!sensors[i]->WriteState(f)) return false;
  return true;
}

SmartPointer<SensorBase> RobotSensors::GetNamedSensor(const string& name)
{
  for(size_t i=0;i<sensors.size();i++)
    if(name == sensors[i]->name) return sensors[i];
  return NULL;
}



void RobotSensors::MakeDefault(Robot* robot)
{
  sensors.resize(0);
  string sensorFn;
  if(robot->properties.get("sensors",sensorFn)) {
    if(LoadSettings(sensorFn.c_str())) {
      //be sure to resize any empty JointPositionSensor's and 
      //JointVelocitySensor's 
      vector<JointPositionSensor*> jps;
      GetTypedSensors(jps);
      for(size_t i=0;i<jps.size();i++)
	if(jps[i]->indices.empty())
	  jps[i]->q.resize(robot->q.n,Zero);
      vector<JointVelocitySensor*> jvs;
      GetTypedSensors(jvs);
      for(size_t i=0;i<jvs.size();i++)
	if(jvs[i]->indices.empty())
	  jvs[i]->dq.resize(robot->q.n,Zero);
      return;
    }
    else {
      printf("RobotSensors::MakeDefault: could not load sensor file %s\n",sensorFn.c_str());
      printf("  Making the standard sensors instead.\n");
      printf("  Press enter to continue.\n");
      getchar();
      sensors.resize(0);
    }
  }
  JointPositionSensor* jp = new JointPositionSensor;
  JointVelocitySensor* jv = new JointVelocitySensor;
  jp->name = "q";
  jv->name = "dq";
  jp->q.resize(robot->q.n,Zero);
  jv->dq.resize(robot->q.n,Zero);
  sensors.push_back(jp);
  sensors.push_back(jv);
}
