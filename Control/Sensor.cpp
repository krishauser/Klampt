#include "Sensor.h"
#include "JointSensors.h"
#include "ForceSensors.h"
#include "InertialSensors.h"
#include "VisualSensors.h"
#include "OtherSensors.h"
#include "Simulation/ControlledSimulator.h"
#include "Simulation/ODESimulator.h"
#include "Simulation/WorldSimulation.h"
#include <KrisLibrary/utils/PropertyMap.h>
#include <KrisLibrary/math/random.h>
#if HAVE_GLEW
#include <GL/glew.h>
#endif //HAVE_GLEW
//TEST: fallback with no opengl
//#undef HAVE_GLEW
//#define HAVE_GLEW 0

#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/GLView.h>
#include <KrisLibrary/GLdraw/GLError.h>
#include "View/ViewWrench.h"
#include "View/ViewCamera.h"
#include <tinyxml.h>
#include <sstream>

using namespace GLDraw;

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
  SetInternalState(state);
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
  GetInternalState(state);
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
  :link(0),patchMin(Zero),patchMax(Zero),patchTolerance(0.001),
  fResolution(Zero),fVariance(Zero),fSensitivity(Zero),fSaturation(Inf),
  falloffCoefficient(0),
  contact(false),force(Zero)
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
	Tsensor.R.mulTranspose(contacts[i].forces[j],flocal);
  if(falloffCoefficient > 0) {
    Vector2 patchCenter = 0.5*(patchMin+patchMax);
    Real weight = 1.0-Abs(4.0*(xlocal.x - patchCenter.x)*(xlocal.y - patchCenter.y))/((patchMax.x - patchMin.x)*(patchMax.y - patchMin.y));
    weight = Pow(weight,falloffCoefficient);
    force.madd(flocal,weight);
  }
  else
    force += flocal;
	
      }
    }
  }

  if(Abs(force.z) > fSensitivity)
    contact = true;
  force.x = Discretize(force.x,fResolution.x,fVariance.x);
  force.y = Discretize(force.y,fResolution.y,fVariance.y);
  force.z = Discretize(force.z,fResolution.z,fVariance.z);
  if(Abs(force.x) > fSaturation.x) 
    force.x = Sign(force.x)*fSaturation.x;
  if(Abs(force.y) > fSaturation.y) 
    force.y = Sign(force.y)*fSaturation.y;
  if(Abs(force.z) > fSaturation.z) 
    force.z = Sign(force.z)*fSaturation.z;
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
  FILL_SENSOR_SETTING(settings,fResolution);
  FILL_SENSOR_SETTING(settings,fVariance);
  FILL_SENSOR_SETTING(settings,fSensitivity);
  FILL_SENSOR_SETTING(settings,fSaturation);
  FILL_SENSOR_SETTING(settings,falloffCoefficient);
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
  GET_SENSOR_SETTING(fResolution);
  GET_SENSOR_SETTING(fVariance);
  GET_SENSOR_SETTING(fSensitivity);
  GET_SENSOR_SETTING(fSaturation);
  GET_SENSOR_SETTING(falloffCoefficient);
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
  SET_SENSOR_SETTING(fResolution);
  SET_SENSOR_SETTING(fVariance);
  SET_SENSOR_SETTING(fSensitivity);
  SET_SENSOR_SETTING(fSaturation);
  SET_SENSOR_SETTING(falloffCoefficient);
  return false;
}

void ContactSensor::DrawGL(const Robot& robot,const vector<double>& measurements)
{
  glPushMatrix();
  glMultMatrix(Matrix4(robot.links[link].T_World*Tsensor));
  glDisable(GL_LIGHTING);
  glColor3f(1,0,0);
  if(measurements.empty() || measurements[0] == 0) {
    glBegin(GL_LINE_LOOP);
    glVertex3f(patchMin.x,patchMin.y,0);
    glVertex3f(patchMax.x,patchMin.y,0);
    glVertex3f(patchMax.x,patchMax.y,0);
    glVertex3f(patchMin.x,patchMax.y,0);
    glEnd();
    if(patchTolerance > 0) {
      glPushMatrix();
      glTranslatef((patchMax.x+patchMin.x)*0.5,(patchMax.y+patchMin.y)*0.5,0.0);
      drawWireBox((patchMax.x-patchMin.x),(patchMax.y-patchMin.y),patchTolerance*2.0);
      glPopMatrix();
    }
  }
  else {
    glPushMatrix();
    glTranslatef((patchMax.x+patchMin.x)*0.5,(patchMax.y+patchMin.y)*0.5,0.0);
    glEnable(GL_LIGHTING);
    GLDraw::GLColor red(1,0,0);
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,red.rgba);
    drawBox((patchMax.x-patchMin.x),(patchMax.y-patchMin.y),0.001);
    glDisable(GL_LIGHTING);
    if(patchTolerance > 0) {
      drawWireBox((patchMax.x-patchMin.x),(patchMax.y-patchMin.y),patchTolerance*2.0);
    }
    glPopMatrix();
  }
  if(measurements.empty() || (!hasForce[0] && !hasForce[1] && !hasForce[2]))  {
    glPopMatrix();
    return;
  }
  Vector3 f(0.0);
  int i=1;
  if(hasForce[0]) { f.x=measurements[i]; i++; }
  if(hasForce[1]) { f.y=measurements[i]; i++; }
  if(hasForce[2]) { f.z=measurements[i]; i++; }
  glColor3f(1,0.5,0);
  glBegin(GL_LINES);
  glVertex3f(0,0,0);
  glVertex3v(f/9.8);
  glEnd();
  glPopMatrix();
}

ForceTorqueSensor::ForceTorqueSensor()
  :link(0),localPos(Zero),fVariance(Zero),tVariance(Zero),f(Zero),t(Zero)
{
  hasForce[0] = hasForce[1] = hasForce[2] = false;
  hasTorque[0] = hasTorque[1] = hasTorque[2] = false;
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
  //Vector3 comw = T*robot->robot->links[link].com;
  //Vector3 pw = T*localPos;
  //TEMP: is the moment measured about the joint axis?
  //Vector3 mw = cross(comw-pw,fw) + mcomw;
  Vector3 mw = mcomw;
  //convert to local frame
  T.R.mulTranspose(fw,f);
  T.R.mulTranspose(mw,t);

  //flip from internally imposed force to externally applied force
  f.inplaceNegative();
  t.inplaceNegative();

  f = Discretize(f,Vector3(0.0),fVariance);
  t = Discretize(t,Vector3(0.0),tVariance);
  for(int i=0;i<3;i++)
    if(!hasForce[i]) f[i] = 0;
  for(int i=0;i<3;i++)
    if(!hasTorque[i]) t[i] = 0;
}

void ForceTorqueSensor::Reset()
{
  f.setZero();
  t.setZero();
}

void ForceTorqueSensor::MeasurementNames(vector<string>& names) const
{
  names.resize(6);
  names[0] = "force_x";
  names[1] = "force_y";
  names[2] = "force_z";
  names[3] = "torque_x";
  names[4] = "torque_y";
  names[5] = "torque_z";
}

void ForceTorqueSensor::GetMeasurements(vector<double>& values) const
{
  values.resize(6);
  f.get(values[0],values[1],values[2]);
  t.get(values[3],values[4],values[5]);
}

void ForceTorqueSensor::SetMeasurements(const vector<double>& values)
{
  Assert(values.size()==6);
  f.set(values[0],values[1],values[2]);
  t.set(values[3],values[4],values[5]);
}

map<string,string> ForceTorqueSensor::Settings() const
{
  map<string,string> settings = SensorBase::Settings();
  FILL_SENSOR_SETTING(settings,link);
  FILL_SENSOR_SETTING(settings,localPos);
  FILL_ARRAY_SENSOR_SETTING(settings,hasForce,3);
  FILL_ARRAY_SENSOR_SETTING(settings,hasTorque,3);
  FILL_SENSOR_SETTING(settings,fVariance);
  FILL_SENSOR_SETTING(settings,tVariance);
  return settings;
}

bool ForceTorqueSensor::GetSetting(const string& name,string& str) const
{
  if(SensorBase::GetSetting(name,str)) return true;
  GET_SENSOR_SETTING(link);
  GET_SENSOR_SETTING(localPos);
  GET_ARRAY_SENSOR_SETTING(hasForce,3);
  GET_ARRAY_SENSOR_SETTING(hasTorque,3);
  GET_SENSOR_SETTING(fVariance);
  GET_SENSOR_SETTING(tVariance);  
  return false;
}

bool ForceTorqueSensor::SetSetting(const string& name,const string& str)
{
  if(SensorBase::SetSetting(name,str)) return true;
  SET_SENSOR_SETTING(link);
  SET_SENSOR_SETTING(localPos);
  SET_ARRAY_SENSOR_SETTING(hasForce,3);
  SET_ARRAY_SENSOR_SETTING(hasTorque,3);
  SET_SENSOR_SETTING(fVariance);
  SET_SENSOR_SETTING(tVariance);  
  return false;
}

void ForceTorqueSensor::DrawGL(const Robot& robot,const vector<double>& measurements)
{
  glPushMatrix();
  glMultMatrix(Matrix4(robot.links[link].T_World));
  if(measurements.size()!=6) {
    //just draw a box
    glEnable(GL_LIGHTING);
    GLDraw::GLColor orange(1,0.5,0);
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,orange);
    drawBox(0.05,0.05,0.05);
  }
  else {
    Vector3 f(0.0),m(0.0);
    for(int i=0;i<3;i++) if(hasForce[i]) f[i] = measurements[i];
    for(int i=0;i<3;i++) if(hasTorque[i]) m[i] = measurements[i+3];
    ViewWrench view;
    view.fscale = 1.0/9.8;
    view.mscale = 1.0/9.8;
    view.DrawGL(localPos,f,m);
  }
  glPopMatrix();
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

void Accelerometer::GetInternalState(vector<double>& state) const
{
  state.resize(4);
  state[0] = last_dt;
  last_v.get(state[1],state[2],state[3]);
}

void Accelerometer::SetInternalState(const vector<double>& state)
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

void GyroSensor::GetInternalState(vector<double>& state) const
{
  state.resize(0);
  state.push_back(last_dt);
  state.push_back(last_w.x);
  state.push_back(last_w.y);
  state.push_back(last_w.z);
}

void GyroSensor::SetInternalState(const vector<double>& state)
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

void IMUSensor::GetInternalState(vector<double>& state) const
{
  vector<double> astate,gstate;
  accelerometer.GetInternalState(astate);
  gyro.GetInternalState(gstate);
  state.resize(0);
  state.insert(state.end(),astate.begin(),astate.end());
  state.insert(state.end(),gstate.begin(),gstate.end());
}

void IMUSensor::SetInternalState(const vector<double>& state)
{
  Assert(state.size()==8);
  vector<double> astate,gstate;
  astate.resize(4);
  gstate.resize(4);
  copy(state.begin(),state.begin()+4,astate.begin());
  copy(state.begin()+4,state.end()+4,gstate.begin());
  accelerometer.SetInternalState(astate);
  gyro.SetInternalState(gstate);
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

void FilteredSensor::DrawGL(const Robot& robot,const vector<double>& measurements)
{
  if(sensor) sensor->DrawGL(robot,measurements);
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

void TimeDelayedSensor::GetInternalState(vector<double>& state) const
{
  if(!sensor) return;
  vector<double> sstate;
  sensor->GetInternalState(sstate);
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

void TimeDelayedSensor::SetInternalState(const vector<double>& state)
{
  if(!sensor) return;
  //just to get the size
  vector<double> sstate;
  sensor->GetInternalState(sstate);
  copy(state.begin(),state.begin()+sstate.size(),sstate.begin());
  sensor->SetInternalState(sstate);
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

void TimeDelayedSensor::DrawGL(const Robot& robot,const vector<double>& measurements)
{
  if(sensor) sensor->DrawGL(robot,measurements);
}







LaserRangeSensor::LaserRangeSensor()
:link(-1),measurementCount(180),depthResolution(0),depthMinimum(0.1),depthMaximum(Inf),
 depthVarianceLinear(0),depthVarianceConstant(0),
 xSweepMagnitude(DtoR(90.0)),xSweepPeriod(0),xSweepPhase(0),xSweepType(SweepSawtooth),
 ySweepMagnitude(0),ySweepPeriod(0),ySweepPhase(0),ySweepType(SweepSinusoid),
 last_dt(0),last_t(0)
{
  Tsensor.setIdentity();
}

void LaserRangeSensor::Advance(Real dt)
{
  last_dt = dt;
}

double EvalPattern(int type,double x,double correction=1.0)
{
  if(type == LaserRangeSensor::SweepSinusoid)
    return Sin(x*TwoPi);
  else if(type == LaserRangeSensor::SweepTriangular)
    return 2.0*(1.0+Abs(Mod(x,2.0) - 1.0))-1.0;
  return 2.0*(Mod(x/correction,1.0)*correction)-1.0;
}

void LaserRangeSensor::Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim)
{
  last_t = sim->time;
  depthReadings.resize(measurementCount);
  //need to make sure that the sawtooth pattern hits the last measurement: scale the time domain so last measurement before
  //loop gets 1
  Real xscale = 1, yscale = 1;
  if(xSweepType == SweepSawtooth && last_dt > 0 && measurementCount > 1) 
    xscale =  1.0 + 1.0/Real(measurementCount-1);
  if(ySweepType == SweepSawtooth && last_dt > 0 && measurementCount > 1) 
    xscale =  1.0 + 1.0/Real(measurementCount-1);
  Real ux0 = (xSweepPeriod == 0 ? 0 : (sim->time - last_dt + xSweepPhase)/xSweepPeriod);
  Real ux1 = (xSweepPeriod == 0 ? 1 : (sim->time + xSweepPhase)/xSweepPeriod);
  Real uy0 = (ySweepPeriod == 0 ? 0 : (sim->time - last_dt + ySweepPhase)/ySweepPeriod);
  Real uy1 = (ySweepPeriod == 0 ? 1 : (sim->time + ySweepPhase)/ySweepPeriod);
  //skip previous measurement
  if(xSweepPeriod != 0 && measurementCount > 1) ux0 += (ux1-ux0)/(measurementCount-1);
  if(ySweepPeriod != 0 && measurementCount > 1) uy0 += (uy1-uy0)/(measurementCount-1);
  Ray3D ray;
  RigidTransform T;
  if(link >= 0) {
    robot->oderobot->GetLinkTransform(link,T);
    T = T*Tsensor;
  }
  else
    T = Tsensor;
  Real xmin=0,xmax=0;
  Real ymin=0,ymax=0;
  for(int i=0;i<measurementCount;i++) {
    Real xtheta,ytheta;
    if(i+1 < measurementCount) {
      xtheta = xSweepMagnitude*EvalPattern(xSweepType,(ux0+Real(i)/Real(measurementCount-1)*(ux1-ux0)),xscale);
      ytheta = ySweepMagnitude*EvalPattern(ySweepType,(uy0+Real(i)/Real(measurementCount-1)*(uy1-uy0)),yscale);
    }
    else {
      xtheta = xSweepMagnitude*EvalPattern(xSweepType,ux1,xscale);
      ytheta = ySweepMagnitude*EvalPattern(ySweepType,uy1,yscale);
    }

    xmin = Min(xtheta,xmin);
    xmax = Max(xtheta,xmax);
    ymin = Min(ytheta,ymin);
    ymax = Max(ytheta,ymax);
    Real x = Sin(xtheta);
    Real y = Cos(xtheta)*Sin(ytheta);
    Real z = Cos(xtheta)*Cos(ytheta);
    ray.source = T*(Vector3(x,y,z)*depthMinimum);
    ray.direction = T.R*Vector3(x,y,z);
    Vector3 pt;
    //need to ignore the robot's link geometry
    int obj = sim->world->RayCast(ray,pt);
    if (obj >= 0) 
      depthReadings[i] = pt.distance(ray.source) + depthMinimum;
    else 
      depthReadings[i] = Inf;
  }
  //process all depth readings
  for(size_t i=0;i<depthReadings.size();i++) {
    if(!IsInf(depthReadings[i])) 
      depthReadings[i] = Discretize(depthReadings[i],depthResolution,depthReadings[i]*depthVarianceLinear + depthVarianceConstant);
    if(depthReadings[i] <= depthMinimum || depthReadings[i] >= depthMaximum) depthReadings[i] = depthMaximum;
  }
}

void LaserRangeSensor::Reset()
{
  depthReadings.resize(0);
}

void LaserRangeSensor::MeasurementNames(vector<string>& names) const
{
  names.resize(measurementCount);
  for(int i=0;i<measurementCount;i++) {
    stringstream ss;
    ss<<"d["<<i<<"]";
    names[i] = ss.str();
  }
}

void LaserRangeSensor::GetMeasurements(vector<double>& values) const
{
  values = depthReadings;
}

void LaserRangeSensor::SetMeasurements(const vector<double>& values)
{
  depthReadings = values;
}

map<string,string> LaserRangeSensor::Settings() const
{
  map<string,string> res = SensorBase::Settings();
  FILL_SENSOR_SETTING(res,link);
  FILL_SENSOR_SETTING(res,Tsensor);
  FILL_SENSOR_SETTING(res,measurementCount);
  FILL_SENSOR_SETTING(res,depthResolution);
  FILL_SENSOR_SETTING(res,depthMinimum);
  FILL_SENSOR_SETTING(res,depthMaximum);
  FILL_SENSOR_SETTING(res,depthVarianceLinear);
  FILL_SENSOR_SETTING(res,depthVarianceConstant);
  FILL_SENSOR_SETTING(res,xSweepMagnitude);
  FILL_SENSOR_SETTING(res,xSweepPeriod);
  FILL_SENSOR_SETTING(res,xSweepPhase);
  FILL_SENSOR_SETTING(res,xSweepType);
  FILL_SENSOR_SETTING(res,ySweepMagnitude);
  FILL_SENSOR_SETTING(res,ySweepPeriod);
  FILL_SENSOR_SETTING(res,ySweepPhase);
  FILL_SENSOR_SETTING(res,ySweepType);
  return res;
}
bool LaserRangeSensor::GetSetting(const string& name,string& str) const
{
  GET_SENSOR_SETTING(link);
  GET_SENSOR_SETTING(Tsensor);
  GET_SENSOR_SETTING(measurementCount);
  GET_SENSOR_SETTING(depthResolution);
  GET_SENSOR_SETTING(depthMinimum);
  GET_SENSOR_SETTING(depthMaximum);
  GET_SENSOR_SETTING(depthVarianceLinear);
  GET_SENSOR_SETTING(depthVarianceConstant);
  GET_SENSOR_SETTING(xSweepMagnitude);
  GET_SENSOR_SETTING(xSweepPeriod);
  GET_SENSOR_SETTING(xSweepPhase);
  GET_SENSOR_SETTING(xSweepType);
  GET_SENSOR_SETTING(ySweepMagnitude);
  GET_SENSOR_SETTING(ySweepPeriod);
  GET_SENSOR_SETTING(ySweepPhase);
  GET_SENSOR_SETTING(ySweepType);
  return false;
}
bool LaserRangeSensor::SetSetting(const string& name,const string& str)
{
  SET_SENSOR_SETTING(link);
  SET_SENSOR_SETTING(Tsensor);
  SET_SENSOR_SETTING(measurementCount);
  SET_SENSOR_SETTING(depthResolution);
  SET_SENSOR_SETTING(depthMinimum);
  SET_SENSOR_SETTING(depthMaximum);
  SET_SENSOR_SETTING(depthVarianceLinear);
  SET_SENSOR_SETTING(depthVarianceConstant);
  SET_SENSOR_SETTING(xSweepMagnitude);
  SET_SENSOR_SETTING(xSweepPeriod);
  SET_SENSOR_SETTING(xSweepPhase);
  SET_SENSOR_SETTING(xSweepType);
  SET_SENSOR_SETTING(ySweepMagnitude);
  SET_SENSOR_SETTING(ySweepPeriod);
  SET_SENSOR_SETTING(ySweepPhase);
  SET_SENSOR_SETTING(ySweepType);
  return false;
}

void LaserRangeSensor::DrawGL(const Robot& robot,const vector<double>& measurements) 
{
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE);
  glBegin(GL_LINES);
  //need to make sure that the sawtooth pattern hits the last measurement: scale the time domain so last measurement before
  //loop gets 1
  Real xscale = 1, yscale = 1;
  if(xSweepType == SweepSawtooth && last_dt > 0 && measurementCount > 1) 
    xscale =  1.0 + 1.0/Real(measurementCount-1);
  if(ySweepType == SweepSawtooth && last_dt > 0 && measurementCount > 1) 
    xscale =  1.0 + 1.0/Real(measurementCount-1);
  Real ux0 = (xSweepPeriod == 0 ? 0 : (last_t - last_dt + xSweepPhase)/(xSweepPeriod));
  Real ux1 = (xSweepPeriod == 0 ? 1 : (last_t + xSweepPhase)/(xSweepPeriod));
  Real uy0 = (ySweepPeriod == 0 ? 0 : (last_t - last_dt + ySweepPhase)/(ySweepPeriod));
  Real uy1 = (ySweepPeriod == 0 ? 1.0 : (last_t + ySweepPhase)/(ySweepPeriod));
  if(xSweepPeriod != 0) ux0 += (ux1-ux0)/(measurementCount);
  if(ySweepPeriod != 0) uy0 += (uy1-uy0)/(measurementCount);
  RigidTransform T = (link >= 0 ? robot.links[link].T_World*Tsensor : Tsensor);
  for(int i=0;i<measurementCount;i++) {
    if(!measurements.empty())
      if(IsInf(depthReadings[i])) continue;
    Real xtheta,ytheta;
    if(i+1 < measurementCount) {
      xtheta = xSweepMagnitude*EvalPattern(xSweepType,(ux0+Real(i)/Real(measurementCount-1)*(ux1-ux0)),xscale);
      ytheta = ySweepMagnitude*EvalPattern(ySweepType,(uy0+Real(i)/Real(measurementCount-1)*(uy1-uy0)),yscale);
    }
    else {
      xtheta = xSweepMagnitude*EvalPattern(xSweepType,ux1,xscale);
      ytheta = ySweepMagnitude*EvalPattern(ySweepType,uy1,yscale);
    }
    Real x = Sin(xtheta);
    Real y = Cos(xtheta)*Sin(ytheta);
    Real z = Cos(xtheta)*Cos(ytheta);
    Vector3 dir = T.R*Vector3(x,y,z);
    glColor4f(1,0,0,0);
    glVertex3v(T.t + depthMinimum*dir);
    glColor4f(1,0,0,1);
    if(measurements.empty())
      glVertex3v(T.t + depthMaximum*dir);
    else
      glVertex3v(T.t + depthReadings[i]*dir);
  }
  glEnd();
  glDisable(GL_BLEND);
}



CameraSensor::CameraSensor()
:link(-1),xres(640),yres(480),
 rgb(true),depth(true),
 xfov(DtoR(56.0)),yfov(DtoR(43.0)),
 zmin(0.4),zmax(4.0),zresolution(0),
 zvarianceLinear(0),zvarianceConstant(0),
 useGLFramebuffers(true),color_tex(0),fb(0),depth_rb(0)
{
  Tsensor.setIdentity();
}

CameraSensor::~CameraSensor()
{
  if(color_tex) glDeleteTextures(1, &color_tex);
#if HAVE_GLEW
  if(depth_rb) glDeleteRenderbuffersEXT(1, &depth_rb);
  if(fb) glDeleteFramebuffersEXT(1, &fb);
#endif //HAVE_GLEW
  color_tex = 0;
  depth_rb = 0;
  fb = 0;
}

void CameraSensor::Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim)
{
  RigidTransform Tlink;
  if(link >= 0) robot->oderobot->GetLinkTransform(link,Tlink);
  else Tlink.setIdentity();

#if HAVE_GLEW
  if(useGLFramebuffers) {
    if(!GLEW_EXT_framebuffer_object) {
      if (GLEW_OK != glewInit())
      {
        fprintf(stderr,"CameraSensor: Couldn't initialize GLEW, falling back to slow mode\n");
        useGLFramebuffers = false;
      }
      if(!GLEW_EXT_framebuffer_object) {
        fprintf(stderr,"CameraSensor: GL framebuffers not supported, falling back to slow mode\n");
        useGLFramebuffers = false;
      }
    }
  }
  if(useGLFramebuffers) {
    if(color_tex == 0) { 
      //RGBA8 2D texture, 24 bit depth texture, 256x256
      glGenTextures(1, &color_tex);
      glBindTexture(GL_TEXTURE_2D, color_tex);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      //NULL means reserve texture memory, but texels are undefined
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, xres, yres, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
    }
    if(fb == 0) {
      //-------------------------
      glGenFramebuffersEXT(1, &fb);
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb);
      //Attach 2D texture to this FBO
      glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, color_tex, 0);
    }
    if(depth_rb == 0) {
      //-------------------------
      glGenRenderbuffersEXT(1, &depth_rb);
      glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depth_rb);
      glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24, xres, yres);
      //-------------------------
      //Attach depth buffer to FBO
      glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, depth_rb);
    }
    //-------------------------
    //Does the GPU support current FBO configuration?
    GLenum status;
    status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
    switch(status)
    {
    case GL_FRAMEBUFFER_COMPLETE_EXT:
      break;
    default:
      fprintf(stderr,"CameraSensor: Couldn't initialize GL framebuffers, falling back to slow mode\n");
      useGLFramebuffers = false;
      //Delete resources
      glDeleteTextures(1, &color_tex);
      glDeleteRenderbuffersEXT(1, &depth_rb);
      //Bind 0, which means render to back buffer, as a result, fb is unbound
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0); 
      glDeleteFramebuffersEXT(1, &fb);
      color_tex = 0;
      depth_rb = 0;
      fb = 0;
      return;
    }
    //-------------------------
    //and now you can render to GL_TEXTURE_2D
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb);
    //float oldcolor[4];
    //glGetFloatv(GL_COLOR_CLEAR_VALUE,oldcolor);
    //glClearColor(0.0, 0.0, 0.0, 0.0);
    //glClearDepth(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //-------------------------
    //set up the POV of the camera
    Camera::Viewport vp;
    GetViewport(vp);
    vp.xform = Tlink*vp.xform;
    GLDraw::GLView view;
    view.setViewport(vp);
    view.setCurrentGL();
    //-------------------------
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    //-------------------------
    //now render the scene from the POV of the camera
    sim->UpdateModel();
    sim->world->DrawGL();
    //DONE: now captured on graphics card in framebuffer
    //----------------
    //Bind 0, which means render to back buffer
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
    //restore previous stuff
    //glClearColor(oldcolor[0],oldcolor[1],oldcolor[2],oldcolor[3]);
    CheckGLErrors("GL errors during camera sensor simulation: ");

    //extract measurements
    measurements.resize(0);
    if(rgb) {
      pixels.resize(4*xres*yres);
      glBindTexture(GL_TEXTURE_2D, color_tex);
      glGetTexImage(GL_TEXTURE_2D,0,GL_RGBA,GL_UNSIGNED_INT_8_8_8_8,&pixels[0]);
      measurements.resize(xres*yres);
      int k=0;
      //don't forget to flip vertically
      for(int j=0;j<yres;j++) {
        for(int i=0;i<xres;i++,k+=4) {
          unsigned int pix = (pixels[k] << 24 ) | (pixels[k+1] << 16 ) | (pixels[k+2] << 8 ) | (pixels[k+3]);
          measurements[(yres-j-1)*xres + i] = double(pix);
        }
      }
    }
    if(depth) {
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb); 
      floats.resize(xres*yres);
      glReadPixels(0, 0, xres, yres, GL_DEPTH_COMPONENT, GL_FLOAT, &floats[0]);
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0); 

      size_t vstart = measurements.size();
      measurements.resize(measurements.size() + xres*yres); 
      //don't forget to flip vertically
      int k=0;
      for(int j=0;j<yres;j++) {
        for(int i=0;i<xres;i++,k++) {
          //nonlinear depth normalization
          //normal linear interpolation would give u = (z - zmin)/(zmax-zmin)
          //instead we gt u = (1/zmin-1/z)/(1/zmin-1/zmax)
          //so 1/z = 1/zmin - u(1/zmin-1/zmax)
          if(floats[k] == 1.0) { //nothing seen
            floats[k] = zmax;
          }
          else {
            floats[k] = 1.0/(1.0/zmin - floats[k]*(1.0/zmin-1.0/zmax));
            floats[k] = Discretize(floats[k],zresolution,zvarianceLinear*floats[k] + zvarianceConstant);
          }
          measurements[vstart+(yres-j-1)*xres + i] = floats[k];
        }
      }
    }
  }
#else
  useGLFramebuffers = false;
#endif //HAVE_GLEW

  if(!useGLFramebuffers) {
    //fallback will use ray casting: (slow!)
    //set up the POV of the camera
    Camera::Viewport vp;
    GetViewport(vp);
    vp.xform = Tlink*vp.xform;
    Ray3D ray;
    Vector3 vsrc;
    Vector3 vfwd,dx,dy;
    vp.getClickSource(0,0,vsrc);
    vp.getViewVector(vfwd);
    dx = vp.xDir();
    dx *= 1.0/(vp.w*vp.scale);
    dy = vp.yDir();
    dy *= 1.0/(vp.w*vp.scale);
    measurements.resize(0);
    int dstart = 0;
    if(rgb) measurements.resize(xres*yres);
    if(depth) {
      dstart = (int)measurements.size();
      measurements.resize(measurements.size()+xres*yres);
    }
    int k=0;
    double background = double(0xff96aaff);
    Vector3 pt;
    for(int j=0;j<yres;j++) {
      Real v = 0.5*yres - Real(j);
      for(int i=0;i<xres;i++,k++) {
        Real u = Real(i) - 0.5*xres;    
        ray.direction = vfwd + u*dx + v*dy;
        ray.direction.inplaceNormalize();
        ray.source = vsrc + ray.direction * zmin / (vfwd.dot(ray.direction));
        int obj = sim->world->RayCast(ray,pt);
        if (obj >= 0) {
          if(rgb) {
            //get color of object
            //TODO: lighting
            RobotWorld::AppearancePtr app = sim->world->GetAppearance(obj);
            float* rgba = app->faceColor.rgba;
            measurements[k] = double(((unsigned char)(rgba[3]*255.0) << 24) | ((unsigned char)(rgba[0]*255.0) << 16) | ((unsigned char)(rgba[1]*255.0) << 8) | ((unsigned char)(rgba[2]*255.0)));
          }
          Real d = vfwd.dot(pt - vsrc);
          if(depth) measurements[dstart+k] = Discretize(d,zresolution,zvarianceLinear*d + zvarianceConstant);
        }
        else {
          //no reading
          if(rgb) measurements[k] = background;
          if(depth) measurements[dstart+k] = zmax;
        }
      }
    }
    static bool warned = false;
    if(!warned) {
      printf("DepthCameraSensor: doing fallback from GLEW... %d rays cast, may be slow\n",k);
      warned = true;
    }

    //need to upload the texture for sensor visualization
    if(color_tex == 0) { 
      //RGBA8 2D texture, 24 bit depth texture, 256x256
      glGenTextures(1, &color_tex);
      if(color_tex != 0) {
        glBindTexture(GL_TEXTURE_2D, color_tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      }
    }
    if(color_tex != 0) {
      //copy measurements into buffer -- don't forget y flip
      vector<unsigned int> image(xres*yres);
      int k=0;
      for(int j=0;j<yres;j++)
        for(int i=0;i<xres;i++,k++)
          image[(yres-j-1)*xres + i] = (unsigned int)measurements[k];
      //NULL means reserve texture memory, but texels are undefined
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, xres, yres, 0, GL_BGRA, GL_UNSIGNED_BYTE, &image[0]);
    }
  }
}

void CameraSensor::Reset()
{
}

void CameraSensor::MeasurementNames(vector<string>& names) const
{
  char buf[64];
  names.resize(0);
  if(rgb) {
    for(int i=0;i<xres;i++) {
      for(int j=0;j<yres;j++) {
        sprintf(buf,"rgb[%d,%d]",i,j);
        names.push_back(buf);
      }
    }
  }
  if(depth) {
    for(int i=0;i<xres;i++) {
      for(int j=0;j<yres;j++) {
        sprintf(buf,"d[%d,%d]",i,j);
        names.push_back(buf);
      }
    }
  }
}

void CameraSensor::GetMeasurements(vector<double>& values) const
{
  values = measurements;
}

void CameraSensor::SetMeasurements(const vector<double>& values)
{
  measurements = values;
  //TODO: copy back into pixel buffers?
}

map<string,string> CameraSensor::Settings() const
{
  map<string,string> res = SensorBase::Settings();
  FILL_SENSOR_SETTING(res,link);
  FILL_SENSOR_SETTING(res,Tsensor);
  FILL_SENSOR_SETTING(res,rgb);
  FILL_SENSOR_SETTING(res,depth);
  FILL_SENSOR_SETTING(res,xres);
  FILL_SENSOR_SETTING(res,xfov);
  FILL_SENSOR_SETTING(res,yres);
  FILL_SENSOR_SETTING(res,yfov);
  FILL_SENSOR_SETTING(res,zresolution);
  FILL_SENSOR_SETTING(res,zmin);
  FILL_SENSOR_SETTING(res,zmax);
  FILL_SENSOR_SETTING(res,zvarianceLinear);
  FILL_SENSOR_SETTING(res,zvarianceConstant);
  return res;
}
bool CameraSensor::GetSetting(const string& name,string& str) const
{
  GET_SENSOR_SETTING(link);
  GET_SENSOR_SETTING(Tsensor);
  GET_SENSOR_SETTING(rgb);
  GET_SENSOR_SETTING(depth);
  GET_SENSOR_SETTING(xres);
  GET_SENSOR_SETTING(xfov);
  GET_SENSOR_SETTING(yres);
  GET_SENSOR_SETTING(yfov);
  GET_SENSOR_SETTING(zresolution);
  GET_SENSOR_SETTING(zmin);
  GET_SENSOR_SETTING(zmax);
  GET_SENSOR_SETTING(zvarianceLinear);
  GET_SENSOR_SETTING(zvarianceConstant);
  return false;
}
bool CameraSensor::SetSetting(const string& name,const string& str)
{
  SET_SENSOR_SETTING(link);
  SET_SENSOR_SETTING(Tsensor);
  SET_SENSOR_SETTING(rgb);
  SET_SENSOR_SETTING(depth);
  SET_SENSOR_SETTING(xres);
  SET_SENSOR_SETTING(xfov);
  SET_SENSOR_SETTING(yres);
  SET_SENSOR_SETTING(yfov);
  SET_SENSOR_SETTING(zresolution);
  SET_SENSOR_SETTING(zmin);
  SET_SENSOR_SETTING(zmax);
  SET_SENSOR_SETTING(zvarianceLinear);
  SET_SENSOR_SETTING(zvarianceConstant);
  return false;
}

void doTriangle(const Vector3& a,const Vector3& b,const Vector3& c)
{
  Vector3 n;
  n.setCross(b-a,c-a);
  n.inplaceNormalize();
  glNormal3v(n);
  glVertex3v(a);
  glVertex3v(b);
  glVertex3v(c);
}

void CameraSensor::DrawGL(const Robot& robot,const vector<double>& measurements) 
{
  Camera::Viewport v;
  GetViewport(v);
  if(link >= 0) 
    v.xform = robot.links[link].T_World*v.xform;

  if(rgb && color_tex != 0) {
    //debugging: draw image in frustum
    glPushMatrix();
    glMultMatrix((Matrix4)v.xform);
    Real d = v.n;
    Real aspectRatio = Real(xres)/Real(yres);
    Real xmin = Real(v.x - v.w*0.5)/(Real(v.w)*0.5);
    Real xmax = Real(v.x + v.w*0.5)/(Real(v.w)*0.5);
    Real ymax = -Real(v.y - v.h*0.5)/(Real(v.h)*0.5);
    Real ymin = -Real(v.y + v.h*0.5)/(Real(v.h)*0.5);
    Real xscale = 0.5*d/v.scale;
    Real yscale = xscale/aspectRatio;
    xmin *= xscale;
    xmax *= xscale;
    ymin *= yscale;
    ymax *= yscale;
    glBindTexture(GL_TEXTURE_2D,color_tex);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(1,1,1,0.5);
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glBegin(GL_TRIANGLE_FAN);
    glTexCoord2f(0,0);
    glVertex3f(xmin,ymin,-d);
    glTexCoord2f(1,0);
    glVertex3f(xmax,ymin,-d);
    glTexCoord2f(1,1);
    glVertex3f(xmax,ymax,-d);
    glTexCoord2f(0,1);
    glVertex3f(xmin,ymax,-d);
    glEnd();
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glPopMatrix();
  }

  size_t vstart = 0;
  if(rgb) vstart = xres*yres;
  if(depth && !measurements.empty()) {
    glPushMatrix();
    glMultMatrix((Matrix4)v.xform);

    glEnable(GL_LIGHTING);
    float white[4]={1,1,1,1};
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,white);
    Real vscale = 0.5/Tan(xfov*0.5);
    Real xscale = (0.5/vscale);
    Real aspectRatio = Real(xres)/Real(yres);
    Real yscale = xscale;
    vector<Vector3> pts(xres*yres);
    int k=0;
    for(int i=0;i<yres;i++)
      for(int j=0;j<xres;j++,k++) {
        double d = measurements[vstart+k];
        double u = Real(j-xres/2)/(xres/2);
        double v = -Real(i-yres/2)/(xres/2);
        double x = xscale*d*u;
        double y = yscale*d*v;
        pts[k].set(x,y,-d);
      }
    glBegin(GL_TRIANGLES);
    k=0;
    for(int i=0;i<yres;i++) {
      for(int j=0;j<xres;j++,k++) {
        if(i+1 >= yres || j+1 >= xres) continue;
        //decide on discontinuities in this cell
        int v11 = k;
        int v12 = k+1;
        int v21 = k+xres;
        int v22 = k+xres+1;
        double z11 = -pts[v11].z;
        double z12 = -pts[v12].z;
        double z21 = -pts[v21].z;
        double z22 = -pts[v22].z;
        bool d1x = (z11 >= zmax || z12 >= zmax || Abs(z11 - z12) > 0.02*(z11+z12));
        bool d1y = (z11 >= zmax || z21 >= zmax || Abs(z11 - z21) > 0.02*(z11+z21));
        bool d2x = (z22 >= zmax || z21 >= zmax || Abs(z22 - z21) > 0.02*(z22+z21));
        bool d2y = (z22 >= zmax || z12 >= zmax || Abs(z22 - z12) > 0.02*(z22+z12));
        bool dupperleft = (d1x || d1y);
        bool dupperright = (d1x || d2y);
        bool dlowerleft = (d2x || d1y);
        bool dlowerright = (d2x || d2y);


        if(dupperleft && !dlowerright) 
          //only draw lower right corner
          doTriangle(pts[v12],pts[v21],pts[v22]);
        else if(!dupperleft && dlowerright) 
          //only draw upper left corner
          doTriangle(pts[v11],pts[v21],pts[v12]);
        else if(!dupperright && dlowerleft) 
          //only draw upper right corner
          doTriangle(pts[v11],pts[v22],pts[v12]);
        else if(dupperright && !dlowerleft) 
          //only draw lower left corner
          doTriangle(pts[v11],pts[v21],pts[v22]);
        else if (!dupperleft && !dlowerright) {
          //fully connected -- should draw better conditioned edge, but whatever
          doTriangle(pts[v12],pts[v21],pts[v22]);
          doTriangle(pts[v11],pts[v21],pts[v12]);
        }
      }
    }
    glEnd();
    glPopMatrix();
  }

  ViewCamera view;
  view.DrawGL(v);
}


void CameraSensor::GetViewport(Camera::Viewport& vp) const
{
  vp.perspective = true;
  vp.x = vp.y = 0;
  vp.w = xres;
  vp.h = yres;
  vp.n = zmin;
  vp.f = zmax;
  vp.setLensAngle(xfov);
  vp.xform = Tsensor;
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
    else if(0==strcmp(e->Value(),"LaserRangeSensor")) {
      sensor = new LaserRangeSensor;
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"CameraSensor")) {
      sensor = new CameraSensor;
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
	fprintf(stderr,"Filtered sensor has unknown sensor named \"%s\"\n",e->Attribute("sensor"));
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
	fprintf(stderr,"Time-delayed sensor has unknown sensor named \"%s\"\n",e->Attribute("sensor"));
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
    s->SetMeasurements(measurementValues);

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
  string sensorXml;
  if(robot->properties.get("sensors",sensorXml)) {
    TiXmlElement n("sensors");
    stringstream ss(sensorXml);
    ss>>n;
    if(ss) {
      //for any named links, convert them to indices
      TiXmlElement* c = n.FirstChildElement();
      while(c) {
        if(c->Attribute("link")) {
          int ind = robot->LinkIndex(c->Attribute("link"));
          if(ind >= 0) {
            c->SetAttribute("link",ind);
          }
        }
        c = c->NextSiblingElement();
      }
      if(LoadSettings(&n)) {
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
    }
    printf("RobotSensors::MakeDefault: invalid sensor data format %s\n",sensorXml.c_str());
    printf("  Making the standard sensors instead.\n");
    printf("  Press enter to continue.\n");
    getchar();
    sensors.resize(0);
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
