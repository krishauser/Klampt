#include "InertialSensors.h"
#include "Common_Internal.h"
#include "Simulation/SimRobotController.h"
#include "Simulation/ODESimulator.h"
#include "Simulation/Simulator.h"

using namespace Klampt;

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

void Accelerometer::SimulateKinematic(RobotModel& robot,WorldModel& world)
{
  RigidTransform T;
  Vector3 w,v,vp;
  T = robot.links[link].T_World;
  robot.GetWorldVelocity(Vector3(0.0),link,robot.dq,v);
  robot.GetWorldAngularVelocity(link,robot.dq,w);
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


void Accelerometer::Simulate(SimRobotController* robot,Simulator* sim)
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

void TiltSensor::SimulateKinematic(RobotModel& robot,WorldModel& world)
{
  RigidTransform T;
  Vector3 w,v;
  T = robot.links[link].T_World;
  robot.GetWorldVelocity(Vector3(0.0),link,robot.dq,v);
  robot.GetWorldAngularVelocity(link,robot.dq,w);

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

void TiltSensor::Simulate(SimRobotController* robot,Simulator* sim)
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

void GyroSensor::SimulateKinematic(RobotModel& robot,WorldModel& world)
{
  RigidTransform T;
  Vector3 w,v;
  T = robot.links[link].T_World;
  robot.GetWorldVelocity(Vector3(0.0),link,robot.dq,v);
  robot.GetWorldAngularVelocity(link,robot.dq,w);

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

void GyroSensor::Simulate(SimRobotController* robot,Simulator* sim)
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

void IMUSensor::SimulateKinematic(RobotModel& robot,WorldModel& world)
{
  RigidTransform T;
  T = robot.links[accelerometer.link].T_World;

  accelerometer.SimulateKinematic(robot,world);
  accel = accelerometer.accel;
  //translate to global frame and remove gravity from acceleration reading
  accel = T.R*accel;
  accel += Vector3(0,0,9.8);
  //integrate velocity and position
  translation.madd(velocity,accelerometer.last_dt);
  translation.madd(accel,0.5*Sqr(accelerometer.last_dt));
  velocity.madd(accel,accelerometer.last_dt);

  gyro.SimulateKinematic(robot,world);
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


void IMUSensor::Simulate(SimRobotController* robot,Simulator* sim)
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
  if(name == "link") {
    accelerometer.SetSetting(name,str); 
    gyro.SetSetting(name,str); 
    return true;
  }
  if(accelerometer.SetSetting(name,str)) return true;
  if(gyro.SetSetting(name,str)) return true;
  return false;
}
