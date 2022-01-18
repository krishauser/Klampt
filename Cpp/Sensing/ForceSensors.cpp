#include "ForceSensors.h"
#include "Common_Internal.h"
#include "View/ViewWrench.h"
#include "Simulation/SimRobotController.h"
#include "Simulation/ODESimulator.h"
#include "Simulation/Simulator.h"
#include <KrisLibrary/robotics/NewtonEuler.h>
#include <KrisLibrary/GLdraw/drawextra.h>

using namespace Klampt;
using namespace GLDraw;


namespace Klampt {

//defined in ODESimulator.cpp
bool HasContact(dBodyID a);

//defined in ODESimulator.cpp
///Will produce bogus o1 and o2 vectors
void GetContacts(dBodyID a,vector<ODEContactList>& contacts);

} //namespace Klampt


ContactSensor::ContactSensor()
  :link(0),patchMin(Zero),patchMax(Zero),patchTolerance(0.001),
  fResolution(Zero),fVariance(Zero),fSensitivity(Zero),fSaturation(Inf),
  falloffCoefficient(0),
  contact(false),force(Zero)
{
  Tsensor.setIdentity();
  hasForce[0] = hasForce[1] = hasForce[2] = false;
}

void ContactSensor::SimulateKinematic(RobotModel& robot,WorldModel& world)
{
  contact = false;
  force.setZero();
}

void ContactSensor::Simulate(SimRobotController* robot,Simulator* sim)
{
  contact = false;
  force.setZero();
  dBodyID body = robot->oderobot->body(link);
  if(!body || !HasContact(body)) {
    return;
  }
  RigidTransform Tlink;
  robot->oderobot->GetLinkTransform(link,Tlink);
  RigidTransform TsensorWorld  = Tlink*Tsensor;
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
        TsensorWorld.R.mulTranspose(contacts[i].forces[j],flocal);
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

void ContactSensor::DrawGL(const RobotModel& robot,const vector<double>& measurements)
{
  glPushMatrix();
  glMultMatrix(Matrix4(robot.links[link].T_World*Tsensor));
  glDisable(GL_LIGHTING);
  glColor3f(1,0,0);
  if(measurements.empty() || measurements[0] == 0) {
    glBegin(GL_LINE_LOOP);
    glVertex3d(patchMin.x,patchMin.y,0);
    glVertex3d(patchMax.x,patchMin.y,0);
    glVertex3d(patchMax.x,patchMax.y,0);
    glVertex3d(patchMin.x,patchMax.y,0);
    glEnd();
    if(patchTolerance > 0) {
      glPushMatrix();
      glTranslated((patchMax.x+patchMin.x)*0.5,(patchMax.y+patchMin.y)*0.5,0.0);
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
  :link(0),fVariance(Zero),tVariance(Zero),f(Zero),t(Zero)
{
  hasForce[0] = hasForce[1] = hasForce[2] = false;
  hasTorque[0] = hasTorque[1] = hasTorque[2] = false;
}

void ForceTorqueSensor::SimulateKinematic(RobotModel& robot,WorldModel& world)
{
  f.setZero();
  t.setZero();

  NewtonEulerSolver ne(robot);
  ne.SetGravityWrenches(Vector3(0,0,-9.806));
  Vector torques;
  ne.CalcTorques(Vector(robot.q.n,0.0),torques);
  Vector3 fw = ne.jointWrenches[link].f;
  Vector3 mw = ne.jointWrenches[link].m;

  RigidTransform T = robot.links[link].T_World;
  T.R.mulTranspose(fw,f);
  T.R.mulTranspose(mw,t);

  f = Discretize(f,Vector3(0.0),fVariance);
  t = Discretize(t,Vector3(0.0),tVariance);
  for(int i=0;i<3;i++)
    if(!hasForce[i]) f[i] = 0;
  for(int i=0;i<3;i++)
    if(!hasTorque[i]) t[i] = 0;
}

void ForceTorqueSensor::Simulate(SimRobotController* robot,Simulator* sim) 
{
  dJointFeedback fb = robot->oderobot->feedback(link);
  Vector3 w,v;
  robot->oderobot->GetLinkVelocity(link,w,v);
  Vector3 fw(fb.f1[0],fb.f1[1],fb.f1[2]);
  RigidTransform T;
  robot->oderobot->GetLinkTransform(link,T);
  Vector3 mcomw = Vector3(fb.t1[0],fb.t1[1],fb.t1[2]);
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
  SET_ARRAY_SENSOR_SETTING(hasForce,3);
  SET_ARRAY_SENSOR_SETTING(hasTorque,3);
  SET_SENSOR_SETTING(fVariance);
  SET_SENSOR_SETTING(tVariance);  
  return false;
}

void ForceTorqueSensor::DrawGL(const RobotModel& robot,const vector<double>& measurements)
{
  glPushMatrix();
  glMultMatrix(Matrix4(robot.links[link].T_World));
  if(measurements.size()!=6) {
    //just draw a box
    glEnable(GL_LIGHTING);
    GLDraw::GLColor orange(1.f,0.5f,0);
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,orange);
    drawBox(0.05f,0.05f,0.05f);
  }
  else {
    Vector3 f(0.0),m(0.0);
    for(int i=0;i<3;i++) if(hasForce[i]) f[i] = measurements[i];
    for(int i=0;i<3;i++) if(hasTorque[i]) m[i] = measurements[i+3];
    ViewWrench view;
    view.fscale = 1.0/9.8;
    view.mscale = 1.0/9.8;
    view.DrawGL(Vector3(0.0),f,m);
  }
  glPopMatrix();
}
