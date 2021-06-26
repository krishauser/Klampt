#include "Controller.h"
#include "FeedforwardController.h"
#include "LoggingController.h"
#include "PathController.h"
#include "JointTrackingController.h"
#include "SerialController.h"
#include "Sensing/JointSensors.h"
#include <KrisLibrary/utils/PropertyMap.h>
#include <tinyxml.h>

namespace Klampt {

RobotController::RobotController(RobotModel& _robot)
  : robot(_robot),time(0),nominalTimeStep(0),sensors(NULL),command(NULL)
{}

bool RobotController::ReadState(File& f) 
{
  if(!ReadFile(f,time)) return false;
  return true;
}

bool RobotController::WriteState(File& f) const 
{
  if(!WriteFile(f,time)) return false;
  return true;
}

void RobotController::SetPIDCommand(const Config& qdes)
{
  Config dqdes(qdes.size(),0.0);
  SetPIDCommand(qdes,dqdes);
}

void RobotController::SetPIDCommand(const Config& _qdes,const Config& dqdes)
{
  Assert(_qdes.size()==robot.links.size());
  Assert(dqdes.size()==robot.links.size());
  Config qdes = _qdes;
  //TEMP: do we want to normalize angles here or at a higher level in the
  //controller?
  //robot.NormalizeAngles(qdes);
  for(size_t i=0;i<robot.drivers.size();i++) {
    if(robot.drivers[i].type == RobotModelDriver::Normal) {
      command->actuators[i].SetPID(qdes(robot.drivers[i].linkIndices[0]),dqdes(robot.drivers[i].linkIndices[0]),command->actuators[i].iterm);
    }
    else {
      robot.q = qdes;
      robot.dq = dqdes;
      //printf("Desired affine driver value %g, vel %g\n",robot.GetDriverValue(i),robot.GetDriverVelocity(i));
      command->actuators[i].SetPID(robot.GetDriverValue(i),robot.GetDriverVelocity(i),command->actuators[i].iterm);
    }
  }
}

void RobotController::SetTorqueCommand(const Vector& torques)
{
  if(torques.size()==robot.drivers.size()) {
    //setting drivers directly
    for(size_t i=0;i<robot.drivers.size();i++)
      command->actuators[i].SetTorque(torques[i]);
  }
  else if(torques.size()==robot.links.size()) {
    //parse out links
    for(size_t i=0;i<robot.drivers.size();i++) {
      if(robot.drivers[i].type == RobotModelDriver::Normal) {
	command->actuators[i].SetTorque(torques(robot.drivers[i].linkIndices[0]));
      }
      else {
	//use dq as a temporary variable
	Vector dq;
	swap(dq,robot.dq);
	robot.dq = torques;
	command->actuators[i].SetTorque(robot.GetDriverVelocity(i));
	swap(dq,robot.dq);
      }
    }
  }
  else {
    FatalError("RobotController::SetTorqueCommand: invalid vector size: %d\n",(int)torques.size());
  }
}

void RobotController::SetFeedforwardPIDCommand(const Config& qdes,const Config& dqdes,const Vector& torques)
{
  Assert(torques.size()==robot.drivers.size());
  SetPIDCommand(qdes,dqdes);
  for(size_t i=0;i<robot.drivers.size();i++)
    command->actuators[i].torque = torques[i];
}


bool RobotController::GetCommandedConfig(Config& q) 
{
  Assert(command->actuators.size() == robot.drivers.size());
  for(size_t i=0;i<command->actuators.size();i++) {
    if(command->actuators[i].mode == ActuatorCommand::PID)
      robot.SetDriverValue(i,command->actuators[i].qdes);
    else {
      LOG4CXX_WARN(KrisLibrary::logger(), "RobotController::GetCommandedConfig: driver " << i << " is not in PID mode");
      return false;
    }
  }
  q = robot.q;
  return true;
}

bool RobotController::GetCommandedVelocity(Config& dq)
{ 
  Assert(command->actuators.size() == robot.drivers.size());
  for(size_t i=0;i<command->actuators.size();i++) {
    if(command->actuators[i].mode == ActuatorCommand::PID)
      robot.SetDriverVelocity(i,command->actuators[i].dqdes);
    else {
      LOG4CXX_WARN(KrisLibrary::logger(),"RobotController::GetCommandedVelocity: driver "<<i<<" is not in PID mode");
      return false;
    }
  }
  dq = robot.dq;
  return true;
}

bool RobotController::GetSensedConfig(Config& q)
{
  JointPositionSensor* s = sensors->GetTypedSensor<JointPositionSensor>();
  if(s==NULL) {
    LOG4CXX_WARN(KrisLibrary::logger(), "RobotController::GetSensedConfig: Warning, robot has no joint position sensor");
    LOG4CXX_WARN(KrisLibrary::logger(), "  Sensor list:");
    for(size_t i=0;i<sensors->sensors.size();i++)
      LOG4CXX_WARN(KrisLibrary::logger(), "    "<<sensors->sensors[i]->Type()<< ": "<<sensors->sensors[i]->name);
    return false;
  }
  if(s->indices.empty())
    q = s->q;
  else {
    //q.resize(robot.q.size());
    //q.set(0.0);
    robot.q.setZero();
    if(command) {
      for(size_t i=0;i<command->actuators.size();i++) {
        if(command->actuators[i].mode == ActuatorCommand::PID)
          robot.SetDriverValue(i,command->actuators[i].qdes);
      }
    }
    q = robot.q;
    for(size_t i=0;i<s->indices.size();i++) {
      Assert(s->indices[i] >= 0 && s->indices[i] < q.n);
      q[s->indices[i]] = s->q[i];
    }
  }
  return true;
}

bool RobotController::GetSensedVelocity(Config& dq)
{
  JointVelocitySensor* s=sensors->GetTypedSensor<JointVelocitySensor>();
  if(s==NULL) {
      LOG4CXX_WARN(KrisLibrary::logger(), "RobotController::GetSensedVelocity: Warning, robot has no joint velocity sensor");
      LOG4CXX_WARN(KrisLibrary::logger(), "  Sensor list:");
      for (size_t i = 0; i<sensors->sensors.size(); i++)
        LOG4CXX_WARN(KrisLibrary::logger(), "    " << sensors->sensors[i]->Type() << ": " << sensors->sensors[i]->name);
    return false;
  }
  if(s->indices.empty())
    dq = s->dq;
  else {
    //dq.resize(robot.q.size());
    //dq.set(0.0);
    robot.dq.setZero();
    if(command) {
      for(size_t i=0;i<command->actuators.size();i++) {
        if(command->actuators[i].mode == ActuatorCommand::PID)
          robot.SetDriverVelocity(i,command->actuators[i].dqdes);
      }
    }    
    dq = robot.dq;
    for(size_t i=0;i<s->indices.size();i++) {
      Assert(s->indices[i] >= 0 && s->indices[i] < dq.n);
      dq[s->indices[i]] = s->dq[i];
    }
  }
  return true;
}



void RobotControllerFactory::RegisterDefault(RobotModel& robot)
{
  Register("JointTrackingController",new JointTrackingController(robot));
  Register("PolynomialPathController",new PolynomialPathController(robot));
  Register("FeedforwardJointTrackingController",new FeedforwardController(robot,make_shared<JointTrackingController>(robot)));
  Register("FeedforwardPolynomialPathController",new FeedforwardController(robot,make_shared<PolynomialPathController>(robot)));
  Register("SerialController",new SerialController(robot));
}

void RobotControllerFactory::Register(RobotController* controller)
{
  Register(controller->Type(),controller);
}

void RobotControllerFactory::Register(const char* name,RobotController* controller)
{
  controllers[name].reset(controller);
}

shared_ptr<RobotController> RobotControllerFactory::CreateByName(const char* name)
{
  for(map<std::string,shared_ptr<RobotController> >::iterator i=controllers.begin();i!=controllers.end();i++)
    if(i->first == name) return i->second;
  return NULL;
}

shared_ptr<RobotController> RobotControllerFactory::CreateByName(const char* name,RobotModel& robot)
{
  for(map<std::string,shared_ptr<RobotController> >::iterator i=controllers.begin();i!=controllers.end();i++)
    if(i->first == name && &i->second->robot==&robot) return i->second;
  return NULL;
}

shared_ptr<RobotController> RobotControllerFactory::Load(TiXmlElement* in,RobotModel& robot)
{
  if(0!=strcmp(in->Value(),"controller")) {
    LOG4CXX_WARN(KrisLibrary::logger(),"Controller does not have type \"controller\", got "<<in->Value());
    return NULL;
  }
  if(in->Attribute("type")==NULL) {
    LOG4CXX_WARN(KrisLibrary::logger(),"Controller does not have \"type\" attribute");
    return NULL;
  }
  shared_ptr<RobotController> c = CreateByName(in->Attribute("type"),robot);
  if(!c) {
    LOG4CXX_WARN(KrisLibrary::logger(),"Unable to load controller of type "<<in->Attribute("type"));
    LOG4CXX_WARN(KrisLibrary::logger(),"Candidates: ");
    for(map<std::string,shared_ptr<RobotController> >::iterator i=controllers.begin();i!=controllers.end();i++)
      LOG4CXX_WARN(KrisLibrary::logger(),"  "<<i->first);
    return NULL;
  }
  TiXmlAttribute* attr = in->FirstAttribute();
  while(attr != NULL) {
    if(0==strcmp(attr->Name(),"type")) {
      //skip
    }
    else if(0==strcmp(attr->Name(),"rate")) {
      double temp=0;
      if(in->QueryValueAttribute("rate",&temp)!=TIXML_SUCCESS || (temp <= 0)){
        LOG4CXX_ERROR(KrisLibrary::logger(),"Invalid rate "<<temp);
        return NULL;
      }
      else 
        c->nominalTimeStep = 1.0/temp;
    }
    else if(0==strcmp(attr->Name(),"timeStep")) {
      double temp=0;
      if(in->QueryValueAttribute("timeStep",&temp)!=TIXML_SUCCESS || temp <= 0){
	LOG4CXX_ERROR(KrisLibrary::logger(),"Invalid timestep "<<temp);
	return NULL;
      }
      c->nominalTimeStep = temp;
    }
    else {
      if(!c->SetSetting(attr->Name(),attr->Value())) {
		  LOG4CXX_ERROR(KrisLibrary::logger(),"Load controller "<<in->Attribute("type")<<" from XML: Unable to set setting "<<attr->Name());
	return NULL;
      }
    }
    attr = attr->Next();
  }
  return c;
}


bool RobotControllerFactory::Save(RobotController* controller,TiXmlElement* out)
{
  out->SetValue("controller");
  out->SetAttribute("type",controller->Type());
  PropertyMap settings=controller->Settings();
  settings.Save(out);
  return true;
}

shared_ptr<RobotController> RobotControllerFactory::Load(const char* fn,RobotModel& robot)
{
  TiXmlDocument doc;
  if(!doc.LoadFile(fn)) return NULL;
  return Load(doc.RootElement(),robot);
}

bool RobotControllerFactory::Save(RobotController* controller,const char* fn)
{
  TiXmlDocument doc;
  if(!Save(controller,doc.RootElement())) return false;
  return doc.SaveFile(fn);
}

map<std::string,shared_ptr<RobotController> > RobotControllerFactory::controllers;



shared_ptr<RobotController> MakeDefaultController(RobotModel* robot)
{
  string controllerXml;
  if(robot->properties.get("controller",controllerXml)) {
    TiXmlElement n("controller");
    stringstream ss(controllerXml);
    ss >> n;
    if(ss) {
      shared_ptr<RobotController> res = RobotControllerFactory::Load(&n,*robot);
      if(res) return res;
    }
  
    LOG4CXX_ERROR(KrisLibrary::logger(), "MakeDefaultController: could not load controller from data "<<controllerXml);
    LOG4CXX_ERROR(KrisLibrary::logger(), "  Making the standard controller instead.");
    //KrisLibrary::loggerWait();
  }
  auto c = make_shared<PolynomialPathController>(*robot);
  auto fc = make_shared<FeedforwardController>(*robot,c);
  auto lc= make_shared<LoggingController>(*robot,fc);
  //defaults -- gravity compensation is better off with free-floating robots
  if(robot->joints[0].type == RobotModelJoint::Floating)
    fc->enableGravityCompensation=false;  //feedforward capability
  else
    fc->enableGravityCompensation=true;  //feedforward capability
  fc->enableFeedforwardAcceleration=false;  //feedforward capability
  lc->save = false;
  return lc;
}

} //namespace Klampt