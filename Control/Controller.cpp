#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "Controller.h"
#include "FeedforwardController.h"
#include "LoggingController.h"
#include "PathController.h"
#include "JointTrackingController.h"
#include "SerialController.h"
#include "JointSensors.h"
#include <KrisLibrary/utils/PropertyMap.h>
#include <tinyxml.h>

RobotController::RobotController(Robot& _robot)
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
    if(robot.drivers[i].type == RobotJointDriver::Normal) {
      command->actuators[i].SetPID(qdes(robot.drivers[i].linkIndices[0]),dqdes(robot.drivers[i].linkIndices[0]),command->actuators[i].iterm);
    }
    else {
      robot.q = qdes;
      robot.dq = dqdes;
      //LOG4CXX_INFO(KrisLibrary::logger(),"Desired affine driver value "<<robot.GetDriverValue(i)<<", vel "<<robot.GetDriverVelocity(i));
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
      if(robot.drivers[i].type == RobotJointDriver::Normal) {
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
    FatalError("RobotController::SetTorqueCommand: invalid vector size: %d\n",torques.size());
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
            LOG4CXX_ERROR(KrisLibrary::logger(),"RobotController::GetCommandedConfig: driver "<<i);
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
            LOG4CXX_ERROR(KrisLibrary::logger(),"RobotController::GetCommandedVelocity: driver "<<i);
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
        LOG4CXX_ERROR(KrisLibrary::logger(),"RobotController: Warning, robot has no joint position sensor\n");
        LOG4CXX_ERROR(KrisLibrary::logger(),"Sensor list:\n");
    for(size_t i=0;i<sensors->sensors.size();i++)
            LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<sensors->sensors[i]->Type()<<": "<<sensors->sensors[i]->name.c_str());
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
        LOG4CXX_ERROR(KrisLibrary::logger(),"RobotController: Warning, robot has no joint velocity sensor\n");
        LOG4CXX_ERROR(KrisLibrary::logger(),"Sensor list:\n");
    for(size_t i=0;i<sensors->sensors.size();i++)
            LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<sensors->sensors[i]->Type()<<": "<<sensors->sensors[i]->name.c_str());
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



void RobotControllerFactory::RegisterDefault(Robot& robot)
{
  Register("JointTrackingController",new JointTrackingController(robot));
  Register("PolynomialPathController",new PolynomialPathController(robot));
  Register("FeedforwardJointTrackingController",new FeedforwardController(robot,new JointTrackingController(robot)));
  Register("FeedforwardPolynomialPathController",new FeedforwardController(robot,new PolynomialPathController(robot)));
  Register("SerialController",new SerialController(robot));
}

void RobotControllerFactory::Register(RobotController* controller)
{
  Register(controller->Type(),controller);
}

void RobotControllerFactory::Register(const char* name,RobotController* controller)
{
  controllers[name] = controller;
}

SmartPointer<RobotController> RobotControllerFactory::CreateByName(const char* name)
{
  for(map<std::string,SmartPointer<RobotController> >::iterator i=controllers.begin();i!=controllers.end();i++)
    if(i->first == name) return i->second;
  return NULL;
}

SmartPointer<RobotController> RobotControllerFactory::CreateByName(const char* name,Robot& robot)
{
  for(map<std::string,SmartPointer<RobotController> >::iterator i=controllers.begin();i!=controllers.end();i++)
    if(i->first == name && &i->second->robot==&robot) return i->second;
  return NULL;
}

SmartPointer<RobotController> RobotControllerFactory::Load(TiXmlElement* in,Robot& robot)
{
  if(0!=strcmp(in->Value(),"controller")) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Controller does not have type \"controller\", got "<<in->Value());
    return NULL;
  }
  if(in->Attribute("type")==NULL) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Controller does not have \"type\" attribute\n");
    return NULL;
  }
  SmartPointer<RobotController> c = CreateByName(in->Attribute("type"),robot);
  if(!c) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to load controller of type "<<in->Attribute("type"));
        LOG4CXX_ERROR(KrisLibrary::logger(),"Candidates: \n");
    for(map<std::string,SmartPointer<RobotController> >::iterator i=controllers.begin();i!=controllers.end();i++)
            LOG4CXX_ERROR(KrisLibrary::logger(),"  "<<i->first.c_str());
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
		LOG4CXX_ERROR(KrisLibrary::logger(),"Load controller  "<<in->Attribute("type")<<" from XML: Unable to set setting "<<attr->Name());
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

SmartPointer<RobotController> RobotControllerFactory::Load(const char* fn,Robot& robot)
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

map<std::string,SmartPointer<RobotController> > RobotControllerFactory::controllers;



SmartPointer<RobotController> MakeDefaultController(Robot* robot)
{
  string controllerXml;
  if(robot->properties.get("controller",controllerXml)) {
    TiXmlElement n("controller");
    stringstream ss(controllerXml);
    ss >> n;
    if(ss) {
      SmartPointer<RobotController> res = RobotControllerFactory::Load(&n,*robot);
      if(res) return res;
    }
  
    LOG4CXX_INFO(KrisLibrary::logger(),"MakeDefaultController: could not load controller from data "<<controllerXml.c_str());
    LOG4CXX_INFO(KrisLibrary::logger(),"  Making the standard controller instead.\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"  Press enter to continue.\n");
    //KrisLibrary::loggerWait();
  }
  PolynomialPathController* c = new PolynomialPathController(*robot);
  FeedforwardController* fc = new FeedforwardController(*robot,c);
  LoggingController* lc=new LoggingController(*robot,fc);
  //defaults -- gravity compensation is better off with free-floating robots
  if(robot->joints[0].type == RobotJoint::Floating)
    fc->enableGravityCompensation=false;  //feedforward capability
  else
    fc->enableGravityCompensation=true;  //feedforward capability
  fc->enableFeedforwardAcceleration=false;  //feedforward capability
  lc->save = false;
  return lc;
}
