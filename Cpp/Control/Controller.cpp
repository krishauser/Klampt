#include "Controller.h"
#include "FeedforwardController.h"
#include "LoggingController.h"
#include "PathController.h"
#include "JointTrackingController.h"
#include "SerialController.h"
#include "Sensing/JointSensors.h"
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
      fprintf(stderr,"RobotController::GetCommandedConfig: driver %d is not in PID mode",i);
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
      fprintf(stderr,"RobotController::GetCommandedVelocity: driver %d is not in PID mode",i);
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
    fprintf(stderr,"RobotController: Warning, robot has no joint position sensor\n");
    fprintf(stderr,"Sensor list:\n");
    for(size_t i=0;i<sensors->sensors.size();i++)
      fprintf(stderr,"  %s: %s\n",sensors->sensors[i]->Type(),sensors->sensors[i]->name.c_str());
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
    fprintf(stderr,"RobotController: Warning, robot has no joint velocity sensor\n");
    fprintf(stderr,"Sensor list:\n");
    for(size_t i=0;i<sensors->sensors.size();i++)
      fprintf(stderr,"  %s: %s\n",sensors->sensors[i]->Type(),sensors->sensors[i]->name.c_str());
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

shared_ptr<RobotController> RobotControllerFactory::CreateByName(const char* name,Robot& robot)
{
  for(map<std::string,shared_ptr<RobotController> >::iterator i=controllers.begin();i!=controllers.end();i++)
    if(i->first == name && &i->second->robot==&robot) return i->second;
  return NULL;
}

shared_ptr<RobotController> RobotControllerFactory::Load(TiXmlElement* in,Robot& robot)
{
  if(0!=strcmp(in->Value(),"controller")) {
    fprintf(stderr,"Controller does not have type \"controller\", got %s\n",in->Value());
    return NULL;
  }
  if(in->Attribute("type")==NULL) {
    fprintf(stderr,"Controller does not have \"type\" attribute\n");
    return NULL;
  }
  shared_ptr<RobotController> c = CreateByName(in->Attribute("type"),robot);
  if(!c) {
    fprintf(stderr,"Unable to load controller of type %s\n",in->Attribute("type"));
    fprintf(stderr,"Candidates: \n");
    for(map<std::string,shared_ptr<RobotController> >::iterator i=controllers.begin();i!=controllers.end();i++)
      fprintf(stderr,"  %s\n",i->first.c_str());
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
	fprintf(stderr,"Invalid rate %g\n",temp);
	return NULL;
      }
      else 
	c->nominalTimeStep = 1.0/temp;
    }
    else if(0==strcmp(attr->Name(),"timeStep")) {
      double temp=0;
      if(in->QueryValueAttribute("timeStep",&temp)!=TIXML_SUCCESS || temp <= 0){
	fprintf(stderr,"Invalid timestep %g\n",temp);
	return NULL;
      }
      c->nominalTimeStep = temp;
    }
    else {
      if(!c->SetSetting(attr->Name(),attr->Value())) {
	fprintf(stderr,"Load controller  %s from XML: Unable to set setting %s\n",in->Attribute("type"),attr->Name());
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

shared_ptr<RobotController> RobotControllerFactory::Load(const char* fn,Robot& robot)
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



shared_ptr<RobotController> MakeDefaultController(Robot* robot)
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
  
    printf("MakeDefaultController: could not load controller from data %s\n",controllerXml.c_str());
    printf("  Making the standard controller instead.\n");
    printf("  Press enter to continue.\n");
    getchar();
  }
  auto c = make_shared<PolynomialPathController>(*robot);
  auto fc = make_shared<FeedforwardController>(*robot,c);
  auto lc= make_shared<LoggingController>(*robot,fc);
  //defaults -- gravity compensation is better off with free-floating robots
  if(robot->joints[0].type == RobotJoint::Floating)
    fc->enableGravityCompensation=false;  //feedforward capability
  else
    fc->enableGravityCompensation=true;  //feedforward capability
  fc->enableFeedforwardAcceleration=false;  //feedforward capability
  lc->save = false;
  return lc;
}
