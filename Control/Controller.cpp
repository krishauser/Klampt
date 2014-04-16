#include "Controller.h"
#include "FeedforwardController.h"
#include "LoggingController.h"
#include "PathController.h"
#include "JointTrackingController.h"
#include "SerialController.h"
#include <utils/PropertyMap.h>
#include <tinyxml.h>

RobotController::RobotController(Robot& _robot)
  : robot(_robot),time(0),sensors(NULL),command(NULL)
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
  robot.NormalizeAngles(qdes);
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
  Assert(torques.size()==robot.drivers.size());
  for(size_t i=0;i<robot.drivers.size();i++)
    command->actuators[i].SetTorque(torques[i]);
}

void RobotController::SetFeedforwardPIDCommand(const Config& qdes,const Config& dqdes,const Vector& torques)
{
  Assert(torques.size()==robot.drivers.size());
  SetPIDCommand(qdes,dqdes);
  for(size_t i=0;i<robot.drivers.size();i++)
    command->actuators[i].torque = torques[i];
}

void RobotControllerFactory::RegisterDefault(Robot& robot)
{
  Register("JointTrackingController",new JointTrackingController(robot));
  Register("MilestonePathController",new MilestonePathController(robot));
  Register("PolynomialPathController",new PolynomialPathController(robot));
  Register("FeedforwardJointTrackingController",new FeedforwardController(robot,new JointTrackingController(robot)));
  Register("FeedforwardMilestonePathController",new FeedforwardController(robot,new MilestonePathController(robot)));
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
    fprintf(stderr,"Controller does not have type \"controller\", got %s\n",in->Value());
    return NULL;
  }
  if(in->Attribute("type")==NULL) {
    fprintf(stderr,"Controller does not have \"type\" attribute\n");
    return NULL;
  }
  SmartPointer<RobotController> c = CreateByName(in->Attribute("type"),robot);
  if(!c) {
    fprintf(stderr,"Unable to load controller of type %s\n",in->Attribute("type"));
    fprintf(stderr,"Candidates: \n");
    for(map<std::string,SmartPointer<RobotController> >::iterator i=controllers.begin();i!=controllers.end();i++)
      fprintf(stderr,"  %s\n",i->first.c_str());
    return NULL;
  }
  TiXmlAttribute* attr = in->FirstAttribute();
  while(attr != NULL) {
    if(0==strcmp(attr->Name(),"type")) {
      attr = attr->Next();
      continue;
    }
    if(!c->SetSetting(attr->Name(),attr->Value())) {
      fprintf(stderr,"Load controller  %s from XML: Unable to set setting %s\n",in->Attribute("type"),attr->Name());
      return NULL;
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

map<std::string,SmartPointer<RobotController> > RobotControllerFactory::controllers;
