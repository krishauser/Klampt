#include "XmlODE.h"
#include "Control/Controller.h"
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/utils/ioutils.h>
#include <sstream>

DEFINE_LOGGER(XmlParser);

namespace Klampt {

int SafeQueryFloat(TiXmlElement* e,const char* attr,double& out)
{
  if(e->Attribute(attr)) {
    std::stringstream ss(e->Attribute(attr));
    if(SafeInputFloat(ss,out)) 
      return TIXML_SUCCESS;
    LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Error reading <"<<e->Value()<<">  attribute "<<attr);
    return TIXML_WRONG_TYPE;
  }
  return TIXML_NO_ATTRIBUTE;
}

XmlODEGeometry::XmlODEGeometry(TiXmlElement* _element)
  :e(_element)
{}

bool XmlODEGeometry::Get(ODEGeometry& mesh)
{
  //const char* fn = e->Attribute("file");
  double padding;
  int preshrink;
  if(e->QueryValueAttribute("padding",&padding)==TIXML_SUCCESS) {
    if(e->QueryValueAttribute("preshrink",&preshrink)==TIXML_SUCCESS && preshrink!=0) {
      if(preshrink == 2)
        mesh.SetPaddingWithPreshrink(padding,true);
      else
        mesh.SetPaddingWithPreshrink(padding,false);
    }
    else {
      mesh.SetPadding(padding);
    }
  }
  SafeQueryFloat(e,"kFriction",mesh.surf().kFriction);
  SafeQueryFloat(e,"kRestitution",mesh.surf().kRestitution);
  SafeQueryFloat(e,"kStiffness",mesh.surf().kStiffness);
  SafeQueryFloat(e,"kDamping",mesh.surf().kDamping);
  return true;
}



bool ParseRef(TiXmlElement* e,ODESimulator& sim,ODEObjectID& o,const char* context="<simulation>")
{
  if(0 == strcmp(e->Value(),"robot")) {
    o.type = 1;
    if(e->Attribute("name")) {
      string fn = string(e->Attribute("name"));
      for(size_t i=0;i<sim.numRobots();i++) {
        if(fn == sim.robot(i)->robot.name) {
          o.index = i;
          break;
        }
      }
      if(o.index < 0) {
        LOG4CXX_WARN(GET_LOGGER(XmlParser),"Error reading XML file, "<<context<<"<robot> name attribute specifies an invalid robot");
        return false;
      }
    }
    else if(e->QueryValueAttribute("index",&o.index)!=TIXML_SUCCESS) {
      LOG4CXX_WARN(GET_LOGGER(XmlParser),"Error reading XML file, "<<context<<"<robot> must have the name or index attributes");
      return false;
    }
    else if(o.index < 0 || o.index >= (int)sim.numRobots()) {
      LOG4CXX_WARN(GET_LOGGER(XmlParser),"Error reading XML file, "<<context<<"<robot> index specifies an invalid robot");
      return false; 
    }
    if(e->QueryValueAttribute("body",&o.bodyIndex)!=TIXML_SUCCESS) {
      LOG4CXX_WARN(GET_LOGGER(XmlParser),"Error reading XML file, "<<context<<"<robot> must have the body attribute");
      return false;
    }
    if(o.bodyIndex < 0 || o.bodyIndex >= sim.robot(o.index)->robot.q.n) {
      LOG4CXX_WARN(GET_LOGGER(XmlParser),"Error reading XML file, "<<context<<"<robot> body specifies an invalid link");
      return false;
    }
    return true;
  }
  else if(0 == strcmp(e->Value(),"object") || 0 == strcmp(e->Value(),"rigidObject")) {
    o.type = 2;
    if(e->QueryValueAttribute("index",&o.index)==TIXML_SUCCESS) {
      if(o.index < 0 || o.index >= (int)sim.numObjects()) {
        LOG4CXX_WARN(GET_LOGGER(XmlParser),"Error reading XML file, "<<context<<"<object> index specifies an invalid object");
        return false;
      }
      return true;
    }
    else if(e->Attribute("name")) {
      string fn=string(e->Attribute("name"));
      for(size_t i=0;i<sim.numObjects();i++) {
        if(fn == sim.object(i)->obj.name) {
          o.index = i;
          break;
        }
      }
      if(o.index < 0) {
        LOG4CXX_WARN(GET_LOGGER(XmlParser),"Error reading XML file, "<<context<<"<object> name attribute specifies an invalid object");
        return false;
      }
      return true;
    }
    else {
      LOG4CXX_WARN(GET_LOGGER(XmlParser),"Error reading XML file, "<<context<<"<object> must have the name or index attributes");
    }
  }
  else if(0 == strcmp(e->Value(),"terrain")) {
    o.type = 0;
    if(e->QueryValueAttribute("index",&o.index)==TIXML_SUCCESS) {
      if(o.index < 0 || o.index >= (int)sim.numTerrains()) {
        LOG4CXX_WARN(GET_LOGGER(XmlParser),"Error reading XML file, "<<context<<"<terrain> index specifies an invalid terrain");
        return false;
      }
      return true;
    }
    else if(e->Attribute("name")) {
      string fn=string(e->Attribute("name"));
      for(size_t i=0;i<sim.numTerrains();i++) {
        if(fn == sim.terrain(i)->name) {
          o.index = i;
          break;
        }
      }
      if(o.index < 0) {
        LOG4CXX_WARN(GET_LOGGER(XmlParser),"Error reading XML file, "<<context<<"<terrain> name attribute specifies an invalid object");
        return false;
      }
      return true;
    }
    else {
      LOG4CXX_WARN(GET_LOGGER(XmlParser),"Error reading XML file, "<<context<<"<terrain> must have the name or index attributes");
    }
  }
  LOG4CXX_WARN(GET_LOGGER(XmlParser),"Error reading XML file, invalid tag "<<e->Value()<<" in context "<<context);
  return false;
}


XmlODESettings::XmlODESettings(TiXmlElement* _element)
  :e(_element)
{}

bool XmlODESettings::GetSettings(ODESimulator& sim)
{
  TiXmlElement* c = e->FirstChildElement();
  while(c!=NULL) {
    const char* name=c->Value();
    if(0 == strcmp(name,"globals")) {
      //parse globals: gravity, etc
      Vector3 gravity;
      if(c->QueryValueAttribute("gravity",&gravity)==TIXML_SUCCESS)
        sim.SetGravity(gravity);
      double worldERP,worldCFM;
      if(c->QueryValueAttribute("ERP",&worldERP)==TIXML_SUCCESS)
        sim.SetERP(worldERP);
      if(c->QueryValueAttribute("CFM",&worldCFM)==TIXML_SUCCESS)
        sim.SetCFM(worldCFM);
      int maxContacts;
      if(c->QueryValueAttribute("maxContacts",&maxContacts)==TIXML_SUCCESS)
        sim.GetSettings().maxContacts = maxContacts;
      int boundaryLayer,adaptiveTimeStepping,rigidObjectCollisions,robotSelfCollisions,robotRobotCollisions;
      if(c->QueryValueAttribute("boundaryLayer",&boundaryLayer)==TIXML_SUCCESS) {
        LOG4CXX_WARN(GET_LOGGER(XmlParser),"Boundary layer settings don't have an effect after world is loaded");
        sim.GetSettings().boundaryLayerCollisions = (bool)boundaryLayer;
      }
      if(c->QueryValueAttribute("adaptiveTimeStepping",&adaptiveTimeStepping)==TIXML_SUCCESS) {
        sim.GetSettings().adaptiveTimeStepping = (bool)adaptiveTimeStepping;
      }
      if(c->QueryValueAttribute("rigidObjectCollisions",&rigidObjectCollisions)==TIXML_SUCCESS)
        sim.GetSettings().rigidObjectCollisions = (bool)rigidObjectCollisions;
      if(c->QueryValueAttribute("robotSelfCollisions",&robotSelfCollisions)==TIXML_SUCCESS)
        sim.GetSettings().robotSelfCollisions = (bool)robotSelfCollisions;
      if(c->QueryValueAttribute("robotRobotCollisions",&robotRobotCollisions)==TIXML_SUCCESS)
        sim.GetSettings().robotRobotCollisions = (bool)robotRobotCollisions;
    }
    else if(0 == strcmp(name,"terrain") || 0 == strcmp(name,"object")  || 0 == strcmp(name,"rigidObject")  || 0 == strcmp(name,"robot")) {
      ODEObjectID obj;
      if(!ParseRef(c,sim,obj)) return false;
      TiXmlElement* eg=c->FirstChildElement("geometry");
      if(eg) {
        XmlODEGeometry g(eg);
        if(obj.IsRobot()) {
          ODERobot* robot=sim.robot(obj.index);
          if(obj.bodyIndex < 0) {
            for(size_t i=0;i<robot->robot.links.size();i++) {
              if(robot->triMesh(i))
                g.Get(*robot->triMesh(i));
            }
          }
          else {
            if(robot->triMesh(obj.bodyIndex))
              g.Get(*robot->triMesh(obj.bodyIndex));
          }
        }
        else if(obj.IsRigidObject()) {
          if(!g.Get(*sim.object(obj.index)->triMesh())) {
            LOG4CXX_WARN(GET_LOGGER(XmlParser),"Error reading <simulation><"<<name<<"> geometry from XML file");
            return false;
          }
        }
        else if(obj.IsEnv()) {
          if(!g.Get(*sim.terrainGeom(obj.index))) {
            LOG4CXX_WARN(GET_LOGGER(XmlParser),"Error reading <simulation><"<<name<<"> geometry from XML file");
            return false;
          }
        }
        else {
          LOG4CXX_ERROR(GET_LOGGER(XmlParser),"INVALID OBJECT REF? "<<name<<" "<<obj.type<<" "<<obj.index);
          return false;
        }
      }
      TiXmlElement *ev=c->FirstChildElement("velocity");
      if(ev) {
        Vector3 v,w;
        if(ev->QueryValueAttribute("linear",&v) != TIXML_SUCCESS)
          v.setZero();
        if(ev->QueryValueAttribute("angular",&w) != TIXML_SUCCESS)
          w.setZero();
        LOG4CXX_INFO(GET_LOGGER(XmlParser),"Setting object "<<obj.index<<" velocity "<<w<<", "<<v);
        if(obj.IsRigidObject())
          sim.object(obj.index)->SetVelocity(w,v);
        else {
          LOG4CXX_WARN(GET_LOGGER(XmlParser),"Can't set velocity of <simulation><"<<name<<">, only rigid objects are supported");
          return false;
        }
      }
    }
    else if(0 == strcmp(name,"joint")) {
      TiXmlElement* o1 = c->FirstChildElement();
      if(o1 == NULL) {
        LOG4CXX_WARN(GET_LOGGER(XmlParser),"Error reading XML file, <simulation><joint> must have at least one child");
        return false;
      }
      TiXmlElement* o2 = o1->NextSiblingElement();
      ODEObjectID a,b;
      if(!ParseRef(o1,sim,a,"<simulation><joint>")) return false;
      if(o2) {
        if(!ParseRef(o2,sim,b,"<simulation><joint>")) return false;
      }
      const char* type = c->Attribute("type");
      if(type == NULL) {
        LOG4CXX_WARN(GET_LOGGER(XmlParser),"Error reading XML file, <simulation><joint> must have \"type\" attribute");
        return false;
      }
      ODEJoint* joint;
      if(0 == strcmp(type,"hinge")) {
        Vector3 pt,axis;
        if(c->QueryValueAttribute("point",&pt) != TIXML_SUCCESS || c->QueryValueAttribute("axis",&axis) != TIXML_SUCCESS) {
          LOG4CXX_WARN(GET_LOGGER(XmlParser),"Error reading XML file, <simulation><joint type=hinge> must have \"point\" and \"axis\" attributes");
          return false;
        }
        joint=sim.AddJoint(a,b);
        joint->MakeHinge(pt,axis);
      }
      else if(0 == strcmp(type,"slider")) {
        Vector3 axis;
        if(c->QueryValueAttribute("axis",&axis) != TIXML_SUCCESS) {
          LOG4CXX_WARN(GET_LOGGER(XmlParser),"Error reading XML file, <simulation><joint type=slider> must have \"axis\" attribute");
          return false;
        }
        joint=sim.AddJoint(a,b);
        joint->MakeSlider(axis);
      }
      else if(0 == strcmp(type,"fixed")) {
        joint=sim.AddJoint(a,b);
        joint->MakeFixed();
      }
      else {
        LOG4CXX_WARN(GET_LOGGER(XmlParser),"Error reading XML file, <simulation><joint> invalid \"type\" attribute");
        return false;
      }
      Real pmin=-Inf,pmax=Inf;
      bool hasLimits = false;
      if(c->QueryValueAttribute("min",&pmin) == TIXML_SUCCESS || c->QueryValueAttribute("minimum",&pmin) == TIXML_SUCCESS) hasLimits=true;
      if(c->QueryValueAttribute("max",&pmax) == TIXML_SUCCESS || c->QueryValueAttribute("minimax",&pmax) == TIXML_SUCCESS) hasLimits=true;
      if(hasLimits)
        joint->SetLimits(pmin,pmax);
      Real friction;
      if(c->QueryValueAttribute("friction",&friction) == TIXML_SUCCESS)
        joint->SetFriction(friction);
    }
    else if(0==strcmp(name,"state")) {
    }
    else {
      LOG4CXX_WARN(GET_LOGGER(XmlParser),"Skipping unknown <simulation> XML element "<<name);
    }
    c=c->NextSiblingElement();
  }
  return true;
}



XmlSimulationSettings::XmlSimulationSettings(TiXmlElement* _element)
  :e(_element)
{}

bool XmlSimulationSettings::GetSettings(Simulator& sim)
{
  LOG4CXX_INFO(GET_LOGGER(XmlParser),"Parsing XML file <simulation> tag...");
  string globals="globals";
  TiXmlElement* c = e->FirstChildElement(globals);
  if(c) {
    //parse timestep
    SafeQueryFloat(c,"timestep",sim.simStep);
  }
  XmlODESettings ode(e);
  if(!ode.GetSettings(sim.odesim)) {
    return false;
  }


  c = e->FirstChildElement("robot");
  if(c)
    LOG4CXX_INFO(GET_LOGGER(XmlParser),"Parsing XML <simulation><robot> controllers / sensors");
  while(c != NULL) {
    ODEObjectID obj;
    if(!ParseRef(c,sim.odesim,obj)) return false;
    int index = obj.index;
    Assert(index < (int)sim.robotControllers.size());
        
    SimRobotController& robotSim=sim.controlSimulators[index];
    TiXmlElement* ec=c->FirstChildElement("controller");
    if(ec) {
      RobotControllerFactory::RegisterDefault(*robotSim.robot);
      shared_ptr<RobotController> controller=RobotControllerFactory::Load(ec,*robotSim.robot);
      if(controller)
        sim.SetController(index,controller); 
      else {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Unable to load controller from xml file");
        return false;
      }
      if(controller->nominalTimeStep > 0)
        sim.controlSimulators[index].controlTimeStep = controller->nominalTimeStep;
    }
    TiXmlElement*es=c->FirstChildElement("sensors");
    if(es) {
      if(!sim.controlSimulators[index].sensors.LoadSettings(es)) {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Unable to load sensors from xml file");
        return false;
      }
    }
    c = c->NextSiblingElement("robot");
  }

  c = e->FirstChildElement("state");
  if(c) {
    LOG4CXX_INFO(GET_LOGGER(XmlParser),"Parsing XML state");
    const char* data=c->Attribute("data");
    if(!data) {
      fprintf(stderr,"No 'data' attribute in state\n");
      return false;
    }
    string decoded=FromBase64(data);
    if(!sim.ReadState(decoded)) {
      fprintf(stderr,"Unable to read state from data\n");
      return false;
    }
  }

  return true;
}

} //namespace Klampt