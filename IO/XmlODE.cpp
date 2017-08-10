#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "XmlODE.h"
#include "Control/Controller.h"
#include <KrisLibrary/utils/stringutils.h>


XmlODEGeometry::XmlODEGeometry(TiXmlElement* _element)
  :e(_element)
{}

bool XmlODEGeometry::Get(ODEGeometry& mesh)
{
  //const char* fn = e->Attribute("file");
  double padding,temp;
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
  if(e->QueryValueAttribute("kFriction",&temp)==TIXML_SUCCESS) {
    mesh.surf().kFriction=temp;
  }
  if(e->QueryValueAttribute("kRestitution",&temp)==TIXML_SUCCESS) {
    mesh.surf().kRestitution=temp;
  }
  if(e->QueryValueAttribute("kStiffness",&temp)==TIXML_SUCCESS) {
    mesh.surf().kStiffness=temp;
  }
  if(e->QueryValueAttribute("kDamping",&temp)==TIXML_SUCCESS) {
    mesh.surf().kDamping=temp;
  }
  return true;
}





XmlODESettings::XmlODESettings(TiXmlElement* _element)
  :e(_element)
{}

bool XmlODESettings::GetSettings(ODESimulator& sim)
{
  string globals="globals";
  TiXmlElement* c = e->FirstChildElement(globals);
  if(c) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Parsing globals...\n");
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
      LOG4CXX_WARN(KrisLibrary::logger(),"XML simulator: warning, boundary layer settings don't have an effect after world is loaded\n");
      sim.GetSettings().boundaryLayerCollisions = boundaryLayer;
    }
    if(c->QueryValueAttribute("adaptiveTimeStepping",&adaptiveTimeStepping)==TIXML_SUCCESS) {
      sim.GetSettings().adaptiveTimeStepping = adaptiveTimeStepping;
    }
    if(c->QueryValueAttribute("rigidObjectCollisions",&rigidObjectCollisions)==TIXML_SUCCESS)
      sim.GetSettings().rigidObjectCollisions = rigidObjectCollisions;
    if(c->QueryValueAttribute("robotSelfCollisions",&robotSelfCollisions)==TIXML_SUCCESS)
      sim.GetSettings().robotSelfCollisions = robotSelfCollisions;
    if(c->QueryValueAttribute("robotRobotCollisions",&robotRobotCollisions)==TIXML_SUCCESS)
      sim.GetSettings().robotRobotCollisions = robotRobotCollisions;
  }
  else c=e->FirstChildElement();

  while(c!=NULL) {
    const char* name=c->Value();
    LOG4CXX_INFO(KrisLibrary::logger(),"Parsing element "<<name);
    if(0 == strcmp(name,"terrain")) {
      int index;
      if(c->QueryValueAttribute("index",&index)==TIXML_SUCCESS) {
	Assert(index < (int)sim.numTerrains());
	TiXmlElement* eg=c->FirstChildElement("geometry");
	if(eg) {
	  XmlODEGeometry g(eg);
	  if(!g.Get(*sim.terrainGeom(index))) {
	    	    LOG4CXX_ERROR(KrisLibrary::logger(),"Error reading terrain geometry from XML\n");
	    return false;
	  }
	}
      }
      else {
		LOG4CXX_ERROR(KrisLibrary::logger(),"Error reading terrain index from XML file\n");
	return false;
      }
    }
    else if(0 == strcmp(name,"object")) {
      int index;
      if(c->QueryValueAttribute("index",&index)==TIXML_SUCCESS) {
	Assert(index < (int)sim.numObjects());
	TiXmlElement* eg=c->FirstChildElement("geometry");
	if(eg) {
	  XmlODEGeometry g(eg);
	  if(!g.Get(*sim.object(index)->triMesh())) {
	    	    LOG4CXX_ERROR(KrisLibrary::logger(),"Error reading object geometry from XML\n");
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
	  LOG4CXX_INFO(KrisLibrary::logger(),"Setting velocity "<<w<<", "<<v<<"\n");
	  sim.object(index)->SetVelocity(w,v);
	}
      }
      else {
		LOG4CXX_ERROR(KrisLibrary::logger(),"Error reading object index from XML file\n");
	return false;
      }
    }
    else if(0 == strcmp(name,"robot")) {
      int index,bodyIndex=-1;
      if(c->QueryValueAttribute("index",&index)==TIXML_SUCCESS) {
	Assert(index < (int)sim.numRobots());
	if(c->QueryValueAttribute("body",&bodyIndex)==TIXML_SUCCESS) {
	}
	else bodyIndex=-1;
	
	ODERobot* robot=sim.robot(index);
	TiXmlElement* eg=c->FirstChildElement("geometry");
	if(eg) {
	  XmlODEGeometry g(eg);
	  if(bodyIndex < 0) {
	    for(size_t i=0;i<robot->robot.links.size();i++) {
	      if(robot->triMesh(i))
		g.Get(*robot->triMesh(i));
	    }
	  }
	  else {
	    Assert(bodyIndex < (int)robot->robot.links.size());
	    if(robot->triMesh(bodyIndex))
	      g.Get(*robot->triMesh(bodyIndex));
	  }
	}
      }
      else {
		LOG4CXX_ERROR(KrisLibrary::logger(),"Error reading robot index from XML file\n");
	return false;
      }
    }
    c=c->NextSiblingElement();
  }
  return true;
}



XmlSimulationSettings::XmlSimulationSettings(TiXmlElement* _element)
  :e(_element)
{}

bool XmlSimulationSettings::GetSettings(WorldSimulation& sim)
{
  string globals="globals";
  TiXmlElement* c = e->FirstChildElement(globals);
  if(c) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Parsing timestep...\n");
    //parse timestep
    double timestep;
    if(c->QueryValueAttribute("timestep",&timestep)==TIXML_SUCCESS)
      sim.simStep = timestep;
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"Parsing ODE...\n");
  XmlODESettings ode(e);
  if(!ode.GetSettings(sim.odesim)) {
    return false;
  }


  LOG4CXX_INFO(KrisLibrary::logger(),"Parsing robot controllers / sensors\n");
  c = e->FirstChildElement("robot");
  while(c != NULL) {
    int index;
    if(c->QueryValueAttribute("index",&index)!=TIXML_SUCCESS) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to read index of robot element\n");
      continue;
    }
    Assert(index < (int)sim.robotControllers.size());
	
    ControlledRobotSimulator& robotSim=sim.controlSimulators[index];
    TiXmlElement* ec=c->FirstChildElement("controller");
    if(ec) {
      RobotControllerFactory::RegisterDefault(*robotSim.robot);
      SmartPointer<RobotController> controller=RobotControllerFactory::Load(ec,*robotSim.robot);
      if(controller)
	sim.SetController(index,controller); 
      else {
		LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to load controller from xml file\n");
	return false;
      }
      if(controller->nominalTimeStep > 0)
	sim.controlSimulators[index].controlTimeStep = controller->nominalTimeStep;
    }
    TiXmlElement*es=c->FirstChildElement("sensors");
    if(es) {
      if(!sim.controlSimulators[index].sensors.LoadSettings(es)) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to load sensors from xml file\n");
	return false;
      }
    }

    /*
    TiXmlElement* es=c->FirstChildElement("sensors");
    if(es) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Parsing sensors...\n");
      TiXmlElement* pos=es->FirstChildElement("position");
      int ival;
      Real fval;
      if(pos) {
	if(pos->QueryValueAttribute("enabled",&ival)==TIXML_SUCCESS) {
	  if(bodyIndex >= 0) 
	    	    LOG4CXX_ERROR(KrisLibrary::logger(),"Warning: cannot enable individual joint encoders yet\n");
	  robotSim.sensors.hasJointPosition=(ival!=0);
	}
	if(pos->QueryValueAttribute("variance",&fval)==TIXML_SUCCESS) {
	  if(robotSim.sensors.qvariance.n==0)
	    robotSim.sensors.qvariance.resize(robotSim.robot->q.n,Zero);
	  if(bodyIndex >= 0)
	    robotSim.sensors.qvariance(bodyIndex)=fval;
	  else
	    robotSim.sensors.qvariance.set(fval);
	}
	if(pos->QueryValueAttribute("resolution",&fval)==TIXML_SUCCESS) {
	  if(robotSim.sensors.qresolution.n==0)
	    robotSim.sensors.qresolution.resize(robotSim.robot->q.n,Zero);
	  if(bodyIndex >= 0)
	    robotSim.sensors.qresolution(bodyIndex)=fval;
	  else
	    robotSim.sensors.qresolution.set(fval);
	}
      }
      TiXmlElement* vel=c->FirstChildElement("velocity");
      if(vel) {
	if(vel->QueryValueAttribute("enabled",&ival)==TIXML_SUCCESS) {
	  if(bodyIndex >= 0) 
	    	    LOG4CXX_ERROR(KrisLibrary::logger(),"Warning: cannot enable individual joint encoders yet\n");
	  robotSim.sensors.hasJointVelocity=(ival!=0);
	}
	if(vel->QueryValueAttribute("variance",&fval)==TIXML_SUCCESS) {
	  if(robotSim.sensors.dqvariance.n==0)
	    robotSim.sensors.dqvariance.resize(robotSim.robot->q.n,Zero);
	  if(bodyIndex >= 0)
	    robotSim.sensors.dqvariance(bodyIndex)=fval;
	  else
	    robotSim.sensors.dqvariance.set(fval);
	}
	if(vel->QueryValueAttribute("resolution",&fval)==TIXML_SUCCESS) {
	  if(robotSim.sensors.dqresolution.n==0)
	    robotSim.sensors.dqresolution.resize(robotSim.robot->q.n,Zero);
	  if(bodyIndex >= 0)
	    robotSim.sensors.dqresolution(bodyIndex)=fval;
	  else
	    robotSim.sensors.dqresolution.set(fval);
	}
      }
      //TODO: other sensors?
    }
    */
    /*
    TiXmlElement* ec=c->FirstChildElement("controller");
    if(ec) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Parsing controller...\n");
      TiXmlAttribute* setting = ec->FirstAttribute();
      while(setting) {
	bool res=robotSim.controller->SetSetting(setting->Name(),setting->Value());
	if(!res) {
	  LOG4CXX_INFO(KrisLibrary::logger(),"Setting "<< or failed parsing\n"	}
	setting = setting->Next();
      }
    }
    */
    c = c->NextSiblingElement("robot");
  }

  LOG4CXX_INFO(KrisLibrary::logger(),"Parsing state\n");
  c = e->FirstChildElement("state");
  if(c) {
    const char* data=c->Attribute("data");
    if(!data) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"No 'data' attribute in state\n");
      return false;
    }
    string decoded=FromBase64(data);
    if(!sim.ReadState(decoded)) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to read state from data\n");
      return false;
    }
  }

  return true;
}

