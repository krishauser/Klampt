#ifndef XML_ODE_H
#define XML_ODE_H

#include <tinyxml.h>
#include <Klampt/Simulation/ODESimulator.h>
#include <Klampt/Simulation/Simulator.h>

namespace Klampt {

class XmlODEGeometry
{
 public:
  XmlODEGeometry(TiXmlElement* element);
  bool Get(ODEGeometry& geom);

  TiXmlElement* e;
};

class XmlODESettings
{
 public:
  XmlODESettings(TiXmlElement* element);
  bool GetSettings(ODESimulator& sim);

  TiXmlElement* e;
};

class XmlSimulationSettings
{
 public:
  XmlSimulationSettings(TiXmlElement* element);
  bool GetSettings(Simulator& sim);

  TiXmlElement* e;
};

} //namespace Klampt

#endif
