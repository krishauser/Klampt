#ifndef XML_WORLD_H
#define XML_WORLD_H

#include <tinyxml.h>
#include <Klampt/Modeling/World.h>

namespace Klampt {

/** @defgroup IO
 * I/O routines.  Most of these will be transparent to the user, and you should just use the Robot.Load and World.ReadFile routines.
 * ROS.h and three.js.h might be useful.
 */

class XmlRobot
{
 public:
  XmlRobot(TiXmlElement* element,string path=string());
  bool GetRobot(RobotModel& robot);

  TiXmlElement* e;
  string path;
};


class XmlRigidObject
{
 public:
  XmlRigidObject(TiXmlElement* element,string path=string());
  bool GetRigidObject(RigidObjectModel& object);

  TiXmlElement* e;
  string path;
};

class XmlTerrain
{
 public:
  XmlTerrain(TiXmlElement* element,string path=string());
  bool GetTerrain(TerrainModel& env);

  TiXmlElement* e;
  string path;
};

class XmlWorld
{
 public:
  XmlWorld();
  bool Load(const string& fn);
  bool Load(TiXmlElement* e,string path=string());
  bool GetWorld(WorldModel& world);
  TiXmlElement* GetElement(const string& name);
  TiXmlElement* GetElement(const string& name,int index);
  TiXmlElement* GetRobot(int index) { return GetElement("robot",index); }
  TiXmlElement* GetRigidObject(int index) { return GetElement("rigidObject",index); }
  TiXmlElement* GetTerrain(int index) { return GetElement("terrain",index); }

  ///The save function saves an XML file to fn and saves all robots, rigid objects, and terrains to
  ///.rob, .obj, and .env files to the folder [itempath]/.  If itempath is not provided, then the
  ///path [path]/[worldfile]/ will be used, where fn is of the form "[path]/[worldfile].xml"
  bool Save(WorldModel& world,const string& fn,string itempath=string());

  TiXmlDocument doc;
  TiXmlElement* elem;
  string path;
  Vector3 goals[10];
  int goalCount;
};

} //namespace Klampt

#endif

