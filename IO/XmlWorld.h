#ifndef XML_WORLD_H
#define XML_WORLD_H

#include <tinyxml.h>
#include "Modeling/World.h"

class XmlRobot
{
 public:
  XmlRobot(TiXmlElement* element,string path=string());
  bool GetRobot(Robot& robot);
  //void GetController();

  TiXmlElement* e;
  string path;
};

class XmlRigidObject
{
 public:
  XmlRigidObject(TiXmlElement* element,string path=string());
  bool GetObject(RigidObject& object);
  //void GetGraspGenerator();

  TiXmlElement* e;
  string path;
};

class XmlTerrain
{
 public:
  XmlTerrain(TiXmlElement* element,string path=string());
  bool GetTerrain(Environment& env);

  TiXmlElement* e;
  string path;
};

class XmlWorld
{
 public:
  XmlWorld();
  bool Load(const string& fn);
  bool Load(TiXmlElement* e,string path=string());
  bool GetWorld(RobotWorld& world);
  TiXmlElement* GetElement(const string& name);
  TiXmlElement* GetElement(const string& name,int index);
  TiXmlElement* GetRobot(int index) { return GetElement("robot",index); }
  TiXmlElement* GetRigidObject(int index) { return GetElement("rigidObject",index); }
  TiXmlElement* GetTerrain(int index) { return GetElement("terrain",index); }

  TiXmlDocument doc;
  TiXmlElement* elem;
  string path;
  Vector3 goals[10];
  int goalCount;
};

#endif

