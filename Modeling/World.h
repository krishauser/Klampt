#ifndef ROBOT_WORLD_H
#define ROBOT_WORLD_H

#include "Robot.h"
#include "Environment.h"
#include "RigidObject.h"
#include "View/ViewRobot.h"
#include "View/ViewEnvironment.h"
#include "View/ViewRigidObject.h"
#include <camera/camera.h>
#include <camera/viewport.h>
#include <GLdraw/GLLight.h>
#include <utils/SmartPointer.h>

struct RobotInfo
{
  string name;
  SmartPointer<Robot> robot;
  ViewRobot view;
};

struct TerrainInfo
{
  string name;
  SmartPointer<Environment> terrain;
  ViewEnvironment view;
};

struct RigidObjectInfo
{
  string name;
  SmartPointer<RigidObject> object;
  ViewRigidObject view;
};

/** @ingroup Modeling
 * @brief The main world class containing multiple robots, objects, and
 * static geometries.  Lights and other viewport information may also be
 * stored here.
 */
class RobotWorld
{
 public:
  RobotWorld();
  bool LoadXML(const char* fn);
  bool SaveXML(const char* fn,const char* elementDir);
  void InitCollisions();
  void UpdateGeometry();
  void SetGLLights();
  void DrawGL();

  //integer id's for objects in the world
  int NumIDs() const;
  int GetID(const string& name,int link=-1) const;
  string GetName(int id) const;
  ///Returns the index of the terrain or -1 otherwise
  int IsTerrain(int id) const;
  ///Returns the index of the rigid object or -1 otherwise
  int IsRigidObject(int id) const;
  ///Returns the index of the robot or -1 otherwise
  int IsRobot(int id) const;
  ///Returns the index of the robot link or -1,-1 otherwise
  pair<int,int> IsRobotLink(int id) const;
  int TerrainID(int index) const;
  int RigidObjectID(int index) const;
  int RobotID(int index) const;
  int RobotLinkID(int index,int link) const;
  Geometry::AnyCollisionGeometry3D& GetGeometry(int id);
  const Geometry::AnyCollisionGeometry3D& GetGeometry(int id) const;
  GLDraw::GeometryAppearance& GetAppearance(int id);
  RigidTransform GetTransform(int id) const;
  void SetTransform(int id,const RigidTransform& T);

  int LoadRobot(const string& fn);
  int AddRobot(const string& name,Robot* robot=NULL);
  void DeleteRobot(const string& name);
  Robot* GetRobot(const string& name);
  ViewRobot* GetRobotView(const string& name);

  int LoadTerrain(const string& fn);
  int AddTerrain(const string& name,Environment* terrain=NULL);
  void DeleteTerrain(const string& name);
  Environment* GetTerrain(const string& name);
  ViewEnvironment* GetTerrainView(const string& name);

  int LoadRigidObject(const string& fn);
  int AddRigidObject(const string& name,RigidObject* obj=NULL);
  void DeleteRigidObject(const string& name);
  RigidObject* GetRigidObject(const string& name);
  ViewRigidObject* GetRigidObjectView(const string& name);

  RobotInfo* ClickRobot(const Ray3D& r,int& body,Vector3& localpt);
  RigidObjectInfo* ClickObject(const Ray3D& r,Vector3& localpt);

  ///Loads an element from the file, using its extension to figure out what
  ///type it is.
  int LoadElement(const string& fn);
  ///Returns true if the given extension is loadable as an element
  bool CanLoadElementExt(const char* ext) const;

  //viewport info
  Camera::Camera camera;
  Camera::Viewport viewport;
  vector<GLDraw::GLLight> lights;
  GLDraw::GLColor background;

  //world occupants
  vector<RobotInfo> robots;
  vector<TerrainInfo> terrains;
  vector<RigidObjectInfo> rigidObjects;
};

void CopyWorld(const RobotWorld& a,RobotWorld& b);

#endif
