#ifndef ROBOT_WORLD_H
#define ROBOT_WORLD_H

#include "Robot.h"
#include "Terrain.h"
#include "RigidObject.h"
#include <Klampt/View/ViewRobot.h>
#include <KrisLibrary/camera/camera.h>
#include <KrisLibrary/camera/viewport.h>
#include <KrisLibrary/GLdraw/GLLight.h>

/** @file World.h
 * @ingroup Modeling
 * @brief Defines the RobotWorld class.
 */

/** @ingroup Modeling
 * @brief The main world class containing multiple robots, objects, and
 * static geometries (terrains).  Lights and other viewport information
 * may also be stored here.
 */
class RobotWorld
{
 public:
  typedef shared_ptr<Geometry::AnyCollisionGeometry3D> GeometryPtr;
  typedef shared_ptr<GLDraw::GeometryAppearance> AppearancePtr;

  RobotWorld();
  bool LoadXML(const char* fn);
  bool SaveXML(const char* fn,const char* elementDir=NULL);
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
  GeometryPtr GetGeometry(int id);
  AppearancePtr GetAppearance(int id);
  RigidTransform GetTransform(int id) const;
  void SetTransform(int id,const RigidTransform& T);

  int LoadRobot(const string& fn);
  int AddRobot(const string& name,Robot* robot=NULL);
  void DeleteRobot(const string& name);
  Robot* GetRobot(const string& name);
  ViewRobot* GetRobotView(const string& name);

  int LoadTerrain(const string& fn);
  int AddTerrain(const string& name,Terrain* terrain=NULL);
  void DeleteTerrain(const string& name);
  Terrain* GetTerrain(const string& name);

  int LoadRigidObject(const string& fn);
  int AddRigidObject(const string& name,RigidObject* obj=NULL);
  void DeleteRigidObject(const string& name);
  RigidObject* GetRigidObject(const string& name);

  ///Returns the ID of the entity the ray hits, or -1 if nothing was hit
  int RayCast(const Ray3D& r,Vector3& worldpt);
  Robot* RayCastRobot(const Ray3D& r,int& body,Vector3& localpt);
  RigidObject* RayCastObject(const Ray3D& r,Vector3& localpt);

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
  vector<shared_ptr<Robot> > robots;
  vector<shared_ptr<Terrain> > terrains;
  vector<shared_ptr<RigidObject> > rigidObjects;

  vector<ViewRobot> robotViews;
};

/** @ingroup Modeling
 * @brief Performs a shallow copy of a RobotWorld.  Since it does not copy geometry,
 * this operation is very fast.
 */
void CopyWorld(const RobotWorld& a,RobotWorld& b);

#endif
