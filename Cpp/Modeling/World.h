#ifndef KLAMPT_WORLD_H
#define KLAMPT_WORLD_H

#include "Robot.h"
#include "Terrain.h"
#include "RigidObject.h"
#include <Klampt/View/ViewRobot.h>
#include <KrisLibrary/camera/camera.h>
#include <KrisLibrary/camera/viewport.h>
#include <KrisLibrary/GLdraw/GLLight.h>

namespace Klampt {

/** @file World.h
 * @ingroup Modeling
 * @brief Defines the WorldModel class.
 */

/** @ingroup Modeling
 * @brief The main world class containing multiple robots, objects, and
 * static geometries (terrains).  Lights and other viewport information
 * may also be stored here.
 */
class WorldModel
{
 public:
  typedef shared_ptr<Geometry::AnyCollisionGeometry3D> GeometryPtr;
  typedef shared_ptr<GLDraw::GeometryAppearance> AppearancePtr;

  WorldModel();
  ///Loads from an XML file.  fn also be a URL if libcurl support is
  ///enabled.
  bool LoadXML(const char* fn);
  ///Saves into an XML file, and all referenced objects are placed in elementDir.
  ///If elementDir==NULL, they will be saved to a folder that has the same base
  ///name as fn.
  bool SaveXML(const char* fn,const char* elementDir=NULL);
  /** @brief Performs a shallow copy of a WorldModel.  Since it does not copy geometry,
   * this operation is very fast.
   */
  void Copy(const WorldModel& other);
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
  int AddRobot(const string& name,RobotModel* robot=NULL);
  void DeleteRobot(const string& name);
  RobotModel* GetRobot(const string& name);
  ViewRobot* GetRobotView(const string& name);

  int LoadTerrain(const string& fn);
  int AddTerrain(const string& name,TerrainModel* terrain=NULL);
  void DeleteTerrain(const string& name);
  TerrainModel* GetTerrain(const string& name);

  int LoadRigidObject(const string& fn);
  int AddRigidObject(const string& name,RigidObjectModel* obj=NULL);
  void DeleteRigidObject(const string& name);
  RigidObjectModel* GetRigidObject(const string& name);

  ///Returns the ID of the entity the ray hits, or -1 if nothing was hit.  Returns hit point *in world frame*.
  int RayCast(const Ray3D& r,Vector3& worldpt);
  ///Same as RayCast but ignores some IDs (see TerrainID, RigidObjectID, RobotID, RobotLinkID)
  int RayCastIgnore(const Ray3D& r,const vector<int>& ignoreIDs,Vector3& worldpt);
  ///Same as RayCast but only checks specified IDs (see TerrainID, RigidObjectID, RobotID, RobotLinkID)
  int RayCastSelected(const Ray3D& r,const vector<int>& selectedIDs,Vector3& worldpt);
  ///Ray casts only robots.  Returns hit robot, link, and point *in local frame*.
  RobotModel* RayCastRobot(const Ray3D& r,int& body,Vector3& localpt);
  ///Ray casts only objects.  Returns hit object and point *in local frame*.
  RigidObjectModel* RayCastObject(const Ray3D& r,Vector3& localpt);

  ///Loads an element from the file fn, using its extension to figure out
  ///what type it is.  fn also be a URL if libcurl support is
  ///enabled.
  int LoadElement(const string& fn);
  ///Returns true if the given extension is loadable as an element
  bool CanLoadElementExt(const char* ext) const;

  //viewport info
  Camera::Camera camera;
  Camera::Viewport viewport;
  vector<GLDraw::GLLight> lights;
  GLDraw::GLColor background;

  //world occupants
  vector<shared_ptr<RobotModel> > robots;
  vector<shared_ptr<TerrainModel> > terrains;
  vector<shared_ptr<RigidObjectModel> > rigidObjects;

  vector<ViewRobot> robotViews;
};

} //namespace Klampt

#endif
