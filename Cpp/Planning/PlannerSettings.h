#ifndef PLANNER_SETTINGS_H
#define PLANNER_SETTINGS_H

#include <Klampt/Modeling/World.h>
#include <KrisLibrary/structs/array2d.h>
#include <KrisLibrary/geometry/CollisionMesh.h>
#include <KrisLibrary/utils/PropertyMap.h>

namespace Klampt {

struct RobotPlannerSettings
{
  Real collisionEpsilon;   ///<threshold for edge feasibility checks
  Vector distanceWeights;  ///<for non-euclidean distance metric
  AABB3D worldBounds;      ///<base position sampling range for free-floating robots
  Real contactEpsilon;     ///<convergence threshold for contact solving
  int contactIKMaxIters;   ///<max iters for contact solving
  PropertyMap properties;  ///<other properties
};

struct ObjectPlannerSettings
{
  bool touchable;          ///<true if its allowed to be touched
  Real collisionEpsilon;   ///<threshold for edge feasibility checks
  Real translationWeight, rotationWeight;  ///<for distance metric
  AABB3D worldBounds;      ///<for translation
  PropertyMap properties;  ///<other properties
};

struct TerrainPlannerSettings
{
  bool touchable;          ///<true if its allowed to be touched
  PropertyMap properties;  ///<other properties
};

/** @brief A structure containing settings that should be used for collision
 * detection, contact solving, etc.  Also performs modified collision
 * checking with enabled/disabled collision checking between different objects.
 * 
 * Make sure to call world.UpdateGeometry() before using the CheckCollision and
 * DistanceLowerBound routines.
 */
struct WorldPlannerSettings
{
  WorldPlannerSettings();

  ///Initializes the structure with default settings.
  void InitializeDefault(WorldModel& world);

  ///Checks collision with the given object id1, and optionally a second
  ///object id2.  The default id2=-1 indicates checking with all other
  ///objects.  tol indicates an extra collision margin on top of the
  ///custom collision margins.
  ///
  ///Note that if id1 indicates a robot and id2 is -1, the robot is checked against
  ///for self collisions AND collisions with all non-robot objects
  bool CheckCollision(WorldModel& world,int id1,int id2=-1,Real tol=0);
  ///Same as the other CheckCollision, except a collision geometry is given.
  bool CheckCollision(WorldModel& world,Geometry::AnyCollisionGeometry3D* mesh,int id=-1,Real tol=0);

  ///Checks self-collisions of all objects in a set of objects. 
  ///tol indicates an extra collision margin on top of the
  ///custom collision margins.  Returns (-1,-1) if no collisions are
  ///found, or the pair of ids of the first colliding objects.
  pair<int,int> CheckCollision(WorldModel& world,const vector<int>& ids,Real tol=0);
  ///Checks collisions between all objects in set 1 against those of set 2.
  ///tol indicates an extra collision margin on top of the
  ///custom collision margins. Returns (-1,-1) if no collisions are
  ///found, or the pair of ids of the first colliding objects.
  pair<int,int> CheckCollision(WorldModel& world,const vector<int>& ids1,const vector<int>& ids2,Real tol=0);

  ///Returns a distance, with the possibility of early termination if the
  ///closest object is farther than the given bound. eps is a distance error
  ///tolerance passed to the distance query.
  Real DistanceLowerBound(WorldModel& world,int id1,int id2=-1,Real eps=0,Real bound=Inf);
  ///Same as the other DistanceLowerBound, except a collision geometry is
  ///given.
  Real DistanceLowerBound(WorldModel& world,Geometry::AnyCollisionGeometry3D* mesh,int id,Real eps=0,Real bound=Inf);

  ///Returns a self-distance for all pairs in a set of objects, with the possibility of
  ///early termination if the closest pair of objects in the set is farther
  ///than the given bound.  If closest1 and closest2 != NULL, they are set to the object
  ///pair that yielded the minimum distance.
  Real DistanceLowerBound(WorldModel& world,const vector<int>& ids,Real eps=0,Real bound=Inf,int* closest1=NULL,int* closest2=NULL);
  ///Returns a distance between two sets of objects, with the possibility of
  ///early termination if the closest pair of objects in the two sets is farther
  ///than the given bound.  If closest1 and closest2 != NULL, they are set to the object
  ///pair that yielded the minimum distance.
  Real DistanceLowerBound(WorldModel& world,const vector<int>& ids1,const vector<int>& ids2,Real eps=0,Real bound=Inf,int* closest1=NULL,int* closest2=NULL);

  ///Returns a list of object IDs that can potentially collide
  void EnumerateCollisionPairs(WorldModel& world,vector<pair<int,int> >& pairs) const;
  ///Enumerates all potentially colliding pairs of ids contained in 
  ///the given objects, including robot self-collisions.  id1=-1,id2=-1 may
  ///also be provided to retrieve all collision pairs.
  void EnumerateCollisionQueries(WorldModel& world,int id1,int id2,
				 vector<pair<int,int> >& pairs,
				 vector<Geometry::AnyCollisionQuery>& queries);
  ///Same as EnumerateCollisionQueries, but a collision geometry is given.
  void EnumerateCollisionQueries(WorldModel& world,Geometry::AnyCollisionGeometry3D* mesh,int id,
				 vector<int>& checkedIDs,
				 vector<Geometry::AnyCollisionQuery>& queries);

  Array2D<bool> collisionEnabled;    //indexed by world ID #
  vector<RobotPlannerSettings> robotSettings;
  vector<ObjectPlannerSettings> objectSettings;
  vector<TerrainPlannerSettings> terrainSettings;
};

} // namespace Klampt

#endif
