#ifndef PLANNER_SETTINGS_H
#define PLANNER_SETTINGS_H

#include "Modeling/World.h"
#include <structs/array2d.h>
#include <geometry/CollisionMesh.h>

struct RobotPlannerSettings
{
  Real collisionEpsilon;   ///<threshold for edge feasibility checks
  Vector distanceWeights;  ///<for non-euclidean distance metric
  AABB3D worldBounds;      ///<base position sampling range for free-floating robots
  Real contactEpsilon;     ///<convergence threshold for contact solving
  int contactIKMaxIters;   ///<max iters for contact solving
};

struct ObjectPlannerSettings
{
  bool touchable;          ///<true if its allowed to be touched
  Real collisionEpsilon;   ///<threshold for edge feasibility checks
  Real translationWeight, rotationWeight;  ///<for distance metric
  AABB3D worldBounds;      ///<for translation
};

struct TerrainPlannerSettings
{
  bool touchable;          ///<true if its allowed to be touched
};

/** @brief A structure containing settings that should be used for collision
 * detection, contact solving, etc.  Also performs modified collision
 * checking with enabled/disabled collision checking between different objects.
 */
struct WorldPlannerSettings
{
  WorldPlannerSettings();

  ///Initializes the structure with default settings.
  void InitializeDefault(RobotWorld& world);

  ///Checks collision with the given object id1, and optionally a second
  ///object id2.  id2=-1 indicates checking with all other objects.  tol
  ///indicates an extra collision margin on top of the custom collision
  ///margins.
  bool CheckCollision(RobotWorld& world,int id1,int id2=-1,Real tol=0);
  ///Same as the other CheckCollision, except a collision geometry is given.
  bool CheckCollision(RobotWorld& world,Geometry::AnyCollisionGeometry3D& mesh,int id=-1,Real tol=0);

  ///Returns a distance, with the possibility of early termination if the
  ///closest object is farther than the given bound.
  ///@todo collision margins aren't taken into account.
  Real DistanceLowerBound(RobotWorld& world,int id1,int id2=-1,Real eps=0,Real bound=Inf);
  ///Same as the other DistanceLowerBound, except a collision geometry is
  ///given.
  Real DistanceLowerBound(RobotWorld& world,Geometry::AnyCollisionGeometry3D& mesh,int id,Real eps=0,Real bound=Inf);

  ///Returns a list of object IDs that can potentially collide
  void EnumerateCollisionPairs(RobotWorld& world,vector<pair<int,int> >& pairs) const;
  ///Enumerates all potentially colliding pairs of ids contained in 
  ///the given objects, including robot self-collisions.  id1=-1,id2=-1 may
  ///also be provided to retrieve all collision pairs.
  void EnumerateCollisionQueries(RobotWorld& world,int id1,int id2,
				 vector<pair<int,int> >& pairs,
				 vector<Geometry::AnyCollisionQuery>& queries);
  ///Same as EnumerateCollisionQueries, but a collision geometry is given.
  void EnumerateCollisionQueries(RobotWorld& world,Geometry::AnyCollisionGeometry3D& mesh,int id,
				 vector<int>& checkedIDs,
				 vector<Geometry::AnyCollisionQuery>& queries);

  Array2D<bool> collisionEnabled;    //indexed by world ID #
  vector<RobotPlannerSettings> robotSettings;
  vector<ObjectPlannerSettings> objectSettings;
  vector<TerrainPlannerSettings> terrainSettings;
};

#endif
