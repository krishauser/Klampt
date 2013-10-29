#ifndef PLANNER_SETTINGS_H
#define PLANNER_SETTINGS_H

#include "Modeling/World.h"
#include <structs/array2d.h>
#include <geometry/CollisionMesh.h>

struct RobotPlannerSettings
{
  Real collisionEpsilon;   //threshold for edge feasibility checks
  Vector distanceWeights;  //for non-euclidean distance metric
  AABB3D worldBounds;      //for free-floating robots
  Real contactEpsilon;     //contact tolerance
  int contactIKMaxIters;   //max iters for contact solving
};

struct ObjectPlannerSettings
{
  bool touchable;          //true if its allowed to be touched
  Real collisionEpsilon;   //threshold for edge feasibility checks
  Real translationWeight, rotationWeight;  //for distance metric
  AABB3D worldBounds;      //for translation
};

struct WorldPlannerSettings
{
  WorldPlannerSettings();
  void InitializeDefault(RobotWorld& world);
  bool CheckCollision(RobotWorld& world,int id1,int id2=-1,Real tol=0);
  bool CheckCollision(RobotWorld& world,Geometry::AnyCollisionGeometry3D& mesh,int id,Real tol=0);
  Real DistanceLowerBound(RobotWorld& world,int id1,int id2=-1,Real eps=0,Real bound=Inf);
  Real DistanceLowerBound(RobotWorld& world,Geometry::AnyCollisionGeometry3D& mesh,int id,Real eps=0,Real bound=Inf);
  void EnumerateCollisionPairs(RobotWorld& world,vector<pair<int,int> >& pairs) const;
  void EnumerateCollisionQueries(RobotWorld& world,int id1,int id2,
				 vector<pair<int,int> >& pairs,
				 vector<Geometry::AnyCollisionQuery>& queries);
  void EnumerateCollisionQueries(RobotWorld& world,Geometry::AnyCollisionGeometry3D& mesh,int id,
				 vector<int>& checkedIDs,
				 vector<Geometry::AnyCollisionQuery>& queries);

  Array2D<bool> collisionEnabled;    //indexed by world ID #
  vector<RobotPlannerSettings> robotSettings;
  vector<ObjectPlannerSettings> objectSettings;
};

#endif
