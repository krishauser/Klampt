#include "PlannerSettings.h"
using namespace Meshing;
using namespace Geometry;

//defined in Modeling/Robot.cpp
Real Radius(const Geometry::AnyGeometry3D& geom);

WorldPlannerSettings::WorldPlannerSettings()
{}

void WorldPlannerSettings::InitializeDefault(RobotWorld& world)
{
  int n=world.NumIDs();
  collisionEnabled.resize(n,n,true);
  for(int i=0;i<n;i++)
    collisionEnabled(i,i) = false;
  for(size_t i=0;i<world.robots.size();i++) {
    int k=world.RobotID(i);
    collisionEnabled(k,k) = true; //can self collide
    int baseid=world.RobotLinkID(i,0);
    Robot* robot = world.robots[i].robot;
    for(size_t j=0;j<robot->links.size();j++) {  //turn off link-to-robot collision
      collisionEnabled(baseid+j,k) = false;
      collisionEnabled(k,baseid+j) = false;
    }
    for(size_t j=0;j<robot->links.size();j++) 
      for(size_t k=0;k<robot->links.size();k++)
	collisionEnabled(baseid+j,baseid+k) = (robot->selfCollisions(j,k)!=NULL);
    //links that are fixed to the terrain
    for(size_t j=0;j<robot->links.size();j++)
      if(robot->parents[j] == -1) {
	for(size_t k=0;k<world.terrains.size();k++)
	  collisionEnabled(baseid+j,world.TerrainID(k)) = false;
      }
  }

  AABB3D bounds;
  bounds.minimize();
  for(size_t i=0;i<world.rigidObjects.size();i++) {
    world.rigidObjects[i].object->geometry.SetTransform(world.rigidObjects[i].object->T);
    bounds.setUnion(world.rigidObjects[i].object->geometry.GetAABB());
  }
  for(size_t i=0;i<world.terrains.size();i++) {
    bounds.setUnion(world.terrains[i].terrain->geometry.GetAABB());
  }
  for(size_t i=0;i<world.robots.size();i++) {
    for(size_t j=0;j<world.robots[i].robot->links.size();j++) {
      world.robots[i].robot->geometry[j].SetTransform(world.robots[i].robot->links[j].T_World);
      bounds.setUnion(world.robots[i].robot->geometry[j].GetAABB());
    }
  }

  objectSettings.resize(world.rigidObjects.size());
  for(size_t i=0;i<objectSettings.size();i++) {
    objectSettings[i].worldBounds = bounds;
    objectSettings[i].touchable = true;
    objectSettings[i].collisionEpsilon = 0.001;
    objectSettings[i].translationWeight = 1.0;
    objectSettings[i].rotationWeight = Radius(world.rigidObjects[i].object->geometry);
  }

  robotSettings.resize(world.robots.size());
  for(size_t i=0;i<robotSettings.size();i++) {
    robotSettings[i].worldBounds = bounds;
    robotSettings[i].collisionEpsilon = 0.005;
    robotSettings[i].distanceWeights.clear();
    robotSettings[i].worldBounds = bounds;
    robotSettings[i].contactEpsilon = 0.001;
    robotSettings[i].contactIKMaxIters = 50;
  }
}

bool CheckCollision(AnyCollisionGeometry3D& m1,AnyCollisionGeometry3D& m2,Real tol)
{
  Assert(tol >= 0);
  AnyCollisionQuery q(m1,m2);
  if(tol == 0) {
    return q.Collide();
  }
  else {
    return q.WithinDistance(tol);
  }
}

Real DistanceLowerBound(AnyCollisionGeometry3D& m1,AnyCollisionGeometry3D& m2,Real epsilon,Real bound=Inf)
{
  Assert(epsilon >= 0);
  AnyCollisionQuery q(m1,m2);
  return q.Distance(0.0,epsilon,bound);
}

bool WorldPlannerSettings::CheckCollision(RobotWorld& world,int id1,int id2,Real tol)
{
  if(id2 < 0) {  //check all
    for(int i=0;i<collisionEnabled.n;i++) {
      if(CheckCollision(world,id1,i,tol)) return true;
    }
    return false;
  }
  else {
    if(!collisionEnabled(id1,id2)) return false;
    int index1,index2;
    index1 = world.IsRobot(id1);
    index2 = world.IsRobot(id2);
    //check for robot collision
    if(index1 >= 0) {
      Robot* robot = world.robots[index1].robot;
      if(index2 >= 0) {
	Robot* robot2 = world.robots[index2].robot;
	for(size_t j=0;j<robot->links.size();j++)
	  for(size_t k=0;k<robot2->links.size();k++)
	    if(collisionEnabled(world.RobotLinkID(index1,j),world.RobotLinkID(index2,k))) {
	      if(::CheckCollision(robot->geometry[j],robot2->geometry[k],tol)) return true;
	    }
      }
      else {
	for(size_t j=0;j<robot->links.size();j++)
	  if(collisionEnabled(world.RobotLinkID(index1,j),id2)) {
	    if(CheckCollision(world,robot->geometry[j],id2,tol)) return true;
	  }
      }
      return false;
    }
    else if(index2 >= 0) {
      Robot* robot2 = world.robots[index2].robot;
      for(size_t j=0;j<robot2->links.size();j++)
	if(collisionEnabled(id1,world.RobotLinkID(index2,j)))
	  if(CheckCollision(world,robot2->geometry[j],id1,tol)) return true;
    }
    //non-robots
    index1 = world.IsTerrain(id1);
    if(index1 >= 0) {
      return CheckCollision(world,world.terrains[index1].terrain->geometry,id2,tol);
    }
    index1 = world.IsRigidObject(id1);
    if(index1 >= 0) {
      RigidObject* obj = world.rigidObjects[index1].object;
      obj->geometry.SetTransform(obj->T);
      return CheckCollision(world,obj->geometry,id2,tol);
    }
    pair<int,int> linkid = world.IsRobotLink(id1);
    if(linkid.first >= 0) {
      Robot* robot = world.robots[linkid.first].robot;
      return CheckCollision(world,robot->geometry[linkid.second],id2,tol);
    }
    return false;
  }
}

bool WorldPlannerSettings::CheckCollision(RobotWorld& world,AnyCollisionGeometry3D& mesh,int id,Real tol)
{
  if(id < 0) {  //check all
    for(int i=0;i<collisionEnabled.n;i++) {
      if(CheckCollision(world,mesh,i,tol)) return true;
    }
    return false;
  }
  else {
    int index;
    index = world.IsTerrain(id);
    if(index >= 0) {
      return ::CheckCollision(mesh,world.terrains[index].terrain->geometry,tol);
    }
    index = world.IsRigidObject(id);
    if(index >= 0) {
      RigidObject* obj = world.rigidObjects[index].object;
      obj->geometry.SetTransform(obj->T);
      return ::CheckCollision(mesh,obj->geometry,tol);
    }
    index = world.IsRobot(id);
    if(index >= 0) {
      Robot* robot = world.robots[index].robot;
      for(size_t j=0;j<robot->links.size();j++)
	if(::CheckCollision(mesh,robot->geometry[j],tol)) return true;
      return false;
    }
    pair<int,int> linkid = world.IsRobotLink(id);
    if(linkid.first >= 0) {
      Robot* robot = world.robots[linkid.first].robot;
      return ::CheckCollision(mesh,robot->geometry[linkid.second],tol);
    }
    return false;
  }
}

Real WorldPlannerSettings::DistanceLowerBound(RobotWorld& world,int id1,int id2,Real eps,Real bound)
{
  Real minDist = bound;
  if(id2 < 0) {  //check all
    for(int i=0;i<collisionEnabled.n;i++) {
      minDist = Min(minDist,DistanceLowerBound(world,id1,i,eps,minDist));
    }
    return minDist;
  }
  else {
    if(!collisionEnabled(id1,id2)) return Inf;
    int index1,index2;
    index1 = world.IsRobot(id1);
    index2 = world.IsRobot(id2);
    //check for robot collision
    if(index1 >= 0) {
      Robot* robot = world.robots[index1].robot;
      if(index2 >= 0) {
	Robot* robot2 = world.robots[index2].robot;
	for(size_t j=0;j<robot->links.size();j++)
	  for(size_t k=0;k<robot2->links.size();k++)
	    if(collisionEnabled(world.RobotLinkID(index1,j),world.RobotLinkID(index2,k))) {
	      minDist = Min(minDist,::DistanceLowerBound(robot->geometry[j],robot2->geometry[k],eps,minDist));
	    }
      }
      else {
	for(size_t j=0;j<robot->links.size();j++)
	  if(collisionEnabled(world.RobotLinkID(index1,j),id2)) {
	    minDist = Min(minDist,DistanceLowerBound(world,robot->geometry[j],id2,eps,minDist));
	  }
      }
      return minDist;
    }
    else if(index2 >= 0) {
      Robot* robot2 = world.robots[index2].robot;
      for(size_t j=0;j<robot2->links.size();j++)
	if(collisionEnabled(id1,world.RobotLinkID(index2,j))) {
	  minDist = Min(minDist,DistanceLowerBound(world,robot2->geometry[j],id1,eps,minDist));
	}
    }
    //non-robots
    index1 = world.IsTerrain(id1);
    if(index1 >= 0) {
      return DistanceLowerBound(world,world.terrains[index1].terrain->geometry,id2,eps,minDist);
    }
    index1 = world.IsRigidObject(id1);
    if(index1 >= 0) {
      RigidObject* obj = world.rigidObjects[index1].object;
      obj->geometry.SetTransform(obj->T);
      return DistanceLowerBound(world,obj->geometry,id2,eps,minDist);
    }
    pair<int,int> linkid = world.IsRobotLink(id1);
    if(linkid.first >= 0) {
      assert(linkid.first < (int)world.robots.size());
      Robot* robot = world.robots[linkid.first].robot;
      assert(linkid.second >= 0 && linkid.second < (int)robot->links.size());
      Real d=DistanceLowerBound(world,robot->geometry[linkid.second],id2,eps,minDist);
      //printf("Link %d on robot %d to object %d has distance %g\n",linkid.second,linkid.first,id2,d);
      //printf("Object %s\n",world.GetName(id2).c_str());
      if(d<=0) {
	Assert(CheckCollision(world,robot->geometry[linkid.second],id2,0));
	//printf("It collides!\n");
      }
      return d;
    }
    return Inf;
  }
}

Real WorldPlannerSettings::DistanceLowerBound(RobotWorld& world,AnyCollisionGeometry3D& mesh,int id,Real eps,Real bound)
{
  Real minDist = bound;
  if(id < 0) {  //check all
    for(int i=0;i<collisionEnabled.n;i++) {
      minDist = Min(minDist,DistanceLowerBound(world,mesh,i,eps,minDist));
    }
    return minDist;
  }
  else {
    int index;
    index = world.IsTerrain(id);
    if(index >= 0) {
      return ::DistanceLowerBound(mesh,world.terrains[index].terrain->geometry,eps,minDist);
    }
    index = world.IsRigidObject(id);
    if(index >= 0) {
      RigidObject* obj = world.rigidObjects[index].object;
      obj->geometry.SetTransform(obj->T);
      return ::DistanceLowerBound(mesh,obj->geometry,eps);
    }
    index = world.IsRobot(id);
    if(index >= 0) {
      Robot* robot = world.robots[index].robot;
      for(size_t j=0;j<robot->links.size();j++) {
	minDist = Min(minDist,::DistanceLowerBound(mesh,robot->geometry[j],eps,minDist));
      }
      return minDist;
    }
    pair<int,int> linkid = world.IsRobotLink(id);
    if(linkid.first >= 0) {
      assert(linkid.first < (int)world.robots.size());
      Robot* robot = world.robots[linkid.first].robot;
      assert(linkid.second >= 0 && linkid.second < (int)robot->links.size());
      return ::DistanceLowerBound(mesh,robot->geometry[linkid.second],eps,minDist);
    }
    return false;
  }
}

void WorldPlannerSettings::EnumerateCollisionPairs(RobotWorld& world,vector<pair<int,int> >& pairs) const
{
  pairs.resize(0);
  int n=world.NumIDs();
  for(int i=0;i<n;i++) {
    if(world.IsRobot(i)>=0) continue;
    for(int j=0;j<i;j++) {
      if(world.IsRobot(j)>=0) continue;
      if(collisionEnabled(i,j)) {
	pairs.push_back(pair<int,int>(i,j));
      }
    }
  }
}

void WorldPlannerSettings::EnumerateCollisionQueries(RobotWorld& world,int id1,int id2,vector<pair<int,int> >& indices,vector<AnyCollisionQuery>& queries)
{
  if(id2 < 0) {  //check all
    for(int i=0;i<collisionEnabled.n;i++) {
      EnumerateCollisionQueries(world,id1,i,indices,queries);
    }
  }
  else {
    if(!collisionEnabled(id1,id2)) return;
    int index1,index2;
    index1 = world.IsRobot(id1);
    index2 = world.IsRobot(id2);
    //check for robot collision
    if(index1 >= 0) {
      Robot* robot = world.robots[index1].robot;
      if(index2 >= 0) {
	Robot* robot2 = world.robots[index2].robot;
	for(size_t j=0;j<robot->links.size();j++)
	  for(size_t k=0;k<robot2->links.size();k++)
	    if(collisionEnabled(world.RobotLinkID(index1,j),world.RobotLinkID(index2,k))) {
	      indices.push_back(pair<int,int>(world.RobotLinkID(index1,j),world.RobotLinkID(index2,k)));
	      queries.push_back(AnyCollisionQuery(robot->geometry[j],robot2->geometry[k]));
	    }
      }
      else {
	for(size_t j=0;j<robot->links.size();j++)
	  if(collisionEnabled(world.RobotLinkID(index1,j),id2)) {
	    vector<int> ids;
	    size_t qsize=queries.size();
	    EnumerateCollisionQueries(world,robot->geometry[j],id2,ids,queries);
	    indices.push_back(pair<int,int>(world.RobotLinkID(index1,j),id2));
	  }
      }
      return;
    }
    else if(index2 >= 0) {
      Robot* robot2 = world.robots[index2].robot;
      for(size_t j=0;j<robot2->links.size();j++)
	if(collisionEnabled(id1,world.RobotLinkID(index2,j))) {
	  vector<int> ids;
	  EnumerateCollisionQueries(world,robot2->geometry[j],id1,ids,queries);
	  indices.push_back(pair<int,int>(world.RobotLinkID(index2,j),id1));
	}
      return;
    }
    vector<int> ids;
    //non-robots
    index1 = world.IsTerrain(id1);
    if(index1 >= 0) {
      EnumerateCollisionQueries(world,world.terrains[index1].terrain->geometry,id2,ids,queries);
      for(size_t i=0;i<ids.size();i++)
	indices.push_back(pair<int,int>(id1,ids[i]));
      return;
    }
    index1 = world.IsRigidObject(id1);
    if(index1 >= 0) {
      RigidObject* obj = world.rigidObjects[index1].object;
      obj->geometry.SetTransform(obj->T);
      EnumerateCollisionQueries(world,obj->geometry,id2,ids,queries);
      for(size_t i=0;i<ids.size();i++)
	indices.push_back(pair<int,int>(id1,ids[i]));
      return;
    }
    pair<int,int> linkid = world.IsRobotLink(id1);
    if(linkid.first >= 0) {
      Robot* robot = world.robots[linkid.first].robot;
      EnumerateCollisionQueries(world,robot->geometry[linkid.second],id2,ids,queries);
      for(size_t i=0;i<ids.size();i++)
	indices.push_back(pair<int,int>(id1,ids[i]));
    }
    return;
  }
}

void WorldPlannerSettings::EnumerateCollisionQueries(RobotWorld& world,AnyCollisionGeometry3D& mesh,int id,vector<int>& collisionIds,vector<AnyCollisionQuery>& queries)
{
  if(id < 0) {  //check all
    for(int i=0;i<collisionEnabled.n;i++) {
      EnumerateCollisionQueries(world,mesh,i,collisionIds,queries);
    }
    return;
  }
  else {
    int index;
    index = world.IsTerrain(id);
    if(index >= 0) {
      queries.push_back(AnyCollisionQuery(mesh,world.terrains[index].terrain->geometry));
      collisionIds.push_back(id);
      return;
    }
    index = world.IsRigidObject(id);
    if(index >= 0) {
      RigidObject* obj = world.rigidObjects[index].object;
      obj->geometry.SetTransform(obj->T);
      queries.push_back(AnyCollisionQuery(mesh,obj->geometry));
      collisionIds.push_back(id);
      return;
    }
    index = world.IsRobot(id);
    if(index >= 0) {
      Robot* robot = world.robots[index].robot;
      for(size_t j=0;j<robot->links.size();j++) {
	queries.push_back(AnyCollisionQuery(mesh,robot->geometry[j]));
	collisionIds.push_back(world.RobotLinkID(index,j));
      }
      return;
    }
    pair<int,int> linkid = world.IsRobotLink(id);
    if(linkid.first >= 0) {
      Robot* robot = world.robots[linkid.first].robot;
      queries.push_back(AnyCollisionQuery(mesh,robot->geometry[linkid.second]));
      collisionIds.push_back(id);
    }
    return;
  }
}
