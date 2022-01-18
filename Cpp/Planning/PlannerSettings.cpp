#include "PlannerSettings.h"
using namespace Meshing;
using namespace Geometry;
using namespace Klampt;

namespace Klampt {

//defined in Modeling/Robot.cpp
Real Radius(const Geometry::AnyGeometry3D& geom);

} //namespace Klampt

WorldPlannerSettings::WorldPlannerSettings()
{}

void WorldPlannerSettings::InitializeDefault(WorldModel& world)
{
  int n=world.NumIDs();
  collisionEnabled.resize(n,n,true);
  for(int i=0;i<n;i++)
    collisionEnabled(i,i) = false;
  for(size_t i=0;i<world.robots.size();i++) {
    int k=world.RobotID(i);
    collisionEnabled(k,k) = true; //can self collide
    int baseid=world.RobotLinkID(i,0);
    RobotModel* robot = world.robots[i].get();
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
	for(size_t k=0;k<world.terrains.size();k++) {
	  collisionEnabled(baseid+j,world.TerrainID(k)) = false;
    collisionEnabled(world.TerrainID(k),baseid+j) = false;
  }
      }
  }

  AABB3D bounds;
  bounds.minimize();
  for(size_t i=0;i<world.rigidObjects.size();i++) {
    if(!world.rigidObjects[i]->geometry.Empty()) {
      world.rigidObjects[i]->geometry->SetTransform(world.rigidObjects[i]->T);
      bounds.setUnion(world.rigidObjects[i]->geometry->GetAABB());
    }
  }
  for(size_t i=0;i<world.terrains.size();i++) {
    if(!world.terrains[i]->geometry.Empty())
      bounds.setUnion(world.terrains[i]->geometry->GetAABB());
  }
  for(size_t i=0;i<world.robots.size();i++) {
    for(size_t j=0;j<world.robots[i]->links.size();j++) {
      if(!world.robots[i]->IsGeometryEmpty(j)) {
	world.robots[i]->geometry[j]->SetTransform(world.robots[i]->links[j].T_World);
	bounds.setUnion(world.robots[i]->geometry[j]->GetAABB());
      }
    }
  }

  terrainSettings.resize(world.terrains.size());
  for(size_t i=0;i<terrainSettings.size();i++) {
    terrainSettings[i].touchable = true;
  }

  objectSettings.resize(world.rigidObjects.size());
  for(size_t i=0;i<objectSettings.size();i++) {
    objectSettings[i].worldBounds = bounds;
    objectSettings[i].touchable = true;
    //1cm or 0.57 degrees movement allowed between checked points
    objectSettings[i].collisionEpsilon = 0.01;
    objectSettings[i].translationWeight = 1.0;
    if(world.rigidObjects[i]->geometry.Empty())
      objectSettings[i].rotationWeight = 1;
    else
      objectSettings[i].rotationWeight = Radius(*world.rigidObjects[i]->geometry);
  }

  robotSettings.resize(world.robots.size());
  for(size_t i=0;i<robotSettings.size();i++) {
    robotSettings[i].worldBounds = bounds;
    //1cm or 0.57 degrees movement allowed between checked points
    robotSettings[i].collisionEpsilon = 0.01;

    robotSettings[i].distanceWeights.clear();
    robotSettings[i].worldBounds = bounds;
    robotSettings[i].contactEpsilon = 0.001;
    robotSettings[i].contactIKMaxIters = 50;
  }
}

bool CheckCollision(AnyCollisionGeometry3D* m1,AnyCollisionGeometry3D* m2,Real tol)
{
  if(!m1 || !m2) return false;
  Assert(tol >= 0);
  AnyCollisionQuery q(*m1,*m2);
  if(tol == 0) {
    return q.Collide();
  }
  else {
    return q.WithinDistance(tol);
  }
}

Real DistanceLowerBound(AnyCollisionGeometry3D* m1,AnyCollisionGeometry3D* m2,Real epsilon,Real bound=Inf)
{
  if(!m1 || !m2) return Inf;
  Assert(epsilon >= 0);
  AnyCollisionQuery q(*m1,*m2);
  return q.Distance(0.0,epsilon,bound);
}

bool WorldPlannerSettings::CheckCollision(WorldModel& world,int id1,int id2,Real tol)
{
  if(id2 < 0) {  //check all
    vector<int> ids(collisionEnabled.n);
    for(int i=0;i<collisionEnabled.n;i++)
      ids[i] = i;
    if(id1 < 0) {  //check all vs all
      return CheckCollision(world,ids,tol).first >= 0;
    }
    else {
      //if id1 is a robot, we'd want to speed up the testing (and separate self collision checking from environment collision checking)
      int index1;
      index1 = world.IsRobot(id1);
      if(index1 >= 0) {
        RobotModel* robot = world.robots[index1].get();
        vector<int> r1links(robot->links.size());
        for(size_t j=0;j<robot->links.size();j++)
          r1links[j] = world.RobotLinkID(index1,j);
        vector<int> idothers;
        for(size_t i=0;i<world.terrains.size();i++)
          idothers.push_back(world.TerrainID(i));
        for(size_t i=0;i<world.rigidObjects.size();i++)
          idothers.push_back(world.RigidObjectID(i));
        for(size_t i=0;i<world.robots.size();i++) {
          if((int)i != index1)
            idothers.push_back(world.RobotID(i));
        }
        return CheckCollision(world,r1links,tol).first >=0 || CheckCollision(world,r1links,idothers,tol).first >= 0;
      }
      else {
        //not a robot, just check everything
        for(int i=0;i<collisionEnabled.n;i++) {
          if(CheckCollision(world,id1,i,tol)) return true;
        }
      }
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
      RobotModel* robot = world.robots[index1].get();
      if(index2 >= 0) {
        RobotModel* robot2 = world.robots[index2].get();
        vector<int> r1links(robot->links.size());
        for(size_t j=0;j<robot->links.size();j++)
          r1links[j] = world.RobotLinkID(index1,j);
        if(robot == robot2)
          //self collision
          return CheckCollision(world,r1links,tol).first >= 0;
        else {
          //this uses BB quick reject tests and is faster than checking all pairs 
          vector<int> r2links(robot2->links.size());
          for(size_t j=0;j<robot2->links.size();j++)
            r2links[j] = world.RobotLinkID(index2,j);
          return CheckCollision(world,r1links,r2links,tol).first >= 0;
        }
      }
      else {
	for(size_t j=0;j<robot->links.size();j++)
	  if(collisionEnabled(world.RobotLinkID(index1,j),id2)) {
	    if(CheckCollision(world,robot->geometry[j].get(),id2,tol)) return true;
	  }
      }
      return false;
    }
    else if(index2 >= 0) {
      RobotModel* robot2 = world.robots[index2].get();
      for(size_t j=0;j<robot2->links.size();j++)
	if(collisionEnabled(id1,world.RobotLinkID(index2,j)))
	  if(CheckCollision(world,robot2->geometry[j].get(),id1,tol)) return true;
    }
    //non-robots
    index1 = world.IsTerrain(id1);
    if(index1 >= 0) {
      return CheckCollision(world,world.terrains[index1]->geometry,id2,tol);
    }
    index1 = world.IsRigidObject(id1);
    if(index1 >= 0) {
      RigidObjectModel* obj = world.rigidObjects[index1].get();
      if(obj->geometry.Empty()) return false;
      obj->geometry->SetTransform(obj->T);
      return CheckCollision(world,obj->geometry,id2,tol);
    }
    pair<int,int> linkid = world.IsRobotLink(id1);
    if(linkid.first >= 0) {
      RobotModel* robot = world.robots[linkid.first].get();
      return CheckCollision(world,robot->geometry[linkid.second].get(),id2,tol);
    }
    return false;
  }
}

void GetGeometries(WorldModel& world,const vector<int>& ids,vector<Geometry::AnyCollisionGeometry3D*>& geoms,vector<int>& activeids)
{
  geoms.reserve(ids.size());
  activeids.reserve(ids.size());
  for(size_t i=0;i<ids.size();i++) {
    int robotindex = world.IsRobot(ids[i]);;
    if(robotindex >=0) {
      //crud, have to expand
      RobotModel* robot = world.robots[robotindex].get();
      for(size_t j=0;j<robot->links.size();j++) {
	Geometry::AnyCollisionGeometry3D* g=robot->geometry[j].get();
	if(g && !g->Empty()) {
	  geoms.push_back(g);
	  activeids.push_back(world.RobotLinkID(robotindex,j));
	}
      }
    }
    else {
      Geometry::AnyCollisionGeometry3D* g=world.GetGeometry(ids[i]).get();
      if(g && !g->Empty()) {
	geoms.push_back(g);
	activeids.push_back(ids[i]);
      }
    }
  }
}

pair<int,int> WorldPlannerSettings::CheckCollision(WorldModel& world,const vector<int>& ids,Real tol)
{
  //first, get all the geometries
  vector<Geometry::AnyCollisionGeometry3D*> geoms;
  vector<int> activeids;
  GetGeometries(world,ids,geoms,activeids);

  Vector3 d(tol*0.5); //adjustment
  vector<AABB3D> bbs(geoms.size());
  for(size_t i=0;i<geoms.size();i++) {
    bbs[i]=geoms[i]->GetAABB();
    bbs[i].bmin -= d;
    bbs[i].bmax += d;
  }

  for(size_t i=0;i<activeids.size();i++) {
    for(size_t j=i+1;j<activeids.size();j++) {
      if(collisionEnabled(activeids[i],activeids[j]) || collisionEnabled(activeids[i],activeids[i])) {
        if(bbs[i].intersects(bbs[j])) {
          if(::CheckCollision(geoms[i],geoms[j],tol))
            return pair<int,int>(activeids[i],activeids[j]);
        }
      }
    }
  }
  return pair<int,int>(-1,-1);
}

pair<int,int> WorldPlannerSettings::CheckCollision(WorldModel& world,const vector<int>& ids1,const vector<int>& ids2,Real tol)
{
  //first, get all the geometries
  vector<Geometry::AnyCollisionGeometry3D*> geoms1,geoms2;
  vector<int> activeids1,activeids2;
  GetGeometries(world,ids1,geoms1,activeids1);
  GetGeometries(world,ids2,geoms2,activeids2);

  Vector3 d(tol*0.5); //adjustment
  vector<AABB3D> bbs1(geoms1.size()),bbs2(geoms2.size());
  for(size_t i=0;i<geoms1.size();i++) {
    bbs1[i]=geoms1[i]->GetAABB();
    bbs1[i].bmin -= d;
    bbs1[i].bmax += d;
  }
  for(size_t i=0;i<geoms2.size();i++) {
    bbs2[i]=geoms2[i]->GetAABB();
    bbs2[i].bmin -= d;
    bbs2[i].bmax += d;
  }

  //quick reject: anything in set2 that doesn't intersect entire set1 bb can be erased from consideration
  AABB3D bb;
  bb.minimize();
  for(size_t i=0;i<bbs1.size();i++)
    bb.setUnion(bbs1[i]);
  for(size_t i=0;i<bbs2.size();i++)
    if(!bbs2[i].intersects(bb)) { //erase
      activeids2[i] = activeids2.back();
      geoms2[i] = geoms2.back();
      bbs2[i] = bbs2.back();
      activeids2.resize(activeids2.size()-1);
      geoms2.resize(geoms2.size()-1);
      bbs2.resize(bbs2.size()-1);
      i--;
    }
  bb.minimize();
  for(size_t i=0;i<bbs2.size();i++)
    bb.setUnion(bbs2[i]);
  for(size_t i=0;i<bbs1.size();i++)
    if(!bbs1[i].intersects(bb)) { //erase
      activeids1[i] = activeids1.back();
      geoms1[i] = geoms1.back();
      bbs1[i] = bbs1.back();
      activeids1.resize(activeids1.size()-1);
      geoms1.resize(geoms1.size()-1);
      bbs1.resize(bbs1.size()-1);
      i--;
    }
  

  for(size_t i=0;i<activeids1.size();i++) {
    for(size_t j=0;j<activeids2.size();j++) {
      if(collisionEnabled(activeids1[i],activeids2[j]) || collisionEnabled(activeids2[j],activeids1[i])) {
        if(bbs1[i].intersects(bbs2[j])) {
          if(::CheckCollision(geoms1[i],geoms2[j],tol))
            return pair<int,int>(activeids1[i],activeids2[j]);
        }
      }
    }
  }
  return pair<int,int>(-1,-1);
}

bool WorldPlannerSettings::CheckCollision(WorldModel& world,AnyCollisionGeometry3D* geom,int id,Real tol)
{
  if(!geom) return false;
  if(id < 0) {  //check all
    for(int i=0;i<collisionEnabled.n;i++) {
      if(CheckCollision(world,geom,i,tol)) return true;
    }
    return false;
  }
  else {
    int index;
    index = world.IsTerrain(id);
    if(index >= 0) {
      return ::CheckCollision(geom,&*world.terrains[index]->geometry,tol);
    }
    index = world.IsRigidObject(id);
    if(index >= 0) {
      RigidObjectModel* obj = world.rigidObjects[index].get();
      if(obj->geometry.Empty()) return false;
      obj->geometry->SetTransform(obj->T);
      return ::CheckCollision(geom,&*obj->geometry,tol);
    }
    index = world.IsRobot(id);
    if(index >= 0) {
      RobotModel* robot = world.robots[index].get();
      for(size_t j=0;j<robot->links.size();j++) {
	if(::CheckCollision(geom,&*robot->geometry[j].get(),tol)) return true;
      }
      return false;
    }
    pair<int,int> linkid = world.IsRobotLink(id);
    if(linkid.first >= 0) {
      RobotModel* robot = world.robots[linkid.first].get();
      return ::CheckCollision(geom,&*robot->geometry[linkid.second].get(),tol);
    }
    return false;
  }
}

Real WorldPlannerSettings::DistanceLowerBound(WorldModel& world,int id1,int id2,Real eps,Real bound)
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
      RobotModel* robot = world.robots[index1].get();
      if(index2 >= 0) {
	RobotModel* robot2 = world.robots[index2].get();
	for(size_t j=0;j<robot->links.size();j++)
	  for(size_t k=0;k<robot2->links.size();k++)
	    if(collisionEnabled(world.RobotLinkID(index1,j),world.RobotLinkID(index2,k))) {
	      minDist = Min(minDist,::DistanceLowerBound(robot->geometry[j].get(),robot2->geometry[k].get(),eps,minDist));
	    }
      }
      else {
	for(size_t j=0;j<robot->links.size();j++)
	  if(collisionEnabled(world.RobotLinkID(index1,j),id2)) {
	    minDist = Min(minDist,DistanceLowerBound(world,robot->geometry[j].get(),id2,eps,minDist));
	  }
      }
      return minDist;
    }
    else if(index2 >= 0) {
      RobotModel* robot2 = world.robots[index2].get();
      for(size_t j=0;j<robot2->links.size();j++)
	if(collisionEnabled(id1,world.RobotLinkID(index2,j))) {
	  minDist = Min(minDist,DistanceLowerBound(world,robot2->geometry[j].get(),id1,eps,minDist));
	}
    }
    //non-robots
    index1 = world.IsTerrain(id1);
    if(index1 >= 0) {
      return DistanceLowerBound(world,world.terrains[index1]->geometry,id2,eps,minDist);
    }
    index1 = world.IsRigidObject(id1);
    if(index1 >= 0) {
      RigidObjectModel* obj = world.rigidObjects[index1].get();
      if(obj->geometry.Empty()) return false;
      obj->geometry->SetTransform(obj->T);
      return DistanceLowerBound(world,obj->geometry,id2,eps,minDist);
    }
    pair<int,int> linkid = world.IsRobotLink(id1);
    if(linkid.first >= 0) {
      assert(linkid.first < (int)world.robots.size());
      RobotModel* robot = world.robots[linkid.first].get();
      assert(linkid.second >= 0 && linkid.second < (int)robot->links.size());
      Real d=DistanceLowerBound(world,robot->geometry[linkid.second].get(),id2,eps,minDist);
      //printf("Link %d on robot %d to object %d has distance %g\n",linkid.second,linkid.first,id2,d);
      //printf("Object %s\n",world.GetName(id2).c_str());
      //if(d<=0) {
      //Assert(CheckCollision(world,robot->geometry[linkid.second],id2,0));
	//printf("It collides!\n");
      //}
      return d;
    }
    return Inf;
  }
}

Real WorldPlannerSettings::DistanceLowerBound(WorldModel& world,AnyCollisionGeometry3D* mesh,int id,Real eps,Real bound)
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
      if(world.terrains[index]->geometry.Empty()) return Inf;
      return ::DistanceLowerBound(mesh,&*world.terrains[index]->geometry,eps,minDist);
    }
    index = world.IsRigidObject(id);
    if(index >= 0) {
      RigidObjectModel* obj = world.rigidObjects[index].get();
      if(obj->geometry.Empty()) return Inf;
      obj->geometry->SetTransform(obj->T);
      return ::DistanceLowerBound(mesh,&*obj->geometry,eps);
    }
    index = world.IsRobot(id);
    if(index >= 0) {
      RobotModel* robot = world.robots[index].get();
      for(size_t j=0;j<robot->links.size();j++) {
	minDist = Min(minDist,::DistanceLowerBound(mesh,&*robot->geometry[j],eps,minDist));
      }
      return minDist;
    }
    pair<int,int> linkid = world.IsRobotLink(id);
    if(linkid.first >= 0) {
      assert(linkid.first < (int)world.robots.size());
      RobotModel* robot = world.robots[linkid.first].get();
      assert(linkid.second >= 0 && linkid.second < (int)robot->links.size());

      return ::DistanceLowerBound(mesh,robot->geometry[linkid.second].get(),eps,minDist);
    }
    return minDist;
  }
}

Real MaxDistance2(const AABB3D& a,const Vector3& pt)
{
  Vector3 furthest;
  if(pt.x < a.bmin.x) furthest.x=a.bmax.x;
  else if(pt.x > a.bmax.x) furthest.x=a.bmin.x;
  else if(Abs(pt.x-a.bmin.x) < Abs(pt.x-a.bmax.x)) furthest.x = a.bmax.x;
  else furthest.x = a.bmin.x;
  if(pt.y < a.bmin.y) furthest.y=a.bmax.y;
  else if(pt.y > a.bmax.y) furthest.y=a.bmin.y;
  else if(Abs(pt.y-a.bmin.y) < Abs(pt.y-a.bmax.y)) furthest.y = a.bmax.y;
  else furthest.y = a.bmin.y;
  if(pt.z < a.bmin.z) furthest.z=a.bmax.z;
  else if(pt.z > a.bmax.z) furthest.z=a.bmin.z;
  else if(Abs(pt.z-a.bmin.z) < Abs(pt.z-a.bmax.z)) furthest.z = a.bmax.z;
  else furthest.z = a.bmin.z;
  return furthest.distanceSquared(pt);
}

Real MaxDistance(const AABB3D& a,const AABB3D& b)
{
  Vector3 fa,fb;
  if(a.bmax.x < b.bmin.x) { fa.x = a.bmin.x; fb.x=b.bmax.x; }  //a to left of b
  else if(b.bmax.x < a.bmin.x) { fa.x = a.bmax.x; fb.x=b.bmin.x; } //b to left of a
  else if(Abs(b.bmax.x-a.bmin.x) > Abs(b.bmin.x-a.bmax.x)) { fa.x = a.bmin.x; fb.x=b.bmax.x; }  //ranges intersect
  else { fa.x = a.bmax.x; fb.x=b.bmin.x; } //ranges intersect

  if(a.bmax.y < b.bmin.y) { fa.y = a.bmin.y; fb.y=b.bmax.y; }  //a to left of b
  else if(b.bmax.y < a.bmin.y) { fa.y = a.bmax.y; fb.y=b.bmin.y; } //b to left of a
  else if(Abs(b.bmax.y-a.bmin.y) > Abs(b.bmin.y-a.bmax.y)) { fa.y = a.bmin.y; fb.y=b.bmax.y; }  //ranges intersect
  else { fa.y = a.bmax.y; fb.y=b.bmin.y; } //ranges intersect

  if(a.bmax.z < b.bmin.z) { fa.z = a.bmin.z; fb.z=b.bmax.z; }  //a to left of b
  else if(b.bmax.z < a.bmin.z) { fa.z = a.bmax.z; fb.z=b.bmin.z; } //b to left of a
  else if(Abs(b.bmax.z-a.bmin.z) > Abs(b.bmin.z-a.bmax.z)) { fa.z = a.bmin.z; fb.z=b.bmax.z; }  //ranges intersect
  else { fa.z = a.bmax.z; fb.z=b.bmin.z; } //ranges intersect
  return fa.distance(fb);
}

Real WorldPlannerSettings::DistanceLowerBound(WorldModel& world,const vector<int>& ids,Real eps,Real bound,int* closest1,int* closest2)
{
  //first, get all the geometries
  vector<Geometry::AnyCollisionGeometry3D*> geoms;
  vector<int> activeids;
  GetGeometries(world,ids,geoms,activeids);

  vector<AABB3D> bbs(geoms.size());
  for(size_t i=0;i<geoms.size();i++) {
    bbs[i]=geoms[i]->GetAABB();
  }

  //reduce upper bound based on upper bound on inter-object distance
  for(size_t i=0;i<activeids.size();i++) {
    for(size_t j=i+1;j<activeids.size();j++) {
      if(!collisionEnabled(activeids[i],activeids[j])) continue;
      Real maxd=MaxDistance(bbs[i],bbs[j]);
      if(maxd < bound) bound=maxd;
    }
  }
  //hopefully sorting pairs is cheaper than collision testing
  vector<pair<Real,pair<int,int> > > sorter;
  for(size_t i=0;i<activeids.size();i++) {
    for(size_t j=i+1;j<activeids.size();j++) {
      if(!collisionEnabled(activeids[i],activeids[j])) continue;
      Real d=bbs[i].distance(bbs[j]);
      if(d > bound) continue;
      sorter.push_back(pair<Real,pair<int,int> >(d,pair<int,int>(i,j)));
    }
  }
  sort(sorter.begin(),sorter.end());
  for(size_t i=0;i<sorter.size();i++) {
    if(sorter[i].first > bound) break;
    int a = sorter[i].second.first;
    int b = sorter[i].second.second;
    Real d=::DistanceLowerBound(geoms[a],geoms[b],eps,bound);
    if(d < bound) {
      bound = d;
      if(closest1 && closest2) {
	*closest1 = activeids[a];
	*closest2 = activeids[b];
      }
    }
  }
  return bound;
}

Real WorldPlannerSettings::DistanceLowerBound(WorldModel& world,const vector<int>& ids1,const vector<int>& ids2,Real eps,Real bound,int* closest1,int* closest2)
{
  //first, get all the geometries
  vector<Geometry::AnyCollisionGeometry3D*> geoms1,geoms2;
  vector<int> activeids1,activeids2;
  GetGeometries(world,ids1,geoms1,activeids1);
  GetGeometries(world,ids2,geoms2,activeids2);

  vector<AABB3D> bbs1(geoms1.size()),bbs2(geoms2.size());
  for(size_t i=0;i<geoms1.size();i++) {
    bbs1[i]=geoms1[i]->GetAABB();
  }
  for(size_t i=0;i<geoms2.size();i++) {
    bbs2[i]=geoms2[i]->GetAABB();
  }

  //reduce upper bound based on upper bound on inter-object distance
  for(size_t i=0;i<activeids1.size();i++) {
    for(size_t j=0;j<activeids2.size();j++) {
      if(!collisionEnabled(activeids1[i],activeids2[j])) continue;
      Real maxd=MaxDistance(bbs1[i],bbs2[j]);
      if(maxd < bound) bound=maxd;
    }
  }
  //hopefully sorting pairs is cheaper than collision testing
  vector<pair<Real,pair<int,int> > > sorter;
  for(size_t i=0;i<activeids1.size();i++) {
    for(size_t j=0;j<activeids2.size();j++) {
      if(!collisionEnabled(activeids1[i],activeids2[j])) continue;
      Real d=bbs1[i].distance(bbs2[j]);
      if(d > bound) continue;
      sorter.push_back(pair<Real,pair<int,int> >(d,pair<int,int>(i,j)));
    }
  }
  sort(sorter.begin(),sorter.end());
  for(size_t i=0;i<sorter.size();i++) {
    if(sorter[i].first > bound) break;
    int a = sorter[i].second.first;
    int b = sorter[i].second.second;
    Real d=::DistanceLowerBound(geoms1[a],geoms2[b],eps,bound);
    if(d < bound) {
      bound = d;
      if(closest1 && closest2) {
	*closest1 = activeids1[a];
	*closest2 = activeids2[b];
      }
    }
  }
  return bound;

}

void WorldPlannerSettings::EnumerateCollisionPairs(WorldModel& world,vector<pair<int,int> >& pairs) const
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

void WorldPlannerSettings::EnumerateCollisionQueries(WorldModel& world,int id1,int id2,vector<pair<int,int> >& indices,vector<AnyCollisionQuery>& queries)
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
      RobotModel* robot = world.robots[index1].get();
      if(index2 >= 0) {
	RobotModel* robot2 = world.robots[index2].get();
	for(size_t j=0;j<robot->links.size();j++)
	  for(size_t k=0;k<robot2->links.size();k++)
	    if(collisionEnabled(world.RobotLinkID(index1,j),world.RobotLinkID(index2,k))) {
	      indices.push_back(pair<int,int>(world.RobotLinkID(index1,j),world.RobotLinkID(index2,k)));
	      queries.push_back(AnyCollisionQuery(*robot->geometry[j],*robot2->geometry[k]));
	    }
      }
      else {
	for(size_t j=0;j<robot->links.size();j++)
	  if(collisionEnabled(world.RobotLinkID(index1,j),id2)) {
	    vector<int> ids;
	    size_t qsize=queries.size();
	    EnumerateCollisionQueries(world,robot->geometry[j].get(),id2,ids,queries);
	    indices.push_back(pair<int,int>(world.RobotLinkID(index1,j),id2));
	  }
      }
      return;
    }
    else if(index2 >= 0) {
      RobotModel* robot2 = world.robots[index2].get();
      for(size_t j=0;j<robot2->links.size();j++)
	if(collisionEnabled(id1,world.RobotLinkID(index2,j))) {
	  vector<int> ids;
	  EnumerateCollisionQueries(world,robot2->geometry[j].get(),id1,ids,queries);
	  indices.push_back(pair<int,int>(world.RobotLinkID(index2,j),id1));
	}
      return;
    }
    vector<int> ids;
    //non-robots
    index1 = world.IsTerrain(id1);
    if(index1 >= 0) {
      EnumerateCollisionQueries(world,&*world.terrains[index1]->geometry,id2,ids,queries);
      for(size_t i=0;i<ids.size();i++)
	indices.push_back(pair<int,int>(id1,ids[i]));
      return;
    }
    index1 = world.IsRigidObject(id1);
    if(index1 >= 0) {
      RigidObjectModel* obj = world.rigidObjects[index1].get();
      if(obj->geometry.Empty()) return;
      obj->geometry->SetTransform(obj->T);
      EnumerateCollisionQueries(world,&*obj->geometry,id2,ids,queries);
      for(size_t i=0;i<ids.size();i++)
	indices.push_back(pair<int,int>(id1,ids[i]));
      return;
    }
    pair<int,int> linkid = world.IsRobotLink(id1);
    if(linkid.first >= 0) {
      RobotModel* robot = world.robots[linkid.first].get();
      EnumerateCollisionQueries(world,robot->geometry[linkid.second].get(),id2,ids,queries);
      for(size_t i=0;i<ids.size();i++)
	indices.push_back(pair<int,int>(id1,ids[i]));
    }
    return;
  }
}

void WorldPlannerSettings::EnumerateCollisionQueries(WorldModel& world,AnyCollisionGeometry3D* mesh,int id,vector<int>& collisionIds,vector<AnyCollisionQuery>& queries)
{
  if(mesh == NULL) return;
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
      if(world.terrains[index]->geometry.Empty()) return;
      queries.push_back(AnyCollisionQuery(*mesh,*world.terrains[index]->geometry));
      collisionIds.push_back(id);
      return;
    }
    index = world.IsRigidObject(id);
    if(index >= 0) {
      RigidObjectModel* obj = world.rigidObjects[index].get();
      if(obj->geometry.Empty()) return;
      obj->geometry->SetTransform(obj->T);
      queries.push_back(AnyCollisionQuery(*mesh,*obj->geometry));
      collisionIds.push_back(id);
      return;
    }
    index = world.IsRobot(id);
    if(index >= 0) {
      RobotModel* robot = world.robots[index].get();
      for(size_t j=0;j<robot->links.size();j++) {
	if(robot->IsGeometryEmpty(j)) continue;
	queries.push_back(AnyCollisionQuery(*mesh,*robot->geometry[j]));
	collisionIds.push_back(world.RobotLinkID(index,j));
      }
      return;
    }
    pair<int,int> linkid = world.IsRobotLink(id);
    if(linkid.first >= 0) {
      RobotModel* robot = world.robots[linkid.first].get();
      if(robot->IsGeometryEmpty(linkid.second)) return;
      queries.push_back(AnyCollisionQuery(*mesh,*robot->geometry[linkid.second]));
      collisionIds.push_back(id);
    }
    return;
  }
}
