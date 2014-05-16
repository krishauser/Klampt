#include "RobotCSpace.h"
#include "Modeling/Interpolate.h"
#include <math/angle.h>
#include <math/random.h>
#include <math3d/rotation.h>
#include <math3d/random.h>
#include <math3d/interpolate.h>
#include <robotics/Rotation.h>
#include <planning/EdgePlanner.h>
#include <Timer.h>
#include <sstream>

RobotCSpace::RobotCSpace(Robot& _robot)
  :robot(_robot),norm(2.0)
{
  floatingRotationWeight=1;
  floatingRotationRadiusScale=1;
}

void RobotCSpace::Sample(Config& q)
{
  for(int i=0;i<robot.joints.size();i++) {
    int link = robot.joints[i].linkIndex;
    switch(robot.joints[i].type) {
    case RobotJoint::Weld:
      break;
    case RobotJoint::Normal:
      robot.q(link) = Rand(robot.qMin(link),robot.qMax(link));
      break;
    case RobotJoint::Spin:
      robot.q(link) = Rand(0,TwoPi);
      break;
    case RobotJoint::Floating:
    case RobotJoint::BallAndSocket:
      {
	Matrix3 R;
	QuaternionRotation qr;
	RandRotation(qr);
	qr.getMatrix(R);
	robot.SetJointByOrientation(i,robot.joints[i].linkIndex,R);
      }
      break;
    default:
      break;
    }
  }
  for(size_t i=0;i<robot.drivers.size();i++) {
    if(robot.drivers[i].type != RobotJointDriver::Normal) {
      Real val = Rand(robot.drivers[i].qmin,robot.drivers[i].qmax);
      robot.SetDriverValue(i,val);
    }
  }
  q = robot.q;
  return;
}

void RobotCSpace::SampleNeighborhood(const Config& c,Real r,Config& q)
{
  robot.q=c;
  for(size_t i=0;i<robot.joints.size();i++) {
    Real ri;
    if(jointRadiusScale.empty() == 0) ri = r / jointWeights[i];
    else ri = r / jointRadiusScale[i];
    int link = robot.joints[i].linkIndex;
    switch(robot.joints[i].type) {
    case RobotJoint::Weld:
      break;
    case RobotJoint::Normal:
      robot.q(link) += Rand(-ri,ri);
      break;
    case RobotJoint::Spin:
      robot.q(link) += Rand(-ri,ri);
      break;
    case RobotJoint::Floating:
    case RobotJoint::BallAndSocket:
      {
	RigidTransform T = robot.links[link].T_World;
	T.t.x += Rand(-ri,ri);
	T.t.y += Rand(-ri,ri);
	T.t.z += Rand(-ri,ri);
	AngleAxisRotation aa;
	SampleSphere(1.0,aa.axis);
	aa.angle = Rand(-ri/floatingRotationRadiusScale,ri/floatingRotationRadiusScale);
	Matrix3 Rperturb;
	aa.getMatrix(Rperturb);
	T.R = Rperturb*T.R;
	robot.SetJointByTransform(i,robot.joints[i].linkIndex,T);
      }
      break;
    default:
      break;
    }
  }
  for(size_t i=0;i<robot.drivers.size();i++) {
    if(robot.drivers[i].type != RobotJointDriver::Normal) {
      Real val = robot.GetDriverValue(i);
      Real scale = 1.0;
      //TODO: figure out the proper scale factor
      robot.SetDriverValue(i,val + Rand(-r,r));
    }
  }
  q = robot.q;
  robot.NormalizeAngles(q);
}

void RobotCSpace::Interpolate(const Config& x,const Config& y,Real u,Config& out)
{
  ::Interpolate(robot,x,y,u,out);
}

void RobotCSpace::Midpoint(const Config& x,const Config& y,Config& out)
{
  ::Interpolate(robot,x,y,0.5,out);
}

Real RobotCSpace::Distance(const Config& a,const Config& b)
{
  if(jointWeights.empty())
    return ::Distance(robot,a,b,norm,floatingRotationWeight);
  else
    return ::Distance(robot,a,b,norm,jointWeights,floatingRotationWeight);
}

bool RobotCSpace::IsFeasible(const Config& x) 
{
  for(int i=0;i<x.n;i++)
    if(x(i) < robot.qMin(i) || x(i) > robot.qMax(i)) return false;
  return true;
}

EdgePlanner* RobotCSpace::LocalPlanner(const Config& x,const Config& y) 
{
  return new TrueEdgePlanner(this,x,y);
}

RobotGeodesicManifold::RobotGeodesicManifold(Robot& _robot)
  :robot(_robot)
{}

void RobotGeodesicManifold::InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx)
{ 
  ::InterpolateDerivative(robot,a,b,u,dx);
}

void RobotGeodesicManifold::InterpolateDerivA(const Config& a,const Config& b,Real u,const Vector& da,Vector& dx) 
{ 
  dx.mul(da,1-u);
  for(size_t i=0;i<robot.joints.size();i++) {
    int k=robot.joints[i].linkIndex;
    if(robot.joints[i].type == RobotJoint::Floating) {
      vector<int> indices;
      robot.GetJointIndices(i,indices);
      Vector3 oldrot(a(indices[3]),a(indices[4]),a(indices[5]));
      Vector3 newrot(b(indices[3]),b(indices[4]),b(indices[5]));
      Vector3 drot(da(indices[3]),da(indices[4]),da(indices[5]));
      assert(robot.links[indices[3]].w == Vector3(0,0,1));
      assert(robot.links[indices[4]].w == Vector3(0,1,0));
      assert(robot.links[indices[5]].w == Vector3(1,0,0));

      EulerAngleRotation ex(oldrot),ey(newrot),eu;
      Matrix3 Rx,Ry,Ru;
      ex.getMatrixZYX(Rx);
      ey.getMatrixZYX(Ry);
      interpolateRotation(Rx,Ry,u,Ru);
      eu.setMatrixZYX(Ru);

      Vector3 dw;
      AngularVelocityEulerAngle(oldrot,drot,2,1,0,dw);
      Vector3 dtheta;
      bool res=EulerAngleDerivative(eu,dw,2,1,0,dtheta);
      dtheta *= (1-u);
      dtheta.get(dx(indices[3]),dx(indices[4]),dx(indices[5]));
    }
    else if(robot.joints[i].type == RobotJoint::BallAndSocket) {
      vector<int> indices;
      robot.GetJointIndices(i,indices);
      Vector3 oldrot(a(indices[0]),a(indices[1]),a(indices[2]));
      Vector3 newrot(b(indices[0]),b(indices[1]),b(indices[2]));
      Vector3 drot(da(indices[0]),da(indices[1]),da(indices[2]));
      assert(robot.links[indices[0]].w == Vector3(0,0,1));
      assert(robot.links[indices[1]].w == Vector3(0,1,0));
      assert(robot.links[indices[2]].w == Vector3(1,0,0));

      EulerAngleRotation ex(oldrot),ey(newrot),eu;
      Matrix3 Rx,Ry,Ru;
      ex.getMatrixZYX(Rx);
      ey.getMatrixZYX(Ry);
      interpolateRotation(Rx,Ry,u,Ru);
      eu.setMatrixZYX(Ru);

      Vector3 dw;
      AngularVelocityEulerAngle(oldrot,drot,2,1,0,dw);
      Vector3 dtheta;
      bool res=EulerAngleDerivative(eu,dw,2,1,0,dtheta);
      dtheta *= (1-u);
      dtheta.get(dx(indices[0]),dx(indices[1]),dx(indices[2]));
    }
  }
}
void RobotGeodesicManifold::InterpolateDerivB(const Config& a,const Config& b,Real u,const Vector& db,Vector& dx) 
{ 
  dx.mul(db,u);
  for(size_t i=0;i<robot.joints.size();i++) {
    //int k=robot.joints[i].linkIndex;
    if(robot.joints[i].type == RobotJoint::Floating) {
      vector<int> indices;
      robot.GetJointIndices(i,indices);
      Vector3 oldrot(a(indices[3]),a(indices[4]),a(indices[5]));
      Vector3 newrot(b(indices[3]),b(indices[4]),b(indices[5]));
      Vector3 drot(db(indices[3]),db(indices[4]),db(indices[5]));
      assert(robot.links[indices[3]].w == Vector3(0,0,1));
      assert(robot.links[indices[4]].w == Vector3(0,1,0));
      assert(robot.links[indices[5]].w == Vector3(1,0,0));

      EulerAngleRotation ex(oldrot),ey(newrot),eu;
      Matrix3 Rx,Ry,Ru;
      ex.getMatrixZYX(Rx);
      ey.getMatrixZYX(Ry);
      interpolateRotation(Rx,Ry,u,Ru);
      eu.setMatrixZYX(Ru);

      Vector3 dw;
      AngularVelocityEulerAngle(newrot,drot,2,1,0,dw);
      Vector3 dtheta;
      bool res=EulerAngleDerivative(eu,dw,2,1,0,dtheta);
      dtheta *= u;
      dtheta.get(dx(indices[3]),dx(indices[4]),dx(indices[5]));
    }
    else if(robot.joints[i].type == RobotJoint::BallAndSocket) {
      vector<int> indices;
      robot.GetJointIndices(i,indices);
      Vector3 oldrot(a(indices[0]),a(indices[1]),a(indices[2]));
      Vector3 newrot(b(indices[0]),b(indices[1]),b(indices[2]));
      Vector3 drot(db(indices[0]),db(indices[1]),db(indices[2]));
      assert(robot.links[indices[0]].w == Vector3(0,0,1));
      assert(robot.links[indices[1]].w == Vector3(0,1,0));
      assert(robot.links[indices[2]].w == Vector3(1,0,0));

      EulerAngleRotation ex(oldrot),ey(newrot),eu;
      Matrix3 Rx,Ry,Ru;
      ex.getMatrixZYX(Rx);
      ey.getMatrixZYX(Ry);
      interpolateRotation(Rx,Ry,u,Ru);
      eu.setMatrixZYX(Ru);

      Vector3 dw;
      AngularVelocityEulerAngle(newrot,drot,2,1,0,dw);
      Vector3 dtheta;
      bool res=EulerAngleDerivative(eu,dw,2,1,0,dtheta);
      dtheta *= u;
      dtheta.get(dx(indices[0]),dx(indices[1]),dx(indices[2]));
    }
  }
}

void RobotGeodesicManifold::InterpolateDeriv2(const Config& a,const Config& b,Real u,Vector& ddx) { FatalError("Not implemented"); }

void RobotGeodesicManifold::Integrate(const Config& a,const Vector& da,Config& b)
{
  ::Integrate(robot,a,da,b);
}





ActiveRobotCSpace::ActiveRobotCSpace(Robot& _robot,const ArrayMapping& _dofs)
  :robot(_robot),dofs(_dofs)
{
  xq=yq=tempq=robot.q;
  invMap.resize(robot.q.n,-1);
  Assert(!dofs.IsOffset());
  for(size_t i=0;i<dofs.mapping.size();i++)
    invMap[dofs.mapping[i]] = int(i);
  for(size_t i=0;i<robot.joints.size();i++)
    if(invMap[robot.joints[i].linkIndex] >= 0)
      joints.push_back(i);
}

void ActiveRobotCSpace::Sample(Config& x)
{
  x.resize(dofs.mapping.size());
  for(size_t j=0;j<joints.size();j++) {
    int i=joints[j];
    int link = robot.joints[i].linkIndex;
    int k=invMap[link];
    switch(robot.joints[i].type) {
    case RobotJoint::Weld:
      break;
    case RobotJoint::Normal:
      x(k) = Rand(robot.qMin(link),robot.qMax(link));
      break;
    case RobotJoint::Spin:
      x(k) = Rand(0,TwoPi);
      break;
    case RobotJoint::Floating:
    case RobotJoint::BallAndSocket:
      {
	Matrix3 R;
	QuaternionRotation qr;
	RandRotation(qr);
	qr.getMatrix(R);
	robot.SetJointByOrientation(i,robot.joints[i].linkIndex,R);
	vector<int> indices;
	robot.GetJointIndices(i,indices);
	for(size_t m=0;m<indices.size();m++)
	  x(invMap[indices[m]]) = robot.q(indices[m]);
      }
      break;
    default:
      break;
    }
  }
  return;
}

void ActiveRobotCSpace::Interpolate(const Config& x,const Config& y,Real u,Config& out)
{
  dofs.Map(x,xq);
  dofs.Map(y,yq);
  ::Interpolate(robot,xq,yq,u,tempq);
  out.resize(dofs.Size());
  dofs.InvMap(tempq,out);
}

void ActiveRobotCSpace::Midpoint(const Config& x,const Config& y,Config& out)
{
  dofs.Map(x,xq);
  dofs.Map(y,yq);
  ::Interpolate(robot,xq,yq,0.5,tempq);
  out.resize(dofs.Size());
  dofs.InvMap(tempq,out);
}

Real ActiveRobotCSpace::Distance(const Config& x,const Config& y)
{
  //what norm to use
  NormAccumulator<Real> norm;
  for(size_t j=0;j<joints.size();j++) {
    int i=joints[j];
    switch(robot.joints[i].type) {
    case RobotJoint::Weld:
      break;
    case RobotJoint::Normal:
      {
	int l=invMap[robot.joints[i].linkIndex];
	norm << (x(l)-y(l));
      }
      break;
    case RobotJoint::Floating:
      {
	vector<int> indices;
	robot.GetJointIndices(i,indices);
	for(size_t j=0;j<indices.size();j++)
	  indices[j] = invMap[indices[j]];
	Assert(indices.size()==6);
	int tx = indices[0];
	int ty = indices[1];
	int tz = indices[2];
	int rz = indices[3];
	int ry = indices[4];
	int rx = indices[5];
	RigidTransform Tx,Ty;
	Tx.t.set(x[tx],x[ty],x[tz]);
	Ty.t.set(y[tx],y[ty],y[tz]);
	EulerAngleRotation Rx(x[rz],x[ry],x[rx]);
	EulerAngleRotation Ry(y[rz],y[ry],y[rx]);
	Rx.getMatrixZXY(Tx.R);
	Ry.getMatrixZXY(Ty.R);
	RigidTransform T; T.mulInverseB(Tx,Ty);
	Real cosangle = Half*(T.R.trace()-One);
	cosangle=Clamp(cosangle,-One,One);
	Real rdiff = Acos(cosangle);
	//sum += T.t.normSquared() + Sqr(rdiff);
	norm.collect(T.t.x);
	norm.collect(T.t.y);
	norm.collect(T.t.z);
	norm.collect(rdiff);
      }
      break;
    default:
      FatalError("TODO");
    }
  }
  return norm;
}

bool ActiveRobotCSpace::IsFeasible(const Config& x) { FatalError("Not implemented"); return true; }

EdgePlanner* ActiveRobotCSpace::LocalPlanner(const Config& x,const Config& y)
{
  return new TrueEdgePlanner(this,x,y);
}

ActiveRobotGeodesicManifold::ActiveRobotGeodesicManifold(Robot& _robot,const ArrayMapping& _dofs)
  :robot(_robot),dofs(_dofs)
{
  xq=yq=tempq=robot.q;
  invMap.resize(robot.q.n,-1);
  Assert(!dofs.IsOffset());
  for(size_t i=0;i<dofs.mapping.size();i++)
    invMap[dofs.mapping[i]] = int(i);
  
  for(size_t i=0;i<robot.joints.size();i++)
    if(invMap[robot.joints[i].linkIndex] >= 0) {
      if(robot.joints[i].type == RobotJoint::Floating || robot.joints[i].type == RobotJoint::Spin || robot.joints[i].type == RobotJoint::FloatingPlanar || robot.joints[i].type == RobotJoint::BallAndSocket)
	joints.push_back(i);
    }
}

void ActiveRobotGeodesicManifold::InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx) 
{ 
  if(joints.empty()) {
    dx.sub(b,a);
    return;
  }
  dofs.Map(a,xq);
  dofs.Map(b,yq);
  Assert(u==0);
  ::InterpolateDerivative(robot,xq,yq,tempq);
  dx.resize(dofs.Size());
  dofs.InvMap(tempq,dx);
}

void ActiveRobotGeodesicManifold::InterpolateDerivA(const Config& a,const Config& b,Real u,const Vector& da,Vector& dx) 
{
  FatalError("Not implemented"); 
}

void ActiveRobotGeodesicManifold::InterpolateDerivB(const Config& a,const Config& b,Real u,const Vector& db,Vector& dx) 
{ 
  FatalError("Not implemented"); 
}

void ActiveRobotGeodesicManifold::InterpolateDeriv2(const Config& a,const Config& b,Real u,Vector& ddx) 
{ 
  FatalError("Not implemented"); 
}

void ActiveRobotGeodesicManifold::Integrate(const Config& a,const Vector& da,Config& b)
{
  if(joints.empty()) {
    b.add(a,da); 
    return;
  }  
  dofs.Map(a,xq);
  dofs.Map(da,yq);
  ::Integrate(robot,xq,yq,tempq);
  b.resize(dofs.Size());
  dofs.InvMap(tempq,b);
}





SingleRobotCSpace::SingleRobotCSpace(RobotWorld& _world,int _index,WorldPlannerSettings* _settings)
  :world(_world),index(_index),settings(_settings),collisionPairsInitialized(false)
{
  Assert(settings != NULL);
  Assert((int)settings->robotSettings.size() > _index);
}

SingleRobotCSpace::SingleRobotCSpace(const SingleRobotCSpace& space)
  :world(space.world),index(space.index),settings(space.settings),collisionPairsInitialized(false)
{}

int SingleRobotCSpace::NumDimensions() const
{
  return (int)world.robots[index].robot->links.size();
}

Robot* SingleRobotCSpace::GetRobot() const
{
  return world.robots[index].robot;
}

bool SingleRobotCSpace::CheckJointLimits(const Config& x)
{
  Robot* robot=GetRobot();
  robot->UpdateConfig(x);
  for(size_t i=0;i<robot->joints.size();i++) {
    if(robot->joints[i].type == RobotJoint::Normal || robot->joints[i].type == RobotJoint::Weld) {
      int k=robot->joints[i].linkIndex;
      if(x(k) < robot->qMin(k) || x(k) > robot->qMax(k)) {
	//printf("Joint %d value %g out of bounds [%g,%g]\n",i,x(i),robot->qMin(i),robot->qMax(i));
	return false;
      }
    }
  }
  for(size_t i=0;i<robot->drivers.size();i++) {
    Real v=robot->GetDriverValue(i);
    if(v < robot->drivers[i].qmin || v > robot->drivers[i].qmax) {
      //printf("Driver %d value %g out of bounds [%g,%g]\n",i,v,robot->drivers[i].qmin,robot->drivers[i].qmax);
      return false;
    }
  }
  return true;
}

void SingleRobotCSpace::InitializeCollisionPairs()
{
  collisionPairsInitialized=true;
  int id = world.RobotID(index);
  collisionPairs.resize(0);
  collisionQueries.resize(0);
  settings->EnumerateCollisionQueries(world,id,-1,collisionPairs,collisionQueries);
}

void SingleRobotCSpace::Sample(Config& x)
{
  Robot* robot = GetRobot();
  const AABB3D& bb=settings->robotSettings[index].worldBounds;
  x = robot->q;
  for(size_t i=0;i<robot->joints.size();i++) {
    if(robot->joints[i].type == RobotJoint::Normal) {
      int k=robot->joints[i].linkIndex;
      x(k) = Rand(robot->qMin(k),robot->qMax(k));
    }
    else if(robot->joints[i].type == RobotJoint::Spin) {
      int k=robot->joints[i].linkIndex;
      x(k) = Rand(0,TwoPi);
    }
    else if(robot->joints[i].type == RobotJoint::Floating) {
      //generate a floating base
      RigidTransform T;
      QuaternionRotation qr;
      RandRotation(qr);
      qr.getMatrix(T.R);
      T.t.x = Rand(bb.bmin.x,bb.bmax.x);
      T.t.y = Rand(bb.bmin.y,bb.bmax.y);
      T.t.z = Rand(bb.bmin.z,bb.bmax.z);
      robot->SetJointByTransform(i,robot->joints[i].linkIndex,T);
      vector<int> indices;
      robot->GetJointIndices(i,indices);
      for(size_t k=0;k<indices.size();k++)
	x(indices[k]) = robot->q(indices[k]);
    }
  }
  for(size_t i=0;i<robot->drivers.size();i++) {
    if(robot->drivers[i].type != RobotJointDriver::Normal) {
      Real val = Rand(robot->drivers[i].qmin,robot->drivers[i].qmax);
      robot->SetDriverValue(i,val);
      for(size_t j=0;j<robot->drivers[i].linkIndices.size();j++)
	x(robot->drivers[i].linkIndices[j]) = robot->q(robot->drivers[i].linkIndices[j]);
    }
  }
  robot->NormalizeAngles(x);
}

void SingleRobotCSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  Robot* robot = GetRobot();
  x = c;
  const Vector& w=settings->robotSettings[index].distanceWeights;
  if(w.n==0) {
    for(int i=0;i<x.n;i++) 
      x(i) += Rand(-r,r);
  }
  else { 
    for(int i=0;i<x.n;i++) 
      x(i) += Rand(-r,r)/w(i);
  }
  for(size_t i=0;i<robot->joints.size();i++) {
    if(robot->joints[i].type == RobotJoint::Weld) {
      int k=robot->joints[i].linkIndex;
      x(k) = c(k);
    }
  }
  for(size_t i=0;i<robot->drivers.size();i++) {
    if(robot->drivers[i].type != RobotJointDriver::Normal) {
      robot->UpdateConfig(c);
      Real val = robot->GetDriverValue(i);
      Real scale = 1.0;
      if(w.n != 0) {
	scale = 0;
	for(size_t j=0;j<robot->drivers[i].linkIndices.size();j++)
	  scale += Sqr(w(robot->drivers[i].linkIndices[j]));
	scale = Sqrt(scale);
      }
      robot->SetDriverValue(i,val + scale*Rand(-r,r));
      for(size_t j=0;j<robot->drivers[i].linkIndices.size();j++)
	x(robot->drivers[i].linkIndices[j]) = robot->q(robot->drivers[i].linkIndices[j]);
    }
  }
}

int SingleRobotCSpace::NumObstacles()
{
  if(!collisionPairsInitialized) InitializeCollisionPairs();
  return GetRobot()->joints.size()+collisionPairs.size();
}

string SingleRobotCSpace::ObstacleName(int obstacle)
{
  Robot* robot=GetRobot();
  if(obstacle < (int)robot->joints.size()) {
    stringstream ss;
    int link = robot->joints[obstacle].linkIndex;
    ss<<"joint_limit["<<robot->LinkName(link)<<"]";
    return ss.str();
  }
  if(!collisionPairsInitialized) InitializeCollisionPairs();
  obstacle -= robot->links.size();

  stringstream ss;
  ss<<"coll["<<world.GetName(collisionPairs[obstacle].first)<<","<<world.GetName(collisionPairs[obstacle].second)<<"]";
  return ss.str();
}

bool SingleRobotCSpace::IsFeasible(const Config& x,int constraint)
{
  Robot* robot=GetRobot();
  if(constraint < (int)robot->joints.size()) {
    if(robot->joints[constraint].type == RobotJoint::Normal  || robot->joints[constraint].type == RobotJoint::Weld) {
      int k=robot->joints[constraint].linkIndex;
      if(x(k) < robot->qMin(k) || x(k) > robot->qMax(k))
	return false;
    }
    return true;
  }

  /* TODO: driver limits
  for(size_t i=0;i<robot->drivers.size();i++) {
    Real v=robot->GetDriverValue(i);
    jointLimitDist = Min(jointLimitDist,v-robot->drivers[i].qmin);
    jointLimitDist = Min(jointLimitDist,robot->drivers[i].qmax-v);
  }
  */

  constraint -= robot->joints.size();
  robot->UpdateConfig(x);
  robot->UpdateGeometry();

  if(!collisionPairsInitialized) InitializeCollisionPairs();
  return !collisionQueries[constraint].Collide();
}


bool SingleRobotCSpace::IsFeasible(const Config& x)
{
  if(!CheckJointLimits(x)) {
    //printf("Configuration is not in joint limits\n");
    return false;
  }
  //this is already called in CheckJointLimits
  //robot->UpdateConfig(x);
  return CheckCollisionFree();
}

bool SingleRobotCSpace::CheckCollisionFree()
{
  Robot* robot = GetRobot();
  robot->UpdateGeometry();

  if(!collisionPairsInitialized) InitializeCollisionPairs();

  for(size_t i=0;i<collisionQueries.size();i++)
    if(collisionQueries[i].Collide()) return false;
  return true;
}

void SingleRobotCSpace::CheckObstacles(const Config& x,vector<bool>& infeasible)
{
  infeasible.resize(NumObstacles(),false);
  Robot* robot=GetRobot();
  robot->UpdateConfig(x);
  for(size_t i=0;i<robot->joints.size();i++) {
    if(robot->joints[i].type == RobotJoint::Normal || robot->joints[i].type == RobotJoint::Weld) {
      int k=robot->joints[i].linkIndex;
      infeasible[i] = (x(k) < robot->qMin(k) || x(k) > robot->qMax(k));
    }
  }
  for(size_t i=0;i<robot->drivers.size();i++) {
    Real v=robot->GetDriverValue(i);
    if(v < robot->drivers[i].qmin || v > robot->drivers[i].qmax) {
      //TODO: what about driver violations
      //fprintf(stderr,"Warning, driver %d violation...\n");
      //infeasible[i] = true;
    }
  }

  robot->UpdateConfig(x);
  robot->UpdateGeometry();

  if(!collisionPairsInitialized) InitializeCollisionPairs();
  for(size_t i=0;i<collisionQueries.size();i++) 
    infeasible[i+(int)robot->joints.size()]=collisionQueries[i].Collide();
}


EdgePlanner* SingleRobotCSpace::LocalPlanner(const Config& a,const Config& b,int obstacle)
{
  if(obstacle < (int)GetRobot()->joints.size()) {
    return new TrueEdgePlanner(this,a,b);
  }
  SingleObstacleCSpace* ospace = new SingleObstacleCSpace(this,obstacle);
  return new EdgePlannerWithCSpaceContainer(ospace,new BisectionEpsilonEdgePlanner(ospace,a,b,settings->robotSettings[index].collisionEpsilon));
}

EdgePlanner* SingleRobotCSpace::LocalPlanner(const Config& a,const Config& b)
{
  return new BisectionEpsilonExplicitEdgePlanner(this,a,b,settings->robotSettings[index].collisionEpsilon);
  //return new ExplicitEdgePlanner(this,a,b);
}

Real SingleRobotCSpace::Distance(const Config& x, const Config& y)
{
  //Real sum = 0;
  Real vmax = 0;
  Robot* robot = world.robots[index].robot;
  const Vector& w=settings->robotSettings[index].distanceWeights;
  for(size_t i=0;i<robot->joints.size();i++) {
    switch(robot->joints[i].type) {
    case RobotJoint::Weld:
      break;
    case RobotJoint::Normal:
      {
	int l=robot->joints[i].linkIndex;
	if(w.n==0)
	  //sum += Sqr(x(l)-y(l));
	  vmax = Max(vmax,Abs(x(l)-y(l)));
	else
	  //sum += w(l)*Sqr(x(l)-y(l));
	  vmax = Max(vmax,w(l)*Abs(x(l)-y(l)));
      }
      break;
    case RobotJoint::Floating:
      {
	vector<int> indices;
	robot->GetJointIndices(i,indices);
	Assert(indices.size()==6);
	int l = indices.back();
	int tx = indices[0];
	int ty = indices[1];
	int tz = indices[2];
	int rz = indices[3];
	//int ry = indices[4];
	//int rx = indices[5];
	robot->UpdateConfig(x);
	RigidTransform Tx = robot->links[l].T_World;
	robot->UpdateConfig(y);
	RigidTransform Ty = robot->links[l].T_World;
	RigidTransform T; T.mulInverseB(Tx,Ty);
	Real cosangle = Half*(T.R.trace()-One);
	cosangle=Clamp(cosangle,-One,One);
	Real rdiff = Acos(cosangle);
	if(w.n==0)
	  //sum += T.t.normSquared() + Sqr(rdiff);
	  vmax = Max(vmax,Sqrt(T.t.normSquared() + Sqr(rdiff)));
	else 
	  //sum += w(tx)*Sqr(T.t.x)+w(ty)*Sqr(T.t.y)+w(tz)*Sqr(T.t.z)+w(rz)*Sqr(rdiff);
	  vmax = Max(vmax,Sqrt(w(tx)*Sqr(T.t.x)+w(ty)*Sqr(T.t.y)+w(tz)*Sqr(T.t.z)+w(rz)*Sqr(rdiff)));
      }
      break;
    default:
      FatalError("TODO");
    }
  }
    //return Sqrt(sum);
    return vmax;
}

void SingleRobotCSpace::Interpolate(const Config& x,const Config& y,Real u,Config& out)
{
  Robot* robot = world.robots[index].robot;
  ::Interpolate(*robot,x,y,u,out);
}

void SingleRobotCSpace::Midpoint(const Config& x,const Config& y,Config& out)
{
  Interpolate(x,y,0.5,out);
}

/*
vector<pair<int,int> > linkIndices;
vector<vector<Geometry::CollisionMeshQueryEnhanced> > linkCollisions;
void GetCollisionList(RobotWorld& world,int robot,WorldPlannerSettings* settings)
{
  if(linkCollisions.empty()) {
    linkIndices.resize(world.robots[robot].robot->links.size());
    linkCollisions.resize(world.robots[robot].robot->links.size());
    for(size_t i=0;i<world.robots[robot].robot->links.size();i++) {
      linkIndices[i].first = (int)i;
      linkIndices[i].second = -1;
    }
    vector<pair<int,int> > pairs;
    settings->EnumerateCollisionPairs(world,pairs);
    for(size_t i=0;i<pairs.size();i++) {
      pair<int,int> l1,l2;
      l1 = world.IsRobotLink(pairs[i].first);
      l2 = world.IsRobotLink(pairs[i].second);
      if(l1.first == robot && l1.second >= 0 && l2.first == robot && l2.second >= 0) { //involves two robot links
	linkIndices.push_back(pair<int,int>(l1.second,l2.second));
	linkCollisions.resize(linkCollisions.size()+1);
	settings->EnumerateCollisionQueries(world,pairs[i].first,pairs[i].second,linkCollisions.back());
      }
      else if(l1.first == robot && l1.second >= 0) {
	//find existing pair
	settings->EnumerateCollisionQueries(world,pairs[i].first,pairs[i].second,linkCollisions[l1.second]);
      }
      else if(l2.first == robot && l2.second >= 0) {
	settings->EnumerateCollisionQueries(world,pairs[i].first,pairs[i].second,linkCollisions[l2.second]);
      }
    }
    for(size_t i=0;i<linkCollisions.size();i++)
      printf("Collide link %d %d has %d elements to collide\n",linkIndices[i].first,linkIndices[i].second,linkCollisions[i].size());
    getchar();
  }
}
*/


void SingleRobotCSpace::GetJointLimits(Vector& bmin,Vector& bmax)
{
  Robot* robot=GetRobot();
  bmin.resize(robot->links.size(),-Inf);
  bmax.resize(robot->links.size(),Inf);
  for(size_t i=0;i<robot->links.size();i++) {
    if(robot->joints[i].type == RobotJoint::Normal) {
      int k=robot->joints[i].linkIndex;
      bmin(k) = robot->qMin(k);
      bmax(k) = robot->qMax(k);
    }
    else if(robot->joints[i].type == RobotJoint::Spin) {
      int k=robot->joints[i].linkIndex;
      bmin(k) = 0;
      bmax(k) = TwoPi;
    }
  }
  /* TODO: driver limits
  robot->UpdateConfig(x);
  for(size_t i=0;i<robot->drivers.size();i++) {
    Real v=robot->GetDriverValue(i);
    jointLimitDist = Min(jointLimitDist,v-robot->drivers[i].qmin);
    jointLimitDist = Min(jointLimitDist,robot->drivers[i].qmax-v);
  }
  */
}

Real SingleRobotCSpace::FreeWorkspaceBound(const Config& x,int constraint)
{
  Robot* robot=GetRobot();
  robot->UpdateConfig(x);
  robot->UpdateGeometry();
  
  if(!collisionPairsInitialized) InitializeCollisionPairs();

  constraint -= robot->joints.size();
  Assert(constraint >= 0 && constraint < (int)collisionPairs.size());
  return collisionQueries[constraint].Distance(0.0,0.0);
}

Real SingleRobotCSpace::WorkspaceMovementBound(const Config& x,const Vector& dx,int constraint)
{
  Robot* robot=GetRobot();

  if(!collisionPairsInitialized) InitializeCollisionPairs();
  constraint -= robot->joints.size();
  Assert(constraint >= 0 && constraint < (int)collisionPairs.size());

  int link=world.IsRobotLink(collisionPairs[constraint].first).second;
  pair<int,int> endid=world.IsRobotLink(collisionPairs[constraint].second);
  int endLink = (endid.first==index ? endid.second : -1);
  if(endLink >= 0) {
    //checking self collision, what to do?
    int lca = robot->LCA(link,endLink);
    if(link != lca && endLink != lca)
      FatalError("Can't do branching structures yet\n");
    if(lca == link) //swap
      swap(link,endLink);
  }

  //printf("Getting lipschitz bounds for %d -> %d\n",link,endLink);

  //compute lipschitz constants
  if(robot->lipschitzMatrix.isEmpty()) {
    robot->ComputeLipschitzMatrix();
  }

  Real d=0;
  int j=link;
  while(j != endLink) {
    Assert( j >= 0 && j < (int)robot->links.size());
    d += Abs(dx(j)) * robot->lipschitzMatrix(j,link);
    j = robot->parents[j];
  }
  return d;
}







SingleRobotCSpace2::SingleRobotCSpace2(RobotWorld& world,int index,
		    WorldPlannerSettings* settings)
  :SingleRobotCSpace(world,index,settings)
{}

SingleRobotCSpace2::SingleRobotCSpace2(const SingleRobotCSpace& space)
  :SingleRobotCSpace(space)
{}

int SingleRobotCSpace2::NumDimensions() const
{
  return SingleRobotCSpace::NumDimensions();
}

void SingleRobotCSpace2::FixDof(int dof,Real value)
{
  fixedDofs.push_back(dof);
  fixedValues.push_back(value);
}

void SingleRobotCSpace2::IgnoreCollisions(int a,int b)
{
  ignoreCollisions.push_back(pair<int,int>(a,b));
  if(!collisionPairsInitialized) {
    fprintf(stderr,"SingleRobotCSpace2::IgnoreCollisions: warning, collision structure needs reinitialization\n");
    Init();
  }
}

void SingleRobotCSpace2::Init()
{
  vector<bool> oldCheckCollisions(ignoreCollisions.size());
  for(size_t i=0;i<ignoreCollisions.size();i++) {
    oldCheckCollisions[i] = settings->collisionEnabled(ignoreCollisions[i].first,ignoreCollisions[i].second);
    settings->collisionEnabled(ignoreCollisions[i].first,ignoreCollisions[i].second) = false;
    settings->collisionEnabled(ignoreCollisions[i].second,ignoreCollisions[i].first) = false;
  }
  InitializeCollisionPairs();
  for(size_t i=0;i<ignoreCollisions.size();i++) {
    settings->collisionEnabled(ignoreCollisions[i].first,ignoreCollisions[i].second) = oldCheckCollisions[i];
    settings->collisionEnabled(ignoreCollisions[i].second,ignoreCollisions[i].first) = oldCheckCollisions[i];
  }
}

int SingleRobotCSpace2::NumObstacles()
{
  if(!collisionPairsInitialized) Init();
  return (int)fixedDofs.size()+SingleRobotCSpace::NumObstacles();
}

std::string SingleRobotCSpace2::ObstacleName(int obstacle)
{
  if(obstacle < (int)fixedDofs.size()) {
    char buf[256];
    sprintf(buf,"fixed[%d]=%g",fixedDofs[obstacle],fixedValues[obstacle]);
    return buf;
  }
  if(!collisionPairsInitialized) Init();
  return SingleRobotCSpace::ObstacleName(obstacle-(int)fixedDofs.size());
}

bool SingleRobotCSpace2::IsFeasible(const Config& x)
{
  for(size_t i=0;i<fixedDofs.size();i++) {
    if(x(fixedDofs[i]) != fixedValues[i]) {
      printf("Fixed degree of freedom %d does not match its fixed value %g!=%g\n",fixedDofs[i],x(fixedDofs[i]),fixedValues[i]);
      return false;
    }
  }
  if(!collisionPairsInitialized) Init();
  bool res=SingleRobotCSpace::IsFeasible(x);
  return res;
}

bool SingleRobotCSpace2::IsFeasible(const Config& x,int obstacle)
{
  if(obstacle < (int)fixedDofs.size())
    if(x(fixedDofs[obstacle]) != fixedValues[obstacle]) {
      return false;
    }
  if(!collisionPairsInitialized) Init();
  bool res=SingleRobotCSpace::IsFeasible(x,obstacle-(int)fixedDofs.size());
  return res;
}

EdgePlanner* SingleRobotCSpace2::LocalPlanner(const Config& a,const Config& b,int obstacle)
{
  if(obstacle < (int)fixedDofs.size())
    return new TrueEdgePlanner(this,a,b);
  return SingleRobotCSpace::LocalPlanner(a,b,obstacle-(int)fixedDofs.size());
}

EdgePlanner* SingleRobotCSpace2::LocalPlanner(const Config& a,const Config& b)
{
  return new BisectionEpsilonExplicitEdgePlanner(this,a,b,settings->robotSettings[index].collisionEpsilon);
  //return new ExplicitEdgePlanner(this,a,b);
}


void SingleRobotCSpace2::Sample(Config& x)
{
  SingleRobotCSpace::Sample(x);
  for(size_t j=0;j<fixedDofs.size();j++)
    x(fixedDofs[j]) = fixedValues[j];
}

void SingleRobotCSpace2::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  SingleRobotCSpace::SampleNeighborhood(c,r,x);
  for(size_t j=0;j<fixedDofs.size();j++)
    x(fixedDofs[j]) = fixedValues[j];
}


SingleRigidObjectCSpace::SingleRigidObjectCSpace(RobotWorld& _world,int _index,WorldPlannerSettings* _settings)
  :world(_world),index(_index),settings(_settings),collisionPairsInitialized(false)
{
  Assert(settings != NULL);
}

void SingleRigidObjectCSpace::Interpolate(const Config& x,const Config& y,Real u,Config& out)
{
  RigidTransform Ta,Tb,Tout;
  ConfigToTransform(x,Ta);
  ConfigToTransform(y,Tb);
  interpolate(Ta,Tb,u,Tout);
  TransformToConfig(Tout,out);
}

void SingleRigidObjectCSpace::Midpoint(const Config& x,const Config& y,Config& out)
{
  Interpolate(x,y,0.5,out);
}

Real SingleRigidObjectCSpace::Distance(const Config& x,const Config& y)
{
  RigidTransform Ta,Tb;
  ConfigToTransform(x,Ta);
  ConfigToTransform(y,Tb);
  Real d = Ta.t.distance(Tb.t);
  Matrix3 Rrel;
  Rrel.mulTransposeB(Ta.R,Tb.R);
  AngleAxisRotation aa;
  aa.setMatrix(Rrel);
  Real wt = settings->objectSettings[index].translationWeight;
  Real wr = settings->objectSettings[index].rotationWeight;
  d = Sqrt(d*d*wt + aa.angle*aa.angle*wr);
  return d;
}

RigidObject* SingleRigidObjectCSpace::GetObject() const
{
  return world.rigidObjects[index].object;
}

bool SingleRigidObjectCSpace::IsFeasible(const Config& q)
{
  RigidTransform T;
  ConfigToTransform(q,T);
  return CheckCollisionFree(T);
}


void SingleRigidObjectCSpace::InitializeCollisionPairs()
{
  collisionPairsInitialized=true;
  int id = world.RigidObjectID(index);
  collisionPairs.resize(0);
  collisionQueries.resize(0);
  settings->EnumerateCollisionQueries(world,id,-1,collisionPairs,collisionQueries);
}

bool SingleRigidObjectCSpace::CheckCollisionFree(const RigidTransform& T)
{
  GetObject()->T = T;
  GetObject()->UpdateGeometry();
  if(!collisionPairsInitialized) InitializeCollisionPairs();
  for(size_t i=0;i<collisionQueries.size();i++)
    if(collisionQueries[i].Collide()) return false;
  return true;
}

void SingleRigidObjectCSpace::Sample(Config& q)
{
  QuaternionRotation quat;
  RigidTransform T;
  RandRotation(quat);
  quat.getMatrix(T.R);
  SampleAABB(settings->objectSettings[index].worldBounds.bmin,settings->objectSettings[index].worldBounds.bmax,T.t);
  TransformToConfig(T,q);
}

void SingleRigidObjectCSpace::SampleNeighborhood(const Config& c,Real r,Config& q)
{
  AngleAxisRotation aa;
  RigidTransform T,T0;
  aa.angle = Rand(0,r);
  SampleSphere(1.0,aa.axis);
  aa.getMatrix(T.R);
  SampleCube(r,T.t);
  ConfigToTransform(c,T0);
  TransformToConfig(T*T0,q);
}

EdgePlanner* SingleRigidObjectCSpace::LocalPlanner(const Config& a,const Config& b)
{
  return new BisectionEpsilonEdgePlanner(this,a,b,settings->objectSettings[index].collisionEpsilon);
}



MultiRobotCSpace::MultiRobotCSpace(RobotWorld& _world,WorldPlannerSettings* _settings)
  :world(_world),settings(_settings)
{
}

MultiRobotCSpace::MultiRobotCSpace(const MultiRobotCSpace& space)
  :world(space.world),settings(space.settings)
{
}


void MultiRobotCSpace::InitRobots(const vector<int>& indices)
{
  robot.elements.clear();
  robotElementIDs.resize(indices.size());
  elementSpaces.resize(indices.size());
  for(size_t i=0;i<indices.size();i++) {
    robot.Add(world.robots[indices[i]].robot,world.robots[indices[i]].name.c_str());
    robotElementIDs[i] = world.RobotID(indices[i]);
    elementSpaces[i] = new SingleRobotCSpace(world,indices[i],settings);
  }
}

void MultiRobotCSpace::AddRobot(int index)
{
  int element = robot.Add(world.robots[index].robot,world.robots[index].name.c_str());
  robotElementIDs.push_back(world.RobotID(index));
  elementSpaces.push_back(new SingleRobotCSpace(world,index,settings));
}

void MultiRobotCSpace::AddRigidObject(int index)
{
  int element = robot.Add(world.rigidObjects[index].object,world.rigidObjects[index].name.c_str());
  robotElementIDs.push_back(world.RigidObjectID(index));
  elementSpaces.push_back(new SingleRigidObjectCSpace(world,index,settings));
}


int MultiRobotCSpace::NumDimensions() const
{
  return robot.NumDof();
}

void MultiRobotCSpace::Sample(Config& x)
{
  x.resize(NumDimensions());
  vector<Config> xelements;
  robot.SplitRefs(x,xelements);
  for(size_t i=0;i<elementSpaces.size();i++)
    elementSpaces[i]->Sample(xelements[i]);
}

void MultiRobotCSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  x.resize(NumDimensions());
  vector<Config> celements,xelements;
  robot.SplitRefs(c,celements);
  robot.SplitRefs(x,xelements);
  for(size_t i=0;i<elementSpaces.size();i++)
    elementSpaces[i]->SampleNeighborhood(celements[i],r,xelements[i]);
}

bool MultiRobotCSpace::IsFeasible(const Config& x)
{
  vector<Config> xelements;
  robot.SplitRefs(x,xelements);
  for(size_t i=0;i<elementSpaces.size();i++) {
    if(!elementSpaces[i]->IsFeasible(xelements[i])) return false;
  }
  //TODO: self-collision checking. 
  //eliminate some of the collision checks
  return true;
}

EdgePlanner* MultiRobotCSpace::LocalPlanner(const Config& a,const Config& b)
{
  Real minEps=1.0;
  for(size_t i=0;i<robotElementIDs.size();i++) {
    int id = robotElementIDs[i];
    int rindex = world.IsRobot(id);
    int oindex = world.IsRigidObject(id);
    if(rindex >= 0)
      minEps = Min(minEps,settings->robotSettings[rindex].collisionEpsilon);
    else
      minEps = Min(minEps,settings->objectSettings[oindex].collisionEpsilon);
  }
  return new BisectionEpsilonEdgePlanner(this,a,b,minEps);
}

Real MultiRobotCSpace::Distance(const Config& x, const Config& y)
{
  vector<Config> xelements,yelements;
  robot.SplitRefs(x,xelements);
  robot.SplitRefs(y,yelements);

  Real d=0;
  for(size_t i=0;i<elementSpaces.size();i++) 
    d+=Sqr(elementSpaces[i]->Distance(xelements[i],yelements[i]));
  return Sqrt(d);
}

void MultiRobotCSpace::Interpolate(const Config& x,const Config& y,Real u,Config& out)
{
  vector<Config> xelements,yelements,outelements;
  robot.SplitRefs(x,xelements);
  robot.SplitRefs(y,yelements);  
  robot.SplitRefs(out,outelements);  
  for(size_t i=0;i<elementSpaces.size();i++) 
    elementSpaces[i]->Interpolate(xelements[i],yelements[i],u,outelements[i]);
}

void MultiRobotCSpace::Midpoint(const Config& x,const Config& y,Config& out)
{
  vector<Config> xelements,yelements,outelements;
  robot.SplitRefs(x,xelements);
  robot.SplitRefs(y,yelements);  
  robot.SplitRefs(out,outelements);  
  for(size_t i=0;i<elementSpaces.size();i++) 
    elementSpaces[i]->Midpoint(xelements[i],yelements[i],outelements[i]);
}
