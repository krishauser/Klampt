#include "RobotCSpace.h"
#include "Klampt/Modeling/Interpolate.h"
#include <KrisLibrary/math/angle.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math3d/rotation.h>
#include <KrisLibrary/math3d/random.h>
#include <KrisLibrary/math3d/interpolate.h>
#include <KrisLibrary/robotics/Rotation.h>
#include <KrisLibrary/planning/EdgePlanner.h>
#include <KrisLibrary/planning/EdgePlannerHelpers.h>
#include <KrisLibrary/planning/CSetHelpers.h>
#include <KrisLibrary/planning/CSpaceHelpers.h>
#include <KrisLibrary/Timer.h>
#include <sstream>

namespace Klampt {

Real RandLaplacian()
{
  Real v = Rand();
  if(v==0) v = Epsilon;
  return -Log(v);
}

Real RandTwoSidedLaplacian()
{
  if(RandBool(0.5)) return RandLaplacian();
  else return -RandLaplacian();
}

Real SafeRand(Real a,Real b,Real unboundedStdDeviation=1.0)
{
  if(IsInf(a) && IsInf(b)) return RandTwoSidedLaplacian()*unboundedStdDeviation;
  else if(IsInf(a)) return b-RandLaplacian()*unboundedStdDeviation;
  else if(IsInf(b)) return a+RandLaplacian()*unboundedStdDeviation;
  else return Rand(a,b);
}

Real SafeAngleRand(Real a,Real b)
{
  if(b-a < TwoPi)
    return Rand(a,b);
  else
    return Rand(0,TwoPi);
}

RobotCSpace::RobotCSpace(RobotModel& _robot)
  :robot(_robot),norm(2.0)
{
  floatingRotationWeight=1;
  floatingRotationRadiusScale=1;

  for(int i=0;i<_robot.q.n;i++) {
    if(IsInf(_robot.qMin[i])!= -1 || IsInf(_robot.qMax[i]) != 1)
      AddConstraint(robot.LinkName(i)+"_joint_limit",new AxisRangeSet(i,_robot.qMin[i],_robot.qMax[i]));
  }
}

RobotCSpace::RobotCSpace(const RobotCSpace& space)
:robot(space.robot),norm(space.norm),
jointWeights(space.jointWeights),floatingRotationWeight(space.floatingRotationWeight),
jointRadiusScale(space.jointRadiusScale),floatingRotationRadiusScale(space.floatingRotationRadiusScale),
unboundedStdDeviation(1.0)
{
  CopyConstraints(&space);
}

int RobotCSpace::NumDimensions() 
{
  return robot.q.n;
}

string RobotCSpace::VariableName(int i)
{
  return robot.LinkName(i);
}

void RobotCSpace::Sample(Config& q)
{
  for(size_t i=0;i<robot.joints.size();i++) {
    int link = robot.joints[i].linkIndex;
    switch(robot.joints[i].type) {
    case RobotModelJoint::Weld:
      break;
    case RobotModelJoint::Normal:
      robot.q(link) = Rand(robot.qMin(link),robot.qMax(link));
      break;
    case RobotModelJoint::Spin:
      robot.q(link) = Rand(0,TwoPi);
      break;
    case RobotModelJoint::FloatingPlanar:
      {
      int p = robot.parents[link];
      assert(p>=0);
      int pp = robot.parents[p];
      assert(pp>=0);
      robot.q(link) = SafeAngleRand(robot.qMax(link),robot.qMin(link));
      robot.q(p) = SafeRand(robot.qMin(p),robot.qMax(p),unboundedStdDeviation);
      robot.q(pp) = SafeRand(robot.qMin(pp),robot.qMax(pp),unboundedStdDeviation);
      break;
      }
    case RobotModelJoint::Floating:
    case RobotModelJoint::BallAndSocket:
      {
        RigidTransform T;
        T.t.x = SafeRand(robot.qMin(link),robot.qMax(link),unboundedStdDeviation);
        T.t.y = SafeRand(robot.qMin(link+1),robot.qMax(link+1),unboundedStdDeviation);
        T.t.z = SafeRand(robot.qMin(link+2),robot.qMax(link+2),unboundedStdDeviation);
        QuaternionRotation qr;
        //TODO: if limits are specified for the rotation DOFs, should we just sample them accordingly?
        if(AngleCCWDiff(robot.qMax(link+3),robot.qMin(link+3)) < TwoPi || 
          AngleCCWDiff(robot.qMax(link+4),robot.qMin(link+4)) < TwoPi ||
          AngleCCWDiff(robot.qMax(link+5),robot.qMin(link+5)) < TwoPi) {
          robot.q(link) = T.t.x;
          robot.q(link+1) = T.t.y;
          robot.q(link+2) = T.t.z;
          robot.q(link+3) = SafeAngleRand(robot.qMax(link+3),robot.qMin(link+3));
          robot.q(link+4) = SafeAngleRand(robot.qMax(link+4),robot.qMin(link+4));
          robot.q(link+5) = SafeAngleRand(robot.qMax(link+4),robot.qMin(link+5));
        }
        else {
          RandRotation(qr);
          qr.getMatrix(T.R);
          robot.SetJointByTransform(i,robot.joints[i].linkIndex,T);
        }
      }
      break;
    default:
      break;
    }
  }
  for(size_t i=0;i<robot.drivers.size();i++) {
    if(robot.drivers[i].type != RobotModelDriver::Normal) {
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
    if(jointRadiusScale.empty()) {
      if(jointWeights.empty()) ri = r;
      else ri = r / jointWeights[i];
    } 
    else ri = r / jointRadiusScale[i];
    int link = robot.joints[i].linkIndex;
    switch(robot.joints[i].type) {
    case RobotModelJoint::Weld:
      break;
    case RobotModelJoint::Normal:
      robot.q(link) += Rand(-ri,ri);
      //reflection sampling strategy
      if(robot.q(link) < robot.qMin(link))
        robot.q(link) = robot.qMin(link) + (robot.qMin(link)-robot.q(link));
      if(robot.q(link) > robot.qMax(link))
        robot.q(link) = robot.qMax(link) - (robot.q(link)-robot.qMax(link));
      robot.q(link) = Clamp(robot.q(link),robot.qMin(link),robot.qMax(link));
      break;
    case RobotModelJoint::Spin:
      robot.q(link) += Rand(-ri,ri);
      break;
    case RobotModelJoint::Floating:
    case RobotModelJoint::BallAndSocket:
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
    if(robot.drivers[i].type != RobotModelDriver::Normal) {
      Real val = robot.GetDriverValue(i);
      Real scale = 1.0;
      //TODO: figure out the proper scale factor
      val += scale*Rand(-r,r);
      //reflection sampling strategy
      if(val < robot.drivers[i].qmin)
        val = robot.drivers[i].qmin + (robot.drivers[i].qmin-val);
      if(val > robot.drivers[i].qmax)
        val = robot.drivers[i].qmax - (val-robot.drivers[i].qmax);
      val = Clamp(val,robot.drivers[i].qmin,robot.drivers[i].qmax);
      robot.SetDriverValue(i,val + Rand(-r,r));
    }
  }
  q = robot.q;
  robot.NormalizeAngles(q);
}

void RobotCSpace::Interpolate(const Config& x,const Config& y,Real u,Config& out)
{
  Klampt::Interpolate(robot,x,y,u,out);
}

Real RobotCSpace::Distance(const Config& a,const Config& b)
{
  if(jointWeights.empty())
    return Klampt::Distance(robot,a,b,norm,floatingRotationWeight);
  else
    return Klampt::Distance(robot,a,b,norm,jointWeights,floatingRotationWeight);
}

void RobotCSpace::Properties(PropertyMap& map)
{
  int euclidean = 1;
  Real v = 1;
  int dim = robot.q.n;
  Vector vmin(robot.qMin),vmax(robot.qMax);
  vector<Real> weights;
  if(jointWeights.empty()) weights.resize(robot.q.n,1.0);
  else weights = jointWeights;
  for(size_t i=0;i<robot.joints.size();i++) {
    int link = robot.joints[i].linkIndex;
    switch(robot.joints[i].type) {
    case RobotModelJoint::Normal:
      v*=robot.qMax(link)-robot.qMin(link);
      break;
    case RobotModelJoint::Weld:
      dim--;
      break;
    case RobotModelJoint::Spin:
      v*=TwoPi;
      vmin(link) = 0;
      vmax(link) = TwoPi;
      euclidean = 0;
      break;
    case RobotModelJoint::Floating:
    case RobotModelJoint::BallAndSocket:
      v*= Pi*4.0/3.0;
      weights[link] = weights[link-1] = weights[link-2] = floatingRotationWeight;
      euclidean = 0;
      break;
    case RobotModelJoint::FloatingPlanar:
      vmin(link) = 0;
      vmax(link) = TwoPi;
      v*= TwoPi;
      weights[link] = floatingRotationWeight;
      euclidean = 0;
      break;
    default:
      euclidean = 0;
      break;
    }
  }

  map.set("euclidean",euclidean);
  map.set("geodesic",1);
  if(dim < robot.q.n) {
    map.set("submanifold",1);
    map.set("intrinsicDimension",dim);
  }
  map.setArray("minimum",vector<Real>(vmin));
  map.setArray("maximum",vector<Real>(vmax));
  map.set("volume",v);
  if(norm == 2)
    map.set("metric","weighted euclidean");
  else if(norm == 1)
    map.set("metric","weighted manhattan");
  else if(IsInf(norm))
    map.set("metric","weighted Linf");
  map.setArray("metricWeights",weights);
}

void RobotCSpace::InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx)
{ 
  Klampt::InterpolateDerivative(robot,a,b,u,dx);
}

void RobotCSpace::InterpolateDerivA(const Config& a,const Config& b,Real u,const Vector& da,Vector& dx) 
{ 
  dx.mul(da,1-u);
  for(size_t i=0;i<robot.joints.size();i++) {
    //int k=robot.joints[i].linkIndex;
    if(robot.joints[i].type == RobotModelJoint::Floating) {
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
      Assert(res);
      dtheta *= (1-u);
      dtheta.get(dx(indices[3]),dx(indices[4]),dx(indices[5]));
    }
    else if(robot.joints[i].type == RobotModelJoint::BallAndSocket) {
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
      Assert(res);
      dtheta *= (1-u);
      dtheta.get(dx(indices[0]),dx(indices[1]),dx(indices[2]));
    }
  }
}
void RobotCSpace::InterpolateDerivB(const Config& a,const Config& b,Real u,const Vector& db,Vector& dx) 
{ 
  dx.mul(db,u);
  for(size_t i=0;i<robot.joints.size();i++) {
    //int k=robot.joints[i].linkIndex;
    if(robot.joints[i].type == RobotModelJoint::Floating) {
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
      Assert(res);
      dtheta *= u;
      dtheta.get(dx(indices[3]),dx(indices[4]),dx(indices[5]));
    }
    else if(robot.joints[i].type == RobotModelJoint::BallAndSocket) {
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
      Assert(res);
      dtheta *= u;
      dtheta.get(dx(indices[0]),dx(indices[1]),dx(indices[2]));
    }
  }
}

void RobotCSpace::InterpolateDeriv2(const Config& a,const Config& b,Real u,Vector& ddx) { FatalError("Not implemented"); }

void RobotCSpace::Integrate(const Config& a,const Vector& da,Config& b)
{
  Klampt::Integrate(robot,a,da,b);
}





ActiveRobotCSpace::ActiveRobotCSpace(RobotModel& _robot,const ArrayMapping& _dofs)
  :robot(_robot),dofs(_dofs)
{
  xq=yq=tempq=robot.q;
  invMap.resize(robot.q.n,-1);
  Assert(!dofs.IsOffset());
  for(size_t i=0;i<dofs.mapping.size();i++)
    invMap[dofs.mapping[i]] = int(i);
  for(size_t i=0;i<robot.joints.size();i++)
    if(invMap[robot.joints[i].linkIndex] >= 0) 
      if(robot.joints[i].type == RobotModelJoint::Floating || robot.joints[i].type == RobotModelJoint::Spin || robot.joints[i].type == RobotModelJoint::FloatingPlanar || robot.joints[i].type == RobotModelJoint::BallAndSocket)
  joints.push_back(i);

  for(size_t i=0;i<dofs.mapping.size();i++) {
    if(IsInf(robot.qMin[dofs.mapping[i]]) != -1 || IsInf(robot.qMax[dofs.mapping[i]]) != 1)
      AddConstraint(robot.LinkName(dofs.mapping[i])+"_joint_limit",new AxisRangeSet(i,robot.qMin[dofs.mapping[i]],robot.qMax[dofs.mapping[i]]));
  }
}

int ActiveRobotCSpace::NumDimensions()
{
  return (int)dofs.mapping.size();
}

string ActiveRobotCSpace::VariableName(int i)
{
  return robot.LinkName(dofs.mapping[i]);
}

void ActiveRobotCSpace::Sample(Config& x)
{
  x.resize(dofs.mapping.size());
  for(size_t j=0;j<joints.size();j++) {
    int i=joints[j];
    int link = robot.joints[i].linkIndex;
    int k=invMap[link];
    switch(robot.joints[i].type) {
    case RobotModelJoint::Weld:
      break;
    case RobotModelJoint::Normal:
      x(k) = Rand(robot.qMin(link),robot.qMax(link));
      break;
    case RobotModelJoint::Spin:
      x(k) = Rand(0,TwoPi);
      break;
    case RobotModelJoint::Floating:
    case RobotModelJoint::BallAndSocket:
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
  Klampt::Interpolate(robot,xq,yq,u,tempq);
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
    case RobotModelJoint::Weld:
      break;
    case RobotModelJoint::Normal:
      {
	int l=invMap[robot.joints[i].linkIndex];
	norm << (x(l)-y(l));
      }
      break;
    case RobotModelJoint::Floating:
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


void ActiveRobotCSpace::Properties(PropertyMap& map)
{
  //TODO
}

void ActiveRobotCSpace::InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx) 
{ 
  if(joints.empty()) {
    dx.sub(b,a);
    return;
  }
  dofs.Map(a,xq);
  dofs.Map(b,yq);
  Assert(u==0);
  Klampt::InterpolateDerivative(robot,xq,yq,tempq);
  dx.resize(dofs.Size());
  dofs.InvMap(tempq,dx);
}

void ActiveRobotCSpace::InterpolateDerivA(const Config& a,const Config& b,Real u,const Vector& da,Vector& dx) 
{
  FatalError("Not implemented"); 
}

void ActiveRobotCSpace::InterpolateDerivB(const Config& a,const Config& b,Real u,const Vector& db,Vector& dx) 
{ 
  FatalError("Not implemented"); 
}

void ActiveRobotCSpace::InterpolateDeriv2(const Config& a,const Config& b,Real u,Vector& ddx) 
{ 
  FatalError("Not implemented"); 
}

void ActiveRobotCSpace::Integrate(const Config& a,const Vector& da,Config& b)
{
  if(joints.empty()) {
    b.add(a,da); 
    return;
  }  
  dofs.Map(a,xq);
  dofs.Map(da,yq);
  Klampt::Integrate(robot,xq,yq,tempq);
  b.resize(dofs.Size());
  dofs.InvMap(tempq,b);
}





SingleRobotCSpace::SingleRobotCSpace(WorldModel& _world,int _index,WorldPlannerSettings* _settings)
  :RobotCSpace(*_world.robots[_index]),world(_world),index(_index),settings(_settings),constraintsDirty(true)
{
  Assert(settings != NULL);
  Assert((int)settings->robotSettings.size() > _index);

  const Vector& w=settings->robotSettings[index].distanceWeights;
  RobotCSpace::jointWeights = w;

  Init();
}

SingleRobotCSpace::SingleRobotCSpace(const SingleRobotCSpace& space)
  :RobotCSpace(space),world(space.world),index(space.index),settings(space.settings),fixedDofs(space.fixedDofs),fixedValues(space.fixedValues),ignoreCollisions(space.ignoreCollisions),constraintsDirty(true)
{
  Init();
}

bool SingleRobotCSpace::CheckJointLimits(const Config& x)
{
  robot.UpdateConfig(x);
  for(size_t i=0;i<robot.joints.size();i++) {
    if(robot.joints[i].type == RobotModelJoint::Normal || robot.joints[i].type == RobotModelJoint::Weld) {
      int k=robot.joints[i].linkIndex;
      if(x(k) < robot.qMin(k) || x(k) > robot.qMax(k)) {
	//printf("Joint %d value %g out of bounds [%g,%g]\n",i,x(i),robot.qMin(i),robot.qMax(i));
	return false;
      }
    }
  }
  for(size_t i=0;i<robot.drivers.size();i++) {
    Real v=robot.GetDriverValue(i);
    if(v < robot.drivers[i].qmin || v > robot.drivers[i].qmax) {
      //printf("Driver %d value %g out of bounds [%g,%g]\n",i,v,robot.drivers[i].qmin,robot.drivers[i].qmax);
      return false;
    }
  }
  return true;
}

bool SingleRobotCSpace::UpdateGeometry(const Config& x)
{
  robot.UpdateConfig(x);
  robot.UpdateGeometry();
  return true;
}


void SingleRobotCSpace::FixDof(int dof,Real value)
{
  fixedDofs.push_back(dof);
  fixedValues.push_back(value);
  constraintsDirty = true;
}

void SingleRobotCSpace::IgnoreCollisions(int a,int b)
{
  ignoreCollisions.push_back(pair<int,int>(a,b));
  constraintsDirty = true;
}

///NOTE: REQUIRES GEOMETRY TO BE UPDATED AT X
class CollisionFreeSet : public CSet
{
public:
  Geometry::AnyCollisionQuery& query;
  Real lipschitzBound;

  CollisionFreeSet(Geometry::AnyCollisionQuery& q,Real _lipschitzBound=Inf):query(q),lipschitzBound(_lipschitzBound) {}
  virtual bool Contains(const Config& x) { return !query.Collide(); }
  virtual Real ObstacleDistance(const Config& x) {
    Real dworkspace = query.Distance(0.0,0.0);
    return dworkspace / lipschitzBound;
  }
};

void SingleRobotCSpace::Init()
{
  if(!constraintsDirty) return;
  constraints.resize(0);
  constraintNames.resize(0);

  //add in fixed DOF constraints
  vector<int> isfixed(robot.q.n,-1);
  for(size_t i=0;i<fixedDofs.size();i++)
    isfixed[fixedDofs[i]] = (int)i;
  for(int i=0;i<robot.q.n;i++) {
    if(isfixed[i] >= 0) {
      int k=isfixed[i];
      stringstream ss;
      ss<<"fixed["<<robot.LinkName(i)<<"]="<<fixedValues[k];
      AddConstraint(ss.str(),new AxisRangeSet(i,fixedValues[k],fixedValues[k]));
    }
    else {
      if(IsInf(robot.qMin[i])!= -1 || IsInf(robot.qMax[i]) != 1)
        AddConstraint(robot.LinkName(i)+"_joint_limit",new AxisRangeSet(i,robot.qMin[i],robot.qMax[i]));
    }
  }
  vector<bool> oldCheckCollisions(ignoreCollisions.size());
  for(size_t i=0;i<ignoreCollisions.size();i++) {
    oldCheckCollisions[i] = settings->collisionEnabled(ignoreCollisions[i].first,ignoreCollisions[i].second);
    settings->collisionEnabled(ignoreCollisions[i].first,ignoreCollisions[i].second) = false;
    settings->collisionEnabled(ignoreCollisions[i].second,ignoreCollisions[i].first) = false;
  }

  AddConstraint("update_geometry",std::bind(std::mem_fn(&SingleRobotCSpace::UpdateGeometry),this,std::placeholders::_1));

  int id = world.RobotID(index);
  collisionPairs.resize(0);
  collisionQueries.resize(0);
  settings->EnumerateCollisionQueries(world,id,-1,collisionPairs,collisionQueries);

  /*
  //compute lipschitz constants
  if(robot.lipschitzMatrix.isEmpty()) 
    robot.ComputeLipschitzMatrix();
  */
  
  for(size_t i=0;i<collisionPairs.size();i++)  {
    stringstream ss;
    ss<<"coll["<<world.GetName(collisionPairs[i].first)<<","<<world.GetName(collisionPairs[i].second)<<"]";

    /* 
    //TODO: c-space free movement bounds
    int link=world.IsRobotLink(collisionPairs[i].first).second;
    pair<int,int> endid=world.IsRobotLink(collisionPairs[i].second);
    int endLink = (endid.first==index ? endid.second : -1);
    int lca = 0;
    if(endLink >= 0) {
      //checking self collision, what to do?
      int lca = robot.LCA(link,endLink);
      if(link != lca && endLink != lca)
        FatalError("Can't do branching structures yet\n");
      if(lca == link) //swap
        swap(link,endLink);
    }

    //printf("Getting lipschitz bounds for %d -> %d\n",link,endLink);

    Real d=0;
    int j=link;
    while(j != lca && j >= 0) {
      Assert( j >= 0 && j < (int)robot.links.size());
      d += Abs(dx(j)) * robot.lipschitzMatrix(j,link);
      j = robot.parents[j];
    }
    j=endLink;
    while(j != lca && j >= 0) {
      Assert( j >= 0 && j < (int)robot.links.size());
      d += Abs(dx(j)) * robot.lipschitzMatrix(j,link);
      j = robot.parents[j];
    }
    */

    AddConstraint(ss.str(),new CollisionFreeSet(collisionQueries[i]));
  }

  for(size_t i=0;i<ignoreCollisions.size();i++) {
    settings->collisionEnabled(ignoreCollisions[i].first,ignoreCollisions[i].second) = oldCheckCollisions[i];
    settings->collisionEnabled(ignoreCollisions[i].second,ignoreCollisions[i].first) = oldCheckCollisions[i];
  }
  constraintsDirty = false;
}

void SingleRobotCSpace::Sample(Config& x)
{
  RobotCSpace::Sample(x);
  const AABB3D& bb=settings->robotSettings[index].worldBounds;
  for(size_t i=0;i<robot.joints.size();i++) {
    if(robot.joints[i].type == RobotModelJoint::Floating) {
      //generate a floating base position
      Vector3 p;
      p.x = Rand(bb.bmin.x,bb.bmax.x);
      p.y = Rand(bb.bmin.y,bb.bmax.y);
      p.z = Rand(bb.bmin.z,bb.bmax.z);
      vector<int> indices;
      robot.GetJointIndices(i,indices);
      for(size_t k=0;k<3;k++)
	x(indices[k]) = p[k];
    }
  }
  for(size_t j=0;j<fixedDofs.size();j++)
    x(fixedDofs[j]) = fixedValues[j];
}


void SingleRobotCSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  RobotCSpace::SampleNeighborhood(c,r,x);
  for(size_t j=0;j<fixedDofs.size();j++)
    x(fixedDofs[j]) = fixedValues[j];
}


bool SingleRobotCSpace::IsFeasible(const Config& x)
{
  ///This is faster than going through all constraints
  if(!CheckJointLimits(x)) return false;

  return CheckCollisionFree(x);
}

bool SingleRobotCSpace::CheckCollisionFree(const Config& x)
{
  UpdateGeometry(x);

  //this method may be faster for many-DOF robots because it does broad-phase checking first to eliminate candidate collisions
  int id = world.RobotID(index);
  vector<int> idrobot(1,id);
  vector<int> idothers;
  for(size_t i=0;i<world.terrains.size();i++)
    idothers.push_back(world.TerrainID(i));
  for(size_t i=0;i<world.rigidObjects.size();i++)
    idothers.push_back(world.RigidObjectID(i));
  for(size_t i=0;i<world.robots.size();i++) {
    if((int)i != index)
      idothers.push_back(world.RobotID(i));
  }
  //environment collision check
  pair<int,int> res = settings->CheckCollision(world,idrobot,idothers);
  if(res.first >= 0) {
    //printf("Collision found: %s (%d) - %s (%d)\n",world.GetName(res.first).c_str(),res.first,world.GetName(res.second).c_str(),res.second);
    return false;
  }
  //self collision check
  res = settings->CheckCollision(world,idrobot);
  if(res.first >= 0) {
    //printf("Self-collision found: %s %s\n",world.GetName(res.first).c_str(),world.GetName(res.second).c_str());
    return false;
  }
  return true;
}


EdgePlannerPtr SingleRobotCSpace::PathChecker(const Config& a,const Config& b,int obstacle)
{
  if(constraints[obstacle]->IsConvex()) {
    return make_shared<TrueEdgeChecker>(this,a,b);
  }
  auto ospace = make_shared<SubsetConstraintCSpace>(this,obstacle);
  return make_shared<EdgePlannerWithCSpaceContainer>(ospace,make_shared<EpsilonEdgeChecker>(ospace.get(),a,b,settings->robotSettings[index].collisionEpsilon));
}

EdgePlannerPtr SingleRobotCSpace::PathChecker(const Config& a,const Config& b)
{
  return make_shared<EpsilonEdgeChecker>(this,a,b,settings->robotSettings[index].collisionEpsilon);
  //uncomment this if you need an explicit edge planner
  //return make_shared<ExplicitEdgePlanner>(this,a,b);
}



/*
vector<pair<int,int> > linkIndices;
vector<vector<Geometry::CollisionMeshQueryEnhanced> > linkCollisions;
void GetCollisionList(WorldModel& world,int robot,WorldPlannerSettings* settings)
{
  if(linkCollisions.empty()) {
    linkIndices.resize(world.robots[robot]->links.size());
    linkCollisions.resize(world.robots[robot]->links.size());
    for(size_t i=0;i<world.robots[robot]->links.size();i++) {
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
  bmin.resize(robot.links.size(),-Inf);
  bmax.resize(robot.links.size(),Inf);
  for(size_t i=0;i<robot.links.size();i++) {
    if(robot.joints[i].type == RobotModelJoint::Normal) {
      int k=robot.joints[i].linkIndex;
      bmin(k) = robot.qMin(k);
      bmax(k) = robot.qMax(k);
    }
    else if(robot.joints[i].type == RobotModelJoint::Spin) {
      int k=robot.joints[i].linkIndex;
      bmin(k) = 0;
      bmax(k) = TwoPi;
    }
  }
  /* TODO: driver limits
  robot.UpdateConfig(x);
  for(size_t i=0;i<robot.drivers.size();i++) {
    Real v=robot.GetDriverValue(i);
    jointLimitDist = Min(jointLimitDist,v-robot.drivers[i].qmin);
    jointLimitDist = Min(jointLimitDist,robot.drivers[i].qmax-v);
  }
  */
}

void SingleRobotCSpace::Properties(PropertyMap& map) 
{
  RobotCSpace::Properties(map);
  if(!fixedDofs.empty()) {
    int dim;
    if(map.get("intrinsicDimension",dim))
      ;
    else {
      dim = robot.q.n;
      dim -= fixedDofs.size();
    }
    map.set("intrinsicDimension",dim);
    Real v;
    map.get("volume",v);
    vector<Real> minimum,maximum;
    map.getArray("minimum",minimum);
    map.getArray("maximum",maximum);
    for(size_t i=0;i<fixedDofs.size();i++) {
      int k=fixedDofs[i];
      if(robot.qMax[k] != robot.qMin[k])
	v /= (robot.qMax[k]-robot.qMin[k]);
      minimum[k] = maximum[k] = fixedValues[i];
    }
    map.set("volume",v);
    map.setArray("minimum",minimum);
    map.setArray("maximum",maximum);    
  }
}



SingleRigidObjectCSpace::SingleRigidObjectCSpace(WorldModel& _world,int _index,WorldPlannerSettings* _settings)
  :SE3CSpace(_settings->objectSettings[_index].worldBounds.bmin,_settings->objectSettings[_index].worldBounds.bmax),
  world(_world),index(_index),settings(_settings),constraintsDirty(true)
{
  Assert(settings != NULL);
  if(settings->objectSettings[index].translationWeight != 1.0)
    fprintf(stderr,"SingleRigidObjectCSpace: translation distance weight is not 1\n");
  SE3CSpace::SetAngleWeight(settings->objectSettings[index].rotationWeight);

  Init();
}

RigidObjectModel* SingleRigidObjectCSpace::GetObject() const
{
  return world.rigidObjects[index].get();
}

void SingleRigidObjectCSpace::IgnoreCollisions(int id)
{
  for(size_t i=0;i<collisionPairs.size();i++) {
    if(collisionPairs[i].second == id) {
      collisionPairs[i] = collisionPairs.back();
      collisionPairs.resize(collisionPairs.size()-1);
      collisionQueries[i] = collisionQueries.back();
      collisionQueries.resize(collisionQueries.size()-1);
    }
  }
}

void SingleRigidObjectCSpace::Init()
{
  if(!constraintsDirty) return;
  MultiCSpace::FlattenConstraints();
  constraints.resize(3);
  constraintNames.resize(3);

  CSet::CPredicate f = std::bind(std::mem_fn(&SingleRigidObjectCSpace::UpdateGeometry),this,std::placeholders::_1);
  CSpace::AddConstraint("update_geometry",f);

  if(collisionPairs.empty()) {
    int id = world.RigidObjectID(index);
    settings->EnumerateCollisionQueries(world,id,-1,collisionPairs,collisionQueries);
  }

  for(size_t i=0;i<collisionPairs.size();i++) 
  {
    stringstream ss;
    ss<<"coll["<<world.GetName(collisionPairs[i].first)<<","<<world.GetName(collisionPairs[i].second)<<"]";
    CSpace::AddConstraint(ss.str(),new CollisionFreeSet(collisionQueries[i]));
  }
  constraintsDirty = false;
}

bool SingleRigidObjectCSpace::UpdateGeometry(const Config& q)
{
  RigidTransform T;
  SE3CSpace::GetTransform(q,GetObject()->T);
  GetObject()->UpdateGeometry();
  return true;
}


EdgePlannerPtr SingleRigidObjectCSpace::PathChecker(const Config& a,const Config& b)
{
  return make_shared<EpsilonEdgeChecker>(this,a,b,settings->objectSettings[index].collisionEpsilon);
}


} // namespace Klampt