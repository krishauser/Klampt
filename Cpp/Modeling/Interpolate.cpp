#include "Interpolate.h"
#include <KrisLibrary/math/angle.h>
#include <KrisLibrary/math/metric.h>
#include <KrisLibrary/math3d/rotation.h>
#include <KrisLibrary/math3d/interpolate.h>
#include <KrisLibrary/robotics/Rotation.h>

namespace Klampt {

void Interpolate(RobotModel& robot,const Config& x,const Config& y,Real u,Config& out)
{
  Assert(&out != &robot.q);
  out.mul(x,1.0-u);
  out.madd(y,u);
  for(size_t i=0;i<robot.joints.size();i++) {
    switch(robot.joints[i].type) {
    case RobotModelJoint::Floating:
      {
	vector<int> indices;
	robot.GetJointIndices(i,indices);
	Vector3 oldrot(x(indices[3]),x(indices[4]),x(indices[5]));
	Vector3 newrot(y(indices[3]),y(indices[4]),y(indices[5]));
	assert(robot.links[indices[3]].w == Vector3(0,0,1));
	assert(robot.links[indices[4]].w == Vector3(0,1,0));
	assert(robot.links[indices[5]].w == Vector3(1,0,0));
	EulerAngleRotation ex(oldrot),ey(newrot),eu;
	Matrix3 Rx,Ry,Ru;
	ex.getMatrixZYX(Rx);
	ey.getMatrixZYX(Ry);
	interpolateRotation(Rx,Ry,u,Ru);
	eu.setMatrixZYX(Ru);
	eu.get(out(indices[3]),out(indices[4]),out(indices[5]));
      }
      break;
    case RobotModelJoint::BallAndSocket:
      {
	vector<int> indices;
	robot.GetJointIndices(i,indices);
	Vector3 oldrot(x(indices[0]),x(indices[1]),x(indices[2]));
	Vector3 newrot(y(indices[0]),y(indices[1]),y(indices[2]));
	assert(robot.links[indices[0]].w == Vector3(0,0,1));
	assert(robot.links[indices[1]].w == Vector3(0,1,0));
	assert(robot.links[indices[2]].w == Vector3(1,0,0));
	EulerAngleRotation ex(oldrot),ey(newrot),eu;
	Matrix3 Rx,Ry,Ru;
	ex.getMatrixZYX(Rx);
	ey.getMatrixZYX(Ry);
	interpolateRotation(Rx,Ry,u,Ru);
	eu.setMatrixZYX(Ru);
	eu.get(out(indices[0]),out(indices[2]),out(indices[3]));
      }
      break;
    case RobotModelJoint::Spin:
      {
	int k=robot.joints[i].linkIndex;
	out[k] = AngleInterp(AngleNormalize(x(k)),AngleNormalize(y(k)),u);
      }
      break;
    case RobotModelJoint::FloatingPlanar:
      {
	vector<int> indices;
	robot.GetJointIndices(i,indices);
	int k=indices[2];
	out[k] = AngleInterp(AngleNormalize(x(k)),AngleNormalize(y(k)),u);
      }
      break;
    default:
      break;
    }
  }
}

void GetEulerAngleZYXInterpDeriv(const EulerAngleRotation& a,const EulerAngleRotation& b, Vector3& deriv)
{
  Matrix3 Ra,Rb;
  a.getMatrixZYX(Ra);
  b.getMatrixZYX(Rb);
  
  Matrix3 R;
  R.mulTransposeA(Ra,Rb);
  MomentRotation m;
  m.setMatrix(R);
  Vector3 dtheta;
  EulerAngleDerivative(a,m,2,1,0,deriv);
}


void IntegrateEulerAngleZYX(const EulerAngleRotation& a,const Vector3& da,EulerAngleRotation& b)
{
  Matrix3 Ra,Rz,Rb;
  a.getMatrixZYX(Ra);
  MomentRotation z;
  AngularVelocityEulerAngle(a,da,2,1,0,z);
  z.getMatrix(Rz);
  Rb = Ra*Rz;
  b.setMatrixZYX(Rb);
}


/** @brief Returns the velocity vector that will move the robot from the
 * current configuration to 'dest' in minimal time. 
 */
void InterpolateDerivative(RobotModel& robot,const Config& a,const Config& b,Vector& dq)
{
  dq = b - a;
  for(size_t i=0;i<robot.joints.size();i++) {
    int k=robot.joints[i].linkIndex;
    if(robot.joints[i].type == RobotModelJoint::Spin) {
      dq(k) = AngleDiff(AngleNormalize(b(k)),AngleNormalize(a(k)));
    }
    else if(robot.joints[i].type == RobotModelJoint::Floating) {
      vector<int> indices;
      robot.GetJointIndices(i,indices);
      EulerAngleRotation oldrot(a(indices[3]),a(indices[4]),a(indices[5]));
      EulerAngleRotation newrot(b(indices[3]),b(indices[4]),b(indices[5]));
      Vector3 dtheta;
      assert(robot.links[indices[3]].w == Vector3(0,0,1));
      assert(robot.links[indices[4]].w == Vector3(0,1,0));
      assert(robot.links[indices[5]].w == Vector3(1,0,0));
      GetEulerAngleZYXInterpDeriv(oldrot,newrot,dtheta);
      dtheta.get(dq(indices[3]),dq(indices[4]),dq(indices[5]));
    }
    else if(robot.joints[i].type == RobotModelJoint::FloatingPlanar) {
      vector<int> indices;
      robot.GetJointIndices(i,indices);
      dq(indices[2]) = AngleDiff(AngleNormalize(b(indices[2])),AngleNormalize(a(indices[2])));
    }
    else if(robot.joints[i].type == RobotModelJoint::BallAndSocket) {
      vector<int> indices;
      robot.GetJointIndices(i,indices);
      EulerAngleRotation oldrot(a(indices[0]),a(indices[1]),a(indices[2]));
      EulerAngleRotation newrot(b(indices[0]),b(indices[1]),b(indices[2]));
      Vector3 dtheta;
      assert(robot.links[indices[0]].w == Vector3(0,0,1));
      assert(robot.links[indices[1]].w == Vector3(0,1,0));
      assert(robot.links[indices[2]].w == Vector3(1,0,0));
      GetEulerAngleZYXInterpDeriv(oldrot,newrot,dtheta);
      dtheta.get(dq(indices[0]),dq(indices[1]),dq(indices[2]));
    }
    else if(robot.joints[i].type == RobotModelJoint::Weld) {
      dq(k) = 0;
    }
  }
}

void InterpolateDerivative(RobotModel& robot,const Config& a,const Config& b,Real u,Vector& dx)
{
  if(u==0) Klampt::InterpolateDerivative(robot,a,b,dx);
  else if(u==1) {
    Klampt::InterpolateDerivative(robot,b,a,dx);
    dx.inplaceNegative();
  }
  else {
    Vector temp;
    Klampt::Interpolate(robot,a,b,u,temp);
    if(u < 0.5) {
      Klampt::InterpolateDerivative(robot,temp,b,dx);
      dx *= 1.0/(1.0-u);
    }
    else {
      Klampt::InterpolateDerivative(robot,temp,a,dx);
      dx *= -1.0/u;
    }
  }
}

void Integrate(RobotModel& robot,const Config& q,const Vector& dq,Config& b)
{
  b = q+dq;
  for(size_t i=0;i<robot.joints.size();i++) {
    int k=robot.joints[i].linkIndex;
    if(robot.joints[i].type == RobotModelJoint::Spin) {
      b(k) = AngleNormalize(b(k));
    }
    else if(robot.joints[i].type == RobotModelJoint::Floating) {
      vector<int> indices;
      robot.GetJointIndices(i,indices);
      EulerAngleRotation rot(q(indices[3]),q(indices[4]),q(indices[5]));
      Vector3 drot(dq(indices[3]),dq(indices[4]),dq(indices[5]));
      assert(robot.links[indices[3]].w == Vector3(0,0,1));
      assert(robot.links[indices[4]].w == Vector3(0,1,0));
      assert(robot.links[indices[5]].w == Vector3(1,0,0));
      EulerAngleRotation rotb;
      IntegrateEulerAngleZYX(rot,drot,rotb);
      rotb.get(b(indices[3]),b(indices[4]),b(indices[5]));
    }
    else if(robot.joints[i].type == RobotModelJoint::FloatingPlanar) {
      vector<int> indices;
      robot.GetJointIndices(i,indices);
      b[indices[2]] = AngleNormalize(b[indices[2]]);
    }
    else if(robot.joints[i].type == RobotModelJoint::BallAndSocket) {
      vector<int> indices;
      robot.GetJointIndices(i,indices);
      EulerAngleRotation rot(q(indices[0]),q(indices[1]),q(indices[2]));
      Vector3 drot(dq(indices[0]),dq(indices[1]),dq(indices[2]));
      assert(robot.links[indices[0]].w == Vector3(0,0,1));
      assert(robot.links[indices[1]].w == Vector3(0,1,0));
      assert(robot.links[indices[2]].w == Vector3(1,0,0));
      EulerAngleRotation rotb;
      IntegrateEulerAngleZYX(rot,drot,rotb);
      rotb.get(b(indices[0]),b(indices[1]),b(indices[2]));
    }
  }
}


Real Distance(const RobotModel& robot,const Config& a,const Config& b,Real normExp,Real floatingRotationWeight)
{
  Assert(a.n == robot.q.n);
  Assert(b.n == robot.q.n);
  NormAccumulator<Real> norm(normExp);
  for(size_t i=0;i<robot.joints.size();i++) {
    int link = robot.joints[i].linkIndex;
    switch(robot.joints[i].type) {
    case RobotModelJoint::Weld:
      break;
    case RobotModelJoint::Normal:
      norm.collect(a(link)-b(link));
      break;
    case RobotModelJoint::Spin:
      norm.collect(AngleDiff(AngleNormalize(a(link)),AngleNormalize(b(link))));
      break;
    case RobotModelJoint::Floating:
      {
	vector<int> indices;
	robot.GetJointIndices(i,indices);
	Assert(indices.size()==6);
	Assert(robot.links[indices[0]].type == RobotLink3D::Prismatic);
	Assert(robot.links[indices[1]].type == RobotLink3D::Prismatic);
	Assert(robot.links[indices[2]].type == RobotLink3D::Prismatic);
	Assert(robot.links[indices[3]].type == RobotLink3D::Revolute);
	Assert(robot.links[indices[4]].type == RobotLink3D::Revolute);
	Assert(robot.links[indices[5]].type == RobotLink3D::Revolute);
	Vector3 ta(a(indices[0]),a(indices[1]),a(indices[2]));
	Vector3 tb(b(indices[0]),b(indices[1]),b(indices[2]));
	EulerAngleRotation ra(a(indices[3]),a(indices[4]),a(indices[5]));
	EulerAngleRotation rb(b(indices[3]),b(indices[4]),b(indices[5]));
	Matrix3 ma,mb,mdiff;
	ra.getMatrixZYX(ma);
	rb.getMatrixZYX(mb);
	mdiff.mulTransposeB(ma,mb);
	AngleAxisRotation aa; aa.setMatrix(mdiff);
	norm.collect(ta.x-tb.x);
	norm.collect(ta.y-tb.y);
	norm.collect(ta.z-tb.z);
	norm.collect(aa.angle*floatingRotationWeight);
      }
      break;
    case RobotModelJoint::BallAndSocket:
      {
	vector<int> indices;
	robot.GetJointIndices(i,indices);
	Assert(indices.size()==3);
	Assert(robot.links[indices[0]].type == RobotLink3D::Revolute);
	Assert(robot.links[indices[1]].type == RobotLink3D::Revolute);
	Assert(robot.links[indices[2]].type == RobotLink3D::Revolute);
	EulerAngleRotation ra(a(indices[0]),a(indices[1]),a(indices[2]));
	EulerAngleRotation rb(b(indices[0]),b(indices[1]),b(indices[2]));
	Matrix3 ma,mb,mdiff;
	ra.getMatrixZYX(ma);
	rb.getMatrixZYX(mb);
	mdiff.mulTransposeB(ma,mb);
	AngleAxisRotation aa; aa.setMatrix(mdiff);
	norm.collect(aa.angle);
      }
      break;
    default:
      FatalError("Can't handle that type of joint yet in Distance");
      break;
    }
  }
  return norm;
}


Real Distance(const RobotModel& robot,const Config& a,const Config& b,Real normExp,const vector<Real>& jointWeights,Real floatingRotationWeight)
{
  Assert(a.n == robot.q.n);
  Assert(b.n == robot.q.n);
  NormAccumulator<Real> norm(normExp);
  for(size_t i=0;i<robot.joints.size();i++) {
    int link = robot.joints[i].linkIndex;
    Real w = jointWeights[i];
    switch(robot.joints[i].type) {
    case RobotModelJoint::Weld:
      break;
    case RobotModelJoint::Normal:
      norm.collect(a(link)-b(link),w);
      break;
    case RobotModelJoint::Spin:
      norm.collect(AngleDiff(AngleNormalize(a(link)),AngleNormalize(b(link))),w);
      break;
    case RobotModelJoint::Floating:
      {
	vector<int> indices;
	robot.GetJointIndices(i,indices);
	Assert(indices.size()==6);
	Assert(robot.links[indices[0]].type == RobotLink3D::Prismatic);
	Assert(robot.links[indices[1]].type == RobotLink3D::Prismatic);
	Assert(robot.links[indices[2]].type == RobotLink3D::Prismatic);
	Assert(robot.links[indices[3]].type == RobotLink3D::Revolute);
	Assert(robot.links[indices[4]].type == RobotLink3D::Revolute);
	Assert(robot.links[indices[5]].type == RobotLink3D::Revolute);
	Vector3 ta(a(indices[0]),a(indices[1]),a(indices[2]));
	Vector3 tb(b(indices[0]),b(indices[1]),b(indices[2]));
	EulerAngleRotation ra(a(indices[3]),a(indices[4]),a(indices[5]));
	EulerAngleRotation rb(b(indices[3]),b(indices[4]),b(indices[5]));
	Matrix3 ma,mb,mdiff;
	ra.getMatrixZYX(ma);
	rb.getMatrixZYX(mb);
	mdiff.mulTransposeB(ma,mb);
	AngleAxisRotation aa; aa.setMatrix(mdiff);
	norm.collect(ta.x-tb.x,w);
	norm.collect(ta.y-tb.y,w);
	norm.collect(ta.z-tb.z,w);
	norm.collect(aa.angle,floatingRotationWeight*w);
      }
      break;
    case RobotModelJoint::BallAndSocket:
      {
	vector<int> indices;
	robot.GetJointIndices(i,indices);
	Assert(indices.size()==3);
	Assert(robot.links[indices[0]].type == RobotLink3D::Revolute);
	Assert(robot.links[indices[1]].type == RobotLink3D::Revolute);
	Assert(robot.links[indices[2]].type == RobotLink3D::Revolute);
	EulerAngleRotation ra(a(indices[0]),a(indices[1]),a(indices[2]));
	EulerAngleRotation rb(b(indices[0]),b(indices[1]),b(indices[2]));
	Matrix3 ma,mb,mdiff;
	ra.getMatrixZYX(ma);
	rb.getMatrixZYX(mb);
	mdiff.mulTransposeB(ma,mb);
	AngleAxisRotation aa; aa.setMatrix(mdiff);
	norm.collect(aa.angle,w);
      }
      break;
    default:
      break;
    }
  }
  return norm;
}

} //namespace Klampt