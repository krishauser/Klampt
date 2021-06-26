#include "ZMP.h"
#include <KrisLibrary/robotics/NewtonEuler.h>

namespace Klampt {

///Utility: returns the center of mass first and second derivatives given joint positions and first and second derivatives.
///Note: changes the robot's configuration and velocity to q and dq, respectively.
void GetCOMDerivs(RobotModel& robot,const Config& q,const Vector& dq,const Vector& ddq,Vector3& cm,Vector3& dcm,Vector3& ddcm,NewtonEulerSolver& ne)
{
  robot.UpdateConfig(q);
  robot.dq = dq;
  ne.CalcLinkAccel(ddq);
  cm = robot.GetCOM();
  dcm.setZero();
  ddcm.setZero();
  for(size_t j=0;j<ne.velocities.size();j++) {
    Vector3 cmofs = robot.links[j].T_World.R*robot.links[j].com;
    Vector3 vcmj = ne.velocities[j].v + cross(ne.velocities[j].w,cmofs);
    Vector3 acmj = ne.accelerations[j].v + cross(ne.accelerations[j].w,cmofs) + cross(ne.velocities[j].w,cross(ne.velocities[j].w,cmofs));
    dcm += robot.links[j].mass*vcmj;
    ddcm += robot.links[j].mass*acmj;
  }
  Real mtotal = robot.GetTotalMass();
  dcm /= mtotal;
  ddcm /= mtotal;
}

///Utility: returns the center of mass first and second derivatives given joint positions and first and second derivatives.
///Note: changes the robot's configuration and velocity to q and dq, respectively.
void GetCOMDerivs(RobotModel& robot,const Config& q,const Vector& dq,const Vector& ddq,Vector3& cm,Vector3& dcm,Vector3& ddcm)
{
  NewtonEulerSolver ne(robot);
  GetCOMDerivs(robot,q,dq,ddq,cm,dcm,ddcm,ne);
  return;
}

///Returns the zero-moment point of a robot executing the given trajectory q(t) with first and second derivatives dq and ddq.
///The ground height is assumed to be 0 by default and gravity is assumed to be 9.8m/s^2.
Vector2 GetZMP(RobotModel& robot,const Config& q,const Vector& dq,const Vector& ddq,NewtonEulerSolver& ne,Real groundHeight,Real g)
{
  Vector3 cm,dcm,ddcm;
  GetCOMDerivs(robot,q,dq,ddq,cm,dcm,ddcm,ne);
  Vector2 zmp;
  zmp.x = cm.x - (cm.z - groundHeight)/g * ddcm.x;
  zmp.y = cm.y - (cm.z - groundHeight)/g * ddcm.y;
  return zmp;
}

///Returns the zero-moment point of a robot executing the given trajectory q(t) with first and second derivatives dq and ddq.
///The ground height is assumed to be 0 by default and gravity is assumed to be 9.8m/s^2.
Vector2 GetZMP(RobotModel& robot,const Config& q,const Vector& dq,const Vector& ddq,Real groundHeight,Real g)
{
  NewtonEulerSolver ne(robot);
  return GetZMP(robot,q,dq,ddq,ne,groundHeight,g);
}

} //namespace Klampt