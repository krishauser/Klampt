#ifndef KLAMPT_PLANNING_ZMP_H
#define KLAMPT_PLANNING_ZMP_H

#include <Klampt/Modeling/Robot.h>
#include <Klampt/Modeling/Paths.h>
#include <KrisLibrary/planning/GeneralizedBezierCurve.h>
class NewtonEulerSolver;

namespace Klampt {

///Utility: returns the center of mass first and second derivatives given joint positions and first and second derivatives.
///Note: changes the robot's configuration and velocity to q and dq, respectively.
void GetCOMDerivs(RobotModel& robot,const Config& q,const Vector& dq,const Vector& ddq,Vector3& cm,Vector3& dcm,Vector3& ddcm,NewtonEulerSolver& ne);

///Utility: returns the center of mass first and second derivatives given joint positions and first and second derivatives.
///Note: changes the robot's configuration and velocity to q and dq, respectively.
void GetCOMDerivs(RobotModel& robot,const Config& q,const Vector& dq,const Vector& ddq,Vector3& cm,Vector3& dcm,Vector3& ddcm);

///Returns the zero-moment point of a robot executing the given trajectory q(t) with first and second derivatives dq and ddq.
///The ground height is assumed to be 0 by default and gravity is assumed to be 9.8m/s^2.
Vector2 GetZMP(RobotModel& robot,const Config& q,const Vector& dq,const Vector& ddq,NewtonEulerSolver& ne,Real groundHeight=0,Real g=9.8);

///Returns the zero-moment point of a robot executing the given trajectory q(t) with first and second derivatives dq and ddq.
///The ground height is assumed to be 0 by default and gravity is assumed to be 9.8m/s^2.
Vector2 GetZMP(RobotModel& robot,const Config& q,const Vector& dq,const Vector& ddq,Real groundHeight=0,Real g=9.8);

///Returns a trajectory of the ZMP along the given path in time increments dt
std::vector<Vector2> GetZMPTrajectory(RobotModel& robot,const GeneralizedCubicBezierCurve& path,Real dt,Real groundHeight=0,Real g=9.8);

///Returns a trajectory of the ZMP along the given path in time increments dt
std::vector<Vector2> GetZMPTrajectory(RobotModel& robot,const Spline::PiecewisePolynomialND& path,Real dt,Real groundHeight=0,Real g=9.8);

} //namespace Klampt

#endif