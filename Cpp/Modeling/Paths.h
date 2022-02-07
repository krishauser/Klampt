#ifndef KLAMPT_MODELING_PATH_H
#define KLAMPT_MODELING_PATH_H

#include "Robot.h"

//forward declarations
namespace ParabolicRamp { class DynamicPath; }
namespace Spline { class PiecewisePolynomialND; }

namespace Klampt {
  using namespace std;

//forward declarations
class MultiPath;

/** @addtogroup Modeling */
/*@{*/

/** @brief A piecewise linear path */
class LinearPath
{
 public:
  LinearPath();
  LinearPath(const vector<Real>& times,const vector<Vector>& milestones);
  bool Save(ostream& out);
  bool Load(istream& in);
  Real StartTime() const { return times.front(); }
  Real EndTime() const { return times.back(); }
  Real Duration() const { return EndTime()-StartTime(); }
  ///Appends a new milestone at the given time
  void Append(Real t,const Config& x) { times.push_back(t); milestones.push_back(x); }
  ///clears the path
  void Clear() { times.resize(0); milestones.resize(0); }
  ///Concatenates the given path onto the end of this one.  If relative = true, the path
  ///is added on with times relative to the end of the current path.
  void Concat(const LinearPath& path,bool relative=true);
  ///Evaluates the path at time t
  void Eval(Real t,Vector& xt) const;
  ///Evaluates the path derivative at time t
  void Deriv(Real t,Vector& dxt) const;
  ///Evaluates the path at time t, using proper interpolation for the given robot
  void Eval(RobotModel& robot,Real t,Vector& xt) const;
  ///Evaluates the path derivative at time t, using proper interpolation for the given robot
  void Deriv(RobotModel& robot,Real t,Vector& dxt) const;

  vector<Real> times;
  vector<Vector> milestones;
};

///Exact, direct conversion from LinearPath to MultiPath
void Convert(const LinearPath& in,MultiPath& out);
///Exact, direct conversion from LinearPath to DynamicPath.
///Warning: discontinuous derivatives. Use Smooth to get a smooth path.
void Convert(const LinearPath& in,ParabolicRamp::DynamicPath& out);
///Exact, direct conversion from LinearPath to PiecewisePolynomial.
///Warning: discontinuous derivatives. Use Smooth to get a smooth path.
void Convert(const LinearPath& in,Spline::PiecewisePolynomialND& out);
///Exact, direct conversion from MultiPath to LinearPath.
///Input path must not have velocities.
void Convert(const MultiPath& in,LinearPath& out);
///Exact, direct conversion from MultiPath to DynamicPath
///Warning: may have discontinuous derivatives if velocities not given.
///Use Smooth to get a smooth path.
void Convert(const MultiPath& in,ParabolicRamp::DynamicPath& out);
///Exact, direct conversion from MultiPath to PiecewisePolynomial.
///Warning: may have discontinuous derivatives if velocities not given.
///Use Smooth to get a smooth path.
void Convert(const MultiPath& in,Spline::PiecewisePolynomialND& out);
///Exact, direct conversion from DynamicPath to MultiPath
void Convert(const ParabolicRamp::DynamicPath& in,MultiPath& out);
///Exact, direct conversion from DynamicPath to PiecewisePolynomial
void Convert(const ParabolicRamp::DynamicPath& in,Spline::PiecewisePolynomialND& out);
///Exact, direct conversion from PiecewisePolynomial to DynamicPath. 
///Input path polynomials must have degree at most 2.
//Not implemented yet
//void Convert(const Spline::PiecewisePolynomialND& in,ParabolicRamp::DynamicPath& out);
///Exact, direct conversion from PiecewisePolynomial to DynamicPath. 
///Input path polynomials must have degree at most 3.
void Convert(const Spline::PiecewisePolynomialND& in,MultiPath& out);

///Extract keyframes from the path representation
void Keyframes(const LinearPath& in,vector<Config>& milestones);
void Keyframes(const LinearPath& in,vector<Real>& times,vector<Config>& milestones);
///Extracts keyframes + tangent vectors.  Will construct tangent vectors by smoothing
void Keyframes(const LinearPath& in,vector<Real>& times,vector<Config>& milestones,vector<Config>& dmilestones);
void Keyframes(RobotModel& robot,const LinearPath& in,vector<Real>& times,vector<Config>& milestones,vector<Config>& dmilestones);
void Keyframes(const MultiPath& in,vector<Config>& milestones);
void Keyframes(const MultiPath& in,vector<Real>& times,vector<Config>& milestones);
void Keyframes(const MultiPath& in,vector<Real>& times,vector<Config>& milestones,vector<Config>& dmilestones);
void Keyframes(RobotModel& robot,const MultiPath& in,vector<Real>& times,vector<Config>& milestones,vector<Config>& dmilestones);
void Keyframes(const ParabolicRamp::DynamicPath& in,vector<Config>& milestones);
void Keyframes(const ParabolicRamp::DynamicPath& in,vector<Real>& times,vector<Config>& milestones);
void Keyframes(const ParabolicRamp::DynamicPath& in,vector<Real>& times,vector<Config>& milestones,vector<Config>& dmilestones);
void Keyframes(const Spline::PiecewisePolynomialND& in,vector<Config>& milestones);
void Keyframes(const Spline::PiecewisePolynomialND& in,vector<Real>& times,vector<Config>& milestones);
void Keyframes(const Spline::PiecewisePolynomialND& in,vector<Real>& times,vector<Config>& milestones,vector<Config>& dmilestones);

///Create a representation that matches the input keyframes
void Interpolate(const vector<Real>& times,const vector<Config>& milestones,LinearPath& out);
///Create a representation that matches the input keyframes
void Interpolate(const vector<Real>& times,const vector<Config>& milestones,MultiPath& out);
///Create a representation that matches the input keyframes.  The path will be smooth and stop
///at each milestone with zero velocity.
void Interpolate(const vector<Real>& times,const vector<Config>& milestones,Spline::PiecewisePolynomialND& out);
///Create a representation that matches the input keyframes.  The path will
///be smooth and will stop at each milestone with zero velocity.
void Interpolate(const vector<Real>& times,const vector<Config>& milestones,ParabolicRamp::DynamicPath& out);
///Create a representation that matches the input keyframes
void Interpolate(const vector<Real>& times,const vector<Config>& milestones,const vector<Vector>& dmilestones,MultiPath& out);
///Create a representation that matches the input keyframes
void Interpolate(const vector<Real>& times,const vector<Config>& milestones,const vector<Vector>& dmilestones,Spline::PiecewisePolynomialND& out);
///Create a representation that matches the input keyframes
void Interpolate(const vector<Real>& times,const vector<Config>& milestones,const vector<Vector>& dmilestones,ParabolicRamp::DynamicPath& out);
///Create a representation that matches the keyframes of the input path
template <class T>
inline void Interpolate(const LinearPath& in,T& out) { Interpolate(in.times,in.milestones,out); }
///Create a representation that matches the keyframes of the input path.  The path will be smooth
///and if the input path has no velocities, the output will stop at each milestone with zero velocity
void Interpolate(const MultiPath& in,ParabolicRamp::DynamicPath& out);
///Create a representation that matches the keyframes of the input path.  The path will be smooth
///and if the input path has no velocities, the output will stop at each milestone with zero velocity
void Interpolate(const MultiPath& in,Spline::PiecewisePolynomialND& out);
///Create a representation that matches the keyframes of the input path. 
void Interpolate(const ParabolicRamp::DynamicPath& in,MultiPath& out);

///Create a representation that matches the input path at its given keyframes and imputed
///tangent vectors.
template <class T>
void Smooth(const LinearPath& in,T& out)
{
  vector<Real> times;
  vector<Vector> milestones,dmilestones;
  Keyframes(in,times,milestones,dmilestones);
  Interpolate(times,milestones,dmilestones,out);
}
template <class T>
void Smooth(const MultiPath& in,ParabolicRamp::DynamicPath& out)
{
  vector<Real> times;
  vector<Vector> milestones,dmilestones;
  Keyframes(in,times,milestones,dmilestones);
  Interpolate(times,milestones,dmilestones,out);
}
template <class T>
void Smooth(RobotModel& robot,const LinearPath& in,T& out)
{
  vector<Real> times;
  vector<Vector> milestones,dmilestones;
  Keyframes(robot,in,times,milestones,dmilestones);
  Interpolate(times,milestones,dmilestones,out);
}
template <class T>
void Smooth(RobotModel& robot,const MultiPath& in,ParabolicRamp::DynamicPath& out)
{
  vector<Real> times;
  vector<Vector> milestones,dmilestones;
  Keyframes(robot,in,times,milestones,dmilestones);
  Interpolate(times,milestones,dmilestones,out);
}

///Split up the path into keyframes at a given resolution
void Discretize(const LinearPath& in,Real res,vector<Real>& times,vector<Config>& milestones);
void Discretize(const MultiPath& in,Real res,vector<Real>& times,vector<Config>& milestones);
void Discretize(const ParabolicRamp::DynamicPath& in,Real res,vector<Real>& times,vector<Config>& milestones);
void Discretize(const Spline::PiecewisePolynomialND& in,Real res,vector<Real>& times,vector<Config>& milestones);
///Split up the path into keyframes and velocities at a given resolution
void Discretize(const LinearPath& in,Real res,vector<Real>& times,vector<Config>& milestones,vector<Vector>& dmilestones);
void Discretize(const MultiPath& in,Real res,vector<Real>& times,vector<Config>& milestones,vector<Vector>& dmilestones);
void Discretize(const ParabolicRamp::DynamicPath& in,Real res,vector<Real>& times,vector<Config>& milestones,vector<Vector>& dmilestones);
void Discretize(const Spline::PiecewisePolynomialND& in,Real res,vector<Real>& times,vector<Config>& milestones,vector<Vector>& dmilestones);
///Split up the path into keyframes and velocities at a given resolution using
///the robot interpolation function
void Discretize(RobotModel& robot,const LinearPath& in,Real res,vector<Real>& times,vector<Config>& milestones);
void Discretize(RobotModel& robot,const LinearPath& in,Real res,vector<Real>& times,vector<Config>& milestones,vector<Vector>& dmilestones);


///Create a representation that matches the input path at keyframes derived from the 
///path at a given resolution
template <class T1,class T2>
void Approximate(const T1& in,Real res,T2& out)
{
  vector<Real> times;
  vector<Vector> milestones,dmilestones;
  Discretize(in,res,times,milestones,dmilestones);
  Interpolate(times,milestones,dmilestones,out);
}
///Create a representation that matches the input path at keyframes derived from the 
///path at a given resolution
template <class T1>
void Approximate(const T1& in,Real res,LinearPath& out)
{
  vector<Real> times;
  vector<Vector> milestones;
  Discretize(in,res,times,milestones);
  Interpolate(times,milestones,out);
}

/*@}*/

} //namespace Klampt

#endif 
