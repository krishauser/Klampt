#ifndef PATH_CONTROLLER_H
#define PATH_CONTROLLER_H

#include "JointTrackingController.h"
#include <Klampt/Modeling/DynamicPath.h>
#include <KrisLibrary/spline/PiecewisePolynomial.h>
#include <list>

namespace Klampt {

/** @ingroup Control
 * @brief A motion queue that runs on a piecewise polynomial path.
 * Can be commanded to reach configurations (with or without velocities
 * specified) smoothly using piecewise linear or cubic curves. 
 *
 * The motion queue stores a trajectory y(t) and an offset t0 designating
 * the parameter along the trajectory at the current time.  The current
 * configuration will be y(t0)
 *
 * y(t) is defined on the range [t0,t0+T] where T is the time
 * remaining to be executed.  Relative time is specified relative to t0,
 * and "absolute" time is specified relative to t=0 (which may not have
 * any relation to true time).  As time advances, call Advance(dt) to
 * shift the 
 *
 * Note: discontinuity checking is not performed.  As a result, the caller
 * must be careful about SetConstant, SetPath, SetPiecewise[X], and Append
 * functions to make sure the endpoint of the queue matches the beginning
 * of the new path.
 *
 * The [X]Ramp functions allow you to add time-optimal acceleration-bounded
 * "ramps".  Configuration bounds are also handled. In order to use these
 * functions you will first need to fill out the accMax and velMax members.
 * If you wish ramps to obey joint limits, fill out qMin and qMax. 
 * Or, you can just call SetLimits(robot) for your robot model to set these
 * limits from your robot..
 */
class PolynomialMotionQueue
{
public:
  PolynomialMotionQueue();
  virtual ~PolynomialMotionQueue() {}
  ///Automatically sets the velocity/acceleration/joint limits for the given
  ///robot model
  void SetLimits(const RobotModel& robot);
  ///Sets a constant trajectory
  void SetConstant(const Config& x);
  ///Sets the trajectory from a PiecewisePolynomialND
  void SetPath(const Spline::PiecewisePolynomialND& path);
  ///Sets the trajectory from a DynamicPath
  void SetPath(const ParabolicRamp::DynamicPath& path);
  ///Sets a piecewise linear path interpolating the given milestones at the
  ///given times
  void SetPiecewiseLinear(const vector<Config>& milestones,const vector<Real>& times);
  ///Sets a piecewise cubic path using Hermite interpolation, interpolating
  ///the given milestones and velocities at the given times
  void SetPiecewiseCubic(const vector<Config>& milestones,const vector<Vector>& velocities,const vector<Real>& times);
  ///Sets a piecewise cubic path that moves along straight-line paths between
  ///the given milestones, but starting and stopping using optimal
  ///accelerations.
  void SetPiecewiseLinearRamp(const vector<Config>& milestones);
  ///Moves forward the internal time marker
  void Advance(Real dt);
  ///Appends a PiecewisePolynomialND.  Performed as a relative append; 
  ///Assumes the spline is defined on the domain [0,x] and the duration
  ///of the current trajectory by x
  void Append(const Spline::PiecewisePolynomialND& path);
  ///Appends a DynamicPath.
  void Append(const ParabolicRamp::DynamicPath& path);
  ///Appends a linear interpolation from the end configuration to config
  ///over duration dt
  void AppendLinear(const Config& config,Real dt);
  ///Appends a cubic interpolation from the end configuration/velocity to
  ///reach config x at velocity v over duration dt
  void AppendCubic(const Config& x,const Config& v,Real dt);
  ///Appends a time-optimal ramp from the end configuration/velocity to x
  void AppendRamp(const Config& x);
  ///Appends a time-optimal ramp from the end configuration to x along a
  ///straight line joint-space path.
  void AppendLinearRamp(const Config& x);
  ///Appends a time-optimal ramp from the end configuration/velocity to x
  ///with velocity v.
  void AppendRamp(const Config& x,const Vector& v);
  ///Retrieves the path, defined on the range [t0,t0+T]
  void GetPath(Spline::PiecewisePolynomialND& path) const;
  ///Cuts off the portion of the path after y(t0+time) if relative=true,
  ///or y(time) if relative=false
  void Cut(Real time,bool relative=true);
  ///Returns the current time t0
  Real CurTime() const;
  ///Returns the configuration at the current time y(t0)
  Config CurConfig() const;
  ///Returns the velocity at the current time y'(t0)
  Config CurVelocity() const;
  ///Returns the configuration at the end time y(t0+T)
  Config Endpoint() const;
  ///Returns the velocity at the end time y'(t0+T)
  Vector EndpointVelocity() const;
  ///Evaluates the trajectory x=y(t0+time) if relative=true, or x=y(time)
  ///if relative=false
  void Eval(Real time,Config& x,bool relative=true) const;
  ///Evaluates the derivative dx=y'(t0+time) if relative=true, or dx=y'(time)
  ///if relative=false
  void Deriv(Real time,Config& dx,bool relative=true) const;
  ///Returns true if there is no more trajectory to be executed
  bool Done() const;
  ///Returns the duration of the trajectory remaining to be executed
  Real TimeRemaining() const;

  Real pathOffset;
  Spline::PiecewisePolynomialND path;

  ///Limits that are used for [X]Ramp functions. velMax and accMax are
  ///mandatory; qMin and qMax are optional.
  Vector qMin,qMax,velMax,accMax;
};

/** @ingroup Control
 * @brief A controller that uses a piecewise polynomial trajectory.
 *
 * Accepts commands set_q,append_q,set_tq,append_tq,set_qv,append_qv,
 * append_q_linear,set_tqv,append_tqv,set_tv,brake.
 */
class PolynomialPathController : public JointTrackingController, public PolynomialMotionQueue
{
 public:
  PolynomialPathController(RobotModel& robot);
  virtual ~PolynomialPathController() {}

  virtual const char* Type() const { return "PolynomialPathController"; }
  virtual void GetDesiredState(Config& q_des,Vector& dq_des);
  virtual void Update(Real dt);  
  virtual void Reset();
  virtual bool ReadState(File& f);
  virtual bool WriteState(File& f) const;

  //commands
  virtual vector<string> Commands() const;
  virtual bool SendCommand(const string& name,const string& str);
};

} //namespace Klampt

#endif
