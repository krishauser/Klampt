#ifndef PATH_CONTROLLER_H
#define PATH_CONTROLLER_H

#include "JointTrackingController.h"
#include "Modeling/DynamicPath.h"
#include <spline/PiecewisePolynomial.h>
#include <list>


/** @ingroup Control
 * @brief A controller that can be commanded to reach configurations
 * (with or without velocities specified) smoothly using acceleration
 * bounded ramps.  Configuration bounds are also handled.  [NOTE: Will
 * be deprecated soon; use the more versatile PolynomialPathController
 * instead.]
 *
 * SetMilestone(x,minTime) tries to arrive at x within time minTime.
 * It is not guaranteed to complete.
 *
 * A SetLinearVelocity sets a linear trajectory from the current
 * configuration x0 along the curve x(0) = x0+dx*t for t <= tmax.
 * Care must be taken to avoid discontinuous changes in velocity.
 *
 * Multiple milestone commands can be queued up in order to execute a
 * long trajectory.  Uses a DynamicPath structure to store the queue.
 *
 * Accepts commands set_q,append_q,set_tq,append_tq,set_qv,append_qv,brake
 */
class MilestonePathController : public JointTrackingController
{
 public:
  MilestonePathController(Robot& robot);
  void SetSpeedLimits(Real velScale,Real accScale);  //Note: does not affect current path
  //immediate movements from current config
  void SetMilestone(const Vector& x,const Vector& dx);
  void SetMilestone(const Vector& x);
  void SetMilestone(const Vector& x,Real minTime);
  void SetLinearVelocity(const Vector& dx,Real tmax);
  void SetPath(const vector<Vector>& path);
  void SetPath(const vector<Vector>& path,const vector<Vector>& pathDeriv);
  void SetPath(const ParabolicRamp::DynamicPath& path);
  void Brake();
  //queuing movements
  void AddMilestone(const Vector& x);
  void AddMilestone(const Vector& x,const Vector& dx);
  Real AddMilestone(const Vector& x,const Vector& dx,Real dt);
  //status queries
  void GetPath(ParabolicRamp::DynamicPath& path) const;
  Config Endpoint() const;
  Vector EndpointVelocity() const;
  bool Done() const { return path.Empty(); }
  Real TimeRemaining() const { return Max(path.GetTotalTime()-pathParameter,0.0); }

  //subclass overides
  virtual const char* Type() const { return "MilestonePathController"; }
  virtual void Update(Real dt);  
  virtual void Reset();
  virtual bool ReadState(File& f);
  virtual bool WriteState(File& f) const;
  virtual void GetDesiredState(Config& q_des,Vector& dq_des);
  Real GetSpeedScale(const Config& q_des,const Vector& dq_des) const;

  //getters/setters
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  //commands
  virtual vector<string> Commands() const;
  virtual bool SendCommand(const string& name,const string& str);

  /// Current commanded position and orientation
  Vector xcur,dxcur;
  /// A smooth ramp to reach the current destination.  xcur is
  /// the point on the ramp evaluated at pathParameter.  dxcur is the
  /// velocity of the ramp at pathParameter, scaled by the current speed
  /// modification (if any).
  ParabolicRamp::DynamicPath path;
  Real pathParameter;

  // Settings
  /// A scaling of the robot's velocity and acceleration bounds
  /// Note: for loading and saving state, these should not be modified during execution!
  Real velScale,accScale;
  /// If modifySpeedByError is set to true, slows down the advancement of
  ///the path by 1/(1+modifySpeedCoeff*err) where err is the sum of
  ///squared errors between the actual and commanded configuration
  bool modifySpeedByError;
  Real modifySpeedCoeff;
};

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
  void SetLimits(const Robot& robot);
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
 * append_q_linear,brake.
 */
class PolynomialPathController : public JointTrackingController, public PolynomialMotionQueue
{
 public:
  PolynomialPathController(Robot& robot);
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

#endif
