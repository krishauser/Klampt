#ifndef PATH_CONTROLLER_H
#define PATH_CONTROLLER_H

#include "JointTrackingController.h"
#include "Modeling/DynamicPath.h"
#include <spline/PiecewisePolynomial.h>
#include <list>


/** @ingroup Control
 * @brief A controller that can be commanded to reach configurations
 * (with or without velocities specified) smoothly using acceleration
 * bounded ramps.  Configuration bounds are also handled.
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
 * @brief A controller that uses a piecewise polynomial trajectory.
 *
 * Accepts commands set_q,append_q,set_tq,append_tq,set_qv,append_qv,brake
 */
class PolynomialPathController : public JointTrackingController
{
 public:
  PolynomialPathController(Robot& robot);
  void SetPath(const Spline::PiecewisePolynomialND& path);
  void SetPath(const vector<Config>& milestones,const vector<Real>& times);
  void SetPath(const ParabolicRamp::DynamicPath& path);
  void Append(const Spline::PiecewisePolynomialND& path);
  void Append(const ParabolicRamp::DynamicPath& path);
  void AppendLinear(const Config& config,Real dt);
  void AppendRamp(const Config& x);
  void AppendRamp(const Config& x,const Vector& v);
  void GetPath(Spline::PiecewisePolynomialND& path) const;
  void Cut(Real time,bool relative=true);
  Config Endpoint() const;
  Vector EndpointVelocity() const;
  bool Done() const;
  Real TimeRemaining() const;

  virtual const char* Type() const { return "PolynomialPathController"; }
  virtual void GetDesiredState(Config& q_des,Vector& dq_des);
  virtual void Update(Real dt);  
  virtual void Reset();
  virtual bool ReadState(File& f);
  virtual bool WriteState(File& f) const;

  //commands
  virtual vector<string> Commands() const;
  virtual bool SendCommand(const string& name,const string& str);

  Real pathOffset;
  Spline::PiecewisePolynomialND path;
};

#endif
