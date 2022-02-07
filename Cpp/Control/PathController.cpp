#include "PathController.h"
#include "Sensing/JointSensors.h"
#include "Modeling/Conversions.h"
#include <KrisLibrary/spline/Hermite.h>
#include <KrisLibrary/Logger.h>
#include <sstream>
#include <fstream>

namespace Klampt {

const static Real gJointLimitEpsilon = 1e-7;
const static Real gVelocityLimitEpsilon = 1e-7;


template <class T>
bool ReadVectorFile(File& f,std::vector<T>& v)
{
  int n;
  if(!ReadFile(f,n)) return false;
  if(n < 0) return false;
  v.resize(n);
  for(int i=0;i<n;i++)
    if(!ReadFile(f,v[i])) return false;
  return true;
}

template <class T>
bool WriteVectorFile(File& f,const std::vector<T>& v)
{
  int n=int(v.size());
  if(!WriteFile(f,n)) return false;
  for(int i=0;i<n;i++)
    if(!WriteFile(f,v[i])) return false;
  return true;
}



PolynomialMotionQueue::PolynomialMotionQueue()
{
  pathOffset = 0;
}

void PolynomialMotionQueue::SetLimits(const RobotModel& robot)
{
  qMin = robot.qMin;
  qMax = robot.qMax;
  velMax = robot.velMax;
  accMax = robot.accMax;
}

void PolynomialMotionQueue::SetConstant(const Config& q)
{
  path = Spline::Constant(q,0,0);
  pathOffset = 0;
}

void PolynomialMotionQueue::SetPath(const Spline::PiecewisePolynomialND& _path)
{
  path = _path;
  pathOffset = 0;
}

void PolynomialMotionQueue::SetPiecewiseLinear(const vector<Config>& milestones,const vector<Real>& times)
{
  if(!milestones.empty()) {
    vector<double> elems(milestones.size());
    path.elements.resize(milestones[0].n);
    for(size_t i=0;i<path.elements.size();i++) {
      for(size_t j=0;j<milestones.size();j++)
        elems[j] = milestones[j](i);
      path.elements[i] = Spline::PiecewiseLinear(elems,times);
    }
  }
  else path.elements.resize(0);
  pathOffset = 0;
}

void PolynomialMotionQueue::SetPiecewiseCubic(const vector<Config>& milestones,const vector<Vector>& velocities,const vector<Real>& times)
{
  Assert(milestones.size()==velocities.size());
  Assert(milestones.size()==times.size());
  if(!milestones.empty()) {
    path.elements.resize(milestones[0].n);
    for(size_t i=0;i<path.elements.size();i++) {
      path.elements[i] = Spline::PiecewisePolynomial();
      for(size_t j=0;j+1<milestones.size();j++) {
        Real dt = times[j+1]-times[j];
        if(dt == 0) //null motion or discontinuous jump requested?
          continue;
        Spline::Polynomial<double> poly;
        Spline::HermitePolynomial(milestones[j][i],velocities[j][i]*dt,milestones[j+1][i],velocities[j+1][i]*dt,poly);
        //time scale it to length dt
        Spline::Polynomial<double> timescale;
        timescale.SetCoef(0,0);
        timescale.SetCoef(1,1.0/dt);
        poly = poly.Evaluate(timescale);
        path.elements[i].Append(poly,dt,true);
      }
    }
  }
  else
    path.elements.resize(0);
  pathOffset = 0;
}

void PolynomialMotionQueue::SetPiecewiseLinearRamp(const vector<Config>& milestones)
{
  vector<ParabolicRamp::Vector> pmilestones(milestones.size());
  for(size_t i=0;i<milestones.size();i++)
    pmilestones[i] = milestones[i];
  ParabolicRamp::DynamicPath dpath;
  dpath.Init(velMax,accMax);
  if(!qMin.empty()) //optional joint limits
    dpath.SetJointLimits(qMin,qMax);
  if(!dpath.SetMilestones(pmilestones)) {
    FatalError("SetPiecewiseLinearRamp: DynamicPath.SetMilestones failed");
  }
  SetPath(dpath);
}

void PolynomialMotionQueue::SetPath(const ParabolicRamp::DynamicPath& _path)
{
  path = Cast(_path);
  pathOffset = 0;
}

void PolynomialMotionQueue::Append(const Spline::PiecewisePolynomialND& _path)
{
  path.Concat(_path,true);
}

void PolynomialMotionQueue::Append(const ParabolicRamp::DynamicPath& _path)
{
  path.Concat(Cast(_path),true);
}

void PolynomialMotionQueue::AppendLinear(const Config& config,Real dt)
{
  if(path.elements.empty()) FatalError("PolynomialMotionQueue::AppendLinear: motion queue is uninitialized.  Wait until after the control loop or call SetMilestone() first\n");
  if(dt == 0 && config != Endpoint()) {
    //want a continuous jump?
    if(config.distance(Endpoint()) > Epsilon) {
      LOG4CXX_WARN(KrisLibrary::logger(),"PolynomialMotionQueue::AppendLinear: Warning, discontinuous jump requested");
      LOG4CXX_WARN(KrisLibrary::logger(),"  Time "<<path.EndTime()<<" distance "<<config.distance(Endpoint()));
    }
    path.Concat(Spline::Linear(config,config,0,0),true);    
  }
  else 
    path.Concat(Spline::Linear(Endpoint(),config,0,dt),true);
}

void PolynomialMotionQueue::AppendCubic(const Config& x,const Vector& v,Real dt)
{
  if(path.elements.empty()) FatalError("PolynomialMotionQueue::AppendCubic: motion queue is uninitialized.  Wait until after the control loop or call SetMilestone() first\n");
  if(dt == 0) {
    if(x != Endpoint()) {
      //want a continuous jump?
      if(x.distance(Endpoint()) > Epsilon) {
        LOG4CXX_WARN(KrisLibrary::logger(),"PolynomialMotionQueue::AppendCubic: Warning, discontinuous jump requested");
        LOG4CXX_WARN(KrisLibrary::logger(),"  Time "<<path.EndTime()<<" distance "<<x.distance(Endpoint()));
      }
      path.Concat(Spline::Linear(x,x,0,0),true);    
    }
  }
  else {
    Config x0 = Endpoint();
    Vector v0 = EndpointVelocity();
    for(int i=0;i<x.n;i++) {
      Spline::Polynomial<double> poly;
      Spline::HermitePolynomial(x0[i],v0[i]*dt,x[i],v[i]*dt,poly);
      //time scale it to length dt
      Spline::Polynomial<double> timescale;
      timescale.SetCoef(0,0);
      timescale.SetCoef(1,1.0/dt);
      poly = poly.Evaluate(timescale);
      //TODO test
      //Real xtest = poly.Evaluate(dt);
      //Real vtest = poly.Derivative(dt); 
      path.elements[i].Append(poly,dt,true);
    }
  }
}

void PolynomialMotionQueue::AppendRamp(const Config& x)
{
  Vector zero(x.n,Zero);
  AppendRamp(x,zero);
}

void PolynomialMotionQueue::AppendRamp(const Config& x,const Vector& v)
{
  if(path.elements.empty()) FatalError("PolynomialMotionQueue::AppendRamp: motion queue is uninitialized.  Wait until after the control loop or call SetMilestone() first\n");
  if(accMax.empty()) 
    FatalError("Cannot append ramp without acceleration limits");
  if(accMax.size() != (int)path.elements.size()) 
    FatalError("Invalid acceleration limit size");
  if(velMax.empty()) velMax.resize(accMax.size(),Inf);
  if(velMax.size() != (int)path.elements.size()) 
    FatalError("Invalid velocity limit size");
  vector<ParabolicRamp::Vector> milestones(2);
  vector<ParabolicRamp::Vector> dmilestones(2);
  milestones[0] = Endpoint();
  milestones[1] = x;
  dmilestones[0] = EndpointVelocity();
  dmilestones[1] = v;
  for(int i=0;i<x.n;i++)
    if(milestones[0][i] != Clamp(milestones[0][i],qMin[i],qMax[i])) {
      LOG4CXX_WARN(KrisLibrary::logger(),"AppendRamp: Current config["<<i<<"] is out of joint limits: "<<qMin[i]<<" <= "<<milestones[0][i]<<" <= "<<qMax[i]<<", clamping");
      milestones[0][i] = Clamp(milestones[0][i],qMin[i],qMax[i]);
    }
  for(int i=0;i<v.n;i++)
    if(dmilestones[0][i] != Clamp(dmilestones[0][i],-velMax[i],velMax[i])) {
      LOG4CXX_WARN(KrisLibrary::logger(),"AppendRamp: Current vel["<<i<<"] is out of velMax limits: |"<<dmilestones[0][i]<<"| <= "<<velMax[i]<<", clamping");
      dmilestones[0][i] = Clamp(dmilestones[0][i],-velMax[i],velMax[i]);
    }

  if(!qMin.empty()) {
    for(int i=0;i<x.n;i++) {
      if(x[i] != Clamp(x[i],qMin[i],qMax[i])) {
        LOG4CXX_WARN(KrisLibrary::logger(),"AppendRamp: clamping desired config "<<i<<" to joint limits: "<<qMin[i]<<" <= "<<x[i]<<" <= "<<qMax[i]);
        Real shrink = gJointLimitEpsilon*(qMax[i]-qMin[i]);
        milestones[1][i] = Clamp(x[i],qMin[i]+shrink,qMax[i]-shrink);
      }
    }
  }
  for(int i=0;i<x.n;i++) {
    if(Abs(v[i]) > velMax[i]) {
      LOG4CXX_WARN(KrisLibrary::logger(),"AppendRamp: clamping desired velocity "<<i<<" to limits: |"<<v[i]<<"| <= "<<velMax[i]);
      Real shrink = gVelocityLimitEpsilon*velMax[i]*2.0;
      dmilestones[1][i] = Clamp(v[i],-velMax[i]+shrink,velMax[i]-shrink);
    }
  }

  ParabolicRamp::DynamicPath dpath;
  dpath.Init(velMax,accMax);
  if(!qMin.empty()) //optional joint limits
    dpath.SetJointLimits(qMin,qMax);
  if(!dpath.SetMilestones(milestones,dmilestones)) {
    LOG4CXX_WARN(KrisLibrary::logger(),"AppendRamp: Warning, DynamicPath::SetMilestones failed!");
    for(int i=0;i<x.n;i++)
      if(milestones[0][i] != Clamp(milestones[0][i],qMin[i],qMax[i])) {
        LOG4CXX_WARN(KrisLibrary::logger(),"  Reason: current config["<<i<<"] is out of joint limits: "<<qMin[i]<<" <= "<<milestones[0][i]<<" <= "<<qMax[i]);
      }
    for(int i=0;i<v.n;i++)
      if(Abs(dmilestones[0][i]) > velMax[i]) {
        LOG4CXX_WARN(KrisLibrary::logger(),"  Reason: current vel["<<i<<"] is out of velMax limits: |"<<dmilestones[0][i]<<"| <= "<<velMax[i]<<", clamping");
      }
    for(int i=0;i<v.n;i++)
      if(accMax[i] == 0) {
        LOG4CXX_WARN(KrisLibrary::logger(),"  Reason: accMax["<<i<<"] is zero?");
      }
  }
  else {
    if(path.EndTime() < pathOffset) {
      LOG4CXX_WARN(KrisLibrary::logger(),"AppendRamp: Warning, path end time is in the past, cutting...");
      Cut(0);
    }
    path.Concat(Cast(dpath),true);
  }
}

void PolynomialMotionQueue::AppendLinearRamp(const Config& x)
{
  if(path.elements.empty()) FatalError("PolynomialMotionQueue::AppendLinearRamp: motion queue is uninitialized.  Wait until after the control loop or call SetMilestone() first\n");
  if(accMax.empty()) 
    FatalError("Cannot append ramp without acceleration limits");
  if(accMax.size() != (int)path.elements.size()) 
    FatalError("Invalid acceleration limit size");
  if(velMax.empty()) velMax.resize(accMax.size(),Inf);
  if(velMax.size() != (int)path.elements.size()) 
    FatalError("Invalid velocity limit size");
  vector<ParabolicRamp::Vector> milestones(2);
  milestones[0] = Endpoint();
  milestones[1] = x;

  if(!qMin.empty()) {
    for(int i=0;i<x.n;i++) {
      if(x[i] != Clamp(x[i],qMin[i],qMax[i])) {
        LOG4CXX_WARN(KrisLibrary::logger(),"AppendLinearRamp: clamping desired config "<<i<<" to joint limits: "<<qMin[i]<<" <= "<<x[i]<<" <= "<<qMax[i]);
        Real shrink = gJointLimitEpsilon*(qMax[i]-qMin[i]);
        milestones[1][i] = Clamp(x[i],qMin[i]+shrink,qMax[i]-shrink);
      }
    }
  }

  ParabolicRamp::DynamicPath dpath;
  dpath.Init(velMax,accMax);
  if(!dpath.SetMilestones(milestones)) {
    LOG4CXX_WARN(KrisLibrary::logger(),"AppendLinearRamp: Warning, DynamicPath::SetMilestones failed!");
  }
  else {
    if(path.EndTime() < pathOffset) {
      LOG4CXX_WARN(KrisLibrary::logger(),"AppendLinearRamp: Warning, path end time is in the past, cutting...");
      Cut(0);
    }
    path.Concat(Cast(dpath),true);
  }
}


void PolynomialMotionQueue::GetPath(Spline::PiecewisePolynomialND& _path) const
{
  Spline::PiecewisePolynomialND front;
  path.Split(pathOffset,front,_path);
}

void PolynomialMotionQueue::Cut(Real time,bool relative)
{
  if(relative)  {
    path.TrimBack(pathOffset+time);
  }
  else {
    path.TrimBack(time);
  }
}

void PolynomialMotionQueue::Eval(Real time,Config& x,bool relative) const
{
  if(relative)
    x = path.Evaluate(time+pathOffset);
  else
    x = path.Evaluate(time);
}

void PolynomialMotionQueue::Deriv(Real time,Config& dx,bool relative) const
{
  if(relative)
    dx = path.Derivative(time+pathOffset);
  else
    dx = path.Derivative(time);
}

Real PolynomialMotionQueue::CurTime() const
{
  return pathOffset;
}

Config PolynomialMotionQueue::CurConfig() const
{
  return Vector(path.Evaluate(pathOffset));
}

Config PolynomialMotionQueue::CurVelocity() const
{
  return Vector(path.Derivative(pathOffset));
}

Config PolynomialMotionQueue::Endpoint() const
{
  return Vector(path.End());
}

Vector PolynomialMotionQueue::EndpointVelocity() const
{
  if(path.elements.empty()) return Vector();
  return Vector(path.Derivative(path.EndTime()));
}

bool PolynomialMotionQueue::Done() const
{
  return pathOffset >= path.EndTime();
}

Real PolynomialMotionQueue::TimeRemaining() const
{
  if(path.elements.empty()) return 0;
  return path.EndTime() - pathOffset;
}

void PolynomialMotionQueue::Advance(Real dt)
{
  pathOffset += dt;
  //keep the path relatively short and keep it at the current time
  if((pathOffset - path.StartTime()) > Max(0.1,0.1*(path.EndTime()-path.StartTime()))) 
    path.TrimFront(pathOffset);
}




PolynomialPathController::PolynomialPathController(RobotModel& robot)
  :JointTrackingController(robot)
{
  PolynomialMotionQueue::SetLimits(robot);
}

void PolynomialPathController::GetDesiredState(Config& q_des,Vector& dq_des)
{
  q_des = CurConfig();
  dq_des = CurVelocity();
}

void PolynomialPathController::Update(Real dt)
{
  if(path.elements.empty()) {
    //first time
    Config q;
    if(GetSensedConfig(q)) {
      Assert(q.n == robot.q.n);
      //clamp -- minor errors
      for(int i=0;i<q.n;i++) 
        q[i] = Clamp(q[i],robot.qMin[i],robot.qMax[i]);
      SetConstant(q);
    }
    else {
      return;
    }
  }

  PolynomialMotionQueue::Advance(dt);

  JointTrackingController::Update(dt);
}

void PolynomialPathController::Reset()
{
  PolynomialMotionQueue::SetConstant(Vector(path.Evaluate(pathOffset)));
}

bool PolynomialPathController::ReadState(File& f)
{
  if(!JointTrackingController::ReadState(f)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"PolynomialPathController: Unable to read JointTrackingController state");
    return false;
  }
  if(!ReadFile(f,pathOffset)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"PolynomialPathController:Unable to read pathOffset");
    return false;
  }
  if(!path.Read(f)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"PolynomialPathController:Unable to read path");
    return false;
  }
  return true;
}

bool PolynomialPathController::WriteState(File& f) const
{
  if(!JointTrackingController::WriteState(f)) return false;
  if(!WriteFile(f,pathOffset)) return false;
  if(!path.Write(f)) return false;    
  return true;
}

vector<string> PolynomialPathController::Commands() const
{
  vector<string> res;
  res.push_back("set_tq");
  res.push_back("set_q");
  res.push_back("set_qv");
  res.push_back("set_tv");
  res.push_back("set_tqv");
  res.push_back("append_tq");
  res.push_back("append_q");
  res.push_back("append_q_linear");
  res.push_back("append_qv");
  res.push_back("append_tqv");
  res.push_back("brake");
  return res;
}

bool PolynomialPathController::SendCommand(const string& name,const string& str)
{
  if(name.substr(0,6) == "append") {
    if(path.elements.empty()) {
      LOG4CXX_WARN(KrisLibrary::logger(),name<<": warning, the controller has not been set up yet with the robot's current configuration... try to take some simulation steps first, call set_tq, or SetConstant(q)");
      return false;
    }
  }

  stringstream ss(str);
  Real t;
  Config q,v;
  if(name == "set_tq") {
    ss>>t>>q;
    if(!ss) return false;
    if(t < pathOffset) {
      LOG4CXX_WARN(KrisLibrary::logger(),"set_tq: warning, cut time "<<t<<" is less than path's current time "<<pathOffset);
      return false;
    }
    if(path.elements.empty()) {
      LOG4CXX_WARN(KrisLibrary::logger(),"set_tq: warning, the controller has not been set up yet with the robot's current configuration... starting at the given configuration");
      path = Spline::Constant(q,0,t);
      pathOffset = 0;
      return true;
    }
    Cut(0);
    Assert(t >= path.EndTime());
    AppendLinear(q,t-path.EndTime());
    return true;
  }
  if(name == "set_tv") {
    ss>>t>>v;
    if(!ss) return false;
    if(t < pathOffset) {
      LOG4CXX_WARN(KrisLibrary::logger(),"set_tv: warning, cut time "<<t<<" is less than path's current time "<<pathOffset);
      return false;
    }
    if(path.elements.empty()) {
      LOG4CXX_WARN(KrisLibrary::logger(),"set_tv: warning, the motion queue has not been set up yet.  Call any of the setX commands first or wait until the first control loop has passed");
      return false;
    }
    Cut(0);
    q = Endpoint();
    q.madd(v,t - pathOffset);
    Assert(t >= path.EndTime());
    AppendLinear(q,t-path.EndTime());
    return true;
  }
  else if(name == "append_tq") {
    ss>>t>>q;
    if(!ss) return false;
    if(t < path.EndTime()) {
      LOG4CXX_WARN(KrisLibrary::logger(),"append_tq: warning, append time "<<t<<" is less than path's endtime "<<path.EndTime());
      return false;
    }
    if(path.elements.empty()) {
      LOG4CXX_WARN(KrisLibrary::logger(),"append_tq: warning, the motion queue has not been set up yet.  Call any of the setX commands first or wait until the first control loop has passed");
      return false;
    }
    AppendLinear(q,t-path.EndTime());
    return true;
  }
  else if(name == "set_q") {
    ss>>q;
    if(!ss) return false;
    if(path.elements.empty()) {
      LOG4CXX_WARN(KrisLibrary::logger(),"set_q: warning, the controller has not been set up yet with the robot's current configuration... starting at the given configuration"); 
      SetConstant(q);
      return true;
    }
    Cut(0);
    AppendRamp(q);
    return true;
  }
  else if(name == "append_q") {
    ss>>q;
    if(!ss) return false;
    if(path.elements.empty()) {
      LOG4CXX_WARN(KrisLibrary::logger(),"append_q: warning, the motion queue has not been set up yet.  Call any of the setX commands first or wait until the first control loop has passed.");
      return false;
    }
    AppendRamp(q);
    return true;
  }
  else if(name == "append_q_linear") {
    ss>>q;
    if(!ss) return false;
    if(path.elements.empty()) {
      LOG4CXX_WARN(KrisLibrary::logger(),"append_q_linear: warning, the motion queue has not been set up yet.  Call any of the setX commands first or wait until the first control loop has passed."); 
      return false;
    }
    AppendLinearRamp(q);
    return true;
  }
  else if(name == "set_qv") {
    ss>>q>>v;
    if(!ss) return false;
    if(path.elements.empty()) {
      LOG4CXX_WARN(KrisLibrary::logger(),"set_qv: warning, the controller has not been set up yet with the robot's current configuration... starting at the given configuration");    
      SetConstant(q);
      return true;
    }
    Cut(0);
    AppendRamp(q,v);
    return true;
  }
  else if(name == "append_qv") {
    ss>>q>>v;
    if(!ss) return false;
    if(path.elements.empty()) {
      LOG4CXX_WARN(KrisLibrary::logger(),"append_qv: warning, the motion queue has not been set up yet.  Call any of the setX commands first or wait until the first control loop has passed."); 
      return false;
    }
    AppendRamp(q,v);
    return true;
  }
  else if(name == "set_tqv") {
    ss>>t>>q>>v;
    if(!ss) return false;
    if(path.elements.empty()) {
      LOG4CXX_WARN(KrisLibrary::logger(),"set_tqv: warning, the controller has not been set up yet with the robot's current configuration... starting at the given configuration"); 
      path = Spline::Constant(q,0,t);
      pathOffset = 0;
      return true;
    }
    Cut(0);
    Assert(t >= path.EndTime());
    AppendCubic(q,v,t-path.EndTime());
    return true;
  }
  else if(name == "append_tqv") {
    ss>>t>>q>>v;
    if(!ss) return false;
    if(path.elements.empty()) {
      LOG4CXX_WARN(KrisLibrary::logger(),"append_tqv: warning, the motion queue has not been set up yet.  Call any of the setX commands first or wait until the first control loop has passed."); 
      return false;
    }
    if(t < path.EndTime()) {
      LOG4CXX_WARN(KrisLibrary::logger(),"append_tqv: requested time "<<t<<" is not after end of existing path "<<path.EndTime());    
      return false;
    }
    AppendCubic(q,v,t-path.EndTime());
    return true;
  }
  else if(name == "brake") {
    //Brake();
    LOG4CXX_ERROR(KrisLibrary::logger(),"PolynomialPathController: Brake is not done yet");
    return false;
    return true;
  }
  else {
    LOG4CXX_ERROR(KrisLibrary::logger(),"PolynomialPathController: Invalid command "<<name);
  }
  return false;
}

} //namespace Klampt