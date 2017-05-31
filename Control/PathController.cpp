#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "PathController.h"
#include "JointSensors.h"
#include "Modeling/Conversions.h"
#include <KrisLibrary/spline/Hermite.h>
#include <sstream>
#include <fstream>

const static Real gJointLimitEpsilon = 1e-7;
const static Real gVelocityLimitEpsilon = 1e-7;

MilestonePathController::MilestonePathController(Robot& robot)
  :JointTrackingController(robot),pathParameter(0),
   velScale(1.0),accScale(1.0),modifySpeedByError(false),modifySpeedCoeff(50.0)
{
  path.Init(robot.velMax,robot.accMax);
  path.SetJointLimits(robot.qMin,robot.qMax);
}

map<string,string> MilestonePathController::Settings() const
{
  map<string,string> res;
  FILL_CONTROLLER_SETTING(res,velScale)
  FILL_CONTROLLER_SETTING(res,accScale)
    return res;
}

bool MilestonePathController::GetSetting(const string& name,string& str) const
{
  READ_CONTROLLER_SETTING(velScale)
  READ_CONTROLLER_SETTING(accScale)
  return false;
}

bool MilestonePathController::SetSetting(const string& name,const string& str)
{
  WRITE_CONTROLLER_SETTING(velScale)
  WRITE_CONTROLLER_SETTING(accScale)
  return false;
}

vector<string> MilestonePathController::Commands() const
{
  vector<string> res;
  res.push_back("set_tq");
  res.push_back("set_tv");
  res.push_back("set_q");
  res.push_back("set_qv");
  res.push_back("append_tq");
  res.push_back("append_tv");
  res.push_back("append_q");
  res.push_back("append_qv");
  res.push_back("brake");
  return res;
}

bool MilestonePathController::SendCommand(const string& name,const string& str)
{
  stringstream ss(str);
  Real t;
  Config q,v;
  if(name == "set_tq") {
    ss>>t>>q;
    if(!ss) return false;
    LOG4CXX_INFO(KrisLibrary::logger(),"set_tq: does this work?\n");
    SetMilestone(q,t);
    return true;
  }
  else if(name == "append_tq") {
    ss>>t>>q;
    if(!ss) return false;
        LOG4CXX_ERROR(KrisLibrary::logger(),"append_tq not done yet\n");
    return false;
    if(t < path.GetTotalTime()) {
      LOG4CXX_INFO(KrisLibrary::logger(),"MilestonePathController: Append time is less than path time\n");
      return false;
    }
    Vector xback = Endpoint();
    path.ramps.resize(path.ramps.size()+1);
    path.ramps.back().SetLinear(xback,q,t-path.GetTotalTime());
    return true;
  }
  else if(name == "set_q") {
    ss>>q;
    if(!ss) return false;
    SetMilestone(q);
    return true;
  }
  else if(name == "append_q") {
    ss>>q;
    if(!ss) return false;
    AddMilestone(q);
    return true;
  }
  else if(name == "set_qv") {
    ss>>q>>v;
    if(!ss) return false;
    SetMilestone(q,v);
    return true;
  }
  else if(name == "append_qv") {
    ss>>q>>v;
    if(!ss) return false;
    AddMilestone(q,v);
    return true;
  }
  else if(name == "set_tv") {
    ss>>t>>v;
    if(!ss) return false;
    SetLinearVelocity(v,t);
    return true;
  }
  else if(name == "append_tv") {
    ss>>t>>v;
    if(!ss) return false;
    if(t < path.GetTotalTime()) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"MilestonePathController: Append time is less than path time\n");
      return false;
    }
    Vector xback = Endpoint();
    path.ramps.resize(path.ramps.size()+1);
    path.ramps.back().SetLinear(xback,xback+t*v,t);
    return true;
  }
  else if(name == "brake") {
    //Brake();
        LOG4CXX_ERROR(KrisLibrary::logger(),"Brake is not done yet\n");
    return false;
    return true;
  }
  return false;
}

void MilestonePathController::SetSpeedLimits(Real _velScale,Real _accScale)
{
  velScale = _velScale;
  accScale = _accScale;
  path.velMax = robot.velMax*velScale;
  path.accMax = robot.accMax*accScale;
}

void MilestonePathController::SetMilestone(const Vector& x,const Vector& dx)
{
  vector<ParabolicRamp::Vector> milestones(2),dmilestones(2);
  milestones[0]=xcur;
  milestones[1]=x;
  dmilestones[0]=dxcur;
  dmilestones[1]=dx;

  for(int i=0;i<x.n;i++)
    if(x[i] != Clamp(x[i],robot.qMin[i],robot.qMax[i])) {
      LOG4CXX_WARN(KrisLibrary::logger(),"SetMilestone: Warning, clamping desired config "<<i<<" to joint limits "<<x[i]<<" in ["<<robot.qMin[i]<<","<<robot.qMax[i]);
      Real shrink = gJointLimitEpsilon*(robot.qMax[i]-robot.qMin[i]);
      milestones[1][i] = Clamp(x[i],robot.qMin[i]+shrink,robot.qMax[i]-shrink);
    }

  path.SetMilestones(milestones,dmilestones);
  pathParameter = 0;
}

void MilestonePathController::SetMilestone(const Vector& x)
{
  vector<ParabolicRamp::Vector> milestones(2),dmilestones(2);
  milestones[0]=xcur;
  milestones[1]=x;
  dmilestones[0]=dxcur;
  dmilestones[1].resize(x.size(),0.0);

  for(int i=0;i<x.n;i++)
    if(x[i] != Clamp(x[i],robot.qMin[i],robot.qMax[i])) {
      LOG4CXX_WARN(KrisLibrary::logger(),"SetMilestone: Warning, clamping desired config "<<i<<" to joint limits "<<x[i]<<" in ["<<robot.qMin[i]<<","<<robot.qMax[i]);
      Real shrink = gJointLimitEpsilon*(robot.qMax[i]-robot.qMin[i]);
      milestones[1][i] = Clamp(x[i],robot.qMin[i]+shrink,robot.qMax[i]-shrink);

    }

  path.SetMilestones(milestones,dmilestones);
  pathParameter = 0;
}

void MilestonePathController::SetMilestone(const Vector& x,Real minTime)
{
  ParabolicRamp::ParabolicRampND ramp;
  ramp.x0=xcur;
  ramp.dx0=dxcur;
  ramp.x1=x;
  ramp.dx1.resize(x.n);
  fill(ramp.dx1.begin(),ramp.dx1.end(),0.0);
  bool res=ramp.SolveMinAccel(path.velMax,minTime);
  if(res) {
    path.ramps.resize(1);
    path.ramps[0] = ramp;
  }
  else {
    vector<ParabolicRamp::Vector> milestones(2),dmilestones(2);
    milestones[0]=xcur;
    milestones[1]=x;
    dmilestones[0]=dxcur;
    dmilestones[1].resize(x.size(),0.0);
    for(int i=0;i<x.n;i++)
      if(x[i] != Clamp(x[i],robot.qMin[i],robot.qMax[i])) {
	LOG4CXX_WARN(KrisLibrary::logger(),"SetMilestone: Warning, clamping desired config "<<i<<" to joint limits "<<x[i]<<" in ["<<robot.qMin[i]<<","<<robot.qMax[i]);
	Real shrink = gJointLimitEpsilon*(robot.qMax[i]-robot.qMin[i]);
	milestones[1][i] = Clamp(x[i],robot.qMin[i]+shrink,robot.qMax[i]-shrink);
      }
    path.SetMilestones(milestones,dmilestones);
  }
}

void MilestonePathController::SetLinearVelocity(const Vector& dx,Real t)
{
  pathParameter = 0;
  path.ramps.resize(1);
  path.ramps[0].SetLinear(xcur,xcur+dx*t,t);
}


void MilestonePathController::SetPath(const vector<Vector>& _path)
{
  Assert(!_path.empty());
  SetMilestone(_path[0]);
  for(size_t i=1;i<_path.size();i++)
    AddMilestone(_path[i]);
}


void MilestonePathController::SetPath(const vector<Vector>& _path,const vector<Vector>& _pathDeriv)
{
  Assert(!_path.empty());
  Assert(_path.size() == _pathDeriv.size());
  SetMilestone(_path[0],_pathDeriv[0]);
  for(size_t i=1;i<_path.size();i++)
    AddMilestone(_path[i],_pathDeriv[i]);
}

void MilestonePathController::SetPath(const ParabolicRamp::DynamicPath& _path)
{
  Assert(!_path.Empty());
  SetMilestone(_path.ramps[0].x0,_path.ramps[0].dx0);
  path.Concat(_path);
}

Real MilestonePathController::GetSpeedScale(const Config& q_des,const Vector& dq_des) const
{
  if(modifySpeedByError) {
    //adjust the path speed by the tracking error
    Real trackingError=0;
    for(size_t i=0;i<robot.joints.size();i++) {
      if(robot.joints[i].type == RobotJoint::Normal) {
	int k=robot.joints[i].linkIndex;
	trackingError += Sqr(q_des(k) - sensors->GetTypedSensor<JointPositionSensor>()->q(k));
      }
    }
    return One/(One+modifySpeedCoeff*trackingError);
  }
  return 1.0;
}
    
void MilestonePathController::GetDesiredState(Config& q_des,Vector& dq_des)
{
  if(pathParameter >= path.GetTotalTime()) {
    q_des = xcur;
    dq_des = dxcur;
  }
  else {
    ParabolicRamp::Vector vqd,vdqd;
    path.Evaluate(pathParameter,vqd);
    path.Derivative(pathParameter,vdqd);
    q_des = vqd;
    dq_des = vdqd;
    Real speedModifier = GetSpeedScale(xcur,dxcur);
    dq_des *= speedModifier;  
  }
}



void MilestonePathController::GetPath(ParabolicRamp::DynamicPath& out) const
{
  ParabolicRamp::DynamicPath cruft;
  path.Split(pathParameter,cruft,out);
  
}



Config MilestonePathController::Endpoint() const
{
  if(path.Empty()) return xcur;
  return path.ramps.back().x1;
}

Config MilestonePathController::EndpointVelocity() const
{
  if(path.Empty()) return dxcur;
  return path.ramps.back().dx1;
}


void MilestonePathController::AddMilestone(const Vector& x)
{
  if(!path.Empty()) path.Append(x);
  else {
    vector<ParabolicRamp::Vector> milestones(2);
    vector<ParabolicRamp::Vector> dmilestones(2);
    milestones[0] = xcur;
    milestones[1] = x;
    dmilestones[0] = dxcur;
    dmilestones[1].resize(x.n,0);
    path.SetMilestones(milestones,dmilestones);
  }
}

void MilestonePathController::AddMilestone(const Vector& x,const Vector& dx)
{
  if(!path.Empty())
    path.Append(x,dx);
  else {
    vector<ParabolicRamp::Vector> milestones(2);
    vector<ParabolicRamp::Vector> dmilestones(2);
    milestones[0] = xcur;
    milestones[1] = x;
    dmilestones[0] = dxcur;
    dmilestones[1] = dx;
    path.SetMilestones(milestones,dmilestones);
  }
}

Real MilestonePathController::AddMilestone(const Vector& x,const Vector& dx,Real dt)
{
  size_t n=path.ramps.size();
  size_t p=n-1;
  path.ramps.resize(path.ramps.size()+1);
  if(path.ramps.size()==1) {
    Assert(dx.isZero());
    path.ramps[0].x0 = xcur;
    path.ramps[0].dx0 = dxcur;
    path.ramps[0].x1 = x;
    path.ramps[0].dx1 = dx;
  }
  else {
    path.ramps[n].x0 = path.ramps[p].x1;
    path.ramps[n].dx0 = path.ramps[p].dx1;
    path.ramps[n].x1 = x;
    path.ramps[n].dx1 = dx;
  }
  bool res=path.ramps[n].SolveMinAccel(path.velMax,dt);
  if(!res)
    res=path.ramps[n].SolveMinTime(path.accMax,path.velMax);
  Assert(res);
  return path.ramps.back().endTime;
}


void MilestonePathController::Update(Real dt)
{
  if(path.Empty()) {
    if(GetSensedConfig(robot.q)) {
      if(GetSensedVelocity(robot.dq)) {
	//first time, read from sensors
	xcur=robot.q;
	dxcur=robot.dq;
	//clamp -- minor errors
	for(int i=0;i<xcur.n;i++) 
	  xcur[i] = Clamp(xcur[i],robot.qMin[i],robot.qMax[i]);
	SetMilestone(robot.q);
      }
      else {
	xcur=robot.q;
	dxcur.resize(robot.q.n,Zero);
	//clamp -- minor errors
	for(int i=0;i<xcur.n;i++) 
	  xcur[i] = Clamp(xcur[i],robot.qMin[i],robot.qMax[i]);
	SetMilestone(robot.q);
      }
    }
    else {
      //don't have a path yet, don't have sensor data yet either
      return; 
    }
  }

  Real speedModifier = GetSpeedScale(xcur,dxcur);
  pathParameter += dt*speedModifier;
  assert(!path.Empty());
  ParabolicRamp::Vector vq,vdq;
  path.Evaluate(pathParameter,vq);
  path.Derivative(pathParameter,vdq);
  xcur=vq;
  dxcur=vdq;
  while(pathParameter >= path.ramps[0].endTime) {
    pathParameter -= path.ramps[0].endTime;
    if(path.ramps.size()==1) {
      path.ramps[0].SetConstant(xcur,0);
      break;
    }
    else {
      path.ramps.erase(path.ramps.begin());
    }
  }
  JointTrackingController::Update(dt);
}

void MilestonePathController::Reset()
{
  path.ramps.clear();
  pathParameter = 0;
  /*
  //xcur = robot.q;
  dxcur.setZero();
  SetMilestone(xcur);
  */
  xcur.clear();
  dxcur.clear();
  JointTrackingController::Reset();
}


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

bool MilestonePathController::ReadState(File& f)
{
  if(!JointTrackingController::ReadState(f)) return false;
  if(!ReadFile(f,xcur)) return false;
  if(!ReadFile(f,dxcur)) return false;
  if(!ReadFile(f,pathParameter)) return false;
  if(!ReadFile(f,velScale)) return false;
  if(!ReadFile(f,accScale)) return false;
  if(!ReadFile(f,modifySpeedByError)) return false;
  if(!ReadFile(f,modifySpeedCoeff)) return false;
  int np;
  Vector temp;
  vector<ParabolicRamp::Vector> milestones;
  vector<ParabolicRamp::Vector> dmilestones;
  if(!ReadFile(f,np)) return false;
  for(int i=0;i<np;i++) {
    if(!ReadFile(f,temp)) return false;
    milestones.push_back(temp);
  }
  for(int i=0;i<np;i++) {
    if(!ReadFile(f,temp)) return false;
    dmilestones.push_back(temp);
  }
  path.SetMilestones(milestones,dmilestones);
  return true;
}

bool MilestonePathController::WriteState(File& f) const
{
  if(!JointTrackingController::WriteState(f)) return false;
  if(!WriteFile(f,xcur)) return false;
  if(!WriteFile(f,dxcur)) return false;
  if(!WriteFile(f,pathParameter)) return false;
  if(!WriteFile(f,velScale)) return false;
  if(!WriteFile(f,accScale)) return false;
  if(!WriteFile(f,modifySpeedByError)) return false;
  if(!WriteFile(f,modifySpeedCoeff)) return false;

  int np=path.ramps.size();
  vector<Vector> milestones,dmilestones;
  if(np != 0) {
    np++;
    milestones.push_back(path.ramps[0].x0);
    dmilestones.push_back(path.ramps[0].dx0);
    for(size_t i=0;i<path.ramps.size();i++) {
      milestones.push_back(path.ramps[i].x1);
      dmilestones.push_back(path.ramps[i].dx1);
    }
  }
  if(!WriteFile(f,np)) return false;
  for(size_t i=0;i<milestones.size();i++)
    if(!WriteFile(f,milestones[i])) return false;
  for(size_t i=0;i<dmilestones.size();i++)
    if(!WriteFile(f,dmilestones[i])) return false;
  return true;
}



PolynomialMotionQueue::PolynomialMotionQueue()
{
  pathOffset = 0;
}

void PolynomialMotionQueue::SetLimits(const Robot& robot)
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
    LOG4CXX_WARN(KrisLibrary::logger(),"PolynomialMotionQueue::AppendLinear: Warning, discontinuous jump requested\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"Time "<<path.EndTime()<<" distance "<<config.distance(Endpoint())<<"\n");
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
      LOG4CXX_WARN(KrisLibrary::logger(),"PolynomialMotionQueue::AppendCubic: Warning, discontinuous jump requested\n");
      LOG4CXX_INFO(KrisLibrary::logger(),"Time "<<path.EndTime()<<" distance "<<x.distance(Endpoint())<<"\n");
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
  if(accMax.size() != path.elements.size()) 
    FatalError("Invalid acceleration limit size");
  if(velMax.empty()) velMax.resize(accMax.size(),Inf);
  if(velMax.size() != path.elements.size()) 
    FatalError("Invalid velocity limit size");
  vector<ParabolicRamp::Vector> milestones(2);
  vector<ParabolicRamp::Vector> dmilestones(2);
  milestones[0] = Endpoint();
  milestones[1] = x;
  dmilestones[0] = EndpointVelocity();
  dmilestones[1] = v;
  for(int i=0;i<x.n;i++)
    if(milestones[0][i] != Clamp(milestones[0][i],qMin[i],qMax[i])) {
      LOG4CXX_WARN(KrisLibrary::logger(),"  Warning, current config["<< i<<"] is out of joint limits: "<<qMin[i]<<" <= "<<milestones[0][i]<<" <= "<<qMax[i]<<", clamping\n");     
      milestones[0][i] = Clamp(milestones[0][i],qMin[i],qMax[i]);
    }
  for(int i=0;i<v.n;i++)
    if(dmilestones[0][i] != Clamp(dmilestones[0][i],-velMax[i],velMax[i])) {
      LOG4CXX_WARN(KrisLibrary::logger(),"  Warning, current vel["<< i<<"] is out of velMax limits: |"<<dmilestones[0][i]<<"| <= "<<velMax[i]<<", clamping\n");      
      dmilestones[0][i] = Clamp(dmilestones[0][i],-velMax[i],velMax[i]);
    }

  if(!qMin.empty()) {
    for(int i=0;i<x.n;i++) {
      if(x[i] != Clamp(x[i],qMin[i],qMax[i])) {
	LOG4CXX_WARN(KrisLibrary::logger(),"AppendRamp: Warning, clamping desired config "<<i<<" to joint limits "<<x[i]<<" in ["<<qMin[i]<<","<<qMax[i]);
	Real shrink = gJointLimitEpsilon*(qMax[i]-qMin[i]);
	milestones[1][i] = Clamp(x[i],qMin[i]+shrink,qMax[i]-shrink);
      }
    }
  }
  for(int i=0;i<x.n;i++) {
    if(Abs(v[i]) > velMax[i]) {
      LOG4CXX_WARN(KrisLibrary::logger(),"AppendRamp: Warning, clamping desired velocity "<<i<<" to limits |"<<v[i]<<"|<="<<velMax[i]);
      Real shrink = gVelocityLimitEpsilon*velMax[i]*2.0;
      dmilestones[1][i] = Clamp(v[i],-velMax[i]+shrink,velMax[i]-shrink);
    }
  }

  ParabolicRamp::DynamicPath dpath;
  dpath.Init(velMax,accMax);
  if(!qMin.empty()) //optional joint limits
    dpath.SetJointLimits(qMin,qMax);
  if(!dpath.SetMilestones(milestones,dmilestones)) {
    LOG4CXX_WARN(KrisLibrary::logger(),"AppendRamp: Warning, DynamicPath::SetMilestones failed!\n");
    for(int i=0;i<x.n;i++)
      if(milestones[0][i] != Clamp(milestones[0][i],qMin[i],qMax[i])) {
	LOG4CXX_INFO(KrisLibrary::logger(),"  Reason: current config["<<i<<"] is out of joint limits: "<<qMin[i]<<" <= "<<milestones[0][i]<<" <= "<<qMax[i]);
      }
    for(int i=0;i<v.n;i++)
      if(Abs(dmilestones[0][i]) > velMax[i]) {
	LOG4CXX_INFO(KrisLibrary::logger(),"  Reason: current velocity["<<i<<"] is out of vel limits: |"<<dmilestones[0][i]<<"| <= "<<velMax[i]);
      }
  }
  else {
    if(path.EndTime() < pathOffset) {
      LOG4CXX_WARN(KrisLibrary::logger(),"AppendRamp: Warning, path end time is in the past, cutting...\n");
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
  if(accMax.size() != path.elements.size()) 
    FatalError("Invalid acceleration limit size");
  if(velMax.empty()) velMax.resize(accMax.size(),Inf);
  if(velMax.size() != path.elements.size()) 
    FatalError("Invalid velocity limit size");
  vector<ParabolicRamp::Vector> milestones(2);
  milestones[0] = Endpoint();
  milestones[1] = x;

  if(!qMin.empty()) {
    for(int i=0;i<x.n;i++) {
      if(x[i] != Clamp(x[i],qMin[i],qMax[i])) {
	LOG4CXX_WARN(KrisLibrary::logger(),"AppendRamp: Warning, clamping desired config "<<i<<" to joint limits "<<x[i]<<" in ["<<qMin[i]<<","<<qMax[i]);
	Real shrink = gJointLimitEpsilon*(qMax[i]-qMin[i]);
	milestones[1][i] = Clamp(x[i],qMin[i]+shrink,qMax[i]-shrink);
      }
    }
  }

  ParabolicRamp::DynamicPath dpath;
  dpath.Init(velMax,accMax);
  if(!dpath.SetMilestones(milestones)) {
    LOG4CXX_WARN(KrisLibrary::logger(),"AppendRamp: Warning, DynamicPath::SetMilestones failed!\n");
  }
  else {
    if(path.EndTime() < pathOffset) {
      LOG4CXX_WARN(KrisLibrary::logger(),"AppendRamp: Warning, path end time is in the past, cutting...\n");
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
  return path.Evaluate(pathOffset);
}

Config PolynomialMotionQueue::CurVelocity() const
{
  return path.Derivative(pathOffset);
}

Config PolynomialMotionQueue::Endpoint() const
{
  return path.End();
}

Vector PolynomialMotionQueue::EndpointVelocity() const
{
  if(path.elements.empty()) return Vector();
  return path.Derivative(path.EndTime());
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




PolynomialPathController::PolynomialPathController(Robot& robot)
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
  PolynomialMotionQueue::SetConstant(path.Evaluate(pathOffset));
}

bool PolynomialPathController::ReadState(File& f)
{
  if(!ReadFile(f,pathOffset)) return false;
  if(!path.Read(f)) return false;
  return true;
}

bool PolynomialPathController::WriteState(File& f) const
{
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
  res.push_back("set_v");
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
      LOG4CXX_ERROR(KrisLibrary::logger(),""<<name.c_str()<<": warning, the controller has not been set up yet with the robot's current configuration... try to take some simulation steps first, call set_tq, or SetConstant(q)\n");      
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
            LOG4CXX_ERROR(KrisLibrary::logger(),"set_tq: warning, cut time "<<t<<" is less than path's endtime "<<pathOffset);
      return false;
    }
    if(path.elements.empty()) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"set_tq: warning, the controller has not been set up yet with the robot's current configuration... starting at the given configuration\n");    
      path = Spline::Constant(q,0,t);
      pathOffset = 0;
      return true;
    }
    Cut(0);
    Assert(t >= path.EndTime());
    AppendLinear(q,t-path.EndTime());
    return true;
  }
  else if(name == "append_tq") {
    ss>>t>>q;
    if(!ss) return false;
    if(t < path.EndTime()) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"append_tq: warning, append time "<<t<<" is less than path's endtime "<<path.EndTime());
      return false;
    }
    if(path.elements.empty()) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"append_tq: warning, the motion queue has not been set up yet.  Call any of the setX commands first or wait until the first control loop has passed.\n");    
      return false;
    }
    AppendLinear(q,t-path.EndTime());
    return true;
  }
  else if(name == "set_q") {
    ss>>q;
    if(!ss) return false;
    if(path.elements.empty()) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"set_q: warning, the controller has not been set up yet with the robot's current configuration... starting at the given configuration\n");    
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
            LOG4CXX_ERROR(KrisLibrary::logger(),"append_q: warning, the motion queue has not been set up yet.  Call any of the setX commands first or wait until the first control loop has passed.\n");    
      return false;
    }
    AppendRamp(q);
    return true;
  }
  else if(name == "append_q_linear") {
    ss>>q;
    if(!ss) return false;
    if(path.elements.empty()) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"append_q_linear: warning, the motion queue has not been set up yet.  Call any of the setX commands first or wait until the first control loop has passed.\n");    
      return false;
    }
    AppendLinearRamp(q);
    return true;
  }
  else if(name == "set_qv") {
    ss>>q>>v;
    if(!ss) return false;
    if(path.elements.empty()) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"set_qq: warning, the controller has not been set up yet with the robot's current configuration... starting at the given configuration\n");    
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
            LOG4CXX_ERROR(KrisLibrary::logger(),"append_qv: warning, the motion queue has not been set up yet.  Call any of the setX commands first or wait until the first control loop has passed.\n");    
      return false;
    }
    AppendRamp(q,v);
    return true;
  }
  else if(name == "set_tqv") {
    ss>>t>>q>>v;
    if(!ss) return false;
    if(path.elements.empty()) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"set_tqv: warning, the controller has not been set up yet with the robot's current configuration... starting at the given configuration\n");    
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
            LOG4CXX_ERROR(KrisLibrary::logger(),"append_tqv: warning, the motion queue has not been set up yet.  Call any of the setX commands first or wait until the first control loop has passed.\n");    
      return false;
    }
    if(t < path.EndTime()) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"append_tqv: requested time "<<t<<" is not after end of existing path "<<path.EndTime());    
      return false;
    }
    AppendCubic(q,v,t-path.EndTime());
    return true;
  }
  else if(name == "brake") {
    //Brake();
        LOG4CXX_ERROR(KrisLibrary::logger(),"Brake is not done yet\n");
    return false;
    return true;
  }
  return false;
}
