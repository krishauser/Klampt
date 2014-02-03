#include "PathController.h"
#include "Modeling/Conversions.h"
#include <sstream>

const static Real gJointLimitEpsilon = 1e-7;
const static Real gVelocityLimitEpsilon = 1e-7;

MilestonePathController::MilestonePathController(Robot& robot)
  :JointTrackingController(robot),pathParameter(0),
   velScale(1.0),accScale(1.0),modifySpeedByError(false),modifySpeedCoeff(50.0)
{
  path.Init(robot.velMax,robot.accMax);
  path.SetJointLimits(robot.qMin,robot.qMax);
  xcur=robot.q;
  dxcur=robot.dq;
  SetMilestone(robot.q);
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
    printf("set_tq: does this work?\n");
    SetMilestone(q,t);
    return true;
  }
  else if(name == "append_tq") {
    ss>>t>>q;
    if(!ss) return false;
    fprintf(stderr,"append_tq not done yet\n");
    return false;
    if(t < path.GetTotalTime()) {
      printf("MilestonePathController: Append time is less than path time\n");
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
      fprintf(stderr,"MilestonePathController: Append time is less than path time\n");
      return false;
    }
    Vector xback = Endpoint();
    path.ramps.resize(path.ramps.size()+1);
    path.ramps.back().SetLinear(xback,xback+t*v,t);
    return true;
  }
  else if(name == "brake") {
    //Brake();
    fprintf(stderr,"Brake is not done yet\n");
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
      printf("SetMilestone: Warning, clamping desired config %d to joint limits %g in [%g,%g]\n",i,x[i],robot.qMin[i],robot.qMax[i]);
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
      printf("SetMilestone: Warning, clamping desired config %d to joint limits %g in [%g,%g]\n",i,x[i],robot.qMin[i],robot.qMax[i]);
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
	printf("SetMilestone: Warning, clamping desired config %d to joint limits %g in [%g,%g]\n",i,x[i],robot.qMin[i],robot.qMax[i]);
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
  //xcur = robot.q;
  dxcur.setZero();
  SetMilestone(xcur);
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





PolynomialPathController::PolynomialPathController(Robot& robot)
  :JointTrackingController(robot)
{
  pathOffset = 0;
  path.elements.resize(robot.q.n);
  for(int i=0;i<robot.q.n;i++)
    path.elements[i] = Spline::Constant(robot.q(i),0,0);
}

void PolynomialPathController::SetPath(const Spline::PiecewisePolynomialND& _path)
{
  path = _path;
  pathOffset = 0;
}

void PolynomialPathController::SetPath(const vector<Config>& milestones,const vector<Real>& times)
{
  vector<double> elems(milestones.size());
  for(size_t i=0;i<path.elements.size();i++) {
    for(size_t j=0;j<milestones.size();j++)
      elems[j] = milestones[j](i);
    path.elements[i] = Spline::PiecewiseLinear(elems,times);
  }
  pathOffset = 0;
}

void PolynomialPathController::SetPath(const ParabolicRamp::DynamicPath& _path)
{
  path = Cast(_path);
  pathOffset = 0;
}

void PolynomialPathController::Append(const Spline::PiecewisePolynomialND& _path)
{
  path.Concat(_path,true);
}

void PolynomialPathController::Append(const ParabolicRamp::DynamicPath& _path)
{
  path.Concat(Cast(_path),true);
}

void PolynomialPathController::AppendLinear(const Config& config,Real dt)
{
  if(dt == 0 && config != Endpoint()) {
    //want a continuous jump?
    printf("PolynomialPathController::AppendLinear: Warning, discontinuous jump requested\n");
    path.Concat(Spline::Linear(config,config,0,0),true);    
  }
  else 
    path.Concat(Spline::Linear(Endpoint(),config,0,dt),true);
}

void PolynomialPathController::AppendRamp(const Config& x)
{
  Vector zero(x.n,Zero);
  AppendRamp(x,zero);
}

void PolynomialPathController::AppendRamp(const Config& x,const Vector& v)
{
  vector<ParabolicRamp::Vector> milestones(2);
  vector<ParabolicRamp::Vector> dmilestones(2);
  milestones[0] = Endpoint();
  milestones[1] = x;
  dmilestones[0] = EndpointVelocity();
  dmilestones[1] = v;

  for(int i=0;i<x.n;i++) {
    if(x[i] != Clamp(x[i],robot.qMin[i],robot.qMax[i])) {
      printf("AppendRamp: Warning, clamping desired config %d to joint limits %g in [%g,%g]\n",i,x[i],robot.qMin[i],robot.qMax[i]);
      Real shrink = gJointLimitEpsilon*(robot.qMax[i]-robot.qMin[i]);
      milestones[1][i] = Clamp(x[i],robot.qMin[i]+shrink,robot.qMax[i]-shrink);
    }
    if(Abs(v[i]) > robot.velMax[i]) {
      printf("AppendRamp: Warning, clamping desired velocity %d to limits |%g|<=%g\n",i,v[i],robot.velMax[i]);
      Real shrink = gVelocityLimitEpsilon*robot.velMax[i]*2.0;
      dmilestones[1][i] = Clamp(v[i],-robot.velMax[i]+shrink,robot.velMax[i]-shrink);
    }
  }

  ParabolicRamp::DynamicPath dpath;
  dpath.Init(robot.velMax,robot.accMax);
  dpath.SetJointLimits(robot.qMin,robot.qMax);
  if(!dpath.SetMilestones(milestones,dmilestones)) {
    printf("AppendRamp: Warning, SetMilestones failed!\n");
    for(int i=0;i<x.n;i++)
      if(milestones[0][i] != Clamp(milestones[0][i],robot.qMin[i],robot.qMax[i])) {
	printf("  Reason: current config[%d] is out of joint limits: %g <= %g <= %g\n",i,robot.qMin[i],milestones[0][i],robot.qMax[i]);
      }
    for(int i=0;i<v.n;i++)
      if(Abs(dmilestones[0][i]) > robot.velMax[i]) {
	printf("  Reason: current velocity[%d] is out of vel limits: |%g| <= %g\n",i,dmilestones[0][i],robot.velMax[i]);
      }
  }
  else
    path.Concat(Cast(dpath),true);
}

void PolynomialPathController::GetPath(Spline::PiecewisePolynomialND& _path) const
{
  Spline::PiecewisePolynomialND front;
  path.Split(pathOffset,front,_path);
}

void PolynomialPathController::Cut(Real time,bool relative)
{
  if(relative)  {
    path.TrimBack(pathOffset+time);
  }
  else {
    path.TrimBack(time);
  }
}

Config PolynomialPathController::Endpoint() const
{
  return path.End();
}

Vector PolynomialPathController::EndpointVelocity() const
{
  return path.Derivative(path.EndTime());
}

bool PolynomialPathController::Done() const
{
  return pathOffset >= path.EndTime();
}

Real PolynomialPathController::TimeRemaining() const
{
  return path.EndTime() - pathOffset;
}

void PolynomialPathController::GetDesiredState(Config& q_des,Vector& dq_des)
{
  q_des = path.Evaluate(pathOffset);
  dq_des = path.Derivative(pathOffset);
}

void PolynomialPathController::Update(Real dt)
{
  pathOffset += dt;
  //keep the path relatively short
  if((pathOffset - path.StartTime()) > Max(0.1,0.1*(path.EndTime()-path.StartTime())))
    path.TrimFront(pathOffset);

  JointTrackingController::Update(dt);
}

void PolynomialPathController::Reset()
{
  path = Spline::Constant(path.Evaluate(pathOffset),0,0);
  pathOffset = 0;
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
  res.push_back("append_tq");
  res.push_back("append_q");
  res.push_back("append_qv");
  res.push_back("brake");
  return res;
}

bool PolynomialPathController::SendCommand(const string& name,const string& str)
{
  stringstream ss(str);
  Real t;
  Config q,v;
  if(name == "set_tq") {
    ss>>t>>q;
    if(!ss) return false;
    if(t < pathOffset) {
      fprintf(stderr,"set_tq: warning, cut time %g is less than path's endtime %g\n",t,pathOffset);
      return false;
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
      fprintf(stderr,"append_tq: warning, append time %g is less than path's endtime %g\n",t,path.EndTime());
      return false;
    }
    AppendLinear(q,t-path.EndTime());
    return true;
  }
  else if(name == "set_q") {
    ss>>q;
    if(!ss) return false;
    Cut(0);
    AppendRamp(q);
    return true;
  }
  else if(name == "append_q") {
    ss>>q;
    if(!ss) return false;
    AppendRamp(q);
    return true;
  }
  else if(name == "set_qv") {
    ss>>q>>v;
    if(!ss) return false;
    Cut(0);
    AppendRamp(q,v);
    return true;
  }
  else if(name == "append_qv") {
    ss>>q>>v;
    if(!ss) return false;
    AppendRamp(q,v);
    return true;
  }
  else if(name == "brake") {
    //Brake();
    fprintf(stderr,"Brake is not done yet\n");
    return false;
    return true;
  }
  return false;
}
