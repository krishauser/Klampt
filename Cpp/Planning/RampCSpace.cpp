#include "RampCSpace.h"
#include "Klampt/Modeling/DynamicPath.h"
#include <KrisLibrary/math/random.h>
using namespace std;
using namespace Klampt;

RampCSpaceAdaptor::RampCSpaceAdaptor(CSpace* _cspace,const Vector& _velMax,const Vector& _accMax)
  :cspace(_cspace),velMax(_velMax),accMax(_accMax),visibilityTolerance(1e-3)
{}

int RampCSpaceAdaptor::NumDimensions() { return cspace->NumDimensions()*2; }

bool RampCSpaceAdaptor::IsFeasible(const Config& q,const Config& dq)
{
  for(int i=0;i<dq.n;i++) {
    if(Abs(dq(i)) > velMax[i]) {
      //printf("Velocity exceeded bound %d\n",i);
      return false;
    }
  }
  return cspace->IsFeasible(q);
}

bool RampCSpaceAdaptor::IsFeasible(const State& s)
{
  Config q,dq;
  q.setRef(s,0,1,velMax.size());
  dq.setRef(s,velMax.size(),1,velMax.size());
  return IsFeasible(q,dq);
}

void RampCSpaceAdaptor::Sample(State& s)
{
  s.resize(velMax.size()*2);
  Config q,dq;
  q.setRef(s,0,1,velMax.size());
  dq.setRef(s,velMax.size(),1,velMax.size());
  cspace->Sample(q);
  for(int i=0;i<dq.n;i++)
    dq(i) =Rand(-velMax[i],velMax[i]);
}

EdgePlannerPtr RampCSpaceAdaptor::LocalPlanner(const State& a,const State& b)
{
  return make_shared<RampEdgeChecker>(this,a,b);
}

Real RampCSpaceAdaptor::Distance(const State& x, const State& y)
{
  ParabolicRamp::ParabolicRampND ramp;
  ramp.x0.resize(x.n/2);
  ramp.dx0.resize(x.n/2);
  ramp.x1.resize(x.n/2);
  ramp.dx1.resize(x.n/2);
  copy(x.begin(),x.begin()+x.n/2,ramp.x0.begin());
  copy(x.begin()+x.n/2,x.end(),ramp.dx0.begin());
  copy(y.begin(),y.begin()+x.n/2,ramp.x1.begin());
  copy(y.begin()+y.n/2,y.end(),ramp.dx1.begin());
  /*
  //this code works if the ramp is defined on Vectors
  ramp.x0.setRef(x,0,1,x.n/2);
  ramp.dx0.setRef(x,x.n/2,1,x.n/2);
  ramp.x1.setRef(y,0,1,y.n/2);
  ramp.dx1.setRef(y,y.n/2,1,y.n/2);
  */
  if(!ramp.SolveMinTime(accMax,velMax)) return Inf;
  return ramp.endTime;
}

void RampCSpaceAdaptor::Interpolate(const State& x,const State& y,Real u,State& out)
{
  ParabolicRamp::ParabolicRampND ramp;
  ramp.x0.resize(x.n/2);
  ramp.dx0.resize(x.n/2);
  ramp.x1.resize(x.n/2);
  ramp.dx1.resize(x.n/2);
  copy(x.begin(),x.begin()+x.n/2,ramp.x0.begin());
  copy(x.begin()+x.n/2,x.end(),ramp.dx0.begin());
  copy(y.begin(),y.begin()+x.n/2,ramp.x1.begin());
  copy(y.begin()+y.n/2,y.end(),ramp.dx1.begin());
  /*
  //this code works if the ramp is defined on Vectors
  ramp.x0.setRef(x,0,1,x.n/2);
  ramp.dx0.setRef(x,x.n/2,1,x.n/2);
  ramp.x1.setRef(y,0,1,y.n/2);
  ramp.dx1.setRef(y,y.n/2,1,y.n/2);
  */
  bool res=ramp.SolveMinTime(accMax,velMax);
  if(!res) {
    CSpace::Interpolate(x,y,u,out);
  }
  else {
    out.resize(x.n);
    Vector q,dq;
    q.setRef(out,0,1,x.n/2);
    dq.setRef(out,x.n/2,1,x.n/2);
    ParabolicRamp::Vector vq,vdq;
    ramp.Evaluate(u*ramp.endTime,vq);
    ramp.Derivative(u*ramp.endTime,vdq);
    q = vq;
    dq = vdq;
    /*
    //this code works if the ramp is defined on Vectors
    ramp.Evaluate(u*ramp.endTime,q);
    ramp.Derivative(u*ramp.endTime,dq);
    */
  }
}

void RampCSpaceAdaptor::Properties(PropertyMap& props)
{
  cspace->Properties(props);
  props.set("euclidean",0);
  props.set("geodesic",1);
  Real v;
  if(!props.get("volume",v)) {
    v = 1.0;
    for(size_t i=0;i<qMin.size();i++)
      v *= qMax[i]-qMin[i];
  }
  for(size_t i=0;i<velMax.size();i++)
    v *= 2.0*velMax[i];
  props.set("metric","execution time");
  props.set("volume",v);
  vector<Real> qvmin,qvmax;
  qvmin = qMin;
  qvmax = qMax;
  for(size_t i=0;i<velMax.size();i++) {
    qvmin.push_back(-velMax[i]); 
    qvmax.push_back(velMax[i]); 
  }
  props.setArray("minimum",qvmin);
  props.setArray("maximum",qvmax);
  props.remove("diameter");
}



RampEdgeChecker::RampEdgeChecker(RampCSpaceAdaptor* _space,const State& a,const State& b)
  :space(_space),start(a),goal(b),checked(0)
{
  Assert(space->accMax.size()*2 == start.n);
  Assert(space->accMax.size()*2 == goal.n);
  Assert(space->accMax.size() == space->velMax.size());
  Vector x0,dx0,x1,dx1;
  x0.setRef(start,0,1,start.n/2);
  dx0.setRef(start,start.n/2,1,start.n/2);
  x1.setRef(goal,0,1,goal.n/2);
  dx1.setRef(goal,goal.n/2,1,goal.n/2);
  path.xMin = space->qMin;
  path.xMax = space->qMax;
  path.velMax = space->velMax;
  path.accMax = space->accMax;
  if(!path.SolveMinTime(x0,dx0,x1,dx1))
    path.ramps.clear();
}

RampEdgeChecker::RampEdgeChecker(RampCSpaceAdaptor* _space,const ParabolicRamp::ParabolicRampND& _ramp)
  :space(_space),checked(0)
{
  path.ramps.resize(1,_ramp);
  start.resize(_ramp.x0.size()+_ramp.dx0.size());
  start.copySubVector(0,Vector(_ramp.x0));
  start.copySubVector(_ramp.x0.size(),Vector(_ramp.dx0));
  goal.resize(start.n);
  goal.copySubVector(0,Vector(_ramp.x1));
  goal.copySubVector(_ramp.x1.size(),Vector(_ramp.dx1));
}

RampEdgeChecker::RampEdgeChecker(RampCSpaceAdaptor* _space,const ParabolicRamp::DynamicPath& _path)
  :space(_space),path(_path),checked(0)
{
  int n=(int)path.ramps.front().x0.size();
  start.resize(n+n);
  start.copySubVector(0,Vector(path.ramps.front().x0));
  start.copySubVector(n,Vector(path.ramps.front().dx0));
  goal.resize(n+n);
  goal.copySubVector(0,Vector(path.ramps.back().x1));
  goal.copySubVector(n,Vector(path.ramps.back().dx1));
}


Real RampEdgeChecker::Duration() const
{
  return path.GetTotalTime();
}

Real RampEdgeChecker::Length() const
{
  return Duration();
}

bool RampEdgeChecker::IsValid() const
{
  for(size_t i=0;i<path.ramps.size();i++) if(!path.ramps[i].IsValid()) return false;
  return true;
}

EdgePlannerPtr RampEdgeChecker::Copy() const
{
  auto copy = make_shared<RampEdgeChecker>(space,path);
  copy->checked = checked;
  return copy;
}

EdgePlannerPtr RampEdgeChecker::ReverseCopy() const
{
  auto copy = make_shared<RampEdgeChecker>(space,goal,start);
  //SolveMinTime is not guaranteed to work!
  if(copy->path.ramps.empty() && !path.ramps.empty()) {
    fprintf(stderr,"RampEdgeChecker::ReverseCopy(): couldn't solve reverse path\n");
    fprintf(stderr,"Press enter to continue...\n");
    getchar();
  }
  //FatalError("Can't do ReverseCopy");
  copy->checked = checked;
  return copy;
}

bool RampEdgeChecker::IsVisible()
{
  if(checked > 0) return true;
  else if(checked < 0) return false;

  //unchecked, now do checking
  if(path.ramps.empty()) { checked=-1; printf("Ramp empty\n"); return false; }
  if(!space->qMin.empty()) {
    ParabolicRamp::Vector bmin,bmax;
    for(size_t r=0;r<path.ramps.size();r++) {
      path.ramps[r].Bounds(0,path.ramps[r].endTime,bmin,bmax);
      for(size_t i=0;i<bmin.size();i++) {
	if(bmin[i] < space->qMin[i]) { printf("Ramp exited joint limit %d\n",i); checked=-1; return false; }
	if(bmax[i] > space->qMax[i]) { printf("Ramp exited joint limit %d\n",i);checked=-1; return false; }
      }
    }
  }
  CSpaceFeasibilityChecker checker(space->cspace);
  for(size_t r=0;r<path.ramps.size();r++) {
    if(!ParabolicRamp::CheckRamp(path.ramps[r],&checker,space->visibilityTolerance)) {
      checked=-1;
      return false;
    }
  }
  checked = 1;
  return true;
}

void RampEdgeChecker::Eval(Real u,State& x) const
{
  x.resize(start.n);
  if(path.ramps.empty()) { 
    x = start;
    return;
  }

  Vector q,dq;
  q.setRef(x,0,1,start.n/2);
  dq.setRef(x,start.n/2,1,start.n/2);
  ParabolicRamp::Vector vq,vdq;
  if(path.ramps.size()==1) {
    path.ramps[0].Evaluate(u*path.ramps[0].endTime,vq);
    path.ramps[0].Derivative(u*path.ramps[0].endTime,vdq);
  }
  else {
    //need to interpolate
    Real ttotal = Duration();
    Real t=ttotal*u;
    bool found=false;
    for(size_t i=0;i<path.ramps.size();i++) {
      if(t <= path.ramps[i].endTime) {
	path.ramps[i].Evaluate(t,vq);
	path.ramps[i].Derivative(t,vdq);
	found = true;
	break;
      }
      t -= path.ramps[i].endTime;
    }
    assert(found);
  }
  q = vq;
  dq = vdq;
}
