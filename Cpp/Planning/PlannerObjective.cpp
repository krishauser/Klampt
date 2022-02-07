#include "PlannerObjective.h"
#include <KrisLibrary/utils/AnyCollection.h>
#include <KrisLibrary/robotics/IKFunctions.h>
#include <IO/JSON.h>
#include <string.h>

namespace Klampt {

struct ErrorAccumulator
{
  /// Sets an L-p norm
  ErrorAccumulator(Real norm=1.0);
  /// Can accept L1, L2, Linf, MAE (mean absolute error)
  /// MSE (mean squared error), RMSE (root mean squared error)
  ErrorAccumulator(const char* type);
  void Add(Real error);
  void Add(Real error,Real weight);
  Real Value() const;

  Real norm;
  bool mean,root;
  Real accumulator;
  Real sumWeights;
};

ErrorAccumulator::ErrorAccumulator(Real _norm)
  :norm(_norm),mean(false),root(true),accumulator(0.0),sumWeights(0.0)
{}

ErrorAccumulator::ErrorAccumulator(const char* type)
  :accumulator(0.0),sumWeights(0.0)
{
  if(0==strcmp(type,"L1")) {
    norm=1.0;
    mean=false;
    root=false;
  }
  else if(0==strcmp(type,"L2")) {
    norm=2.0;
    mean=false;
    root=true;
  }
  else if(0==strcmp(type,"Linf")) {
    norm=Inf;
    mean=false;
    root=false;
  }
  else if(0==strcmp(type,"RMSE")) {
    norm=2.0;
    mean=true;
    root=true;
  }
  else if(0==strcmp(type,"MSE")) {
    norm=2.0;
    mean=true;
    root=false;
  }
  else if(0==strcmp(type,"MAE")) {
    norm=1.0;
    mean=true;
    root=false;
  }
}

void ErrorAccumulator::Add(Real error)
{
  if(norm == 1.0)
    accumulator += Abs(error);
  else if(norm == 2.0)
    accumulator += error*error;
  else if(IsInf(norm))
    accumulator = Max(accumulator,Abs(error));
  else
    accumulator += Pow(Abs(error),norm);
  sumWeights += 1.0;
}

void ErrorAccumulator::Add(Real error,Real weight)
{
  if(norm == 1.0)
    accumulator += Abs(error)*weight;
  else if(norm == 2.0)
    accumulator += error*error*weight;
  else if(IsInf(norm))
    accumulator = Max(accumulator,weight*Abs(error));
  else
    accumulator += weight*Pow(Abs(error),norm);
  sumWeights += weight;
}

Real ErrorAccumulator::Value() const
{
  if(IsInf(norm)) return accumulator;
  if(norm == 1.0) {
    if(mean) return accumulator/sumWeights;
    return accumulator;
  }
  if(norm == 2.0) {
    if(mean && !root) return accumulator/sumWeights;
    else if(mean && root) return Sqrt(accumulator/sumWeights);
    else if(root) return Sqrt(accumulator);
    return accumulator;
  }
  if(mean && !root) return accumulator/sumWeights;
  else if(mean && root) return Pow(accumulator/sumWeights,1.0/norm);
  else if(root) return Pow(accumulator,1.0/norm);
  return accumulator;
}


Real PlannerObjectiveBase::IncrementalCost(Real tstart,const ParabolicRamp::DynamicPath& path)
{
  Real c = 0.0;
  Real t = tstart;
  for(size_t i=0;i<path.ramps.size();i++) {
    c += IncrementalCost(t,path.ramps[i]);
    t += path.ramps[i].endTime;
  }
  return c;
}

Real PlannerObjectiveBase::PathCost(const ParabolicRamp::DynamicPath& path,Real tstart)
{
  Real c = 0.0;
  Real t = tstart;
  for(size_t i=0;i<path.ramps.size();i++) {
    c += IncrementalCost(t,path.ramps[i]);
    t += path.ramps[i].endTime;
  }
  c += TerminalCost(t,Vector(path.ramps.back().x1),Vector(path.ramps.back().dx1));
  return c;
}

Real TimeObjective::PathCost(const ParabolicRamp::DynamicPath& path,Real tstart)
{
  return tstart+path.GetTotalTime();
}

Real TimeObjective::IncrementalCost(Real t,const ParabolicRamp::ParabolicRampND& ramp)
{
  return ramp.endTime;
}

Real TimeObjective::Delta(PlannerObjectiveBase* priorGoal)
{
  if(priorGoal->TypeString() != TypeString()) return Inf;
  return 0;
}



TerminalTimeObjective::TerminalTimeObjective(Real _tgoal)
  : tgoal(_tgoal)
{}

Real TerminalTimeObjective::TerminalCost(Real tend,const Vector& qend,const Vector& dqend) {
  return Abs(tend-tgoal);
}

Real TerminalTimeObjective::PathCost(const ParabolicRamp::DynamicPath& path,Real tstart) {
  return Abs(path.GetTotalTime()+tstart-tgoal);
}

Real TerminalTimeObjective::Delta(PlannerObjectiveBase* priorGoal)
{
  if(priorGoal->TypeString() != TypeString()) return Inf;
  return priorGoal->TerminalCost(tgoal,Config(),Vector());
}

ConfigObjective::ConfigObjective(const Config& _qgoal)
    : qgoal(_qgoal)
{
}

Real ConfigObjective::Delta(PlannerObjectiveBase* priorGoal)
{
  if(priorGoal->TypeString() != TypeString()) return Inf;
  return priorGoal->TerminalCost(0.0,qgoal,Vector());
}

VelocityObjective::VelocityObjective(const Vector& _vgoal)
    : vgoal(_vgoal)
{}

Real VelocityObjective::Delta(PlannerObjectiveBase* priorGoal)
{
  if(priorGoal->TypeString() != TypeString()) return Inf;
  return priorGoal->TerminalCost(0.0,Config(),vgoal);
}




CartesianObjective::CartesianObjective(RobotModel* _robot)
    : robot(_robot)
{
  ikGoal.link = -1;
}

Real CartesianObjective::TerminalCost(Real tend,const Vector& qend,const Vector& dqend)
{
  Vector3 pworld;
  robot->UpdateConfig(qend);
  robot->GetWorldPosition(ikGoal.localPosition,ikGoal.link,pworld);
  return ikGoal.endPosition.distance(pworld);
}

Real CartesianObjective::Delta(PlannerObjectiveBase* priorGoal)
{
  if(priorGoal->TypeString() != TypeString()) return Inf;
  const CartesianObjective* cartObj = dynamic_cast<const CartesianObjective*>(priorGoal);
  return ikGoal.endPosition.distance(cartObj->ikGoal.endPosition);
}



IKObjective::IKObjective(RobotModel* _robot)
  : robot(_robot)
{
  ikGoal.link = -1;
  posCoeff = 1;
  oriCoeff = 1;
}

Real IKObjective::TerminalCost(Real tend,const Vector& qend,const Vector& dqend)
{
  Assert(ikGoal.link >= 0 && ikGoal.link < robot->q.n);
  Assert(ikGoal.link >= 0 && ikGoal.link < (int)robot->links.size());
  Assert(qend.n == robot->q.n);
  robot->UpdateConfig(qend);
  Vector3 poserr,orierr;
  EvalIKError(ikGoal,robot->links[ikGoal.link].T_World,&poserr.x,&orierr.x);
  return posCoeff*poserr.norm() + oriCoeff*orierr.norm();
}

Real IKObjective::Delta(PlannerObjectiveBase* priorGoal)
{
  if(priorGoal->TypeString() != TypeString()) return Inf;
  return ikGoal.endPosition.distance(dynamic_cast<const IKObjective*>(priorGoal)->ikGoal.endPosition);
}




CompositeObjective::CompositeObjective()
: norm(1.0)
{}

CompositeObjective::~CompositeObjective()
{
}

void CompositeObjective::Add(const shared_ptr<PlannerObjectiveBase>& obj,Real weight)
{
  components.push_back(obj);
  if(!weights.empty())
    weights.push_back(weight);
  else if(weight!=1.0) {
    weights.resize(components.size(),1.0);
    weights.back()=weight;
  }
}

string CompositeObjective::Description()
{ 
  string desc = TypeString();
  desc += "(";
  for(size_t i=0;components.size();i++) {
    if(i > 0) desc += ",";
    desc += components[i]->Description();
    if(!weights.empty() && weights[i] != 1.0) {
      char buf[64];
      snprintf(buf,64,"*%g\n",weights[i]);
      desc += buf;
    }
  }
  desc += ")";
  return desc;
}

Real CompositeObjective::TerminalCost(Real tend,const Vector& qend,const Vector& dqend) 
{ 
  ErrorAccumulator accum(norm);
  for(size_t i=0;i<components.size();i++) {
    Real d=components[i]->TerminalCost(tend,qend,dqend);
    Real w = (weights.empty()?1.0:weights[i]);
    accum.Add(d,w);
  }
  return accum.Value();
}

Real CompositeObjective::DifferentialCost(Real t,const Vector& q,const Vector& dq)
{
  ErrorAccumulator accum(norm);
  for(size_t i=0;i<components.size();i++) {
    Real d=components[i]->DifferentialCost(t,q,dq);
    Real w = (weights.empty()?1.0:weights[i]);
    accum.Add(d,w);
  }
  return accum.Value();
}


Real CompositeObjective::IncrementalCost(Real t,const ParabolicRamp::ParabolicRampND& ramp)
{
  //TODO: this isn't exactly the same as the sum of differential costs except
  //for norm = 1
  ErrorAccumulator accum(norm);
  for(size_t i=0;i<components.size();i++) {
    Real d=components[i]->IncrementalCost(t,ramp);
    Real w = (weights.empty()?1.0:weights[i]);
    accum.Add(d,w);
  }
  return accum.Value();
}


Real CompositeObjective::PathCost(const ParabolicRamp::DynamicPath& path,Real tstart)
{
  //TODO: this isn't exactly the same as the sum of increment costs except
  //for norm = 1
  ErrorAccumulator accum(norm);
  for(size_t i=0;i<components.size();i++) {
    Real d=components[i]->PathCost(path,tstart);
    Real w = (weights.empty()?1.0:weights[i]);
    accum.Add(d,w);
  }
  return accum.Value();
}

Real CompositeObjective::Delta(PlannerObjectiveBase* priorGoal)
{ 
  if(priorGoal->TypeString() != TypeString()) return Inf;
  CompositeObjective* pcomposite = dynamic_cast<CompositeObjective*>(priorGoal);
  if(pcomposite->components.size() != components.size()) return Inf;

  ErrorAccumulator accum(norm);
  for(size_t i=0;i<pcomposite->components.size();i++) {
    Real w1 = (weights.empty()?1.0:weights[i]);
    Real w2 = (pcomposite->weights.empty()?1.0:pcomposite->weights[i]);
    Real d=components[i]->Delta(pcomposite->components[i].get());
    if(IsInf(d)) return Inf;
    accum.Add(d,w1);
    accum.Add(w1-w2);
  }
  return accum.Value();
}

bool CompositeObjective::TerminalTimeInvariant() const
{
  for(size_t i=0;i<components.size();i++)
    if(!components[i]->TerminalTimeInvariant()) return false;
  return true;
}

bool CompositeObjective::DifferentialTimeInvariant() const
{
  for(size_t i=0;i<components.size();i++)
    if(!components[i]->DifferentialTimeInvariant()) return false;
  return true;
}

bool CompositeObjective::PathInvariant() const
{
  for(size_t i=0;i<components.size();i++)
    if(!components[i]->PathInvariant()) return false;
  return true;
}






CartesianTrackingObjective::CartesianTrackingObjective(RobotModel* _robot)
  :robot(_robot),link(-1),localPosition(0.0),
   endTimeWeight(0.0),endPosWeight(0.0),endPosMatWeight(0.0)
{}

int CartesianTrackingObjective::FindSegment(Real t) const
{
  if(times.empty()) return -1;
  vector<Real>::const_iterator n=std::upper_bound(times.begin(),times.end(),t);
  vector<Real>::const_iterator i = n; --i;
  if(n == times.begin()) { assert(t < times[0]); return -1; }
  if(n == times.end()) { assert(t >= times.back()); return times.size(); }
  assert(t >= *i && t < *n);
  return (i-times.begin());
}


//int_{0 to 1} d(p,a+u(b-a)) du
//d(p,a+u(b-a)) = (a-p + u(b-a)).A(a-p + u(b-a))
//    = (a-p).A(a-p) + 2 u (b-a).A(a-p) + u^2 (b-a).A(b-a)
//int_{0 to 1} d(p,a+u(b-a)) du 
//    = int_{0 to 1} (a-p).A(a-p) + 2 u (b-a).A(a-p) + u^2 (b-a).A(b-a)
//    = (a-p).A(a-p) + (b-a).A(a-p) + 1/3 (b-a).A(b-a)
//    = ((a+b)/2-p).A((a+b)/2-p) + 1/12 (b-a).A(b-a)
Real CartesianTrackingObjective::TerminalCost(Real tend,const Vector& qend,const Vector& dqend)
{
  assert(positions.size()==times.size());
  if(positions.empty()) return 0.0;

  //printf("TerminalCost\n");

  //compute workspace position at terminal config
  robot->UpdateConfig(qend);
  Vector3 p = robot->links[link].T_World*localPosition;

  Real sum=0.0;
  if(tend < times.back()) {
    //need to integrate the remainder of the trajectory
    int index=FindSegment(tend);
    if(index >= 0) {
      //first segment: integrate from tend to the next segment portion
      Real dt = times[index+1]-tend;
      Real u = (tend-times[index])/(times[index+1]-times[index]);
      Vector3 pu = positions[index]+u*(positions[index+1]-positions[index]);
      Vector3 p1 = positions[index+1];
      Vector3 pmid = (p1 + pu)*0.5;
      Real w = (weights.empty() ? 1.0 : weights[index]);
      Real err = (matWeights.empty() ?
		  w*(p.distanceSquared(pmid) + p1.distanceSquared(pu)/12.0)
		  : (p-pmid).dot(matWeights[index]*(p-pmid)) + (p1-pu).dot(matWeights[index]*(p1-pu))/12.0);
      sum += err*dt;
    }
    else assert(index == -1);
    //remaining segments
    for(size_t i=(size_t)index+1;i+1<times.size();i++) {
      Real dt = times[i+1]-times[i];
      Vector3 p0 = positions[i];
      Vector3 p1 = positions[i+1];
      Vector3 pmid = (p1 + p0)*0.5;
      Real w = (weights.empty() ? 1.0 : weights[i]);
      Real err = (matWeights.empty() ?
		  w*(p.distanceSquared(pmid) + p1.distanceSquared(p0)/12.0)
		  : (p-pmid).dot(matWeights[i]*(p-pmid)) + (p1-p0).dot(matWeights[i]*(p1-p0))/12.0);
      sum += err*dt;
    }
  }
  sum += endPosWeight*p.distanceSquared(positions.back());
  sum += (p-positions.back()).dot(endPosMatWeight*(p-positions.back()));
  return sum;
}

Real CartesianTrackingObjective::DifferentialCost(Real t,const Vector& q,const Vector& dq)
{
  if(times.empty()) return 0.0;
  //penalize exceeding end time
  if(t >= times.back()) return endTimeWeight;
  int index=FindSegment(t);
  return DifferentialCost(t,q,index);
}

Real CartesianTrackingObjective::DifferentialCost(Real t,const Vector& q,int index)
{
  if(index < 0) return 0.0;

  robot->UpdateConfig(q);
  Vector3 p = robot->links[link].T_World*localPosition;
  
  Real u=(t-times[index])/(times[index+1]-times[index]);
  Vector3 pu = positions[index]+u*(positions[index+1]-positions[index]);
  Real err = (matWeights.empty() ? (weights.empty() ? p.distanceSquared(pu) : weights[index]*p.distanceSquared(pu)) : (p-pu).dot(matWeights[index]*(p-pu)));
  return err;
}

Real CartesianTrackingObjective::IncrementalCost(Real t,const ParabolicRamp::ParabolicRampND& ramp)
{
  if(times.empty()) return 0.0;
  if(t>=times.back()) return ramp.endTime*endTimeWeight;
  if(t+ramp.endTime<times.front()) return 0.0;

  assert(times.front() == 0.0);

  Real sum=0.0;
  int sindex = FindSegment(t);
  int eindex = FindSegment(t+ramp.endTime);
  Real tindex=t;
  int index=sindex;
  if(sindex < 0) {
    //start in middle of ramp
    tindex=times.front();
    index = 0;
  }
  while(index < eindex && index+1 < (int)times.size()) {
    Real tnext = times[index+1];
    sum += IntegrateSegment(index,tindex,tnext,t,ramp);
    tindex = tnext;
    index++;
  }
  if(index == eindex && eindex < (int)times.size()) 
    sum += IntegrateSegment(eindex,tindex,t+ramp.endTime,t,ramp);
  else
    sum += (t+ramp.endTime - times.back())*endTimeWeight;
  return sum;
}

//Constants for Gaussian quadrature with n=5
const static Real g5points [5] = {0.0,Sqrt(5.0-2*Sqrt(10.0/7.0))/3.0,-Sqrt(5.0-2*Sqrt(10.0/7.0))/3.0,Sqrt(5.0+2*Sqrt(10.0/7.0))/3.0,-Sqrt(5.0+2*Sqrt(10.0/7.0))/3.0};
const static Real g5weights [5] = {128.0/225.0,(322.0+13*Sqrt(70.0))/900,(322.0+13*Sqrt(70.0))/900,(322.0-13*Sqrt(70.0))/900,(322.0-13*Sqrt(70.0))/900};

//piecewise linear discretization:
//line-to-line distance
//int_{-1/2 to -1/2} d((p+q)/2+u(q-p),(a+b)/2+u(b-a)) du
//let r = (p+q)/2, c = (a+b)/2
//d(r+u(q-p),c+u(b-a)) = (c-r + u(b-a+p-q)).A(c-r + u(b-a+p-q))
//    = (c-r).A(c-r) + 2 u (b-a+p-q).A(c-r) + u^2 (b-a+p-q).A(b-a+p-q)
//int_{1/2 to -1/2} d(r+u(q-p),c+u(b-a)) du 
//    = int_{0 to 1} (c-r).A(c-r) + 2 u (b-a+p-q).A(c-r) + u^2 (b-a+p-q).A(b-a+p-q)
//    = (c-r).A(c-r) + 1/12 (b-a+p-q).A(b-a+p-q)
Real CartesianTrackingObjective::IntegrateSegment(int index,Real a,Real b,Real tstart,const ParabolicRamp::ParabolicRampND& ramp)
{
  //printf("IntegrateSegment\n");
  assert(index >= 0 && index+1 < (int)times.size());
  assert(a <= b);
  assert(times[index] <= a && b <= times[index+1]);
  assert(tstart-Epsilon <= a && a <= tstart+ramp.endTime+Epsilon);
  assert(tstart-Epsilon <= b && b <= tstart+ramp.endTime+Epsilon);

  Real sum=0.0;
  for(int i=0;i<5;i++) {
    Real t = (a+b)*0.5 + g5points[i]*(b-a)*0.5;
    vector<Real> qt;
    ramp.Evaluate(t-tstart,qt);
    sum += g5weights[i] * DifferentialCost(t,Vector(qt),index);
  }
  return sum*(b-a)*0.5;
}

Real CartesianTrackingObjective::PathCost(const ParabolicRamp::DynamicPath& path,Real tstart)
{
  return PlannerObjectiveBase::PathCost(path,tstart);
}

Real CartesianTrackingObjective::Delta(PlannerObjectiveBase* priorGoal)
{
  if(priorGoal->TypeString() != TypeString()) return Inf;
  //printf("Delta\n");
  const CartesianTrackingObjective* cartObj = dynamic_cast<const CartesianTrackingObjective*>(priorGoal);
  if(link != cartObj->link) return Inf;
  if(localPosition != cartObj->localPosition) return Inf;
  //TODO: allow different time scales?
  size_t n = std::min(positions.size(),cartObj->positions.size());
  Real sum=0.0;
  for(size_t i=0;i+1<n;i++) {
    Real dt = times[i+1]-times[i];
    Vector3 p0 = positions[i];
    Vector3 p1 = positions[i+1];
    Vector3 pmid = (p1 + p0)*0.5;
    Vector3 q0 = cartObj->positions[i];
    Vector3 q1 = cartObj->positions[i+1];
    Vector3 qmid = (q1 + q0)*0.5;
    Real w = (weights.empty() ? 1.0 : weights[i]);
    Real err = (matWeights.empty() ?
		w*(qmid.distanceSquared(pmid) + (p1-p0+q0-q1).normSquared()/12.0)
		: (qmid-pmid).dot(matWeights[i]*(qmid-pmid)) + (p1-p0+q0-q1).dot(matWeights[i]*(p1-p0+q0-q1))/12.0);
    sum += err*dt;
  }
  for(size_t i=n;i+1<positions.size();i++) {
    Real dt = times[i+1]-times[i];
    Vector3 p0 = positions[i];
    Vector3 p1 = positions[i+1];
    Vector3 pmid = (p1 + p0)*0.5;
    Vector3 q = cartObj->positions.back();
    Real w = (weights.empty() ? 1.0 : weights[i]);
    Real err = (matWeights.empty() ?
		w*(q.distanceSquared(pmid) + (p1-p0+q).normSquared()/12.0)
		: (q-pmid).dot(matWeights[i]*(q-pmid)) + (p1-p0+q).dot(matWeights[i]*(p1-p0+q))/12.0);
    sum += err*dt;
  }
  for(size_t i=n;i+1<cartObj->positions.size();i++) {
    Real dt = cartObj->times[i+1]-cartObj->times[i];
    Vector3 p0 = cartObj->positions[i];
    Vector3 p1 = cartObj->positions[i+1];
    Vector3 pmid = (p1 + p0)*0.5;
    Vector3 q = positions.back();
    Real w = (cartObj->weights.empty() ? 1.0 : cartObj->weights[i]);
    Real err = (cartObj->matWeights.empty() ?
		w*(q.distanceSquared(pmid) + (p1-p0+q).normSquared()/12.0)
		: (q-pmid).dot(cartObj->matWeights[i]*(q-pmid)) + (p1-p0+q).dot(matWeights[i]*(p1-p0+q))/12.0);
    sum += err*dt;
  }
  return sum*10;
}

Vector3 CartesianTrackingObjective::GetDesiredPosition(Real t) const
{
  if(t>=times.back()) return positions.back();
  if(t<=times.front()) return positions.front();
  int index = FindSegment(t);
  Real u=(t-times[index])/(times[index+1]-times[index]);
  return positions[index]+u*(positions[index+1]-positions[index]);
}

void CartesianTrackingObjective::GetDifferentialCostFunction(Real t,Matrix3& A,Vector3& b) const
{
  if(t < times.front() || t >= times.back()) {
    A.setZero();
    b.setZero();
    return;
  }
  int index=FindSegment(t);
  Real u=(t-times[index])/(times[index+1]-times[index]);
  Vector3 pu = positions[index]+u*(positions[index+1]-positions[index]);
  b = pu;
  if(matWeights.empty()) {
    A.setIdentity();
    if (!weights.empty())  A *= weights[index];
  }
  else A = matWeights[index];
}

bool SavePlannerObjective(PlannerObjectiveBase* obj,AnyCollection& msg)
{
  msg.clear();
  string type = string(obj->TypeString());
  msg["type"] = type;
  if(type == "time") {
  }
  else if(type == "term_time") {
    msg["data"] = dynamic_cast<TerminalTimeObjective*>(obj)->tgoal;
  }
  else if(type == "config") {
    msg["data"] = vector<Real>(dynamic_cast<ConfigObjective*>(obj)->qgoal);
  }
  else if(type == "velocity") {
    msg["data"] = vector<Real>(dynamic_cast<VelocityObjective*>(obj)->vgoal);
  }
  else if(type == "composite") {
    CompositeObjective* cobj = dynamic_cast<CompositeObjective*>(obj);
    msg["norm"] = cobj->norm;
    for(size_t i=0;i<cobj->components.size();i++) {
      if(!SavePlannerObjective(cobj->components[i].get(),msg["components"][i])) {
	fprintf(stderr,"SavePlannerObjective: error saving component %d of composite objective\n",(int)i);
	return false;
      }
    }
    msg["weights"] = cobj->weights;
  }
  else if(type == "cartesian") {
    CartesianObjective* cobj = dynamic_cast<CartesianObjective*>(obj);
    msg["link"] = cobj->ikGoal.link;
    Convert(cobj->ikGoal.localPosition,msg["plocal"]);
    Convert(cobj->ikGoal.endPosition,msg["pworld"]);
  }
  else if(type == "ik") {
    IKObjective* cobj = dynamic_cast<IKObjective*>(obj);
    Convert(cobj->ikGoal,msg["data"]);
  }
  else {
    fprintf(stderr,"SavePlannerObjective: unknown objective type %s\n",type.c_str());
    return false;
  }
  return true;
}


PlannerObjectiveBase* LoadPlannerObjective(AnyCollection& msg,RobotModel* robot)
{
  string type;
  bool res = msg["type"].as<string>(type);
  if(!res) {
    fprintf(stderr,"LoadPlannerObjective: message didn't contain 'type' member\n");
    return NULL;
  }
  if(type == "time") {
    return new TimeObjective();
  }
  else if(type == "term_time") {
    return new TerminalTimeObjective((Real)msg["data"]);
  }
  else if(type == "config") {
    vector<Real> q;
    if(!msg["data"].asvector(q)) {
      fprintf(stderr,"LoadPlannerObjective: config message didn't contain 'data' member\n");
      return NULL;
    }
    if(robot && q.size() != robot->links.size()) {
      fprintf(stderr,"LoadPlannerObjective: config message contains desired configuration of incorrect length %d vs %d\n",(int)q.size(),(int)robot->links.size());
      return NULL;
    }
    return new ConfigObjective(Vector(q));
  }
  else if(type == "velocity") {
    vector<Real> v;
    if(!msg["data"].asvector(v)) {
      fprintf(stderr,"LoadPlannerObjective: velocity  message didn't contain 'data' member\n");
      return NULL;
    }
    if(robot && v.size() != robot->links.size()) {
      fprintf(stderr,"LoadPlannerObjective: velocity message contains desired velocity of incorrect length %d vs %d\n",(int)v.size(),(int)robot->links.size());
      return NULL;
    }
    return new VelocityObjective(Vector(v));
  }
  else if(type == "composite") {
    vector<shared_ptr<AnyCollection> > items;
    AnyCollection msgcomp = msg["components"];
    if(msgcomp.depth() == 0) {
      fprintf(stderr,"LoadPlannerObjective: composite message didn't contain 'components' member\n");
      return NULL;
    }
    msgcomp.enumerate(items);
    vector<shared_ptr<PlannerObjectiveBase> > components;
    for(size_t i=0;i<items.size();i++) {
      components.push_back(shared_ptr<PlannerObjectiveBase>(LoadPlannerObjective(*items[i],robot)));
      if(components.back()==NULL) return NULL;
    }
    CompositeObjective* obj = new CompositeObjective;
    if(msg["norm"].as<Real>(obj->norm)) {}
    if(msg["weights"].asvector(obj->weights)) {}
    else obj->weights.resize(components.size(),1);
    obj->components=components;
    return obj;
  }
  else {
    fprintf(stderr,"LoadPlannerObjective: message of unknown type %s\n",type.c_str());
    return NULL;
  }
}


PlannerObjectiveBase* LoadPlannerObjective(istream& in,RobotModel* robot)
{
  AnyCollection msg;
  in>>msg;
  if(!in) {
    fprintf(stderr,"LoadPlannerObjective: Unable to parse message\n");
    return NULL;
  }
  return LoadPlannerObjective(msg,robot);
}


bool SavePlannerObjective(PlannerObjectiveBase* obj,ostream& out)
{
  AnyCollection msg;
  if(!SavePlannerObjective(obj,msg)) return false;
  out<<msg;
  if(!out) {
    fprintf(stderr,"LoadPlannerObjective: Unable to write message\n");
    return false;
  }
  return true;
}

} //namespace Klampt