#include "Paths.h"
#include "Robot.h"
#include "Interpolate.h"
#include "MultiPath.h"
#include <KrisLibrary/spline/TimeSegmentation.h>
#include <KrisLibrary/spline/PiecewisePolynomial.h>
#include "ParabolicRamp.h"
#include "DynamicPath.h"
#include "Conversions.h"
using namespace std;

namespace Klampt {

LinearPath::LinearPath()
{}

LinearPath::LinearPath(const vector<Real>& _times,const vector<Vector>& _milestones)
  :times(_times),milestones(_milestones)
{}

bool LinearPath::Save(ostream& out)
{
  Assert(times.size()==milestones.size());
  for(size_t i=0;i<times.size();i++) {
    out<<times[i]<<"    "<<milestones[i]<<endl;
  }
  return true;
}

bool LinearPath::Load(istream& in)
{
  times.clear();
  milestones.clear();
  Real t;
  Vector x;
  while(in) {
    in >> t >> x;
    if(in) {
      times.push_back(t);
      milestones.push_back(x);
    }
  }
  if(in.bad()) {
    return false;
  }
  return true;
}

void LinearPath::Concat(const LinearPath& path,bool relative)
{
  if(relative && !milestones.empty()) {
    Real tofs = times.back();
    Assert(path.times[0] >= 0);
    for(size_t i=0;i<path.times.size();i++)
      times.push_back(tofs + path.times[i]);
  }
  else {
    times.insert(times.end(),path.times.begin(),path.times.end());
  }
  milestones.insert(milestones.end(),path.milestones.begin(),path.milestones.end());
}

void LinearPath::Eval(Real t,Vector& xt) const
{
  Real param;
  int seg = Spline::TimeSegmentation::Map(times,t,param);
  if(seg < 0) xt = milestones.front();
  else if(seg+1 >= (int)milestones.size()) xt=milestones.back();
  else {
    xt.mul(milestones[seg],1.0-param);
    xt.madd(milestones[seg+1],param);
  }
}

void LinearPath::Deriv(Real t,Vector& dxt) const
{
  Real param;
  int seg = Spline::TimeSegmentation::Map(times,t,param);
  dxt.resize(milestones.front().size());
  if(seg < 0) dxt.setZero();
  else if(seg+1 >= (int)milestones.size()) dxt.setZero();
  else {
    dxt.sub(milestones[seg+1],milestones[seg]);
    dxt *= 1.0/(times[seg+1]-times[seg]);
  }
}

void LinearPath::Eval(RobotModel& robot,Real t,Vector& xt) const
{
  Real param;
  int seg = Spline::TimeSegmentation::Map(times,t,param);
  if(seg < 0) xt = milestones.front();
  else if(seg+1 >= (int)milestones.size()) xt=milestones.back();
  else {
    Interpolate(robot,milestones[seg],milestones[seg+1],param,xt);
  }
}

void LinearPath::Deriv(RobotModel& robot,Real t,Vector& dxt) const
{
  Real param;
  int seg = Spline::TimeSegmentation::Map(times,t,param);
  dxt.resize(milestones.front().size());
  if(seg < 0) dxt.setZero();
  else if(seg+1 >= (int)milestones.size()) dxt.setZero();
  else {
    InterpolateDerivative(robot,milestones[seg],milestones[seg+1],param,dxt);
  }
}

ParabolicRamp::ParabolicRampND PPRamp(const Vector& a,const Vector& b,const Vector& va,const Vector& vb,Real duration)
{
  ParabolicRamp::ParabolicRampND ramp;
  ramp.x0 = a;
  ramp.x1 = b;
  ramp.dx0 = va;
  ramp.dx1 = vb;
  ParabolicRamp::Vector vinf(a.size(),Inf);
  bool res=ramp.SolveMinAccel(vinf,duration);
  Assert(res != false);
  return ramp;
}

Spline::PiecewisePolynomialND QuadraticPolynomial(const Vector& a,const Vector& b,const Vector& va,const Vector& vb,Real ta,Real tb)
{
  ParabolicRamp::ParabolicRampND ramp = PPRamp(a,b,va,vb,tb-ta);
  return Cast(ramp);
}

///Exact, direct conversion from one path format to another.
///Note that not all formats can be directly converted.
void Convert(const LinearPath& in,MultiPath& out)
{
  out.sections.resize(1);
  out.SetTimedMilestones(in.times,in.milestones,0);
}

void Convert(const LinearPath& in,ParabolicRamp::DynamicPath& out)
{
  out.ramps.resize(in.milestones.size()-1);
  for(size_t i=0;i<out.ramps.size();i++)
    out.ramps[i].SetLinear(in.milestones[i],in.milestones[i+1],in.times[i+1]-in.times[i]);
}

void Convert(const LinearPath& in,Spline::PiecewisePolynomialND& out)
{
  out.elements.resize(0);
  for(size_t i=0;i+1<in.milestones.size();i++)
    out.Concat(Spline::Linear(in.milestones[i],in.milestones[i+1],in.times[i],in.times[i+1]));
}

void Convert(const MultiPath& in,LinearPath& out)
{
  Assert(!in.HasVelocity());
  LinearPath temp;
  out.Clear();
  for(size_t i=0;i<in.sections.size();i++) {
    in.GetTimedMilestones(temp.times,temp.milestones,i);
    out.Concat(temp);
  }
}
void Convert(const MultiPath& in,ParabolicRamp::DynamicPath& out)
{
  if(!in.HasVelocity()) {
    LinearPath temp;
    Convert(in,temp);
    Convert(temp,out);
  }
  else {
    vector<Real> times;
    vector<Config> milestones;
    vector<Vector> dmilestones;
    Keyframes(in,times,milestones,dmilestones);
    Interpolate(times,milestones,dmilestones,out);
  }
}

void Convert(const MultiPath& in,Spline::PiecewisePolynomialND& out)
{
  if(!in.HasVelocity()) {
    LinearPath temp;
    Convert(in,temp);
    Convert(temp,out);
  }
  else {
    vector<Real> times;
    vector<Config> milestones;
    vector<Vector> dmilestones;
    Keyframes(in,times,milestones,dmilestones);
    Interpolate(times,milestones,dmilestones,out);
  }
}

void Convert(const ParabolicRamp::DynamicPath& in,Spline::PiecewisePolynomialND& out)
{
  out = Cast(in);
}

void Convert(const ParabolicRamp::DynamicPath& in,MultiPath& out)
{
  Spline::PiecewisePolynomialND spline;
  Convert(in,spline);
  Convert(spline,out);
}

void Convert(const Spline::PiecewisePolynomialND& in,MultiPath& out)
{
  vector<Real> times;
  vector<Vector> milestones,dmilestones;
  Keyframes(in,times,milestones,dmilestones);
  out.sections.resize(1);
  out.SetTimedMilestones(times,milestones);
  out.sections[0].velocities = dmilestones;
}



///Extract keyframes from the path representation
void Keyframes(const LinearPath& in,vector<Config>& milestones) { milestones=in.milestones; }

void Keyframes(const LinearPath& in,vector<Real>& times,vector<Config>& milestones) { times=in.times; milestones=in.milestones; }

void Keyframes(const LinearPath& in,vector<Real>& times,vector<Config>& milestones,vector<Config>& dmilestones) {
  Keyframes(in,times,milestones);
  dmilestones.resize(times.size());
  if(times.empty()) return;
  if(times.size()==1) { dmilestones[0].resize(milestones[0].size(),0.0); }
  else {
    //TODO: one-sided parabola for start and stop
    dmilestones[0] = (milestones[1]-milestones[0])*(1.0/(times[1]-times[0]));
    for(size_t i=1;i+1<dmilestones.size();i++) 
      dmilestones[i] = (milestones[i+1]-milestones[i-1])*(1.0/(times[i+1]-times[i-1]));
    dmilestones.back() = (milestones.back()-milestones[milestones.size()-2])*(1.0/(times.back()-times[times.size()-2]));
  }
}

void Keyframes(const MultiPath& in,vector<Config>& milestones)
{
  milestones.resize(0);
  vector<Config> temp;
  for(size_t i=0;i<in.sections.size();i++) {
    in.GetMilestones(temp,i);
    milestones.insert(milestones.end(),temp.begin(),temp.end());
  }
}

void Keyframes(const MultiPath& in,vector<Real>& times,vector<Config>& milestones)
{
  times.resize(0);
  milestones.resize(0);
  vector<Real> ttemp;
  vector<Config> mtemp;
  for(size_t i=0;i<in.sections.size();i++) {
    in.GetTimedMilestones(ttemp,mtemp,i);
    times.insert(times.end(),ttemp.begin(),ttemp.end());
    milestones.insert(milestones.end(),mtemp.begin(),mtemp.end());
  }
}

void Keyframes(const MultiPath& in,vector<Real>& times,vector<Config>& milestones,vector<Config>& dmilestones)
{
  Keyframes(in,times,milestones);
  dmilestones.resize(times.size());
  if(times.empty()) return;
  if(times.size()==1) { dmilestones[0].resize(milestones[0].size(),0.0); }
  else {
    //TODO: one-sided parabola
    dmilestones[0] = (milestones[1]-milestones[0])*(1.0/(times[1]-times[0]));
    for(size_t i=1;i+1<dmilestones.size();i++) 
      dmilestones[i] = (milestones[i+1]-milestones[i-1])*(1.0/(times[i+1]-times[i-1]));
    dmilestones.back() = (milestones.back()-milestones[milestones.size()-2])*(1.0/(times.back()-times[times.size()-2]));
  }
}

void Keyframes(const ParabolicRamp::DynamicPath& in,vector<Config>& milestones)
{
  milestones.resize(in.ramps.size()+1);
  for(size_t i=0;i<in.ramps.size();i++)
    milestones[i] = in.ramps[i].x0;
  milestones.back()=in.ramps.back().x1;
}

void Keyframes(const ParabolicRamp::DynamicPath& in,vector<Real>& times,vector<Config>& milestones)
{
  Keyframes(in,milestones);
  times.resize(in.ramps.size()+1);
  times[0]=0;
  for(size_t i=0;i<in.ramps.size();i++) 
    times[i+1] = times[i]+in.ramps[i].endTime;
}

void Keyframes(const ParabolicRamp::DynamicPath& in,vector<Real>& times,vector<Config>& milestones,vector<Config>& dmilestones)
{
  vector<ParabolicRamp::Vector> vmilestones,vdmilestones;
  in.GetMilestones(vmilestones,vdmilestones);
  milestones.resize(vmilestones.size());
  dmilestones.resize(vdmilestones.size());
  copy(vmilestones.begin(),vmilestones.end(),milestones.begin());
  copy(vdmilestones.begin(),vdmilestones.end(),dmilestones.begin());
  times.resize(in.ramps.size()+1);
  times[0]=0;
  for(size_t i=0;i<in.ramps.size();i++) 
    times[i+1] = times[i]+in.ramps[i].endTime;
}

void Keyframes(const Spline::PiecewisePolynomialND& in,vector<Config>& milestones)
{
  vector<Real> times;
  Keyframes(in,times,milestones);
}

void Keyframes(const Spline::PiecewisePolynomialND& in,vector<Real>& times,vector<Config>& milestones)
{
  set<Real> splits;
  for(size_t i=0;i<in.elements.size();i++)
    for(size_t j=0;j<in.elements[i].times.size();j++)
      splits.insert(in.elements[i].times[j]);
  times.resize(splits.size());
  copy(splits.begin(),splits.end(),times.begin());
  milestones.resize(times.size());
  for(size_t i=0;i<times.size();i++)
    milestones[i] = in.Evaluate(times[i]);
}

void Keyframes(const Spline::PiecewisePolynomialND& in,vector<Real>& times,vector<Config>& milestones,vector<Config>& dmilestones)
{
  Keyframes(in,times,milestones);
  dmilestones.resize(times.size());
  for(size_t i=0;i<times.size();i++)
    dmilestones[i] = in.Derivative(times[i]);
}


void Interpolate(const vector<Real>& times,const vector<Config>& milestones,LinearPath& out)
{
  out.times = times;
  out.milestones = milestones;
}

void Interpolate(const vector<Real>& times,const vector<Config>& milestones,MultiPath& out)
{
  out.sections.resize(1);
  out.SetTimedMilestones(times,milestones,0);
}

void Interpolate(const vector<Real>& times,const vector<Config>& milestones,Spline::PiecewisePolynomialND& out)
{
  Vector zero(milestones[0].size(),0.0);
  out.elements.resize(0);
  for(size_t i=0;i+1<milestones.size();i++) 
    out.Concat(QuadraticPolynomial(milestones[i],milestones[i+1],zero,zero,times[i],times[i+1]));
}

void Interpolate(const vector<Real>& times,const vector<Config>& milestones,ParabolicRamp::DynamicPath& out)
{
  out.Clear();
  Vector zero(milestones[0].size(),0.0);
  for(size_t i=0;i+1<milestones.size();i++) 
    out.ramps.push_back(PPRamp(milestones[i],milestones[i+1],zero,zero,times[i+1]-times[i]));

}

void Interpolate(const vector<Real>& times,const vector<Config>& milestones,const vector<Vector>& dmilestones,MultiPath& out)
{
  out.sections.resize(1);
  out.SetTimedMilestones(times,milestones,0);
  out.sections[0].velocities = dmilestones;
}

///Create a representation that matches the input keyframes
void Interpolate(const vector<Real>& times,const vector<Config>& milestones,const vector<Vector>& dmilestones,Spline::PiecewisePolynomialND& out)
{
  out.elements.resize(0);
  for(size_t i=0;i+1<times.size();i++) 
    out.Concat(QuadraticPolynomial(milestones[i],milestones[i+1],dmilestones[i],dmilestones[i+1],times[i],times[i+1]));
}
///Create a representation that matches the input keyframes
void Interpolate(const vector<Real>& times,const vector<Config>& milestones,const vector<Vector>& dmilestones,ParabolicRamp::DynamicPath& out)
{
  out.Clear();
  for(size_t i=0;i+1<times.size();i++) 
    out.ramps.push_back(PPRamp(milestones[i],milestones[i+1],dmilestones[i],dmilestones[i+1],times[i+1]-times[i]));
}
///Create a representation that matches the keyframes of the input path.  The path will be smooth
///and if the input path has no velocities, the output will stop at each milestone with zero velocity
void Interpolate(const MultiPath& in,ParabolicRamp::DynamicPath& out)
{
  if(in.HasVelocity()) {
    vector<Real> times;
    vector<Vector> milestones,dmilestones;
    Keyframes(in,times,milestones,dmilestones);
    Interpolate(times,milestones,dmilestones,out);
  }
  else {
    vector<Real> times;
    vector<Vector> milestones;
    Keyframes(in,times,milestones);
    Interpolate(times,milestones,out);
  }
}

void Interpolate(const MultiPath& in,Spline::PiecewisePolynomialND& out)
{
  if(in.HasVelocity()) {
    vector<Real> times;
    vector<Vector> milestones,dmilestones;
    Keyframes(in,times,milestones,dmilestones);
    Interpolate(times,milestones,dmilestones,out);
  }
  else {
    vector<Real> times;
    vector<Vector> milestones;
    Keyframes(in,times,milestones);
    Interpolate(times,milestones,out);
  }

}
void Interpolate(const ParabolicRamp::DynamicPath& in,MultiPath& out)
{
  vector<Real> times;
  vector<Vector> milestones,dmilestones;
  Keyframes(in,times,milestones,dmilestones);
  Interpolate(times,milestones,dmilestones,out);
}

///Split up the path into keyframes at a given resolution
void Discretize(const LinearPath& in,Real res,vector<Real>& times,vector<Config>& milestones)
{
  times.resize(0);
  milestones.resize(0);
  times.reserve((int)Ceil(in.Duration()/res));
  milestones.reserve((int)Ceil(in.Duration()/res));
  Real t = in.StartTime();
  while(t < in.EndTime()) {
    times.push_back(t);
    milestones.resize(milestones.size()+1);
    in.Eval(t,milestones.back());
    t += res;
  }
  times.push_back(in.times.back());
  milestones.push_back(in.milestones.back());
}

void Discretize(const MultiPath& in,Real res,vector<Real>& times,vector<Config>& milestones)
{
  times.resize(0);
  milestones.resize(0);
  times.reserve((int)Ceil(in.Duration()/res));
  milestones.reserve((int)Ceil(in.Duration()/res));
  Real t = 0;
  while(t < in.Duration()) {
    times.push_back(t);
    milestones.resize(milestones.size()+1);
    in.Evaluate(t,milestones.back());
    t += res;
  }
  times.push_back(in.Duration());
  milestones.push_back(in.sections.back().milestones.back());
}

void Discretize(const ParabolicRamp::DynamicPath& in,Real res,vector<Real>& times,vector<Config>& milestones)
{
  times.resize(0);
  milestones.resize(0);
  Real T=in.GetTotalTime();
  times.reserve((int)Ceil(T/res));
  milestones.reserve((int)Ceil(T/res));
  Real t = 0;
  ParabolicRamp::Vector x;
  while(t < T) {
    times.push_back(t);
    in.Evaluate(t,x);
    milestones.push_back(Vector(x));
    t += res;
  }
  times.push_back(T);
  milestones.push_back(Vector(in.ramps.back().x1));
}

void Discretize(const Spline::PiecewisePolynomialND& in,Real res,vector<Real>& times,vector<Config>& milestones)
{
  times.resize(0);
  milestones.resize(0);
  Real T=in.EndTime()-in.StartTime();
  times.reserve((int)Ceil(T/res));
  milestones.reserve((int)Ceil(T/res));
  Real t = in.StartTime();
  while(t < in.EndTime()) {
    times.push_back(t);
    milestones.push_back(Vector(in.Evaluate(t)));
    t += res;
  }
  times.push_back(in.EndTime());
  milestones.push_back(Vector(in.End()));  
}

///Split up the path into keyframes at a given resolution
void Discretize(const LinearPath& in,Real res,vector<Real>& times,vector<Config>& milestones,vector<Vector>& dmilestones)
{
  times.resize(0);
  milestones.resize(0);
  dmilestones.resize(0);
  Real T=in.Duration();
  times.reserve((int)Ceil(T/res));
  milestones.reserve((int)Ceil(T/res));
  dmilestones.reserve((int)Ceil(T/res));
  Real t = in.StartTime();
  while(t < in.EndTime()) {
    times.push_back(t);
    milestones.resize(milestones.size()+1);
    dmilestones.resize(dmilestones.size()+1);
    in.Eval(t,milestones.back());
    in.Deriv(t,dmilestones.back());
    t += res;
  }
  times.push_back(in.times.back());
  milestones.push_back(in.milestones.back());
  dmilestones.resize(dmilestones.size()+1);
  in.Deriv(T,dmilestones.back());
}

void Discretize(const MultiPath& in,Real res,vector<Real>& times,vector<Config>& milestones,vector<Vector>& dmilestones)
{
  times.resize(0);
  milestones.resize(0);
  dmilestones.resize(0);
  Real T=in.Duration();
  times.reserve((int)Ceil(T/res));
  milestones.reserve((int)Ceil(T/res));
  dmilestones.reserve((int)Ceil(T/res));
  Real t = 0;
  while(t < T) {
    times.push_back(t);
    milestones.resize(milestones.size()+1);
    dmilestones.resize(dmilestones.size()+1);
    in.Evaluate(t,milestones.back(),dmilestones.back());
    t += res;
  }
  times.push_back(T);
  milestones.resize(milestones.size()+1);  
  dmilestones.resize(dmilestones.size()+1);
  in.Evaluate(T,milestones.back(),dmilestones.back());
}

void Discretize(const ParabolicRamp::DynamicPath& in,Real res,vector<Real>& times,vector<Config>& milestones,vector<Vector>& dmilestones)
{
  times.resize(0);
  milestones.resize(0);
  dmilestones.resize(0);
  Real T=in.GetTotalTime();
  times.reserve((int)Ceil(T/res));
  milestones.reserve((int)Ceil(T/res));
  dmilestones.reserve((int)Ceil(T/res));
  Real t = 0;
  ParabolicRamp::Vector x;
  while(t < T) {
    times.push_back(t);
    in.Evaluate(t,x);
    milestones.push_back(Vector(x));
    in.Derivative(t,x);
    dmilestones.push_back(Vector(x));
    t += res;
  }
  times.push_back(T);
  milestones.push_back(Vector(in.ramps.back().x1));
  dmilestones.push_back(Vector(in.ramps.back().dx1));
}

void Discretize(const Spline::PiecewisePolynomialND& in,Real res,vector<Real>& times,vector<Config>& milestones,vector<Vector>& dmilestones)
{
  times.resize(0);
  milestones.resize(0);
  dmilestones.resize(0);
  Real T=in.EndTime()-in.StartTime();
  times.reserve((int)Ceil(T/res));
  milestones.reserve((int)Ceil(T/res));
  Real t = in.StartTime();
  while(t < in.EndTime()) {
    times.push_back(t);
    milestones.push_back(Vector(in.Evaluate(t)));
    t += res;
  }
  times.push_back(in.EndTime());
  milestones.push_back(Vector(in.End()));  
  dmilestones.push_back(Vector(in.Derivative(in.EndTime())));
}


void Discretize(RobotModel& robot,const LinearPath& in,Real res,vector<Real>& times,vector<Config>& milestones)
{
  times.resize(0);
  milestones.resize(0);
  Real T=in.Duration();
  times.reserve((int)Ceil(T/res));
  milestones.reserve((int)Ceil(T/res));
  Real t = in.StartTime();
  while(t < in.EndTime()) {
    times.push_back(t);
    milestones.resize(milestones.size()+1);
    in.Eval(robot,t,milestones.back());
    t += res;
  }
  times.push_back(in.times.back());
  milestones.push_back(in.milestones.back());
}


void Discretize(RobotModel& robot,const LinearPath& in,Real res,vector<Real>& times,vector<Config>& milestones,vector<Vector>& dmilestones)
{
  times.resize(0);
  milestones.resize(0);
  Real T=in.Duration();
  times.reserve((int)Ceil(T/res));
  milestones.reserve((int)Ceil(T/res));
  dmilestones.reserve((int)Ceil(T/res));
  Real t = in.StartTime();
  while(t < in.EndTime()) {
    times.push_back(t);
    milestones.resize(milestones.size()+1);
    dmilestones.resize(dmilestones.size()+1);
    in.Eval(robot,t,milestones.back());
    in.Deriv(robot,t,dmilestones.back());
    t += res;
  }
  times.push_back(in.times.back());
  milestones.push_back(in.milestones.back());
  dmilestones.resize(dmilestones.size()+1);
  in.Deriv(robot,T,dmilestones.back());
}

} //namespace Klampt