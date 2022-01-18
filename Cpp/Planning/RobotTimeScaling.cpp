#include "RobotTimeScaling.h"
#include "RobotConstrainedInterpolator.h"
#include "Modeling/SplineInterpolate.h"
#include "Modeling/Interpolate.h"
#include "TimeScaling.h"
#include "ConstrainedInterpolator.h"
#include <KrisLibrary/robotics/IKFunctions.h>
#include <KrisLibrary/Timer.h>
#include <sstream>
#include <fstream>

namespace Klampt {

#define DO_CHECK_BOUNDS 0
#define DO_SAVE_LIMITS 0
#define DO_SAVE_PLOT 0
#define DO_TEST_TRIANGULAR 0

#define SAVE_INTERPOLATING_CURVES 0
#define SAVE_LORES_INTERPOLATING_CURVES 0

#define SPLINE_INTERPOLATE_FUNC MonotonicInterpolate
//#define SPLINE_INTERPOLATE_FUNC SplineInterpolate

//f-tolerances are set to a scaling of the x-tolerances by this amount
const static Real gConstraintToleranceScale = 1e-2;
//const static Real gConstraintToleranceScale = 1e-3;

//time scaling grids are set to the spline resolution + this many bisections
const static int gNumTimescaleBisectIters = 1;

void RecursiveBisect(const GeneralizedCubicBezierCurve& c,Real duration,Real tol,GeneralizedCubicBezierSpline& output,int depth=0)
{
  if(c.OuterLength() <= tol) {
    output.segments.push_back(c);
    output.durations.push_back(duration);
    return;
  }

  GeneralizedCubicBezierCurve c1,c2;
  c.Bisect(c1,c2);
  RecursiveBisect(c1,duration*0.5,tol,output,depth+1);
  RecursiveBisect(c2,duration*0.5,tol,output,depth+1);
}

void DiscretizeSpline(GeneralizedCubicBezierSpline& path,Real xtol)
{
  GeneralizedCubicBezierSpline out;
  Real duniform = 1.0/path.segments.size();
  for(size_t i=0;i<path.segments.size();i++) {
    Real d = (path.durations.empty() ? duniform : path.durations[i]);
    RecursiveBisect(path.segments[i],d,xtol,out);
  }
  swap(path.segments,out.segments);
  swap(path.durations,out.durations);
}

bool CheckBounds(RobotModel& robot,const TimeScaledBezierCurve& traj,const vector<Real>& times)
{
  Vector v,a,maxv,maxa;
  Vector oldv,diffa,maxdiffa;
  bool res=true;
  for(size_t i=0;i<times.size();i++) {
    //double-checking velocity and acceleration bounds
    traj.Deriv(times[i],v);
    traj.Accel(times[i],a);
    if(i==0) { maxv=v; maxa=a; }
    else {
      for(int j=0;j<v.n;j++) {
	maxv[j] = Max(maxv[j],Abs(v[j]));
	maxa[j] = Max(maxa[j],Abs(a[j]));
      }
    }
    for(int j=0;j<v.n;j++) {
      if(Abs(v[j]) > robot.velMax[j]+1e-3) {
	printf("Exceeded vel max %s=%g at time %g\n",robot.LinkName(j).c_str(),v(j),times[i]);
	res = false;
      }
      if(Abs(a[j]) > robot.accMax[j]+1e-3) {
	printf("Exceeded accel max %s=%g at time %g\n",robot.LinkName(j).c_str(),a(j),times[i]);
	res = false; 
      }
    }
    if(!oldv.empty()) {
      diffa = (v-oldv)/(times[i]-times[i-1]);
      for(int j=0;j<v.n;j++) {
	if(Abs(diffa[j]) > robot.accMax[j]+1e-3) {
	  printf("Diff accel max %s=%g at time %g\n",robot.LinkName(j).c_str(),diffa(j),times[i]);
	  res = false;
	}
      }	    
    }
    oldv = v;
  }
  cout<<"Max vel "<<maxv<<endl;
  cout<<"Max accel "<<maxa<<endl;
  cout<<"End vel "<<v<<endl;
  cout<<"End accel "<<a<<endl;
  return res;
}


bool CheckBounds(RobotModel& robot,const TimeScaledBezierCurve& traj,Real dt)
{
  Real T=traj.EndTime();
  int numdivs = (int)Ceil(T/dt);
  vector<Real> times(numdivs);
  for(int i=0;i<numdivs;i++) 
    times[i] = T*Real(i)/Real(numdivs-1);  
  return CheckBounds(robot,traj,times);
}


void SaveLimits(RobotModel& robot,const TimeScaledBezierCurve& traj,Real dt,const char* fn)
{
  Real T=traj.EndTime();
  int numdivs = (int)Ceil(T/dt);
  printf("Saving time-scaled values to %s\n",fn);
  ofstream out(fn,ios::out);
  out<<"t";
  for(size_t i=0;i<robot.links.size();i++)
    out<<",q["<<robot.linkNames[i]<<"]";
  for(size_t i=0;i<robot.links.size();i++)
    out<<",v["<<robot.linkNames[i]<<"]";
  for(size_t i=0;i<robot.links.size();i++)
    out<<",a["<<robot.linkNames[i]<<"]";
  out<<",activeVLimit,activeAlimit,vsaturation,asaturation";
  out<<endl;

  Vector q,v,a,maxv,maxa;
  for(int i=0;i<=numdivs;i++) {
    //double-checking velocity and acceleration bounds
    Real t = Real(i)/Real(numdivs)*T;
    traj.Eval(t,q);
    traj.Deriv(t,v);
    traj.Accel(t,a);
    out<<t;
    for(int j=0;j<q.n;j++)
      out<<","<<q(j);
    for(int j=0;j<v.n;j++)
      out<<","<<v(j);
    for(int j=0;j<a.n;j++)
      out<<","<<a(j);
    Real maxVSat=0,maxASat=0;
    int maxVSatInd=0,maxASatInd=0;
    for(int j=0;j<v.n;j++)
      if(Abs(v(j))/robot.velMax(j) > maxVSat) {
	maxVSat = Abs(v(j))/robot.velMax(j);
	maxVSatInd = j;
      }
    for(int j=0;j<a.n;j++)
      if(Abs(a(j))/robot.accMax(j) > maxASat) {
	maxASat = Abs(a(j))/robot.accMax(j);
	maxASatInd = j;
      }
    out<<","<<robot.linkNames[maxVSatInd];
    out<<","<<robot.linkNames[maxASatInd];
    out<<","<<maxVSat<<","<<maxASat<<endl;
  }
  out<<endl;
  for(size_t i=0;i<traj.timeScaling.times.size();i++) {
    //double-checking velocity and acceleration bounds
    Real t = traj.timeScaling.times[i];
    traj.Eval(t,q);
    traj.Deriv(t,v);
    traj.Accel(t,a);
    out<<t;
    for(int j=0;j<q.n;j++)
      out<<","<<q(j);
    for(int j=0;j<v.n;j++)
      out<<","<<v(j);
    for(int j=0;j<a.n;j++)
      out<<","<<a(j);
    Real maxSat=0;
    int maxSatInd=0;
    bool maxSatA=false;
    for(int j=0;j<v.n;j++)
      if(Abs(v(j))/robot.velMax(j) > maxSat) {
	maxSat = Abs(v(j))/robot.velMax(j);
	maxSatInd = j;
      }
    for(int j=0;j<a.n;j++)
      if(Abs(a(j))/robot.accMax(j) > maxSat) {
	maxSat = Abs(a(j))/robot.accMax(j);
	maxSatInd = j;
	maxSatA=true;
      }
    if(maxSatA)
      out<<",a["<<robot.linkNames[maxSatInd]<<"]";
    else
      out<<",v["<<robot.linkNames[maxSatInd]<<"]";
    out<<","<<maxSat<<endl;
  }
}


bool TimeOptimizePath(RobotModel& robot,const vector<Real>& oldtimes,const vector<Config>& oldconfigs,Real dt,vector<Real>& newtimes,vector<Config>& newconfigs)
{
  //make a smooth interpolator
  Vector dx0,dx1,temp;
  RobotCSpace cspace(robot);
  TimeScaledBezierCurve traj;
  traj.path.durations.resize(oldconfigs.size()-1);
  for(size_t i=0;i+1<oldconfigs.size();i++) {
    traj.path.durations[i] = oldtimes[i+1]-oldtimes[i];
    if(!(traj.path.durations[i] > 0)) {
      fprintf(stderr,"TimeOptimizePath: input path does not have monotonically increasing time: segment %d range [%g,%g]\n",i,oldtimes[i],oldtimes[i+1]);
      return false;
    }
    Assert(traj.path.durations[i] > 0);
  }
  traj.path.segments.resize(oldconfigs.size()-1);
  for(size_t i=0;i+1<oldconfigs.size();i++) {
    traj.path.segments[i].space = &cspace;
    traj.path.segments[i].manifold = &cspace;
    traj.path.segments[i].x0 = oldconfigs[i];
    traj.path.segments[i].x3 = oldconfigs[i+1];
    if(i > 0) {
      InterpolateDerivative(robot,oldconfigs[i],oldconfigs[i+1],dx0);
      InterpolateDerivative(robot,oldconfigs[i],oldconfigs[i-1],temp);
      dx0 -= temp*(traj.path.durations[i]/traj.path.durations[i-1]);
      dx0 *= 0.5;
    }
    else dx0.resize(0);
    if(i+2 < oldconfigs.size()) {
      InterpolateDerivative(robot,oldconfigs[i+1],oldconfigs[i+2],dx1);
      InterpolateDerivative(robot,oldconfigs[i+1],oldconfigs[i],temp);
      dx1 *= traj.path.durations[i]/traj.path.durations[i+1];
      dx1 -= temp;
      dx1 *= 0.5;
    }
    else dx1.resize(0);
    traj.path.segments[i].SetNaturalTangents(dx0,dx1);
  }
  Timer timer;
  bool res=traj.OptimizeTimeScaling(robot.velMin,robot.velMax,-1.0*robot.accMax,robot.accMax);
  if(!res) {
    printf("Failed to optimize time scaling\n");
    return false;
  }
  else {
    printf("Optimized into a path with duration %g, (took %gs)\n",traj.EndTime(),timer.ElapsedTime());
  }
  double T = traj.EndTime();
  int numdivs = (int)Ceil(T/dt);

  printf("Discretizing at time resolution %g\n",T/numdivs);
  numdivs++;
  newtimes.resize(numdivs);
  newconfigs.resize(numdivs);

  for(int i=0;i<numdivs;i++) {
    newtimes[i] = T*Real(i)/Real(numdivs-1);
    traj.Eval(newtimes[i],newconfigs[i]);
  }
#if DO_CHECK_BOUNDS
  CheckBounds(robot,traj,newtimes);
#endif  //DO_CHECKBOUNDS
  return true;
}

bool InterpolateConstrainedPath(RobotModel& robot,const Config& a,const Config& b,const vector<IKGoal>& ikGoals,vector<Config>& milestones,Real xtol)
{
  RobotConstrainedInterpolator interp(robot,ikGoals);
  interp.ftol = xtol*gConstraintToleranceScale;
  interp.xtol = xtol;
  if(!interp.Make(a,b,milestones)) return false;
  return true;
  /*
  RobotIKFunction f(robot);
  for(size_t i=0;i<ikGoals.size();i++)
    f.UseIK(ikGoals[i]);
  GetDefaultIKDofs(robot,ikGoals,f.activeDofs);
  ActiveRobotCSpace space(robot,f.activeDofs);
  Config activeA(f.activeDofs.Size()),activeB(f.activeDofs.Size());
  f.activeDofs.InvMap(a,activeA);
  f.activeDofs.InvMap(b,activeB);
  
  ConstrainedInterpolator interp(&space,&f);
  interp.ftol = xtol*gConstraintToleranceScale;
  interp.xtol = xtol;
  if(!interp.Make(activeA,activeB,milestones)) {
    return false;
  }

  //lift milestones back into original space
  Vector q;
  for(size_t i=0;i<milestones.size();i++) {
    Real time = Real(i)/Real(milestones.size()-1); 
    Interpolate(robot,a,b,time,q);
    f.activeDofs.Map(milestones[i],q);
    milestones[i] = q;
  }
  return true;
  */
}

bool InterpolateConstrainedPath(RobotModel& robot,const vector<Config>& milestones,const vector<IKGoal>& ikGoals,vector<Config>& path,Real xtol)
{
  if(ikGoals.empty()) {
    path = milestones;
    return true;
  }
  RobotSmoothConstrainedInterpolator interp(robot,ikGoals);
  interp.ftol = xtol*gConstraintToleranceScale;
  interp.xtol = xtol;
  GeneralizedCubicBezierSpline spline;
  if(!MultiSmoothInterpolate(interp,milestones,spline)) return false;
  path.resize(spline.segments.size()+1);
  path[0] = spline.segments[0].x0;
  for(size_t i=0;i<spline.segments.size();i++)
    path[i+1] = spline.segments[i].x3;
  return true;
}

void SmoothDiscretizePath(RobotModel& robot,const vector<Config>& oldconfigs,int n,vector<Real>& times,vector<Config>& configs)
{
  times.resize(n);
  configs.resize(n);
  RobotCSpace space(robot);
  vector<GeneralizedCubicBezierCurve> curves;
  SPLINE_INTERPOLATE_FUNC(oldconfigs,curves,&space,&space);

  times[0] = 0;
  for(int i=1;i<n;i++) 
    times[i] = Real(i)/Real(n-1);
  configs[0] = oldconfigs[0];
  configs.back() = oldconfigs.back();
  Vector temp;
  for(int i=1;i+1<n;i++) {
    Real u0 = Floor(times[i]*curves.size());
    int segind = (int)u0;
    Real u=times[i]*curves.size()-u0;
    curves[segind].Eval(u,configs[i]);
  }
}


bool InterpolateConstrainedMultiPath(RobotModel& robot,const MultiPath& path,vector<GeneralizedCubicBezierSpline>& paths,Real xtol)
{
  //sanity check -- make sure it's a continuous path
  if(!path.IsContinuous()) {
    fprintf(stderr,"InterpolateConstrainedMultiPath: path is discontinuous\n");
    return false;
  }
  if(path.sections.empty()) {
    fprintf(stderr,"InterpolateConstrainedMultiPath: path is empty\n");
    return false;
  }

  if(path.settings.contains("resolution")) {
    //see if the resolution is high enough to just interpolate directly
    Real res=path.settings.as<Real>("resolution");
    if(res <= xtol) {
      printf("InterpolateConstrainedMultiPath: Direct interpolating trajectory with res %g\n",res);
      //just interpolate directly
      RobotCSpace space(robot);
      paths.resize(path.sections.size());
      for(size_t i=0;i<path.sections.size();i++) {
	if(path.sections[i].times.empty()) {
	  SPLINE_INTERPOLATE_FUNC(path.sections[i].milestones,paths[i].segments,
			       &space,&space);
	  //uniform timing
	  paths[i].durations.resize(paths[i].segments.size());
	  Real dt=1.0/Real(paths[i].segments.size());
	  for(size_t j=0;j<paths[i].segments.size();j++)
	    paths[i].durations[j] = dt;
	}
	else {
	  SPLINE_INTERPOLATE_FUNC(path.sections[i].milestones,path.sections[i].times,paths[i].segments,
			       &space,&space);
	  //get timing from path
	  paths[i].durations.resize(paths[i].segments.size());
	  for(size_t j=0;j<paths[i].segments.size();j++)
	    paths[i].durations[j] = path.sections[i].times[j+1]-path.sections[i].times[j];
	}
      }
      return true;
    }
  }
  printf("InterpolateConstrainedMultiPath: Discretizing constrained trajectory at res %g\n",xtol);

  RobotCSpace cspace(robot);

  //create transition constraints and derivatives
  vector<vector<IKGoal> > stanceConstraints(path.sections.size());
  vector<vector<IKGoal> > transitionConstraints(path.sections.size()-1);
  vector<Config> transitionDerivs(path.sections.size()-1);
  for(size_t i=0;i<path.sections.size();i++) 
    path.GetIKProblem(stanceConstraints[i],i);
  for(size_t i=0;i+1<path.sections.size();i++) {
    //put all nonredundant constraints into transitionConstraints[i]
    transitionConstraints[i]=stanceConstraints[i];
    for(size_t j=0;j<stanceConstraints[i+1].size();j++) {
      bool res=AddGoalNonredundant(stanceConstraints[i+1][j],transitionConstraints[i]);
      if(!res) {
	fprintf(stderr,"InterpolateConstrainedMultiPath: Conflict between goal %d of stance %d and stance %d\n",j,i+1,i);
	fprintf(stderr,"  Link %d\n",stanceConstraints[i+1][j].link);
	return false;
      }
    }

    const Config& prev=path.sections[i].milestones[path.sections[i].milestones.size()-2];
    const Config& next=path.sections[i+1].milestones[1];
    cspace.InterpolateDeriv(prev,next,0.5,transitionDerivs[i]);
    transitionDerivs[i] *= 0.5;

    //check for overshoots a la MonotonicInterpolate
    Vector inslope,outslope;
    cspace.InterpolateDeriv(prev,path.sections[i].milestones.back(),1.0,inslope);
    cspace.InterpolateDeriv(path.sections[i].milestones.back(),next,0.0,outslope);
    for(int j=0;j<transitionDerivs[i].n;j++) {
      if(Sign(transitionDerivs[i][j]) != Sign(inslope[j]) || Sign(transitionDerivs[i][j]) != Sign(outslope[j])) transitionDerivs[i][j] = 0;
      else {
	if(transitionDerivs[i][j] > 0) {
	  if(transitionDerivs[i][j] > 3.0*outslope[j])
	    transitionDerivs[i][j] = 3.0*outslope[j];
	  if(transitionDerivs[i][j] > 3.0*inslope[j])
	    transitionDerivs[i][j] = 3.0*inslope[j];
	}
	else {
	  if(transitionDerivs[i][j] < 3.0*outslope[j])
	    transitionDerivs[i][j] = 3.0*outslope[j];
	  if(transitionDerivs[i][j] < 3.0*inslope[j])
	    transitionDerivs[i][j] = 3.0*inslope[j];
	}
      }
    }

    //project "natural" derivative onto transition manifold
    RobotIKFunction f(robot);
    f.UseIK(transitionConstraints[i]);
    GetDefaultIKDofs(robot,transitionConstraints[i],f.activeDofs);
    Vector temp(f.activeDofs.Size()),dtemp(f.activeDofs.Size()),dtemp2;
    f.activeDofs.InvMap(path.sections[i].milestones.back(),temp);
    f.activeDofs.InvMap(transitionDerivs[i],dtemp);
    Matrix J;
    f.PreEval(temp);
    f.Jacobian(temp,J);
    RobustSVD<Real> svd;
    if(!svd.set(J)) {
      fprintf(stderr,"Unable to set SVD of transition constraints %d\n",i);
      return false;
    }
    svd.nullspaceComponent(dtemp,dtemp2);
    dtemp -= dtemp2;
    f.activeDofs.Map(dtemp,transitionDerivs[i]);
  }

  //start constructing path
  paths.resize(path.sections.size());   
  for(size_t i=0;i<path.sections.size();i++) {
    paths[i].segments.resize(0);
    paths[i].durations.resize(0);

    Vector dxprev,dxnext;
    if(i>0) 
      dxprev.setRef(transitionDerivs[i-1]); 
    if(i<transitionDerivs.size()) 
      dxnext.setRef(transitionDerivs[i]); 
    if(stanceConstraints[i].empty()) {
      SPLINE_INTERPOLATE_FUNC(path.sections[i].milestones,paths[i].segments,&cspace,&cspace);
      DiscretizeSpline(paths[i],xtol);

      //Note: discretizeSpline will fill in the spline durations
    }
    else {
      printf("Trying MultiSmoothInterpolate...\n");
      RobotSmoothConstrainedInterpolator interp(robot,stanceConstraints[i]);
      interp.ftol = xtol*gConstraintToleranceScale;
      interp.xtol = xtol;
      if(!MultiSmoothInterpolate(interp,path.sections[i].milestones,dxprev,dxnext,paths[i])) {
	/** TEMP - test no inter-section smoothing**/
	//if(!MultiSmoothInterpolate(interp,path.sections[i].milestones,paths[i])) {
	fprintf(stderr,"InterpolateConstrainedMultiPath: Unable to interpolate section %d\n",i);
	return false;
      }
    }
    //set the time scale if the input path is timed
    if(!path.sections[i].times.empty()) {
      //printf("Time scaling section %d to duration %g\n",i,path.sections[i].times.back()-path.sections[i].times.front());
      paths[i].TimeScale(path.sections[i].times.back()-path.sections[i].times.front());
    }
  }
  return true;
}


bool DiscretizeConstrainedMultiPath(RobotModel& robot,const MultiPath& path,MultiPath& out,Real xtol)
{
  if(path.settings.contains("resolution")) {
    //see if the resolution is high enough to just interpolate directly
    Real res=path.settings.as<Real>("resolution");
    if(res <= xtol) {
      out = path;
      return true;
    }
  }

  vector<GeneralizedCubicBezierSpline> paths;
  if(!InterpolateConstrainedMultiPath(robot,path,paths,xtol))
    return false;

  out = path;
  {
    out.settings.set("resolution",xtol);
    out.settings.set("program","DiscretizeConstrainedMultiPath");
  }
  Real tofs = (path.HasTiming() ? path.sections[0].times.front() : 0);
  for(size_t i=0;i<out.sections.size();i++) {
    out.sections[i].velocities.resize(0);
    paths[i].GetPiecewiseLinear(out.sections[i].times,out.sections[i].milestones);
    //shift section timing
    Real tscale = (path.HasTiming() ? path.sections[i].times.back()-path.sections[i].times.front() : 1.0) / out.sections[i].times.back();
    for(size_t j=0;j<out.sections[i].times.size();j++)
      out.sections[i].times[j] = tofs + out.sections[i].times[j]*tscale;
    tofs = out.sections[i].times.back();
  }
  return true;
}

Real OptimalTriangularTimeScaling(const GeneralizedCubicBezierSpline& path,const Vector& vmin,const Vector& vmax,const Vector& amin,const Vector& amax,TimeScaledBezierCurve& traj)
{
  traj.path = path;
  traj.pathSegments.resize(traj.path.durations.size()+1);
  traj.pathSegments[0] = 0;
  for(size_t i=0;i<traj.path.durations.size();i++)
    traj.pathSegments[i+1] = traj.pathSegments[i]+traj.path.durations[i];
  traj.timeScaling.params = traj.pathSegments;
  traj.timeScaling.times.resize(traj.timeScaling.params.size());
  traj.timeScaling.ds.resize(traj.timeScaling.params.size());

  //get T=1 triangular time scaling
  Real pathlen = path.TotalTime();
  Real invpathlen = 1.0/pathlen;
  Real u=0;
  for(size_t i=0;i<=path.segments.size();i++) {
    if(u < 0.5*pathlen) {
      traj.timeScaling.times[i] = Sqrt(0.5*u*invpathlen);
      traj.timeScaling.ds[i] = 4.0*pathlen*traj.timeScaling.times[i];
    }
    else {
      traj.timeScaling.times[i] = 1.0 - Sqrt(0.5 - 0.5*u*invpathlen);
      traj.timeScaling.ds[i] = 4.0*pathlen*(1.0 - traj.timeScaling.times[i]);
    }
    //printf("u: %g / %g, t: %g / 1\n",u,pathlen,traj.timeScaling.times[i]);
    if(i < path.segments.size()) {
      Real du = path.durations[i];
      u += du;
    }
  }

  Real vmaxrel = 0, amaxrel = 0;
  for(size_t i=0;i<path.segments.size();i++) {
    Real du = path.durations[i];
    Vector vimin,vimax,aimin,aimax;
    path.segments[i].GetDerivBounds(vimin,vimax,aimin,aimax);
    vimin /= du;
    vimax /= du;
    aimin /= Sqr(du);
    aimax /= Sqr(du);

    //get the max height of velocity profile, and speed at this point
    Assert(traj.timeScaling.times[i+1] > traj.timeScaling.times[i]);
    Real h = Max(traj.timeScaling.ds[i],traj.timeScaling.ds[i+1]);
    Real dds = (Sqr(traj.timeScaling.ds[i+1])-Sqr(traj.timeScaling.ds[i]))/(2.0*path.durations[i]);
    for(int j=0;j<vmin.n;j++) {
      if (vimin[j]*h < vmaxrel*vmin[j])
	vmaxrel = vimin[j]*h / vmin[j];
      if (vimax[j]*h > vmaxrel*vmax[j])
	vmaxrel = vimax[j]*h / vmax[j];
    }
    if(u <= 0.5*pathlen) {
      //positive acceleration
      for(int j=0;j<vmin.n;j++) {
	if((aimin[j]*Sqr(h) + vimin[j]*dds) < amaxrel*amin[j]) 
	  amaxrel = (aimin[j]*Sqr(h) + vimin[j]*dds)/amin[j];
	if((aimax[j]*Sqr(h) + vimax[j]*dds) > amaxrel*amax[j]) 
	  amaxrel = (aimax[j]*Sqr(h) + vimax[j]*dds)/amax[j];
      }
    }
    if(u+du >= 0.5*pathlen) {
      //negative acceleration
      for(int j=0;j<vmin.n;j++) {
	if((aimin[j]*Sqr(h) - vimin[j]*dds) < amaxrel*amin[j]) 
	  amaxrel = (aimin[j]*Sqr(h) - vimin[j]*dds)/amin[j];
	if((aimax[j]*Sqr(h) - vimax[j]*dds) > amaxrel*amax[j]) 
	  amaxrel = (aimax[j]*Sqr(h) - vimax[j]*dds)/amax[j];

      }
    }
    u += du;
  }
  //can set rate to 1.0/Max(vmaxrel,amaxrel)
  Real maxRate = Max(vmaxrel,amaxrel);
  //duration of ramp up is 2*pathlen/2 / rate, ramp down is the same
  Real T = 2.0*Sqrt(maxRate/4.0);
  printf("Pathlen %g, time %g, max relative velocity %g, acceleration %g\n",pathlen,T,vmaxrel,amaxrel);

  for(size_t i=0;i<=path.durations.size();i++) 
    traj.timeScaling.ds[i] *= 1.0/Sqrt(maxRate);

  for(size_t i=0;i<path.durations.size();i++) {
    Real dt = 2*path.durations[i]/(traj.timeScaling.ds[i]+traj.timeScaling.ds[i+1]);    
    traj.timeScaling.times[i+1]=traj.timeScaling.times[i]+dt;
  }
  printf("Recalculated time %g\n",traj.timeScaling.times.back());
  return T;
}

bool GenerateAndTimeOptimizeMultiPath(RobotModel& robot,MultiPath& multipath,Real xtol,Real dt)
{
  Timer timer;
  vector<GeneralizedCubicBezierSpline > paths;
  if(!InterpolateConstrainedMultiPath(robot,multipath,paths,xtol))
    return false;
  printf("Generated interpolating path in time %gs\n",timer.ElapsedTime());

  RobotCSpace cspace(robot);
  for(size_t i=0;i<multipath.sections.size();i++) {
    for(size_t j=0;j<paths[i].segments.size();j++) {
      paths[i].segments[j].space = &cspace;
      paths[i].segments[j].manifold = &cspace;
    }
    for(int iters=0;iters<gNumTimescaleBisectIters;iters++)
      paths[i].Bisect();
  }

#if SAVE_INTERPOLATING_CURVES
  int index=0;
  printf("Saving sections, element %d to section_x_bezier.csv\n",index);
  for(size_t i=0;i<paths.size();i++) {
    {
      stringstream ss;
      ss<<"section_"<<i<<"_bezier.csv";
      ofstream out(ss.str().c_str(),ios::out);
      out<<"duration,x0,x1,x2,x3"<<endl;
      for(size_t j=0;j<paths[i].segments.size();j++) {
	out<<paths[i].durations[j]<<","<<paths[i].segments[j].x0[index]<<","<<paths[i].segments[j].x1[index]<<","<<paths[i].segments[j].x2[index]<<","<<paths[i].segments[j].x3[index]<<endl;
      }
      out.close();
    }
    {
      stringstream ss;
      ss<<"section_"<<i<<"_bezier_vel.csv";
      ofstream out(ss.str().c_str(),ios::out);
      out<<"v(0),v(0.5),v(1)"<<endl;
      Vector temp;
      for(size_t j=0;j<paths[i].segments.size();j++) {
	paths[i].segments[j].Deriv(0,temp);
	temp /= paths[i].durations[j];
	out<<temp[index];
	paths[i].segments[j].Deriv(0.5,temp);
	temp /= paths[i].durations[j];
	out<<","<<temp[index];
	paths[i].segments[j].Deriv(1,temp);
	temp /= paths[i].durations[j];
	out<<","<<temp[index];
	out<<endl;
      }
      out.close();
    }
    {
      stringstream ss;
      ss<<"section_"<<i<<"_bezier_acc.csv";
      ofstream out(ss.str().c_str(),ios::out);
      out<<"a(0),a(1)"<<endl;
      Vector temp;
      for(size_t j=0;j<paths[i].segments.size();j++) {
	paths[i].segments[j].Accel(0,temp);
	temp /= Sqr(paths[i].durations[j]);
	out<<temp[index];
	paths[i].segments[j].Accel(1,temp);
	temp /= Sqr(paths[i].durations[j]);
	out<<","<<temp[index];
	out<<endl;
      }
      out.close();
    }
  }
#endif //SAVE_INTERPOLATING_CURVES
#if SAVE_LORES_INTERPOLATING_CURVES
  paths.clear();
  if(!InterpolateConstrainedMultiPath(robot,multipath,paths,xtol*2.0))
    return false;
  for(size_t i=0;i<multipath.sections.size();i++) {
    for(size_t j=0;j<paths[i].segments.size();j++) {
      paths[i].segments[j].space = &cspace;
      paths[i].segments[j].manifold = &cspace;
    }
  }
  for(size_t i=0;i<paths.size();i++) {
    {
      stringstream ss;
      ss<<"section_"<<i<<"_bezier_x2.csv";
      ofstream out(ss.str().c_str(),ios::out);
      out<<"duration,x0,x1,x2,x3"<<endl;
      for(size_t j=0;j<paths[i].segments.size();j++) {
	out<<paths[i].durations[j]<<","<<paths[i].segments[j].x0[index]<<","<<paths[i].segments[j].x1[index]<<","<<paths[i].segments[j].x2[index]<<","<<paths[i].segments[j].x3[index]<<endl;
      }
      out.close();
    }
    {
      stringstream ss;
      ss<<"section_"<<i<<"_bezier_vel_x2.csv";
      ofstream out(ss.str().c_str(),ios::out);
      out<<"v(0),v(0.5),v(1)"<<endl;
      Vector temp;
      for(size_t j=0;j<paths[i].segments.size();j++) {
	paths[i].segments[j].Deriv(0,temp);
	temp /= paths[i].durations[j];
	out<<temp[index];
	paths[i].segments[j].Deriv(0.5,temp);
	temp /= paths[i].durations[j];
	out<<","<<temp[index];
	paths[i].segments[j].Deriv(1,temp);
	temp /= paths[i].durations[j];
	out<<","<<temp[index];
	out<<endl;
      }
      out.close();
    }
    {
      stringstream ss;
      ss<<"section_"<<i<<"_bezier_acc_x2.csv";
      ofstream out(ss.str().c_str(),ios::out);
      out<<"a(0),a(1)"<<endl;
      Vector temp;
      for(size_t j=0;j<paths[i].segments.size();j++) {
	paths[i].segments[j].Accel(0,temp);
	temp /= Sqr(paths[i].durations[j]);
	out<<temp[index];
	paths[i].segments[j].Accel(1,temp);
	temp /= Sqr(paths[i].durations[j]);
	out<<","<<temp[index];
	out<<endl;
      }
      out.close();
    }
  }

  paths.clear();
  if(!InterpolateConstrainedMultiPath(robot,multipath,paths,xtol*4.0))
    return false;
  for(size_t i=0;i<multipath.sections.size();i++) {
    for(size_t j=0;j<paths[i].segments.size();j++) {
      paths[i].segments[j].space = &cspace;
      paths[i].segments[j].manifold = &cspace;
    }
  }
  for(size_t i=0;i<paths.size();i++) {
    {
      stringstream ss;
      ss<<"section_"<<i<<"_bezier_x4.csv";
      ofstream out(ss.str().c_str(),ios::out);
      out<<"duration,x0,x1,x2,x3"<<endl;
      for(size_t j=0;j<paths[i].segments.size();j++) {
	out<<paths[i].durations[j]<<","<<paths[i].segments[j].x0[index]<<","<<paths[i].segments[j].x1[index]<<","<<paths[i].segments[j].x2[index]<<","<<paths[i].segments[j].x3[index]<<endl;
      }
      out.close();
    }
    {
      stringstream ss;
      ss<<"section_"<<i<<"_bezier_vel_x4.csv";
      ofstream out(ss.str().c_str(),ios::out);
      out<<"v(0),v(0.5),v(1)"<<endl;
      Vector temp;
      for(size_t j=0;j<paths[i].segments.size();j++) {
	paths[i].segments[j].Deriv(0,temp);
	temp /= paths[i].durations[j];
	out<<temp[index];
	paths[i].segments[j].Deriv(0.5,temp);
	temp /= paths[i].durations[j];
	out<<","<<temp[index];
	paths[i].segments[j].Deriv(1,temp);
	temp /= paths[i].durations[j];
	out<<","<<temp[index];
	out<<endl;
      }
      out.close();
    }
    {
      stringstream ss;
      ss<<"section_"<<i<<"_bezier_acc_x4.csv";
      ofstream out(ss.str().c_str(),ios::out);
      out<<"a(0),a(1)"<<endl;
      Vector temp;
      for(size_t j=0;j<paths[i].segments.size();j++) {
	paths[i].segments[j].Accel(0,temp);
	temp /= Sqr(paths[i].durations[j]);
	out<<temp[index];
	paths[i].segments[j].Accel(1,temp);
	temp /= Sqr(paths[i].durations[j]);
	out<<","<<temp[index];
	out<<endl;
      }
      out.close();
    }
  }

  paths.clear();
  if(!InterpolateConstrainedMultiPath(robot,multipath,paths,xtol*8.0))
    return false;
  for(size_t i=0;i<multipath.sections.size();i++) {
    for(size_t j=0;j<paths[i].segments.size();j++) {
      paths[i].segments[j].space = &cspace;
      paths[i].segments[j].manifold = &cspace;
    }
  }
  for(size_t i=0;i<paths.size();i++) {
    {
      stringstream ss;
      ss<<"section_"<<i<<"_bezier_x8.csv";
      ofstream out(ss.str().c_str(),ios::out);
      out<<"duration,x0,x1,x2,x3"<<endl;
      for(size_t j=0;j<paths[i].segments.size();j++) {
	out<<paths[i].durations[j]<<","<<paths[i].segments[j].x0[index]<<","<<paths[i].segments[j].x1[index]<<","<<paths[i].segments[j].x2[index]<<","<<paths[i].segments[j].x3[index]<<endl;
      }
      out.close();
    }
    {
      stringstream ss;
      ss<<"section_"<<i<<"_bezier_vel_x8.csv";
      ofstream out(ss.str().c_str(),ios::out);
      out<<"v(0),v(0.5),v(1)"<<endl;
      Vector temp;
      for(size_t j=0;j<paths[i].segments.size();j++) {
	paths[i].segments[j].Deriv(0,temp);
	temp /= paths[i].durations[j];
	out<<temp[index];
	paths[i].segments[j].Deriv(0.5,temp);
	temp /= paths[i].durations[j];
	out<<","<<temp[index];
	paths[i].segments[j].Deriv(1,temp);
	temp /= paths[i].durations[j];
	out<<","<<temp[index];
	out<<endl;
      }
      out.close();
    }
    {
      stringstream ss;
      ss<<"section_"<<i<<"_bezier_acc_x8.csv";
      ofstream out(ss.str().c_str(),ios::out);
      out<<"a(0),a(1)"<<endl;
      Vector temp;
      for(size_t j=0;j<paths[i].segments.size();j++) {
	paths[i].segments[j].Accel(0,temp);
	temp /= Sqr(paths[i].durations[j]);
	out<<temp[index];
	paths[i].segments[j].Accel(1,temp);
	temp /= Sqr(paths[i].durations[j]);
	out<<","<<temp[index];
	out<<endl;
      }
      out.close();
    }
  }


#endif //SAVE_LORES_INTERPOLATING_CURVES

  //concatenate sections into a single curve
  TimeScaledBezierCurve traj;
  vector<int> edgeToSection,sectionEdges(1,0);
  for(size_t i=0;i<multipath.sections.size();i++) {
    traj.path.Concat(paths[i]);
    for(size_t j=0;j<paths[i].segments.size();j++)
      edgeToSection.push_back((int)i);
    sectionEdges.push_back(sectionEdges.back()+(int)paths[i].segments.size());
  }

  timer.Reset();
  bool res=traj.OptimizeTimeScaling(robot.velMin,robot.velMax,-1.0*robot.accMax,robot.accMax);
  if(!res) {
    printf("Failed to optimize time scaling\n");
    return false;
  }
  else {
    printf("Optimized into a path with duration %g, (took %gs)\n",traj.EndTime(),timer.ElapsedTime());
  }
  double T = traj.EndTime();
  int numdivs = (int)Ceil(T/dt);
  printf("Discretizing at time resolution %g\n",T/numdivs);
  numdivs++;

  Vector x,v;
  int sCur = -1;
  for(int i=0;i<numdivs;i++) {
    Real t=T*Real(i)/Real(numdivs-1);
    int trajEdge = traj.timeScaling.TimeToSegment(t);
    if(trajEdge == (int)edgeToSection.size()) trajEdge--; //end of path
    Assert(trajEdge < (int)edgeToSection.size());
    int s=edgeToSection[trajEdge];
    if(s < sCur) {
      fprintf(stderr,"Strange: edge index is going backward? %d -> %d\n",sCur,s);
      fprintf(stderr,"  time %g, division %d, traj segment %d\n",t,i,trajEdge);
    }
    Assert(s - sCur >=0);
    while(sCur < s) {
      //close off the current section and add a new one
      Real switchTime=traj.timeScaling.times[sectionEdges[sCur+1]];
      Assert(switchTime <= t);
      traj.Eval(switchTime,x);
      traj.Deriv(switchTime,v);
      if(sCur >= 0) {
	multipath.sections[sCur].times.push_back(switchTime);
	multipath.sections[sCur].milestones.push_back(x);
	multipath.sections[sCur].velocities.push_back(v);
      }
      multipath.sections[sCur+1].milestones.resize(0);
      multipath.sections[sCur+1].velocities.resize(0);
      multipath.sections[sCur+1].times.resize(0);
      multipath.sections[sCur+1].milestones.push_back(x);
      multipath.sections[sCur+1].velocities.push_back(v);
      multipath.sections[sCur+1].times.push_back(switchTime);
      sCur++;
    }
    if(t == multipath.sections[s].times.back()) continue;
    traj.Eval(t,x);
    traj.Deriv(t,v);
    multipath.sections[s].times.push_back(t);
    multipath.sections[s].milestones.push_back(x);
    multipath.sections[s].velocities.push_back(v);
  }

#if DO_TEST_TRIANGULAR
  timer.Reset();
  TimeScaledBezierCurve trajTri;
  Real Ttrap = 0;
  printf("%d paths?\n",paths.size());
  for(size_t i=0;i<paths.size();i++)
    Ttrap += OptimalTriangularTimeScaling(paths[i],robot.velMin,robot.velMax,-1.0*robot.accMax,robot.accMax,trajTri);
  printf("Optimal trapezoidal time scaling duration %g, calculated in %gs\n",Ttrap,timer.ElapsedTime());
  printf("Saving plot to opt_tri_multipath.csv\n");
  trajTri.Plot("opt_tri_multipath.csv",robot.velMin,robot.velMax,-1.0*robot.accMax,robot.accMax);
#endif // DO_TEST_TRAPEZOIDAL


#if DO_CHECK_BOUNDS  
  CheckBounds(robot,traj,dt);
#endif // DO_CHECK_BOUNDS

#if DO_SAVE_PLOT
  printf("Saving plot to opt_multipath.csv\n");
  traj.Plot("opt_multipath.csv",robot.velMin,robot.velMax,-1.0*robot.accMax,robot.accMax);
#endif //DO_SAVE_PLOT

#if DO_SAVE_LIMITS
  SaveLimits(robot,traj,dt,"opt_multipath_limits.csv");
#endif // DO_SAVE_LIMITS

  {
    multipath.settings.set("resolution",xtol);
    multipath.settings.set("program","GenerateAndTimeOptimizeMultiPath");
  }
  return true;
}


void EvaluateMultiPath(RobotModel& robot,const MultiPath& path,Real t,Config& q,Real xtol,Real contactol,int numIKIters)
{
  RobotCSpace space(robot);
  GeneralizedCubicBezierCurve curve(&space,&space);
  Real duration,param;
  int seg=path.Evaluate(t,curve,duration,param,MultiPath::InterpLinear);
  if(seg < 0) seg = 0;
  if(seg >= (int)path.sections.size()) seg = (int)path.sections.size()-1;
  curve.Eval(param,q);

  //solve for constraints
  bool solveIK = false;
  if(!path.settings.contains("resolution"))
    solveIK = true;
  else {
    Real res = path.settings.as<Real>("resolution");
    if(res > xtol) solveIK=true;
  }
  if(solveIK) {
    vector<IKGoal> ik;
    path.GetIKProblem(ik,seg);
    if(!ik.empty()) {
      swap(q,robot.q);
      robot.UpdateFrames();

      int iters=numIKIters;
      bool res=SolveIK(robot,ik,contactol,iters,0);
      if(!res) printf("Warning, couldn't solve IK problem at sec %d, time %g\n",seg,t);
      swap(q,robot.q);
    }
  }
}

} //namespace Klampt 