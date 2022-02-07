#include "motorcalibrate.h"
#include "Modeling/Resources.h"
#include "Modeling/Robot.h"
#include "Modeling/Interpolate.h"
#include <KrisLibrary/robotics/ConstrainedDynamics.h>
#include <KrisLibrary/robotics/NewtonEuler.h>
#include <KrisLibrary/math/differentiation.h>
#include <KrisLibrary/optimization/Minimization.h>
#include <KrisLibrary/utils/AnyCollection.h>
#include <KrisLibrary/utils/stringutils.h>
#include <string.h>
#include <fstream>
using namespace std;
using namespace Math;
using namespace Klampt;

/** @brief Simulates a single DOF under PD control and stick-slip friction.
 *
 * Given a linearized model
 *  x'' = minv * (t + f* - d*x' + k*x + c)
 * with PD torques t = kp(xdes-x)-kd(x'des-x')
 * and frictional forces f* = min_f |x''| s.t. |f| <= mu_d + |x'|*mu_v
 * returns the ending x(T),x'(T)
 */

//TODO create error dialogs for error prompts

void SimulateDOF(Real minv,Real d,Real k,Real c,
		 Real x0,Real dx0,
		 Real xDes,Real dxDes,
		 Real kP,Real kD,Real Iterm,Real dryFriction,Real viscousFriction,
		 Real T,Real timestep,Real torquemin,Real torquemax,
		 Real& xT,Real& dxT)
{
  xT=x0;
  dxT=dx0;
  //printf("desired (%g,%g)\n",xDes,dxDes);
  //printf("(%g,%g) ",xT,dxT);
  Real t=0;
  while(t < T) {
    //in absence of frictional forces, this is the acceleration
    Real tdes = kP*(xDes-xT)+kD*(dxDes-dxT)+Iterm;
    Real ddx0 = (Clamp(tdes,torquemin,torquemax)-d*dxT - k*xT - c)*minv;
    Real kf = dryFriction + Abs(dxT)*viscousFriction;
    Real ddx = ddx0;
    Real dt = Min(timestep,T-t);
    if(kf > 0) {
      //x'' = ddx0 + f / m;
      //x'[t+1] = x' + dt * x''
      //try to set x'[t+1] to zero
      //(-x'/dt-ddx0)*m = f
      Real f = (-dxT/dt-ddx0) / minv;
      if(Abs(f) > kf) {  //slip friction
	f = -Sign(dxT) * kf;
	ddx += f*minv;
      }
      else { // stick friction
	ddx = -dxT / dt;
      }
    }
    if(t + timestep > T) {
      xT += dxT*dt + 0.5*ddx*Sqr(dt);
      dxT += ddx*dt;
      //printf("(%g,%g) ",xT,dxT);
      //printf("\n");
      //getchar();
      return;
    }
    xT += dxT*dt + 0.5*ddx*Sqr(dt);
    dxT += ddx*dt;
    t += dt;
    //printf("(%g,%g) ",xT,dxT);
  }
}

///Rolls out a whole simulation trace
void SimulateDOF(const vector<Real>& minvs,const vector<Real>& ds,const vector<Real>& ks,const vector<Real>& cs,
		 Real x0,Real dx0,
		 const vector<Real>& xDes,const vector<Real>& dxDes,
		 Real kP,Real kD,Real Iterm,Real dryFriction,Real viscousFriction,
		 
		 const vector<Real>& durations,Real timestep,Real torquemin,Real torquemax,
		 vector<Real>& xT,vector<Real>& dxT)
{
  Assert(minvs.size()==ds.size());
  Assert(minvs.size()==ks.size());
  Assert(minvs.size()==cs.size());
  Assert(minvs.size()==xDes.size());
  Assert(minvs.size()==dxDes.size());
  Assert(minvs.size()==durations.size());
  xT.resize(durations.size()+1);
  dxT.resize(durations.size()+1);
  xT[0] = x0;
  dxT[0] = dx0;
  for(size_t i=0;i<durations.size();i++) {
    ::SimulateDOF(minvs[i],ds[i],ks[i],cs[i],
		  xT[i],dxT[i],xDes[i],dxDes[i],
		  kP,kD,Iterm,dryFriction,viscousFriction,
		  durations[i],timestep,torquemin,torquemax,
		  xT[i+1],dxT[i+1]);
  }
}

class Simulate1DOFFunc : public VectorFieldFunction
{
public:
  Real minv,d,k,c,I,x0,dx0,xDes,dxDes,T,torquemin,torquemax;
  Simulate1DOFFunc(Real _minv,Real _d,Real _k,Real _c,
		   Real _I,Real _x0,Real _dx0,
		   Real _xDes,Real _dxDes,
		   Real _T,Real _torquemin,Real _torquemax)
    :minv(_minv),d(_d),k(_k),c(_c),I(_I),x0(_x0),dx0(_dx0),xDes(_xDes),dxDes(_dxDes),T(_T),torquemin(_torquemin),torquemax(_torquemax)
  {}
  virtual int NumDimensions() { return 2; }
  virtual void Eval(const Vector& params,Vector& out) {
    Assert(params.n == 5);
    Real kP = params(0);
    Real kI = params(1);
    Real kD = params(2);
    Real mu_d = params(3);
    Real mu_v = params(4);
    Real xT,dxT;
    SimulateDOF(minv,d,k,c,x0,dx0,
		xDes,dxDes,
		kP,kD,kI*I,mu_d,mu_v,
		T,gDefaultTimestep,torquemin,torquemax,
		xT,dxT);
    out.resize(2);
    out(0) = xT;
    out(1) = dxT;
  }
};

class OptimizeDofFunction : public ScalarFieldFunction
{
public:
  const vector<Real> &minvs,&ds,&ks,&cs,&x0s,&dx0s,&x1s,&dx1s,&xDes,&dxDes,&dts;
  vector<shared_ptr<Simulate1DOFFunc> > fs;

  OptimizeDofFunction(const vector<Real>& _minvs,const vector<Real>& _ds,const vector<Real>& _ks,const vector<Real>& _cs,
		      const vector<Real>& _x0s,const vector<Real>& _dx0s,
		      const vector<Real>& _x1s,const vector<Real>& _dx1s,
		      const vector<Real>& _xDes,const vector<Real>& _dxDes,
		      const vector<Real>& _dts,Real torquemin,Real torquemax)
    :minvs(_minvs),ds(_ds),ks(_ks),cs(_cs),x0s(_x0s),dx0s(_dx0s),x1s(_x1s),dx1s(_dx1s),xDes(_xDes),dxDes(_dxDes),dts(_dts)
  {
    fs.resize(minvs.size());
    for(size_t i=0;i<minvs.size();i++) {
      fs[i] = make_shared<Simulate1DOFFunc>(minvs[i],ds[i],ks[i],cs[i],
				   0,x0s[i],dx0s[i],
				   xDes[i],dxDes[i],
				   dts[i],torquemin,torquemax);
    }
  }
  void PreEval(const Vector& params)
  {
    for(size_t i=0;i<fs.size();i++) 
      fs[i]->PreEval(params);
  }
  Real Eval(const Vector& params)
  {
    Real err = 0;
    Vector fout;
    Real I=0;
    for(size_t i=0;i<fs.size();i++) {
      //printf("Result %g,%g\n",x1s[i],dx1s[i]);
      if(i > 0) {
	if(x0s[i] == x1s[i-1]) {
	  //continuity
	  //start from end of prior simulation
	  fs[i]->x0 = fout[0]; 
	  fs[i]->dx0 = fout[1]; 
	  fs[i]->I = I;
	}
	else {
	  fs[i]->I = 0;
	}
      }
      fs[i]->Eval(params,fout);
      err += Sqr(fout[0]-x1s[i]) + Sqr(fout[1]-dx1s[i])*gDefaultVelocityWeight;
      I += (xDes[i]-fout[0])*dts[i];
    }
    if(!IsFinite(err)) return Inf;
    return Sqrt(err/fs.size());
  }
  void Gradient(const Vector& params,Vector& grad)
  {
    Vector temp=params;
    Vector h(5);
    h(0) = Max(1e-2*params(0),1e-2);
    h(1) = Max(1e-2*params(1),1e-2);
    h(2) = Max(1e-2*params(2),1e-2);
    h(3) = 1e-2;
    h(4) = 1e-2;
    grad.resize(5);
    GradientCenteredDifference(*this,temp,h,grad);
    for(int i=0;i<grad.n;i++)
      if(!IsFinite(grad[i])) {
	printf("Warning, instability at parameter %d = %g\n",i,params(i));
	grad[i] = 0;
      }
    //cout<<"Parameter gradient"<<grad<<endl;
  }
};

//For a bunch of linearized single-dof models
//minvs[i]^-1 *x'' + ds[i]*x' + ks[i]*x + cs[i] = t + f*
//with t = kP*(xDes[i]-x) + kD(dxDes[i]-x') + kI*int(xDes[i]-x)
//and f given by dry and viscous friction terms
//optimizes the kP, kD, kI, and friction terms using descent over numIters iters
Real OptimizeDof(const vector<Real>& minvs,const vector<Real>& ds,const vector<Real>& ks,const vector<Real>& cs,
		 const vector<Real>& x0s,const vector<Real>& dx0s,
		 const vector<Real>& x1s,const vector<Real>& dx1s,
		 const vector<Real>& xDes,const vector<Real>& dxDes,
		 const vector<Real>& dts,Real torquemin,Real torquemax,
		 Real& kP,Real& kI,Real& kD,Real& dryFriction,Real& viscousFriction,
		 int numIters)
{
  OptimizeDofFunction f(minvs,ds,ks,cs,
			x0s,dx0s,
			x1s,dx1s,
			xDes,dxDes,
dts,torquemin,torquemax);  
  Optimization::BCMinimizationProblem  minProblem(&f);
  minProblem.bmin.resize(5,Zero);
  minProblem.bmax.resize(5,Inf);
  minProblem.tolgrad = 1e-9;
  minProblem.tolf = 1e-9;
  minProblem.tolx = 1e-9;
  minProblem.x.resize(5);
  minProblem.x(0) = kP;
  minProblem.x(1) = kI;
  minProblem.x(2) = kD;
  minProblem.x(3) = dryFriction;
  minProblem.x(4) = viscousFriction;
  Real fx = f(minProblem.x);
  cout<<"Initial RMSE "<<fx<<endl;
  bool paused = false;
  if(!IsFinite(fx) || fx > 1e2) {
    cout<<"Initial instability? "<<endl;
    printf("%g, %g: x'' = %g*(PID(%g,%g) - %g*x' + %g*x + %g)\n",x0s[0],dx0s[0],minvs[0],xDes[0],dxDes[0],ds[0],ks[0],cs[0]);
    Vector res;
    for(size_t i=0;i<f.fs.size();i++) { 
      (*f.fs[i])(minProblem.x,res);
      printf("%g, %g: x'' = %g*(PID(%g,%g) - %g*x' + %g*x + %g)\n",res[0],res[1],minvs[i],xDes[i],dxDes[i],ds[i],ks[i],cs[i]);
      if(gErrorGetchar && (i+1)%100 == 0) getchar();
      if(!IsFinite(res[0]) || !IsFinite(res[1]) || Abs(res[1]) > 1e2) {
	printf("Large error on step %d, this may require tuning initial parameters\n",i);
	if(gErrorGetchar) {
	  printf("Press enter to continue\n");
	  getchar();
	  paused = true;
	}
	break;
      }
    }
  }
  int maxIters = numIters;
  ConvergenceResult res = minProblem.SolveSD(maxIters);
  fx=f(minProblem.x);
  cout<<"SD result: "<<res<<" after "<<maxIters<<" iters, RMSE "<<fx<<endl;
  if(paused) {
    printf("Press enter to continue\n");
    getchar();
  }
  kP = minProblem.x(0);
  kI = minProblem.x(1);
  kD = minProblem.x(2);
  dryFriction = minProblem.x(3);
  viscousFriction = minProblem.x(4);
  return fx;
}

Real GOptimizeDof(const vector<Real>& minvs,const vector<Real>& ds,const vector<Real>& ks,const vector<Real>& cs,
		 const vector<Real>& x0s,const vector<Real>& dx0s,
		 const vector<Real>& x1s,const vector<Real>& dx1s,
		 const vector<Real>& xDes,const vector<Real>& dxDes,
		 const vector<Real>& dts,Real torquemin,Real torquemax,
		 Real& kP,Real& kI,Real& kD,Real& dryFriction,Real& viscousFriction,
		 int numIters)
{
  Real f0 = OptimizeDof(minvs,ds,ks,cs,x0s,dx0s,x1s,dx1s,xDes,dxDes,dts,torquemin,torquemax,kP,kI,kD,dryFriction,viscousFriction,numIters);
  //try big changes
  Vector params(5);
  params[0] = kP;
  params[1] = kI;
  params[2] = kD;
  params[3] = dryFriction;
  params[4] = viscousFriction;
  Vector best = params;
  Vector bound = params*2.0;
  for(int i=0;i<params.n;i++) 
    if(bound[i] < 1) bound[i] = 1;
  for(int iter=0;iter<numIters/100;iter++) {
    for(int i=0;i<params.n;i++) 
      params(i) = Rand(0,bound[i]);
    Real f = OptimizeDof(minvs,ds,ks,cs,x0s,dx0s,x1s,dx1s,xDes,dxDes,dts,torquemin,torquemax,params[0],params[1],params[2],params[3],params[4],numIters);
    if(f < f0) {
      cout<<"Got a better solution with a hop, RMSE "<<f<<endl;
      best = params;
      bound = params*2.0;
      for(int i=0;i<params.n;i++) 
	if(bound[i] < 1) bound[i] = 1;
      f0 = f;
    }
  }
  params = best;
  kP = params[0];
  kI = params[1];
  kD = params[2];
  dryFriction = params[3];
  viscousFriction = params[4];
  return f0;
}



//given the current (q,dq) of the robot, computes linearized 1-d models of the
//robot's dynamics
//ddq = A * t + b
//Assume all other links use steady state torque t = -A^-1 b
void LinearizeRobot(RobotModel& robot,const vector<int>& fixedLinks,
		    Vector& minv,Vector& d,Vector& k,Vector& c)
{
  minv.resize(robot.links.size());
  d.resize(robot.links.size());
  k.resize(robot.links.size());
  c.resize(robot.links.size());
  //Approximation: rather than linearize the gravity and coriolis terms, just assume constant
  d.setZero();
  k.setZero();
  Matrix A;
  Vector b;
  vector<int> fixedDofs;
  for(size_t i=0;i<robot.joints.size();i++)
    if(robot.joints[i].type == RobotModelJoint::Weld) 
      fixedDofs.push_back(robot.joints[i].linkIndex);

  Vector tsteady;
  if(fixedLinks.empty()) {
    //if only fixed dofs are included, do it unconstrained
    NewtonEulerSolver ne(robot);
    Vector ddqref;
    ne.SetGravityWrenches(gGravity);
    ne.CalcResidualAccel(ddqref);
    ne.CalcKineticEnergyMatrixInverse(A);
    ne.CalcResidualTorques(tsteady);
    b = ddqref;
  }
  else {
    bool res = ConstrainedForwardDynamics(robot,fixedLinks,fixedDofs,A,b);
    Assert(res == true);
    Vector zero(robot.links.size(),Zero);
    res=ConstrainedCalcTorque(robot,fixedLinks,fixedDofs,zero,tsteady);
    if(!res) {
      cout<<"Warning, unable to solve for constrained torques"<<endl;
      cout<<"   Calculating free-floating torques instead, may be erroneous..."<<endl;
      NewtonEulerSolver ne(robot);
      ne.SetGravityWrenches(gGravity);
      ne.CalcResidualTorques(tsteady);
    }
  }

  A.getDiagCopy(0,minv);
  c = b;
  //add on steady state torques
  for(int i=0;i<b.n;i++) {
    Real oldt = tsteady(i);
    tsteady(i) = 0;
    c(i) += A.dotRow(i,tsteady);
    tsteady(i) = oldt;
    if(FuzzyZero(minv(i))) {
      c(i) = 0;
    }
    else if(IsFinite(minv(i))) {
      c(i) /= minv(i);
    }
    else {
      fprintf(stderr,"Invalid mass matrix inverse, entry %d: %g\n",i,minv(i));
      fprintf(stderr,"Most likely reason is an invalid setting for the fixed links\n");
      //if(gErrorGetchar) {
      //printf("Press enter to continue\n");
      //getchar();
      //}
      fprintf(stderr,"Aborting...\n");
      Abort();
    }
    if(robot.joints[i].type == RobotModelJoint::Weld) 
      c(i) = 0;
  }
  if(c.maxAbsElement() > 1000) {
    fprintf(stderr,"Warning, very high torques?\n");
    cout<<c<<endl;
    cout<<"Steady state torques: "<<tsteady<<endl;
    cout<<"Linearized offset vector: "<<b<<endl;
    cout<<"Mass matrix inverse diag"<<minv<<endl;
    if(gErrorGetchar) {
      printf("Press enter to continue\n");
      getchar();
    }
  }
}

struct MotorCalibrationProblem
{
  RobotModel* robot;
  /** time step of the simulator */
  Real simDt;
  /** not used */
  bool motorInterpolates;
  vector<LinearPathResource> commandedQ,sensedQ;
  vector<LinearPathResource> commandedV,sensedV;
  Real vErrorWeight;
  vector<int> fixedLinks;
  vector<int> estimateDrivers;
  //true if want to: save info to disk, save pre-optimization paths to disk, save post-optimizaiton paths to disk
  bool saveInfo,savePreOptimize,savePostOptimize;
};

void RunCalibrationInd(MotorCalibrationProblem& problem,int numIters)
{
  printf("Beginning calibration...\n");
  vector<Real> dts;
  vector<vector<Real> > minvs,ds,ks,cs,x0s,dx0s,x1s,dx1s,xcmds,dxcmds;
  size_t ndof = problem.robot->links.size();
  minvs.resize(ndof);
  ds.resize(ndof);
  ks.resize(ndof);
  cs.resize(ndof);
  x0s.resize(ndof);
  dx0s.resize(ndof);
  x1s.resize(ndof);
  dx1s.resize(ndof);
  xcmds.resize(ndof);
  dxcmds.resize(ndof);
  vector<int> pathIndex;
  pathIndex.push_back(0);
  Timer timer;
  Vector minv,d,k,c;
  for(size_t trial=0;trial<problem.commandedQ.size();trial++) {
    size_t n=problem.sensedQ[trial].milestones.size()-1;
    size_t istart = dts.size();
    for(size_t j=0;j<ndof;j++) {
      minvs[j].resize(istart+n);
      ds[j].resize(istart+n);
      ks[j].resize(istart+n);
      cs[j].resize(istart+n);
      x0s[j].resize(istart+n);
      dx0s[j].resize(istart+n);
      x1s[j].resize(istart+n);
      dx1s[j].resize(istart+n);
      xcmds[j].resize(istart+n);
      dxcmds[j].resize(istart+n);
    }
    dts.resize(istart+n);
    pathIndex.push_back(int(istart+n));
    printf("Linearizing trial %d\n",trial);
    timer.Reset();
    for(size_t i=0;i<n;i++) {
      problem.robot->UpdateConfig(problem.sensedQ[trial].milestones[i]);
      problem.robot->dq = problem.sensedV[trial].milestones[i];
      LinearizeRobot(*problem.robot,problem.fixedLinks,
		     minv,d,k,c);
      /*
      cout<<"Constrained mass matrix inv"<<endl;
      for(int j=0;j<minv.n;j++) {
	cout<<"  "<<problem.robot->linkNames[j]<<": "<<minv(j)<<endl;
      }
      getchar();
      */
      dts[istart+i] = problem.sensedQ[trial].times[i+1]-problem.sensedQ[trial].times[i];
      for(int j=0;j<minv.n;j++) {
	minvs[j][istart+i] = minv[j];
	ds[j][istart+i] = d[j];
	ks[j][istart+i] = k[j];
	cs[j][istart+i] = c[j];
	x0s[j][istart+i] = problem.sensedQ[trial].milestones[i][j];
	dx0s[j][istart+i] = problem.sensedV[trial].milestones[i][j];
	x1s[j][istart+i] = problem.sensedQ[trial].milestones[i+1][j];
	dx1s[j][istart+i] = problem.sensedV[trial].milestones[i+1][j];
	xcmds[j][istart+i] = problem.commandedQ[trial].milestones[i][j];
	dxcmds[j][istart+i] = problem.commandedV[trial].milestones[i][j];
      }
    }
    printf("Time: %g, time per milestone: %g\n",timer.ElapsedTime(),timer.ElapsedTime()/n);
  }

  //save debug info to disk
  if(problem.saveInfo) {
    cout<<"Saving data to motorcalibrate.csv"<<endl;
    ofstream out("motorcalibrate.csv",ios::out);
    out<<"dt,";
    for(size_t k=0;k<problem.estimateDrivers.size();k++) {
      int d = problem.estimateDrivers[k];
      int j = problem.robot->drivers[d].linkIndices[0];
      out<<"m^-1["<<j<<"],d["<<j<<"],k["<<j<<"],c["<<j<<"],x0["<<j<<"],dx0["<<j<<"],x1["<<j<<"],dx1["<<j<<"],xcmd["<<j<<"],dxcmd["<<j<<"],";
    }
    out<<endl;
    for(size_t i=0;i<dts.size();i++) {
      out<<dts[i]<<",";
      for(size_t k=0;k<problem.estimateDrivers.size();k++) {
	int d = problem.estimateDrivers[k];
	int j = problem.robot->drivers[d].linkIndices[0];
	out<<minvs[j][i]<<","<<ds[j][i]<<","<<ks[j][i]<<","<<cs[j][i]<<","<<x0s[j][i]<<","<<dx0s[j][i]<<","<<x1s[j][i]<<","<<dx1s[j][i]<<","<<xcmds[j][i]<<","<<dxcmds[j][i]<<",";
      }
      out<<endl;
    }
  }

  if(problem.savePreOptimize) {
    for(size_t trial=0;trial<problem.commandedQ.size();trial++) {
      LinearPathResource path;
      path.times = problem.commandedQ[trial].times;
      path.milestones = problem.commandedQ[trial].milestones;
      for(size_t k=0;k<problem.estimateDrivers.size();k++) {
	int d = problem.estimateDrivers[k];
	int j = problem.robot->drivers[d].linkIndices[0];
	Real& kP = problem.robot->drivers[d].servoP;
	Real& kI = problem.robot->drivers[d].servoI;
	Real& kD = problem.robot->drivers[d].servoD;
	Real& dryFriction = problem.robot->drivers[d].dryFriction;
	Real& viscousFriction = problem.robot->drivers[d].viscousFriction;
	int s = pathIndex[trial];
	int e = pathIndex[trial+1];
	vector<Real> minv_path(minvs[j].begin()+s,minvs[j].begin()+e);
	vector<Real> d_path(ds[j].begin()+s,ds[j].begin()+e);
	vector<Real> k_path(ks[j].begin()+s,ks[j].begin()+e);
	vector<Real> c_path(cs[j].begin()+s,cs[j].begin()+e);
	vector<Real> xcmd_path(xcmds[j].begin()+s,xcmds[j].begin()+e);
	vector<Real> dxcmd_path(dxcmds[j].begin()+s,dxcmds[j].begin()+e);
	vector<Real> dts_path(dts.begin()+s,dts.begin()+e);
	Real x0 = x0s[j][s];
	Real dx0 = dx0s[j][s];
	Real torquemin=problem.robot->drivers[d].tmin;
	Real torquemax=problem.robot->drivers[d].tmax;
	vector<Real> xtraj,dxtraj;
	SimulateDOF(minv_path,d_path,k_path,c_path,
		    x0,dx0,
		    xcmd_path,dxcmd_path,
		    kP,kI,kD,dryFriction,viscousFriction,
		    dts_path,gDefaultTimestep,torquemin,torquemax,
		    xtraj,dxtraj);
	Assert(xtraj.size()==path.milestones.size());
	for(size_t m=0;m<xtraj.size();m++)
	  path.milestones[m][j] = xtraj[m];
      }
      stringstream ss;
      ss<<"motorcalibrate_before_"<<trial<<".path";
      cout<<"Saving pre-calibration path to "<<ss.str()<<endl;
      path.Save(ss.str().c_str());
    }
  }

  //do the estimation
  vector<Real> rmsds(problem.estimateDrivers.size());
  for(size_t k=0;k<problem.estimateDrivers.size();k++) {
    int d = problem.estimateDrivers[k];
    Assert(problem.robot->drivers[d].type == RobotModelDriver::Normal);
    int j = problem.robot->drivers[d].linkIndices[0];

    Real& kP = problem.robot->drivers[d].servoP;
    Real& kI = problem.robot->drivers[d].servoI;
    Real& kD = problem.robot->drivers[d].servoD;
    Real& dryFriction = problem.robot->drivers[d].dryFriction;
    Real& viscousFriction = problem.robot->drivers[d].viscousFriction;
    printf("Driver %d, link %d (%s)\n",d,j,problem.robot->linkNames[j].c_str());
    printf("Initial kP: %g, kI: %g, kD: %g\n",kP,kI,kD);
    printf("Initial dry friction: %g, viscous friction: %g\n",dryFriction,viscousFriction);
    timer.Reset();
    Real res = GOptimizeDof(minvs[j],ds[j],ks[j],cs[j],
		x0s[j],dx0s[j],x1s[j],dx1s[j],xcmds[j],dxcmds[j],
		dts,problem.robot->drivers[d].tmin,problem.robot->drivers[d].tmax,
		kP,kI,kD,dryFriction,viscousFriction,numIters);   
    printf("Optimized kP: %g, kI: %g, kD: %g\n",kP,kI,kD);
    printf("Optimized dry friction: %g, viscous friction: %g\n",dryFriction,viscousFriction);
    printf("Optimized RMSD: %g\n",res);
    printf("Time %g\n",timer.ElapsedTime());
    printf("\n");
    rmsds[k]=res;
    if(gStepGetchar) {
      getchar();
    }
  }

  if(problem.savePostOptimize) {
    for(size_t trial=0;trial<problem.commandedQ.size();trial++) {
      LinearPathResource path;
      path.times = problem.commandedQ[trial].times;
      path.milestones = problem.commandedQ[trial].milestones;
      for(size_t k=0;k<problem.estimateDrivers.size();k++) {
	int d = problem.estimateDrivers[k];
	int j = problem.robot->drivers[d].linkIndices[0];
	Real& kP = problem.robot->drivers[d].servoP;
	Real& kI = problem.robot->drivers[d].servoI;
	Real& kD = problem.robot->drivers[d].servoD;
	Real& dryFriction = problem.robot->drivers[d].dryFriction;
	Real& viscousFriction = problem.robot->drivers[d].viscousFriction;
	int s = pathIndex[trial];
	int e = pathIndex[trial+1];
	vector<Real> minv_path(minvs[j].begin()+s,minvs[j].begin()+e);
	vector<Real> d_path(ds[j].begin()+s,ds[j].begin()+e);
	vector<Real> k_path(ks[j].begin()+s,ks[j].begin()+e);
	vector<Real> c_path(cs[j].begin()+s,cs[j].begin()+e);
	vector<Real> xcmd_path(xcmds[j].begin()+s,xcmds[j].begin()+e);
	vector<Real> dxcmd_path(dxcmds[j].begin()+s,dxcmds[j].begin()+e);
	vector<Real> dts_path(dts.begin()+s,dts.begin()+e);
	Real x0 = x0s[j][s];
	Real dx0 = dx0s[j][s];
	Real torquemin=problem.robot->drivers[d].tmin;
	Real torquemax=problem.robot->drivers[d].tmax;
	vector<Real> xtraj,dxtraj;
	SimulateDOF(minv_path,d_path,k_path,c_path,
		    x0,dx0,
		    xcmd_path,dxcmd_path,
		    kP,kI,kD,dryFriction,viscousFriction,
		    dts_path,gDefaultTimestep,torquemin,torquemax,
		    xtraj,dxtraj);
	Assert(xtraj.size()==path.milestones.size());
	for(size_t m=0;m<xtraj.size();m++)
	  path.milestones[m][j] = xtraj[m];
      }
      stringstream ss;
      ss<<"motorcalibrate_after_"<<trial<<".path";
      cout<<"Saving post-calibration path to "<<ss.str()<<endl;
      path.Save(ss.str().c_str());
    }
  }

  printf("Final RMSDs:\n");
  for(size_t k=0;k<problem.estimateDrivers.size();k++) {
	int d = problem.estimateDrivers[k];
	int j = problem.robot->drivers[d].linkIndices[0];
    printf("driver %d (%s),",d,problem.robot->linkNames[j].c_str());
  }
  printf("\n");
  for(size_t k=0;k<problem.estimateDrivers.size();k++) {
    printf("%g,",rmsds[k]);
  }
  printf("\n");
}


/*
void SimulateODE(Simulator& sim,Real advanceDt,Real settleTime,
		 const LinearPathResource& cmdQ,const LinearPathResource& cmdV,
		 Real tmax,
		 vector<Config>& qout,vector<Vector>& dqout)
{
  sim.robotControllers[0]->SendCommand("append_tq",sim.time+advanceDt,cmdQ.milestones[0]);
  sim.Advance(settleTime);
  Config q,dq;
  sim.controlSimulators[0].GetSensedConfig(q);
  sim.controlSimulators[0].GetSensedVelocity(dq);
  qout.resize(1);
  dqout.resize(1);
  qout[0] = q;
  dqout[0] = dq;

  Real tadjust = sim.time-cmdQ.times[0]+advanceDt;
  for(size_t i=0;i<cmdQ.times.size();i++)
    sim.robotControllers[0]->SendCommand("append_tq",cmdQ.times[i]+tadjust,cmdQ.milestones[i]);
  Real t=0;
  while(t < tmax) {
    sim.Advance(advanceDt);
    
    sim.controlSimulators[0].GetSensedConfig(q);
    sim.controlSimulators[0].GetSensedVelocity(dq);
    qout.push_back(q);
    dqout.push_back(dq);
  }
}

void SimulateConstrained(RobotModel& robot,Real simDt,bool motorInterpolate,
			 const vector<int>& fixedLinks,
			 const LinearPathResource& cmdQ,const LinearPathResource& cmdV,
			 Real tmax,
			 vector<Config>& qout,vector<Vector>& dqout)
{
  qout.resize(1);
  dqout.resize(1);
  qout[0] = q;
  dqout[0] = dq;

  vector<int> fixedDofs;
  for(size_t i=0;i<robot.joints.size();i++)
    if(robot.joints[i].type == RobotModelJoint::Weld)
      fixedDofs.push_back(robot.joints[i].linkIndex);

  Vector q=robot.q,dq=robot.dq;
  Real t=0;
  int cmdpathindex = 0;
  Vector qcmd,dqcmd,qdriver,dqdriver,torquedriver,kfdriver,torque,ddq;
  qdriver.resize(robot.drivers.size());
  dqdriver.resize(robot.drivers.size());
  torquedriver.resize(robot.drivers.size());
  kfdriver.resize(robot.drivers.size());
  torque.resize(robot.links.size());
  while(t < tmax) {
    //seek to find the joint commands
    while(cmdpathindex < (int)cmdQ.times.size() && t >= cmdQ.times[cmdpathindex]) {
      cmdpathindex++;
    }
    if(cmdpathindex >= (int)cmdQ.times.size()) {
      qcmd = cmdQ.milestones.back();
      dqcmd.sub(qcmd,qcmd);
    }
    else {
      if(motorInterpolates) {
	Real dt=cmdQ.times[cmdpathindex]-cmdQ.times[cmdpathindex-1];
	Real u=(t-cmdQ.times[cmdpathindex-1])/dt;
	Interpolate(robot,cmdQ.milestones[cmdpathindex-1],cmdQ.milestones[cmdpathindex],qcmd);
	//finite difference
	InterpolateDerivative(robot,cmdQ.milestones[cmdpathindex-1],cmdQ.milestones[cmdpathindex],dqcmd);
	dqcmd /= dt;
      }
      else {
	qcmd = cmdQ.milestones[cmdpathindex-1];
	if(cmdV.milestones.empty())
	  dqcmd.sub(qcmd,qcmd);
	else
	  dqcmd = cmdV.milestones[cmdpathindex-1];
      }
    }
    
    //simulate torques with qcmd,dqcmd
    robot.UpdateConfig(q);
    robot.dq = dq;
    for(size_t j=0;j<robot.drivers.size();j++) {
      qdriver[j] = robot.GetDriverValue(i);
      dqdriver[j] = robot.GetDriverVelocity(i);
      kfdriver[j] = robot.drivers[j].kFriction + Abs(robot.GetDriverVelocity(i))*robot.drivers[j].viscousFriction;
    }

    robot.UpdateConfig(qcmd);
    robot.dq = dqcmd;
    for(size_t j=0;j<robot.drivers.size();j++) {
      Real dcmd = robot.GetDriverValue(i);
      Real ddcmd = robot.GetDriverVelocity(i);
      driverTorque[j] = robot.drivers[j].kP*(dcmd - qdriver[j]) + robot.drivers[j].kI*Idriver[j] + robot.drivers[j].kD*(ddcmd - dqdriver[j]);

      //TEMP: this is not right since it doesn't take external joint forces into account
      if(Abs(driverTorque[j]) < kfdriver[j])
	driverTorque[j] = 0;
      else
	driverTorque[j] -= Sign(driverTorque[j])*driverTorque[j];
    }

    //TODO: compute integration with frictional limits

    ConstrainedCalcAccel(robot,problem.fixedLinks,fixedDofs,torque,ddq);
    q.madd(dq,simDt);
    q.madd(ddq,0.5*Sqr(simDt));
    dq.madd(ddq,simDt);

    qout.push_back(q);
    dqout.push_back(dq);
    t += simDt;
  }
}


class MotorCalibrationError : public ScalarFieldFunction
{
public:
  MotorCalibrationProblem& problem;

  MotorCalibrationError(MotorCalibrationProblem& _problem)
    :problem(_problem)
  {}
  void DriversToVector(Vector& x)
  {
    x.resize((int)estimateDrivers.size()*5);
    for(size_t i=0;i<estimateDrivers.size();i++) {
      int d=estimateDrivers[i];
      x(i*5) = robot->drivers[d].servoP;
      x(i*5+1) = robot->drivers[d].servoI;
      x(i*5+2) = robot->drivers[d].servoD;
      x(i*5+3) = robot->drivers[d].dryFriction;
      x(i*5+4) = robot->drivers[d].viscousFriction;
    }
  }
  void VectorToDrivers(const Vector& x)
  {
    Assert(x.n == (int)estimateDrivers.size()*5);
    for(size_t i=0;i<estimateDrivers.size();i++) {
      int d=estimateDrivers[i];
      robot->drivers[d].servoP = x(i*5);
      robot->drivers[d].servoI = x(i*5+1);	
      robot->drivers[d].servoD = x(i*5+2);	
      robot->drivers[d].dryFriction = x(i*5+3);
      robot->drivers[d].viscousFriction = x(i*5+4);
    }
  }
  void PreEval(const Vector& x) {
    VectorToDrivers(x);
  }
  Real Eval(const Vector& x) {
    Assert(problem.commandedQ.size()==problem.sensedQ.size());
    if(!problem.commandedV.empty())
      Assert(problem.commandedV.size() == problem.commandedQ.size());

    for(size_t i=0;i<problem.commandedQ.size();i++) {
      Assert(problem.commandedQ[i].milestones.size()==problem.sensedQ[i].milestones.size());
      if(!problem.commandedV.empty())
	Assert(problem.commandedV[i].milestones.size()==problem.commandedQ[i].milestones.size());

      //setup simulation state
      problem.robot->UpdateConfig(problem.sensedQ[i].milestones[0]); 
      if(!problem.sensedV.empty())
	problem.robot->dq = problem.sensedV[i].milestones[0];
      else
	problem.robot->dq.setZero();
      Real tduration=problem.sensedQ[i].times.back()-problem.sensedQ[i].times.front();
      vector<Config> qout,dqout;
      SimulateConstrained(*problem.robot,problem.simDt,problem.motorInterpolate,
			  problem.fixedLinks,
			  problem.commandedQ[i],problem.commandedV[i],
			  tduration,qout,dqout);
    }
  }
};

bool SolveCalibration(MotorCalibrationProblem& problem,int numIters)
{

}

*/

void Difference(RobotModel& robot,const LinearPathResource& path,LinearPathResource& vpath)
{
  vpath.times=path.times;
  vpath.milestones.resize(path.times.size());
  InterpolateDerivative(robot,path.milestones[0],path.milestones[1],vpath.milestones[0]);
  vpath.milestones[0] /= (path.times[1]-path.times[0]);
  for(size_t i=1;i<path.times.size();i++) {
    InterpolateDerivative(robot,path.milestones[i-1],path.milestones[i],vpath.milestones[i]);
    vpath.milestones[i] /= (path.times[i]-path.times[i-1]);
  }
}

void test()
{
  RobotModel robot;
  if(!robot.Load("data/free_cube.rob")) {
    fprintf(stderr,"Failed to load robot\n");
    return;
  }
  vector<int> fixedLinks(1,6),fixedDofs;
  Matrix A;
  Vector b;
  ConstrainedForwardDynamics(robot,fixedLinks,fixedDofs,A,b);
}

string motorcalibrate(AnyCollection settings){
  string robotfn;
  bool res=settings["robot"].as(robotfn); assert(res);
  int numIters = int(settings["numIters"]);
  vector<int> drivers,fixedLinks;
  vector<string> commandedPaths,sensedPaths;
  res=settings["drivers"].asvector(drivers); assert(res);
  res=settings["fixedLinks"].asvector(fixedLinks); assert(res);
  res=settings["commandedPaths"].asvector(commandedPaths); assert(res);
  res=settings["sensedPaths"].asvector(sensedPaths); assert(res);
  gMaxMilestones = (unsigned int)(settings["maxMilestones"]);
  gDefaultTimestep = Real(settings["dt"]);
  gDefaultVelocityWeight = Real(settings["velocityErrorWeight"]);
  assert(commandedPaths.size()==sensedPaths.size());

  RobotModel robot;
  if(!robot.Load(robotfn.c_str())) {
    fprintf(stderr,"Failed to load robot\n");
    return NULL;
  }

  MotorCalibrationProblem problem;
  problem.robot = &robot;
  problem.simDt = gDefaultTimestep;
  problem.vErrorWeight = gDefaultVelocityWeight;
  problem.motorInterpolates = true;
  problem.fixedLinks = fixedLinks;
  problem.saveInfo = true;
  problem.savePreOptimize = true;
  problem.savePostOptimize = true;
  if(fixedLinks.empty()) {
    for(size_t i=0;i<robot.joints.size();i++)
      if(robot.joints[i].type == RobotModelJoint::Floating || robot.joints[i].type == RobotModelJoint::FloatingPlanar)
	printf("Warning, fixed links are empty, and this is not a fixed-base robot?\n");
  }
  if(drivers.empty()) {
    problem.estimateDrivers.resize(robot.drivers.size());
    for(size_t i=0;i<robot.drivers.size();i++)
      problem.estimateDrivers[i] = (int)i;
  }
  else
    problem.estimateDrivers = drivers;

  problem.commandedQ.resize(commandedPaths.size());
  problem.sensedQ.resize(commandedPaths.size());
  for(size_t i=0;i<commandedPaths.size();i++) {
    if(!problem.commandedQ[i].Load(commandedPaths[i].c_str())) {
      fprintf(stderr,"Failed to load path %s\n",commandedPaths[i].c_str());
      return NULL;
    }
    if(!problem.sensedQ[i].Load(sensedPaths[i].c_str())) {
      fprintf(stderr,"Failed to load path %s\n",sensedPaths[i].c_str());
      return NULL;
    }
    if((int)problem.commandedQ[i].times.size() > gMaxMilestones) {
      problem.commandedQ[i].times.resize(gMaxMilestones+1);
      problem.commandedQ[i].milestones.resize(gMaxMilestones+1);
    }
    if((int)problem.sensedQ[i].times.size() > gMaxMilestones) {
      problem.sensedQ[i].times.resize(gMaxMilestones+1);
      problem.sensedQ[i].milestones.resize(gMaxMilestones+1);
    }
  }
  problem.commandedV.resize(problem.commandedQ.size());
  problem.sensedV.resize(problem.sensedQ.size());
  for(size_t i=0;i<problem.commandedQ.size();i++) {
    Difference(robot,problem.commandedQ[i],problem.commandedV[i]);
    Difference(robot,problem.sensedQ[i],problem.sensedV[i]);
  }

  RunCalibrationInd(problem,numIters);
  stringstream ret_stream;
  bool asURDF = (0==strcmp(FileExtension(robotfn.c_str()),"urdf"));
  if(asURDF) {
    //write as <robot><klampt> children
    for(size_t i=0;i<robot.drivers.size();i++) {
      if(robot.drivers[i].type == RobotModelDriver::Normal) {
	int link = robot.drivers[i].linkIndices[0];
	ret_stream<<"<link name=\""<<robot.linkNames[link]<<"\" "<<
	  "servoP="<<robot.drivers[i].servoP<<" "
	  "servoI="<<robot.drivers[i].servoI<<" "
	  "servoD="<<robot.drivers[i].servoD<<" "
	  "dryFriction="<<robot.drivers[i].dryFriction<<" "
	  "viscousFriction="<<robot.drivers[i].viscousFriction<<" />"<<endl;
      }
    }
  }
  else {
    //write as .rob file children
    ret_stream<<"servoP ";
    for(size_t i=0;i<robot.drivers.size();i++)
      ret_stream<<robot.drivers[i].servoP<<" ";
    ret_stream<<"\n";
    ret_stream<<"servoI ";
    for(size_t i=0;i<robot.drivers.size();i++)
      ret_stream<<robot.drivers[i].servoI<<" ";
    ret_stream<<"\n";
    ret_stream<<"servoD ";
    for(size_t i=0;i<robot.drivers.size();i++)
      ret_stream<<robot.drivers[i].servoD<<" ";
    ret_stream<<"\n";
    ret_stream<<"dryFriction ";
    for(size_t i=0;i<robot.drivers.size();i++)
      ret_stream<<robot.drivers[i].dryFriction<<" ";
    ret_stream<<"\n";
    ret_stream<<"viscousFriction ";
    for(size_t i=0;i<robot.drivers.size();i++)
      ret_stream<<robot.drivers[i].viscousFriction<<" ";
  }
  string output=ret_stream.str();
  cout<<endl;
  cout<<"Copy the following lines into your robot file:"<<endl;
  cout<<output<<endl;
  return output;
}

int main_shell(int argc,char** argv)
{
  RobotModel::disableGeometryLoading = true;

  MotorCalibrateSettings settings;
  settings["robot"]=string();
  settings["numIters"]=1000;
  settings["maxMilestones"]=gMaxMilestones;
  settings["dt"]=gDefaultTimestep;
  settings["velocityErrorWeight"]=gDefaultVelocityWeight;
  settings["drivers"]=vector<int>();
  settings["fixedLinks"]=vector<int>();
  settings["commandedPaths"]=vector<string>();
  settings["sensedPaths"]=vector<string>();
  if(argc <= 1) {
    printf("Usage: MotorCalibrate settings_file\n");
    printf("Writing default settings to motorcalibrate_default.settings");
    settings.write("motorcalibrate_default.settings");
    return 0;
  }
  for(int i=1;i<argc;i++)
    settings.read(argv[i]);

  motorcalibrate(settings);
  return 0;
}

#ifndef HAVE_QT
int main(int argc,char** argv){
  return main_shell(argc,argv);
}
#endif
