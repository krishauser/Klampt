#include "Modeling/Resources.h"
#include "Modeling/Robot.h"
#include "Modeling/Interpolate.h"
#include <robotics/ConstrainedDynamics.h>
#include <math/differentiation.h>
#include <math/LDL.h>
#include <optimization/Minimization.h>
#include <utils/AnyCollection.h>
#include <fstream>
#include <timer.h>
using namespace std;
using namespace Math;

//set this to something small if you want this to be faster
//const static size_t gMaxMilestones = 100;
static size_t gMaxMilestones = 100000;

static Vector3 gGravity(0,0,-9.8);
static Real gDefaultTimestep = 1e-3;
static Real gDefaultVelocityWeight = Sqr(0.005);

/** @brief Simulates a single DOF under PD control and stick-slip friction.
 *
 * Given a linearized model
 *  x'' = minv * (t + f* - d*x' + k*x + c)
 * with PD torques t = kp(xdes-x)-kd(x'des-x')
 * and frictional forces f* = min_f |x''| s.t. |f| <= mu_d + |x'|*mu_v
 * returns the ending x(T),x'(T)
 */
void SimulateDOF(Real minv,Real d,Real k,Real c,
		 Real x0,Real dx0,
		 Real xDes,Real dxDes,
		 Real kP,Real kD,Real dryFriction,Real viscousFriction,
		 Real T,Real timestep,
		 Real& xT,Real& dxT)
{
  xT=x0;
  dxT=dx0;
  //printf("desired (%g,%g)\n",xDes,dxDes);
  //printf("(%g,%g) ",xT,dxT);
  Real t=0;
  while(t < T) {
    //in absence of frictional forces, this is the acceleration
    Real ddx0 = (kP*(xDes-xT)+kD*(dxDes-dxT) - d*dxT - k*xT - c)*minv;
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
      xT += dxT*dt + 0.5*ddx*Sqr(timestep);
      dxT += ddx*dt;
      //printf("(%g,%g) ",xT,dxT);
      //printf("\n");
      //getchar();
      return;
    }
    xT += dxT*dt + 0.5*ddx*Sqr(timestep);
    dxT += ddx*dt;
    t += dt;
    //printf("(%g,%g) ",xT,dxT);
  }
}

class Simulate1DOFFunc : public VectorFieldFunction
{
public:
  Real minv,d,k,c,I,x0,dx0,xDes,dxDes,T;
  Simulate1DOFFunc(Real _minv,Real _d,Real _k,Real _c,
		   Real _I,Real _x0,Real _dx0,
		   Real _xDes,Real _dxDes,
		   Real _T)
    :minv(_minv),d(_d),k(_k),c(_c),I(_I),x0(_x0),dx0(_dx0),xDes(_xDes),dxDes(_dxDes),T(_T)
  {}
  virtual int NumDimensions() { return 2; }
  virtual void Eval(const Vector& params,Vector& out) {
    Assert(params.n == 5);
    Real kP = params(0);
    Real kI = params(1);
    Real kD = params(2);
    Real mu_d = params(3);
    Real mu_v = params(4);
    Real cI = c - kI*I;
    Real xT,dxT;
    SimulateDOF(minv,d,k,cI,x0,dx0,
		xDes,dxDes,
		kP,kD,mu_d,mu_v,
		T,gDefaultTimestep,
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
  vector<SmartPointer<Simulate1DOFFunc> > fs;

  OptimizeDofFunction(const vector<Real>& _minvs,const vector<Real>& _ds,const vector<Real>& _ks,const vector<Real>& _cs,
		      const vector<Real>& _x0s,const vector<Real>& _dx0s,
		      const vector<Real>& _x1s,const vector<Real>& _dx1s,
		      const vector<Real>& _xDes,const vector<Real>& _dxDes,
		      const vector<Real>& _dts)
    :minvs(_minvs),ds(_ds),ks(_ks),cs(_cs),x0s(_x0s),dx0s(_dx0s),x1s(_x1s),dx1s(_dx1s),xDes(_xDes),dxDes(_dxDes),dts(_dts)
  {
    fs.resize(minvs.size());
    for(size_t i=0;i<minvs.size();i++) {
      fs[i] = new Simulate1DOFFunc(minvs[i],ds[i],ks[i],cs[i],
				   0,x0s[i],dx0s[i],
				   xDes[i],dxDes[i],
				   dts[i]);
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
    h(0) = Max(1e-2*params(0),1e-3);
    h(1) = Max(1e-2*params(1),1e-3);
    h(2) = Max(1e-2*params(2),1e-3);
    h(3) = 1e-3;
    h(4) = 1e-2;
    grad.resize(5);
    GradientCenteredDifference(*this,temp,h,grad);
    for(int i=0;i<grad.n;i++)
      if(!IsFinite(grad[i])) {
	printf("Warning, instability at parameter %d = %g\n",i,params(i));
	grad[i] = 0;
      }
  }
};

//For a bunch of linearized single-dof models
//minvs[i]^-1 *x'' + ds[i]*x' + ks[i]*x + cs[i] = t + f*
//with t = kP*(xDes[i]-x) + kD(dxDes[i]-x') + kI*int(xDes[i]-x)
//and f given by dry and viscous friction terms
//optimizes the kP, kD, kI, and friction terms using descent over numIters iters
void OptimizeDof(const vector<Real>& minvs,const vector<Real>& ds,const vector<Real>& ks,const vector<Real>& cs,
		 const vector<Real>& x0s,const vector<Real>& dx0s,
		 const vector<Real>& x1s,const vector<Real>& dx1s,
		 const vector<Real>& xDes,const vector<Real>& dxDes,
		 const vector<Real>& dts,
		 Real& kP,Real& kI,Real& kD,Real& dryFriction,Real& viscousFriction,
		 int numIters)
{
  OptimizeDofFunction f(minvs,ds,ks,cs,x0s,dx0s,x1s,dx1s,xDes,dxDes,dts);  
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
  if(!IsFinite(fx) || fx > 1e2) {
    cout<<"Initial instability? "<<endl;
    printf("%g, %g: x'' = %g*(PID(%g,%g) - %g*x' + %g*x + %g)\n",x0s[0],dx0s[0],minvs[0],xDes[0],dxDes[0],ds[0],ks[0],cs[0]);
    Vector res;
    for(size_t i=0;i<f.fs.size();i++) { 
      (*f.fs[i])(minProblem.x,res);
      printf("%g, %g: x'' = %g*(PID(%g,%g) - %g*x' + %g*x + %g)\n",res[0],res[1],minvs[i],xDes[i],dxDes[i],ds[i],ks[i],cs[i]);
      if(!IsFinite(res[0]) || !IsFinite(res[1]) || Abs(res[1]) > 1e2) {
	getchar();
	break;
      }
    }
  }
  int maxIters = numIters;
  ConvergenceResult res = minProblem.SolveSD(maxIters);
  cout<<"SD result: "<<res<<" after "<<maxIters<<" iters, RMSE "<<f(minProblem.x)<<endl;
  kP = minProblem.x(0);
  kI = minProblem.x(1);
  kD = minProblem.x(2);
  dryFriction = minProblem.x(3);
  viscousFriction = minProblem.x(4);
}

//given the current (q,dq) of the robot, computes linearized 1-d models of the
//robot's dynamics
//ddq = A * t + b
//steady state torque is t = -A^-1 b
//if we set one of its components to zero, 
void LinearizeRobot(Robot& robot,const vector<int>& fixedLinks,
		    Vector& minv,Vector& d,Vector& k,Vector& c)
{
  minv.resize(robot.links.size());
  d.resize(robot.links.size());
  k.resize(robot.links.size());
  c.resize(robot.links.size());
  d.setZero();
  k.setZero();
  Matrix A;
  Vector b;
  vector<int> fixedDofs;
  for(size_t i=0;i<robot.joints.size();i++)
    if(robot.joints[i].type == RobotJoint::Weld)
      fixedDofs.push_back(robot.joints[i].linkIndex);
  bool res = ConstrainedForwardDynamics(robot,fixedLinks,fixedDofs,A,b);
  Assert(res == true);
  SVDecomposition<Real> svd;
  svd.set(A);
  Vector tsteady;
  svd.backSub(b,tsteady);
  tsteady.inplaceNegative();

  A.getDiagCopy(0,minv);
  c = b;
  //add on steady state torques
  for(int i=0;i<b.n;i++) {
    Real oldt = tsteady(i);
    tsteady(i) = 0;
    c(i) += A.dotRow(i,tsteady);
    tsteady(i) = oldt;
    c(i) /= minv(i);
  }
}

struct MotorCalibrationProblem
{
  Robot* robot;
  /** time step of the simulator */
  Real simDt;
  /** not used */
  bool motorInterpolates;
  vector<LinearPathResource> commandedQ,sensedQ;
  vector<LinearPathResource> commandedV,sensedV;
  Real vErrorWeight;
  vector<int> fixedLinks;
  vector<int> estimateDrivers;
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
  Timer timer;
  Vector minv,d,k,c;
  for(size_t trial=0;trial<problem.commandedQ.size();trial++) {
    size_t n=problem.sensedQ[trial].milestones.size()-1;
    if(n > gMaxMilestones) n=gMaxMilestones;
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
  for(size_t k=0;k<problem.estimateDrivers.size();k++) {
    int d = problem.estimateDrivers[k];
    Assert(problem.robot->drivers[d].type == RobotJointDriver::Normal);
    int j = problem.robot->drivers[d].linkIndices[0];
    bool anydiff = false;
    for(size_t i=0;i<minvs[j].size();i++)
      if(minvs[j][i] != minvs[j][0])
	anydiff = true;
    if(!anydiff) {
      printf("Uhhh... mass matrix %s is a constant?!?!\n",problem.robot->linkNames[j].c_str());
      getchar();
    }
    Real& kP = problem.robot->drivers[d].servoP;
    Real& kI = problem.robot->drivers[d].servoI;
    Real& kD = problem.robot->drivers[d].servoD;
    Real& dryFriction = problem.robot->drivers[d].dryFriction;
    Real& viscousFriction = problem.robot->drivers[d].viscousFriction;
    printf("Driver %d, link %d (%s)\n",d,j,problem.robot->linkNames[j].c_str());
    printf("Initial kP: %g, kI: %g, kD: %g\n",kP,kI,kD);
    printf("Initial dry friction: %g, viscous friction: %g\n",dryFriction,viscousFriction);
    timer.Reset();
    OptimizeDof(minvs[j],ds[j],ks[j],cs[j],
		x0s[j],dx0s[j],x1s[j],dx1s[j],xcmds[j],dxcmds[j],
		dts,kP,kI,kD,dryFriction,viscousFriction,numIters);
    printf("Optimized kP: %g, kI: %g, kD: %g\n",kP,kI,kD);
    printf("Optimized dry friction: %g, viscous friction: %g\n",dryFriction,viscousFriction);
    printf("Time %g\n",timer.ElapsedTime());
  }
}


/*
void SimulateODE(WorldSimulation& sim,Real advanceDt,Real settleTime,
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

void SimulateConstrained(Robot& robot,Real simDt,bool motorInterpolate,
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
    if(robot.joints[i].type == RobotJoint::Weld)
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

void Difference(Robot& robot,const LinearPathResource& path,LinearPathResource& vpath)
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
  Robot robot;
  if(!robot.Load("data/free_cube.rob")) {
    fprintf(stderr,"Failed to load robot\n");
    return;
  }
  vector<int> fixedLinks(1,6),fixedDofs;
  Matrix A;
  Vector b;
  ConstrainedForwardDynamics(robot,fixedLinks,fixedDofs,A,b);
}


class MotorCalibrateSettings : public AnyCollection
{
public:
  MotorCalibrateSettings() {
  }
  bool read(const char* fn) {
    ifstream in(fn,ios::in);
    if(!in) return false;
    AnyCollection newEntries;
    if(!newEntries.read(in)) return false;
    merge(newEntries);
    return true;
  }
  bool write(const char* fn) {
    ofstream out(fn,ios::out);
    if(!out) return false;
    AnyCollection::write(out);
    out.close();
    return true;
  }
};


int main(int argc,const char** argv)
{
  //test();
  //return 0;

  MotorCalibrateSettings settings;
  settings["robot"]=string();
  settings["numIters"]=1000;
  settings["maxMilestones"]=(int)gMaxMilestones;
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

  Robot robot;
  if(!robot.Load(robotfn.c_str())) {
    fprintf(stderr,"Failed to load robot\n");
    return 1;
  }

  MotorCalibrationProblem problem;
  problem.robot = &robot;
  problem.simDt = gDefaultTimestep;
  problem.vErrorWeight = gDefaultVelocityWeight;
  problem.motorInterpolates = true;
  problem.fixedLinks = fixedLinks;
  if(fixedLinks.empty()) {
    printf("Warning, fixed links are empty: is this a fixed-base robot?\n");
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
      return 1;
    }
    if(!problem.sensedQ[i].Load(sensedPaths[i].c_str())) {
      fprintf(stderr,"Failed to load path %s\n",sensedPaths[i].c_str());
      return 1;
    }
  }
  problem.commandedV.resize(problem.commandedQ.size());
  problem.sensedV.resize(problem.sensedQ.size());
  for(size_t i=0;i<problem.commandedQ.size();i++) {
    Difference(robot,problem.commandedQ[i],problem.commandedV[i]);
    Difference(robot,problem.sensedQ[i],problem.sensedV[i]);
  }

  RunCalibrationInd(problem,numIters);
  printf("servoP ");
  for(size_t i=0;i<robot.drivers.size();i++)
    printf("%g ",robot.drivers[i].servoP);
  printf("\n");
  printf("servoI ");
  for(size_t i=0;i<robot.drivers.size();i++)
    printf("%g ",robot.drivers[i].servoI);
  printf("\n");
  printf("servoD ");
  for(size_t i=0;i<robot.drivers.size();i++)
    printf("%g ",robot.drivers[i].servoD);
  printf("\n");
  printf("dryFriction ");
  for(size_t i=0;i<robot.drivers.size();i++)
    printf("%g ",robot.drivers[i].dryFriction);
  printf("\n");
  printf("viscousFriction ");
  for(size_t i=0;i<robot.drivers.size();i++)
    printf("%g ",robot.drivers[i].viscousFriction);
  return 0;
}
