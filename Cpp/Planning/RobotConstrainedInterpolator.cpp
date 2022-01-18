#include "RobotConstrainedInterpolator.h"
using namespace std;
using namespace Klampt;

RobotConstrainedInterpolator::RobotConstrainedInterpolator(RobotModel& robot,const vector<IKGoal>& goals)
  :ConstrainedInterpolator(&space,&f),space(robot),f(robot)
{
  f.UseIK(goals);
  GetDefaultIKDofs(robot,goals,f.activeDofs);
  robot.ConfigureDriverConstraints(f);
}

void RobotConstrainedInterpolator::ConstraintValue(const Config& x,Vector& v)
{
  //TODO: should just update robot.q direct from x...
  Vector xtemp(f.activeDofs.Size());
  f.activeDofs.InvMap(x,xtemp);
  (*constraint)(xtemp,v);
}

bool RobotConstrainedInterpolator::Project(Config& x)
{
  solver.tolf = ftol;
  solver.tolx = solver.tolmin = ftol*1e-2;
  solver.verbose = 0;
  if(solver.bmin.empty() && !xmin.empty()) {
    solver.bmin.resize(f.activeDofs.Size());
    solver.bmax.resize(f.activeDofs.Size());
    f.activeDofs.InvMap(xmin,solver.bmin);
    f.activeDofs.InvMap(xmax,solver.bmax);
  }
  int iters=maxNewtonIters;
  solver.x.resize(f.activeDofs.Size());
  f.activeDofs.InvMap(x,solver.x);
  if(!solver.GlobalSolve(iters)) return false;
  f.activeDofs.Map(solver.x,x);
  return true;
}


RobotSmoothConstrainedInterpolator::RobotSmoothConstrainedInterpolator(RobotModel& robot,const vector<IKGoal>& goals)
  :SmoothConstrainedInterpolator(&space,&f),space(robot),f(robot)
{
  SmoothConstrainedInterpolator::manifold = &space;
  f.UseIK(goals);
  GetDefaultIKDofs(robot,goals,f.activeDofs);  
}

void RobotSmoothConstrainedInterpolator::ConstraintValue(const Config& x,Vector& v)
{
  //TODO: should just update robot.q direct from x...
  Vector xtemp(f.activeDofs.Size());
  f.activeDofs.InvMap(x,xtemp);
  (*constraint)(xtemp,v);
}


bool RobotSmoothConstrainedInterpolator::Project(Config& x)
{
  solver.tolf = ftol;
  solver.tolx = solver.tolmin = ftol*1e-2;
  solver.verbose = 0;
  if(solver.bmin.empty() && !xmin.empty()) {
    solver.bmin.resize(f.activeDofs.Size());
    solver.bmax.resize(f.activeDofs.Size());
    f.activeDofs.InvMap(xmin,solver.bmin);
    f.activeDofs.InvMap(xmax,solver.bmax);
  }
  int iters=maxNewtonIters;
  solver.x.resize(f.activeDofs.Size());
  f.activeDofs.InvMap(x,solver.x);
  if(!solver.GlobalSolve(iters)) return false;
  f.activeDofs.Map(solver.x,x);
  return true;
}

bool RobotSmoothConstrainedInterpolator::ProjectVelocity(const Config& x,Vector& v)
{
  Vector temp,vtemp;
  temp.resize(f.activeDofs.Size());
  vtemp.resize(f.activeDofs.Size());
  f.activeDofs.InvMap(x,temp);
  f.activeDofs.InvMap(v,vtemp);
  f.PreEval(temp);
  Matrix J;
  f.Jacobian(temp,J);

  if(!solver.bmin.empty()) {
    //look through active contraints, set that column to 0
    for(int i=0;i<temp.n;i++) {
      if(temp(i)==solver.bmin(i) || temp(i) == solver.bmax(i)) {
	vtemp(i) = 0;
	for(int j=0;j<J.m;j++)
	  J(j,i) = 0;
      }
    }
  }

  RobustSVD<Real> svd;
  bool res=svd.set(J);
  if(!res) {
    fprintf(stderr,"SmoothConstrainedInterpolator: Numerical error projecting velocity?\n");
    return false;
  }
  Vector vtemp2;
  svd.nullspaceComponent(vtemp,vtemp2);
  vtemp -= vtemp2;
  f.activeDofs.Map(vtemp,v);
  return true;
}
