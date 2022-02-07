#include "SelfTest.h"
#include "RampCSpace.h"
#include <KrisLibrary/Timer.h>
#include <time.h>

namespace Klampt {

//tests shortcutting on randomly generated paths between a and b
void TestShortcutting(SingleRobotCSpace* cspace,MotionPlannerFactory& plannerFactory,const Config& a,const Config& b)
{
  const static int maxPlanIters = 1000;
  const static int maxShortcutIters = 200;
  Timer timer;
  for(int p = 0; p < 10; p++) {
    timer.Reset();
    MotionPlannerInterface* planner = plannerFactory.Create(cspace);
    int mstart=planner->AddMilestone(a);
    int mgoal=planner->AddMilestone(b);
    planner->PlanMore(maxPlanIters);
    if(!planner->IsConnected(mstart,mgoal)) {
      delete planner;
      continue;
    }
    
    MilestonePath mpath;
    planner->GetPath(mstart,mgoal,mpath);
    delete planner;
    mpath.Shortcut();
    
    vector<Vector> milestones;
    milestones.resize(mpath.NumMilestones());
    for(int i=0;i<mpath.NumMilestones();i++) {
      milestones[i] = mpath.GetMilestone(i);
    }

    double tPlanning = timer.ElapsedTime();
    timer.Reset();
    
    mpath.edges.clear();
    mpath.CreateEdgesFromMilestones(cspace,milestones);
    mpath.edges.resize(mpath.edges.size()-1);
    bool res=mpath.IsFeasible();
    if(!res) printf("Warning, path is not feasible\n");
    printf("Time to check path feasibility: %g\n",timer.ElapsedTime());
    timer.Reset();
    
    printf("Planned a path successfully!\n");
    
    //shortcuts
    ParabolicRamp::DynamicPath path;
    path.Init(cspace->robot.velMax,cspace->robot.accMax);  
    vector<ParabolicRamp::Vector> vmilestones(milestones.size());
    copy(milestones.begin(),milestones.end(),vmilestones.begin());
    path.SetMilestones(vmilestones);
    
    //seed the RNG
    srand((unsigned int)time(NULL));
    ParabolicRamp::DynamicPath porig = path;
    Real tol = cspace->settings->robotSettings[cspace->index].collisionEpsilon;

    CSpaceFeasibilityChecker feas(cspace);
    ParabolicRamp::RampFeasibilityChecker checker(&feas,tol);
    Real torig = path.GetTotalTime();
    int ns = 0;
    double tShortcut[10];
    double durShortcut[10];
    for(int i=0;i<10;i++) {
      timer.Reset();
      ns += path.Shortcut(maxShortcutIters/10,checker);
      //TODO: exact?
      //ns += path.Shortcut(&checker,-1,Inf,maxShortcutIters/10);
      tShortcut[i] = timer.ElapsedTime();
      durShortcut[i] = path.GetTotalTime();
    }
    Real tnew = path.GetTotalTime();
    printf("Performed %d shortcuts, reduced time from %g to %g\n",ns,torig,tnew);
    printf("**** Timing ****\n");
    printf("Time to plan: %g\n",tPlanning);
    printf("Sections of %d shortcuts each:\n",maxShortcutIters/10);
    for(int i=0;i<10;i++) 
      printf("Time to compute %g, path duration %g\n",tShortcut[i],durShortcut[i]);

    timer.Reset();
    for(size_t i=0;i<path.ramps.size();i++) {
      if(!checker.Check(path.ramps[i])) {
	printf("Shortcutted path ramp %d was found to be infeasible!\n",i);
	getchar();
      }
    }
    /*
      printf("Regular shortcutting\n");
      for(int trial = 0; trial < 5; trial++) {
      path = porig;
      timer.Reset();
      path.Shortcut(cspace,tol,Inf,maxShortcutIters);
      printf("Time to compute %g, path duration %g\n",timer.ElapsedTime(),path.GetTotalTime());
      }
    */

    printf("Dynamic shortcutting\n");
    /* for(int trial = 0;trial < 5; trial++)*/ {
      path = porig;
      timer.Reset();
      ns = path.OnlineShortcut(0.0,0.01,checker);
      printf("Window %g: %d shortcuts, %g seconds, path len %g\n",0.0,ns,timer.ElapsedTime(),path.GetTotalTime());
    }
  }
}

void TestDynamicShortcutting(SingleRobotCSpace& freeSpace,const ParabolicRamp::DynamicPath& porig)
{
  Real tol = freeSpace.settings->robotSettings[freeSpace.index].collisionEpsilon;
  ParabolicRamp::DynamicPath path = porig;
  CSpaceFeasibilityChecker feas(&freeSpace);
  ParabolicRamp::RampFeasibilityChecker checker(&feas,tol);
  Timer timer;
  int ns=0;
  timer.Reset();
  ns = path.OnlineShortcut(0.1,0.01,checker);
  printf("Dynamic shortcutting with window %g made %d shortcuts, took %g seconds\n",0.1,ns,timer.ElapsedTime());

  path = porig;
  timer.Reset();
  ns=path.OnlineShortcut(0.5,0.01,checker);
  printf("Dynamic shortcutting with window %g made %d shortcuts, took %g seconds\n",0.5,ns,timer.ElapsedTime());

  path = porig;
  timer.Reset();
  ns=path.OnlineShortcut(1.0,0.01,checker);
  printf("Dynamic shortcutting with window %g made %d shortcuts, took %g seconds\n",1.0,ns,timer.ElapsedTime());

  path = porig;
  timer.Reset();
  ns=path.OnlineShortcut(2.0,0.01,checker);
  printf("Dynamic shortcutting with window %g made %d shortcuts, took %g seconds\n",2.0,ns,timer.ElapsedTime());
}


} //namespace Klampt