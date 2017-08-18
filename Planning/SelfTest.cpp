#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "SelfTest.h"
#include "RampCSpace.h"
#include <KrisLibrary/Timer.h>
#include <time.h>

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
    if(!res) LOG4CXX_WARN(KrisLibrary::logger(),"Warning, path is not feasible\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"Time to check path feasibility: "<<timer.ElapsedTime());
    timer.Reset();
    
    LOG4CXX_INFO(KrisLibrary::logger(),"Planned a path successfully!\n");
    
    //shortcuts
    ParabolicRamp::DynamicPath path;
    path.Init(cspace->robot.velMax,cspace->robot.accMax);  
    vector<ParabolicRamp::Vector> vmilestones(milestones.size());
    copy(milestones.begin(),milestones.end(),vmilestones.begin());
    path.SetMilestones(vmilestones);
    
    //seed the RNG
    srand(time(NULL));
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
    LOG4CXX_INFO(KrisLibrary::logger(),"Performed "<<ns<<" shortcuts, reduced time from "<<torig<<" to "<<tnew);
    LOG4CXX_INFO(KrisLibrary::logger(),"**** Timing ****\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"Time to plan: "<<tPlanning);
    LOG4CXX_INFO(KrisLibrary::logger(),"Sections of "<<maxShortcutIters/10);
    for(int i=0;i<10;i++) 
      LOG4CXX_INFO(KrisLibrary::logger(),"Time to compute "<<tShortcut[i]<<", path duration "<<durShortcut[i]);

    timer.Reset();
    for(size_t i=0;i<path.ramps.size();i++) {
      if(!checker.Check(path.ramps[i])) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Shortcutted path ramp "<<i);
	if(KrisLibrary::logger()->isEnabledFor(log4cxx::Level::ERROR_INT)) getchar();
      }
    }
    /*
      LOG4CXX_INFO(KrisLibrary::logger(),"Regular shortcutting\n");
      for(int trial = 0; trial < 5; trial++) {
      path = porig;
      timer.Reset();
      path.Shortcut(cspace,tol,Inf,maxShortcutIters);
      LOG4CXX_INFO(KrisLibrary::logger(),"Time to compute "<<timer.ElapsedTime()<<", path duration "<<path.GetTotalTime());
      }
    */

    LOG4CXX_INFO(KrisLibrary::logger(),"Dynamic shortcutting\n");
    /* for(int trial = 0;trial < 5; trial++)*/ {
      path = porig;
      timer.Reset();
      ns = path.OnlineShortcut(0.0,0.01,checker);
      LOG4CXX_INFO(KrisLibrary::logger(),"Window "<<0.0<<": "<<ns<<" shortcuts, "<<timer.ElapsedTime()<<" seconds, path len "<<path.GetTotalTime());
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
  LOG4CXX_INFO(KrisLibrary::logger(),"Dynamic shortcutting with window "<<0.1<<" made "<<ns<<" shortcuts, took "<<timer.ElapsedTime());

  path = porig;
  timer.Reset();
  ns=path.OnlineShortcut(0.5,0.01,checker);
  LOG4CXX_INFO(KrisLibrary::logger(),"Dynamic shortcutting with window "<<0.5<<" made "<<ns<<" shortcuts, took "<<timer.ElapsedTime());

  path = porig;
  timer.Reset();
  ns=path.OnlineShortcut(1.0,0.01,checker);
  LOG4CXX_INFO(KrisLibrary::logger(),"Dynamic shortcutting with window "<<1.0<<" made "<<ns<<" shortcuts, took "<<timer.ElapsedTime());

  path = porig;
  timer.Reset();
  ns=path.OnlineShortcut(2.0,0.01,checker);
  LOG4CXX_INFO(KrisLibrary::logger(),"Dynamic shortcutting with window "<<2.0<<" made "<<ns<<" shortcuts, took "<<timer.ElapsedTime());
}

