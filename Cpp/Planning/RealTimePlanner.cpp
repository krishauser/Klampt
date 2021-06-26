#include "RealTimePlanner.h"
#include "RealTimeIKPlanner.h"
#include "RealTimeRRTPlanner.h"
#include "Interface/RobotInterface.h"
#include <KrisLibrary/robotics/IKFunctions.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math/differentiation.h>
#include <KrisLibrary/optimization/Minimization.h>
#include <string.h>
#include <typeinfo>

namespace Klampt {

//extracts IK problems from the plannerobjective
void Extract(PlannerObjectiveBase* obj,RobotModel* robot,vector<IKGoal>& ikproblem,vector<pair<int,Real> >& joint_constraints)
{
  if(typeid(*obj) == typeid(CompositeObjective)) {
    CompositeObjective* cobj = dynamic_cast<CompositeObjective*>(obj);
    for(size_t i=0;i<cobj->components.size();i++)
      Extract(cobj->components[i].get(),robot,ikproblem,joint_constraints);
  }
  else if(typeid(*obj) == typeid(CartesianObjective)) {
    CartesianObjective* cobj = dynamic_cast<CartesianObjective*>(obj);
    ikproblem.push_back(cobj->ikGoal);
  }
  else if(typeid(*obj) == typeid(IKObjective)) {
    IKObjective* cobj = dynamic_cast<IKObjective*>(obj);
    ikproblem.push_back(cobj->ikGoal);
  }
  else if(typeid(*obj) == typeid(ConfigObjective)) {
    ConfigObjective* cobj = dynamic_cast<ConfigObjective*>(obj);
    if(!joint_constraints.empty())
      fprintf(stderr,"Cannot support multiple configuration constraints yet\n");
    assert(joint_constraints.empty());
    joint_constraints.resize(cobj->qgoal.n);
    for(int i=0;i<cobj->qgoal.n;i++) {
      joint_constraints[i].first = (int)i;
      joint_constraints[i].second = cobj->qgoal(i); 
    }
  }
  else {
    fprintf(stderr,"Cant support objective type %s\n",typeid(*obj).name());
    ikproblem.clear();
  }
}

bool Optimize(shared_ptr<PlannerObjectiveBase> obj,RobotModel* robot,int iters,Real tol)
{
  vector<IKGoal> ikproblem;
  vector<pair<int,Real> > joint_constraints;
  Extract(obj.get(),robot,ikproblem,joint_constraints);
  if(!joint_constraints.empty()) {
    if(!ikproblem.empty())
      fprintf(stderr,"Cannot support IK and configuration objectives simultaneously yet\n");
    assert(joint_constraints.size()==robot->links.size());
    if(joint_constraints.size()!=robot->links.size())
      fprintf(stderr,"Cannot support partial joint constraints yet\n");
    assert(joint_constraints.size()==robot->links.size());
    for(size_t i=0;i<joint_constraints.size();i++) {
      int k=joint_constraints[i].first;
      robot->q(k) = Clamp(joint_constraints[i].second,robot->qMin[k],robot->qMax[k]);
    }
    return true;
  }

  //only ik constraints
  bool res=SolveIK(*robot,ikproblem,tol,iters,0);
  return res;
}

//-1: cutoff hit
//0: not visible
//1: visible
int IsVisible(const EdgePlannerPtr& e,Timer& timer,Real cutoff,int numStepsPerCheck=5)
{
  return (e->IsVisible()?1:0);
}

RealTimePlanner::RealTimePlanner()
  :protocol(ExponentialBackoff),currentSplitTime(0.1),currentPadding(0.01),
   currentExternalPadding(0.01),
   //protocol(Constant),currentSplitTime(0.1),currentPadding(0.05),
   ///protocol(Constant),currentSplitTime(0.5),currentPadding(0.05),
  maxPadding(5.0)
{
  pathStartTime = 0;
  cognitiveMultiplier = 1.0;
  acceptTimeOverruns = false;
}

RealTimePlanner::~RealTimePlanner()
{
}


void RealTimePlanner::SetSpace(SingleRobotCSpace* space)
{
  if(planner) {
    planner->Init(space,&space->robot,space->settings);
  }
  else {
    fprintf(stderr,"RealTimePlanner::SetSpace: warning, underlying planner not set\n");
  }
}

shared_ptr<PlannerObjectiveBase> RealTimePlanner::Objective() const
{
  if(!planner) return NULL;
  return planner->goal; 
}

void RealTimePlanner::Reset(shared_ptr<PlannerObjectiveBase> newgoal)
{
  if(!planner) return;

  if(protocol == ExponentialBackoff) {
    if(planner->goal && newgoal) {  //reduce timestep depending on the delta from the prior goal
      Real d=planner->goal->Delta(newgoal.get());
      assert(d >= 0);
      //TODO: figure the correlation between task changes and likelihood of
      //success with a shorter planning period
      //works well for cartesian tracking tasks
      //Real u=Exp(-0.1*d);
      Real u=Exp(-2*d);
      //works well for position tasks
      //Real u=Exp(-2*d);
      currentSplitTime = currentSplitTime*u + (1.0-u)*currentPadding;
      fprintf(planner->flog,"Movement with distance %g, updating split time to %g\n",d,currentSplitTime); 
    }
    else {
      //reset the split time
      currentSplitTime = 0.1;
    }
  }
  planner->goal = newgoal;
}

//default implementation: fix the split time, call PlanFrom
//returns true if the path changed and planTime < splitTime
bool RealTimePlanner::PlanUpdate(Real tglobal,Real& splitTime,Real& planTime)
{
  if(!planner) {
    splitTime = planTime = 0;
    return false;
  }
  assert(!currentPath.ramps.empty());
  assert(currentPath.IsValid());
  if(currentSplitTime < currentPadding+currentExternalPadding+0.001) {
    currentSplitTime=currentPadding+currentExternalPadding+0.001;
  }

  //advance the current path time
  ParabolicRamp::DynamicPath before;
  if(tglobal > pathStartTime) {
    ParabolicRamp::DynamicPath updatedPath;
    printf("split at %g for path of duration %g\n",tglobal-pathStartTime,currentPath.GetTotalTime());
    currentPath.Split(tglobal-pathStartTime,before,updatedPath);
    currentPath = updatedPath;
    pathStartTime = tglobal;
  }
  else if(tglobal < pathStartTime) {
    FatalError("RealTimePlanner::PlanUpdate: Time is not moving forward!");
  }

  //Here's where the planning takes place
  Timer timer;
  splitTime = currentSplitTime;
  ParabolicRamp::DynamicPath after;
  currentPath.Split(splitTime,before,after);
  Assert(before.IsValid());
  Assert(after.IsValid());
  assert(FuzzyEquals(before.GetTotalTime(),splitTime));
  //printf("Split took time %g\n",timer.ElapsedTime());
  fprintf(planner->flog,"***** Planning for time %gs (split %g, padding %g, ext %g)*********\n",(currentSplitTime-currentPadding-currentExternalPadding)*cognitiveMultiplier,currentSplitTime,currentPadding,currentExternalPadding);

  timer.Reset();
  planner->stopPlanning = false;
  planner->SetTime(currentSplitTime);
  int res=planner->PlanFrom(after,(currentSplitTime-currentPadding-currentExternalPadding)*cognitiveMultiplier);
  planTime = timer.ElapsedTime()/cognitiveMultiplier;

  fprintf(planner->flog,"***** Planning took time %gs *********\n",timer.ElapsedTime());
  //printf("Planning took time %g\n",planTime);
  //collect statistics
  if(res==DynamicMotionPlannerBase::Failure) planFailTimeStats.collect(planTime);
  else if(res==DynamicMotionPlannerBase::Success) planSuccessTimeStats.collect(planTime);
  else if(res==DynamicMotionPlannerBase::Timeout) planTimeoutTimeStats.collect(planTime);

  //now decide whether to update the path 
  bool updatePath = false;
  if(res==DynamicMotionPlannerBase::Success) {
    updatePath = true;
    //if the planning time exceeds the split time, disallow it
    if(!acceptTimeOverruns && planTime > currentSplitTime-currentExternalPadding) {
      fprintf(planner->flog,"Planner returned Success, but time budget overrun\n");
      updatePath = false;
    }
  }
  else if(res==DynamicMotionPlannerBase::Timeout) {
    //TEMP: RRT semantics indicate that a better path may have been found
    updatePath = true;
    //if the planning time exceeds the split time, disallow it
    if(!acceptTimeOverruns && planTime > currentSplitTime-currentExternalPadding) {
      fprintf(planner->flog,"Planner returned Timeout, time budget overrun\n");
      updatePath = false;
    }
  }
  else {
    fprintf(planner->flog,"Planner returned Failure\n");
  }

  //update currentSplitTime and currentPadding
  if(protocol == Constant) { // do nothing 
  }
  else if(protocol == ExponentialBackoff) {
    //update currentPadding
    if(planTime > splitTime-currentExternalPadding) {
      currentSplitTime += currentPadding;
      currentPadding *= 2.0;
      //cap the padding at the max
      if(currentPadding > maxPadding)
	currentPadding = maxPadding;
      //if the current padding is exceedingly low, boost it up
      //so that it doesnt have to adapt by doubling
      if(res == DynamicMotionPlannerBase::Timeout && currentPadding < planTime-(currentSplitTime-currentExternalPadding)) 
	currentPadding = planTime-(currentSplitTime-currentExternalPadding);
      fprintf(planner->flog,"Not enough padding, setting padding to %g\n",currentPadding);
    }
    else if(planTime+currentPadding*0.5 < splitTime-currentExternalPadding) {
      currentPadding *= 0.5;
      fprintf(planner->flog,"Too much padding, setting padding to %g\n",currentPadding);
    }
    if(currentPadding < 0.001) currentPadding=0.001;
      
    //update currentSplitTime
    if(res == DynamicMotionPlannerBase::Success) {
      currentSplitTime *= 0.33;
      fprintf(planner->flog,"Success, setting split time to %g\n",currentSplitTime); 
    }
    else if(res == DynamicMotionPlannerBase::Timeout) {
      currentSplitTime *= 2.0;
      fprintf(planner->flog,"Failure, setting split time to %g\n",currentSplitTime); 
    }
  }
  else if(protocol == Learning) {
    //TODO
    /*
      if(planSuccessTimeStats.number()==0)
      currentSplitTime *= 2.0;
      else
      currentSplitTime = 
    */
  }

  if(updatePath) {
    for(size_t i=0;i<after.ramps.size();i++)
      Assert(after.ramps[i].IsValid());
    for(size_t i=0;i<before.ramps.size();i++)
      Assert(before.ramps[i].IsValid());

    //debugging path
    if(!Vector(after.ramps.front().x0).isEqual(Vector(before.ramps.back().x1),1e-5)) {
      cout<<"Updated path start configuration mismatch"<<endl;
      cout<<Vector(before.ramps.back().x1)<<endl;
      cout<<Vector(after.ramps.front().x0)<<endl;
    }
    if(!Vector(after.ramps.front().dx0).isEqual(Vector(before.ramps.back().dx1),1e-5)) {
      cout<<"Updated path start velocity mismatch"<<endl;
      cout<<Vector(before.ramps.back().dx1)<<endl;
      cout<<Vector(after.ramps.front().dx0)<<endl;
    }
    Assert(Vector(after.ramps.front().x0).isEqual(Vector(before.ramps.back().x1),1e-5));
    Assert(Vector(after.ramps.front().dx0).isEqual(Vector(before.ramps.back().dx1),1e-5));
    //minor corrections
    after.ramps.front().x0 = before.ramps.back().x1;
    after.ramps.front().dx0 = before.ramps.back().dx1;
    for(size_t i=0;i<after.ramps.front().ramps.size();i++) {
      after.ramps.front().ramps[i].x0=after.ramps.front().x0[i];
      after.ramps.front().ramps[i].dx0=after.ramps.front().dx0[i];
    }
    Assert(after.accMax == before.accMax);
    Assert(after.velMax == before.velMax);

    //update current path
    if(sendPathCallback) {
      if(sendPathCallback->Send(tglobal,splitTime,after)) {
	currentPath = before;
	currentPath.Concat(after);
      }
      else {
	//Send failed for some reason -- now expand the padding
	MarkSendFailure();
	fprintf(planner->flog,"Send failed! Increased external padding to %g, split time %g\n",currentExternalPadding,currentSplitTime); 
      }
    }
    else {
      //No send callback set, assuming path update should just be done internally
      currentPath = before;
      currentPath.Concat(after);
    }
    Assert(currentPath.IsValid());
    Assert(Vector(currentPath.ramps.back().dx1).isZero());
    return true;
  }
  else
    return false;
}

void RealTimePlanner::MarkSendFailure()
{
  currentSplitTime += currentExternalPadding;
  currentExternalPadding = currentExternalPadding*2.0;
  if(currentExternalPadding > 1.0) {
    fprintf(planner->flog,"Warning... multiple send failures, padding now > 1s.\n");
    fprintf(planner->flog,"Please debug the communication system.\n");
    currentExternalPadding = 1.0;
  }
}

void RealTimePlanner::SetConstantPath(const Config& q)
{
  currentPath.ramps.resize(1);
  currentPath.ramps[0].SetConstant(q);
  if(planner) {
    currentPath.xMin = planner->qMin;
    currentPath.xMax = planner->qMax;
    currentPath.velMax = planner->velMax;
    currentPath.accMax = planner->accMax;
  }
  else {
    printf("RealTimePlanner: Warning, dynamic limits are not set yet\n");
  }
}

void RealTimePlanner::SetCurrentPath(Real tglobal,const ParabolicRamp::DynamicPath& path)
{
  pathStartTime = tglobal;
  currentPath = path;
}





/** @brief Shared data structure for a multithreaded real time planner.
 * The planner must have a RealTimePlannerDataSender as a sendPathCallback.
 */
struct RealTimePlannerData
{
  Mutex mutex;
  RealTimePlanner* planner;
  bool resetStartConfig;      //(in) true if the start configuration should be reset
  Config startConfig;         //(in) the start config
  shared_ptr<PlannerObjectiveBase> objective;   //(in) the planning objective, can be NULL
  bool active;             //(in) set this to false to quit
  bool pause;              //(in) set this to true to pause the planner
  Real globalTime;         //(in) time measured in calling thread
  Real startPlanTime;       //(out) time of planning
  bool planning;           //(out) currently planning

  bool pathRefresh;         //(in/out) whether to refresh the path
  Real tcut;                //(out) the path cut time, relative to startPlanTime
  ParabolicRamp::DynamicPath path;  //(out) the path to splice in 
  bool pathRefreshSuccess;  //(in) whether the execution thread read in the path successfully
};

/** @brief For use with a multithreaded RealTimePlannerBase
 */
class RealTimePlannerDataSender : public SendPathCallbackBase
{
public:
  RealTimePlannerDataSender(RealTimePlannerData* data);
  virtual bool Send(Real tplanstart,Real tcut,const ParabolicRamp::DynamicPath& path);
  RealTimePlannerData* data;
};

RealTimePlannerDataSender::RealTimePlannerDataSender(RealTimePlannerData* _data)
  :data(_data)
{}

bool RealTimePlannerDataSender::Send(Real tplanstart,Real tcut,const ParabolicRamp::DynamicPath& path)
{
  {
    ScopedLock lock(data->mutex);
    Assert(data->startPlanTime == tplanstart);
    data->pathRefresh = true;
    data->tcut = tcut;
    data->path = path;
  }  
  int iters=0;
  while(true) {
    ThreadSleep(0.001);
    {
      ScopedLock lock(data->mutex);
      if(!data->pathRefresh) {  //this signals that the calling thread picked up the data
	bool res = data->pathRefreshSuccess;
	return res; //unlocks mutex
      }
    }//unlocks mutex
    iters ++ ;
    if(iters % 1000 == 0) {
      fprintf(stderr,"Data sender is waiting for a long time... Did you forget to call RealTimePlanningThread.SendUpdate()?\n");
    }
  }
}  

void* planner_thread_func(void * ptr)
{
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(ptr);
  //read initial configuration
  {
    ScopedLock(data->mutex);
    if(data->resetStartConfig) {
      data->planner->SetConstantPath(data->startConfig);
      data->resetStartConfig = false;
    }
  }
  //FOR SOME REASON, LOGGING FAILS UNLESS DONE HERE
  //data->planner->LogBegin();

  while (true) {
    //first wait for a lock
    bool start=false;
    Real startTime;
    {
      ScopedLock(data->mutex);

      //update general status
      data->planning = false;

      //parse the data
      if(!data->active) {
	printf("Planner_thread_func: quitting\n");
	//quit
	return NULL;  //unlocks the mutex
      }
      if(!data->planner || !data->planner->planner) {
	//no planner set
	ThreadSleep(0.1);
	continue;
      }
      assert(data->pathRefresh == false);
      if(data->resetStartConfig == true) {
	printf("Planning thread: resetting start configuration\n");
	data->planner->SetConstantPath(data->startConfig);
	data->resetStartConfig = false;
      }
      if(data->objective) {
	if(!data->pause) {  //don't start if pause is true
	  start=true;
	  startTime = data->startPlanTime = data->globalTime;
	  data->planning=true;
	}
      }
      if(data->objective != data->planner->planner->goal) {
	printf("Planning thread: changing objective\n");
	if(data->objective) {
	  //planner takes ownership
	  data->planner->Reset(data->objective);
	}
	else {
	  data->planner->Reset(NULL);
	}
      }
    }  //unlocks the mutex

    if(start) {
      //do the planning -- the callback will output the result to the
      //UI thread
      Real splitTime, planTime;
      bool res = data->planner->PlanUpdate(startTime, splitTime, planTime);
      
      printf("Planning thread: plan result %d\n",(int)res);    
      ThreadYield();
    }
    else {
      //printf("Planning thread waiting for an objective...\n");
      ThreadSleep(0.01);
    }
  }
  return NULL;
}

RealTimePlanningThread::RealTimePlanningThread()
{
  internal = new RealTimePlannerData;
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(internal);
  data->active = false;
  data->pause = false;
  data->resetStartConfig = false;
  data->planner = NULL;
  data->planning = false;
  data->pathRefresh = false;
}

RealTimePlanningThread::~RealTimePlanningThread()
{
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(internal);
  if(data->active)
    Stop();
  delete data;
}

void RealTimePlanningThread::SetStartConfig(const Config& qstart)
{
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(internal);
  if(!this->planner)
    SetPlanner(make_shared<RealTimePlanner>());
  else {
    StopPlanning();
    //poll to wait for planning to stop
    int iters=0;
    while(IsPlanning()) {
      ThreadSleep(0.01);
      iters++;
      if(iters % 100 == 0)
	printf("SetStartConfig... Waiting for planner to stop...\n");
    }
  }
  ScopedLock lock(data->mutex);
  data->startConfig = qstart;
  data->resetStartConfig = true;

  if(this->planner->currentPath.xMin.empty() && this->planner->planner) {
    //initialize dynamic bounds
    this->planner->currentPath.xMin = this->planner->planner->qMin;
    this->planner->currentPath.xMax = this->planner->planner->qMax;
    this->planner->currentPath.velMax = this->planner->planner->velMax;
    this->planner->currentPath.accMax = this->planner->planner->accMax;
  }

  ResumePlanning();
}

void RealTimePlanningThread::SetCSpace(SingleRobotCSpace* space)
{
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(internal);
  if(!this->planner)
    SetPlanner(make_shared<RealTimePlanner>());
  else {
    StopPlanning();
    int iters=0;
    while(IsPlanning()) {
      ThreadSleep(0.01);
      iters++;
      if(iters % 100 == 0)
	printf("SetCSpace... Waiting for planner to stop...\n");
    }
  }
  ScopedLock lock(data->mutex);
  this->planner->SetSpace(space);
  if(this->planner->planner && this->planner->currentPath.xMin.empty()) {
    //initialize dynamic bounds
    this->planner->currentPath.xMin = this->planner->planner->qMin;
    this->planner->currentPath.xMax = this->planner->planner->qMax;
    this->planner->currentPath.velMax = this->planner->planner->velMax;
    this->planner->currentPath.accMax = this->planner->planner->accMax;
  }

  ResumePlanning();
}

void RealTimePlanningThread::ResetCurrentPath(Real tglobal,const ParabolicRamp::DynamicPath& path)
{
  FatalError("ResetCurrentPath not implemented yet");
}

void RealTimePlanningThread::SetObjective(shared_ptr<PlannerObjectiveBase> newgoal)
{
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(internal);
  //set the objective function
  ScopedLock lock(data->mutex);
  data->objective = newgoal;
 }

PlannerObjectiveBase* RealTimePlanningThread::GetObjective() const
{
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(internal);
  return data->objective.get();
}

void RealTimePlanningThread::SetPlanner(const shared_ptr<DynamicMotionPlannerBase>& planner)
{
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(internal);
  if(!this->planner) SetPlanner(make_shared<RealTimePlanner>());
  else {
    StopPlanning();
    int iters=0;
    while(IsPlanning()) {
      ThreadSleep(0.01);
      iters++;
      if(iters % 100 == 0)
	printf("SetCSpace... Waiting for planner to stop...\n");
    }
  }
  ScopedLock lock(data->mutex);
  this->planner->planner = planner;
  if(this->planner->currentPath.xMin.empty()) {
    //initialize dynamic bounds
    this->planner->currentPath.xMin = this->planner->planner->qMin;
    this->planner->currentPath.xMax = this->planner->planner->qMax;
    this->planner->currentPath.velMax = this->planner->planner->velMax;
    this->planner->currentPath.accMax = this->planner->planner->accMax;
  }

  ResumePlanning();
}
void RealTimePlanningThread::SetPlanner(const shared_ptr<RealTimePlanner>& planner)
{
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(internal);
  this->planner = planner;
  if(planner)
    this->planner->sendPathCallback.reset(new RealTimePlannerDataSender(data));
  ScopedLock lock(data->mutex);
  data->planner = this->planner.get();
}

bool RealTimePlanningThread::Start()
{
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(internal);
  if(data->active) return true;
  data->active = true;
  data->pause = false;
  data->globalTime = 0;
  data->pathRefresh = false;
  printf("Creating planning thread\n");
  thread = ThreadStart(planner_thread_func,data);
  return true;
}

bool RealTimePlanningThread::IsPlanning()
{
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(internal);
  //do we need to lock the mutex?  probably not.
  if(!data->planner) return false;
  if(!data->planner->planner) return false;
  return data->planning;
}

void RealTimePlanningThread::PausePlanning()
{
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(internal);
  data->pause = true;
}

void RealTimePlanningThread::BreakPlanning()
{
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(internal);
  if(data->planner)
    planner->StopPlanning();
}

void RealTimePlanningThread::StopPlanning()
{
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(internal);
  if(data->planner)
    planner->StopPlanning();
  data->pause = true;
}

void RealTimePlanningThread::ResumePlanning()
{
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(internal);
  data->pause = false;
}

void RealTimePlanningThread::Stop()
{
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(internal);
  if(!data->active) return;

  printf("Stopping planning thread\n");
  if(data->planner)
  {
    ScopedLock lock(data->mutex);
    planner->StopPlanning();
    data->active = false;
  }
  ThreadJoin(thread);
}

bool RealTimePlanningThread::HasUpdate()
{
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(internal);
  return data->pathRefresh;
}

bool RealTimePlanningThread::SendUpdate(MotionQueueInterface* robotInterface)
{
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(internal);
  //see if the planning thread has a plan update
  if(!data->pathRefresh) {
    data->globalTime = robotInterface->GetCurTime();
    return false;
  }

  //if so, mark that it's refreshed and send the path
  ScopedLock lock(data->mutex);
  printf("Exec thread: SendUpdate: Refreshing path...\n");
  data->pathRefresh = false;
  MotionQueueInterface::MotionResult res = robotInterface->SendPathImmediate(data->startPlanTime+data->tcut,data->path);
  Real t=robotInterface->GetCurTime();
  if(res == MotionQueueInterface::Success) {
    data->pathRefreshSuccess = true;
    printf("Exec thread: Plan+send successful, split %g, delay %g\n",data->tcut,t - data->startPlanTime);
    data->globalTime = robotInterface->GetCurTime();
    return true;
  }
  else {
    data->pathRefreshSuccess = false;
    printf("Exec thread: Plan+send overrun, split %g, delay %g\n",data->tcut,t - data->startPlanTime);
    data->globalTime = robotInterface->GetCurTime();
    return false;
  }
}

Real RealTimePlanningThread::ObjectiveValue()
{
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(internal);
  ScopedLock lock(data->mutex);
  if(!data->pathRefresh) return Inf;
  if(!data->objective) return 0;
  return data->objective->PathCost(data->path,data->startPlanTime+data->tcut);
}






DynamicMotionPlannerBase::DynamicMotionPlannerBase()
  :robot(NULL),settings(NULL),cspace(NULL),tstart(0)
{
  flog = stdout;
}

DynamicMotionPlannerBase::~DynamicMotionPlannerBase()
{
  LogEnd();
}

bool DynamicMotionPlannerBase::LogBegin(const char* fn)
{
  return true;
  if(flog != stdout)
    fclose(flog);
  flog = fopen(fn,"a");
  if(!flog) {
    cout<<"DynamicMotionPlannerBase: Couldn't open log to file "<<fn<<endl;
    getchar();
    flog = stdout;
    return false;
  }
  fprintf(flog,"Log begin...\n");
  return true;
}

bool DynamicMotionPlannerBase::LogEnd()
{
  if(flog != stdout) {
    fclose(flog);
    flog = stdout;
  }
  return true;
}

void DynamicMotionPlannerBase::Init(CSpace* _space,RobotModel* _robot,WorldPlannerSettings* _settings)
{
  cspace = _space;
  robot = _robot;
  settings = _settings;
  tstart = 0;
  stopPlanning = false;
  SetDefaultLimits();
}

void DynamicMotionPlannerBase::SetGoal(shared_ptr<PlannerObjectiveBase> newgoal)
{
  goal = newgoal;
}

void DynamicMotionPlannerBase::SetTime(Real _tstart)
{
  tstart = _tstart; 
}

void DynamicMotionPlannerBase::SetDefaultLimits()
{
  SetLimits();
}

void DynamicMotionPlannerBase::SetLimits(Real qScale,Real vScale,Real aScale)
{
  qMin = robot->qMin + (1.0-qScale)*0.5*(robot->qMax-robot->qMin);
  qMax = robot->qMax - (1.0-qScale)*0.5*(robot->qMax-robot->qMin);
  velMax = robot->velMax*vScale;
  accMax = robot->accMax*aScale;
}


bool DynamicMotionPlannerBase::StopPlanning()
{
  stopPlanning = true;
  return true;
}

int DynamicMotionPlannerBase::Shortcut(ParabolicRamp::DynamicPath& path,Real timeLimit)
{
  if(timeLimit <= 0) return 0;
  path.xMin = qMin;
  path.xMax = qMax;
  path.velMax = velMax;
  path.accMax = accMax;
  Timer timer;
  //do shortcutting with the remaining time
  int num=0;
  Real pathEpsilon = settings->robotSettings[0].collisionEpsilon;
  CSpaceFeasibilityChecker feas(cspace);
  ParabolicRamp::RampFeasibilityChecker checker(&feas,pathEpsilon);
  while(timer.ElapsedTime() < timeLimit && !stopPlanning) {
    Real t1 = Sqr(Rand())*path.GetTotalTime();
    Real t2 = Sqr(Rand())*path.GetTotalTime();
    if(path.TryShortcut(t1,t2,checker))
      num++;
  }
  return num;
}

inline bool InBounds(const vector<Real>& x,const vector<Real>& bmin,const vector<Real>& bmax)
{
  Assert(x.size()==bmin.size());
  Assert(x.size()==bmax.size());
  for(size_t i=0;i<x.size();i++)
    if(x[i] < bmin[i] || x[i] > bmax[i]) return false;
  return true;
}


int DynamicMotionPlannerBase::SmartShortcut(Real tstart,ParabolicRamp::DynamicPath& path,Real timeLimit) 
{
  if(goal->PathInvariant()) return Shortcut(path,timeLimit);
  if(timeLimit <= 0) return 0;

  Timer timer;
  //do shortcutting with the remaining time
  int num=0;
  Real pathEpsilon = settings->robotSettings[0].collisionEpsilon;
  CSpaceFeasibilityChecker feas(cspace);
  ParabolicRamp::RampFeasibilityChecker checker(&feas,pathEpsilon);
  ParabolicRamp::DynamicPath intermediate;
  ParabolicRamp::ParabolicRampND leadin,leadout;
  path.xMin = intermediate.xMin = qMin;
  path.xMax = intermediate.xMax = qMax;
  path.velMax = intermediate.velMax = velMax;
  path.accMax = intermediate.accMax = accMax;

  vector<Real> startTimes(path.ramps.size()+1);
  vector<Real> accumCosts(path.ramps.size()+1);
  accumCosts[0] = 0.0;
  startTimes[0] = tstart;
  for(size_t i=0;i<path.ramps.size();i++) {
    accumCosts[i+1] = accumCosts[i]+goal->IncrementalCost(startTimes[i],path.ramps[i]);
    startTimes[i+1] = startTimes[i]+path.ramps[i].endTime;
  }
  Real terminalCost = goal->TerminalCost(startTimes.back(),Vector(path.ramps.back().x1),Vector(path.ramps.back().dx1));

  bool differentialTimeInvariant = goal->DifferentialTimeInvariant();
  bool terminalTimeInvariant = goal->TerminalTimeInvariant();
  int i1,i2;
  Real u1,u2;
  vector<Real> x0,x1,dx0,dx1;
  while(timer.ElapsedTime() < timeLimit) {
    if(stopPlanning) return num;
    Real t1 = Sqr(Rand())*(startTimes.back()-startTimes.front());
    Real t2 = Sqr(Rand())*(startTimes.back()-startTimes.front());
    if(t1 > t2) swap(t1,t2);

    i1 = path.GetSegment(t1,u1);
    i2 = path.GetSegment(t2,u2);
    if(i1 == i2) continue;
    assert(i1 >= 0 && i1 < (int)path.ramps.size());
    assert(i2 >= 0 && i2 < (int)path.ramps.size());
    path.ramps[i1].Evaluate(u1,x0);
    path.ramps[i2].Evaluate(u2,x1);
    path.ramps[i1].Derivative(u1,dx0);
    path.ramps[i2].Derivative(u2,dx1);
    //could be some numerical error
    if(!InBounds(x0,intermediate.xMin,intermediate.xMax)) continue;
    if(!InBounds(x1,intermediate.xMin,intermediate.xMax)) continue;
    if(!intermediate.SolveMinTime(x0,dx0,x1,dx1)) continue;
    //path time reduction t2-t1
    Real shortcutCost = goal->IncrementalCost(t1,intermediate);
    leadin = path.ramps[i1];
    leadin.TrimBack(path.ramps[i1].endTime-u1);
    leadin.x1 = intermediate.ramps.front().x0;
    leadin.dx1 = intermediate.ramps.front().dx0;
    leadout = path.ramps[i2];
    leadout.TrimFront(u2);
    leadout.x0 = intermediate.ramps.back().x1;
    leadout.dx0 = intermediate.ramps.back().dx1;
    Real leadinCost = goal->IncrementalCost(startTimes[i1],leadin);
    Real leadoutCost = goal->IncrementalCost(t2,leadout);
    Real newTerminalCost = terminalCost;
    Real deltat = intermediate.GetTotalTime() - (t2-t1);

    bool doShortcut = false;
    if(!terminalTimeInvariant) {  //then measure the new cost of the goal
      newTerminalCost = goal->TerminalCost(startTimes.back()+deltat,Vector(path.ramps.back().x1),Vector(path.ramps.back().dx1));
    }
    if(differentialTimeInvariant) { //then it doesn't matter what happens after the intermediate segment
      //prior cost of intermediate segments
      Real cold = accumCosts[i2+1]-accumCosts[i1]-leadinCost-leadoutCost;
      if(shortcutCost+newTerminalCost < cold+terminalCost) doShortcut = true;
    }
    else { //need to count everything that happens in the intermediate segments
      Real t2new = t2 + deltat;
      Real adjCost = 0.0;
      adjCost = goal->IncrementalCost(t2new,leadout);
      Real tnew = t2new+leadout.endTime;
      for(int i=i2+1;i<(int)path.ramps.size();i++) {
	adjCost += goal->IncrementalCost(tnew,leadout);	
	tnew += path.ramps[i].endTime;
      }
      //prior cost from t1 to end
      Real cold = accumCosts.back()-accumCosts[i1]-leadinCost;
      if(shortcutCost+adjCost+newTerminalCost < cold+terminalCost) doShortcut = true;
    }
    if(!doShortcut) continue;

    bool feasible = true;
    for(size_t i=0;i<intermediate.ramps.size();i++) {
      if(!checker.Check(intermediate.ramps[i])) {
	feasible = false;
	break;
      }
    }
    if(!feasible) continue;

    num++;
    //perform shortcut
    //crop i1 and i2
    path.ramps[i1] = leadin;
    path.ramps[i2] = leadout;
    //replace intermediate ramps with test
    path.ramps.erase(path.ramps.begin()+i1+1,path.ramps.begin()+i2);
    path.ramps.insert(path.ramps.begin()+i1+1,intermediate.ramps.begin(),intermediate.ramps.end());
  
    //check for consistency
    for(size_t i=0;i+1<path.ramps.size();i++) {
      assert(path.ramps[i].x1 == path.ramps[i+1].x0);
      assert(path.ramps[i].dx1 == path.ramps[i+1].dx0);
    }

    //adjust costs
    terminalCost = newTerminalCost;
    if(differentialTimeInvariant) {       
    }
    else {
    }
    //TODO: faster method that avoids recomputing costs
    startTimes.resize(path.ramps.size()+1);
    accumCosts.resize(path.ramps.size()+1);
    accumCosts[0] = 0.0;
    startTimes[0] = tstart;
    for(size_t i=0;i<path.ramps.size();i++) {
      accumCosts[i+1] = accumCosts[i]+goal->IncrementalCost(startTimes[i],path.ramps[i]);
      startTimes[i+1] = startTimes[i]+path.ramps[i].endTime;
    }
  }
  return num;
}

bool DynamicMotionPlannerBase::GetMilestoneRamp(const Config& q0,const Vector& dq0,const Config& q1,ParabolicRamp::DynamicPath& ramp) const
{
  ramp.Init(velMax,accMax);
  ramp.SetJointLimits(qMin,qMax);
  vector<ParabolicRamp::Vector> x(2),dx(2);
  x[0] = q0;
  x[1] = q1;
  dx[0] = dq0;
  dx[1].resize(q1.n,0.0);
  return ramp.SetMilestones(x,dx);
  return true;
}

bool DynamicMotionPlannerBase::GetMilestoneRamp(const ParabolicRamp::DynamicPath& curPath,const Config& q,ParabolicRamp::DynamicPath& ramp) const
{
  return GetMilestoneRamp(Vector(curPath.StartConfig()),Vector(curPath.StartVelocity()),q,ramp);
}

//returns true if the ramp from the current config to q is collision free
bool DynamicMotionPlannerBase::CheckMilestoneRamp(const ParabolicRamp::DynamicPath& curPath,const Config& q,ParabolicRamp::DynamicPath& ramp) const
{
  if(!GetMilestoneRamp(curPath,q,ramp)) {
    printf("CheckMilestoneRamp: Failed to get milestone ramp\n");
    return false;
  }
  //check bounding box exactly
  ParabolicRamp::Vector bmin,bmax;
  for(size_t r=0;r<ramp.ramps.size();r++) {
    ramp.ramps[r].Bounds(bmin,bmax);
    for(size_t i=0;i<bmin.size();i++) {
      if(bmin[i] < robot->qMin(i)) {
	printf("CheckMilestoneRamp: Bound on link %d failed check: %g < %g\n",(int)i,bmin[i],robot->qMin(i));
	return false;
      }
      if(bmax[i] > robot->qMax(i)) {
	printf("CheckMilestoneRamp: Bound on link %d failed check: %g > %g\n",(int)i,bmax[i],robot->qMax(i));
	return false;
      }
    }
  }
  //now check collisions approximately
  Real pathEpsilon = settings->robotSettings[0].collisionEpsilon;
  CSpaceFeasibilityChecker checker(cspace);
  for(size_t r=0;r<ramp.ramps.size();r++) {
    if(!CheckRamp(ramp.ramps[r],&checker,pathEpsilon)) {
      printf("CheckMilestoneRamp: Collision check failed.\n");
      return false;
    }
  }
  return true;
}

//returns the cost of going from the start to  q (assuming no collision detection)
Real DynamicMotionPlannerBase::EvaluateDirectPathCost(const ParabolicRamp::DynamicPath& currentPath,const Config& q) 
{
  if(!goal) return 0;
  ParabolicRamp::DynamicPath ramp;
  if(!GetMilestoneRamp(currentPath,q,ramp)) return Inf;
  return goal->PathCost(ramp,tstart);
}

//returns the cost of using the given path
Real DynamicMotionPlannerBase::EvaluatePathCost(const ParabolicRamp::DynamicPath& path,Real pathtstart)
{
  if(!goal) return 0;
  if(pathtstart < 0) pathtstart= tstart;
  return goal->PathCost(path,pathtstart);
}

//returns the cost of using the given path
Real DynamicMotionPlannerBase::EvaluateTerminalCost(const Config& q,Real tEnd)
{
  if(!goal) return 0;
  Config zero(q.n,0.0);
  return goal->TerminalCost(tEnd,q,zero);
}


int DynamicIKPlanner::PlanFrom(ParabolicRamp::DynamicPath& path,Real cutoff)
{
  if (!goal) return Failure;
  //check if we've converged
  Real Cstart = EvaluatePathCost(path);
  if(Cstart < 1e-3) return Failure;

  robot->UpdateConfig(Vector(path.EndConfig()));
  bool res=Optimize(goal,robot,10,1e-3);
  if(!res) { //optimization failed, do we do anything?
    fprintf(flog,"IK optimization failed\n");
  }
  Config q = robot->q;
  for(int i=0;i<10;i++) {
    //check for an improvement
    Real cost=EvaluateDirectPathCost(path,q);
    if(cost < Cstart) {
      ParabolicRamp::DynamicPath ramp;
      if(CheckMilestoneRamp(path,q,ramp)) {
	printf("IK planner: success\n");
	path = ramp;
	return Success;
      }
    }
    if(IsInf(cost)) {
      fprintf(flog,"IK Planner: bisection %d couldn't achieve feasible ramp\n",i);
      fprintf(flog,"  %s\n",LexicalCast(q).c_str());
    }
    if(stopPlanning) {
      fprintf(flog,"IK Planner: Timeout called\n");
      return Timeout;
    }
    q = (q + Vector(path.EndConfig()))*0.5;
  }
  fprintf(flog,"IK planner: collision checks failed\n");
  return Failure;
}

DynamicPerturbationPlanner::DynamicPerturbationPlanner()
  : perturbationStep(0.01),perturbLimit(100)
{}

void DynamicPerturbationPlanner::SetGoal(shared_ptr<PlannerObjectiveBase> newgoal)
{
  iteration = 0;
  DynamicMotionPlannerBase::SetGoal(newgoal);
}

int DynamicPerturbationPlanner::PlanFrom(ParabolicRamp::DynamicPath& path,Real cutoff)
{
  if (!goal) return Failure;
  //check if we've converged
  Real Cstart = EvaluatePathCost(path);
  if(Cstart < 1e-3) return Failure;

  iteration++;
  Config q;
  if(iteration < perturbLimit) 
    cspace->SampleNeighborhood(Vector(path.EndConfig()),iteration*perturbationStep,q);
  else
    cspace->Sample(q);
  Real cost=EvaluateDirectPathCost(path,q);
  if(cost < Cstart) {
    ParabolicRamp::DynamicPath ramp;
    if(CheckMilestoneRamp(path,q,ramp)) {
      path = ramp;
      iteration=0;
      return Success;
    }
    if(stopPlanning) return Timeout;
    ParabolicRamp::Vector temp;
    path.Evaluate(0,temp);
    Config qmid;
    cspace->Interpolate(Vector(temp),q,0.5,qmid);
    q = qmid;
  }
  return Failure;
}


DynamicPerturbationIKPlanner::DynamicPerturbationIKPlanner()
  : perturbationStep(0.01),perturbLimit(100)
{}

void DynamicPerturbationIKPlanner::SetGoal(shared_ptr<PlannerObjectiveBase> newgoal)
{
  iteration = 0;
  DynamicMotionPlannerBase::SetGoal(newgoal);
}

int DynamicPerturbationIKPlanner::PlanFrom(ParabolicRamp::DynamicPath& path,Real cutoff)
{
  if (!goal) return Failure;
  //check if we've converged
  Real Cstart = EvaluatePathCost(path);
  if(Cstart < 1e-3) return Failure;

  iteration++;
  Config q;
  if(iteration > perturbLimit) 
    cspace->Sample(q);
  else
    cspace->SampleNeighborhood(Vector(path.EndConfig()),iteration*perturbationStep,q);
  robot->UpdateConfig(q);
  bool res=Optimize(goal,robot,1,1e-3);
  if(!res) { //IK failed, do we do anything
  }
  q = robot->q;

  for(int i=0;i<10;i++) {
    Real cost=EvaluateDirectPathCost(path,q);
    if(cost < Cstart) {
      ParabolicRamp::DynamicPath ramp;
      if(CheckMilestoneRamp(path,q,ramp)) {
	path = ramp;
	iteration=0;
	return Success;
      }
    }
    if(stopPlanning) return Timeout;
    ParabolicRamp::Vector temp;
    path.Evaluate(0,temp);
    Config qmid;
    cspace->Interpolate(Vector(temp),q,0.5,qmid);
    q = qmid;
  }
  return Failure;
}



DynamicRRTPlanner::DynamicRRTPlanner()
  : delta(0.3),smoothTime(0.5),ikSolveProbability(0.5)
{}

void DynamicRRTPlanner::SetGoal(shared_ptr<PlannerObjectiveBase> newgoal)
{
  //reset
  iteration = 0;
  DynamicMotionPlannerBase::SetGoal(newgoal);
}

Vector& DynamicRRTPlanner::MakeState(const Config& q,const Config& dq)
{
  Assert(dq.n == q.n);
  tempV.resize(q.n*2);
  tempV.copySubVector(0,q);
  tempV.copySubVector(q.n,dq);
  return tempV;
}
Vector& DynamicRRTPlanner::MakeState(const Config& q)
{
  tempV.resize(q.n*2);
  tempV.copySubVector(0,q);
  for(int i=0;i<q.n;i++)
    tempV(q.n+i)=0;
  return tempV;
}

RRTPlanner::Node* DynamicRRTPlanner::TryIKExtend(RRTPlanner::Node* node,bool search)
{
  Config q;
  q.setRef(node->x,0,1,node->x.n/2);
  Assert(q.n == robot->q.n);
  robot->UpdateConfig(q);
  bool res=Optimize(goal,robot,10,1e-3);
  if(!res) { //optimization failed, do we do anything?
  }
  Config qik=robot->q;
  /*
    Real d=cspace->Distance(q,qik);
    if(d > delta) {
    cspace->Interpolate(q,qik,delta/d,robot->q);
    qik = robot->q;
    }
  */
  bool feasible=false;
  for(int i=0;i<4;i++) {
    if(cspace->IsFeasible(qik)) {
      feasible=true;
      break;
    }
    else {
      //bisect the line from q to qik
      qik += q;
      qik *= 0.5;
    }
  }
  if(!feasible) return NULL;

  //add a node in the rrt tree
  State x=MakeState(qik);
  if(search) {
    RRTPlanner::Node* closest = rrt->ClosestMilestone(x);
    return ((TreeRoadmapPlanner*)(rrt.get()))->Extend(closest,x);
  }
  else
    return ((TreeRoadmapPlanner*)(rrt.get()))->Extend(node,x);
}

Real DynamicRRTPlanner::EvaluateNodePathCost(RRTPlanner::Node* n) 
{
  Config q,dq;
  q.setRef(n->x,0,1,n->x.n/2);
  dq.setRef(n->x,n->x.n/2,1,n->x.n/2);
  Real time=0;
  while(n->getParent() != NULL) {
    time += ((RampEdgeChecker*)(n->edgeFromParent().get()))->Duration();
    n = n->getParent();
  }
  return goal->TerminalCost(time,q,dq);
}

//perform lazy collision checking of the path up to n
//ndelete allows you to check if any nodes are deleted
//pass in nodes to check, they will be set to NULL if they lie in the
//deleted subtree
bool DynamicRRTPlanner::CheckPath(RRTPlanner::Node* n,vector<RRTPlanner::Node*>& ndelete,Timer& timer,Real cutoff)
{
  //lazy collision checking
  RRTPlanner::Node* temp=n;
  while(temp->getParent() != NULL) {
    if(timer.ElapsedTime() >= cutoff) return false;
    //assume existing path is feasible, so check if temp is on the existing path
    for(size_t i=0;i<existingNodes.size();i++)
      if(temp == existingNodes[i]) {
	Assert(((RampEdgeChecker*)(temp->edgeFromParent().get()))->IsValid());
	while(temp->getParent() != NULL) {
	  Assert(((RampEdgeChecker*)(temp->edgeFromParent().get()))->IsValid());
	  temp = temp->getParent();
	}
	return true;
      }

    if(!IsVisible(temp->edgeFromParent(),timer,cutoff)) {
      for(size_t i=0;i<ndelete.size();i++)
	if(ndelete[i]==temp || ndelete[i]->hasAncestor(temp)) ndelete[i]=NULL; 
      //delete the subtree
      rrt->DeleteSubtree(temp);
      return false;
    }
    else {
      Assert(((RampEdgeChecker*)(temp->edgeFromParent().get()))->IsValid());
    }
    temp = temp->getParent();
  }
  return true;
}

int DynamicRRTPlanner::PlanFrom(ParabolicRamp::DynamicPath& path,Real cutoff)
{
  if (!goal) return Failure;

  if(stateSpace==NULL) {
    stateSpace.reset(new RampCSpaceAdaptor(cspace,Vector(velMax),Vector(accMax)));
    stateSpace->qMin=Vector(qMin);
    stateSpace->qMax=Vector(qMax);
    stateSpace->visibilityTolerance = settings->robotSettings[0].collisionEpsilon;
  }


  Assert(path.IsValid());
  Timer timer;
  if(goal->TerminalCost(tstart+path.GetTotalTime(),Vector(path.ramps.back().x1),Vector(path.ramps.back().dx1)) < 1e-3) {
    fprintf(flog,"Already at solution, doing shortcutting...\n");
    Shortcut(path,cutoff);
    Assert(path.IsValid());
    return Success;
  }

  RRTPlanner::Node* bestNode=NULL;
  Real bestPathCost = EvaluatePathCost(path);
  fprintf(flog,"*** Total cost %g, terminal cost %g ***\n",bestPathCost,goal->TerminalCost(0.0,Vector(path.ramps.back().x1),Vector(path.ramps.back().dx1)));

  rrt = NULL;
  //create new RRT tree
  rrt.reset(new RRTPlanner(stateSpace.get()));

  existingNodes.resize(0);

  //printf("Adding old path...\n");
  //initialize tree with existing path
  RRTPlanner::Node* n=rrt->AddMilestone(MakeState(Vector(path.ramps[0].x0),Vector(path.ramps[0].dx0)));
  if(!stateSpace->IsFeasible(n->x)) {
    fprintf(flog,"Warning, start state is infeasible!\n");
  }
  RRTPlanner::Node* root = n;
  assert(root == rrt->milestones[0]);
  existingNodes.push_back(n);
  vector<RRTPlanner::Node*> iknodes;
  RRTPlanner::Node* nik = NULL;
  //check the ik extension
  if(ikSolveProbability > 0)
    nik=TryIKExtend(n,false);
  if(nik && EvaluateNodePathCost(nik) < bestPathCost) {
    if(nik->edgeFromParent()->IsVisible()) {  //only need to check path to parent assuming an already feasible path
      Assert(((RampEdgeChecker*)(nik->edgeFromParent().get()))->IsValid());
      bestNode = nik;
      bestPathCost = EvaluateNodePathCost(nik);
    }
    else {
      //fprintf(flog,"Deleted subtree derived from path ik %d\n",0);
      rrt->DeleteSubtree(nik);
      //assert(root == rrt->milestones[0]);
    }
  }
  //extending path destinations
  for(size_t i=0;i<path.ramps.size();i++) {
    n = ((TreeRoadmapPlanner*)rrt.get())->Extend(n,MakeState(Vector(path.ramps[i].x1),Vector(path.ramps[i].dx1)));
    //sometimes there's an error with SolveMinTime...
    ((RampEdgeChecker*)(n->edgeFromParent().get()))->path.ramps.resize(1,path.ramps[i]);
    Assert(path.ramps[i].IsValid());
    Assert(((RampEdgeChecker*)(n->edgeFromParent().get()))->IsValid());
    /*
    if(!n->edgeFromParent()->IsVisible()) {
      fprintf(flog,"Prior path edge %d became infeasible\n",i);
      rrt->DeleteSubtree(n);
      assert(root == rrt->milestones[0]);
      break;
    }
    */

    existingNodes.push_back(n);
    //fprintf(flog,"Node %d \n",i+1);

    if(ikSolveProbability > 0 && i+1 == path.ramps.size() && bestNode == NULL) {
      //check the ik extension
      nik=(RandBool(ikSolveProbability)?TryIKExtend(n,false):NULL);
      //if(nik) fprintf(flog,"IK node %d\n",i+1);
      if(nik && EvaluateNodePathCost(nik) < bestPathCost) {
	if(nik->edgeFromParent()->IsVisible()) {
	  Assert(((RampEdgeChecker*)(nik->edgeFromParent().get()))->IsValid());
	  bestNode = nik;
	  bestPathCost = EvaluateNodePathCost(nik);
	}
	else {
	  //fprintf(flog,"Deleted subtree derived from path ik %d\n",i);
	  rrt->DeleteSubtree(nik);
	}
      }
    }
    if(stopPlanning) return Timeout;
  }
  //sanity check
  for(size_t i=0;i<existingNodes.size();i++) {
    if(existingNodes[i]->getParent()) {
      Assert(((RampEdgeChecker*)(existingNodes[i]->edgeFromParent().get()))->IsValid());
      n = existingNodes[i];
      assert(n->edgeFromParent()->End() == n->x);
      assert(n->edgeFromParent()->Start() == n->getParent()->x);
    }
  }
  for(size_t i=0;i<rrt->milestones.size();i++) {
    n = rrt->milestones[i];
    if(n->getParent() != NULL) {
      assert(n->edgeFromParent()->End() == n->x);
      assert(n->edgeFromParent()->Start() == n->getParent()->x);
    }
    else assert(i == 0);
  }

  if(bestPathCost < 1e-3) bestPathCost=0;

  if(stopPlanning) return Timeout;

  Real planTimeLimit = (1.0-smoothTime)*cutoff;
  Real rrtCutoff = cutoff;
  Real currentPathTime = path.GetTotalTime();
  if(currentPathTime > 0)  //give some time to smoothing
    rrtCutoff = Max(cutoff-currentPathTime,planTimeLimit); 

  Real t;
  fprintf(flog,"Starting RRT planning with %g seconds left\n",rrtCutoff-timer.ElapsedTime());
  while((t=timer.ElapsedTime()) < rrtCutoff) {
    //if(timer.ElapsedTime() > planTimeLimit) { //smooth only if an improvement has been made?
    if(stopPlanning) return Timeout;
    if(bestNode && t > planTimeLimit) {
      //start smoothing
      break;
    }
    Config dest,x;
    Vector q;
    RRTPlanner::Node* closest;
    cspace->Sample(dest);
    x=MakeState(dest);
    //pick closest milestone, step in that direction
    closest=rrt->ClosestMilestone(x);
    q.setRef(closest->x,0,1,dest.n);
    Real dist=cspace->Distance(q,dest);
    if(dist > delta) {
      Vector dest2;
      cspace->Interpolate(q,dest,delta/dist,dest2);
      x = MakeState(dest2);
    }
    q.setRef(x,0,1,dest.n);
    if(!cspace->IsFeasible(q)) continue;

    RRTPlanner::Node* n=((TreeRoadmapPlanner*)(rrt.get()))->Extend(closest,x);

    //add a dynamic point in the middle
    RRTPlanner::Node* n2 = rrt->SplitEdge(n->getParent(),n,0.5); 
    if(!stateSpace->IsFeasible(n2->x)) {
      //fprintf(flog,"SplitEdge failed\n");
      rrt->DeleteSubtree(n2);
      continue;
    }
    //fix up the split paths
    
    nik=(RandBool(ikSolveProbability)?TryIKExtend(n):NULL);
    if(nik && EvaluateNodePathCost(nik) < bestPathCost) {
      //fprintf(flog,"Actual cost %g <=> %g\n",EvaluateNodePathCost(nik),bestPathCost);
      vector<RRTPlanner::Node*> delnodes(1,n);
      if(CheckPath(nik,delnodes,timer,cutoff)) {
	//fprintf(flog,"Extend + IK succeeded to a feasible node\n");
	bestNode = nik;
	bestPathCost = EvaluateNodePathCost(nik);
	
	while(nik->getParent() != NULL) {
	  Assert(((RampEdgeChecker*)(nik->edgeFromParent().get()))->IsValid());
	  nik = nik->getParent();
	}
      }
      else {
	//fprintf(flog,"Extend + IK succeeded but path check failed\n");
	n=delnodes[0];
	
	//can't use these nodes if they don't end in zero velocity!
	if(n) {
	  Assert(n->x.n == dest.n*2);
	  bool usable=true;
	  for(int i=dest.n;i<dest.n*2;i++)
	    if(n->x(i) != 0) {
	      usable = false;
	      break;
	    }
	  if(!usable) n=NULL;
	}
      }
    }
    if(n && EvaluateNodePathCost(n) < bestPathCost) {
      //fprintf(flog,"Actual cost %g <=> %g\n",EvaluateNodePathCost(n),bestPathCost);
      vector<RRTPlanner::Node*> delnodes;
      if(CheckPath(n,delnodes,timer,cutoff)) {
	//fprintf(flog,"Extend succeeded to a feasible node\n");
	bestNode = n;
	bestPathCost = EvaluateNodePathCost(n);
	
	while(n->getParent() != NULL) {
	  Assert(((RampEdgeChecker*)(n->edgeFromParent().get()))->IsValid());
	  n = n->getParent();
	}
      }
      else {
	fprintf(flog,"Extend succeeded but path check failed\n");
      }
    }
  }
  //sanity check
  for(size_t i=0;i<rrt->milestones.size();i++) {
    n = rrt->milestones[i];
    if(n->getParent() != NULL) {
      assert(n->edgeFromParent()->End() == n->x);
      assert(n->edgeFromParent()->Start() == n->getParent()->x);
    }
    else assert(i == 0);
  }

  if(bestNode) {
    fprintf(flog,"Successfully reduced path cost to %g with %g time left\n",bestPathCost,cutoff-timer.ElapsedTime());
    MilestonePath rampPath;
    assert(root == rrt->milestones[0]);    
    n = bestNode;
    int parent = 0;
    while(n->getParent() != NULL) {
      Assert(n->edgeFromParent()->End() == n->x);
      if(n->edgeFromParent()->Start() != n->getParent()->x) {
	fprintf(flog,"Parent %d node is incorrect\n",parent+1);
	fprintf(flog,"%s\n",LexicalCast(n->edgeFromParent()->Start()).c_str());
	fprintf(flog,"%s\n",LexicalCast(n->getParent()->x).c_str());
      }
      Assert(n->edgeFromParent()->Start() == n->getParent()->x);
      parent += 0;
      n = n->getParent();
    }
    Assert(bestNode == rrt->milestones[0] || bestNode->hasAncestor(rrt->milestones[0]));
    rrt->CreatePath(rrt->milestones[0],bestNode,rampPath);
    Assert(!rampPath.edges.empty());
    Assert(Vector(rampPath.edges.front()->Start()) == rrt->milestones[0]->x);

    /*
    //test feasibility of last state
    if(!stateSpace->IsFeasible(bestNode->x)) {
      fprintf(flog,"Problem: last node is infeasible?\n");
      getchar();
    }
    */

     //unnecessary now, that bug was squashed
    int numExisting=-1;
    RRTPlanner::Node* n = bestNode;
    while(n->getParent() != NULL) {
      //assume existing path is feasible
      for(size_t i=0;i<existingNodes.size();i++) {
	if(numExisting < 0 && n == existingNodes[i]) {
	  numExisting=(int)i;
	  break;
	}
      }
      Assert(((RampEdgeChecker*)(n->edgeFromParent().get()))->IsValid());
      n = n->getParent();
    }
    //sanity check
    for(size_t i=0;i<existingNodes.size();i++) {
      if(existingNodes[i]->getParent()) {
	Assert(((RampEdgeChecker*)(existingNodes[i]->edgeFromParent().get()))->IsValid());
      }
    }

    //read out the DynamicPath
    path.ramps.resize(0);
    path.ramps.reserve(rampPath.edges.size());
    for(size_t i=0;i<rampPath.edges.size();i++) {
      RampEdgeChecker* e=(RampEdgeChecker*)(rampPath.edges[i].get());
      path.Concat(e->path);

      //sanity checks
      Assert(e->IsValid());
    }
    for(size_t i=0;i<path.ramps.size();i++) {
      if(path.ramps[i].ramps.empty()) fprintf(flog,"Empty ramp %d / %d.\n",(int)i,(int)rampPath.edges.size());
      Assert(path.ramps[i].IsValid());
      Assert(path.ramps[i].x0.size() == path.ramps[i].ramps.size());
      if(i > 0) {
	Assert(path.ramps[i].x0 == path.ramps[i-1].x1);
	Assert(path.ramps[i].dx0 == path.ramps[i-1].dx1);
      }
    }
    if(!Vector(path.ramps.back().dx1).isZero()) {
      fprintf(flog,"Weird, path does not end in zero velocity?\n");
      fprintf(flog,"%s\n",LexicalCast(Vector(path.ramps.back().dx1)).c_str());
      fprintf(flog,"Last milestone: %s\n",LexicalCast(bestNode->x).c_str());
      fprintf(flog,"About to Abort() after getchar...\n");
      getchar();
      Abort();
    }
    Assert(Vector(path.ramps.back().dx1).isZero());

    fprintf(flog,"Devoting %g seconds to smoothing new path\n",cutoff-timer.ElapsedTime());
    Assert(path.IsValid());
    Shortcut(path,cutoff-timer.ElapsedTime());
    assert(path.IsValid());
    return Success;
  }
  fprintf(flog,"Devoting %g seconds to smoothing old path\n",cutoff-timer.ElapsedTime());
  if(stopPlanning) return Timeout;
  Assert(path.IsValid());
  Shortcut(path,cutoff-timer.ElapsedTime());
  assert(path.IsValid());
  return Timeout;
}

















DynamicHybridTreePlanner::DynamicHybridTreePlanner()
  : delta(0.3),smoothTime(0.5),ikSolveProbability(0.5)
{}

void DynamicHybridTreePlanner::SetGoal(shared_ptr<PlannerObjectiveBase> newgoal)
{
  //reset
  iteration = 0;
  DynamicMotionPlannerBase::SetGoal(newgoal);
}

DynamicHybridTreePlanner::Node* DynamicHybridTreePlanner::AddChild(Node* node,const Config& q)
{
  assert(q(0) == 0.0);
  Config xn(node->q.n+node->dq.n),xq(xn.n);
  xn.copySubVector(0,node->q);
  xn.copySubVector(node->q.n,node->dq);
  xq.setZero();
  xq.copySubVector(0,q);
  EdgePlannerPtr e=stateSpace->LocalPlanner(xn,xq);
  shared_ptr<RampEdgeChecker> ptr((RampEdgeChecker*)e.get());
  return AddChild(node,ptr);
}

DynamicHybridTreePlanner::Node* DynamicHybridTreePlanner::AddChild(Node* node,const ParabolicRamp::ParabolicRampND& ramp)
{
  shared_ptr<RampEdgeChecker> ptr(new RampEdgeChecker(stateSpace.get(),ramp));
  return AddChild(node,ptr);
}


DynamicHybridTreePlanner::Node* DynamicHybridTreePlanner::AddChild(Node* node,const ParabolicRamp::DynamicPath& path)
{
  shared_ptr<RampEdgeChecker> ptr(new RampEdgeChecker(stateSpace.get(),path));
  return AddChild(node,ptr);
}

DynamicHybridTreePlanner::Node* DynamicHybridTreePlanner::AddChild(Node* node,shared_ptr<RampEdgeChecker>& e)
{
  if(e->path.ramps.empty()) return NULL;
  Node* c = new Node;
  node->addChild(c);
  c->edgeFromParent().e = e;
  c->edgeFromParent().cost = goal->IncrementalCost(node->t,e->path);
  c->q = Vector(e->path.ramps.back().x1);
  c->dq = Vector(e->path.ramps.back().dx1);
  assert(c->q(0) == 0.0);
  assert(c->dq(0) == 0.0);
  c->reachable = false;
  c->t = node->t+e->Duration();
  c->depth = node->depth+1;
  c->sumPathCost = node->sumPathCost + c->edgeFromParent().cost;
  c->terminalCost = goal->TerminalCost(c->t,c->q,c->dq);
  c->totalCost = c->sumPathCost+c->terminalCost;
  return c;
}

class TrackingRampFunction : public ScalarFieldFunction
{
public:
  TrackingRampFunction(CartesianTrackingObjective* obj,Real tstart,const Config& qstart,const Vector& dqstart);

  virtual std::string Label() const { return "TrackingRamp"; }
  virtual std::string VariableLabel(int i) const {
    char buf[64];
    if(i<(int)x0.size()) { snprintf(buf,64,"q%d",i+1); return buf; }
    else { return "speed"; }
  }
  //layout: x=(q,speed);
  void AssembleRamp(const Vector& x);
  virtual void PreEval(const Vector& x) { AssembleRamp(x); }
  virtual Real Eval(const Vector& x);
  virtual void Gradient(const Vector& x,Vector& grad);
  virtual void Hessian(const Vector& x,Matrix& H) { FatalError("Hessian not defined in subclass of ScalarFieldFunction"); }

  CartesianTrackingObjective* obj;
  Real tstart;
  ParabolicRamp::DynamicPath path;
  vector<Real> x0,dx0,x1,dx1;
};

TrackingRampFunction::TrackingRampFunction(CartesianTrackingObjective* _obj,Real _tstart,const Config& qstart,const Vector& dqstart)
{
  obj = _obj;
  tstart = _tstart;
  x0 = qstart;
  dx0 = dqstart;
  dx1.resize(x0.size(),0.0);
  path.xMin = obj->robot->qMin;
  path.xMax = obj->robot->qMax;
  path.accMax = obj->robot->accMax;
  path.velMax = obj->robot->velMax;
}

void TrackingRampFunction::AssembleRamp(const Vector& x)
{
  Vector q;
  q.setRef(x,0,1,x0.size());
  Real speed = x(x0.size());
  if(speed > 1.0) speed = 1.0;
  x1 = q;
  for(int i=0;i<q.n;i++) {
    if(q(i) < obj->robot->qMin(i))
      x1[i] = obj->robot->qMin(i);
    else if(q(i) > obj->robot->qMax(i))
      x1[i] = obj->robot->qMax(i);
  }
  if(!path.SolveMinTime(x0,dx0,x1,dx1)) {
    path.ramps.clear();
  }
  if(speed < 1.0) {
    if(!path.SolveMinAccel(x0,dx0,x1,dx1,path.GetTotalTime()/speed))
      path.ramps.clear();
  }
}

Real TrackingRampFunction::Eval(const Vector& x)
{
  if(path.ramps.empty()) return Inf;
  PlannerObjectiveBase* pobj = obj;
  return (pobj->IncrementalCost(tstart,path)+obj->TerminalCost(tstart+path.GetTotalTime(),Vector(x1),Vector(dx1)))*100;
}

void TrackingRampFunction::Gradient(const Vector& x,Vector& grad)
{
  if(path.ramps.empty()) {
    grad.resize(x.n,0.0);
    return;
  }
  //boundary-aware centered differences
  grad.resize(x.n);
  Vector temp=x;
  GradientCenteredDifference(*this,temp,0.005,grad);
  for(int i=0;i<grad.n;i++)
    if(!IsFinite(grad[i])) grad[i]=0.0;
}

Real ClampAbs(Vector& x,const vector<Real>& xmax)
{
  Real u=1.0;
  int lim = -1;
  for(int i=0;i<x.n;i++) {
    if(x(i)*u < -xmax[i]) {
      lim = i;
      u = -xmax[i]/x(i);
    }
    else if(x(i)*u > xmax[i]) {
      u = xmax[i]/x(i);
      lim = i;
    }
  }
  assert(u >= 0.0);
  if(u != 1.0) {
    x *= u;
    x(lim) = xmax[lim]*Sign(x(lim));
    for(int i=0;i<x.n;i++)
      assert(Abs(x(i)) <= xmax[i]);
  }
  return u;
}

Real MinBrakingTime(const Vector& vel,const Vector& acc)
{
  Assert(vel.n == acc.n);
  Real maxT = 0.0;
  for(int i=0;i<vel.n;i++) {
    if(acc(i) != 0.0 || vel(i) != 0.0)
      maxT = Max(maxT,Abs(vel(i)/acc(i)));
  }
  return maxT;
}

DynamicHybridTreePlanner::Node* DynamicHybridTreePlanner::TryIKExtend(Node* node,bool search)
{
  if(typeid(*goal) == typeid(CartesianTrackingObjective)) {
    //special optimization -- try to optimize a local path from the given node
    if(search) {
      //Assert(IsInf(node->getParent()->terminalCost));
      node = node->getParent();
    }
    CartesianTrackingObjective* obj = dynamic_cast<CartesianTrackingObjective*>(&*goal);
    assert(obj != NULL);
    //create an optimized parabolic path from node->q,node->dq,node->t,
    //straight path, and then ending with a braking trajectory
    //variables a=(a1,...,an), dt1, linear segment for time dt2, braking for
    //time dt3
    //y(t+node->t) = q + dq t + 1/2 a t^2
    //y(dt1+node->t) = q + dq dt1 + 1/2 a dt1^2
    //y'(dt1+node->t) = dq + a dt1
    //y(dt2+dt1+node->t) = y(dt1+node->t) + y'(dt1+node->t)*dt2
    //y'(dt2+dt1+node->t) = y'(dt1+node->t)
    //braking trajectory subsequently
    TrackingRampFunction f(obj,node->t,node->q,node->dq);
    Optimization::BCMinimizationProblem opt(&f);
    opt.verbose = 0;
    //initial time estimate
    robot->UpdateConfig(node->q);
    Vector3 w;
    robot->GetWorldPosition(obj->localPosition,obj->link,w);
    Real dt0 = 6.0*w.distance(obj->positions.front());
    //setup an initial feasible point
    //choose initial configuration on the path pushed forward by time dt0
    //robot->UpdateConfig(node->q+dt0*node->dq);
    robot->UpdateConfig(node->q);
    Vector3 pd=obj->GetDesiredPosition(node->t+dt0);
    //find the least-squares offset to get toward pd
    vector<IKGoal> ikproblem(1);
    ikproblem[0].link = obj->link;
    ikproblem[0].localPosition = obj->localPosition;
    ikproblem[0].SetFixedPosition(pd);
    int iters=10;
    bool res=SolveIK(*robot,ikproblem,1e-3,iters,0);
    Config qik=robot->q;
    bool feasible=false;
    for(int i=0;i<4;i++) {
      if(cspace->IsFeasible(qik)) {
	feasible=true;
	break;
      }
      else {
	//bisect the line from q to qik
	qik += node->q;
	qik *= 0.5;
      }
    }
    if(!feasible) return NULL;

    opt.x.resize(robot->q.n+1);
    opt.x.copySubVector(0,qik);
    opt.x(robot->q.n) = 1.0;
    Vector x0 = opt.x;
    Real f0 = f(opt.x);
    if(!IsFinite(f0)) {
      fprintf(flog,"Warning: initial point for optimization is not feasible\n");
      return NULL;
    }
    /*
    //Early exit
    if(!cspace->IsFeasible(Vector(f.path.ramps.back().x1))) return NULL;
    assert(f.path.IsValid());
    return AddChild(node,f.path);
    */

    assert(IsFinite(f0));
    opt.bmin.resize(opt.x.n);
    opt.bmax.resize(opt.x.n);
    opt.bmin.copySubVector(0,Vector(stateSpace->qMin));
    opt.bmin(robot->q.n) = 0.01;
    opt.bmax.copySubVector(0,Vector(stateSpace->qMax));
    opt.bmax(robot->q.n) = 1.0;
    for(int i=0;i<opt.x.n;i++) {
      if(opt.x(i) < opt.bmin(i) || opt.x(i) > opt.bmax(i)) 
	fprintf(flog,"Error on %d: %g in [%g,%g]\n",i,opt.x(i),opt.bmin(i),opt.bmax(i));
      assert(opt.x(i) >= opt.bmin(i) && opt.x(i) <= opt.bmax(i));
    }
    iters=10;
    opt.SolveQuasiNewton_Ident(iters);
    //fprintf(flog,"Quasi newton solved f %g->%g\n",f0,f(opt.x));
    f.PreEval(opt.x);
    Vector dest=Vector(f.path.ramps.back().x1);
    for(int iters=0;iters<4;iters++) {
      if(cspace->IsFeasible(dest)) {
	assert(f.path.IsValid());
	return AddChild(node,f.path);
      }
      dest = (node->q + dest)*0.5;
      opt.x.copySubVector(0,dest);
      f.PreEval(opt.x);
      if(f.path.ramps.empty()) {
	fprintf(flog,"Quasi-newton optimized point not feasible\n");
	return NULL;
      }
    }
    //f.PreEval(x0);
    //return AddChild(node,f.path);
    fprintf(flog,"Quasi-newton optimized point not feasible\n");
    return NULL;
  }

  const Config& q=node->q;
  Assert(q.n == robot->q.n);
  robot->UpdateConfig(q);
  bool res=Optimize(goal,robot,10,1e-3);
  if(!res) { //optimization failed, do we do anything?
  }
  Config qik=robot->q;
  /*
    Real d=cspace->Distance(q,qik);
    if(d > delta) {
    cspace->Interpolate(q,qik,delta/d,robot->q);
    qik = robot->q;
    }
  */
  bool feasible=false;
  for(int i=0;i<4;i++) {
    if(cspace->IsFeasible(qik)) {
      feasible=true;
      break;
    }
    else {
      //bisect the line from q to qik
      qik += q;
      qik *= 0.5;
    }
  }
  if(!feasible) return NULL;

  //add a node in the rrt tree
  if(search) {
    Node* closest = Closest(qik);
    return AddChild(closest,qik);
  }
  else
    return AddChild(node,qik);
}

//perform lazy collision checking of the path up to n
//pass in nodes to check, they will be set to NULL if they lie in the
//deleted subtree
bool DynamicHybridTreePlanner::CheckPath(Node* n,Timer& timer,Real cutoff,Node** split)
{
  //lazy collision checking
  Node* temp=n;
  vector<Node*> checkNodes;
  while(temp->getParent() != NULL) {
    if(temp->reachable) break;
    checkNodes.push_back(temp);
    temp = temp->getParent();
  }
  while(temp != NULL) {
    if(!temp->reachable) {
      fprintf(flog,"Warning: inconsitency in reachability marking\n");
      getchar();
    }
    temp = temp->getParent();
  }
  reverse(checkNodes.begin(),checkNodes.end());

  for(size_t i=0;i<checkNodes.size();i++) {
    if(timer.ElapsedTime() >= cutoff) {
      fprintf(flog,"Path check ran up against cutoff\n");
      return false;
    }
    //assume existing path is feasible, so check if temp is on the existing path
    temp = checkNodes[i];

    int res=IsVisible(temp->edgeFromParent().e,timer,cutoff);
    if(res==-1) {
      //cutoff
      fprintf(flog,"Path check ran up against cutoff\n");
      return false;
    }
    else if (res == 0) {
      //delete the subtree
      Node* p=temp->getParent();
      p->eraseChild(temp);
      if(split) *split = p;
      return false;
    }
    else {
      temp->reachable = true;
    }
  }
  return true;

}

struct ClosestCallback : public Graph::CallbackBase<DynamicHybridTreePlanner::Node*>
{
  RampCSpaceAdaptor* space;
  const Config& q;
  Real costBranch;
  Real closestDist;
  DynamicHybridTreePlanner::Node* closest;
  ParabolicRamp::ParabolicRampND ramp;

  ClosestCallback(RampCSpaceAdaptor* _space,const Config& _q)
    :space(_space),q(_q),costBranch(Inf),closestDist(Inf),closest(NULL)
  {
    ramp.x0.resize(q.n);
    ramp.dx0.resize(q.n);
    ramp.x1.resize(q.n);
    ramp.dx1.resize(q.n);
    copy(q.begin(),q.end(),ramp.x1.begin());
    fill(ramp.dx1.begin(),ramp.dx1.end(),0.0);
  }

  void Visit(DynamicHybridTreePlanner::Node* n) {
    /*
    if(n->q.distance(q) < closestDist) {
      closestDist = n->q.distance(q);
      closest = n;
    }
    return;
    */
    if(n->sumPathCost > costBranch) return;
    assert(n->q(0) == 0.0);
    assert(n->dq(0) == 0.0);
    copy(n->q.begin(),n->q.end(),ramp.x0.begin());
    copy(n->dq.begin(),n->dq.end(),ramp.dx0.begin());
    if(!ramp.SolveMinTime(space->accMax,space->velMax)) return;
    if(ramp.endTime < closestDist) {
      closestDist = ramp.endTime;
      closest = n;
    }
  }
};

DynamicHybridTreePlanner::Node* DynamicHybridTreePlanner::Closest(const Config& q,Real costBranch)
{
  ClosestCallback callback(stateSpace.get(),q);
  callback.costBranch = costBranch;
  root->DFS(callback);
  return callback.closest;
}

DynamicHybridTreePlanner::Node* DynamicHybridTreePlanner::ExtendToward(const Config& qdes,Real costBranch)
{
  //pick closest milestone, step in that direction
  Vector q;
  Node* closest=Closest(qdes,costBranch);
  Real dist=cspace->Distance(closest->q,qdes);
  if(dist > delta) {
    Vector dest2;
    cspace->Interpolate(closest->q,qdes,delta/dist,dest2);
    if(!cspace->IsFeasible(dest2)) return NULL;
    return AddChild(closest,dest2);    
  }
  if(!cspace->IsFeasible(qdes)) return NULL;  
  return AddChild(closest,qdes);
}

DynamicHybridTreePlanner::Node* DynamicHybridTreePlanner::SplitEdge(DynamicHybridTreePlanner::Node* p,DynamicHybridTreePlanner::Node* n,Real u)
{
  Assert(p==n->getParent());
  const ParabolicRamp::DynamicPath& path=n->edgeFromParent().e->path;
  Assert(!path.ramps.empty());
  ParabolicRamp::DynamicPath before,after;
  path.Split(u*path.GetTotalTime(),before,after);
  p->detachChild(n);
  Node* s=AddChild(p,before);
  if(!s) {
    fprintf(flog,"Warning, failure to add child in splitEdge\n");
    return NULL;
  }
  //do not allow stopping at a nonstationary configuration
  s->terminalCost = Inf;
  s->totalCost = Inf;
  s->reachable = n->reachable;
  s->addChild(n);
  //there may be numerical errors in using after
  after.ramps.front().x0 = before.ramps.back().x1;
  after.ramps.front().dx0 = before.ramps.back().dx1;
  for(size_t i=0;i<after.ramps.front().ramps.size();i++) {
    after.ramps.front().ramps[i].x0 = after.ramps.front().x0[i]; 
    after.ramps.front().ramps[i].dx0 = after.ramps.front().dx0[i]; 
  }
  Assert(after.IsValid());
  Config x = n->edgeFromParent().e->End();
  n->edgeFromParent().e->path = after;
  n->edgeFromParent().cost = goal->IncrementalCost(s->t,n->edgeFromParent().e->path);
  return s;
}

int DynamicHybridTreePlanner::PlanFrom(ParabolicRamp::DynamicPath& path,Real cutoff)
{
  if (!goal) return Failure;

  if(stateSpace==NULL) {
    stateSpace.reset(new RampCSpaceAdaptor(cspace,Vector(velMax),Vector(accMax)));
    stateSpace->qMin = qMin;
    stateSpace->qMax = qMax;
    assert((int)path.velMax.size() == (int)robot->q.size());
    stateSpace->visibilityTolerance = settings->robotSettings[0].collisionEpsilon;
  }

  Assert(path.IsValid());
  Timer timer;
  /*
  if(goal->TerminalCost(path.GetTotalTime(),path.ramps.back().x1,path.ramps.back().dx1) < 1e-3) {
    fprintf(flog,"Already at solution, doing shortcutting...\n");
    Shortcut(path,cutoff);
    Assert(path.IsValid());
    return Success;
  }
  */

  //stats
  int numExistingNodes=0,numIKExisting=0,numIKExistingImprove=0,numIKExistingUnreachable=0,numGeneratedNodes=0,numFailSplitNodes=0,numImproveNodes=0,numUnreachableNodes=0,numIKNodes=0,numIKImproveNodes=0,numUnreachableIKNodes=0;

  Node* bestNode=NULL;
  Real bestTotalCost = EvaluatePathCost(path,tstart);
  fprintf(flog,"*** Current total cost %g, terminal cost %g *** \n",bestTotalCost,goal->TerminalCost(tstart+path.GetTotalTime(),Vector(path.ramps.back().x1),Vector(path.ramps.back().dx1)));
  if(bestTotalCost < 1e-5) return Success;

  //fprintf(flog,"Adding old path...\n");
  //initialize tree with existing path
  if(!stateSpace->IsFeasible(Vector(path.ramps[0].x0),Vector(path.ramps[0].dx0))) {
    fprintf(flog,"Warning, start state is infeasible!\n");
  }
  root.reset(new Node());
  root->t = tstart;
  root->q = path.ramps[0].x0;
  root->dq = path.ramps[0].dx0;
  root->sumPathCost = 0.0;
  root->terminalCost = goal->TerminalCost(root->t,root->q,root->dq);
  root->totalCost = root->terminalCost;
  root->reachable = true;
  root->depth = 0;

  vector<Node*> iknodes;
  //check the ik extension
  Node* nik=NULL;
  if(ikSolveProbability > 0) TryIKExtend(root.get(),false);
  if(nik) numIKExisting++;
  if(nik && nik->totalCost < bestTotalCost) {
    numIKExistingImprove++;
    int res=IsVisible(nik->edgeFromParent().e,timer,cutoff);
    if(res==1) {  //only need to check path to parent assuming an already feasible path
      nik->reachable = true;
      Assert(nik->edgeFromParent().e->path.IsValid());
      bestNode = nik;
      bestTotalCost = nik->totalCost;
      fprintf(flog,"Optimization of current state succeeded\n");

      Node* ntemp = bestNode;
      while(ntemp != NULL) {
	assert(ntemp->reachable);
	ntemp = ntemp->getParent();
      }
    }
    else if(res == 0) {
      fprintf(flog,"Optimization of current state failed to yield feasible path\n");
      root->eraseChild(nik);
      numIKExistingUnreachable++;
    }
    else return Timeout;
  }
  else
    fprintf(flog,"Optimization of current state failed\n");
  //extending path destinations
  Node* n = root.get();
  for(size_t i=0;i<path.ramps.size();i++) {
    if(stopPlanning) return Timeout;

    n = AddChild(n,path.ramps[i]);
    if(!n) {
      fprintf(flog,"Warning, failure to add child for existing path\n");
      continue;
    }
    n->reachable = true;
    numExistingNodes++;
    /*
    if(!n->edgeFromParent().e->IsVisible()) {
      fprintf(flog,"Prior path edge %d became infeasible\n",i);
      break;
    }
    */

    //if((i%3 == 1 || i+1 == path.ramps.size()) && bestNode == NULL) {
    if(ikSolveProbability > 0 && (i%3 == 1 || i+1 == path.ramps.size())) {
      //check the ik extension
      //nik=(RandBool(ikSolveProbability)?TryIKExtend(n,true):NULL);
      //nik=TryIKExtend(n,true);
      nik=TryIKExtend(n,false);
      //if(nik) fprintf(flog,"IK node %d\n",i+1);
      if(nik) numIKExisting++;
      if(nik && nik->totalCost < bestTotalCost) {
	numIKExistingImprove++;
	if(CheckPath(nik,timer,cutoff)) {
	  bestNode = nik;
	  bestTotalCost = nik->totalCost;

	  Node* ntemp = bestNode;
	  while(ntemp != NULL) {
	    assert(ntemp->reachable);
	    ntemp = ntemp->getParent();
	  }
	  fprintf(flog,"Optimization of endpoint state succeeded\n");
	}
	else {
	  fprintf(flog,"Optimization of path node %d failed to produce feasible path\n",i);
	  numIKExistingUnreachable++;
	  if(timer.LastElapsedTime() >= cutoff)
	    return Timeout;
	}
      }
      else {
	fprintf(flog,"Optimization of path node %d failed\n",i);
      }
    }
  }
  //if(bestTotalCost < 1e-3) bestTotalCost=0;
  fprintf(flog,"Local optimization of existing path has cost %g\n",bestTotalCost);
  Node* ntemp = bestNode;
  while(ntemp != NULL) {
    assert(ntemp->reachable);
    ntemp = ntemp->getParent();
  }

  Real planTimeLimit = (1.0-smoothTime)*cutoff;
  Real rrtCutoff = cutoff;
  Real currentPathTime = path.GetTotalTime();
  if(currentPathTime > 0)  //give some time to smoothing
    rrtCutoff = Max(cutoff-currentPathTime,planTimeLimit); 

  Real t=timer.ElapsedTime();
  if(rrtCutoff-t > 0) fprintf(flog,"Starting randomized planning with %gs left\n",rrtCutoff-t);
  while((t=timer.ElapsedTime()) < rrtCutoff) {
    if(stopPlanning) return Timeout;
    //if(timer.ElapsedTime() > planTimeLimit) { //smooth only if an improvement has been made?
    if(bestNode && t > planTimeLimit) {
      //start smoothing
      break;
    }
    Config dest,x;
    Vector q;
    cspace->Sample(dest);
    n = ExtendToward(dest,bestTotalCost);
    if(n == NULL) continue;
    if(n->sumPathCost > bestTotalCost) {
      n->getParent()->eraseChild(n);
      continue;
    }

    numGeneratedNodes++;

    //add a dynamic point in the middle
    Node* n2 = SplitEdge(n->getParent(),n,0.5); 
    if(!cspace->IsFeasible(n2->q)) {
      //fprintf(flog,"SplitEdge failed\n");
      n2->getParent()->eraseChild(n2);
      numFailSplitNodes++;
      continue;
    }
    //fix up the split paths

    //fprintf(flog,"Extended to node with cost %g, best is %g\n",n->totalCost,bestTotalCost);
    if(n->totalCost < bestTotalCost) {
      //fprintf(flog,"Actual cost %g < %g\n",n->totalCost,bestTotalCost);
      numImproveNodes++;
      if(CheckPath(n,timer,cutoff)) {
	//fprintf(flog,"Extend succeeded to a feasible node\n");
	bestNode = n;
	bestTotalCost = n->totalCost;
      }
      else {
	numUnreachableNodes++;
	fprintf(flog,"Extend succeeded but path check failed\n");
	continue;
      }
    }
    
    //It looks like setting search=true is detremental in some narrow passages
    //nik=(RandBool(ikSolveProbability)?TryIKExtend(n,true):NULL);
    nik=(RandBool(ikSolveProbability)?TryIKExtend(n,false):NULL);
    if(nik) {
      //fprintf(flog,"IK to node with cost %g, best is %g\n",nik->totalCost,bestTotalCost);
      numIKNodes++;
    }
    if(nik && nik->totalCost < bestTotalCost) {
      numIKImproveNodes++;
      Node* n2 = SplitEdge(nik->getParent(),nik,0.5); 
      if(!cspace->IsFeasible(n2->q)) {
	//fprintf(flog,"Extend + IK succeeded but path check failed\n");
	continue;
      }
      //fprintf(flog,"Actual cost %g <=> %g\n",nik->totalCost,bestTotalCost);
      Node* split=NULL;
      if(CheckPath(nik,timer,cutoff,&split)) {
	//fprintf(flog,"Extend + IK succeeded to a feasible node\n");
	bestNode = nik;
	bestTotalCost = nik->totalCost;
      }
      else {
	//fprintf(flog,"Extend + IK succeeded but path check failed\n");
	numUnreachableIKNodes++;

	if(split==n2) {
	  /*
	  //check braking control
	  ParabolicRamp::ParabolicRampND ramp;
	  ramp.x0=split->q;
	  ramp.dx0=split->dq;
	  ramp.SolveBraking(stateSpace->accMax);
	  if(cspace->IsFeasible(ramp.x1)) {
	    fprintf(flog,"Braking at 0.33 wouldn't help\n");
	    Node* c=AddChild(split,ramp);
	    if(c->totalCost < bestTotalCost) {
	      fprintf(flog,"Braking would help objective function\n");
	      if(c->edgeFromParent().e->IsVisible()) {
		fprintf(flog,"Braking worked!\n");
		bestNode = c;
		bestTotalCost = c->totalCost;
	      }
	      else {
		fprintf(flog,"Braking failed.\n");
		split->eraseChild(c);
	      }
	    }
	    else {
	      fprintf(flog,"Braking wouldn't help\n");
	      split->eraseChild(c);
	    }
	  }
	  */
	}
      }
    }
  }

  fprintf(flog,"Stats:\n");
  fprintf(flog,"  Existing %d, %d ik, %d improve, %d unreachable\n",numExistingNodes,numIKExisting,numIKExistingImprove,numIKExistingUnreachable);
  fprintf(flog,"  New %d, %d fail split, %d improve, %d unreachable\n",numGeneratedNodes,numFailSplitNodes,numImproveNodes,numUnreachableNodes);
  fprintf(flog,"  IK %d, %d improve, %d unreachable\n",numIKNodes,numIKImproveNodes,numUnreachableIKNodes);

  if(!bestNode) {
    fprintf(flog,"Failed to improve upon old path\n");
    Assert(path.IsValid());
    Real t=timer.ElapsedTime();
    if(cutoff > t) {
      fprintf(flog,"Devoting %g seconds to smoothing old path\n",cutoff-t);
      SmartShortcut(tstart,path,cutoff-t);
      Assert(path.IsValid());
    }
    return Timeout;
  }

  //success reducing path cost
  fprintf(flog,"Successfully reduced path cost to %g with %g time left\n",bestTotalCost,cutoff-timer.ElapsedTime());
  assert(bestNode->reachable);
  assert(bestNode->terminalCost <= bestTotalCost);

  //read out the DynamicPath
  n = bestNode;
  vector<Node*> npath;
  while(n->getParent() != NULL) {
    assert(n->reachable);
    npath.push_back(n);
    n = n->getParent();
  }
  reverse(npath.begin(),npath.end());
  vector<Real> velMax,accMax,xMin,xMax;
  swap(velMax,path.velMax);
  swap(accMax,path.accMax);
  swap(xMin,path.xMin);
  swap(xMax,path.xMax);
  path.ramps.resize(0);
  for(size_t i=0;i<npath.size();i++)
    path.Concat(npath[i]->edgeFromParent().e->path);
  swap(velMax,path.velMax);
  swap(accMax,path.accMax);
  swap(xMin,path.xMin);
  swap(xMax,path.xMax);

  if(!Vector(path.ramps.back().dx1).isZero()) {
    fprintf(flog,"Weird, path does not end in zero velocity?\n");
    fprintf(flog,"%s\n",LexicalCast(Vector(path.ramps.back().dx1)).c_str());
    fprintf(flog,"Last milestone %s, %s\n",LexicalCast(bestNode->q).c_str(),LexicalCast(bestNode->dq).c_str());
    fprintf(flog,"About to Abort() after getchar...\n");
    getchar();
    Abort();
  }
  Assert(Vector(path.ramps.back().dx1).isZero());

  Assert(path.IsValid());
  if(cutoff > timer.ElapsedTime()) {
    Vector xold = Vector(path.ramps.back().x1);
    fprintf(flog,"Devoting %g seconds to smoothing new path\n",cutoff-timer.ElapsedTime());
    SmartShortcut(tstart,path,cutoff-timer.ElapsedTime());
    assert(path.IsValid());
    assert(xold == Vector(path.ramps.back().x1));
  }
  return Success;
}


} //namespace Klampt