#include "UserInterface.h"
#include "Planning/RealTimePlanner.h"
#include "Planning/RealTimeRRTPlanner.h"
#include "Planning/RealTimeIKPlanner.h"
#include <robotics/IKFunctions.h>
#include <GLdraw/GL.h>
#include <GLdraw/drawextra.h>
#include <sstream>
#ifdef CYGWIN
#undef WIN32
#endif //CYGWIN
#ifndef WIN32
#include <unistd.h>
#endif //WIN32

/** @brief Adaptor for sending paths to PhysicalRobotInterface from a
 * RealTimePlannerBase.
 */
class MotionQueueInterfaceSender : public SendPathCallbackBase
{
public:
  MotionQueueInterface* iface;

  MotionQueueInterfaceSender(MotionQueueInterface* _interface)
    :iface(_interface)
  {}
  
  virtual bool Send(Real tplanstart,Real tcut,const ParabolicRamp::DynamicPath& path) {
    //impose hard real time constraint
    Real t= iface->GetCurTime();
    printf("Plan time elapsed = %g, split time ... %g\n",t-tplanstart,tcut);
    if(t >= tplanstart+tcut) {
      //too late
      printf("Path send violates hard real-time constraint\n");
      return false;
    }
    
    MotionQueueInterface::MotionResult res=iface->SendPathImmediate(tplanstart+tcut,path);
    if(res == MotionQueueInterface::TransmitError) {
      return false;
    }
    else if(res != MotionQueueInterface::Success) {
      printf("Hmmm.. failed path check.  Should debug!\n");
      return false;
    }
      
    return true;
  }
};


void PrintConfig(ostream& out,const vector<double>& q)
{
  for(size_t i=0;i<q.size();i++) out<<q[i]<<" ";
}

double MaxAbsError(const vector<double>& a,const vector<double>& b)
{
  assert(a.size()==b.size());
  double e=0;
  for(size_t i=0;i<a.size();i++)
    e = Max(e,Abs(a[i]-b[i]));
  return e;
}






RobotUserInterface::RobotUserInterface()
  :world(NULL),viewport(NULL),robotInterface(NULL)
{}

void RobotUserInterface::GetClickRay(int mx,int my,Ray3D& ray) const
{
  viewport->getClickSource(mx,viewport->h-my,ray.source);
  viewport->getClickVector(mx,viewport->h-my,ray.direction);
}

string JointCommandInterface::ActivateEvent(bool enabled)
{
  currentLink=-1;
  sendCommand = false;
  return "";
}
 
string JointCommandInterface::MouseInputEvent(int mx,int my,bool drag) 
{ 
  if(drag) {
    if(currentLink >= 0) {
      //alter current desired configuration
      Config q;
      robotInterface->GetEndConfig(q);
      Robot* robot=GetRobot();
      robot->UpdateConfig(q);
      for(size_t i=0;i<robot->drivers.size();i++) {
	if(robot->DoesDriverAffect(i,currentLink)) {
	  Real val = robot->GetDriverValue(i);
	  val = Clamp(val+my*0.02,robot->drivers[i].qmin,robot->drivers[i].qmax);
	  robot->SetDriverValue(i,val);
	}
      }
      command = robot->q;
      sendCommand = true;
    }
    stringstream ss;
    ss<<"Drag "<<currentLink<<" "<<mx<<" "<<my<<endl;
    return ss.str();
  }
  else {
    Ray3D ray;
    GetClickRay(mx,my,ray);

    Config q;
    robotInterface->GetCurConfig(q);
    Robot* robot=GetRobot();
    robot->UpdateConfig(q);
    int link;
    Vector3 localPos;
    RobotInfo* rob=world->ClickRobot(ray,link,localPos);

    if(rob) {
      currentLink = link;
      world->robots[0].view.SetGrey();
      world->robots[0].view.SetColor(currentLink,GLColor(1,1,0));
    }
    else {
      world->robots[0].view.SetGrey();
      currentLink = -1;
    }
    return "";
  }
}

string JointCommandInterface::UpdateEvent()
{
  if(sendCommand) {
    robotInterface->SendMilestoneImmediate(command);
    sendCommand = false;
  }
  return "";
}



InputProcessingInterface::InputProcessingInterface()
{}

void InputProcessingInterface::SetProcessor(SmartPointer<InputProcessorBase>& processor)
{
  inputProcessor = processor;
  inputProcessor->world = world;
  inputProcessor->viewport = viewport;
}

string InputProcessingInterface::ActivateEvent(bool enabled)
{
  if(!inputProcessor) {
    if(enabled) {
      inputProcessor = new StandardInputProcessor;
      inputProcessor->world = world;
      inputProcessor->viewport = viewport;
      inputProcessor->Activate(enabled);
    }
  }
  else inputProcessor->Activate(enabled);
  return "";
}

bool InputProcessingInterface::ObjectiveChanged()
{
  if(!inputProcessor) return false;
  return inputProcessor->HasUpdate();
}

SmartPointer<PlannerObjectiveBase> InputProcessingInterface::GetObjective()
{
  if(!inputProcessor) {
    currentObjective = NULL;
  }
  else {
    PlannerObjectiveBase* obj = inputProcessor->MakeObjective(GetRobot());
    currentObjective = obj;
  }
  return currentObjective;
}

CartesianObjective* InputProcessingInterface::GetCartesianObjective()
{
  GetObjective();
  if(!currentObjective) return NULL;
  CartesianObjective* pobj = dynamic_cast<CartesianObjective*>(&*currentObjective);
  assert(pobj != NULL);
  return pobj;
}

void InputProcessingInterface::DrawGL()
{
  if(inputProcessor) inputProcessor->DrawGL();
  if(currentObjective) {
    Robot* robot=GetRobot();
    CartesianObjective* cobj = dynamic_cast<CartesianObjective*>(&*currentObjective);
    if(cobj) {
      glPointSize(5.0);
      glEnable( GL_POINT_SMOOTH);
      glDisable( GL_LIGHTING);
      glBegin( GL_POINTS);
      glColor3f(0, 1, 1);
      glVertex3v(robot->links[cobj->ikGoal.link].T_World
		 * cobj->ikGoal.localPosition);
      glColor3f(0, 1, 0.5);
      glVertex3v(cobj->ikGoal.endPosition);
      glEnd();
    }
    ConfigObjective* qobj = dynamic_cast<ConfigObjective*>(&*currentObjective);
    if(qobj) {
      world->robots[0].view.SetColors(GLColor(1,0.5,0,0.5));
      glEnable(GL_LIGHTING);
      robot->UpdateConfig(qobj->qgoal);
      world->robots[0].view.Draw();
      glDisable(GL_LIGHTING);
    }
    CartesianTrackingObjective* ptrack = dynamic_cast<CartesianTrackingObjective*>(&*currentObjective);
    if(ptrack) {
      glPointSize(5.0);
      glEnable( GL_POINT_SMOOTH);
      glDisable( GL_LIGHTING);
      glBegin( GL_POINTS);
      glColor3f(0, 1, 1);
      glVertex3v(robot->links[ptrack->link].T_World
		 * ptrack->localPosition);
      glEnd();
      glLineWidth(3.0);
      glColor3f(0, 1, 0.5);
      glBegin(GL_LINE_STRIP);
      for(size_t i=0;i<ptrack->positions.size();i++) 
	glVertex3v(ptrack->positions[i]);
      glEnd();
      glLineWidth(1.0);
    }
    //TODO: others
  }
}

string InputProcessingInterface::MouseInputEvent(int mx, int my, bool drag)
{
  Config q;
  robotInterface->GetEndConfig(q);
  GetRobot()->UpdateConfig(q);
  if (drag) {
    inputProcessor->Drag(mx,my);
    stringstream ss;
    ss << "Drag " << mx << " " << my << endl;
    return ss.str();
  } else {
    inputProcessor->Hover(mx,my);
    return "";
  }
}

string InputProcessingInterface::SpaceballEvent(const RigidTransform& T)
{
  Config q;
  robotInterface->GetEndConfig(q);
  GetRobot()->UpdateConfig(q);
  inputProcessor->Spaceball(T);
  return "";
}

string InputProcessingInterface::UpdateEvent()
{
  inputProcessor->SetGlobalTime(robotInterface->GetCurTime());
  return "";
}

string IKCommandInterface::UpdateEvent()
{
  InputProcessingInterface::UpdateEvent();
  CartesianObjective* pobj = GetCartesianObjective();
  if(!pobj) return "";

  vector<IKGoal> problem(1, pobj->ikGoal);
  
  Config q;
  robotInterface->GetEndConfig(q);
  Robot* robot = GetRobot();
  robot->UpdateConfig(q);
  int iters = 100;
  bool res = SolveIK(*robot, problem, 1e-3, iters, 0);
  
  robotInterface->SendMilestoneImmediate(robot->q);
  
  return "";
}



PlannerCommandInterface::PlannerCommandInterface()
  :planner(NULL),lastPlanTime(0),nextPlanTime(0),startObjectiveThreshold(Inf),started(false)
{}

PlannerCommandInterface::~PlannerCommandInterface()
{
  SafeDelete(planner);
}

string PlannerCommandInterface::ActivateEvent(bool enabled)
{
  InputProcessingInterface::ActivateEvent(enabled);
  started = false;
  if(planner) {
    planner->currentPath.ramps.resize(0);
    lastPlanTime = nextPlanTime = robotInterface->GetCurTime();
    planner->sendPathCallback = new MotionQueueInterfaceSender(robotInterface);
  }
  return "";
}

string PlannerCommandInterface::UpdateEvent()
{
  InputProcessingInterface::UpdateEvent();
  if(!planner) return "";
  if(!started) {
    //2 conditions must be met before start: robot is stopped, robot hits the
    //startobjectivethreshold.

    //robot still moving
    if(robotInterface->GetEndTime() > robotInterface->GetCurTime()) {
      printf("Waiting until the robot stops...\n");
      return "";
    }
    if(IsInf(startObjectiveThreshold)) {
      started=true;
    }
    else if(ObjectiveChanged()) {
      SmartPointer<PlannerObjectiveBase> obj = GetObjective();
      Config q,v;
      robotInterface->GetEndConfig(q);
      v.resize(q.n,Zero);
      if(obj->TerminalCost(0,q,v) > startObjectiveThreshold) {
	printf("Waiting until the objective gets below %g, currently %g...\n",startObjectiveThreshold,obj->TerminalCost(0,q,v));
	return "";
      }
      else started=true;
    } 
    else {
      printf("Waiting for an objective...\n");
    }
  }
  if(!started) return "";
  if(planner->currentPath.ramps.empty()) {
    Config q;
    robotInterface->GetEndConfig(q);
    planner->SetConstantPath(q);
  }

  //wait until next planning step, otherwise try planning
  if(robotInterface->GetCurTime()  < nextPlanTime) return "";

  stringstream ss;

  double t = robotInterface->GetCurTime();
  lastPlanTime = t;
  if(ObjectiveChanged()) {
    planner->Reset(GetObjective());
  }

  //cout<<"Path advance "<<startPlanTime-lastPlanTime<<endl;
  /*
  //TODO: debug with mirror config at time startPlanTime, not the queried
  //config from the server
  Config qcur;
  robotInterface->GetCurConfig(qcur);
  //double check the assumptions of the planner
  if(MaxAbsError(planner->currentPath.ramps.front().x0,qcur)>1e-2) {
    cout<<"Major discrepancy between predicted and actual path"<<endl;
    cout<<"\tCur predicted config "; PrintConfig(cout,planner->currentPath.ramps.front().x0); cout<<endl;
    cout<<"\tCur actual config "; PrintConfig(cout,qcur); cout<<endl;
    getchar();
    
    robotInterface->GetCurConfig(qcur);
    if(robotInterface->GetEndTime() <= robotInterface->GetCurTime()) {
      planner->currentPath.ramps.resize(1);
      planner->currentPath.ramps[0].SetConstant(qcur);
      updatedCurrent = planner->currentPath;
      startPlanTime = robotInterface->GetCurTime();
    }
  }
  */
  //cout<<"End time "<<GetEndTime()<<", predicted end time "<<t+updatedCurrent.GetTotalTime()<<endl;    
  
  Timer timer;
  Real splitTime=0,planTime=0;
  bool res=false;
  if(planner->goal != NULL) {
    res=planner->PlanUpdate(t,splitTime,planTime);
    printf("Plan update: time %g, time elapsed %g, result %d\n",t,planTime,(int)res);
    
    ss<<"Plan "<<planTime<<", padding "<<planner->currentPadding<<", split "<<splitTime<<", res "<<res<<endl;
    assert(planner->currentPath.IsValid());
  }
  
  //some time elapsed -- we're simulating two threads
  t = robotInterface->GetCurTime();
  nextPlanTime = t + planTime;
  return ss.str();
}


string IKPlannerCommandInterface::ActivateEvent(bool enabled)
{
  if(!planner) {
    printf("IK planner activated, 150ms loop\n");
    assert(settings != NULL);
    cspace = new SingleRobotCSpace(*world,0,settings);
    
    planner = new RealTimeIKPlanner;
    planner->protocol = RealTimePlannerBase::Constant;
    planner->currentSplitTime=0.1;
    planner->currentPadding=0.05;
    planner->SetSpace(cspace);
    assert(planner->settings != NULL);
  }
  PlannerCommandInterface::ActivateEvent(enabled);
  return "";
}

string RRTCommandInterface::ActivateEvent(bool enabled)
{
  if(!planner) {
    cspace = new SingleRobotCSpace(*world,0,settings);
    
    RealTimeRRTPlanner* p=new RealTimeRRTPlanner;
    p->delta = 0.5;
    planner = p;
    //planner->protocol = RealTimePlannerBase::Constant;
    planner->protocol = RealTimePlannerBase::ExponentialBackoff;
    planner->currentSplitTime=0.1;
    planner->currentPadding=0.05;
    //test insufficient padding
    //planner->currentPadding=0.01;
    planner->SetSpace(cspace);
  }
  PlannerCommandInterface::ActivateEvent(enabled);
  return "";
}





#ifndef WIN32
RealTimePlannerDataSender::RealTimePlannerDataSender(RealTimePlannerData* _data)
  :data(_data)
{}

bool RealTimePlannerDataSender::Send(Real tplanstart,Real tcut,const ParabolicRamp::DynamicPath& path)
{
  pthread_mutex_lock(&data->mutex);
  Assert(data->startPlanTime == tplanstart);
  data->pathRefresh = true;
  data->tcut = tcut;
  data->path = path;
  pthread_mutex_unlock(&data->mutex);
  
  while(true) {
    usleep(1);
    pthread_mutex_lock(&data->mutex);
    if(!data->pathRefresh) {  //this signals that the calling thread picked up the data
      bool res = data->pathRefreshSuccess;
      pthread_mutex_unlock(&data->mutex);
      return res;
    }
    pthread_mutex_unlock(&data->mutex);
  }
}  

void* planner_thread_func(void * ptr)
{
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(ptr);
  while (true) {
    //first wait for a lock
    pthread_mutex_lock(&data->mutex);

    //parse the data
    if(!data->active) {
      //quit
      pthread_mutex_unlock(&data->mutex);
      return NULL;
    }
    assert(data->pathRefresh == false);
    bool start=false;
    Real startTime;
    if(data->objective) {
      start=true;
      startTime = data->startPlanTime = data->globalTime;
    }
    if(data->objective != data->planner->goal) {
      if(data->objective) {
	data->planner->Reset(data->objective);      }
      else {
	printf("Planning thread: NULL objective function set...\n");
	data->planner->Reset(NULL);
      }
    }
    pthread_mutex_unlock(&data->mutex);

    if(start) {
      //do the planning -- the callback will output the result to the
      //UI thread
      Real splitTime, planTime;
      bool res = data->planner->PlanUpdate(startTime, splitTime, planTime);
      
      printf("Planning thread: result %d\n",(int)res);    
    }
    usleep(1);
  }
  return NULL;
}


MTPlannerCommandInterface::MTPlannerCommandInterface()
  : planner(NULL)
{
  data.mutex = PTHREAD_MUTEX_INITIALIZER;
}

MTPlannerCommandInterface::~MTPlannerCommandInterface()
{
  SafeDelete(planner);
}
  
string MTPlannerCommandInterface::ActivateEvent(bool enabled)
{
  InputProcessingInterface::ActivateEvent(enabled);
  if(enabled) {
    Assert(robotInterface->GetEndTime() <= robotInterface->GetCurTime());
    Config q;
    robotInterface->GetCurConfig(q);
    planner->SetConstantPath(q);

    Assert(&planner->cspace->world == &planningWorld);
    data.planner = planner;
    data.planner->sendPathCallback = new RealTimePlannerDataSender(&data);
    data.active = true;
    data.globalTime = 0;
    data.pathRefresh = false;
    pthread_create(&planningThread,NULL,planner_thread_func,&data);
    printf("Creating planning thread\n");
  }
  else {
    pthread_mutex_lock(&data.mutex);
    data.active = false;
    pthread_mutex_unlock(&data.mutex);
    pthread_join(planningThread,NULL);
  }
  return "";
}

string MTPlannerCommandInterface::UpdateEvent()
{
  if (!planner)
    return "";

  inputProcessor->SetGlobalTime(robotInterface->GetCurTime());

  //tell the planner to stop if there's a new objective
  if(ObjectiveChanged()) {
    printf("\n");
    printf("****** STOP PLANNING **********\n");
    printf("\n");
    planner->StopPlanning();
  }

  //now lock the planning thread's mutex
  pthread_mutex_lock(&data.mutex);

  //update the objective function if necessary
  if(ObjectiveChanged()) {
    //set the objective function
    PlannerObjectiveBase* obj = inputProcessor->MakeObjective(planner->cspace->GetRobot());
    data.objective = obj;
  }
  
  //see if the planning thread has a plan update
  if(data.pathRefresh) {
    //if so, mark that it's refreshed
    data.pathRefresh = false;
    MotionQueueInterface::MotionResult res = robotInterface->SendPathImmediate(data.startPlanTime+data.tcut,data.path);
    Real t=robotInterface->GetCurTime();
    if(res == MotionQueueInterface::Success) {
      data.pathRefreshSuccess = true;
      printf("Exec thread: Planner successful, split %g, delay %g\n",data.tcut,t - data.startPlanTime);
    }
    else {
      data.pathRefreshSuccess = false;
      printf("Exec thread: Planner overrun, split %g, delay %g\n",data.tcut,t - data.startPlanTime);
    }
  }
  else {
    //printf("Sim thread: waiting for plan to complete\n");
  }
  data.globalTime = robotInterface->GetCurTime();
  pthread_mutex_unlock(&data.mutex);
  return "";
}

string MTIKPlannerCommandInterface::ActivateEvent(bool enabled)
{
  
  if (!planner) {
    CopyWorld(*world,planningWorld);
    cspace = new SingleRobotCSpace(planningWorld, 0, settings);
    
    planner = new RealTimeIKPlanner;
    //planner->protocol = RealTimePlannerBase::Constant;
    planner->currentSplitTime = 0.05;
    planner->currentPadding = 0.025;
    planner->currentExternalPadding = 0.02;
    planner->SetSpace(cspace);
  }
  return MTPlannerCommandInterface::ActivateEvent(enabled);
}

string MTRRTCommandInterface::ActivateEvent(bool enabled)
{
  if (!planner) {
    CopyWorld(*world,planningWorld);
    cspace = new SingleRobotCSpace(planningWorld, 0, settings);
    
    //RealTimeRRTPlanner* p = new RealTimeRRTPlanner;
    RealTimeTreePlanner* p = new RealTimeTreePlanner;
    p->delta = 0.5;
    planner = p;
    planner->SetSpace(cspace);
  }
  return MTPlannerCommandInterface::ActivateEvent(enabled);
}
#endif //WIN32
