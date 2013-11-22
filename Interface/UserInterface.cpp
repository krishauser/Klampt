#include "UserInterface.h"
#include "Planning/RealTimePlanner.h"
#include "Planning/RealTimeRRTPlanner.h"
#include "Planning/RealTimeIKPlanner.h"
#include <robotics/IKFunctions.h>
#include <GLdraw/GL.h>
#include <GLdraw/drawExtra.h>
#include <sstream>


/** @brief Adaptor for sending paths to PhysicalRobotInterface from a
 * RealTimePlannerBase.
 */
class MotionQueueInterfaceSender : public SendPathCallbackBase
{
public:
  int blah;
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
    inputProcessor = new StandardInputProcessor;
    inputProcessor->world = world;
    inputProcessor->viewport = viewport;
  }
  else inputProcessor->Reset();
  return "";
}

bool InputProcessingInterface::ObjectiveChanged()
{
  if(!inputProcessor) return false;
  return inputProcessor->HasUpdate();
}

SmartPointer<PlannerObjectiveBase> InputProcessingInterface::GetObjective()
{
  if(!inputProcessor) return NULL;
  PlannerObjectiveBase* obj = inputProcessor->MakeObjective(GetRobot());
  return obj;
}

CartesianObjective* InputProcessingInterface::GetCartesianObjective()
{
  currentObjective = GetObjective();
  if(!currentObjective) return NULL;
  CartesianObjective* pobj = dynamic_cast<CartesianObjective*>(&*currentObjective);
  assert(pobj != NULL);
  return pobj;
}

void InputProcessingInterface::DrawGL()
{
  if(inputProcessor) inputProcessor->DrawGL();
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
  :planner(NULL),lastPlanTime(0),nextPlanTime(0)
{}

PlannerCommandInterface::~PlannerCommandInterface()
{
  SafeDelete(planner);
}

string PlannerCommandInterface::ActivateEvent(bool enabled)
{
  InputProcessingInterface::ActivateEvent(enabled);
  if(planner) {
    planner->currentPath.ramps.resize(0);
    lastPlanTime = nextPlanTime = robotInterface->GetCurTime();
  }
  return "";
}

string PlannerCommandInterface::UpdateEvent()
{
  InputProcessingInterface::UpdateEvent();
  if(!planner) return "";
  if(planner->currentPath.ramps.empty()) {
    if(robotInterface->GetEndTime() <= robotInterface->GetCurTime()) { //done moving
      printf("Planner initialized\n");
      Config q;
      robotInterface->GetCurConfig(q);
      planner->SetConstantPath(q);
    }
    else
      //wait until done moving
      return "";
  }

  //wait until next planning step, otherwise try planning
  if(robotInterface->GetCurTime()  < nextPlanTime) return "";

  stringstream ss;

  double t = robotInterface->GetCurTime();
  lastPlanTime = t;
  if(ObjectiveChanged()) {
    CartesianObjective* obj=GetCartesianObjective();
    planner->Reset(obj);
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
  PlannerCommandInterface::ActivateEvent(enabled);
  if(!planner) {
    printf("IK planner activated\n");
    cspace = new SingleRobotCSpace(*world,0,settings);
    
    planner = new RealTimeIKPlanner;
    planner->protocol = RealTimePlannerBase::Constant;
    planner->currentSplitTime=0.1;
    planner->currentPadding=0.05;
    planner->SetSpace(cspace);
    planner->sendPathCallback = new MotionQueueInterfaceSender(robotInterface);
  }
  return "";
}

string RRTCommandInterface::ActivateEvent(bool enabled)
{
  PlannerCommandInterface::ActivateEvent(enabled);
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
    planner->sendPathCallback = new MotionQueueInterfaceSender(robotInterface);
  }
  return "";
}
