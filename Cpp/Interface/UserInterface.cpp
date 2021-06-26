#include "UserInterface.h"
#include "Planning/RealTimePlanner.h"
#include "Planning/RealTimeRRTPlanner.h"
#include "Planning/RealTimeIKPlanner.h"
#include <KrisLibrary/robotics/IKFunctions.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <sstream>
#ifndef WIN32
#include <unistd.h>
#endif //WIN32
using namespace GLDraw;

namespace Klampt {

const static Real gPlannerStopDeltaThreshold = 0.2;

/** @brief Adaptor for sending paths to PhysicalRobotInterface from a
 * RealTimePlanner.
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
    /*
    Real t= iface->GetCurTime();
    printf("Plan time elapsed = %g, split time ... %g\n",t-tplanstart,tcut);
    if(t >= tplanstart+tcut) {
      //too late
      printf("MotionQueueInterfaceSender: Path send violates hard real-time constraint\n");
      return false;
    }
    */
    MotionQueueInterface::MotionResult res=iface->SendPathImmediate(tplanstart+tcut,path);
    if(res == MotionQueueInterface::TransmitError) {
      printf("MotionQueueInterfaceSender: Transmission error\n");
      return false;
    }
    else if(res != MotionQueueInterface::Success) {
      printf("MotionQueueInterfaceSender: Hmmm.. failed path check.  Should debug!\n");
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
  :world(NULL),viewport(NULL),planningWorld(NULL),settings(NULL),robotInterface(NULL)
{}

void RobotUserInterface::GetClickRay(int mx,int my,Ray3D& ray) const
{
  viewport->getClickSource(float(mx),float(viewport->h-my),ray.source);
  viewport->getClickVector(float(mx),float(viewport->h-my),ray.direction);
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
      RobotModel* robot=GetRobot();
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
    RobotModel* robot=GetRobot();
    robot->UpdateConfig(q);
    int link;
    Vector3 localPos;
    RobotModel* rob=world->RayCastRobot(ray,link,localPos);

    if(rob) {
      currentLink = link;
      world->robotViews[0].PushAppearance();
      world->robotViews[0].SetColor(currentLink,GLColor(1,1,0));
    }
    else {
      world->robotViews[0].RestoreAppearance();
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
{
}

InputProcessingInterface::~InputProcessingInterface()
{
}

void InputProcessingInterface::SetProcessor(shared_ptr<InputProcessorBase>& processor)
{
  inputProcessor = processor;
  inputProcessor->world = world;
  inputProcessor->viewport = viewport;
}

string InputProcessingInterface::ActivateEvent(bool enabled)
{
  if(!inputProcessor) {
    if(enabled) {
      inputProcessor.reset(new StandardInputProcessor);
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

shared_ptr<PlannerObjectiveBase> InputProcessingInterface::GetObjective()
{
  currentObjective = NULL;
  
  if(inputProcessor) {
    PlannerObjectiveBase* obj = inputProcessor->MakeObjective(GetRobot());
    currentObjective.reset(obj);
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
    RobotModel* robot=GetRobot();
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
      world->robotViews[0].SetColors(GLColor(1,0.5,0,0.5));
      glEnable(GL_LIGHTING);
      robot->UpdateConfig(qobj->qgoal);
      world->robotViews[0].Draw();
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
  RobotModel* robot = GetRobot();
  robot->UpdateConfig(q);
  int iters = 100;
  bool res = SolveIK(*robot, problem, 1e-3, iters, 0);
  
  robotInterface->SendMilestoneImmediate(robot->q);
  
  return "";
}



PlannerCommandInterface::PlannerCommandInterface()
  :lastPlanTime(0),nextPlanTime(0),startObjectiveThreshold(Inf),started(false)
{}

PlannerCommandInterface::~PlannerCommandInterface()
{
}

string PlannerCommandInterface::ActivateEvent(bool enabled)
{
  InputProcessingInterface::ActivateEvent(enabled);
  started = false;
  if(planner) {
    planner->currentPath.ramps.resize(0);
    lastPlanTime = nextPlanTime = robotInterface->GetCurTime();
    planner->sendPathCallback.reset(new MotionQueueInterfaceSender(robotInterface));
  }
  return "";
}

string PlannerCommandInterface::Instructions() const
{
  if(!started) {
    if(robotInterface->GetEndTime() > robotInterface->GetCurTime()) {
      return "Waiting until the physical robot stops...";
    }
    else if(currentObjective) {
      stringstream ss;
      ss<<"Waiting until the error gets below threshold "<<startObjectiveThreshold;
      return ss.str();
    }
    else
      return InputProcessingInterface::Instructions();
  }
  return InputProcessingInterface::Instructions();
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
      return "";
    }
    if(IsInf(startObjectiveThreshold)) {
      started=true;
    }
    else if(ObjectiveChanged()) {
      shared_ptr<PlannerObjectiveBase> obj = GetObjective();
      Assert(currentObjective != NULL);
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
  if(robotInterface->HadExternalChange()) {
    if(robotInterface->GetCurTime()  < robotInterface->GetEndTime()) {
      fprintf(stderr,"Cannot handle an externally changed robot that is still moving\n");
      return "";
    }
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
    plannerObjective = GetObjective();
    planner->Reset(plannerObjective);
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
  if(planner->Objective() != NULL) {
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
    cspace.reset(new SingleRobotCSpace(*planningWorld,0,settings));
    
    planner = make_shared<RealTimePlanner>();
    planner->planner.reset(new DynamicIKPlanner);
    //planner = new RealTimePerturbationIKPlanner;
    //planner->LogBegin();
    //planner->protocol = RealTimePlanner::Constant;
    planner->currentSplitTime=0.10;
    planner->currentPadding=0.05;
    planner->SetSpace(cspace.get());
    assert(planner->planner->settings != NULL);
  }
  PlannerCommandInterface::ActivateEvent(enabled);
  return "";
}

string RRTCommandInterface::ActivateEvent(bool enabled)
{
  if(!planner) {
    planningWorld->InitCollisions();
    cspace.reset(new SingleRobotCSpace(*planningWorld,0,settings));
    
    planner = make_shared<RealTimePlanner>();
    DynamicRRTPlanner* p = new DynamicRRTPlanner;
    p->delta = 0.5;
    planner->planner.reset(p);
    planner->planner->LogBegin();
    //planner->protocol = RealTimePlanner::Constant;
    planner->protocol = RealTimePlanner::ExponentialBackoff;
    planner->currentSplitTime=0.1;
    planner->currentPadding=0.05;
    //test insufficient padding
    //planner->currentPadding=0.01;
    planner->SetSpace(cspace.get());
  }
  PlannerCommandInterface::ActivateEvent(enabled);
  return "";
}






MTPlannerCommandInterface::MTPlannerCommandInterface()
  : startObjectiveThreshold(Inf),started(false)
{
}

MTPlannerCommandInterface::~MTPlannerCommandInterface()
{
}

string MTPlannerCommandInterface::Instructions() const
{
  if(!started) {
    if(robotInterface->GetEndTime() > robotInterface->GetCurTime()) {
      return "Waiting until the physical robot stops...";
    }
    else if(currentObjective) {
      stringstream ss;
      ss<<"Waiting until the error gets below threshold "<<startObjectiveThreshold;
      return ss.str();
    }
    else
      return InputProcessingInterface::Instructions();
  }
  return InputProcessingInterface::Instructions();
}
  
string MTPlannerCommandInterface::ActivateEvent(bool enabled)
{
  InputProcessingInterface::ActivateEvent(enabled);
  if(enabled) {
    Assert(robotInterface->GetEndTime() <= robotInterface->GetCurTime());

    Config qstart;
    robotInterface->GetCurConfig(qstart);
    planningThread.SetStartConfig(qstart);
    planningThread.Start();
  }
  else {
    planningThread.Stop();
  }
  return "";
}

string MTPlannerCommandInterface::UpdateEvent()
{
  if (!planningThread.planner)
    return "";

  inputProcessor->SetGlobalTime(robotInterface->GetCurTime());

  if(!started) {
    //2 conditions must be met before start: robot is stopped, robot hits the
    //startobjectivethreshold.

    //robot still moving
    if(robotInterface->GetEndTime() > robotInterface->GetCurTime()) {
      return "";
    }
    if(IsInf(startObjectiveThreshold)) {
      started=true;
    }
    else if(ObjectiveChanged()) {
      shared_ptr<PlannerObjectiveBase> obj = GetObjective();
      Assert(currentObjective != NULL);
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

  if(robotInterface->HadExternalChange()) {
    if(robotInterface->GetCurTime()  < robotInterface->GetEndTime()) {
      fprintf(stderr,"Cannot handle an externally changed robot that is still moving\n");
      return "";
    }
    Config q;
    robotInterface->GetEndConfig(q);
    planningThread.SetStartConfig(q);
  }

  //tell the planner to stop if there's a new objective
  shared_ptr<PlannerObjectiveBase> obj;
  bool changedObjective = false;
  if(ObjectiveChanged()) {
    RobotModel* robot = planningWorld->robots[0].get();
    changedObjective = true;
    obj.reset(inputProcessor->MakeObjective(robot));
    //this is the visualization objective -- must be a different pointer
    currentObjective.reset(inputProcessor->MakeObjective(robot));
    //evaluate the objective
    PlannerObjectiveBase* oldObj = planningThread.GetObjective();
    if((oldObj && !obj) || (oldObj && oldObj->Delta(obj.get()) > gPlannerStopDeltaThreshold)) {
      printf("\n");
      if(obj)
	printf("********* Objective changed, STOP PLANNING **********\n");
      else
	printf("********* Objective deleted, STOP PLANNING **********\n");
      printf("\n");
      planningThread.BreakPlanning();
    }
  }

  if(changedObjective) {
    //planning thread needs this to be persistent across GetObjective() calls
    planningThread.SetObjective(obj);
  }

  //send any pending updates to the robotinterface
  planningThread.SendUpdate(robotInterface);
  return "";
}

string MTIKPlannerCommandInterface::ActivateEvent(bool enabled)
{
  if (!cspace) {
    Assert(planningWorld != NULL);
    planningWorld->InitCollisions();
    cspace.reset(new SingleRobotCSpace(*planningWorld, 0, settings));
    
    auto planner = make_shared<RealTimePlanner>();
    planner->planner.reset(new DynamicIKPlanner);
    //planner->protocol = RealTimePlanner::Constant;
    planner->currentSplitTime = 0.05;
    planner->currentPadding = 0.025;
    planner->currentExternalPadding = 0.02;
    planner->SetSpace(cspace.get());
    planningThread.SetPlanner(planner);
  }
  return MTPlannerCommandInterface::ActivateEvent(enabled);
}

string MTRRTCommandInterface::ActivateEvent(bool enabled)
{
  if (!cspace) {
    Assert(planningWorld != NULL);
    planningWorld->InitCollisions();
    cspace.reset(new SingleRobotCSpace(*planningWorld, 0, settings));
    
    //DynamicRRTPlanner* p = new DynamicRRTPlanner;
    DynamicHybridTreePlanner* p = new DynamicHybridTreePlanner;
    p->delta = 0.5;
    auto planner = make_shared<RealTimePlanner>();
    planner->planner.reset(p);
    planner->SetSpace(cspace.get());
    planningThread.SetPlanner(planner);
  }
  return MTPlannerCommandInterface::ActivateEvent(enabled);
}


} // namespace Klampt
