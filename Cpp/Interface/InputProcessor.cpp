#include "InputProcessor.h"
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/utils/AnyCollection.h>
#include <sstream>
using namespace GLDraw;
using namespace std;
using namespace Klampt;

InputProcessorBase::InputProcessorBase()
  : world(NULL),viewport(NULL),currentTime(0)
{}

RobotModel* InputProcessorBase::GetRobot() const
{
  return world->robots[0].get();
}

void InputProcessorBase::GetClickRay(int mx, int my, Ray3D& ray) const
{
  viewport->getClickSource(mx, viewport->h - my, ray.source);
  viewport->getClickVector(mx, viewport->h - my, ray.direction);
}

StandardInputProcessor::StandardInputProcessor()
  :move(false),changed(false),currentLink(-1),useSpaceball(false),pathCost(0)
{}

void StandardInputProcessor::Activate(bool enabled)
{
  currentLink = -1;			    
  move = false;
  changed = false;
  useSpaceball = false;
}

void StandardInputProcessor::Hover(int mx,int my)
{
  if(move) changed = true;
  useSpaceball = false;
  move = false;
  Ray3D ray;
  GetClickRay(mx, my, ray);
  
  int link;
  Vector3 localPos;
  RobotModel* rob = world->RayCastRobot( ray, link, localPos);
  RobotModel* robot = GetRobot();
  if (rob) {
    currentLink = link;
    currentPoint = localPos;
    currentDestination = robot->links[currentLink].T_World*localPos;
    world->robotViews[0].RestoreAppearance();
    world->robotViews[0].PushAppearance();
    world->robotViews[0].SetColor(currentLink,GLColor(1, 1, 0));
  } else {
    world->robotViews[0].RestoreAppearance();
    currentLink = -1;
  }
}

void StandardInputProcessor::Drag(float dx,float dy)
{
  if (currentLink < 0) return;

  move = true;
  changed = true;
  //alter current desired configuration
  Vector3 wp = currentDestination;
  Vector3 ofs;
  Vector3 vv;
  viewport->getViewVector(vv);
  Real d = (wp - viewport->position()).dot(vv);
  viewport->getMovementVectorAtDistance(dx, -dy, d, ofs);
    
  currentDestination += ofs;
}

void StandardInputProcessor::Spaceball(const RigidTransform& T)
{
  if(currentLink >= 0) {
    RigidTransform Tlink;
    Tlink = GetRobot()->links[currentLink].T_World;

    currentDesiredTransform = Tlink*T;
    useSpaceball = true;
  }
}

PlannerObjectiveBase* StandardInputProcessor::MakeObjective(RobotModel* robot) 
{ 
  if(!move) return NULL;
  assert(currentLink >= 0);
  changed = false;
  CartesianObjective* pgoal = new CartesianObjective(robot);
  pgoal->ikGoal.link = currentLink;
  pgoal->ikGoal.localPosition = currentPoint;
  if(useSpaceball) {
    goal.SetFixedRotation(currentDesiredTransform.R);
    goal.SetFixedPosition(currentDesiredTransform.t);
  }
  else {
    pgoal->ikGoal.SetFixedPosition(currentDestination);
  }
  return pgoal;
}

void StandardInputProcessor::DrawGL()
{
  if(currentLink < 0) return;
  RobotModel* robot=GetRobot();
  glPointSize(5.0);
  glEnable( GL_POINT_SMOOTH);
  glDisable( GL_LIGHTING);
  glBegin( GL_POINTS);
  glColor3f(0, 1, 1);
  glVertex3v(robot->links[currentLink].T_World
	     * currentPoint);
  glColor3f(0, 1, 0.5);
  glVertex3v(currentDestination);
  glEnd();
}



PredictiveExtrapolationInputProcessor::PredictiveExtrapolationInputProcessor()
  :currentInputTime(0), numInputs(0), sumVelocity(Zero), 
   alpha(2.0),weightDecay(0.025),speedDecay(0.1),
   predictionOffset(0.0),tracking(false), lastObjective(NULL)
{}

void PredictiveExtrapolationInputProcessor::Activate(bool enabled)
{
  StandardInputProcessor::Activate(enabled);
  currentInputTime = 0;
  sumVelocity.setZero();
  numInputs = 0;
  lastObjective = NULL;
}
void PredictiveExtrapolationInputProcessor::Hover(int mx,int my)
{
  StandardInputProcessor::Hover(mx,my);
  currentInputTime = 0;
  sumVelocity.setZero();
  numInputs = 0;
  lastObjective = NULL;
}

void PredictiveExtrapolationInputProcessor::Drag(float mx,float my)
{
  if(currentLink < 0) return;

  Vector3 oldDest = currentDestination;
  StandardInputProcessor::Drag(mx,my);
  if(numInputs > 0) {
    if(currentTime != currentInputTime) {
      Vector3 v = (currentDestination-oldDest)/(currentTime-currentInputTime);

      //vs = alpha int e(-alpha t) v(t) dt
      //   = alpha sum[k=0 to n] int[tk to tk+1] e(-alpha t) vk dt
      //   = alpha int[0 to t1] e(-alpha t)v0 dt + alpha sum[k=1 to n]int[tk to tk+1] e(-alpha t) vk dt
      //   = v0 alpha int[0 to t1] e(-alpha t) dt + e(-alpha t1) vs'
      //   = v0 [1-e(-alpha t1)] + e(-alpha t1) vs'
      sumVelocity += (1.0-Exp(-alpha*(currentTime-currentInputTime)))*(v-sumVelocity);
      //sumVelocity += alpha*(v-sumVelocity);
    }
  }
  numInputs++;
  currentInputTime = currentTime;
}

void PredictiveExtrapolationInputProcessor::SetPredictionTime(Real splitTime)
{
  predictionOffset = splitTime;
}
  
PlannerObjectiveBase* PredictiveExtrapolationInputProcessor::MakeObjective(RobotModel* robot)
{
  if(!move) return NULL;
  assert(currentLink >= 0);

  if(tracking) {
    CartesianTrackingObjective* userGoal = new CartesianTrackingObjective(robot);
    userGoal->link = currentLink;
    userGoal->localPosition = currentPoint;
    userGoal->positions.push_back(currentDestination);
    userGoal->times.push_back(0.0);
    userGoal->weights.push_back(1.0);
    Real dt = 0.25;
    Real decay = Exp(-weightDecay*dt/predictionOffset);
    Real w=1.0;
    while(w > 0.01) {
      //integral of weightdecay exp(-weightdecay*t) from 0 to dt is
      //(1-exp(-weightdecay*dt))
      if(w*decay <= 0.01) break;
      userGoal->times.push_back(userGoal->times.back()+dt);
      userGoal->positions.push_back(currentDestination+(1.0-Exp(-userGoal->times.back()*speedDecay))/speedDecay*sumVelocity);
      userGoal->weights.push_back(w*(1-decay));
      w*=decay;
    }
    //weightdecay integral[0 to inf] w*exp(-weightdecay*t) = w
    userGoal->endPosWeight = w;
    lastObjective = userGoal;
    return userGoal;
  }
  else {
    IKGoal goal;
    goal.link = currentLink;
    goal.localPosition = currentPoint;
    goal.SetFixedPosition(currentDestination);
    CartesianObjective* userGoal = new CartesianObjective(robot);
    userGoal->ikGoal = goal;
    //predict where goal will be at the split time
    userGoal->ikGoal.endPosition += (currentTime+predictionOffset-currentInputTime)*sumVelocity;
    lastObjective = userGoal;
    return userGoal;
  }
}

void PredictiveExtrapolationInputProcessor::DrawGL() {
}





SerializedObjectiveProcessor::SerializedObjectiveProcessor(AsyncReaderThread* _reader)
  :reader(_reader)
{
}

void SerializedObjectiveProcessor::Activate(bool enabled)
{
  if(reader) {
    if(enabled) {
      bool res=reader->Start();
      if(!res) {
	fprintf(stderr,"SerializedObjectiveProcessor: Reader thread could not start\n");
	fprintf(stderr,"Waiting and retrying at 1s intervals...\n");
	while(!res) {
	  ThreadSleep(1);
	  res = reader->Start();
	  fprintf(stderr,"...\n");
	}
	fprintf(stderr,"Done!\n");
      }
    }
    else {
      reader->Stop();
      //wait a second for socket to close on server
      ThreadSleep(0.5);
    }
  }
}

 
bool SerializedObjectiveProcessor::HasUpdate()
{
  return reader!=NULL && reader->UnreadCount() > 0;
}

PlannerObjectiveBase* SerializedObjectiveProcessor::MakeObjective(RobotModel* robot)
{
  if(!reader) return NULL;
  string payload = reader->Newest();
  //cout<<"SerializedObjectiveProcessor: Got a message: "<<payload<<endl;
  if(payload.length()==0) return NULL;
  stringstream ss(payload);
  return LoadPlannerObjective(ss,robot);
}


