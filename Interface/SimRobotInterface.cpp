#include "SimRobotInterface.h"
#include "Control/FeedforwardController.h"
#include "Control/LoggingController.h"
#include "Control/PathController.h"

typedef PolynomialPathController MyController;
inline MyController* GetController(RobotController* rc)
{
  LoggingController* lc = dynamic_cast<LoggingController*>(rc);
  if(!lc) {
    FatalError("Robot controller isn't a LoggingController");
  }
  FeedforwardController* fc = dynamic_cast<FeedforwardController*>(&*lc->base);
  if(!fc) {
    FatalError("LoggingController base is not a feedforward controller");
  }
  MyController* c = dynamic_cast<MyController*>(&*fc->base);
  if(!c) {
    FatalError("Feedforward base is not a PolynomialPathController");
  }
  return c;
}


SimRobotInterface::SimRobotInterface(WorldSimulation* _sim,int _index)
  :sim(_sim),index(_index)
{}

  //interfaces to simulated or actual world
Real SimRobotInterface::GetCurTime()
{
  return sim->time;
}

void SimRobotInterface::GetCurConfig(Config& x)
{
  sim->controlSimulators[index].GetCommandedConfig(x);
}

void SimRobotInterface::GetCurVelocity(Config& dx)
{
  sim->controlSimulators[index].GetCommandedVelocity(dx);
}

Real SimRobotInterface::GetEndTime()
{
  return sim->time+GetController(sim->robotControllers[index])->TimeRemaining();
}

void SimRobotInterface::GetEndConfig(Config& x)
{
  x=GetController(sim->robotControllers[index])->Endpoint();
}

void SimRobotInterface::GetEndVelocity(Config& dx)
{
  dx=GetController(sim->robotControllers[index])->EndpointVelocity();
}

MotionQueueInterface::MotionResult SimRobotInterface::SendMilestone(const Config& x)
{
  GetController(sim->robotControllers[index])->AppendRamp(x);
  return Success;
}

MotionQueueInterface::MotionResult SimRobotInterface::SendMilestoneImmediate(const Config& x)
{
  GetController(sim->robotControllers[index])->Cut(0);
  GetController(sim->robotControllers[index])->AppendRamp(x);
  return Success;
}

MotionQueueInterface::MotionResult SimRobotInterface::SendPathImmediate(Real tbreak,const ParabolicRamp::DynamicPath& path)
{
  MyController* c=GetController(sim->robotControllers[index]);
  if(tbreak > sim->time) {
    c->Cut(tbreak);
    c->Append(path);
  }
  else 
    c->SetPath(path);
  return Success;
}
