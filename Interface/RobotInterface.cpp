#include "RobotInterface.h"

DefaultMotionQueueInterface::DefaultMotionQueueInterface(PolynomialPathController* _controller)
  :controller(_controller)
{}

bool DefaultMotionQueueInterface::HadExternalChange()
{
  return false;
}

  //interfaces to simulated or actual world
Real DefaultMotionQueueInterface::GetCurTime()
{
  return controller->time;
}

void DefaultMotionQueueInterface::GetCurConfig(Config& x)
{
  controller->GetCommandedConfig(x);
}

void DefaultMotionQueueInterface::GetCurVelocity(Config& dx)
{
  controller->GetCommandedVelocity(dx);
}

Real DefaultMotionQueueInterface::GetEndTime()
{
  return controller->time+controller->TimeRemaining();
}

void DefaultMotionQueueInterface::GetEndConfig(Config& x)
{
  x=controller->Endpoint();
}

void DefaultMotionQueueInterface::GetEndVelocity(Config& dx)
{
  dx=controller->EndpointVelocity();
}

void DefaultMotionQueueInterface::GetConfig(Real t,Config& x)
{
  controller->Eval(t - controller->time,x);
}

MotionQueueInterface::MotionResult DefaultMotionQueueInterface::SendMilestone(const Config& x)
{
  controller->AppendRamp(x);
  return Success;
}

MotionQueueInterface::MotionResult DefaultMotionQueueInterface::SendMilestoneImmediate(const Config& x)
{
  controller->Cut(0);
  controller->AppendRamp(x);
  return Success;
}

MotionQueueInterface::MotionResult DefaultMotionQueueInterface::SendPathImmediate(Real tbreak,const ParabolicRamp::DynamicPath& path)
{
  if(tbreak > controller->time) {
    controller->Cut(tbreak - controller->time);
    controller->Append(path);
  }
  else {
    fprintf(stderr,"Warning, sending immediate path with time < current sim time\n");
    return InvalidParams;
    controller->SetPath(path);
  }
  return Success;
}
