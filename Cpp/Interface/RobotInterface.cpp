#include "RobotInterface.h"
using namespace Klampt;

MotionQueueInterface::MotionQueueInterface()
{}

DefaultMotionQueueInterface::DefaultMotionQueueInterface(PolynomialMotionQueue* _queue)
  :queue(_queue)
{}

bool DefaultMotionQueueInterface::HadExternalChange()
{
  return false;
}

  //interfaces to simulated or actual world
Real DefaultMotionQueueInterface::GetCurTime()
{
  return queue->CurTime();
}

void DefaultMotionQueueInterface::GetCurConfig(Config& x)
{
  x = queue->CurConfig();
}

void DefaultMotionQueueInterface::GetCurVelocity(Config& dx)
{
  dx = queue->CurVelocity();
}

Real DefaultMotionQueueInterface::GetEndTime()
{
  return queue->CurTime()+queue->TimeRemaining();
}

void DefaultMotionQueueInterface::GetEndConfig(Config& x)
{
  x=queue->Endpoint();
}

void DefaultMotionQueueInterface::GetEndVelocity(Config& dx)
{
  dx=queue->EndpointVelocity();
}

void DefaultMotionQueueInterface::GetConfig(Real t,Config& x)
{
  queue->Eval(t,x,false);
}

MotionQueueInterface::MotionResult DefaultMotionQueueInterface::SendMilestone(const Config& x)
{
  queue->AppendRamp(x);
  return Success;
}

MotionQueueInterface::MotionResult DefaultMotionQueueInterface::SendMilestoneImmediate(const Config& x)
{
  queue->Cut(0);
  queue->AppendRamp(x);
  return Success;
}

MotionQueueInterface::MotionResult DefaultMotionQueueInterface::SendPathImmediate(Real tbreak,const ParabolicRamp::DynamicPath& path)
{
  if(tbreak > queue->CurTime()) {
    queue->Cut(tbreak - queue->CurTime());
    queue->Append(path);
  }
  else {
    fprintf(stderr,"Warning, sending immediate path with time < current sim time\n");
    return InvalidParams;
    queue->SetPath(path);
  }
  return Success;
}
