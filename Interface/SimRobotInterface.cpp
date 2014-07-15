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
  :DefaultMotionQueueInterface(GetController(_sim->robotControllers[_index]))
{
  //HACK: the command and sensors pointer in controller are not the same
  //as the top level interface, meaning that GetConfig messes up
  controller->command = _sim->robotControllers[_index]->command;
  controller->sensors = _sim->robotControllers[_index]->sensors;
}
