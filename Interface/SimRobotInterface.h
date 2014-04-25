#ifndef SIM_ROBOT_INTERFACE_H
#define SIM_ROBOT_INTERFACE_H

#include "RobotInterface.h"
#include "Simulation/WorldSimulation.h"

/** @brief A simulation mode RobotInterface.
 * Currently only supports the SimViewProgram default controller.
 */
class SimRobotInterface : public DefaultMotionQueueInterface
{
 public:
  SimRobotInterface(WorldSimulation* sim,int index=0);
};

#endif
