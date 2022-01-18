#ifndef KLAMPT_SIMULATION_SETTINGS_H
#define KLAMPT_SIMULATION_SETTINGS_H

namespace Klampt {

//Set these values to 0 to get all warnings
const static double gTorqueLimitWarningThreshold = Inf;
const static double gJointLimitWarningThreshold = Inf;
//const static double gTorqueLimitWarningThreshold = 0;
//const static double gJointLimitWarningThreshold = 0;

//turn this to 0 to allow joints to go through their stops
#define USE_JOINT_STOPS 1

//Change the default padding settings.
//More settings can be found in the ODESimulatorSettings constructor in
//ODESimulator.cpp
const static double gDefaultRobotPadding = 0.0025;
const static double gDefaultRigidObjectPadding = 0.0025;
const static double gDefaultEnvPadding = 0.0;

//Change the default collision testing settings.
const static bool gBoundaryLayerCollisionsEnabled = true;
const static bool gRigidObjectCollisionsEnabled = true;
const static bool gRobotSelfCollisionsEnabled = false;
const static bool gRobotRobotCollisionsEnabled = true;
const static bool gAdaptiveTimeStepping = true;

} //namespace Klampt

#endif
