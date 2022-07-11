#ifndef SIM_VIEW_PROGRAM
#define SIM_VIEW_PROGRAM

#include "WorldViewProgram.h"
#include <Klampt/Simulation/Simulator.h>
#include <Klampt/Control/PathController.h>

#if HAVE_GLUT || HAVE_GLUI

namespace Klampt {

//For a simulation set up with SimViewProgram, this retrieves the PolynomialMotionQueue for the controller
PolynomialMotionQueue* GetMotionQueue(RobotController* rc);

/** @brief A simple GLUT/GLUI program for drawing and interacting with a simulation.
 *
 * You will want to override the Handle_X functions of GLUTNavigationProgram/
 * GLUINavigationProgram.
 *
 * Then call Run() at the end of your main() function.
 */
class SimViewProgram : public WorldViewProgram
{
public:
  SimViewProgram(WorldModel* world);

  ///Loads from a world XML file
  bool LoadAndInitSim(const char* xmlFile);

  ///Loads from a command line
  bool LoadAndInitSim(int argc,const char** argv);

  ///Initializes simulation default controllers, sensors, and contact feedback
  void InitSim();

  ///Returns the simulation to its initial state
  void ResetSim();

  ///Renders the state of the simulation
  virtual void RenderWorld();

  ///Draws contact points
  void DrawContacts(Real pointSize = 5.0, Real fscale = 0.01, Real nscale=0.05);
  ///Draws wrenches
  void DrawWrenches(Real fscale=-1);

  ///Loads and sends a milestone path file
  bool LoadMilestones(const char* fn);

  ///Loads and sends a linear path file
  bool LoadLinearPath(const char* fn);

  ///Loads a simulation state file
  bool LoadState(const char* fn);

  ///Loads a multipath file and possibly discretizes it into a fine-grained linear path before sending
  bool LoadMultiPath(const char* fn,bool constrainedInterpolate=true,Real interpolateTolerance=1e-2,Real durationScale=1.0);

  ///Sends a linear path to the controller.  The path starts pathDelay
  ///seconds after the current time
  bool SendLinearPath(const vector<Real>& times,const vector<Config>& milestones,Real pathDelay=0.5); 

  ///Logs the state of all objects in the world to the given CSV file
  void DoLogging(const char* fn="simtest_log.csv");

  int simulate;
  Simulator sim;
  string initialState;
};

} // namespace Klampt

#endif //HAVE_GLUT || HAVE_GLUI

#endif