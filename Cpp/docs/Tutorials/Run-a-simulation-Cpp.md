# Klamp't Tutorial: Run a simulation with C++ API

In this tutorial we learn how to run a simulation, interact with the simulated robot via the poser, and to simulate paths and trajectories. It is assumed that you have already successfully installed Klampt.

Difficulty: easy

Time: 5 - 10 minutes

### C++ Simulation

In this tutorial we'll make a C++ command line program that performs simulation without a GUI.

To make your first app that compiles with Klamp't, first create a folder with two files, main.cpp and CMakeLists.txt.

This very basic main.cpp runs a simulation for 5 seconds (simulation time) from the command line:

```cpp
#include <Klampt/Interface/SimulationGUI.h>

int main(int argc,const char** argv) {
  //create a world
  WorldModel world;

  //The SimGUIBackend class offers many helpers for setting
  //up default simulations, doing logging, sending paths, etc.
  //Advanced users may want to just create a WorldSimulation
  //class for more fine-grained control.
  SimGUIBackend backend(&world);
  WorldSimulation& sim=backend.sim;

  //If you have a fixed world that you want to use, you can
  //load any files like this
  //world.LoadElement("Klampt/data/robots/athlete.rob");
  //world.LoadElement("Klampt/data/terrains/plane.env");

  //Alternatively, this is helpful for loading command
  //line arguments like SimTest
  if(!backend.LoadAndInitSim(argc,argv)) {
    cerr<<"Error loading simulation from command line"<<endl;
    return 1;
  }

  //Uncomment+edit the following line to change the controller
  //time step for robot 0 (100Hz is the default)
  //sim.controlSimulators[0].controlTimeStep = 0.01;

  //Uncomment+edit the following line to change the underlying
  //simulation time step (1kHz is the default)
  //sim.simStep = 0.001;

  //pick some duration between printouts in main loop
  double dt = 0.1;
  //run the simulation
  while(sim.time < 5) {
    //** add control code here **

    //move the sim forward by the given time
    sim.Advance(dt);
    //update the world
    sim.UpdateModel();
    //print time, robot 0's configuration
    cout<<sim.time<<'\t'<<world.robots[0]->q<<endl;

    //Uncomment the following line to log the true state of
    //the robot to disk.
    //backend.DoStateLogging_LinearPath(0,"test_state.path");
  }
  return 0;
}
```
The CMakeLists.txt file for this example would be the following:
```
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(KLAMPT_ROOT "Put the path to the Klampt folder here")
SET (CMAKE_MODULE_PATH "${KLAMPT_ROOT}/CMakeModules")
FIND_PACKAGE(Klampt REQUIRED) 
ADD_DEFINITIONS(${KLAMPT_DEFINITIONS})
INCLUDE_DIRECTORIES(${KLAMPT_INCLUDE_DIRS})
ADD_EXECUTABLE(MyApp main.cpp)
TARGET_LINK_LIBRARIES(MyApp ${KLAMPT_LIBRARIES})
```
Where you specify the Klamp't path in the first line.
Build your file using
```
cmake .  
make
```
And test it using
```
./MyApp [Path to Klampt]/data/athlete_fractal_1.xml
```
If you would like to visualize the path that the robot took, you may save it to a LinearPath by uncommenting the line beginning in backend.DoStateLogging... Recompile, run the simulation, and then run it through RobotPose.
```
[Path to Klampt]/RobotPose [Path to Klampt]/data/athlete_fractal_1.xml test_state.path
```
Clicking on the test_state resource and pressing play, you will see the simulation state of the robot sliding partway down the hill.

Note: an alternate way of visualizing the simulation would be to use a GUI system. The simplest way of doing this would be to use GLUI and duplicate the structure of Klampt/Main/simtest.cpp. To implement user-chosen forces or a custom control loop, you should create a subclass of SimTestGUI and overload the virtual function SimTestGUI::OnCommand(const string& cmd,const string& args). When you get a cmd called "advance", implement your control loop and then call return SimTestGUI::OnCommand("advance",args). You can also see  [Custom controller tutorial](Custom-controller.md) for more details.

### Sending milestones to the controller

By default, a robot is set up with a polynomial trajectory controller, which stores and executes a smooth interpolating path between keyframes. (See the documentation of PolynomialPathController in Klampt/Control/PathController.h for details). The preferred, most extensible way of communicating with a robot's controller is with the SendCommand() function, which defines a string-based interface for accepting commands. For example, we can make the robot move its leg pitch (link 7) up after 2 seconds by adding the following lines at the location marked by  ** add control code here **:
```cpp
if(sim.time >= 2.0 && sim.time-dt < 2.0) {
  //move link 7 (hip pitch) 1 radian up
  Config q;
  sim.robotControllers[0]->GetCommandedConfig(q);
  q[7] -= 1.0;
  //LexicalCast is needed to convert config to string
  sim.robotControllers[0]->SendCommand("set_q",LexicalCast(q));
  //then move link 7 (hip pitch) 1.5 radians down
  q[7] += 1.5;
  sim.robotControllers[0]->SendCommand("append_q",LexicalCast(q));
}
```
Make the program again and run it:
```
make  
rm test_state.path  
./MyApp [Path to Klampt]/data/athlete_fractal_1.xml
```
_Note_: the rm line is needed because the logging function just appends to the file. To get a clean path from another simulation run, you must delete the existing path file.

Observe the path again using RobotPose. Notice that the path produces a smooth interpolation.

### Sending milestones to the controller

Now we will send a trajectory. Replace the previous added lines with the following, which sends a Linear Path piecewise linear trajectory:
```cpp
if(sim.time >= 2.0 && sim.time-dt < 2.0) {
  //move link 6 (hip pitch) 1 radian up
  Config q;
  sim.robotControllers[0]->GetCommandedConfig(q);
  vector<Real> times(3);
  vector<Config> milestones(3);
  times[0] = 0;
  times[1] = 1;
  times[2] = 2;
  milestones[0] = q;
  q[7] -= 1.0;
  milestones[1] = q;
  q[7] += 1.5;
  milestones[2] = q;
  backend.SendLinearPath(times,milestones);
}
```
Here, the commanded configuration will meet the milestones at precisely the desired points in time, but with sharp velocity discontinuities. To see this, re-compile, re-run, and observe the path.

Another convenient way to send a path from disk is by calling backend.LoadLinearPath(fn). As a final exercise, try running the athlete_flex.path file starting at t=2s.

### Playing God: applying forces and constraining velocities

The robot controller is not able to apply arbitrary forces to its body or the world. This encapsulation is deliberate, because a robot cannot "play God" -- it can only affect its body or the world via its actuators. But it is often useful to generate simulation scenarios by "playing God," and to do so, you must access the underlying rigid bodies in the Open Dynamics Engine (ODE) simulator.

The first step in doing so is to access the ODERigidObject or ODERobot out of the WorldSimulator. To do so, you would call something like this:
```cpp
ODERobot* simrobot = sim.odesim.robot(my_robot_index);
//or...
ODERigidObject* simobject = sim.odesim.object(my_object_index);
```
To apply forces, you may use ODE’s API. The way to do this is as follows.
```cpp
dBodyAddForceAtPos(simrobot->body(my_link_index),fx,fy,fz,px,py,pz);
//or...
dBodyAddForceAtPos(simobject->body(),fx,fy,fz,px,py,pz);
```
Where the force (fx,fy,fz) and point (px,py,pz) are in world coordinates. For more information, consult the  [ODE documentation](http://ode-wiki.org/wiki/index.php?title=Manual).

Directly controlling the movement of a body (e.g., to move along a predetermined path, or according to a joystick) is possible but takes a few extra steps, because Klamp't by default gives control of the body to the simulator. First, you will need to know the translational and angular velocity along which the body should be moving at each time step. Let us assume you have determined these quantities as (vx,vy,vz) and (wx,wy,wz); both are in world coordinates. Then, you will need to tell ODE to disable dynamic simulation, and during your time step you will need to set the velocities directly as follows:
```
dBodyID bodyID = simrobot->body(my_link);
//or
dBodyID bodyID = simobject->body();
dBodySetKinematic(bodyID);  // this only needs to be set once at the beginning of simulation
dBodySetLinearVel(bodyID,vx,vy,vz);
dBodySetAngularVel(bodyID,wx,wy,wz);
```

### Extracting contact forces

For research and evaluation purposes it is often useful to record the contact forces generated by the simulation, and Klamp't provides several functions for doing so. This tutorial also illuminates some of the differences between the indexing and body ID schemes.

The first step in extracting contact feedback is to enable it. Contact feedback is enabled on a per body pair basis; this is primarily done to save a little overhead in computation and memory. Each rigid body in the world, including environment objects and robot links, is given a unique ID, and this ID is used to identify the corresponding body in the simulator. To get the ID of an object in the world, you would call:
```
int terrainid = world.TerrainID(terrain_index);
int objectid = world.RigidObjectID(object_index);
int linkid = world.RobotLinkID(robot_index,int link_index);
```
IDs are constant throughout the life of the simulation. (NOTE: IDs will change if you add or remove elements from the world -- this is not yet supported in simulation.) IDs are assigned contiguously, and hence it is possible to just loop through integers ranging from 0 to world.NumIDs()-1 to enable all contact pairs. So we will do this before our simulation loop to allow us to observe all contact pairs:
```cpp
for (int i=0;i<world.NumIDs();i++)
  for (int j=i+1;j<world.NumIDs();j++)
    sim.EnableContactFeedback(i,j);
```
Don't worry, the computational overhead is not that high. If we really wanted to be selective, we could just do something like this to enable only collision feedback between the terrain and all links on the robot:
```
for (size_t i=0;i<world.robots[0].robot->links.size();i++)
  sim.EnableContactFeedback(terrainid,world.RobotLinkID(robot_index,i));
```
But let's assume we just enabled all of them. Then, during the simulation loop, we can use the following code to see what objects are in contact. We can also print out the average contact force/torque:
```cpp
bool contacted=false;
for (int i=0;i<world.NumIDs();i++)
  for (int j=i+1;j<world.NumIDs();j++) {
    //you could loop over a selective set of id pairs rather than i and j, if you wanted...
    if(sim.HadContact(i,j)) {
      if(!contacted) { printf("Touching bodies:\n"); contacted=true; }
      Vector3 f = sim.MeanContactForce(i,j);
      Vector3 t = sim.MeanContactTorque(i,j);
      printf("  %s - %s: force %g %g %g, torque %g %g %g\n",world.GetName(i).c_str(),world.GetName(j)).c_str(),f.x,f.y,f.z,t.x,t.y,t.z);
    }
  }
```
If you don't want to get mean contact force/torque, there are other methods of WorldSimulator you can use to retrieve other types of feedback. For example, you can get detailed contact point information using the  [ContactFeedbackInfo](http://motion.cs.illinois.edu/klampt/klampt_docs/structContactFeedbackInfo.html)  structure:
```cpp
ContactFeedbackInfo* info = sim.GetContactFeedback(i,j); 
```
Please consult the [detailed API documentation for the Simulation module](http://motion.cs.illinois.edu/klampt/klampt_docs/group__Simulation.html) for more information
