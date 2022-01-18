# Klamp't Tutorial: Implement a custom controller for a simulated robot in C++

In this tutorial we learn how to connect a user-defined controller to a Klamp't simulation. Simulated Klampt robots supports low-level PID commands or torques sent to the robot's motors, as well as point-to-point motions and joint trajectories.

Difficulty: intermediate

Time: 10-30 minutes

The C++ API provides a controller interface that is the most performance-oriented. The controller will be compiled code, it avoids translation or repackaging of messages, and it provides direct access to Klamp't planning routines and sensors. The only downside is that Klamp't apps must be recompiled to use the controller (like in SimTest), but you can also make your own simulation app that accepts your controller.

What we'll do is create a new directory with three files, MyController.h, main.cpp, and CMakeLists.txt. Let's start with MyController.h. We'll add a new subclass of RobotController (found in Klampt/Control/Controller.h) and override, at a minimum, the Type method, which provides a name for the controller, and the Update method, which reads from the sensors member and writes to the command member.

Your MyController.h will look something like this:
```
#include <Klampt/Controller.h>
using namespace Klampt;

class MyController : public RobotController
{
public:
  MyController(Robot& robot) : RobotController(robot) {}
  virtual ~MyController() {}
  virtual const char* Type() const { return "MyController"; }
  virtual void Reset() { 
    //put any initialization code here
    RobotController::Reset(); 
  } 
  virtual void Update(Real dt) {
    //We'll put our code here: read from this->sensors, and write to this->command.
    //See Sensor.h and Command.h for details on these structures
    printf("Whee, the control loop is being called!\n");
    //call the base class's Update method
    RobotController::Update(dt);
  }
};
```
Your controller code should go into the indicated line in Update. For example, we'll implement a controller that moves a link along a fixed velocity for 1 second:
```
Vector qcmd,vcmd;
Vector qactual,vactual;
GetCommandedConfig(qcmd);  //convenience function in RobotController
GetCommandedVelocity(vcmd);  //convenience function in RobotController
GetSensedConfig(qactual);  //convenience function in RobotController
GetSensedVelocity(vactual);  //convenience function in RobotController
int link = 7;  //the movement link
if(time >= 1.0 && time < 2.0) {
  Real speed = -1.5;
  vcmd[link] = speed;
}
else
  vcmd[link] = 0;
SetPIDCommand(qcmd,vcmd); //convenience function in RobotController
```
Note that, internally, the GetSensedX and SetPIDCommand convenience functions read from and write to the sensors and command structures, respectively. If you wish to use more complex sensors, e.g. force sensors, gyroscopes, etc., then you will need to read them out of the RobotController's sensors' member. See the documentation of the [Robot Sensors](http://motion.cs.illinois.edu/klampt/klampt_docs/classRobotSensors.html) class and the Klampt/Control/Sensors.h file for more information.

Let's now switch to main.cpp. We could run a command-line simulation like in the [Klamp't simulation tutorial](Run-a-simulation-Cpp.md), but let's make things more interesting by making a GUI. The plan is to implement a new version of the SimTest program that will allow us to use this controller. For simplicity we'll copy our main.cpp file from the GLUI version of SimTest, given in Main/simtest.cpp:
```
#include <Klampt/Interface/SimTestGUI.h>
using namespace Klampt

int main(int argc,const char** argv)
{
  WorldModel world;
  SimTestBackend backend(&world);
  if(!backend.LoadAndInitSim(argc,argv)) {
    return 1;
  }
  GLUISimTestGUI gui(&backend,&world);
  gui.SetWindowTitle("SimTest");
  gui.Run();
  return 0;
}
```
(If you really cared about getting all the bells and whistles of Qt, you will need to copy the Main/SimTestQt folder into your own folder and make the subsequent modifications to the main.cpp file.)

Next, we need to make Klamp't aware of the new controller type. Klamp't uses a  _controller factory_  that is used to create a controller when its type is specified in the world simulation settings, and you need to register your controller with the factory. This is done as follows. First include your controller's header file:
```
#include "MyController.h"
```
Then, add the following line before gui.Run():
```
RobotControllerFactory::RegisterDefault(new MyController(*(world.robots[0])));
```
Now we are ready to compile your new simulation program. In CMakeLists.txt, you will
```
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(KLAMPT_ROOT [Put the path to the Klampt folder here])
SET (CMAKE_MODULE_PATH "${KLAMPT_ROOT}/CMakeModules")
FIND_PACKAGE(Klampt REQUIRED) 
ADD_DEFINITIONS(${KLAMPT_DEFINITIONS})
INCLUDE_DIRECTORIES(${KLAMPT_INCLUDE_DIRS})
ADD_EXECUTABLE(MySim main.cpp)
TARGET_LINK_LIBRARIES(MySim ${KLAMPT_LIBRARIES})
```
If all goes well you should see a new executable named MySim.

We're not done yet. We need to instruct Klamp't to use this controller for a simulation. Let's try the Hubo robot. First copy the world XML file Klampt/data/athlete_plane.xml to a new document, say Klampt/data/athlete_mycontroller.xml. Open it in a text editor.

Under the XML <world> tag , add the lines:
```
<simulation>
  <robot index="0"> 
    <controller type="MyController" />.
  </robot>  
</simulation>
```
Now save and close the file. Let's run the simulation
```
./MySim data/athlete_mycontroller.xml
```
and see the controller in action! The robot should lift its leg.

_Remarks:_  If you want to include your controller as a default for all Klamp't applications to use, you may do the following. Open up Klampt/Control/Controller.cpp. Go to the definition of RobotControllerFactory::RegisterDefault method, add a #include of your controller's header file, and add the line
```
Register(new MyController(robot))
```
Next, navigate to the main Klampt directory and recompile Klamp't and SimTest via the command:
```
make SimTest
```
This will also build the Klamp't library and will make your controller available for use in any programs that use Klamp't.

![Example of C++ controller in simulation](custom_controller.jpg)

This is only the beginning; there are an infinite number of ways to implement a controller, ranging from simple to highly complex behaviors. For example, for point-to-point motions or motions through multiple keyframes, you may subclass the PolynomialPathController class and use its motion queuing functionality. You are only limited by your imagination!
