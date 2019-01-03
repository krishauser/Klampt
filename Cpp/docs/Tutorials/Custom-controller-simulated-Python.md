

# Klamp't Tutorial: Implement a custom controller for a simulated robot in Python

In this tutorial we learn how to connect a user-defined controller to a Klamp't simulation. [C++](Documentation/Tutorials/Custom-controller-simulated-cpp.md), and Python controller support low-level PID commands or torques sent to the robot's motors. The C++ and Python APIs also support point-to-point motions and joint trajectories.

Difficulty: intermediate

Time: 10-30 minutes

The Python interface gives you a lot of flexibility for controlling robots. The easiest way to understand its interface is to work through the exercise in Klampt/Python/exercises/control. This tutorial provides some more information about the several control methods available in Python.

### Manual control interface
The standard Klamp't simulation loop is given as follows:
```
import klampt
world = klampt.WorldModel()
world.readFile("my_world_file.xml")
sim = klampt.Simulator(world)
controller = sim.getController(0)
while sim.getTime() < 10:
    #TODO put your control code here
    sim.simulate(0.01)
print "End configuration:",controller.getSensedConfig()
```
In the indicated TODO line, you should call methods of the controller object, which is of the SimRobotController type.

**Sensing**: To get the encoder configuration and velocity, you may call controller.getSensedConfig() and controller.getSensedVelocity(). To get other sensors, you may call controller.getSensor(index).getMeasurements() or controller.getNamedSensor(name).getMeasurements().

**Low-level control**: To send a PID command to the robot, you may call controller.setPIDCommand(qdes,dqdes). To send raw torques, you may call controller.setTorques(t).

**Trajectory queuing**: To send the robot smoothly to a desired configuration, you may call controller.setMilestone(qdes). To queue up several moves, you can then call controller.addMilestone(qdes2), controller.addMilestone(qdes3), etc. See also the Simulate the execution of a keyframe path ([C++](Documentation/Tutorials/Run-a-simulation-Cpp.md),[Python](Documentation/Tutorials/Run-a-simulation-Python.md)) for a concrete example.

_Tip_: You may also observe your controller behavior in real-time by copying the Klampt/Python/demos/gltemplate.py program, and then inserting your control code into the control_loop() method.

### Standardized Python controller interface

The Python version of SimTest, Klampt/Python/demos/simtest.py, accepts arbitrary controllers given as Python scripts on the command line. These scripts must contain a make(robot) function which return an object that implements the interface in BaseController (Klampt/Python/control/controller.py).

Most importantly, your controller should implement the following:

-   output(self,**inputs): given a set of named inputs, produce a dictionary of named outputs. The semantics of the inputs and outputs are defined below.
-   advance(self,**inputs): advance by a single time step, performing any necessary changes to the controller's state.  _Note: output should NOT change internal state!_

The inputs argument to both functions is a dictionary with the following elements:

-   t: the current simulation time
-   dt: the controller time step
-   q: the robot's current sensed configuration
-   dq: the robot's current sensed velocity
-   qcmd: the robot's current commanded configuration
-   dqcmd: the robot's current commanded configuration
-   The names of each sensors in the simulated robot controller, mapped to a list of its measurements.

The return value of output should be a dictionary that contains one of the following combinations of keys, signifying which type of joint control should be used:

-   qcmd: use PI control.
-   qcmd and dqcmd: use PID control.
-   qcmd, dqcmd, and torquecmd: use PID control with feedforward torques.
-   dqcmd and tcmd: perform velocity control with the given actuator velocities, executed for time tcmd.
-   torquecmd: use torque control.

Several existing controllers have been implemented in Python/Klampt/control/controller.py to make the design and composition of controllers a bit easier, e.g., finite state machines, switching controllers, linear controllers, etc.

Examine Python/Klampt/control/trajectory_controller.py to see how a standard trajectory controller is implemented.
