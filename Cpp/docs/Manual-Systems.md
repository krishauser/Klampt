
# Klamp't Manual: System Integration

* [Connecting an external controller to a Klamp't simulated robot](#connecting-an-external-controller-to-a-klamp-t-simulated-robot)
    + [Direct instantiation](#direct-instantiation)
    + [Socket communication with controller](#socket-communication-with-controller)
    + [ROS communication with controller](#ros-communication-with-controller)
    + [Controller communication protocol (CCP)](#controller-communication-protocol--ccp-)
* [Connecting a Klamp't controller to a physical robot](#connecting-a-klamp-t-controller-to-a-physical-robot)
* [Connecting a Klamp't Planner to a Simulation Controller](#connecting-a-klamp-t-planner-to-a-simulation-controller)


Klamp't supports a number of mechanisms for connecting controllers and simulated robots with external modules. An external controller can be connected to a Klamp't simulated robot, or a Klamp't controller can be connected to a physical robot.

## Connecting an external controller to a Klamp't simulated robot

There are two options for doing so (plus a third variant):

1. **Direct instantiation**. You must define a subclass of `RobotController` and perform all necessary processing in its `Update` method. (To integrate with `SimTest` you must edit and recompile Klamp't).
2. **Socket communication with controller**. This interface allows you to communicate directly with `SimTest`. To do so, a robot is given a `SerialController` controller, which acts as a relay to an external client by sending sensor data and receiving motor commands over a socket. Data is serialized in JSON format.
3. **(another instance of method 2) ROS joint\_trajectory and joint\_state messages.** These are supported in `SimTest` via a `SerialController` and the `rosserialrelay.py` script.

More details for each of the methods are given below.

### Direct instantiation 
Once you have created your new controller, a new controller object of your class should be sent to the WorldSimulation.SetController() method when launching your own simulation. Or, the controller can be registered using `RobotControllerFactory::Register`, and its type can be specified in the world XML file.


### Socket communication with controller
This procedure consists of first setting a robot to use a SerialController controller, and writing a binding for your external controller to connect to the server socket, and process messages using the controller communication protocol (CCP).

As an example, consider an external Python controller.

1. Run `./SimTest data/tx90serialinput.xml` in one window. The `SerialController` controller in `SimTest` will listen for clients to connect to `localhost:3456` (the port is specified in the world XML file).  Once a client connects, it will write _sensor messages_ to the socket at a fixed rate and then receive _command messages_ from the socket as they are generated.
2. Run `python Python/control/serialcontroller.py data/motions/tx90sway.txt` in another window. This script connects as a client and begins receiving _sensor messages_ over the socket, processes them (in this case using a trajectory controller), and sends the resulting _command messages_ back over the socket.

### ROS communication with controller
The [rosserialrelay.py](../../Python/control/rosserialrelay.py) script runs a daemon to relay ROS messages to a `SerialController`. It reads position, velocity, and/or feedforward torque commands from the `/[robot_name]/joint_trajectory` ROS topic and writes sensed joint states to the `/[robot_name]/joint_states` ROS topic. It directly translates these items to a `SerialController` on localhost:3456 by default.  As usual, you may start up the `SerialController` through SimTest's Controller window, or by specifying a SerialController as a robot's controller via the world XML file.

A more direct method for use in the simtest.py controller interface is provided by the [roscontroller.py](../../Python/control/roscontroller.py) script. It functions nearly identically to [rosserialrelay.py](../../Python/control/rosserialrelay.py), but without the need to communicate over a socket or to edit XML files to set up the `SerialController` instance.

Note that in both cases, you must build the klampt ROS package (a Catkin workspace has already been provided for you in the [Python/control/klampt_catkin](../../Python/control/klampt_catkin) folder), and use rosrun to start the scripts. Please refer to the ROS documentation for details.

### Controller communication protocol (CCP)

A _sensor message_ is a structure with the following elements:

- `t`: the current simulation time.
- `dt`: the controller time step.
- `q`: the robot's current sensed configuration
- `dq`: the robot's current sensed velocity
- `qcmd`: the robot's current commanded configuration
- `dqcmd`: the robot's current commanded configuration
- The names of each sensors in the simulated robot controller, mapped to a list of its measurements.

A _command message_ is a structure which contains one of the following combinations of keys, signifying which type of joint control should be used:

- `qcmd`: use PI control.
- `qcmd` and `dqcmd`: use PID control.
- `qcmd`, `dqcmd`, and `torquecmd`: use PID control with feedforward torques.
- `dqcmd` and `tcmd`: perform velocity control with the given actuator velocities, executed for time `tcmd`.
- `torquecmd`: use torque control.

Each command key (except tcmd) must be associated with a list of drivervalues.  Note that these are driver values rather than configuration values; as a result the controller must be aware of which drivers are present in the .rob file (as well as their ordering).

CCP messages are serialized in JSON format for socket communication with a `SerialController`, or as Python dictionaries as used in `klampt_sim`.


## Connecting a Klamp't controller to a physical robot

To connect a Klamp't controller to a physical robot, a wrapper around the control loop should repeatedly fill in the controller's sensor data from the physical data, and write the actuator commands to the physical motors.

The standard interface is given in the `ControlledRobot` base class in [Klampt/Control/ControlledRobot.h](../Control/ControlledRobot.h). Your subclass should override the `Init`, `ReadSensorData`, and `WriteCommandData` methods to provide whatever code is necessary to communicate with your robot. See [the cartpole.cpp example program](https://github.com/krishauser/Klampt-examples/Cpp/cartpole.cpp) for an example.

## Connecting a Klamp't Planner to a Simulation Controller

A planner can communicate asynchronously with a controller in real-time using several methods. The general technique is to instantiate a planning thread that sends / receives information with the controller whenever planning is completed.

As an example, consider the real-time planning classes in [Planning/RealTimePlanner.h](../Planning/RealTimePlanner.h) and their interfaces in [Interface/UserInterface.h](../Interface/UserInterface.h). The real time planners send trajectory information to the controller via a `MotionQueueInterface`, which just relays information to the PolynomialPathController in the simulation.

[The reason why the interface is used rather than communicating directly with a `PolynomialPathController` is that it is possible to implement a `MotionQueueInterface` to send trajectory data to the robot directly. The real-time planning demos produced by the IML on the physical TX90L robot use a `MotionQueueInterface` that communicates with the real controller over Ethernet via a simple serial API. This approach often saves bandwidth over implementing a `ControlledRobot` subclass.]

