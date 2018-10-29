# Programming notes and tips 

## Common "gotchas"

- A robot model's configuration is used in many places as input and output!  To guard against losing configuration, *think of the robot's configuration as a temporary variable.* If you need to keep the configuration between computations, just store it before the computations, and restore it afterwards.  The standard pattern (in Python) is:

    ```python
    q = robot.getConfig()
    ... do stuff ...
    robot.setConfig(q)
    ```

- The current configuration of a robot model does not have an effect on a real or simulated robot.  To drive the actual robot, you must use the interface in its Robot Controller.  To work with the current configuration of a real or simulated robot, the configuration must be read from the Robot Controller.

- A robot model does not necessarily correspond to a real robot.  Its kinematics, geometry, and limits need to be calibrated from the specifications of the real robot.

- When performing multithreading, such as in the Python vis module, crashes can occur if locking is not performed properly to prevent simultaneous access to World Models.

- Data can inadvertently "leak" from simulation to planning if a shared World Model is used, since the simulation will update the model for visualization.  An easy way to get around such conflicts is simply to copy the world into separate planning and simulation worlds.  (This is fast, since none of the geometry is actually copied.)  The standard pattern (in Python) is:

    ```python
    world = WorldModel()
    ... set up the model ...
    simWorld = world.copy()
    sim = Simulator(simWorld)
    planWorld = world   #no need to copy again
    planner = MyPlanner(planWorld)
    ```


## General

- Ask questions and report issues/bugs. This will help us make improvements to Klamp't. If you write a piece of code that you think will be useful to others, consider making it a contribution to the library.
- Practice _self-documenting code_. Name files, functions, classes, and variables descriptively. Comment as you go.
- Use _visual debugging_ to debug your algorithms. If you are using C++, output intermediate configurations or paths to disk and inspect them with the `RobotPose` program.  If you are using Python, call `klampt.io.resource.edit()`.
- _Think statefully_. Decompose your programs into algorithms, state, parameters, and data. State is what the algorithm changes during its running. Parameters are values that are given as input to the algorithm when it begins (arguments and settings), and they do not change during execution. Data is the static knowledge available to the algorithms and the information logged as a side effect of its execution.
- When prototyping long action sequences, build in functionality to save and restore the state of your system at intermediate points.


## C++ Programming

Klamp't is written in C++, and using C++ will give you full access to its functionality and maximum performance. But, it does require comfort with large code bases and moderate-to-advanced C++ programming abilities.

Here are some conventions and suggestions for programming C++ apps that use Klamp't.

- Use a debugger (e.g., GDB) to debug crashes.
- Use STL and smart pointers ([KrisLibrary/utils/SmartPointer.h](http://github.com/krishauser/KrisLibrary/utils/SmartPointer.h)) rather than managing memory yourself.
- `KrisLibrary` contains a lot of functionality, including linear algebra routines, 3D math, optimization, geometric routines, OpenGL drawing, statistics, and graph structures. Browse [KrisLibrary](http://github.com/krishauser/KrisLibrary) before you reinvent the wheel.
- Avoid hard-coding. A much better practice is to place all settings into a class (e.g., with a `robotLeftHandXOffsetAmount` member) that gets initialized to a default value in the class' constructor. If you need to hard-code values, define them as const static variables or `#define`s at the top of your file. Name them descriptively, e.g., `gRobotLeftHandXOffsetAmount` is much better than `shift` or (God forbid) `thatStupidVariable`, when you come back to the file a month from now.
- The `main()` function in [Klampt/Main/simtest.cpp](../Main/simtest.cpp) is a good reference for setting up a world and a simulation from command-line arguments.

## Python Programming

The Klamp't Python API in [Klampt/Python](../Python) is much cleaner and easier to work with than the C++ API. For beginners or for rapid prototyping, this is the best API to use. However, it does not contain all of the functionality of the C++ API.

Missing features include:

- Advanced IK constraint types
- Trajectory optimization
- Some contact processing algorithms
- Robot reachability bound determination
- Advanced force/torque balance solvers
- Advanced motion planners (optimal planning with custom objective functions, kinodynamic planning, etc)
- Direct access to a robot's trajectory queue.

### Getting started
The core modeling and simulation Klamp't functionality is found in the `klampt` module, which automatically imports several classes that wrap C++ functionality via SWIG. Users will typically load a `WorldModel`, construct a `Simulator`, and implement a robot controller by interacting with the `SimRobotController`. They may also wish to use the `RobotModel` to compute forward kinematics and dynamics.

It should be noted that the documentation of these basic classes are found under the [klampt.robotsim](http://klampt.org/pyklampt_docs/namespaceklampt_1_1robotsim.html) submodule. Their online documentation may also look somewhat strange for Python users, having been converted from C++ comments via SWIG.  

Other native Python modules exist for a whole host of other functions, such as computing IK solutions via the `klampt.model.ik` module, or do other kinds of planning tasks via the [klampt.plan.robotplanning](http://motion.pratt.duke.edu/klampt/pyklampt_docs/namespaceklampt_1_1plan_1_1robotplanning.html) module.

### Useful Sub-Modules

Sub-modules of the Klamp't Python API not discussed elsewhere are as follows:

- [cartesian_trajectory](http://motion.pratt.duke.edu/klampt/pyklampt_docs/namespaceklampt_1_1model_1_1cartesian__trajectory.html): reliable Cartesian interpolation functions between arbitrary task space points. Also defines a convenient &quot;bump&quot; function that modify joint-space paths to achieve a task-space displacement.
- [collide](http://motion.pratt.duke.edu/klampt/pyklampt_docs/namespaceklampt_1_1model_1_1collide.html): defines a WorldCollider class that enables querying the collision status of the world and subsets of bodies in the world.
- config: a uniform interface for determining a flattened list of floats describing the configuration of a world entity, a mathematical object, or an IK goal.
- contact: allows querying contact maps from a simulator and computing wrench matrices, and equilibrium testing.
- coordinates: a coordinate transform manager, similar to the tf module in ROS, that lets you attach points / vectors to frames and determine relative or world coordinates.
- hold.py: defines a Hold class and writes / reads holds to / from disk.
- ik: convenience routines for setting up and solving IK constraints. _We do not yet allow solving across multiple robots and objects but this functionality may be supported in the future._
- map: convenient object-oriented interface for accessing worlds, robots, objects, links, etc. For example, you can write

    ```python
    wm = map.map(world)
    wm.robots[0].links[4].transform
    ```

    instead of

    ```
    world.robot(0).link(4).getTransform().
    ````

    Most notably used in the [sim.batch](../Python/klampt/sim/batch.py) module.

- sensing: functions for processing simulated sensor data.
- subrobot: a class that is `RobotModel`-like but only modifies selected degrees of freedom of the robot (e.g., an arm, a leg). Many `klampt` module functions accept `SubRobotModel`s in the place of `RobotModel`s.
- types: retrieving the resource manager type string for various Klamp't objects.
- loading: functions for loading/saving Klamp't objects to strings and/or disk in both native format and JSON formats.
- resource: functions for loading/saving/editing Klamp't resources.
- cspaceutils: contains helpers for constructing composite CSpaces and slices of CSpaces.
- batch: functions for batch Monte-Carlo simulation of many simulation initial conditions.
- settle: convenience functions to let objects fall under gravity and extract their equilibrium configurations.
- simlog: simulation logging classes (used in SimpleSimulator)
- simulation: a more full-featured simulation class than standard Simulation. Defines sensor and actuator emulators, sub-step force appliers, etc.

The klampt module does not (yet) contain interfaces to trajectory optimization and state estimation.


### Utilities and Demos

The [Python/utils](../Python/utils) and [Python/demos](../Python/demos) folders contain a few example utilities and programs that can be built upon to start getting a flavor of programming Klamp't applications in Python.

- demos/gltest.py: a simple simulation with force sensor output.
- demos/gltemplate.py: a simulation with GUI hooks and mouse-clicking capabilities.
- demos/kbdrive.py: drive a simulated robot around using the keyboard.  The first 10 joints can be driven via a positive velocity with the top row of keys 1,2,...,0 and a negative velocity with the second row of keys q,w,...,p.
- demos/robotiq.py: modeling and simulating the RobotiQ 3-finger Adaptive Gripper. This code emulates the underactuated transmission mechanism of each finger.
- demos/robotiqtest.py: performs a simulation of the RobotiQ gripper closing and opening on an object.
- demos/simtest.py: an imitation of SimTest program programmed entirely in Python, and an entry point to fast prototyping of controllers using the Python API.
- demos/sphero.py: simulates the Sphero 2.0 robot driving around.
- demos/vistemplate.py: demonstrates how to use the basic interface to the visualization module.
- demos/visplugin.py: demonstrates how to develop plugins for the visualization module.
- utils/config\_to\_driver\_trajectory.py: converts a linear path from configuration space (# of DOF) to driver space (# of actuators).
- utils/driver\_to\_config\_trajectory.py: converts a linear path from driver space (# of actuators) to configuration space (# of DOF).
- utils/discretize\_path.py: splits a linear path into a fixed time-domain discretization.
- utils/make\_thumbnails.py: generates thumbnails of a folder full of world, robot, object files, etc.
- utils/multipath\_to\_path.py: simple script to convert a MultiPath to a timed milestone trajectory. Parameters at the top of the script govern the speed of the trajectory.
- utils/multipath\_to\_timed\_multipath.py: simple script to convert a MultiPath to a timed MultiPath. Parameters at the top of the script govern the speed of the trajectory.
- utils/tri2off.py: converts old-style .tri files to .off files.

Like the compiled SimTest, simtest.py simulates a world file and possibly robot trajectories. The user interface is a simplified SimTest, with 's' beginning simulation and 'm' saving frames to disk. Right-dragging applies spring forces to the robot.

