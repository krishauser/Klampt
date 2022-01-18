# Programming notes and tips (C++)

## Common "gotchas"

- A robot model's configuration is used in many places as input and output!  To guard against losing configuration, *think of the robot's configuration as a temporary variable.* If you need to keep the configuration between computations, just store it before the computations, and restore it afterwards.  The standard pattern (in C++) is:

    ```cpp
    Config q = robot.q;
    ... do stuff, which changes the robot's configuration ...
    robot.SetConfig(q)
    ```

- The current configuration of a robot model does not have an effect on a real or simulated robot.  To drive the actual robot, you must use the interface in its Robot Controller.  To work with the current configuration of a real or simulated robot, the configuration must be read from the Robot Controller.

- A robot model does not necessarily correspond to a real robot.  Its kinematics, geometry, and limits need to be calibrated from the specifications of the real robot.

- Data can inadvertently "leak" from simulation to planning if a shared World Model is used, since the simulation will update the model for visualization.  An easy way to get around such conflicts is simply to copy the world into separate planning and simulation worlds.  (This is fast, since none of the geometry is actually copied.)  The standard pattern (in C++) is:

    ```cpp
    using namespace Klampt;
    WorldModel world, simWorld;
    ... set up the world model ...
    simWorld.Copy(world);
    WorldSimulation sim;
    sim.Init(&simWorld);  //the simulation will modify simWorld only
    MyPlanner planner(world);  //leaving you free to modify world, e.g., planning,  without interference
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
- Use STL and smart pointers rather than managing memory yourself.
- `KrisLibrary` contains a lot of functionality, including linear algebra routines, 3D math, optimization, geometric routines, OpenGL drawing, statistics, and graph structures. Browse [KrisLibrary](http://github.com/krishauser/KrisLibrary) before you reinvent the wheel.
- Avoid hard-coding. A much better practice is to place all settings into a class (e.g., with a `robotLeftHandXOffsetAmount` member) that gets initialized to a default value in the class' constructor. If you need to hard-code values, define them as const static variables or `#define`s at the top of your file. Name them descriptively, e.g., `gRobotLeftHandXOffsetAmount` is much better than `shift` or (God forbid) `thatStupidVariable`, when you come back to the file a month from now.
- The `main()` function in [Klampt/Main/simtest.cpp](../Main/simtest.cpp) is a good reference for setting up a world and a simulation from command-line arguments.

## Python Programming

The Klamp't Python API in [Klampt/Python](../Python) is much cleaner and easier to work with than the C++ API. For beginners or for rapid prototyping, this is the best API to use. However, it does not contain all of the functionality of the C++ API.

Missing features include:

- Robot reachability bound determination
- Advanced force/torque balance solvers
- Advanced motion planners (optimal planning with custom objective functions, kinodynamic planning, etc)
- Direct access to a robot's trajectory queue.

