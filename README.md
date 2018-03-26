# Klamp't [![Build Status](https://travis-ci.org/krishauser/Klampt.svg?branch=master)](https://travis-ci.org/krishauser/Klampt)

![Klamp't image](Documentation/images/klampt-image.jpg)



Klamp't (Kris' Locomotion and Manipulation Planning Toolbox) is an open-source, cross-platform software package for robot modeling, simulating, planning, optimization, and visualization. It aims to provide an accessible, wide range of programming tools for learning robotics, analyzing robots, developing algorithms, and prototyping intelligent behaviors. It has particular strengths in robot manipulation and locomotion.

Historically, it began development at Indiana University since 2009 primarily as a research platform. Beginning in 2013 it has been used in education at Indiana University and Duke University. Since then, it has been adopted by other labs around the world, such as Stanford, University of Pisa, and Worcester Polytechnic Institute. It has been used in several real-world projects, including the DARPA Robotics Challenge, Amazon Picking Challenge (2015-2016), the IROS 2016 Robot Grasping and Manipulation Challenge, and Stanford's SimGrasp toolbox.

More information can be found on the Klamp't website (http://klampt.org)

- [Features](#features)
- [Comparison to related packages](#comparison-to-related-packages)
- [Installation](#installation)
- [Documentation](#documentation)
- [Tutorials](#tutorials)
- [Version history](#version-history)
- [Contributors](#contributors)


## Features

- Unified C++ and Python package for robot modeling, kinematics, dynamics, control, motion planning, simulation, and visualization.
- Supports legged and fixed-based robots.
- Interoperable with [Robot Operating System](http://ros.org) (ROS) and [Open Motion Planning Library](https://ompl.kavrakilab.org/) (OMPL).
- Many sampling-based motion planners implemented.
- Fast trajectory optimization routines.
- Real-time motion planning routines.
- Forward and inverse kinematics, forward and inverse dynamics
- Contact mechanics computations (force closure, support polygons, stability of rigid bodies and actuated robots)
- Planning models are fully decoupled from simulation models. This helps simulate uncertainty and modeling errors.
- Robust rigid body simulation with triangle mesh / triangle mesh collisions.
- Simulation of PID controlled, torque controlled, and velocity controlled motors.
- Simulation of various sensors including cameras, depth sensors, laser range finders, gyroscopes, force/torque sensors, and accelerometers.
- Works on several platforms:
    - \*nux environments
    - Windows
    - MacOS
    
(Please let us know if you are able to compile on other platforms in order to help us support them in the future.)

## Comparison to related packages

- **ROS (Robot Operating System)** is a middleware system designed for distributed control of physical robots, and Klamp't is designed to be interoperable with it. Various ROS software packages can replicate many of the functions of Klamp't when used together (Gazebo, KDE, Rviz, MoveIt!), but this approach is difficult since these tools are not as tightly integrated as they are in Klamp't. ROS has limited support for legged robots, and is poorly suited for prototyping high-rate feedback control systems. ROS is heavy-weight, has a steep learning curve especially for non-CS students, and is also not completely cross-platform (only Ubuntu is fully supported).
- **OpenRAVE (Robotics and Animation Virtual Environment)** is similar to Klamp't and was developed concurrently by a similar group at CMU. OpenRAVE has more sophisticated manipulation planning functionality. Does not support planning for legged robots, but simulation is possible with some effort. Simulation models are often conflated with planning models whereas in Klamp't they are fully decoupled. OpenRAVE is no longer actively supported.
- **Gazebo, Webots, V-REP, etc** are robot simulation packages built off of the same class of rigid body simulations as Klamp't. They have more sophisticated sensor simulation capabilities, cleaner APIs, and nicer visualizations but are typically built for mobile robots and have limited functionality for modeling, planning, and optimization. Klamp't also has improved mesh-mesh collision handling that makes collision handling much more stable.



## Installation

An installation tutorial is available for
- [Linux](Documentation/Tutorials/Install-Linux.md)
- [Windows](Documentation/Tutorials/Install-Windows.md)
- [Mac OSX](Documentation/Tutorials/Install-Mac.md).

You can also use Docker [as described here](Documentation/Install-Docker.md).


## Documentation

A more detailed manual of the library's components is available here:
- [Installation](Documentation/Manual-Installation.md)
- [Organization](Documentation/Manual-Organization.md)
- [Running apps](Documentation/Manual-Apps.md)
- [Math](Documentation/Manual-Math.md) 
- [Kinematic modeling](Documentation/Manual-Modeling.md)
- [Inverse kinematics](Documentation/Manual-IK.md)
- [Geometry and appearance](Documentation/Manual-Geometry.md)
- [Paths and trajectories](Documentation/Manual-Paths.md)
- [Motion planning](Documentation/Manual-Planning.md)
- [Simulation](Documentation/Manual-Simulation.md)
- [Dynamics and contact mechanics](Documentation/Manual-Dynamics.md)
- [Visualization](Documentation/Manual-Visualization.md)
- [Control and sensing](Documentation/Manual-Control.md)
- [System integration](Documentation/Manual-Systems.md)
- [I/O](Documentation/Manual-IO.md)
- [File types](Documentation/Manual-FileTypes.md)
- [Resource management](Documentation/Manual-Resources.md) (incomplete)

API documentation is available here
- [Klamp't API](http://klampt.org/klampt_docs)
- [Klamp't Python bindings API](http://klampt.org/pyklampt_docs)
- [KrisLibrary API](http://klampt.org/krislibrary_docs)

Other documentation includes:
- [FAQ](Documentation/FAQ.md)
- [Programming notes](Documentation/Programming-Notes.md)
- [Notes about reading Python documentation](Documentation/Reading-Python-Docs.md), which can be a little confusing for some classes due to the Python binding generator (SWIG).



## Tutorials 

Tutorials are available in the [Tutorials folder](Documentation/Tutorials), with a more complete list on the [Klamp't website](http://klampt.org#tutorials).  (Stay tuned... we are in the process of updating the HTML tutorials to Markdown so they can be browsed more easily on Github.)

- Install Klampt ([Linux](Documentation/Tutorials/Install-Linux.md), [Windows](Documentation/Tutorials/Install-Windows.md), [Mac](Documentation/Tutorials/Install-Mac.md))
- Edit a world file
- Dynamically create a world using code (Python)
- Generate a path/trajectory from keyframes (Apps, Python)
- Animate a video of a path/trajectory (Apps, Python)
- Simulate the execution of a keyframe path (Apps, Python)
- Implement a custom controller for a simulated robot (C++, Python)
- Implement a custom controller for a real robot (C++, Python)
- Process clicks on the robot or world (C++, Python)
- Run a motion planner (C++, Python)
- Add a custom feasibility test to a motion planner (C++, Python)
- Set up a simulated camera sensor and save frames to disk (Apps, Python)
- Display ROS point cloud messages (Python)


## Version history

**0.7 Latest version** (3/24/2017)

- Improved simulation stability, including adaptive time stepping and instability detection/recovery.
- The proprietary `.tri` geometry file format has been replaced with the Object File Format (OFF) for better compatibility with 3D modeling packages.
- Simulated visual, depth, and laser range sensors are now fully supported.
- ROS sensor simulation broadcasting is enabled in Klampt/IO/ROS.h.
- World XML files can now be saved to disk.
- Robot sensors and controllers can be attached directly to a robot model using the sensors / controller properties in the robot's `.rob` or `.urdf` file.
- The motion planning structure in KrisLibrary has been completely revamped in preparation for support of optimal and kinodynamic planning, but this should be a mostly transparent change to Klamp't users.
- The Python interface is now better organized.  _However, the module structure is incompatible with code developed for versions 0.6.2 and earlier_. In particular, math modules (`vectorops`, `so3`, `se3`) are now in the `math` subpackage, and visualization modules (`glprogram`, `glrobotprogram`, etc) are now in the vis subpackage.
- Custom Python simulations of sensors, actuators, and force appliers that work on fast simulation rates are easier to integrate with slower control loops in the sim.simulation module.
- Revamped and enhanced Python visualization functionality in the vis module. Multiple windows, simultaneous viewports, trajectory visualization, custom in-visualization plotting, automatic viewport determination, and thumbnail saving are now supported.
- Cartesian trajectory generation, file loading utilities are added to Python.

**0.6.2** (7/31/2016)

- New Python APIs for visualization
- Geometry caching helps load times and memory usage for large scenes
- A global IK solver has been added to the Python API
- ROS broadcasting / subscribing is enabled in the C++ API.

**0.6.1** (3/21/2015)

- Added functionality in Python API to load/save/edit resources, manipulate transforms and robot configurations via widgets, change appearance of objects, and run programs through Qt.
- Removed the Python `collide` module. All prior functionality is now placed in the `Geometry3D` class in the standard `klampt` module.
- Real-time planning interface has been greatly simplified.
- The `MilestonePathController` class will be deprecated, use `PolynomialPathController` instead.
- Minor bug fixes

**0.6.** (7/4/2014)

- CMake build system makes it easier to build across multiple platforms
- Easy connections with external controllers via ROS or a serial protocol
- More user-friendly Qt application front ends
- More demos, example code, and tutorials
- Direct loading of URDF files with &lt;klampt&gt; XML tag
- More calibrated robots: Baxter, RobotiQ 3-finger adaptive gripper
- Unification of locomotion and manipulation via the GeneralizedRobot mechanism
- Fixed build for Cygwin
- More sophisticated logging capabilities in SimTest (contacts, commanded/actual/sensed paths)
- Miscellaneous debugging throughout

**0.5. Initial release** (11/17/2013)

## Contributors

[Kris Hauser](mailto:kris.hauser@duke.edu) has been the primary maintainer throughout the project. Other major contributors include Jordan Tritell, Jingru Luo, and Alessio Rocchi.

Adam Konnecker, Cam Allen, Steve Kuznetsov have helped with the Mac build.

As an open-source project, we welcome contributions and suggestions from the community.


