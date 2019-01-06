# Klamp't [![Build Status](https://travis-ci.org/krishauser/Klampt.svg?branch=master)](https://travis-ci.org/krishauser/Klampt)

![Klamp't image](Cpp/docs/images/klampt-image.jpg)



Klamp't (Kris' Locomotion and Manipulation Planning Toolbox) is an open-source, cross-platform software package for robot modeling, simulating, planning, optimization, and visualization. It aims to provide an accessible, wide range of programming tools for learning robotics, analyzing robots, developing algorithms, and prototyping intelligent behaviors. It has particular strengths in robot manipulation and locomotion.

Historically, it began development at Indiana University since 2009 primarily as a research platform. Beginning in 2013 it has been used in education at Indiana University and Duke University. Since then, it has been adopted by other labs around the world.

More information can be found on the Klamp't website (http://klampt.org)

- [Features](#features)
- [Installation](#installation)
- [Documentation](#documentation)
- [Tutorials](#tutorials)
- [Version history](#version-history)
- [Who uses Klamp't?](#who-uses-klampt)
- [Comparison to related packages](#comparison-to-related-packages)
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


## Installation

Quick start (Python API only):
- `pip install klampt`

To run a visualization:
- `pip install PyOpenGL`
- `pip install PyQt5`
- `git clone http://github.com/krishauser/Klampt-examples` (this is needed to run example programs)
- `cd Klampt-examples/Python/demos`
- `python gl_vis.py`

Installation instructions are also available for
- [Linux](Tutorials/Install-Linux.md)
- [Windows](Tutorials/Install-Windows.md)
- [Mac OSX](Tutorials/Install-Mac.md)
- [Jupyter notebook](Jupyter/README.md)
- [Docker](Tutorials/Install-Docker.md)


## Documentation

[Python Manual and API Documentation](http://pythondocs.klampt.org/)

[C++ Manual](Cpp/docs/Manual.md)

API documentation is available here
- [Klamp't C++ API](http://cppdocs.klampt.org)
- [KrisLibrary C++ API](http://motion.pratt.duke.edu//krislibrary_docs)


## Version history

**0.8.1 Latest version** (1/3/2018)
-   Cleaned up documentation, separating C++ and Python docs.  Sphinx is now used for Python docs.
-   Added klampt_path utility to Python API.
-   Added Python API bindings for higher order dynamics functions that were already in the C++
    library (Cartesian accelerations, Hessians, derivatives of mass matrices).
-   Added rootfind Python C++ extension back.

**0.8.0 ** (12/17/2018)
-	Cleaner file structure, with C++ files in the Cpp directory.
-	Improved build system for Python, allowing easy installation via pip install python.
-	Integration with Jupyter Notebook 
-	Added Python utility programs (klampt_browse, klampt_sim, klampt_thumbnail).
-	Improvements support Python visualization on Mac.
-	Upgraded to PyQt5.  PyQt4 is still supported for now.
-	Geometry conversions exposed in Python via the convert function.
-	Improved usage of some graphics card resources for streaming point clouds.
-	Support for LOG4CXX logging.
-	Removed dependencies on Boost and upgraded to C++11.
-	Removed dependencies on GLUT and GLUI.  (Some examples still need to be upgraded to Qt.)
-	Cleaned up some cruft in KrisLibrary.


**0.7** (3/24/2017)

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



## Who uses Klamp't?

(This is not an exhaustive list; if you are using Klampt and would like to be listed, let us know!)

* [Intelligent Motion Laboratory (IML)](http://motion.pratt.duke.edu), on the Baxter, [TRINA](http://motion.pratt.duke.edu/nursing), [RoboSimian](http://motion.pratt.duke.edu/locomotion/tactile.html), and UR5 robots.  Klamp't has also been used in dozens of academic papers.

* Multiple IML open-source projects, including:

  * [Global Redundancy Resolution](https://github.com/krishauser/GlobalRedundancyResolution)
  * [Semi-Infinite Optimization](https://github.com/krishauser/SemiInfiniteOptimization)
  * [Inverse Kinematics Database (IKDB)](https://github.com/krishauser/ikdb)
  * [Robotic Systems Open Textbook](https://github.com/krishauser/RoboticSystemsBook)

* Duke courses (ECE 383, ECE 489)

* Team Duke in the Amazon Picking Challenge 2016-2018.

* [SimGrasp](https://web.stanford.edu/~shiquan/SimGrasp/sim-grasp-manual/about.html) at Stanford University

* Research labs at Brown, RPI, Columbia, and IIT Pisa

* TeamHubo in the DARPA Robotics Challenge

* [IROS 2016 Manipulation Challenge, Simulation Track](https://github.com/krishauser/IROS2016ManipulationChallenge)


## Comparison to related packages

- **ROS (Robot Operating System)** is a middleware system designed for distributed control of physical robots, and Klamp't is designed to be interoperable with it. Various ROS software packages can replicate many of the functions of Klamp't when used together (Gazebo, KDE, Rviz, MoveIt!), but this approach is difficult since these tools are not as tightly integrated as they are in Klamp't. ROS has limited support for legged robots, and is poorly suited for prototyping high-rate feedback control systems. ROS is heavy-weight, has a steep learning curve especially for non-CS students, and is also not completely cross-platform (only Ubuntu is fully supported).
- **OpenRAVE (Robotics and Animation Virtual Environment)** is similar to Klamp't and was developed concurrently by a similar group at CMU. OpenRAVE has more sophisticated manipulation planning functionality. Does not support planning for legged robots, but simulation is possible with some effort. Simulation models are often conflated with planning models whereas in Klamp't they are fully decoupled. OpenRAVE is no longer actively supported.
- **Gazebo, Webots, V-REP, etc** are robot simulation packages built off of the same class of rigid body simulations as Klamp't. They have more sophisticated sensor simulation capabilities, cleaner APIs, and nicer visualizations but are typically built for mobile robots and have limited functionality for modeling, planning, and optimization. Klamp't also has improved mesh-mesh collision handling that makes collision handling much more stable.



## Contributors

[Kris Hauser](mailto:kris.hauser@duke.edu) has been the primary maintainer throughout the project. Other major contributors include Jordan Tritell, Jingru Luo, and Alessio Rocchi.

Adam Konnecker, Cam Allen, Steve Kuznetsov have helped with the Mac build.

As an open-source project, we welcome contributions and suggestions from the community.


