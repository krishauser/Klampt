# Klamp't [![Build Status](https://travis-ci.org/krishauser/Klampt.svg?branch=master)](https://travis-ci.org/krishauser/Klampt)

![Klamp't image](Cpp/docs/images/klampt-image.jpg)



Klamp't (Kris' Locomotion and Manipulation Planning Toolbox) is an open-source, cross-platform software package for robot modeling, simulating, planning, optimization, and visualization. It aims to provide an accessible, wide range of programming tools for learning robotics, analyzing robots, developing algorithms, and prototyping intelligent behaviors. It has particular strengths in robot manipulation and locomotion.

Historically, it began development at Indiana University since 2009 primarily as a research platform.  Since then, it has been adopted in education and research labs around the world.

More information can be found on the Klamp't website (http://klampt.org)

- [Klamp't ](#klampt-)
  - [Features](#features)
  - [Installation](#installation)
  - [Documentation](#documentation)
  - [Reporting bugs and getting help](#reporting-bugs-and-getting-help)
  - [Version history](#version-history)
  - [Who uses Klamp't?](#who-uses-klampt)
  - [Comparison to related packages](#comparison-to-related-packages)
  - [Contributors](#contributors)


## Features

- Unified C++ and Python package for robot modeling, kinematics, dynamics, control, motion planning, simulation, and visualization.
- Interoperable with [Robot Operating System](http://ros.org) (ROS 1) and [Open Motion Planning Library](https://ompl.kavrakilab.org/) (OMPL).
- Stable file formats and tooling to save, load, and visualize robots (URDF), meshes, configurations, trajectories, poses, and more. 
- Built-in conversions to and from Numpy, JSON, ROS 1, Open3D, trimesh, PyTorch, and Sympy objects (in the Python API).
- Many geometry types implemented, including meshes, point clouds, signed distance functions, occupancy grids, geometric primitives, convex polytopes, and heightmaps.  Conversions, collision, distance, and ray-casting queries are available between [most pairs of geometry types](Cpp/docs/Manual-Geometry.md).
- Many sampling-based motion planners implemented (RRT, EST, SBL, RRT*, Lazy-RRG*, Lazy-PRM*, and more).
- Fast trajectory optimization routines.
- Real-time motion planning routines.
- Forward and inverse kinematics, forward and inverse dynamics.
- Contact mechanics computations (force closure, support polygons, stability of rigid bodies and actuated robots).
- Planning models are fully decoupled from simulation models. This helps simulate uncertainty and modeling errors.
- Robust rigid body simulation supporting triangle-soup and point cloud collisions.  No need to create convex decompositions!
- Simulation of PID controlled, torque controlled, and velocity controlled motors.
- Simulation of various sensors including cameras, depth sensors, laser range finders, gyroscopes, force/torque sensors, and accelerometers.
- Works on several platforms:
    - \*nux environments (x86_64, i686, Aarch64)
    - Windows
    - MacOS up to 11+
    - Google Colab


Note: newer versions of MacOS (11+) dropped OpenGL 2.0 support, so Klampt may not build. We're currently looking for alternative cross-platform graphics engines.

(Please let us know if you are able to compile on other platforms in order to help us support them in the future.)


## Installation

Quick start (Python API only):
- `pip install klampt` (or `pip3 install klampt`)

To run a visualization (you may need to replace `pip` with `pip3` and `python` with `python3`):
- `pip install PyOpenGL`
- `pip install PyQt6`
- `git clone http://github.com/krishauser/Klampt-examples` (this is needed to run example programs)
- `cd Klampt-examples/Python3/demos/sim`
- `python interactive_camera.py`

Installation instructions are also available for
- [Linux](Cpp/docs/Tutorials/Install-Linux.md)
- [Windows](Cpp/docs/Tutorials/Install-Windows.md)
- [Mac OSX](Cpp/docs/Tutorials/Install-Mac.md)
- [Jupyter notebook](Jupyter/README.md)
- [Docker](Cpp/docs/Tutorials/Install-Docker.md)

Klamp't works best when it is installed on your local machine, but it can also be run online through your web browser using Google Colab or Binder (or any other Jupyterhub server).

- Google Colab [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/gist/krishauser/1a518571493d2582f8bda908d9db02fb/klamptcolab.ipynb)
- Binder [![Open in Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/krishauser/Klampt-examples/binder?labpath=Jupyter)

Note that the UI functionality is drastically limited compared to a local installation.

## Documentation

[Python Manual and API Documentation](http://pythondocs.klampt.org/)

[C++ Manual](Cpp/docs/Manual.md)

API documentation is available here
- [Klamp't C++ API](http://cppdocs.klampt.org)
- [KrisLibrary C++ API](http://motion.cs.illinois.edu/software/krislibrary/krislibrary_docs)

## Reporting bugs and getting help

If you identify a programming bug or issue, please [raise them on this Github site](https://github.com/krishauser/Klampt/issues).
If you have general questions installing or programming with Klamp't, please ask them on the Klamp't forum, which
is available on GitQ: [https://gitq.com/krishauser/Klampt](https://gitq.com/krishauser/Klampt). 

## Version history

Note: If you have a `pip` installed Klampt from version 0.10.1, you may get the latest Python API updates by cloning the Git repo, then run `cd Klampt/Python; python patch_a_pip_install.py`. This provides all of the updates listed below that are prefixed with "Python API:" without needing to build from source.

**master** (1/8/2026)
-   Minor bug fixes.
-   Fixed sensor saving / loading within robot files.
-   Fixed crashing with Group geometry rendering after the geometry is transformed.
-   Fixed improper collision geometry when a geometry is updated.
-   Collada (DAE) files are now loaded without Assimp flipping the Y-Z axes.  GLTF and GLB files are now loaded to flip the Y-Z axes (placing Z up).
-   Group geometries can be loaded / saved as collections of triangle meshes from many file formats (DAE, Blend, GLB, GLTF).
-   Fixed OpenGL warnings for specular highlights out of OpenGL's range.
-   Fixed error in geometry conversion to ImplicitSurface type after collision data initialized.
-   Fixed error in fy / cy properties of cameras not being read properly.
-   Python API: Renamed `MotionPlan` to `KinematicPlanner` to distinguish from optimizers, kinodynamic planners, etc.  `MotionPlan` is still available as an alias; will be deprecated in some future version.
-   Python API: Added cost function queries and isOptimizing query for `MotionPlan` objects.
-   Python API: `cartesian_move_to` now supports link - transform pairs as arguments, omitting an `ik.objective` call.
-   Python API: OpenGL interface now supports non-integer screen-device scales.
-   Python API: New constructor for `Geometry3D` objects from file name. 
-   Python API: `[GEOM_DATA].copy()` now works as expected.
-   Python API: Fixed improper collision geometry when a geometry is updated via `set()`. 
-   Python API: `Geometry3D` has accessors to attached appearances, for meshes.  You can also extract appearance data using `getAppearance()`  
-   Python API: Fixed heightmap / viewport pose orientation flip on `getPose()`.
-   Python API: Added `Geometry3D.refreshCollider()` method to update the collider when the underlying geometry data is updated.
-   Python API: robots with some transparent and some opaque links are now drawn correctly.
-   Python API: `vis` item attributes are now consistently in *snake_case*.  Old camelCase attributes are still supported, for now. 
-   Python API: added geometry editor option in visualization to update the geometry's current transform along with the geometry.
-   Python API: fixed screwy AABBEditor behavior with non-identity frames.
-   Python API: Trajectory discretization is slightly faster due to seekIndex linear time argument.  Hermite trajectory time warping is fixed using new `timeWarp` method.

**0.10.1** (4/20/2025)
-   C++ API: Can use `KrisLibrary::setLogLevel()` function to control logging verbosity, with or without use of LOG4CXX library.
-   Python API: Added `klampt.set_log_level()` function to control logging verbosity, with or without use of LOG4CXX library.
-   Pip install version `0.10.1.post1` tags a specific version of Assimp to avoid Assimp's recent mesh import bug.

Full version history [is available here](Cpp/docs/Version-History.md)


## Who uses Klamp't?

(This is not an exhaustive list; if you are using Klampt and would like to be listed, let us know!)

* [Intelligent Motion Laboratory (IML)](http://motion.cs.illinois.edu), has used Klampt in dozens of academic papers, and approximately 10 robot platforms. It has been used on the [Kinova Gen 3](https://motion.cs.illinois.edu/stuff/index.html), teleoperation on three [TRINA platforms](https://motion.cs.illinois.edu/nursing), legged locomotion on the [RoboSimian](https://motion.cs.illinois.edu/locomotion/tactile.html), [Eye Examination Robots](https://motion.cs.illinois.edu/examinations/index.html) using the Universal Robots series (UR3, UR5) cobots. 

* Multiple IML open-source projects, including:

  * [Global Redundancy Resolution](https://github.com/krishauser/GlobalRedundancyResolution)
  * [Semi-Infinite Optimization](https://github.com/krishauser/SemiInfiniteOptimization)
  * [Inverse Kinematics Database (IKDB)](https://github.com/krishauser/ikdb)
  * [Robotic Systems Open Textbook](https://github.com/krishauser/RoboticSystemsBook)

* Courses at UIUC (CS 498) and Duke (ECE 383, ECE 489)

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

[Kris Hauser](mailto:kkhauser@illinois.edu) has been the primary maintainer throughout the project. Other major contributors include Zherong Pan, Gao Tang, Jordan Tritell, Jingru Luo, and Alessio Rocchi.

Adam Konnecker, Cam Allen, and Steve Kuznetsov have helped with the Mac build.  Hayden Bader helped with the prebuilt Docker container.

As an open-source project, we welcome contributions and suggestions from the community.


