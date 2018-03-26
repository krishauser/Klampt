# Klamp't Manual: Organization

The main philosophy behind the Klamp't design is to decouple Modeling, Planning, Control, and Simulation modules. This division provides a clear logical structure for developing large software systems for operating complex intelligent robots.

- _Modeling_ refers to the underlying knowledge representation available to the robot, e.g., limb lengths, physical parameters, environment, and other objects in its vicinity. The Modeling module contains methods for representing this knowledge. It also includes the ubiquitous mathematical models, such as kinematics and dynamics, trajectory representations (e.g., splines), and contact mechanics that required for planning and control.
- _Planning_ refers to the computation of paths, trajectories, feedback control strategies, configurations, or contact points for a robot. Planning may be performed either offline or online.
- _Control_ refers to the high-rate processing of sensor information into low-level robot controls (e.g., motor commands). This also includes state estimation. Note that the boundary between planning and control is fuzzy, because a fast planner can be used as a controller, or a planner can compute a feedback control strategy.
- _Simulation_ refers to a physical simulation of a virtual world that is meant _as a stand-in for the real world and robot_. The simulation module constructs a detailed physical rigid-body simulation and instantiates a controller and virtual sensors for a simulated robot. The controller then applies actuator commands that apply forces in the simulation.
- Auxiliary modules include _Visualization,_ referring to the display of a simulated or animated robot and its environment, _User interface_, and _I/O_, referring to the serialization and management of resources.

![Organization of library](images/concepts-overview.png)

Planning, control, and simulation are related by the use of (largely) common models. However, the simulation model does not need to be the same as the planner or controller's model. For example, an object's position may be imperfectly sensed, or a free-floating robot like a humanoid may not know precisely where its torso lies in 3D space. Also, for computational practicality a planner might work on a simplified model of the robot (e.g., ignoring the arms during biped walking) while the controller must expand that information into the full robot representation.



## C++ API file structure

- **Modeling** : `Klampt/{Modeling, Contact}/`, which depends heavily on `KrisLibrary/robotics` for basic robot kinematics and dynamics, and `KrisLibrary/{math3d, geometry, meshing}/` for 3-D geometry
- **Planning** : `Klampt/Planning/`, which depends heavily on `KrisLibrary/{planning, optimization}/`
- **Control** : `Klampt/Control/`
- **Simulation** : `Klampt/Simulation/`
- **Visualization:** `Klampt/View/`
- **User interface:** `Klampt/Interface/`
- **I/O:** native I/O is mostly embedded into models. Import/export to XML world files, ROS, and other external formats are found in `Klampt/IO/`.
- **Apps:** `Klampt/Main/`
- **Examples:** `Klampt/Examples`


## Python API file structure

The Klamp't Python API is primarily given in the `klampt` module found in `Klampt/Python`.  This module contains functionality in its sub-modules for modeling, simulation, planning, and visualization.  Control is handled in a separate module.

- `Klampt/Python/klampt`: the main Klamp't module, and includes robot kinematics, dynamics, simulation, and geometry representations. Also includes low-level IK solving and motion planning modules.
- `Klampt/Python/klampt/math`: basic 3D geometry.
- `Klampt/Python/klampt/modeling`: other modeling, including IK, trajectories, Cartesian interpolation, and sub-robot indexing. Setting and getting &quot;configurations&quot; for many objects.
- `Klampt/Python/klampt/plan`: motion planning for robots.
- `Klampt/Python/klampt/sim`: more advanced simulation functionality, such as logging and custom actuator and sensor emulation.
- `Klampt/Python/klampt/io`: Unified I/O for all types of Klamp't objects. Supports JSON formats for some objects as well. Resource loading, saving, and visual editing.
- `Klampt/Python/klampt/vis`: Visualization.
- `Klampt/Python/control`: custom control modules.
- `Klampt/Python/demos`: demonstrations about how to use various aspects of the Python klampt API.
- `Klampt/Python/exercises`: exercises for implementing basic concepts in Klamp't.
- `Klampt/Python/utils`: utility programs.

## Apps
- `RobotTest`, `RobotPose`, `SimTest`, `SimUtil`: the main useful apps
- `MotorCalibrate`, `URDFtoRob`: useful programs for setting up robot files.
- `Merge`: utility for merging several robots
- `Pack`, `Unpack`: utilities for building and separating composite resources, not widely used anymore.

## Build and temporary files

- `CMakeFiles.txt`: CMake main file describing the build configuration.
- `CMakeModules/': Auxiliary CMake files.  If you are building other projects using Klampt, the `KlamptDependencies.cmake` file will be useful to include.
- `CMakeCache.txt`: CMake cache file. May need to be deleted if you are having build problems.
- `CMakeFiles/`, `Python/build`, `Makefile` `cmake_install.cmake`, `cmake_uninstall.cmake`: temporary build files.  These can be safely deleted after a successful build.
- `tests`: A few basic test programs, built on `make test`.
- `lib/libKlampt.a': the main Klamp't library file to be included into new projects.
- `doxygen.conf`: documentation build file.
- `update.sh`: Linux script to cleanly update all of Klampt and KrisLibrary.
