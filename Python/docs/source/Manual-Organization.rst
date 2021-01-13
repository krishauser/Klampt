Organization
============================

The main philosophy behind the Klamp't design is to decouple Modeling,
Planning, Control, and Simulation modules. This division provides a
clear logical structure for developing large software systems for
operating complex intelligent robots.

-  *Modeling* refers to the underlying knowledge representation
   available to the robot, e.g., limb lengths, physical parameters,
   environment, and other objects in its vicinity. The Modeling module
   contains methods for representing this knowledge. It also includes
   the ubiquitous mathematical models, such as kinematics and dynamics,
   trajectory representations (e.g., splines), and contact mechanics
   that required for planning and control.
-  *Planning* refers to the computation of paths, trajectories, feedback
   control strategies, configurations, or contact points for a robot.
   Planning may be performed either offline or online.
-  *Control* refers to the high-rate processing of sensor information
   into low-level robot controls (e.g., motor commands). This also
   includes state estimation. Note that the boundary between planning
   and control is fuzzy, because a fast planner can be used as a
   controller, or a planner can compute a feedback control strategy.
-  *Simulation* refers to a physical simulation of a virtual world that
   is meant *as a stand-in for the real world and robot*. The simulation
   module constructs a detailed physical rigid-body simulation and
   instantiates a controller and virtual sensors for a simulated robot.
   The controller then applies actuator commands that apply forces in
   the simulation.
-  Auxiliary modules include *Visualization,* referring to the display
   of a simulated or animated robot and its environment, *User
   interface*, and *I/O*, referring to the serialization and management
   of resources.

|Organization of library|

Planning, control, and simulation are related by the use of (largely)
common models. However, the simulation model does not need to be the
same as the planner or controller's model. For example, an object's
position may be imperfectly sensed, or a free-floating robot like a
humanoid may not know precisely where its torso lies in 3D space. Also,
for computational practicality a planner might work on a simplified
model of the robot (e.g., ignoring the arms during biped walking) while
the controller must expand that information into the full robot
representation.

Module structure
----------------------

The Klamp't Python API is organized as follows:


-  ``klampt``: the main Klamp't module, and includes robot
   kinematics, dynamics, simulation, and geometry representations. Also
   includes low-level IK solving and motion planning modules.
-  ``klampt.math``: basic 3D geometry.
-  ``klampt.modeling``: other modeling, including IK,
   trajectories, Cartesian interpolation, and sub-robot indexing.
   Setting and getting "configurations" for many objects.
-  ``klampt.plan``: motion planning for robots.
-  ``klampt.sim``: more advanced simulation functionality,
   such as logging and custom actuator and sensor emulation.
-  ``klampt.io``: Unified I/O for all types of Klamp't
   objects. Supports JSON formats for some objects as well. Resource
   loading, saving, and visual editing.
-  ``klampt.vis``: Visualization.
-  ``Klampt.control``: control modules.

You should also obtain the `Klampt-examples Github project <https://github.com/krishauser/Klampt-examples>`_ 
for examples of how to run the Klampt Python API.  Example Klampt data files are stored in the ``Klampt-examples/data``,
and include

- ``data``: world XML files
- ``data/robots``: robot files 
- ``data/objects``: rigid object files
- ``data/terrains``: terrain files
- ``data/motions``: motions
- ``data/resources``: an example resource collection
- ``data/simulation_test_worlds``: worlds to test simulator functionality

Python code examples are found in ``Klampt-examples/Python3``, and include:

-  ``Python3/demos``: demonstrations about how to use various
   aspects of the Python klampt API.
-  ``Python3/exercises``: exercises for implementing basic
   concepts in Klamp't.
-  ``Python3/utils``: utility programs.


Klampt project source
----------------------

The project source can be found in `https://github.com/krishauser/Klampt <https://github.com/krishauser/Klampt>`__.
The master branch will be kept up-to-date with bug fixes and other upgrades.
**It is recommended that you build from source if you are doing any significant development using Klampt.**

The C++ API is found in the ``Klampt/Cpp`` folder.  Files to build the Python API are found in ``Klampt/Python``.

Apps
~~~~

Once the apps are built, they will be found in the Klampt/bin/ folder.

-  ``RobotTest``, ``RobotPose``, ``SimTest``, ``SimUtil``: the main
   useful apps
-  ``MotorCalibrate``, ``URDFtoRob``: useful programs for setting up
   robot files.
-  ``Merge``: utility for merging several robots
-  ``Pack``, ``Unpack``: utilities for building and separating composite
   resources, not widely used anymore.

Build and temporary files
~~~~~~~~~~~~~~~~~~~~~~~~~

Build files

-  ``CMakeFiles.txt``: CMake main file describing the build
   configuration.
-  ``CMakeModules/``: Auxiliary CMake files.  If you are building other projects using Klampt, the ``KlamptDependencies.cmake``
   file will be useful to include.
-  ``tests``: A few basic test programs, built on ``make test``.
-  ``Cpp/Documentation/doxygen.conf``: C++ documentation build file (uses Doxygen)
-  ``Python/docs/``: Python documentation build files (uses Sphinx)
-  ``update.sh``: Linux script to cleanly update all of Klampt and
   KrisLibrary.
-  ``dist/linux_Python``: Docker-based scripts to build Python wheels.
-  ``dist/windows``: Windows script to cleanly update, build, and package all of Klampt and
   KrisLibrary.

Temporary files

-  ``lib/libKlampt.a``: Once built, the main Klamp't library file will be stored
   here to be included into new projects.
-  ``CMakeCache.txt``: CMake cache file. May need to be deleted if you
   are having build problems.
-  ``CMakeFiles/``, ``Python/build``, ``Makefile``
   ``cmake_install.cmake``, ``cmake_uninstall.cmake``: temporary build
   files. These can be safely deleted after a successful build.


.. |Organization of library| image:: _static/images/concepts-overview.png

