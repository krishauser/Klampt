RobotSim Python Wrappers

Kris Hauser

12/15/2012
r2 3/10/2012
r1 10/15/2012

************** About ****************

This is a Python wrapper for much of the RobotSim functionality
regarding robot kinematics, dynamics, and simulation.

The simulation engine is ODE, with some modifications to improve contact
detection.

Functionality is provided to compute inverse kinematics completely in
C++, which is about 20x faster than using the rootfind library.


************** Getting up and running ****************

1. Build the RobotSim static library using 'make lib' in the root
   RobotSim directory.
2. Type 'make' to build the Python library using Python distutils.  You 
   may have to edit some directories in setup.py to get this running.
3. Type 'make docs' to build the Doxygen documentation of the Python API,
   which is found in docs/html/index.html and robotsim.html.  (Note: the
   SWIG auto-generated documentation in robotsim.html is somewhat hard to
   read.)

Once built, the fastest way to get started is to look at these examples:
- test.py, which loads a world file and solve an inverse kinematics query.
- gltest.py, which loads a world file, simulates it using ODE, and sends
  go-to commands.
To run the examples, copy them up one directory and run python [file].
For gltest, PyOpenGL must be installed.  The data files for these programs
are located in the robot/tx90blocks.xml file and in the robot/data folder.

************** Python API ****************

The core functionality is provided by robotsim.py

The files contact.py, ik.py, map.py, collide.py, and cspace.py
provide convenience routines built on top of robotsim.py.

- contact.py allows querying contact maps from a simulator and computing
  wrench matrices.

- ik.py provides convenience routines for generalized IK solving.  Since
  GeneralizedIKSolvers are somewhat more expensive than regular IKSolvers,
  this will find an optimized set of objectives and the best solver for
  a set of IK goals.

- map.py provides convenient object-oriented access to getters/setters.
  For example, you can write
    wm = map.map(world)
    wm.robots[0].links[4].transform
  instead of
    world.robot(0).getLink(4).getTransform().

- robotcollide.py defines a WorldCollider class that enables querying the
  collision status of the world and subsets of bodies in the world.

- robotcspace.py defines a configuration space for a robot in a world to be
  used with cspace.py (defined externally to this package).



************** World files ****************

These files set up the scene with robots, objects, and static geometries
and describe how they are simulated.  They consist of two parts: the world
model, which is used by both planners and simulators, and the simulation
setup, which is optional and used only for simulating.

The XML heirarchy is as follows:

world
  terrain : name, file, scale, translation, position, kFriction, rotateX, rotateY, rotateZ
    display : color, texture
  rigidObject : name, file, position, rotateX, rotateY, rotateZ
    geometry : mesh, scale, translate
    physics : mass, com, inertia, automass, kFriction, kRestitution, kStiffness, kDamping
  robot : name, file
  simulation
    globals : gravity, ERP, CFM, maxContacts
    env : index
      geometry : padding, kFriction, kRestitution, kStiffness, kDamping
    object : index
      geometry : padding, kFriction, kRestitution, kStiffness, kDamping
    robot : index, body
      geometry : padding, kFriction, kRestitution, kStiffness, kDamping
      sensors 
        position : enabled, variance, resolution
        velocity : enabled, variance, resolution
      controller : gravityCompensation, feedforwardAcceleration, log, vscale, ascale
    state : data

To initialize from a different simulation state, you can run the simulation, 
then print the string outputted by Simulation.getState().  Take this string
and put it in the simulation/state.data attribute.


************** Robot files (.rob) ****************

These text files describe a robot's kinematic and dynamic model which is
used by the simulation and planners.


************** Rigid object files (.obj) ****************

These text files describe a rigid object's geometry and physical parameters
to be used by the simulation and planners.


************** Triangle meshes (.tri) ****************

A simple file format for triangle meshes.  See KrisLibrary for reference.


