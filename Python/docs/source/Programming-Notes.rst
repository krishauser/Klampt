Programming notes and tips
==========================

Common "gotchas"
----------------

-  A robot model's configuration is used in many places as input and
   output! To guard against losing configuration, *think of the robot's
   configuration as a temporary variable.* If you need to keep the
   configuration between computations, just store it before the
   computations, and restore it afterwards. The standard pattern (in
   Python) is:

   .. code:: python

       q = robot.getConfig()
       #... do stuff ...
       robot.setConfig(q)

-  The current configuration of a robot model does not have an effect on
   a real or simulated robot. To drive the actual robot, you must use
   the interface in its Robot Controller. To work with the current
   configuration of a real or simulated robot, the configuration must be
   read from the Robot Controller.

-  A robot model does not necessarily correspond to a real robot. Its
   kinematics, geometry, and limits need to be calibrated from the
   specifications of the real robot.

-  When performing multithreading, such as in the Python vis module,
   crashes can occur if locking is not performed properly to prevent
   simultaneous access to World Models.

-  Data can inadvertently "leak" from simulation to planning if a shared
   World Model is used, since the simulation will update the model for
   visualization. An easy way to get around such conflicts is simply to
   copy the world into separate planning and simulation worlds. (This is
   fast, since none of the geometry is actually copied.) The standard
   pattern (in Python) is:

   .. code:: python

       world = WorldModel()
       #... set up the model ...
       simWorld = world.copy()
       sim = Simulator(simWorld)
       planWorld = world   #no need to copy again
       planner = MyPlanner(planWorld)

General
-------

-  `Ask questions on Gitq <https://gitq.com/krishauser/Klampt>`__ and
   `report issues/bugs on Github <https://github.com/krishauser/Klampt/issues>`__.
   This will help us make improvements to Klamp't.
-  If you write a piece of code that you think
   will be useful to others, consider making it a contribution to the
   library!
-  Practice *self-documenting code*. Name files, functions, classes, and
   variables descriptively. Comment as you go.
-  Use *visual debugging* to debug your algorithms. :func:`klampt.vis.debug`
   is your friend.  Also, for many objects, you may call :func:`klampt.io.resource.edit`
   to pop up a window for editing.
-  *Save to disk*. Most Klamp't objects can be saved / loaded to disk using
   the functions in the :mod:`klampt.io.loader` module, or better yet,
   :func:`klampt.io.resource.set` and :func:`klampt.io.resource.get`.  The
   files can later be inspected using ``klampt_browser`` and other tools.
-  *Think statefully*. Decompose your programs into algorithms, state,
   parameters, and data. State is what the algorithm changes during its
   running. Parameters are values that are given as input to the
   algorithm when it begins (arguments and settings), and they do not
   change during execution. Data is the static knowledge available to
   the algorithms and the information logged as a side effect of its
   execution.
-  When prototyping long action sequences, build in functionality to
   save and restore the state of your system at intermediate points.


Missing Python Features
-----------------------

The Python API is much cleaner and easier to work with than the C++
API.  However, it does not contain all of the functionality of the C++ API.
Missing features include:

-  Some contact processing algorithms
-  Robot reachability bound determination
-  Advanced force/torque balance solvers
-  Advanced motion planners (kinodynamic planning, minimum constraint removal, etc)
-  Direct access to a robot's trajectory queue.


Useful Submodules
~~~~~~~~~~~~~~~~~~

The Klamp't Python API contains several submodules that are not discussed in
detail in the manual.  You may wish to look these over.


-  `cartesian\_trajectory <klampt.model.cartesian_trajectory.html>`__: reliable
   methods for converting Cartesian space trajectories to joint space
   trajectories.
-  `config <klampt.model.config.html>`__: treats the configuration of
   multiple objects as a single flat configuration vector.
-  `coordinates <klampt.model.coordinates.html>`__: a coordinate manager,
   similar to the ROS ``tf`` module.
-  `create <klampt.model.create.html>`__: helpers to create robots, geometric
   primitives, and piles of objects.
-  `access <klampt.model.access.html>`__: provides a more Pythonic way to access
   items in a world.
-  `subrobot <klampt.model.subrobot.html>`__: defines :class:`~klampt.model.subrobot.SubRobotModel`,
   a class that is ``RobotModel``-like but only modifies selected degrees of
   freedom (e.g., an arm, a leg).
-  `types <klampt.model.types.html>`__: retrieves the type string for various
   Klamp't objects.
-  `cspaceutils <klampt.plan.cspaceutils.html>`__: contains helpers for
   constructing composite CSpaces and slices of CSpaces.
-  `settle <klampt.sim.html#module-klampt.sim.settle>`__: a convenience
   function to let objects fall under gravity and extract their
   equilibrium configurations.
-  `simlog <klampt.sim.html#module-klampt.sim.simlog>`__: simulation logging classes (used in SimpleSimulator)
-  `simulation <klampt.sim.html#module-klampt.sim.simulation>`__: a more full-featured simulation class than standard
   Simulation. Defines sensor and actuator emulators, sub-step force
   appliers, etc.


