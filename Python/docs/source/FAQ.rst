Frequently asked questions (FAQ)
================================

What are some examples of cool things that Klampt can do?
---------------------------------------------------------

#. *Collects (nearly) all of the robotics essentials in a single software
   package*.  Visualization, kinematics, inverse kinematics, dynamics, collision
   detection, planning, and control are tightly integrated and easy to use. 
   The days of installing several packages just to solve an IK problem are
   over!

#. *Robust physics simulation without the pain*. (Or at least, with less pain.)
   Robots can be imported directly using their CAD models without having to
   define bounding volumes.

#. *Really simple visual debugging*.  With only a couple lines of code, you can
   see motions, configurations, geometries, and transforms, and even edit them.

#. *Well-supported file I/O*.  Almost all Klampt objects can be saved
   to disk, browsed visually, and loaded from disk in a single line of code.
   This makes it super easy to share data between programs, and to build large
   software systems with points, transforms, configurations, and paths treated
   as "assets".

#. *Integrated handling of multiple geometry types*.  Klamp't handles triangle
   meshes, point clouds, geometric primitives, and implicit surfaces.
   They can be used in collision detection, planning, and even
   simulation without preprocessing.

#. *Deploy robotics code using Jupyter notebook*.  The robot and world
   can be visualized, modified, and simulated in 3D (using WebGL) inside the
   notebook, and you can let collaborators modify parameters to rapidly see the
   results. 

#. *Make thousands of slightly different worlds, and simulate robot behavior in
   all of them*.  Copying Klamp't worlds is very cheap!  Moreover, you can
   save all the simulation trajectories and browse them easily.


Should I learn the Python bindings or C++?
------------------------------------------

This is mostly a matter of preference. Python tends to be cleaner,
easier to use, and faster for prototyping. However, the Python bindings
provide a subset of the C++ functionality.

Where should I get started learning the Klampt API?
---------------------------------------------------

The best place to start learning would be to get a sense of the `organization
of the library <Manual-Organization.html>`__, and then to learn the
`modeling functions <Manual-Modeling.html>`__.  You should be comfortable with
the concepts of the WorldModel, RobotModel, RigidObjectModel, and TerrainModel
classes, and their file structure. Browsing the ``data`` directory in the
`Klampt-examples <https://github.com/krishauser/Klampt-examples>`__ repository
will help you become familiarized with what is possible.

It is also essential to become comfortable with
`visualization <Manual-Visualization.html>`__ to be able to visualize whether
your programs are making sense.

After this, depending on your interests, most people choose one of three
routes:

#. Low-level `simulation <Manual-Simulation.html>`__ and
   `control <Manual-Control.html>`__ functionality
#. Medium-level `trajectory representation <Manual-Paths.html>`__, execution
   tools (:meth:`klampt.model.trajectory.path_to_trajectory`, :meth:`klampt.model.trajectory.execute_trajectory`)
   and `inverse kinematics <Manual-IK.html>`__ and `Cartesian path execution <klampt.model.cartesian_path.html>`__.
#. High-level `motion planning <Manual-Planning.html>`__ tools.


How do I import a URDF file properly?
-------------------------------------

Please see the
`tutorial here <https://github.com/krishauser/Klampt/blob/master/Cpp/docs/Tutorials/Import-and-calibrate-urdf.md>`__.

My simulator goes unstable and/or crashes. Help!
------------------------------------------------

There are two reasons that the simulator may go unstable: 1) the
simulated robot is controlled in an inherently unstable manner, or 2)
rigid body simulation artifacts due to poor collision handling or
numerical errors. The second reason may also cause ODE to crash,
typically on Linux systems. In testing we have found that configuring
ODE with double precision fixes such crashes.

*Unstable robot*: an unstably controlled robot will oscillate and
jitter, and if these oscillations become violent enough they may also
cause rigid body simulation instability/crashing. If the robot goes
unstable, then its PID constants and dryFriction/viscousFriction terms
need to be tuned. These values must be set carefully in order to avoid
oscillation and, ideally should be calibrated against the physical
motors' behavior. This is currently an entirely manual process that must
be done for every new robot. As a rule of thumb, large PID damping terms
are usually problematic, and should be emulated as viscous friction.

*Collision handling errors*: Klamp't uses a contact handling method
wherein each mesh is wrapped within a thin *boundary layer* that is used
for collision detection. When objects make contact only along their
boundary layers, the simulation is robust, but if their underlying
meshes penetrate one another, then the simulator must fall back to less
robust contact detection methods. This occurs if objects are moving too
quickly or light objects in contact are subject to high compressive
forces. If this happens, Klamp't will print a warning of the form
"ODECustomMesh: Triangles penetrate margin X, cannot trust contact
detector". The simulator status will also return "unreliable."

To avoid penetration, there are two remedies: 1) increase the thickness
of the boundary layer, or 2) make the boundary layer stiffer. See the
`Simulation section of the Klamp't Manual <Manual-Simulation.html>`__ for
more details on how to implement these fixes.

The simulator runs slowly. How can I make it faster?
----------------------------------------------------

Unless you are simulating a huge number of joints, the limiting steps in
simulation are usually contact detection and calculating the contact
response.

The speed of contact detection is governed by the resolution of the
meshes in contact. Simpler meshes will lead to faster contact detection.
Most 3D modeling packages will provide mesh simplification operators.

The speed of contact response is governed by the number of contact
points retained in the contact handling procedure after clustering. The
maxContacts simulation parameter governs the number of clusters and can
be reduced to achieve a faster simulation. However, setting this value
too low will lead to a loss of physical realism.


How do I tune PID gains for my robot?
-------------------------------------

The easiest way is to tune them manually in the .rob or .urdf file
while observing the response to forces applied with the mouse in ``klampt_sim``. 
Some tips include

- If the simulation goes unstable, turn kD to 0, and increase the dryFriction
  and viscousFriction terms until it stabilizes.
- All terms for a joint should be proportional to the amount of mass controlled
  by it (F = MA).  As a result, gains for joints controlling small elements (like a
  finger) will be much smaller than joints controlling large elements
  (like a shoulder).
- kP will often be much larger than kD or kI (2 or 3 orders of magnitude).
- If your joints don't move as commanded, the dryFriction and viscousFriction
  terms are too large.
- Start fine-tuning distal links (those farther from the root) and then proceed
  toward the root.

If you have built from source, and have data from your real robot,
you can also use the `MotorCalibrate` program.see the
`tutorial here <https://github.com/krishauser/Klampt/blob/master/Cpp/docs/Tutorials/Import-and-calibrate-urdf.md>`__
for instructions.


How do I implement a behavior script?
-------------------------------------

Many engineers and students tend to approach robotics from a "scripting"
approach, whereby a complex behavior is broken down into a script or
state machine of painstakingly hand-tuned, heuristic behaviors. Unlike
some other packages, Klamp't does not try to make scripting convenient.
This choice was made deliberately in order to discourage the use of
heuristic behaviors. The philosophy is that *hand-tuned behaviors should
be rare in intelligent robots*. However, it is true that scripts / state
machines are sometimes the easiest way to accomplish a given behavior
with the current generation of robot AI tools.

To implement a behavior script in Klamp't, the script should be run
in a separate thread from the execution thread. It can then monitor the
state of the execution thread (e.g., waiting for a movement to finish)
and react accordingly.
The multithreaded visualization available in Linux and Windows is very
convenient for this. See the `visualization documentation <Manual-Visualization.html#multithreaded-mode>`__
for instructions and examples.  

To implement a state machine, a controller should manually maintain and
simulate its behavior in its feedback loop. A framework for such
controllers is the ``StateMachineController`` class in
`Python/control/controller.py <../Python/control/controller.py>`__.
