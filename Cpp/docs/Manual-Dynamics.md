# Klamp't Manual: Dynamics and contact mechanics

## Robot Dynamic Parameters

Each body contains inertial properties, which are collectively set referred to as a `Mass` object.
- mass (float)
- center of mass (`Vector3`) given in local coordinates
- inertia matrix (`Matrix3`) H<sub>L</sub> given in local coordinates

Robot (.rob) and Rigid Object (.obj) files can automatically infer inertial properties from link masses using the `automass` line.

### Python API

- RobotLink.getMass(): returns the link's `Mass` structure
- RobotLink.setMass(mass): sets the link's `Mass` structure
- RigidObjectModel.getMass(): returns the object's `Mass` structure
- RigidObjectModel.setMass(mass): sets the object's `Mass` structure

`Mass` structure
- `mass.get/setMass(m)`: get/sets total mass to a float
- `mass.get/setCom(com)`: get/sets center of mass to a float 3-tuple in link-local coordinates 
- `mass.getInertia(inertia)`: gets inertia as a 9-vector, indicating rows of the inertia matrix in link-local coordinates 
- `mass.setInertia(inertia)`: sets inertia, either a float, float 3-tuple, or float 9-tuple. If float, sets inertia matrix to H<sub>L</sub>=I<sub>3x3</sub>\*`inertia`.  If 3-tuple, sets matrix to H<sub>L</sub>=diag(`inertia`)

## Robot Dynamics

The fundamental Langrangian mechanics equation is

<center> <em> B(q)q''+C(q,q')+G(q)=&tau;+&Sigma;<sub>i</sub> J<sub>i</sub>(q)<sup>T</sup> f<sub>i</sub> </em> </center>


Where:
- _q_ is configuration,
- _q'_ is velocity,
- _q''_ is acceleration,
- _B(q)_ is the positive semidefinite _mass matrix_,
- _C(q,q')_ is the _Coriolis force_,
- _G(q)_ is the _generalized gravity_,
- _&tau;_ is the link torque,
- _f<sub>i</sub>_  are _external forces_, and
- J<sub>i</sub>(q)  are the Jacobians of the points at which the points are applied.

A robot's motion under given torques and external forces can be computed by determining the acceleration q'' by multiplying both sides by _B<sup>-1</sup>_
and integrating the equation forward in time.

### C++ API
Klamp't has several methods for calculating and manipulating dynamics terms. The first set of methods is found in the `Robot` class, which use the &quot;classic&quot; Euler-Lagrange method that expands the terms mathematically in terms of Jacobians and Jacobian derivatives, and runs in _O(n<sup>3</sup>)_. 
- `CalcAcceleration`: used to convert the RHS to accelerations (_forward dynamics_).
- `CalcTorques` is used to convert from accelerations to the RHS (_inverse dynamics_).
- `KineticEnergy`: 
- `KineticEnergyInv`: 
- `GravityTorques`: 
- `CoriolisForce`: 
Note that these are actually defined in `RobotKinematics3D` and `RobotDynamics3D`.

The second set of methods uses the Newton-Euler rigid body equations and the Featherstone algorithm (KrisLibrary/robotics/NewtonEuler.h). These equations are _O(n)_ for sparsely branched chains and are typically faster than the classic methods for modestly sized robots (e.g., n&gt;6). Although NewtonEuler is designed particularly for the `CalcAccel` and `CalcTorques` methods for forward and inverse dynamics, it is also possible to use it to calculate the C+G term in O(n) time, and it can calculate the _B_ or _B<sup>-1</sup>_ matrices in _O(n<sup>2</sup>)_ time.

### Python API
The `RobotModel` class can compute each of these items using the Newton-Euler method:

- `robot.getCom()`: returns robot’s center of mass as a 3-tuple of floats
- `robot.getMassMatrix()`: returns nxn array (n nested length-n lists) describing the mass matrix _B_ at the robot’s current `Config`
- `robot.getMassMatrixInv()`: returns nxn array (n nested length-n lists) describing the inverse mass matrix _B<sup>-1</sup>_ at the robot’s current `Config`
- `robot.getCoriolisForceMatrix()`: returns nxn array (n nested length-n lists) describing the Coriolis force matrix  such that  at the robot’s current `Config` / `Velocity`
- `robot.getCoriolisForces()`: returns the length-n list giving the Coriolis torques  at the robot’s current `Config` / `Velocity`
- `robot.getGravityForces(gravity)`: returns the length-n list giving the generalized gravity G at the robot’s current `Config`, given the 3-tuple gravity vector (usually (0,0,-9.8))
- `robot.torquesFromAccel(ddq)`: solves for the inverse dynamics, returning the length-n list of joint torques that would produce the acceleration ddq at the current `Config`/`Velocity`
- `robot.accelFromTorques(torques)`: solves for the forward dynamics, returning the length-n list of joint accelerations produced by the torques torques at the current `Config`/`Velocity`


## Contact mechanics

Klamp't supports several operations for working with contacts. Currently these support legged locomotion more conveniently than object manipulation, because the manipulated object must be defined as part of the robot, and robot-object contact is considered self-contact.

### C++ API
These routines can be found in KrisLibrary/robotics, in particular Contact.h, Stability.h, and TorqueSolver.h.

- A `ContactPoint` is either a frictionless or frictional point contact. Consist of a position, normal, and coefficient of friction.
- A `ContactFormation` defines a set of contacts on multiple links of a robot. Consists of a list of links and a list of lists of contacts. For all indices `i`, `contacts[i]` is the set of contacts that affect `links[i]`.   Optionally, self-contacts may be defined by providing the list of target links `targets[i]`, with -1 denoting the world coordinate frame. Contact quantities may be given target space or in link-local coordinates is application-defined.
- The `TestCOMEquilibrium` functions test whether the center of mass of a rigid body can be stably supported against gravity by valid contact forces at the given contact list.
- The `EquilibriumTester` class provides richer functionality than `TestCOMEquilibrium`, such as force limiting and adding robustness factors. It may also save some memory allocations when testing multiple centers of mass with the same contact list.
- The `SupportPolygon` class explicitly computes a support polygon for a given contact list, and provides even faster testing than EquilibriumTester for testing large numbers of centers of mass (typically around 10-20).
- The `TorqueSolver` class solves for equilibrium of an articulated robot under gravity and torque constraints. It can handle both statically balanced and dynamically moving robots.

### Python API
These routines can be found in [klampt.model.contact](http://motion.pratt.duke.edu/klampt/pyklampt_docs/namespaceklampt_1_1model_1_1contact.html) which are thin wrappers around the underlying C++ functions.

- A `ContactPoint` is either a frictionless or frictional point contact. Consist of a position, normal, and coefficient of friction.  Unlike in C++, the `ContactPoint` data structure also has the option of referring to which objects are in contact.
- `forceClosure` tests whether a given set of contacts is in force closure.
- `comEquilibrium` tests whether the center of mass of a rigid body can be stably supported against gravity by valid contact forces at the given contact list.
- `supportPolygon` computes a support polygon for a given contact list. Testing the resulting boundaries of the support polygon is much faster than calling comEqulibrium multiple times.

- equilibriumTorques solves for equilibrium of an articulated robot under gravity and torque constraints. It can handle both statically balanced and dynamically moving robots.

## Holds, Stances, and Grasps

The contact state of a single link, or a related set of links, is modeled with three higher-level concepts.
- `Hold`: a set of contacts of a link against the environment and are used for locomotion planning.
- `Stance`: a set of `Hold`s. 
- `Grasp`: like a Hold, but can also contain constraints on the values of related link DOFs (e.g., the fingers). Generally used for manipulation planning but could also be part of locomotion as well (grasping a rail for stability, for example).

A `Hold` is defined as a set of contacts (the `contacts` member) and the associated IK constraint (the `ikConstraint` member) that keeps a link on the robot placed at those contacts.  These contacts are considered fixed in the world frame. `Hold`s may be saved and loaded from disk. The C++ API defines them in Klampt/Contact/Hold.h, which also defines convenience setup routines in the `Setup*` methods. The Python API defines them in `klampt.model.contact`.

A `Stance` (Klampt/Contact/Stance.h) defines all contact constraints of a robot. It is defined simply as a map from links to `Hold`s in C++, and is simply a list of `Hold`s in Python.

A `Grasp` (Klampt/Contact/Grasp.h) is more sophisticated than a `Hold` and are most appropriate for modeling hands that make contact with fingers. A `Grasp` defines an IK constraint of some link (e.g., a palm) relative to some movable object or the environment, as well as the values of related link DOFs (e.g., the fingers) and possibly the contact state. _Note: support for planning with `Grasp`s is limited in the current version._

