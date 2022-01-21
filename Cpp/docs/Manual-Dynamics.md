# Klamp't Manual: Dynamics and contact mechanics

* [Robot Dynamic Parameters](#robot-dynamic-parameters)
* [Robot Dynamics](#robot-dynamics)
  + [API summary](#api-summary)
* [Contact mechanics](#contact-mechanics)
  + [API summary](#api-summary-1)
* [Holds, Stances, and Grasps](#holds--stances--and-grasps)

## Robot Dynamic Parameters

Each body contains inertial properties, which are collectively set referred to as a `Mass` object.
- mass (float)
- center of mass (`Vector3`) given in local coordinates
- inertia matrix (`Matrix3`) H<sub>L</sub> given in local coordinates

Robot (.rob) and Rigid Object (.obj) files can automatically infer inertial properties from link masses using the `automass` line.

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

### API summary
Klamp't has several methods for calculating and manipulating dynamics terms. The first set of methods is found in the `RobotModel` class, which use the &quot;classic&quot; Euler-Lagrange method that expands the terms mathematically in terms of Jacobians and Jacobian derivatives, and runs in _O(n<sup>3</sup>)_. 
- `CalcAcceleration`: used to convert the RHS to accelerations (_forward dynamics_).
- `CalcTorques` is used to convert from accelerations to the RHS (_inverse dynamics_).
- `KineticEnergy`: 
- `KineticEnergyInv`: 
- `GravityTorques`: 
- `CoriolisForce`: 
Note that these are actually defined in `RobotKinematics3D` and `RobotDynamics3D` in `KrisLibrary`.

The second set of methods uses the Newton-Euler rigid body equations and the Featherstone algorithm (KrisLibrary/robotics/NewtonEuler.h). These equations are _O(n)_ for sparsely branched chains and are typically faster than the classic methods for modestly sized robots (e.g., n&gt;6). Although NewtonEuler is designed particularly for the `CalcAccel` and `CalcTorques` methods for forward and inverse dynamics, it is also possible to use it to calculate the C+G term in O(n) time, and it can calculate the _B_ or _B<sup>-1</sup>_ matrices in _O(n<sup>2</sup>)_ time.


## Contact mechanics

Klamp't supports several operations for working with contacts. Currently these support legged locomotion more conveniently than object manipulation, because the manipulated object must be defined as part of the robot, and robot-object contact is considered self-contact.

### API summary
These routines can be found in KrisLibrary/robotics, in particular Contact.h, Stability.h, and TorqueSolver.h.

- A `ContactPoint` is either a frictionless or frictional point contact. Consist of a position, normal, and coefficient of friction.
- A `ContactFormation` defines a set of contacts on multiple links of a robot. Consists of a list of links and a list of lists of contacts. For all indices `i`, `contacts[i]` is the set of contacts that affect `links[i]`.   Optionally, self-contacts may be defined by providing the list of target links `targets[i]`, with -1 denoting the world coordinate frame. Contact quantities may be given target space or in link-local coordinates is application-defined.
- The `TestCOMEquilibrium` functions test whether the center of mass of a rigid body can be stably supported against gravity by valid contact forces at the given contact list.
- The `EquilibriumTester` class provides richer functionality than `TestCOMEquilibrium`, such as force limiting and adding robustness factors. It may also save some memory allocations when testing multiple centers of mass with the same contact list.
- The `SupportPolygon` class explicitly computes a support polygon for a given contact list, and provides even faster testing than EquilibriumTester for testing large numbers of centers of mass (typically around 10-20).
- The `TorqueSolver` class solves for equilibrium of an articulated robot under gravity and torque constraints. It can handle both statically balanced and dynamically moving robots.

## Holds, Stances, and Grasps

The contact state of a single link, or a related set of links, is modeled with three higher-level concepts.
- `Hold`: a set of contacts of a link against the environment and are used for locomotion planning.
- `Stance`: a set of `Hold`s. 
- `Grasp`: like a Hold, but can also contain constraints on the values of related link DOFs (e.g., the fingers). Generally used for manipulation planning but could also be part of locomotion as well (grasping a rail for stability, for example).

A `Hold` is defined as a set of contacts (the `contacts` member) and the associated IK constraint (the `ikConstraint` member) that keeps a link on the robot placed at those contacts.  These contacts are considered fixed in the world frame. `Hold`s may be saved and loaded from disk. The C++ API defines them in Klampt/Cpp/Contact/Hold.h, which also defines convenience setup routines in the `Setup*` methods. The Python API defines them in `klampt.model.contact`.

A `Stance` (Klampt/Cpp/Contact/Stance.h) defines all contact constraints of a robot. It is defined simply as a map from links to `Hold`s in C++, and is simply a list of `Hold`s in Python.

A `Grasp` (Klampt/Cpp/Contact/Grasp.h) is more sophisticated than a `Hold` and are most appropriate for modeling hands that make contact with fingers. A `Grasp` defines an IK constraint of some link (e.g., a palm) relative to some movable object or the environment, as well as the values of related link DOFs (e.g., the fingers) and possibly the contact state. _Note: support for planning with `Grasp`s is limited in the current version._

