Dynamics and contact mechanics
==============================================

Robot Dynamic Parameters
------------------------

Each body contains inertial properties, which are collectively set
referred to as a ``Mass`` object.

-  mass (float)
-  center of mass (``Vector3``) given in local coordinates
-  inertia matrix (``Matrix3``) :math:`H_L` given in local coordinates

Robot (.rob) and Rigid Object (.obj) files can automatically infer
inertial properties from link masses using the ``automass`` line.

API summary
~~~~~~~~~~~

-  ``RobotLink.getMass()``: returns the link's ``Mass`` structure
-  ``RobotLink.setMass(mass)``: sets the link's ``Mass`` structure
-  ``RigidObjectModel.getMass()``: returns the object's ``Mass`` structure
-  ``RigidObjectModel.setMass(mass)``: sets the object's ``Mass`` structure

``Mass`` structure

-  ``mass.get/setMass(m)``: get/sets total mass to a float
-  ``mass.get/setCom(com)``: get/sets center of mass to a float 3-tuple
   in link-local coordinates
-  ``mass.getInertia(inertia)``: gets inertia as a 9-vector, indicating
   rows of the inertia matrix in link-local coordinates
-  ``mass.setInertia(inertia)``: sets inertia, either a float, float
   3-tuple, or float 9-tuple. If float, sets inertia matrix to
   :math:`H_L=I_{3x3} \cdot`  ``inertia``. If 3-tuple, sets matrix to
   :math:`H_L=diag` (``inertia``)

Robot Dynamics
--------------

The fundamental Langrangian mechanics equation is

   :math:`B(q)\ddot{q}+C(q,\dot{q})+G(q) = \tau + \sum_i J_i(q)^T f_i`

Where:

-  :math:`q` is configuration,
-  :math:`\dot{q}` is velocity,
-  :math:`\ddot{q}` is acceleration,
-  :math:`B(q)` is the positive semidefinite *mass matrix*,
-  :math:`C(q,\dot{q})` is the *Coriolis force*,
-  :math:`G(q)` is the *generalized gravity*,
-  :math:`\tau` is the link torque,
-  :math:`f_i` are *external forces*, and
-  :math:`J_i(q)` are the Jacobians of the points at which the points are
   applied.

A robot's motion under given torques and external forces can be
computed by determining the acceleration :math:`\ddot{q}` by multiplying both sides
by :math:`B^{-1}` and integrating the equation forward in time.  This is known as
*forward dynamics*.

The reverse operation, determining a required right-hand-side (torques + forces)
to effect a given acceleration, is known as *inverse dynamics*

API summary
~~~~~~~~~~~

The ``RobotModel`` class can compute each of these items using the
Newton-Euler method:

-  ``robot.getCom()``: returns robot’s center of mass as a 3-tuple of
   floats
-  ``robot.getMassMatrix()``: returns nxn array (n nested length-n
   lists) describing the mass matrix *B* at the robot’s current
   ``Config``
-  ``robot.getMassMatrixInv()``: returns nxn array (n nested length-n
   lists) describing the inverse mass matrix *B-1* at the robot’s
   current ``Config``
-  ``robot.getCoriolisForceMatrix()``: returns nxn array (n nested
   length-n lists) describing the Coriolis force matrix such that at the
   robot’s current ``Config`` / ``Velocity``
-  ``robot.getCoriolisForces()``: returns the length-n list giving the
   Coriolis torques at the robot’s current ``Config`` / ``Velocity``
-  ``robot.getGravityForces(gravity)``: returns the length-n list giving
   the generalized gravity G at the robot’s current ``Config``, given
   the 3-tuple gravity vector (usually (0,0,-9.8))
-  ``robot.torquesFromAccel(ddq)``: solves for the inverse dynamics,
   returning the length-n list of joint torques that would produce the
   acceleration ddq at the current ``Config``/``Velocity``
-  ``robot.accelFromTorques(torques)``: solves for the forward dynamics,
   returning the length-n list of joint accelerations produced by the
   torques torques at the current ``Config``/``Velocity``

Using the Featherstone algorithm, forward and inverse dynamics take O(n) time.
Similarly, retrieving the COM, generalized gravity vector, and Coriolis force
vector are O(n).  The other methods are :math:`O(n^2)`, which is optimal.



Contact mechanics
-----------------

Klamp't supports several operations for working with contacts. Currently
these support legged locomotion more conveniently than object
manipulation, because the manipulated object must be defined as part of
the robot, and robot-object contact is considered self-contact.

API summary
~~~~~~~~~~~

These routines can be found in
`klampt.model.contact <klampt.model.contact.html>`__
which are thin wrappers around the underlying C++ functions.

-  :class:`ContactPoint`: either a frictionless or frictional point
   contact. Consist of a position, normal, and coefficient of friction.
   May also refer to which objects are in contact.
-  ``forceClosure`` tests whether a given set of contacts is in force
   closure.
-  ``comEquilibrium`` tests whether the center of mass of a rigid body
   can be stably supported against gravity by valid contact forces at
   the given contact list.
-  ``supportPolygon`` computes a support polygon for a given contact
   list. Testing the resulting boundaries of the support polygon is much
   faster than calling ``comEqulibrium`` multiple times.
-  ``equilibriumTorques`` solves for equilibrium of an articulated robot
   under gravity and torque constraints. It can handle both statically
   balanced and dynamically moving robots.

Holds, Stances, and Grasps
--------------------------

The contact state of a single link, or a related set of links, is
modeled with three higher-level concepts.

-  ``Hold``: a set of contacts of a link against the environment and are
   used for locomotion planning.
-  ``Stance``: a set of ``Hold``\ s.
-  ``Grasp``: not available in the Python API at the moment.

A :class:`~klampt.model.contact.Hold` is defined as a set of contacts
(the ``contacts`` member) and the associated IK constraint
(the ``ikConstraint`` member) that keeps a
link on the robot placed at those contacts. These contacts are
considered fixed in the world frame. ``Hold``\ s may be saved and loaded
from disk. 

A ``Stance`` defines all contact constraints
of a robot. It is defined simply as a list of ``Hold``\ s.

