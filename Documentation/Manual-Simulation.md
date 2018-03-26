# Klamp't Manual: Simulation

Simulation functionality in Klamp't is built on top of the Open Dynamics Engine (ODE) rigid body simulation package, but adds emulators for robot sensors and actuators, and features a robust contact handling mechanism. When designing new robots and scenarios, it is important to understand a few details about how Klamp't works in order to achieve realistic simulations.

## Boundary-layer contact detection.

Other rigid body simulators tend to suffer from significant collision handling artifacts during mesh-mesh collision: objects will jitter rapidly, interpenetrate, or react to &quot;phantom&quot; collisions. The primary cause is that contact points, normals, and penetration depths are estimated incorrectly or inconsistently from step-to-step. Klamp't uses a new _boundary layer contact detection_ procedure that leads to accurate and consistent estimation of contact regions. Moreover, the boundary layer can simulate some limited compliance in the contact interface, such as soft rubber coatings or soft ground.

In Klamp't, contact is detected along the boundary layers rather than the underlying mesh. The thickness of the boundary layer is a simulation parameter called _padding_. Padding for each body can be set via the padding attribute in the&lt;simulation&gt;{&lt;robot&gt;,&lt;object&gt;,&lt;terrain&gt;}&lt;geometry&gt; XML element, with all bodies padded with 2.5mm by default. This allows it to handle thin-shell meshes as illustrated in the following figures.

![Simulation boundary layer 1](images/simulation-contact.png)
![Simulation boundary layer 2](images/simulation-contact2.png)
![Simulation boundary layer 3](images/simulation-contact3.png)
![Simulation boundary layer 4](images/simulation-contact4.png)
 
The first step of Klamp't's collision handling routine is to compute all contacts between all pairs of geometric primitives within the padding range. This is somewhat slow when fine meshes are in contact. In order to reduce the number of contacts that must be handled by ODE, Klamp't then performs a clustering step to reduce the number of contacts to a manageable number. The maximum number of contacts between two pairs of bodies is given by the _maxContacts_ global parameter, which can be set as an attribute in the XML &lt;simulation&gt; tag.

For more details, please see: _K. Hauser. Robust Contact Generation for Robot Simulation with Unstructured Meshes. In proceedings of International Symposium of Robotics Research, 2013._

## Collision response.

In addition to padding, each body also has coefficients of restitution, friction, stiffness, and damping (kRestitution, kFriction, kStiffness, and kDamping attributes in &lt;simulation&gt;{&lt;robot&gt;,&lt;object&gt;,&lt;terrain&gt;}&lt;geometry&gt; XML elements). The stiffness and damping coefficients can be set to non-infinite values to simulate softness in the boundary layer. When two bodies come into contact, their coefficients are blended using arithmetic mean for kRestitution, and harmonic means for kFriction, kStiffness, and kDamping.

The blending mechanism is convenient because only one set of parameters needs to be set for each body, rather than each pair of bodies, and is a reasonable approximation of most material types. Currently there is no functionality to specify custom properties between pairs of bodies.


## Actuator simulation
Klamp't handles actuators in one of two modes: PID control and torque control modes. It also simulates dry friction (stiction) and viscous friction (velocity-dependent friction) in joints using the dryFriction and viscousFriction parameters in .rob files. Actuator commands are converted to torques (if in PID mode), capped to torque limits, and then applied directly to the links. ODE then handles the friction terms.

In PID mode, the torque applied by the actuator is 

<center> &tau;=k<sub>P</sub>(&theta;<sub>D</sub> - &theta;<sub>A</sub>)+k<sub>D</sub>(&theta;'<sub>D</sub> - &theta;'<sub>A</sub>)+k<sub>I</sub>I </center>

where
- k<sub>P</sub>, k<sub>I</sub>, and k<sub>D</sub> are the PID constants,
- &theta;<sub>D</sub>  and &theta;'<sub>D</sub> are the desired position and velocity,
- &theta;<sub>A</sub>  and &theta;'<sub>A</sub> are the actual position and velocity,
- and I is an integral error term.

The friction forces resist the motion of the joint, and Klamp't uses a simple stick-slip friction model where the sticking mode breaking force is equal to &mu;<sub>D</sub>
 and the sliding mode friction force is

<center>-sgn(&theta;'<sub>A</sub>)(&mu;<sub>D</sub>+&mu;<sub>V</sub>|&theta;'<sub>A</sub>|). </center>

where &mu;<sub>V</sub> is the viscous friction force.  _Note: passive damping should be handled via the friction terms rather than the PID gain k<sub>D</sub>_.

Like all simulators, Klamp't does not perfectly simulate all of the physical phenomena affecting real robots. Some common phenomena include:

- Backlash in the gears.
- Back EMF.
- Angle-dependent torques in cable drives.
- Motor-induced inertial effects, which are significant particularly for highly geared motors. Can be approximated by adding a new motor link connected by an affine driver to its respective link.
- Velocity-dependent torque limits (e.g. power limits). Can be approximated in a controller by editing the robot's driver torque limits depending on velocity. Can be correctly implemented by adding a WorldSimulationHook or editing the ControlledRobotSimulator class.
- Motor overheating. Can be implemented manually by simulating heat production/dissipation as a differential equation dependent on actuator torques. May be implemented in a WorldSimulationHook.


## Python API

To create and manage a simulation:

- `sim = Simulator(world): creates a simulator for a given `WorldModel` (note: cannot modify the number of entities in the world at this point, undefined behavior will occur if you do!)
- `sim.getWorld()`: retrieves the simulation’s WorldModel
- `sim.updateWorld()`: updates the WorldModel to reflect the current state of the simulator
- `sim.simulate(dt)`: advances the simulation by time dt (in seconds)
- `sim.fakeSimulate(dt)`: fake-simulates.  Useful for fast prototyping of controllers
- `sim.getTime()`: returns the accumulated simulation time
- `sim.getState()`: returns a string encoding the simulation state
- `sim.setState(state)`: sets the simulation state given the result from a previous `getState()` call
- `sim.reset()`: reverts the simulation back to the initial state
- `sim.setGravity(g)`: sets the gravity to the 3-tuple `g` (default (0,0,-9.8))
- `sim.setSimStep(dt)`: sets the internal simulation time step to `dt`. If `simulate()` is called with a larger value dt', then the simulation will integrate physics forward over several substeps of length at most `dt`

To modify the properties of simulated rigid bodies: [NOTE: reference frame is centered at center of mass]
- `body = sim.body([RobotLinkModel or RigidObjectModel])`: retrieves the simulated body according to a link or rigid object.
- `body.getID()`: retrieves integer ID of associated object in world
- `body.enable(enabled=True)/isEnabled()`: pass `False` to disable simulation of the body
- `body.enableDynamics(enabled=True)/isDynamicsEnabled()`: pass `False` to drive a body kinematically along a given path
- `body.getTransform()/setTransform(R,t)`: gets/sets SE(3) element representing transform of body coordinates w.r.t. world
- `body.getVelocity()/setVelocity(w,v)`: gets/sets the angular velocity w and translational velocity v of the body coordinates w.r.t. world
- `body.getSurface()/setSurface(SurfaceParameters)`: gets/sets the body’s surface parameters
- `body.getCollisionPadding()/setCollisionPadding(m)`: gets/sets the body’s collision margin (nonzero yields more robust collision handling)
- `body.applyForceAtPoint(fw,pw)`, applyForceAtLocalPoint(fw,pl): adds a world-space force fw to a point, either pw in world coordinates or pl in body coordinates.  Applied over duration of next Simulator.simulate() call
- `body.applyWrench(f,t)`: adds a force f at COM and torque t over the duration of te next Simulator.simulate() call

To inspect the contact status of objects:
- `sim.enableContactFeedbackAll()`: turns on contact feedback for all objects
- `sim.enableContactFeedback(id1,id2)`: turns on contact feedback for contacts between objects with ids id1 and id2
- `sim.inContact/hadContact(id1,id2)`: returns `True` if objects id1 and id2 are in contact at the end of the time step / had contact during the prior time step
- `sim.hadPenetration/hadSeparation(id1,id2)`: returns True if objects id1 and id2 penetrated / were separated at any point during the prior time step
- `sim.getContacts(id1,id2)`: returns a list of contacts between id1 and id2 on the current time step.  Each contact is a 7-list `[px,py,pz,nx,ny,nz,kFriction]`
- `sim.getContactForces(id1,id2)`: returns a list of contact forces, one for each of the contacts in `sim.getContacts(id1,id2)`
- `sim.contactForce/contactTorque(id1,id2)`: returns the contact force / torque at the end of last time step
- `Sim.meanContactForce(id1,id2)`: returns the mean contact force over the entire last time step
- `from model import contact; contact.simContactMap(sim)`: returns a map from (id1,id2) pairs to `contact.ContactPoint` objects.
