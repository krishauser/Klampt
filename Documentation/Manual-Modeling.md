# Klamp't Manual: Kinematic modeling

Every physical object in Klamp't, whether real or imaginary, belongs to a World Model.  Specifically, a **[World Model](#world-models)** contains some number of entities, which can be of the types robot, robot link, rigid object, or terrain, as follows:
- **[Robots](#robot-models)** are articulated and possibly actuated objects containing multiple robot links
- **[Robot links](#robot-link-models)** are individual links of a robot
- **[Rigid objects](#rigid-object-models)** can rotate and translate, no independent actuation
- **[Terrains](#terrain-models)** do not move

Each robot link, rigid object, and terrain has an associated body, which has a coordinate frame and usually some attached **[Geometry](Manual-Geometry.md)** and **[Appearance](Manual-Geometry.md#appearance)**.

![World elements illustration](images/modeling-world.png)

## World Models

A World Model can also:
- be drawn in OpenGL
- Can be created dynamically or loaded from an XML file (examples in [Klampt/data/](../data))
- Load robots, rigid objects, and terrains from many formats (examples are in Klampt/data/robots, Klampt/data/objects, and Klampt/data/terrains)
- Can be copied quickly without copying geometries
- Can be saved to disk

Each entity in a world has
 - An index into the list of entities of that type
 - A unique ID# in the World
 - A name string, which is ideally unique.
 - Consists of one (or more) bodies.

Note that the entity's ID# is not the same as its index.  The index is the index into the array containing it; this is not unique when compared across entity types. For example, a robot and a terrain could have index 0 (the first robot and the first terrain) but they will have different ID#s.

If names are not unique, entities must be addressed by index. Furthermore, some modules like `klampt.model.coordinate` assume names are unique; if not, unexpected behavior may result.



### C++ API

See the RobotWorld class (Klampt/Modeling/World.h)

### Python API

A world is implemented in the WorldModel class.

The key functions are:
- ```world = WorldModel()```: constructor
- ```world.readFile([world file])```: reads from XML doc
- ```world.loadElement([robot, object or terrain file])```: adds an element to the World. Accepts .rob, .urdf, .obj, .env, or geometry files
- ```world.num[Robots,RigidObjects,Terrains]()```: returns the number of elements of the given type
- ``` world.[robot,rigidObject,terrain](index)```: returns a reference to the index'th element of the given type
- ```world.[robot,rigidObject,terrain](name)```: returns a reference to the element of the given type with name name (name must be a str)
- ```world.numIDs()```: returns the number of elements. Unique IDs run from 0 to ```numIDs()```-1.
- ```world.geometry(id)```: returns the Geometry3D of the element with the given unique ID
- ```world.appearance(id)```: returns the Appearance of the element with the given unique ID
- ```world.drawGL()```: renders the world in OpenGL

## Robot Models

A Robot Model defines the kinematic structure of a robot, and more. Robot Models provide the following functions

- Describes a list of links with their parents (an open linkage, specified in topologically sorted order)
- Stores kinematic characteristics: link lengths, joint axis types, joint stops, inertial characteristics, and link geometry.
- Stores actuation limits
- Stores a &quot;current&quot; robot configuration and velocity. _Note: these should be thought of as temporary variables, see notes below._
- Computes and stores the robot's &quot;current&quot; link frames via forward kinematics.
- Computes the robot's Lagrangian dynamics terms.
- Stores link collision geometries and performs collision detection.
- Stores information about which links can self-collide.
- Names each link and contains semantics of the how the degrees of freedom of the robot map to &quot;joints&quot; and actuators.
- Loads and saves robot descriptions from disk.

For now, we will discuss only the kinematics of a Robot Model, saving [discussion of dynamics](Manual-Dynamics.md) for later.  The basic kinematic components are:
- Contains N<sub>L</sub> degrees of freedom (DOFs) and N<sub>L</sub> Robot Links.
- Each DOF has a numeric value describing its position.  The list of all DOFs is the robot's Configuration.
- Each DOF may also have a velocity.  The set of all velocities is the robot's Velocity.
- Each DOF has joint limits, velocity limits, acceleration limits, and torque limits.

*IMPORTANT*: The configuration / velocity of a Robot Model do not directly correspond with those of a physical / simulated robot. They do not even have to respect the joint/velocity limits. Think of them as temporary variables to help you perform calculations. (Sending commands to a physical / simulated robot will be [covered elsewhere](Manual-Control.md)!)

Klamp't works with arbitrary tree-structured articulated robots. Parallel mechanisms are not directly supported.  

### C++ API

The Robot Model in the C++ API is implemented by the `Robot` class in [Klampt/Modeling/Robot.h](../Modeling/Robot.h).  This structure is based heavily on the KrisLibrary/robotics package for defining articulated robot kinematics and dynamics. `Robot` has the following class hierarchy:

<centering> `Robot` -&gt; `RobotWithGeometry` -&gt; `RobotDynamics3D` -&gt; `RobotKinematics3D` -&gt; `Chain` </centering>

The reasons for the class hierarchy are largely historical, but meaningful. For example, a protein backbone might be modeled as a `RobotKinematics3D` but not a `RobotDynamics3D`.

- `Chain` stores the topological sorting of the articulation (the parents member).
- `RobotKinematics3D` stores the kinematic and dynamic information of links, joint limits, the current configuration and the current link frames.  It also provides methods for computing forward kinematics, jacobians, and the center of mass.
- `RobotDynamics3D` stores the actuator limits and the current velocity. It provides methods for computing information related to the robot's dynamics.
- `RobotGeometry3D` stores link collision geometries and information about which links can self collide.  It performs self-collision testing and collision testing with other geometries.
- `Robot` defines link names and semantics of `Joints` and `Drivers`.

A `Robot` configuration is defined by a `Config` class (which is simply a typedef for `Vector`, see KrisLibrary/math/vector.h). The robot model's configuration is described in `Robot.q`, and the current transforms of its links are given in `Robot.links[index].T_World`. To ensure consistency between the configuration and the link frames, the `Robot.UpdateConfig(q)` method should be called to change the robot's configuration. `UpdateConfig` performs forward kinematics to compute the link frames, while simple assignment of the form `Robot.q=q` does not.



### Python API

The following lsits the key kinematic operations of RobotModel class.   

- ```robot = world.robot([name or index])```: access RobotModel reference by name or index
- ```robot.getID()```:  gets the unique ID of the RobotModel in its WorldModel
- ```robot.num[Links/Drivers]()```: returns the number of links or drivers (joints are abstracted away in the Python API)
- ```robot.get[Link/Driver](index)```: returns a reference to the index'th RobotModelLink / RobotModelDriver
- ```robot.get[Link/Driver](name)```: returns a reference to the RobotModelLink / RobotModelDriver with the given name (name must have type str)
- ```robot.get[Config/Velocity]()```: returns the model's Configuration/Velocity as a list of floats with length ```numLinks()```
- ```robot.setConfig(q)```: sets the model's Configuration to q given as a list/tuple of floats with length ```numLinks()```. Also, updates the forward kinematics of all Robot Links.
- ```robot.setVelocity(dq)```: sets the model's Velocity to dq given as a list/tuple of floats with length ```numLinks()```
- ```robot.get[Joint/Velocity/Acceleration]Limits()```: returns minimum/maximum of model's Configuration/Velocity/acceleration as a pair of lists of floats with length ```numLinks()```
- ```robot.set[Joint/Velocity/Acceleration]Limits(vmin,vmax)```: sets minimum/maximum Configuration/Velocity/acceleration given two lists of floats with length ```numLinks()```
- ```robot.drawGL()```: renders the robot in OpenGL

The configuration and velocity of a robot are a list of floats.  A `Config` object is simply a list of floating point numbers, and the robot model's configuration is retrieved / set using `RobotModel.setConfig(q)`/ `RobotModel.getConfig()`. Note: unlike the C++ API, upon calling `setConfig()` the link transforms and geometries are automatically updated using forward kinematics.


## Robot Link Models

A Robot Link corresponds to one of the robot model's DOFs.
- It holds all physical information about the link in its reference coordinate frame (name, index, parent, reference transform to parent, type of joint, joint axis, mass)
- It contains a collision geometry, which may be empty.  This model is specified relative to the link's coordinate system. 
- It also holds information regarding its current transformation given the robot model's current configuration.
- It also helps you calculate Jacobians, velocities, etc

The parent index of each link must be less than the link's index (topologically sorted order). A parent of -1 indicates that the link is attached to the world coordinate frame.  Each link may be _prismatic_ or _revolute_ and moves along or around a link axis given by a 3D vector. 

The current transformation of a link is calculated via forward kinematics, and describes its coordinates (position and orientation) _relative to the world frame_. For example, in the image shown below, the transforms of each link are shown with red axis indicating the local x direction, green axis indicating y, and blue axis indicating z.  The highlighted link is drawn with longer axes.

![Link illustration](images/modeling-link-frame.png)

### C++ API

Links are stored in the `Robot.links` member, which is an array of `RobotLink3D`s. The parent index of each link is stored in the `Robot.parents` member, which is a list of `int`s. The link type is stored in `RobotLink3D.type` and its axis is stored in `RobotLink3D.w`. `RobotLink3D` also contains mass parameters (`mass`, `inertia`, `com`), the reference transformation to its parent (`T0_Parent`), and the link's current transformation `T_World`.

Link geometries are stored in the `Robot.geometry` list, but to take advantage of the cache the `Robot.geomManagers` variable should be used for saving/loading/modifying the geometry. _Note: The transform of each collision geometry is only updated to the current link transform after `robot.UpdateGeometry()` is called_.


### Python API

The link functionality is given in the `RobotModelLink`. Changing from revolute to prismatic types is not supported at the moment.  Note: unlike the C++ API, geometry current transforms are updated automatically after `RobotModel.setConfig(q)`.

Configuration-independent functions that define the kinematic structure of the robot:
- ```link = robot.link([name or index])```: access RobotModelLink reference
- ```link.getID()```:  gets the unique ID of this link in the WorldModel
- ```link.getName()```: returns a string naming this link
- ```link.getRobot()```: returns a RobotModel to which this link belongs
- ```link.getIndex()```: returns the link index on the RobotModel for this link
- ```link.getParent()```: returns the index of the link's parent (-1 for no parent)
- ```link.setParent(p)```: sets the index of the link's parent (-1 for no parent)
- ```link.getAxis()```: returns a 3-tuple of the link's rotation/translation axis in local coordinates
- ```link.setAxis(axis)```: sets the link's rotation/translation axis to the given 3-tuple, specifying its local coordinates
- ```link.getParentTransform()```: returns a pair (R,t) defining the reference coordinate transform of this link with respect to its parent (see klampt.so3 for the format of R)
- ```link.setParentTransform(R,t)```: sets the reference coordinate transform of this link with respect to its parent (see klampt.so3 for the format of R)
- ```link.geometry()```: returns a reference to the Geometry3D attached to this link
- ```link.appearance()```: returns a reference to the Appearance attached to this link
- ```link.getMass()```: returns the link's Mass structure
- ```link.setMass(mass)```: sets the link's Mass structure
- ```link.drawLocalGL()```: renders the link's geometry in OpenGL in local coordinates

Configuration-dependent functions that describe the physical layout of the robot in its current configuration:
- ```link.getTransform()```: returns a pair (R,t) defining the coordinate transform of this link with respect to the world frame (see klampt.so3 for the format of R)
- ```link.setTransform(R,t)```: sets the coordinate transform of this link with respect to the world frame. Note```: this does NOT perform inverse kinematics or change the transforms of any other links. The transform is overwritten when the robot's setConfig() method is called. (see klampt.so3 for the format of R)
- ```link.getWorldPosition(pLocal)```: given a 3-tuple specifying the local coordinates of a point P, returns a 3-tuple giving the world coordinates of P.  Equivalent to se3.apply(- ```link.getTransform(),pLocal).
- ```link.getLocalPosition(pWorld)```: given a 3-tuple specifying the local coordinates of a point P, returns a 3-tuple giving the world coordinates of P.  Equivalent to se3.apply(se3.inv(link.getTransform()),pWorld).
- ```link.getWorldDirection(dLocal)```: given a 3-tuple specifying the local coordinates of a direction D, returns a 3-tuple giving the world coordinates of D.  Equivalent to so3.- ```apply(link.getTransform()[0],dLocal).
- ```link.getLocalDirection(dWorld)```: given a 3-tuple specifying the local coordinates of a point P, returns a 3-tuple giving the world coordinates of P.  Equivalent to so3.apply(so3.inv(link.getTransform()[0]),dWorld).
- ```link.getOrientationJacobian()```: returns a 3xNL matrix of the orientation Jacobian of this link
- ```link.getPositionJacobian()```: given a 3-tuple specifying the local coordinates of a point P, returns a 3xNL matrix of the position Jacobian of this point
- ```link.getJacobian(p)```: given a 3-tuple specifying the local coordinates of a point P, returns a 6xNL matrix of the orientation Jacobian stacked on the position Jacobian
- ```link.drawWorldGL()```: renders the link's geometry in OpenGL in world coordinates




## Kinematics Example

Load a planar 3R robot model and figure out where its end effector would lie at the configuration [0,pi/4,pi/4].

```python
>>> from klampt import *
>>> import math
>>> world = WorldModel()
>>> world.loadElement("data/robots/planar3R.rob")
Reading robot file robots/planar3R.rob...
   Parsing robot file, 3 links read...
Loaded geometries in time 0.000598229s, 36 total primitive elements
Initialized robot collision data structures in time 0.00084037s
Done loading robot file robots/planar3R.rob.
0
>>> robot= world.robot(0)
>>> robot.setConfig([0,math.pi/4,math.pi/4])
>>> link = robot.link(2)
>>> link.getWorldPosition([1,0,0])
[1.7071067811865477, 0.0, 1.7071067811865475]
```

To get the Jacobian matrix corresponding to this configuration, all that is needed is

```python
>>> link.getPositionJacobian([1,0,0])
[[-1.7071067811865475, -1.7071067811865475, -1.0], [0.0, 0.0, 0.0], [1.7071067811865477, 0.7071067811865477, 1.5701957963021318e-16]]
```

(The second row is all zeroes because the robot moves in the X-Z plane.)

## Robot Joints and Drivers

The DOFs of a robot are considered as generic variables that define the extents of the articulations between links.  However, more information about how sets of DOFs group together and map to motors also available in the Robot Model. A Robot Model's DOFs are organized into N<sub>J</sub> &le; N<sub>L</sub> _Joints_, and N<sub>D</sub> &le; N<sub>L</sub> _Drivers_
- `Joint`: topology of one or more DOFs.  Joints can be normal, welded, freely rotating, or free floating.  Joints specify how to handle interpolation, and the distance between configurations.
- `Driver`: mapping from actuator controls to movement of one or more DOFs.  They specify how actuator commands are transmitted to forces / torques on links (e.g., the transmission).
- For fixed-base, fully actuated robots, each DOF has a single joint and a single driver: N<sub>L</sub>=N<sub>J</sub>=N<sub>D</sub>

**Joints.** Most Joints will be of the `Normal` type, which map directly to a single DOF in the normal way. However, free-floating bases and other special types of Joints designate groups of DOFs that should be interpreted in special ways. These special Joints include:

- `Weld` joints, which indicate that a DOF should not move.
- `Spin` joints, which are able to rotate freely and infinitely.
- `Floating` joints, which translate and rotate freely in 3D (e.g., free-floating bases)
- `FloatingPlanar` joints, which translate and rotate freely in 2D (e.g., mobile wheeled bases)
- `BallAndSocket` joints, which rotate freely in 3D.
- `Closed` joints, which indicate a closed kinematic loop. _Note: this is simply a placeholder for potential future capabilities; these are not yet handled in Klamp't._


In C++ the joint type is referred to in CamelCase, while in files and in python the joint type is referred to in lowercase.

**Drivers.** Although many robots are driven by motors that transmit torques directly to single DOFs, the `Robot` class can represent other drive systems that apply forces to multiple DOFs. For example, a cable-driven finger may have a single cable actuating three links, a mobile base may only be able to move forward and turn, and a satellite may have thrusters. Free-floating bases may have no drive systems whatsoever.

A robot is set up with a list of Drivers available to produce torques.
- `Normal` drivers act as one would expect a motor that drives a single DOF to behave.
- `Affine` drivers act as connected transmissions with linear relationships between multiple DOF (such as certain cable drives or gear linkages).
- The other driver types are not fully tested and/or supported, although we hope to add some of this functionality in the future.

### Floating bases and virtual links

To represent free-floating bases, you should use a set of 5 massless _virtual links_ and 1 physical link that represent the x, y, and z translations and rotations around the z, y, and x axes (roll-pitch-yaw convention). Likewise, a mobile robot may be represented by 2 virtual links + 1 physical link: two for x, y translations connected by prismatic joints, and the last for &theta;, connected to its parent by a revolute joint. A ball-and-socket joint may be represented by 2 virtual links + 1 physical link.

**C++ API**. See `RobotKinematics3D.InitializeRigidObject` for an example of how to set up a floating base.

**Python API**. See `klampt.model.floatingbase.py` for utility functions for setting up a floating base.


## Rigid Object Models


A Rigid Object Model is a Collision Geometry associated with a `RigidTransform` and other dynamic parameters. `Rigid Object`s may be loaded from `.obj` files or raw geometry files. In the latter case, the dynamic parameters are set to default values (e.g., `mass = 1`).

### C++ API
See Klampt/Modeling/RigidObject.h.

### Python API
See the RigidObjectModel class.


## Terrain Models

A Terrain Model  is defined very simply as a Collision Geometry annotated with friction coefficients. They may be loaded from .env files or raw geometry files. In the latter case, some default friction value is assigned (set to 0.5).

### C++ API
See Klampt/Modeling/Terrain.h.

### Python API
See the TerrainModel class.

