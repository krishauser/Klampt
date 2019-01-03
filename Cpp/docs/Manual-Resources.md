# Resource management

When building a large, complex robot system, you will typically encounter dozens or hundreds of robot configurations, transforms, inverse kinematics constraints, and paths.  Klamp't has many functions to help make managing these elements easier.

_Visual editing_ is a sort of what-you-see-is-what-you-get (WYSIWYG) editing approach for robotics, and Klamp't tries to make visual editing part of the robot hacking workflow in as painless a manner as possible.

This is made more convenient through the Klamp't resource management mechanism. When working on a large project, it is recommended that configurations, paths, holds, etc. be stored in dedicated sub-project folders to avoid polluting the main Klamp't folder. Resources are compatible with the RobotPose app, as well as the C++ and Python APIs.


_Python API_. The klampt.io.resource module allows you to easily load, save, or edit resources. Visual editing is supported for Config, Configs, Vector, and RigidTransform types.  As an example, the below code loads an xform resource visually edits

> from klampt.io import resource
> from klampt.math import se3
> resource.get("test.xform",type="RigidTransform",default=se3.identity())

See the `Klampt-examples/Python/demos/resourcetest.py` demo for more examples about how to use this module.

Currently supported types include:

- Config (.config)
- Configs (.configs): A configuration list
- Trajectory (.path): A piecewise linear trajectory object
- MultiPath (.xml): See klampt.model.multipath
- Hold (.hold)
- Stance (.stance)
- Grasp (.xml)
- GeometricPrimitive (.geom)
- TriMesh (.off, .tri, etc.)
- PointCloud (.pcd)
- Robot (.rob)
- RigidObject (.obj)
- World (.xml)

Klamp't also supports the following additional types which do not have a dedicated file extension:

- Vector3
- Matrix3
- RigidTransform
- Matrix
- IKGoal


_C++ API_. The Klampt/Modeling/Resources.h file lists all available resource types. Note that a sub-project folder can be loaded all at once through the ResourceLibrary class (KrisLibrary/utils/ResourceLibrary.h). After initializing a ResourceLibrary instance with the MakeRobotResourceLibrary function in (Klampt/Modeling/Resources.h) to make it Klamp't-aware, the LoadAll/SaveAll() methods can load an entire folder of resources. These resources can be accessed by name or type using the Get\*() methods.

Alternatively, resource libraries can be saved to XML files via the LoadXml/SaveXml() methods. This mechanism may be useful in the future, for example to send complex robot data across a network.


## Visual editing and visualization support

The resource types that can be visually edited include 

| Type            | `vis.add` | `vis.edit` | `resource.get` / `resource.edit` | 
| --------------- | --------- | ---------- | -------------------------------- |
| Vector3         | Y         | Y          | Y                                |
| Matrix3         | Y         | Y          | Y                                |
| RigidTransform  | Y         | Y          | Y                                |
| Config          | Y         | Y          | Y                                |
| Configs         | Y         | Y          | Y                                |
| Trajectory      | Y         | Y          | Y                                |
| MultiPath       | Y         | N          | N                                |
| Hold            | Y         | N          | N                                |
| Stance          | Y         | N          | N                                |
| Grasp           | Y         | N          | N                                |
| Geometry3D      | Y         | N          | N                                |
| Robot           | Y         | Y (config) | N                                |
| RigidObject     | Y         | Y (transform) | N                             |
| World           | Y         | N           | Y                               |
| Anything in the coordinates module | Y | N | N                              |


The object types with a Y in the first column can be visually inspected using `vis.add(name,object)`.  In the second column, you can edit them using `vis.edit(name)`


Calling `resource.get` or `resource.edit` on a supported object will use the corresponding classes in vis.editors.  In addition, you can also use the following editors:

- ObjectTransformEditor: edit RigidObject transforms 
- SelectionEditor: edit subsets of a RobotModel

As an example, you can run

> from klampt import *
> from klampt.vis import editors
> world = WorldModel()
> ...setup world
> links = editors.run(editors.SelectionEditor("active_ik_links",value=[6,7,8,9,10,11],description="Robot arm links for IK",world=world))





## Resource browsing app

The Klamp't Python API comes with a script to quickly browse through resources, and even edit them. Running `klampt_browser` will allow you to browse through directories and select multiple objects and resources to display. 

`klampt_browser` stores a "reference world", which is by default empty.  You can select worlds, robots, rigid objects, and meshes and then add to the reference world using the "Add to world" button, or you can specify elements of the world as command line elements:

> klampt_browser Klampt-examples/data/athlete_plane.xml

Once you have a reference world, selecting resources like Config, Configs, Trajectory, and IKGoal will show up in context of the first robot in the reference world.

For selected resources of any type that can be edited in the vis module (see list above), you can click the "Edit" button and then save the edited value to disk.



## RobotPose

The RobotPose app can also browse and edit resources.  TODO: describe in more detail.

