# Klamp't Manual: I/O

Klamp't supports many types of I/O formats:
- Custom file formats (old-style): Klamp't defines several formats for its custom types.  Compared to the JSON custom formats, these are older but are more compatible with C++ backend, especially the RobotPose app.
- Custom file formats (JSON): We are beginning to use a newer JSON format that will be supported more heavily in the future. 
- URDF files: Klamp't natively supports ROS' Universal Robot Description Format.
- Geometry files: Klamp't natively supports OFF (Object File Format), OBJ (Wavefront OBJ), and PCD (Point Cloud Data) file formats.  If it is built with Assimp support, then any file format that Assimp reads can also be read as a mesh.
- ROS: Klamp't can publish and subscribe to several ROS message types.
- Three.js: Klamp't can export Three.js scenes.
- MPEG: Klamp't's SimTest app and Python vis module can export MPEG videos of simulations, if ffmpeg is installed.
- HTML: Klamp't's Python vis module can export HTML files for simulation playback.



## Custom file formats in Klamp't

| Object type    | Extension |
| ---------------|-----------|
| World model    |  .xml     |
| Robot model    |  .rob     |
| Rigid object model  | .obj |
| Terrain model  | .env      |
| Point          | .vector3  |
| Rigid transform| .xform    |
| Geometric primitive | .geom|
| Configuration  |  .config  |
| Configuration list | .configs |
| IK Objective   |  .ikgoal  |
| Piecewise linear trajectory | .path |
| Multipath      | .xml      |
| Hold           |  .hold    |
| Stance         |  .stance  |

These are described in more detail in the [File Types section](Manual-FileTypes.md).

### C++ I/O functions

TODO

## Python I/O functions

To load/save general objects, there are are two modules to use: [io.loader](http://motion.pratt.duke.edu/klampt/pyklampt_docs/loader_8py.html) and [io.resource](http://motion.pratt.duke.edu/klampt/pyklampt_docs/resource_8py.html).  The resource module is nice to use because it performs automatic identification of types, allows file browsing, and will launch a visual editor if an object does not exist.

As an example with an IK Objective:

```python
obj = ik.objective(...) #set up objective

#method 1: using the resource module
from klampt.io import resource
resource.set("test_objective.ikgoal",obj)     # Saves obj to disk in a file compatible with robotPose
obj2 = resource.get("test_objective.ikgoal")  # Loads the same item from disk 
#Note: can also use .json extension, and this will automatically be converted to JSON
#resource.set("test_objective.json",obj) 
#obj2 = resource.get("test_objective.json") 
#Note: can also launch a file browser
#resource.save(obj,[optional directory])
#resource.load("IKObjective",[optional directory])

#method 2: serializing and deserializing in the old format
from klampt.io import loader
s = loader.writeIKObjective(obj)  #converts to a string representation compatible with RobotPose
print s
obj2 = loader.readIKObjective(s)  #reads from a string compatible with RobotPose

#method 3: saving and loading in the old format
loader.save(obj,"IKObjective","test_ik_objective.ikgoal")  #saves to a file on disk compatible with RobotPose
obj2 = loader.load("IKObjective","test_ik_objective.ikgoal")  #reads from the file on disk compatible with RobotPose

#method 4: serializing and deserializing in the JSON format
import json
jsonobj = loader.toJson(obj)  #converts to a data structure compatible with JSON I/O routines
s = json.dumps(jsonobj) #converts to a JSON string
print s
jsonobj2 = json.parse(s)  #converts from a JSON string
obj2 = loader.fromJson(jsonobj2)  #converts from a JSON-compatible data structure

#method 5: saving and loading in the JSON format
jsonobj = loader.toJson(obj)  #converts to a data structure compatible with JSON I/O routines
with open("test_ik_objective.json",'w') as f:
    json.dump(f,jsonobj)  #saves to a JSON file on disk
with open("test_ik_objective.json",'r') as f:
    jsonobj2 = json.load(f)
obj2 = loader.fromJson(jsonobj2)  #converts from a JSON-compatible data structure
```

### World loading and saving

To load items into a WorldModel, use the readFile and loadElement methods.  For the most part, these will automatically figure out the type of the loaded object.  Geometry files are converted into static terrains.  To dynamically create worlds, you can call world.makeRobot/makeRigidObject/makeTerrain(name) and then call element.loadFile(fn) on the resulting element.

To save a WorldModel, you can use the writeFile() method. This will dump all elements contained in the world into a folder of the same name as the file, but without the .xml extension.  Here, the paths of geometry files will be preserved, unless the geometry has been modified.  To save individual elements, you can use the element.saveFile(fn) method.  


## Custom file formats

### World (.xml) files

### Robot (.rob and .urdf) files

Robots are loaded from Klamp't-specific .rob files or more widely-used URDF files.  These are simple text files that are editable by hand.
Although URDF is more commonly used in the robotics field, there are some convenient aspects of .rob files that may be useful. For example, the `mount` command allows robot grippers and other attachments to be added automatically at load-time.  This is annoying to do with URDF, requiring a separate command line step with the xacro tool.

The basic URDF file format does not specify some aspects of Klamp't robots. These can be added under the `<klampt>` XML tag. See the [file format documentation](Manual-FileTypes.md) or the Klampt [import robot tutorial](http://motion.pratt.duke.edu/klampt/tutorial_import_robot.html) for more details.

For simulation purposes, Klamp't will need some motor parameters to be tweaked (`servoP`, `servoI`, `servoD`, `dryFriction`, `viscousFriction`). This can be done by hand by tuning and &quot;exercising&quot; the robot in simulation. The Driver window in SimTest can be used for this purpose.  An automatic solution is given by the `MotorCalibrate` program, which will optimize the constants to match a dataset of sensed and commanded joint angles that you record while exercising the physical robot.  See [the apps documentation](Manual-Apps#motorcalibrate) for more details.

The [URDFtoRob](Manual-Apps#urdftorob) program converts from .urdf to .rob files. Geometric primitive link geometries will be converted to triangle meshes.


### Rigid object (.obj) files

### Robot configuration (.config) files

### Piecewise-linear path (.path) files



## ROS Communication

If you compile with ROS installed on your system, Klamp't will support many ROS types, including Pose, PoseStamped, WrenchStamped, Float32MultiArray, JointState, PointCloud2, Image, CameraInfo, and JointTrajectory.

TODO: describe in more detail