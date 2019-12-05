# Klamp't Manual: I/O

* [Custom file formats in Klamp't](#custom-file-formats-in-klamp-t)
    + [API summary](#api-summary)
    + [Robot (.rob and .urdf) loading and saving](#robot--rob-and-urdf--loading-and-saving)
* [ROS Communication](#ros-communication)

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

### API summary

TODO

### Robot (.rob and .urdf) loading and saving

Robots are loaded from Klamp't-specific .rob files or more widely-used URDF files.  These are simple text files that are editable by hand.
Although URDF is more commonly used in the robotics field, there are some convenient aspects of .rob files that may be useful. For example, the `mount` command allows robot grippers and other attachments to be added automatically at load-time.  This is annoying to do with URDF, requiring a separate command line step with the xacro tool.

The basic URDF file format does not specify some aspects of Klamp't robots. These can be added under the `<klampt>` XML tag. See the [file format documentation](Manual-FileTypes.md) or the Klampt [import robot tutorial](https://github.com/krishauser/Klampt/blob/master/Cpp/docs/Tutorials/Import-and-calibrate-urdf.md) for more details.

For simulation purposes, Klamp't will need some motor parameters to be tweaked (`servoP`, `servoI`, `servoD`, `dryFriction`, `viscousFriction`). This can be done by hand by tuning and &quot;exercising&quot; the robot in simulation. The Driver window in SimTest can be used for this purpose.  An automatic solution is given by the `MotorCalibrate` program, which will optimize the constants to match a dataset of sensed and commanded joint angles that you record while exercising the physical robot.  See [the apps documentation](Manual-Apps#motorcalibrate) for more details.

The [URDFtoRob](Manual-Apps.md#urdftorob) program converts from .urdf to .rob files. Geometric primitive link geometries will be converted to triangle meshes.



## ROS Communication

If you compile with ROS installed on your system, Klamp't will support many ROS types, including Pose, PoseStamped, WrenchStamped, Float32MultiArray, JointState, PointCloud2, Image, CameraInfo, and JointTrajectory.

TODO: describe in more detail