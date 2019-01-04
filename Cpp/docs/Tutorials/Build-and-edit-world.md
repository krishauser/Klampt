
# Klamp't Tutorial: Build and edit a world file

In this tutorial we will learn the first step in setting up a plan or simulation: creating and modifying a world.  You will need to download the [Klampt-examples](https://github.com/krishauser/Klampt-examples) repository for this to work.

Klampt allows you to create a world in 2 ways:
* Create a static XML file
* Programmatic creation.

They can be used at the same time as well. For a simple, static world setting, editing XML file is usually sufficient. However, more advanced users, such as checking for collisions with terrain and objects in the existing world before you create a new object, will require the ability to create a world programmatically.

Difficulty: easy

Time: 20 minutes



## World XML editing

In this tutorial we will be going through an example step by step to set up a simple world XML file. A Klamp't world contains some number of entities, which can be of the type robot, rigid object, and terrain.  These are typically loaded from robot models and geometries located in the Klampt-examples/data folder.

* A robot can be understood as an articulated and usually actuated objects containing multiple robot links. There are two file formats, .rob (used by only Klampt) or .urdf (more widely used, e.g., in ROS). You can make your own robot file or use the available models located at Klampt-examples/data/robots.
* A terrain is an obstacle in the robot's enviroment, which will never move, such as a ground plane.  Some example terrain files can be found in Klampt-examples/data/terrains. 
* A rigid object is an obstacle that can be moved, such as an object that the robot should move from point A to B.  Examples can be found in Klampt-examples/data/objects.

If you are unsure about when to use terrain and when to use objects, a rule of thumb is that you should set up items you won't directly interact with but will limit your working space as terrains, such as walls, ground or a table).  However, you should not set the table as a terrain if you plan to knock it over, because a terrain cannot be moved.  Rigid objects need more data, like mass properties, to be specified.  If you are planning on using physics simulation, you will also need a terrain for them to rest on to stop a rigid object from falling through infinite space!

You can certainly make your own geometries with a CAD program (e.g. stl file) or download existing geometries for customized appearance. There are four currently supported types of geometry: primitives (GeometricPrimitive), triangle meshes (TriangleMesh), point clouds (PointCloud), groups (Group).

Now let us go through an example of editing an XML file that sets up a simple world. The code below comes from Klampt-examples/data/tx90cuptable.xml, but there are lots of other example files in that folder.

```xml
<?xml version="1.0" encoding="UTF-8"?> 
<world> 
    <!-- the robot --> 
    <robot name="TX90L-pr2" file="Klampt-examples/data/robots/tx90pr2.rob" /> 
    <!-- the ground plane --> 
    <terrain file="Klampt-examples/data/terrains/block.off" /> 
</world>
```

This XML should be rather straight forward to interpret. The \<robot\> tag first loads a robot tx90pr2.rob from the Klampt-examples/data/robots folder. The \<terrain\> tag loads "Klampt-examples/terrains/block.off" as a ground plane, which is the solid ground the robot is standing on.  You may need to change the paths around to reflect where the Klampt-examples folder is located on your system. 

*Note*: When loading objects specified by a path, Klampt first tries to look relative to the current file's directory.  If a file cannot be found, it will look on the absolute path. For example, the above code assumes the Klampt-examples directory is in the same folder as this file. To specify an absolute path, use something like "/home/MYLOGIN/Klampt-examples/robots/tx90pr2.rob" where you replace MYLOGIN with your actual login name.

If we then save this to myworld.xml launch this using RobotPose, as follows:
```
RobotPose myworld.xml
```
or, if you have not installed the Klampt apps:
```
[PATH_TO_KLAMPT]/bin/RobotPose myworld.xml
```
we should see something like this:

![Simple world image](images/myworld1.png)

Now let's make this a bit more interesting.  Let's put together a table using several blocks.  To do so we will use the cube.off file, which is a unit cube with its origin at its lower corner.  We can scale and translate this cube to make our table.  The following XML file has 5 blocks making up a table. We also would like to change the color of the table, so the \<display\> tag is used to set a brownish RGB color.  It also adds a \<rigidObject\> tag which places a mug hovering over the table.

```xml
<?xml version="1.0" encoding="UTF-8"?> 
<world> 
    <!-- the robot --> 
    <robot name="TX90L-pr2" file="Klampt-examples/data/robots/tx90pr2.rob" /> 
    <!-- the ground plane --> 
    <terrain file="Klampt-examples/data/terrains/block.off" /> 
    <!-- the table --> 
    <terrain file="Klampt-examples/data/terrains/cube.off" scale="0.05 0.05 0.6" translation="0.6 0.25 0.01"> 
        <display color="0.4 0.3 0.2"/> 
    </terrain> 
    <terrain file="Klampt-examples/data/terrains/cube.off" scale="0.05 0.05 0.6" translation="0.6 -0.25 0.01"> 
        <display color="0.4 0.3 0.2"/> 
    </terrain> 
    <terrain file="Klampt-examples/data/terrains/cube.off" scale="0.05 0.05 0.6" translation="1.1 0.25 0.01"> 
        <display color="0.4 0.3 0.2"/> 
    </terrain> 
    <terrain file="Klampt-examples/data/terrains/cube.off" scale="0.05 0.05 0.6" translation="1.1 -0.25 0.01"> 
        <display color="0.4 0.3 0.2"/> 
    </terrain> 
    <terrain file="Klampt-examples/data/terrains/cube.off" scale="0.6 0.6 0.02" translation="0.575 -0.275 0.615"> 
        <display color="0.4 0.3 0.2"/> 
    </terrain> 
    <!-- a mug --> 
    <rigidObject name="srimug" file="Klampt-examples/data/objects/srimugsmooth.obj" position="0.7 0.1 0.75" /> 
</world>
```

Launching this again in RobotPose, we get this:

![Simple world image](images/myworld2.png)



## Programmatic creation

TODO