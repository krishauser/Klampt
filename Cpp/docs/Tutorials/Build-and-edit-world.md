
# Klamp't Tutorial: Build and edit a world file

In this tutorial we will learn the first step in setting up your simulation--creating and modifying a world. Klampt allows you to create a world in 2 ways: static XML file editing or programmatic creation. These 2 approaches can be used at the same time as well. For a simple,relatively static world setting, editing XML file is usually sufficient. However, if you want to add/delete objects as your simulation goes or say check for collisions with terrian and objects in the existing world before you create a new object, the ability to create world programmatically becomes very handy.

Difficulty: simple

Time: 20 minutes
### Python API
In this tutorial we will be creating a simple world using python API. We will be going through an example step by step to set up a tx90 robot in a world that has coffee mug on a table through XML editing, we will then add a shelf to the world programmactically in the main file.

A World model contains some number of entities, which can be of the type robot, robot link, rigid object, and terrain. Each entity has an index into the list of entities of that type, a unique ID number in the World and could be consisted of one or more bodies where each body has a coordinate frame and usually some attached geometry and an appearance.The 3 main elements-- robot, terrain and objects can be built using robot models and geometry primitives located in Klampt/data folder.

A robot can be understood as articulated and possibly actuated objects containing multiple robot links. You can make your own robot.rob file or use the available robot models located at Klampt/data/robots folder.You should checkout the RobotModel class API and learn how to get more information about the robot model you choose. After selecting a robot, the next step is to set up terrian. Terrian is the enviroment the robot stays in and the purpose of the terrian is to defines the physical boundary of the world. Some example terrian files can be found in Klampt/data/terrians. If you are unsure about when to use terrian and when to use objects, a rule of thumb is that you should set up items you won't directly interact with but will limit your working space as terrians(for example, walls, ground or a table, however, you should not set the table as a terrian if you plan to knock it over, because a terrian cannot be moved). If you do want to interact with certain objects and want the objects to have mass and dynamics accosiated with them like objects in the real physical world do, you should set it up as a "rigidObject". The Terrian and rigidObject files available in Klampt/data folder are mostly basic shape primitives that you can connect and combine to make reasonably realistic looking real-world items. However, you can certainly scan and make your own geometrics(e.g. stl file) or download existing geometrics database and load it to the world for customized appearance. There are four currently supported types of geometry: primitives (GeometricPrimitive), triangle meshes ( TriangleMesh), point clouds ( PointCloud), groups (Group).

To play around with the world and objects in the world please refer to this  [Python klampt.robotsim Namespace Reference](http://motion.pratt.duke.edu/klampt/pyklampt_docs/namespaceklampt_1_1robotsim.html)  .

Now let us go through an example of editing XML file that sets up a simple world. The code below comes from Klampt/data/tx90cuptable.xml, there are lots of other example scenarios in the data folder you can reference if you are learing how to build a world through XML file editing.
```
<?xml version="1.0" encoding="UTF-8"?> 
<world> 
	<robot name="TX90L-pr2" file="robots/tx90pr2.rob" /> 
	<terrain file="terrains/block.off" /> 
	<!-- the table --> 
	<terrain file="terrains/cube.off" scale="0.05 0.05 0.6" translation="0.6 0.25 0.01"> 
		<display color="0.4 0.3 0.2"/> 
	</terrain> 
	<terrain file="terrains/cube.off" scale="0.05 0.05 0.6" translation="0.6 -0.25 0.01"> 
		<display color="0.4 0.3 0.2"/> 
	</terrain> 
	<terrain file="terrains/cube.off" scale="0.05 0.05 0.6" translation="1.1 0.25 0.01"> 
		<display color="0.4 0.3 0.2"/> 
	</terrain> 
	<terrain file="terrains/cube.off" scale="0.05 0.05 0.6" translation="1.1 -0.25 0.01"> 
		<display color="0.4 0.3 0.2"/> 
	</terrain> 
	<terrain file="terrains/cube.off" scale="0.6 0.6 0.02" translation="0.575 -0.275 0.615"> 
		<display color="0.4 0.3 0.2"/> 
	</terrain> 
	<rigidObject name="srimug" file="objects/srimugsmooth.obj" position="0.7 0.1 0.75"> 
		<graspGenerator type="GraspIt" file="objects/solution2.txt" /> 
	</rigidObject>
	<target position="-0.5 0 0.2" /> 
	<simulation> 
		<object index="0"> 
			<geometry padding="0.005" /> 
		</object> 
	</simulation> 
</world>
```
This XML should be rather straight forward to read and easy to understand. It first loads a robot tx90pr2.rob from the data/robots folder and specify its type as "robot". Then it loads "terrains/block.off" as terrian, this is the solid ground the robot is standing on. Then the code uses 5 cubes with different scaling and translation and group and combine them in a single terrian "table". Lastly it loades "objects/srimugsmooth.obj as a rigid object and associated it's grasping with a GraspIt solution, this connection is not neccessary at the moment but it is convenient if you are planning to do some object grasping in the future.


You can open your text editor and type in the following code to visualize the world.
```
from klampt import *
import sys
import time
from klampt.sim import *
from klampt import vis

if __name__ == "__main__":
    if len(sys.argv)<=1:
        print "USAGE: visualize_world.py [world_file]"
        exit()
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn) 
    vis.add("world",world)
    vis.show()
    
    t0 = time.time()
    while vis.shown():
        t1 = time.time()
	#update your code here	       
       	time.sleep(max(0.01-(t1-t0),0.001))
       	t0 = t1
```
Name this file visualize_world.py and in the folder that contain this file and run:
```
python visualize_world.py your_path_to/data/tx90cuptable.xml
```
Result should look like this:
<p align="center">
<img src="http://motion.pratt.duke.edu/klampt/tutorials/cuptable.png"
width="75%" height="75%">
</p>