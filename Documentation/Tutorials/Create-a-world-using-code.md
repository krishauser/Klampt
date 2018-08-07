
# Klamp't Tutorial: Dynamically create a world using code

In this tutorial we will learn how to create a world using code. Inside a simulation, world is generally defined as an instance of the [WorldModel](http://motion.pratt.duke.edu/klampt/pyklampt_docs/classklampt_1_1robotsim_1_1WorldModel.html) class containing robots, rigid objects, and static environment geometry. In addition to the loaded robot world from the XML file, the world can also be created and edited using code. This tutorial describes how to create a robot world using code.

Difficulty: simple

Time: 20 minutes

### Programmatic creation of a world

Now that we have covered creation of a world through XML editing [Build and edit a world file](Documentation/Tutorials/Build-and-edit-world.md), let's do an exmaple of adding objects into the world progrommatically. We will add a cube shelf as terrian to the world by writting a function called "make_shelf":

```
def make_shelf(world,width,depth,height,wall_thickness=0.005):
	"""Makes a new axis-aligned "shelf" centered at the origin with
	dimensions width x depth x height. Walls have thickness wall_thickness. 
	If mass=inf, then the box is a Terrain, otherwise it's a RigidObject
	with automatically determined inertia.
	"""
	left = Geometry3D()
	right = Geometry3D()
	back = Geometry3D()
	bottom = Geometry3D()
	top = Geometry3D()
	left.loadFile("your_path_to/data/objects/cube.off")
	right.loadFile("your_path_to/data/objects/cube.off")
	back.loadFile("your_path_to/data/objects/cube.off")
	bottom.loadFile("your_path_to/data/objects/cube.off")
	top.loadFile("your_path_to/data/objects/cube.off")
	left.transform([wall_thickness,0,0,0,depth,0,0,0,height],[-width*0.5,-depth*0.5,0])
	right.transform([wall_thickness,0,0,0,depth,0,0,0,height],[width*0.5,-depth*0.5,0])
	back.transform([width,0,0,0,wall_thickness,0,0,0,height],[-width*0.5,depth*0.5,0])
	bottom.transform([width,0,0,0,depth,0,0,0,wall_thickness],[-width*0.5,-depth*0.5,0])
	top.transform([width,0,0,0,depth,0,0,0,wall_thickness],[-width*0.5,-depth*0.5,height-wall_thickness])
	shelfgeom = Geometry3D()
	shelfgeom.setGroup()
	for i,elem in enumerate([left,right,back,bottom,top]):
		g = Geometry3D(elem)
		shelfgeom.setElement(i,g)
	shelf = world.makeTerrain("shelf")
	shelf.geometry().set(shelfgeom)
	shelf.appearance().setColor(0.2,0.6,0.3,1.0)
	return shelf
```
The code first create the left, right, back, bottom and top pieces of the shelf as Geometry3D. A three-D geometry can either be a reference to a world item's geometry, in which case modifiers change the world item's geometry, or it can be a standalone geometry. For more information on Geometry3D class, refer to  [Python klampt.robotsim.Geometry3D Namespace Reference](http://motion.pratt.duke.edu/klampt/pyklampt_docs/classklampt_1_1robotsim_1_1Geometry3D.html)  .

The next step is to load the 3D geometry, in this case cube.off located in data/objects folder. Followed by setting up scaling and transform for each of them (similar to the steps when editing XML). Each geometry stores a "current" transform, which is automatically updated for world items' geometries. The proximity queries are performed with respect to the transformed geometries.

Lastly, the make_shelf function group the seperate pieces together and combine it as one terrain named "shelf" and assign a color to the shelf.

We now make changes to our visualize_world.py to include the make_shelf function, add shelf parameters and call make_shelf function. To properly use of the code, please change the path to the objects to the correct path on your local computer.
```
from klampt import *
import sys
import time
from klampt.sim import *
from klampt import vis

shelf_dims = (0.4,0.4,0.3)
shelf_offset_x=0.8
shelf_offset_y = 0.1
shelf_height = 0.65

def make_shelf(world,width,depth,height,wall_thickness=0.005):
	"""Makes a new axis-aligned "shelf" centered at the origin with
	dimensions width x depth x height. Walls have thickness wall_thickness. 
	If mass=inf, then the box is a Terrain, otherwise it's a RigidObject
	with automatically determined inertia.
	"""
	left = Geometry3D()
	right = Geometry3D()
	back = Geometry3D()
	bottom = Geometry3D()
	top = Geometry3D()
	left.loadFile("your_path_to/data/objects/cube.off")
	right.loadFile("your_path_to/data/objects/cube.off")
	back.loadFile("your_path_to/data/objects/cube.off")
	bottom.loadFile("your_path_to/data/objects/cube.off")
	top.loadFile("your_path_to/data/objects/cube.off")
	left.transform([wall_thickness,0,0,0,depth,0,0,0,height],[-width*0.5,-depth*0.5,0])
	right.transform([wall_thickness,0,0,0,depth,0,0,0,height],[width*0.5,-depth*0.5,0])
	back.transform([width,0,0,0,wall_thickness,0,0,0,height],[-width*0.5,depth*0.5,0])
	bottom.transform([width,0,0,0,depth,0,0,0,wall_thickness],[-width*0.5,-depth*0.5,0])
	top.transform([width,0,0,0,depth,0,0,0,wall_thickness],[-width*0.5,-depth*0.5,height-wall_thickness])
	shelfgeom = Geometry3D()
	shelfgeom.setGroup()
	for i,elem in enumerate([left,right,back,bottom,top]):
		g = Geometry3D(elem)
		shelfgeom.setElement(i,g)
	shelf = world.makeTerrain("shelf")
	shelf.geometry().set(shelfgeom)
	shelf.appearance().setColor(0.2,0.6,0.3,1.0)
	return shelf
if __name__ == "__main__":
    if len(sys.argv)<=1:
        print "USAGE: visualize_world.py [world_file]"
        exit()
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)
    shelf = make_shelf(world,*shelf_dims)
    shelf.geometry().translate((shelf_offset_x,shelf_offset_y,shelf_height))
    vis.add("world",world)
    vis.show()
    t0 = time.time()
    while vis.shown():
       	t1 = time.time()
       	time.sleep(max(0.01-(t1-t0),0.001))
       	t0 = t1
```
Again, go into the folder that contains visualize_world.py file run:
```
python visualize_world.py your_path_to/data/tx90cuptable.xml
```
Now on top of what you had in the world, there should be a new shelf lying on the table surface:
<p align="center">
<img src="http://motion.pratt.duke.edu/klampt/tutorials/shelf.png"
width="75%" height="75%">
</p>

This wraps up our discussion on the basics of creating a world in Klampt, for more advanced usage, including dynamically creating objects from database and adjust placement and orientation of the objects until collision free, please refer to the main function file in [IROS2016ManipulationChallenge github repository](https://github.com/krishauser/IROS2016ManipulationChallenge).