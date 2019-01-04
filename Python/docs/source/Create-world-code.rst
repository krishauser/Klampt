Klamp't Tutorial: Dynamically create a world using code
=======================================================

In this tutorial we will learn how to create a world using code. Inside
a simulation, world is generally defined as an instance of the
:class:`~klampt.WorldModel`
class containing robots, rigid objects, and static environment geometry.
Typically, a world file is specified using an XML file, as shown in the
`Build and edit a world file <https://github.com/krishauser/klampt/blob/master/docs/Tutorials/Build-and-edit-world.md>`__
tutorial.  But in this example, we'll add objects into the world programmatically.


Let's start with the ``myworld.xml`` file created in the previous tutorial.  Now
create a Python file named ``world_create_test.py`` with the following contents:

.. code:: python

    from klampt import *
    from klampt import vis

    w = WorldModel()
    if not w.readFile("myworld.xml"):
        raise RuntimeError("Couldn't read the world file")
    vis.add("world",world)
    vis.run()

Running this, using::

    python world_create_test.py

will pop up the same visualization as in the RobotPose program.  Now, let's change this by
writing a function to modify the world.  This ``make_shelf`` function will add a cube shelf
as terrain to the world:

.. code:: python

    #you will need to change this to the absolute or relative path to Klampt-examples
    KLAMPT_EXAMPLES = 'Klampt-examples'

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
        left.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
        right.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
        back.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
        bottom.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
        top.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
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

The code first create the left, right, back, bottom and top pieces of
the shelf as a Geometry3D. A three-D geometry can either be a reference to
a world item's geometry, in which case modifiers change the world item's
geometry, or it can be a standalone geometry.  See the :class:`~klampt.Geometry3D`
documentation for more details.

The next step is to load the 3D geometry, in this case ``cube.off`` located
in ``Klampt-examples/data/objects``. Each of these are set up by scaling and
translating each of them, similar to the steps when editing XML. Each geometry
stores a "current" transform, which is automatically updated for world
items' geometries. The proximity queries are performed with respect to
the transformed geometries.

Lastly, the function groups the seperate pieces together and
combine it as one terrain named "shelf" and assign a color to the shelf.

We now make changes to ``world_create_test.py`` to include the
``make_shelf`` function, add shelf parameters and call the function
after the world has been loaded, but before it is drawn.

::

    from klampt import *
    from klampt import vis

    #you will need to change this to the absolute or relative path to Klampt-examples
    KLAMPT_EXAMPLES = 'Klampt-examples'

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
        left.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
        right.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
        back.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
        bottom.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
        top.loadFile(KLAMPT_EXAMPLES+"/data/objects/cube.off")
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
    
    w = WorldModel()
    if not w.readFile("myworld.xml"):
        raise RuntimeError("Couldn't read the world file")

    shelf = make_shelf(world,*shelf_dims)
    shelf.geometry().translate((shelf_offset_x,shelf_offset_y,shelf_height))

    vis.add("world",world)
    vis.run()

Running this script again, you can see that on top of what was in the world XML file,
there should be a new shelf lying on the table surface:

.. image:: _static/images/shelf.png

