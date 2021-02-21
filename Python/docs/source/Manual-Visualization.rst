Visualization
=============================

Klamp't allows you to easily produce custom interactive visualizations
with just a few lines of Python code.  This is extremely useful for
debugging!

The `klampt.vis <klampt.vis.html>`__ module features:

- Hardware-accelerated OpenGL visualizations using PyQT (available with
  `pip install PyQt`) or GLUT (`pip install PyOpenGL`)
- Unified interface to Jupyter notebooks and HTML output
- Can draw most anything supported in Klampt, as well as text
- One-line visual debugging with ``vis.debug()``
- One-line visual editing with ``resource.edit()``
- Automatic animations
- Simple plotting
- Multi-window management
- Animation exporting (available with ffmpeg and ``pip install Pillow``)
- Camera fitting and automatic following control
- In-GUI scene customization available (available with ``pip install pyqtgraph``)


Quick start
-------------

*Ultra-quick start*: use ``vis.debug()``, which pops up a window to display
the given Klamp't objects. For example::

    from klampt import *

    #some Klamp't code to load a world with a robot
    w = WorldModel()
    w.readFile("Klampt-examples/data/tx90_scenario.xml")
    r = w.robot(0)
    q0 = r.getConfig()
    r.randomizeConfig()
    qrand = r.getConfig()

    #show it
    vis.debug(qrand,{'color':[1,0,0,0.5]},world=w)

    #some more Klamp't code to load a geometry
    g = Geometry3D()
    g.loadFile("Klampt-examples/data/objects/srimugsmooth.off")

    #show it
    vis.debug(g)

*Ultra-quick start2*: If you want to visually edit a Klamp't object, just call
``io.resource.edit(name,object,...)``. 
The :func:`klampt.io.resource.edit` function has many options to configure how the editor
should be configured.  For example, to edit robot configurations and show objects around the
robot, you would use ``io.resource.edit(name,config,world=world,referenceObject=robot)``.

For any more advanced usage, you will want to manage a scene using the vis module. 
The basic use of the module is straightforward:

1. Add things to the visualization scene with ``vis.add(name,thing)``.  Worlds,
   geometries, points, transforms, trajectories, contact points, and more can
   be added in this manner.
2. Modify the scene using modifier calls like aft
   ``vis.setColor(name,r,g,b,a)``, ``vis.hide(name)``, or ``vis.remove(name)``.
3. Launch visualization window(s)
4. Continue adding, modifying, and removing things as you desire.
5. You may then close the window when you are done, or wait until the user 
   closes the window.

For example::

    from klampt import *
    from klampt.model.trajectory import RobotTrajectory
    import time

    #some Klamp't code
    w = WorldModel()
    w.readFile("Klampt-examples/data/tx90_scenario.xml")
    r = w.robot(0)
    q0 = r.getConfig()
    r.randomizeConfig()
    qrand = r.getConfig()
    r.setConfig(q0)

    #add a "world" item to the scene manager
    vis.add("world",w)
    #show qrand as a ghost configuration in transparent red
    vis.add("qrand",qrand,color=(1,0,0,0.5))
    #show a Trajectory between q0 and qrand
    vis.add("path_to_qrand",RobotTrajectory(r,[0,1],[q0,qrand]))

    #To control interaction / animation, launch the loop via one of the following:

    #OpenGL on Linux / windows
    vis.show()              #open the window
    t0 = time.time()
    while vis.shown():
        #do something, e.g. the following
        if time.time() > 5:
            vis.setColor("qrand",0,1,1,0.5)  #sets qrand to show in cyan after 5 seconds
        time.sleep(0.01)    #loop is called ~100x times per second
    vis.kill()              #safe cleanup

    #Mac OpenGL workaround: launch the vis loop and window in single-threaded mode
    #vis.loop()

    #for IPython, the screen is redrawn only after a cell is run, so you should just call
    #vis.show() in this cell, and then the inner loop 


More advanced functions allow you to dynamically launch multiple windows,
capture user input, embed the visualization into Qt windows, and create
animations as standalone HTML web pages.


Visualization backends
-----------------------

The first time you call a ``klampt.vis`` function, the :func:`~klampt.vis.visualization.init`
function is called to initialize one of four possible backends: PyQt, GLUT,
IPython (Jupyter notebook), or HTML (compatible with Google Colab or as
standalone web pages). 

- The PyQt and GLUT backends deliver interactive OpenGL visualizations that
  (typically run in a separate thread from the main thread.  Your main thread
  of code can update the visualization asynchronously. 
- The IPython and HTML backends are single-threaded and require you to
  structure your code to update the visualization and obtain the output where
  needed.  See the `klampt.vis <klampt.vis.html#ipython-jupyter-notebook>`__
  documentation for more details.

If run by default, init() will auto-determine the visualization backend to use.
This is usually the most sensible / powerful backend available for your system:

- Running from a console: use PyQt if available, falling back to GLUT, then
  HTML.  (Note that most vis programs are written assuming OpenGL support and
  don't support HTML output.)
- Running from an IPython notebook: use IPython output.

If you would like to use another backend, you can call 
``vis.init(DESIRED BACKENDS)`` before any other ``klampt.vis`` call.
For example, if you'd like to view a PyQT window from an IPython notebook, call
``vis.init('PyQt')`` at the top your notebook.  Also, if you'd like to use HTML
output (Google Colab users), call ``vis.init('HTML')`` at the top of your
notebook.


Visualization window management
--------------------------------

Window creation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

it can be thought of as
having an existing hidden window which hosts the scene manager. You will
then configure the window or scene manager, and in the most cross-platform
compatible mode of operation, you will show it using one
of the following methods.

-  ``vis.spin(duration)``: shows the window until it is closed or
   ``duration`` seconds have elapsed.
-  ``vis.dialog()``: shows the current window in a dialog format, and does
   not return until the user closes the window or presses the OK button.
-  ``vis.run()``: shows the window, and once the user closes the window,
   the visualization is killed.
-  ``vis.kill()``: performs all cleanup of the vis module.

These methods block the calling thread until the window is closed.
You can call ``spin`` and ``dialog`` multiple times in a row.  If you have PyQt
installed, and want to customize the UI, you can use the method

-  ``vis.customUI(makefunc)``: takes a 1-parameter function
   makefunc(glwidget) that takes the Klampt QGLWidget as an argument,
   creates a QWindow or QDialog to be shown, and returns it.

Multithreaded mode
~~~~~~~~~~~~~~~~~~~

In Linux and Windows, a *multithreaded mode* is available which allows you
to conveniently run visualizations in parallel with your main code.  This
means you can very easily pop up a visualization window to observe a
processing loop (such as a simulation or planner) in real-time.

Suppose you had a loop like this:

.. code:: python

    import klampt
    
    world = klampt.WorldModel()
    #...configure stuff...
    while not done():
      #...do stuff to world... 

The corresponding live visualization of the loop would look like this:

.. code:: python

    import klampt
    from klampt import vis

    world = klampt.WorldModel()
    vis.add("world",world)    #world is now referenced by the vis module and is shared between threads!
    #...configure stuff...
    vis.show()
    while not done() and vis.shown():
      vis.lock()
      #...do stuff to world... #this code is executed at approximately 10 Hz due to the sleep call
      vis.unlock()
      time.sleep(0.1)
    if done():
      vis.show(False)         #hides the window if not closed by user

Specifically, the multithreaded mode uses the following functions:

-  ``vis.show()``: shows the current window and returns immediately to
   the calling thread.
-  ``vis.shown()``: returns True if the window is shown and not closed
   by the user.
-  ``vis.show(False)``: hides the current window.

   .. note::

      ``vis.hide()`` doesn't do the opposite of ``vis.show``.  It refers to
      hiding items in the scene manager.

When you call ``show`` the visualization is run in a separate thread from
the main Python script. The visualization and Klamp't objects that it refers to can then be
configured and modified by the main Python thread. However, some care is
needed when directly modifying Klamp't objects that are referred to in
the visualization. To prevent conflicts in threading which may cause the
program to crash, all references to shared objects in the main thread
should be placed between ``vis.lock()`` and ``vis.unlock()`` calls, as shown
in the above code.

Multithreaded mode workaround on Mac
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For Mac users, multithreaded mode is not available.  You can mimic a
multithreaded loop using the ``vis.loop()`` function, which takes several
callback functions to be run inside the visualization loop.  This version
is written as follows:

.. code:: python

    world = klampt.WorldModel()
    #...configure stuff...

    vis.add("world",world)

    def setup():
      vis.show()

    def callback():
      #...do stuff to world... #this code is executed at approximately 10 Hz due to the sleep call
      time.sleep(0.1)
      if done():
        vis.show(False)         #hides the window if not closed by user

    def cleanup():
      #can perform optional cleanup code here
      pass

    vis.loop(setup=setup,callback=callback,cleanup=cleanup)

Note that the ``loop`` function can also be run on Linux and Windows, so
if you are writing cross-platform code, the main rule to remember is not to use
``vis.show()`` outside of a loop setup callback.


The plugin stack
~~~~~~~~~~~~~~~~~

The vis module lets you *override* or *stack* plugins together,
even with the existing scene graph manager. In fact, the scene graph
manager is itself a plugin. 

Each window has a *plugin stack* with at least one plugin.
The stack can be modified using the following functions:

-  ``vis.setPlugin`` overrides the plugin stack used by the current
   window.
-  ``vis.pushPlugin`` and ``vis.popPlugin`` modify the plugin stack used
   by the current window.


Split screen and multiple windows
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To do split screen, call ``vis.addPlugin(plugin)`` with the root plugin
for the new viewport.

The vis module can handle multiple windows. The vis module stores an
*active window*, which is the window to which subsequent vis calls will
be passed. (not the window currently selected by the user). The relevant
functions are:

-  ``vis.createWindow()``: creates and returns the identifier for a new
   window. If this is the first createWindow call, no new window is
   created, and instead the ID of the hidden window is returned.
-  ``vis.setWindow(id)``: changes the active window.
-  ``vis.getWindow()``: returns the active window.




klampt.vis Scene Manager
-------------------------

Using the scene manager, the main thread can easily add and remove items
to be drawn. Simple functions are available to build multi-viewport
GUIs, to customize appearances, control animations, and other
visualization functions. For more information see the documentation of
`klampt.vis <klampt.vis.html>`__,
and the example code in
`Klampt-examples/Python3/demos/vistemplate.py <https://github.com/krishauser/Klampt-examples/blob/0.8.3/Python3/demos/vis_template.py>`__.

-  ``vis.add(name,item)``: adds a named item to the scene manager.
-  ``vis.clear()``: clears all items.
-  ``vis.remove(name)``: removes an existing item.
-  ``vis.hide(name,hidden=True)``: hides/unhides an existing item.

   .. note::
      ``vis.show()`` doesn't do the opposite of ``vis.hide()``.  To
      un-hide an item, call ``vis.hide(False)``.

-  ``vis.edit(name,doedit=True)``: turns on/off visual editing, if the
   item allows it.

Here are the accepted types in the scene manager.

+-----------------------------+------------------------------------------+------------------------------------------+
|    Type                     | Notes                                    | Attributes                               |
+=============================+==========================================+==========================================+
| ``str``                     | Draws a label                            | ``position``\*                           |
+-----------------------------+------------------------------------------+------------------------------------------+
| ``WorldModel``              |                                          |                                          |
+-----------------------------+------------------------------------------+------------------------------------------+
| ``RobotModel``              |                                          |                                          |
+-----------------------------+------------------------------------------+------------------------------------------+
| ``RigidObjectModel``        |                                          |                                          |
+-----------------------------+------------------------------------------+------------------------------------------+
| ``TerrainModel``            |                                          |                                          |
+-----------------------------+------------------------------------------+------------------------------------------+
| ``Geometry3D``              |                                          |                                          |
+-----------------------------+------------------------------------------+------------------------------------------+
| ``PointCloud``              |                                          | ``size`` (1)                             |
+-----------------------------+------------------------------------------+------------------------------------------+
| ``Vector3``                 |                                          | ``size`` (5)                             |
+-----------------------------+------------------------------------------+------------------------------------------+
| ``RigidTransform``          |                                          | ``fancy`` (False), ``length`` (0.1),     |
|                             |                                          | ``width`` (0.01)                         |
+-----------------------------+------------------------------------------+------------------------------------------+
| ``Config``                  | Shows a ghost of the robot               | ``robot`` (0)                            |
+-----------------------------+------------------------------------------+------------------------------------------+
| ``Configs``                 | If contains 2D or 3D points, draws a     | ``robot`` (0), ``maxConfigs`` (20)       |
|                             | polyline (& uses Trajectory attributes)  |                                          |
+-----------------------------+------------------------------------------+------------------------------------------+
| ``Trajectory``              | Draws 2D, 3D, SE(3), or end-effector     | ``robot`` (0), ``width`` (3),            |
|                             | paths                                    | ``pointSize`` (None), ``pointColor``     |
|                             |                                          | (None), ``endeffectors`` (all terminal   |
|                             |                                          | links)                                   |
+-----------------------------+------------------------------------------+------------------------------------------+
| ``IKGoal``                  |                                          | ``length`` (0.1), ``width`` (0.01)       |
+-----------------------------+------------------------------------------+------------------------------------------+
| ``coordinates.Point``       |                                          | ``size`` (5)                             |
+-----------------------------+------------------------------------------+------------------------------------------+
| ``coordinates.Direction``   |                                          | ``length`` (0.15)                        |
+-----------------------------+------------------------------------------+------------------------------------------+
| ``coordinates.Frame``       |                                          | ``length`` (0.1), ``width`` (0.01)       |
+-----------------------------+------------------------------------------+------------------------------------------+
| ``coordinates.Transform``   | Draws a curve between frames             |                                          |
+-----------------------------+------------------------------------------+------------------------------------------+
| ``ContactPoint``            |                                          | ``size`` (5), ``length`` (0.1)           |
+-----------------------------+------------------------------------------+------------------------------------------+

\* denotes a mandatory attribute.  Values in parentheses are defaults.

Note: ``color``, ``label``, and ``hide_label`` are always accepted attributes.


Item path conventions and references
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-  The world, if one exists, should be given the name ``'world'``.
-  Configurations and paths are drawn with reference to the first robot
   in the world.
-  The Simulator, if one exists, should be given the name ``'sim'``.
   Then, the vis module will save movies along simulation time instead
   of real (wall-clock) time.

For composite items like WorldModels, sub-items can be referred to by
passing a tuple or list of strings as the ``name`` argument to any of
these functions. For example, ``("world",robotname,linkname)`` refers
to a given link of a given robot inside the "world" item. 

For example, if you've added a RobotWorld under the name ``'world'`` containing a
robot called ``'myRobot'``, then ``setColor(('world','myRobot'),0,1,0)`` will
turn the robot green. If ``'link5'`` is the robot's 5th link, then
``setColor(('world','myRobot','link5'),0,0,1)`` will turn the 5th
link blue.

To retrieve the path to a sub-item, ``vis.getItemName(object)`` can be used. [new in 0.8.3]

Customizing item appearance
~~~~~~~~~~~~~~~~~~~~~~~~~~~

TODO: describe these functions

Animations
~~~~~~~~~~

The scene manager accepts animations for certain types of items.
Animations are currently supported for points, so3 elements, se3
elements, rigid objects, and robots.

-  ``vis.animate(name,animation,speed=1.0,endBehavior='loop')``: Sends
   an animation to the
   object. May be a Trajectory or a list of configurations.

   -  ``speed``: a modulator on the animation speed. If the animation is
      a list of
      milestones, it is by default run at 1 milestone per second.
   -  ``endBehavior``: either 'loop' (animation repeats forever) or
      'halt' (plays once).

-  ``vis.pauseAnimation(paused=True)``: Turns on/off animation globally.
-  ``vis.stepAnimation(amount)``: Moves forward the animation time by
   the given amount, in seconds.
-  ``vis.animationTime(newtime=None)``: Gets/sets the current animation
   time

   -  If newtime == None (default), this gets the animation time.
   -  If newtime != None, this sets a new animation time.


Scene and camera control
~~~~~~~~~~~~~~~~~~~~~~~~

The background color can be changed with ``vis.setBackgroundColor``. In OpenGL modes, a
background image can be set using ``klampt.vis.scene().setBackgroundImage``

If PyQTGraph is installed (``pip install pyqtgraph``), the menu has an "Edit appearance..." item that launches
a GUI to edit colors and properties of the scene.

Overall scene appearance can be saved/loaded using ``vis.saveJsonConfig`` and ``vis.loadJsonConfig``.

The camera can be modified in several ways. 

- :func:`~klampt.vis.visualization.getViewport` returns a :class:`~klampt.vis.glviewport.GLViewport` instance (in OpenGL)
  or a JSON structure (In IPython / HTML) that can be modified.  Then, :func:`~klampt.vis.visualization.setViewport` can
  be called to change the viewport.
- You can save and load the viewport from files.  In PyQt, there are menu items for doing this, but in
  other backends, this must be done using ``vis.getViewport().save_file(fn)`` (These are the same format
  as the camera files used in the RobotTest, SimTest, and RobotPose apps.)
- To auto-fit a scene, use :func:`~klampt.vis.visualization.autoFitCamera`.
- To follow an object, use :func:`~klampt.vis.visualization.followCamera`.


User interaction and customization
-----------------------------------

There are several ways to provide user interaction in the visualizer:

- Items in the visualization world can be edited using ``vis.edit(itemname)``. 
  To retrieve the object's configuration after or during editing, use ``vis.getItemConfig(itemname)``.

  In OpenGL, the editing happens via mouse interaction. 

  In Jupyter, a widget will be displayed in the output of
  the cell in which ``vis.edit`` was called.  (To customize Jupyter widgets further, you can create
  them `manually <klampt.vis.ipython.html>`__.)

- (OpenGL) Keyboard-triggered actions can be added with ``vis.addAction``. The calling
  pattern is::

      vis.addAction(lambda: [DO SOMETHING HERE], "My action",'k')

  which will trigger the lambda function when 'k' is pressed or "My action" is selected from the
  menu bar.  You can also use the prefix ``'Ctrl+'`` or ``'Shift+'`` to require modifiers to be held,
  e.g. ``Ctrl+k``.
  More information is available in the :func:`~klampt.vis.visualization.addAction` documentation.
- (OpenGL) Add custom :class:`~klampt.vis.glinterface.GLPluginInterface` plugins to the visualization.
  See the section below for more details.
- (PyQt) Embed the visualizer into a Qt window, and add buttons, etc.  To use this, you will need to
  define a hook that will capture the OpenGL window and add it into your main window, such as the
  following code::

      from klampt import *
      from PyQt5.QtCore import *
      from PyQt5.QtGui import *
      from PyQt5.QtWidgets import *

      #TODO: set up world

      def make_gui(glwidget):
          #place your Qt code here and place the glwidget where it needs to be
          w = QMainWindow()
          glwidget.setMaximumSize(4000,4000)
          glwidget.setSizePolicy(QSizePolicy(QSizePolicy.Maximum,QSizePolicy.Maximum))
          area = QWidget(w)
          layout = QVBoxLayout()
          layout.addWidget(glwidget)
          layout.addWidget(QPushButton("Click me"))
          area.setLayout(layout)
          w.setCentralWidget(area)
          return w

      vis.customUI(make_gui)
      vis.add("world",world)
      vis.show()
      vis.spin(float('inf'))

Note that the HTML backend doesn't support any user interaction.


Making your own plugins
-----------------------

The :class:`~klampt.vis.glinterface.GLPluginInterface` class allows plugins functions to draw,
process mouse and keyboard input, etc. Users are also welcome to use
Klamp't object OpenGL calls in their own frameworks. For more
information, see the :class:`~klampt.vis.glinterface.GLPluginInterface` documentation 
and the simple example file
``Klampt-examples/Python3/demos/gl_vis.py``.

For each GUI event (``display``, ``mousefunc``, etc), the event cascades through
the plugin stack until one plugin's handler catches it by returning
True. Note: when implementing a plugin, you should not call any handler
functions yourself. Instead, the GUI will call these in response to OS
events. As a result, ``GLPluginInterface`` handlers are run inside the
visualization thread, and will not need to call the ``vis.lock()`` and
``vis.unlock()`` functions to modify Klamp't objects.

**Handlers:**

-  ``plugin.initialize()``: called once when OpenGL has been initialized
-  ``plugin.displayfunc()``: called each refresh cycle. No OpenGL calls
   have been set up here.
-  ``plugin.display()``: called each refresh cycle, with the background
   cleared and the current 3D perspective camera viewport set.
-  ``plugin.display_screen()``: called each refresh cycle, with the
   OpenGL viewport aligned to the window in orthographic projection.
   Used to draw text.
-  ``plugin.reshapefunc(w,h)``: called when the user or OS resizes the
   window.
-  ``plugin.keyboardfunc(c,x,y)``: called when the user types character
   c with the mouse at (x,y).
-  ``plugin.keyboardupfunc(c,x,y)``: called when the keyboard character
   c is released with the mouse at (x,y).
-  ``plugin.mousefunc(button,state,x,y)``: called when the mouse is
   clicked or released, with a given button, state, and position (x,y)
-  ``plugin.motionfunc(x,y,dx,dy)``: called when the mouse is moved to
   (x,y) with delta (dx,dy) from its previous position.
-  ``plugin.idle()``: called when the GUI is not working.
-  ``plugin.eventfunc(type,args="")``: Generic hook for other events,
   e.g., button presses, from the GUI.  Currently not used.
-  ``plugin.closefunc()``: called before the viewport is closed.

**Configuration functions** (these may be called during plugin setup,
especially during initialize())

-  ``plugin.add_action(callback,short_name,key,description=None)``:
   Defines a new generic GUI action. The action will be available in a
   menu in Qt or as keyboard commands in GLUT.
-  ``plugin.reshape(w,h)``: Asks to resize the viewport.
-  ``plugin.idlesleep(seconds)``: Asks to sleep the idle function.
   Usually called in idle to approximate a fixed-time loop."""
-  ``plugin.modifiers()``: Retrieves a list of currently pressed
   keyboard modifiers, i.e., combinations of 'ctrl', 'shift', 'alt'.
-  ``plugin.refresh()``: Asks the GUI for a redraw. If you are animating
   something, this must be called in ``idle()``.
-  ``plugin.draw_text(point,text,size=12,color=None)``: Draws text of
   the given size and color at the point (x,y) or (x,y,z).
-  ``plugin.click_ray(x,y)``: Returns the world-space ray
   (source,direction) associated with the camera click at x,y.
-  ``plugin.viewport()``: Retrieves the Viewport instance associated
   with the window.

Drawing your own world
~~~~~~~~~~~~~~~~~~~~~~~

You can completely override the standard vis scene manager using your own plugin,
but you will be responsible for all UI and OpenGL drawing functions.
Klampt provides convenience plugin base classes that show worlds and simulations in
the :mod:`klampt.vis.glrobotprogram` module. 

The following code shows how to subclass the :class:`~klampt.vis.glrobotprogram.GLWorldPlugin`
class, as well as the *mousefunc* and *motionfunc* callbacks to capture objects clicked on
by the mouse.

.. code:: python

    import klampt
    from klampt import vis
    from klampt.vis.glrobotprogram import GLWorldPlugin

    class MyPlugin(GLWorldPlugin):
      def __init__(self,world):
        GLWorldPlugin.__init__(self,world)

      def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        #the current example prints out the list of objects clicked whenever
        #you right click
        print("mouse",button,state,x,y)
        if button==2:
          if state==0:
            print([o.getName() for o in self.click_world(x,y)])
            return
        GLWorldPlugin.mousefunc(self,button,state,x,y)

      def motionfunc(self,x,y,dx,dy):
        return GLWorldPlugin.motionfunc(self,x,y,dx,dy)
    
    world = klampt.WorldModel()
    if not world.readFile("Klampt-examples/data/athlete_plane.xml"):
      raise RuntimeError("Couldn't load world")
    vis.run(MyPlugin(world))


Compatibility
~~~~~~~~~~~~~

**OpenGL mode**

* Drawable: All of the items listed above, plus SimRobotSensor, SubRobotModel, and Appearance.
* Editable: RobotModel, RigidObjectModel, Vector3, RigidTransform, Config, coordinates.Point, coordinates.Frame, coordinates.Transform
* Quirks:
    * All items by default except for WorldModel, RobotModel, RigidObjectModel, and TerrainModel have their labels drawn.
    * "size" attribute of points is in pixels.
    * No shadow mapping.

**IPython mode**

* Drawable: all of the items listed above.
* Editable: RobotModel, RigidObjectModel, Vector3, RigidTransform, Config, coordinates.Point, coordinates.Frame, coordinates.Transform
* Quirks:
    * Hiding windows and re-showing windows is disabled.
    * ``vis.update()`` must be called to receive changes in world object configurations.
    * Does not support plugins.
    * Scene viewports are in a JSON structure format that is incompatible with OpenGL viewports. This will be changed in a future release.
    * Entities in the scene need to have unique identifiers, or else they will not be drawn.
    * Label drawing is disabled. 
    * Texture mapping doesn't work.
    * "size" attribute of points is in absolute units. 
    * RigidTransform doesn't support "fancy" mode.
    * Configs doesn't support "maxConfigs". 
    * RobotTrajectory does not show end effector trajectories.
    * Curves aren't drawn between coordinate frames and their parents.
    * setItemConfig doesn't update the visualization except for WorldModel items.  Workaround: either re-add the item, or use ``vis.nativeWindow().setTransform()``.

**HTML mode**

* Drawable: all of the items listed above.
* Editable: no user interaction is possible.
* Quirks: all the quirks from IPython mode, plus the caveats listed in the `klampt.vis documentation <klampt.vis.html#html>`__ .

The most compatible way to launch visualizations is to either use ``vis.run()``,
``vis.debug()``, or ``vis.loop()``.
