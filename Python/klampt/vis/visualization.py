"""Klamp't visualization routines.  See
`vistemplate.py in Klampt-examples <https://github.com/krishauser/Klampt-examples/Python/demos/vistemplate.py>`_
for an example of how to run this module.

OVERVIEW
--------

The visualization module lets you draw most Klamp't objects in a 3D world 
using a simple interface.  It also lets you customize the GUI using Qt 
widgets, OpenGL drawing, and keyboard/mouse intercept routines.  

Main features include:

- Simple interface to modify the visualization
- Simple interface to animate and render trajectories
- Simple interface to edit certain Klamp't objects (configurations, points,
  transforms)
- Simple interface to drawing text and text labels, and drawing plots
- Multi-window, multi-viewport support
- Automatic camera setup
- Unified interface to PyQt, GLUT, IPython, and HTML backends.

    - PyQT is the backend with the fullest amount of features.  
    - GLUT loses resource editing and advanced windowing functionality.
    - IPython loses plugins, resource editing, custom drawing, and advanced 
      windowing functionality.
    - HTML loses plugins, resource editing, custom drawing, and advanced 
      windowing functionality.

The resource editing functionality in the :mod:`klampt.io.resource` module
(which uses the widgets in :mod:`klampt.vis.editors`) use this module as well.


INSTRUCTIONS
-------------------

Basic use of the vis module is fairly straightforward:

0. (optional) Configure the rendering backend.
1. Add things to the visualization scene with ``vis.add(name,thing)``.  Worlds,
   geometries, points, transforms, trajectories, contact points, and more can
   be added in this manner.
2. Modify the appearance of things using modifier calls like
   ``vis.setColor(name,r,g,b,a)``.
3. Launch windows and/or visualization thread (OpenGL or IPython modes)
4. Continue adding, modifying, and removing things as you desire. 

More advanced functions allow you to dynamically launch multiple windows,
capture user input, and embed the visualization into Qt windows.

The default scene manager lets you set up and modify the visualization scene
(Steps 1 and 2).  These just mirror the methods in
:class:`VisualizationScene`, which is how the default scene manager is
implemented.  See Klampt-examples/Python/demos/vistemplate.py for more
examples.

The vis module provides some basic methods for capturing user interaction.
To add other user interaction functionality, you may create a
:class:`~klampt.vis.glinterface.GLPluginInterface` subclass to add
functionality on top of the default visualization world.  To do so, call
``vis.pushPlugin(plugin)``.  Note that custom rendering (the ``display()``
and ``display_screen()`` methods) are only available with the OpenGL
rendering backend.

Only one rendering backend can be chosen during the lifetime of your process,
and each backend has its own quirks with regards to window launching and
configuration.  We'll describe the different options below.


OpenGL (PyQt, GLUT)
~~~~~~~~~~~~~~~~~~~~

OpenGL-based visualizations use either PyQt or GLUT to handle windowing and are
used by default. They run in the current process, have the best performance,
and offer the richest set of features.  

Quick start
^^^^^^^^^^^^^^^

- To show the visualization and quit when the user closes the window::

        vis.run()

- To show the visualization and return when the user closes the window::

        vis.dialog()
        ... do stuff afterwards ... 
        vis.kill()   #cleanup, not strictly needed but good practice

- To show the visualization and run a script alongside it until the user
  closes the window (multithreaded mode)::
 
        vis.show()
        while vis.shown():
            vis.lock()
            ... do stuff ...
            [to exit the loop call vis.show(False)]
            vis.unlock()
            time.sleep(dt)
        ... do stuff afterwards ...
        vis.kill()   #cleanup, not strictly needed but good practice

- To show the visualization and run python commands until the user closes
  the window (single-threaded mode)::
 
        def callback():
            ... do stuff ...
            [to exit the loop manually call vis.show(False)]
        vis.loop(setup=vis.show,callback=callback)
        vis.kill()   #cleanup, not strictly needed but good practice

- To run a window with a custom plugin (GLPluginInterface) and terminate on
  closure::
 
        vis.run(plugin)
  
- To show a dialog or parallel window::

        vis.setPlugin(plugin)
        ... then call  
        vis.dialog()
        ... or
        vis.show()
        ... do stuff afterwards ... 
        vis.kill()

- To add a GLPluginInterface that just customizes a few things on top of
  the default visualization::

        vis.pushPlugin(plugin)
        vis.dialog()
        vis.popPlugin()

- To run plugins side-by-side in the same window::

        vis.setPlugin(plugin1)
        vis.addPlugin(plugin2)  #this creates a new split-screen
        vis.dialog()
        ... or
        vis.show()
        ... do stuff afterwards ... 
        vis.kill()

- To run a custom Qt window or dialog containing a visualization window::

        vis.setPlugin([desired plugin or None for visualization])
        def makeMyUI(qtglwidget):
            return MyQtMainWindow(qtglwidget)
        vis.customUI(makeMyUI)
        vis.dialog()   #if you return a QDialog
        ... or
        vis.show()     #if you return a QWidget or QMainWindow
        ... do stuff afterwards ... 
        vis.kill()

- To launch a second window after the first is closed: just call whatever you
  want again. Note: if show was previously called with a plugin and you wish to
  revert to the default visualization, you should call setPlugin(None) first to 
  restore the default.

- To create a separate window with a given plugin::

        w1 = vis.createWindow("Window 1")  #w1=0
        show()
        w2 = vis.createWindow("Window 2")  #w2=1
        vis.setPlugin(plugin)
        vis.dialog()
        #to restore commands to the original window
        vis.setWindow(w1)
        while vis.shown():
            ...
        vis.kill()


Implementation Details
^^^^^^^^^^^^^^^^^^^^^^

There are two primary modes of running OpenGL visualizations: multi-threaded 
and single-threaded.

- Multi-threaded mode pops up a window using :func:`show`, and the caller can 
  then continue to interact with the vis module.

  IMPORTANT: multi-threaded mode is only supported on some systems (Linux,
  Windows using Qt). Due to weird OpenGL and Qt behavior in multi-threaded
  programs, if you are using multithreaded mode, you should only interact
  with OpenGL and the visualization using the methods in this module. 
  Custom OpenGL calls can be implemented inside GLPluginInterface plugins
  and customDrawFunc.

- Single-threaded mode blocks the calling thread using :func:`loop`.  To 
  interact with the scene, the caller will provide callbacks that can modify 
  the visualization world, pop up windows etc.
  Single-threaded mode is the most compatible, and is the only mode that works
  with GLUT and Mac OS.

There are also some convenience functions that will work in both modes, such
as :func:`run`, :func:`spin`, and func:`dialog`.

.. note::

    In multithreaded mode, when changing the data shown by the window (e.g., 
    modifying the configurations of robots in a WorldModel) you must call
    ``vis.lock()`` before accessing the data and then call ``vis.unlock()``
    afterwards.

The biggest drawback of single-threaded operation is that you can only start
blocking dialogs at the outer-most level, not inside loop().  So if you have
a GUI that's running in real-time, in a multi-threaded visualization your
code can pop up a dialog (like an editor) and the continue running with the
returned value.  There are some workarounds in single-thread mode (providing
a callback to the dialog function) but these are not nearly as convenient. 

It is possible to start in single-threaded mode and convert to multi-threaded,
but the converse is not possible.

In OpenGL mode, you can also completely override the scene manager and run your
own OpenGL calls using a subclass of
:class:`~klampt.vis.glinterface.GLPluginInterface`.  Here, you will need to
perform all the necessary OpenGL drawing / interaction inside its hooks.
To use this, you should call ``vis.setPlugin(plugin)`` to override the default
visualization behavior before creating your window. See
Klampt-examples/Python/demos/visplugin.py for an example of this in use.


IPython (Jupyter notebook)
~~~~~~~~~~~~~~~~~~~~~~~~~~

IPython visualizations run in a Jupyter notebook in a web browser, using a
WebGL widget to render the content.  The Python code communicates with the 
browser upon certain calls to update the visualization. 

This mode will be enabled by default inside a Jupyter notebook, or if you first
call ``vis.init('IPython')``.  In the cell in which you want to show the WebGL
widget, call ``vis.show()``.  To show multiple widgets you can create new vis
windows, and use ``setWindow`` to switch between which widget you'd like
subsequent calls to modify.

The WebGL widget is updated automatically upon ``addX``, ``setColor``,
``clear``, and ``hide`` calls, but it can't tell when something changes in the
world, like a robot configuration or object transform.  When the state of
something in the world changes, you must manually make a call to
``vis.update()``.

.. note::

    For optimal performance when calling a lot of scene modifiers, you should
    use ``vis.lock() / unlock()`` calls to block off the start and end of
    scene modification. Doing so means that the WebGL widget
    is only re-rendered at the very end of your calls.  

    Note the semantics here are slightly different from the normal sense of
    locking / unlocking, which is to prevent thread clashes amongst changes to 
    the underlying object data.

This mode does NOT support plugins, dialogs, or custom draw functions.  Also,
certain types of geometries like VolumeGrids are not supported.

Animations are supported, but you will manually have to advance the animations 
and call ``vis.update()`` or ``vis.scene().update()`` for each frame.  
See :class:`~klampt.vis.ipython.widgets.Playback` for a convenient widget
that handles this somewhat automatically.


HTML
~~~~~~~~~~~~~~~~~~~~
This output mode outputs an HTML + Javascript page that uses WebGL to display
the scene.  Unlike other methods, this is not a live visualization that
interacts with your Python script, but rather you create a scene / animation
and then retrieve the HTML afterwards.  This mode is appropriate for Google
Colab, for example.  

To show the HTML page in an IPython notebook (e.g., Jupyter or Colab),
call ``vis.show()`` after the animation is done.  This will show the visualizer
in an IFrame, whose size is the current viewport size.

To use it with a specified time step, use the following::

    vis.add("world",world)
    dt = 1.0/30.0   #30fps
    while not done:
        ... change the world, add / remove things, etc ... 
        vis.stepAnimation(dt)
    vis.show()

To have it capture a real-time process, use the ``vis.update()`` function
instead, as follows::

    vis.add("world",world)
    while not done:
        ... change the world, add / remove things, etc ... 
        vis.update()
        time.sleep(1.0/3.0)
    vis.show()

(Don't mix animations and updates, or else the resulting animation will be
strange.)

You may also retrieve raw HTML, either by casting ``vis.nativeWindow()`` to
a str, calling ``vis.nativeWindow().iframe(width,height)``, or
``vis.nativeWindow().page()``.  In the first case, the HTML does not contain
the <html> or <body> tags, and takes up the whole screen.  The .iframe()
option places the code into an IFrame of a given size, and the page() option
wraps the code into a full HTML page.

For example, to turn a Trajectory ``traj`` into a WebGL animation, use this
code::

    vis.add("world",world)
    vis.animate(("world",world.robot(0).getName()),traj)
    t = 0
    while t < traj.endTime():
        vis.stepAnimation(dt)
        t += dt
    vis.show()

To turn a Simulator into a WebGL animation, use this code::

    sim = Simulator(world)
    vis.add("world",world)
    while not done:
        ... add forces, send commands to the robot, etc...
        sim.simulate(dt)
        vis.stepAnimation(dt)
    vis.show()


WINDOWING API
--------------

- :func:`debug`: a super easy way to visualize Klamp't items.
- :func:`init`: initializes the visualization.  Can configure here
  what backend(s) to use.
- :func:`createWindow`: creates a new visualization window and returns an
  integer identifier.
- :func:`setWindow`: sets the active window for all subsequent calls.  ID 0 is
  the default visualization window.
- :func:`getWindow`: gets the active window ID.
- :func:`nativeWindow`: returns the current window object used by the backend.
- :func:`setWindowTitle`: sets the title of the visualization window.
- :func:`getWindowTitle`: returns the title of the visualization window.
- :func:`resizeWindow`: resizes the window.  For OpenGL, this can be done
  after the window is shown. Otherwise, it must take place before showing.
- :func:`scene`: returns the current :class:`VisualizationScene`
- :func:`setPlugin`: sets the current plugin (a
  :class:`~klampt.vis.glinterface.GLPluginInterface` instance).  This plugin
  will now capture input from the visualization and can override any of the
  default behavior of the visualizer. Set plugin=None if you want to return to
  the default visualization.
- :func:`pushPlugin`: adds a new plugin (e.g., to capture input) on top of
  the old one.
- :func:`splitView`: adds a second scene / viewport to the current
  window. If a plugin is provided (a :class:`GLPluginInterface` instance) then
  the new view is set to use this plugin.
- :func:`run`: pops up a dialog and then kills the program afterwards.
- :func:`kill`: kills all previously launched visualizations and terminates the
  visualization thread. Afterwards, you may not be able to start new windows.
  Call this to cleanly quit.
- :func:`multithreaded`: returns true if multithreading is available.
- :func:`loop`: Runs the visualization
  thread inline with the main thread.  The setup() function is called at the
  start, the callback() function is run every time the event thread is idle,
  and the cleanup() function is called on termination.

  NOTE FOR MAC USERS: having the GUI in a separate thread is not supported on
  Mac, so the ``loop`` function must be used rather than ``show``/``spin``.

  NOTE FOR GLUT USERS: this may only be run once.
- :func:`dialog`: pops up a dialog box (does not return to calling thread until
  closed).  
- :func:`show`: shows/hides a visualization window.  If not called 
  from the visualization loop, a new visualization thread is run in parallel
  with the calling script. 
- :func:`spin`: shows the visualization window for the desired amount
  of time before returning, or until the user closes the window.
- :func:`shown`: returns true if the window is shown.
- :func:`lock`: locks the visualization scene for editing.  The visualization 
  will be paused until unlock() is called.
- :func:`unlock` unlocks the visualization world.  Must only be called once
  after every lock().
- :func:`update`: manually triggers a redraw of the current scene.
- :func:`threadCall`: Call a user-defined function inside the visualization
  thread. Advanced users may wish to inject calls that are incompatible with 
  being run outside the Qt or OpenGL thread.
- :func:`customUI`: launches a user-defined UI window with the OpenGL window
  embedded into it (Qt only).

SCENE MODIFICATION API
------------------------

The following methods operate on the current scene (as returned from
:func:`scene`). 

Objects accepted by :func:`add` include:

- text (str)
- Vector3 (3-list)
- Matrix3 (:mod:`klampt.math.so3` item)
- RigidTransform (:mod:`klampt.math.se3` item)
- Config (n-list): n must match the number of links of a RobotModel in "world"
- Configs (list of n-lists)
- :class:`Trajectory` and its subclasses
- polylines (use :class:`Trajectory` type)
- :class:`WorldModel`
- :class:`RobotModel`
- :class:`RobotModelLink`
- :class:`RigidObjectModel`
- :class:`TerrainModel`
- :class:`Geometry3D`
- :class:`GeometricPrimitive`
- :class:`PointCloud`
- :class:`TriangleMesh`
- :class:`Simulator`
- :class:`ContactPoint`
- objects from the :mod:`~klampt.model.coordinates` module.

See func:`setAttribute` for a list of attributes that can be used to customize
an object's appearance. Common attributes include ``color``, ``hide_label``,
``type``, ``position`` (for text) and ``size`` (for points).

In OpenGL modes and IPython mode, many objects can be edited using the
:func:`edit` function.  In OpenGL, this will provide a visual editing widget,
while in IPython this will pop up IPython widgets.

If you are modifying the internal data of an object in an external loop
(as opposed to inside a plugin) be sure to call :func:`lock`/:func:`unlock` 
before/after doing so to prevent the visualization from accessing the
object's data .

Scene management functions
~~~~~~~~~~~~~~~~~~~~~~~~~~

- :func:`add`: adds an item to the 
  visualization.  name is a unique identifier.  If an item with the same name
  already exists, it will no longer be shown.  Keyword attributes can be
  given to customize the appearance of the object (see :func:`setAttribute`.)
- :func:`clear`: clears the visualization world.
- :func:`listItems`: prints out all names of visualization objects in the scene
  or under a given object
- :func:`getItemName`: retrieves the name / path of a given object in the
  scene, or returns None if the object doesnt exist.
- :func:`dirty`: marks the given item as dirty and recreates the OpenGL display
  lists.  You may need to call this if you modify an item's geometry, for
  example.
- :func:`remove`: removes an item from the visualization.
- :func:`setItemConfig`: sets the configuration of a named item.
- :func:`getItemConfig`: returns the configuration of a named item.
- :func:`hide`: hides/unhides an item.  The item is not removed, it just
  becomes invisible.
- :func:`hideLabel`: hides/unhides an item's text label.
- :func:`setLabel`: changes an item's text label from its name to a custom
  string.
- :func:`setAppearance`: changes the Appearance of an item.
- :func:`revertAppearance`: restores the Appearance of an item
- :func:`setAttribute`: sets an attribute to change an item's appearance.
- :func:`getAttribute`: gets an attribute of an item's appearance.
- :func:`getAttributes`: gets all relevant attributes of an item's appearance.
- :func:`setColor`: changes the color of an item.
- :func:`setDrawFunc`: sets a custom OpenGL drawing function for an item.


Animation functions
~~~~~~~~~~~~~~~~~~~~~~~~~~

- :func:`animate` Starts an animation on an item. The animation be a
  :class:`Trajectory` or a list of configurations.  Works with points, so3
  elements, se3 elements, rigid objects, or robots. 
- :func:`pauseAnimation`: Turns animation on/off.
- :func:`stepAnimation`: Moves forward the animation time by the given 
  amount, in seconds.
- :func:`animationTime`: Gets/sets the current animation time
- :func:`setTimeCallback`: sets a function that will return the global time
   used by the animation timing, visualization plots, and movie-saving
   functions.  This must be monotonically non-decreasing.

Text and plots
~~~~~~~~~~~~~~~~

Like other items in the visualization scene, text and plots are referred to by
string identifiers. 

Text is usually attached to 2D pixel coordinates, but in OpenGL mode can also
be attached to 3D points.  Use the ``position`` attribute to control where the
text is located and the ``size`` attribute to control its size.

Plots are added to the scene and then items are added to the plot.  The
configuration of a visualization item is then shown as a live display (OpenGL).
You may also log custom numeric data with :func:`logPlot` and event data using
:func:`logPlotEvent`.

- :func:`addText`: adds text to the visualizer.
- :func:`clearText`: clears all previously added text.
- :func:`addPlot`: creates a new empty plot.
- :func:`addPlotItem`: adds a visualization item to a plot.
- :func:`logPlot`: logs a custom visualization item to a plot
- :func:`logPlotEvent`: logs an event on the plot.
- :func:`hidePlotItem`: hides an item in the plot. 
- :func:`setPlotDuration`: sets the plot duration.
- :func:`setPlotRange`: sets the y range of a plot.
- :func:`setPlotPosition`: sets the upper left position of the plot on the screen.
- :func:`setPlotSize`: sets the width and height of the plot.
- :func:`savePlot`: saves a plot to a CSV (extension .csv) or Trajectory 
  (extension .traj) file.

User Interaction
~~~~~~~~~~~~~~~~

- :func:`edit`: turns on/off visual editing of some item.  Points, transforms,
  ``coordinates.Point``, ``coordinates.Transform``, ``coordinates.Frame``,
  :class:`RobotModel`, and :class:`RigidObjectModel` are currently accepted.
- :func:`pick`: asks the user to click on an item in the scene. 
- :func:`addAction`: adds a keyboard shortcut (GLUT) / menu items (Qt) /
  buttons (IPython).
- :func:`addButton`: adds a button to the view.
- :func:`addSelect`: adds a selection dropdown or listbox.
- :func:`addInput`: adds a string, integer, or float input box.
- :func:`addWidget`: adds a widget(s) from a JSON description.

Global appearance / camera control functions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- :func:`getViewport`: Returns the :class:`GLViewport` for the currently active
  view.
- :func:`setViewport`: Sets the :class:`GLViewport` for the currently active
  scene.  (This may also be used to resize windows.)
- :func:`setBackgroundColor`: Sets the background color for the active
  view.
- :func:`autoFitCamera`: Automatically fits the camera to all objects in the
  visualization.  A scale > 1 magnifies the zoom.
- :func:`followCamera`: Sets the camera to follow a target.
- :func:`saveJsonConfig`: Saves the configuration to a JSON object or JSON
  file.
- :func:`loadJsonConfig`: Loads the configuration from a JSON object or JSON
  file.
- :func:`screenshot`: returns a screenshot of the scene.  Can retrieve depth,
  as well.
- :func:`screenshotCallback`: sets a callback that will receive a screenshot
  of the scene after rendering is done.

Utility functions
~~~~~~~~~~~~~~~~~

- :func:`objectToVisType` Auto-determines a type of object compatible with the
  visualizer.
- :func:`autoFitViewport`: Automatically fits a :class:`GLViewport` to see all
  the given objects.


NAMING CONVENTION
-----------------

The world, if one exists, should be given the name 'world'.  Configurations and
paths are drawn with reference to the first robot in the world.

All items that refer to a name (except add) can either be given a top level
item name (a string) or a sub-item (a sequence of strings, given a path from
the root to the leaf). For example, if you've added a RobotWorld under the
name 'world' containing a robot called 'myRobot', then::

    vis.setColor(('world','myRobot'),0,1,0)

will turn the robot green.  If 'link5' is the robot's 5th link, then::

    vis.setColor(('world','myRobot','link5'),0,0,1)

will turn the 5th link blue.

A shortcut is to use the :func:`getItemName` function on the given robot,
e.g.::

    robot = world.robot(0)
    vis.setColor(vis.getItemName(robot),0,0,1)

If there's a Simulator instance added to the scene under the name 'sim' or
'simulator', the animation timing for Qt movie saving  and HTML animations
will follow the simulation time rather than wall clock time.

"""


import threading
from ..robotsim import *
from ..math import vectorops,so3,se3
from . import gldraw
from . import glinit
from .glinterface import GLPluginInterface
from .glprogram import GLPluginProgram
from .glviewport import GLViewport
from . import glcommon
import time
import signal
import weakref
import sys
from ..model import types
from ..model import config
from ..model import coordinates
from ..model.subrobot import SubRobotModel
from ..model.trajectory import *
from ..model.multipath import MultiPath
from ..model.contact import ContactPoint,Hold
from ..model.collide import bb_empty,bb_create,bb_union
from typing import Callable,Union,Sequence
import warnings

#Type for references to items in a visualization world
ItemPath = Union[str,Sequence[str]]

#the global lock for all visualization calls
_globalLock = threading.RLock()
#the chosen backend
_backend = None
#the _WindowManager instance
_window_manager = None
#the OpenGL module (will be taken from glinit.init())
GL = None

def _isnotebook():
    try:
        shell = get_ipython().__class__.__name__
        if shell == 'ZMQInteractiveShell':
            return True   # Jupyter notebook or qtconsole
        elif shell == 'TerminalInteractiveShell':
            return False  # Terminal running IPython
        elif shell == 'Shell':
            if 'google.colab' in sys.modules:
                return True
            return False  # Other type (?)
        else:
            return False  # Other type (?)
    except NameError:
        return False      # Probably standard Python interpreter

def init(backends=None):
    """Initializes the vis module using some visualization backend.  `backends`
    can be None, in which case it tries using PyQt, then GLUT, then IPython in
    that order.  It can also be a string or list of strings from the following
    set:

    - 'PyQt': uses PyQT + OpenGL
    - 'PyQt4' / 'PyQt5': uses a specific version of PyQT
    - 'GLUT': uses GLUT + OpenGL
    - 'IPython': uses an IPython widget
    - 'HTML': outputs an HTML / Javascript widget

    """
    global _backend,_window_manager,GL
    if backends is None:
        if _isnotebook():
            if 'google.colab' in sys.modules:
                backends = ['HTML']
            else:
                backends = ['IPython']
        else:
            backends = ['PyQt','GLUT','HTML']
    if isinstance(backends,str):
        backends = [backends]
    if _backend is not None:
        #already initialized
        if _backend in backends or _backend[:4] in backends:
            if _backend in ['IPython','HTML']:
                #force a reset for IPython / HTML
                _window_manager.reset()
            return _backend
        print("vis.init(): Trying to reset from backend",_backend,"to one of",backends)
        _window_manager.reset()
        _window_manager = None
        _backend = None
    
    OpenGLBackends = ['PyQt','PyQt4','PyQt5','GLUT']
    order = [[]]
    for backend in backends:
        if backend in ['IPython','HTML']:
            order.append(backend)
            order.append([])
        else:
            order[-1].append(backend)
    for trials in order:
        if trials == 'IPython':
            _backend = 'IPython'
            from .backends import vis_ipython
            _window_manager = vis_ipython.IPythonWindowManager()
            return _backend
        elif trials == 'HTML':
            _backend = 'HTML'
            from .backends import vis_html
            _window_manager = vis_html.HTMLWindowManager()
            return _backend
        elif len(trials)>0:
            res = glinit.init(trials)
            GL = glinit.GL()
            if res is not None:
                _backend = glinit.active()
                if glinit.active() == 'GLUT':
                    from .backends import vis_glut
                    _window_manager = vis_glut.GLUTWindowManager()
                    warnings.warn("klampt.visualization: QT is not available, falling back to poorer\nGLUT interface.  Returning to another GLUT thread will not work\nproperly.")
                else:
                    from .backends import vis_qt
                    _window_manager = vis_qt.QtWindowManager()

                import atexit
                atexit.register(kill)
                return res
    return None

def _init():
    global _backend
    if _backend is not None:
        return
    if init() is None: 
        raise RuntimeError("Unable to initialize visualization")

def debug(*args,**kwargs):
    """A super easy way to visualize Klamp't items.

    The argument list can be a list of Klamp't items, and can also include
    strings or dicts.  If a string precedes an item, then it will be labeled
    by the string.  If a dict follows an item, the dict will specify 
    attributes for the item.  It can also contain the 'animation' key, in
    which case it should contain a Trajectory animating the item.

    Keyword arguments may include:

    - title: the window title
    - animation: if only one item is given, sets a looping animation
    - centerCamera: the name of the item that the camera should look at, or
      True to center on the whole scene.
    - followCamera: the name of the item that the camera will follow if
      animated, or None (default).
    - dialog: True if a dialog should be shown (default), False if a standard
      show() should be used.
    - anything else: Treated as named klamp't item.

    """
    global _backend
    _init()
    oldWindow = getWindow()
    if oldWindow is None:
        oldWindow = 0
    myWindow = createWindow()
    nextName = None
    lastName = None
    itemcount = 0
    if 'world' in kwargs:
        add('world',kwargs['world'])
        del kwargs['world']
    animationDuration = 0
    for i,arg in enumerate(args):
        if isinstance(arg,str):
            nextName = arg
        elif isinstance(arg,dict):
            if lastName is None:
                warnings.warn("vis.debug(): dict of attributes must follow an object")
                continue
            for (k,v) in arg.items():
                if k == 'animation':
                    animate(lastName,v)
                    animationDuration = max(animationDuration,v.endTime())
                else:
                    try:
                        setAttribute(lastName,k,v)
                    except Exception:
                        warnings.warn("vis.debug(): Couldn't set attribute {} of item {}".format(k,lastName))
        else:
            label = None
            if nextName is None:
                name = None
                if hasattr(arg,'getName'):
                    try:
                        name = arg.getName()
                    except Exception:
                        pass
                if hasattr(arg,'name') and isinstance(arg.name,str):
                    name = arg.name
                if name is None:
                    try:
                        type = types.object_to_type(arg)
                        name = type + '['+str(itemcount)+']'
                    except ValueError:
                        name = 'item['+str(itemcount)+']'
            else:
                name = nextName
                label = name
            add(name,arg)
            itemcount += 1
            lastName = name
            nextName = None

    title = None
    doDialog = True
    centerCamera = None
    followCameraItem = None
    animation = None
    for k,v in kwargs.items():
        if k=='title':
            title = v
        elif k=='world':
            pass
        elif k=='dialog':
            doDialog = v
        elif k=='animation':
            animation = v
        elif k=='followCamera':
            followCameraItem = v
        elif k=='centerCamera':
            centerCamera = v
        else:
            add(k,v)
            lastName = k
            itemcount += 1
    if title is not None:
        setWindowTitle(title)
    else:
        setWindowTitle("Klampt debugging: "+','.join(scene().items.keys()))
    if animation is not None:
        animate(lastName,animation)
    if centerCamera is True:
        autoFitCamera(rotate=False)
    elif centerCamera:
        if isinstance(centerCamera,int):
            centerCamera = 'item['+str(centerCamera)+']'
        elif not isinstance(centerCamera,(str,tuple)):
            centerCamera = getItemName(centerCamera)
        if centerCamera is None:
            warnings.warn("vis.debug(): could not center camera, invalid object name")
        else:
            vp = getViewport()
            try:
                autoFitViewport(vp,[scene().getItem(centerCamera)],rotate=False)
                setViewport(vp)
            except Exception:
                warnings.warn("vis.debug(): Centering camera failed")
                import traceback
                traceback.print_exc()
    if followCameraItem is not None:
        if followCameraItem is True:
            followCamera(lastName,center=True)
        else:
            if not isinstance(followCameraItem,(str,tuple)):
                followCameraItem = getItemName(followCameraItem)
            if followCameraItem is None:
                warnings.warn("vis.debug(): could not follow camera, invalid object name")
            followCamera(followCameraItem,center=True)
    if _backend == 'HTML':
        #dump out the animation
        if animation is not None:
            animationDuration = max(animationDuration,animation.endTime())
        if animationDuration > 0:
            dt = 1.0/30.0
            t = 0
            while t < animationDuration:
                stepAnimation(dt)
                t += dt
        show()
        setWindow(oldWindow)
    elif _backend == 'IPython':
        #setup a Playback widget from the animation
        if animation is not None:
            animationDuration = max(animationDuration,animation.endTime())
        if animationDuration > 0:
            framerate = 30
            my_scene = scene()
            def advance():
                my_scene.stepAnimation(1.0/framerate)
                my_scene.update()
            def reset():
                my_scene.animationTime(0)
            from .ipython.widgets import Playback
            playback = Playback(nativeWindow(),advance=advance,reset=reset,maxframes=int(animationDuration*framerate),framerate=framerate)
            from IPython.display import display
            display(playback)
        show()
    else:
        if doDialog:
            dialog()
            setWindow(oldWindow)
        else:
            show()
            #open ended...
    


def nativeWindow():
    """Returns the active window data used by the backend.  The result will be
    a subclass of :class:`~klampt.vis.glprogram.GLPluginProgram` if OpenGL is
    used (PyQt or GLUT) or a :class:`~klampt.vis.ipython.widgets.KlamptWidget`
    """
    global _window_manager
    if _window_manager is None:
        return None
    return _window_manager.frontend()

def scene() -> "VisualizationScene":
    """Returns the active window data used by the backend.  The result will be
    a subclass of :class:`VisualizationScene`.
    """
    global _window_manager
    if _window_manager is None:
        return None
    return _window_manager.scene()

def createWindow(title=None) -> int:
    """Creates a new window (and sets it active).

    Returns:
        int: an identifier of the window (for use with :func:`setWindow`).
    """
    global _globalLock,_window_manager
    _init()
    with _globalLock:
        id = _window_manager.createWindow(title)
    return id

def setWindow(id : int) -> None:
    """Sets the currently active window. 

    Note:
        ID 0 is the default visualization window.
    """
    global _globalLock,_window_manager
    _init()
    with _globalLock:
        _window_manager.setWindow(id)

def getWindow() -> int:
    """Retrieves ID of currently active window or -1 if no window is active"""
    global _window_manager
    _init()
    return _window_manager.getWindow()

def setPlugin(plugin: GLPluginInterface) -> None:
    """Lets the user capture input via a glinterface.GLPluginInterface class.
    Set plugin to None to disable plugins and return to the standard
    visualization.

    Args:
        plugin (GLPluginInterface): a plugin that will hereafter capture input
            from the visualization and can override any of the default behavior
            of the visualizer. Can be set to None if you want to return to the
            default visualization.
    """
    global _globalLock,_window_manager
    _init()
    with _globalLock:
        _window_manager.setPlugin(plugin)

def pushPlugin(plugin: GLPluginInterface) -> None:
    """Adds a new plugin on top of the old one.

    Args:
        plugin (GLPluginInterface): a plugin that will optionally intercept GUI
            callbacks. Unhandled callbacks will be forwarded to the next plugin
            on the stack.

    """
    global _globalLock,_window_manager
    _init()
    with _globalLock:
        _window_manager.pushPlugin(plugin)

def popPlugin() -> None:
    """Reverses a prior pushPlugin() call"""
    global _window_manager
    _init()
    with _globalLock:
        _window_manager.popPlugin()

def splitView(plugin : GLPluginInterface=None) -> None:
    """Adds a second OpenGL viewport in the same window, governed by the given
    plugin.

    Args:
        plugin (GLPluginInterface): the plugin used for the second viewport.
            If None, the new viewport will have the default visualization
            plugin.

    """
    global _window_manager
    _init()
    with _globalLock:
        _window_manager.splitView(plugin)

def addPlugin(plugin : GLPluginInterface=None) -> None:
    """Adds a second OpenGL viewport in the same window, governed by the given
    plugin.  DEPRECATED: use :func:`splitView` instead.

    Args:
        plugin (GLPluginInterface): the plugin used for the second viewport.
            If None, the new viewport will have the default visualization
            plugin.

    """
    warnings.warn("{} will be deprecated in favor of {} in a future version of Klampt".format('addPlugin','splitView'),DeprecationWarning)
    splitView(plugin)

def run(plugin : GLPluginInterface=None) -> None:
    """A blocking call to start a single window and then kill the visualization
    once the user closes the window. 

    Args:
        plugin (GLPluginInterface, optional): If given, the plugin used to handle all
            rendering and user input.  If plugin is None, the default visualization is
            used. 

    Note:
        Works in both multi-threaded and single-threaded mode.
    """
    global _window_manager
    _init()
    if plugin is not None:
        setPlugin(plugin)
    _window_manager.run()
    kill()

def multithreaded() -> bool:
    """Returns true if the current GUI system allows multithreading.  Useful for apps
    that will work cross-platform with Macs and systems with only GLUT.
    """
    global _window_manager
    _init()
    return _window_manager.multithreaded()

def dialog():
    """A blocking call to start a single dialog window with the current plugin.  It is
    closed by pressing OK or closing the window."""
    global _window_manager
    _init()
    return _window_manager.dialog()

def setWindowTitle(title : str) -> None:
    global _window_manager
    _init()
    _window_manager.setWindowName(title)

def getWindowTitle() -> str:
    global _window_manager
    return _window_manager.getWindowName()

def resizeWindow(w : int, h : int):
    """Resizes the current window.  For OpenGL, this can be done after the
    window is shown. Otherwise, it must take place before showing."""
    global _window_manager
    return _window_manager.resizeWindow(w,h)

def kill() -> None:
    """This should be called at the end of the calling program to cleanly terminate the
    visualization thread"""
    global _backend,_window_manager
    if _backend is None:
        return
    _window_manager.kill()
    _window_manager = None
    _backend = None

def loop(setup : Callable=None, callback : Callable=None, cleanup : Callable=None) -> None:
    """Runs the visualization thread inline with the main thread.
    The setup() function is called at the start, the callback() function is run
    every time the event thread is idle, and the cleanup() function is called
    on termination.

    NOTE FOR MAC USERS: a multithreaded GUI is not supported on Mac, so the loop()
    function must be used rather than "show and wait".

    NOTE FOR GLUT USERS: this may only be run once.
    """
    global _window_manager
    _init()
    _window_manager.loop(setup,callback,cleanup)

def show(display : bool=True) -> None:
    """Shows or hides the current window.

    NOTE FOR MAC USERS: due to a lack of support of multithreading on Mac, this
    will not work outside of the setup / callback / cleanup functions given in a
    call to loop().
    """
    global _window_manager
    _init()
    with _globalLock:
        if display:
            _window_manager.show()
        else:
            _window_manager.hide()

def spin(duration : float) -> None:
    """Spin-shows a window for a certain duration or until the window is closed."""
    global _window_manager
    _init()
    _window_manager.spin(duration)

def lock() -> None:
    """Begins a locked section.  Needs to be called any time you modify a
    visualization item outside of the visualization thread.  unlock() must be
    called to let the visualization thread proceed.
    """
    global _window_manager
    _window_manager.lock()

def unlock() -> None:
    """Ends a locked section acquired by lock()."""
    global _window_manager
    _window_manager.unlock()

def update() -> None:
    """Manually triggers a redraw of the current window."""
    global _window_manager
    _window_manager.update()

def shown() -> bool:
    """Returns true if a visualization window is currently shown."""
    global _globalLock,_window_manager
    _init()
    with _globalLock:
        res = _window_manager.shown()
    return res

def customUI(func : Callable) -> None:
    """Tells the next created window/dialog to use a custom UI function. Only
    available in PyQT mode.

    This is used to build custom editors and windows that are compatible with
    other UI functionality.  

    Args:
        func (function): a 1-argument function that takes a configured Klamp't 
            QtWindow as its argument and returns a QDialog, QMainWindow, or
            QWidget.

    (Could also be used with GLUT, but what would you do with a GLUTWindow?)
    """
    global _globalLock,_window_manager
    _init()
    with _globalLock:
        _window_manager.set_custom_ui(func)

def threadCall(func : Callable) -> None:
    """Call `func` inside the visualization thread. This is useful for some
    odd calls that are incompatible with being run outside the Qt or OpenGL
    thread.

    Possible use cases include performing extra OpenGL draws for camera
    simulation.
    """
    global _globalLock,_window_manager
    with _globalLock:
        _window_manager.threadCall(func)


######### CONVENIENCE ALIASES FOR VisualizationScene methods ###########
def addAction(hook : Callable, short_text : str, key : str=None, description : str=None) -> None:
    """Adds a callback to the window that can be triggered by menu choice or
    keyboard. Alias for nativeWindow().addAction().

    Args:
        hook (function): a python callback function, taking no arguments, called
            when the action is triggered.
        short_text (str): the text shown in the menu bar.
        key (str, optional): a shortcut keyboard command (e.g., can be 'k' or 'Ctrl+k').
        description (str, optional): if provided, this is a tooltip that shows up
            when the user hovers their mouse over the menu item.
    """
    _init()
    nativeWindow().addAction(hook,short_text,key,description)

def clear() -> None:
    """Clears the visualization world."""
    if _backend is None:
        return
    scene().clear()

def add(name : str, item,keepAppearance=False,**kwargs) -> None:
    """Adds an item to the visualization.

    Args:
        name (str): a unique identifier.  If an item with the same name already
            exists, it will no longer be shown. 
        keepAppearance (bool, optional): if True, then if there was an item that
            had the same name, the prior item's appearance will be kept.
        kwargs: key-value pairs to be added into the attributes dictionary.  e.g.
            vis.add("geom",geometry,color=[1,0,0,1]) adds a geometry while setting
            its color to red.
    """
    _init()
    scene().add(name,item,keepAppearance,**kwargs)
    
def listItems(name=None,indent=0) -> None:
    _init()
    scene().listItems(name,indent)

def getItemName(object) -> ItemPath:
    """Retrieves the name / path of a given object in the scene, or returns
    None if the object doesnt exist.
    """
    if _backend is None:
        return None
    return scene().getItemName(object)

def dirty(item_name : str='all') -> None:
    """Marks the given item as dirty and recreates the OpenGL display lists. 
    You may need to call this if you modify an item's geometry, for example. 
    
    If things start disappearing from your world when you create a new window,
    you may need to call this too.
    """
    scene().dirty(item_name)

def animate(name : ItemPath, animation, speed : float=1.0, endBehavior : str='loop') -> None:
    """Sends an animation to the named object.
    Works with points, so3 elements, se3 elements, rigid objects, or robots, and may work
    with other objects as well.

    Args:
        name (str): the named object
        animation: may be a Trajectory or a list of configurations.
        speed (float, optional): a modulator on the animation speed.  If the animation
            is a list of milestones, it is by default run at 1 milestone per second.
        endBehavior (str, optional): either 'loop' (animation repeats forever) or 'halt'
            (plays once).

    """
    scene().animate(name,animation,speed,endBehavior)

def pauseAnimation(paused=True) -> None:
    """Pauses or unpauses the animation."""
    scene().pauseAnimation(paused)

def stepAnimation(amount : float) -> None:
    """Moves forward the animation time by ``amount``, given in seconds"""
    scene().stepAnimation(amount)

def animationTime(newtime : float=None) -> None:
    """Gets/sets the current animation time

    If newtime is None (default), this gets the animation time.

    If newtime is not None, this sets a new animation time.
    """
    return scene().animationTime(newtime)

def setTimeCallback(timefunc : Callable=None) -> None:
    """Sets a function that will return the window's global time.  This
    will be used by the animation timing, visualization plots, and movie-saving
    functions.
    
    Args:
        timefunc (callable): returns a monotonically non-decreasing float.
            If None, reverts back to using time.time().
    """
    _init()
    scene().setTimeCallback(timefunc)
    

def remove(name : ItemPath) -> None:
    """Removes an item from the visualization"""
    return scene().remove(name)

def getItemConfig(name : ItemPath) -> Sequence[float]:
    """Returns a configuration of an item from the visualization.  Useful for 
    interacting with edited objects.

    Returns:
        list: a list of floats describing the item's current configuration.  Returns
            None if name doesnt refer to an object."""
    return scene().getItemConfig(name)

def setItemConfig(name : ItemPath, value : Sequence[float]) -> None:
    """Sets a configuration of an item from the visualization.

    Args:
        name (str): the item to set the configuration.
        value (list of floats): the item's configuration.  The number of items
            depends on the object's type.  See the config module for more information.

    """
    return scene().setItemConfig(name,value)

def setLabel(name : ItemPath, text : str) -> None:
    """Changes the label of an item in the visualization"""
    setAttribute(name,"label",text)

def hideLabel(name : ItemPath, hidden=True) -> None:
    """Hides or shows the label of an item in the visualization"""
    return scene().hideLabel(name,hidden)

def hide(name : ItemPath, hidden=True) -> None:
    """Hides an item in the visualization.  

    Note: the opposite of hide() is not show(), it's hide(False).
    """
    scene().hide(name,hidden)

def edit(name : ItemPath, doedit=True) -> None:
    """Turns on/off visual editing of some item. 

    In OpenGL mode, currently accepts items of type:

    - Vector3 (3-list)
    - Matrix3 (9-list)
    - Config (n-list, where n is the # of robot links)
    - RigidTransform (se3 object)
    - :class:`~klampt.robotsim.RobotModel`
    - :class:`~klampt.robotsim.RigidObjectModel`
    - :class:`~klampt.model.coordinates.Point`
    - :class:`~klampt.model.coordinates.Transform`
    - :class:`~klampt.model.coordinates.Frame`

    In IPython mode, currently accepts items of type:

    - Vector3 (3-lists)
    - Config (n-list, where n is the # of robot links)
    - RigidTransform (se3 objects)
    - :class:`~klampt.robotsim.RobotModel`
    - :class:`~klampt.robotsim.RigidObjectModel`
    - :class:`~klampt.model.coordinates.Point`
    - :class:`~klampt.model.coordinates.Transform`
    - :class:`~klampt.model.coordinates.Frame`

    Returns:
        Varies: None if doedit=False, otherwise, an editor object which
        depends on the backend.  In OpenGL mode, returns a Widget. In
        Jupyter, returns a VBox containing controls.
    """
    return scene().edit(name,doedit)

def pick(click_callback : Callable, hover_callback : Callable=None,
        highlight_color=(1,1,0,0.3), filter : Callable=None, tolerance=0.01) -> None:
    """Picks an item from the scene.  ``click_callback`` is called once the 
    user left-clicks on an item.  ``hover_callback`` is called when a user
    hovers over an item. 
    
    Once the user clicks, callbacks are disabled. To re-enable callbacks, call
    pick() again.

    Args:
        click_callback (callable): a function f(name,item,point) that receives 
            the name of the item, the item itself, and the point clicked on, 
            in world coordinates.  Set this to None to disable picking.
        hover_callback (callable): a function f(name,item,point) that receives 
            the name of the item, the item itself, and the point hovered over, 
            in world coordinates.  Set this to None to disable picking.
        highlight_color (tuple, optional): if given, hovered items will be
            colored with the given color.
        filter (callable, optional): a function f(name,item) that returns True
            if the item should be included in the selection.
        tolerance (float, optional): a collision tolerance for non-volumetric
            objects.
    """
    scene().pick(click_callback,hover_callback,highlight_color,filter,tolerance)

def setAppearance(name : ItemPath, appearance : Appearance) -> None:
    """Changes the Appearance of an item, for an item that uses the Appearance
    item to draw (config, geometry, robots, rigid bodies).
    """
    scene().setAppearance(name,appearance)

def setAttribute(name : ItemPath, attr : str, value) -> None:
    """Sets an attribute of an item's appearance.

    Args:
        name (str): the name of the item
        attr (str): the name of the attribute (see below)
        value: the value (see below)

    Accepted attributes are:

    - 'robot': the index of the robot associated with this (default 0)
    - 'color': the item's color (r,g,b) or (r,g,b,a)
    - 'size': the size of the plot, text, point, ContactPoint, or IKObjective
    - 'length': the length of axes in RigidTransform, or normal in ContactPoint
    - 'width': the width of axes and trajectory curves
    - 'duration': the duration of a plot
    - 'pointSize': for a trajectory, the size of points (default None, set to 0
      to disable drawing points)
    - 'pointColor': for a trajectory, the size of points (default None)
    - 'endeffectors': for a RobotTrajectory, the list of end effectors to plot
      (default the last link).
    - 'maxConfigs': for a Configs resource, the maximum number of drawn
      configurations (default 10)
    - 'fancy': for RigidTransform objects, whether the axes are drawn with
      boxes or lines (default False)
    - 'type': for ambiguous items, like a 3-item list when the robot has 3
      links, specifies the type to be used.  For example, 'Config' draws the
      item as a robot configuration, while 'Vector3' or 'Point' draws it as a
      point.
    - 'label': a replacement label (str)
    - 'hide_label': if True, the label will be hidden

    """
    scene().setAttribute(name,attr,value)

def getAttribute(name : ItemPath, attr : str):
    """Gets an attribute of an item's appearance. If not previously set by the
    user, the default value will be returned.

    Args:
        name (str): the name of the item
        attr (str): the name of the attribute (see :func:`setAttribute`)
    """
    return scene().getAttribute(name,attr)

def getAttributes(name : ItemPath) -> dict:
    """Gets a dictionary of all relevant attributes of an item's appearance. 
    If not previously set by the user, default values will be returned.

    Args:
        name (str): the name of the item
    """
    return scene().getAttributes(name)

def revertAppearance(name : ItemPath) -> None:
    scene().revertAppearance(name)

def setColor(name : ItemPath, r : float, g : float, b : float, a : float=1.0) -> None:
    scene().setColor(name,r,g,b,a)

def setDrawFunc(name : ItemPath, func : Callable) -> None:
    """Sets a custom OpenGL drawing function for an item.

    Args:
        name (str): the name of the item
        func (function or None): a one-argument function draw(data) that takes the item data
            as input.  Set func to None to revert to default drawing.
    """
    scene().setDrawFunc(name,func)

def _getOffsets(object):
    if isinstance(object,WorldModel):
        res = []
        for i in range(object.numRobots()):
            res += _getOffsets(object.robot(i))
        for i in range(object.numRigidObjects()):
            res += _getOffsets(object.rigidObject(i))
        return res
    elif isinstance(object,RobotModel):
        q = object.getConfig()
        object.setConfig([0.0]*len(q))
        worig = [object.link(i).getTransform()[1] for i in range(object.numLinks())]
        object.setConfig(q)
        wnew = [object.link(i).getTransform()[1] for i in range(object.numLinks())]
        return [vectorops.sub(b,a) for a,b in zip(worig,wnew)]
    elif isinstance(object,RobotModelLink):
        return [object.getTransform()[1]]
    elif isinstance(object,RigidObjectModel):
        return [object.getTransform()[1]]
    elif isinstance(object,Geometry3D):
        return [object.getCurrentTransform()[1]]
    elif isinstance(object,VisAppearance):
        res = _getOffsets(object.item)
        if len(res) != 0: return res
        if len(object.subAppearances) == 0:
            bb = object.getBounds()
            if bb is not None and not bb_empty(bb):
                return [vectorops.mul(vectorops.add(bb[0],bb[1]),0.5)]
        else:
            res = []
            for a in object.subAppearances.values():
                res += _getOffsets(a)
            return res
    return []

def _getBounds(object):
    if isinstance(object,WorldModel):
        res = []
        for i in range(object.numRobots()):
            res += _getBounds(object.robot(i))
        for i in range(object.numRigidObjects()):
            res += _getBounds(object.rigidObject(i))
        return res
    elif isinstance(object,RobotModel):
        res = []
        for i in range(object.numLinks()):
            bb = object.link(i).geometry().getBB()
            if bb is not None and not bb_empty(bb):
                res += list(bb)
        return res
    elif isinstance(object,RobotModelLink):
        return list(object.geometry().getBB())
    elif isinstance(object,RigidObjectModel):
        return list(object.geometry().getBB())
    elif isinstance(object,Geometry3D):
        return list(object.getBB())
    elif isinstance(object,VisAppearance):
        if len(object.subAppearances) == 0:
            if isinstance(object.item,TerrainModel):
                return []
            bb = object.getBounds()
            if bb is not None and not bb_empty(bb):
                return list(bb)
        else:
            res = []
            for a in object.subAppearances.values():
                res += _getBounds(a)
            return res
    return []


def autoFitViewport(viewport : Viewport, objects : Sequence, zoom=True,rotate=True) -> None: 
    from ..model.sensing import fit_plane_centroid
    ofs = sum([_getOffsets(o) for o in objects],[])
    pts = sum([_getBounds(o) for o in objects],[])
    #print("Bounding box",bb,"center",center)
    #raw_input()
    #reset
    viewport.camera.rot = [0.,0.,0.]
    viewport.camera.tgt = [0.,0.,0.]
    viewport.camera.dist = 6.0
    viewport.clippingplanes = (0.2,20)
    if len(ofs) == 0:
        return

    #print(pts)
    #print(ofs)
    pts = pts + ofs # just in case

    bb = bb_create(*pts)
    center = vectorops.mul(vectorops.add(bb[0],bb[1]),0.5)
    viewport.camera.tgt = center
    if zoom:
        radius = max(vectorops.distance(bb[0],center),0.25)
        viewport.camera.dist = 1.2*radius / math.tan(math.radians(viewport.fov*0.5))
    #default: oblique view
    if rotate:
        viewport.camera.rot = [0,math.radians(30),math.radians(45)]
        #fit a plane to these points
        try:
            centroid,normal = fit_plane_centroid(ofs)
        except Exception as e:
            try:
                centroid,normal = fit_plane_centroid(pts)
            except Exception as e:
                warnings.warn("Exception occurred during fitting to points")
                import traceback
                traceback.print_exc()
                raise
                return
        if normal[2] > 0:
            normal = vectorops.mul(normal,-1)
        z,x,y = so3.matrix(so3.inv(so3.canonical(normal)))
        roll = 0
        yaw = math.atan2(normal[0],normal[1])
        pitch = math.atan2(-normal[2],vectorops.norm(normal[0:2]))
        viewport.camera.rot = [roll,pitch,yaw]
    else:
        x = [1,0,0]
        y = [0,0,1]
        z = [0,1,0]
    if zoom:
        radius = max([abs(vectorops.dot(x,vectorops.sub(center,pt))) for pt in pts] + [abs(vectorops.dot(y,vectorops.sub(center,pt)))*viewport.w/viewport.h for pt in pts])
        radius = max(radius,0.25)
        zmin = min([vectorops.dot(z,vectorops.sub(center,pt)) for pt in pts])
        zmax = max([vectorops.dot(z,vectorops.sub(center,pt)) for pt in pts])
        #orient camera to point along normal direction
        viewport.camera.tgt = center
        viewport.camera.dist = 1.2*radius / math.tan(math.radians(viewport.fov*0.5))
        near,far = viewport.clippingplanes
        if viewport.camera.dist + zmin < near:
            near = max((viewport.camera.dist + zmin)*0.5, radius*0.1)
        if viewport.camera.dist + zmax > far:
            far = max((viewport.camera.dist + zmax)*1.5, radius*3)
        viewport.clippingplanes = (near,far)

def addText(name : str, text : str, position=None, **kwargs) -> None:
    """Adds text to the visualizer.  You must give an identifier to all pieces 
    of text, which will be used to access the text as any other vis object. 

    Args:
        name (str): the text's unique identifier.
        text (str): the string to be drawn
        pos (list, optional): the position of the string. If pos=None, this is
            added to the on-screen "console" display.  If pos has length 2, it
            is the (x,y) position of the upper left corner of the text on the
            screen.  Negative units anchor the text to the right or bottom of
            the window.  If pos has length 3, the text is drawn in the world
            coordinates. 
        kwargs (optional): optional keywords to give to setAppearance.

    To customize the text appearance, you can set the 'color', 'size', and
    'position' attributes, either through the keyword arguments, or using
    setAttribute().  To refer to this item, use the identifier given in
    ``name``.
    """
    _init()
    if position is None:
        scene().addText(name,text,**kwargs)
    else:
        scene().addText(name,text,position=position,**kwargs)

def clearText() -> None:
    """Clears all text in the visualization."""
    scene().clearText()

def addPlot(name : str) -> None:
    """Creates a new empty plot with the identifier ``name``."""
    add(name,VisPlot())

def addPlotItem(name : str, itemname : str) -> None:
    """Adds a scene item named ``itemname`` to the plot.  All of the item's
    configuration variables will be plotted by default, see
    :func:`hidePlotItem` to turn off drawing some of these channels.
    """
    scene().addPlotItem(name,itemname)

def logPlot(name : str, itemname : str,value) -> None:
    """Logs a custom visualization item to a plot.  ``itemname`` can be an
    arbitrary identifier; future logPlot calls with this itemname will add
    values to the plotted curve.
    """
    scene().logPlot(name,itemname,value)

def logPlotEvent(name : str,eventname : str,color=None):
    """Logs an event on the plot."""
    scene().logPlotEvent(name,eventname,color)

def hidePlotItem(name : str, itemname : str,hidden=True):
    """Hides an item in the plot.  To hide a particular channel of a given item
    pass a pair ``(itemname,channelindex)``. 

    For example, to hide configurations 0-5 of 'robot', call::
    
        hidePlotItem('plot',('robot',0))
        ...
        hidePlotItem('plot',('robot',5))

    """
    scene().hidePlotItem(name,itemname,hidden)

def setPlotDuration(name : str, time : float) -> None:
    """Sets the plot duration."""
    setAttribute(name,'duration',time)

def setPlotRange(name : str, vmin : float, vmax : float) -> None: 
    """Sets the y range of a plot to [vmin,vmax]."""
    setAttribute(name,'range',(vmin,vmax))

def setPlotPosition(name : str, x : float, y : float) -> None:
    """Sets the upper left position of the plot on the screen."""
    setAttribute(name,'position',(x,y))

def setPlotSize(name : str, w : int, h : int) -> None:
    """sets the width and height of the plot, in pixels."""
    setAttribute(name,'size',(w,h))

def savePlot(name : str, fn : str) -> None:
    """Saves a plot to a CSV (extension .csv) or Trajectory (extension .traj) file."""
    scene().savePlot(name,fn)

def autoFitCamera(zoom=True,rotate=True,scale=1) -> None:
    """Automatically fits the camera to all items in the visualization. 

    Args:
        zoom (bool, optional): zooms the scene to the objects
        rotate (bool, optional): rotates the scene to face the objects
        scale (float, optional): a scale > 1 magnifies the camera zoom.
    """
    print("klampt.vis: auto-fitting camera to scene.")
    scene().autoFitCamera(zoom,rotate,scale)

def followCamera(target,translate=True,rotate=False,center=False) -> None:
    """Sets the camera to follow a target.  The camera starts from its current
    location and keeps the target in the same position on screen. 

    It can operate in the following modes:

    - translation (``translate=True, rotate=False``): the camera moves with the
      object.  This is default.
    - look-at (``translate=False, rotate=True``): the camera stays in the
      current location but rotates to aim toward the object.
    - follow (``translate=True, rotate=True``): the camera moves as though it
      were fixed to the object.

    Args:
        target (str, Trajectory, or None): the target that is to be followed.
            If this is None, the camera no longer follows anything.
        translate (bool, optional): whether the camera should follow using
            translation.
        rotate (bool, optional): whether the camera should follow using
            rotation.
        center (bool, optional): whether the camera should first aim toward the
            object before following. Default is False.
    """
    scene().followCamera(target,translate,rotate,center)

def getViewport() -> GLViewport:
    """Returns the :class:`GLViewport` of the current scene"""
    return scene().getViewport()

def setViewport(viewport : GLViewport) -> None:
    """Sets the current scene to use a given :class:`GLViewport`"""
    scene().setViewport(viewport)

def setBackgroundColor(r : float, g : float, b : float, a : float=1) -> None: 
    """Sets the background color of the current scene."""
    scene().setBackgroundColor(r,g,b,a)

def saveJsonConfig(fn : str=None) -> Union[dict,None]:
    """Saves the visualization options to a JSON object or file.

    If fn is provided, it's saved to a file.  Otherwise, it is returned.
    """
    return scene().saveJsonConfig(fn)

def loadJsonConfig(jsonobj_or_fn : Union[dict,str]):
    """Loads the visualization options from a JSON object or file.

    jsonobj_or_fn can either by a dict (previously obtained by saveJsonConfig
    or a str indicating a filename (previously saved using saveJsonConfig.)
    """
    return scene().loadJsonConfig(jsonobj_or_fn)

def screenshot(format='auto',want_depth=False):
    """Returns a screenshot of the scene. Currently only available in OpenGL modes
    (PyQt, GLUT). 

    ``format`` describes the desired image format.

    - 'auto': equivalent to 'numpy' if numpy is available, 'Image' if PIL is
      available, or 'bytes' otherwise. 
    - 'numpy': the image will be a numpy uint8 array of shape (h,w,3). 
    - 'Image': the image will be a PIL Image if Python Imaging Library is
      available.
    - 'bytes': the image will be in the form ``(w,h,bytes)`` where ``bytes`` is a
      raw bytes array of length 4*w*h.  The buffer follows OpenGL convention with
      the start in the lower-left corner.

    Can also provide a depth image if want_depth=True.  The format will a (w,h)
    numpy float array, a 'F' (float) Image, or (w,h,array) an array of floats.

    In multithreaded mode, this will block until rendering is
    complete.
    """
    global _window_manager
    if _window_manager is None:
        return None
    return _window_manager.screenshot(format,want_depth)

def screenshotCallback(fn : Callable, format='auto', want_depth=False) -> None:
    """Sets a callback ``fn`` that will receive a screenshot of the scene when
    rendering is done.

    See :func:`screenshot` for a description of the ``format`` and ``want_depth``
    arguments.

    Currently only available in OpenGL modes (PyQt, GLUT). 

    To repeatedly receive screenshots, ``fn`` can call
    ``vis.screenshotCallback(fn)`` again.
    """
    global _window_manager
    if _window_manager is None:
        return None
    return _window_manager.screenshotCallback(fn,format,want_depth)

def objectToVisType(item, world : WorldModel) -> str:
    """Returns the default type for the given item in the current world"""
    itypes = types.object_to_types(item,world)
    if isinstance(itypes,(list,tuple)):
        #ambiguous, still need to figure out what to draw
        validtypes = []
        for t in itypes:
            if t=='Vector3':
                validtypes.append(t)
            elif t == 'Config':
                if world is not None:
                    match = False
                    for i in range(world.numRobots()):
                        if len(item) == world.robot(i).numLinks():
                            validtypes.append(t)
                            match = True
                            break
                    if not match and len(itypes) == 1:
                        warnings.warn("Config-like item of length {} doesn't match any of the # links of robots in the world: {}".format(len(item),[world.robot(i).numLinks() for i in range(world.numRobots())]))
            elif t=='RigidTransform':
                validtypes.append(t)
            elif t=='Geometry3D':
                validtypes.append(t)
            elif t=='Trajectory':
                validtypes.append(t)
        if len(validtypes) > 1:
            warnings.warn("Unable to draw item of ambiguous types {}\n  (Try vis.setAttribute(item,'type',desired_type_str) to disambiguate)".format(validtypes))
            return
        if len(validtypes) == 0:
            warnings.warn("Unable to draw any of types {}".format(itypes))
            return
        return validtypes[0]
    return itypes


#_defaultCompressThreshold = 1e-2
_defaultCompressThreshold = 1e-3

class VisPlotItem:
    def __init__(self,itemname,linkitem):
        self.name = itemname
        self.itemnames = []
        self.linkitem = linkitem
        self.traces = []
        self.hidden = []
        self.traceRanges = []
        self.luminosity = []
        self.compressThreshold = _defaultCompressThreshold
        if linkitem is not None:
            q = config.getConfig(linkitem.item)
            assert q is not None
            from collections import deque
            self.traces = [deque() for i in range(len(q))]
            self.itemnames = config.getConfigNames(linkitem.item)

    def customUpdate(self,item,t,v):
        for i,itemname in enumerate(self.itemnames):
            if item == itemname:
                self.updateTrace(i,t,v)
                self.traceRanges[i] = (min(self.traceRanges[i][0],v),max(self.traceRanges[i][1],v))
                return
        else:
            from collections import deque
            self.itemnames.append(item)
            self.traces.append(deque())
            i = len(self.itemnames)-1
            self.updateTrace(i,t,v)
            self.traceRanges[i] = (min(self.traceRanges[i][0],v),max(self.traceRanges[i][1],v))
        #raise ValueError("Invalid item specified: "+str(item))

    def update(self,t):
        if self.linkitem is None:
            return
        q = config.getConfig(self.linkitem.item)
        assert len(self.traces) == len(q)
        for i,v in enumerate(q):
            self.updateTrace(i,t,v)
            self.traceRanges[i] = (min(self.traceRanges[i][0],v),max(self.traceRanges[i][1],v))

    def discard(self,tstart):
        for t in self.traces:
            if len(t)<=1: return
            while len(t) >= 2:
                if t[1][0] < tstart:
                    t.popleft()
                else:
                    break

    def updateTrace(self,i,t,v):
        import random
        assert i < len(self.traces)
        assert i <= len(self.hidden)
        assert i <= len(self.luminosity)
        while i >= len(self.hidden):
            self.hidden.append(False)
        while i >= len(self.traceRanges):
            self.traceRanges.append((v,v))
        if i >= len(self.luminosity):
            initialLuminosity = [0.5,0.25,0.75,1.0]
            while i >= len(self.luminosity):
                if len(self.luminosity)<len(initialLuminosity):
                    self.luminosity.append(initialLuminosity[len(self.luminosity)])
                else:
                    self.luminosity.append(random.uniform(0,1))
        trace = self.traces[i]
        #trange = self.traceRanges[i][1] - self.traceRanges[i][0]
        if len(trace) > 0 and trace[-1][0] >= t:
            tsafe = trace[-1][0]+1e-8
            if v != trace[-1][1]:
                trace.append((tsafe,v))
                return
            trace[-1] = (tsafe,v)
            return
        if self.compressThreshold is None:
            trace.append((t,v))
        else:
            if len(trace) < 2:
                trace.append((t,v))
            else:
                pprev = trace[-2]
                prev = trace[-1]
                assert prev > pprev,"Added two items with the same time?"
                assert t > prev[0]
                slope_old = (prev[1]-pprev[1])/(prev[0]-pprev[0])
                slope_new = (v-prev[1])/(t-prev[0])
                if (slope_old > 0 != slope_new > 0) or abs(slope_old-slope_new) > self.compressThreshold:
                    trace.append((t,v))
                else:
                    #near-linear, just extend along straight line
                    trace[-1] = (t,v)


class VisPlot:
    def __init__(self):
        self.items = []
        self.colors = []
        self.events = dict()
        self.eventColors = dict()
        self.outfile = None
        self.outformat = None

    def __del__(self):
        self.endSave()

    def update(self,t,duration,compressThreshold):
        for i in self.items:
            i.compressThreshold = compressThreshold
            i.update(t)
        if self.outfile:
            self.dumpCurrent()
            self.discard(t-duration)
        else:
            self.discard(t-60.0)

    def discard(self,tmin):
        for i in self.items:
            i.discard(tmin)
        delevents = []
        for e,times in self.events.items():
            while len(times) > 0 and times[0] < tmin:
                times.popleft()
            if len(times)==0: 
                delevents.append(e)
        for e in delevents:
            del self.events[e]

    def addEvent(self,name,t,color=None):
        if name in self.events:
            self.events[name].append(t)
        else:
            from collections import deque
            self.events[name] = deque([t])
            if color is None:
                import random
                color = (random.uniform(0.01,1),random.uniform(0.01,1),random.uniform(0.01,1))
                color = vectorops.mul(color,1.0/max(color))
        if color is not None:
            self.eventColors[name] = color
            if len(color)==3:
                self.eventColors[name] += [1.0]

    def autoRange(self):
        vmin = float('inf')
        vmax = -float('inf')
        for i in self.items:
            for j in range(len(i.traceRanges)):
                if not i.hidden[j]:
                    vmin = min(vmin,i.traceRanges[j][0])
                    vmax = max(vmax,i.traceRanges[j][1])
        if math.isinf(vmin):
            return (0.,1.)
        if vmax == vmin:
            vmax += 1.0
        return (float(vmin),float(vmax))

    def renderGL(self,window,x,y,w,h,duration,vmin=None,vmax=None):
        if GL is None: raise RuntimeError("OpenGL wasn't initialized yet?")
        if vmin is None:
            vmin,vmax = self.autoRange()
        import random
        while len(self.colors) < len(self.items):
            c = (random.uniform(0.01,1),random.uniform(0.01,1),random.uniform(0.01,1))
            c = vectorops.mul(c,1.0/max(c))
            self.colors.append(c)
        GL.glColor3f(0,0,0)
        GL.glBegin(GL.GL_LINE_LOOP)
        GL.glVertex2f(x,y)
        GL.glVertex2f(x+w,y)
        GL.glVertex2f(x+w,y+h)
        GL.glVertex2f(x,y+h)
        GL.glEnd()
        window.draw_text((x-18,y+4),'%.2f'%(vmax,),9)
        window.draw_text((x-18,y+h+4),'%.2f'%(vmin,),9)
        tmax = 0
        for i in self.items:
            for trace in i.traces:
                if len(trace)==0: continue
                tmax = max(tmax,trace[-1][0])
        for i,item in enumerate(self.items):
            for j,trace in enumerate(item.traces):
                if len(trace)==0: continue
                labelheight = trace[-1][1]
                if len(item.name)==0:
                    label = item.itemnames[j]
                else:
                    label = str(item.name) + '.' + item.itemnames[j]
                labelheight = (labelheight - vmin)/(vmax-vmin)
                labelheight = y + h - h*labelheight
                GL.glColor3fv(vectorops.mul(self.colors[i],item.luminosity[j]))
                window.draw_text((x+w+3,labelheight+4),label,9)
                GL.glBegin(GL.GL_LINE_STRIP)
                for k in range(len(trace)-1):
                    if trace[k+1][0] > tmax-duration:
                        u,v = trace[k]
                        if trace[k][0] < tmax-duration:
                            #interpolate so x is at tmax-duration
                            u2,v2 = trace[k+1]
                            #u + s(u2-u) = tmax-duration
                            s = (tmax-duration-u)/(u2-u)
                            v = v + s*(v2-v)
                            u = (tmax-duration)
                        u = (u-(tmax-duration))/duration
                        v = (v-vmin)/(vmax-vmin)
                        GL.glVertex2f(x+w*u,y+(1-v)*h)
                u,v = trace[-1]
                u = (u-(tmax-duration))/duration
                v = (v-vmin)/(vmax-vmin)
                GL.glVertex2f(x+w*u,y+(1-v)*h)
                GL.glEnd()

        if len(self.events) > 0:
            for e,times in self.events.items():
                for t in times:
                    if t < tmax-duration: continue
                    labelx = (t - (tmax-duration))/duration
                    labelx = x + w*labelx
                    c = self.eventColors[e]
                    GL.glColor4f(c[0]*0.5,c[1]*0.5,c[2]*0.5,c[3])
                    window.draw_text((labelx,y+h+12),e,9)
            GL.glEnable(GL.GL_BLEND)
            GL.glBlendFunc(GL.GL_SRC_ALPHA,GL.GL_ONE_MINUS_SRC_ALPHA)
            GL.glBegin(GL.GL_LINES)
            for e,times in self.events.items():
                for t in times:
                    if t < tmax-duration: continue
                    labelx = (t - (tmax-duration))/duration
                    labelx = x + w*labelx
                    c = self.eventColors[e]
                    GL.glColor4f(c[0],c[1],c[2],c[3]*0.5)
                    GL.glVertex2f(labelx,y)
                    GL.glVertex2f(labelx,y+h)
            GL.glEnd()
            GL.glDisable(GL.GL_BLEND)

    def beginSave(self,fn):
        import os
        ext = os.path.splitext(fn)[1]
        if ext == '.csv' or ext == '.traj':
            self.outformat = ext
        else:
            raise ValueError("Invalid extension for visualization plot, can only accept .csv or .traj")
        self.outfile = open(fn,'w')
        if self.outformat == '.csv':
            #output a header
            self.outfile.write("time")
            for i in self.items:
                self.outfile.write(",")
                fullitemnames = []
                if len(i.name) != 0:
                    name = None
                    if isinstance(i.name,(list,tuple)):
                        name = '.'.join(v for v in i.name)
                    else:
                        name = i.name
                    fullitemnames = [name+'.'+itemname for itemname in i.itemnames]
                else:
                    fullitemnames = i.itemnames
                self.outfile.write(",".join(fullitemnames))
            self.outfile.write("\n")
        self.dumpAll()

    def endSave(self):
        if self.outfile is not None:
            self.outfile.close()

    def dumpAll(self):
        assert self.outfile is not None
        if len(self.items) == 0: return
        cols = []
        mindt = float('inf')
        mint = float('inf')
        maxt = -float('inf')
        for i in self.items:
            if len(i.traces) == 0:
                continue
            for j,trace in enumerate(i.traces):
                times,vals = list(zip(*trace))
                if isinstance(vals[0],(int,float)):
                    vals = [[v] for v in vals]
                traj = Trajectory(times,vals)
                cols.append(traj)
                mint = min(mint,traj.times[0])
                maxt = max(maxt,traj.times[-1])
                for k in range(len(traj.times)-1):
                    mindt = min(mindt,traj.times[k+1] - traj.times[k])
        assert mindt > 0, "For some reason, there is a duplicate time?"
        N = int((maxt - mint)/mindt)
        dt = (maxt - mint)/N
        times = [mint + i*(maxt-mint)/N for i in range(N+1)]
        for i in range(N+1):
            vals = [col.eval(times[i]) for col in cols]
            if self.outformat == '.csv':
                self.outfile.write(str(times[i])+',')
                self.outfile.write(','.join([str(v[0]) for v in vals]))
                self.outfile.write('\n')
            else:
                self.outfile.write(str(times[i])+'\t')
                self.outfile.write(str(len(vals))+' ')
                self.outfile.write(' '.join([str(v[0]) for v in vals]))
                self.outfile.write('\n')

    def dumpCurrent(self):
        if len(self.items) == 0: return
        assert len(self.items[0].traces) > 0, "Item has no channels?"
        assert len(self.items[0].traces[0]) > 0, "Item has no readings yet?"
        t = self.items[0].traces[0][-1]
        vals = []
        for i in self.items:
            if len(i.traces) == 0:
                continue
            for j,trace in enumerate(i.traces):
                vals.append(trace[-1][1])
        if self.outformat == '.csv':
            self.outfile.write(str(t)+',')
            self.outfile.write(','.join([str(v) for v in vals]))
            self.outfile.write('\n')
        else:
            self.outfile.write(str(t)+'\t')
            self.outfile.write(str(len(vals))+' ')
            self.outfile.write(' '.join([str(v) for v in vals]))
            self.outfile.write('\n')

def drawTrajectory(traj,width,color,pointSize=None,pointColor=None):
    """Draws a trajectory of points or transforms.

    By default draws points along the trajectory.  To turn this off, set 
    pointSize = 0.
    """
    if GL is None: raise RuntimeError("OpenGL not initialized?")
    if isinstance(traj,list):
        if len(traj)==0:
            return
        if pointSize is None:
            pointSize = width+2
        if pointColor is None:
            pointColor = (color[0]*0.75,color[1]*0.75,color[2]*0.75,color[3])
        #R3 trajectory
        GL.glDisable(GL.GL_LIGHTING)
        GL.glColor4f(*color)
        if len(traj) == 1:
            GL.glPointSize(max(width,pointSize))
            GL.glBegin(GL.GL_POINTS)
            GL.glVertex3fv(traj[0])
            GL.glEnd()
        if len(traj) >= 2 and width > 0:
            GL.glLineWidth(width)
            GL.glBegin(GL.GL_LINE_STRIP)
            for p in traj:
                GL.glVertex3fv(p)
            GL.glEnd()
            GL.glLineWidth(1.0)
        if len(traj) >= 2 and pointSize > 0:
            GL.glColor4f(*pointColor)
            GL.glPointSize(pointSize)
            GL.glBegin(GL.GL_POINTS)
            for p in traj:
                GL.glVertex3fv(p)
            GL.glEnd()
    elif isinstance(traj,SE3Trajectory):
        pointTraj = []
        for m in traj.milestones:
            pointTraj.append(m[9:12])
        drawTrajectory(pointTraj,width,color,pointSize,pointColor)
    elif isinstance(traj,SE3HermiteTrajectory):
        pointTraj = []
        velTraj = []
        for m in traj.milestones:
            pointTraj.append(m[9:12])
            velTraj.append(m[21:24])
        drawTrajectory(HermiteTrajectory(traj.times,pointTraj,velTraj),width,color,pointSize,pointColor)
    else:
        if len(traj.milestones)==0:
            return
        wp = traj.waypoint(traj.milestones[0])
        if len(wp) == 3:
            if len(wp) == len(traj.milestones[0]):   
                drawTrajectory(traj.milestones,width,color,pointSize,pointColor)
            else:  #discrepancy, must be hermite
                if width > 0:
                    discretized = traj.discretize(traj.duration()/len(traj.milestones)*0.1)
                    drawTrajectory(discretized.milestones,width,color,0,None)
                if pointSize is None or pointSize > 0:
                    drawTrajectory([traj.waypoint(m) for m in traj.milestones],0,color,pointSize,pointColor)
        elif len(wp) == 2:
            #R2 trajectory
            if len(wp) == len(traj.milestones[0]): 
                drawTrajectory([v + [0.0] for v in traj.milestones],width,color,pointSize,pointColor)
            else:  #discrepancy, must be hermite
                if width > 0:
                    discretized = traj.discretize(traj.duration()/len(traj.milestones)*0.1)
                    drawTrajectory([m  + [0.0] for m in discretized.milestones],width,color,0,None)
                if pointSize is None or pointSize > 0:
                    drawTrajectory([traj.waypoint(m) + [0.0] for m in traj.milestones],0,color,pointSize,pointColor)


def drawRobotTrajectory(traj,robot,ees,width=2,color=(1,0.5,0,1),pointSize=None,pointColor=None):
    """Draws trajectories for the robot's end effectors.  Note: no additional discretization is performed,
    only the end effector points at the trajectory's milestones are shown.  If you want more accurate trajectories,
    first call traj.discretize(eps)."""
    for i,ee in enumerate(ees):
        if ee < 0: ees[i] = robot.numLinks()-1
    pointTrajectories = []
    for ee in ees:
        pointTrajectories.append([])
    if isinstance(traj,Trajectory):
        traj = traj.milestones
    for m in traj:
        robot.setConfig(m)
        for ee,eetraj in zip(ees,pointTrajectories):
            eetraj.append(robot.link(ee).getTransform()[1])
    for ptraj in pointTrajectories:
        drawTrajectory(ptraj,width,color,pointSize,pointColor)


class _CascadingDict:
    """A hierarchical dictionary structure that can be defined with respect
    to a parent dict or _CascadingDict.  Items in this dict override the
    items in the parent.  Deleting an item from this dict does not delete
    from the parent.

    Be careful when modifying sub-items of top-level keys.  You may not know
    whether the key accesses the parent or this object::

        parent = {'foo':[1,2,3]}
        obj = _CascadingDict(parent=parent)
        obj['foo'][0] = 'hello'   #this actually changes the value of parent['foo']
        obj2 = _CascadingDict(parent=parent)
        print(obj2['foo'])   #prints ['hello',2,3]

        obj['foo'] = 4       #modifies the object's key 'foo'
        print(obj2['foo'])   #still prints ['hello',2,3], since the non-overridden key
                             #is still pointing to parent

    """
    def __init__(self,rhs=None,parent=None):
        if rhs is not None:
            if isinstance(rhs,_CascadingDict):
                self.overrides = rhs
            else:
                self.overrides = dict(rhs)
        else:
            self.overrides = dict()
        self.parent = None
    def setParent(self,parent):
        self.parent = parent
    def __getitem__(self,item):
        try:
            return self.overrides[item]
        except KeyError:
            if self.parent is None:
                raise
            else:
                return self.parent[item]
    def __setitem__(self,key,value):
        self.overrides[key] = value
    def __delitem__(self,key):
        del self.overrides[key]
    def get(self,item,default=None):
        try:
            return self.__getitem__(item)
        except KeyError:
            return default
    def flatten(self):
        """Returns a normal dict containing all items in this or its parents"""
        pdict = {}
        if isinstance(self.parent,_CascadingDict):
            pdict = self.parent.flatten()
        elif isinstance(self.parent,dict):
            pdict = self.parent.copy()
        for k,v in self.overrides.items():
            pdict[k] = v
        return pdict

    def __str__(self):
        return str(self.flatten())

    def __repr__(self):
        return repr(self.flatten())

    def __contains__(self,item):
        if item in self.overrides:
            return True
        if self.parent is None:
            return False
        return item in self.parent

_default_str_attributes = {'color':[0,0,0,1], 'position':None, 'size':12 }
_default_Trajectory_attributes = { 'robot':0, "width":3, "color":(1,0.5,0,1), "pointSize":None, "pointColor":None }
_default_RobotTrajectory_attributes = { 'robot':0, "width":3, "color":(1,0.5,0,1), "pointSize":None, "pointColor":None , "endeffectors":[-1]}
_default_VisPlot_attributes = {'compress':_defaultCompressThreshold, 'duration':5., 'position':None, 'range':(None,None), 'size':(200,150), 'hide_label':True}
_default_Point_attributes = { "size":5.0, "color":(0,0,0,1) }
_default_Direction_attributes = { "length":0.15, "color":[0,1,1,1] }
_default_Frame_attributes = { "length":0.1, "width":0.01 }
_default_ContactPoint_attributes = { "size":5.0, "length":0.05, "color":[1,0.5,0,1] }
_default_IKObjective_attributes = { "size":5.0, "color":(0,0,0,1), "length":0.1, "width": 0.01, "axis_color":[0.5,0,0.5,1], "axis_width":3.0, "axis_length":0.1 }
_default_Geometry_attributes = { "size":None, "color":None }
_default_RigidTransform_attributes = { "fancy":False, "length":0.1, "width":0.01 }

def _default_attributes(item,type=None):
    """Returns an attribute dictionary for the defaults of a given item.

    If the item is ambiguous, you can provide the type argument.
    """
    res = {}
    if isinstance(item,str):
        return _default_str_attributes
    elif isinstance(item,(WorldModel,RobotModel)):
        pass
    elif hasattr(item,'appearance'):
        res['color'] = item.appearance().getColor()
        pass
    elif isinstance(item,(Trajectory,MultiPath)):
        
        if isinstance(item,RobotTrajectory):
            return _default_RobotTrajectory_attributes
        else:
            return _default_Trajectory_attributes
    elif isinstance(item,VisPlot):
        return _default_VisPlot_attributes
    elif isinstance(item,coordinates.Point):
        return _default_Point_attributes
    elif isinstance(item,coordinates.Direction):
        return _default_Direction_attributes
    elif isinstance(item,coordinates.Frame):
        return _default_Frame_attributes
    elif isinstance(item,coordinates.Transform):
        pass
    elif isinstance(item,coordinates.Group):
        res['hide_label'] = True
    elif isinstance(item,ContactPoint):
        return _default_ContactPoint_attributes
    elif isinstance(item,Hold):
        pass
    elif isinstance(item,IKObjective):
        return _default_IKObjective_attributes
    elif isinstance(item,(GeometricPrimitive,TriangleMesh,PointCloud,Geometry3D)):
        return _default_Geometry_attributes
    else:
        if type is not None:
            itypes = type
        else:
            try:
                itypes = objectToVisType(item,None)
                res["type"]=itypes
            except Exception as e:
                if hasattr(item,'drawGL'):
                    #assume it's a SimRobotSensor, Appearance, or SubRobotModel
                    return
                warnings.warn(str(e))
                warnings.warn("Unsupported object type {} of type {}".format(item,item.__class__.__name__))
                return
        if itypes is None:
            warnings.warn("Unable to convert item {} to drawable".format(str(item)))
            return
        elif itypes == 'Config':
            pass
        elif itypes == 'Configs':
            res["maxConfigs"] = min(10,len(item))
        elif itypes == 'Vector3':
            return _default_Point_attributes
        elif itypes == 'RigidTransform':
            return _default_RigidTransform_attributes
        else:
            warnings.warn("klampt.vis: Unable to draw item of type \"%s\""%(str(itypes),))
            res['hidden'] = True
            res['hide_label'] = True
    return res


class VisAppearance:
    """The core class that governs all of the drawing of an object
    in the visualization.  Can accommodate any drawable Klampt type.
    """
    def __init__(self,item,name = None, type=None):
        self.name = name
        self.useDefaultAppearance = True
        self.customAppearance = None
        self.customDrawFunc = None
        #For group items, this allows you to customize appearance of sub-items
        self.subAppearances = {}
        self.animation = None
        self.animationStartTime = 0
        self.animationSpeed = 1.0
        self.attributes = _default_attributes(item,type)
        if not isinstance(self.attributes,_CascadingDict):
            self.attributes = _CascadingDict(self.attributes)
        if 'hide_label' not in self.attributes:
            self.attributes['hide_label'] = False
        if 'hidden' not in self.attributes:
            self.attributes['hidden'] = False
        self.attributes['label'] = name
        #used for Qt text rendering
        self.widget = None
        #used for visual editing of certain items
        self.editor = None
        #cached drawing
        self.displayCache = [glcommon.CachedGLObject()]
        self.displayCache[0].name = name
        #temporary configuration of the item
        self.drawConfig = None
        self.transformChanged = False
        self.setItem(item)

    def setItem(self,item):
        self.item = item
        self.subAppearances = {}
        #Parse out sub-items which can have their own appearance changed
        if isinstance(item,WorldModel):
            for i in range(item.numRobots()):
                self.subAppearances[("Robot",i)] = VisAppearance(item.robot(i),item.robot(i).getName())
            for i in range(item.numRigidObjects()):
                self.subAppearances[("RigidObject",i)] = VisAppearance(item.rigidObject(i),item.rigidObject(i).getName())
            for i in range(item.numTerrains()):
                self.subAppearances[("Terrain",i)] = VisAppearance(item.terrain(i),item.terrain(i).getName())
        elif isinstance(item,RobotModel):
            for i in range(item.numLinks()):
                self.subAppearances[("Link",i)] = VisAppearance(item.link(i),item.link(i).getName())
        elif isinstance(item,coordinates.Group):
            for n,f in item.frames.items():
                self.subAppearances[("Frame",n)] = VisAppearance(f,n)
            for n,p in item.points.items():
                self.subAppearances[("Point",n)] = VisAppearance(p,n)
            for n,d in item.directions.items():
                self.subAppearances[("Direction",n)] = VisAppearance(d,n)
            for n,g in item.subgroups.items():
                self.subAppearances[("Subgroup",n)] = VisAppearance(g,n)
        elif isinstance(item,Hold):
            if item.ikConstraint is not None:
                self.subAppearances["ikConstraint"] = VisAppearance(item.ikConstraint,"ik")
            for n,c in enumerate(item.contacts):
                self.subAppearances[("contact",n)] = VisAppearance(c,n)
        
    def markChanged(self,config=True,appearance=True):
        if appearance:
            for c in self.displayCache:
                c.mark_changed()
        for (k,a) in self.subAppearances.items():
            a.markChanged(config,appearance)
        if config:
            self.update_editor(True)
            self.transformChanged = True

    def destroy(self):
        for c in self.displayCache:
            c.destroy()
        for (k,a) in self.subAppearances.items():
            a.destroy()
        self.subAppearances = {}
        
    def drawText(self,text,point):
        """Draws the given text at the given point"""
        if len(point) != 3:
            warnings.warn("drawText INCORRECT POINT SIZE {} {}".format(point,text))
            return
        if not all(math.isfinite(v) for v in point):
            warnings.warn("drawText INVALID POINT {} {}".format(point,text))
            return
        self.widget.addLabel(text,point[:],[0,0,0])

    def updateAnimation(self,t):
        """Updates the configuration, if it's being animated"""
        if not self.animation:
            if self.drawConfig is not None:
                self.markChanged(config=True,appearance=False)
            self.drawConfig = None
        else:
            u = self.animationSpeed*(t-self.animationStartTime)
            q = self.animation.eval(u,self.animationEndBehavior)
            self.drawConfig = config.getConfig(q)
            self.markChanged(config=True,appearance=False)
        for n,app in self.subAppearances.items():
            app.updateAnimation(t)

    def updateTime(self,t):
        """Updates in real time"""
        if isinstance(self.item,VisPlot):
            compressThreshold = self.attributes['compress']
            duration = self.attributes['duration']
            for items in self.item.items:
                if items.linkitem:
                    items.linkitem.swapDrawConfig()
            self.item.update(t,duration,compressThreshold)
            for items in self.item.items:
                if items.linkitem:
                    items.linkitem.swapDrawConfig()

    def swapDrawConfig(self):
        """Given self.drawConfig!=None, swaps out the item's curren
        configuration  with self.drawConfig.  Used for animations"""
        if self.drawConfig: 
            try:
                newDrawConfig = config.getConfig(self.item)
                if len(newDrawConfig) != len(self.drawConfig):
                    warnings.warn("Incorrect length of draw configuration? {} vs {}".format(len(self.drawConfig),len(newDrawConfig)))
                config.setConfig(self.item,self.drawConfig)
                self.drawConfig = newDrawConfig
            except Exception as e:
                warnings.warn("Exception thrown during animation update.  Probably have incorrect length of configuration")
                import traceback
                traceback.print_exc()
                self.animation = None
                pass
        for n,app in self.subAppearances.items():
            app.swapDrawConfig()

    def clearDisplayLists(self):
        if isinstance(self.item,WorldModel):
            for r in range(self.item.numRobots()):
                for link in range(self.item.robot(r).numLinks()):
                    self.item.robot(r).link(link).appearance().refresh()
            for i in range(self.item.numRigidObjects()):
                self.item.rigidObject(i).appearance().refresh()
            for i in range(self.item.numTerrains()):
                self.item.terrain(i).appearance().refresh()
        elif hasattr(self.item,'appearance'):
            self.item.appearance().refresh()
        elif isinstance(self.item,RobotModel):
            for link in range(self.item.numLinks()):
                self.item.link(link).appearance().refresh()
        for n,o in self.subAppearances.items():
            o.clearDisplayLists()
        self.markChanged(config=False,appearance=True)

    def transparent(self):
        """Returns true if the item is entirely transparent, None if mixed transparency, and False otherwise"""
        if len(self.subAppearances)!=0:
            anyTransp = False
            anyOpaque = False
            for n,app in self.subAppearances.items():
                if app.transparent():
                    anyTransp = True
                else:
                    anyOpaque = True
            if anyTransp and anyOpaque:
                return None
            else:
                return anyTransp
        if hasattr(self.item,'appearance'):
            if self.useDefaultAppearance or 'color' not in self.attributes:
                if isinstance(self.item,WorldModel):
                    #corner case: empty world
                    return False
                else:
                    return self.item.appearance().getColor()[3] < 1.0
        try:
            return (self.attributes['color'][3] < 1.0)
        except:
            return False

    def getAttributes(self):
        if len(self.subAppearances) > 0:
            return {}
        return self.attributes.flatten()

    def drawGL(self,world=None,viewport=None,draw_transparent=None):
        """Draws the specified item in the specified world, with all the
        current modifications in attributes.

        If a name or label are given, and
        ``self.attributes['hide_label'] != False``, then the label is shown.

        The drawing passes are controlled by ``draw_transparent`` -- opaque
        items should be rasterized before transparent ones.

        Args:
            world (WorldModel): the world model
            viewport (Viewport): the C++ viewport of the current view, which is
                compatible with the Klampt C++ Widget class.
            draw_transparent (bool or None): If None, then everything is drawn. 
                If True, then only transparent items are drawn.  If False, then
                only opaque items are drawn.  (This only affects WorldModels)
        """
        if self.attributes["hidden"]:
            return
        if self.customDrawFunc is not None:
            self.customDrawFunc(self.item)
            return
       
        item = self.item
        name = None
        if not self.attributes["hide_label"]:
            name = self.attributes['label']
        #set appearance
        if not self.useDefaultAppearance and hasattr(item,'appearance'):
            if not hasattr(self,'oldAppearance'):
                self.oldAppearance = item.appearance().clone()
            if self.customAppearance is not None:
                item.appearance().set(self.customAppearance)
            elif "color" in self.attributes:
                item.appearance().setColor(*self.attributes["color"])

        if self.editor is not None:
            assert viewport is not None
            #some weird logic to determine whether to draw now...
            if draw_transparent is None or draw_transparent == self.transparent() or (self.transparent() is None and draw_transparent==False):
                #KLUDGE:
                #might be a robot, make sure the appearances are all up to date before drawing the editor
                for n,app in self.subAppearances.items():
                    if app.useDefaultAppearance or not hasattr(app.item,'appearance'):
                        continue
                    if not hasattr(app,'oldAppearance'):
                        app.oldAppearance = app.item.appearance().clone()
                    if app.customAppearance is not None:
                        app.item.appearance().set(app.customAppearance)
                    elif "color" in app.attributes:
                        app.item.appearance().setColor(*app.attributes["color"])

                if isinstance(self.editor,RobotPoser) and isinstance(self.item,(list,tuple)) and world is not None:
                    #KLUDGE: config editor; make sure that the robot in the world model is restored
                    #use the 
                    robot = world.robot(0)
                    oldconfig = robot.getConfig()
                    oldAppearances = []
                    if "color" in self.attributes:
                        for i in range(robot.numLinks()):
                            oldAppearances.append(robot.link(i).appearance().clone())
                            robot.link(i).appearance().setColor(*self.attributes['color'])
                else:
                    oldconfig = None

                self.editor.drawGL(viewport)

                if oldconfig is not None:
                    world.robot(0).setConfig(oldconfig)
                    if len(oldAppearances) > 0:
                        for i in range(robot.numLinks()):
                            robot.link(i).appearance().set(oldAppearances[i])

                #Restore sub-appearances
                for n,app in self.subAppearances.items():
                    if app.useDefaultAppearance or not hasattr(app.item,'appearance'):
                        continue
                    app.item.appearance().set(app.oldAppearance)
            if isinstance(self.editor,RobotPoser):
                #the widget took care of everything, dont continue drawing the item
                #revert the robot's actual pose
                #revert appearance if necessary
                if not self.useDefaultAppearance and hasattr(item,'appearance'):
                    item.appearance().set(self.oldAppearance)
                return

        if len(self.subAppearances)!=0:
            for n,app in self.subAppearances.items():
                if draw_transparent is True:
                    if not app.transparent():
                        continue
                elif draw_transparent is False:
                    if app.transparent():
                        continue
                app.widget = self.widget
                app.drawGL(world,viewport,draw_transparent)
        elif hasattr(item,'drawGL'):
            item.drawGL()
        elif hasattr(item,'drawWorldGL'):
            item.drawWorldGL()
        elif isinstance(item,str):
            pos = self.attributes["position"]
            if pos is not None and len(pos)==3:
                col = self.attributes["color"]
                self.widget.addLabel(self.item,pos,col)
        elif isinstance(item,VisPlot):
            pass
        elif isinstance(item,Trajectory):
            doDraw = False
            centroid = None
            if len(item.milestones) == 0:
                return
            robot = (world.robot(self.attributes["robot"]) if world is not None and world.numRobots() > 0 else None)
            if robot is not None:
                robotConfig = robot.getConfig()
            treatAsRobotTrajectory = (item.__class__ == Trajectory and len(item.milestones) > 0 and robot and len(item.milestones[0]) == robot.numLinks())
            if isinstance(item,RobotTrajectory) or treatAsRobotTrajectory:
                ees = self.attributes.get("endeffectors",[-1])
                if world:
                    doDraw = (len(ees) > 0)
                    for i,ee in enumerate(ees):
                        if ee < 0: ees[i] = robot.numLinks()-1
                    if doDraw:
                        robot.setConfig(item.milestones[0])
                        centroid = vectorops.div(vectorops.add(*[robot.link(ee).getTransform()[1] for ee in ees]),len(ees))
            elif isinstance(item,(SE3Trajectory,SE3HermiteTrajectory)):
                doDraw = True
                centroid = item.waypoint(item.milestones[0])[1]
            else:
                wp = item.waypoint(item.milestones[0])
                if len(wp) == 3:
                    #R3 trajectory
                    doDraw = True
                    centroid = wp
                elif len(item.waypoint(item.milestones[0])) == 2:
                    #R2 trajectory
                    doDraw = True
                    centroid = wp+[0.0]
                else:
                    #don't know how to interpret this trajectory
                    pass
            if doDraw:
                assert len(centroid)==3
                def drawRaw():
                    width = self.attributes["width"]
                    color = self.attributes["color"]
                    pointSize = self.attributes["pointSize"]
                    pointColor = self.attributes["pointColor"]
                    if isinstance(item,RobotTrajectory) or treatAsRobotTrajectory:
                        ees = self.attributes.get("endeffectors",[-1])
                        drawRobotTrajectory(item,robot,ees,width,color,pointSize,pointColor) 
                    else:
                        drawTrajectory(item,width,color,pointSize,pointColor)
                self.displayCache[0].draw(drawRaw)
                if name is not None:
                    self.drawText(name,centroid)
                if robot is not None:
                    robot.setConfig(robotConfig)
        elif isinstance(item,MultiPath):
            robot = (world.robot(self.attributes["robot"]) if world is not None and world.numRobots() > 0 else None)
            if robot is not None and item.numSections() > 0:
                if len(item.sections[0].configs[0]) == robot.numLinks():
                    ees = self.attributes.get("endeffectors",[-1])
                    centroid = None
                    if len(ees) > 0:
                        for i,ee in enumerate(ees):
                            if ee < 0: ees[i] = robot.numLinks()-1
                        robot.setConfig(item.sections[0].configs[0])
                        centroid = vectorops.div(vectorops.add(*[robot.link(ee).getTransform()[1] for ee in ees]),len(ees))
                    width = self.attributes["width"]
                    color = self.attributes["color"]
                    pointSize = self.attributes["pointSize"]
                    pointColor = self.attributes["pointColor"]
                    color2 = [1-c for c in color]
                    color2[3] = color[3]
                    def drawRaw():
                        for i,s in enumerate(item.sections):
                            drawRobotTrajectory(s.configs,robot,ees,width,(color if i%2 == 0 else color2),pointSize,pointColor)
                    #draw it!
                    self.displayCache[0].draw(drawRaw)
                    if name is not None and centroid is not None:
                        self.drawText(name,centroid)
        elif isinstance(item,coordinates.Point):
            def drawRaw():
                GL.glDisable(GL.GL_LIGHTING)
                GL.glEnable(GL.GL_POINT_SMOOTH)
                GL.glPointSize(self.attributes["size"])
                GL.glColor4f(*self.attributes["color"])
                GL.glBegin(GL.GL_POINTS)
                GL.glVertex3f(0,0,0)
                GL.glEnd()
                #write name
            GL.glDisable(GL.GL_DEPTH_TEST)
            self.displayCache[0].draw(drawRaw,[so3.identity(),item.worldCoordinates()])
            GL.glEnable(GL.GL_DEPTH_TEST)
            if name is not None:
                self.drawText(name,item.worldCoordinates())
        elif isinstance(item,coordinates.Direction):
            def drawRaw():
                GL.glDisable(GL.GL_LIGHTING)
                GL.glDisable(GL.GL_DEPTH_TEST)
                L = self.attributes["length"]
                source = [0,0,0]
                GL.glColor4f(*self.attributes["color"])
                GL.glBegin(GL.GL_LINES)
                GL.glVertex3f(*source)
                GL.glVertex3f(*vectorops.mul(item.localCoordinates(),L))
                GL.glEnd()
                GL.glEnable(GL.GL_DEPTH_TEST)
                #write name
            self.displayCache[0].draw(drawRaw,item.frame().worldCoordinates(),parameters = item.localCoordinates())
            if name is not None:
                self.drawText(name,vectorops.add(item.frame().worldCoordinates()[1],item.worldCoordinates()))
        elif isinstance(item,coordinates.Frame):
            t = item.worldCoordinates()
            if item.parent() is not None:
                tp = item.parent().worldCoordinates()
            else:
                tp = se3.identity()
            tlocal = item.relativeCoordinates()
            def drawRaw():
                GL.glDisable(GL.GL_DEPTH_TEST)
                GL.glDisable(GL.GL_LIGHTING)
                GL.glLineWidth(2.0)
                gldraw.xform_widget(tlocal,self.attributes["length"],self.attributes["width"])
                GL.glLineWidth(1.0)
                #draw curve between frame and parent
                if item.parent() is not None:
                    d = vectorops.norm(tlocal[1])
                    vlen = d*0.5
                    v1 = so3.apply(tlocal[0],[-vlen]*3)
                    v2 = [vlen]*3
                    #GL.glEnable(GL.GL_BLEND)
                    #GL.glBlendFunc(GL.GL_SRC_ALPHA,GL.GL_ONE_MINUS_SRC_ALPHA)
                    #GL.glColor4f(1,1,0,0.5)
                    GL.glColor3f(1,1,0)
                    gldraw.hermite_curve(tlocal[1],v1,[0,0,0],v2,0.03*max(0.1,vectorops.norm(tlocal[1])))
                    #GL.glDisable(GL.GL_BLEND)
                GL.glEnable(GL.GL_DEPTH_TEST)

            #For some reason, cached drawing is causing OpenGL problems
            #when the frame is rapidly changing
            self.displayCache[0].draw(drawRaw,transform=tp, parameters = tlocal)
            #GL.glPushMatrix()
            #GL.glMultMatrixf(sum(zip(*se3.homogeneous(tp)),()))
            #drawRaw()
            #GL.glPopMatrix()
            #write name
            if name is not None:
                self.drawText(name,t[1])
        elif isinstance(item,coordinates.Transform):
            #draw curve between frames
            t1 = item.source().worldCoordinates()
            if item.destination() is not None:
                t2 = item.destination().worldCoordinates()
            else:
                t2 = se3.identity()
            d = vectorops.distance(t1[1],t2[1])
            vlen = d*0.5
            v1 = so3.apply(t1[0],[-vlen]*3)
            v2 = so3.apply(t2[0],[vlen]*3)
            def drawRaw():
                GL.glDisable(GL.GL_DEPTH_TEST)
                GL.glDisable(GL.GL_LIGHTING)
                GL.glColor3f(1,1,1)
                gldraw.hermite_curve(t1[1],v1,t2[1],v2,0.03)
                GL.glEnable(GL.GL_DEPTH_TEST)
                #write name at curve
            self.displayCache[0].draw(drawRaw,transform=None,parameters = (t1,t2))
            if name is not None:
                self.drawText(name,spline.hermite_eval(t1[1],v1,t2[1],v2,0.5))
        elif isinstance(item,coordinates.Group):
            pass
        elif isinstance(item,ContactPoint):
            def drawRaw():
                GL.glDisable(GL.GL_LIGHTING)
                GL.glEnable(GL.GL_POINT_SMOOTH)
                GL.glPointSize(self.attributes["size"])
                l = self.attributes["length"]
                GL.glColor4f(*self.attributes["color"])
                GL.glBegin(GL.GL_POINTS)
                GL.glVertex3f(0,0,0)
                GL.glEnd()
                GL.glBegin(GL.GL_LINES)
                GL.glVertex3f(0,0,0)
                GL.glVertex3f(l,0,0)
                GL.glEnd()
            self.displayCache[0].draw(drawRaw,[so3.canonical(item.n),item.x])
        elif isinstance(item,Hold):
            pass
        elif isinstance(item,IKObjective):
            if hasattr(item,'robot'):
                #need this to be built with a robot element.
                #Otherwise, can't determine the correct transforms
                robot = item.robot
            elif world:
                if world is not None and world.numRobots() > 0:
                    robot = world.robot(self.attributes.get("robot",0))
                else:
                    robot = None
            else:
                robot = None
            if robot is not None:
                link = robot.link(item.link())
                dest = robot.link(item.destLink()) if item.destLink()>=0 else None
                while len(self.displayCache) < 3:
                    self.displayCache.append(glcommon.CachedGLObject())
                self.displayCache[1].name = self.name+" target position"
                self.displayCache[2].name = self.name+" curve"
                if item.numPosDims() != 0:
                    lp,wp = item.getPosition()
                    #set up parameters of connector
                    p1 = se3.apply(link.getTransform(),lp)
                    if dest is not None:
                        p2 = se3.apply(dest.getTransform(),wp)
                    else:
                        p2 = wp
                    d = vectorops.distance(p1,p2)
                    v1 = [0.0]*3
                    v2 = [0.0]*3
                    if item.numRotDims()==3: #full constraint
                        R = item.getRotation()
                        def drawRaw():
                            gldraw.xform_widget(se3.identity(),self.attributes["length"],self.attributes["width"])
                        t1 = se3.mul(link.getTransform(),(so3.identity(),lp))
                        t2 = (R,wp) if dest==None else se3.mul(dest.getTransform(),(R,wp))
                        self.displayCache[0].draw(drawRaw,transform=t1)
                        self.displayCache[1].draw(drawRaw,transform=t2)
                        vlen = d*0.1
                        v1 = so3.apply(t1[0],[-vlen]*3)
                        v2 = so3.apply(t2[0],[vlen]*3)
                    elif item.numRotDims()==0: #point constraint
                        def drawRaw():
                            GL.glDisable(GL.GL_LIGHTING)
                            GL.glEnable(GL.GL_POINT_SMOOTH)
                            GL.glPointSize(self.attributes["size"])
                            GL.glColor4f(*self.attributes["color"])
                            GL.glBegin(GL.GL_POINTS)
                            GL.glVertex3f(0,0,0)
                            GL.glEnd()
                        self.displayCache[0].draw(drawRaw,transform=(so3.identity(),p1))
                        self.displayCache[1].draw(drawRaw,transform=(so3.identity(),p2))
                        #set up the connecting curve
                        vlen = d*0.5
                        d = vectorops.sub(p2,p1)
                        v1 = vectorops.mul(d,0.5)
                        #curve in the destination
                        v2 = vectorops.cross((0,0,0.5),d)
                    else: #hinge constraint
                        p = [0,0,0]
                        d = [0,0,0]
                        def drawRawLine():
                            GL.glDisable(GL.GL_LIGHTING)
                            GL.glEnable(GL.GL_POINT_SMOOTH)
                            GL.glPointSize(self.attributes["size"])
                            GL.glColor4f(*self.attributes["color"])
                            GL.glBegin(GL.GL_POINTS)
                            GL.glVertex3f(*p)
                            GL.glEnd()
                            GL.glColor4f(*self.attributes["color"])
                            GL.glLineWidth(self.attributes["width"])
                            GL.glBegin(GL.GL_LINES)
                            GL.glVertex3f(*p)
                            GL.glVertex3f(*vectorops.madd(p,d,self.attributes["length"]))
                            GL.glEnd()
                            GL.glLineWidth(1.0)
                        ld,wd = item.getRotationAxis()
                        p = lp
                        d = ld
                        self.displayCache[0].draw(drawRawLine,transform=link.getTransform(),parameters=(p,d))
                        p = wp
                        d = wd
                        self.displayCache[1].draw(drawRawLine,transform=dest.getTransform() if dest else se3.identity(),parameters=(p,d))
                        #set up the connecting curve
                        d = vectorops.sub(p2,p1)
                        v1 = vectorops.mul(d,0.5)
                        #curve in the destination
                        v2 = vectorops.cross((0,0,0.5),d)
                    def drawConnection():
                        GL.glDisable(GL.GL_LIGHTING)
                        GL.glDisable(GL.GL_DEPTH_TEST)
                        GL.glColor3f(1,0.5,0)
                        gldraw.hermite_curve(p1,v1,p2,v2,0.03*max(0.1,vectorops.distance(p1,p2)))
                        #GL.glBegin(GL.GL_LINES)
                        #GL.glVertex3f(*p1)
                        #GL.glVertex3f(*p2)
                        #GL.glEnd()
                        GL.glEnable(GL.GL_DEPTH_TEST)
                    #TEMP for some reason the cached version sometimes gives a GL error
                    self.displayCache[2].draw(drawConnection,transform=None,parameters = (p1,v1,p2,v2))
                    #drawConnection()
                    if name is not None:
                        self.drawText(name,wp)
                else:
                    wp = link.getTransform()[1]
                    if item.numRotDims()==3: #full constraint
                        R = item.getRotation()
                        def drawRaw():
                            gldraw.xform_widget(se3.identity(),self.attributes["length"],self.attributes["width"])
                        self.displayCache[0].draw(drawRaw,transform=link.getTransform())
                        self.displayCache[1].draw(drawRaw,transform=se3.mul(link.getTransform(),(R,[0,0,0])))
                    elif item.numRotDims() > 0:
                        #axis constraint
                        d = [0,0,0]
                        def drawRawLine():
                            GL.glDisable(GL.GL_LIGHTING)
                            GL.glColor4f(*self.attributes["axis_color"])
                            GL.glLineWidth(self.attributes["axis_width"])
                            GL.glBegin(GL.GL_LINES)
                            GL.glVertex3f(0,0,0)
                            GL.glVertex3f(*vectorops.mul(d,self.attributes["axis_length"]))
                            GL.glEnd()
                            GL.glLineWidth(1.0)
                        ld,wd = item.getRotationAxis()
                        d = ld
                        self.displayCache[0].draw(drawRawLine,transform=link.getTransform(),parameters=d)
                        d = wd
                        self.displayCache[1].draw(drawRawLine,transform=(dest.getTransform()[0] if dest else so3.identity(),wp),parameters=d)
                    else:
                        #no drawing
                        pass
                    if name is not None:
                        self.drawText(name,wp)
        elif isinstance(item,(GeometricPrimitive,TriangleMesh,PointCloud,Geometry3D)):
            #this can be tricky if the mesh or point cloud has colors
            if not hasattr(self,'appearance'):
                self.appearance = Appearance()
                self.appearance.setColor(0.5,0.5,0.5,1)
            c = self.attributes["color"]
            if c is not None:
                self.appearance.setColor(*c)
            s = self.attributes["size"]
            if s:
                self.appearance.setPointSize(s)
            self.appearance.setSilhouette(0)
            self.appearance.setCreaseAngle(0)
            wp = None
            geometry = None
            lighting = True
            restoreLineWidth = False
            if isinstance(self.item,GeometricPrimitive):
                if not hasattr(self,'geometry'):
                    self.geometry = Geometry3D(self.item)
                geometry = self.geometry
                if self.item.type in ['Point','Segment']:
                    lighting = False
                    if self.item.type == 'Segment':
                        GL.glLineWidth(self.attributes.get('width',3.0))
                        restoreLineWidth = True
            elif isinstance(self.item,PointCloud):
                if not hasattr(self,'geometry'):
                    self.geometry = Geometry3D(self.item)
                lighting = False
                geometry = self.geometry
            elif isinstance(self.item,TriangleMesh):
                if not hasattr(self,'geometry'):
                    self.geometry = Geometry3D(self.item)
                geometry = self.geometry
            else:
                assert isinstance(self.item,Geometry3D)
                if self.item.type() == 'GeometricPrimitive':
                    prim = self.item.getGeometricPrimitive()
                    if prim.type in ['Point','Segment']:
                        lighting = False
                        if prim.type == 'Segment':
                            GL.glLineWidth(self.attributes.get('width',3.0))
                            restoreLineWidth = True
                elif self.item.type() == 'PointCloud':
                    lighting = False
                geometry = self.item
            if lighting:
                GL.glEnable(GL.GL_LIGHTING)
            else:
                GL.glDisable(GL.GL_LIGHTING)
            self.appearance.drawWorldGL(geometry)
            if restoreLineWidth:
                GL.glLineWidth(1.0)
            if name is not None:
                bmin,bmax = geometry.getBB()
                wp = vectorops.mul(vectorops.add(bmin,bmax),0.5)
                self.drawText(name,wp)
        else:
            try:
                itypes = self.attributes['type']
            except KeyError:
                try:
                    itypes = objectToVisType(item,world)
                except Exception as e:
                    import traceback
                    traceback.print_exc()
                    warnings.warn("Unsupported object type {} of type {}".format(item,item.__class__.__name__))
                    return
            if itypes is None:
                warnings.warn("Unable to convert item {} to drawable".format(str(item)))
                return
            elif itypes == 'Config':
                rindex = self.attributes.get("robot",0)
                if world and rindex < world.numRobots():
                    robot = world.robot(rindex)
                    if robot.numLinks() != len(item):
                        warnings.warn("Unable to draw Config %s, does not have the same # of DOFs as the robot: %d != %d"%(self.name,robot.numLinks(),len(item)))
                    else:
                        if not self.useDefaultAppearance:
                            oldAppearance = [robot.link(i).appearance().clone() for i in range(robot.numLinks())]
                            for i in range(robot.numLinks()):
                                if self.customAppearance is not None:
                                    robot.link(i).appearance().set(self.customAppearance)
                                elif "color" in self.attributes:
                                    robot.link(i).appearance().setColor(*self.attributes["color"])

                        oldconfig = robot.getConfig()
                        robot.setConfig(item)
                        robot.drawGL()
                        robot.setConfig(oldconfig)
                        if not self.useDefaultAppearance:
                            for (i,app) in enumerate(oldAppearance):
                                robot.link(i).appearance().set(app)
                else:
                    warnings.warn("Unable to draw Config items without a world or robot")
            elif itypes == 'Configs':
                def drawAsTrajectory():
                    width = self.attributes.get("width",3.0)
                    color = self.attributes["color"]
                    pointSize = self.attributes.get("pointSize",None)
                    pointColor = self.attributes.get("pointColor",None)
                    drawTrajectory(Trajectory(list(range(len(item))),item),width,color,pointSize,pointColor)
                if len(item) == 0:
                    pass
                elif world and world.numRobots() >= 1:
                    maxConfigs = self.attributes.get("maxConfigs",min(10,len(item)))
                    robot = world.robot(self.attributes.get("robot",0))
                    if robot.numLinks() != len(item[0]):
                        if len(item[0]) in [2,3]: #interpret as a trajectory
                            self.displayCache[0].draw(drawAsTrajectory)
                            centroid = item[0] + [0]*(3-len(item[0]))
                            if name is not None:
                                self.drawText(name,centroid)
                        else:
                            warnings.warn("Configs items aren't the right size for the robot")
                    if not self.useDefaultAppearance:
                        oldAppearance = [robot.link(i).appearance().clone() for i in range(robot.numLinks())]
                        for i in range(robot.numLinks()):
                          if self.customAppearance is not None:
                            robot.link(i).appearance().set(self.customAppearance)
                          elif "color" in self.attributes:
                            robot.link(i).appearance().setColor(*self.attributes["color"])

                    oldconfig = robot.getConfig()
                    for i in range(maxConfigs):
                        idx = int(i*len(item))//maxConfigs
                        robot.setConfig(item[idx])
                        robot.drawGL()
                    robot.setConfig(oldconfig)
                    if not self.useDefaultAppearance:
                        for (i,app) in enumerate(oldAppearance):
                            robot.link(i).appearance().set(app)
                elif len(item[0]) in [2,3]: #interpret as a trajectory
                    self.displayCache[0].draw(drawAsTrajectory)
                    centroid = item[0] + [0]*(3-len(item[0]))
                    if name is not None:
                        self.drawText(name,centroid)
                else:
                    warnings.warn("Unable to draw Configs items without a world or robot")
            elif itypes == 'Vector3':
                def drawRaw():
                    GL.glDisable(GL.GL_LIGHTING)
                    GL.glEnable(GL.GL_POINT_SMOOTH)
                    GL.glPointSize(self.attributes.get("size",5.0))
                    GL.glColor4f(*self.attributes.get("color",[0,0,0,1]))
                    GL.glBegin(GL.GL_POINTS)
                    GL.glVertex3f(0,0,0)
                    GL.glEnd()
                self.displayCache[0].draw(drawRaw,[so3.identity(),item])
                if name is not None:
                    self.drawText(name,item)
            elif itypes == 'RigidTransform':
                def drawRaw():
                    fancy = self.attributes.get("fancy",False)
                    if fancy: GL.glEnable(GL.GL_LIGHTING)
                    else: GL.glDisable(GL.GL_LIGHTING)
                    gldraw.xform_widget(se3.identity(),self.attributes.get("length",0.1),self.attributes.get("width",0.01),fancy=fancy)
                self.displayCache[0].draw(drawRaw,transform=item)
                if name is not None:
                    self.drawText(name,item[1])
            else:
                warnings.warn("Unable to draw item of type \"%s\""%(str(itypes),))

        #revert appearance
        if not self.useDefaultAppearance and hasattr(item,'appearance'):
            item.appearance().set(self.oldAppearance)

    def getBounds(self):
        """Returns a bounding box (bmin,bmax) or None if it can't be found"""
        if len(self.subAppearances)!=0:
            bb = bb_create()
            for n,app in self.subAppearances.items():
                bb = bb_union(bb,app.getBounds())
            return bb
        item = self.item
        if isinstance(item,coordinates.Point):
            return [item.worldCoordinates(),item.worldCoordinates()]
        elif isinstance(item,coordinates.Direction):
            T = item.frame().worldCoordinates()
            d = item.localCoordinates()
            L = self.attributes.get("length",0.1)
            return bb_create(T[1],se3.apply(T,vectorops.mul(d,L)))
        elif isinstance(item,coordinates.Frame):
            T = item.worldCoordinates()
            L = self.attributes.get("length",0.1)
            return bb_create(T[1],se3.apply(T,(L,0,0)),se3.apply(T,(0,L,0)),se3.apply(T,(0,0,L)))
        elif isinstance(item,ContactPoint):
            L = self.attributes.get("length",0.05)
            return bb_create(item.x,vectorops.madd(item.x,item.n,L))
        elif isinstance(item,WorldModel):
            pass
        elif hasattr(item,'geometry'):
            return item.geometry().getBB()
        elif hasattr(item,'getBB'):
            return item.getBB()
        elif isinstance(item,(str,VisPlot)):
            pass
        else:
            try:
                vtype = objectToVisType(item,None)
                if 'Vector3' == vtype:
                    #assumed to be a point
                    return (item,item)
                elif 'RigidTransform' == vtype:
                    #assumed to be a rigid transform
                    T = item
                    L = self.attributes.get("length",0.1)
                    return bb_create(T[1],se3.apply(T,(L,0,0)),se3.apply(T,(0,L,0)),se3.apply(T,(0,0,L)))
            except Exception:
                raise
                pass
            warnings.warn("Empty bound for object {} type {}".format(self.name,self.item.__class__.__name__))
        return bb_create()

    def getCenter(self):
        bb = self.getBounds()
        if bb_empty(bb):
            item = self.item
            if hasattr(item,'getCurrentTransform'):
                return item.getCurrentTransform()[1]
            elif hasattr(item,'getTransform'):
                return item.getTransform()[1]
            return [0,0,0]
        return vectorops.interpolate(bb[0],bb[1],0.5)

    def getTransform(self):
        if len(self.subAppearances) != 0:
            return (so3.identity(),self.getCenter())
        item = self.item
        if isinstance(item,coordinates.Frame):
            T = item.worldCoordinates()
            L = self.attributes.get("length",0.1)
            return (T[0],se3.apply(T,(L/2,L/2,L/2)))
        elif hasattr(item,'geometry'):
            return item.geometry().getCurrentTransform()
        elif hasattr(item,'getCurrentTransform'):
            return item.getCurrentTransform()
        elif hasattr(item,'getTransform'):
            return item.getTransform()
        else:
            try:
                vtype = objectToVisType(item,None)
                if 'RigidTransform' == vtype:
                    T = item
                    L = self.attributes.get("length",0.1)
                    return (T[0],se3.apply(T,(L/2,L/2,L/2)))
            except Exception:
                raise
        return (so3.identity(),self.getCenter())
    
    def rayCast(self,source,direction,tolerance=0.01):
        """Returns a point on the item intersected by the ray
        ``source + t*direction``, or None if it does not intersect.

        For non-volumetric items, the item is treated as though it
        were expanded by ``tolerance`` units.
        """
        geom = None
        if isinstance(self.item,Geometry3D):
            geom = self.item
        elif hasattr(self.item,'geometry') and callable(self.item.geometry):
            geom = self.item.geometry()
        #TODO: ray cast other items like points, transforms, trajectories
        if geom is None: return None
        margin = geom.getCollisionMargin()
        if geom.type() == 'PointCloud':
            geom.setCollisionMargin(tolerance)
        elif geom.type() == 'GeometricPrimitive' and geom.getGeometricPrimitive().type in ['Point','Segment','Line']:
            geom.setCollisionMargin(tolerance)
        (hit,pt) = geom.rayCast(source,direction)
        geom.setCollisionMargin(margin)  #restore margin
        return pt if hit else None

    def getSubItem(self,path):
        if len(path) == 0: return self
        for k,v in self.subAppearances.items():
            if v.name == path[0]:
                try:
                    return v.getSubItem(path[1:])
                except ValueError as e:
                    raise ValueError("Invalid sub-path specified "+str(path)+" at "+str(e))
        raise ValueError("Invalid sub-item specified "+str(path[0]))

    def make_editor(self,world=None):
        if self.editor is not None:
            return 
        item = self.item
        if isinstance(item,coordinates.Point):
            res = PointPoser()
            res.set(item.worldCoordinates())
            res.setAxes(item.frame().worldCoordinates()[0])
        elif isinstance(item,coordinates.Direction):
            res = PointPoser()
            res.set(item.worldCoordinates())
            res.setAxes(item.frame().worldCoordinates()[0])
        elif isinstance(item,coordinates.Frame):
            res = TransformPoser()
            res.set(*item.worldCoordinates())
        elif isinstance(item,RobotModel):
            res = RobotPoser(item)
        elif isinstance(item,SubRobotModel):
            res = RobotPoser(item._robot)
            res.setActiveDofs(item._links);
        elif isinstance(item,RigidObjectModel):
            res = ObjectPoser(item)
        elif isinstance(item,(list,tuple)):
            #determine if it's a rotation, transform, or point
            itype = objectToVisType(item,None)
            if itype == 'Vector3':
                res = PointPoser()
                res.set(item)
            elif itype == 'Matrix3':
                res = TransformPoser()
                res.enableRotation(True)
                res.enableTranslation(False)
                res.set(item)
            elif itype == 'RigidTransform':
                res = TransformPoser()
                res.enableRotation(True)
                res.enableTranslation(True)
                res.set(*item)
            elif itype == 'Config':
                if world is not None and world.numRobots() > 0 and world.robot(0).numLinks() == len(item):
                    #it's a valid configuration
                    oldconfig = world.robot(0).getConfig()
                    world.robot(0).setConfig(item)
                    res = RobotPoser(world.robot(0))
                    world.robot(0).setConfig(oldconfig)
                else:
                    warnings.warn("VisAppearance.make_editor(): Editor for object of type {} cannot be associated with a robot".format(itype))
                    return
            else:
                warnings.warn("VisAppearance.make_editor(): Editor for object of type {} not defined".format(itype))
                return
        else:
            warnings.warn("VisAppearance.make_editor(): Editor for object of type {} not defined".format(item.__class__.__name__))
            return
        self.editor = res

    def update_editor(self,item_to_editor=False):
        for (name,item) in self.subAppearances.items():
            item.update_editor(item_to_editor)
        if self.editor is None:
            return
        item = self.item
        if item_to_editor:
            if isinstance(item,coordinates.Point):
                self.editor.set(self.item.worldCoordinates())
            elif isinstance(item,coordinates.Direction):
                self.editor.set(self.item.worldCoordinates())
            elif isinstance(item,coordinates.Frame):
                self.editor.set(*self.item.worldCoordinates())
            elif isinstance(self.item,RobotModel):
                self.editor.set(self.item.getConfig())
            elif isinstance(self.item,SubRobotModel):
                self.editor.set(self.item.tofull(self.item.getConfig()))
            elif isinstance(self.item,RigidObjectModel):
                self.editor.set(*self.item.getTransform())
            elif isinstance(self.item,(list,tuple)):
                itype = objectToVisType(self.item,None)
                if itype in ('Vector3','Matrix3'):
                    self.editor.set(self.item)
                elif itype == 'RigidTransform':
                    self.editor.set(*self.item)
                elif itype == 'Config':
                    self.editor.set(self.item)
            else:
                raise RuntimeError("Uh... unsupported type with an editor?")
        else:
            if not self.editor.hasFocus():
                return
            if isinstance(item,coordinates.Point):
                self.item._localCoordinates = se3.apply(se3.inv(self.item._frame.worldCoordinates()),self.editor.get())
            elif isinstance(item,coordinates.Direction):
                self.item._localCoordinates = se3.apply(se3.inv(self.item._frame.worldCoordinates()),self.editor.get())
            elif isinstance(item,coordinates.Frame):  
                self.item._worldCoordinates = self.editor.get()
                self.item._relativeCoordinates = se3.mul(se3.inv(self.item.parent().worldCoordinates()),self.editor.get())
                #TODO: updating downstream frames?
            elif isinstance(self.item,RobotModel):
                self.item.setConfig(self.editor.getConditioned(self.item.getConfig()))
            elif isinstance(self.item,SubRobotModel):
                self.item.setConfig(self.item.fromfull(self.editor.get()))
            elif isinstance(self.item,RigidObjectModel):
                self.item.setTransform(*self.editor.get())
            elif isinstance(self.item,(tuple,list)):
                def setList(a,b):
                    if isinstance(a,(list,tuple)) and isinstance(b,(list,tuple)):
                        if len(a) == len(b):
                            for i in range(len(a)):
                                if not setList(a[i],b[i]):
                                    if isinstance(a,list):
                                        a[i] = b[i]
                                    else:
                                        return False
                            return True
                    return False
                v = self.editor.get()
                if not setList(self.item,v):
                    self.item = v
            elif isinstance(self.item,tuple):
                warnings.warn("Edited a tuple... maybe a point or an xform? can't actually edit")
                self.item = self.editor.get()
            else:
                raise RuntimeError("Uh... unsupported type with an editor?")
                
    def remove_editor(self):
        self.editor = None


class VisualizationScene:
    """Holds all of the visualization information for a scene, including
    labels, edit status, and animations.
    
    Attributes:
        items (dict of str->VisAppearance): the top level items in the scene.
        labels (list): internally used
        t (float): current scene time, internally used
        timeCallback (callable): a function time()->float that is used to get
            the time update.  If None, ``time.time()`` is used.
        startTime (float): first time that updateTime() was called, internally
            used.
        doRefresh (bool): whether any item wants to refresh the visualization.
        cameraController: if given, a class to update the camera whenever
            updateCamera is called.

    """
    def __init__(self):
        self.items = {}
        self.labels = []
        self.t = 0
        self.timeCallback = None
        self.startTime = None
        self.animating = True
        self.currentAnimationTime = 0
        self.doRefresh = False
        self.cameraController = None

    def getItem(self,item_name):
        """Returns an VisAppearance according to the given name or path"""
        if isinstance(item_name,(list,tuple)):
            components = item_name
            if len(components)==1: 
                return self.getItem(components[0])
            if components[0] not in self.items:
                raise ValueError("Invalid top-level item specified: "+str(item_name))
            return self.items[components[0]].getSubItem(components[1:])
        if item_name in self.items:
            return self.items[item_name]

    def dirty(self,item_name='all'):
        """Marks an item or everything as dirty, forcing a deep redraw."""
        global _globalLock
        with _globalLock:
            if item_name == 'all':
                for (name,itemvis) in self.items.items():
                    itemvis.markChanged()
                    if hasattr(itemvis,'appearance'):
                        itemvis.appearance.refresh(True)
            else:
                item = self.getItem(item_name)
                item.markChanged()
                if hasattr(item,'appearance'):
                    item.appearance.refresh(True)

    def clear(self):
        """Clears the visualization world"""
        global _globalLock
        with _globalLock:
            for (name,itemvis) in self.items.items():
                itemvis.destroy()
            self.items = {}
            self.currentAnimationTime = 0
            self.doRefresh = True

    def clearText(self):
        """Clears all text in the visualization."""
        global _globalLock
        with _globalLock:
            del_items = []
            for (name,itemvis) in self.items.items():
                if isinstance(itemvis.item,str):
                    itemvis.destroy()
                    del_items.append(name)
            for n in del_items:
                del self.items[n]

    def listItems(self,root=None,indent=0):
        """Prints out all items in the visualization world."""
        if root is None:
            for name,value in self.items.items():
                self.listItems(value,indent)
        else:
            if isinstance(root,str):
                root = self.getItem(root)
            if indent > 0:
                print(" "*(indent-1), end=' ')
            print(root.name)
            for n,v in root.subAppearances.items():
                self.listItems(v,indent+2)

    def getItemName(self,object):
        """Retrieves a str or tuple of strs giving the path to an object"""
        name = None
        if hasattr(object,'getName'):
            name = object.getName()
        elif hasattr(object,'name'):
            if callable(object.name):
                name = object.name()
            else:
                name = object.name
        if name is not None and name in self.items:
            if self.items[name].item is object:
                return name
        if isinstance(object,RobotModelLink):  #a link?
            robot = object.robot()
            robpath = self.getItemName(robot)
            if robpath:
                return robpath + (name,)
            else:
                warnings.warn("Couldnt find link {} under robot".format(name))
        def objectPath(app,obj,name):
            if app.item == object:
                return ()
            if name is not None and app.name == name:
                return ()
            for (subitem,subapp) in app.subAppearances.items():
                subpath = objectPath(subapp,obj,name)
                if subpath is not None:
                    return (subapp.name,)+subpath
            return None
        if hasattr(object,'world'):
            #look through the world for the object
            world = self.items.get('world',None)
            if world is not None:
                p = objectPath(self.items['world'],object,name)
                if p is not None:
                    return ('world',)+p
                else:
                    warnings.warn("Couldnt find object {} under world".format(name))
        for (k,v) in self.items.items():
            p = objectPath(v,object,name)
            if p is not None:
                if len(p)==0:  #top level item
                    return k
                return (k,)+p
            else:
                warnings.warn("Couldnt find object {} under {}".format(name,k))
        return None

    def add(self,name,item,keepAppearance=False,**kwargs):
        """Adds a named item to the visualization world.  If the item already
        exists, the appearance information will be reinitialized if keepAppearance=False
        (default) or be kept if keepAppearance=True."""
        global _globalLock
        assert not isinstance(name,(list,tuple)),"Cannot add sub-path items"
        with _globalLock:
            if keepAppearance and name in self.items:
                self.items[name].setItem(item)
            else:
                #need to erase prior item visualizer
                if name in self.items:
                    self.items[name].destroy()

                type = kwargs.get('type',None)
                if type is None:
                    world = self.items.get('world',None)
                    if world is not None: world = world.item
                    try:
                        type = objectToVisType(item,world)
                    except Exception:
                        type = None
                app = VisAppearance(item,name,type)
                self.items[name] = app
            item = self.items[name]
            for (attr,value) in kwargs.items():
                self._setAttribute(item,attr,value)
        #self.refresh()

    def addText(self,name,text,**kwargs):
        self.add(name,text,True,**kwargs)

    def animate(self,name,animation,speed=1.0,endBehavior='loop'):
        global _globalLock
        with _globalLock:
            if hasattr(animation,'__iter__'):
                #a list of milestones -- loop through them with 1s delay
                print("visualization.animate(): Making a Trajectory with unit durations between",len(animation),"milestones")
                animation = Trajectory(list(range(len(animation))),animation)
            if isinstance(animation,MultiPath):
                world = self.items.get('world',None)
                if world is not None:
                    world=world.item
                    if world.numRobots() > 0:
                        #discretize multipath
                        robot = world.robot(0)
                        animation = animation.getTrajectory(robot,0.1)
                    else:
                        animation = animation.getTrajectory()
                else:
                    animation = animation.getTrajectory()
            assert isinstance(animation,Trajectory) or animation is None,"Must animate() with a Trajectory object or list of milestones"
            item = self.getItem(name)
            item.animation = animation
            item.animationStartTime = self.currentAnimationTime
            item.animationSpeed = speed
            item.animationEndBehavior = endBehavior
            item.markChanged(config=True,appearance=False)

    def pauseAnimation(self,paused=True):
        global _globalLock
        with _globalLock:
            self.animating = not paused

    def stepAnimation(self,amount):
        global _globalLock
        with _globalLock:
            self.animationTime(self.currentAnimationTime + amount)

    def animationTime(self,newtime=None):
        global _globalLock
        if newtime is None:
            #query mode
            return self.currentAnimationTime

        #update mode
        with _globalLock:
            self.currentAnimationTime = newtime
            self.doRefresh = True
            for (k,v) in self.items.items():
                #do animation updates
                v.updateAnimation(self.currentAnimationTime)
        return 

    def remove(self,name):
        global _globalLock
        with _globalLock:
            assert name in self.items,"Can only remove top level objects from visualization, try hide() instead"
            item = self.getItem(name)
            item.destroy()
            del self.items[name]
            self.doRefresh = True

    def getItemConfig(self,name):
        global _globalLock
        with _globalLock:
            res = config.getConfig(self.getItem(name).item)
        return res

    def setItemConfig(self,name,value):
        global _globalLock
        with _globalLock:
            item = self.getItem(name)
            if isinstance(item.item,(list,tuple)):
                #TODO: broadcast value to the shape of item
                item.item = value
            elif isinstance(item.item,str):
                item.item = value
            else:
                config.setConfig(item.item,value)
            if item.editor:
                item.update_editor(item_to_editor = True)
            item.markChanged(config=True,appearance=False)
            self.doRefresh = True

    def addLabel(self,text,point,color):
        global _globalLock
        with _globalLock:
            self.labels.append((text,point,color))

    def hideLabel(self,name,hidden=True):
        global _globalLock
        with _globalLock:
            item = self.getItem(name)
            item.attributes["hide_label"] = hidden
            item.markChanged(config=False,appearance=True)
            self.doRefresh = True

    def hide(self,name,hidden=True):
        global _globalLock
        with _globalLock:
            self.getItem(name).attributes['hidden'] = hidden
            self.doRefresh = True

    def addPlotItem(self,plotname,itemname):
        global _globalLock
        with _globalLock:
            plot = self.getItem(plotname)
            assert plot is not None and isinstance(plot.item,VisPlot),(plotname+" is not a valid plot")
            plot = plot.item
            for i in plot.items:
                assert i.name != itemname,(str(itemname)+" is already in the plot "+plotname)
            item = self.getItem(itemname)
            assert item is not None,(str(itemname)+" is not a valid item")
            plot.items.append(VisPlotItem(itemname,item))

    def logPlot(self,plotname,itemname,value):
        global _globalLock
        with _globalLock:
            customIndex = -1
            plot = self.getItem(plotname)
            assert plot is not None and isinstance(plot.item,VisPlot),(plotname+" is not a valid plot")
            compress = plot.attributes['compress']
            plot = plot.item
            for i,item in enumerate(plot.items):
                if len(item.name)==0:
                    customIndex = i
            if customIndex < 0:
                customIndex = len(plot.items)
                plot.items.append(VisPlotItem('',None))
            t = self.t
            if self.startTime is not None:
                if self.timeCallback is None:
                    t = time.time() - self.startTime
                else:
                    t = self.timeCallback() - self.startTime
            else:
                t = 0
            plot.items[customIndex].compressThreshold = compress
            plot.items[customIndex].customUpdate(itemname,t,value)

    def logPlotEvent(self,plotname,eventname,color):
        global _globalLock
        with _globalLock:
            plot = self.getItem(plotname)
            assert plot is not None and isinstance(plot.item,VisPlot),(plotname+" is not a valid plot")
            t = self.t
            if self.startTime is not None:
                if self.timeCallback is None:
                    t = time.time() - self.startTime
                else:
                    t = self.timeCallback() - self.startTime
            else:
                t = 0
            plot.item.addEvent(eventname,t,color)

    def hidePlotItem(self,plotname,itemname,hidden=True):
        global _globalLock
        with _globalLock:
            plot = self.getItem(plotname)
            assert plot is not None and isinstance(plot.item,VisPlot),plotname+" is not a valid plot"
            plot = plot.item
            identified = False
            if isinstance(itemname,(tuple,list)):
                for i in plot.items:
                    if i.name == itemname[0]:
                        assert itemname[1] < len(i.hidden),("Invalid component index of item "+str(itemname[0]))
                        identified = True
                        i.hidden[itemname] = hidden
            else:
                for i in plot.items:
                    if i.name == itemname:
                        for j in range(len(i.hidden)):
                            i.hidden[j] = hidden
            assert identified,("Invalid item "+str(itemname)+" specified in plot "+plotname)
            self.doRefresh = True

    def savePlot(self,plotname,fn):
        global _globalLock
        with _globalLock:
            plot = self.getItem(plotname)
            assert plot is not None and isinstance(plot.item,VisPlot),plotname+" is not a valid plot"
            plot = plot.item
            if fn is not None:
                plot.beginSave(fn)
            else:
                plot.endSave(fn)

    def setAppearance(self,name,appearance):
        global _globalLock
        with _globalLock:
            item = self.getItem(name)
            item.useDefaultAppearance = False
            item.customAppearance = appearance
            item.markChanged(config=False,appearance=True)
            self.doRefresh = True

    def _setAttribute(self,item,attr,value):
        """Internal use only"""
        item.attributes[attr] = value
        if value==None:
            del item.attributes[attr]
        if attr=='color':
            item.useDefaultAppearance = False
        if len(item.subAppearances) > 0 and attr not in ['label','hidden']:
            #some attributes don't get inherited
            for n,app in item.subAppearances.items():
                self._setAttribute(app,attr,value)
        if attr=='type':
            #modify the parent attributes
            item.attributes.setParent(_default_attributes(item.item,type=value))
        item.markChanged(config=False,appearance=True)

    def setAttribute(self,name,attr,value):
        global _globalLock
        with _globalLock:
            item = self.getItem(name)
            if item is None:
                raise ValueError("Item "+str(name)+" doesn't exist in scene")
            self._setAttribute(item,attr,value)
            self.doRefresh = True

    def getAttribute(self,name,attr):
        global _globalLock
        with _globalLock:
            item = self.getItem(name)
            res = item.attributes[attr]
        return res

    def getAttributes(self,name):
        global _globalLock
        with _globalLock:
            item = self.getItem(name)
            res = item.getAttributes()
        return res

    def revertAppearance(self,name):
        global _globalLock
        with _globalLock:
            item = self.getItem(name)
            item.useDefaultAppearance = True
            item.markChanged(config=False,appearance=True)
            self.doRefresh = True

    def setColor(self,name,r,g,b,a=1.0):
        global _globalLock
        with _globalLock:
            item = self.getItem(name)
            self._setAttribute(item,"color",[r,g,b,a])
            item.markChanged(config=False,appearance=True)
            self.doRefresh = True

    def setDrawFunc(self,name,func):
        global _globalLock
        with _globalLock:
            item = self.getItem(name)
            item.customDrawFunc = func
            self.doRefresh = True

    def autoFitCamera(self,zoom=True,rotate=True,scale=1.0):
        vp = self.getViewport()
        try:
            autoFitViewport(vp,list(self.items.values()),zoom=zoom,rotate=rotate)
            vp.camera.dist /= scale
            self.setViewport(vp)
        except Exception as e:
            warnings.warn("Unable to auto-fit camera")
            import traceback
            traceback.print_exc()

    def followCamera(self,target,translate,rotate,center):
        if target is None:
            self.cameraController = None
            return
        vp = self.getViewport()
        if isinstance(target,str) or isinstance(target,(tuple,list)):
            try:
                target = self.getItem(target)
            except KeyError:
                raise ValueError("Invalid item "+str(target))
            target_center = target.getCenter()
            if center:
                if translate:
                    _camera_translate(vp,target_center)
                else:
                    _camera_lookat(vp,target_center)

            if translate and rotate:
                self.cameraController = _TrackingCameraController(vp,target)
            elif translate:
                self.cameraController = _TranslatingCameraController(vp,target)
            elif rotate:
                self.cameraController = _TargetCameraController(vp,target)
            else:
                self.cameraController = None
        elif isinstance(target,Trajectory):
            self.cameraController = _TrajectoryCameraController(vp,target)
        elif isinstance(target,SimRobotSensor):
          self.cameraController = _SimCamCameraController(vp,target)
        else:
            raise ValueError("Invalid value for target, must either be str or a Trajectory")

    def setTimeCallback(self,cb):
        """Sets a callback in updateTime() to set the current time"""
        self.timeCallback = cb

    def updateTime(self,t=None):
        """The backend will call this during an idle loop to update the
        visualization time.  This may also update animations if currently
        animating."""
        if t is None:
            if self.timeCallback is None:
                t = time.time()
            else:
                t = self.timeCallback()
        if self.startTime is None:
          self.startTime = t
        oldt = self.t
        self.t = t-self.startTime
        if self.t - oldt < 0:
            warnings.warn("Time is going negative?")
        if self.animating:
            if self.t - oldt > 0:
                self.stepAnimation(self.t - oldt)
        for (k,v) in self.items.items():
            #do other updates
            v.updateTime(self.t)

    def updateCamera(self):
        """Updates the camera, if controlled.  The backend should call
        this whenever the scene is to be drawn."""
        if self.cameraController is not None:
            vp = self.cameraController.update(self.currentAnimationTime)
            if vp is not None:
                self.setViewport(vp)

    def edit(self,name,doedit=True):
        raise NotImplementedError("Needs to be implemented by subclass")
    
    def pick(self,click_callback,hover_callback,highlight_color,filter,tolerance):
        raise NotImplementedError("Needs to be implemented by subclass")

    def getViewport(self):
        raise NotImplementedError("Needs to be implemented by subclass")

    def setViewport(self,viewport):
        raise NotImplementedError("Needs to be implemented by subclass")

    def setBackgroundColor(self,r,g,b,a=1): 
        raise NotImplementedError("Needs to be implemented by subclass")

    def renderGL(self,view):
        """Renders the scene in OpenGL"""
        vp = view.to_viewport()
        self.labels = []
        world = self.items.get('world',None)
        if world is not None: world=world.item
        #draw solid items first
        delayed = []
        for (k,v) in self.items.items():
            transparent = v.transparent()
            if transparent is not False:
                delayed.append(k)
                if transparent is True:
                    continue
            v.widget = self
            v.swapDrawConfig()
            v.drawGL(world,viewport=vp,draw_transparent=False)
            v.swapDrawConfig()
            #allows garbage collector to delete these objects
            v.widget = None 

        for k in delayed:
            v = self.items[k]
            v.widget = self
            v.swapDrawConfig()
            v.drawGL(world,viewport=vp,draw_transparent=True)
            v.swapDrawConfig()
            #allows garbage collector to delete these objects
            v.widget = None 

        #cluster label points and draw labels
        pointTolerance = view.camera.dist*0.03
        if pointTolerance > 0:
            pointHash = {}
            for (text,point,color) in self.labels:
                index = tuple([int(x/pointTolerance) for x in point])
                try:
                    pointHash[index][1].append((text,color))
                except KeyError:
                    pointHash[index] = [point,[(text,color)]]
            for (p,items) in pointHash.values():
                self._renderGLLabelRaw(view,p,*list(zip(*items)))

    def renderScreenGL(self,view,window):
        cx = 20
        cy = 20
        GL.glDisable(GL.GL_LIGHTING)
        GL.glDisable(GL.GL_DEPTH_TEST)
        for (k,v) in self.items.items():
            if isinstance(v.item,VisPlot) and not v.attributes['hidden']:
                pos = v.attributes['position']
                duration = v.attributes['duration']
                vrange = v.attributes['range']
                w,h = v.attributes['size']
                if pos is None:
                    v.item.renderGL(window,cx,cy,w,h,duration,vrange[0],vrange[1])
                    cy += h+18
                else:
                    x = pos[0]
                    y = pos[1]
                    if x < 0:
                        x = view.w + x
                    if y < 0:
                        y = view.h + y
                    v.item.renderGL(window,x,y,w,h,duration,vrange[0],vrange[1])
        for (k,v) in self.items.items():
            if isinstance(v.item,str) and not v.attributes['hidden']:
                pos = v.attributes['position']
                col = v.attributes['color']
                size = v.attributes['size']
                if pos is None:
                    #draw at console
                    window.draw_text((cx,cy+size),v.item,size,col)
                    cy += (size*15)/10
                elif len(pos)==2:
                    x = pos[0]
                    y = pos[1]
                    if x < 0:
                        x = view.w + x
                    if y < 0:
                        y = view.h + y
                    window.draw_text((x,y+size),v.item,size,col)
        GL.glEnable(GL.GL_DEPTH_TEST)

    def _renderGLLabelRaw(self,view,point,textList,colorList):
        #assert not self.makingDisplayList,"drawText must be called outside of display list"
        assert self.window is not None
        invCameraRot = so3.inv(view.camera.matrix()[0])
        for i,(text,c) in enumerate(zip(textList,colorList)):
            if i+1 < len(textList): text = text+","

            projpt = view.project(point,clip=False)
            if projpt[2] > view.clippingplanes[0]:
                d = float(12)/float(view.w)*projpt[2]*0.7
                point = vectorops.add(point,so3.apply(invCameraRot,(0,-d,0)))

            GL.glDisable(GL.GL_LIGHTING)
            GL.glDisable(GL.GL_DEPTH_TEST)
            GL.glColor3f(*c)
            self.draw_text(point,text,size=12)
            GL.glEnable(GL.GL_DEPTH_TEST)
            
    def clearDisplayLists(self):
        for i in self.items.values():
            i.clearDisplayLists()

    def rayCast(self,x,y,filter=None,tolerance=0.01):
        """Performs ray casting with the scene

        Args:
            x (float): the screen x-coordinate of the mouse cursor
            y (float): the screen y-coordinate of the mouse cursor
            filter (callable, optional): if given, a function f(name,item) that
                returns True if the item should be considered.
            tolerance (float, optional): a tolerance for ray intersection with
                non-volumetric items.
        
        Returns:
            tuple: a pair ``(visapp,pt)`` with visapp the first intersected 
            :class:`VisAppearance` and pt the clicked point.  If nothing 
            intersects, then (None,None) is returned.
        """
        (s,d) = self.view.click_ray(x,y)
        data = [(None,None),float('inf')]  #closest and closest distance
        def doclick(name,visappearance,data=data):
            item = visappearance.item
            if filter is not None or filter(name,item):
                if len(visappearance.subAppearances) != 0:
                    for sname,app in visappearance.subAppearances.items():
                        doclick(sname,app)
                else:
                    pt = visappearance.rayCast(s,d,tolerance)
                    if pt is not None:
                        dist = vectorops.dot(vectorops.sub(pt,s),d)
                        if dist < data[1]:
                            data[0] = (visappearance,pt)
                            data[1] = dist
        for name,item in self.items.items():
            doclick(name,item)
        return data[0]

    def saveJsonConfig(self,fn=None):
        def dumpitem(v):
            if len(v.subAppearances) > 0:
                items = []
                for (k,app) in v.subAppearances.items():
                    jsapp = dumpitem(app)
                    if len(jsapp) > 0:
                        items.append({"name":k,"appearance":jsapp})
                return items
            else:
                return v.attributes.flatten()
        out = {}
        for (k,v) in self.items.items():
            out[k] = dumpitem(v) 
        if fn is None:
            return out
        else:
            import json
            f = open(fn,'w')
            json.dump(out,f)
            f.close()
            return out
                
    def loadJsonConfig(self,jsonobj_or_file):
        if isinstance(jsonobj_or_file,str):
            import json
            f = open(jsonobj_or_file,'r')
            jsonobj = json.load(f)
            f.close()
        else:
            jsonobj = jsonobj_or_file

        def parseitem(js,app):
            if isinstance(js,dict):
                for (attr,value) in js.items():
                    app.attributes[attr] = value
                    app.markChanged()
            elif isinstance(js,list):
                for val in js:
                    if not isinstance(val,dict) or "name" not in val or "appearance" not in val:
                        warnings.warn("JSON object {} does not contain a valid subappearance".format(js))
                    name = val["name"]
                    jsapp = val["appearance"]
                    if isinstance(name,list):
                        name = tuple(name)
                    if name not in app.subAppearances:
                        warnings.warn("JSON object {} subappearance {} not in visualization".format(js,name))
                    else:
                        parseitem(jsapp,app.subAppearances[name])
            else:
                warnings.warn("JSON object {} does not contain a dict of attributes or list of sub-appearances".format(js))

        parsed = set()
        for (k,v) in self.items.items():
            if k in jsonobj:
                parsed.add(k)
                parseitem(jsonobj[k],v)
            else:
                warnings.warn("Visualization object {} not in JSON object".format(k))
        for (k,v) in jsonobj.items():
            if k not in parsed:
                warnings.warn("JSON object {} not in visualization".format(k))


def _camera_translate(vp,tgt):
    vp.camera.tgt = tgt

def _camera_lookat(vp,tgt):
    T = vp.get_transform()
    vp.camera.tgt = tgt
    vp.camera.dist = max(vectorops.distance(T[1],tgt),0.1)
    #set R to point at target
    zdir = vectorops.unit(vectorops.sub(tgt,T[1]))
    xdir = vectorops.unit(vectorops.cross(zdir,[0,0,1]))
    ydir = vectorops.unit(vectorops.cross(zdir,xdir))
    R = xdir + ydir + zdir
    vp.camera.set_orientation(R,'xyz')

class _TrackingCameraController:
    def __init__(self,vp,target):
        self.vp = vp
        T = vp.get_transform()
        target.swapDrawConfig()
        self.viewportToTarget = se3.mul(se3.inv(target.getTransform()),T)
        target.swapDrawConfig()
        self.target = target
    def update(self,t):
        self.target.swapDrawConfig()
        T = se3.mul(self.target.getTransform(),self.viewportToTarget)
        self.target.swapDrawConfig()
        self.vp.set_transform(T)
        return self.vp

class _TranslatingCameraController:
    def __init__(self,vp,target):
        self.vp = vp
        target.swapDrawConfig()
        self.last_target_pos = target.getCenter()
        target.swapDrawConfig()
        self.target = target
    def update(self,t):
        self.target.swapDrawConfig()
        t = self.target.getCenter()
        self.target.swapDrawConfig()
        self.vp.camera.tgt = vectorops.add(self.vp.camera.tgt,vectorops.sub(t,self.last_target_pos))
        self.last_target_pos = t
        return self.vp

class _TargetCameraController:
    def __init__(self,vp,target):
        self.vp = vp
        target.swapDrawConfig()
        self.last_target_pos = target.getCenter()
        target.swapDrawConfig()
        self.target = target
    def update(self,t):
        self.target.swapDrawConfig()
        t = self.target.getCenter()
        self.target.swapDrawConfig()
        tgt = vectorops.add(self.vp.camera.tgt,vectorops.sub(t,self.last_target_pos))
        self.last_target_pos = t
        _camera_lookat(self.vp,tgt)
        return self.vp

class _TrajectoryCameraController:
    def __init__(self,vp,trajectory):
        self.vp = vp
        if isinstance(trajectory,SE3Trajectory):
            pass
        elif isinstance(trajectory,SO3Trajectory):
            pass
        else:
            assert isinstance(trajectory,Trajectory)
            pass
        self.trajectory = trajectory
    def update(self,t):
        if isinstance(self.trajectory,(SE3Trajectory,SE3HermiteTrajectory)):
            T = self.trajectory.eval(t,'loop')
            self.vp.set_transform(T)
        elif isinstance(self.trajectory,(SO3Trajectory,SO3HermiteTrajectory)):
            R = self.trajectory.eval(t,'loop')
            self.vp.camera.set_orientation(R,'xyz')
        else:
            trans = self.trajectory.eval(t,'loop')
            T = self.vp.get_transform()
            ofs = vectorops(self.vp.tgt,T[0])
            self.vp.camera.tgt = vectorops.add(trans,ofs)
        return self.vp

class _SimCamCameraController:
    def __init__(self,vp,target):
        self.vp = vp
        self.target = target
    def update(self,t):
        from ..model import sensing
        T = sensing.get_sensor_xform(self.target,self.target.robot())
        self.vp.set_transform(T)
        return self.vp


class _WindowManager:
    def reset(self):
        raise NotImplementedError()
    def frontend(self):
        raise NotImplementedError()
    def createWindow(self,title):
        raise NotImplementedError()
    def setWindow(self,id):
        raise NotImplementedError()
    def getWindow(self):
        raise NotImplementedError()
    def scene(self):
        raise NotImplementedError()
    def getWindowName(self):
        raise NotImplementedError()
    def setWindowName(self,name):
        raise NotImplementedError()
    def resizeWindow(self,w,h):
        raise NotImplementedError()
    def setPlugin(self,plugin):
        raise NotImplementedError()
    def pushPlugin(self,plugin):
        raise NotImplementedError()
    def popPlugin(self):
        raise NotImplementedError()
    def splitView(self,plugin):
        raise NotImplementedError()
    def multithreaded(self):
        return False
    def run(self):
        raise NotImplementedError()
    def loop(self,setup,callback,cleanup):
        raise NotImplementedError()
    def spin(self,duration):
        raise NotImplementedError()
    def show(self):
        raise NotImplementedError()
    def shown(self):
        raise NotImplementedError()
    def hide(self):
        raise NotImplementedError()
    def dialog(self):
        raise NotImplementedError()
    def lock(self):
        global _globalLock
        _globalLock.acquire()
    def unlock(self):
        global _globalLock
        _globalLock.release()
    def update(self):
        pass
    def cleanup(self):
        pass
    def kill(self):
        pass
    def threadCall(self,func):
        func()
    def screenshot(self,*args):
        raise NotImplementedError()
    def screenshotCallback(self,fn,*args):
        raise NotImplementedError()


class _ThreadedWindowManager(_WindowManager):
    def __init__(self):
        #signals to visualization thread
        self.quit = False
        self.in_vis_loop = False
        self.vis_thread_running = False
        self.vis_thread = None
        self.in_app_thread = False
        self.threadcalls = []

    def reset(self):
        self.kill()
        self.in_app_thread = False
        self.threadcalls = []

    def run_app_thread(self,callback):
        raise NotImplementedError()

    def multithreaded(self):
        return (True if sys.platform != 'darwin' else False)

    def kill(self):
        self.quit = True
        if self.in_vis_loop:
            #if the thread is running, _quit=True just signals to the vis loop to quit.
            #otherwise, the program needs to be killed and cleaned up
            if not self.vis_thread_running:
                #need to clean up Qt resources
                self.cleanup()
            return
        if self.vis_thread_running:
            if self.vis_thread is not None:
                self.vis_thread.join()
                self.vis_thread = None
            else:
                #should we be tolerant to weird things that happen with kill?
                self.vis_thread_running = False
            assert self.vis_thread_running == False

        self.quit = False

    def loop(self,setup,callback,cleanup):
        if self.vis_thread_running or self.in_vis_loop:
            if setup is not None:
                setup()
            self.show()
            dt = 1.0/30.0
            while self.shown():
                t0 = time.time()
                if callback is not None:
                    self.lock()
                    callback()
                    self.unlock()
                t1 = time.time()
                time.sleep(max(0,dt-(t1-t0)))
            if cleanup is not None:
                cleanup()
            return
            raise RuntimeError("Cannot call loop() after show(), inside dialog(), or inside loop() callbacks")
        self.in_vis_loop = True
        try:
            if setup is not None:
                setup()
            self.quit = False
            self.run_app_thread(callback)
            if cleanup is not None:
                cleanup()
        finally:
            self.in_vis_loop = False

    def spin(self,duration):
        if self.in_vis_loop:
            raise RuntimeError("spin() cannot be used inside loop()")
        if self.multithreaded():
            #use existing thread
            self.show()
            t = 0
            while t < duration:
                if not self.shown(): break
                time.sleep(min(0.04,duration-t))
                t += 0.04
            self.hide()
        else:
            #use single thread
            t0 = time.time()
            def timed_break():
                t1 = time.time()
                if t1 - t0 >= duration:
                    self.hide()
            self.loop(callback=timed_break,setup=lambda:self.show(),cleanup=None)
        return

    def run(self):
        if self.vis_thread_running:
            #already multithreaded, can't go back to single thread
            self.show()
            while self.shown():
                time.sleep(0.1)
        else:
            #run in a single thread
            self.loop(setup=None,callback=None,cleanup=None)

    def _start_app_thread(self):
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        self.vis_thread = threading.Thread(target=self.run_app_thread)
        self.vis_thread.setDaemon(True)
        self.vis_thread.start()
        time.sleep(0.1)

    def threadCall(self,func):
        self.threadcalls.append(func)

