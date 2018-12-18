"""Klamp't visualization routines.  See Python/demos/vistemplate.py for an
example of how to run this module.

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
- Unified interface to PyQt and GLUT (with loss of resource editing functionality
  under GLUT)
- Automatic camera setup

The resource editing functionality in the klampt.io.resource module (based on 
klampt.vis.editors) use this module as well.

There are two primary modes of running the visualization: multi-threaded and single-threaded.
- Multi-threaded mode pops up a window using show(), and the caller can then continue to interact
  with the vis module.
  IMPORTANT: multi-threaded mode is only supported on some systems (Linux, Windows using Qt).
  Due to weird OpenGL and Qt behavior in multi-threaded programs, if you are using multithreaded mode,
  you should only interact with OpenGL and the visualization using the methods in this module.
- Single-threaded mode blocks the calling thread using loop().  To interact with the scene, the caller
  will provide callbacks that can modify the visualization world, pop up windows etc.
  Single-threaded mode is the most compatible, and is the only mode that works with GLUT and Mac OS.
There are also some convenience functions that will work in both modes, such as run(), spin(), and
dialog().

The biggest drawback of single-threaded operation is that you can only start blocking dialogs at the outer-
most level, not inside loop().  So if you have a GUI that's running in real-time, in a multi-threaded
visualization your code can pop up a dialog (like an editor) and the continue running with the returned
value.  There are some workarounds in single-thread mode (providing a callback to the dialog function)
but these are not nearly as convenient. 

It is possible to start in single-threaded mode and convert to multi-threaded, but the converse is not
possible.

To set up and modify the visualization scene, there are three primary ways to do so:
  - Default scene manager.  You can use the scene manager to add items to the visualization world and
    then customize them using the vis.X routines in this module (like
    add, setColor, animate, etc).  See Python/demos/vistemplate.py for more information.  
    These just mirror the methods in VisualizationPlugin, which is how the default scene manager is
    implemented.
  - Custom GLPluginInterface.  The user creates a subclass of GLPluginInterface and performs
    all the necessary OpenGL drawing / interaction inside its hooks.  In this case, you will call
    vis.setPlugin(plugin) to override the default visualization behavior before creating your
    window. See Python/demos/visplugin.py for more information.
  - Hybrid visualization.  A GLPluginInterface subclass is created to add functionality on top of
    default the visualization world.  To augment the default scene manager, call vis.pushPlugin(plugin).
    Another option for hybrid visualization is to  subclass the vis.VisualizationPlugin class, and
    selectively augment / override the default functionality.

Instructions:

  - To add things to the default visualization:
    Call the VisualizationPlugin aliases (add, animate, setColor, etc)

  - To show the visualization and quit when the user closes the window:
    vis.run()

  - To show the visualization and return when the user closes the window:
    vis.dialog()
    ... do stuff afterwards ... 
    vis.kill()

  - To show the visualization and run a script alongside it until the user
    closes the window (multithreaded mode):
    vis.show()
    while vis.shown():
        vis.lock()
        ... do stuff ...
        [to exit the loop call vis.show(False)]
        vis.unlock()
        time.sleep(dt)
    ... do stuff afterwards ...
    vis.kill()

  - To show the visualization and run python commands until the user closes
    the window (single-threaded mode)
    def callback():
        ... do stuff ...
        [to exit the loop manually call vis.show(False)]
    vis.loop(setup=vis.show,callback=callback)
    vis.kill()

  - To run a window with a custom plugin (GLPluginInterface) and terminate on
    closure: 
    vis.run(plugin)
  
  - To show a dialog or parallel window
    vis.setPlugin(plugin)
    ... then call  
    vis.dialog()
    ... or
    vis.show()
    ... do stuff afterwards ... 
    vis.kill()

  - To add a GLPluginInterface that just customizes a few things on top of
    the default visualization:
    vis.pushPlugin(plugin)
    vis.dialog()
    vis.popPlugin()

  - To run plugins side-by-side in the same window:
    vis.setPlugin(plugin1)
    vis.addPlugin(plugin2)  #this creates a new split-screen
    vis.dialog()
    ... or
    vis.show()
    ... do stuff afterwards ... 
    vis.kill()

  - To run a custom Qt window or dialog containing a visualization window
    vis.setPlugin([desired plugin or None for visualization])
    def makeMyUI(qtglwidget):
        return MyQtMainWindow(qtglwidget)
    vis.setCustomUI(makeMyUI)
    vis.dialog()
    ... or 
    vis.show()
    ... do stuff afterwards ... 
    vis.kill()

  - To launch a second window after the first is closed: just call whatever you
    want again. Note: if show was previously called with a plugin and you wish to
    revert to the default visualization, you should call setPlugin(None) first to 
    restore the default.

  - To create a separate window with a given plugin:
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

Note: in multithreaded mode, when changing the data shown by the window (e.g., modifying
the configurations of robots in a WorldModel) you must call vis.lock() before
accessing the data and then call vis.unlock() afterwards.

The main interface is as follows:

def createWindow(title): creates a new visualization window and returns an
    integer identifier.
def setWindow(id): sets the active window for all subsequent calls.  ID 0 is
    the default visualization window.
def getWindow(): gets the active window ID.
def setWindowTitle(title): sets the title of the visualization window.
def getWindowTitle(): returns the title of the visualization window
def setPlugin(plugin=None): sets the current plugin (a GLPluginInterface instance). 
    This plugin will now capture input from the visualization and can override
    any of the default behavior of the visualizer. Set plugin=None if you want to return
    to the default visualization.
def addPlugin(plugin): adds a second OpenGL viewport governed by the given plugin (a
    GLPluginInterface instance).    
def run([plugin]): pops up a dialog and then kills the program afterwards.
def kill(): kills all previously launched visualizations and terminates the visualization thread.
    Afterwards, you may not be able to start new windows. Call this to cleanly quit.
def multithreaded(): returns true if multithreading is available.
def loop(setup=None,callback=None,cleanup=None): Runs the visualization thread inline with the main thread.
    The setup() function is called at the start, the callback() function is run every time the event thread
    is idle, and the cleanup() function is called on termination.

    NOTE FOR MAC USERS: having the GUI in a separate thread is not supported on Mac, so the loop
    function must be used rather than show/spin.

    NOTE FOR GLUT USERS: this may only be run once.
def dialog(): pops up a dialog box (does not return to calling thread until closed).
def dialogInLoop(callback): for use inside loop(). Pops up a dialog box window, and returns
    immediately to calling thread.
    The callback function has signature callback(okPressed), where okPressed is True if the user
    pressed the OK button, and False if Cancel or the close box was clicked.
def show(hidden=False): shows/hides a visualization window.  If not called from the visualization loop,
    a new visualization thread is run in parallel with the calling script. 
def spin(duration): shows the visualization window for the desired amount
    of time before returning, or until the user closes the window.
def shown(): returns true if the window is shown.
def lock(): locks the visualization world for editing.  The visualization will
    be paused until unlock() is called.
def unlock(): unlocks the visualization world.  Must only be called once
    after every lock().
def customUI(make_func): launches a user-defined UI window by calling make_func(gl_backend)
    in the visualization thread.  This can be used to build custom editors and windows that
    are compatible with other visualization functionality.  Here gl_backend is an instance of
    _GLBackend instantiated for the current plugin, and make_func returns a QDialog for dialog()
    constructed windows, or QMainWindow (or similar Qt object) for show() constructed windows.
def getViewport(): Returns the currently active viewport.

The following VisualizationPlugin methods are also added to the klampt.vis namespace
and operate on the default plugin.  If you are calling these methods from an external
loop (as opposed to inside a plugin) be sure to lock/unlock the visualization before/after
calling these methods.

def add(name,item,keepAppearance=False): adds an item to the visualization.
    name is a unique identifier.  If an item with the same name already exists,
    it will no longer be shown.  If keepAppearance=True, then the prior item's
    appearance will be kept, if a prior item exists.
def clear(): clears the visualization world.
def listItems(): prints out all names of visualization objects
def listItems(name): prints out all names of visualization objects under the given name
def dirty(item_name='all'): marks the given item as dirty and recreates the
    OpenGL display lists.  You may need to call this if you modify an item's geometry,
    for example.
def remove(name): removes an item from the visualization.
def setItemConfig(name,vector): sets the configuration of a named item.
def getItemConfig(name): returns the configuration of a named item.
def hide(name,hidden=True): hides/unhides an item.  The item is not removed,
    it just becomes invisible.
def edit(name,doedit=True): turns on/off visual editing of some item.  Only points,
    transforms, coordinate.Point's, coordinate.Transform's, coordinate.Frame's,
    robots, and objects are currently accepted.
def hideLabel(name,hidden=True): hides/unhides an item's text label.
def setAppearance(name,appearance): changes the Appearance of an item.
def revertAppearance(name): restores the Appearance of an item
def setAttribute(name,attribute,value): sets an attribute of the appearance
    of an item.  Accepted attributes are:
    - 'color': the item's color (r,g,b) or (r,g,b,a)
    - 'size': a plot or text's size
    - 'length': the length of axes in RigidTransform
    - 'width': the width of axes and trajectory curves
    - 'duration': the duration of a plot
    - 'endeffectors': for a robot Trajectory, the list of end effectors to plot (default the last link).
    - 'maxConfigs': for a Configs resource, the maximum number of drawn configurations (default 10)
    - 'fancy': for RigidTransform objects, whether the axes are drawn with boxes or lines (default False)
    - 'type': for ambiguous items, like a 3-item list when the robot has 3 links, specifies the type to be
       used.  For example, 'Config' draws the item as a robot configuration, while 'Vector3' or 'Point'
       draws it as a point.
def setColor(name,r,g,b,a=1.0): changes the color of an item.
def setDrawFunc(name,func): sets a custom OpenGL drawing function for an item.
    func is a one-argument function that takes the item data as input.  Set
    func to None to revert to default drawing.
def animate(name,animation,speed=1.0,endBehavior='loop'): Sends an animation to the
    object. May be a Trajectory or a list of configurations.  Works with points,
    so3 elements, se3 elements, rigid objects, or robots. 
    - speed: a modulator on the animation speed.  If the animation is a list of
      milestones, it is by default run at 1 milestone per second.
    - endBehavior: either 'loop' (animation repeats forever) or 'halt' (plays once).
def pauseAnimation(paused=True): Turns on/off animation.
def stepAnimation(amount): Moves forward the animation time by the given amount
    in seconds
def animationTime(newtime=None): Gets/sets the current animation time
    If newtime == None (default), this gets the animation time.
    If newtime != None, this sets a new animation time.
def addText(name,text,position=None): adds text.  You need to give an
    identifier to all pieces of text, which will be used to access the text as any other
    vis object.  If position is None, this is added as an on-screen display.  If position
    is of length 2, it is the (x,y) position of the upper left corner of the text on the
    screen.  Negative units anchor the text to the right or bottom of the window. 
    If position is of length 3, the text is drawn in the world coordinates.  You can
    then set the color, 'size' attribute, and 'position' attribute of the text using the
    identifier given in 'name'.
def clearText(): clears all previously added text.
def addPlot(name): creates a new empty plot.
def addPlotItem(name,itemname): adds a visualization item to a plot.
def logPlot(name,itemname,value): logs a custom visualization item to a plot
def logPlotEvent(name,eventname,color=None): logs an event on the plot.
def hidePlotItem(name,itemname,hidden=True): hides an item in the plot.  To hide a
    particular channel of a given item pass a pair (itemname,channelindex).  For example,
    to hide configurations 0-5 of 'robot', call hidePlotItem('plot',('robot',0)), ...,
    hidePlotItem('plot',('robot',5)).
def setPlotDuration(name,time): sets the plot duration.
def setPlotRange(name,vmin,vmax): sets the y range of a plot.
def setPlotPosition(name,x,y): sets the upper left position of the plot on the screen.
def setPlotSize(name,w,h): sets the width and height of the plot.
def savePlot(name,fn): saves a plot to a CSV (extension .csv) or Trajectory (extension
    .traj) file.
def autoFitCamera(scale=1.0): Automatically fits the camera to all objects in the
    visualization.  A scale > 1 magnifies the camera zoom.

Utility function:
def autoFitViewport(viewport,objects): Automatically fits the viewport's camera to 
    see all the given objects.

NAMING CONVENTION:

The world, if one exists, should be given the name 'world'.  Configurations and paths are drawn
with reference to the first robot in the world.

All items that refer to a name (except add) can either be given a top level item name
(a string) or a sub-item (a sequence of strings, given a path from the root to the leaf). 
For example, if you've added a RobotWorld under the name 'world' containing a robot called
'myRobot', then setColor(('world','myRobot'),0,1,0) will turn the robot green.  If 'link5'
is the robot's 5th link, then setColor(('world','myRobot','link5'),0,0,1) will turn the 5th
link blue.
"""


from OpenGL.GL import *
from threading import Thread,RLock
from ..robotsim import *
from ..math import vectorops,so3,se3
import gldraw
from glinit import *
from glinit import _GLBackend,_PyQtAvailable,_PyQt5Available,_PyQt4Available,_GLUTAvailable
from glinterface import GLPluginInterface
from glprogram import GLPluginProgram
import glcommon
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

class WindowInfo:
    """Mode can be hidden, shown, or dialog"""
    def __init__(self,name,frontend,vis,glwindow=None):
        self.name = name
        self.frontend = frontend
        self.vis = vis
        self.glwindow = glwindow
        self.mode = 'hidden'
        self.guidata = None
        self.custom_ui = None
        self.doRefresh = False
        self.doReload = False
        self.worlds = []
        self.active_worlds = []

_globalLock = RLock()
#the VisualizationPlugin instance of the currently active window
_vis = None
#the GLPluginProgram of the currently active window.  Accepts _vis as plugin or other user-defined plugins as well
_frontend = GLPluginProgram()
#the window title for the next created window
_window_title = "Klamp't visualizer"
#a list of WorldModel's in the current window.  A world cannot be used in multiple simultaneous
#windows in GLUT.  If a world is reused with a different window, its display lists will be refreshed.
#Note: must be proxies to allow for deletion
_current_worlds = []
#list of WindowInfo's
_windows = []
#the index of the current window
_current_window = None

def createWindow(title):
    """Creates a new window (and sets it active)."""
    global _globalLock,_frontend,_vis,_window_title,_current_worlds,_windows,_current_window
    _globalLock.acquire()
    if len(_windows) == 0:
        #save the defaults in window 0
        _windows.append(WindowInfo(_window_title,_frontend,_vis))    
        _windows[-1].worlds = _current_worlds
        _windows[-1].active_worlds = _current_worlds[:]
    #make a new window
    _window_title = title
    _frontend = GLPluginProgram()
    _vis = VisualizationPlugin()
    _frontend.setPlugin(_vis)
    _windows.append(WindowInfo(_window_title,_frontend,_vis))
    _current_worlds = []
    id = len(_windows)-1
    _current_window = id
    _globalLock.release()
    return id

def setWindow(id):
    """Sets currently active window."""
    global _globalLock,_frontend,_vis,_window_title,_windows,_current_window,_current_worlds
    if id == _current_window:
        return
    _globalLock.acquire()
    if len(_windows) == 0:
        #save the defaults in window 0
        _windows.append(WindowInfo(_window_title,_frontend,_vis)) 
        _windows[-1].worlds = _current_worlds
        _windows[-1].active_worlds = _current_worlds[:]
    assert id >= 0 and id < len(_windows),"Invalid window id"
    _window_title,_frontend,_vis,_current_worlds = _windows[id].name,_windows[id].frontend,_windows[id].vis,_windows[id].worlds
    #print "vis.setWindow(",id,") the window has status",_windows[id].mode
    if not _PyQtAvailable:
        #PyQt interface allows sharing display lists but GLUT does not.
        #refresh all worlds' display lists that were once active.
        for w in _current_worlds:
            if w in _windows[_current_window].active_worlds:
                print "klampt.vis.setWindow(): world",w().index,"becoming active in the new window",id
                _refreshDisplayLists(w())
                _windows[_current_window].active_worlds.remove(w)
    _windows[id].active_worlds = _current_worlds[:]
    _current_window = id
    _globalLock.release()

def getWindow():
    """Retrieves ID of currently active window or -1 if no window is active"""
    global _current_window
    if _current_window == None: return 0
    return _current_window

def setPlugin(plugin):
    """Lets the user capture input via a glinterface.GLPluginInterface class.
    Set plugin to None to disable plugins and return to the standard visualization"""
    global _globalLock,_frontend,_windows,_current_window
    _globalLock.acquire()
    if not isinstance(_frontend,GLPluginProgram):
        _frontend = GLPluginProgram()
        if _current_window != None:
            if _windows[_current_window].glwindow != None:
                _frontend.window = _windows[_current_window].glwindow
    if plugin == None:
        global _vis
        if _vis is None:
            raise RuntimeError("Visualization disabled")
        _frontend.setPlugin(_vis)
    else:
        _frontend.setPlugin(plugin)
    if hasattr(plugin,'world'):
        _checkWindowCurrent(plugin.world)
    _onFrontendChange()
    _globalLock.release()

def pushPlugin(plugin):
    """Adds a new glinterface.GLPluginInterface plugin on top of the old one."""
    global _globalLock,_frontend
    _globalLock.acquire()
    assert isinstance(_frontend,GLPluginProgram),"Can't push a plugin after addPlugin"
    if len(_frontend.plugins) == 0:
        global _vis
        if _vis is None:
            raise RuntimeError("Visualization disabled")
        _frontend.setPlugin(_vis)
    _frontend.pushPlugin(plugin)
    _onFrontendChange()
    _globalLock.release()

def popPlugin():
    """Reverses a prior pushPlugin() call"""
    global _frontend
    _globalLock.acquire()
    _frontend.popPlugin()
    _onFrontendChange()
    _globalLock.release()

def addPlugin(plugin):
    """Adds a second OpenGL viewport in the same window, governed by the given plugin (a
    glinterface.GLPluginInterface instance)."""
    global _frontend
    _globalLock.acquire()
    #create a multi-view widget
    if isinstance(_frontend,glcommon.GLMultiViewportProgram):
        _frontend.addView(plugin)
    else:
        if len(_frontend.plugins) == 0:
            setPlugin(None)
        multiProgram = glcommon.GLMultiViewportProgram()
        multiProgram.window = None
        if _current_window != None:
            if _windows[_current_window].glwindow != None:
                multiProgram.window = _windows[_current_window].glwindow
        multiProgram.addView(_frontend)
        multiProgram.addView(plugin)
        multiProgram.name = _window_title
        _frontend = multiProgram
    _onFrontendChange()
    _globalLock.release()


def run(plugin=None):
    """A blocking call to start a single window and then kill the visualization
    once the user closes the window.  If plugin == None, the default visualization is used. 
    Otherwise, plugin is a glinterface.GLPluginInterface object, and it is used
    to handle all rendering and user input.

    Works in both multi-threaded and single-threaded mode.
    """
    global _vis_thread_running
    setPlugin(plugin)
    if _vis_thread_running:
        #already multithreaded, can't go back to single thread
        show()
        while shown():
            time.sleep(0.1)
    else:
        #run in a single thread
        loop(setup=show)
    setPlugin(None)
    kill()

def multithreaded():
    global _use_multithreaded
    return _use_multithreaded

def dialog():
    """A blocking call to start a single dialog window with the current plugin.  It is
    closed by pressing OK or closing the window."""
    _dialog()

def setWindowTitle(title):
    global _window_title
    _window_title = title
    _onFrontendChange()

def getWindowTitle():
    global _window_title
    return _window_title

def kill():
    """This should be called at the end of the calling program to cleanly terminate the
    visualization thread"""
    global _vis,_globalLock
    if _vis is None:
        print "vis.kill() Visualization disabled"
        return
    _kill()

def loop(setup=None,callback=None,cleanup=None):
    """Runs the visualization thread inline with the main thread.
    The setup() function is called at the start, the callback() function is run every time the event thread
    is idle, and the cleanup() function is called on termination.

    NOTE FOR MAC USERS: a multithreaded GUI is not supported on Mac, so the loop()
    function must be used rather than "show and wait".

    NOTE FOR GLUT USERS: this may only be run once.
    """
    _loop(setup,callback,cleanup)


def show(display=True):
    """Shows or hides the current window.

    NOTE FOR MAC USERS: due to a lack of support of multithreading on Mac, this will not work outside
    of the setup / callback / cleanup functions given in a call to loop()."""
    _globalLock.acquire()
    if display:
        _show()
    else:
        _hide()
    _globalLock.release()

def spin(duration):
    """Spin-shows a window for a certain duration or until the window is closed."""
    global _use_multithreaded,_in_vis_loop
    if _in_vis_loop:
        raise RuntimeError("spin() cannot be used inside loop()")
    if _use_multithreaded:
        #use existing thread
        show()
        t = 0
        while t < duration:
            if not shown(): break
            time.sleep(min(0.04,duration-t))
            t += 0.04
        show(False)
    else:
        #use single thread
        t0 = time.time()
        def timed_break():
            t1 = time.time()
            if t1 - t0 >= duration:
                show(False)
        loop(callback=timed_break,setup=lambda:show())
    return

def lock():
    """Begins a locked section.  Needs to be called any time you modify a visualization item outside
    of the visualization thread.  unlock() must be called to let the visualization thread proceed."""
    global _globalLock
    _globalLock.acquire()

def unlock():
    """Ends a locked section acquired by lock()."""
    global _globalLock,_windows
    for w in _windows:
        if w.glwindow:
            w.doRefresh = True
    _globalLock.release()

def shown():
    """Returns true if a visualization window is currently shown."""
    global _globalLock,_vis_thread_running,_current_window
    _globalLock.acquire()
    res = (_vis_thread_running and _current_window != None and _windows[_current_window].mode in ['shown','dialog'] or _windows[_current_window].guidata is not None)
    _globalLock.release()
    return res

def customUI(func):
    """Tells the next created window/dialog to use a custom UI function.  func is a 1-argument function that 
    takes a QtWindow or GLUTWindow as its argument."""
    global _globalLock
    _globalLock.acquire()
    _set_custom_ui(func)
    _globalLock.release()

def getViewport():
    """Returns the GLViewport of the current window (see klampt.vis.glprogram.GLViewport)"""
    return _frontend.get_view()

def setViewport(viewport):
    """Sets the current window to use a given GLViewport (see klampt.vis.glprogram.GLViewport)"""
    _frontend.set_view(viewport)



######### CONVENIENCE ALIASES FOR VisualizationPlugin methods ###########
def clear():
    """Clears the visualization world."""
    global _vis
    if _vis is None:
        return
    _vis.clear()

def add(name,item,keepAppearance=False):
    """Adds an item to the visualization. name is a unique identifier.  If an item with
    the same name already exists, it will no longer be shown.  If keepAppearance=True, then
    the prior item's appearance will be kept, if a prior item exists."""
    global _vis
    if _vis is None:
        print "Visualization disabled"
        return
    _globalLock.acquire()
    _checkWindowCurrent(item)
    _globalLock.release()
    _vis.add(name,item,keepAppearance)

def listItems(name=None,indent=0):
    global _vis
    if _vis is None:
        print "Visualization disabled"
        return
    _vis.listItems(name,indent)

def dirty(item_name='all'):
    """Marks the given item as dirty and recreates the OpenGL display lists.  You may need
    to call this if you modify an item's geometry, for example.  If things start disappearing
    from your world when you create a new window, you may need to call this too."""
    global _vis
    if _vis is None:
        print "Visualization disabled"
        return
    _vis.dirty(item_name)

def animate(name,animation,speed=1.0,endBehavior='loop'):
    """Sends an animation to the named object.
    Works with points, so3 elements, se3 elements, rigid objects, or robots, and may work
    with other objects as well.

    Parameters:
    - animation: may be a Trajectory or a list of configurations.
    - speed: a modulator on the animation speed.  If the animation is a list of
      milestones, it is by default run at 1 milestone per second.
    - endBehavior: either 'loop' (animation repeats forever) or 'halt' (plays once).
    """
    global _vis
    if _vis is None:
        print "Visualization disabled"
        return
    _vis.animate(name,animation,speed,endBehavior)

def pauseAnimation(paused=True):
    global _vis
    if _vis is None:
        print "Visualization disabled"
        return
    _vis.pauseAnimation(paused)

def stepAnimation(amount):
    global _vis
    if _vis is None:
        print "Visualization disabled"
        return
    _vis.stepAnimation(amount)

def animationTime(newtime=None):
    """Gets/sets the current animation time
    If newtime == None (default), this gets the animation time.
    If newtime != None, this sets a new animation time.
    """
    global _vis
    if _vis is None:
        print "Visualization disabled"
        return 0
    return _vis.animationTime(newtime)

def remove(name):
    global _vis
    if _vis is None:
        return
    return _vis.remove(name)

def getItemConfig(name):
    global _vis
    if _vis is None:
        return None
    return _vis.getItemConfig(name)

def setItemConfig(name,value):
    global _vis
    if _vis is None:
        return
    return _vis.setItemConfig(name,value)

def hideLabel(name,hidden=True):
    global _vis
    if _vis is None:
        return
    return _vis.hideLabel(name,hidden)

def hide(name,hidden=True):
    global _vis
    if _vis is None:
        return
    _vis.hide(name,hidden)

def edit(name,doedit=True):
    """Turns on/off visual editing of some item.  Only points, transforms,
    coordinate.Point's, coordinate.Transform's, coordinate.Frame's, robots,
    and objects are currently accepted."""
    global _vis
    if _vis is None:
        return
    _vis.edit(name,doedit)

def setAppearance(name,appearance):
    global _vis
    if _vis is None:
        return
    _vis.setAppearance(name,appearance)

def setAttribute(name,attr,value):
    global _vis
    if _vis is None:
        return
    _vis.setAttribute(name,attr,value)

def revertAppearance(name):
    global _vis
    if _vis is None:
        return
    _vis.revertAppearance(name)

def setColor(name,r,g,b,a=1.0):
    global _vis
    if _vis is None:
        return
    _vis.setColor(name,r,g,b,a)

def setDrawFunc(name,func):
    global _vis
    if _vis is None:
        return
    _vis.setDrawFunc(name,func)

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
    elif isinstance(object,RigidObjectModel):
        return [object.getTransform()[1]]
    elif isinstance(object,Geometry3D):
        return object.getCurrentTransform()[1]
    elif isinstance(object,VisAppearance):
        res = _getOffsets(object.item)
        if len(res) != 0: return res
        if len(object.subAppearances) == 0:
            bb = object.getBounds()
            if bb != None and not aabb_empty(bb):
                return [vectorops.mul(vectorops.add(bb[0],bb[1]),0.5)]
        else:
            res = []
            for a in object.subAppearances.itervalues():
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
            if bb != None and not aabb_empty(bb):
                res += list(bb)
        return res
    elif isinstance(object,RigidObjectModel):
        return list(object.geometry().getBB())
    elif isinstance(object,Geometry3D):
        return list(object.getBB())
    elif isinstance(object,VisAppearance):
        if len(object.subAppearances) == 0:
            if isinstance(object.item,TerrainModel):
                return []
            bb = object.getBounds()
            if bb != None and not aabb_empty(bb):
                return list(bb)
        else:
            res = []
            for a in object.subAppearances.itervalues():
                res += _getBounds(a)
            return res
    return []

def _fitPlane(pts):
    import numpy as np
    if len(pts) < 3:
        raise ValueError("Point set is degenerate")
    centroid = vectorops.div(vectorops.add(*pts),len(pts))
    A = np.array([vectorops.sub(pt,centroid) for pt in pts])
    U,S,V = np.linalg.svd(A,full_matrices=False)
    imin = 0
    smin = S[0]
    zeros = []
    for i in xrange(len(S)):
        if abs(S[i]) < 1e-6:
            zeros.append(i)
        if abs(S[i]) < smin:
            smin = S[i]
            imin = i
    if len(zeros) > 1:
        raise ValueError("Point set is degenerate")
    assert V.shape == (3,3)
    #normal is the corresponding row of U
    normal = V[imin,:]
    return centroid,normal.tolist()

def autoFitViewport(viewport,objects): 
    ofs = sum([_getOffsets(o) for o in objects],[])
    pts = sum([_getBounds(o) for o in objects],[])
    #print "Bounding box",bb,"center",center
    #raw_input()
    #reset
    viewport.camera.rot = [0.,0.,0.]
    viewport.camera.tgt = [0.,0.,0.]
    viewport.camera.dist = 6.0
    viewport.clippingplanes = (0.2,20)
    if len(ofs) == 0:
        return

    bb = aabb_create(*pts)
    center = vectorops.mul(vectorops.add(bb[0],bb[1]),0.5)
    viewport.camera.tgt = center
    radius = max(vectorops.distance(bb[0],center),0.1)
    viewport.camera.dist = 1.2*radius / math.tan(math.radians(viewport.fov*0.5))
    #default: oblique view
    viewport.camera.rot = [0,math.radians(30),math.radians(45)]
    #fit a plane to these points
    try:
        centroid,normal = _fitPlane(ofs)
    except Exception as e:
        try:
            centroid,normal = _fitPlane(pts)
        except Exception as e:
            print "Exception occurred during fitting to points"
            print ofs
            print pts
            raise
            return
    if normal[2] > 0:
        normal = vectorops.mul(normal,-1)
    z,x,y = so3.matrix(so3.inv(so3.canonical(normal)))
    #print z,x,y
    #raw_input()
    radius = max([abs(vectorops.dot(x,vectorops.sub(center,pt))) for pt in pts] + [abs(vectorops.dot(y,vectorops.sub(center,pt)))*viewport.w/viewport.h for pt in pts])
    zmin = min([vectorops.dot(z,vectorops.sub(center,pt)) for pt in pts])
    zmax = max([vectorops.dot(z,vectorops.sub(center,pt)) for pt in pts])
    #print "Viewing direction",normal,"at point",center,"with scene size",radius
    #orient camera to point along normal direction
    viewport.camera.tgt = center
    viewport.camera.dist = 1.2*radius / math.tan(math.radians(viewport.fov*0.5))
    near,far = viewport.clippingplanes
    if viewport.camera.dist + zmin < near:
        near = max((viewport.camera.dist + zmin)*0.5, radius*0.1)
    if viewport.camera.dist + zmax > far:
        far = max((viewport.camera.dist + zmax)*1.5, radius*3)
    viewport.clippingplanes = (near,far)
    roll = 0
    yaw = math.atan2(normal[0],normal[1])
    pitch = math.atan2(-normal[2],vectorops.norm(normal[0:2]))
    #print "Roll pitch and yaw",roll,pitch,yaw
    #print "Distance",viewport.camera.dist
    viewport.camera.rot = [roll,pitch,yaw]

def addText(name,text,pos=None):
    """Adds text to the visualizer.  You must give an identifier to all pieces of
    text, which will be used to access the text as any other vis object. 

    Parameters:
    - name: the text's unique identifier.
    - text: the string to be drawn
    - pos: the position of the string. If pos=None, this is added to the on-screen "console" display. 
      If pos has length 2, it is the (x,y) position of the upper left corner of the text on the
      screen.  Negative units anchor the text to the right or bottom of the window. 
      If pos has length 3, the text is drawn in the world coordinates. 

    To customize the text appearance, you can set the color, 'size' attribute, and 'position'
    attribute of the text using the identifier given in 'name'.
    """
    global _vis
    if _vis is None:
        return
    _vis.addText(name,text,pos)

def clearText():
    """Clears all text in the visualization."""
    global _vis
    if _vis is None:
        return
    _vis.clearText()

def addPlot(name):
    add(name,VisPlot())

def addPlotItem(name,itemname):
    global _vis
    if _vis is None:
        return
    _vis.addPlotItem(name,itemname)

def logPlot(name,itemname,value):
    """Logs a custom visualization item to a plot"""
    global _vis
    if _vis is None:
        return
    _vis.logPlot(name,itemname,value)

def logPlotEvent(name,eventname,color=None):
    """Logs an event on the plot."""
    global _vis
    if _vis is None:
        return
    _vis.logPlotEvent(name,eventname,color)

def hidePlotItem(name,itemname,hidden=True):
    global _vis
    if _vis is None:
        return
    _vis.hidePlotItem(name,itemname,hidden)

def setPlotDuration(name,time):
    setAttribute(name,'duration',time)

def setPlotRange(name,vmin,vmax): 
    setAttribute(name,'range',(vmin,vmax))

def setPlotPosition(name,x,y):
    setAttribute(name,'position',(x,y))

def setPlotSize(name,w,h):
    setAttribute(name,'size',(w,h))

def savePlot(name,fn):
    global _vis
    if _vis is None:
        return
    _vis.savePlot(name,fn)

def autoFitCamera(scale=1):
    global _vis
    if _vis is None:
        return
    print "klampt.vis: auto-fitting camera to scene."
    _vis.autoFitCamera(scale)




def objectToVisType(item,world):
    itypes = types.objectToTypes(item,world)
    if isinstance(itypes,(list,tuple)):
        #ambiguous, still need to figure out what to draw
        validtypes = []
        for t in itypes:
            if t == 'Config':
                if world != None and len(item) == world.robot(0).numLinks():
                    validtypes.append(t)
            elif t=='Vector3':
                validtypes.append(t)
            elif t=='RigidTransform':
                validtypes.append(t)
        if len(validtypes) > 1:
            print "Unable to draw item of ambiguous types",validtypes
            print "  (Try vis.setAttribute(item,'type',desired_type_str) to disambiguate)"
            return
        if len(validtypes) == 0:
            print "Unable to draw any of types",itypes
            return
        return validtypes[0]
    return itypes

def aabb_create(*ptlist):
    if len(ptlist) == 0:
        return [float('inf')]*3,[float('-inf')]*3
    else:
        bmin,bmax = list(ptlist[0]),list(ptlist[0])
        for i in xrange(1,len(ptlist)):
            x = ptlist[i]
            bmin = [min(a,b) for (a,b) in zip(bmin,x)]
            bmax = [max(a,b) for (a,b) in zip(bmax,x)]
        return bmin,bmax

def aabb_expand(bb,bb2):
    bmin = [min(a,b) for a,b in zip(bb[0],bb2[0])]
    bmax = [max(a,b) for a,b in zip(bb[1],bb2[1])]
    return (bmin,bmax)

def aabb_empty(bb):
    return any((a > b) for (a,b) in zip(bb[0],bb[1]))

_defaultCompressThreshold = 1e-2

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
        for e,times in self.events.iteritems():
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
            if color == None:
                import random
                color = (random.uniform(0.01,1),random.uniform(0.01,1),random.uniform(0.01,1))
                color = vectorops.mul(color,1.0/max(color))
        if color != None:
            self.eventColors[name] = color
            if len(color)==3:
                self.eventColors[name] += [1.0]

    def autoRange(self):
        vmin = float('inf')
        vmax = -float('inf')
        for i in self.items:
            for j in xrange(len(i.traceRanges)):
                if not i.hidden[j]:
                    vmin = min(vmin,i.traceRanges[j][0])
                    vmax = max(vmax,i.traceRanges[j][1])
        if math.isinf(vmin):
            return (0.,1.)
        if vmax == vmin:
            vmax += 1.0
        return (float(vmin),float(vmax))

    def render(self,window,x,y,w,h,duration,vmin=None,vmax=None):
        if vmin == None:
            vmin,vmax = self.autoRange()
        import random
        while len(self.colors) < len(self.items):
            c = (random.uniform(0.01,1),random.uniform(0.01,1),random.uniform(0.01,1))
            c = vectorops.mul(c,1.0/max(c))
            self.colors.append(c)
        glColor3f(0,0,0)
        glBegin(GL_LINE_LOOP)
        glVertex2f(x,y)
        glVertex2f(x+w,y)
        glVertex2f(x+w,y+h)
        glVertex2f(x,y+h)
        glEnd()
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
                glColor3fv(vectorops.mul(self.colors[i],item.luminosity[j]))
                window.draw_text((x+w+3,labelheight+4),label,9)
                glBegin(GL_LINE_STRIP)
                for k in xrange(len(trace)-1):
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
                        glVertex2f(x+w*u,y+(1-v)*h)
                u,v = trace[-1]
                u = (u-(tmax-duration))/duration
                v = (v-vmin)/(vmax-vmin)
                glVertex2f(x+w*u,y+(1-v)*h)
                glEnd()

        if len(self.events) > 0:
            for e,times in self.events.iteritems():
                for t in times:
                    if t < tmax-duration: continue
                    labelx = (t - (tmax-duration))/duration
                    labelx = x + w*labelx
                    c = self.eventColors[e]
                    glColor4f(c[0]*0.5,c[1]*0.5,c[2]*0.5,c[3])
                    window.draw_text((labelx,y+h+12),e,9)
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
            glBegin(GL_LINES)
            for e,times in self.events.iteritems():
                for t in times:
                    if t < tmax-duration: continue
                    labelx = (t - (tmax-duration))/duration
                    labelx = x + w*labelx
                    
                    glColor4f(c[0],c[1],c[2],c[3]*0.5)
                    glVertex2f(labelx,y)
                    glVertex2f(labelx,y+h)
            glEnd()
            glDisable(GL_BLEND)

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
                times,vals = zip(*trace)
                if isinstance(vals[0],(int,float)):
                    vals = [[v] for v in vals]
                traj = Trajectory(times,vals)
                cols.append(traj)
                mint = min(mint,traj.times[0])
                maxt = max(maxt,traj.times[-1])
                for k in xrange(len(traj.times)-1):
                    mindt = min(mindt,traj.times[k+1] - traj.times[k])
        assert mindt > 0, "For some reason, there is a duplicate time?"
        N = int((maxt - mint)/mindt)
        dt = (maxt - mint)/N
        times = [mint + i*(maxt-mint)/N for i in range(N+1)]
        for i in xrange(N+1):
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

def drawTrajectory(traj,width,color):
    """Draws a trajectory of points or transforms"""
    if isinstance(traj,list):
        #R3 trajectory
        glDisable(GL_LIGHTING)
        glColor4f(*color)
        if len(traj) == 1:
            glPointSize(width)
            glBegin(GL_POINTS)
            glVertex3f(*traj[0])
            glEnd()
        if len(traj) >= 2:
            glLineWidth(width)
            glBegin(GL_LINE_STRIP)
            for p in traj:
                glVertex3f(*p)
            glEnd()
            glLineWidth(1.0)
    elif isinstance(traj,SE3Trajectory):
        pointTraj = []
        for m in traj.milestones:
            pointTraj.append(m[9:])
        drawTrajectory(pointTraj,width,color)
    else:
        if len(traj.milestones[0]) == 3:
            drawTrajectory(traj.milestones,width,color)
        elif len(traj.milestones[0]) == 2:
            #R2 trajectory
            drawTrajectory([v + [0.0] for v in traj.milestones],width,color)


def drawRobotTrajectory(traj,robot,ees,width=2,color=[1,0.5,0,1]):
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
        drawTrajectory(ptraj,width,color)

class VisAppearance:
    def __init__(self,item,name = None):
        self.name = name
        self.hidden = False
        self.useDefaultAppearance = True
        self.customAppearance = None
        self.customDrawFunc = None
        #For group items, this allows you to customize appearance of sub-items
        self.subAppearances = {}
        self.animation = None
        self.animationStartTime = 0
        self.animationSpeed = 1.0
        self.attributes = {}
        #used for Qt text rendering
        self.widget = None
        #used for visual editing of certain items
        self.editor = None
        #cached drawing
        self.displayCache = [glcommon.CachedGLObject()]
        self.displayCache[0].name = name
        #temporary configuration of the item
        self.drawConfig = None
        self.setItem(item)
    def setItem(self,item):
        self.item = item
        self.subAppearances = {}
        #Parse out sub-items which can have their own appearance changed
        if isinstance(item,WorldModel):
            for i in xrange(item.numRobots()):
                self.subAppearances[("Robot",i)] = VisAppearance(item.robot(i),item.robot(i).getName())
            for i in xrange(item.numRigidObjects()):
                self.subAppearances[("RigidObject",i)] = VisAppearance(item.rigidObject(i),item.rigidObject(i).getName())
            for i in xrange(item.numTerrains()):
                self.subAppearances[("Terrain",i)] = VisAppearance(item.terrain(i),item.terrain(i).getName())
        elif isinstance(item,RobotModel):
            for i in xrange(item.numLinks()):
                self.subAppearances[("Link",i)] = VisAppearance(item.link(i),item.link(i).getName())
        elif isinstance(item,coordinates.Group):
            for n,f in item.frames.iteritems():
                self.subAppearances[("Frame",n)] = VisAppearance(f,n)
            for n,p in item.points.iteritems():
                self.subAppearances[("Point",n)] = VisAppearance(p,n)
            for n,d in item.directions.iteritems():
                self.subAppearances[("Direction",n)] = VisAppearance(d,n)
            for n,g in item.subgroups.iteritems():
                self.subAppearances[("Subgroup",n)] = VisAppearance(g,n)
        elif isinstance(item,Hold):
            if item.ikConstraint is not None:
                self.subAppearances["ikConstraint"] = VisAppearance(item.ikConstraint,"ik")
            for n,c in enumerate(item.contacts):
                self.subAppearances[("contact",n)] = VisAppearance(c,n)
        for (k,a) in self.subAppearances.iteritems():
            a.attributes = self.attributes
            
    def markChanged(self):
        for c in self.displayCache:
            c.markChanged()
        for (k,a) in self.subAppearances.iteritems():
            a.markChanged()
        self.update_editor(True)
        self.doRefresh = True

    def destroy(self):
        for c in self.displayCache:
            c.destroy()
        for (k,a) in self.subAppearances.iteritems():
            a.destroy()
        self.subAppearances = {}
        
    def drawText(self,text,point):
        """Draws the given text at the given point"""
        if self.attributes.get("text_hidden",False): return
        self.widget.addLabel(text,point[:],[0,0,0])

    def updateAnimation(self,t):
        """Updates the configuration, if it's being animated"""
        if not self.animation:
            self.drawConfig = None
        else:
            u = self.animationSpeed*(t-self.animationStartTime)
            q = self.animation.eval(u,self.animationEndBehavior)
            self.drawConfig = q
        for n,app in self.subAppearances.iteritems():
            app.updateAnimation(t)

    def updateTime(self,t):
        """Updates in real time"""
        if isinstance(self.item,VisPlot):
            compressThreshold = self.attributes.get('compress',_defaultCompressThreshold)
            duration = self.attributes.get('duration',5.)
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
                #self.item = 
                config.setConfig(self.item,self.drawConfig)
                self.drawConfig = newDrawConfig
            except Exception as e:
                print "Warning, exception thrown during animation update.  Probably have incorrect length of configuration"
                import traceback
                traceback.print_exc()
                pass
        for n,app in self.subAppearances.iteritems():
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
        for n,o in self.subAppearances.iteritems():
            o.clearDisplayLists()
        self.markChanged()

    def draw(self,world=None):
        """Draws the specified item in the specified world.  If name
        is given and text_hidden != False, then the name of the item is
        shown."""
        if self.hidden: return
        if self.customDrawFunc is not None:
          self.customDrawFunc(self.item)
          return
       
        item = self.item
        name = self.name
        #set appearance
        if not self.useDefaultAppearance and hasattr(item,'appearance'):
            #print "Has custom appearance"
            if not hasattr(self,'oldAppearance'):
                self.oldAppearance = item.appearance().clone()
            if self.customAppearance != None:
                #print "Changing appearance of",name
                item.appearance().set(self.customAppearance)
            elif "color" in self.attributes:
                #print "Changing color of",name
                item.appearance().setColor(*self.attributes["color"])

        if len(self.subAppearances)!=0:
            for n,app in self.subAppearances.iteritems():
                app.widget = self.widget
                app.draw(world)
        elif hasattr(item,'drawGL'):
            item.drawGL()
        elif hasattr(item,'drawWorldGL'):
            item.drawWorldGL()
        elif isinstance(item,str):
            pos = self.attributes.get("position",None)
            if pos is not None and len(pos)==3:
                col = self.attributes.get("color",(0,0,0))
                self.widget.addLabel(self.item,pos,col)
        elif isinstance(item,VisPlot):
            pass
        elif isinstance(item,Trajectory):
            doDraw = False
            centroid = None
            robot = (world.robot(0) if world is not None and world.numRobots() > 0 else None)
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
            elif isinstance(item,SE3Trajectory):
                doDraw = True
                centroid = item.milestones[0][9:]
            else:
                if len(item.milestones[0]) == 3:
                    #R3 trajectory
                    doDraw = True
                    centroid = item.milestones[0]
                elif len(item.milestones[0]) == 2:
                    #R2 trajectory
                    doDraw = True
                    centroid = item.milestones[0]+[0.0]
                else:
                    #don't know how to interpret this trajectory
                    pass
            if doDraw:
                def drawRaw():
                    pointTrajectories = []
                    width = self.attributes.get("width",3)
                    color = self.attributes.get("color",[1,0.5,0,1])
                    if isinstance(item,RobotTrajectory) or treatAsRobotTrajectory:
                        ees = self.attributes.get("endeffectors",[-1])
                        drawRobotTrajectory(item,robot,ees,width,color)                        
                    else:
                        drawTrajectory(item,width,color)
                self.displayCache[0].draw(drawRaw,se3.identity())
                if name != None:
                    self.drawText(name,centroid)
        elif isinstance(item,MultiPath):
            robot = (world.robot(0) if world is not None and world.numRobots() > 0 else None)
            if robot is not None and item.numSections() > 0:
                if len(item.sections[0].configs[0]) == robot.numLinks():
                    ees = self.attributes.get("endeffectors",[-1])
                    if len(ees) > 0:
                        for i,ee in enumerate(ees):
                            if ee < 0: ees[i] = robot.numLinks()-1
                        robot.setConfig(item.sections[0].configs[0])
                        centroid = vectorops.div(vectorops.add(*[robot.link(ee).getTransform()[1] for ee in ees]),len(ees))
                    width = self.attributes.get("width",3)
                    color = self.attributes.get("color",[1,0.5,0,1])
                    color2 = [1-c for c in color]
                    color2[3] = color[3]
                    def drawRaw():
                        for i,s in enumerate(item.sections):
                            drawRobotTrajectory(s.configs,robot,ees,width,(color if i%2 == 0 else color2))
                    #draw it!
                    self.displayCache[0].draw(drawRaw,se3.identity())
                    if name != None:
                        self.drawText(name,centroid)
        elif isinstance(item,coordinates.Point):
            def drawRaw():
                glDisable(GL_LIGHTING)
                glEnable(GL_POINT_SMOOTH)
                glPointSize(self.attributes.get("size",5.0))
                glColor4f(*self.attributes.get("color",[0,0,0,1]))
                glBegin(GL_POINTS)
                glVertex3f(0,0,0)
                glEnd()
                #write name
            glDisable(GL_DEPTH_TEST)
            self.displayCache[0].draw(drawRaw,[so3.identity(),item.worldCoordinates()])
            glEnable(GL_DEPTH_TEST)
            if name != None:
                self.drawText(name,item.worldCoordinates())
        elif isinstance(item,coordinates.Direction):
            def drawRaw():
                glDisable(GL_LIGHTING)
                glDisable(GL_DEPTH_TEST)
                L = self.attributes.get("length",0.15)
                source = [0,0,0]
                glColor4f(*self.attributes.get("color",[0,1,1,1]))
                glBegin(GL_LINES)
                glVertex3f(*source)
                glVertex3f(*vectorops.mul(item.localCoordinates(),L))
                glEnd()
                glEnable(GL_DEPTH_TEST)
                #write name
            self.displayCache[0].draw(drawRaw,item.frame().worldCoordinates(),parameters = item.localCoordinates())
            if name != None:
                self.drawText(name,vectorops.add(item.frame().worldCoordinates()[1],item.worldCoordinates()))
        elif isinstance(item,coordinates.Frame):
            t = item.worldCoordinates()
            if item.parent() != None:
                tp = item.parent().worldCoordinates()
            else:
                tp = se3.identity()
            tlocal = item.relativeCoordinates()
            def drawRaw():
                glDisable(GL_DEPTH_TEST)
                glDisable(GL_LIGHTING)
                glLineWidth(2.0)
                gldraw.xform_widget(tlocal,self.attributes.get("length",0.1),self.attributes.get("width",0.01))
                glLineWidth(1.0)
                #draw curve between frame and parent
                if item.parent() != None:
                    d = vectorops.norm(tlocal[1])
                    vlen = d*0.5
                    v1 = so3.apply(tlocal[0],[-vlen]*3)
                    v2 = [vlen]*3
                    #glEnable(GL_BLEND)
                    #glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
                    #glColor4f(1,1,0,0.5)
                    glColor3f(1,1,0)
                    gldraw.hermite_curve(tlocal[1],v1,[0,0,0],v2,0.03*max(0.1,vectorops.norm(tlocal[1])))
                    #glDisable(GL_BLEND)
                glEnable(GL_DEPTH_TEST)

            #For some reason, cached drawing is causing OpenGL problems
            #when the frame is rapidly changing
            self.displayCache[0].draw(drawRaw,transform=tp, parameters = tlocal)
            #glPushMatrix()
            #glMultMatrixf(sum(zip(*se3.homogeneous(tp)),()))
            #drawRaw()
            #glPopMatrix()
            #write name
            if name != None:
                self.drawText(name,t[1])
        elif isinstance(item,coordinates.Transform):
            #draw curve between frames
            t1 = item.source().worldCoordinates()
            if item.destination() != None:
                t2 = item.destination().worldCoordinates()
            else:
                t2 = se3.identity()
            d = vectorops.distance(t1[1],t2[1])
            vlen = d*0.5
            v1 = so3.apply(t1[0],[-vlen]*3)
            v2 = so3.apply(t2[0],[vlen]*3)
            def drawRaw():
                glDisable(GL_DEPTH_TEST)
                glDisable(GL_LIGHTING)
                glColor3f(1,1,1)
                gldraw.hermite_curve(t1[1],v1,t2[1],v2,0.03)
                glEnable(GL_DEPTH_TEST)
                #write name at curve
            self.displayCache[0].draw(drawRaw,transform=None,parameters = (t1,t2))
            if name != None:
                self.drawText(name,spline.hermite_eval(t1[1],v1,t2[1],v2,0.5))
        elif isinstance(item,coordinates.Group):
            pass
        elif isinstance(item,ContactPoint):
            def drawRaw():
                glDisable(GL_LIGHTING)
                glEnable(GL_POINT_SMOOTH)
                glPointSize(self.attributes.get("size",5.0))
                l = self.attributes.get("length",0.05)
                glColor4f(*self.attributes.get("color",[1,0.5,0,1]))
                glBegin(GL_POINTS)
                glVertex3f(0,0,0)
                glEnd()
                glBegin(GL_LINES)
                glVertex3f(0,0,0)
                glVertex3f(l,0,0)
                glEnd()
            self.displayCache[0].draw(drawRaw,[so3.canonical(item.n),item.x])
        elif isinstance(item,Hold):
            pass
        elif isinstance(item,IKObjective):
            if hasattr(item,'robot'):
                #need this to be built with a robot element.
                #Otherwise, can't determine the correct transforms
                robot = item.robot
            elif world:
                if world is not None and world.numRobots() >= 1:
                    robot = world.robot(0)
                else:
                    robot = None
            else:
                robot = None
            if robot != None:
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
                    if dest != None:
                        p2 = se3.apply(dest.getTransform(),wp)
                    else:
                        p2 = wp
                    d = vectorops.distance(p1,p2)
                    v1 = [0.0]*3
                    v2 = [0.0]*3
                    if item.numRotDims()==3: #full constraint
                        R = item.getRotation()
                        def drawRaw():
                            gldraw.xform_widget(se3.identity(),self.attributes.get("length",0.1),self.attributes.get("width",0.01))
                        t1 = se3.mul(link.getTransform(),(so3.identity(),lp))
                        t2 = (R,wp) if dest==None else se3.mul(dest.getTransform(),(R,wp))
                        self.displayCache[0].draw(drawRaw,transform=t1)
                        self.displayCache[1].draw(drawRaw,transform=t2)
                        vlen = d*0.1
                        v1 = so3.apply(t1[0],[-vlen]*3)
                        v2 = so3.apply(t2[0],[vlen]*3)
                    elif item.numRotDims()==0: #point constraint
                        def drawRaw():
                            glDisable(GL_LIGHTING)
                            glEnable(GL_POINT_SMOOTH)
                            glPointSize(self.attributes.get("size",5.0))
                            glColor4f(*self.attributes.get("color",[0,0,0,1]))
                            glBegin(GL_POINTS)
                            glVertex3f(0,0,0)
                            glEnd()
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
                            glDisable(GL_LIGHTING)
                            glEnable(GL_POINT_SMOOTH)
                            glPointSize(self.attributes.get("size",5.0))
                            glColor4f(*self.attributes.get("color",[0,0,0,1]))
                            glBegin(GL_POINTS)
                            glVertex3f(*p)
                            glEnd()
                            glColor4f(*self.attributes.get("color",[0.5,0,0.5,1]))
                            glLineWidth(self.attributes.get("width",3.0))
                            glBegin(GL_LINES)
                            glVertex3f(*p)
                            glVertex3f(*vectorops.madd(p,d,self.attributes.get("length",0.1)))
                            glEnd()
                            glLineWidth(1.0)
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
                        glDisable(GL_LIGHTING)
                        glDisable(GL_DEPTH_TEST)
                        glColor3f(1,0.5,0)
                        gldraw.hermite_curve(p1,v1,p2,v2,0.03*max(0.1,vectorops.distance(p1,p2)))
                        #glBegin(GL_LINES)
                        #glVertex3f(*p1)
                        #glVertex3f(*p2)
                        #glEnd()
                        glEnable(GL_DEPTH_TEST)
                    #TEMP for some reason the cached version sometimes gives a GL error
                    self.displayCache[2].draw(drawConnection,transform=None,parameters = (p1,v1,p2,v2))
                    #drawConnection()
                    if name != None:
                        self.drawText(name,wp)
                else:
                    wp = link.getTransform()[1]
                    if item.numRotDims()==3: #full constraint
                        R = item.getRotation()
                        def drawRaw():
                            gldraw.xform_widget(se3.identity(),self.attributes.get("length",0.1),self.attributes.get("width",0.01))
                        self.displayCache[0].draw(drawRaw,transform=link.getTransform())
                        self.displayCache[1].draw(drawRaw,transform=se3.mul(link.getTransform(),(R,[0,0,0])))
                    elif item.numRotDims() > 0:
                        #axis constraint
                        d = [0,0,0]
                        def drawRawLine():
                            glDisable(GL_LIGHTING)
                            glColor4f(*self.attributes.get("color",[0.5,0,0.5,1]))
                            glLineWidth(self.attributes.get("width",3.0))
                            glBegin(GL_LINES)
                            glVertex3f(0,0,0)
                            glVertex3f(*vectorops.mul(d,self.attributes.get("length",0.1)))
                            glEnd()
                            glLineWidth(1.0)
                        ld,wd = item.getRotationAxis()
                        d = ld
                        self.displayCache[0].draw(drawRawLine,transform=link.getTransform(),parameters=d)
                        d = wd
                        self.displayCache[1].draw(drawRawLine,transform=(dest.getTransform()[0] if dest else so3.identity(),wp),parameters=d)
                    else:
                        #no drawing
                        pass
                    if name != None:
                        self.drawText(name,wp)
        elif isinstance(item,(GeometricPrimitive,TriangleMesh,PointCloud,Geometry3D)):
            if not hasattr(self,'appearance'):
                self.appearance = Appearance()
            c = self.attributes.get("color",[0.5,0.5,0.5,1])
            self.appearance.setColor(*c)
            s = self.attributes.get("size",None)
            if s:
                self.appearance.setPointSize(s)
            wp = None
            geometry = None
            lighting = True
            if isinstance(self.item,GeometricPrimitive):
                if not hasattr(self,'geometry'):
                    self.geometry = Geometry3D(self.item)
                geometry = self.geometry
                if self.item.type not in ['Sphere','AABB']:
                    lighting = False
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
                    if prim.type not in ['Sphere','AABB']:
                        lighting = False
                elif self.item.type() == 'PointCloud':
                    lighting = False
                geometry = self.item
            if lighting:
                glEnable(GL_LIGHTING)
            else:
                glDisable(GL_LIGHTING)
            self.appearance.drawWorldGL(geometry)
            if name != None:
                bmin,bmax = geometry.getBB()
                wp = vectorops.mul(vectorops.add(bmin,bmax),0.5)
                self.drawText(name,wp)
        else:
            try:
                itypes = self.attributes['type']
            except KeyError:
                try:
                    itypes = objectToVisType(item,world)
                except:
                    print "visualization.py: Unsupported object type",item,"of type:",item.__class__.__name__
                    return
            if itypes == None:
                print "Unable to convert item",item,"to drawable"
                return
            elif itypes == 'Config':
                if world:
                    robot = world.robot(0)
                    if not self.useDefaultAppearance:
                        oldAppearance = [robot.link(i).appearance().clone() for i in xrange(robot.numLinks())]
                        for i in xrange(robot.numLinks()):
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
                    print "Unable to draw Config items without a world"
            elif itypes == 'Configs':
                if world:
                    maxConfigs = self.attributes.get("maxConfigs",min(10,len(item)))
                    robot = world.robot(0)
                    if not self.useDefaultAppearance:
                        oldAppearance = [robot.link(i).appearance().clone() for i in xrange(robot.numLinks())]
                        for i in xrange(robot.numLinks()):
                          if self.customAppearance is not None:
                            robot.link(i).appearance().set(self.customAppearance)
                          elif "color" in self.attributes:
                            robot.link(i).appearance().setColor(*self.attributes["color"])

                    oldconfig = robot.getConfig()
                    for i in xrange(maxConfigs):
                        idx = int(i*len(item))/maxConfigs
                        robot.setConfig(item[idx])
                        robot.drawGL()
                    robot.setConfig(oldconfig)
                    if not self.useDefaultAppearance:
                        for (i,app) in enumerate(oldAppearance):
                            robot.link(i).appearance().set(app)
                else:
                    print "Unable to draw Configs items without a world"
            elif itypes == 'Vector3':
                def drawRaw():
                    glDisable(GL_LIGHTING)
                    glEnable(GL_POINT_SMOOTH)
                    glPointSize(self.attributes.get("size",5.0))
                    glColor4f(*self.attributes.get("color",[0,0,0,1]))
                    glBegin(GL_POINTS)
                    glVertex3f(0,0,0)
                    glEnd()
                self.displayCache[0].draw(drawRaw,[so3.identity(),item])
                if name != None:
                    self.drawText(name,item)
            elif itypes == 'RigidTransform':
                def drawRaw():
                    fancy = self.attributes.get("fancy",False)
                    if fancy: glEnable(GL_LIGHTING)
                    else: glDisable(GL_LIGHTING)
                    gldraw.xform_widget(se3.identity(),self.attributes.get("length",0.1),self.attributes.get("width",0.01),fancy=fancy)
                self.displayCache[0].draw(drawRaw,transform=item)
                if name != None:
                    self.drawText(name,item[1])
            else:
                print "Unable to draw item of type \"%s\""%(str(itypes),)

        #revert appearance
        if not self.useDefaultAppearance and hasattr(item,'appearance'):
            item.appearance().set(self.oldAppearance)

    def getBounds(self):
        """Returns a bounding box (bmin,bmax) or None if it can't be found"""
        if len(self.subAppearances)!=0:
            bb = aabb_create()
            for n,app in self.subAppearances.iteritems():
                bb = aabb_expand(bb,app.getBounds())
            return bb
        item = self.item
        if isinstance(item,coordinates.Point):
            return [item.worldCoordinates(),item.worldCoordinates()]
        elif isinstance(item,coordinates.Direction):
            T = item.frame().worldCoordinates()
            d = item.localCoordinates()
            L = self.attributes.get("length",0.1)
            return aabb_create(T[1],se3.apply(T,vectorops.mul(d,L)))
        elif isinstance(item,coordinates.Frame):
            T = item.worldCoordinates()
            L = self.attributes.get("length",0.1)
            return aabb_create(T[1],se3.apply(T,(L,0,0)),se3.apply(T,(0,L,0)),se3.apply(T,(0,0,L)))
        elif isinstance(item,ContactPoint):
            L = self.attributes.get("length",0.05)
            return aabb_create(item.x,vectorops.madd(item.x,item.n,L))
        elif isinstance(item,WorldModel):
            pass
        elif hasattr(item,'geometry'):
            return item.geometry().getBB()
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
                    return (item[1],item[1])
            except Exception:
                pass
            print "Empty bound for object",self.name,"type",self.item.__class__.__name__
        return aabb_create()

    def getSubItem(self,path):
        if len(path) == 0: return self
        for k,v in self.subAppearances.iteritems():
            if v.name == path[0]:
                try:
                    return v.getSubItem(path[1:])
                except ValueError,e:
                    raise ValueError("Invalid sub-path specified "+str(path)+" at "+str(e))
        raise ValueError("Invalid sub-item specified "+path[0])

    def make_editor(self,world=None):
        if self.editor != None:
            return 
        item = self.item
        if isinstance(item,coordinates.Point):
            res = PointPoser()
            res.set(self.item.worldCoordinates())
            res.setAxes(self.item.frame().worldCoordinates()[0])
        elif isinstance(item,coordinates.Direction):
            res = PointPoser()
            res.set(self.item.worldCoordinates())
            res.setAxes(self.item.frame().worldCoordinates()[0])
        elif isinstance(item,coordinates.Frame):
            res = TransformPoser()
            res.set(*self.item.worldCoordinates())
        elif isinstance(self.item,RobotModel):
            res = RobotPoser(self.item)
            self.hidden = True
        elif isinstance(self.item,SubRobotModel):
            res = RobotPoser(self.item._robot)
            res.setActiveDofs(self.item.links);
            self.hidden = True
        elif isinstance(self.item,RigidObjectModel):
            res = ObjectPoser(self.item)
        elif isinstance(self.item,(list,tuple)):
            #determine if it's a rotation, transform, or point
            itype = objectToVisType(self.item,None)
            if itype == 'Vector3':
                res = PointPoser()
                res.set(self.item)
            elif itype == 'Matrix3':
                res = TransformPoser()
                res.enableRotation(True)
                res.enableTranslation(False)
                res.set(self.item)
            elif itype == 'RigidTransform':
                res = TransformPoser()
                res.enableRotation(True)
                res.enableTranslation(True)
                res.set(*self.item)
            elif itype == 'Config':
                if world is not None and world.numRobots() > 0 and world.robot(0).numLinks() == len(item):
                    #it's a valid configuration
                    oldconfig = world.robot(0).getConfig()
                    world.robot(0).setConfig(self.item)
                    res = RobotPoser(world.robot(0))
                    world.robot(0).setConfig(oldconfig)
                    self.hidden = True
                else:
                    print "VisAppearance.make_editor(): Warning, editor for object of type",itype,"cannot be associated with a robot"
                    return
            else:
                print "VisAppearance.make_editor(): Warning, editor for object of type",itype,"not defined"
                return
        else:
            print "VisAppearance.make_editor(): Warning, editor for object of type",self.item.__class__.__name__,"not defined"
            return
        self.editor = res

    def update_editor(self,item_to_editor=False):
        for (name,item) in self.subAppearances.iteritems():
            item.update_editor(item_to_editor)
        if self.editor == None:
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
                            for i in xrange(len(a)):
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
                print "Edited a tuple... maybe a point or an xform? can't actually edit"
                self.item = self.editor.get()
            else:
                raise RuntimeError("Uh... unsupported type with an editor?")
                
    def remove_editor(self):
        self.editor = None
        self.hidden = False

class VisualizationPlugin(glcommon.GLWidgetPlugin):
    def __init__(self):
        glcommon.GLWidgetPlugin.__init__(self)
        self.items = {}
        self.labels = []
        self.t = time.time()
        self.startTime = self.t
        self.animating = True
        self.currentAnimationTime = 0
        self.doRefresh = False

    def initialize(self):
        #keep or refresh display lists?
        #self._clearDisplayLists()
        return glcommon.GLWidgetPlugin.initialize(self)

    def addLabel(self,text,point,color):
        self.labels.append((text,point,color))

    def display(self):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.display(self)
        self.labels = []
        world = self.items.get('world',None)
        if world != None: world=world.item
        for (k,v) in self.items.iteritems():
            v.widget = self
            v.swapDrawConfig()
            v.draw(world)
            v.swapDrawConfig()
            v.widget = None #allows garbage collector to delete these objects
        #cluster label points
        pointTolerance = self.view.camera.dist*0.03
        pointHash = {}
        for (text,point,color) in self.labels:
            index = tuple([int(x/pointTolerance) for x in point])
            try:
                pointHash[index][1].append((text,color))
            except KeyError:
                pointHash[index] = [point,[(text,color)]]
        for (p,items) in pointHash.itervalues():
            self._drawLabelRaw(p,*zip(*items))
        _globalLock.release()

    def display_screen(self):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.display_screen(self)
        cx = 20
        cy = 20
        glDisable(GL_LIGHTING)
        glDisable(GL_DEPTH_TEST)
        for (k,v) in self.items.iteritems():
            if isinstance(v.item,VisPlot):
                pos = v.attributes.get('position',None)
                duration = v.attributes.get('duration',5.)
                vrange = v.attributes.get('range',(None,None))
                w,h = v.attributes.get('size',(200,150))
                if pos is None:
                    v.item.render(self.window,cx,cy,w,h,duration,vrange[0],vrange[1])
                    cy += h+18
                else:
                    x = pos[0]
                    y = pos[1]
                    if x < 0:
                        x = self.view.w + x
                    if y < 0:
                        y = self.view.h + y
                    v.item.render(self.window,x,y,w,h,duration,vrange[0],vrange[1])
        for (k,v) in self.items.iteritems():
            if isinstance(v.item,str):
                pos = v.attributes.get('position',None)
                col = v.attributes.get('color',(0,0,0))
                size = v.attributes.get('size',12)
                if pos is None:
                    #draw at console
                    self.window.draw_text((cx,cy+size),v.item,size,col)
                    cy += (size*15)/10
                elif len(pos)==2:
                    x = pos[0]
                    y = pos[1]
                    if x < 0:
                        x = self.view.w + x
                    if y < 0:
                        y = self.view.h + y
                    self.window.draw_text((x,y+size),v.item,size,col)
        glEnable(GL_DEPTH_TEST)
        _globalLock.release()

    def reshapefunc(self,w,h):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.reshapefunc(self,w,h)
        _globalLock.release()
    def keyboardfunc(self,c,x,y):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.keyboardfunc(self,c,x,y)
        _globalLock.release()
    def keyboardupfunc(self,c,x,y):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.keyboardupfunc(self,c,x,y)
        _globalLock.release()
    def mousefunc(self,button,state,x,y):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.mousefunc(self,button,state,x,y)
        _globalLock.release()
    def motionfunc(self,x,y,dx,dy):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.motionfunc(self,x,y,dx,dy)
        _globalLock.release()
    def eventfunc(self,type,args=""):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.eventfunc(self,type,args)
        _globalLock.release()
    def closefunc(self):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.closefunc(self)
        _globalLock.release()

    def _drawLabelRaw(self,point,textList,colorList):
        #assert not self.makingDisplayList,"drawText must be called outside of display list"
        assert self.window != None
        for i,(text,c) in enumerate(zip(textList,colorList)):
            if i+1 < len(textList): text = text+","

            projpt = self.view.project(point,clip=False)
            if projpt[2] > self.view.clippingplanes[0]:
                d = float(12)/float(self.view.w)*projpt[2]*0.7
                point = vectorops.add(point,so3.apply(so3.inv(self.view.camera.matrix()[0]),(0,-d,0)))

            glDisable(GL_LIGHTING)
            glDisable(GL_DEPTH_TEST)
            glColor3f(*c)
            self.draw_text(point,text,size=12)
            glEnable(GL_DEPTH_TEST)

    def _clearDisplayLists(self):
        for i in self.items.itervalues():
            i.clearDisplayLists()

    def idle(self):
        global _globalLock
        _globalLock.acquire()
        oldt = self.t
        self.t = time.time()
        if self.animating:
            self.currentAnimationTime += (self.t - oldt)
            for (k,v) in self.items.iteritems():
                #do animation updates
                v.updateAnimation(self.currentAnimationTime)
        for (k,v) in self.items.iteritems():
            #do other updates
            v.updateTime(self.t-self.startTime)
        _globalLock.release()
        return False

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
        _globalLock.acquire()
        if item_name == 'all':
            for (name,itemvis) in self.items.iteritems():
                itemvis.markChanged()
        else:
            self.getItem(item_name).markChanged()
        _globalLock.release()

    def clear(self):
        """Clears the visualization world"""
        global _globalLock
        _globalLock.acquire()
        for (name,itemvis) in self.items.iteritems():
            itemvis.destroy()
        self.items = {}
        _globalLock.release()

    def clearText(self):
        """Clears all text in the visualization."""
        global _globalLock
        _globalLock.acquire()
        del_items = []
        for (name,itemvis) in self.items.iteritems():
            if isinstance(itemvis.item,str):
                itemvis.destroy()
                del_items.append(name)
        for n in del_items:
            del self.items[n]
        _globalLock.release()

    def listItems(self,root=None,indent=0):
        """Prints out all items in the visualization world."""
        if root == None:
            for name,value in self.items.iteritems():
                self.listItems(value,indent)
        else:
            if isinstance(root,str):
                root = self.getItem(root)
            if indent > 0:
                print " "*(indent-1),
            print root.name
            for n,v in root.subAppearances.iteritems():
                self.listItems(v,indent+2)

    def add(self,name,item,keepAppearance=False):
        """Adds a named item to the visualization world.  If the item already
        exists, the appearance information will be reinitialized if keepAppearance=False
        (default) or be kept if keepAppearance=True."""
        global _globalLock
        assert not isinstance(name,(list,tuple)),"Cannot add sub-path items"
        _globalLock.acquire()
        if keepAppearance and name in self.items:
            self.items[name].setItem(item)
        else:
            #need to erase prior item visualizer
            if name in self.items:
                self.items[name].destroy()
            app = VisAppearance(item,name)
            self.items[name] = app
        _globalLock.release()
        #self.refresh()

    def addText(self,name,text,pos=None):
        self.add(name,text,True)
        if pos is not None:
            self.setAttribute(name,'position',pos)


    def animate(self,name,animation,speed=1.0,endBehavior='loop'):
        global _globalLock
        _globalLock.acquire()
        if hasattr(animation,'__iter__'):
            #a list of milestones -- loop through them with 1s delay
            print "visualization.animate(): Making a Trajectory with unit durations between",len(animation),"milestones"
            animation = Trajectory(range(len(animation)),animation)
        if isinstance(animation,HermiteTrajectory):
            animation = animation.configTrajectory()
        if isinstance(animation,MultiPath):
            world = self.items.get('world',None)
            if world != None:
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
        item.markChanged()
        _globalLock.release()

    def pauseAnimation(self,paused=True):
        global _globalLock
        _globalLock.acquire()
        self.animating = not paused
        _globalLock.release()

    def stepAnimation(self,amount):
        global _globalLock
        _globalLock.acquire()
        self.currentAnimationTime += amount
        self.doRefresh = True
        _globalLock.release()

    def animationTime(self,newtime=None):
        global _globalLock
        if self==None:
            print "Visualization disabled"
            return 0
        if newtime != None:
            _globalLock.acquire()
            self.currentAnimationTime = newtime
            _globalLock.release()
        return self.currentAnimationTime

    def remove(self,name):
        global _globalLock
        _globalLock.acquire()
        assert name in self.items,"Can only remove top level objects from visualization, try hide() instead"
        item = self.getItem(name)
        item.destroy()
        del self.items[name]
        self.doRefresh = True
        _globalLock.release()

    def getItemConfig(self,name):
        global _globalLock
        _globalLock.acquire()
        res = config.getConfig(self.getItem(name).item)
        _globalLock.release()
        return res

    def setItemConfig(self,name,value):
        global _globalLock
        _globalLock.acquire()
        item = self.getItem(name)
        if isinstance(item.item,(list,tuple,str)):
            item.item = value
        else:
            config.setConfig(item.item,value)
        if item.editor:
            item.update_editor(item_to_editor = True)
        self.doRefresh = True
        _globalLock.release()

    def hideLabel(self,name,hidden=True):
        global _globalLock
        _globalLock.acquire()
        item = self.getItem(name)
        item.attributes["text_hidden"] = hidden
        item.markChanged()
        self.doRefresh = True
        _globalLock.release()

    def edit(self,name,doedit=True):
        global _globalLock
        _globalLock.acquire()
        obj = self.getItem(name)
        if obj == None:
            _globalLock.release()
            raise ValueError("Object "+name+" does not exist in visualization")
        if doedit:
            world = self.items.get('world',None)
            if world != None:
                world=world.item
            obj.make_editor(world)
            if obj.editor:
                self.klamptwidgetmaster.add(obj.editor)
        else:
            if obj.editor:
                self.klamptwidgetmaster.remove(obj.editor)
                obj.remove_editor()
        self.doRefresh = True
        _globalLock.release()

    def widgetchangefunc(self,edit):
        """Called by GLWidgetPlugin on any widget change"""
        for name,item in self.items.iteritems():
            item.update_editor()

    def hide(self,name,hidden=True):
        global _globalLock
        _globalLock.acquire()
        self.getItem(name).hidden = hidden
        self.doRefresh = True
        _globalLock.release()

    def addPlotItem(self,plotname,itemname):
        global _globalLock
        _globalLock.acquire()
        plot = self.getItem(plotname)
        assert plot != None and isinstance(plot.item,VisPlot),(plotname+" is not a valid plot")
        plot = plot.item
        for i in plot.items:
            assert i.name != itemname,(str(itemname)+" is already in the plot "+plotname)
        item = self.getItem(itemname)
        assert item != None,(str(itemname)+" is not a valid item")
        plot.items.append(VisPlotItem(itemname,item))
        _globalLock.release()

    def logPlot(self,plotname,itemname,value):
        global _globalLock
        _globalLock.acquire()
        customIndex = -1
        plot = self.getItem(plotname)
        assert plot != None and isinstance(plot.item,VisPlot),(plotname+" is not a valid plot")
        compress = plot.attributes.get('compress',_defaultCompressThreshold)
        plot = plot.item
        for i,item in enumerate(plot.items):
            if len(item.name)==0:
                customIndex = i
        if customIndex < 0:
            customIndex = len(plot.items)
            plot.items.append(VisPlotItem('',None))
        plot.items[customIndex].compressThreshold = compress
        plot.items[customIndex].customUpdate(itemname,self.t - self.startTime,value)
        _globalLock.release()

    def logPlotEvent(self,plotname,eventname,color):
        global _globalLock
        _globalLock.acquire()
        plot = self.getItem(plotname)
        assert plot != None and isinstance(plot.item,VisPlot),(plotname+" is not a valid plot")
        plot.item.addEvent(eventname,self.t-self.startTime,color)
        _globalLock.release()

    def hidePlotItem(self,plotname,itemname,hidden=True):
        global _globalLock
        _globalLock.acquire()
        plot = self.getItem(plotname)
        assert plot != None and isinstance(plot.item,VisPlot),plotname+" is not a valid plot"
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
                    for j in xrange(len(i.hidden)):
                        i.hidden[j] = hidden
        assert identified,("Invalid item "+str(itemname)+" specified in plot "+plotname)
        self.doRefresh = True
        _globalLock.release()

    def savePlot(self,plotname,fn):
        global _globalLock
        _globalLock.acquire()
        plot = self.getItem(plotname)
        assert plot != None and isinstance(plot.item,VisPlot),plotname+" is not a valid plot"
        plot = plot.item
        if fn != None:
            plot.beginSave(fn)
        else:
            plot.endSave(fn)
        _globalLock.release()

    def setAppearance(self,name,appearance):
        global _globalLock
        _globalLock.acquire()
        item = self.getItem(name)
        item.useDefaultAppearance = False
        item.customAppearance = appearance
        item.markChanged()
        self.doRefresh = True
        _globalLock.release()

    def setAttribute(self,name,attr,value):
        global _globalLock
        _globalLock.acquire()
        item = self.getItem(name)
        item.attributes[attr] = value
        if value==None:
            del item.attributes[attr]
        item.markChanged()
        self.doRefresh = True
        _globalLock.release()

    def revertAppearance(self,name):
        global _globalLock
        _globalLock.acquire()
        item = self.getItem(name)
        item.useDefaultApperance = True
        item.markChanged()
        self.doRefresh = True
        _globalLock.release()

    def setColor(self,name,r,g,b,a=1.0):
        global _globalLock
        _globalLock.acquire()
        item = self.getItem(name)
        item.attributes["color"] = [r,g,b,a]
        item.useDefaultAppearance = False
        item.markChanged()
        self.doRefresh = True
        _globalLock.release()

    def setDrawFunc(self,name,func):
        global _globalLock
        _globalLock.acquire()
        item = self.getItem(name)
        item.customDrawFunc = func
        self.doRefresh = True
        _globalLock.release()

    def autoFitCamera(self,scale=1.0):
        vp = None
        if self.window == None:
            global _frontend
            vp = _frontend.get_view()
        else:
            vp = self.window.get_view()
        try:
            autoFitViewport(vp,self.items.values())
            vp.camera.dist /= scale
        except Exception as e:
            print "Unable to auto-fit camera"
            print e



_vis = VisualizationPlugin() 
_frontend.setPlugin(_vis)

#signals to visualization thread
_quit = False
_in_vis_loop = False
_vis_thread_running = False
_vis_thread = None
_in_app_thread = False
_use_multithreaded = (True if sys.platform != 'darwin' else False)

if _PyQtAvailable:
    if _PyQt5Available:
        from PyQt5.QtWidgets import QDialog,QInputDialog,QDialogButtonBox,QMainWindow,QApplication,QAction
    else:
        from PyQt4.QtGui import QDialog,QInputDialog,QDialogButtonBox,QMainWindow,QApplication,QAction
    #Qt specific startup
    #need to set up a QDialog and an QApplication
    class _MyDialog(QDialog):
        def __init__(self,windowinfo):
            QDialog.__init__(self)
            self.windowinfo = windowinfo
            glwidget = windowinfo.glwindow
            glwidget.setMinimumSize(640,480)
            glwidget.setMaximumSize(4000,4000)
            glwidget.setSizePolicy(QSizePolicy(QSizePolicy.Maximum,QSizePolicy.Maximum))

            self.description = QLabel("Press OK to continue")
            self.description.setSizePolicy(QSizePolicy(QSizePolicy.Preferred,QSizePolicy.Fixed))
            self.layout = QVBoxLayout(self)
            self.layout.addWidget(glwidget)
            self.layout.addWidget(self.description)
            self.buttons = QDialogButtonBox(QDialogButtonBox.Ok,Qt.Horizontal, self)
            self.buttons.accepted.connect(self.accept)
            self.layout.addWidget(self.buttons)
            self.setWindowTitle(windowinfo.name)
            glwidget.name = windowinfo.name
        def accept(self):
            global _globalLock
            _globalLock.acquire()
            self.windowinfo.glwindow.hide()
            _globalLock.release()
            print "#########################################"
            print "klampt.vis: Dialog accept"
            print "#########################################"
            return QDialog.accept(self)
        def reject(self):
            global _globalLock
            _globalLock.acquire()
            self.windowinfo.glwindow.hide()
            print "#########################################"
            print "klampt.vis: Dialog reject"
            print "#########################################"
            _globalLock.release()
            return QDialog.reject(self)
        
    class _MyWindow(QMainWindow):
        def __init__(self,windowinfo):
            QMainWindow.__init__(self)
            self.windowinfo = windowinfo
            self.glwidget = windowinfo.glwindow
            self.glwidget.setMinimumSize(self.glwidget.width,self.glwidget.height)
            self.glwidget.setMaximumSize(4000,4000)
            self.glwidget.setSizePolicy(QSizePolicy(QSizePolicy.Maximum,QSizePolicy.Maximum))
            self.setCentralWidget(self.glwidget)
            self.glwidget.setParent(self)
            self.setWindowTitle(windowinfo.name)
            self.glwidget.name = windowinfo.name
            self.saving_movie = False
            self.movie_timer = QTimer(self)
            self.movie_timer.timeout.connect(self.movie_update)
            self.movie_frame = 0
            self.movie_time_last = 0
            self.saving_html = False
            self.html_saver = None
            self.html_start_time = 0
            self.html_timer = QTimer(self)
            self.html_timer.timeout.connect(self.html_update)
            #TODO: for action-free programs, don't add this... but this has to be detected after initializeGL()?
            mainMenu = self.menuBar()
            fileMenu = mainMenu.addMenu('&Actions')
            self.glwidget.actionMenu = fileMenu
            visMenu = mainMenu.addMenu('&Visualization')
            a = QAction('Save world...', self)
            a.setStatusTip('Saves world to xml file')
            a.triggered.connect(self.save_world)
            visMenu.addAction(a)
            a = QAction('Add to world...', self)
            a.setStatusTip('Adds an item to the world')
            a.triggered.connect(self.add_to_world)
            visMenu.addAction(a)
            a = QAction('Save camera...', self)
            a.setStatusTip('Saves camera settings')
            a.triggered.connect(self.save_camera)
            visMenu.addAction(a)
            a = QAction('Load camera...', self)
            a.setStatusTip('Loads camera settings')
            a.triggered.connect(self.load_camera)
            visMenu.addAction(a)
            a = QAction('Start/stop movie output', self)
            a.setShortcut('Ctrl+M')
            a.setStatusTip('Starts / stops saving movie frames')
            a.triggered.connect(self.toggle_movie_mode)
            visMenu.addAction(a)
            a = QAction('Start/stop html output', self)
            a.setShortcut('Ctrl+H')
            a.setStatusTip('Starts / stops saving animation to HTML file')
            a.triggered.connect(self.toggle_html_mode)
            visMenu.addAction(a)
        def getWorld(self):
            if not hasattr(self.glwidget.program,'plugins'):
                return None
            for p in self.glwidget.program.plugins:
                if hasattr(p,'world'):
                    return p.world
                elif isinstance(p,VisualizationPlugin):
                    world = p.items.get('world',None)
                    if world != None: return world.item
            return None
        def getSimulator(self):
            if not hasattr(self.glwidget.program,'plugins'):
                return None
            for p in self.glwidget.program.plugins:
                if hasattr(p,'sim'):
                    return p.sim
                elif isinstance(p,VisualizationPlugin):
                    sim = p.items.get('sim',None)
                    if sim != None: return sim.item
            return None
        def save_camera(self):
            if not hasattr(self.glwidget.program,'get_view'):
                print "Program does not appear to have a camera"
                return
            v = self.glwidget.program.get_view()
            fn = QFileDialog.getSaveFileName(caption="Viewport file (*.txt)",filter="Viewport file (*.txt);;All files (*.*)",options=QFileDialog.DontUseNativeDialog)
            if fn is None:
                return
            f = open(str(fn),'w')
            f.write("VIEWPORT\n")
            f.write("FRAME %d %d %d %d\n"%(v.x,v.y,v.w,v.h))
            f.write("PERSPECTIVE 1\n")
            aspect = float(v.w)/float(v.h)
            rfov = v.fov*math.pi/180.0
            scale = 1.0/(2.0*math.tan(rfov*0.5/aspect)*aspect)
            f.write("SCALE %f\n"%(scale,))
            f.write("NEARPLANE %f\n"%(v.clippingplanes[0],))
            f.write("FARPLANE %f\n"%(v.clippingplanes[1],))
            f.write("CAMTRANSFORM ")
            mat = se3.homogeneous(v.camera.matrix())
            f.write(' '.join(str(v) for v in sum(mat,[])))
            f.write('\n')
            f.write("ORBITDIST %f\n"%(v.camera.dist,))
            f.close()
        def load_camera(self):
            v = self.glwidget.program.get_view()
            fn = QFileDialog.getOpenFileName(caption="Viewport file (*.txt)",filter="Viewport file (*.txt);;All files (*.*)",options=QFileDialog.DontUseNativeDialog)
            if fn is None:
                return
            f = open(str(fn),'r')
            read_viewport = False
            mat = None
            for line in f:
                entries = line.split()
                if len(entries) == 0:
                    continue
                kw = entries[0]
                args = entries[1:]
                if kw == 'VIEWPORT':
                    read_viewport = True
                    continue
                else:
                    if not read_viewport:
                        print "File does not appear to be a valid viewport file, must start with VIEWPORT"
                        break
                if kw == 'FRAME':
                    v.x,v.y,v.w,v.h = [int(x) for x in args]
                elif kw == 'PERSPECTIVE':
                    if args[0] != '1':
                        print "WARNING: CANNOT CHANGE TO ORTHO MODE IN PYTHON VISUALIZATION"
                elif kw == 'SCALE':
                    scale = float(args[0])
                    aspect = float(v.w)/float(v.h)
                    #2.0*math.tan(rfov*0.5/aspect)*aspect = 1.0/scale
                    #math.tan(rfov*0.5/aspect) = 0.5/(scale*aspect)
                    #rfov*0.5/aspect = math.atan(0.5/(scale*aspect))
                    #rfov = 2*aspect*math.atan(0.5/(scale*aspect))
                    rfov = math.atan(0.5/(scale*aspect))*2*aspect
                    v.fov = math.degrees(rfov)
                elif kw == 'NEARPLANE':
                    v.clippingplanes = (float(args[0]),v.clippingplanes[1])
                elif kw == 'FARPLANE':
                    v.clippingplanes = (v.clippingplanes[0],float(args[0]))
                elif kw == 'CAMTRANSFORM':
                    mat = [args[0:4],args[4:8],args[8:12],args[12:16]]
                    for i,row in enumerate(mat):
                        mat[i] = [float(x) for x in row]
                elif kw == 'ORBITDIST':
                    v.camera.dist = float(args[0])
                else:
                    raise RuntimeError("Invalid viewport keyword "+kw)
            if mat != None:
                v.camera.set_matrix(se3.from_homogeneous(mat))
            self.glwidget.program.set_view(v)
            f.close()

        def save_world(self):
            w = self.getWorld()
            if w is None:
                print "Program does not appear to have a world"
            fn = QFileDialog.getSaveFileName(caption="World file (elements will be saved to folder)",filter="World file (*.xml);;All files (*.*)")
            if fn != None:
                w.saveFile(str(fn))
                print "Saved to",fn,"and elements were saved to a directory of the same name."
        def add_to_world(self):
            w = self.getWorld()
            if w is None:
                print "Program does not appear to have a world"
            fn = QFileDialog.getOpenFileName(caption="World element",filter="Robot file (*.rob *.urdf);;Object file (*.obj);;Terrain file (*.env *.off *.obj *.stl *.wrl);;All files (*.*)")
            if fn != None:
                w.loadElement(str(fn))
                for p in self.glwidget.program.plugins:
                    if isinstance(p,VisualizationPlugin):
                        p.getItem('world').setItem(w)
        def toggle_movie_mode(self):
            self.saving_movie = not self.saving_movie
            if self.saving_movie:
                self.movie_timer.start(33)
                sim = self.getSimulator()
                if sim != None:
                    self.movie_time_last = sim.getTime()
            else:
                self.movie_timer.stop()
                dlg =  QInputDialog(self)                 
                dlg.setInputMode( QInputDialog.TextInput) 
                dlg.setLabelText("Command")
                dlg.setTextValue('ffmpeg -y -f image2 -i image%04d.png klampt_record.mp4')
                dlg.resize(500,100)                             
                ok = dlg.exec_()                                
                cmd = dlg.textValue()
                #(cmd,ok) = QInputDialog.getText(self,"Process with ffmpeg?","Command", text='ffmpeg -y -f image2 -i image%04d.png klampt_record.mp4')
                if ok:
                    import os,glob
                    os.system(str(cmd))
                    print "Removing temporary files"
                    for fn in glob.glob('image*.png'):
                        os.remove(fn)
        def movie_update(self):
            sim = self.getSimulator()
            if sim != None:
                while sim.getTime() >= self.movie_time_last + 1.0/30.0:
                    self.glwidget.program.save_screen('image%04d.png'%(self.movie_frame))
                    self.movie_frame += 1
                    self.movie_time_last += 1.0/30.0
            else:
                self.glwidget.program.save_screen('image%04d.png'%(self.movie_frame))
                self.movie_frame += 1
        def toggle_html_mode(self):
            self.saving_html = not self.saving_html
            if self.saving_html:
                world = self.getSimulator()
                if world is None:
                    world = self.getWorld()
                if world is None:
                    print "There is no world in the current plugin, can't save"
                    self.saving_html = False
                    return
                fn = QFileDialog.getSaveFileName(caption="Save path HTML file to...",filter="HTML file (*.html);;All files (*.*)")
                if fn is None:
                    self.saving_html = False
                    return
                from ..io import html
                self.html_start_time = time.time()
                self.html_saver = html.HTMLSharePath(fn)
                self.html_saver.dt = 0.033;
                self.html_saver.start(world)
                self.html_timer.start(33)
            else:
                self.html_saver.end()
                self.html_timer.stop()
        def html_update(self):
            t = None
            if self.html_saver.sim == None:
                #t = time.time()-self.html_start_time
                t = self.html_saver.last_t + 0.034
            self.html_saver.animate(t)
        def closeEvent(self,event):
            global _globalLock
            _globalLock.acquire()
            self.windowinfo.glwindow.hide()
            self.windowinfo.mode = 'hidden'
            self.windowinfo.glwindow.idlesleep()
            self.windowinfo.glwindow.setParent(None)
            if self.saving_movie:
                self.toggle_movie_mode()
            if self.saving_html:
                self.toggle_html_mode()
            #self.html_timer.deleteLater()
            #self.movie_timer.deleteLater()
            print "#########################################"
            print "klampt.vis: Window close"
            print "#########################################"
            _globalLock.release()
        def close(self):
            """Called to clean up resources"""
            self.html_timer.stop()
            self.movie_timer.stop()
            self.html_timer.deleteLater()
            self.movie_timer.deleteLater()
            self.movie_timer = None
            self.html_timer = None
        def detachGLWindow(self):
            """Used for closing and restoring windows, while saving the OpenGL context"""
            self.glwidget.setParent(None)
        def attachGLWindow(self):
            """Used for closing and restoring windows, while saving the OpenGL context"""
            self.glwidget.setParent(self)
            self.setCentralWidget(self.glwidget)

    def _run_app_thread(callback=None):
        global _vis_thread_running,_in_app_thread,_vis,_widget,_window,_quit,_showdialog,_showwindow,_globalLock
        _vis_thread_running = True

        _GLBackend.initialize("Klamp't visualization")
        
        res = None
        while not _quit:
            _globalLock.acquire()
            for i,w in enumerate(_windows):
                if w.glwindow == None and w.mode != 'hidden':
                    print "vis: creating GL window"
                    w.glwindow = _GLBackend.createWindow(w.name)
                    w.glwindow.setProgram(w.frontend)
                    w.glwindow.setParent(None)
                    w.glwindow.refresh()
                if w.doRefresh:
                    if w.mode != 'hidden':
                        w.glwindow.updateGL()
                    w.doRefresh = False
                if w.doReload and w.glwindow != None:
                    w.glwindow.setProgram(w.frontend)
                    if w.guidata:
                        w.guidata.setWindowTitle(w.name)
                        w.guidata.glwidget = w.glwindow
                        w.guidata.attachGLWindow()
                    w.doReload = False
                if w.mode == 'dialog':
                    print "#########################################"
                    print "klampt.vis: Dialog on window",i
                    print "#########################################"
                    if w.custom_ui == None:
                        dlg = _MyDialog(w)
                    else:
                        dlg = w.custom_ui(w.glwindow)
                    #need to cache the bastards to avoid deleting the GL object. Not sure why it's being kept around.
                    #alldlgs.append(dlg)
                    #here's the crash -- above line deleted the old dialog, which for some reason kills the widget
                    if dlg != None:
                        w.glwindow.show()
                        _globalLock.release()
                        res = dlg.exec_()
                        _globalLock.acquire()
                    print "#########################################"
                    print "klampt.vis: Dialog done on window",i
                    print "#########################################"
                    w.glwindow.hide()
                    w.glwindow.setParent(None)
                    w.mode = 'hidden'
                if w.mode == 'shown' and w.guidata == None:
                    print "#########################################"
                    print "klampt.vis: Making window",i
                    print "#########################################"
                    if w.custom_ui == None:
                        w.guidata = _MyWindow(w)
                    else:
                        w.guidata = w.custom_ui(w.glwindow)
                    w.guidata.setWindowTitle(w.name)
                    w.glwindow.show()
                    w.guidata.show()
                    if w.glwindow.initialized:
                        #boot it back up again
                        w.glwindow.idlesleep(0)
                if w.mode == 'shown' and not w.guidata.isVisible():
                    print "#########################################"
                    print "klampt.vis: Showing window",i
                    print "#########################################"
                    if hasattr(w.guidata,'attachGLWindow'):
                        w.guidata.attachGLWindow()
                    else:
                        w.glwindow.setParent(w.guidata)
                    w.glwindow.show()
                    w.guidata.show()
                if w.mode == 'hidden' and w.guidata != None:
                    #prevent deleting the GL window
                    if hasattr(w.guidata,'detachGLWindow'):
                        w.guidata.detachGLWindow()
                    else:
                        w.guidata.setParent(None)
                    if w.guidata.isVisible():
                        print "#########################################"
                        print "klampt.vis: Hiding window",i
                        print "#########################################"
                        w.glwindow.hide()
                        w.guidata.hide()
                    w.guidata.close()
                    w.guidata = None
            _globalLock.release()
            _in_app_thread = True
            _GLBackend.app.processEvents()
            _in_app_thread = False
            if callback:
                callback()
            else:
                if not _in_vis_loop:
                    #give other threads time to work
                    time.sleep(0.001)
            if _in_vis_loop and (len(_windows)==0 or all(w.mode == 'hidden' for w in _windows)):
                print "klampt.vis: No windows shown, breaking out of vis loop"
                _vis_thread_running = False
                return
        print "Visualization thread closing and cleaning up Qt..."
        _cleanup()
        _vis_thread_running = False
        return res

    def _cleanup():
        for w in _windows:
            w.vis.clear()
            if w.glwindow:
                w.glwindow.setParent(None)
                w.glwindow.close()
                #must be explicitly deleted for some reason in PyQt5...
                del w.glwindow
        _GLBackend.app.processEvents()

        #must be explicitly deleted for some reason in PyQt5...
        del _GLBackend.app
        _GLBackend.app = None

elif _GLUTAvailable:
    print "klampt.visualization: QT is not available, falling back to poorer"
    print "GLUT interface.  Returning to another GLUT thread will not work"
    print "properly."
    print ""
    
    class GLUTHijacker(GLPluginProgram):
        def __init__(self,windowinfo):
            GLPluginProgram.__init__(self)
            self.windowinfo = windowinfo
            self.name = windowinfo.name
            self.view = windowinfo.frontend.view
            self.clearColor = windowinfo.frontend.clearColor
            self.actions = windowinfo.frontend.actions
            self.frontend = windowinfo.frontend
            self.inDialog = False
            self.hidden = False
            self.callback = None
        def initialize(self):
            self.frontend.window = self.window
            if not self.frontend.initialize(): return False
            GLPluginProgram.initialize(self)
            return True
        def display(self):
            global _globalLock
            _globalLock.acquire()
            self.frontend.display()
            _globalLock.release()
            return True
        def display_screen(self):
            global _globalLock
            _globalLock.acquire()
            self.frontend.display_screen()
            glColor3f(1,1,1)
            glRasterPos(20,50)
            gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_18,"(Do not close this window except to quit)")
            if self.inDialog:
                glColor3f(1,1,0)
                glRasterPos(20,80)
                gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_18,"In Dialog mode. Press 'Esc' to return to normal mode")
            else:
                glColor3f(1,1,0)
                glRasterPos(20,80)
                gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_18,"In Window mode. Press 'Esc' to hide window")
            _globalLock.release()
        def keyboardfunc(self,c,x,y):
            if len(c)==1 and ord(c)==27:
                if self.inDialog:
                    print "Esc pressed, hiding dialog"
                    self.inDialog = False
                else:
                    print "Esc pressed, hiding window"
                global _globalLock
                _globalLock.acquire()
                self.windowinfo.mode = 'hidden'
                self.hidden = True
                glutHideWindow()
                _globalLock.release()
                return True
            else:
                return self.frontend.keyboardfunc(c,x,y)
        def keyboardupfunc(self,c,x,y):
            return self.frontend.keyboardupfunc(c,x,y)
        def motionfunc(self,x,y,dx,dy):
            return self.frontend.motionfunc(x,y,dx,dy)
        def mousefunc(self,button,state,x,y):
            return self.frontend.mousefunc(button,state,x,y)
        
        def idlefunc(self):
            global _quit,_showdialog
            global _globalLock
            _globalLock.acquire()
            if _quit or (_in_vis_loop and self.hidden):
                if bool(glutLeaveMainLoop):
                    glutLeaveMainLoop()
                else:
                    print "Not compiled with freeglut, can't exit main loop safely. Press Ctrl+C instead"
                    raw_input()
            if self.hidden:
                print "hidden, waiting...",self.windowinfo.mode
                if self.windowinfo.mode == 'shown':
                    print "Showing window"
                    glutSetWindow(self.window.glutWindowID)
                    glutShowWindow()
                    self.hidden = False
                elif self.windowinfo.mode == 'dialog':
                    print "Showing window in dialog mode"
                    self.inDialog = True
                    glutSetWindow(self.window.glutWindowID)
                    glutShowWindow()
                    self.hidden = False
            _globalLock.release()
            if self.callback:
                self.callback()
            return self.frontend.idlefunc()

    def _run_app_thread(callback=None):
        global _vis_thread_running,_vis,_old_glut_window,_quit,_windows
        import weakref
        _vis_thread_running = True
        _GLBackend.initialize("Klamp't visualization")
        w = _GLBackend.createWindow("Klamp't visualization")
        hijacker = GLUTHijacker(_windows[0])
        hijacker.callback = callback
        _windows[0].guidata = weakref.proxy(hijacker)
        w.setProgram(hijacker)
        _GLBackend.run()
        print "Visualization thread closing..."
        _cleanup()
        _vis_thread_running = False
        return

    def _cleanup():
        for w in _windows:
            w.vis.clear()
    
def _kill():
    global _quit,_in_vis_loop,_vis_thread_running,_vis_thread,_cleanup
    _quit = True
    if _in_vis_loop:
        #if the thread is running, _quit=True just signals to the vis loop to quit.
        #otherwise, the program needs to be killed and cleaned up
        if not _vis_thread_running:
            #need to clean up Qt resources
            _cleanup()
        return
    if _vis_thread_running:
        _vis_thread.join()
        assert _vis_thread_running == False

    _quit = False

if _PyQtAvailable:
    if _PyQt5Available:
        from PyQt5 import QtCore
    else:
        from PyQt4 import QtCore
    class MyQThread(QtCore.QThread):
        def __init__(self,func,*args):
            self.func = func
            self.args = args
            QtCore.QThread.__init__(self)
        def run(self):
            self.func(*self.args)

def _loop(setup,callback,cleanup):
    global _in_vis_loop,_vis_thread_running,_use_multithreaded,_quit
    if _vis_thread_running or _in_vis_loop:
        raise RuntimeError("Cannot call loop() after show(), inside dialog(), or inside loop() callbacks")
    _in_vis_loop = True
    try:
        if setup is not None:
            setup()
        _quit = False
        _run_app_thread(callback)
        if cleanup is not None:
            cleanup()
    finally:
        _in_vis_loop = False

def _start_app_thread():
    global _vis_thread
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    if _PyQtAvailable and False:
        #for some reason, QThread doesn't allow for mouse events to be posted?
        _vis_thread = MyQThread(_run_app_thread)
        _vis_thread.start()
    else:
        _vis_thread = Thread(target=_run_app_thread)
        _vis_thread.setDaemon(True)
        _vis_thread.start()
    time.sleep(0.1)


def _show():
    global _windows,_current_window,_in_vis_loop,_vis_thread_running,_vis_thread
    if len(_windows)==0:
        _windows.append(WindowInfo(_window_title,_frontend,_vis)) 
        _current_window = 0
    _windows[_current_window].mode = 'shown'
    _windows[_current_window].worlds = _current_worlds
    _windows[_current_window].active_worlds = _current_worlds[:]
    if _in_vis_loop:
        #this will be handled in the loop, no need to start it
        return
    if not _vis_thread_running:
        _start_app_thread()

def _hide():
    global _windows,_current_window,_vis_thread_running
    if _current_window == None:
        return
    _windows[_current_window].mode = 'hidden'

def _dialog():
    global __windows,_current_window,_in_vis_loop,_vis_thread_running,_vis_thread
    if len(_windows)==0:
        _windows.append(WindowInfo(_window_title,_frontend,_vis,None))
        _current_window = 0
    if _vis_thread_running:
        if _in_vis_loop:
            #single threaded
            raise RuntimeError("Can't call dialog() inside loop().  Try dialogInLoop() instead.")
        #just show the dialog and let the thread take over
        assert _windows[_current_window].mode == 'hidden',"dialog() called inside dialog?"
        print "#########################################"
        print "klampt.vis: Creating dialog on window",_current_window
        print "#########################################"
        _globalLock.acquire()
        _windows[_current_window].mode = 'dialog'
        _windows[_current_window].worlds = _current_worlds
        _windows[_current_window].active_worlds = _current_worlds[:]
        _globalLock.release()

        if not _in_app_thread or threading.current_thread().__class__.__name__ == '_MainThread':
            print "vis.dialog(): Waiting for dialog on window",_current_window,"to complete...."
            while _windows[_current_window].mode == 'dialog':
                time.sleep(0.1)
            print "vis.dialog(): ... dialog done, status is now",_windows[_current_window].mode
        else:
            #called from another dialog or window!
            print "vis: Creating a dialog from within another dialog or window"
            _globalLock.acquire()
            w = _windows[_current_window]
            if w.glwindow == None:
                print "vis: creating GL window"
                w.glwindow = _GLBackend.createWindow(w.name)
                w.glwindow.setProgram(w.frontend)
                w.glwindow.setParent(None)
                w.glwindow.refresh()
            if w.custom_ui == None:
                dlg = _MyDialog(w)
            else:
                dlg = w.custom_ui(w.glwindow)
            print "#########################################"
            print "klampt.vis: Dialog starting on window",_current_window
            print "#########################################"
            if dlg != None:
                w.glwindow.show()
                _globalLock.release()
                res = dlg.exec_()
                _globalLock.acquire()
            print "#########################################"
            print "klampt.vis: Dialog done on window",_current_window
            print "#########################################"
            w.glwindow.hide()
            w.glwindow.setParent(None)
            w.mode = 'hidden'
            _globalLock.release()
        return None
    else:
        _windows[_current_window].mode = 'dialog'
        _windows[_current_window].worlds = _current_worlds
        _windows[_current_window].active_worlds = _current_worlds[:]
        if _use_multithreaded:
            print "#########################################"
            print "klampt.vis: Running multi-threaded dialog, waiting to complete..."
            _start_app_thread()
            while _windows[_current_window].mode == 'dialog':
                time.sleep(0.1)
            print "klampt.vis: ... dialog done."
            print "#########################################"
            return None
        else:
            print "#########################################"
            print "klampt.vis: Running single-threaded dialog"
            _in_vis_loop = True
            res = _run_app_thread()
            _in_vis_loop = False
            print "klampt.vis: ... dialog done."
            print "#########################################"
            return res

def _set_custom_ui(func):
    global _windows,_current_window,_vis_thread_running
    if len(_windows)==0:
        print "Making first window for custom ui"
        _windows.append(WindowInfo(_window_title,_frontend,_vis,None))
        _current_window = 0
    _windows[_current_window].custom_ui = func
    print "setting custom ui on window",_current_window
    return

def _onFrontendChange():
    global _windows,_frontend,_window_title,_current_window,_vis_thread_running
    if _current_window == None:
        return
    w = _windows[_current_window]
    w.doReload = True
    w.name = _window_title
    w.frontend = _frontend
    if w.glwindow:
        w.glwindow.reshape(_frontend.view.w,_frontend.view.h)
    if w.guidata and not _PyQtAvailable:
        w.guidata.frontend = _frontend
        _frontend.window = w.guidata.window

def _refreshDisplayLists(item):
    if isinstance(item,WorldModel):
        for i in xrange(item.numRobots()):
            _refreshDisplayLists(item.robot(i))
        for i in xrange(item.numRigidObjects()):
            _refreshDisplayLists(item.rigidObject(i))
        for i in xrange(item.numTerrains()):
            _refreshDisplayLists(item.terrain(i))
    elif isinstance(item,RobotModel):
        for i in xrange(item.numLinks()):
            _refreshDisplayLists(item.link(i))
    elif hasattr(item,'appearance'):
        item.appearance().refresh(False)

def _checkWindowCurrent(item):
    global _windows,_current_window,_world_to_window,_current_worlds
    if isinstance(item,int):
        if not all(w().index != item for w in _current_worlds):
            print "klampt.vis: item appears to be in a new world, but doesn't have a full WorldModel instance"
    if isinstance(item,WorldModel):
        #print "Worlds active in current window",_current_window,":",[w().index for w in _current_worlds]
        if all(item != w() for w in _current_worlds):
            #PyQt interface allows sharing display lists but GLUT does not.
            #refresh all worlds' display lists that will be shifted to the current window.
            for i,win in enumerate(_windows):
                #print "Window",i,"active worlds",[w().index for w in win.active_worlds]
                if any(item == w() for w in win.active_worlds):
                    if not _PyQtAvailable:
                        print "klampt.vis: world",item.index,"was shown in a different window, now refreshing display lists"
                        _refreshDisplayLists(item)
                    win.active_worlds.remove(weakref.ref(item))
            _current_worlds.append(weakref.ref(item))
            #print "klampt.vis: world added to the visualization's world (items:",[w().index for w in _current_worlds],")"
        #else:
        #    print "klampt.vis: world",item,"is already in the current window's world"
    elif hasattr(item,'world'):
        _checkWindowCurrent(item.world)

