"""Klamp't world visualization routines.  See demos/vistemplate.py for an
example of how to run this module.

WHAT DO WE WANT:
- Everything visualization-related is goverend by the klampt.vis module.
- Simple startup: create a GLPluginInterface and run a GUI using a single
  line of code, or simply add items to the visualization and run the GUI.
- Parallelize GUI and script code: launch a GUI, modify items of a world /
  simulation through a script, close the GUI or wait for the user to close
  the window.
- Window customizability with Qt.  Add the GLPluginInterface to an existing window
  and launch it.
- Add/remove/animate/configure all items in visualization using one-liners
- Multiple windows, shown either sequentially or simultaneously

CHALLENGES:
- Desire for window customizability breaks abstraction over GLUT / Qt.  Solution:
  None.  Possibly demand Qt?  Compatibility vs. complexity tradeoff.
- Parallel GUI / script execution has locking issues.  Solution: put 
  locks around everything visualization related, and require user to call
  vis.acquire() and vis.release().
- Multiple windows has the problem of GL display lists not passing from context
  to context. 
  Solutions:
  - Print warnings if the same world is passed to multiple simultaneous windows.
  - When windows are closed they become "dormant" and can wake up again:
    - Qt: simply hide the GL widget but keep it around to be passed to other windows.
    - GLUT: hide the window rather than closing it. 
- GLUT doesn't allow modal dialog boxes. 
  Solution: disable signals to non-modal windows?

NEW PROBLEM: 
- Qt seems to save display lists, so that each gl window has the same GL context?
  Cannot reuse widgets for new windows.
- GLUT does not, so things disappear when you create a new window.

Instructions:

- To add things to the default visualization:
  Call the VisualizationPlugin aliases (add, animate, setColor, etc)

- To show the visualization and quit when the user closes the window:
  run()

- To show the visualization and return when the user closes the window:
  dialog()
  ... do stuff afterwards ... 
  kill()

- To show the visualization and be able to run a script alongside it
  until the user closes the window:
  show()
  while shown():
      lock()
      ... do stuff ...
      [to exit the loop call show(False)]
      unlock()
      time.sleep(dt)
  ... do stuff afterwards ...
  kill()

- To run a window with a custom plugin (GLPluginInterface) and terminate on
  closure: 
  run(plugin)

- To show a dialog or parallel window
  setPlugin(plugin)
  ... then call  
  dialog()
  ... or
  show()
  ... do stuff afterwards ... 
  kill()

- To add a GLPluginInterface that just customizes a few things on top of
  the default visualization:
  pushPlugin(plugin)
  dialog()
  popPlugin()

- To run plugins side-by-side in the same window:
  setPlugin(plugin1)
  addPlugin(plugin2)  #this creates a new split-screen
  dialog()
  ... or
  show()
  ... do stuff afterwards ... 
  kill()

- To run a custom dialog in a QtWindow
  setPlugin([desired plugin or None for visualization])
  setParent(qt_window)
  dialog()
  ... or 
  show()
  ... do stuff afterwards ... 
  kill()

- To launch a second window after the first is closed: just call whatever you
  want again. Note: if show was previously called with a plugin and you wish to
  revert to the default visualization, you should call setPlugin(None) first to 
  restore the default.

- To create a separate window with a given plugin:
  w1 = createWindow()  #w1=0
  show()
  w2 = createWindow()  #w2=1
  setPlugin(plugin)
  dialog()
  #to restore commands to the original window
  setWindow(w1)
  while shown():
      ...
  kill()

Due to weird OpenGL behavior when opening/closing windows, you should only
run visualizations using the methods in this module.

Note: when changing the data shown by the window (e.g., modifying the
configurations of robots in a WorldModel) you must call lock() before
accessing the data and then call unlock() afterwards.

The main interface is as follows:

def createWindow(title=None): creates a new visualization window and returns an
    integer identifier.
def setWindow(id): sets the active window for all subsequent calls.  ID 0 is
    the default visualization window.
def getWindow(): gets the active window ID.
def setWindowTitle(title): sets the title of the visualization window.
def getWindowTitle(): returns the title of the visualization window
def setPlugin(plugin=None): sets the current plugin
def run([plugin]): pops up a dialog and then kills the program afterwards.
def kill(): kills all previously launched visualizations.  Afterwards, you may not
    be able to start new windows. Call this to cleanly quit.
def dialog(): pops up a dialog box (does not return to calling
    thread until closed).
def show(hidden=False): shows/hides a visualization window run in parallel with the calling script.
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
    _GLBackend instantiated for the current plugin.


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
    OpenGL display lists.
def remove(name): removes an item from the visualization.
def setItemConfig(name,vector): sets the configuration of a named item.
def getItemConfig(name): returns the configuration of a named item.
def hide(name,hidden=True): hides/unhides an item.  The item is not removed,
    it just becomes invisible.
def edit(name,doedit=True): turns on/off visual editing of some item.  Only points,
    transforms, coordinate.Point's, coordinate.Transform's, coordinate.Frame's,
    robots, and objects are accepted at this point.
def hideLabel(name,hidden=True): hides/unhides an item's text label.
def animate(name,animation,speed=1.0,endBehavior='loop'): Sends an animation to the
    object. May be a Trajectory or a list of configurations.  Works with points,
    so3 elements, se3 elements, rigid objects, or robots. 
    - speed: a modulator on the animation speed.  If the animation is a list of
      milestones, it is by default run at 1 milestone per second.
    - endBehavior: either 'loop' (animation repeats forever) or 'halt' (plays once).
def setAppearance(name,appearance): changes the Appearance of an item.
def revertAppearance(name): restores the Appearance of an item
def setAttribute(name,attribute,value): sets an attribute of the appearance
    of an item.  Typical attributes are color, size, length, width...
    TODO: document all accepted attributes.
def setColor(name,r,g,b,a=1.0): changes the color of an item.
def setPlugin(plugin): plugin must be an instance of a GLPluginBase. 
    This plugin will now capture input from the visualization and can override
    any of the default behavior of the visualizer.
def pauseAnimation(paused=True): Turns on/off animation.
def stepAnimation(amount): Moves forward the animation time by the given amount
    in seconds
def animationTime(newtime=None): Gets/sets the current animation time
    If newtime == None (default), this gets the animation time.
    If newtime != None, this sets a new animation time.

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
from glinit import _GLBackend,_PyQtAvailable,_GLUTAvailable
from glinterface import GLPluginInterface
from glprogram import GLPluginProgram
import glcommon
import time
import signal
from ..model import types
from ..model import config
from ..model import coordinates
from ..model.subrobot import SubRobotModel
from ..model.trajectory import *
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
        self.worlds = []
        self.active_worlds = []

_globalLock = RLock()
#the VisualizationPlugin instance of the currently active window
_vis = None
#the GLPluginProgram of the currently active window.  Accepts _vis as plugin or other user-defined plugins as well
_frontend = GLPluginProgram()
#the window title for the next created window
_window_title = "Klamp't visualizer"
#a list of world indices in the current window.  A world cannot be used in multiple simultaneous
#windows in GLUT.  If a world is reused with a different window, its display lists will be refreshed.
_current_worlds = []
#list of WindowInfo's
_windows = []
#the index of the current window
_current_window = None

def createWindow(name):
    """Creates a new window (and sets it active)."""
    global _globalLock,_frontend,_vis,_window_title,_current_worlds,_windows,_current_window
    _globalLock.acquire()
    if len(_windows) == 0:
        #save the defaults in window 0
        _windows.append(WindowInfo(_window_title,_frontend,_vis))    
        _windows[-1].worlds = _current_worlds
        _windows[-1].active_worlds = _current_worlds[:]
    #make a new window
    _window_title = name
    _frontend = GLPluginProgram()
    _vis = VisualizationPlugin()
    _windows.append(WindowInfo(_window_title,_frontend,_vis))
    _current_worlds = []
    id = len(_windows)-1
    _current_window = id
    _globalLock.release()
    return id

def setWindow(id):
    """Sets currently active window."""
    global _globalLock,_frontend,_vis,_window_title,_windows,_current_window
    if id == _current_window:
        return
    _globalLock.acquire()
    if len(_windows) == 0:
        #save the defaults in window 0
        _windows.append(WindowInfo(_window_title,_frontend,_vis)) 
        _windows[-1].worlds = _current_worlds
        _windows[-1].active_worlds = _current_worlds[:]
    assert id >= 0 and id < len(_windows)
    _window_title,_frontend,_vis,_current_worlds = _windows[id].name,_windows[id].frontend,_windows[id].vis,_windows[id].worlds
    #refresh all worlds' display lists
    for w in _current_worlds:
        if w not in _windows[id].active_worlds:
            print "vis.setWindow(): world",w,"becoming active in a different window"
            _refreshDisplayLists(w)
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
        if _vis==None:
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
        if _vis==None:
            raise RuntimeError("Visualization disabled")
        _frontend.setPlugin(_vis)
    _frontend.pushPlugin(plugin)
    _onFrontendChange()
    _globalLock.release()

def popPlugin():
    global _frontend
    _globalLock.acquire()
    _frontend.popPlugin()
    _onFrontendChange()
    _globalLock.release()

def addPlugin(plugin):
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
    """A blocking call to start a single window.  If plugin == None,
    the default visualization is used.  Otherwise, the plugin is used."""
    setPlugin(plugin)
    show()
    while shown():
      time.sleep(0.1)
    setPlugin(None)
    kill()

def dialog():
    _dialog()

def setWindowTitle(title):
    global _window_title
    _window_title = title
    _onFrontendChange()

def getWindowTitle():
    global _window_title
    return _window_title

def kill():
    global _vis,_globalLock
    if _vis==None:
        print "Visualization disabled"
        return
    _kill()

def show(hidden=False):
    _globalLock.acquire()
    if hidden:
        _hide()
    else:
        _show()
    _globalLock.release()

def spin(duration):
    show()
    t = 0
    while t < duration:
        if not shown(): break
        time.sleep(min(0.1,duration-t))
        t += 0.1
    return

def lock():
    global _globalLock
    _globalLock.acquire()

def unlock():
    global _globalLock,_windows
    for w in _windows:
        if w.glwindow:
            w.doRefresh = True
    _globalLock.release()

def shown():
    global _globalLock,_thread_running,_current_window
    _globalLock.acquire()
    res = (_thread_running and _current_window != None and _windows[_current_window].mode in ['shown','dialog'])
    _globalLock.release()
    return res

def customUI(func):
  global _globalLock
  _globalLock.acquire()
  _set_custom_ui(func)
  _globalLock.release()




######### CONVENIENCE ALIASES FOR VisualizationPlugin methods ###########
def clear():
    global _vis
    if _vis==None:
        return
    _vis.clear()

def add(name,item,keepAppearance=False):
    global _vis
    if _vis==None:
        print "Visualization disabled"
        return
    _checkWindowCurrent(item)
    _vis.add(name,item,keepAppearance)

def listItems(name=None,indent=0):
    global _vis
    if _vis==None:
        print "Visualization disabled"
        return
    _vis.listItems(name,indent)

def dirty(item_name='all'):
    global _vis
    if _vis==None:
        print "Visualization disabled"
        return
    _vis.dirty(item_name)

def animate(name,animation,speed=1.0,endBehavior='loop'):
    global _vis
    if _vis==None:
        print "Visualization disabled"
        return
    _vis.animate(name,animation,speed,endBehavior)

def pauseAnimation(paused=True):
    global _vis
    if _vis==None:
        print "Visualization disabled"
        return
    _vis.pauseAnimation(paused)

def stepAnimation(amount):
    global _vis
    if _vis==None:
        print "Visualization disabled"
        return
    _vis.stepAnimation(amount)

def animationTime(newtime=None):
    global _vis
    if _vis==None:
        print "Visualization disabled"
        return 0
    return _vis.animationTime(newtime)

def remove(name):
    global _vis
    if _vis==None:
        return
    return _vis.remove(name)

def getItemConfig(name):
    global _vis
    if _vis==None:
        return None
    return _vis.getItemConfig(name)

def setItemConfig(name,value):
    global _vis
    if _vis==None:
        return
    return _vis.setItemConfig(name,value)

def hideLabel(name,hidden=True):
    global _vis
    if _vis==None:
        return
    return _vis.hideLabel(name,hidden)

def hide(name,hidden=True):
    global _vis
    if _vis==None:
        return
    _vis.hide(name,hidden)

def edit(name,doedit=True):
    global _vis
    if _vis==None:
        return
    _vis.edit(name,doedit)

def setAppearance(name,appearance):
    global _vis
    if _vis==None:
        return
    _vis.setAppearance(name,appearance)

def setAttribute(name,attr,value):
    global _vis
    if _vis==None:
        return
    _vis.setAttribute(name,attr,value)

def revertAppearance(name):
    global _vis
    if _vis==None:
        return
    _vis.revertAppearance(name)

def setColor(name,r,g,b,a=1.0):
    global _vis
    if _vis==None:
        return
    _vis.setColor(name,r,g,b,a)





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
            return
        if len(validtypes) == 0:
            print "Unable to draw any of types",itypes
            return
        return validtypes[0]
    return itypes

class VisAppearance:
    def __init__(self,item,name = None):
        self.name = name
        self.hidden = False
        self.useDefaultAppearance = True
        self.customAppearance = None
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

    def update(self,t):
        """Updates the configuration, if it's being animated"""
        if not self.animation:
            self.drawConfig = None
        else:
            u = self.animationSpeed*(t-self.animationStartTime)
            q = self.animation.eval(u,self.animationEndBehavior)
            self.drawConfig = q
        for n,app in self.subAppearances.iteritems():
            app.update(t)

    def swapDrawConfig(self):
        """Given self.drawConfig!=None, swaps out the item's curren
        configuration  with self.drawConfig.  Used for animations"""
        if self.drawConfig: 
            try:
                newDrawConfig = config.getConfig(self.item)
                self.item = config.setConfig(self.item,self.drawConfig)
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
       
        item = self.item
        name = self.name
        #set appearance
        if not self.useDefaultAppearance and hasattr(item,'appearance'):
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
        elif isinstance(item,Trajectory):
            doDraw = False
            centroid = None
            if isinstance(item,RobotTrajectory):
                ees = self.attributes.get("endeffectors",[-1])
                if world:
                    doDraw = (len(ees) > 0)
                    robot = world.robot(0)
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
            if doDraw:
                def drawRaw():
                    pointTrajectories = []
                    if isinstance(item,RobotTrajectory):
                        robot = world.robot(0)
                        ees = self.attributes.get("endeffectors",[-1])
                        for i,ee in enumerate(ees):
                            if ee < 0: ees[i] = robot.numLinks()-1
                        if world:
                            for ee in ees:
                                pointTrajectories.append([])
                            for m in item.milestones:
                                robot.setConfig(m)
                                for ee,eetraj in zip(ees,pointTrajectories):
                                    eetraj.append(robot.link(ee).getTransform()[1])
                    elif isinstance(item,SE3Trajectory):
                        pointTrajectories.append([])
                        for m in item.milestones:
                            pointTrajectories[-1].append(m[9:])
                    else:
                        if len(item.milestones[0]) == 3:
                            #R3 trajectory
                            pointTrajectories.append(item.milestones)
                        elif len(item.milestones[0]) == 2:
                            #R2 trajectory
                            pointTrajectories.append([v + [0.0] for v in item.milestones])
                    glDisable(GL_LIGHTING)
                    glLineWidth(self.attributes.get("width",3))
                    glColor4f(*self.attributes.get("color",[1,0.5,0,1]))
                    for traj in pointTrajectories:
                        if len(traj) == 1:
                            glBegin(GL_POINTS)
                            glVertex3f(*traj[0])
                            glEnd()
                        if len(traj) >= 2:
                            glBegin(GL_LINE_STRIP)
                            for p in traj:
                                glVertex3f(*p)
                            glEnd()
                    glLineWidth(1.0)
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
        else:
            try:
                itypes = objectToVisType(item,world)
            except:
                print "Unknown object type",item.__class__.__name__
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
                    print "Unable to draw Config tiems without a world"
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
                    gldraw.xform_widget(se3.identity(),self.attributes.get("length",0.1),self.attributes.get("width",0.01))
                self.displayCache[0].draw(drawRaw,transform=item)
                if name != None:
                    self.drawText(name,item[1])
            elif itypes == 'IKGoal':
                if hasattr(item,'robot'):
                    #need this to be built with a robot element.
                    #Otherwise, can't determine the correct transforms
                    robot = item.robot
                elif world:
                    if world.numRobots() >= 1:
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
            else:
                print "Unable to draw item of type",itypes

        #revert appearance
        if not self.useDefaultAppearance and hasattr(item,'appearance'):
            item.appearance().set(self.oldAppearance)

    def getSubItem(self,path):
        if len(path) == 0: return self
        for k,v in self.subAppearances.iteritems():
            if v.name == path[0]:
                try:
                    return v.getSubItem(path[1:])
                except ValueError,e:
                    raise ValueError("Invalid sub-path specified "+str(path)+" at "+str(e))
        raise ValueError("Invalid sub-item specified "+path[0])

    def make_editor(self):
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
            elif isinstance(self.item,(list,tuple)):
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
            #do animation updates
            v.update(self.currentAnimationTime)
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
            self.draw_text(point,text,size=10)
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
        _globalLock.release()
        return False

    def getItem(self,item_name):
        if isinstance(item_name,(list,tuple)):
            components = item_name
            if len(components)==1: 
                return self.getItem(components[0])
            if components[0] not in self.items:
                raise ValueError("Invalid top-level item specified: "+item_name)
            return self.items[components[0]].getSubItem(components[1:])
        if item_name in self.items:
            return self.items[item_name]

    def dirty(self,item_name='all'):
        global _globalLock
        _globalLock.acquire()
        if item_name == 'all':
            if (name,itemvis) in self.items.iteritems():
                itemvis.markChanged()
        else:
            self.getItem(item_name).markChanged()
        _globalLock.release()

    def clear(self):
        global _globalLock
        _globalLock.acquire()
        for (name,itemvis) in self.items.iteritems():
            itemvis.destroy()
        self.items = {}
        _globalLock.release()

    def listItems(self,root=None,indent=0):
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

    def animate(self,name,animation,speed=1.0,endBehavior='loop'):
        global _globalLock
        _globalLock.acquire()
        if hasattr(animation,'__iter__'):
            #a list of milestones -- loop through them with 1s delay
            print "visualization.animate(): Making a Trajectory with unit durations between",len(animation),"milestones"
            animation = Trajectory(range(len(animation)),animation)
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
        if isinstance(item.item,(list,tuple)):
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
            obj.make_editor()
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



_vis = VisualizationPlugin() 
_frontend.setPlugin(_vis)

#signals to visualization thread
_quit = False
_thread_running = False

if _PyQtAvailable:
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
            self.glwidget.setMinimumSize(640,480)
            self.glwidget.setMaximumSize(4000,4000)
            self.glwidget.setSizePolicy(QSizePolicy(QSizePolicy.Maximum,QSizePolicy.Maximum))
            self.setCentralWidget(self.glwidget)
            self.setWindowTitle(windowinfo.name)
        def closeEvent(self,event):
            global _globalLock
            _globalLock.acquire()
            self.windowinfo.glwindow.hide()
            self.windowinfo.mode = 'hidden'
            self.windowinfo.glwindow.idlesleep()
            print "#########################################"
            print "klampt.vis: Window close"
            print "#########################################"
            _globalLock.release()

    alldlgs = []
    def _run_app_thread():
        global _thread_running,_vis,_widget,_window,_quit,_showdialog,_showwindow,_window_title
        global alldlgs
        _thread_running = True

        _GLBackend.initialize("Klamp't visualization")
        
        res = None
        while not _quit:
            _globalLock.acquire()
            for i,w in enumerate(_windows):
                if w.glwindow == None and w.mode != 'hidden':
                    w.glwindow = _GLBackend.createWindow(w.name)
                    w.glwindow.setProgram(w.frontend)
                    w.glwindow.setParent(None)
                    w.glwindow.refresh()
                if w.doRefresh:
                    w.glwindow.updateGL()
                    w.doRefresh = False
                if w.mode == 'dialog':
                    print "#########################################"
                    print "klampt.vis: Dialog on window",i
                    print "#########################################"
                    w.glwindow.show()
                    w.glwindow.idlesleep(0)
                    w.glwindow.refresh()
                    if w.custom_ui == None:
                        dlg = _MyDialog(w)
                    else:
                        dlg = w.custom_ui(w.glwindow)
                    #need to cache the bastards to avoid deleting the GL object. Not sure why it's being kept around.
                    #alldlgs.append(dlg)
                    #here's the crash -- above line deleted the old dialog, which for some reason kills the widget
                    w.glwindow.refresh()
                    _globalLock.release()
                    res = dlg.exec_()
                    _globalLock.acquire()
                    print "#########################################"
                    print "klampt.vis: Dialog done on window",i
                    print "#########################################"
                    w.glwindow.hide()
                    w.glwindow.setParent(None)
                    w.glwindow.idlesleep()
                    w.mode = 'hidden'
                if w.mode == 'shown' and w.guidata == None:
                    print "#########################################"
                    print "klampt.vis: Making window",i
                    print "#########################################"
                    if w.custom_ui == None:
                        w.guidata = _MyWindow(w)
                    else:
                        w.guidata = w.custom_ui(w.glwindow)
                    w.glwindow.show()
                    w.glwindow.idlesleep(0)
                if w.mode == 'shown' and not w.guidata.isVisible():
                    print "#########################################"
                    print "klampt.vis: Showing window",i
                    print "#########################################"
                    w.glwindow.show()
                    w.glwindow.setParent(w.guidata)
                    w.glwindow.idlesleep(0)
                    w.guidata.show()
                if w.mode == 'hidden' and w.guidata != None:
                    if w.guidata.isVisible():
                        print "#########################################"
                        print "klampt.vis: Hiding window",i
                        print "#########################################"
                        w.glwindow.setParent(None)
                        w.glwindow.idlesleep()
                        w.glwindow.hide()
                        w.guidata.hide()
                    #prevent deleting the GL window
                    w.glwindow.setParent(None)
                    w.guidata = None
            _globalLock.release()
            _GLBackend.app.processEvents()
            time.sleep(0.001)
        print "Visualization thread closing..."
        for w in _windows:
            w.vis.clear()
            if w.glwindow:
                w.glwindow.close()
        _thread_running = False
        return res


elif _GLUTAvailable:
    print "klampt.visualization: QT is not available, falling back to poorer"
    print "GLUT interface.  Returning to another GLUT thread will not work"
    print "properly."
    print ""
    
    class GLUTHijacker(GLPluginInterface):
        def __init__(self,windowinfo):
            glinterface.GLPluginInterface.__init__(self)
            self.windowinfo = windowinfo
            self.name = windowinfo.name
            self.frontend = windowinfo.frontend
            self.inDialog = False
        def initialize(self):
            if not self.frontend.initialize(self): return False
            GLPluginProgram.initialize(self)
            return True
        def display(self):
            global _globalLock
            _globalLock.acquire()
            self.frontend.display(self)
            _globalLock.release()
            return True
        def display_screen(self):
            global _globalLock
            _globalLock.acquire()
            self.frontend.display_screen(self)
            glColor3f(1,1,1)
            glRasterPos(20,50)
            gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_18,"(Do not close this window except to quit)")
            if self.inDialog:
                glColor3f(1,1,0)
                glRasterPos(20,80)
                gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_18,"In Dialog mode. Press 'q' to return to normal mode")
            _globalLock.release()
        def keyboardfunc(self,c,x,y):
            if self.inDialog and c=='q':
                print "Q pressed, hiding dialog"
                self.inDialog = False
                global _globalLock
                _globalLock.acquire()
                self.windowinfo.mode = 'hidden'
                glutIconifyWindow()
                _globalLock.release()
            else:
                GLPluginProgram.keyboardfunc(self,c,x,y)

        def idlefunc(self):
            global _quit,_showdialog
            global _globalLock
            _globalLock.acquire()
            if _quit:
                if bool(glutLeaveMainLoop):
                    glutLeaveMainLoop()
                else:
                    print "Not compiled with freeglut, can't exit main loop safely. Press Ctrl+C instead"
                    raw_input()
            if not self.inDialog:
                if self.windowinfo.mode == 'shown':
                    glutShowWindow()
                elif self.windowinfo.mode == 'dialog':
                    self.inDialog = True
                    glutShowWindow()
                else:
                    glutIconifyWindow()
            _globalLock.release()
            GLPluginProgram.idlefunc(self)


    def _run_app_thread():
        global _thread_running,_vis,_old_glut_window,_quit
        _thread_running = True
        _GLBackend.initialize("Klamp't visualization")
        _GLBackend.addPlugin(GLUTHijacker(windows[0]))
        _GLBackend.run()
        print "Visualization thread closing..."
        for w in _windows:
            w.vis.clear()
        _thread_running = False
        return
    
def _kill():
    global _quit
    _quit = True
    while _thread_running:
        time.sleep(0.01)
    _quit = False

def _show():
    global _windows,_current_window,_thread_running
    if len(_windows)==0:
        _windows.append(WindowInfo(_window_title,_frontend,_vis)) 
        _current_window = 0
    if not _thread_running:
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        thread = Thread(target=_run_app_thread)
        thread.setDaemon(True)
        thread.start()
        time.sleep(0.1)
    _windows[_current_window].mode = 'shown'

def _hide():
    global _windows,_current_window,_thread_running
    if _current_window == None:
        return
    _windows[_current_window].mode = 'hidden'

def _dialog():
    global __windows,_current_window,_thread_running
    if len(_windows)==0:
        _windows.append(WindowInfo(_window_title,_frontend,_vis,None))
        _current_window = 0
    if not _thread_running:
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        thread = Thread(target=_run_app_thread)
        thread.setDaemon(True)
        thread.start()
        #time.sleep(0.1)
    assert _windows[_current_window].mode == 'hidden'
    _windows[_current_window].mode = 'dialog'
    while _windows[_current_window].mode == 'dialog':
        time.sleep(0.1)
    return

def _set_custom_ui(func):
    global _windows,_current_window,_thread_running
    if len(_windows)==0:
        _windows.append(WindowInfo(_window_title,_frontend,_vis,None))
        _current_window = 0
    
    _windows[_current_window].custom_ui = func
    return

def _onFrontendChange():
    global _windows,_frontend,_window_title,_current_window,_thread_running
    if _current_window == None:
        return
    _windows[_current_window].name = _window_title
    _windows[_current_window].frontend = _frontend
    if _windows[_current_window].glwindow:
        _windows[_current_window].glwindow.reshape(_frontend.view.w,_frontend.view.h)
        _windows[_current_window].glwindow.setProgram(_frontend)
    if _windows[_current_window].guidata:
        _windows[_current_window].guidata.setWindowTitle(_window_title)
        _windows[_current_window].guidata.glwidget = _windows[_current_window].glwindow
        _windows[_current_window].guidata.setCentralWidget(_windows[_current_window].glwindow)

def _refreshDisplayLists(item):
    if isinstance(item,int):
        w = WorldModel()
        w.index = item
        _refreshDisplayLists(w)
        #prevents deletion
        w.index = -1
    elif isinstance(item,WorldModel):
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
    global _windows,_current_window,_world_to_window
    if isinstance(item,int):
        if item not in _current_worlds:
            for w in _windows:
                if item in w.active_worlds:
                    print "klampt.vis: warning, world",item,"was shown in a different window, now refreshing display lists"
                    _refreshDisplayLists(item)
                    w.active_worlds.remove(item)
            _current_worlds.append(item)
            print "klampt.vis: world added to the visualization's world (items:",_current_worlds,")"
        else:
          print "klampt.vis: world",item,"is already in the visualization's world"
    elif isinstance(item,WorldModel):
        _checkWindowCurrent(item.index)
    elif hasattr(item,'world'):
        _checkWindowCurrent(item.world)

