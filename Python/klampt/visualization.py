"""Klamp't world visualization routines.  See demos/vistemplate.py for an
example of how to run this module.

Due to weird OpenGL behavior when opening/closing windows, you should only
run visualizations using the methods in this module, or manually running
glprogram / qtprogram code, but NOT BOTH.  Since the editors in the
resource module use this module, you should also use it if you plan to use
the resource editor.

Note: when changing the data shown by the window (e.g., modifying the
configurations of robots in a WorldModel) you must call lock() before
accessing the data and then call unlock() afterwards.

The main interface is as follows:

def setWindowTitle(title): sets the title of the visualization window.
def getWindowtitle(): returns the title of the visualization window
def kill(): kills all previously launched visualizations.  Afterwards, you may not
    be able to start new windows. Call this to cleanly quit.
def dialog(): pops up a dialog box (does not return to calling
    thread until closed).
def show(visible=True): shows or hides a visualization window run
    simultaneously with the calling thread.  To hide the window, pass False.
def spin(duration): shows the visualization window for the desired amount
    of time before returning, or until the user closes the window.
def shown(): returns true if the window is shown.
def clear(): clears the visualization world.
def dirty(item_name='all'): marks the given item as dirty and recreates the
    OpenGL display lists.
def lock(): locks the visualization world for editing.  The visualization will
    be paused until unlock() is called.
def unlock(): unlocks the visualization world.  Must only be called once
    after every lock().
def add(name,item,keepAppearance=False): adds an item to the visualization.
    name is a unique identifier.  If an item with the same name already exists,
    it will no longer be shown.  If keepAppearance=True, then the prior item's
    appearance will be kept, if a prior item exists.
def remove(name): removes an item from the visualization.
def setItemConfig(name,vector): sets the configuration of a named item.
def getItemConfig(name): returns the configuration of a named item.
def hide(name,hidden=True): hides/unhides an item.  The item is not removed,
    it just becomes invisible.
def hideLabel(name,hidden=True): hides/unhides an item's text label.
def animate(name,animation,speed=1.0): Sends an animation to the object.
    May be a Trajectory or a list of configurations.  Works with points,
    so3 elements, se3 elements, rigid objects, or robots.
def setAppearance(name,appearance): changes the Appearance of an item.
def revertAppearance(name): restores the Appearance of an item
def setAttribute(name,attribute,value): sets an attribute of the appearance
    of an item.
def setColor(name,r,g,b,a=1.0): changes the color of an item.
def setPlugin(plugin): plugin must be an instance of a GLPluginBase. 
    This plugin will now capture input from the visualization and can override
    any of the default behavior of the visualizer.

def setCustom(function,args): Advanced usage.  This function is called within
    the visualization thread, and is necessary to do anything with the opengl
    visualization including drawing.
def pauseAnimation(paused=True): Turns on/off animation.
def stepAnimation(amount): Moves forward the animation time by the given amount
    in seconds
def animationTime(newtime=None): Retrieves the current animation time
    If newtime != None, this sets a new animation time.
"""


from OpenGL.GL import *
from threading import Thread,Lock
from robotsim import *
import vectorops,so3,se3
import resource
import gldraw
import glcommon
import time
import signal
import coordinates
from trajectory import *

_baseClass = None
_globalLock = Lock()
_viz = None
_plugin = None
_window_title = "Klamp't visualizer"

def setPlugin(plugin):
    """Lets the user capture input via a glcommon.GLPluginBase class.
    Set plugin to None to disable plugins"""
    global _vis,_plugin
    if _vis==None:
        print "Visualization disabled"
        return
    _plugin = plugin
    if glcommon._PyQtAvailable:
        global _widget
        if _widget != None:
            #already running
            _widget.setPlugin(_vis)
            if plugin:
                _widget.pushPlugin(plugin)
    elif glcommon._GLUTAvailable:
        global _app
        if _app != None:
            _app.setPlugin(_vis)
            if plugin:
                _app.pushPlugin(plugin)

def dialog():
    global _vis
    if _vis==None:
        print "Visualization disabled"
        return
    _dialog()

def setWindowTitle(title):
    global _window_title
    _window_title = title

def getWindowTitle():
    global _window_title
    return _window_title

def kill():
    global _vis,_globalLock
    if _vis==None:
        print "Visualization disabled"
        return
    _kill()

def dirty(item_name='all'):
    global _vis
    if _vis==None:
        print "Visualization disabled"
        return
    _globalLock.acquire()
    if item_name == 'all':
        if (name,itemvis) in _vis.items.iteritems():
            itemvis.markChanged()
    else:
        _vis.items[item_name].markChanged()
    _globalLock.release()

def show(visible=True):
    global _vis,_globalLock
    _globalLock.acquire()
    if visible == False:
        _hide()
    else:
        if _vis==None:
            print "Visualization disabled"
            _globalLock.release()
            return
        else:
            _show()
    _globalLock.release()
    return

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
    global _globalLock
    _globalLock.release()

def shown():
    global _thread_running,_showwindow
    return _thread_running and _showwindow

def customRun(function,args=()):
    return _customRun(lambda : function(*args))

def clear():
    global _vis,_globalLock
    if _vis==None:
        return
    _globalLock.acquire()
    _vis.items = {}
    _globalLock.release()

def add(name,item,keepAppearance=False):
    global _vis,_globalLock
    if _vis==None:
        print "Visualization disabled"
        return
    _globalLock.acquire()
    if keepAppearance and name in _vis.items:
        _vis.items[name].setItem(item)
    else:
        #need to erase prior item visualizer
        if name in _vis.items:
            _vis.items[name].destroy()
        app = VisAppearance(item,name)
    _vis.items[name] = app
    _globalLock.release()

def animate(name,animation,speed=1.0):
    global _vis,_globalLock
    if _vis==None:
        print "Visualization disabled"
        return
    _globalLock.acquire()
    if hasattr(animation,'__iter__'):
        #a list of milestones -- loop through them with 1s delay
        print "visualization.animate(): Making a Trajectory with unit durations between",len(animation),"milestones"
        animation = Trajectory(range(len(animation)),animation)
    _vis.items[name].animation = animation
    _vis.items[name].animationStartTime = _vis.animationTime
    _vis.items[name].animationSpeed = speed
    _vis.items[name].markChanged()
    _globalLock.release()

def pauseAnimation(paused=True):
    global _vis,_globalLock
    if _vis==None:
        print "Visualization disabled"
        return
    _globalLock.acquire()
    _vis.animate = not paused
    _globalLock.release()

def stepAnimation(amount):
    global _vis,_globalLock
    if _vis==None:
        print "Visualization disabled"
        return
    _globalLock.acquire()
    _vis.animationTime += amount
    _globalLock.release()

def animationTime(newtime=None):
    global _vis,_globalLock
    if _vis==None:
        print "Visualization disabled"
        return 0
    if newtime != None:
        _globalLock.acquire()
        _vis.animationTime = newtime
        _globalLock.release()
    return _vis.animationTime

def remove(name):
    global _vis,_globalLock
    if _vis==None:
        return
    _globalLock.acquire()
    _vis.items[name].destroy()
    del _vis.items[name]
    _globalLock.release()

def hideLabel(name,hidden=True):
    global _vis,_globalLock
    if _vis==None:
        return
    _globalLock.acquire()
    _vis.items[name].attributes["text_hidden"] = hidden
    _vis.items[name].markChanged()
    _globalLock.release()

def hide(name,hidden=True):
    global _vis,_globalLock
    if _vis==None:
        return
    _globalLock.acquire()
    _vis.items[name].hidden = hidden
    _globalLock.release()

def setAppearance(name,appearance):
    global _vis,_globalLock
    if _vis==None:
        return
    _globalLock.acquire()
    _vis.items[name].useDefaultAppearance = False
    _vis.items[name].customAppearance = appearance
    _vis.items[name].markChanged()
    _globalLock.release()

def setAttribute(name,attr,value):
    global _vis,_globalLock
    if _vis==None:
        return
    _globalLock.acquire()
    _vis.items[name].attributes[attr] = value
    if value==None:
        del _vis.items[name].attributes[attr]
    _vis.items[name].markChanged()
    _globalLock.release()


def revertAppearance(name):
    global _vis,_globalLock
    if _vis==None:
        return
    _globalLock.acquire()
    _vis.items[name].useDefaultApperance = True
    _vis.items[name].markChanged()
    _globalLock.release()

def setColor(name,r,g,b,a=1.0):
    global _vis,_globalLock
    if _vis==None:
        return
    _globalLock.acquire()
    _vis.items[name].attributes["color"] = [r,g,b,a]
    _vis.items[name].useDefaultAppearance = False
    _vis.items[name].markChanged()
    _globalLock.release()




def getItemConfig(item):
    """Returns a flattened version of the configuration of the given item.
    Nearly all Klamp't objects are recognized, including RobotModel's,
    RigidObjectModel's, WorldModel's, and all variable types in the
    coordinates module.
    """
    if isinstance(item,RobotModel):
        return item.getConfig()
    elif isinstance(item,RigidObjectModel):
        R,t = item.getTransform()
        return R+t
    elif isinstance(item,WorldModel):
        res = []
        for i in range(item.numRobots()):
            res += getItemConfig(item.robot(i))
        for i in range(item.numRigidObjects()):
            res += getItemConfig(item.rigidObject(i))
        return res
    elif isinstance(item,coordinates.Point):
        return item.localCoordinates()
    elif isinstance(item,coordinates.Direction):
        return item.localCoordinates()
    elif isinstance(item,coordinates.Frame):
        R,t = item.relativeCoordinates()
        return R+t
    elif isinstance(item,coordinates.Group):
        res = []
        for n,f in item.frames.iteritems():
            res += getItemConfig(f)
        for n,p in item.points.iteritems():
            res += getItemConfig(p)
        for n,d in item.directions.iteritems():
            res += getItemConfig(d)
        for n,g in item.subgroups.iteritems():
            res += getItemConfig(g)
        return res
    elif hasattr(item,'__iter__'):
        if not hasattr(item[0],'__iter__'):
            return item[:]
        else:
            return sum(item,[])
    else:
        return []

def setItemConfig(item,vector):
    """Sets the configuration of the given item to the given vector.
    Nearly all Klamp't objects are recognized, including RobotModel's,
    RigidObjectModel's, WorldModel's, and all variable types in the
    coordinates module.
    """
    if isinstance(item,RobotModel):
        assert len(vector)==item.numLinks(),"Robot model config has %d DOFs"%(item.numLinks(),)
        item.setConfig(vector)
    elif isinstance(item,RigidObjectModel):
        assert len(vector)==12,"Rigid object model config has 12 DOFs"
        item.setTransform(vector[:9],vector[9:])
    elif isinstance(item,WorldModel):
        k=0
        for i in range(item.numRobots()):
            n = item.robot(i).numLinks()
            setItemConfig(item.robot(i),vector[k:k+n])
            k += n
        for i in range(item.numRigidObjects()):
            n = 12
            setItemConfig(item.robot(i),vector[k:k+n])
            k += n
        assert k == len(vector),"World model has %d DOFs"%(k,)
    elif isinstance(item,coordinates.Point):
        assert len(vector)==3,"Point config has 3 DOFs"
        item._localCoordinates = vector[:]
    elif isinstance(item,coordinates.Direction):
        assert len(vector)==3,"Direction config has 3 DOFs"
        item._localCoordinates = vector[:]
    elif isinstance(item,coordinates.Frame):
        assert len(vector)==12,"Frame config has 12 DOFs"
        item._relativeCoordinates = (vector[:9],vector[9:])
    elif isinstance(item,coordinates.Group):
        k = 0
        for n,f in item.frames.iteritems():
            setItemConfig(f,vector[k:k+12])
            k += 12
        for n,p in item.points.iteritems():
            setItemConfig(p,vector[k:k+3])
            k += 3
        for n,d in item.directions.iteritems():
            setItemConfig(d,vector[k:k+3])
            k += 3
        for n,g in item.subgroups.iteritems():
            raise NotImplementedError("TODO: set configuration of Group's subgroups")
    elif hasattr(item,'__iter__'):
        if not hasattr(item[0],'__iter__'):
            return vector[:]
        else:
            #split vector according to the sizes of the items
            res = []
            k = 0
            for v in item:
                n = len(v)
                res.append(vector[k:k+n])
                k += n
            return res
    return item

class CachedGLObject:
    """An object whose drawing is accelerated by means of a display list.
    The draw function may draw the object in the local frame, and the
    object may be transformed without having to recompile the display list.
    """
    def __init__(self):
        self.name = ""
        #OpenGL display list
        self.glDisplayList = None
        #marker for recursive calls
        self.makingDisplayList = False
        #parameters for display lists
        self.displayListParameters = None
        #dirty bit to indicate whether the display list should be recompiled
        self.changed = False

    def destroy(self):
        """Must be called to free up resources used by this object"""
        if self.glDisplayList != None:
            glDeleteLists(self.glDisplayList,1)
            self.glDisplayList = None

    def markChanged(self):
        """Marked by an outside source to indicate the object has changed and
        should be redrawn."""
        self.changed = True
    
    def draw(self,renderFunction,transform=None,parameters=None):
        """Given the function that actually makes OpenGL calls, this
        will draw the object.

        If parameters is given, the object's local appearance is assumed
        to be defined deterministically from these parameters.  The display
        list will be redrawn if the parameters change.
        """
        if self.makingDisplayList:
            renderFunction()
            return
        if self.glDisplayList == None or self.changed or parameters != self.displayListParameters:
            self.displayListParameters = parameters
            self.changed = False
            if self.glDisplayList == None:
                print "Generating new display list",self.name
                self.glDisplayList = glGenLists(1)
            print "Compiling display list",self.name
            if transform:
                glPushMatrix()
                glMultMatrixf(sum(zip(*se3.homogeneous(transform)),()))
            
            glNewList(self.glDisplayList,GL_COMPILE_AND_EXECUTE)
            self.makingDisplayList = True
            renderFunction()
            self.makingDisplayList = False
            glEndList()

            if transform:
                glPopMatrix()
        else:
            if transform:
                glPushMatrix()
                glMultMatrixf(sum(zip(*se3.homogeneous(transform)),()))
            glCallList(self.glDisplayList)
            if transform:
                glPopMatrix()

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
        #cached drawing
        self.displayCache = [CachedGLObject()]
        self.displayCache[0].name = name
        #temporary configuration of the item
        self.drawConfig = None
        self.setItem(item)
    def setItem(self,item):
        self.item = item
        self.subAppearances = {}
        #Parse out sub-items which can have their own appearance changed
        if isinstance(item,coordinates.Group):
            for n,f in item.frames.iteritems():
                self.subAppearances[("Frame",n)] = VisAppearance(f,n)
            for n,p in item.points.iteritems():
                self.subAppearances[("Point",n)] = VisAppearance(p,n)
            for n,d in item.directions.iteritems():
                self.subAppearances[("Direction",n)] = VisAppearance(d,n)
            for n,g in item.subgroups.iteritems():
                self.subAppearances[("Subgroup",n)] = VisAppearance(g,n)
        for (k,a) in self.subAppearances.iteritems():
            a.attributes = self.attributes
            
    def markChanged(self):
        for c in self.displayCache:
            c.markChanged()
        for (k,a) in self.subAppearances.iteritems():
            a.markChanged()

    def destroy(self):
        for c in self.displayCache:
            c.destroy()
        for (k,a) in self.subAppearances.iteritems():
            a.destroy()
        self.subAppearances = {}
        
    def drawText(self,text,point):
        """Draws the given text at the given point"""
        if self.attributes.get("text_hidden",False): return
        self.widget.addLabel(text,point,[0,0,0])

    def update(self,t):
        """Updates the configuration, if it's being animated"""
        if not self.animation:
            self.drawConfig = None
        else:
            u = self.animationSpeed*(t-self.animationStartTime)
            q = self.animation.eval(u,'loop')
            self.drawConfig = q
        for n,app in self.subAppearances.iteritems():
            app.update(t)

    def swapDrawConfig(self):
        """Given self.drawConfig!=None, swaps out the item's curren
        configuration  with self.drawConfig.  Used for animations"""
        if self.drawConfig: 
            try:
                newDrawConfig = getItemConfig(self.item)
                self.item = setItemConfig(self.item,self.drawConfig)
                self.drawConfig = newDrawConfig
            except Exception as e:
                print "Warning, exception thrown during animation update.  Probably have incorrect length of configuration"
                print e
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
                print "Changing appearance of",name
                item.appearance().set(self.customAppearance)
            elif "color" in self.attributes:
                print "Changing color of",name
                item.appearance().setColor(*self.attributes["color"])

        if hasattr(item,'drawGL'):
            item.drawGL()
        elif len(self.subAppearances)!=0:
            for n,app in self.subAppearances.iteritems():
                app.widget = self.widget
                app.draw(world)            
        elif isinstance(item,coordinates.Point):
            def drawRaw():
                glDisable(GL_DEPTH_TEST)
                glDisable(GL_LIGHTING)
                glEnable(GL_POINT_SMOOTH)
                glPointSize(self.attributes.get("size",5.0))
                glColor4f(*self.attributes.get("color",[0,0,0,1]))
                glBegin(GL_POINTS)
                glVertex3f(0,0,0)
                glEnd()
                glEnable(GL_DEPTH_TEST)
                #write name
            self.displayCache[0].draw(drawRaw,[so3.identity(),item.worldCoordinates()])
            if name != None:
                self.drawText(name,vectorops.add(item.worldCoordinates(),[0,0,-0.05]))
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
                self.drawText(name,vectorops.add(vectorops.add(item.frame().worldCoordinates()[1],item.worldCoordinates()),[0,0,-0.05]))
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
                    gldraw.hermite_curve(tlocal[1],v1,[0,0,0],v2,0.03)
                    #glDisable(GL_BLEND)
                glEnable(GL_DEPTH_TEST)

            #For some reason, cached drawing is causing OpenGL problems
            #when the frame is rapidly changing
            #self.displayCache[0].draw(drawRaw,transform=tp, parameters = tlocal)
            glPushMatrix()
            glMultMatrixf(sum(zip(*se3.homogeneous(tp)),()))
            drawRaw()
            glPopMatrix()
            #write name
            if name != None:
                self.drawText(name,se3.apply(t,[-0.05]*3))
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
        else:
            types = resource.objectToTypes(item,world)
            if isinstance(types,(list,tuple)):
                #ambiguous, still need to figure out what to draw
                validtypes = []
                for t in types:
                    if t == 'Config':
                        if world != None and len(t) == world.robot(0).numLinks():
                            validtypes.append(t)
                    elif t=='Vector3':
                        validtypes.append(t)
                    elif t=='RigidTransform':
                        validtypes.append(t)
                if len(validtypes) > 1:
                    print "Unable to draw item of ambiguous types",validtypes
                    return
                if len(validtypes) == 0:
                    print "Unable to draw any of types",types
                    return
                types = validtypes[0]
            if types == 'Config':
                if world:
                    robot = world.robot(0)
                    if not self.useDefaultAppearance:
                        oldAppearance = [robot.link(i).appearance().clone() for i in xrange(robot.numLinks())]
                        for i in xrange(robot.numLinks()):
                            robot.link(i).appearance().set(self.customAppearance)
                    oldconfig = robot.getConfig()
                    robot.setConfig(item)
                    robot.drawGL()
                    robot.setConfig(oldconfig)
                    if not self.useDefaultAppearance:
                        for (i,app) in enumerate(oldAppearance):
                            robot.link(i).appearance().set(app)
                else:
                    print "Unable to draw Config's without a world"
            elif types == 'Vector3':
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
                    self.drawText(name,vectorops.add(item,[0,0,-0.05]))
            elif types == 'RigidTransform':
                def drawRaw():
                    gldraw.xform_widget(se3.identity(),self.attributes.get("length",0.1),self.attributes.get("width",0.01))
                self.displayCache[0].draw(drawRaw,transform=item)
                if name != None:
                    self.drawText(name,se3.apply(item,[-0.05]*3))
            elif types == 'IKGoal':
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
                        self.displayCache.append(CachedGLObject())
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
                            glDisable(GL_DEPTH_TEST)
                            glDisable(GL_LIGHTING)
                            glColor3f(1,0.5,0)
                            gldraw.hermite_curve(p1,v1,p2,v2,0.03)
                            glEnable(GL_DEPTH_TEST)
                        self.displayCache[2].draw(drawConnection,transform=None,parameters = (p1,v1,p2,v2))
                        if name != None:
                            self.drawText(name,vectorops.add(wp,[-0.05]*3))
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
                            self.drawText(name,se3.apply(wp,[-0.05]*3))
            else:
                print "Unable to draw item of type",types

        #revert appearance
        if not self.useDefaultAppearance and hasattr(item,'appearance'):
            item.appearance().set(self.oldAppearance)


if glcommon._PyQtAvailable or glcommon._GLUTAvailable:

    if glcommon._PyQtAvailable:
        import qtprogram
        _BaseClass = qtprogram.GLPluginProgram
    else:
        import glprogram
        _BaseClass = glprogram.GLPluginProgram
           
    class VisualizationPlugin(glcommon.GLPluginBase):
        def __init__(self):
            glcommon.GLPluginBase.__init__(self)
            self.items = {}
            self.labels = []
            self.t = time.time()
            self.animate = True
            self.animationTime = 0

        def addLabel(self,text,point,color):
            for (p,textList,pcolor) in self.labels:
                if pcolor == color and vectorops.distance(p,point) < 0.1:
                    textList.append(text)
                    return
            self.labels.append((point,[text],color))

        def display(self):
            self.labels = []
            world = self.items.get('world',None)
            if world != None: world=world.item
            for (k,v) in self.items.iteritems():
                v.widget = self
                #do animation updates
                v.update(self.animationTime)
                v.swapDrawConfig()
                v.draw(world)
                v.swapDrawConfig()
                v.widget = None #allows garbage collector to delete these objects
            for (p,textlist,color) in self.labels:
                self.drawLabelRaw(p,textlist,color)

        def drawLabelRaw(self,point,textList,color):
            #assert not self.makingDisplayList,"drawText must be called outside of display list"
            for i,text in enumerate(textList):
                if i+1 < len(textList): text = text+","
                if glcommon._GLUTAvailable:
                    glRasterPos3f(*point)
                    glColor3f(0,0,0)
                    glDisable(GL_LIGHTING)
                    glDisable(GL_DEPTH_TEST)
                    gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_10,text)
                    glEnable(GL_DEPTH_TEST)
                elif glcommon._PyQtAvailable:
                    glColor3f(0,0,0)
                    glDisable(GL_DEPTH_TEST)
                    self.window.renderText(point[0],point[1],point[2],text)
                    glEnable(GL_DEPTH_TEST)
                point = vectorops.add(point,[0,0,-0.05])

        def clearDisplayLists(self):
            for i in self.items.itervalues():
                i.clearDisplayLists()

        def idlefunc(self):
            oldt = self.t
            self.t = time.time()
            if self.animate:
                self.animationTime += (self.t - oldt)
            return False

    _vis = VisualizationPlugin()        

if glcommon._PyQtAvailable:
    from PyQt4.QtCore import *
    from PyQt4.QtGui import *
    #Qt specific startup
    #need to set up a QDialog and an QApplication
    _widget = None
    #_vis.initWindow(self)
    _window = None
    _app = None
    _quit = False
    _thread_running = False
    _showdialog = False
    _showwindow = False
    _custom_run_method = None
    _custom_run_retval = None
    class _MyDialog(QDialog):
        def __init__(self):
            QDialog.__init__(self)
            global _vis,_widget,_window_title
            _widget.setMinimumSize(_vis.width,_vis.height)
            _widget.setMaximumSize(4000,4000)
            _widget.setSizePolicy(QSizePolicy(QSizePolicy.Maximum,QSizePolicy.Maximum))
            self.description = QLabel("Press OK to continue")
            self.layout = QVBoxLayout(self)
            self.layout.addWidget(_widget)
            self.layout.addWidget(self.description)
            self.buttons = QDialogButtonBox(QDialogButtonBox.Ok,Qt.Horizontal, self)
            self.buttons.accepted.connect(self.accept)
            self.layout.addWidget(self.buttons)
            self.setWindowTitle(_window_title)

    class _MyWindow(QMainWindow):
        def __init__(self):
            global _vis,_widget,_window_title
            QMainWindow.__init__(self)
            _widget.setMinimumSize(_vis.width,_vis.height)
            _widget.setMaximumSize(4000,4000)
            _widget.setSizePolicy(QSizePolicy(QSizePolicy.Maximum,QSizePolicy.Maximum))
            self.setCentralWidget(_widget)
            self.setWindowTitle(_window_title)
        def closeEvent(self,event):
            global _showwindow,_widget
            _showwindow = False
            _widget.setParent(None)
            print "Closing window"
            self.hide()

    def _run_app_thread():
        global _thread_running,_app,_vis,_widget,_window,_quit,_showdialog,_showwindow,_window_title
        global _custom_run_method,_custom_run_retval,_globalLock
        _thread_running = True
        #Do Qt setup
        _app = QApplication([_window_title])
        _widget = _BaseClass()
        _widget.initWindow()
        _widget.setPlugin(_vis)
        if _plugin is not None:
            _widget.pushPlugin(_plugin)
        #res = _app.exec_()
        res = None
        while not _quit:
            _globalLock.acquire()
            if _window:
                if _showwindow:
                    if not _window.isVisible():
                        #print "Show window, window is not None"
                        _window.show()
                else:
                    _widget.setParent(None)
                    _window.hide()
                    _showwindow = False
                    _window = None
            else:
                if _showwindow:
                    #print "Creating window"
                    _window=_MyWindow()
            
            if _showdialog:
                _window = None
                _dialog=_MyDialog()
                _globalLock.release()
                res = _dialog.exec_()
                _globalLock.acquire()
                _showdialog = False
                _widget.setParent(None)

            if _custom_run_method:
                res = _custom_run_method()
                _custom_run_retval = res
                _custom_run_method = None

            _app.processEvents()
            _globalLock.release()
            time.sleep(0.001)
        print "Visualization thread closing..."
        for (name,itemvis) in _vis.items.iteritems():
            itemvis.destroy()
        _thread_running = False
        return res
    
    def _kill():
        global _quit
        _quit = True
        while _thread_running:
            time.sleep(0.01)

    def _show():
        global _window,_showwindow,_thread_running
        if not _thread_running:
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            thread = Thread(target=_run_app_thread)
            thread.setDaemon(True)
            thread.start()
        _showwindow = True

    def _hide():
        global _window,_showwindow,_thread_running
        _showwindow = False

    def _dialog():
        global _app,_vis,_widget,_dialog_shown,_showwindow,_showdialog,_thread_running
        if not _thread_running:
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            thread = Thread(target=_run_app_thread)
            thread.start()
        old_show_window = _showwindow
        _showwindow = False
        _showdialog = True
        while _showdialog:
            time.sleep(0.1)
        _showwindow = old_show_window
        return

    def _customRun(func):
        global _app,_vis,_widget,_dialog_shown,_showwindow,_showdialog,_thread_running
        global _custom_run_method,_custom_run_retval
        if not _thread_running:
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            thread = Thread(target=_run_app_thread)
            thread.start()
        old_show_window = _showwindow
        _showwindow = False
        _custom_run_method = func
        while _custom_run_method:
            time.sleep(0.1)
        _showwindow = old_show_window
        return _custom_run_retval

elif glcommon._GLUTAvailable:
    from OpenGL.GLUT import *
    from glprogram import GLPluginProgram
    print "klampt.visualization: QT is not available, falling back to poorer"
    print "GLUT interface.  Returning to another GLUT thread will not work"
    print "properly."
    print ""
    _app = None
    _quit = False
    _thread_running = False
    _showdialog = False
    _showwindow = False
    _custom_run_method = None
    _custom_run_retval = None
    _old_glut_window = None
    class GLUTVisualization(GLPluginProgram):
        def __init__(self):
            global _vis,_window_title
            GLPluginProgram.__init__(self,_window_title)
            self.inDialog = False
        def initialize(self):
            GLPluginProgram.initialize(self)
        def display(self):
            global _globalLock
            _globalLock.acquire()
            GLPluginProgram.display(self)
            _globalLock.release()
        def display_screen(self):
            global _globalLock
            _globalLock.acquire()
            GLPluginProgram.display_screen(self)
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
                global _showwindow,_showdialog,_globalLock
                _globalLock.acquire()
                _showdialog = False
                if not _showwindow:
                    glutIconifyWindow()
                _globalLock.release()
            else:
                GLPluginProgram.keyboardfunc(self,c,x,y)

        def idlefunc(self):
            global _thread_running,_app,_vis,_quit,_showdialog,_showwindow,_old_glut_window
            global _custom_run_method,_custom_run_retval,_globalLock
            _globalLock.acquire()
            if _quit:
                if bool(glutLeaveMainLoop):
                    glutLeaveMainLoop()
                else:
                    print "Not compiled with freeglut, can't exit main loop safely. Press Ctrl+C instead"
                    raw_input()
            if not self.inDialog:
                if _showwindow:
                    glutShowWindow()
                elif _showdialog:
                    self.inDialog = True
                    glutShowWindow()
                else:
                    glutIconifyWindow()
                    if _old_glut_window is not None:
                        glutSetWindow(_old_glut_window)
                        #need to refresh all appearances of objects for changed opengl context
                        _vis.clearDisplayLists()

                if _custom_run_method:
                    pass
            _globalLock.release()
            GLPluginProgram.idlefunc(self)


    def _run_app_thread():
        global _thread_running,_app,_vis,_old_glut_window,_quit
        _thread_running = True
        if glutGetWindow() != 0:
            _old_glut_window = glutGetWindow()
            #need to refresh all appearances of objects for changed opengl context
            _vis.clearDisplayLists()
        _app = GLUTVisualization()
        #avoid calling _app.setPlugin, it calls refresh() which should not be
        #called until after GLUT is initialized on run()
        _app.setPlugin(_vis)
        if _plugin is not None:
            _app.pushPlugin(_plugin)
        _app.run()
        _app.setPlugin(None)
        print "Visualization thread closing..."
        for (name,itemvis) in _vis.items.iteritems():
            itemvis.destroy()
        _thread_running = False
        return
    
    def _kill():
        global _quit
        _quit = True
        while _thread_running:
            time.sleep(0.01)


    def _show():
        global _showwindow,_thread_running
        if not _thread_running:
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            thread = Thread(target=_run_app_thread)
            thread.start()
        _showwindow = True

    def _hide():
        global _showwindow,_thread_running
        _showwindow = False

    def _dialog():
        global _app,_vis,_dialog_shown,_showwindow,_showdialog,_thread_running
        if not _thread_running:
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            thread = Thread(target=_run_app_thread)
            thread.start()
        old_show_window = _showwindow
        _showwindow = False
        _showdialog = True
        while _showdialog:
            time.sleep(0.1)
        _showwindow = old_show_window
        return

    def _customRun(func):
        global _app,_vis,_dialog_shown,_showwindow,_showdialog,_thread_running
        global _custom_run_method,_custom_run_retval
        if not _thread_running:
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            thread = Thread(target=_run_app_thread)
            thread.start()
        old_show_window = _showwindow
        _showwindow = False
        _custom_run_method = func
        while _custom_run_method:
            time.sleep(0.1)
        _showwindow = old_show_window
        return _custom_run_retval
