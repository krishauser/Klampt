"""Klamp't world visualization routines.

Due to weird OpenGL behavior when opening/closing windows, you should only
run visualizations using the methods in this module, or manually running
glprogram / qtprogram code, NOT BOTH.  Since the editors in the
resource module use this module, you should also use it if you plan to use
the resource editor.

Note: when changing the data shown by the window (e.g., a WorldModel)
you must call lock() before accessing the data and then call unlock() afterwards.

The main interface is as follows:

def setWindowTitle(title): sets the title of the visualization window.
def getWindowtitle(): returns the title of the visualization window
def dialog(): pops up a dialog box (does not return to calling
    thread until closed).
def show(visible=True): shows or hides a visualization window run
    simultaneously with the calling thread.  To hide the window, pass False.
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
def hide(name,hidden=True): hides/unhides an item.  The item is not removed,
    it just becomes invisible.
def hideLabel(name,hidden=True): hides/unhides an item's text label.
def setAnimation(name,animation,speed=1.0): TODO: implement me. Should send
    an animation to the object.  May be a Trajectory or a list of
    configurations
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
"""


from OpenGL.GL import *
from threading import Thread,Lock
import vectorops,so3,se3
import resource
import gldraw
import glcommon
import time
import signal
import coordinates

_baseClass = None
_globalLock = Lock()
_viz = None
_window_title = "Klamp't visualizer"

def setPlugin(plugin):
    """Lets the user capture input via a glcommon.GLPluginBase class.
    Set plugin to None to disable plugins"""
    global _vis
    if _vis==None:
        print "Visualization disabled"
        return
    _vis.setPlugin(plugin)

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
    if visible == False:
        _hide()
        return
    if _vis==None:
        print "Visualization disabled"
        return
    _show()

def lock():
    global _globalLock
    _globalLock.acquire()

def unlock():
    global _globalLock
    _globalLock.release()

def shown():
    global _showwindow
    return _showwindow

def customRun(function,args=()):
    return _customRun(lambda : function(*args))

def clear():
    global _vis,_globalLock
    if _vis==None:
        return
    _vis.items = {}

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

def setAnimation(name,animation,speed):
    global _vis,_globalLock
    if _vis==None:
        print "Visualization disabled"
        return
    _globalLock.acquire()
    _vis.items[name].animation = animation
    _vis.items[name].animationSpeed = animationSpeed
    _vis.items[name].markChanged()
    _globalLock.release()

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





class VisAppearance:
    def __init__(self,item,name = None):
        self.name = name
        self.hidden = False
        self.useDefaultAppearance = True
        self.customAppearance = None
        #For group items, this allows you to customize appearance of sub-items
        self.subAppearances = {}
        self.animation = None
        self.animationSpeed = 1.0
        self.attributes = {}
        #used for Qt text rendering
        self.widget = None
        #OpenGL display list
        self.glDisplayList = None
        #marker for recursive calls
        self.makingDisplayList = False
        #parameters for display lists
        self.displayListParameters = None
        #dirty bit to indicate whether the display list should be recompiled
        self.changed = False
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
        self.changed = True
        for (k,a) in self.subAppearances.iteritems():
            a.changed = True

    def destroy(self):
        if self.glDisplayList != None:
            glDeleteLists(self.glDisplayList,1)
            self.glDisplayList = None
        for (k,a) in self.subAppearances.iteritems():
            a.destroy()
        self.subAppearances = {}
    def drawText(self,text,point):
        """Draws the given text at the given point"""
        if self.attributes.get("text_hidden",False): return
        self.widget.addLabel(text,point,[1,1,1])

    def cachedDraw(self,drawFunction,transform=None,parameters=None):
        if self.makingDisplayList:
            drawFunction()
            return
        if self.glDisplayList == None or self.changed or parameters != self.displayListParameters:
            self.displayListParameters = parameters
            if self.glDisplayList == None:
                print "Generating new display list",self.name
                self.glDisplayList = glGenLists(1)
            print "Compiling display list",self.name
            glNewList(self.glDisplayList,GL_COMPILE_AND_EXECUTE)
            self.makingDisplayList = True
            drawFunction()
            self.makingDisplayList = False
            glEndList()
        else:
            if transform:
                glPushMatrix()
                glMultMatrixf(sum(zip(*se3.homogeneous(transform)),()))
            glCallList(self.glDisplayList)
            if transform:
                glPopMatrix()

    def draw(self,world=None):
        """Draws the specified item in the specified world.  If name
        is given and text_hidden != False, then the name of the item is
        shown."""
        if self.hidden: return
        item = self.item
        name = self.name
        #set appearance
        if not self.useDefaultAppearance and hasattr(item,'appearance'):
            oldAppearance = item.appearance().clone()
            if self.customAppearance != None:
                item.appearance().set(self.customAppearance)
            elif "color" in self.attributes:
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
                glColor4f(*self.attributes.get("color",[1,1,1,1]))
                glBegin(GL_POINTS)
                glVertex3f(0,0,0)
                glEnd()
                glEnable(GL_DEPTH_TEST)
                #write name
            self.cachedDraw(drawRaw,[so3.identity(),item.worldCoordinates()])
            if name != None:
                self.drawText(name,vectorops.add(item.worldCoordinates(),[0,0,-0.05]))
        elif isinstance(item,coordinates.Direction):
            def drawRaw():
                glDisable(GL_LIGHTING)
                glDisable(GL_DEPTH_TEST)
                L = self.attributes.get("length",0.15)
                source = [0,0,0]
                glColor4f(*self.attributes.get("color",[1,1,1,1]))
                glBegin(GL_LINES)
                glVertex3f(*source)
                glVertex3f(*vectorops.mul(item.localCoordinates(),L))
                glEnd()
                glEnable(GL_DEPTH_TEST)
                #write name
            self.cachedDraw(drawRaw,item.frame().worldCoordinates(),parameters = item.localCoordinates())
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
                    glEnable(GL_BLEND)
                    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
                    #glColor4f(1,1,0,0.5)
                    glColor3f(1,1,0)
                    gldraw.hermite_curve(tlocal[1],v1,[0,0,0],v2,0.03)
                    glDisable(GL_BLEND)
                glEnable(GL_DEPTH_TEST)
                #write name
            self.cachedDraw(drawRaw,transform=tp, parameters = tlocal)
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
            self.cachedDraw(drawRaw,transform=None,parameters = (t1,t2))
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
                    robot.setConfig(item)
                    for i in xrange(robot.numLinks()):
                        robot.link(i).drawGL()
                    if not self.useDefaultAppearance:
                        for i,app in enumerate(oldAppearance):
                            robot.link(i).appearance().set(app)
                else:
                    print "Unable to draw Config's without a world"
            elif types == 'Vector3':
                def drawRaw():
                    glDisable(GL_LIGHTING)
                    glEnable(GL_POINT_SMOOTH)
                    glPointSize(self.attributes.get("size",5.0))
                    glColor4f(*self.attributes.get("color",[1,1,1,1]))
                    glBegin(GL_POINTS)
                    glVertex3f(0,0,0)
                    glEnd()
                self.cachedDraw(drawRaw,[so3.identity(),item])
                if name != None:
                    self.drawText(name,vectorops.add(item,[0,0,-0.05]))
            elif types == 'RigidTransform':
                def drawRaw():
                    gldraw.xform_widget(se3.identity(),self.attributes.get("length",0.1),self.attributes.get("width",0.01))
                self.cachedDraw(drawRaw,transform=item)
                if name != None:
                    self.drawText(name,se3.apply(item,[-0.05]*3))
            else:
                print "Unable to draw item of type",types

        #revert appearance
        if not self.useDefaultAppearance and hasattr(item,'appearance'):
            item.appearance().set(oldAppearance)


if glcommon._PyQtAvailable or glcommon._GLUTAvailable:

    if glcommon._PyQtAvailable:
        import qtprogram
        _baseClass = qtprogram.GLPluginProgram
    else:
        import glprogram
        _baseClass = glprogram.GLPluginProgram
           
    class VisualizationProgram(_baseClass):
        def __init__(self):
            _baseClass.__init__(self,"Klamp't Visualizer")
            self.items = {}
            self.labels = []

        def addLabel(self,text,point,color):
            for (p,textList,pcolor) in self.labels:
                if pcolor == color and vectorops.distance(p,point) < 0.1:
                    textList.append(text)
                    return
            self.labels.append((point,[text],color))

        def display(self):
            _globalLock.acquire()
            self.labels = []
            world = self.items.get('world',None)
            if world != None: world=world.item
            for (k,v) in self.items.iteritems():
                v.widget = self
                v.draw(world)
                v.widget = None #allows garbage collector to delete these objects
            for (p,textlist,color) in self.labels:
                self.drawLabelRaw(p,textlist,color)
            _baseClass.display(self)
            _globalLock.release()
        def drawLabelRaw(self,point,textList,color):
            #assert not self.makingDisplayList,"drawText must be called outside of display list"
            for i,text in enumerate(textList):
                if i+1 < len(textList): text = text+","
                if glcommon._GLUTAvailable:
                    glRasterPos3f(*point)
                    glColor3f(1,1,1)
                    glDisable(GL_LIGHTING)
                    gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_10,text)
                elif glcommon._PyQtAvailable:
                    glColor3f(1,1,1)
                    glDisable(GL_DEPTH_TEST)
                    self.renderText(point[0],point[1],point[2],text)
                    glEnable(GL_DEPTH_TEST)
                point = vectorops.add(point,[0,0,-0.05])


        def idle(self):
            pass
        
    _vis = VisualizationProgram()
    #_vis.initWindow(self)

if glcommon._PyQtAvailable:
    from PyQt4.QtCore import *
    from PyQt4.QtGui import *
    #Qt specific startup
    #need to set up a QDialog and an QApplication
    _window = None
    _app = None
    _quit = False
    _showdialog = False
    _showwindow = False
    _thread_running = False
    _custom_run_method = None
    _custom_run_retval = None
    class _MyDialog(QDialog):
        def __init__(self):
            QDialog.__init__(self)
            global _vis,_window_title
            _vis.setMinimumSize(_vis.width,_vis.height)
            _vis.setMaximumSize(4000,4000)
            _vis.setSizePolicy(QSizePolicy(QSizePolicy.Maximum))
            self.description = QLabel("Press OK to continue")
            self.layout = QVBoxLayout(self)
            self.layout.addWidget(_vis)
            self.layout.addWidget(self.description)
            self.buttons = QDialogButtonBox(QDialogButtonBox.Ok,Qt.Horizontal, self)
            self.buttons.accepted.connect(self.accept)
            self.layout.addWidget(self.buttons)
            self.setWindowTitle(_window_title)

    class _MyWindow(QMainWindow):
        def __init__(self):
            global _vis,_window_title
            QMainWindow.__init__(self)
            _vis.setMinimumSize(_vis.width,_vis.height)
            _vis.setMaximumSize(4000,4000)
            _vis.setSizePolicy(QSizePolicy(QSizePolicy.Maximum))
            self.setCentralWidget(_vis)
            self.setWindowTitle(_window_title)
        def closeEvent(self,event):
            global _showwindow,_vis
            _showwindow = False
            _vis.setParent(None)
            self.hide()

    def _run_app_thread():
        global _thread_running,_app,_vis,_window,_quit,_showdialog,_showwindow
        global _custom_run_method,_custom_run_retval
        _thread_running = True
        #Do Qt setup
        _app = QApplication(["Klamp't visualizer"])
        _vis.initWindow()
        #res = _app.exec_()
        res = None
        while not _quit:
            if _window:
                if _showwindow:
                    _window.show()
                else:
                    _window.hide()
                    _showwindow = False
                    _window = None
                    _vis.setParent(None)
            else:
                if _showwindow:
                    _window=_MyWindow()
            
            if _showdialog:
                _window = None
                _dialog=_MyDialog()
                res = _dialog.exec_()
                _showdialog = False
                _vis.setParent(None)

            if _custom_run_method:
                res = _custom_run_method()
                _custom_run_retval = res
                _custom_run_method = None

            _app.processEvents()
            time.sleep(0.001)
        print "Visualization thread closing..."
        for (name,itemvis) in _vis.items.iteritems():
            itemvis.destroy()
        _thread_running = False
        return res
    
    def _kill():
        global _quit
        _quit = True

    def _show():
        global _window,_showwindow,_thread_running
        if not _thread_running:
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            thread = Thread(target=_run_app_thread)
            thread.start()
        _showwindow = True

    def _hide():
        global _window,_showwindow,_thread_running
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
