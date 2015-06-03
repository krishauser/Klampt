"""Klamp't world visualization routines.

Due to weird OpenGL behavior when opening/closing windows, you should only
run visualizations using the methods in this module, or manually running
glprogram / qtprogram code, NOT BOTH.  Since the editors in the
resource module use this module, you should also use it if you plan to use
the resource editor.

The main interface is as follows:

def setWindowTitle(title): sets the title of the visualization window.
def getWindowtitle(): returns the title of the visualization window
def dialog(): pops up a dialog box (does not return to calling
    thread until closed).
def show(lock or False): shows or hides a visualization window run
    simultaneously with the calling thread.  To show the window,
    you must pass in a global lock that keeps the visualization from
    accessing corrupt data as you change the visualization data (e.g., a
    WorldModel.)  To hide the window, pass False.
def shown(): returns true if the window is shown.
def clear(): clears the visualization world.
def add(name,item): adds an item to the visualization.  name is a unique
    identifier.  If an item with the same name already exists, it will no
    longer be shown
def remove(name): removes an item from the visualization.
def hide(name,hidden=True): hides/unhides an item.  The item is not removed,
    it just becomes invisible.
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
_globalLock = None
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
    global _vis
    if _vis==None:
        print "Visualization disabled"
        return
    _kill()

def show(lock):
    global _vis,_globalLock
    if lock == False:
        _hide()
        return
    _globalLock = lock
    if _vis==None:
        print "Visualization disabled"
        return
    _show()

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

def add(name,item):
    global _vis,_globalLock
    if _vis==None:
        print "Visualization disabled"
        return
    if _globalLock:_globalLock.acquire()
    _vis.items[name] = (item,VisAppearance())
    if _globalLock:_globalLock.release()

def setAnimation(name,animation,speed):
    global _vis,_globalLock
    if _vis==None:
        print "Visualization disabled"
        return
    if _globalLock:_globalLock.acquire()
    _vis.items[name][1].animation = animation
    _vis.items[name][1].animationSpeed = animationSpeed
    if _globalLock:_globalLock.release()

def remove(name):
    global _vis,_globalLock
    if _vis==None:
        return
    if _globalLock:_globalLock.acquire()
    del _vis.items[name]
    if _globalLock:_globalLock.release()

def hide(name,hidden=True):
    global _vis,_globalLock
    if _vis==None:
        return
    if _globalLock:_globalLock.acquire()
    _vis.items[name][1].hidden = True
    if _globalLock:_globalLock.release()

def setAppearance(name,appearance):
    global _vis,_globalLock
    if _vis==None:
        return
    if _globalLock:_globalLock.acquire()
    _vis.items[name][1].useDefaultAppearance = False
    _vis.items[name][1].customAppearance = appearance
    if _globalLock:_globalLock.release()

def revertAppearance(name):
    global _vis,_globalLock
    if _vis==None:
        return
    if _globalLock:_globalLock.acquire()
    _vis.items[name][1].useDefaultApperance = True
    if _globalLock:_globalLock.release()

def setColor(name,r,g,b,a=1.0):
    global _vis,_globalLock
    if _vis==None:
        return
    if _globalLock:_globalLock.acquire()
    _vis.items[name][1].color = [r,g,b,a]
    _vis.items[name][1].useDefaultAppearance = False
    if _globalLock:_globalLock.release()





class VisAppearance:
    def __init__(self):
        self.hidden = False
        self.color = None
        self.useDefaultAppearance = True
        self.customAppearance = None
        #TODO: customize appearance of sub-items
        #self.subAppearances = []
        self.animation = None
        self.animationSpeed = 1.0
        self.attributes = {}
    def drawItem(self,item,world=None):
        if self.hidden: return
        #set appearance
        if not self.useDefaultAppearance and hasattr(item,'appearance'):
            oldAppearance = item.appearance().clone()
            if self.customAppearance != None:
                item.appearance().set(self.customAppearance)
            elif self.color != None:
                item.appearance().setColor(*self.color)
                
        if hasattr(item,'drawGL'):
            item.drawGL()
        elif isinstance(item,coordinates.Point):
            glDisable(GL_LIGHTING)
            glPointSize(self.attributes.get("size",5.0))
            glColor4f(*self.color)
            glBegin(GL_POINTS)
            glVertex3f(*item.worldCoordinates())
            glEnd()
            #TODO: write name at point
        elif isinstance(item,coordinates.Direction):
            glDisable(GL_LIGHTING)
            L = self.attributes.get("length",0.15)
            source = item.frame().worldCoordinates()[1]
            glColor4f(*self.color)
            glBegin(GL_LINES)
            glVertex3f(*source)
            glVertex3f(*vectorops.madd(source,item.worldCoordinates(),L))
            glEnd()
            #TODO: write name at origin
        elif isinstance(item,coordinates.Frame):
            gldraw.xform_widget(item.worldCoordinates(),self.attributes.get("length",0.1),self.attributes.get("width",0.01))
            #TODO: write name at origin
            #TODO: draw curve between frame and parent
            pass
        elif isinstance(item,coordinates.Transform):
            #TODO: draw curve between frames
            pass
        elif isinstance(item,coordinates.Group):
            for n,f in item.frames:
                self.drawItem(f,world)
            for n,p in item.points:
                self.drawItem(p,world)
            for n,d in item.points:
                self.drawItem(d,world)
            for n,g in item.subgroups:
                self.drawItem(g,world)
        else:
            types = resource.objectToTypes(item,world)
            if isinstance(types,(list,tuple)):
                #ambiguous, still need to figure out what to draw
                print "Unable to draw item of ambiguous types",types
            else:
                if types == 'Config':
                    if world:
                        world.robot(0).setConfig(item)
                        for i in xrange(world.robot(0).numLinks):
                            self.drawItem(world.robot(0).link(i),world)
                elif types == 'RigidTransform':
                    gldraw.xform_widget(item,self.attributes.get("length",0.1),self.attributes.get("width",0.01))
                else:
                    print "Unable to draw item of type",types

        #revert appearance
        if not self.useDefaultAppearance and hasattr(item,'appearance'):
            item.appearance().set(oldAppearance)


if glcommon._PyQtAvailable or glcommon._GLUTAvailable:

    if glcommon._PyQtAvailable:
        import qtprogram
        _baseClass = qtprogram.GLRealtimeProgram
    else:
        import glprogram
        _baseClass = glprogram.GLRealtimeProgram
           
    class GLPluginProgram(_baseClass):
        def __init__(self,name="GLWidget"):
            _baseClass.__init__(self,name)
            self.iface = None
        def setPlugin(self,iface):
            self.iface = iface
            if iface:
                iface.widget = self
                iface.reshapefunc(self.width,self.height)
            self.refresh()
        def initialize(self):
            if self.iface: self.iface.initialize()
            _baseClass.initialize(self)
        def reshapefunc(self,w,h):
            if self.iface==None or not self.iface.reshapefunc(w,h):
                _baseClass.reshapefunc(self,w,h)
        def keyboardfunc(self,c,x,y):
            if self.iface==None or not self.iface.keyboardfunc(c,x,y):
                _baseClass.keyboardfunc(self,c,x,y)
        def keyboardupfunc(self,c,x,y):
            if self.iface==None or not self.iface.keyboardupfunc(c,x,y):
                _baseClass.keyboardupfunc(self,c,x,y)
        def specialfunc(self,c,x,y):
            if self.iface==None or not self.iface.specialfunc(c,x,y):
                _baseClass.specialfunc(self,c,x,y)
        def specialupfunc(self,c,x,y):
            if self.iface==None or not self.iface.specialupfunc(c,x,y):
                _baseClass.specialupfunc(self,c,x,y)
        def motionfunc(self,x,y,dx,dy):
            if self.iface==None or not self.iface.motionfunc(x,y,dx,dy):
                _baseClass.motionfunc(self,x,y,dx,dy)
        def mousefunc(self,button,state,x,y):
            if self.iface==None or not self.iface.mousefunc(button,state,x,y):
                _baseClass.mousefunc(self,button,state,x,y)
        def idlefunc(self):
            if self.iface!=None: self.iface.idlefunc()
            _baseClass.idlefunc(self)
        def display(self):
            if self.iface!=None:
                self.iface.display()
        def display_screen(self):
            if self.iface!=None:
                self.iface.display_screen()

    class VisualizationProgram(GLPluginProgram):
        def __init__(self):
            GLPluginProgram.__init__(self,"Klamp't Visualizer")
            self.items = {}

        def display(self):
            if _globalLock:_globalLock.acquire()
            for (k,v) in self.items.iteritems():
                v[1].drawItem(v[0],self.items.get('world',(None,None))[0])
            GLPluginProgram.display(self)
            if _globalLock:_globalLock.release()

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
        while not _quit:
            if _window:
                if _showwindow:
                    _window.show()
                else:
                    _window.hide()
                    _showwindow = False
                    _window = None
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
