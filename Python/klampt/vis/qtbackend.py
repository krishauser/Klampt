from OpenGL.GL import *
from PyQt4 import QtGui
from PyQt4.QtCore import *
from PyQt4.QtOpenGL import *
import math
from glcommon import GLMultiProgramInterface

GLUT_UP = 1
GLUT_DOWN = 0
GLUT_ACTIVE_CTRL = 2
GLUT_ACTIVE_SHIFT = 1
GLUT_ACTIVE_ALT = 4

def toGlutButton(button):
    if button==Qt.LeftButton:
        return 0
    elif button==Qt.RightButton:
        return 2
    elif button==Qt.MidButton:
        return 1
    return 0

def toModifierList(modifiers):
    res = []
    if modifiers & Qt.AltModifier:
        res.append('alt')
    if modifiers & Qt.ShiftModifier:
        res.append('shift')
    if modifiers & Qt.ControlModifier:
        res.append('ctrl')
    return res

class QtGLWindow(QGLWidget):
    """A basic OpenGL window using Qt.  Should not be used directly, use
    the functions in QtBackend instead.

    Attributes:
        - name: title of the window (only has an effect before calling
          run())
        - width, height: width/height of the window (only has an effect
          before calling run(), and these are updated when the user resizes
          the window.
        - clearColor: the RGBA floating point values of the background color.
    """
    def __init__(self,name="OpenGL window",parent=None,shared=None):
        QGLWidget.__init__(self)
        self.name = name
        self.plugin = None
        self.width = 640
        self.height = 480
        self.sizePolicy = "resize"
        self.clearColor = [1.0,1.0,1.0,0.0]
        #keyboard state information
        self.modifierList = []
        #mouse state information
        self.lastx,self.lasty = None,None
        self.initialized = False
        self.refreshed = False 

    def setPlugin(self,plugin):
        if hasattr(plugin,'name'):
            self.name = plugin.name
            if self.initialized:
                self.window.setWindowTitle(plugin.name)
        self.plugin = plugin
        plugin.window = self
        if self.initialized:
            self.reshape(plugin,width,plugin.height)
        else:  
            self.width,self.height = plugin.width,plugin.height

    def addPlugin(self,plugin):
        if self.plugin == None:
            self.setPlugin(plugin)
        else:
            #create a multi-view widget
            if isinstance(self.plugin,GLMultiProgramInterface):
                self.plugin.addPlugin(plugin)
            else:
                multiProgram = GLMultiProgramInterface()
                multiProgram.window = self
                multiProgram.addPlugin(self.plugin)
                multiProgram.addPlugin(plugin)
                self.plugin = multiProgram
                self.width,self.height = self.plugin.width,self.plugin.height

    def setParent(self,parent=None,shared=None):
        QGLWidget.__init__(self,parent,shared)

    def initialize(self):
        """ Open a window and initialize """
        if self.initialized:
            raw_input("initialize called twice... this may invalidate textures and display lists... are you sure you want to continue?")
        assert self.plugin != None, "QGLWidget initialized without a plugin"
        self.setFocusPolicy(Qt.StrongFocus)
        self.setWindowTitle(self.name)
        self.idleTimer = QTimer()
        self.idleTimer.timeout.connect(lambda:self.plugin.idlefunc())
        self.idleTimer.start(0)
        format = QGLFormat()
        format.setRgba(True)
        format.setDoubleBuffer(True)
        format.setDepth(True)
        self.setFormat(format)
        self.setFixedSize(self.width,self.height)

        self.setMouseTracking(True)
        #init function
        self.plugin.initialize()
        self.initialized = True

    def sizeHint(self):
        return QSize(self.width,self.height)

    #QtGLWidget bindings
    def initializeGL(self):
        try:
            return self.initialize()
        except Exception,e:
            import traceback
            traceback.print_exc()
            exit(-1)
    def resizeGL(self,w,h): 
        self.plugin.reshapefunc(w,h)
        return
    def paintGL(self) :
        self.refreshed = False
        try:
            res = self.plugin.displayfunc()
        except Exception,e:
            import traceback
            traceback.print_exc()
            exit(-1)
        return
    #QWidget bindings
    def mouseMoveEvent(self,e):
        x,y = e.pos().x(),e.pos().y()
        if self.lastx == None: dx,dy = 0,0
        else: dx, dy = x - self.lastx, y - self.lasty
        try:
            res = self.plugin.motionfunc(x,y,dx,dy)
        except Exception,e:
            import traceback
            traceback.print_exc()
            exit(-1)
        self.lastx,self.lasty = x,y
    def mousePressEvent(self,e):
        x,y = e.pos().x(),e.pos().y()
        self.modifierList = toModifierList(e.modifiers())
        self.lastx,self.lasty = x,y
        self.plugin.mousefunc(toGlutButton(e.button()),GLUT_DOWN,x,y)
    def mouseReleaseEvent(self,e):
        x,y = e.pos().x(),e.pos().y()
        self.modifierList = toModifierList(e.modifiers())
        self.lastx,self.lasty = x,y
        self.plugin.mousefunc(toGlutButton(e.button()),GLUT_UP,x,y)
    def keyPressEvent(self,e):
        c = str(e.text())
        if len(c)==0: return #some empty press, like shift/control
        self.modifierList = toModifierList(e.modifiers())
        self.plugin.keyboardfunc(c,self.lastx,self.lasty)
    def keyReleaseEvent(self,e):
        c = e.text()
        if len(c)==0: return #some empty press, like shift/control
        self.modifierList = toModifierList(e.modifiers())
        self.plugin.keyboardupfunc(c,self.lastx,self.lasty)

    def modifiers(self):
        """Call this to retrieve modifiers. Called by frontend."""
        return self.modifierList

    def idlesleep(self,duration=float('inf')):
        """Sleeps the idle callback for t seconds.  If t is not provided,
        the idle callback is slept forever"""
        if time==0:
            self.idleTimer.start(0)
        else:
            self.idleTimer.stop()
            if duration!=float('inf'):
                QTimer.singleShot(duration*1000,lambda x:self.idleTimer.start(0));

    def close(self):
        """Call close() after this widget should be closed down, to stop
        any existing Qt callbacks."""
        self.idleTimer.stop()
        self.plugin.window = None
        self.plugin = None

    def refresh(self):
        if not self.refreshed:
            self.refreshed = False
            #TODO: resolve whether it's better to call updateGL here or to schedule
            # a timer event
            self.updateGL()
            #QTimer.singleShot(0,lambda:self.updateGL());

    def reshape(self,w,h):
        self.width,self.height = w,h
        self.setFixedSize(self.width,self.height)
        self.refresh()

    def draw_text(self,point,text,size=12,color=None):
        if color:
            glColor3f(*color)
        if len(point) == 2:
            self.renderText(point[0],point[1],0,text)
        else:
            self.renderText(point[0],point[1],point[2],text)


class QtBackend:
    """
    To use as a standalone program: Set up your GLProgramInterface, then call run() to start the Qt main loop. 
    
    For more control over windowing, you can use the createWindow function to
    construct new windows and addPlugin to add plugins to those windows.

    IMPORTANT NOTE: only one window may be created for a given world.  If you want to
    use multiple windows, then a new world should be loaded for each world.
    """
    def __init__(self):
        self.app = None
        self.window = None

    def initialize(self,program_name):
        if self.app == None:
            self.app = QtGui.QApplication([program_name])

    def createWindow(self,name,parent=None):
        self.initialize(name)
        return QtGLWindow(name,parent)

    def addPlugin(self,plugin,window):
        """ Open a window and initialize. Users should not call this
        directly! Call create() or run() on the plugin instead. """
        if window == None:
            if self.window == None:
                self.window = self.createWindow(plugin.name)
            window = self.window
        window.addPlugin(plugin)

    def run(self):
        """Starts the main loop"""
        assert self.window != None, "No windows create()'ed"
        self.window.show()
        self.app.exec_()


