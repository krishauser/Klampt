from OpenGL.GL import *
try:
    from PyQt5 import QtGui
    from PyQt5.QtCore import *
    from PyQt5.QtWidgets import *
    from PyQt5.QtOpenGL import *
except ImportError:
    from PyQt4 import QtGui
    from PyQt4.QtCore import *
    from PyQt4.QtGui import *
    from PyQt4.QtOpenGL import *
import sys
import math
import weakref

GLUT_UP = 1
GLUT_DOWN = 0
GLUT_ACTIVE_CTRL = 2
GLUT_ACTIVE_SHIFT = 1
GLUT_ACTIVE_ALT = 4

keymap = {Qt.Key_F1:'f1',
    Qt.Key_F2:'f2',
    Qt.Key_F3:'f3',
    Qt.Key_F4:'f4',
    Qt.Key_F5:'f5',
    Qt.Key_F6:'f6',
    Qt.Key_F7:'f7',
    Qt.Key_F8:'f8',
    Qt.Key_F9:'f9',
    Qt.Key_F10:'f10',
    Qt.Key_F11:'f11',
    Qt.Key_F12:'f12',
    Qt.Key_Up:'up',
    Qt.Key_Left:'left',
    Qt.Key_Down:'down',
    Qt.Key_Right:'right',
    Qt.Key_Home:'home',
    Qt.Key_End:'end',
    Qt.Key_Delete:'delete',
    Qt.Key_Enter:'enter'
}


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
        - width, height: width/height of the window in screen units.  These are initialized from
          the GLProgram viewport on run(), but thereafter Qt manages them.  After, the user must
          call resize(w,h) to change the dimensions.
        - devwidth, devheight: width/height of the window in OpenGL device pixel units
          (Note that these may be different from the screen dimensions due to Retina displays)
        - clearColor: the RGBA floating point values of the background color.
    """
    idlesleep_signal = pyqtSignal(float)
    refresh_signal = pyqtSignal()
    reshape_signal = pyqtSignal(int,int)

    def __init__(self,name="OpenGL window",parent=None):
        format = QGLFormat()
        format.setRgba(True)
        format.setDoubleBuffer(True)
        format.setDepth(True)
        format.setSampleBuffers(True)
        format.setSamples(4)
        if not hasattr(QtGLWindow,"_firstWidget"):
            QtGLWindow._firstWidget = self
            QGLWidget.__init__(self,format,parent)
        else:
            shareWidget = QtGLWindow._firstWidget
            QGLWidget.__init__(self,format,shareWidget=shareWidget)
            #self.setContext(self.context(),shareContext=shareWidget.context())
        
        self.name = name
        self.program = None
        self.width = 640
        self.height = 480
        if hasattr(self,'devicePixelRatio'):
            self.devheight = self.devicePixelRatio()*self.height
            self.devwidth = self.devicePixelRatio()*self.width
        else:
            self.devheight = self.height
            self.devwidth = self.width
        self.sizePolicy = "resize"
        self.clearColor = [1.0,1.0,1.0,0.0]
        #keyboard state information
        self.modifierList = []
        #mouse state information
        self.lastx,self.lasty = None,None
        self.initialized = False
        self.refreshed = False 

        self.setFixedSize(self.width,self.height)
        self.setWindowTitle(self.name)
        self.idleTimer = None
        self.nextIdleEvent = 0
        self.actions = []
        self.actionMenu = None
        self.inpaint = False

        self.idlesleep_signal.connect(self.do_idlesleep)
        self.refresh_signal.connect(self.do_refresh)
        self.reshape_signal.connect(self.do_reshape)

    def setProgram(self,program):
        """User will call this to set up the program variable"""
        from .glprogram import GLProgram
        assert isinstance(program,GLProgram)
        print("######### QGLWidget setProgram ###############")
        if hasattr(program,'name'):
            self.name = program.name
            if self.initialized:
                self.setWindowTitle(program.name)
        self.program = program
        program.window = weakref.proxy(self)
        if hasattr(self,'devicePixelRatio'):
            program.view.screenDeviceScale = self.devicePixelRatio()
        else:
            program.view.screenDeviceScale = 1
        if self.initialized:
            program.initialize()
            program.reshapefunc(self.width,self.height)
            def f():
                self.nextIdleEvent = 0
                if self.program: self.program.idlefunc()
                if self.nextIdleEvent == 0:
                    self.idleTimer.start(0)
            self.idleTimer.timeout.connect(f)
        else:
            self.reshape(program.view.w,program.view.h)

    def setParent(self,parent=None):
        QGLWidget.setParent(self,parent)
        

    def initialize(self):
        """ Opens a window and initializes.  Called internally, and must be in the visualization thread."""
        assert self.program != None, "QGLWidget initialized without a GLProgram"
        try:
            glEnable(GL_MULTISAMPLE)
        except Exception:
            print("QGLWidget.initialize(): perhaps Qt didn't initialize properly?")
            pass
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        def f():
            self.nextIdleEvent = 0
            if self.program: self.program.idlefunc()
            if self.nextIdleEvent == 0:
                self.idleTimer.start(0)
        self.idleTimer = QTimer(self)
        self.idleTimer.timeout.connect(f)
        self.idleTimer.setSingleShot(True)
        self.idleTimer.start(0)
        #init function
        self.program.initialize()

        if self.actionMenu is not None:
            for a in self.actions:
                self.actionMenu.addAction(a)
        else:
            print("QtGLWidget.initialize: no action menu?")

        self.initialized = True

    def hide(self):
        """Hides the window, if already shown"""
        QGLWidget.hide(self)
        if self.initialized:
            self.idlesleep()           

    def show(self):
        """Restores from a previous hide call"""
        QGLWidget.show(self)
        if self.initialized:
            #boot it back up again
            self.idlesleep(0)
        self.refresh()

    def close(self):
        """Qt thread should call close() after this widget should be closed down to stop
        any existing Qt callbacks."""
        print("######### QGLWidget close ###############")
        self.idleTimer.stop()
        self.idleTimer.deleteLater()
        self.idleTimer = None
        if self.program:
            self.program.window = None
        self.program = None
        self.initialized = False

    def add_action(self,hook,short_text,key,description=None):
        """This is called by the user to add actions to the menu bar.

        Parameters:
        - hook: a python callback function, taking no arguments.
        - short_text: the text shown in the menu bar
        - key: a shortcut keyboard command (can be 'k' or 'Ctrl+k')
        - description: if provided, this is a tooltip that shows up when the user
          hovers their mouse over the menu item.
        """
        a = QAction(short_text, self)
        a.setShortcut(key)
        if description == None:
            description = short_text
        a.setStatusTip(description)
        a.triggered.connect(hook)
        self.actions.append(a)

    def sizeHint(self):
        return QSize(self.width,self.height)

    #QtGLWidget bindings
    def initializeGL(self):
        print("######### QGLWidget Initialize GL ###############")
        if self.initialized:
            print("QGLWidget.initializeGL: already initialized?")
        try:
            self.makeCurrent()
            return self.initialize()
        except Exception as e:
            import traceback
            print("QGLWidget.initializeGL: hit an exception?")
            traceback.print_exc()
            exit(-1)
    def resizeGL(self,w,h): 
        if self.program == None:
            print("QGLWidget.resizeGL: called after close?")
            return
        if not self.isVisible():
            return
        (self.devwidth,self.devheight) = (w,h)
        return
    def paintGL(self):
        if self.program == None:
            print("QGLWidget.paintGL: called after close?")
            return
        if not self.isVisible():
            print("QGLWidget.paintGL: called while invisible?")
            return
        if self.inpaint:
            return
        self.inpaint = True
        self.refreshed = False
        try:
            glRenderMode(GL_RENDER)
            res = self.program.displayfunc()
        except Exception as e:
            import traceback
            print("QGLWidget.paintGL: hit an exception?")
            traceback.print_exc()
            exit(-1)
        self.inpaint = False
        return
    #QWidget bindings
    def mouseMoveEvent(self,e):
        if self.program == None:
            print("QGLWidget.mouseMoveEvent: called after close?")
            return
        self.modifierList = toModifierList(e.modifiers())
        x,y = e.pos().x(),e.pos().y()
        if self.lastx == None: dx,dy = 0,0
        else: dx, dy = x - self.lastx, y - self.lasty
        try:
            res = self.program.motionfunc(x,y,dx,dy)
        except Exception as e:
            import traceback
            traceback.print_exc()
            exit(-1)
        self.lastx,self.lasty = x,y
    def mousePressEvent(self,e):
        x,y = e.pos().x(),e.pos().y()
        self.modifierList = toModifierList(e.modifiers())
        self.lastx,self.lasty = x,y
        self.program.mousefunc(toGlutButton(e.button()),GLUT_DOWN,x,y)
    def mouseReleaseEvent(self,e):
        x,y = e.pos().x(),e.pos().y()
        self.modifierList = toModifierList(e.modifiers())
        self.lastx,self.lasty = x,y
        self.program.mousefunc(toGlutButton(e.button()),GLUT_UP,x,y)
    def keyPressEvent(self,e):
        if e.isAutoRepeat():
            return
        if e.key() in keymap:
            self.modifierList = toModifierList(e.modifiers())
            self.program.keyboardfunc(keymap[e.key()],self.lastx,self.lasty)
            return
        else:
            c = str(e.text())
            if len(c)==0: return #some empty press, like shift/control
            self.modifierList = toModifierList(e.modifiers())
            self.program.keyboardfunc(c,self.lastx,self.lasty)
    def keyReleaseEvent(self,e):
        if e.isAutoRepeat():
            return
        if e.key() in keymap:
            self.modifierList = toModifierList(e.modifiers())
            self.program.keyboardupfunc(keymap[e.key()],self.lastx,self.lasty)
            return
        else:
            c = str(e.text())
            if len(c)==0: return #some empty press, like shift/control
            self.modifierList = toModifierList(e.modifiers())
            self.program.keyboardupfunc(c,self.lastx,self.lasty)

    def modifiers(self):
        """Call this to retrieve modifiers. Called by frontend."""
        return self.modifierList

    @pyqtSlot(float)
    def do_idlesleep(self,duration):
        if self.idleTimer is None:
            import traceback
            traceback.print_stack()
            raise ValueError("Idlesleep %f was called before initialize"%(duration,))
        if duration<=0:
            self.nextIdleEvent = 0
            self.idleTimer.start(0)
        elif duration == float('inf'):
            #print "Stopping idle timer",self.name,"forever"
            self.idleTimer.stop()
        else:
            #print "Stopping idle timer",self.name,duration
            self.idleTimer.start(int(1000*duration))
            self.nextIdleEvent = duration

    def idlesleep(self,duration=float('inf')):
        """Externally callable. Sleeps the idle callback for t seconds.  If t is not provided,
        the idle callback is slept forever"""
        self.idlesleep_signal.emit(duration)

    @pyqtSlot()
    def do_refresh(self):
        self.makeCurrent()
        #self.updateGL()
        self.update()

    def refresh(self):
        """Externally callable. Requests a refresh of the window"""
        if not self.refreshed:
            self.refreshed = True
            if not self.isVisible():
                return
            self.refresh_signal.emit()

    def resizeEvent(self,event):
        """Called internally by Qt when Qt resizes the window"""
        QGLWidget.resizeEvent(self,event)

        self.width,self.height = (event.size().width(),event.size().height())
        if hasattr(self,'devicePixelRatio'):
            scale = self.devicePixelRatio() 
        else:
            scale = 1
        (self.devwidth,self.devheight) = (self.width*scale,self.height*scale)
        #self.window().resize(event.size())
        #self.window().adjustSize()
        if self.program:
            self.program.reshapefunc(self.width,self.height)
        self.refresh()

    @pyqtSlot(int,int)
    def do_reshape(self,w,h):
        (self.width,self.height) = (w,h)
        self.setFixedSize(self.width,self.height)
        self.window().resize(self.sizeHint())
        self.window().adjustSize()
        if self.isVisible():
            self.do_refresh()

    def reshape(self,w,h):
        """Externally callable. Reshapes the window"""
        self.reshape_signal.emit(w,h)

    def draw_text(self,point,text,size=12,color=None):
        if color:
            if len(color)==3:
                glColor3f(*color)
            else:
                glColor4f(*color)

        font = QtGui.QFont()
        font.setPixelSize(size)
        if len(point) == 2:
            self.renderText(point[0],point[1],0,text,font)
        else:
            self.renderText(point[0],point[1],point[2],text,font)


class QtBackend:
    """
    Backend implementation of OpenGL visualization using Qt. Usually hidden from the user.

    To use as a standalone program: Set up your GLProgramInterface, call createWindow to
    construct new windows and setProgram to set the GLProgram used in that window.
    Then call run() to start the Qt main loop. 
    """
    def __init__(self):
        self.app = None
        self.window = None

    def initialize(self,program_name):
        print("INITIALIZING Qt BACKEND")
        if self.app == None:
            #this is needed for some X11 multithreading bug 
            QCoreApplication.setAttribute(Qt.AA_X11InitThreads)
            self.app = QApplication([program_name])

    def createWindow(self,name,parent=None):
        self.initialize(name)
        return QtGLWindow(name,parent)

    def run(self):
        """Starts the main loop"""
        assert self.window != None, "No windows create()'ed"
        self.window.show()
        self.app.exec_()

    def kill(self):
        if self.window:
            self.window.close()
            del self.window
        del self.app
        self.app = None
        self.window = None