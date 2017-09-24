from OpenGL.GL import *
from PyQt4 import QtGui
from PyQt4.QtCore import *
from PyQt4.QtOpenGL import *
import math

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
        - width, height: width/height of the window (only has an effect
          before calling run(), and these are updated when the user resizes
          the window.
        - clearColor: the RGBA floating point values of the background color.
    """
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
        self.idleTimer = QTimer()
        self.nextIdleEvent = 0
        self.actions = []
        self.actionMenu = None
        self.inpaint = False

    def setProgram(self,program):
        from glprogram import GLProgram
        assert isinstance(program,GLProgram)
        if hasattr(program,'name'):
            self.name = program.name
            if self.initialized:
                self.setWindowTitle(program.name)
        self.program = program
        program.window = self
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
        """ Open a window and initialize """
        assert self.program != None, "QGLWidget initialized without a GLProgram"
        glEnable(GL_MULTISAMPLE)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        def f():
            self.nextIdleEvent = 0
            if self.program: self.program.idlefunc()
            if self.nextIdleEvent == 0:
                self.idleTimer.start(0)
        self.idleTimer.timeout.connect(f)
        self.idleTimer.setSingleShot(True)
        self.idleTimer.start(0)
        #init function
        self.program.initialize()

        if self.actionMenu is not None:
            for a in self.actions:
                self.actionMenu.addAction(a)
        else:
            print "QtGLWidget.initialize: no action menu?"

        self.initialized = True

    def add_action(self,hook,short_text,key,description=None):
        a = QtGui.QAction(short_text, self)
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
        print "######### QGLWidget Initialize GL ###############"
        if self.initialized:
            print "QGLWidget.initializeGL: already initialized?"
        try:
            self.makeCurrent()
            return self.initialize()
        except Exception,e:
            import traceback
            print "QGLWidget.initializeGL: hit an exception?"
            traceback.print_exc()
            exit(-1)
    def resizeGL(self,w,h): 
        if self.program == None:
            print "QGLWidget.resizeGL: called after close?"
            return
        if not self.isVisible():
            return
        (self.width,self.height) = (w,h)
        self.program.reshapefunc(w,h)
        return
    def paintGL(self):
        if self.program == None:
            print "QGLWidget.paintGL: called after close?"
            return
        if not self.isVisible():
            print "QGLWidget.paintGL: called while invisible?"
            return
        if self.inpaint:
            return
        self.inpaint = True
        self.refreshed = False
        try:
            glRenderMode(GL_RENDER)
            res = self.program.displayfunc()
        except Exception,e:
            import traceback
            print "QGLWidget.paintGL: hit an exception?"
            traceback.print_exc()
            exit(-1)
        self.inpaint = False
        return
    #QWidget bindings
    def mouseMoveEvent(self,e):
        if self.program == None:
            print "QGLWidget.mouseMoveEvent: called after close?"
            return
        x,y = e.pos().x(),e.pos().y()
        if self.lastx == None: dx,dy = 0,0
        else: dx, dy = x - self.lastx, y - self.lasty
        try:
            res = self.program.motionfunc(x,y,dx,dy)
        except Exception,e:
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

    def idlesleep(self,duration=float('inf')):
        """Sleeps the idle callback for t seconds.  If t is not provided,
        the idle callback is slept forever"""
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

    def close(self):
        """Call close() after this widget should be closed down, to stop
        any existing Qt callbacks."""
        #print "######### QGLWidget close ###############"
        self.idleTimer.stop()
        if self.program:
            self.program.window = None
        self.program = None

    def refresh(self):
        if not self.refreshed:
            self.refreshed = True
            if not self.isVisible():
                return
            #TODO: resolve whether it's better to call updateGL here or to schedule
            # a timer event
            #first method: may have issues with being called from a different thread?
            #self.makeCurrent()
            #self.updateGL()
            #second method: works even when called from a different thread.
            def dorefresh():
                self.makeCurrent()
                #self.updateGL()
                self.update()
            QTimer.singleShot(0,dorefresh)

    def reshape(self,w,h):
        (self.width,self.height) = (w,h)
        def doreshape():
            self.setFixedSize(self.width,self.height)
            self.window().resize(self.sizeHint())
            self.window().adjustSize()
            if self.isVisible():
                self.refresh()
        if not self.initialized: doreshape()
        else: QTimer.singleShot(0,doreshape)

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
        if self.app == None:
            #this is needed for some X11 multithreading bug 
            QCoreApplication.setAttribute(Qt.AA_X11InitThreads)
            self.app = QtGui.QApplication([program_name])

    def createWindow(self,name,parent=None):
        self.initialize(name)
        return QtGLWindow(name,parent)

    def run(self):
        """Starts the main loop"""
        assert self.window != None, "No windows create()'ed"
        self.window.show()
        self.app.exec_()


