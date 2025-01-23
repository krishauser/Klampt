from OpenGL.GL import *
from .. import glinit
if glinit.active() == 'PyQt6':
    from PyQt6 import QtGui
    from PyQt6.QtCore import *
    from PyQt6.QtWidgets import *
    from PyQt6.QtOpenGL import *
    from PyQt6.QtOpenGLWidgets import QOpenGLWidget
    from PyQt6.QtGui import QAction
    Key = Qt.Key
    Button = Qt.MouseButton
    Modifier = Qt.KeyboardModifier
    Focus = Qt.FocusPolicy
    Attribute = Qt.ApplicationAttribute
    PYQT_VERSION = 6
elif glinit.active() == 'PyQt5':
    from PyQt5 import QtGui
    from PyQt5.QtCore import *
    from PyQt5.QtWidgets import *
    from PyQt5.QtOpenGL import *
    Key = Qt
    Button = Qt
    Modifier = Qt
    Focus = Qt
    Attribute = Qt

    #hack needed to get OpenCV working with PyQt5
    import os
    from PyQt5.QtCore import QLibraryInfo
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = QLibraryInfo.location(QLibraryInfo.PluginsPath)

    PYQT_VERSION = 5
else:
    raise RuntimeError("Can't load qtbackend without previously initializing with glinit module")

import sys
import math
import weakref
import signal

GLUT_UP = 1
GLUT_DOWN = 0
GLUT_ACTIVE_CTRL = 2
GLUT_ACTIVE_SHIFT = 1
GLUT_ACTIVE_ALT = 4

keymap = {Key.Key_F1:'f1',
    Key.Key_F2:'f2',
    Key.Key_F3:'f3',
    Key.Key_F4:'f4',
    Key.Key_F5:'f5',
    Key.Key_F6:'f6',
    Key.Key_F7:'f7',
    Key.Key_F8:'f8',
    Key.Key_F9:'f9',
    Key.Key_F10:'f10',
    Key.Key_F11:'f11',
    Key.Key_F12:'f12',
    Key.Key_Up:'up',
    Key.Key_Left:'left',
    Key.Key_Down:'down',
    Key.Key_Right:'right',
    Key.Key_Home:'home',
    Key.Key_End:'end',
    Key.Key_Delete:'delete',
    Key.Key_Enter:'enter'
}


def toGlutButton(button):
    if button==Button.LeftButton:
        return 0
    elif button==Button.RightButton:
        return 2
    else:
        if PYQT_VERSION == 6:
            if button==Button.MiddleButton:
                return 1
        else:
            if button==Button.MidButton:
                return 1
    return 0

def toModifierList(modifiers):
    res = []
    if modifiers & Modifier.AltModifier:
        res.append('alt')
    if modifiers & Modifier.ShiftModifier:
        res.append('shift')
    if modifiers & Modifier.ControlModifier:
        res.append('ctrl')
    return res

class QtGLWindow(QOpenGLWidget):
    """A basic OpenGL window using Qt.  Should not be used directly, use
    the functions in QtBackend instead.

    Attributes:
        name (str): title of the window (only has an effect before calling run())
        width, height (int): width/height of the window in screen units.  These are initialized
            from the GLProgram viewport on run(), but thereafter Qt manages them.  After, the
            user must call resize(w,h) to change the dimensions.
        devwidth, devheight (int): width/height of the window in OpenGL device pixel units
          (Note that these may be different from the screen dimensions due to Retina displays)
    """
    idlesleep_signal = pyqtSignal(float)
    refresh_signal = pyqtSignal()
    reshape_signal = pyqtSignal(int,int)

    def __init__(self,name="OpenGL window",parent=None):
        QOpenGLWidget.__init__(self,parent=parent)
        format = QtGui.QSurfaceFormat.defaultFormat()
        if format.depthBufferSize() < 24:
            format.setDepthBufferSize(24)
        
        if PYQT_VERSION == 5:
            format.setSwapBehavior(QtGui.QSurfaceFormat.DoubleBuffer)
        else:
            format.setSwapBehavior(QtGui.QSurfaceFormat.SwapBehavior.DoubleBuffer)
        format.setSamples(4)
        self.setFormat(format)
    
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
        self.painted = False
        def im_painted():
            self.painted = True        
        self.aboutToCompose.connect(im_painted)

    def renderText(self, x:float, y:float, z:float, text:str, font:QtGui.QFont):
        """Provided for compatibility between PyQt6 and earlier versions"""
        if PYQT_VERSION >= 5:
            width = self.width
            height = self.height

            from OpenGL import arrays
            model = glGetDoublev(GL_MODELVIEW_MATRIX)
            proj = glGetDoublev(GL_PROJECTION_MATRIX)
            view = glGetIntegerv(GL_VIEWPORT)
            proj = arrays.GLdoubleArray.asArray(proj)
            model = arrays.GLdoubleArray.asArray(model)
            model = model.T
            proj = proj.T
            
            (tx,ty,tz) = self.project((x, y, z), 
                        model, proj, view)

            ty = height - ty#  // y is inverted

            painter = QtGui.QPainter(self)
            rgba = glGetFloatv(GL_CURRENT_COLOR)
            col = QtGui.QColor()
            col.setRgbF(rgba[0], rgba[1], rgba[2], rgba[3])
            painter.setPen(col)
            painter.setFont(QtGui.QFont("Helvetica", 8))
            if PYQT_VERSION == 6:
                painter.setRenderHints(QtGui.QPainter.RenderHint.Antialiasing | QtGui.QPainter.RenderHint.TextAntialiasing)
            else:
                painter.setRenderHints(QtGui.QPainter.Antialiasing | QtGui.QPainter.TextAntialiasing)
            painter.drawText(int(tx), int(ty), text)
            painter.end()
        else:
            QOpenGLWidget.renderText(self, x, y, z, text, font)

    def project(self,pt, model, proj, view):
        """Compatibility needed for renderText"""
        pt = [pt[0],pt[1],pt[2],1.0]
        pt = proj.dot(model.dot(pt))
        if pt[3] == 0.0:
            return pt[:3]

        pt[0] /= pt[3]
        pt[1] /= pt[3]
        pt[2] /= pt[3]

        winx = view[0] + (1 + pt[0]) * view[2] / 2
        winy = view[1] + (1 + pt[1]) * view[3] / 2
        winz = (1 + pt[2]) / 2
        return winx,winy,winz
        
    def setProgram(self,program):
        """User will call this to set up the program variable"""
        from ..glprogram import GLProgram
        assert isinstance(program,GLProgram)
        print("######### QtGLWindow setProgram ###############")
        if hasattr(program,'name'):
            self.name = program.name
            if self.initialized:
                self.setWindowTitle(program.name)
        self.program = program
        program.window = weakref.proxy(self)
        if hasattr(self,'devicePixelRatio'):
            program.view.screenDeviceScale = self.devicePixelRatio()
            if int(program.view.screenDeviceScale) != program.view.screenDeviceScale:
                raise ValueError("Screen-device scale is not integer, no idea how this will work...")
            else:
                program.view.screenDeviceScale = int(program.view.screenDeviceScale)
        else:
            program.view.screenDeviceScale = 1
        if self.initialized:
            program.initialize()
            program.reshapefunc(self.width,self.height)
            def idleCallback():
                self.nextIdleEvent = 0
                if self.program: self.program.idlefunc()
                if self.nextIdleEvent == 0:
                    self.idleTimer.start(0)
            self.idleTimer.timeout.connect(idleCallback)
        else:
            self.reshape(program.view.w,program.view.h)

    def setParent(self,parent=None):
        QOpenGLWidget.setParent(self,parent)
        
    def initialize(self):
        """ Opens a window and initializes.  Called internally, and must be in the visualization thread."""
        assert self.program != None, "QtGLWindow initialized without a GLProgram"
        try:
            glEnable(GL_MULTISAMPLE)
        except Exception:
            print("QtGLWindow.initialize(): perhaps Qt didn't initialize properly?")
            pass
        self.setMouseTracking(True)
        self.setFocusPolicy(Focus.StrongFocus)
        def idleCallback():
            self.nextIdleEvent = 0
            if self.program: self.program.idlefunc()
            if self.nextIdleEvent == 0:
                self.idleTimer.start(0)
        self.idleTimer = QTimer(self)
        self.idleTimer.timeout.connect(idleCallback)
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
        QOpenGLWidget.hide(self)
        if self.initialized:
            self.idlesleep()           

    def show(self):
        """Restores from a previous hide call"""
        QOpenGLWidget.show(self)
        if self.initialized:
            #boot it back up again
            self.idlesleep(0)
        self.refresh()

    def close(self):
        """Qt thread should call close() after this widget should be closed down to stop
        any existing Qt callbacks."""
        if not self.initialized:
            return
        print("######### QtGLWindow close ###############")
        self.idleTimer.stop()
        self.idleTimer.deleteLater()
        self.idleTimer = None
        if self.program:
            self.program.window = None
        self.program = None
        self.initialized = False

    def add_action(self,hook,short_text,key=None,description=None):
        """This is called by the user to add actions to the menu bar.

        Args:
            hook (function): a python callback function, taking no arguments, called
                when the action is triggered.
            short_text (str): the text shown in the menu bar.
            key (str, optional): a shortcut keyboard command (can be 'k' or 'Ctrl+k').
            description (str, optional): if provided, this is a tooltip that shows up
                when the user hovers their mouse over the menu item.
        """
        a = QAction(short_text, self)
        if key is not None:
            a.setShortcut(key)
        if description is None:
            description = short_text
        a.setStatusTip(description)
        a.triggered.connect(hook)
        self.actions.append(a)

    def sizeHint(self):
        return QSize(self.width,self.height)

    def get_screen(self,format,want_depth):
        # if want_depth:
        #     raise NotImplementedError("Depth screenshots not supported in Qt")
        if PYQT_VERSION == 4:
            raise NotImplementedError("Screenshots no longer supported in PyQt4 version")
        import io
        qimg = self.grabFramebuffer()
        if format == 'auto':
            try:
                import numpy as np
                format = 'numpy'
            except ImportError:
                try:
                    from PIL import Image
                    format = 'Image'
                except ImportError:
                    format = 'bytes'
        if format == 'numpy':
            import numpy as np
            width = qimg.width()
            height = qimg.height()

            ptr = qimg.bits()
            ptr.setsize(height * width * 4)
            rgb = np.frombuffer(ptr, np.uint8).reshape((height, width, 4))
            rgb = np.copy(rgb[:,:,:3][:,:,::-1])
        elif format == 'Image':
            from PIL import Image
            buffer = QBuffer()
            if PYQT_VERSION < 6:
                buffer.open(QBuffer.ReadWrite)
            else:
                buffer.open(QBuffer.OpenModeFlag.ReadWrite)
            qimg.save(buffer, "PNG")
            rgb = Image.open(io.BytesIO(buffer.data()))
        else:
            width = qimg.width()
            height = qimg.height()
            rgb = (width,height,qimg.bits().tobytes())

        if want_depth:
            self.makeCurrent()
            x,y,w,h = self.program.view.x*self.program.view.screenDeviceScale,self.program.view.y*self.program.view.screenDeviceScale,self.program.view.w*self.program.view.screenDeviceScale,self.program.view.h*self.program.view.screenDeviceScale
            n,f = self.program.view.n,self.program.view.f
            try:
                depthdata = glReadPixels( x, y, w, h, GL_DEPTH_COMPONENT, GL_FLOAT)
            except OpenGL.error.GLError:
                import numpy as np
                import warnings
                warnings.warn("Depth screenshots not supported in QOpenGLWidget")
                depth = np.zeros((h,w),np.float32)
                if format == 'Image':
                    from PIL import Image
                    depth = Image.fromarray(depth)
                elif format == 'numpy':
                    pass
                else:
                    depth = (w,h,depth.flatten().tolist())
                return (rgb,depth)
            if format == 'numpy':
                import numpy as np
                depth = np.frombuffer(depthdata,dtype=np.float32).reshape((h,w))
                depth = np.flip(depth,0)
                depth = (n*f)/(f - depth*(f-n))
            elif format == 'Image':
                from PIL import Image,ImageMath
                depth = Image.frombuffer("F", (w, h), depthdata, "raw", "F", 0, 0)
                depth = depth.transpose(Image.FLIP_TOP_BOTTOM)
                depth = ImageMath.eval("%f / (%f - a*%f)"%(n*f,f,f-n),a=depth)
            else:
                import struct
                deptharray = [struct.unpack("<f",depthdata[i:i+4]) for i in range(0,w*h*4,4)] 
                for i in range(w*h):
                    deptharray[i] = n*f/(f - deptharray[i]*(f-n))
                depth = (w,h,deptharray)
            return (rgb,depth)
        return rgb


    #QtGLWidget bindings
    def initializeGL(self):
        print("######### QtGLWindow Initialize GL ###############")
        if self.initialized:
            print("QtGLWindow.initializeGL: already initialized?")
        try:
            self.makeCurrent()
            return self.initialize()
        except Exception as e:
            import traceback
            print("QtGLWindow.initializeGL: hit an exception?")
            traceback.print_exc()
            exit(-1)
    def resizeGL(self,w,h): 
        if self.program == None:
            print("QtGLWindow.resizeGL: called after close?")
            return
        if not self.isVisible():
            print("QtGLWindow.resizeGL: called when invisible?")
            return
        (self.devwidth,self.devheight) = (w,h)
        return
    def paintGL(self):
        if self.program == None:
            print("QtGLWindow.paintGL: called after close?")
            return
        if not self.isVisible():
            print("QtGLWindow.paintGL: called while invisible?")
            return
        if self.inpaint:
            print("QtGLWindow.paintGL: called in the middle of painting?")
            return
        self.inpaint = True
        self.refreshed = False
        try:
            glRenderMode(GL_RENDER)
            res = self.program.displayfunc()
        except Exception as e:
            import traceback
            print("QtGLWindow.paintGL: hit an exception?")
            traceback.print_exc()
            exit(-1)
        self.inpaint = False
        return
    #QWidget bindings
    def mouseMoveEvent(self,e):
        if self.program == None:
            print("QtGLWindow.mouseMoveEvent: called after close?")
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
    def wheelEvent(self,e):
        #x,y = e.pos().x(),e.pos().y()
        numDegrees = e.angleDelta() / 8
        self.modifierList = toModifierList(e.modifiers())
        #self.lastx,self.lasty = x,y
        self.program.mousewheelfunc(numDegrees.x(),numDegrees.y(),self.lastx,self.lasty)
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
            #print("Stopping idle timer",self.name,"forever")
            self.idleTimer.stop()
            self.nextIdleEvent = 1
        else:
            #print("Delaying idle timer",self.name,duration)
            self.idleTimer.start(int(1000*duration))
            self.nextIdleEvent = duration

    def idlesleep(self,duration=float('inf')):
        """Externally callable. Sleeps the idle callback for t seconds.  If t is not provided,
        the idle callback is slept forever"""
        self.idlesleep_signal.emit(duration)

    @pyqtSlot()
    def do_refresh(self):
        self.makeCurrent()
        self.update()
        self.refreshed = True

    def refresh(self):
        """Externally callable. Requests a refresh of the window"""
        if not self.refreshed:
            if not self.isVisible():
                return
            self.refresh_signal.emit()

    def resizeEvent(self,event):
        """Called internally by Qt when Qt resizes the window"""
        QOpenGLWidget.resizeEvent(self,event)

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
        """Renders text at the given point (may be 2d or 3d).  Frontend should call this to draw text
        in either the display or display_screen method.

        Args:
            point (list of floats): either a 2d or 3d point at which to draw the text
            text (str): the text to draw
            size (int, optional): if given, it renders a font in the given size. 
            color (list of 3 floats or list of 4 floats) if given, then an RGB or RGBA color value.

        """
        if color:
            if len(color)==3:
                glColor3f(*color)
            else:
                glColor4f(*color)

        font = QtGui.QFont('Arial')
        #font.setPixelSize(size)
        font.setPointSize(size)
        glPixelStorei(GL_UNPACK_ALIGNMENT,4)   # Needed for correct font rendering?
        if len(point) == 2:
            self.renderText(point[0],point[1],0,text,font)
        else:
            self.renderText(point[0],point[1],point[2],text,font)

    def points_to_pixels(self,pts):
        return pts*QtGui.QGuiApplication.primaryScreen().physicalDotsPerInch()/72

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
            if PYQT_VERSION == 4:
                QCoreApplication.setAttribute(Attribute.AA_X11InitThreads | Attribute.AA_ShareOpenGLContexts )
            else:
                QCoreApplication.setAttribute(Attribute.AA_ShareOpenGLContexts)
            self.app = QApplication([program_name])

    def createWindow(self,name,parent=None):
        self.initialize(name)
        self.window = QtGLWindow(name,parent)
        return self.window

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