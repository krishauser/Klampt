"""This module defines convenient classes for building 3D GUI programs
over OpenGL (GLUT).

- GLProgram takes care of basic user input.
- GLNavigationProgram allows 3D navigation with the mouse.
- GLRealtimeProgram calls a subclass-defined idle() function roughly on a
  constant time step.
"""

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import camera
import se3
import so3
import vectorops
import math
import time
from robotsim import Viewport

class GLProgram:
    """A basic OpenGL program using GLUT.  Set up your window parameters,
    then call run() to start the GLUT main loop.

    Attributes:
        - name: title of the window (only has an effect before calling
          run())
        - width, height: width/height of the window (only has an effect
          before calling run(), and these are updated when the user resizes
          the window.
        - clearColor: the RGBA floating point values of the background color.
        - glutInitialized: true if GLUT has been initialized
    """
    def __init__(self,name="OpenGL Program"):
        self.name = name
        self.width = 640
        self.height = 480
        self.clearColor = [1.0,1.0,1.0,0.0]
        self.glutInitialized = False
        self.lastx = 0
        self.lasty = 0

    def initWindow(self):
        """ Open a window and initialize. Users should not call this
        directly! Call run() instead. """
        glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE)

        x = 0
        y = 0
        glutInitWindowPosition (x, y);
        glutInitWindowSize (self.width, self.height);
        glutCreateWindow (self.name)
  
        # set window callbacks
        glutReshapeFunc (self.reshapefunc)
        glutKeyboardFunc (self.keyboardfunc)
        glutKeyboardUpFunc (self.keyboardupfunc)
        glutSpecialFunc (self.specialfunc)
        glutSpecialUpFunc (self.specialupfunc)
        glutMotionFunc (self._motionfunc)
        glutPassiveMotionFunc (self._motionfunc)
        glutMouseFunc (self._mousefunc)
        glutDisplayFunc (self.displayfunc)
        glutIdleFunc(self.idlefunc)

        #init function
        self.initialize()
        self.glutInitialized = True

    def run(self):
        """Starts the main loop"""
        # Initialize Glut
        glutInit ([])
        if bool(glutSetOption):
           glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS)
        self.initWindow()
        glutMainLoop ()

    def initialize(self):
        """Called after GLUT is initialized, but before main loop.
        May be overridden.  Users should not call this directly!"""
        glutPostRedisplay()
        glEnable(GL_MULTISAMPLE)
        pass

    def refresh(self):
        """Call this to redraw the screen on the next event loop"""
        if self.glutInitialized:
            glutPostRedisplay()

    def reshapefunc(self,w,h):
        """Called on window resize.  May be overridden."""
        self.width = w
        self.height = h
        glutPostRedisplay()
        
    def keyboardfunc(self,c,x,y):
        """Called on keypress down. May be overridden."""
        pass
    def keyboardupfunc(self,c,x,y):
        """Called on keyboard up (if your system allows it). May be overridden."""
        pass
    def specialfunc(self,c,x,y):
        """Called on special character keypress down.  May be overridden"""
        pass
    def specialupfunc(self,c,x,y):
        """Called on special character keypress up up (if your system allows
        it).  May be overridden"""
        pass

    def motionfunc(self,x,y,dx,dy):
        """Called when the mouse moves on screen.  May be overridden."""
        pass
    def mousefunc(self,button,state,x,y):
        """Called when the mouse is clicked.  May be overridden."""
        pass
    
    def displayfunc(self):
        """All OpenGL calls go here.  May be overridden, although you
        may wish to override display() and display_screen() instead."""
        if self.width == 0 or self.height == 0:
            #hidden?
            print "GLProgram.displayfunc called on hidden window?"
            return
        self.prepare_GL()
        self.display()
        self.prepare_screen_GL()
        self.display_screen()
        glutSwapBuffers ()
        
    def idlefunc(self):
        """Called on idle.  May be overridden."""
        self.idlesleep()

    def idlesleep(self,duration=float('inf')):
        """Sleeps the idle callback for t seconds.  If t is not provided,
        the idle callback is slept forever"""
        if time==0:
            glutIdleFunc(_idlefunc);
        else:
            glutIdleFunc(None);
            if duration!=float('inf'):
                glutTimerFunc(duration*1000,lambda x:glutIdleFunc(_idlefunc),0);

    def prepare_GL(self):
        """Prepare drawing in world coordinate frame
        """
        
        # Viewport
        glViewport(0,0,self.width,self.height)
        
        # Initialize
        glClearColor(*self.clearColor)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_NORMALIZE)
        glShadeModel(GL_FLAT)

    def prepare_screen_GL(self):
        """Prepare drawing on screen
        """
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0,self.width,self.height,0,-1,1);
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
       
    def display(self):
        """Do drawing of objects in world"""
        pass

    def display_screen(self):
        """Do drawing of objects on screen"""
        pass

    def save_screen(self,fn):
        """Saves a screenshot"""
        try:
            import Image
        except ImportError:
            print "Cannot save screens to disk, the Python Imaging Library is not installed"
            return
        screenshot = glReadPixels( 0,0, self.width, self.height, GL_RGBA, GL_UNSIGNED_BYTE)
        im = Image.frombuffer("RGBA", (self.width, self.height), screenshot, "raw", "RGBA", 0, 0)
        print "Saving screen to",fn
        im.save(fn)

    def draw_text(self,x,y,text,size=12,color=None):
        """If called in the display_screen method, renders text at the given x,y position.
        If size is given, it renders a font in the given size. If color is given, then it
        is an RGB or RGBA color value."""
        import ctypes
        if size <= 10:
            font = GLUT_BITMAP_HELVETICA_10
        elif size <= 12:
            font = GLUT_BITMAP_HELVETICA_12
        elif size <= 13:
            font = GLUT_BITMAP_8_BY_13
        elif size <= 16:
            font = GLUT_BITMAP_9_BY_15
        elif size <= 21:
            font = GLUT_BITMAP_HELVETICA_12
        else:
            font = GLUT_TIMES_NEW_ROMAN_24
        if color is None:
            glColor3f(0,0,0)
        elif len(color)==3:
            glColor3f(color[0],color[1],color[2])
        else:
            glColor4f(color[0],color[1],color[2],color[3])
        glRasterPos2i(x,y)
        for c in text:
            glutBitmapCharacter(font, ctypes.c_int( ord(c) ))

    def _motionfunc(self,x,y):
        """Internal use"""
        dx = x - self.lastx
        dy = y - self.lasty
        self.motionfunc(x,y,dx,dy)
        self.lastx = x
        self.lasty = y
    def _mousefunc(self,button,state,x,y):
        """Internal use"""
        self.mousefunc(button,state,x,y)
        self.lastx = x
        self.lasty = y
    
class GLNavigationProgram(GLProgram):
    """A more advanced form of GLProgram that allows you to navigate a
    camera around a 3D world.  Click-drag rotates, Control-drag translates,
    Shift-drag zooms.

    Attributes:
        - camera: an orbit camera (see :class:`orbit`)
        - fov: the camera field of view in x direction
        - clippingplanes: a pair containing the near and far clipping planes
    """
    def __init__(self,name):
        GLProgram.__init__(self,name)
        self.camera = camera.orbit()
        self.camera.dist = 6.0
        #x field of view in degrees
        self.fov = 30
        #near and far clipping planes
        self.clippingplanes = (0.2,20)
        #mouse state information
        self.modifiers = 0
        self.dragging = False
        self.clearColor = [0.8,0.8,0.9,0]        

    def get_view(self):
        """Returns a tuple describing the viewport, which could be saved to
        file."""
        return (self.width,self.height,self.camera,self.fov,self.clippingplanes)

    def set_view(self,v):
        """Sets the viewport to a tuple previously returned by get_view(),
        e.g. a prior view that was saved to file."""
        self.width,self.height,self.camera,self.fov,self.clippingplanes = v
        if self.glutInitialized:
            glutReshapeWindow(self.width,self.height)
            self.refresh()

    def viewport(self):
        """Gets a Viewport instance corresponding to the current view.
        This is used to interface with the Widget classes"""
        vp = Viewport()
        vp.x,vp.y,vp.w,vp.h = 0,0,self.width,self.height
        vp.n,vp.f = self.clippingplanes
        vp.perspective = True
        aspect = float(self.width)/float(self.height)
        rfov = self.fov*math.pi/180.0
        vp.scale = 1.0/(2.0*math.tan(rfov*0.5/aspect)*aspect)
        vp.setRigidTransform(*se3.inv(self.camera.matrix()))
        return vp

    def click_ray(self,x,y):
        """Returns a pair of 3-tuples indicating the ray source and direction
        in world coordinates for a screen-coordinate point (x,y)"""
        R,t = se3.inv(self.camera.matrix())
        #from x and y compute ray direction
        u = float(x-self.width/2)/self.width
        v = float(self.height-y-self.height/2)/self.width
        aspect = float(self.width)/float(self.height)
        rfov = self.fov*math.pi/180.0
        scale = 2.0*math.tan(rfov*0.5/aspect)*aspect
        d = (u*scale,v*scale,-1.0)
        d = vectorops.div(d,vectorops.norm(d))
        return (t,so3.apply(R,d))
    
    def prepare_GL(self):
        GLProgram.prepare_GL(self)

        # Projection
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = float(self.width)/float(self.height)
        gluPerspective (self.fov/aspect,aspect,self.clippingplanes[0],self.clippingplanes[1])

        # Initialize ModelView matrix
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        
        # View transformation
        mat = se3.homogeneous(self.camera.matrix())
        cols = zip(*mat)
        pack = sum((list(c) for c in cols),[])
        glMultMatrixf(pack)

        # Default light source
        glLightfv(GL_LIGHT0,GL_POSITION,[0,-1,2,0])
        glLightfv(GL_LIGHT0,GL_DIFFUSE,[1,1,1,1])
        glLightfv(GL_LIGHT0,GL_SPECULAR,[1,1,1,1])
        glEnable(GL_LIGHT0)

        glLightfv(GL_LIGHT1,GL_POSITION,[-1,2,1,0])
        glLightfv(GL_LIGHT1,GL_DIFFUSE,[0.5,0.5,0.5,1])
        glLightfv(GL_LIGHT1,GL_SPECULAR,[0.5,0.5,0.5,1])
        glEnable(GL_LIGHT1)

    def motionfunc(self,x,y,dx,dy):
        if self.dragging:
            if self.modifiers & GLUT_ACTIVE_CTRL:
                R,t = self.camera.matrix()
                delta = so3.apply(so3.inv(R),[float(dx)*self.camera.dist/self.width,-float(dy)*self.camera.dist/self.width,0])
                self.camera.tgt = vectorops.add(self.camera.tgt,delta)
            elif self.modifiers & GLUT_ACTIVE_SHIFT:
                self.camera.dist *= math.exp(dy*0.01)
            else:
                self.camera.rot[2] += float(dx)*0.01
                self.camera.rot[1] += float(dy)*0.01 
        self.refresh()
    
    def mousefunc(self,button,state,x,y):
        if state == 0:
            self.dragging = True
        else:
            self.dragging = False
        self.modifiers = glutGetModifiers()


class GLRealtimeProgram(GLNavigationProgram):
    """A GLNavigationProgram that refreshes the screen at a given frame rate.

    Attributes:
        - ttotal: total elapsed time assuming a constant frame rate
        - fps: the frame rate in Hz
        - dt: 1.0/fps
        - counter: a frame counter
    """
    def __init__(self,name):        
        GLNavigationProgram.__init__(self,name)
        self.ttotal = 0.0
        self.fps = 50
        self.dt = 1.0/self.fps
        self.running = True
        self.counter = 0
        self.lasttime = time.time()

    # idle callback
    def idlefunc (self):
        t = self.dt - (time.time() - self.lasttime)
        if (t > 0):
            time.sleep(t)
        
        self.ttotal += self.dt
        self.counter += 1

        #do something random
        self.idle()
        
        self.lasttime = time.time()
        glutPostRedisplay()

    def idle(self):
        pass

class GLPluginProgram(GLRealtimeProgram):
    """This base class should be used with a GLPluginBase object to handle the
    GUI functionality (see glcommon.py.  Call setPlugin() on this object to set
    the currently used plugin.  pushPlugin()/popPlugin() can also be used to
    set a hierarchy of plugins."""
    def __init__(self,name="GLWidget"):
        GLRealtimeProgram.__init__(self,name)
        self.plugins = []
    def setPlugin(self,plugin):
        for p in self.plugins:
            p.window = None
        self.plugins = []
        if plugin:
            self.pushPlugin(plugin)
    def pushPlugin(self,plugin):
        self.plugins.append(plugin)
        plugin.window = self
        plugin.reshapefunc(self.width,self.height)
        self.refresh()
    def popPlugin(self):
        if len(self.plugins)==0: return None
        res = self.plugins[-1]
        self.plugins.pop(-1)
        res.window = None
        return res
    def initialize(self):
        for plugin in self.plugins:
            plugin.initialize()
        GLRealtimeProgram.initialize(self)
    def reshapefunc(self,w,h):
        for plugin in self.plugins[::-1]:
            if plugin.reshapefunc(w,h): return
        GLRealtimeProgram.reshapefunc(self,w,h)
    def keyboardfunc(self,c,x,y):
        for plugin in self.plugins[::-1]:
            if plugin.keyboardfunc(c,x,y): return
        GLRealtimeProgram.keyboardfunc(self,c,x,y)
    def keyboardupfunc(self,c,x,y):
        for plugin in self.plugins[::-1]:
            if plugin.keyboardupfunc(c,x,y): return
        GLRealtimeProgram.keyboardupfunc(self,c,x,y)
    def specialfunc(self,c,x,y):
        for plugin in self.plugins[::-1]:
            if plugin.specialfunc(c,x,y): return
        GLRealtimeProgram.specialfunc(self,c,x,y)
    def specialupfunc(self,c,x,y):
        for plugin in self.plugins[::-1]:
            if plugin.specialupfunc(c,x,y): return
        GLRealtimeProgram.specialupfunc(self,c,x,y)
    def motionfunc(self,x,y,dx,dy):
        for plugin in self.plugins[::-1]:
            if plugin.motionfunc(x,y,dx,dy): return
        GLRealtimeProgram.motionfunc(self,x,y,dx,dy)
    def mousefunc(self,button,state,x,y):
        for plugin in self.plugins[::-1]:
            if plugin.mousefunc(button,state,x,y): return
        GLRealtimeProgram.mousefunc(self,button,state,x,y)
    def idlefunc(self):
        for plugin in self.plugins:
            if plugin.idlefunc(): return
        GLRealtimeProgram.idlefunc(self)
    def display(self):
        for plugin in self.plugins:
            if plugin.display(): return
        GLRealtimeProgram.display(self)
    def display_screen(self):
        for plugin in self.plugins:
            if plugin.display_screen(): return
        GLRealtimeProgram.display_screen(self)
