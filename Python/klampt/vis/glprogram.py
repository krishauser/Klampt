"""This module defines convenient classes for building 3D GUI programs.

- GLProgram takes care of basic user input.
- GLNavigationProgram allows 3D navigation with the mouse.
- GLRealtimeProgram calls a subclass-defined idle() function roughly on a
  constant time step.
"""

from OpenGL.GL import *
from OpenGL.GLU import *
import camera
from ..math import so3,se3,vectorops
from ..robotsim import Viewport
import math
import time


class GLProgram:
    """A basic OpenGL program using some _GLBackend.  Set up your window parameters,
    then call run() to create a new window with this program.

    Assumes that glinit.py has been imported to define _GLBackend.

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
        global _GLBackend
        self.window = None
        self.name = name
        self.x,self.y = 0,0
        self.width = 640
        self.height = 480
        self.clearColor = [1.0,1.0,1.0,0.0]
        self.glutInitialized = False
        self.lastx = 0
        self.lasty = 0

    def run(self):
        """Starts a new event loop with this object as the main program.
        Note: might not return, in the case of GLUT.
        """
        import visualization
        visualization.run(self)

    def initialize(self):
        """Called after the GL context is initialized, but before main loop.
        May be overridden.  Users should not call this directly!"""
        return True

    def refresh(self):
        """Call this to redraw the screen on the next event loop"""
        self.window.refresh()

    def modifiers(self):
        """Retrieves a list of currently pressed keyboard modifiers.
        Values can be any combination of 'ctrl', 'shift', 'alt'.
        """
        return self.window.modifiers()

    def reshapefunc(self,w,h):
        """Called on window resize.  May be overridden."""
        assert isinstance(self.x,int) and isinstance(self.y,int)
        self.width = w
        self.height = h
        self.refresh()
        return True
        
    def keyboardfunc(self,c,x,y):
        """Called on keypress down. May be overridden."""
        return False
    def keyboardupfunc(self,c,x,y):
        """Called on keyboard up (if your system allows it). May be overridden."""
        return False
    def specialfunc(self,c,x,y):
        """Called on special character keypress down.  May be overridden"""
        return False
    def specialupfunc(self,c,x,y):
        """Called on special character keypress up (if your system allows
        it).  May be overridden"""
        return False

    def motionfunc(self,x,y,dx,dy):
        """Called when the mouse moves on screen.  May be overridden."""
        return False
    def mousefunc(self,button,state,x,y):
        """Called when the mouse is clicked.  May be overridden."""
        return False
    
    def displayfunc(self):
        """All OpenGL calls go here.  May be overridden, although you
        may wish to override display() and display_screen() instead."""
        if self.width == 0 or self.height == 0:
            #hidden?
            print "GLProgram.displayfunc called on hidden window?"
            return False
        self.prepare_GL()
        self.display()
        self.prepare_screen_GL()
        self.display_screen()
        return True
        
    def idlefunc(self):
        """Called on idle.  May be overridden."""
        self.idlesleep()

    def idlesleep(self,duration=float('inf')):
        """Sleeps the idle callback for t seconds.  If t is not provided,
        the idle callback is slept forever"""
        self.window.idlesleep(duration)

    def prepare_GL(self):
        """Prepare drawing in world coordinate frame
        """
        # Viewport
        glViewport(self.x,self.y,self.width,self.height)
        
        # Initialize
        glClearColor(*self.clearColor)
        glScissor(self.x,self.y,self.width,self.height);
        glEnable(GL_SCISSOR_TEST);
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
        return True

    def display_screen(self):
        """Do drawing of objects on screen"""
        return True

    def closefunc(self):
        """Called when window is closed"""
        return True

    def save_screen(self,fn):
        """Saves a screenshot"""
        try:
            import Image
        except ImportError:
            print "Cannot save screens to disk, the Python Imaging Library is not installed"
            return
        screenshot = glReadPixels( self.x, self.y, self.width, self.height, GL_RGBA, GL_UNSIGNED_BYTE)
        im = Image.frombuffer("RGBA", (self.width, self.height), screenshot, "raw", "RGBA", 0, 0)
        print "Saving screen to",fn
        im.save(fn)

    def draw_text(self,point,text,size=12,color=None):
        self.window.draw_text(point,text,size,color)

    
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
        self.dragging = False
        self.clearColor = [0.8,0.8,0.9,0]        

    def get_view(self):
        """Returns a tuple describing the viewport, which could be saved to
        file."""
        return (self.x,self.y,self.width,self.height,self.camera,self.fov,self.clippingplanes)

    def set_view(self,v):
        """Sets the viewport to a tuple previously returned by get_view(),
        e.g. a prior view that was saved to file."""
        self.x,self.y,self.width,self.height,self.camera,self.fov,self.clippingplanes = v
        self.reshape(self.width,self.height)

    def viewport(self):
        """Gets a Viewport instance corresponding to the current view.
        This is used to interface with the Widget classes"""
        vp = Viewport()
        vp.x,vp.y,vp.w,vp.h = self.x,self.y,self.width,self.height
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
        u = float(x-(self.x + self.width/2))/self.width
        v = float((self.y + self.height/2) -y)/self.width
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
            if 'ctrl' in self.modifiers():
                R,t = self.camera.matrix()
                delta = so3.apply(so3.inv(R),[float(dx)*self.camera.dist/self.width,-float(dy)*self.camera.dist/self.width,0])
                self.camera.tgt = vectorops.add(self.camera.tgt,delta)
            elif 'shift' in self.modifiers():
                self.camera.dist *= math.exp(dy*0.01)
            else:
                self.camera.rot[2] += float(dx)*0.01
                self.camera.rot[1] += float(dy)*0.01 
            self.refresh()
            return True
        return False
    
    def mousefunc(self,button,state,x,y):
        if button == 0:
            if state == 0:
                self.dragging = True
            else:
                self.dragging = False
            return True
        return False


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
        self.refresh()

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
        plugin.window = self.window
        plugin.reshapefunc(self.width,self.height)
        if self.window:
            self.refresh()
    def popPlugin(self):
        if len(self.plugins)==0: return None
        res = self.plugins[-1]
        self.plugins.pop(-1)
        res.window = None
        if self.window:
            self.refresh()
        return res
    def initialize(self):
        for plugin in self.plugins:
            plugin.window = self.window
            if not plugin.initialize():
                return False
        return GLRealtimeProgram.initialize(self)
    def reshapefunc(self,w,h):
        for plugin in self.plugins[::-1]:
            if plugin.reshapefunc(w,h): return True
        return GLRealtimeProgram.reshapefunc(self,w,h)
    def keyboardfunc(self,c,x,y):
        for plugin in self.plugins[::-1]:
            if plugin.keyboardfunc(c,x,y): return True
        return GLRealtimeProgram.keyboardfunc(self,c,x,y)
    def keyboardupfunc(self,c,x,y):
        for plugin in self.plugins[::-1]:
            if plugin.keyboardupfunc(c,x,y): return True
        return GLRealtimeProgram.keyboardupfunc(self,c,x,y)
    def specialfunc(self,c,x,y):
        for plugin in self.plugins[::-1]:
            if plugin.specialfunc(c,x,y): return True
        return GLRealtimeProgram.specialfunc(self,c,x,y)
    def specialupfunc(self,c,x,y):
        for plugin in self.plugins[::-1]:
            if plugin.specialupfunc(c,x,y): return True
        return GLRealtimeProgram.specialupfunc(self,c,x,y)
    def motionfunc(self,x,y,dx,dy):
        for plugin in self.plugins[::-1]:
            if plugin.motionfunc(x,y,dx,dy): return True
        return GLRealtimeProgram.motionfunc(self,x,y,dx,dy)
    def mousefunc(self,button,state,x,y):
        for plugin in self.plugins[::-1]:
            if plugin.mousefunc(button,state,x,y): return True
        return GLRealtimeProgram.mousefunc(self,button,state,x,y)
    def idlefunc(self):
        for plugin in self.plugins:
            if plugin.idlefunc(): return
        return GLRealtimeProgram.idlefunc(self)
    def displayfunc(self):
        for plugin in self.plugins:
            if plugin.displayfunc(): return True
        return GLRealtimeProgram.displayfunc(self)
    def display(self):
        for plugin in self.plugins:
            if plugin.display(): return True
        return GLRealtimeProgram.display(self)
    def display_screen(self):
        for plugin in self.plugins:
            if plugin.display_screen(): return True
        return GLRealtimeProgram.display_screen(self)
