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

class GLViewport:
    """
    A class describing an OpenGL camera view.
        - x,y: upper left hand corner of the view in the OpenGL canvas, in screen pixels
        - w,h: width and height of the view, in screen pixels
        - screenDeviceScale: if not 1, multiply screen pixel coordinates by this to get
          openGL pixel coordinates (usually Mac Retina displays)
        - orthogonal: if true, does an orthogonal projection. (Not supported)
        - camera: an orbit camera (see :class:`orbit`)
        - fov: the camera field of view in x direction
        - clippingplanes: a pair containing the near and far clipping planes
    """
    def __init__(self):
        self.orthogonal = False
        self.x,self.y = 0,0
        self.w,self.h = 640,480
        self.screenDeviceScale = 1
        self.camera = camera.orbit()
        self.camera.dist = 6.0
        #x field of view in degrees
        self.fov = 30
        #near and far clipping planes
        self.clippingplanes = (0.2,20)

    def contains(self,x,y):
        return x >= self.x and y >= self.y and x < self.x + self.w and y < self.y + self.h

    def fit(self,center,radius):
        """Fits the viewport to an object filling a sphere of a certain center
        and radius"""
        self.camera.tgt = center
        self.camera.dist = radius*2
        zmin,zmax = self.clippingplanes
        if radius < self.clippingplanes[0]:
            zmin = radius*0.5
        if radius*3 > self.clippingplanes[1]:
            zmax =radius*3.5
        self.clippingplanes = (zmin,zmax)

    def toViewport(self):
        """Returns a Klampt C++ Viewport() instance corresponding to this view.
        This is used to interface with the Widget classes"""
        vp = Viewport()
        vp.x,vp.y,vp.w,vp.h = self.x,self.y,self.w,self.h
        vp.n,vp.f = self.clippingplanes
        vp.perspective = True
        aspect = float(self.w)/float(self.h)
        rfov = self.fov*math.pi/180.0
        vp.scale = 1.0/(2.0*math.tan(rfov*0.5/aspect)*aspect)
        vp.setRigidTransform(*self.camera.matrix())
        return vp

    def click_ray(self,x,y):
        """Returns a pair of 3-tuples indicating the ray source and direction
        in world coordinates for a screen-coordinate point (x,y)"""
        R,t = self.camera.matrix()
        #from x and y compute ray direction
        u = float(x-(self.x + self.w/2))/self.w
        v = float((self.y + self.h/2) -y)/self.w
        aspect = float(self.w)/float(self.h)
        rfov = self.fov*math.pi/180.0
        scale = 2.0*math.tan(rfov*0.5/aspect)*aspect
        d = (u*scale,v*scale,-1.0)
        d = vectorops.div(d,vectorops.norm(d))
        return (t,so3.apply(R,d))

    def project(self,pt,clip=True):
        """Given a point in world space, returns the (x,y,z) coordinates of the projected
        pixel.  z is given in absolute coordinates, while x,y are given in pixel values.

        If clip=True and the point is out of the viewing volume, then None is returned.
        Otherwise, if the point is exactly at the focal plane then the middle of the viewport
        is returned.
        """
        ploc = se3.apply(se3.inv(self.camera.matrix()),pt)
        if clip:
            if -ploc[2] <= self.clippingplanes[0] or -ploc[2] >= self.clippingplanes[1]:
                return None
        if abs(ploc[2]) < 1e-8:
            return (self.x+self.w/2,self.y+self.h/2)
        #d = (u*scale,v*scale,-1.0)
        #ploc.x = ploc.z*d.x
        #ploc.y = ploc.z*d.y
        aspect = float(self.w)/float(self.h)
        rfov = self.fov*math.pi/180.0
        scale = 2.0*math.tan(rfov*0.5/aspect)*aspect
        u = -ploc[0]/(ploc[2]*scale)
        v = -ploc[1]/(ploc[2]*scale)
        if clip and (abs(u) > 0.5 or abs(v) > 0.5):
            return None
        x = u*self.w + (self.x + self.w/2)
        y = (self.y + self.h/2) - v*self.w
        return (x,y,-ploc[2])

    def setCurrentGL(self):
        """Sets up the view in the current OpenGL context"""
        # Projection
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = float(self.w)/float(self.h)
        gluPerspective (self.fov/aspect,aspect,self.clippingplanes[0],self.clippingplanes[1])

        # Initialize ModelView matrix
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        
        # View transformation
        mat = se3.homogeneous(se3.inv(self.camera.matrix()))
        cols = zip(*mat)
        pack = sum((list(c) for c in cols),[])
        glMultMatrixf(pack)

class GLProgramAction:
    def __init__(self,hook,short_text,key,description=None):
        self.hook = hook
        self.short_text = short_text
        self.key = key
        self.description = description
        if description == None:
            self.description = short_text

class GLProgram:
    """A basic OpenGL visualization, run as part of some _GLBackend.
    For the most part there is a one-to-one correspondence and the
    backend just relays the input / drawing messages

    Assumes that glinit.py has been imported to define _GLBackend.

    Attributes:
        - name: title of the window (only has an effect before calling
          run())
        - window: the QtBackend or GLUTBackend instance
        - view: GLViewport instance.  If this is provided to an empty _GLBackend
          window, the w,h gives a hint to the size of the window.  It is then updated
          by the user and setting the viewport size has no effect on the window.
        - clearColor: the RGBA floating point values of the background color.
        - glutInitialized: true if GLUT has been initialized
    """
    def __init__(self,name="OpenGL Program"):
        global _GLBackend
        self.window = None
        self.name = name
        self.view = GLViewport()
        self.clearColor = [1.0,1.0,1.0,0.0]
        self.actions = []

    def add_action(self,hook,short_text,key,description=None):
        """Defines a new generic GUI action.  The action will be available in a menu in
        Qt or as keyboard commands in GLUT."""
        self.actions.append(GLProgramAction(hook,short_text,key,description))

    def run(self):
        """Starts a new event loop with this object as the main program.
        Note: might not return, in the case of GLUT.
        """
        import visualization
        visualization.setWindowTitle(self.name)
        visualization.run(self)

    def initialize(self):
        """Called after the GL context is initialized, but before main loop.
        May be overridden.  Users should not call this directly!"""
        assert self.window != None
        for a in self.actions:
            self.window.add_action(a.hook,a.short_text,a.key,a.description)
        return True

    def refresh(self):
        """Call this to redraw the screen on the next event loop"""
        self.window.refresh()

    def modifiers(self):
        """Retrieves a list of currently pressed keyboard modifiers.
        Values can be any combination of 'ctrl', 'shift', 'alt'.
        """
        return self.window.modifiers()

    def reshape(self,w,h):
        """Asks to resize the GL window"""
        if self.window:
            return self.window.reshape(w,h)
        else:
            self.view.w,self.view.h = w,h

    def reshapefunc(self,w,h):
        """Called on window resize.  May be overridden."""
        self.view.w = w
        self.view.h = h
        self.refresh()
        return True

    def print_help(self):
        #Put your help printouts here
        print "************** Help **************"
        print "?: print this help message"
        for a in self.actions:
            print a.key,":",a.description
        print "**********************************"
        
    def keyboardfunc(self,c,x,y):
        """Called on keypress down. May be overridden.  c is either the ASCII/unicode
        character of the key pressed or a string describing the character (up,down,left,right,
        home,end,delete,enter,f1,...,f12)"""
        if c == '?':
            self.print_help()
            return True
        if 'alt' in self.modifiers():
            c = 'Alt+'+c
        if 'ctrl' in self.modifiers():
            c = 'Ctrl+'+c
        for a in self.actions:
            if c == a.key:
                a.hook()
                self.refresh()
                return True
        return False
    def keyboardupfunc(self,c,x,y):
        """Called on keyboard up (if your system allows it). May be overridden."""
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
        if self.view.w == 0 or self.view.h == 0:
            #hidden?
            print "GLProgram.displayfunc called on hidden window?"
            return False
        self.prepare_GL()
        self.display()
        self.prepare_screen_GL()
        self.display_screen()
        return True
        
    def idlefunc(self):
        """Called on idle.  Default value stops all additional idle calls.  Must be
        overridden if you want to do something in the idle loop."""
        #print "Sleeping idle from",self.__class__.__name__
        self.idlesleep()

    def idlesleep(self,duration=float('inf')):
        """Sleeps the idle callback for t seconds.  If t is not provided,
        the idle callback is slept forever"""
        self.window.idlesleep(duration)

    def prepare_GL(self):
        """Prepare drawing in world coordinate frame
        """
        # Viewport
        view = self.view
        ydevice = (self.window.height - view.y - view.h)
        glViewport(view.x*view.screenDeviceScale,ydevice*view.screenDeviceScale,view.w*view.screenDeviceScale,view.h*view.screenDeviceScale)
        
        # Initialize
        glClearColor(*self.clearColor)
        glScissor(view.x*view.screenDeviceScale,ydevice*view.screenDeviceScale,view.w*view.screenDeviceScale,view.h*view.screenDeviceScale)
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
        glOrtho(0,self.view.w*self.view.screenDeviceScale,self.view.h*self.view.screenDeviceScale,0,-1,1);
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
       
    def display(self):
        """Do drawing of objects in world"""
        return True

    def display_screen(self):
        """Do drawing of objects on screen"""
        return True

    def closefunc(self):
        """Called by the window when it is closed"""
        return True

    def save_screen(self,fn,multithreaded=True):
        """Saves a screenshot"""
        try:
            from PIL import Image
        except ImportError:
            try:
                import Image
            except ImportError:
                print "Cannot save screens to disk, the Python Imaging Library is not installed"
                return
        if hasattr(self.window,'makeCurrent'):
            self.window.makeCurrent()
        glReadBuffer(GL_FRONT);
        x,y,w,h = self.view.x*self.view.screenDeviceScale,self.view.y*self.view.screenDeviceScale,self.view.w*self.view.screenDeviceScale,self.view.h*self.view.screenDeviceScale
        screenshot = glReadPixels( x, y, w, h, GL_RGBA, GL_UNSIGNED_BYTE)
        im = Image.frombuffer("RGBA", (w, h), screenshot, "raw", "RGBA", 0, 0)
        print "Saving screen to",fn
        if not multithreaded:
            im.save(fn)
        else:
            import threading
            def func(im,fn):
                im.save(fn)
            th = threading.Thread(target=func,args=(im,fn))
            th.start()


    def draw_text(self,point,text,size=12,color=None):
        self.window.draw_text(point,text,size,color)

    
class GLNavigationProgram(GLProgram):
    """A more advanced form of GLProgram that allows you to navigate a
    camera around a 3D world.  Click-drag rotates, Control-drag translates,
    Shift-drag zooms.
    """
    def __init__(self,name):
        GLProgram.__init__(self,name)
        #mouse state information
        self.dragging = False
        self.clearColor = [0.8,0.8,0.9,0]        

    def get_view(self):
        """Returns a GLViewport describing the viewport, which could be saved to
        file."""
        return self.view

    def set_view(self,v):
        """Sets the viewport to a tuple previously returned by get_view(),
        e.g. a prior view that was saved to file."""
        self.view = v
        self.reshape(self.view.w,self.view.h)

    
    def prepare_GL(self):
        GLProgram.prepare_GL(self)

        self.view.setCurrentGL()

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
                R,t = self.view.camera.matrix()
                aspect = float(self.view.w)/self.view.h
                rfov = self.view.fov*math.pi/180.0
                scale = 2.0*math.tan(rfov*0.5/aspect)*aspect
                delta = so3.apply(R,[-scale*float(dx)*self.view.camera.dist/self.view.w,scale*float(dy)*self.view.camera.dist/self.view.w,0])
                self.view.camera.tgt = vectorops.add(self.view.camera.tgt,delta)
            elif 'shift' in self.modifiers():
                self.view.camera.dist *= math.exp(dy*0.01)
            else:
                self.view.camera.rot[2] -= float(dx)*0.01
                self.view.camera.rot[1] -= float(dy)*0.01 
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
        tcur = time.time()
        tsleep = self.dt - (tcur - self.lasttime)
        if tsleep > 0.001:
            #print "Elapsed time",tcur-self.lasttime,"sleep",tsleep,"window",self.window.name
            self.idlesleep(tsleep)
            return
        
        self.ttotal += self.dt
        self.counter += 1

        #do something random
        self.idle()
        
        self.lasttime = tcur
        self.refresh()
        return True

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
        import copy
        for p in self.plugins:
            p.window = None
            p.view = copy.copy(p.view)
        self.plugins = []
        if plugin:
            self.pushPlugin(plugin)
    def pushPlugin(self,plugin):
        self.plugins.append(plugin)
        plugin.window = self.window
        if self.window:
            if self.window.initialized:
                print "GLPluginProgram.pushPlugin called after window was initialized, some actions may not be available"
            plugin.view = self.view
            plugin.reshapefunc(self.view.w,self.view.h)
            self.refresh()
        elif len(self.plugins) == 1 and hasattr(plugin,'view') and plugin.view != None:
            self.view = plugin.view
        else:
            plugin.view = self.view
    def popPlugin(self):
        if len(self.plugins)==0: return None
        res = self.plugins[-1]
        self.plugins.pop(-1)
        res.window = None
        if self.window:
            self.refresh()
        return res
    def set_view(self,v):
        GLRealtimeProgram.set_view(self,v)
        for p in self.plugins:
            p.view = self.view
    def initialize(self):
        #print "GLPluginProgram initialize:",len(self.plugins),"plugins"
        for plugin in self.plugins:
            plugin.window = self.window
            if not plugin.initialize():
                print "GLPluginProgram.initialize(): Plugin of type",plugin.__class__.__name__,"Did not initialize"
                return False
            if hasattr(plugin,'actions'):
                #print "Adding",len(plugin.actions),"actions for plugin",plugin.__class__.__name__
                for a in plugin.actions:
                    self.add_action(*a)
        return GLRealtimeProgram.initialize(self)
    def idle(self):
        anyhandled = False
        for plugin in self.plugins:
            if hasattr(plugin,'idle') and plugin.idle():
                anyhandled = True
        if not anyhandled:
            return False
        return True
    def reshapefunc(self,w,h):
        GLRealtimeProgram.reshapefunc(self,w,h)
        for plugin in self.plugins:
            plugin.reshapefunc(w,h)
        return
    def keyboardfunc(self,c,x,y):
        for plugin in self.plugins[::-1]:
            if plugin.keyboardfunc(c,x,y): return True
        return GLRealtimeProgram.keyboardfunc(self,c,x,y)
    def keyboardupfunc(self,c,x,y):
        for plugin in self.plugins[::-1]:
            if plugin.keyboardupfunc(c,x,y): return True
        return GLRealtimeProgram.keyboardupfunc(self,c,x,y)
    def motionfunc(self,x,y,dx,dy):
        for plugin in self.plugins[::-1]:
            if plugin.motionfunc(x,y,dx,dy): return True
        return GLRealtimeProgram.motionfunc(self,x,y,dx,dy)
    def mousefunc(self,button,state,x,y):
        for plugin in self.plugins[::-1]:
            if plugin.mousefunc(button,state,x,y): return True
        return GLRealtimeProgram.mousefunc(self,button,state,x,y)
    def displayfunc(self):
        for plugin in self.plugins[::-1]:
            if plugin.displayfunc(): 
                break
        GLRealtimeProgram.displayfunc(self)
    def display(self):
        for plugin in self.plugins:
            if plugin.display(): return True
        return GLRealtimeProgram.display(self)
    def display_screen(self):
        for plugin in self.plugins:
            if plugin.display_screen(): return True
        return GLRealtimeProgram.display_screen(self)
