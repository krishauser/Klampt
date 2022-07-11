"""Internal classes for building 3D GUI programs.

- GLProgram takes care of basic user input.
- GLNavigationProgram allows 3D navigation with the mouse.
- GLRealtimeProgram calls a subclass-defined idle() function roughly on a
  constant time step.
"""

from . import glinit
from OpenGL import GL
from .glviewport import GLViewport
from ..math import so3,se3,vectorops
import math
import time
import warnings

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
        name (str): title of the window (only has an effect before calling
            run())
        window: the QtBackend or GLUTBackend instance
        view (GLViewport): describes the OpenGL viewport.  If this is provided to an
            empty _GLBackend window, the w,h gives a hint to the size of the window. 
            It is then updated by the user and setting the viewport size has no effect on the window.
        clearColor (list of 4 floats): the RGBA floating point values of the background color.
        actions (list of GLProgramAction): the list of actions.  Must be populated using
            add_action before init().
    """
    def __init__(self,name="OpenGL Program"):
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
        from . import visualization
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
        print("************** Help **************")
        print("?: print this help message")
        for a in self.actions:
            print(a.key,":",a.description)
        print("**********************************")
        
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
            print("GLProgram.displayfunc called on hidden window?")
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
        GL.glViewport(view.x*view.screenDeviceScale,ydevice*view.screenDeviceScale,view.w*view.screenDeviceScale,view.h*view.screenDeviceScale)
        
        # Initialize
        GL.glClearColor(*self.clearColor)
        GL.glScissor(view.x*view.screenDeviceScale,ydevice*view.screenDeviceScale,view.w*view.screenDeviceScale,view.h*view.screenDeviceScale)
        GL.glEnable(GL.GL_SCISSOR_TEST);
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT);
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_LIGHTING)
        GL.glEnable(GL.GL_NORMALIZE)
        GL.glShadeModel(GL.GL_FLAT)

    def prepare_screen_GL(self):
        """Prepare drawing on screen
        """
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        GL.glOrtho(0,self.view.w*self.view.screenDeviceScale,self.view.h*self.view.screenDeviceScale,0,-1,1);
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()
       
    def display(self):
        """Do drawing of objects in world"""
        return True

    def display_screen(self):
        """Do drawing of objects on screen"""
        return True

    def closefunc(self):
        """Called by the window when it is closed"""
        return True

    def get_screen(self,format='auto',want_depth=False):
        """Retrieves a screenshot"""
        if hasattr(self.window,'makeCurrent'):
            self.window.makeCurrent()
        GL.glReadBuffer(GL.GL_FRONT);
        x,y,w,h = self.view.x*self.view.screenDeviceScale,self.view.y*self.view.screenDeviceScale,self.view.w*self.view.screenDeviceScale,self.view.h*self.view.screenDeviceScale
        screenshot = GL.glReadPixels( x, y, w, h, GL.GL_RGB, GL.GL_UNSIGNED_BYTE)
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
            rgb = np.frombuffer(screenshot,dtype=np.uint8).reshape((h,w,3))
            rgb = np.flip(rgb,0)
        elif format == 'Image':
            from PIL import Image
            rgb = Image.frombuffer("RGB", (w, h), screenshot, "raw", "RGB", 0, 0)
            rgb = rgb.transpose(Image.FLIP_TOP_BOTTOM)
        else:
            rgb = (w,h,screenshot)
        if want_depth:
            n,f = self.view.clippingplanes
            depthdata = GL.glReadPixels( x, y, w, h, GL.GL_DEPTH_COMPONENT, GL.GL_FLOAT)
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
        else:
            return rgb

    def save_screen(self,fn,multithreaded=True):
        """Saves a screenshot"""
        try:
            from PIL import Image
        except ImportError:
            try:
                import Image
            except ImportError:
                warnings.warn("Cannot save screens to disk, the Python Imaging Library is not installed")
                return
        im = self.get_screen('Image')
        print("Saving screen to",fn)
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
        if v.h != self.view.h or v.w != self.view.w:
            #minimize possible flickering?
            self.reshape(self.view.w,self.view.h)
        self.view = v
    
    def prepare_GL(self):
        """Prepares for OpenGL rendering with the current modelview matrix
        and default lights."""
        GLProgram.prepare_GL(self)
        self.view.set_current_GL()
        self.set_lights_GL()

    def set_lights_GL(self):
        """Sets the default OpenGL lights"""
        GL.glLightfv(GL.GL_LIGHT0,GL.GL_POSITION,[0,-1,2,0])
        GL.glLightfv(GL.GL_LIGHT0,GL.GL_AMBIENT,[0.05,0.05,0.05,1])
        GL.glLightfv(GL.GL_LIGHT0,GL.GL_DIFFUSE,[1,1,1,1])
        GL.glLightfv(GL.GL_LIGHT0,GL.GL_SPECULAR,[1,1,1,1])
        GL.glEnable(GL.GL_LIGHT0)

        GL.glLightfv(GL.GL_LIGHT1,GL.GL_POSITION,[-1,2,1,0])
        GL.glLightfv(GL.GL_LIGHT1,GL.GL_DIFFUSE,[0.5,0.5,0.5,1])
        GL.glLightfv(GL.GL_LIGHT1,GL.GL_SPECULAR,[0.5,0.5,0.5,1])
        GL.glEnable(GL.GL_LIGHT1)

    def motionfunc(self,x,y,dx,dy):
        if self.dragging:
            if 'ctrl' in self.modifiers():
                R,t = self.view.camera.matrix()
                aspect = float(self.view.w)/self.view.h
                rfov = math.radians(self.view.fov)
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
        ttotal (float): total elapsed time assuming a constant frame rate
        fps (float): the frame rate in Hz
        dt (float): 1.0/fps
        counter (int): a frame counter
        lasttime (float): time.time() value on the last frame.
    """
    def __init__(self,name):        
        GLNavigationProgram.__init__(self,name)
        self.ttotal = 0.0
        self.fps = 50
        self.dt = 1.0/self.fps
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
    GUI functionality (see glcommon.py).  Call setPlugin() on this object to set
    the currently used plugin.  pushPlugin()/popPlugin() can also be used to
    set a hierarchy of plugins."""
    def __init__(self,name="GLPluginProgram"):
        GLRealtimeProgram.__init__(self,name)
        self.plugins = []
    def setPlugin(self,plugin):
        warnings.warn("setPlugin will be deprecated in favor of set_plugin in a future version of Klampt",DeprecationWarning)
        return self.set_plugin(plugin)
    def pushPlugin(self,plugin):
        warnings.warn("pushPlugin will be deprecated in favor of push_plugin in a future version of Klampt",DeprecationWarning)
        return self.push_plugin(plugin)
    def popPlugin(self):
        warnings.warn("popPlugin will be deprecated in favor of pop_plugin in a future version of Klampt",DeprecationWarning)
        return self.pop_plugin()
    def set_plugin(self,plugin):
        #first, detatch existing plugins
        import copy
        for p in self.plugins:
            p.window = None
            p.view = copy.copy(p.view)
        #now just set this plugin
        self.plugins = []
        if plugin:
            self.push_plugin(plugin)
    def push_plugin(self,plugin):
        self.plugins.append(plugin)
        plugin.window = self.window
        if self.window:
            if self.window.initialized:
                print("GLPluginProgram.pushPlugin called after window was initialized, some actions may not be available")
            plugin.view = self.view
            plugin.reshapefunc(self.view.w,self.view.h)
            self.refresh()
        elif len(self.plugins) == 1 and hasattr(plugin,'view') and plugin.view != None:
            self.view = plugin.view
        else:
            plugin.view = self.view
    def pop_plugin(self):
        import copy
        if len(self.plugins)==0: return None
        res = self.plugins[-1]
        self.plugins.pop(-1)
        res.window = None
        res.view = copy.copy(res.view)
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
                warnings.warn("GLPluginProgram.initialize(): Plugin of type",plugin.__class__.__name__,"Did not initialize")
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
                return
        GLRealtimeProgram.displayfunc(self)
    def display(self):
        for plugin in self.plugins:
            if plugin.display(): return True
        return GLRealtimeProgram.display(self)
    def display_screen(self):
        for plugin in self.plugins:
            if plugin.display_screen(): return True
        return GLRealtimeProgram.display_screen(self)
