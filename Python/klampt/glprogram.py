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

_currentObject = None
def _reshapefunc(w,h):
    return _currentObject.reshapefunc(w,h)
def _keyboardfunc(c,x,y):
    return _currentObject.keyboardfunc(c,x,y)
def _keyboardupfunc(c,x,y):
    return _currentObject.keyboardupfunc(c,x,y)
def _specialfunc(c,x,y):
    return _currentObject.specialfunc(c,x,y)
def _specialupfunc(c,x,y):
    return _currentObject.specialupfunc(c,x,y)
def _motionfunc(x,y):
    return _currentObject.motionfunc(x,y)
def _mousefunc(button,state,x,y):
    return _currentObject.mousefunc(button,state,x,y)
def _displayfunc():
    return _currentObject.displayfunc()
def _idlefunc():
    return _currentObject.idlefunc()


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
    """
    def __init__(self,name="OpenGL Program"):
        self.name = name
        self.width = 640
        self.height = 480
        self.clearColor = [1.0,1.0,1.0,0.0]

    def run(self):
        """Starts the main loop"""
        global _currentObject
        # Initialize Glut
        glutInit ([])
        # Open a window
        glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH)

        x = 0
        y = 0
        glutInitWindowPosition (x, y);
        glutInitWindowSize (self.width, self.height);
        glutCreateWindow (self.name)
  
        # keyboard callback
        _currentObject = self
        glutReshapeFunc (_reshapefunc)
        glutKeyboardFunc (_keyboardfunc)
        glutKeyboardUpFunc (_keyboardupfunc)
        glutSpecialFunc (_specialfunc)
        glutSpecialUpFunc (_specialupfunc)
        glutMotionFunc (_motionfunc)
        glutMouseFunc (_mousefunc)
        glutDisplayFunc (_displayfunc)
        glutIdleFunc(_idlefunc)

        #init function
        self.initialize()
        glutMainLoop ()

    def initialize(self):
        """Called after GLUT is initialized, but before main loop.
        May be overridden."""
        glutPostRedisplay()
        pass

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
    def motionfunc(self,x,y):
        """Called when the mouse moves on screen.  May be overridden."""
        pass
    def mousefunc(self,button,state,x,y):
        """Called when the mouse is clicked.  May be overridden."""
        pass
    
    def displayfunc(self):
        """All OpenGL calls go here.  May be overridden, although you
        may wish to override display() and display_screen() instead."""
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
        import Image
        screenshot = glReadPixels( 0,0, self.width, self.height, GL_RGBA, GL_UNSIGNED_BYTE)
        im = Image.frombuffer("RGBA", (self.width, self.height), screenshot, "raw", "RGBA", 0, 0)
        print "Saving screen to",fn
        im.save(fn)

    
class GLNavigationProgram(GLProgram):
    """A more advanced form of GLProgram that allows you to navigate a
    camera around a 3D world.  Click-drag rotates, Control-drag translates,
    Shift-drag zooms.

    Attributes:
        - camera: an orbit camera (see :class:`orbit`)
        - fov: the camera field of view
        - clippingplanes: a pair containing the near and far clipping planes
    """
    def __init__(self,name):
        GLProgram.__init__(self,name)
        self.camera = camera.orbit()
        self.camera.dist = 10.0
        #field of view in degrees
        self.fov = 30
        #near and far clipping planes
        self.clippingplanes = (0.2,20)
        #mouse state information
        self.lastx = 0
        self.lasty = 0
        self.modifiers = 0
        self.dragging = False
        self.clearColor = [0.8,0.8,0.9,0]        

    def get_view(self):
        return (self.width,self.height,self.camera,self.fov,self.clippingplanes)

    def set_view(self,v):
        self.width,self.height,self.camera,self.fov,self.clippingplanes = v
        glutReshapeWindow(self.width,self.height)
        glutPostRedisplay()

    def click_ray(self,x,y):
        """Returns a pair of 3-tuples indicating the ray source and direction
        in world coordinates for a screen-coordinate point (x,y)"""
        R,t = se3.inv(self.camera.matrix())
        #from x and y compute ray direction
        u = float(x-self.width/2)
        v = float(self.height-y-self.height/2)
        scale = math.tan(self.fov*math.pi/180.0)/self.height
        #HACK: I don't know why this seems to work!
        scale *= 0.925
        d = (u*scale,v*scale,-1.0)
        d = vectorops.div(d,vectorops.norm(d))
        return (t,so3.apply(R,d))
    
    def prepare_GL(self):
        GLProgram.prepare_GL(self)

        # Projection
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective (self.fov,float(self.width)/float(self.height),self.clippingplanes[0],self.clippingplanes[1])

        # Initialize ModelView matrix
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        
        # View transformation
        mat = se3.homogeneous(self.camera.matrix())
        cols = zip(*mat)
        pack = sum((list(c) for c in cols),[])
        glMultMatrixf(pack)

        # Light source
        glLightfv(GL_LIGHT0,GL_POSITION,[0,-1,2,0])
        glLightfv(GL_LIGHT0,GL_DIFFUSE,[1,1,1,1])
        glLightfv(GL_LIGHT0,GL_SPECULAR,[1,1,1,1])
        glEnable(GL_LIGHT0)

        glLightfv(GL_LIGHT1,GL_POSITION,[-1,2,1,0])
        glLightfv(GL_LIGHT1,GL_DIFFUSE,[0.5,0.5,0.5,1])
        glLightfv(GL_LIGHT1,GL_SPECULAR,[0.5,0.5,0.5,1])
        glEnable(GL_LIGHT1)

    def motionfunc(self,x,y):
        dx = x - self.lastx
        dy = y - self.lasty
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
        self.lastx = x
        self.lasty = y
        glutPostRedisplay()
    
    def mousefunc(self,button,state,x,y):
        if state == 0:
            self.dragging = True
        else:
            self.dragging = False
        self.modifiers = glutGetModifiers()
        self.lastx = x
        self.lasty = y


class GLRealtimeProgram(GLNavigationProgram):
    """A GLNavigationProgram that refreshes the screen at a given frame rate.

    Attributes:
        - ttotal: total elapsed time
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
