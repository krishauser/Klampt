from OpenGL.GL import *
from OpenGL.GLUT import *
import weakref
from glcommon import GLMultiProgramInterface

class GLUTWindow:
    """A GLUT window. Should not be used directly.

        Attributes:
        - name: title of the window (only has an effect before calling
          run())
        - index: the GLUT index
        - width, height: width/height of the window (only has an effect
          before calling run(), and these are updated when the user resizes
          the window.
        - clearColor: the RGBA floating point values of the background color.
        - glutInitialized: true if GLUT has been initialized
    """
    def __init__(self,name):
        """Note: must be called after GLUT is initialized."""
        self.name = name
        self.plugin = None
        self.width = 640
        self.height = 480
        self.clearColor = [1.0,1.0,1.0,0.0]
        self.lastx = 0
        self.lasty = 0
        self.modifierList = []

    def initialize(self):
        assert self.plugin != None, "plugin member needs to be set"
        plugin = self.plugin
        glutInitWindowPosition (plugin.x, plugin.y);
        glutInitWindowSize (self.width, self.height);
        glutCreateWindow (self.name)
        plugin.x = 0
        plugin.y = 0
  
        def glutsafe(func,update_modifiers=False):
            def safefunc(*args):
                if update_modifiers:
                    self._updateModifiers()
                try:
                    return func(*args)
                except Exception, e:
                    import traceback
                    traceback.print_exc()
                    glutLeaveMainLoop()
                return 
            return safefunc

        # set window callbacks
        glutReshapeFunc (glutsafe(plugin.reshapefunc))
        glutKeyboardFunc (glutsafe(plugin.keyboardfunc,update_modifiers=True))
        glutKeyboardUpFunc (glutsafe(plugin.keyboardupfunc,update_modifiers=True))
        glutSpecialFunc (glutsafe(plugin.specialfunc,update_modifiers=True))
        glutSpecialUpFunc (glutsafe(plugin.specialupfunc,update_modifiers=True))
        glutMotionFunc (glutsafe(self._motionfunc,update_modifiers=True))
        glutPassiveMotionFunc (glutsafe(self._motionfunc,update_modifiers=True))
        glutMouseFunc (glutsafe(self._mousefunc,update_modifiers=True))
        glutDisplayFunc (glutsafe(self._displayfunc))
        glutIdleFunc(glutsafe(plugin.idlefunc))
        glutCloseFunc(glutsafe(self._closefunc))

        #init function
        self.plugin.initialize()
        glEnable(GL_MULTISAMPLE)
        glutPostRedisplay()

    def setPlugin(self,plugin):
        if hasattr(plugin,'name'):
            self.name = plugin.name
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

    def modifiers(self):
        """Call this to retrieve modifiers. Called by frontend."""
        return self.modifierList

    def refresh(self):
        """Call this to redraw the screen on the next event loop. Called by frontend."""
        glutPostRedisplay()

    def idlesleep(self,duration=float('inf')):
        """Sleeps the idle callback for t seconds.  If t is not provided,
        the idle callback is slept forever. Called by frontend."""
        if duration==0:
            glutIdleFunc(_idlefunc);
        else:
            glutIdleFunc(None);
            if duration!=float('inf'):
                glutTimerFunc(duration*1000,lambda x:glutIdleFunc(_idlefunc),0);

    def reshape(self,w,h):
        """Resizes the GL window. Called by frontend."""
        print "reshaping",w,h
        self.width,self.height = w,h
        glutReshapeWindow(self.width,self.height)

    def draw_text(self,point,text,size=12,color=None):
        """If called in the display_screen method, renders text at the given point (may be 2d
        or 3d).
        If size is given, it renders a font in the given size. If color is given, then it
        is an RGB or RGBA color value.  Called by frontend."""
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
        if len(point)==3:
            glRasterPos3f(*point)
        else:
            glRasterPos2f(*point)
        for c in text:
            glutBitmapCharacter(font, ctypes.c_int( ord(c) ))

    def close(self):
        if self.index != None:
            glutDestroyWindow(self.index)
            self.index = None
            self._closefunc()

    def _updateModifiers(self):
        m = []
        modifiers = glutGetModifiers()
        if modifiers & GLUT_ACTIVE_CTRL:
            m.append('ctrl')
        elif modifiers & GLUT_ACTIVE_SHIFT:
            m.append('shift')
        elif modifiers & GLUT_ACTIVE_ALT:
            m.append('alt')
        self.modifierList = m

    def _reshapefunc(self,w,h):
        """Internal use"""
        self.width = w
        self.height = h
        self.plugin.reshapefunc(w,h)
        glutPostRedisplay()

    def _motionfunc(self,x,y):
        """Internal use"""
        dx = x - self.lastx
        dy = y - self.lasty
        self.plugin.motionfunc(x,y,dx,dy)
        self.lastx = x
        self.lasty = y

    def _mousefunc(self,button,state,x,y):
        """Internal use"""
        self.plugin.mousefunc(button,state,x,y)
        self.lastx = x
        self.lasty = y

    def _displayfunc(self):
        """Internal use."""
        if self.width == 0 or self.height == 0:
            #hidden?
            print "GLProgram.displayfunc called on hidden window?"
            return
        self.plugin.displayfunc()
        glutSwapBuffers ()

    def _closefunc(self):
        self.plugin.closefunc()
        self.plugin.window = None

class GLUTBackend:
    """A basic OpenGL program using GLUT.  Set up your GLProgramInterface class,
    call addPlugin(plugin), then call run() to start the GLUT main loop.

    NOTE: the run() call may not return depending on your GLUT system.

    For more control over windowing, you can use the createWindow function to
    construct new windows and addPlugin to add plugins to those windows.

    IMPORTANT NOTE: only one window may be created for a given world.  If you want to
    use multiple windows, then a new world should be loaded for each world.
    """
    def __init__(self):
        self.glutInitialized = False
        self.window = None

    def initialize(self,program_name):
        if self.glutInitialized == False:
            glutInit ([])
            if bool(glutSetOption):
               glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS)
            glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE)
            self.glutInitialized = True

    def createWindow(self,name):
        self.initialize()
        return GLUTWindow(name)

    def addPlugin(self,plugin,window=None):
        """ Open a window and initialize. Users should not call this
        directly!  Use the visualization.py functions instead. """
        if window == None:
            if not self.window:
                self.window = self.createWindow(plugin.name)
            window = self.window
        window.addPlugin(plugin)    

    def run(self):
        """Starts the main loop.  NOTE: if freeglut is not installed, this
        will not return."""
        # Initialize Glut
        assert self.window != None,"Need to define at least one GL interface"
        self.window.initialize()
        glutMainLoop ()

