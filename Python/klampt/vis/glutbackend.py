from OpenGL.GL import *
from OpenGL.GLUT import *
import weakref
from glcommon import GLMultiViewportProgram

keymap = {GLUT_KEY_F1:'f1',
    GLUT_KEY_F2:'f2',
    GLUT_KEY_F3:'f3',
    GLUT_KEY_F4:'f4',
    GLUT_KEY_F5:'f5',
    GLUT_KEY_F6:'f6',
    GLUT_KEY_F7:'f7',
    GLUT_KEY_F8:'f8',
    GLUT_KEY_F9:'f9',
    GLUT_KEY_F10:'f10',
    GLUT_KEY_F11:'f11',
    GLUT_KEY_F12:'f12',
    GLUT_KEY_LEFT:'left',
    GLUT_KEY_UP:'up',
    GLUT_KEY_RIGHT:'right',
    GLUT_KEY_DOWN:'down',
    GLUT_KEY_PAGE_UP:'page up',
    GLUT_KEY_PAGE_DOWN:'page down',
    GLUT_KEY_HOME:'home',
    GLUT_KEY_END:'end',
    GLUT_KEY_INSERT:'insert'
}

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
        self.program = None
        self.width = 640
        self.height = 480
        self.clearColor = [1.0,1.0,1.0,0.0]
        self.lastx = 0
        self.lasty = 0
        self.initialized = False
        self.glutWindowID = None
        self.modifierList = []

    def initialize(self):
        assert self.program != None, "program member needs to be set"
        assert not self.initialized,"initialized twice?"
        program = self.program
        #glutInitWindowPosition (0,0);
        glutInitWindowSize (self.width, self.height);
        self.glutWindowID = glutCreateWindow (self.name)
        program.view.x = 0
        program.view.y = 0
  
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
        glutReshapeFunc (glutsafe(self._reshapefunc))
        glutKeyboardFunc (glutsafe(program.keyboardfunc,update_modifiers=True))
        glutKeyboardUpFunc (glutsafe(program.keyboardupfunc,update_modifiers=True))
        glutSpecialFunc (glutsafe(self._specialfunc,update_modifiers=True))
        glutSpecialUpFunc (glutsafe(self._specialupfunc,update_modifiers=True))
        glutMotionFunc (glutsafe(self._motionfunc,update_modifiers=True))
        glutPassiveMotionFunc (glutsafe(self._motionfunc,update_modifiers=True))
        glutMouseFunc (glutsafe(self._mousefunc,update_modifiers=True))
        glutDisplayFunc (glutsafe(self._displayfunc))
        glutIdleFunc(glutsafe(program.idlefunc))
        glutCloseFunc(glutsafe(self._closefunc))

        #init function
        self.program.initialize()
        glEnable(GL_MULTISAMPLE)
        glutPostRedisplay()
        self.initialized = True
        print "Initialized"

    def add_action(self,*args):
        pass

    def setProgram(self,program):
        from glprogram import GLProgram
        assert isinstance(program,GLProgram)
        if hasattr(program,'name'):
            self.name = program.name
            if self.initialized:
                glutSetWindowTitle(program.name)
        self.program = program
        program.window = weakref.proxy(self)
        if self.initialized:
            program.initialize()
            program.reshapefunc(self.width,self.height)
            self.idlesleep(0)
        else:
            self.reshape(program.view.w,program.view.h)

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
            glutIdleFunc(self.program.idlefunc);
        else:
            glutIdleFunc(None);
            if duration!=float('inf'):
                glutTimerFunc(int(duration*1000),lambda x:glutIdleFunc(self.program.idlefunc),0);

    def reshape(self,w,h):
        """Resizes the GL window. Called by frontend."""
        print "reshaping",w,h
        self.width,self.height = w,h
        if self.initialized:
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
        self.program.reshapefunc(w,h)
        glutPostRedisplay()

    def _motionfunc(self,x,y):
        """Internal use"""
        dx = x - self.lastx
        dy = y - self.lasty
        self.program.motionfunc(x,y,dx,dy)
        self.lastx = x
        self.lasty = y

    def _mousefunc(self,button,state,x,y):
        """Internal use"""
        self.program.mousefunc(button,state,x,y)
        self.lastx = x
        self.lasty = y


    def _specialfunc(self,c,x,y):
        if c in keymap:
            self.program.keyboardfunc(keymap[c],x,y)

    def _specialupfunc(self,c,x,y):
        if c in keymap:
            self.program.keyboardupfunc(keymap[c],x,y)

    def _displayfunc(self):
        """Internal use."""
        if self.width == 0 or self.height == 0:
            #hidden?
            print "GLProgram.displayfunc called on hidden window?"
            return
        self.program.displayfunc()
        glutSwapBuffers ()

    def _closefunc(self):
        self.program.closefunc()
        self.program.window = None

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
        self.windows = []

    def initialize(self,program_name):
        if self.glutInitialized == False:
            glutInit ([])
            if bool(glutSetOption):
               glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS)
            glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE)
            self.glutInitialized = True

    def createWindow(self,name):
        self.initialize(name)
        w = GLUTWindow(name)
        self.windows.append(w)
        return w

    def run(self):
        """Starts the main loop.  NOTE: if freeglut is not installed, this
        will not return."""
        # Initialize Glut
        assert len(self.windows) >= 1,"Need to define at least one GL interface"
        for w in self.windows:
            w.initialize()
        glutMainLoop ()

