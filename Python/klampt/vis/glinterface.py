
class GLProgramInterface:
    """Users can add their own hooks into the visualizer by overloading this
    class's methods.  Each method should return True if the user event was
    processed.  A return value of True stops cascading
    events to a parent interface."""
    def __init__(self):
        self.window = None
        self.x,self.y = 0,0
        self.width,self.height = 100,100
    def initialize(self):
        """Called by backend after the GL context is created but before the event loop starts."""
        return False
    def displayfunc(self):
        return False
    def display(self):
        return False
    def display_screen(self):
        return False
    def reshapefunc(self,w,h):
        self.width,self.height = w,h
        return False
    def keyboardfunc(self,c,x,y):
        return False
    def keyboardupfunc(self,c,x,y):
        return False
    def specialfunc(self,c,x,y):
        return False
    def specialupfunc(self,c,x,y):
        return False
    def mousefunc(self,button,state,x,y):
        return False
    def motionfunc(self,x,y,dx,dy):
        return False
    def idlefunc(self):
        return True
    def eventfunc(self,type,args=""):
        """Generic hook for other events, e.g., button presses, from the GUI"""
        return False
    def closefunc(self):
        return False

    #functions to request operations of the backend
    def reshape(self,w,h):
        """Asks to resize the GL window"""
        return self.window.reshape(w,h)
    def idlesleep(self,seconds):
        """Asks to sleep the idle function for seconds seconds."""
        self.window.idlesleep(seconds)
    def modifiers(self):
        """Retrieves a list of currently pressed keyboard modifiers.
        Values can be any combination of 'ctrl', 'shift', 'alt'.
        """
        return self.window.modifiers()
    def refresh(self):
        """Asks for a redraw"""
        self.window.refresh()
    def draw_text(self,point,text,size=12,color=None):
        """Draws text of the given size and color at the point (x,y) or (x,y,z).  The
        former method is usually called during display_screen."""
        self.window.draw_text(point,text,size,color)

    #3D viewport accessors -- not supported directly through the backend
    def click_ray(self,x,y):
        """Returns the ray associated with the camera click at x,y.  ONLY SUPPORTED
        BY GLPluginProgram."""
        return self.window.click_ray(x,y)
    def viewport(self):
        """Returns the Viewport instance associated with the current GL view.  ONLY SUPPORTED
        BY GLPluginProgram"""
        return self.window.viewport()
