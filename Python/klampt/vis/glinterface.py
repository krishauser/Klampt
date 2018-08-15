
class GLPluginInterface:
    """Users can add their own hooks into the visualizer by overloading this
    class's methods.  Each method should return True if the user event was
    processed.  A return value of True stops cascading
    events to a parent interface."""
    def __init__(self):
        self.window = None
        self.view = None
        self.actions = []
    def initialize(self):
        """Called by backend after the GL context is created but before the event loop starts."""
        return True
    def displayfunc(self):
        return False
    def display(self):
        return False
    def display_screen(self):
        return False
    def reshapefunc(self,w,h):
        return False
    def keyboardfunc(self,c,x,y):
        return False
    def keyboardupfunc(self,c,x,y):
        return False
    def mousefunc(self,button,state,x,y):
        return False
    def motionfunc(self,x,y,dx,dy):
        return False
    def idle(self):
        return True
    def eventfunc(self,type,args=""):
        """Generic hook for other events, e.g., button presses, from the GUI"""
        return False
    def closefunc(self):
        return False

    def add_action(self,callback,short_name,key,description=None):
        """Defines a new generic GUI action.  The action will be available in a menu in
        Qt or as keyboard commands in GLUT."""
        if not callable(callback):
            raise ValueError("Invalid callback given to add_action(callback,short_name,key,description=None)")
        self.actions.append((callback,short_name,key,description))

    #functions to request operations of the backend
    def reshape(self,w,h):
        """Asks to resize the GL window"""
        if self.window:
            return self.window.reshape(w,h)
    def idlesleep(self,seconds):
        """Asks to sleep the idle function for seconds seconds."""
        if self.window:
            self.window.idlesleep(seconds)
    def modifiers(self):
        """Retrieves a list of currently pressed keyboard modifiers.
        Values can be any combination of 'ctrl', 'shift', 'alt'.
        """
        return self.window.modifiers()
    def refresh(self):
        """Asks for a redraw"""
        if self.window:
            self.window.refresh()
    def draw_text(self,point,text,size=12,color=None):
        """Draws text of the given size and color at the point (x,y) or (x,y,z).  The
        former method is usually called during display_screen."""
        if self.window:
            self.window.draw_text(point,text,size,color)

    #3D viewport accessors -- not supported directly through the backend
    def click_ray(self,x,y):
        """Returns the world-space ray associated with the camera click at x,y."""
        return self.view.click_ray(x,y)
    def viewport(self):
        """Returns the Viewport instance associated with the current GL view."""
        return self.view.toViewport()
