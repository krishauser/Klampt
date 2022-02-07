"""Defines the GLPluginInterface class, which is a unified base class for 
plugins to the klampt.vis module.
"""

from typing import Callable,List,Tuple
from ..model.typing import Vector3

class GLPluginInterface:
    """Users can add their own hooks into the visualizer by overloading this
    class's methods.  Each method should return True if the user event was
    processed.  A return value of True stops cascading
    events to a parent interface.

    Attributes:
        window (QtGLWindow or GLUTWindow): the window to which this plugin
            is attached.  May be None if not attached to an OpenGL window.
        view (GLViewport): the viewport that is currently being used for this
            plugin.
        actions (list): internally used.
    """
    def __init__(self):
        self.window = None
        self.view = None
        self.actions = []
    def initialize(self) -> bool:
        """Called by backend after the GL context is created but before the event loop starts."""
        return True
    def displayfunc(self) -> bool:
        return False
    def display(self) -> bool:
        return False
    def display_screen(self) -> bool:
        return False
    def reshapefunc(self,w,h) -> bool:
        return False
    def keyboardfunc(self,c,x,y) -> bool:
        return False
    def keyboardupfunc(self,c,x,y) -> bool:
        return False
    def mousefunc(self,button,state,x,y) -> bool:
        return False
    def motionfunc(self,x,y,dx,dy) -> bool:
        return False
    def idle(self) -> bool:
        return True
    def eventfunc(self,type,args="") -> bool:
        """Generic hook for other events, e.g., button presses, from the GUI"""
        return False
    def closefunc(self) -> bool:
        return False

    def add_action(self,callback : Callable, short_name : str, key : str,description : str=None) -> None:
        """Defines a new generic GUI action.  The action will be available in
        a menu in Qt or as keyboard commands in GLUT."""
        if not callable(callback):
            raise ValueError("Invalid callback given to add_action(callback,short_name,key,description=None)")
        self.actions.append((callback,short_name,key,description))

    #functions to request operations of the backend
    def reshape(self,w,h) -> None:
        """Asks to resize the GL window"""
        if self.window:
            return self.window.reshape(w,h)
    def idlesleep(self,seconds) -> None:
        """Asks to sleep the idle function for seconds seconds."""
        if self.window:
            self.window.idlesleep(seconds)
    def modifiers(self) -> List[str]:
        """Retrieves a list of currently pressed keyboard modifiers.
        Values can be any combination of 'ctrl', 'shift', 'alt'.
        """
        return self.window.modifiers()
    def refresh(self) -> None:
        """Asks for a redraw"""
        if self.window:
            self.window.refresh()
    def draw_text(self, point, text : str, size=12, color=None) -> None:
        """Draws text of the given size and color at the point (x,y) or
        (x,y,z).  The former method is usually called during display_screen.
        """
        if self.window:
            self.window.draw_text(point,text,size,color)

    #3D viewport accessors -- not supported directly through the backend
    def click_ray(self,x,y) -> Tuple[Vector3,Vector3]:
        """Returns the world-space ray associated with the camera click at
        pixel coordinates x,y."""
        if self.view is None:
            raise RuntimeError("Can't get click_ray for a GLPluginInterface that's not bound to a GLProgram")
        return self.view.click_ray(x,y)
    def viewport(self) -> 'Viewport':
        """Returns the :class:`~klampt.robotsim.Viewport` instance associated
        with the current GL view.  Used for interfacing with C++ widgets.
        """
        if self.view is None:
            raise RuntimeError("Can't get viewport for a GLPluginInterface that's not bound to a GLProgram")
        return self.view.to_viewport()
