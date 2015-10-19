from robotsim import WidgetSet

_PyQtAvailable = False
_GLUTAvailable = False

class GLPluginBase:
    """Users can add their own hooks into the visualizer by overloading this
    class's methods and calling setPlugin.  Each method should return True
    if the user event was processed."""
    def __init__(self):
        self.window = None
        self.width = 100
        self.height = 100
    def display(self):
        return False
    def display_screen(self):
        return False
    def viewport(self):
        return self.window.viewport()
    def idlesleep(self,seconds):
        self.window.idlesleep(seconds)
    def refresh(self):
        self.window.refresh()
    def click_ray(self,x,y):
        return self.window.click_ray(x,y)
    def initialize(self):
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

class GLWidgetPlugin(GLPluginBase):
    """A GL plugin that sends user events to one or more Klamp't widgets"""    
    def __init__(self):
        GLPluginBase.__init__(self)
        self.klamptwidgetbutton = 2
        self.klamptwidgetmaster = WidgetSet()
        self.klamptwidgetdragging = False
    def addWidget(self,widget):
        self.klamptwidgetmaster.add(widget)
    def display(self):
        self.klamptwidgetmaster.drawGL(self.viewport())
    def keyboardfunc(self,c,x,y):
        self.klamptwidgetmaster.keypress(c)
        return False
    def keyboardupfunc(self,c,x,y):
        return False
    def specialfunc(self,c,x,y):
        self.klamptwidgetmaster.keypress(c)
        return False
    def specialupfunc(self,c,x,y):
        return False
    def mousefunc(self,button,state,x,y):
        if button == self.klamptwidgetbutton:
            if state == 0:  #down
                if self.klamptwidgetmaster.beginDrag(x,self.height-y,self.viewport()):
                    self.klamptwidgetdragging = True
            else:
                if self.klamptwidgetdragging:
                    self.klamptwidgetmaster.endDrag()
                self.klamptwidgetdragging = False
            if self.klamptwidgetmaster.wantsRedraw():
                self.refresh()
            return True
        return False
    def motionfunc(self,x,y,dx,dy):
        if self.klamptwidgetdragging:
            self.klamptwidgetmaster.drag(dx,-dy,self.viewport())
            if self.klamptwidgetmaster.wantsRedraw():
                self.refresh()
            return True
        else:
            self.klamptwidgetmaster.hover(x,self.height-y,self.viewport())
            if self.klamptwidgetmaster.wantsRedraw():
                self.refresh()
        return False
    def idlefunc(self):
        self.klamptwidgetmaster.idle()
        return True

try:
    from PyQt4.QtCore import *
    _PyQtAvailable = True
except ImportError:
    print "QT is not available... try sudo apt-get install python-qt4 python-qt4-gl"
    try:
        from OpenGL.GLUT import *
        _GLUTAvailable = True
    except ImportError:
        print "Neither QT nor GLUT are available... visualization disabled"
