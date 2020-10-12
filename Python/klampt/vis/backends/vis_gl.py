from ..visualization import _globalLock,VisualizationScene
from .. import glcommon
import weakref
import time

class WindowInfo:
    """Mode can be hidden, shown, or dialog"""
    def __init__(self,name,frontend,glwindow=None):
        self.name = name
        self.frontend = frontend
        self.glwindow = glwindow
        self.mode = 'hidden'
        self.guidata = None
        self.custom_ui = None
        self.doRefresh = False
        self.doReload = False
        #needed for GLUT to work properly with multiple windows
        self.worlds = []
        self.active_worlds = []


class GLVisualizationFrontend(glcommon.GLPluginProgram):
    def __init__(self):
        glcommon.GLPluginProgram.__init__(self)
        self.scene = GLVisualizationPlugin()
        self.setPlugin(self.scene)
        self.scene.program = weakref.proxy(self)

    def addAction(self,hook,short_text,key,description):
        self.add_action(hook,short_text,key,description)


class GLVisualizationPlugin(glcommon.GLWidgetPlugin,VisualizationScene):
    def __init__(self):
        glcommon.GLWidgetPlugin.__init__(self)
        VisualizationScene.__init__(self)
        #need to set this to a weakref of the GLProgram being used with this plugin. Automatically done in GLVisualizationFrontend()
        self.program = None

    def initialize(self):
        #keep or refresh display lists?
        #self._clearDisplayLists()
        return glcommon.GLWidgetPlugin.initialize(self)

    def getViewport(self):
        return self.view

    def setViewport(self,viewport):
        self.program.set_view(viewport)

    def setBackgroundColor(self,r,g,b,a=1): 
        if self.program is not None:
            self.program.clearColor = [r,g,b,a]
        else:
            print("setBackgroundColor(): doesn't work yet because scene is not bound to a window")

    def edit(self,name,doedit=True):
        global _globalLock
        _globalLock.acquire()
        obj = self.getItem(name)
        if obj is None:
            _globalLock.release()
            raise ValueError("Object "+name+" does not exist in visualization")
        if doedit:
            world = self.items.get('world',None)
            if world is not None:
                world=world.item
            obj.make_editor(world)
            if obj.editor:
                self.klamptwidgetmaster.add(obj.editor)
        else:
            if obj.editor:
                self.klamptwidgetmaster.remove(obj.editor)
                obj.remove_editor()
        self.doRefresh = True
        _globalLock.release()

    def display(self):
        global _globalLock
        _globalLock.acquire()
        #for items currently being edited AND having the appearance changed, draw the reference object
        #according to the vis settings
        #glcommon.GLWidgetPlugin.display(self)
        #restore any reference objects
        self.updateCamera()
        self.renderGL(self.view)

        _globalLock.release()

    def widgetchangefunc(self,edit):
        """Called by GLWidgetPlugin on any widget change"""
        for name,item in self.items.items():
            item.update_editor()

    def display_screen(self):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.display_screen(self)
        self.renderScreenGL(self.view,self.window)
        _globalLock.release()

    def reshapefunc(self,w,h):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.reshapefunc(self,w,h)
        _globalLock.release()
    def keyboardfunc(self,c,x,y):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.keyboardfunc(self,c,x,y)
        _globalLock.release()
    def keyboardupfunc(self,c,x,y):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.keyboardupfunc(self,c,x,y)
        _globalLock.release()
    def mousefunc(self,button,state,x,y):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.mousefunc(self,button,state,x,y)
        _globalLock.release()
    def motionfunc(self,x,y,dx,dy):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.motionfunc(self,x,y,dx,dy)
        _globalLock.release()
    def eventfunc(self,type,args=""):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.eventfunc(self,type,args)
        _globalLock.release()
    def closefunc(self):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.closefunc(self)
        _globalLock.release()

    def idle(self):
        global _globalLock
        _globalLock.acquire()
        VisualizationScene.updateTime(self,time.time())
        _globalLock.release()
        return False


