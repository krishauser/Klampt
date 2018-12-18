from .glinterface import GLPluginInterface
from .glprogram import GLProgram,GLPluginProgram
import math
from OpenGL.GL import *
import weakref

class GLWidgetPlugin(GLPluginInterface):
    """A GL plugin that sends user events to one or more Klamp't widgets.
    To use, add this to a GLPluginProgram and call addWidget to add widgets"""    
    def __init__(self):
        from ..robotsim import WidgetSet

        GLPluginInterface.__init__(self)
        self.klamptwidgetbutton = 2
        self.klamptwidgetmaster = WidgetSet()
        self.klamptwidgetdragging = False
    def addWidget(self,widget):
        self.klamptwidgetmaster.add(widget)
    def widgetchangefunc(self,event):
        """Called whenever a widget is clicked or dragged.
        event can be 'mousedown', 'mousedrag', 'mouseup'. 
        Subclasses can use this to respond to widget click events"""
        pass
    def widgethoverfunc(self):
        """Called whenever a widget changes appearance due to hover.
        Subclasses can use this to respond to widget click events"""
        pass

    def display(self):
        self.klamptwidgetmaster.drawGL(self.viewport())
        return False
    def keyboardfunc(self,c,x,y):
        if len(c)==1:
            self.klamptwidgetmaster.keypress(c)
        return False
    def keyboardupfunc(self,c,x,y):
        return False
    def mousefunc(self,button,state,x,y):
        if button == self.klamptwidgetbutton:
            if state == 0:  #down
                if self.klamptwidgetmaster.beginDrag(x,self.view.h-y,self.viewport()):
                    self.widgetchangefunc("mousedown")
                    self.klamptwidgetdragging = True
            else:
                if self.klamptwidgetdragging:
                    self.widgetchangefunc("mouseup")
                    self.klamptwidgetmaster.endDrag()
                self.klamptwidgetdragging = False
            if self.klamptwidgetmaster.wantsRedraw():
                self.refresh()
            return True
        return False
    def motionfunc(self,x,y,dx,dy):
        if self.klamptwidgetdragging:
            self.klamptwidgetmaster.drag(dx,-dy,self.viewport())
            self.widgetchangefunc("mousedrag")
            if self.klamptwidgetmaster.wantsRedraw():
                self.refresh()
            return True
        else:
            self.klamptwidgetmaster.hover(x,self.view.h-y,self.viewport())
            if self.klamptwidgetmaster.wantsRedraw():
                self.widgethoverfunc()
                self.refresh()
        return False
    def idlefunc(self):
        self.klamptwidgetmaster.idle()
        return True

class GLMultiViewportProgram(GLProgram):
    def __init__(self):
        GLProgram.__init__(self)
        self.views = []
        self.activeView = None
        self.dragging = False
        self.sizePolicy = 'fit'
        self.broadcast = False
        self.defaultSizes = []
        #self.height = self.view.h
    def initialize(self):
        if not GLProgram.initialize(self): return False
        for v in self.views:
            v.window = self.window
            if not v.initialize():
                return False
        return True
    def addView(self,view):
        if isinstance(view,GLPluginInterface):
            plugin = view
            pview = GLPluginProgram()
            pview.window = self.window
            pview.setPlugin(view)
            view = pview
        assert isinstance(view,GLProgram)
        self.views.append(view)
        #spoofs reshape, motion functions
        view.window = weakref.proxy(self)
        self.defaultSizes.append((view.view.w,view.view.h))
        self.fit()
        #print "Added a view, total",len(self.views),"size now",self.view.w,self.view.h
        return view
    def removeView(self,view):
        view.window = None
        for i,p in enumerate(self.views):
            if p is view:
                self.views.pop(i)
                self.defaultSizes.pop(i)
                self.fit()
                self.activeView = None
                return
    def clearViews(self):
        for p in self.views:
            p.window = None
        self.views = []
        self.defaultSizes = []
        self.activeView = None
    def updateActive(self,x,y):
        if not self.view.contains(x,y):
            return
        self.activeView = None
        for i,p in enumerate(self.views):
            if p.view.contains(x,y):
                #print "Selecting view",x,y,":",i
                self.activeView = i
                return
        return
    def fit(self):
        if len(self.views) == 0: return
        rowlen = int(math.ceil(math.sqrt(len(self.views))))
        assert rowlen > 0
        rowheights = [0]*int(math.ceil(float(len(self.views))/rowlen))
        colwidths = [0]*rowlen
        for i,p in enumerate(self.views):
            col = i % rowlen
            row = int(i / rowlen)
            rowheights[row] = max(self.defaultSizes[i][1],rowheights[row])
            colwidths[col] = max(self.defaultSizes[i][0],colwidths[col])
        cumrowheights = [0]
        cumcolwidths = [0]
        for h in rowheights:
            cumrowheights.append(cumrowheights[-1]+h)
        for w in colwidths:
            cumcolwidths.append(cumcolwidths[-1]+w)
        if self.sizePolicy == 'fit':
            self.view.w = sum(colwidths)
            self.view.h = sum(rowheights)
            for i,p in enumerate(self.views):
                col = i % rowlen
                row = int(i / rowlen)
                p.view.x,p.view.y = (cumcolwidths[col],cumrowheights[row])
            self.width = self.view.w
            self.height = self.view.h
            if self.window != None:
                self.window.reshape(self.view.w,self.view.h)
        else:
            #squeeze
            self.width = self.view.w
            self.height = self.view.h
            for i,p in enumerate(self.views):
                col = i % rowlen
                row = int(i / rowlen)
                p.view.x = float(self.view.w)*float(cumcolwidths[col])/float(cumcolwidths[-1])
                p.view.y = float(self.view.h)*float(cumrowheights[row])/float(cumrowheights[-1])
                p.view.w = float(self.view.w)*float(colwidths[col]) / float(cumcolwidths[-1])
                p.view.h = float(self.view.h)*float(rowheights[row]) / float(cumrowheights[-1])
                p.view.x = self.view.x+int(p.view.x)
                p.view.y = self.view.y+int(p.view.y)
                p.view.w = int(p.view.w)
                p.view.h = int(p.view.h)
                #print "View",i,"shape",(p.view.x,p.view.y,p.view.w,p.view.h)
                p.reshapefunc(p.view.w,p.view.h)
        if self.window != None:
            self.refresh()

    def reshapefunc(self,w,h):
        if (w,h) != (self.view.w,self.view.h):
            self.view.w,self.view.h = w,h
            self.height = self.view.h
            self.sizePolicy = 'squash'
            self.fit()
        return True
    def displayfunc(self):
        anyTrue = False
        glClearColor(0,0,0,0)
        glScissor(0,0,self.view.w,self.view.h)
        glEnable(GL_SCISSOR_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        for p in self.views:
            try:
                if p.displayfunc():
                    anyTrue = True
            except Exception:
                print("Error running displayfunc() for plugin",p.__class__.__name__)
                raise
        return anyTrue
    def display(self):
        anyTrue = False
        for p in self.views:
            try:
                if p.display():
                    anyTrue = True
            except Exception:
                print("Error running display() for plugin",p.__class__.__name__)
                raise
        return anyTrue
    def display_screen(self):
        anyTrue = False
        for p in self.views:
            try:
                if p.display_screen():
                    anyTrue = True
            except Exception:
                print("Error running display_screen() for plugin",p.__class__.__name__)
                raise
        return anyTrue
    def keyboardfunc(self,c,x,y):
        if self.broadcast:
            for p in self.views:
                p.keyboardfunc(c,x,y)
            return True
        self.updateActive(x,y)
        if self.activeView != None:
            return True if self.views[self.activeView].keyboardfunc(c,x,y) else False
        return False
    def keyboardupfunc(self,c,x,y):
        if self.broadcast:
            for p in self.views:
                p.keyboardupfunc(c,x,y)
            return True
        self.updateActive(x,y)
        if self.activeView != None:
            return True if self.views[self.activeView].keyboardupfunc(c,x,y) else False
        return False
    def mousefunc(self,button,state,x,y):
        if self.broadcast:
            for p in self.views:
                p.mousefunc(button,state,x,y)
            return True
        if state == 0:
            #button down
            self.updateActive(x,y)
            self.dragging = True
        else:
            self.dragging = False
        if self.activeView != None:
            return True if self.views[self.activeView].mousefunc(button,state,x,y) else False
        return False
    def motionfunc(self,x,y,dx,dy):
        if self.broadcast:
            for p in self.views:
                p.motionfunc(x,y,dx,dy)
            return True
        if not self.dragging:
            self.updateActive(x,y)
        if self.activeView != None:
            return True if self.views[self.activeView].motionfunc(x,y,dx,dy) else False
        return False
    def idlefunc(self):
        for p in self.views:
            p.idlefunc()
        return True
    def refresh(self):
        """Spoofs the window's refresh function"""
        self.window.refresh()
    def reshape(self,w,h):
        """Spoofs the window's reshape function"""
        raise NotImplementedError("Can't have a viewport reshaping a multi-viewport yet")
        self.sizePolicy = 'squash'
        self.fit()
        return True
    def draw_text(self,point,text,size=12,color=None):
        """Draws text of the given size and color at the given point.  Usually
        called during display_screen."""
        #if self.activeView == None:
        self.window.draw_text(point,text,size,color)
        #the GL viewport should be setup already
        #else:
        #    ox,oy = self.pluginOrigins[self.activeView]
        #    self.window.draw_text(ox+x,oy+y,text,size,color)
    def click_ray(self,x,y):
        #print "Getting click ray"
        if self.activeView == None:
            return self.window.click_ray(x,y)
        else:
            return self.views[self.activeView].click_ray(x,y)
    def get_view(self):
        #print "Getting viewport..."
        if self.activeView == None:
            return self.window.get_view()
        else:
            return self.views[self.activeView].get_view()
    def set_view(self,vp):
        #print "Getting viewport..."
        if self.activeView == None:
            return self.window.get_view(vp)
        else:
            return self.views[self.activeView].get_view(vp)

_CACHED_DISPLAY_LISTS = set()
_CACHED_WARN_THRESHOLD = 1000
_CACHED_DELETED_LISTS = list()

class CachedGLObject:
    """An object whose drawing is accelerated by means of a display list.
    The draw function may draw the object in the local frame, and the
    object may be transformed without having to recompile the display list.
    """
    def __init__(self):
        self.name = ""
        #OpenGL display list
        self.glDisplayList = None
        #marker for recursive calls
        self.makingDisplayList = False
        #parameters for render function
        self.displayListRenderArgs = None
        #other parameters for display lists
        self.displayListParameters = None
        #dirty bit to indicate whether the display list should be recompiled
        self.changed = False

    def __del__(self):
        self.destroy()

    def destroy(self):
        """Must be called to free up resources used by this object"""
        if self.glDisplayList != None:
            global _CACHED_DELETED_LISTS,_CACHED_DISPLAY_LISTS
            if len(_CACHED_DELETED_LISTS) > 100:
                for dl in _CACHED_DELETED_LISTS:
                    glDeleteLists(dl,1)
                _CACHED_DELETED_LISTS = list()
            else:
                _CACHED_DELETED_LISTS.append(self.glDisplayList)
            _CACHED_DISPLAY_LISTS.remove(self.glDisplayList)
            self.glDisplayList = None

    def markChanged(self):
        """Marked by an outside source to indicate the object has changed and
        should be redrawn."""
        self.changed = True
    
    def draw(self,renderFunction,transform=None,args=None,parameters=None):
        """Given the function that actually makes OpenGL calls, this
        will draw the object.

        If parameters is given, the object's local appearance is assumed
        to be defined deterministically from these parameters.  The display
        list will be redrawn if the parameters change.
        """
        from ..math import se3
        if args == None:
            args = ()
        if self.makingDisplayList:
            renderFunction(*args)
            return
        if self.glDisplayList == None or self.changed or parameters != self.displayListParameters or args != self.displayListRenderArgs:
            self.displayListRenderArgs = args
            self.displayListParameters = parameters
            self.changed = False
            if self.glDisplayList == None:
                #print "Generating new display list",self.name
                global _CACHED_WARN_THRESHOLD,_CACHED_DISPLAY_LISTS,_CACHED_DELETED_LISTS
                if len(_CACHED_DELETED_LISTS) > 0:
                    self.glDisplayList = _CACHED_DELETED_LISTS[-1]
                    _CACHED_DELETED_LISTS.pop(-1)
                else:
                    self.glDisplayList = glGenLists(1)
                _CACHED_DISPLAY_LISTS.add(self.glDisplayList)
                if len(_CACHED_DISPLAY_LISTS) > _CACHED_WARN_THRESHOLD:
                    print("GLCachedObject: Creating",len(_CACHED_DISPLAY_LISTS),"GL objects",self.glDisplayList,"watch me for memory usage...")
                    _CACHED_WARN_THRESHOLD += 1000
            #print "Compiling display list",self.name
            if transform:
                glPushMatrix()
                glMultMatrixf(sum(list(zip(*se3.homogeneous(transform))),()))
            
            glNewList(self.glDisplayList,GL_COMPILE_AND_EXECUTE)
            self.makingDisplayList = True
            try:
                renderFunction(*args)
            except GLError:
                import traceback
                print("Error encountered during draw, display list",self.glDisplayList)
                traceback.print_exc()
            self.makingDisplayList = False
            glEndList()

            if transform:
                glPopMatrix()
        else:
            if transform:
                glPushMatrix()
                glMultMatrixf(sum(list(zip(*se3.homogeneous(transform))),()))
            glCallList(self.glDisplayList)
            if transform:
                glPopMatrix()
