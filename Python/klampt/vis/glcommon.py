from glinterface import GLPluginInterface
from glprogram import GLProgram
import math

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
            view = GLPluginProgram()
            view.window = self.window
            view.setPlugin(view)
        assert isinstance(view,GLProgram)
        self.views.append(view)
        #spoofs reshape, motion functions
        view.window = self
        self.fit()
        return view
    def removeView(self,view):
        view.window = None
        for i,p in enumerate(self.views):
            if p is view:
                self.views.pop(i)
                self.fit()
                self.activeView = None
                return
    def updateActive(self,x,y):
        if not self.view.contains(x,y):
            return
        self.activeView = None
        for i,p in enumerate(self.views):
            if p.view.contains(x,y):
                self.activeView = i
                return
        return
    def fit(self):
        rowlen = int(math.ceil(math.sqrt(len(self.views))))
        assert rowlen > 0
        rowheights = [0]*int(len(self.views)/rowlen)
        colwidths = [0]*rowlen
        for i,p in enumerate(self.views):
            col = i % rowlen
            row = int(i / rowlen)
            rowheights[row] = max(p.view.h,rowheights[row])
            colwidths[col] = max(p.view.w,colwidths[col])
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
            if self.window != None:
                self.window.reshape(self.view.w,self.view.h)
        else:
            #squeeze
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
                p.reshapefunc(p.view.w,p.view.h)
        if self.window != None:
            self.refresh()

    def reshapefunc(self,w,h):
        if (w,h) != (self.view.w,self.view.h):
            self.view.w,self.view.h = w,h
            self.sizePolicy = 'squash'
            self.fit()
        return True
    def displayfunc(self):
        anyTrue = False
        for p in self.views:
            try:
                if p.displayfunc():
                    anyTrue = True
            except Exception:
                print "Error running displayfunc() for plugin",p.__class__.__name__
                raise
        return anyTrue
    def display(self):
        anyTrue = False
        for p in self.views:
            try:
                if p.display():
                    anyTrue = True
            except Exception:
                print "Error running display() for plugin",p.__class__.__name__
                raise
        return anyTrue
    def display_screen(self):
        anyTrue = False
        for p in self.views:
            try:
                if p.display_screen():
                    anyTrue = True
            except Exception:
                print "Error running display_screen() for plugin",p.__class__.__name__
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
    def specialfunc(self,c,x,y):
        if self.broadcast:
            for p in self.views:
                p.specialfunc(c,x,y)
            return True
        self.updateActive(x,y)
        if self.activeView != None:
            return True if self.views[self.activeView].specialfunc(c,x,y) else False
        return False
    def specialupfunc(self,c,x,y):
        if self.broadcast:
            for p in self.views:
                p.specialupfunc(c,x,y)
            return True
        self.updateActive(x,y)
        if self.activeView != None:
            return True if self.views[self.activeView].specialupfunc(c,x,y) else False
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
        print "Getting click ray"
        if self.activeView == None:
            return self.window.click_ray(x,y)
        else:
            return self.views[self.activeView].click_ray(x,y)
    def viewport(self):
        print "Getting viewport..."
        if self.activeView == None:
            return self.window.viewport()
        else:
            return self.views[self.activeView].viewport()
