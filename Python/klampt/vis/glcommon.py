from glinterface import GLProgramInterface
import math

class GLWidgetPlugin(GLProgramInterface):
    """A GL plugin that sends user events to one or more Klamp't widgets"""    
    def __init__(self):
        from ..robotsim import WidgetSet

        GLProgramInterface.__init__(self)
        self.klamptwidgetbutton = 2
        self.klamptwidgetmaster = WidgetSet()
        self.klamptwidgetdragging = False
    def addWidget(self,widget):
        self.klamptwidgetmaster.add(widget)
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

class GLMultiProgramInterface(GLProgramInterface):
    def __init__(self):
        GLProgramInterface.__init__(self)
        self.plugins = []
        self.activePlugin = None
        self.dragging = False
        self.sizePolicy = 'fit'
        self.broadcast = False
    def addPlugin(self,plugin):
        self.plugins.append(plugin)
        #spoofs reshape, motion functions
        plugin.window = self
        self.fit()
    def removePlugin(self,plugin):
        plugin.window = None
        for i,p in enumerate(self.plugins):
            if p is plugin:
                self.plugins.pop(i)
                self.fit()
                self.activePlugin = None
                return
    def updateActive(self,x,y):
        if x < self.x or x > self.x + self.width or y < self.y or y > self.y + self.height:
            return
        self.activePlugin = None
        for i,p in enumerate(self.plugins):
            if x >= p.x and x < p.x + p.width:
                if y >= p.y and y < p.y + p.height:
                    self.activePlugin = i
                    return
        return
    def fit(self):
        rowlen = int(math.ceil(math.sqrt(len(self.plugins))))
        assert rowlen > 0
        rowheights = [0]*int(len(self.plugins)/rowlen)
        colwidths = [0]*rowlen
        for i,p in enumerate(self.plugins):
            col = i % rowlen
            row = int(i / rowlen)
            rowheights[row] = max(p.height,rowheights[row])
            colwidths[col] = max(p.width,colwidths[col])
        cumrowheights = [0]
        cumcolwidths = [0]
        for h in rowheights:
            cumrowheights.append(cumrowheights[-1]+h)
        for w in colwidths:
            cumcolwidths.append(cumcolwidths[-1]+w)
        if self.sizePolicy == 'fit':
            self.height = sum(rowheights)
            self.width = sum(colwidths)
            for i,p in enumerate(self.plugins):
                col = i % rowlen
                row = int(i / rowlen)
                p.x,p.y = (cumcolwidths[col],cumrowheights[row])
            if self.window != None:
                self.window.reshape(self.width,self.height)
        else:
            #squeeze
            for i,p in enumerate(self.plugins):
                col = i % rowlen
                row = int(i / rowlen)
                p.x = float(self.width)*float(cumcolwidths[col])/float(cumcolwidths[-1])
                p.y = float(self.height)*float(cumrowheights[row])/float(cumrowheights[-1])
                p.width = float(self.width)*float(colwidths[col]) / float(cumcolwidths[-1])
                p.height = float(self.height)*float(rowheights[row]) / float(cumrowheights[-1])
                p.x = self.x+int(p.x)
                p.y = self.y+int(p.y)
                p.width = int(p.width)
                p.height = int(p.height)
                p.reshapefunc(p.width,p.height)
        if self.window != None:
            self.refresh()

    def initialize(self):
        for p in self.plugins:
            if not p.initialize():
                return False
        return True
    def reshapefunc(self,w,h):
        self.width,self.height = w,h
        self.sizePolicy = 'squash'
        self.fit()
        return True
    def displayfunc(self):
        anyTrue = False
        for p in self.plugins:
            assert isinstance(p.x,int) and isinstance(p.y,int)
            if p.displayfunc():
                anyTrue = True
        return anyTrue
    def display(self):
        anyTrue = False
        for p in self.plugins:
            if p.display():
                anyTrue = True
        return anyTrue
    def display_screen(self):
        anyTrue = False
        for p in self.plugins:
            if p.display_screen():
                anyTrue = True
        return anyTrue
    def keyboardfunc(self,c,x,y):
        if self.broadcast:
            for p in self.plugins:
                p.keyboardfunc(c,x,y)
            return True
        self.updateActive(x,y)
        if self.activePlugin != None:
            return True if self.plugins[self.activePlugin].keyboardfunc(c,x,y) else False
        return False
    def keyboardupfunc(self,c,x,y):
        if self.broadcast:
            for p in self.plugins:
                p.keyboardupfunc(c,x,y)
            return True
        self.updateActive(x,y)
        if self.activePlugin != None:
            return True if self.plugins[self.activePlugin].keyboardupfunc(c,x,y) else False
        return False
    def specialfunc(self,c,x,y):
        if self.broadcast:
            for p in self.plugins:
                p.specialfunc(c,x,y)
            return True
        self.updateActive(x,y)
        if self.activePlugin != None:
            return True if self.plugins[self.activePlugin].specialfunc(c,x,y) else False
        return False
    def specialupfunc(self,c,x,y):
        if self.broadcast:
            for p in self.plugins:
                p.specialupfunc(c,x,y)
            return True
        self.updateActive(x,y)
        if self.activePlugin != None:
            return True if self.plugins[self.activePlugin].specialupfunc(c,x,y) else False
        return False
    def mousefunc(self,button,state,x,y):
        if self.broadcast:
            for p in self.plugins:
                p.mousefunc(button,state,x,y)
            return True
        if state == 0:
            #button down
            self.updateActive(x,y)
            self.dragging = True
        else:
            self.dragging = False
        if self.activePlugin != None:
            return True if self.plugins[self.activePlugin].mousefunc(button,state,x,y) else False
        return False
    def motionfunc(self,x,y,dx,dy):
        if self.broadcast:
            for p in self.plugins:
                p.motionfunc(x,y,dx,dy)
            return True
        if not self.dragging:
            self.updateActive(x,y)
        if self.activePlugin != None:
            return True if self.plugins[self.activePlugin].motionfunc(x,y,dx,dy) else False
        return False
    def idlefunc(self):
        for p in self.plugins:
            p.idle()
        return True

    def reshape(self,w,h):
        """Spoofs the window's reshape function"""
        self.width,self.height = w,h
        self.sizePolicy = 'squash'
        self.fit()
        return True
    def draw_text(self,point,text,size=12,color=None):
        """Draws text of the given size and color at the given point.  Usually
        called during display_screen."""
        #if self.activePlugin == None:
        self.window.draw_text(point,text,size,color)
        #the GL viewport should be setup already
        #else:
        #    ox,oy = self.pluginOrigins[self.activePlugin]
        #    self.window.draw_text(ox+x,oy+y,text,size,color)
