from ..visualization import _WindowManager,VisualizationScene,VisPlot,objectToVisType
from ..ipython import KlamptWidget
from ...io.html import HTMLSharePath
from IPython.display import display,HTML
from ...robotsim import WorldModel
import pkg_resources
import math
import time

_PAGE_TEMPLATE = """<!DOCTYPE html>
<html style="overflow:hidden;">
   <head>
      <title>__TITLE__</title>
      <meta charset="UTF-8">    
   </head>
   <body style="margin:0px;height:100%%;overflow:hidden;">
     %s
   </body>
</html>"""

_VIS_HTML_BOILERPLATE = pkg_resources.resource_filename('klampt','data/vis_html_boilerplate.html')

class HTMLVisualizationScene(VisualizationScene):
    """Handles the conversion between vis calls and the html output."""
    def __init__(self):
        VisualizationScene.__init__(self)
        self.sp = HTMLSharePath(filename=None,boilerplate=_VIS_HTML_BOILERPLATE)
        self.kw = KlamptWidget()
        self.animating = False
        self._textItems = set()
        self.rpcs_this_frame = []

    def addAction(self,hook,short_text,key,description):
        raise NotImplementedError("Can't add actions to this frontend")

    def clear(self):
        VisualizationScene.clear(self)
        self.sp = HTMLSharePath(filename=None,boilerplate=_VIS_HTML_BOILERPLATE)
        self.kw = KlamptWidget()
        self.animating = False
        self._textItems = set()
        self.rpcs_this_frame = []
        #reset the visualization clock
        self.startTime = None
        self.t = 0

    def clearText(self):
        VisualizationScene.clearText(self)
        if len(self._textItems) > 0:
            self.kw.beginRpc()
            for t in self._textItems:
                self.kw.remove(t)
            self.rpcs_this_frame += self.kw._rpc_calls
            self.kw.endRpc()
            self._textItems = set()

    def add(self,name,item,keepAppearance=False,**kwargs):
        self.kw.beginRpc()
        handled = False
        if 'size' in kwargs:
            if hasattr(item,'__iter__') and len(item)==3:
                self.kw.addSphere(name,item[0],item[1],item[2],kwargs['size'])
                handled = True
        if not handled:
            try:
                self.kw.add(name,item)
            except ValueError:
                self.kw.endRpc()
                raise ValueError("Can't draw items of type "+item.__class__.__name__+" in HTML form")
        if 'color' in kwargs:
            self.kw.setColor(name,*kwargs['color'])
        if name=='world' or name=='sim':
            self.sp.start(item)
        self.rpcs_this_frame += self.kw._rpc_calls
        self.kw.endRpc()
        
        #VisualizationScene.add(self,name,item,keepAppearance,**kwargs)
        VisualizationScene.add(self,name,item,keepAppearance)

    def addText(self,name,text,pos=None):
        self._textItems.add(name)
        self.kw.addText(name,text,pos)

    def remove(self,name):
        VisualizationScene.remove(name)
        self.kw.remove(name)

    def setItemConfig(self,name,value):
        self.kw.setItemConfig(name,value)

    def hideLabel(self,name,hidden=True):
        #no labels, ignore silently
        pass

    def edit(self,name,doedit=True):
        raise RuntimeError("IPython: can't do visual editing.  Try klampt.ipython.EditPoint, etc.")

    def hide(self,name,hidden=True):
        VisualizationScene.hide(self,name,hidden)
        self.kw.hide(name,hidden)

    def _setAttribute(self,item,attr,value):
        VisualizationScene._setAttribute(self,item,attr,value)
        self.kw.beginRpc()
        if attr == 'color':
            self.kw.setColor(item.name,*value)
        elif attr == 'size':
            #modify point size
            if hasattr(item.item,'__iter__') and len(item.item)==3:
                self.kw.addSphere(name,item.item[0],item.item[1],item.item[2],value)
            pass
        #can't handle any other attributes right now
        self.rpcs_this_frame += self.kw._rpc_calls
        self.kw.endRpc()
        
    def setDrawFunc(self,name,func):
        raise RuntimeError("IPython: can't set up custom draw functions")

    def getViewport(self):
        cam = self.kw.getCamera()
        #TODO: convert camera message to GLViewport
        return cam

    def setViewport(self,viewport):
        #TODO: convert from GLViewport to camera message
        self.kw.beginRpc()
        self.kw.setCamera(viewport)
        self.rpcs_this_frame += self.kw._rpc_calls
        self.kw.endRpc()

    def setBackgroundColor(self,r,g,b,a=1): 
        raise RuntimeError("IPython: can't yet change the background color")

    def stepAnimation(self,dt):
        VisualizationScene.stepAnimation(self,dt)
        self.kw.beginRpc()
        self.updateCamera()
        #look through changed items and update them
        def updateItem(item):
            if isinstance(item,VisPlot):
                return
            if isinstance(item.item,WorldModel):
                return
            if item.transformChanged:
                raise NotImplementedError("TODO: update moved things of type",item.item.__class__.__name__)
                self.kw.setTransform()
                item.transformChanged = False
            for k,c in item.subAppearances.items():
                updateItem(c)
        for k,v in self.items.items():
            updateItem(v)
        self.rpcs_this_frame += self.kw._rpc_calls
        self.kw.endRpc()
        if "world" in self.items:
            for k,v in self.items["world"].subAppearances.items():
                v.swapDrawConfig()
        self.sp.animate(self.currentAnimationTime,rpc=self.rpcs_this_frame)
        if "world" in self.items:
            for k,v in self.items["world"].subAppearances.items():
                v.swapDrawConfig()
        self.rpcs_this_frame = []
        self.updateTime(self.currentAnimationTime)

    def update(self):
        #self.kw.update()   #this isn't necessary
        self.kw.beginRpc()
        self.updateCamera()
        #look through changed items and update them
        def updateItem(item):
            if isinstance(item,VisPlot):
                return
            if item.transformChanged:
                raise NotImplementedError("TODO: update moved things")
                self.kw.setTransform()
                item.transformChanged = False
            for k,c in item.subAppearances.items():
                updateItem(c)
        for k,v in self.items.items():
            updateItem(v)
        self.rpcs_this_frame += self.kw._rpc_calls
        self.kw.endRpc()
        t = time.time()
        if self.startTime is None:
            self.startTime = t
        t = t-self.startTime
        if t > self.t + 1.0/30.0: # 30fps
            if "world" in self.items:
                for k,v in self.items["world"].subAppearances.items():
                    v.swapDrawConfig()
            self.sp.animate(t,rpc=self.rpcs_this_frame)
            if "world" in self.items:
                for k,v in self.items["world"].subAppearances.items():
                    v.swapDrawConfig()
            self.rpcs_this_frame = []
            self.t = t
        self.updateTime(t)

    def iframe(self,w=None,h=None):
        """Returns an IFrame containing the scene, with width w and height h.
        If w and h are not given, then the viewport's size are used."""
        if w is None:
            w = self.kw.width
        if h is None:
            h = self.kw.height
        def html_escape_quotes(s):
            return s.replace('"','&quot;')
        return '<iframe width="%d" height="%d" srcdoc="%s">'%(w,h,html_escape_quotes(self.page()))

    def __str__(self):
        """Returns the HTML representation of this scene"""
        return self.sp.end()

    def page(self):
        """Returns a full HTML page representing the scene"""
        return _PAGE_TEMPLATE%(str(self),)
        



class HTMLWindowManager(_WindowManager):
    def __init__(self):
        self.windows = [HTMLVisualizationScene()]
        self.current_window = 0
        self.displayed = False
        self.quit = False
    def frontend(self):
        return self.windows[self.current_window]
    def scene(self):
        return self.windows[self.current_window]
    def createWindow(self,title):
        self.windows.append(HTMLVisualizationScene())
        self.current_window = len(self.windows)-1
        return self.current_window
    def setWindow(self,id):
        assert id >= 0 and id < len(self.windows)
        self.current_window = id
    def getWindow(self):
        return self.current_window
    def setPlugin(self,plugin):
        raise NotImplementedError("HTML does not accept plugins")
    def pushPlugin(self,plugin):
        raise NotImplementedError("HTML does not accept plugins")
    def popPlugin(self):
        raise NotImplementedError("HTML does not accept plugins")
    def addPlugin(self,plugin):
        raise NotImplementedError("HTML does not accept plugins")
    def lock(self):
        for w in self.windows:
            w.beginRpc()
    def unlock(self):
        for w in self.windows:
            w.endRpc()
    def multithreaded(self):
        return False
    def run(self):
        self.show()
    def loop(self,setup,callback,cleanup):
        setup()
        t = 0
        self.quit = False
        while t < duration:
            if not self.shown(): break
            callback()
        self.show()
        cleanup()
    def spin(self,duration):
        t = 0
        self.quit = False
        while t < duration:
            if not self.shown(): break
            self.frontend().stepAnimation(0.04)
            t += 0.04
        self.show()
    def show(self):
        self.quit = False
        self.displayed = True
        display(HTML(self.frontend().iframe()))
    def shown(self):
        return self.displayed and not self.quit
    def hide(self):
        self.quit = True
    def update(self):
        self.frontend().update()
    
