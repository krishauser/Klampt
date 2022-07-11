from ..visualization import _WindowManager,VisualizationScene,VisPlot,objectToVisType
from ..ipython import KlamptWidget
from ...io.html import HTMLSharePath
from IPython.display import display,HTML
from ...robotsim import WorldModel
import warnings
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
    def __init__(self,title='Klampt HTML output'):
        VisualizationScene.__init__(self)
        self.title = title
        self.sp = HTMLSharePath(filename=None,boilerplate=_VIS_HTML_BOILERPLATE)
        self.kw = KlamptWidget()
        self.animating = False
        self._textItems = set()
        
    def addAction(self,hook,short_text,key,description):
        raise NotImplementedError("Can't add actions to HTML frontends")

    def clear(self):
        VisualizationScene.clear(self)
        self.sp = HTMLSharePath(filename=None,boilerplate=_VIS_HTML_BOILERPLATE)
        self.kw = KlamptWidget()
        self.animating = False
        self._textItems = set()
        #reset the visualization clock
        self.startTime = None
        self.t = 0

    def clearText(self):
        VisualizationScene.clearText(self)
        if len(self._textItems) > 0:
            for t in self._textItems:
                self.kw.remove(t)
            self._textItems = set()

    def add(self,name,item,keepAppearance=False,**kwargs):
        VisualizationScene.add(self,name,item,keepAppearance,**kwargs)
        if isinstance(item,str):
            self._textItems.add(name)
            self.kw.addText(name,item,kwargs.get('position',None))
            if 'color' in kwargs:
                self.kw.setColor(self,name,*kwargs['color'])
            return
        if name=='world' or name=='sim':
            self.sp.start(item)
            if name == 'world':
                self.kw.world = item
            else:
                self.kw.world = item.world
        else:
            try:
                self.kw.add(name,item,**kwargs)
            except ValueError:
                raise ValueError("Can't draw items of type "+item.__class__.__name__+" in HTML form")
        
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

    def setColor(self,name,r,g,b,a=1):
        self.setAttribute(name,'color',(r,g,b,a))

    def setAttribute(self,name,attr,value):
        VisualizationScene.setAttribute(self,name,attr,value)
        item = self.getItem(name)
        if attr == 'color':
            self.kw.setColor(item.name,*value)
        elif attr == 'size':
            #modify point size
            if hasattr(item.item,'__iter__') and len(item.item)==3:
                self.kw.addSphere(item.name,item.item[0],item.item[1],item.item[2],value)
            pass
        #can't handle any other attributes right now
        
    def setDrawFunc(self,name,func):
        raise RuntimeError("IPython: can't set up custom draw functions")

    def getViewport(self):
        cam = self.kw.getCamera()
        #TODO: convert camera message to GLViewport
        return cam

    def setViewport(self,viewport):
        #TODO: convert from GLViewport to camera message
        self.kw.setCamera(viewport)
        
    def setBackgroundColor(self,r,g,b,a=1): 
        raise RuntimeError("IPython: can't yet change the background color")

    def stepAnimation(self,dt):
        VisualizationScene.stepAnimation(self,dt)
        self.updateCamera()
        #look through changed items and update them
        def updateItem(item):
            if isinstance(item,VisPlot):
                return
            if isinstance(item.item,WorldModel):
                return
            if item.transformChanged:
                if isinstance(item.item,list):
                    #it's either a ghost, a point, or an se3 item
                    item.swapDrawConfig()
                    self.kw.add(item.name,item.item)
                    item.swapDrawConfig()
                else:
                    raise NotImplementedError("TODO: update moved things of type",item.item.__class__.__name__)
                    #self.kw.setTransform(...)
                item.transformChanged = False
            for k,c in item.subAppearances.items():
                updateItem(c)
        for k,v in self.items.items():
            updateItem(v)
        
        if "world" in self.items:
            for k,v in self.items["world"].subAppearances.items():
                v.swapDrawConfig()
        self.sp.animate(self.currentAnimationTime,rpc=self.kw._rpc_calls)
        if "world" in self.items:
            for k,v in self.items["world"].subAppearances.items():
                v.swapDrawConfig()
        self.kw.endRpc(strict=True)
        self.kw.beginRpc(strict=True)
        self.updateTime(self.currentAnimationTime)

    def update(self):
        #self.kw.update()   #this isn't necessary
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
        t = time.time()
        if self.startTime is None:
            self.startTime = t
        t = t-self.startTime
        if t > self.t + 1.0/30.0: # 30fps
            if "world" in self.items:
                for k,v in self.items["world"].subAppearances.items():
                    v.swapDrawConfig()
            self.sp.animate(t,rpc=self.kw._rpc_calls)
            if "world" in self.items:
                for k,v in self.items["world"].subAppearances.items():
                    v.swapDrawConfig()
            self.kw.endRpc(strict=True)
            self.kw.beginRpc(strict=True)
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
        return '<iframe width="%d" height="%d" srcdoc="%s"></iframe>'%(w,h,html_escape_quotes(self.page()))

    def __str__(self):
        """Returns the HTML representation of this scene"""
        if len(self.kw._rpc_calls) > 0:
            return self.sp.end(self.kw._rpc_calls)
        else:
            return self.sp.end()

    def page(self):
        """Returns a full HTML page representing the scene"""
        return _PAGE_TEMPLATE.replace('__TITLE__',self.title)%(str(self),)
        



class HTMLWindowManager(_WindowManager):
    def __init__(self):
        self.windows = [HTMLVisualizationScene()]
        self.current_window = 0
        self.displayed = False
        self.quit = False
    def reset(self):
        self.windows = [HTMLVisualizationScene()]
        self.current_window = 0
        self.displayed = False
        self.quit = False
    def frontend(self):
        return self.windows[self.current_window]
    def scene(self):
        return self.windows[self.current_window]
    def createWindow(self,title):
        if title is None:
            title = "Klampt HTML output"
        self.windows.append(HTMLVisualizationScene(title))
        self.current_window = len(self.windows)-1
        return self.current_window
    def setWindow(self,id):
        assert id >= 0 and id < len(self.windows)
        self.current_window = id
    def setWindowName(self,name):
        self.windows[self.current_window].title = name
    def getWindow(self):
        return self.current_window
    def resizeWindow(self,w,h):
        self.frontend().kw.width = w
        self.frontend().kw.height = h
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
        if setup is not None:
            setup()
        self.quit = False
        iters = 0
        while not self.quit:
            if callback is not None:
                callback()
            iters += 1
            if iters > 5000:
                print("HTML visualization window run for more than 5000 iterations... breaking loop")
                self.quit = True
        self.show()
        if cleanup is not None:
            cleanup()
    def spin(self,duration):
        assert not math.isinf(duration),"Can't spin HTML visualization windows for an infinite amount of time"
        assert duration < 300,"Can't spin HTML visualization windows for more than 5 minutes (performance issue)"
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
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            display(HTML(self.frontend().iframe()))
    def shown(self):
        return self.displayed and not self.quit
    def hide(self):
        self.quit = True
    def update(self):
        self.frontend().update()
    
