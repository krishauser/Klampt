from ..visualization import _WindowManager,VisualizationScene,VisPlot,objectToVisType,_globalLock
from ..ipython import KlamptWidget,EditPoint,EditTransform,EditConfig,Playback
from ...model import coordinates
from ...model.subrobot import SubRobotModel
from ...robotsim import WorldModel,RobotModel,RigidObjectModel
from IPython.display import display,HTML
import ipywidgets as widgets
import math
import weakref
import time
import warnings

class KlamptWidgetAdaptor(KlamptWidget,VisualizationScene):
    """Handles the conversion between vis calls and the KlamptWidget."""
    def __init__(self):
        KlamptWidget.__init__(self)
        VisualizationScene.__init__(self)
        self._textItems = set()
        self.plot_axs = dict()
        self.axs = None
        self._editors = dict()
        self._buttons = []

    def addAction(self,hook,short_text,key,description):
        w = max(75,9*len(short_text))
        item_layout = widgets.Layout(min_width=str(w)+'px')
        self._buttons.append(widgets.Button(description=short_text,layout=item_layout))
        self._buttons[-1].on_click(lambda _:hook())

    def clear(self):
        KlamptWidget.clear(self)
        VisualizationScene.clear(self)
        self._textItems = set()
        self._editors = dict()
        self.plot_axs = dict()
        self.axs = None

    def clearText(self):
        VisualizationScene.clearText(self)
        if len(self._textItems) > 0:
            self.beginRpc()
            for t in self._textItems:
                KlamptWidget.remove(self,t)
            self.endRpc()
            self._textItems = set()

    def add(self,name,item,keepAppearance=False,**kwargs):
        if isinstance(item,str):
            return self.addText(name,item,**kwargs)
        VisualizationScene.add(self,name,item,keepAppearance,**kwargs)
        try:
            KlamptWidget.add(self,name,item,**kwargs)
        except ValueError:
            raise ValueError("Can't draw items of type "+item.__class__.__name__+" in Jupyter notebook")

    def addText(self,name,text,position=None,**kwargs):
        self._textItems.add(name)
        KlamptWidget.addText(self,name,text,position)
        if 'color' in kwargs:
            KlamptWidget.setColor(self,name,*kwargs['color'])

    def remove(self,name):
        if name in self._textItems:
            self._textItems.remove(name)
            KlamptWidget.remove(self,name)
        else:
            VisualizationScene.remove(self,name)
            KlamptWidget.remove(self,name)
        if name in self._editors:
            del self._editors[name]

    def setItemConfig(self,name,value):
        VisualizationScene.setItemConfig(self,name,value)
        if name in self._editors:
            print("vis_ipython: TODO: update editor from setItemConfig")

    def hideLabel(self,name,hidden=True):
        #no labels, ignore silently
        pass

    def make_editor(self,obj,world):
        if obj.editor is not None:
            return 
        item = obj.item
        if isinstance(item,coordinates.Point):
            res = EditPoint(item.worldCoordinates(),klampt_widget=weakref.proxy(self),point_name=obj.name)
        elif isinstance(item,coordinates.Direction):
            res = EditPoint(item.worldCoordinates(),klampt_widget=weakref.proxy(self),point_name=obj.name)
        elif isinstance(item,coordinates.Frame):
            res = EditTransform(item.worldCoordinates(),klampt_widget=weakref.proxy(self),xform_name=obj.name)
        elif isinstance(item,RobotModel):
            link_selector = 'all'
            if item.numLinks() > 10:
                link_selector = 'dropdown'
            elif item.numLinks() > 4:
                link_selector = 'dropdown'
            res = EditConfig(item,klampt_widget=weakref.proxy(self),link_selector=link_selector)
        elif isinstance(item,SubRobotModel):
            link_selector = 'all'
            if len(item.links) > 10:
                link_selector = 'dropdown'
            elif len(item.links) > 4:
                link_selector = 'dropdown'
            res = EditConfig(item._robot,klampt_widget=weakref.proxy(self),link_subset=item.links,link_selector=link_selector)
        elif isinstance(item,RigidObjectModel):
            res = EditTransform(item.getTransform(),klampt_widget=weakref.proxy(self),xform_name=obj.name,callback=lambda T:item.setTranform(*T))
        elif isinstance(item,(list,tuple)):
            #determine if it's a rotation, transform, or point
            itype = objectToVisType(item,None)
            if itype == 'Vector3':
                res = EditPoint(item,klampt_widget=weakref.proxy(self),point_name=obj.name)
            elif itype == 'Matrix3':
                warnings.warn("vis_ipython.make_editor(): Editor for object of type %s not defined"%(itype,))
                return
            elif itype == 'RigidTransform':
                res = EditTransform(item,klampt_widget=weakref.proxy(self),xform_name=obj.name)
            elif itype == 'Config':
                if world is not None and world.numRobots() > 0 and world.robot(0).numLinks() == len(item):
                    #it's a valid configuration
                    oldconfig = world.robot(0).getConfig()
                    world.robot(0).setConfig(item)
                    link_selector = 'all'
                    if len(item) > 10:
                        link_selector = 'dropdown'
                    elif len(item) > 4:
                        link_selector = 'dropdown'
                    res = EditConfig(world.robot(0),ghost=obj.name,klampt_widget=weakref.proxy(self),link_selector=link_selector)
                    world.robot(0).setConfig(oldconfig)
                else:
                    warnings.warn("vis_ipython.make_editor(): Editor for object of type %s cannot be associated with a robot"%(itype,))
                    return
            else:
                warnings.warn("vis_ipython.make_editor(): Editor for object of type %s not defined"%(itype,))
                return
        else:
            warnings.warn("vis_ipython.make_editor(): Editor for object of type %s not defined"%(item.__class__.__name__,))
            return
        obj.editor = res

    def edit(self,name,doedit=True):
        global _globalLock
        obj = self.getItem(name)
        if obj is None:
            raise ValueError("Object "+name+" does not exist in visualization")
        if doedit:
            world = self.items.get('world',None)
            if world is not None:
                world=world.item
            self.make_editor(obj,world)
            if obj.editor is not None:
                self._editors[name] = obj.editor
                if self.displayed:
                    display(HTML('<h3>'+obj.name+'</h3>'))
                    display(obj.editor)
        else:
            if obj.editor:
                del self._editors[name]
        self.doRefresh = True
        return obj.editor if doedit else None

    def setGhostConfig(self,q,name="ghost",robot=0):
        """Needed for EditConfig to work on ghosts"""
        KlamptWidget.setGhostConfig(self,q,name,robot)
        obj = self.getItem(name)
        if obj is not None:
            obj.item = q

    def hide(self,name,hidden=True):
        VisualizationScene.hide(self,name,hidden)
        KlamptWidget.hide(self,name,hidden)

    def setColor(self,name,r,g,b,a=1):
        self.setAttribute(name,'color',(r,g,b,a))

    def setAttribute(self,name,attr,value):
        try:
            VisualizationScene.setAttribute(self,name,attr,value)
        except Exception:
            #hack -- config edit widgets access things that aren't in the scene
            pass
        item = self.getItem(name)
        if attr == 'color':
            KlamptWidget.setColor(self,item.name,*value)
        elif attr == 'size':
            #modify point size
            if hasattr(item.item,'__iter__') and len(item.item)==3:
                KlamptWidget.addSphere(self,item.name,item.item[0],item.item[1],item.item[2],value)
            pass
        #can't handle any other attributes right now

    def setDrawFunc(self,name,func):
        raise RuntimeError("IPython: can't set up custom draw functions")

    def getViewport(self):
        cam = KlamptWidget.getCamera(self)
        #TODO: convert camera message to GLViewport
        return cam

    def setViewport(self,viewport):
        #TODO: convert from GLViewport to camera message
        KlamptWidget.setCamera(self,viewport)

    def setBackgroundColor(self,r,g,b,a=1): 
        raise RuntimeError("IPython: can't yet change the background color")

    def update(self):
        if "world" in self.items:
            for k,v in self.items["world"].subAppearances.items():
                v.swapDrawConfig()
        KlamptWidget.update(self)
        if "world" in self.items:
            for k,v in self.items["world"].subAppearances.items():
                v.swapDrawConfig()

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
                    KlamptWidget.add(self,item.name,item.item)
                    item.swapDrawConfig()
                else:
                    raise NotImplementedError("TODO: update moved things of type",item.item.__class__.__name__)
                    #KlamptWidget.setTransform()
                item.transformChanged = False
            for k,c in item.subAppearances.items():
                updateItem(c)
        self.beginRpc()
        self.updateCamera()
        for k,v in self.items.items():
            updateItem(v)
        self.endRpc()

    def display_widgets(self):
        if self._buttons:
            if len(self._buttons) > 8:
                box_layout = widgets.Layout(overflow='scroll',
                        flex_flow='row',
                        display='flex')
                carousel = widgets.Box(children=self._buttons, layout=box_layout)
                display(carousel)
            else:
                display(widgets.HBox(children=self._buttons))
        for k,v in self._editors.items():
            display(HTML('<h3>'+k+'</h3>'))
            display(v)

    def display_plots(self):
        plots = dict()
        for (k,v) in self.items.items():
            if isinstance(v,VisPlot) and not v.attributes['hidden']:
                plots[k] = v
        if len(plots) == 0:
            return

        from matplotlib import pyplot as plt
        if len(self.plot_axs)==0:
            rows = int(math.floor(math.sqrt(len(plots))))
            cols = int(math.ceil(len(plots)/rows))
            self.axs = plt.subplots(rows,cols)
            plotcnt = 0
            for (k,v) in plots.items():
                self.plot_axs[k] = self.axs[plotcnt//cols][plotcnt%cols]
                plotcnt += 1
        if len(plots) != len(self.plot_axs):
            warnings.warn("vis_ipython: Unable to redo plot layout")
        #render plots
        for (k,v) in plots.items():
            vmin,vmax = v.autoRange()
            pos = v.attributes['position']
            duration = v.attributes['duration']
            vrange = v.attributes['range']
            w,h = v.attributes['size']
            if k not in self.plot_axs:
                continue
            ax = self.plot_axs[k]
            #determine last time
            tmax = 0
            for i in v.items:
                for trace in i.traces:
                    if len(trace)==0: continue
                    tmax = max(tmax,trace[-1][0])
            #plot 
            for item in v.items:
                for j,trace in enumerate(item.traces):
                    if len(trace)==0: continue
                    if len(item.name)==0:
                        label = item.itemnames[j]
                    else:
                        label = str(item.name) + '.' + item.itemnames[j]
                    x = []
                    y = []
                    tmax = max(tmax,trace[-1][0])
                    for k in range(len(trace)-1):
                        if trace[k+1][0] > tmax-duration:
                            u,v = trace[k]
                            if trace[k][0] < tmax-duration:
                                x.append(tmax-duration)
                                #interpolate so x is at tmax-duration
                                u2,v2 = trace[k+1]
                                #u + s(u2-u) = tmax-duration
                                s = (tmax-duration-u)/(u2-u)
                                v = v + s*(v2-v)
                                u = (tmax-duration)
                            x.append(u)
                            y.append(v)
                    u,v = trace[-1]
                    u = (u-(tmax-duration))/duration
                    v = (v-vmin)/(vmax-vmin)
                    x.append(u)
                    y.append(v)
                    ax.scatter(x,y,label=label)
            ax.set_title(k)
            ax.set_xlabel('time')
            ax.legend()
        plt.show()



class IPythonWindowManager(_WindowManager):
    def __init__(self):
        self.windows = [KlamptWidgetAdaptor()]
        self.current_window = 0
        self.displayed = False
        self.quit = False
    def reset(self):
        self.windows = [KlamptWidgetAdaptor()]
        self.current_window = 0
        self.displayed = False
        self.quit = False
    def frontend(self):
        return self.windows[self.current_window]
    def scene(self):
        return self.windows[self.current_window]
    def createWindow(self,title):
        self.windows.append(KlamptWidgetAdaptor())
        self.current_window = len(self.windows)-1
        return self.current_window
    def setWindow(self,id):
        assert id >= 0 and id < len(self.windows)
        self.current_window = id
    def getWindow(self):
        return self.current_window
    def setWindowName(self,name):
        warnings.warn("IPython does not accept window names, skipping")
        pass
    def resizeWindow(self,w,h):
        self.frontend().width = w
        self.frontend().height = h
    def setPlugin(self,plugin):
        if plugin is not None:
            raise NotImplementedError("IPython does not accept plugins")
    def pushPlugin(self,plugin):
        raise NotImplementedError("IPython does not accept plugins")
    def popPlugin(self):
        raise NotImplementedError("IPython does not accept plugins")
    def addPlugin(self,plugin):
        raise NotImplementedError("IPython does not accept plugins")
    def lock(self):
        for w in self.windows:
            w.beginRpc()
    def unlock(self):
        for w in self.windows:
            w.endRpc()
    def multithreaded(self):
        return False
    def run(self):
        self.spin(float('inf'))
    def loop(self,setup,callback,cleanup):
        if setup is not None:
            setup()
        playback = Playback(self.scene())
        playback.quiet = False
        playback.reset = setup
        playback.advance = callback
        if cleanup is not None:
            warnings.warn("vis_ipython: An IPython visualization will run forever, and cleanup() will never be called")
        self.show()
        display(playback)
    def spin(self,duration):
        self.show()
        t = 0
        while t < duration:
            if not self.shown(): break
            time.sleep(min(0.04,duration-t))
            t += 0.04
        self.show(False)
    def show(self):
        if self.displayed:
            warnings.warn("vis_ipython: showing widgets more than once can result in unexpected behavior")
        self.quit = False
        self.displayed = True
        display(self.frontend())
        self.frontend().display_widgets()
        self.frontend().display_plots()
    def shown(self):
        return self.displayed and not self.quit
    def hide(self):
        self.quit = True
    def update(self):
        self.frontend().update()
    
