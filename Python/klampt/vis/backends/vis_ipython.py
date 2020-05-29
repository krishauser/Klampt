from ..visualization import _WindowManager,VisualizationScene,VisPlot
from ..ipython import KlamptWidget
from IPython.display import display
import math

class KlamptWidgetAdaptor(KlamptWidget,VisualizationScene):
    """Handles the conversion between vis calls and the KlamptWidget."""
    def __init__(self):
        KlamptWidget.__init__(self)
        VisualizationScene.__init__(self)
        self._textItems = set()
        self.plot_axs = dict()
        self.axs = None

    def addAction(self,hook,short_text,key,description):
        raise NotImplementedError("Can't add actions to this frontend")

    def clear(self):
        KlamptWidget.clear(self)
        VisualizationScene.clear(self)
        self._textItems = set()

    def clearText(self):
        VisualizationScene.clearText(self)
        if len(self._textItems) > 0:
            self.beginRpc()
            for t in self._textItems:
                self.remove(t)
            self.endRpc()
            self._textItems = set()

    def add(self,name,item,keepAppearance=False,**kwargs):
        try:
            KlamptWidget.add(self,name,item)
        except ValueError:
            raise ValueError("Can't draw items of type "+item.__class__.__name__+" in Jupyter notebook")
        if 'color' in kwargs:
            KlamptWidget.setColor(self,name,*kwargs['color'])

        VisualizationScene.add(self,name,item,keepAppearance,**kwargs)

    def addText(self,name,text,pos=None):
        self._textItems.add(name)
        KlamptWidget.addText(self,name,text,pos)

    def remove(self,name):
        VisualizationScene.remove(self,name)
        KlamptWidget.remove(self,name)

    def setItemConfig(self,name,value):
        VisualizationScene.setItemConfig(self,name,value)

    def hideLabel(self,name,hidden=True):
        #no labels, ignore silently
        pass

    def edit(self,name,doedit=True):
        raise RuntimeError("IPython: can't do visual editing.  Try klampt.ipython.EditPoint, etc.")

    def hide(self,name,hidden=True):
        VisualizationScene.hide(self,name,hidden)
        KlamptWidget.hide(self,name,hidden)

    def _setAttribute(self,item,attr,value):
        VisualizationScene._setAttribute(self,item,attr,value)
        if attr == 'color':
            KlamptWidget.setColor(self,item.name,*value)
        elif attr == 'size':
            #TODO: modify point size
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
        VisualizationScene.updateAnimationTime(self)
        KlamptWidget.update(self)

        #look through changed items and update them
        def updateItem(item):
            if isinstance(item,VisPlot):
                return
            if item.doRefresh:
                raise NotImplementedError("TODO: update moved things")
                KlamptWidget.setTransform()
                item.doRefresh = False
            for k,c in item.subAppearances.items():
                updateItem(c)
        self.beginRpc()
        for k,v in self.items.items():
            updateItem(v)
        self.endRpc()

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
            print("Unable to redo plot layout")
        #render plots
        for (k,v) in plots.items():
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
            for i in v.items:
                for trace in i.traces:
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
    def frontend(self):
        return self.windows[self.current_window]
    def scene(self):
        return self.windows[self.current_window]
    def createWindow(self):
        self.windows.append(KlamptWidgetAdaptor())
        self.current_window = len(self.windows)-1
        return self.current_window
    def setWindow(self,id):
        assert id >= 0 and id < len(self.windows)
        self.current_window = id
    def getWindow(self):
        return self.current_window
    def setPlugin(self,plugin):
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
        setup()
        self.show()
        t = 0
        while t < duration:
            if not self.shown(): break
            callback()
        self.show(False)
        cleanup()
    def spin(self,duration):
        self.show()
        t = 0
        while t < duration:
            if not self.shown(): break
            time.sleep(min(0.04,duration-t))
            t += 0.04
        self.show(False)
    def show(self):
        self.quit = False
        self.displayed = True
        display(self.frontend())
        self.frontend().display_plots()
    def shown(self):
        return self.displayed and not self.quit
    def hide(self):
        self.quit = True
    def update(self):
        self.frontend().update()
    
