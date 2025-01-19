from ..visualization import _WindowManager,VisualizationScene,VisPlot,objectToVisType,_globalLock
from ...robotsim import WorldModel,RobotModel,RigidObjectModel,Geometry3D,GeometricPrimitive,PointCloud,TriangleMesh,ImplicitSurface,OccupancyGrid,ConvexHull
from ...model.trajectory import Trajectory,RobotTrajectory
from ...control.utils import TimedLooper
from ...io.rerun_convert import *
from klampt import Viewport
import math
import weakref
import time
import warnings
import rerun as rr

def log_rr(name : str, item, attributes : dict, world : WorldModel):
    if isinstance(item,WorldModel):
        log_rr_world(item,name,attributes.get('hide_label',False))
    elif isinstance(item,RobotModel):
        log_rr_robot(item,name,attributes.get('hide_label',False),attributes.get('color',None))
    elif isinstance(item,RigidObjectModel):
        log_rr_rigid_object(item,name,attributes.get('hide_label',False),attributes.get('color',None))
    elif isinstance(item,Geometry3D):
        log_rr_geometry(item,name,appearance=attributes.get('appearance',None),hide_label=attributes.get('hide_label',False),color=attributes.get('color',None))
    elif isinstance(item,(GeometricPrimitive,PointCloud,TriangleMesh,ImplicitSurface,OccupancyGrid,ConvexHull)):
        geom = Geometry3D(item)
        log_rr_geometry(geom,name,appearance=attributes.get('appearance',None),hide_label=attributes.get('hide_label',False),color=attributes.get('color',None))
    elif isinstance(item,Trajectory):
        nd = 0
        if len(item.milestones) >= 1:
            nd = len(item.milestones[0])
            robot = None
            if isinstance(item,RobotTrajectory):
                robot = item.robot
            elif world is not None:
                robot_index = attributes.get('robot',0)
                robot = world.robot(robot_index)
            if robot is not None and robot.numLinks() == nd:
                with _globalLock:
                    ees = attributes.get('end_effectors',[-1])
                    for ee in ees:
                        if ee < 0: ee = robot.numLinks()+ee
                        assert ee >= 0 and ee < robot.numLinks(),"Invalid end effector specified"
                        ee_traj = SE3Trajectory(item.times,[robot.link(ee).getTransform() for q in item.milestones])
                        log_rr_trajectory(ee_traj,name+'/'+str(ee),color=attributes.get('color',None),hide_label=attributes.get('hide_label',False))
                return
        if isinstance(item,SE3Trajectory) or nd in [2,3]:
            log_rr_trajectory(item,name,attributes.get('hide_label',False))
    else:
        type = objectToVisType(item,world)
        if type in ['Vector3','Point']:
            pt = point_to_rr([0.0]*3,attributes.get('size',5)*0.01,attributes.get('color',(1,0,0,1)))
            if attributes.get('hide_label',False):
                pt.labels = [name]
            rr.log(name,pt,static=True)
            update_rr_transform([so3.identity(),item],name)
        elif type == 'RigidTransform':
            log_rr_transform(item,name,attributes.get('length',0.1),attributes.get('hide_label',False)) 
        elif type == 'Config':
            assert world is not None
            robot = attributes.get('robot',0)
            robot = world.robot(robot)
            log_rr_robot(robot,name,attributes.get('hide_label',False),color=attributes.get('color',None))
        elif type == 'Config':
            assert world is not None
            robot = attributes.get('robot',0)
            robot = world.robot(robot)
            oldConfig = robot.getConfig()
            with _globalLock:
                robot.setConfig(item)
                log_rr_robot(robot,name,attributes.get('hide_label',False),color=attributes.get('color',None))
                robot.setConfig(oldConfig)
        elif type == 'Configs':
            assert world is not None
            robot = attributes.get('robot',0)
            robot = world.robot(robot)
            oldConfig = robot.getConfig()
            with _globalLock:
                for i,config in enumerate(item):
                    robot.setConfig(config)
                    log_rr_robot(robot,name+'/'+str(i),hide_label=True,color=attributes.get('color',None))
                robot.setConfig(oldConfig)
        else:
            raise ValueError("Unable to log item of type "+type)


def update_config_rr(name : str, item, attributes : dict, world : WorldModel):
    if isinstance(item,WorldModel):
        update_rr_world(item,name)
    elif isinstance(item,RobotModel):
        update_rr_robot(item,name)
    elif isinstance(item,RigidObjectModel):
        update_rr_rigid_object(item,name)
    elif isinstance(item,Geometry3D):
        update_rr_transform(item.getCurrentTransform(),name)
    elif isinstance(item,Trajectory):
        log_rr(name, item, attributes, world)
    else:
        type = objectToVisType(item,world)
        if type in ['Vector3','Point']:
            update_rr_transform([so3.identity(),item],name)
        elif type == 'RigidTransform':
            update_rr_transform(item,name) 
        elif type == 'Config':
            assert world is not None
            robot = attributes.get('robot',0)
            robot = world.robot(robot)
            update_rr_robot(robot,name)
        else:
            raise ValueError("Unable to update item of type "+type)


class KlamptRerunAdaptor(VisualizationScene):
    """Handles the conversion between vis calls and rerun."""
    def __init__(self):
        VisualizationScene.__init__(self)
        self._textItems = set()

    def addAction(self,hook,short_text,key,description):
        print("Unable to add actions to Rerun viewers. ",short_text," will not be available.")

    def clear(self):
        VisualizationScene.clear(self)
        self._textItems = set()

    def clearText(self):
        VisualizationScene.clearText(self)
        if len(self._textItems) > 0:
            rr.log('text',rr.Clear(recursive=True))
            self._textItems = set()

    def add(self,name,item,keepAppearance=False,**kwargs):
        if isinstance(item,str):
            return self.addText(name,item,**kwargs)
        VisualizationScene.add(self,name,item,keepAppearance,**kwargs)
        world = None
        try:
            world = self.getItem('world').item
        except Exception:
            pass
        log_rr(name,item,kwargs,world)

    def addText(self,name,text,position=None,**kwargs):
        self._textItems.add(name)
        rr.log('text',rr.TextDocument('\n'.join(self._textItems)))

    def remove(self,name):
        if name in self._textItems:
            self._textItems.remove(name)
        else:
            VisualizationScene.remove(self,name)
            rr.log(name,rr.Clear(recursive=True))

    def setItemConfig(self,name,value):
        VisualizationScene.setItemConfig(self,name,value)

    def hideLabel(self,name,hidden=True):
        #no labels, ignore silently
        pass

    def edit(self,name,doedit=True):
        print("Unable to edit anything in Rerun viewers.")

    def hide(self,name,hidden=True):
        VisualizationScene.hide(self,name,hidden)
        print("TODO: hide items in rerun viewer")

    def setColor(self,name,r,g,b,a=1):
        self.setAttribute(name,'color',(r,g,b,a))

    def setAttribute(self,name,attr,value):
        try:
            VisualizationScene.setAttribute(self,name,attr,value)
        except Exception:
            #hack -- config edit widgets access things that aren't in the scene
            pass
        item = self.getItem(name)
        world = None
        try:
            world = self.getItem('world').item
        except Exception:
            pass
        log_rr(name,item.item,item.attributes,world)
        
    def setDrawFunc(self,name,func):
        raise RuntimeError("Can't set up custom draw functions in Rerun")

    def getViewport(self):
        cam = Viewport()
        return cam

    def setViewport(self,viewport):
        print("Rerun: can't change the viewport through code")

    def setBackgroundColor(self,r,g,b,a=1): 
        print("Rerun: can't change the background color")

    def update(self):
        VisualizationScene.updateTime(self)
        if "world" in self.items:
            for k,v in self.items["world"].subAppearances.items():
                v.swapDrawConfig()
            update_rr_world(self.items['world'].item,'world')
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
                    update_config_rr(item.name,item.item,item.attributes,self.items['world'].item)
                    item.swapDrawConfig()
                else:
                    raise NotImplementedError("TODO: update moved things of type",item.item.__class__.__name__)
                item.transformChanged = False
            for k,c in item.subAppearances.items():
                updateItem(c)
        self.updateCamera()
        for k,v in self.items.items():
            updateItem(v)

    def display_plots(self):
        plots = dict()
        for (k,v) in self.items.items():
            if isinstance(v,VisPlot) and not v.attributes['hidden']:
                plots[k] = v
        if len(plots) == 0:
            return
        for k,v in plots.items():
            for i in v.items:
                if len(i.traces) == 1:
                    id = k+'/'+i.name
                    rr.log(id,rr.SeriesLine(name = i.name))
                    rr.send_columns(id,[t[0] for t in i.traces[0]],[t[1] for t in i.traces[0]])
                else:
                    for j,trace in enumerate(i.traces):
                        id = k+'/'+i+'/'+i.itemnames[j]
                        rr.log(id,rr.SeriesLine(name = i+'/'+i.itemnames[j]))
                        rr.send_columns(id,[t[0] for t in trace],[t[1] for t in trace])



class RerunWindowManager(_WindowManager):
    def __init__(self):
        self.windows = [KlamptRerunAdaptor()]
        self.current_window = 0
        self.displayed = False
        self.quit = False
        rr.init('Klampt',spawn=True)
    def reset(self):
        self.windows = [KlamptRerunAdaptor()]
        self.current_window = 0
        self.displayed = False
        self.quit = False
    def frontend(self):
        return self.windows[self.current_window]
    def scene(self):
        return self.windows[self.current_window]
    def createWindow(self,title):
        raise NotImplementedError("Rerun does not accept multiple windows")
        self.windows.append(KlamptRerunAdaptor())
        self.current_window = len(self.windows)-1
        return self.current_window
    def setWindow(self,id):
        assert id >= 0 and id < len(self.windows)
        self.current_window = id
    def getWindow(self):
        return self.current_window
    def setWindowName(self,name):
        warnings.warn("Rerun does not accept window names, skipping")
        pass
    def resizeWindow(self,w,h):
        self.frontend().width = w
        self.frontend().height = h
    def setPlugin(self,plugin):
        if plugin is not None:
            raise NotImplementedError("Rerun does not accept plugins")
    def pushPlugin(self,plugin):
        raise NotImplementedError("Rerun does not accept plugins")
    def popPlugin(self):
        raise NotImplementedError("Rerun does not accept plugins")
    def addPlugin(self,plugin):
        raise NotImplementedError("Rerun does not accept plugins")
    def lock(self):
        pass
    def unlock(self):
        pass
    def multithreaded(self):
        return True
    def run(self):
        self.spin(float('inf'))
    def loop(self,setup,callback,cleanup):
        if setup is not None:
            setup()
        self.show()
        dt = 0.04
        looper = TimedLooper(dt)
        while looper:
            if not self.shown():
                break
            if callback is not None:
                callback()
        if cleanup is not None:
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
        if self.displayed:
            warnings.warn("vis_ipython: showing widgets more than once can result in unexpected behavior")
        self.quit = False
        self.displayed = True
    def shown(self):
        return self.displayed and not self.quit
    def hide(self):
        self.quit = True
    def update(self):
        self.frontend().update()
        self.frontend().display_plots()
