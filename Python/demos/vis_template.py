#!/usr/bin/python

import sys
from klampt import *
from klampt import vis
from klampt.robotsim import setRandomSeed
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import RobotPoser
from klampt.model import ik,coordinates,config,trajectory,collide
from klampt.math import vectorops,so3,se3
from klampt.vis import GLSimulationPlugin
import time
import math

#set this to True to test multi-threaded visualization
MULTITHREADED = False
#set this to True to demonstrate the code for manually animating a path
MANUAL_ANIMATION = False
#set this to True to demonstrate the code for manually setting up editing widgets in a GLPluginInterface
MANUAL_EDITING = False


def basic_template(world):
    """Shows how to pop up a visualization window with a world"""
    #add the world to the visualizer
    vis.add("world",world)

    #adding a point
    vis.add("point",[1,1,1])
    vis.setColor("point",0,1,0)
    vis.setAttribute("point","size",5.0)

    #adding lines is currently not super convenient because a list of lists is treated as
    #a Configs object... this is a workaround to force the vis module to treat it as a polyline.
    vis.add("line",trajectory.Trajectory([0,1],[[0,0,0],[1,1,1]]))
    vis.setAttribute("line","width",1.0)

    sphere = GeometricPrimitive()
    sphere.setSphere([1.5,1,1],0.2)
    vis.add("sphere",sphere)
    vis.setColor("sphere",0,0,1,0.5)

    box = GeometricPrimitive()
    box.setAABB([-1,-1,0],[-0.9,-0.9,0.2])
    g = Geometry3D(box)
    vis.add("box",g)
    vis.setColor("box",0,0,1,0.5)

    vis.setWindowTitle("Basic visualization test")
    if not MULTITHREADED:
        #single-threaded code
        def callback():
            #TODO: you may modify the world here.      
            pass
        vis.loop(setup=vis.show, callback=callback)   
    else:
        #multithreaded code
        vis.show()
        while vis.shown():
            vis.lock()
            #TODO: you may modify the world here.  Do not modify the internal state of any
            #visualization items outside of the lock
            vis.unlock()
            #outside of the lock you can use any vis.X functions, including vis.setItemConfig()
            #to modify the state of objects
            time.sleep(0.01)
    #quit the visualization thread nicely
    vis.kill()

def edit_template(world):
    """Shows how to pop up a visualization window with a world in which the robot configuration and a transform can be edited"""
    #add the world to the visualizer
    vis.add("world",world)
    xform = se3.identity()
    vis.add("transform",xform)
    robotPath = ("world",world.robot(0).getName())  #compound item reference: refers to robot 0 in the world
    vis.edit(robotPath)   
    vis.edit("transform")

    #This prints how to get references to items in the visualizer
    print "Visualization items:"
    vis.listItems(indent=2)

    vis.setWindowTitle("Visualization editing test")
    if not MULTITHREADED:
        vis.loop(setup=vis.show)
    else:
        vis.show()
        while vis.shown():
            vis.lock()
            #TODO: you may modify the world here.
            vis.unlock()
            time.sleep(0.01)
    print "Resulting configuration",vis.getItemConfig(robotPath)
    print "Resulting transform (config)",vis.getItemConfig("transform")  # this is a vector describing the item parameters
    xform = list(xform)  #convert se3 element from tuple to list
    config.setConfig(xform,vis.getItemConfig("transform"))
    print "Resulting transform (se3)",xform
    #quit the visualization thread nicely
    vis.kill()

def animation_template(world):
    """Shows how to animate a robot."""
    #first, build a trajectory with 10 random configurations
    robot = world.robot(0)
    times = range(10)
    milestones = []
    for t in times:
        robot.randomizeConfig()
        milestones.append(robot.getConfig())
    traj = trajectory.RobotTrajectory(robot,times,milestones)
    vis.add("world",world)
    robotPath = ("world",world.robot(0).getName())  #compound item reference: refers to robot 0 in the world

    #we're also going to visualize the end effector trajectory
    #eetraj = traj.getLinkTrajectory(robot.numLinks()-1,0.05)
    #vis.add("end effector trajectory",eetraj)

    #uncomment this to automatically visualize the end effector trajectory
    vis.add("robot trajectory",traj)
    vis.setAttribute("robot trajectory","endeffectors",[13,20])

    vis.setWindowTitle("Animation test")
    MANUAL_ANIMATION = False
    if not MANUAL_ANIMATION:
        #automatic animation, just call vis.animate
        vis.animate(robotPath,traj)
    if not MULTITHREADED:
        def callback():
            if MANUAL_ANIMATION:
                #with manual animation, you just set the robot's configuration based on the current time.
                t = vis.animationTime()
                q = traj.eval(t,endBehavior='loop')
                robot.setConfig(q)
            pass    
        vis.loop(callback=callback,setup=vis.show)
    else:
        vis.show()
        while vis.shown():
            vis.lock()
            if MANUAL_ANIMATION:
                #with manual animation, you just set the robot's configuration based on the current time.
                t = vis.animationTime()
                q = traj.eval(t,endBehavior='loop')
                robot.setConfig(q)
            vis.unlock()
            time.sleep(0.01)
    #quit the visualization thread nicely
    vis.kill()

def coordinates_template(world):
    """Tests integration with the coordinates module."""
    #add the world to the visualizer
    vis.add("world",world)
    coordinates.setWorldModel(world)
    #add the coordinate Manager to the visualizer
    vis.add("coordinates",coordinates.manager())

    vis.setWindowTitle("Coordinates visualiation test")
    if MANUAL_EDITING:
        #manually adds a poser, and adds a callback whenever the widget changes
        widgets = GLWidgetPlugin()
        widgets.addWidget(RobotPoser(world.robot(0)))
        #update the coordinates every time the widget changes
        widgets.widgetchangefunc = (lambda self:coordinates.updateFromWorld())
        vis.pushPlugin(widgets)
        if not MULTITHREADED:
            vis.loop(callback=None,setup=vis.show)
        else:
            vis.show()
            while vis.shown():
                time.sleep(0.01)
    else:
        vis.edit(("world",world.robot(0).getName()))
        if not MULTITHREADED:
            def callback():
                coordinates.updateFromWorld()
            vis.loop(callback=callback,setup=vis.show)
        else:
            vis.show()
            while vis.shown():
                vis.lock()
                #reads the coordinates from the world
                coordinates.updateFromWorld()
                vis.unlock()
                time.sleep(0.01)

    #quit the visualization thread nicely
    vis.kill()

def viewport_template(world):
    """Changes the parameters of the viewport and main window"""
    #add the world to the visualizer
    vis.add("world",world)
    vp = vis.getViewport()
    vp.w,vp.h = 800,800
    vis.setViewport(vp)

    #this auto-sizes the camera
    vis.autoFitCamera()
    vis.setWindowTitle("Viewport modification test")
    vis.spin(float('inf'))
    vis.kill()

def multiwindow_template(world):
    """Tests multiple windows and views."""
    vis.add("world",world)
    vp = vis.getViewport()
    vp.w,vp.h = 800,800
    vis.setViewport(vp)

    vis.setWindowTitle("vis.spin test: will close in 5 seconds...")
    vis.spin(5.0)

    #Now testing ability to re-launch windows
    vis.setWindowTitle("Shown again.  Close me to proceed.")
    vis.spin(float('inf'))

    vis.setWindowTitle("Dialog test. Close me to proceed.")
    vp = vis.getViewport()
    vp.w,vp.h = 400,600
    vis.setViewport(vp)
    vis.dialog()

    vp.w,vp.h = 640,480
    vis.setViewport(vp)
    for i in range(3):
        widgets = GLWidgetPlugin()
        widgets.addWidget(RobotPoser(world.robot(0)))
        vis.addPlugin(widgets)
    vis.setWindowTitle("Split screen test")
    vis.spin(float('inf'))
    
    vis.setPlugin(None)
    vis.setWindowTitle("Back to normal. Close me to quit.")
    vis.dialog()
    vis.kill()


def modification_template(world):
    """Tests a variety of miscellaneous vis functions"""
    vis.add("world",world)

    robot = world.robot(0)
    vis.setColor(("world",world.terrain(0).getName()),1,0,0,0.5)     #turn the terrain red and 50% opaque
    import random
    for i in range(10):
        #set some links to random colors
        randlink = random.randint(0,robot.numLinks()-1)
        color = (random.random(),random.random(),random.random())
        vis.setColor(("world",robot.getName(),robot.link(randlink).getName()),*color)

    #test the on-screen text display
    vis.addText("text2","Here's some red text")
    vis.setColor("text2",1,0,0)
    vis.addText("text3","Here's bigger text")
    vis.setAttribute("text3","size",24)
    vis.addText("text4","Transform status")
    vis.addText("textbottom","Text anchored to bottom of screen",(20,-30))
    
    #test a point
    pt = [2,5,1]
    vis.add("some point",pt)
    #test a rigid transform
    vis.add("some blinking transform",[so3.identity(),[1,3,0.5]])
    vis.edit("some point")
    #vis.edit("some blinking transform")
    #vis.edit("coordinates:ATHLETE:ankle roll 3")

    #test an IKObjective
    link = world.robot(0).link(world.robot(0).numLinks()-1)
    #point constraint
    obj = ik.objective(link,local=[[0,0,0]],world=[pt])
    #hinge constraint
    #obj = ik.objective(link,local=[[0,0,0],[0,0,0.1]],world=[pt,[pt[0],pt[1],pt[2]+0.1]])
    #transform constraint
    #obj = ik.objective(link,R=link.getTransform()[0],t=pt)
    vis.add("ik objective",obj)

    #enable plotting
    vis.addPlot('plot')
    vis.addPlotItem('plot','some point')
    vis.setPlotDuration('plot',10.0)

    #run the visualizer, which runs in a separate thread
    vis.setWindowTitle("Manual animation visualization test")
    iteration = 0
    def change_callback():
        vis.lock()
        #TODO: you may modify the world here.  This line tests a sin wave.
        pt[2] = 1 + math.sin(iteration*0.03)
        vis.unlock()
        #changes to the visualization with vis.X functions can done outside the lock
        if (iteration % 100) == 0:
            if (iteration / 100)%2 == 0:
                vis.hide("some blinking transform")
                vis.addText("text4","The transform was hidden")
                vis.logPlotEvent('plot','hide')
            else:
                vis.hide("some blinking transform",False)
                vis.addText("text4","The transform was shown")
                vis.logPlotEvent('plot','show')
        #this is another way of changing the point's data without needing a lock/unlock
        #vis.add("some point",[2,5,1 + math.sin(iteration*0.03)],keepAppearance=True)
        #or
        #vis.setItemConfig("some point",[2,5,1 + math.sin(iteration*0.03)])

        if iteration == 200:
            vis.addText("text2","Going to hide the text for a second...")
        if iteration == 400:
            #use this to remove text
            vis.clearText()
        if iteration == 500:
            vis.addText("text2","Text added back again")
            vis.setColor("text2",1,0,0)
        iteration += 1

    if not MULTITHREADED:
        vis.loop(callback=change_callback,setup=vis.show)
    else:
        vis.show()
        while vis.shown():
            change_callback()
            time.sleep(0.01)
    
    #use this to remove a plot
    vis.remove("plot")
    vis.kill()

class MyGLPlugin(vis.GLPluginInterface):
    def __init__(self,world):
        vis.GLPluginInterface.__init__(self)
        self.world = world
        self.collider = collide.WorldCollider(world)
        self.quit = False

        #adds an action to the window's menu
        def doquit():
            self.quit = True
        self.add_action(doquit,"Quit",'Ctrl+q',"Quit the program")

    def initialize(self):
        vis.add("instructions1","Right-click to get the list of intersecting items")
        vis.add("instructions2","Press q, Ctrl+q, or select Quit from the menu to quit")
        vis.GLPluginInterface.initialize(self)
        return True

    def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        #the current example prints out the list of objects clicked whenever
        #you right click
        print "mouse",button,state,x,y
        if button==2:
            if state==0:
                print "Click list...",[o.getName() for o in self.click_world(x,y)]
            return True
        return False

    def motionfunc(self,x,y,dx,dy):
        return False

    def keyboardfunc(self,c,x,y):
        print "Pressed",c
        if c == 'q':
            self.quit = True
            return True
        return False

    def click_world(self,x,y):
        """Helper: returns a list of world objects sorted in order of
        increasing distance."""
        #get the viewport ray
        (s,d) = self.click_ray(x,y)

        #run the collision tests
        collided = []
        for g in self.collider.geomList:
            (hit,pt) = g[1].rayCast(s,d)
            if hit:
                dist = vectorops.dot(vectorops.sub(pt,s),d)
                collided.append((dist,g[0]))
        return [g[1] for g in sorted(collided)]

def plugin_template(world):
    """Demonstrates the GLPluginInterface functionality"""
    #create a subclass of GLPluginInterface
    plugin = MyGLPlugin(world)
    vis.pushPlugin(plugin)   #put the plugin on top of the standard visualization functionality.
    #vis.setPlugin(plugin)   #use this to completely replace the standard visualization functionality with your own.

    vis.add("world",world)
    vis.setWindowTitle("GLPluginInterface template")
    #run the visualizer 
    if not MULTITHREADED:
        def callback():
            if plugin.quit:
                vis.show(False)
        vis.loop(callback=callback,setup=vis.show)
    else:
        #if plugin.quit is True
        vis.show()
        while vis.shown() and not plugin.quit:
            vis.lock()
            #TODO: you may modify the world here
            vis.unlock()
            #changes to the visualization must be done outside the lock
            time.sleep(0.01)
        if plugin.quit:
            #if you want to do other stuff after the window quits, the window needs to be hidden 
            vis.show(False)
    print "Waiting for 2 s..."
    time.sleep(2.0)
    #quit the visualization thread nicely
    vis.kill()


class MyGLSimulationViewer(GLSimulationPlugin):
    """A custom simulation plugin that allows moving to random configurations"""
    def __init__(self,world):
        #initialize the simulation
        GLSimulationPlugin.__init__(self,world)

        #put custom action hooks here
        self.add_action(self.some_function,'Some random function','f')
        self.add_action(self.go_to_random,'Go to a random configuration (trajectory module)','r')
        self.add_action(self.go_to_random_milestone,'Go to a random milestone (controller)','m')
        self.iterations = 0
        self.trajectory = None
        self.trajectoryStart = None

    def some_function(self):
        print "some_function() is called"

    def go_to_random(self):
        """Moves to a random destination using the trajectory module and tracking the trajectory using PID commands"""
        c = self.sim.controller(0)
        robot = self.world.robot(0)
        q0 = c.getCommandedConfig()
        robot.randomizeConfig()
        q1 = robot.getConfig()
        times = [0,5.0]
        milestones = [q0,q1]
        self.trajectory = trajectory.path_to_trajectory(trajectory.RobotTrajectory(robot,times,milestones),velocities='trapezoidal')
        self.trajectoryStart = self.sim.getTime()

    def go_to_random_milestone(self):
        """Moves to a random destination using the default controller"""
        c = self.sim.controller(0)
        robot = self.world.robot(0)
        robot.randomizeConfig()
        c.setMilestone(robot.getConfig())

        #turn off trajectory tracking, if on
        self.trajectory = None

    def control_loop(self):
        #Put your control handler here
        if self.iterations < 100:
            print "Control loop",self.iterations,"is called"
        elif self.iterations == 100:
            print "... you get the idea ..."
        self.iterations += 1
        if self.trajectory is not None:
            #run a trajectory controller
            c = self.sim.controller(0)
            t = self.sim.getTime()
            q = None
            if t - self.trajectoryStart > self.trajectory.duration():
                q = self.trajectory.milestones[-1]
                self.trajectory = None
            else:
                q= self.trajectory.eval(t - self.trajectoryStart)
            nlinks = len(q)
            c.setPIDCommand(q,[0]*nlinks)

    def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        #the current example prints out the list of objects clicked whenever
        #you right click
        print "mouse",button,state,x,y
        if button==2:
            if state==0:
                print [o.getName() for o in self.click_world(x,y)]
            return
        GLSimulationPlugin.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y,dx,dy):
        return GLSimulationPlugin.motionfunc(self,x,y,dx,dy)

def simulation_template(world):
    """Runs a custom simulation plugin"""
    viewer = MyGLSimulationViewer(world)
    vis.run(viewer)
    vis.kill()

#Code for the QT template
from klampt.vis import glinit
if glinit._PyQtAvailable:
    from PyQt5.QtWidgets import *
    class MyQtMainWindow(QMainWindow):
        def __init__(self,klamptGLWindow):
            """When called, this be given a QtGLWidget object(found in klampt.vis.qtbackend).

            You will need to place this widget into your Qt window's layout.
            """
            QMainWindow.__init__(self)
             # Splitter to show 2 views in same widget easily.
            self.splitter = QSplitter()
            self.left = QFrame()
            self.leftLayout = QVBoxLayout()
            self.left.setLayout(self.leftLayout)
            self.right = QFrame()
            self.rightLayout = QVBoxLayout()
            self.right.setLayout(self.rightLayout)
            
            self.glwidget = klamptGLWindow
            self.glwidget.setParent(self.right)
            self.rightLayout.addWidget(self.glwidget)
        
            self.helloButton = QPushButton("Hello world!")
            self.leftLayout.addWidget(self.helloButton)

            self.splitter.addWidget(self.left)
            self.splitter.addWidget(self.right)
            self.splitter.setHandleWidth(7)
            self.setCentralWidget(self.splitter)

        def closeEvent(self,event):
            reply = QMessageBox.question(self, "Confirm quit", "Do you really want to quit?",
                                    QMessageBox.Yes|QMessageBox.No);
            if reply == QMessageBox.Yes:
                vis.show(False)

def qt_template(world):
    """Runs a custom Qt frame around a visualization window"""
    if not glinit._PyQtAvailable:
        print "PyQt5 is not available on your system, try sudo apt-get install python-qt5"
        return
    
    #Qt objects must be explicitly deleted for some reason in PyQt5...
    g_mainwindow = None
    #All Qt functions must be called in the vis thread.
    #To hook into that thread, you will need to pass a window creation function into vis.customUI.
    def makefunc(gl_backend):
        global g_mainwindow
        g_mainwindow = MyQtMainWindow(gl_backend)
        return g_mainwindow
    vis.customUI(makefunc)
    vis.add("world",world)
    vis.setWindowTitle("Klamp't Qt test")
    vis.spin(float('inf'))
    vis.kill()
    #Safe cleanup of all Qt objects created in makefunc.
    #If you don't have this, PyQt5 complains about object destructors being called from the wrong thread
    del g_mainwindow

if __name__ == "__main__":
    print """================================================================================
    vis_template.py: Demonstrates examples about how to run the visualization framework.
    """
    if len(sys.argv)<=1:
        print "USAGE: vis_template.py [world_file]"
        print "   (Try python vis_template.py ../../data/athlete_plane.xml)"
    print """================================================================================"""
    if len(sys.argv)<=1:
        exit()

    #creates a world and loads all the items on the command line
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)

    templates = {'1':basic_template,'2':edit_template,'3':animation_template,
                '4':coordinates_template,'5':multiwindow_template,'6':modification_template,
                '7':plugin_template,'8':simulation_template,'9':qt_template}
    print "Available templates"
    import inspect
    for k in sorted(templates.keys()):
        fname = 'untitled'
        for x in inspect.getmembers(templates[k]):
            if x[0] == '__name__':
                fname = x[1]
        print " %s) %s: %s"%(k,fname,inspect.getdoc(templates[k]))
    entry = raw_input("Which template do you want to run? (1-%d) > "%(len(templates),))
    if entry not in templates:
        print "Invalid selection"
        exit(1)
    templates[entry](world)
