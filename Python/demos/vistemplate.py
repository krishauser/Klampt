#!/usr/bin/python

import sys
from klampt import *
from klampt import vis
from klampt.robotsim import setRandomSeed
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import RobotPoser
from klampt.model import ik,coordinates
from klampt.math import so3
import time
import math

if __name__ == "__main__":
    print "vistemplate.py: This example demonstrates how to run the visualization framework"
    if len(sys.argv)<=1:
        print "USAGE: vistemplate.py [world_file]"
        exit()

    #creates a world and loads all the items on the command line
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)

    coordinates.setWorldModel(world)

    #add the world to the visualizer
    vis.add("world",world)
    #add the coordinate Manager to the visualizer
    #vis.add("coordinates",coordinates.manager())
    vp = vis.getViewport()
    vp.w,vp.h = 800,800
    vis.setViewport(vp)

    #do this if you want to test the robot configuration auto-fitting
    #vis.add("text1","Using a random configuration")
    #setRandomSeed(int(time.time()))
    #world.robot(0).randomizeConfig()
    
    #test a point
    pt = [2,5,1]
    vis.add("some point",pt)
    #test a rigid transform
    vis.add("some blinking transform",[so3.identity(),[1,3,0.5]])
    #test an IKObjective
    link = world.robot(0).link(world.robot(0).numLinks()-1)
    #point constraint
    obj = ik.objective(link,local=[[0,0,0]],world=[pt])
    #hinge constraint
    #obj = ik.objective(link,local=[[0,0,0],[0,0,0.1]],world=[pt,[pt[0],pt[1],pt[2]+0.1]])
    #transform constraint
    #obj = ik.objective(link,R=link.getTransform()[0],t=pt)
    vis.add("ik objective",obj)
    vis.edit("some point")
    #vis.edit("some blinking transform")
    #vis.edit("coordinates:ATHLETE:ankle roll 3")

    #test the on-screen text display
    vis.addText("text2","Here's some red text")
    vis.setColor("text2",1,0,0)
    vis.addText("text3","Here's bigger text")
    vis.setAttribute("text3","size",24)
    vis.addText("text4","Transform status")
    vis.addText("textbottom","Text anchored to bottom of screen",(20,-30))

    vis.addPlot('plot')
    vis.addPlotItem('plot','some point')
    vis.setPlotDuration('plot',10.0)

    print "Visualization items:"
    vis.listItems(indent=2)

    vis.autoFitCamera()

    print "Starting visualization window..."
    #run the visualizer, which runs in a separate thread
    vis.setWindowTitle("Basic visualization test")
    vis.show()
    iteration = 0
    while vis.shown():
        vis.lock()
        #TODO: you may modify the world here.  This line tests a sin wave.
        pt[2] = 1 + math.sin(iteration*0.03)
        vis.unlock()
        #changes to the visualization must be done outside the lock
        if (iteration % 100) == 0:
            if (iteration / 100)%2 == 0:
                vis.hide("some blinking transform")
                vis.addText("text4","The transform was hidden")
                vis.logPlotEvent('plot','hide')
            else:
                vis.hide("some blinking transform",False)
                vis.addText("text4","The transform was shown")
                vis.logPlotEvent('plot','show')
        #this is another way of changing the point's data
        #vis.add("some point",[2,5,1 + math.sin(iteration*0.03)],keepAppearance=True)
        time.sleep(0.01)
        iteration += 1
    vis.clearText()

    """
    #Now testing ability to re-launch windows
    print "Showing again..."
    vis.show()
    while vis.shown():
        time.sleep(0.01)
    """

    print "Doing a dialog..."
    vis.setWindowTitle("Dialog test")
    print "calling dialog()"
    vis.dialog()

    print "Doing a split screen program..."
    vp.w,vp.h = 640,480
    vis.setViewport(vp)
    for i in range(3):
        widgets = GLWidgetPlugin()
        widgets.addWidget(RobotPoser(world.robot(0)))
        #update the coordinates every time the widget changes
        widgets.widgetchangefunc = (lambda self:coordinates.updateFromWorld())
        vis.addPlugin(widgets)
    vis.setWindowTitle("Split screen test")
    vis.show()
    while vis.shown():
        time.sleep(0.1)
    
    print "Showing a dialog, back to normal..."
    vis.setPlugin(None)
    vis.dialog()
    print "Showing again, back to normal..."
    vis.setWindowTitle("Basic visualization test")
    vis.show()
    while vis.shown():
        time.sleep(0.01)

    print "Ending klampt.vis visualization."
    vis.kill()
