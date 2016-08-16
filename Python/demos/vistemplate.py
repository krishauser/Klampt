#!/usr/bin/python

import sys
from klampt import *
from klampt import vis
from klampt.vis.glrobotprogram import GLSimulationProgram
from klampt.vis.glprogram import GLPluginProgram
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
    vis.add("coordinates",coordinates.manager())
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
    vis.edit("coordinates:ATHLETE:ankle roll 3")

    print "Visualization items:"
    vis.listItems(indent=2)

    #run the visualizer in a separate thread
    vis.show()
    iteration = 0
    while vis.shown():
        vis.lock()
        #TODO: you may modify the world here.  This line tests a sin wave.
        pt[2] = 1 + math.sin(iteration*0.03)
        vis.unlock()
        #changes to the visualization must be done outside the lock
        if (iteration / 100)%2 == 0:
            vis.hide("some blinking transform")
        else:
            vis.hide("some blinking transform",False)
        #this is another way of changing the point's data
        #vis.add("some point",[2,5,1 + math.sin(iteration*0.03)],keepAppearance=True)
        time.sleep(0.01)
        iteration += 1

    #Now testing ability to re-launch windows
    """
    print "Showing again..."
    vis.show()
    while vis.shown():
        time.sleep(0.01)
    """

    print "Doing a dialog..."
    vis.dialog()

    print "Doing a split screen program..."
    secondscreen = GLPluginProgram()
    widgets = GLWidgetPlugin()
    widgets.addWidget(RobotPoser(world.robot(0)))
    #update the coordinates every time the widget changes
    widgets.widgetchangefunc = (lambda self:coordinates.updateFromWorld())
    secondscreen.pushPlugin(widgets)
    vis.addPlugin(secondscreen)
    #vis.setPlugin(secondscreen)
    vis.show()
    while vis.shown():
        time.sleep(0.1)
    
    print "Showing a dialog, back to normal..."
    vis.setPlugin(None)
    vis.dialog()
    print "Showing again, back to normal..."
    vis.show()
    while vis.shown():
        time.sleep(0.01)

    print "Ending klampt.vis visualization."
    vis.kill()
