#!/usr/bin/python

import sys
from klampt import *
from klampt import visualization
from klampt import coordinates
from klampt import so3
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
    visualization.add("world",world)
    #add the coordinate Manager to the visualizer
    visualization.add("coordinates",coordinates.manager())
    #test a point
    pt = [2,5,1]
    visualization.add("some point",pt)
    #test a rigid transform
    visualization.add("some blinking transform",[so3.identity(),[1,3,0.5]])
    #test an IKObjective
    link = world.robot(0).link(world.robot(0).numLinks()-1)
    #point constraint
    #obj = ik.objective(link,local=[[0,0,0]],world=[pt])
    #hinge constraint
    obj = ik.objective(link,local=[[0,0,0],[0,0,0.1]],world=[pt,[pt[0],pt[1],pt[2]+0.1]])
    #transform constraint
    #obj = ik.objective(link,R=link.getTransform()[0],t=pt)
    visualization.add("ik objective",obj)

    print "Starting visualization..."
    #run the visualizer in a separate thread
    visualization.show()
    iteration = 0
    while visualization.shown():
        visualization.lock()
        #TODO: you may modify the world here.  This line tests a sin wave.
        pt[2] = 1 + math.sin(iteration*0.03)
        visualization.unlock()
        #changes to the visualization must be done outside the lock
        if (iteration / 100)%2 == 0:
            visualization.hide("some blinking transform")
        else:
            visualization.hide("some blinking transform",False)
        #this is another way of changing the point's data
        #visualization.add("some point",[2,5,1 + math.sin(iteration*0.03)],keepAppearance=True)
        time.sleep(0.01)
        iteration += 1
    
    print "Ending visualization."
    visualization.kill()
