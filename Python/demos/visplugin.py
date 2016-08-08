#!/usr/bin/python

import sys
from klampt import *
from klampt.math import *
from klampt import vis
from klampt.model import collide
import time

class MyGLPlugin(vis.GLPluginInterface):
    def __init__(self,world):
        vis.GLPluginInterface.__init__(self)
        self.world = world
        self.collider = collide.WorldCollider(world)
        self.quit = False

    def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        #the current example prints out the list of objects clicked whenever
        #you right click
        print "mouse",button,state,x,y
        if button==2:
            if state==0:
                print [o.getName() for o in self.click_world(x,y)]
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
        print self.window.width,self.window.height
        print s,d

        #run the collision tests
        collided = []
        for g in self.collider.geomList:
            (hit,pt) = g[1].rayCast(s,d)
            if hit:
                dist = vectorops.dot(vectorops.sub(pt,s),d)
                collided.append((dist,g[0]))
        return [g[1] for g in sorted(collided)]

if __name__ == "__main__":
    print "visplugin.py: This example demonstrates how to simulate a world and read user input using the klampt.vis framework"
    if len(sys.argv)<=1:
        print "USAGE: visplugin.py [world_file]"
        exit()

    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)
    world.enableInitCollisions(True)

    print "Press 'q' to exit"
    #the plugin is pushed on the visualizer stack to get interactivity with the visualizer... if you
    #don't care about interactivity, you may leave it out.  See vistemplate.py for
    #an example of this
    plugin = MyGLPlugin(world)
    vis.pushPlugin(plugin)
    #add the world to the visualizer
    vis.add("world",world)
    vis.listItems()
    vis.setColor("world:Terrain0",1,0,0)
    vis.setColor("world:ATHLETE:hex pitch",0.5,0.5,0.5,0.5)
    #run the visualizer in a separate thread
    vis.show()
    while vis.shown() and not plugin.quit:
        vis.lock()
        #TODO: you may modify the world here
        vis.unlock()
        #changes to the visualization must be done outside the lock
        time.sleep(0.01)
    print "Ending visualization."
    vis.kill()
