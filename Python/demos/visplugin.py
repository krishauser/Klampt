#!/usr/bin/python

import sys
from klampt import *
from klampt import visualization
from klampt.glcommon import *
import time

class MyGLPlugin(GLPluginBase):
    def __init__(self,world):
        self.world = world
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

if __name__ == "__main__":
    print "visplugin.py: This example demonstrates how to simulate a world and read user input using the visualization framework"
    if len(sys.argv)<=1:
        print "USAGE: visplugin.py [world_file]"
        exit()

    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)

    print "Press 'q' to exit"
    #the plugin is used to get interactivity with the visualizer... if you
    #don't care about interactivity, you may leave it out.  See vistemplate.py for
    #an example of this
    plugin = MyGLPlugin(world)
    visualization.setPlugin(plugin)
    #add the world to the visualizer
    visualization.add("world",world)
    #run the visualizer in a separate thread
    visualization.show()
    while visualization.shown() and not plugin.quit:
        visualization.lock()
        #TODO: you may modify the world here
        visualization.unlock()
        #changes to the visualization must be done outside the lock
        time.sleep(0.01)
    print "Ending visualization."
    visualization.kill()
