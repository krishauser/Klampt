#!/usr/bin/python

import sys
import math
from klampt import *
from klampt import PointPoser,RobotPoser
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import vis


class MyGLViewer(GLWidgetPlugin):
    def __init__(self,world):
        GLWidgetPlugin.__init__(self)
        self.world = world
        self.poseWidget = PointPoser()
        self.robotWidget = RobotPoser(world.robot(0))
        self.addWidget(self.poseWidget)
        self.addWidget(self.robotWidget)

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example prints the config when [space] is pressed
        if c == ' ':
            config = self.robotWidget.get()
            print "Config:",config
            self.world.robot(0).setConfig(config)
        elif c == 's':
            fn = "widgets_test_world.xml"
            print "Saving file to",fn
            self.world.saveFile(fn)
        else:
            GLWidgetPlugin.keyboardfunc(self,c,x,y)



if __name__ == "__main__":
    print "widgets.py: This example demonstrates how to manually add visualization widgets to pose a robot"
    if len(sys.argv)<=1:
        print "USAGE: widgets.py [world_file]"
        exit()
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)
    viewer = MyGLViewer(world)
    vis.run(viewer)
