#!/usr/bin/python

import sys
import math
from klampt import *
from klampt import WidgetSet,PointPoser,RobotPoser
from klampt.glprogram import *

class GLWidgetProgram(GLRealtimeProgram):
    """A base class for using widgets.  Right-clicks are passed onto widgets.
    
    Subclasses should call self.widgetMaster.add() upon initializiation to
    add widgets to the program.
    """
    def __init__(self,world,name="My GL Widget Program"):
        GLRealtimeProgram.__init__(self,name)
        self.world = world
        self.widgetMaster = WidgetSet()
        self.widgetButton = 2  #right-clicks
        self.draggingWidget = False

    def display(self):
        #Put your display handler here
        #the next few lines draw everything but the robot
        for i in xrange(self.world.numTerrains()):
            self.world.terrain(i).drawGL()
        for i in xrange(self.world.numRigidObjects()):
            self.world.rigidObject(i).drawGL()
        for i in xrange(1,self.world.numRobots()):
            self.world.robot(i).drawGL()
        #this line will draw the robot
        self.widgetMaster.drawGL(self.viewport())

    def idle(self):
        self.widgetMaster.idle()
        if self.widgetMaster.wantsRedraw():
            self.refresh()

    def mousefunc(self,button,state,x,y):
        print "mouse",button,state,x,y
        if button==self.widgetButton:
            if state==0:
                if self.widgetMaster.beginDrag(x,self.height-y,self.viewport()):
                    self.draggingWidget = True
            else:
                if self.draggingWidget:
                    self.widgetMaster.endDrag()
                    self.draggingWidget = False
            if self.widgetMaster.wantsRedraw():
                self.refresh()
            self.lastx,self.lasty = x,y
            return
        GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y,dx,dy):
        if self.draggingWidget:
            self.widgetMaster.drag(dx,-dy,self.viewport())
            if self.widgetMaster.wantsRedraw():
                self.refresh()
        else:
            res = self.widgetMaster.hover(x,self.height-y,self.viewport())
            if self.widgetMaster.wantsRedraw():
                self.refresh()
            GLRealtimeProgram.motionfunc(self,x,y,dx,dy)

    def specialfunc(self,c,x,y):
        #Put your keyboard special character handler here
        print c,"pressed"
        self.widgetMaster.keypress(c)
        self.refresh()

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        print c,"pressed"
        self.widgetMaster.keypress(c)
        self.refresh()


class MyGLViewer(GLWidgetProgram):
    def __init__(self,world):
        GLWidgetProgram.__init__(self,world,"My GL program")
        self.poseWidget = PointPoser()
        self.widgetMaster.add(self.poseWidget)
        self.robotWidget = RobotPoser(world.robot(0))
        self.widgetMaster.add(self.robotWidget)

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example prints the config when [space] is pressed
        if c == ' ':
            config = self.robotWidget.get()
            print "Config:",config
            self.world.robot(0).setConfig(config)
        else:
            GLWidgetProgram.keyboardfunc(self,c,x,y)


if __name__ == "__main__":
    print "widgets.py: This example demonstrates how to use a widget to pose a robot"
    if len(sys.argv)<=1:
        print "USAGE: gltemplate.py [world_file]"
        exit()
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)
    viewer = MyGLViewer(world)
    viewer.run()
