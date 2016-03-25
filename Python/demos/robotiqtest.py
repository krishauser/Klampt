#!/usr/bin/python
import sys
from klampt import *
from klampt.glrobotprogram import GLSimulationProgram
import robotiq 


class MyGLViewer(GLSimulationProgram):
    def __init__(self,world):
        GLSimulationProgram.__init__(self,world,"RobotiQ test program")
        #Put your initialization code here
        #the current example creates a collision class, simulator, 
        #simulation flag, and screenshot flags
        self.collider = robotcollide.WorldCollider(world)
        self.robotiq_emulator = robotiq.Emulator(self.sim)

    def control_loop(self):
        #Put your control handler here
        
        #TODO: right now, just sets g to an oscillation between 0 and 199
        controller = self.sim.controller(0)
        g = int(self.sim.getTime()*50.0)
        maxval = 120
        if int(g/maxval)%2 == 1:
            g = maxval-1 - g%maxval
        else:
            g = g % maxval
        print g
        g = [g,g,g]
        qdes = self.robotiq_emulator.send_command(g,scissor=30)


if __name__ == "__main__":
    print """robotiqtestpy: A program to test the behavior of the RobotiQ
    emulator.  Right now it just opens and closes the gripper repeatedly."""
    world = WorldModel()

    if not world.readFile('robotiq.xml'):
        print "robotiq.xml couldn't be read, exiting"
        exit(1)

    viewer = MyGLViewer(world)
    viewer.run()
