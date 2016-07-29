#!/usr/bin/python
import sys
from klampt import *
from klampt.glrobotprogram import GLSimulationProgram
import robotiq 


class MyGLViewer(GLSimulationProgram):
    def __init__(self,world):
        GLSimulationProgram.__init__(self,world,"RobotiQ test program")
        #Put any controller modules or sensor / actuator emulators here
        self.robotiqEmulator = robotiq.Emulator(self.sim)
        self.sim.addEmulator(0,self.robotiqEmulator)

    def control_loop(self):
        #Put your control handler here
        
        #right now, just sets g to an oscillation between 0 and 199
        #TODO: build a BaseController that outputs qcmd to the emulator
        g = int(self.sim.getTime()*50.0)
        maxval = 120
        if int(g/maxval)%2 == 1:
            g = maxval-1 - g%maxval
        else:
            g = g % maxval
        #print g
        g = [g,g,g]

        self.robotiqEmulator.send_command(g,scissor=30)


if __name__ == "__main__":
    print """robotiqtest.py: A program to test the behavior of the RobotiQ
    emulator.  Right now it just opens and closes the gripper repeatedly.

    Press s to toggle simulation."""
    world = WorldModel()

    if not world.readFile('robotiq.xml'):
        print "robotiq.xml couldn't be read, exiting"
        exit(1)

    viewer = MyGLViewer(world)
    viewer.run()
