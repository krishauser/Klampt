from klampt import *
from klampt.glprogram import *
import numpy as np

class GLTest(GLRealtimeProgram):
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"GLTest")
        self.world = world
        self.q = world.robot(0).getConfig()
        
    def display(self):
        self.world.drawGL()

    def motionfunc(self,x,y,dx,dy):
        if self.modifiers & GLUT_ACTIVE_SHIFT:
            self.q[2] = float(y)/400
            self.q[3] = float(x)/400
            self.world.robot(0).setConfig(self.q)
        else:
            GLRealtimeProgram.motionfunc(self,x,y,dx,dy)
        
    def idle(self):
        pass

if __name__ == "__main__":
    print """mousetest.py: A simple program where the mouse motion, when
    shift-clicking, gets translated into joint values for an animated robot."""

    world = WorldModel()
    res = world.readFile("../data/tx90blocks.xml")
    if not res:
        raise RuntimeError("Unable to load world")
    #set a custom initial configuration of the world
    GLTest(world).run()

