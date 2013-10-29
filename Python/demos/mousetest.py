from robot import *
from geometry.glprogram import *
import numpy as np

class GLTest(GLRealtimeProgram):
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"GLTest")
        self.world = world
        self.q = world.robot(0).getConfig()
        
    def display(self):
        self.world.drawGL()

    def motionfunc(self,x,y):
        if self.modifiers & GLUT_ACTIVE_SHIFT:
            self.q[2] = float(y)/400
            self.q[3] = float(x)/400
            self.world.robot(0).setConfig(self.q)
        else:
            GLRealtimeProgram.motionfunc(self,x,y)
        
    def idle(self):
        pass

if __name__ == "__main__":
    world = WorldModel()
    res = world.readFile("../tx90_files/tx90blocks.xml")
    if not res:
        raise RuntimeError("Unable to load world")
    #set a custom initial configuration of the world
    GLTest(world).run()

