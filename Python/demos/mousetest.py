from klampt import *
from klampt import vis
from klampt.vis.glinterface import *

class GLTest(GLPluginInterface):
    def __init__(self,world):
        GLPluginInterface.__init__(self)
        self.world = world
        self.q = world.robot(0).getConfig()
        
    def display(self):
        self.world.drawGL()

    def motionfunc(self,x,y,dx,dy):
        if 'shift' in self.modifiers():
            self.q[2] = float(y)/400
            self.q[3] = float(x)/400
            self.world.robot(0).setConfig(self.q)
            return True
        return False
        
    def idle(self):
        return True

if __name__ == "__main__":
    print """mousetest.py: A simple program where the mouse motion, when
    shift-clicking, gets translated into joint values for an animated robot."""

    world = WorldModel()
    res = world.readFile("../../data/tx90blocks.xml")
    if not res:
        raise RuntimeError("Unable to load world")
    #set a custom initial configuration of the world
    vis.setWindowTitle("mousetest.py")
    vis.run(GLTest(world))

