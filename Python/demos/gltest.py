from robot import *
from robot.glprogram import *
import numpy as np

class GLTest(GLRealtimeProgram):
    def __init__(self,world,sim):
        GLRealtimeProgram.__init__(self,"GLTest")
        self.world = world
        self.sim = sim

    def display(self):
        self.sim.updateWorld()
        self.world.drawGL()

    def idle(self):
        rfs = sim.getController(0).getNamedSensor("RF_ForceSensor")
        print "Sensor values:",rfs.getMeasurements()
        sim.simulate(self.dt)
        return

if __name__ == "__main__":
    print "gltest.py: This example demonstrates how to simulate a world and read a force sensor"
    world = WorldModel()
    res = world.readFile("../data/hubo_plane.xml")
    if not res:
        raise RuntimeError("Unable to load world")
    sim = Simulator(world)
    GLTest(world,sim).run()

