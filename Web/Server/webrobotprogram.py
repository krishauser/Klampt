from klampt import *
import sys
sys.path.append("Web/Server")
from klampt import vectorops,so3,se3
import math
import time
import kviz


class WebRobotProgram:
    """A program that creates a world and serves it to a client.
    All visualization updates need to be handled via updating the world model
    and/or calls to the kviz module.

    Attributes:
    - world: the RobotWorld instance provided on startup.  All elements
      are assumed to be instantiated already.
    - JSON_message_count: the index of the currently sent frame
    - jString: the JSON string to be sent back to the client
    """
    def __init__(self,files):
        """Arguments:
        - files: names of local files to load
        """        
        #create a world from the given files
        world = WorldModel()
        for fn in files:
            print "trying to load:" + fn;
            res = world.readFile(fn)
            if not res:
                raise RuntimeError("Unable to load model "+fn)

        self.world = world


class WebSimulationProgram(WebRobotProgram):
    """A program that runs a simulation rather than a pure model.  A subclass of WebRobotProgram.

    Attributes:
    - sim: a Simulator for the given world.
        """
    def __init__(self,files):
        WebRobotProgram.__init__(self,files)
        self.sim = Simulator(self.world)
        self.dt = 1.0/50.0
        
    def advance(self):
        """Default: just advances the simulation and updates the world"""
        self.sim.simulate(self.dt)
        self.sim.updateWorld()

