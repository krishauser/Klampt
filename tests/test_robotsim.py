import unittest
from klampt.sim import *

class robotsimTest(unittest.TestCase):

    def setUp(self):
        self.world = WorldModel()
        self.world.readFile('data/athlete_plane.xml')
        self.sim = SimpleSimulator(self.world)

    def test_getPIDGains(self):
        self.assertTrue(self.world.numRobots()==1)
        c = self.sim.controller(0)
        pid_gains = c.getPIDGains()
        self.assertIsNotNone(pid_gains)
        self.assertTrue(len(pid_gains),3)
        
if __name__ == '__main__':
    unittest.main()