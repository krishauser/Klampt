import unittest
from klampt.sim import *

class robotsimTest(unittest.TestCase):

    def setUp(self):
        self.world = WorldModel()

    def test_getStatus_unstable(self):
        self.world.loadTerrain('data/terrains/plane.tri')
        self.world.loadRobot('data/robots/swingup.rob')

        sim = SimpleSimulator(self.world)
        robot = self.world.robot(0)
        sim.controller(0).setPIDCommand(robot.getConfig(), robot.getVelocity())
        for i in range(10):
            sim.simulate(0.01)
        self.assertEqual(Simulator.STATUS_NORMAL, sim.getStatus(), sim.getStatusString())
        sim.controller(0).setPIDGains([0.0], [0.0], [0.0])
        robot.setTorqueLimits([1e6])
        robot.setVelocityLimits([1e18])
        robot.setAccelerationLimits([1e18])

        sim.controller(0).setTorque([1e6])
        sim.simulate(0.01)
        self.assertEqual(Simulator.STATUS_UNSTABLE, sim.getStatus(), '%f is the final velocity for the joint, should be infinite'%sim.getActualVelocity(0)[0])
        


    def test_getStatus_adaptive_time_stepping(self):
        pass

    def test_getStatus_unreliable(self):
        pass
        
if __name__ == '__main__':
    unittest.main()