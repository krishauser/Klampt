import unittest
from klampt.sim import *
from klampt import vis

class robotsimTest(unittest.TestCase):

    def setUp(self):
        self.world = WorldModel()

    def test_getStatus_unstable(self):
        self.world.loadRobot('data/robots/swingup.rob')
        self.assertEqual(self.world.numRobots(),1)

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

    def test_getStatus_status_is_matching(self):
        self.world.loadTerrain('data/terrains/plane.off')
        self.world.loadRobot('data/robots/pr2gripper.rob')
        self.world.loadRigidObject('data/objects/sphere_5cm.obj')
        self.assertEqual(self.world.numRobots(),1)
        self.assertEqual(self.world.numTerrains(),1)
        self.assertEqual(self.world.numRigidObjects(),1)
        robot = self.world.robot(0)
        sphere = self.world.rigidObject(0)

        pt = robot.link(0).getParentTransform()
        ct = sphere.getTransform()
        sphere.setTransform(ct[0], [0.14, 0, 0.028])
        robot.link(0).setParentTransform(pt[0], [.0, .0, 0.03])
        robot.setConfig([0.0, 1.0491851842008302, 1.0491875315795012, -1.0491852558664032, 1.0491875316019472])
        robot.setTorqueLimits([100, 100, 50, 100, 50])
        sim = SimpleSimulator(self.world)
        c = sim.controller(0)
        c.setPIDCommand([1.0], [.0])
        c.setPIDGains([15], [50], [0.1])
        c.setPIDCommand([0], [0])
        sim.simulate(0.1)

        if sim.getStatus() == Simulator.STATUS_UNSTABLE:
            self.assertEqual("unstable", sim.getStatusString())
        else:
            print "Warning, test test_getStatus_status_is_matching is useless as the simulation is now stable"

    def test_getStatus_adaptive_time_stepping(self):
        self.world.loadTerrain('data/terrains/plane.off')
        self.world.loadRigidObject('data/objects/sphere_5cm.obj')
        self.assertEqual(self.world.numRobots(),1)
        self.assertEqual(self.world.numRigidObjects(),1)
        sphere = self.world.rigidObject(0)
        mass = sphere.getMass()
        mass.setMass(100)
        mass.setInertia([100,100,100])
        sphere.setMass(mass)
        #need to set up contact parameters so the mesh doesn't just fall through too quickly
        cparams = sphere.getContactParameters()
        cparams.kStiffness = 10000.0
        cparams.kDamping = 2500.0
        sphere.setContactParameters(cparams)

        hadAdaptiveTimeStepping = False
        ct = sphere.getTransform()
        sphere.setTransform(ct[0], [0, 0, 0.0263])
        sim = SimpleSimulator(self.world)
        for i in range(20):
            sim.simulate(0.01)
            print "CURRENT STATUS",sim.getStatus()
            if sim.getStatus() == Simulator.STATUS_ADAPTIVE_TIME_STEPPING:
                hadAdaptiveTimeStepping = True

        self.assertEqual(hadAdaptiveTimeStepping,True)

    def test_getStatus_unreliable(self):
        self.world.loadTerrain('data/terrains/plane.off')
        self.world.loadRigidObject('data/objects/sphere_5cm.obj')
        self.assertEqual(self.world.numTerrains(),1)
        self.assertEqual(self.world.numRigidObjects(),1)
        sphere = self.world.rigidObject(0)
        mass = sphere.getMass()
        mass.setMass(1000)
        sphere.setMass(mass)

        ct = sphere.getTransform()
        sphere.setTransform(ct[0], [0, 0, 0.0263])
        sim = SimpleSimulator(self.world)
        sim.simulate(0.1)

        self.assertEqual(Simulator.STATUS_CONTACT_UNRELIABLE, sim.getStatus())
        
if __name__ == '__main__':
    unittest.main()