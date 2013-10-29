"""A test script that does some assorted testing of the robot module"""

from robot import *
from robot import rootfind
import time

world = WorldModel()
world.readFile('../data/tx90blocks.xml')
r = world.robot(0)
sim = Simulator(world)

def testIK():
    # Test builtin IK solver
    #On Thinkpad T410, runs at about 1.4ms
    s = ik.solver(ik.objective(r.getLink(7),local=(0.0,0.0,0.0),world=(0.6,0.0,0.6)))
    t1 = time.time()
    for iters in xrange(100):
        s.sampleInitial()
        (res,n)=s.solve(1000)
        if res:
            print "IK solution:",r.getConfig(),"found in",n,"iterations, residual",s.getResidual()
        else:
            print "IK failed:",r.getConfig(),"found in",n,"iterations, residual",s.getResidual()
    t2 = time.time()
    print "IK time:",t2-t1

    #testing the rootfind implementation of ik
    #On Thinkpad T410, runs at about 30ms
    class IKVectorField:
        def __init__(self,solver):
            self.solver = solver
    
        def eval(self, x):
            self.solver.robot.setConfig(x)
            return self.solver.getResidual()
        
            def jacobian(self, x):
                self.solver.robot.setConfig(x)
                return self.solver.getJacobian()
            
            def num_vars(self):
                return self.solver.robot.numLinks()

            def num_fns(self):
                return len(self.solver.getResidual())

    t1 = time.time()
    ikfunc = IKVectorField(s)
    rootfind.setFTolerance(1e-3)
    rootfind.setVectorField(ikfunc);
    boundList = zip(*r.getJointLimits())
    s.setActiveDofs(range(r.numLinks()))
    for iters in xrange(100):
        s.sampleInitial()
        tup = rootfind.findRootsBounded(r.getConfig(),boundList,1000)
        (res,x,n) = tup
        if res==0 or res==1:
            print "IK solution:",x,"found in",n,"iterations, residual",ikfunc.eval(x)
        else:
            print "IK failed:",x,"found in",n,"iterations, residual",ikfunc.eval(x)

    rootfind.destroy()
    t2 = time.time()
    print "Rootfind time:",t2-t1    


# Test basic simulator
sim.enableContactFeedbackAll()
print contact.simContactMap(sim)

t1 = time.time()
sim.simulate(5.0)
t2 = time.time()
print "Time for 1 s of simulation:",t2-t1
print world.robot(0).getConfig()
print world.robot(0).getJointLimits()

#print contact state
for aindex in xrange(world.numRigidObjects()):
    for bindex in xrange(world.numTerrains()):
        a = world.rigidObject(aindex)
        b = world.terrain(bindex)
        c = sim.getContacts(a.getID(),b.getID())
        if len(c) > 0:
            print len(c),"contacts between",a.getName(),"and",b.getName()

for ((a,b),clist) in contact.simContactMap(sim).iteritems():
    print a.getName(),"--",b.getName(),":"
    for c in clist:
        print "   ",c.x,c.n,c.kFriction
