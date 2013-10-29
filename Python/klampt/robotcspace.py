import cspace
import robotsim
import robotcollide
from cspaceutils import AdaptiveCSpace

class RobotCSpace(AdaptiveCSpace):
    """A basic robot cspace that allows collision free motion"""
    def __init__(self,robot,collider=None):
        AdaptiveCSpace.__init__(self)
        self.robot = robot
        self.bound = zip(*robot.getJointLimits())
        self.collider = collider

        #set this to false to turn off the adaptive tester, which may
        #have some overhead
        self.adaptive = True

        #adaptive tests
        self.addFeasibleTest(lambda(x): self.inJointLimits(x),"joint limits")
        #TODO explode these into individual self collision / env collision
        #tests
        self.addFeasibleTest(lambda(x): not self.selfCollision(),"self collision")
        self.addFeasibleTest(lambda(x): not self.envCollision(),"env collision")


    def feasible(self,x):
        if self.adaptive:
            #Use the adaptive tester
            self.robot.setConfig(x)
            return AdaptiveCSpace.feasible(self,x)

        #Use the regular tester
        if not self.inJointLimits(x): return False
        #check collisions
        if self.collider:
            self.robot.setConfig(x)
            if self.selfCollision(): return False
            if self.envCollision(): return False
        return True

    def inJointLimits(self,x):
        #check joint limits
        for (xi,bi) in zip(x,self.bound):
            if xi < bi[0] or xi > bi[1]:
                return False
        return True

    def selfCollision(self):
        if not self.collider: return False
        return any(self.collider.robotSelfCollisions(self.robot.index))

    def envCollision(self):
        if not self.collider: return False
        for o in xrange(self.collider.world.numRigidObjects()):
            if any(self.collider.robotObjectCollisions(self.robot.index,o)):
                return True;
        for o in xrange(self.collider.world.numTerrains()):
            if any(self.collider.robotTerrainCollisions(self.robot.index,o)):
                return True;
        return False
                  

class ClosedLoopRobotCSpace(RobotCSpace):
    """A closed loop cspace"""
    def __init__(self,robot,iks,collider=None):
        RobotCSpace.__init__self(robot,collider)
        self.solver = robotsim.IKSolver(robot)
        if hasattr(iks,'__iter__'):
            for ik in iks:
                self.solver.add(ik)
        else:
            self.solver.add(ik)

        #IK solve iterations
        self.maxIters = 100
        self.tol = 1e-3

        #adaptive checker
        self.addFeasibleTest(lambda(x): self.closedLoop())

    def sample():
        self.solver.sampleInitial()
        (res,iters) = self.solver.solve(self.maxIters,self.tol)

    def feasible(self,x):
        if self.adaptive:
            #Use the adaptive tester
            robot.setConfig(x)
            return AdaptiveCSpace.feasible(self,x)
            
        if not self.inJointLimits(x): return False
        robot.setConfig(x)
        if not self.closedLoop(): return False;
        if self.selfCollision(): return False
        if self.envCollision(): return False
        return True

    def closedLoop(self,tol=None):
        e = self.solver.getError()
        if tol==None: tol = self.tol
        return max(abs(ei) for ei in e) <= tol


