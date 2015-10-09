import cspace
import robotsim
import robotcollide
from cspaceutils import AdaptiveCSpace

class RobotCSpace(AdaptiveCSpace):
    """A basic robot cspace that allows collision free motion.

    Warning: if your robot has non-standard joints, like a free-
    floating base or continuously rotating (spin) joints, you will need to
    overload the sample() method."""
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
        self.properties['geodesic'] = 1
        volume = 1
        for b in self.bound:
            if b[0] != b[1]: volume *= b[1]-b[0]
        self.properties['volume'] = volume

    def sample(self):
        """Overload this to implement custom sampling strategies or to handle
        non-standard joints"""
        return AdaptiveCSpace.sample(self)

    def feasible(self,x):
        """Feasibility test.  If self.adaptive=True, uses the adaptive
        feasibility tester which may speed up collision testing."""
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
        """Checks joint limits of the configuration x"""
        for (xi,bi) in zip(x,self.bound):
            if xi < bi[0] or xi > bi[1]:
                return False
        return True

    def selfCollision(self):
        """Checks whether the robot at its current configuration is in
        self collision"""
        #This should be faster than going through the collider... 
        return self.robot.selfCollides()
        #if not self.collider: return False
        #return any(self.collider.robotSelfCollisions(self.robot.index))

    def envCollision(self):
        """Checks whether the robot at its current configuration is in
        collision with the environment."""
        if not self.collider: return False
        for o in xrange(self.collider.world.numRigidObjects()):
            if any(self.collider.robotObjectCollisions(self.robot.index,o)):
                return True;
        for o in xrange(self.collider.world.numTerrains()):
            if any(self.collider.robotTerrainCollisions(self.robot.index,o)):
                return True;
        return False
                  

class ClosedLoopRobotCSpace(RobotCSpace):
    """A closed loop cspace.  Allows one or more IK constraints to be
    maintained during the robot's motion."""
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

    def sample(self):
        """Samples directly on the contact manifold"""
        self.solver.sampleInitial()
        (res,iters) = self.solver.solve(self.maxIters,self.tol)
        return self.robot.getConfig()

    def feasible(self,x):
        if self.adaptive:
            #Use the adaptive tester
            self.robot.setConfig(x)
            return AdaptiveCSpace.feasible(self,x)
            
        if not self.inJointLimits(x): return False
        self.robot.setConfig(x)
        if not self.closedLoop(): return False;
        if self.selfCollision(): return False
        if self.envCollision(): return False
        return True

    def closedLoop(self,tol=None):
        """Returns true if the closed loop constraint has been met at the
        robot's current configuration."""
        e = self.solver.getError()
        if tol==None: tol = self.tol
        return max(abs(ei) for ei in e) <= tol


