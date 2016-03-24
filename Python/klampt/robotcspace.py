import cspace
import robotsim
import robotcollide
from cspaceutils import AdaptiveCSpace,EmbeddedCSpace
import math
import random

class RobotCSpace(AdaptiveCSpace):
    """A basic robot cspace that allows collision free motion.

    Warning: if your robot has non-standard joints, like a free-
    floating base or continuously rotating (spin) joints, you will need to
    overload the sample() method."""
    def __init__(self,robot,collider=None):
        """Arguments:
        - robot: the robot which should move.
        - collider: optional: a robotcollide.WorldCollider instance containing
          the world in which the robot lives.  Any ignored collisions will be
          respected in the collision checker.
        """
        AdaptiveCSpace.__init__(self)
        self.robot = robot
        self.bound = zip(*robot.getJointLimits())
        self.collider = collider

        #set this to false to turn off the adaptive tester, which may
        #have some overhead
        self.adaptive = True

        self.extraFeasibilityTests = []

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

    def addConstraint(self,checker):
        self.extraFeasibilityTests.append(checker)
        self.addFeasibleTest(checker)

    def sample(self):
        """Overload this to implement custom sampling strategies or to handle
        non-standard joints.  This one will handle spin joints and
        rotational axes of floating bases."""
        res = AdaptiveCSpace.sample(self)
        for i,x in enumerate(res):
            if math.isnan(x):
                res[i] = random.uniform(0,math.pi*2.0)
        return res

    def feasible(self,x):
        """Feasibility test.  If self.adaptive=True, uses the adaptive
        feasibility tester which may speed up collision testing."""
        if self.adaptive:
            #Use the adaptive tester
            self.robot.setConfig(x)
            return AdaptiveCSpace.feasible(self,x)

        #Use the regular tester
        if not self.inJointLimits(x): return False
        #check extras
        for f in self.extraFeasibilityTesters:
            if not f(x): return False
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

                  
class RobotSubsetCSpace(EmbeddedCSpace):
    """A basic robot cspace that allows collision free motion of a *subset*
    of joints.  The subset is given by the indices in the list "subset"
    provided to the constructor.  The configuration space is R^k where k
    is the number of DOFs in the subset.

    This class will automatically disable all collisions for inactive robot links
    in the collider.

    Note: to convert from start/goal robot configurations to the CSpace, call
    the project(qrobot) method for the start and goal. (see EmbeddedCSpace.project())

    Note: to convert from a planned path back to the robot's full configuration space,
    you will need to call the lift(q) method for all configurations q in the planned
    path. (see EmbeddedCSpace.lift()) 

    Warning: if your robot has non-standard joints, like a free-
    floating base or continuously rotating (spin) joints, you will need to
    overload the sample() method."""
    def __init__(self,robot,subset,collider=None):
        EmbeddedCSpace.__init__(self,RobotCSpace(robot,collider),subset,xinit=robot.getConfig())
        self.collider = collider
        if self.collider:
            inactive = []
            for i in range(robot.numLinks()):
                if i not in subset: inactive.append(i)
            #disable self-collisions for inactive objects
            for i in inactive:
                rindex = self.collider.robots[robot.index][i]
                self.collider.mask[rindex] = set()


class ClosedLoopRobotCSpace(RobotCSpace):
    """A closed loop cspace.  Allows one or more IK constraints to be
    maintained during the robot's motion."""
    def __init__(self,robot,iks,collider=None):
        RobotCSpace.__init__(self,robot,collider)
        self.solver = robotsim.IKSolver(robot)
        if hasattr(iks,'__iter__'):
            for ik in iks:
                self.solver.add(ik)
        else:
            self.solver.add(iks)

        #IK solve iterations
        self.maxIters = 100
        self.tol = 1e-3

        #adaptive checker
        self.addFeasibleTest(lambda x: self.closedLoop(),'closed loop constraint')

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
        #check extras
        for f in self.extraFeasibilityTesters:
            if not f(x): return False
        #check collisions
        if self.selfCollision(): return False
        if self.envCollision(): return False
        return True

    def closedLoop(self,tol=None):
        """Returns true if the closed loop constraint has been met at the
        robot's current configuration."""
        e = self.solver.getError()
        if tol==None: tol = self.tol
        return max(abs(ei) for ei in e) <= tol

class ImplicitManifoldRobotCSpace(RobotCSpace):
    """A closed loop cspace with an arbitrary numerical manifold f(q)=0
    to constrain the robot's motion."""
    def __init__(self,robot,implicitConstraint,collider=None):
        RobotCSpace.__init__self(robot,collider)
        self.implicitConstraint = implicitConstraint

        #root finding iterations
        self.maxIters = 100
        self.tol = 1e-3

        #adaptive checker
        self.addFeasibleTest(lambda x: self.onManifold(x),'implicit manifold constraint')

    def sample(self):
        """Samples directly on the contact manifold"""
        x = RobotCSpace.sample()
        return self.solveManifold(x)

    def feasible(self,x):
        if self.adaptive:
            #Use the adaptive tester
            self.robot.setConfig(x)
            return AdaptiveCSpace.feasible(self,x)
            
        if not self.inJointLimits(x): return False
        self.robot.setConfig(x)
        if not self.closedLoop(): return False;
        #check extras
        for f in self.extraFeasibilityTesters:
            if not f(x): return False
        #check collisions
        if self.selfCollision(): return False
        if self.envCollision(): return False
        return True

    def onManifold(self,x,tol=None):
        """Returns true if the manifold constraint has been met at x."""
        e = self.implicitConstraint.eval(x)
        if tol==None: tol = self.tol
        return max(abs(ei) for ei in e) <= tol

    def solveManifold(self,x,tol=None,maxIters=None):
        """Solves the manifold constraint starting from x, to the given
        tolerance and with the given maximum iteration count.  Default
        uses the values set as attributes of this class.
        """
        if tol==None: tol = self.tol
        if maxIters==None: maxIters = self.maxIters
        import rootfind
        rootfind.setXTolerance(1e-8)
        rootfind.setFTolerance(tol)
        rootfind.setVectorField(self.implicitConstraint)
        (res,x,val) = rootfind.findRootsBounded(x,self.bound)
        return x
