from cspace import CSpace
import robotsim
import robotcollide
from cspaceutils import AdaptiveCSpace,EmbeddedCSpace
import math
import random

class RobotCSpace(CSpace):
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
        CSpace.__init__(self)
        self.robot = robot
        self.bound = zip(*robot.getJointLimits())
        self.collider = collider
        self.addFeasibilityTest(lambda x: self.inJointLimits(x),"joint limits")

        #TODO explode these into individual self collision / env collision
        #tests
        def setconfig(x):
            self.robot.setConfig(x)
            return True
        if collider:
            self.addFeasibilityTest(setconfig,"dummy")
            self.addFeasibilityTest(lambda x : not self.selfCollision(),"self collision")
            self.addFeasibilityTest(lambda x: not self.envCollision(),"env collision")
        self.properties['geodesic'] = 1
        volume = 1
        for b in self.bound:
            if b[0] != b[1]: volume *= b[1]-b[0]
        self.properties['volume'] = volume

    def addConstraint(self,checker,name=None):
        self.addFeasiblilityTest(checker,name)

    def sample(self):
        """Overload this to implement custom sampling strategies or to handle
        non-standard joints.  This one will handle spin joints and
        rotational axes of floating bases."""
        res = CSpace.sample(self)
        for i,x in enumerate(res):
            if math.isnan(x):
                res[i] = random.uniform(0,math.pi*2.0)
        return res

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

    def interpolate(self,a,b,u):
        return self.robot.interpolate(a,b,u)

    def distance(self,a,b):
        return self.robot.distance(a,b)

    def sendPathToController(self,path,controller):
        """Given a planned CSpace path 'path' and a SimRobotController 'controller',
        sends the path so that it is executed correctly by the controller (this assumes
        a fully actuated robot)."""
        controller.setMilestone(path[0])
        for q in path[1:]:
            controller.appendMilestoneLinear(q)

                  
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

    def liftPath(self,path):
        """Given a CSpace path path, lifts this to the full robot configuration"""
        return [self.lift(q) for q in path]

    def sendPathToController(self,path,controller):
        """Given a planned CSpace path 'path' and a SimRobotController 'controller',
        sends the path so that it is executed correctly by the controller (this assumes
        a fully actuated robot)."""
        lpath = self.liftPath(path)
        controller.setMilestone(lpath[0])
        for q in lpath[1:]:
            controller.appendMilestoneLinear(q)

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

        self.addFeasibilityTest(lambda x: self.closedLoop(),'closed loop constraint')

    def setIKActiveDofs(self,activeSet):
        """Marks that only a subset of the DOFs of the robot are to be used for solving
        the IK constraint."""
        self.solver.setActiveDofs(activeSet)

    def sample(self):
        """Samples directly on the contact manifold"""
        self.solver.sampleInitial()
        (res,iters) = self.solver.solve(self.maxIters,self.tol)
        return self.robot.getConfig()

    def solveConstraints(self,x):
        """Given an initial configuration of the robot x, attempts to solve the IK constraints 
        given in this space.  Return value is the best configuration found via local optimization."""
        self.robot.setConfig(x)
        (res,iters) = self.solver.solve(self.maxIters,self.tol)
        if not res: print "IK failed solve"
        return self.robot.getConfig()

    def closedLoop(self,tol=None):
        """Returns true if the closed loop constraint has been met at the
        robot's current configuration."""
        e = self.solver.getResidual()
        if tol==None: tol = self.tol
        return max(abs(ei) for ei in e) <= tol

    def interpolate(self,a,b,u):
        """Interpolates on the manifold.  Used by edge collision checking"""
        x = RobotCSpace.interpolate(self,a,b,u)
        return self.solveConstraints(x)

    def interpolationPath(self,a,b,epsilon=1e-2):
        """Creates a discretized path on the contact manifold between the points a and b, with
        resolution epsilon"""
        d = self.distance(a,b)
        nsegs = int(math.ceil(d/epsilon))
        if nsegs <= 1: return [a,b]
        res = [a]
        for i in xrange(nsegs-1):
            u = float(i+1)/float(nsegs)
            res.append(self.interpolate(a,b,u))
        res.append(b)
        return res

    def discretizePath(self,path,epsilon=1e-2):
        """Given a CSpace path path, generates a path that satisfies closed-loop constraints
        up to the given distance between milestones"""
        if path is None: return None
        if len(path)==0: return []
        respath = [path[0]]
        for a,b in zip(path[:-1],path[1:]):
            respath += self.interpolationPath(a,b,epsilon)[1:]
        return respath

class ImplicitManifoldRobotCSpace(RobotCSpace):
    """A closed loop cspace with an arbitrary numerical manifold f(q)=0
    to constrain the robot's motion."""
    def __init__(self,robot,implicitConstraint,collider=None):
        RobotCSpace.__init__self(robot,collider)
        self.implicitConstraint = implicitConstraint

        #root finding iterations
        self.maxIters = 100
        self.tol = 1e-3

        self.addFeasibilityTest(lambda x: self.onManifold(x),'implicit manifold constraint')

    def sample(self):
        """Samples directly on the contact manifold"""
        x = RobotCSpace.sample()
        return self.solveManifold(x)

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

    def interpolate(self,a,b,u):
        """Interpolates on the manifold.  Used by edge collision checking"""
        x = RobotCSpace.interpolate(self,a,b,u)
        return self.solveManifold(x)
