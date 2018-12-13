from cspace import CSpace
from .. import robotsim
from ..model import collide
from cspaceutils import EmbeddedCSpace
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
        - collider (optional): a collide.WorldCollider instance containing
          the world in which the robot lives.  Any ignored collisions will be
          respected in the collision checker.
        """
        CSpace.__init__(self)
        self.robot = robot
        self.setBounds(zip(*robot.getJointLimits()))
        self.collider = collider
        self.addFeasibilityTest((lambda x: self.inJointLimits(x)),"joint limits")

        def setconfig(x):
            self.robot.setConfig(x)
            return True
        if collider:
            bb0 = ([float('inf')]*3,[float('-inf')]*3)
            bb = [bb0[0],bb0[1]]
            def calcbb(x):
                bb[0] = bb0[0]
                bb[1] = bb0[1]
                for i in xrange(self.robot.numLinks()):
                    g = self.robot.link(i).geometry()
                    if not g.empty():
                        bbi = g.getBB()
                        bb[0] = [min(a,b) for (a,b) in zip(bb[0],bbi[0])]
                        bb[1] = [max(a,b) for (a,b) in zip(bb[1],bbi[1])]
                return True
            def objCollide(o):
                obb = self.collider.world.rigidObject(o).geometry().getBB()
                if not collide.bb_intersect(obb,bb): return False
                return any(True for _ in self.collider.robotObjectCollisions(self.robot.index,o))
            def terrCollide(o):
                obb = self.collider.world.terrain(o).geometry().getBB()
                if not collide.bb_intersect(obb,bb): return False
                return any(True for _ in self.collider.robotTerrainCollisions(self.robot.index,o))
            self.addFeasibilityTest(setconfig,"setconfig")
            self.addFeasibilityTest(calcbb,"calcbb",dependencies="setconfig")
            self.addFeasibilityTest((lambda x: not self.selfCollision()),"self collision",dependencies="setconfig")
            #self.addFeasibilityTest((lambda x: not self.envCollision()),"env collision")
            for o in range(self.collider.world.numRigidObjects()):
                self.addFeasibilityTest((lambda x,o=o: not objCollide(o)),"obj collision "+str(o)+" "+self.collider.world.rigidObject(o).getName(),dependencies="calcbb")
            for o in range(self.collider.world.numTerrains()):
                self.addFeasibilityTest((lambda x,o=o: not terrCollide(o)),"terrain collision "+str(o)+" "+self.collider.world.terrain(o).getName(),dependencies="calcbb")
        else:
            self.addFeasibilityTest(setconfig,"setconfig")
            self.addFeasibilityTest((lambda x: not self.selfCollision()),"self collision",dependencies="setconfig")

        self.properties['geodesic'] = 1

    def addConstraint(self,checker,name=None):
        self.addFeasibilityTest(checker,name)

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

    def selfCollision(self,x=None):
        """Checks whether the robot at its current configuration is in
        self collision"""
        #This should be faster than going through the collider... 
        if x is not None: self.robot.setConfig(x)
        return self.robot.selfCollides()
        #if not self.collider: return False
        #return any(self.collider.robotSelfCollisions(self.robot.index))

    def envCollision(self,x=None):
        """Checks whether the robot at its current configuration is in
        collision with the environment."""
        if not self.collider: return False
        if x is not None: self.robot.setConfig(x)
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
            #determine moving objects, which includes all links in the subset and descendants
            moving = [False]*robot.numLinks()
            for i in range(robot.numLinks()):
                if i in subset: moving[i] = True
                else:
                    p = robot.link(i).getParent()
                    if p >= 0 and moving[p]: moving[i]=True
            #disable self-collisions for non moving objects
            for i,mv in enumerate(moving):
                if not mv:
                    rindices = self.collider.robots[robot.index]
                    rindex = rindices[i]
                    if rindex < 0:
                        continue
                    newmask = set()
                    for j in range(robot.numLinks()):
                        if rindices[j] in self.collider.mask[rindex] and moving[j]:
                            newmask.add(rindices[j])
                    self.collider.mask[rindex] = newmask

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
    maintained during the robot's motion.

    Attributes:
    - solver: the IKSolver that is used.
    - maxIters: the maximum number of iterations for numerical IK solver
    - tol: how closely the IK constraint must be met, in meters/radians

    To satisfy the IK constraint, the motion planner ensures that configuration
    samples are projected to the manifold of closed-loop IK solutions.  To create
    edges between samples a and b, the straight line path a and b is projected to
    the manifold via an IK solve.
    """
    def __init__(self,robot,iks,collider=None):
        RobotCSpace.__init__(self,robot,collider)
        self.solver = robotsim.IKSolver(robot)
        if hasattr(iks,'__iter__'):
            for ik in iks:
                self.solver.add(ik)
        else:
            self.solver.add(iks)

        #root finding iterations
        self.maxIters = 100
        self.tol = 1e-3
        self.addFeasibilityTest((lambda x: self.closedLoop(x)),'closed loop constraint')

    def setIKActiveDofs(self,activeSet):
        """Marks that only a subset of the DOFs of the robot are to be used for solving
        the IK constraint."""
        self.solver.setActiveDofs(activeSet)

    def sample(self):
        """Samples directly on the contact manifold.  The basic method samples arbitrarily in
        the configuration space and then solves IK constraints.  This may be an ineffective
        method especially for floating-base robots, since the floating joints may be sampled
        arbitrarily."""
        x = RobotCSpace.sample(self)
        return self.solveConstraints(x)

    def sampleneighborhood(self,c,r):
        """Samples a neighborhood in ambient space and then projects onto the contact manifold"""
        x = RobotCSpace.sampleneighborhood(self,c,r)
        return self.solveConstraints(x)

    def solveConstraints(self,x):
        """Given an initial configuration of the robot x, attempts to solve the IK constraints 
        given in this space.  Return value is the best configuration found via local optimization."""
        self.robot.setConfig(x)
        self.solver.setMaxIters(self.maxIters)
        self.solver.setTolerance(self.tol)
        res = self.solver.solve()
        return self.robot.getConfig()

    def closedLoop(self,config=None,tol=None):
        """Returns true if the closed loop constraint has been met at config,
        or if config==None, the robot's current configuration."""
        if config is not None: self.robot.setConfig(config)
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

    def sendPathToController(self,path,controller,epsilon=1e-2):
        """Given a CSpace path path, sends the path to be executed to the SimRobotController.
        This discretizes the path and sends it as a piecewise linear curve, limited in speed
        by the robot's maximum velocity.

        NOTE: this isn't the best thing to do for robots with slow acceleration limits
        and/or high inertias because it ignores acceleration.  A better solution can be found
        in the MInTOS package or the C++ code in Klampt/Planning/RobotTimeScaling.h."""
        dpath = self.discretizePath(path,epsilon)
        vmax = controller.model().getVelocityLimits()
        assert len(dpath[0]) == len(vmax)
        controller.setMilestone(dpath[0])
        for a,b in zip(dpath[:-1],dpath[1:]):
            dt = 0.0
            for i in xrange(len(a)):
                if vmax[i] == 0:
                    if a[i] != b[i]: print "ClosedLoopRobotCSpace.sendPathToController(): Warning, path moves on DOF %d with maximum velocity 0"%(i,)
                else:
                    dt = max(dt,abs(a[i]-b[i])/vmax[i])
            #this does a piecewise lienar interpolation
            controller.appendLinear(dt,b)

class ImplicitManifoldRobotCSpace(RobotCSpace):
    """A closed loop cspace with an arbitrary numerical manifold f(q)=0
    to constrain the robot's motion.  The argument implicitConstraint
    should be a function f(q) returning a list of values that should be
    equal to 0 up to the given tolerance.  Essentially this is a
    ClosedLoopRobotCSpace except with a user-provided function.

    See ClosedLoopRobotCSpace.
    """
    def __init__(self,robot,implicitConstraint,collider=None):
        RobotCSpace.__init__self(robot,collider)
        self.implicitConstraint = implicitConstraint

        #root finding iterations
        self.maxIters = 100
        self.tol = 1e-3

        self.addFeasibilityTest((lambda x: self.onManifold(x)),'implicit manifold constraint')

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
