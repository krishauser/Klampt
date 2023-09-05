from .cspace import CSpace
from .. import robotsim
from ..model import collide
from .cspaceutils import EmbeddedCSpace
import math
import random

class RobotCSpace(CSpace):
    """A basic robot cspace that allows collision free motion.

    Args:
        robot (RobotModel): the robot that's moving.
        collider (:class:`WorldCollider`, optional): a collide.WorldCollider
            instance instantiated with the world in which the robot lives. 
            Any ignored collisions in the collider will be respected in the
            feasibility tests of this CSpace.

            If this is not provided, then only self-collisions will be checked.

    .. warning::

        If your robot has non-standard joints, like a free-
        floating base or continuously rotating (spin) joints, you may need to
        overload the :meth:`sample` method.  The default implementation
        assumes that everything with unbounded limits is a rotational joint.
        
    """
    def __init__(self,robot,collider=None):
        CSpace.__init__(self)
        self.robot = robot
        self.setBounds(list(zip(*robot.getJointLimits())))
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
                for i in range(self.robot.numLinks()):
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

        self.joint_limit_failures = [0]*len(self.bound)
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
        for i,(xi,bi) in enumerate(zip(x,self.bound)):
            if xi < bi[0] or xi > bi[1]:
                self.joint_limit_failures[i] += 1
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
        for o in range(self.collider.world.numRigidObjects()):
            if any(self.collider.robotObjectCollisions(self.robot.index,o)):
                return True
        for o in range(self.collider.world.numTerrains()):
            if any(self.collider.robotTerrainCollisions(self.robot.index,o)):
                return True
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


class ClosedLoopRobotCSpace(RobotCSpace):
    """A closed loop cspace.  Allows one or more IK constraints to be
    maintained during the robot's motion.

    Attributes:
        solver (IKSolver): the solver containing all IK constraints
        maxIters (int): the maximum number of iterations for numerical IK
            solver
        tol (float): how closely the IK constraint must be met, in meters and/
            or radians

    To satisfy the IK constraint, the motion planner ensures that configuration
    samples are projected to the manifold of closed-loop IK solutions.  To
    create edges between samples a and b, the straight line path a and b is
    projected to the manifold via an IK solve.
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
        """Marks that only a subset of the DOFs of the robot are to be used for
        solving the IK constraint.

        Args:
            activeSet (list of int): the robot DOF indices that should be active.
        """
        self.solver.setActiveDofs(activeSet)

    def sample(self):
        """Samples directly on the contact manifold.  The basic method samples
        arbitrarily in the configuration space and then solves IK constraints. 

        Note that this may be an ineffective method especially for floating-base
        robots, since the floating joints may be sampled arbitrarily.  To maximize
        performance, better problem-speciifc sampling distributions should be
        implemented by a subclass, if possible.
        """
        x = RobotCSpace.sample(self)
        res = self.solveConstraints(x)
        return res

    def sampleneighborhood(self,c,r):
        """Samples a neighborhood in ambient space and then projects onto the
        contact manifold.
        """
        x = RobotCSpace.sampleneighborhood(self,c,r)
        return self.solveConstraints(x)

    def solveConstraints(self,x):
        """Given an initial configuration of the robot x, attempts to solve the
        IK constraints given in this space.  Return value is the best
        configuration found via local optimization.
        """
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
        """Creates a discretized path on the contact manifold between the
        points a and b, with resolution epsilon.
        """
        d = self.distance(a,b)
        nsegs = int(math.ceil(d/epsilon))
        if nsegs <= 1: return [a,b]
        res = [a]
        for i in range(nsegs-1):
            u = float(i+1)/float(nsegs)
            res.append(self.interpolate(a,b,u))
        res.append(b)
        return res

    def discretizePath(self,path,epsilon=1e-2):
        """Given a :class:`CSpace` path ``path``, generates a path that
        satisfies closed-loop constraints up to the given distance between
        milestones.
        """
        if path is None: return None
        if len(path)==0: return []
        respath = [path[0]]
        for a,b in zip(path[:-1],path[1:]):
            respath += self.interpolationPath(a,b,epsilon)[1:]
        return respath

    def sendPathToController(self,path,controller,epsilon=1e-2):
        """Given a :class:`CSpace` path ``path``, sends the path to be executed
        to the :class:`SimRobotController` ``controller``.

        This discretizes the path and sends it as a piecewise linear curve,
        limited in speed by the robot's maximum velocity.

        .. note::
            This isn't the best thing to do for robots with slow acceleration
            limits and/or high inertias because it ignores acceleration.  A
            better solution can be found in the MInTOS package or the C++ code
            in Klampt/Cpp/Planning/RobotTimeScaling.h.
        """
        dpath = self.discretizePath(path,epsilon)
        vmax = controller.model().getVelocityLimits()
        assert len(dpath[0]) == len(vmax)
        controller.setMilestone(dpath[0])
        for a,b in zip(dpath[:-1],dpath[1:]):
            dt = 0.0
            for i in range(len(a)):
                if vmax[i] == 0:
                    if a[i] != b[i]: print("ClosedLoopRobotCSpace.sendPathToController(): Warning, path moves on DOF %d with maximum velocity 0"%(i,))
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

    See :class:`ClosedLoopRobotCSpace`.
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
        from klampt import rootfind
        rootfind.setXTolerance(1e-8)
        rootfind.setFTolerance(tol)
        rootfind.setVectorField(self.implicitConstraint)
        (res,x,val) = rootfind.findRootsBounded(x,self.bound)
        return x

    def interpolate(self,a,b,u):
        """Interpolates on the manifold.  Used by edge collision checking"""
        x = RobotCSpace.interpolate(self,a,b,u)
        return self.solveManifold(x)


class EmbeddedRobotCSpace(EmbeddedCSpace):
    """A basic robot cspace that allows collision free motion of a *subset*
    of joints.  The subset is given by the indices in the list "subset"
    provided to the constructor.  The configuration space is R^k where k
    is the number of DOFs in the subset.

    Args:
        ambientspace (RobotCSpace): a RobotCSpace, ClosedLoopRobotCSpace, etc.
        subset (list of ints): the indices of moving DOFs
        xinit (configuration, optional): the reference configuration, or None
            to use the robot's current configuration as the reference.
    """
    def __init__(self,ambientspace,subset,xinit=None):
        self.robot = ambientspace.robot
        if xinit is None:
            xinit = self.robot.getConfig()
        EmbeddedCSpace.__init__(self,ambientspace,subset,xinit)
        #do monkey-patching needed to make the sampler work properly for closed-loop spaces
        if isinstance(ambientspace,ImplicitManifoldRobotCSpace):
            from klampt import rootfind
            def subsetImplicitConstraint(x):
                return self.ambientSpace.implicitConstraint(self.lift(x))
            def solveManifold(x):
                rootfind.setXTolerance(1e-8)
                rootfind.setFTolerance(self.ambientspace.tol)
                rootfind.setVectorField(subsetImplicitConstraint)
                (res,x,val) = rootfind.findRootsBounded(x,self.bound)
                return x
            def sample():
                return solveManifold(self.lift(CSpace.sample(self)))
            def sampleneighborhood(c,r):
                return solveManifold(self.lift(CSpace.sampleneighborhood(self,c,r)))
            self.sample = sample
            self.sampleneighborhood = sampleneighborhood
        if isinstance(ambientspace,ClosedLoopRobotCSpace):
            #sanity check
            activedofs = ambientspace.solver.getActiveDofs()
            if len(activedofs) > len(subset):
                raise ValueError("ClosedLoopRobotCSpace IK solver must be configured with moving dofs that are within the subset of embedded dofs")
            elif activedofs != subset:
                ssubset = set(subset)
                for i in activedofs:
                    if i not in ssubset:
                        raise ValueError("ClosedLoopRobotCSpace IK solver must be configured with moving dofs that are within the subset of embedded dofs")
            def sample():
                xseed = self.lift(CSpace.sample(self))
                return self.project(self.ambientspace.solveConstraints(xseed))
            def sampleneighborhood(c,r):
                xseed = self.lift(CSpace.sampleneighborhood(self,c,r))
                return self.project(self.ambientspace.solveConstraints(xseed))
            self.sample = sample
            self.sampleneighborhood = sampleneighborhood

    def disableInactiveCollisions(self):
        """This modifies the collider in ambientspace to only check collisions
        between moving pairs.  Should be called before `setup()` in most cases.
        """
        robot = self.robot
        collider = self.ambientspace.collider
        subset = self.mapping

        active = [False]*robot.numLinks()
        for i in subset:
            active[i] = True
        for i in range(robot.numLinks()):
            if active[robot.link(i).getParent()]:
                active[i] = True
        inactive = []
        for i in range(robot.numLinks()):
            if not active[i]:
                inactive.append(i)
        #disable self-collisions for inactive objects
        for i in inactive:
            rindices = collider.robots[robot.index]
            rindex = rindices[i]
            if rindex < 0:
                continue
            newmask = set()
            for j in range(robot.numLinks()):
                if rindices[j] in collider.mask[rindex] and active[j]:
                    newmask.add(rindices[j])
            collider.mask[rindex] = newmask

    def discretizePath(self,path,epsilon=1e-2):
        """Only useful for ClosedLoopRobotCSpace"""
        if hasattr(self.ambientspace,'discretizePath'):
            return self.ambientspace.discretizePath(self.liftPath(path),epsilon)
        else:
            return self.liftPath(path)

    def sendPathToController(self,path,controller):
        """Sends a planned path so that it is executed correctly by the
        controller (assumes a fully actuated robot).

        Args:
            path (list of Configs): a path in the embedded space or the
                ambient space, as returned by a planner.
            controller (SimRobotController): the robot's controller
        """
        if len(path[0]) == len(self.mapping):
            path = self.liftPath(path)
        if hasattr(self.ambientspace,'discretizePath'):
            path = self.ambientspace.discretizePath(path)
        self.ambientspace.sendPathToController(path,controller)



class RobotSubsetCSpace(EmbeddedCSpace):
    """A basic robot cspace that allows collision free motion of a *subset*
    of joints.  The subset is given by the indices in the list "subset"
    provided to the constructor.  The configuration space is R^k where k
    is the number of DOFs in the subset.

    This class will automatically disable all collisions for inactive robot 
    links in the collider.

    .. note::
        To convert from start/goal robot configurations to the CSpace, call
        the `project(qrobot)` method for the start and goal.
        (see :meth:`EmbeddedCSpace.project`)

    .. note::
        To convert from a planned path back to the robot's full configuration
        space, you will need to call the `lift(q)` method for all
        configurations q in the planned path. (see :meth:`EmbeddedCSpace.lift`) 

    .. warning::
        If your robot has non-standard joints, like a free-floating base or
        continuously rotating (spin) joints, you will need to overload the
        :meth:`sample` method.

    .. deprecated:: 0.8.6

        Deprecated this in favor of EmbeddedRobotCSpace, which is adaptable
        to ClosedLoopRobotCSpace and ImplicitManifoldRobotCSpace.  To convert
        code to EmbeddedRobotCSpace, convert

        ``space = RobotSubsetCSpace(robot,subset,collider)`` to

        ``space = EmbeddedCSpace(RobotCSpace(robot,collider),subset);``
        ``space.disableInactiveCollisions()``

    """
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
        """Given a planned :class:`CSpace` path ``path`` and a
        :class:`SimRobotController` ``controller``, sends the path so that it
        is executed correctly by the controller.

        .. note:
            This assumes a fully actuated robot.  It won't work for robots with
            free-floating bases.

        """
        lpath = self.liftPath(path)
        controller.setMilestone(lpath[0])
        for q in lpath[1:]:
            controller.appendMilestoneLinear(q)
