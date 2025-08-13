from .cspace import CSpace
from .. import robotsim
from ..model import collide
from .cspaceutils import EmbeddedCSpace
from ..robotsim import RobotModel,RobotModelLink,RigidObjectModel,IKObjective
from ..math import se3
import math
import random
from ..model.typing import Config,Configs,RigidTransform
from typing import Optional,Union,List,Callable

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
    
    .. warning::

        This implementation does NOT respect driver definitions.  Use
        :func:`~klampt.plan.robotplanning.make_space` instead.
        
    """
    def __init__(self,robot : RobotModel, collider : Optional[collide.WorldCollider]=None):
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

    def addConstraint(self, checker : Callable, name=None):
        self.addFeasibilityTest(checker,name)

    def sample(self) -> Config:
        """Overload this to implement custom sampling strategies or to handle
        non-standard joints.  This one will handle spin joints and
        rotational axes of floating bases."""
        res = CSpace.sample(self)
        for i,x in enumerate(res):
            if math.isnan(x):
                res[i] = random.uniform(0,math.pi*2.0)
        return res

    def inJointLimits(self,x : Config) -> bool:
        """Checks joint limits of the configuration x"""
        for i,(xi,bi) in enumerate(zip(x,self.bound)):
            if xi < bi[0] or xi > bi[1]:
                self.joint_limit_failures[i] += 1
                return False
        return True

    def selfCollision(self, x : Optional[Config]=None) -> bool:
        """Checks whether the robot at its current configuration is in
        self collision"""
        #This should be faster than going through the collider... 
        if x is not None: self.robot.setConfig(x)
        return self.robot.selfCollides()
        #if not self.collider: return False
        #return any(self.collider.robotSelfCollisions(self.robot.index))

    def envCollision(self,x : Optional[Config]=None) -> bool:
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

    def executablePath(self,path : Configs) -> Configs:
        """Given a planned CSpace path, returns a path that can be executed using
        :func:`klampt.model.trajectory.execute_path`.
        """
        return path


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
    def __init__(self,robot: RobotModel, iks : List[IKObjective], collider : Optional[collide.WorldCollider]=None):
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

    def setIKActiveDofs(self,activeSet : List[int]):
        """Marks that only a subset of the DOFs of the robot are to be used for
        solving the IK constraint.

        Args:
            activeSet (list of int): the robot DOF indices that should be active.
        """
        self.solver.setActiveDofs(activeSet)

    def sample(self) -> Config:
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

    def sampleneighborhood(self,c : Config, r : float) -> Config:
        """Samples a neighborhood in ambient space and then projects onto the
        contact manifold.
        """
        x = RobotCSpace.sampleneighborhood(self,c,r)
        return self.solveConstraints(x)

    def solveConstraints(self,x : Config) -> Config:
        """Given an initial configuration of the robot x, attempts to solve the
        IK constraints given in this space.  Return value is the best
        configuration found via local optimization.
        """
        self.robot.setConfig(x)
        self.solver.setMaxIters(self.maxIters)
        self.solver.setTolerance(self.tol)
        res = self.solver.solve()
        return self.robot.getConfig()

    def closedLoop(self,config : Optional[Config] = None, tol : Optional[float]=None) -> bool:
        """Returns true if the closed loop constraint has been met at config,
        or if config==None, the robot's current configuration."""
        if config is not None: self.robot.setConfig(config)
        e = self.solver.getResidual()
        if tol is None: tol = self.tol
        return max(abs(ei) for ei in e) <= tol

    def interpolate(self,a,b,u) -> Config:
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

    def discretizePath(self, path : Configs, epsilon : float = 1e-2) -> Configs:
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

    def executablePath(self,path : Configs, epsilon : float = 1e-2) -> Configs:
        """Given a :class:`CSpace` path, returns a path that can be executed
        using :func:`klampt.model.trajectory.execute_path`.  This is the same 
        as discretizePath, but is provided for compatibility with the
        RobotCSpace interface.

        .. note::

            This isn't the best thing to do for robots with slow acceleration
            limits and/or high inertias because it ignores acceleration.  A
            better solution can be found in the MInTOS package or the C++ code
            in Klampt/Cpp/Planning/RobotTimeScaling.h.

        Returns:
            list of Configs: A closely spaced sequence of configurations
            discretized at the given resolution.
        """
        return self.discretizePath(path,epsilon)


class ImplicitManifoldRobotCSpace(RobotCSpace):
    """A closed loop cspace with an arbitrary numerical manifold f(q)=0
    to constrain the robot's motion.  The argument implicitConstraint
    should be a function f(q) returning a list of values that should be
    equal to 0 up to the given tolerance.  Essentially this is a
    ClosedLoopRobotCSpace except with a user-provided function.

    See :class:`ClosedLoopRobotCSpace`.
    """
    def __init__(self,robot : RobotModel, implicitConstraint : Callable, collider=None):
        RobotCSpace.__init__(self,robot,collider)
        self.implicitConstraint = implicitConstraint

        #root finding iterations
        self.maxIters = 100
        self.tol = 1e-3

        self.addFeasibilityTest((lambda x: self.onManifold(x)),'implicit manifold constraint')

    def sample(self) -> Config:
        """Samples directly on the contact manifold"""
        x = RobotCSpace.sample()
        return self.solveManifold(x)

    def onManifold(self,x : Config, tol : Optional[float] = None) -> bool:
        """Returns true if the manifold constraint has been met at x."""
        e = self.implicitConstraint.eval(x)
        if tol is None: tol = self.tol
        return max(abs(ei) for ei in e) <= tol

    def solveManifold(self,x : Config, tol : Optional[float]=None, maxIters : Optional[int]=None):
        """Solves the manifold constraint starting from x, to the given
        tolerance and with the given maximum iteration count.  Default
        uses the values set as attributes of this class.
        """
        if tol is None: tol = self.tol
        if maxIters is None: maxIters = self.maxIters
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
    def __init__(self,ambientspace : RobotCSpace, subset : List[int], xinit : Optional[Config] = None):
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
        """Modifies the collider in ambientspace to only check self-collisions
        between moving link pairs, and one moving link vs a static link. 
        Should be called before `setup()` in most cases.
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

    def discretizePath(self,path : Configs, epsilon=1e-2):
        """Only useful for ClosedLoopRobotCSpace"""
        if hasattr(self.ambientspace,'discretizePath'):
            return self.ambientspace.discretizePath(self.liftPath(path),epsilon)
        else:
            return self.liftPath(path)

    def executablePath(self,path : Configs) -> Configs:
        """Sends a planned path so that it is executed correctly by the
        controller (assumes a fully actuated robot).

        Args:
            path (list of Configs): a path in the embedded space or the
                ambient space, as returned by a planner.
            controller (SimRobotController): the robot's controller
        """
        if len(path[0]) == len(self.mapping):
            path = self.liftPath(path)
        if hasattr(self.ambientspace,'executablePath'):
            path = self.ambientspace.executablePath(path)
        return path



class RobotCSpaceWithObject(RobotCSpace):
    """A robot cspace that attaches one or more objects to one or more robot
    links.  This is used for grasped objects.

    By default, collisions between the attached object and its link are ignored.
    If the attached link does not have an enabled self collision with another link,
    then collisions between the object and that link are also ignored.  This is
    useful for finger links that are grasping the object, since they will usually
    be overlapping the object in the model but these should be ignored as far as
    feasibility goes.  

    If you would NOT like to ignore collisions between the object and one of these
    disabled links, you must add this as an extra feasibility test.
    """
    def __init__(self,robot : RobotModel, collider : Optional[collide.WorldCollider]=None):
        RobotCSpace.__init__(self,robot,collider)
        self.attachments = []  #list of (objectIndex, linkIndex, relTransform)
        setconfig_index = self.feasibilityTestNames.index("setconfig")
        assert setconfig_index >= 0, "setconfig feasibility test not found"
        self.feasibilityTests[setconfig_index] = self.updateConfig  #replace old setconfig with new one that sets object transforms

    def updateConfig(self,x):
        """Updates the robot's configuration and sets the transforms of
        attached objects to the correct relative transforms."""
        self.robot.setConfig(x)
        for obj,link,relTransform in self.attachments:
            obj.setTransform(*se3.mul(self.robot.link(link).getTransform(),relTransform))
        return True

    def attachObject(self,objectIndex : int, linkIndex : int, relTransform : Optional[RigidTransform]=None):
        """Attaches the object with index objectIndex to the link with index
        linkIndex.  The object will be moved with the robot, and collisions
        between the robot and the object will be checked.
        
        Note: collisions will be ignored between objectIndex and linkIndex.
        If you want to ignore collisions between other links, such as finger
        links, you must set the collider's collision mask appropriately.
        """
        if not self.collider:
            raise ValueError("Cannot attach objects without a collider")
        obj = self.collider.world.rigidObject(objectIndex)
        if obj.id < 0:
            raise ValueError("Object %d is not in the world"%(objectIndex,))
        link = self.robot.link(linkIndex)
        if relTransform is None:
            relTransform = se3.mul(se3.inv(link.getTransform()),obj.getTransform())
        for a in self.attachments:
            if a[0].id == obj.id:
                raise ValueError("Object %d is already attached to the robot"%(objectIndex,))
        self.attachments.append((obj, linkIndex, relTransform))
        try:
            obj_collision_index = self.feasibilityTestNames.index("obj collision "+str(objectIndex)+" "+obj.getName())
        except ValueError:
            print("Object collision feasibility test not found")
            print("Options:",self.feasibilityTestNames)
            raise
        self.feasibilityTests[obj_collision_index] = lambda x:not self.objSelfCollision(obj,link)

    def selfCollision(self, x : Optional[Config]=None) -> bool:
        """Checks whether the robot at its current configuration is in
        self collision.  Ignores the object attached to the robot."""
        #This should be faster than going through the collider... 
        if x is not None: self.updateConfig(x)
        if self.robot.selfCollides(): 
            return True
        return False
        
    def objSelfCollision(self,obj:RigidObjectModel, link:RobotModelLink):
        """Checks whether the object attached to the link collides with the
        rest of the robot."""
        for alt_link in range(self.robot.numLinks()):
            if alt_link == link.index: continue
            if not self.robot.selfCollisionEnabled(link.index,alt_link):
                #ignore collisions with links that are not enabled for self-collision
                continue
            if self.collider is not None and not self.collider.isCollisionEnabled((self.robot.link(alt_link),obj)):
                continue
            if obj.geometry().collides(self.robot.link(alt_link).geometry()):
                return True
        return False

    def envCollision(self,x : Optional[Config]=None) -> bool:
        """Checks whether the robot and attached objects at its current configuration is in
        collision with the environment."""
        if not self.collider: return False
        if x is not None: self.updateConfig(x)
        attached_objects = set(obj.index for obj,_,_ in self.attachments)
        for o in range(self.collider.world.numRigidObjects()):
            if obj.index in attached_objects:
                #ignore collisions with attached objects
                continue
            if any(self.collider.robotObjectCollisions(self.robot.index,o)):
                return True
        for o in range(self.collider.world.numTerrains()):
            if any(self.collider.robotTerrainCollisions(self.robot.index,o)):
                return True
        #collide attached objects with the rest of the world
        for obj,_,_ in self.attachments:
            for o in range(self.collider.world.numRigidObjects()):
                if obj.index in attached_objects: continue
                if any(self.collider.objectObjectCollisions(obj.index,o)):
                    return True
            for o in range(self.collider.world.numTerrains()):
                if any(self.collider.objectTerrainCollisions(obj.index,o)):
                    return True
        return False
    
