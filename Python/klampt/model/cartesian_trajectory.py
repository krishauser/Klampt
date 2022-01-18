"""Utilities for Cartesian IK solving, interpolation, and path adjustment.

To generate a path that moves to a Cartesian target, the
:func:`cartesian_move_to` function is the most convenient.

To interpolate between two Cartesian task space points,
:func:`cartesian_interpolate_linear` and :func:`cartesian_interpolate_bisect`
are approximately equivalent.

To interpolate along a Cartesian task space path, the
:func:`cartesian_path_interpolate` function implements two methods for solving
for a feasible robot path.  A pointwise approach moves greedily along the path,
while a roadmap approach gives a probabilistically complete solver for the
redundancy resolution. 

To move an existing joint-space path by a cartesian offset, the
:func:`cartesian_bump` is used. This is useful for adapting motion primitives
to new sensor data.

All the classes in this module use a unified representation of the workspace of
one or more :class:`IKObjective` constraints, which is retrieved by
``config.getConfig(constraints)``.  For example, the workspace of a position-
only constraint is its 3D world-space target coordinates.  The workspace of a 
fixed transform constraint is the 12D concatenation of the rotation matrix and
translation vector.  See the :mod:`~klampt.model.config` module for details.
"""

from .trajectory import *
from ..robotsim import IKObjective,IKSolver
from . import ik
from . import config
from collections import deque
import math
import warnings
from typing import Union,Optional,List,Sequence,Callable
from .typing import Vector,Vector3,Matrix3,RigidTransform

def set_cartesian_constraints(
        x: Vector,
        constraints: Sequence[IKObjective],
        solver: IKSolver
    ) -> None:
    """For ``x`` a workspace parameter setting (obtained via
    ``config.getConfig(constraints)``), a set of constraints, and a
    :class:`IKSolver` object, modifies the constraints and the solver so that
    the solver is setup to match the workspace parameter setting x.
    """
    config.setConfig(constraints,x)
    solver.clear()
    for c in constraints:
        solver.add(c)

def solve_cartesian(
        x: Vector,
        constraints: Sequence[IKObjective],
        solver: IKSolver
    ) -> bool:
    """For ``x`` a workspace parameter setting (obtained via
    ``config.getConfig(constraints)``), a set of constraints, and a IKSolver
    object, returns True if the solver can find a solution, starting from the
    robot's current configuration). Returns True if successful.
    """
    set_cartesian_constraints(x,constraints,solver)
    return solver.solve()

def _make_canonical(robot,constraints,startConfig,endConfig,solver):
    if not hasattr(constraints,'__iter__'):
        constraints = [constraints]
    for c in constraints:
        if isinstance(c,(int,str)):
            newconstraints = []
            for d in constraints:
                if isinstance(d,(int,str)):
                    newconstraints.append(ik.objective(robot.link(d),R=so3.identity(),t=[0,0,0]))
                else:
                    assert isinstance(d,IKObjective)
                    newconstraints.append(d)
            if solver:
                newSolver = IKSolver(solver)
                newSolver.clear()
                for d in newconstraints:
                    newSolver.add(d)
            else:
                newSolver = None
            constraints = newconstraints
            solver = newSolver
            break
    if solver is None:
        solver = ik.solver(constraints)
    if startConfig=='robot':
        startConfig = robot.getConfig()
    if endConfig=='robot':
        endConfig = robot.getConfig()
    return constraints,startConfig,endConfig,solver

def cartesian_interpolate_linear(
        robot: Union[RobotModel,SubRobotModel],
        a: Vector,
        b: Vector,
        constraints: Union[int,str,IKObjective,Sequence[int],Sequence[str],Sequence[IKObjective]],
        startConfig: Union[str,Vector] = 'robot',
        endConfig: Optional[Vector] = None,
        delta: float = 1e-2,
        solver: Optional[IKSolver] = None,
        feasibilityTest: Optional[Callable[[Vector],bool]] = None,
        maximize: bool = False
    ) -> Optional[RobotTrajectory]:
    """Resolves a continuous robot trajectory that interpolates between two
    cartesian points for specified IK constraints.  The output path is only a
    kinematic resolution, and has time domain [0,1].

    Differs from :func:`cartesian_interpolate_bisect` in that the solver moves
    incrementally along the path from a to b in linear fashion.

    Args:
        robot: the RobotModel or SubRobotModel.
        a (list of floats): start point of the Cartesian trajectory.
            Assumed derived from config.getConfig(constraints)
        b (list of floats): start point of the Cartesian trajectory.
            Assumed derived from config.getConfig(constraints)
        constraints: one or more link indices, link names, or
            :class:`IKObjective` objects specifying the Cartesian task.
            Interpreted as follows:

            * int or str: the specified link's entire pose is constrained
            * IKObjective: the links, constraint types, local positions, and
              local axes are used as constraints. The world space elements are
              considered temporary and will change to match the Cartesian
              trajectory.
            * list of int, list of str, or list of IKObjective: concatenates
              the specified constraints together

        startConfig (optional): either 'robot' (configuration taken from the
            robot), a configuration, or None (any configuration)
        endConfig (optional): same type as startConfig.
        delta (float, optional): the maximum configuration space distance
            between points on the output path
        solver (IKSolver, optional): if provided, an IKSolver configured with
            the desired parameters for IK constraint solving.
        feasibilityTest (function, optional): a function f(q) that returns
            false when a configuration q is infeasible
        maximize (bool, optional): if true, goes as far as possible along the
            path.

    Returns: 
        A configuration space path that interpolates
        the Cartesian path, or None if no solution can be found.

    """
    assert delta > 0,"Spatial resolution must be positive"
    constraints,startConfig,endConfig,solver = _make_canonical(robot,constraints,startConfig,endConfig,solver)

    assert startConfig is not None,"Unable to cartesian interpolate without a start configuration"
    robot.setConfig(startConfig)
    set_cartesian_constraints(a,constraints,solver)
    if not solver.isSolved():
        if not solver.solve():
            warnings.warn("cartesian_interpolate_linear(): initial configuration cannot be solved to match initial Cartesian coordinates, residual {}".format(solver.getResidual()),RuntimeWarning)
            return None
        warnings.warn("cartesian_interpolate_linear(): Warning, initial configuration does not match initial Cartesian coordinates, solving")
        startConfig = robot.getConfig()
    if feasibilityTest is not None and not feasibilityTest(startConfig):
        warnings.warn("cartesian_interpolate_linear(): initial configuration is infeasible",RuntimeWarning)
        return None
    if endConfig is not None:
        #doing endpoint-constrained cartesian interpolation
        set_cartesian_constraints(b,constraints,solver)
        robot.setConfig(endConfig)
        if not solver.isSolved():
            warnings.warn("cartesian_interpolate_linear(): Error, end configuration does not match final Cartesian coordinates, residual {}".format(solver.getResidual()),RuntimeWarning)
            return None
        if feasibilityTest is not None and not feasibilityTest(startConfig):
            warnings.warn("cartesian_interpolate_linear(): Error: final configuration is infeasible")
            return None

    res = RobotTrajectory(robot)
    t = 0
    res.times.append(t)
    res.milestones.append(startConfig)
    qmin0,qmax0 = solver.getJointLimits()
    tol0 = solver.getTolerance()
    solver.setTolerance(tol0*0.1)
    set_cartesian_constraints(a,constraints,solver)
    if not solver.isSolved():
        solver.solve()
        res.times.append(t+1e-7)
        res.milestones.append(robot.getConfig())
        t = res.times[-1]
    paramStallTolerance = 0.01*solver.getTolerance() / config.distance(constraints,a,b)
    stepsize = 0.1
    while t < 1:
        tookstep = False
        tend = min(t+stepsize,1)
        x = config.interpolate(constraints,a,b,tend)
        if endConfig is not None:
            robot.setConfig(robot.interpolate(startConfig,endConfig,tend))
            solver.setBiasConfig(robot.getConfig())
        q = res.milestones[-1]
        solver.setJointLimits([max(vmin,v-delta) for v,vmin in zip(q,qmin0)],[min(vmax,v+delta) for v,vmax in zip(q,qmax0)])
        #print "Trying step",tend-t,"time t=",tend
        if solve_cartesian(x,constraints,solver):
            #valid step, increasing step size
            #print "Accept and increase step"
            tookstep = True
            stepsize *= 1.5
        else:
            #do a line search
            while stepsize > paramStallTolerance:
                stepsize *= 0.5
                tend = min(t+stepsize,1)
                x = config.interpolate(constraints,a,b,tend)
                if endConfig is not None:
                    robot.setConfig(robot.interpolate(startConfig,endConfig,tend))
                    solver.setBiasConfig(robot.getConfig())
                else:
                    robot.setConfig(q)
                #print "Trying step",tend-t,"time t=",tend
                if solve_cartesian(x,constraints,solver):
                    #print "Accept"
                    tookstep = True
                    break
                else:
                    solver.setTolerance(tol0)
                    if solver.isSolved():
                        #print "Grudgingly accepted"
                        tookstep = True
                        break
                    solver.setTolerance(tol0*0.1)
        if not tookstep:
            warnings.warn("cartesian_interpolate_linear(): Failed to take a valid step along straight line path at time {} residual {}".format(res.times[-1],solver.getResidual()),RuntimeWarning)
            #x = config.interpolate(constraints,a,b,res.times[-1])
            #set_cartesian_constraints(x,constraints,solver)
            #robot.setConfig(res.milestones[-1])
            #print "Last residual",solver.getResidual()
            #x = config.interpolate(constraints,a,b,tend)
            #set_cartesian_constraints(x,constraints,solver)
            #print "Residual from last config",solver.getResidual()
            solver.setJointLimits(qmin0,qmax0)
            solver.setTolerance(tol0)
            if maximize:
                return res
            return None
        x = robot.getConfig()
        if feasibilityTest is not None and not feasibilityTest(x):
            warnings.warn("cartesian_interpolate_linear(): Infeasibility at time {}".format(tend),RuntimeWarning)
            solver.setJointLimits(qmin0,qmax0)
            solver.setTolerance(tol0)
            if maximize:
                return res
            return None
        #print "Distances from last:",max(abs(a-b) for (a,b) in zip(res.milestones[-1],x))
        res.times.append(tend)
        res.milestones.append(x)
        t = tend
    solver.setJointLimits(qmin0,qmax0)
    solver.setTolerance(tol0)
    if endConfig is not None:
        if robot.distance(res.milestones[-2],endConfig) > delta:
            #hit a local minimum, couldn't reach the goal
            if maximize:
                res.times.pop(-1)
                res.milestones.pop(-1)
                return res
            #print "Hit a local minimum while trying to reach end configuration"
            return None
        else:
            #clean up the end configuration
            res.milestones[-1] = endConfig
    return res

class _BisectNode:
    def __init__(self,a,b,ua,ub,qa,qb):
        self.a,self.b = a,b
        self.ua,self.ub = ua,ub
        self.qa,self.qb = qa,qb
        self.left,self.right = None,None

def cartesian_interpolate_bisect(
        robot: Union[RobotModel,SubRobotModel],
        a: Vector,
        b: Vector,
        constraints: Union[int,str,IKObjective,Sequence[int],Sequence[str],Sequence[IKObjective]],
        startConfig: Union[str,Vector] = 'robot',
        endConfig: Optional[Vector] = None,
        delta: float = 1e-2,
        solver: Optional[IKSolver] = None,
        feasibilityTest: Optional[Callable[[Vector],bool]] = None,
        maximize: bool = False,
        growthTol: int = 10
    ) -> Optional[RobotTrajectory]:
    """Resolves a continuous robot trajectory that interpolates between two
    cartesian points for a single link of a robot.  Note that the output path
    is only a kinematic resolution, and has time domain [0,1].

    Differs from :func:`cartesian_interpolate_linear` in that the solver
    creates the path from a to b using bisection.  This function may be more
    capable, but doesn't accept the ``maximize`` argument in case the query
    doesn't have a solution.

    Args:
        robot (RobotModel or SubRobotModel): the robot.
        a (list of floats): start point of the Cartesian trajectory.
            Assumed derived from config.getConfig(constraints)
        b (list of floats): start point of the Cartesian trajectory.
            Assumed derived from config.getConfig(constraints)
        constraints: one or more link indices, link names, or
            :class:`IKObjective` objects specifying the Cartesian task space. 
            Interpreted as follows:

            * int or str: the specified link's entire pose is constrained
            * IKObjective: the links, constraint types, local positions, and
              local axes are used as constraints. The world space elements are
              considered temporary and will change to match the Cartesian
              trajectory.
            * list of int, list of str, or list of IKObjective: concatenates
              the specified constraints together

        startConfig (optional): either 'robot' (configuration taken from the
            robot), a configuration, or None (any configuration)
        endConfig (optional): same type as startConfig.
        delta (float, optional): the maximum configuration space
            distance between points on the output path
        solver (IKSolver, optional): if provided, an IKSolver configured with
            the desired parameters for IK constraint solving.
        feasibilityTest (function, optional): a function f(q) that returns
            false when a configuration q is infeasible
        growthTol (float, optional): the end path can be at most ``growthTol`` 
            times the length of the length between the start and goal
            configurations.

    Returns: 
        A configuration space path that interpolates the Cartesian path, or
        None if no solution can be found.

    """
    assert delta > 0,"Spatial resolution must be positive"
    assert growthTol > 1,"Growth parameter must be in range [1,infty]"
    constraints,startConfig,endConfig,solver = _make_canonical(robot,constraints,startConfig,endConfig,solver)

    assert startConfig is not None,"Unable to cartesian bisection interpolate without a start configuration"
    if endConfig is None:
        #find an end point
        robot.setConfig(startConfig)
        if not solve_cartesian(b,constraints,solver):
            warnings.warn("cartesian_interpolate_bisect(): could not find an end configuration to match final Cartesian coordinates")
            return None
        endConfig = robot.getConfig()
    robot.setConfig(startConfig)
    set_cartesian_constraints(a,constraints,solver)
    if not solver.isSolved():
        if not solver.solve():
            warnings.warn("cartesian_interpolate_bisect(): initial configuration cannot be solved to match initial Cartesian coordinates, residual {}".format(solver.getResidual()),RuntimeWarning)
            return None
        warnings.warn("cartesian_interpolate_bisect(): initial configuration does not match initial Cartesian coordinates, solving")
        startConfig = robot.getConfig() 
    robot.setConfig(endConfig)
    set_cartesian_constraints(b,constraints,solver)
    if not solver.isSolved():
        if not solver.solve():
            warnings.warn("cartesian_interpolate_bisect(): final configuration cannot be solved to match final Cartesian coordinates, residual {}".format(solver.getResidual()),RuntimeWarning)
            return None
        warnings.warn("cartesian_interpolate_bisect(): final configuration does not match final Cartesian coordinates, solving")
        endConfig = robot.getConfig()   
    if feasibilityTest is not None and not feasibilityTest(startConfig):
        warnings.warn("cartesian_interpolate_bisect(): initial configuration is infeasible",RuntimeWarning)
        return None
    if feasibilityTest is not None and not feasibilityTest(endConfig):
        warnings.warn("cartesian_interpolate_bisect(): Error: final configuration is infeasible",RuntimeWarning)
        return None
    root = _BisectNode(a,b,0,1,startConfig,endConfig)
    root.d = robot.distance(startConfig,endConfig)
    dtotal = root.d
    dorig = root.d
    scalecond = 0.5*(2 - 2.0/growthTol)
    q = deque()
    q.append(root)
    while len(q) > 0:
        n = q.pop()
        d0 = n.d
        if d0 <= delta:
            continue
        m = config.interpolate(constraints,n.a,n.b,0.5)
        qm = robot.interpolate(n.qa,n.qb,0.5)
        um = (n.ua+n.ub)*0.5
        robot.setConfig(qm)
        solver.setBiasConfig(qm)
        if not solve_cartesian(m,constraints,solver):
            solver.setBiasConfig([])
            warnings.warn("cartesian_interpolate_bisect(): Failed to solve at point {}".format(um),RuntimeWarning)
            return None
        solver.setBiasConfig([])
        d1 = robot.distance(n.qa,qm)
        d2 = robot.distance(qm,n.qb)
        #print d1,d2
        #print qm,"->",robot.getConfig()
        qm = robot.getConfig()
        d1 = robot.distance(n.qa,qm)
        d2 = robot.distance(qm,n.qb)
        dtotal += d1+d2 - d0 
        if dtotal > dorig*growthTol or (d1 > scalecond*d0) or (d2 > scalecond*d0):
            warnings.warn("cartesian_interpolate_bisect(): Excessive growth condition reached {} {} {} at point {}".format(d0,d1,d2,um),RuntimeWarning)
            #print(n.qa)
            #print(qm)
            #print(n.qb)
            return None
        if feasibilityTest and not feasibilityTest(qm):
            warnings.warn("cartesian_interpolate_bisect(): Violation of feasibility test at point {}".format(um),RuntimeWarning)
            return None
        n.left = _BisectNode(n.a,m,n.ua,um,n.qa,qm)
        n.left.d = d1
        n.right = _BisectNode(m,n.b,um,n.ub,qm,n.qb)
        n.right.d = d2
        if d1 < d2:
            q.append(n.left)
            q.append(n.right)
        else:
            q.append(n.right)
            q.append(n.left)
    #done resolving, now output path from left to right of tree
    res = RobotTrajectory(robot,[0],[startConfig])
    q = [root]
    while len(q) > 0:
        n = q.pop(-1)
        if n.left is None:
            #leaf node
            res.times.append(n.ub)
            res.milestones.append(n.qb)
        else:
            q.append(n.right)
            q.append(n.left)
    return res

def cartesian_path_interpolate(
        robot: Union[RobotModel,SubRobotModel],
        path: Union[Trajectory,Sequence[Vector]],
        constraints: Union[int,str,IKObjective,Sequence[int],Sequence[str],Sequence[IKObjective]],
        startConfig: Union[str,Vector] = 'robot',
        endConfig: Optional[Vector] = None,
        delta: float = 1e-2,
        method: str = 'any',
        solver: Optional[IKSolver] = None,
        feasibilityTest: Optional[Callable[[Vector],bool]] = None,
        numSamples: int = 1000,
        maximize: bool = False
    ) -> Optional[RobotTrajectory]:
    """Resolves a continuous robot trajectory that follows a cartesian path 
    for one or more links of a robot.

    .. note::

        The output path is only a kinematic resolution, and may not respect
        the robot's velocity / acceleration limits.

    .. note::

        Only compatible with :class:`Trajectory`, not
        :class:`HermiteTrajectory`.  If a single link is provided, an
        :class:`SE3Trajectory` can be provided (but not
        :class:`SE3HermiteTrajectory`.)

    Args:
        robot (RobotModel or SubRobotModel): the robot.
        path (Trajectory or list of milestones): a cartesian path for the
            parameters of the the given constraints.  If only milestones are
            given, the milestones are spaced 1s apart in time.
        constraints: one or more link indices, link names, or or
            :class:`IKObjective` objects specifying the Cartesian task space. 
            Interpreted as follows:

            * int or str: the specified link's entire pose is constrained
            * IKObjective: the links, constraint types, local positions, and
              local axes are used as constraints. The world space elements
              are considered temporary and will change to match the Cartesian
              trajectory.
            * list of int, list of str, or list of IKObjective: concatenates
              the specified constraints together

        startConfig (optional): either 'robot' (configuration taken from the
            robot), a configuration, or None (any configuration)
        endConfig (optional): same type as startConfig.
        delta (float, optional): the maximum configuration space distance
            between points on the output path
        method (str): method used.  Can be 'any', 'pointwise', or 'roadmap'.
        solver (IKSolver, optional): if provided, an IKSolver configured with
            the desired parameters for IK constraint solving.
        feasibilityTest (function, optional): a function f(q) that returns
            False when a configuration q is infeasible
        numSamples (int, optional): if 'roadmap' or 'any' method is used,
            the # of configuration space samples that are used.
        maximize (bool, optional): if true, goes as far as possible along
            the path.

    Returns: 
        A configuration space path that interpolates the Cartesian path,
        or None if no solution can be found.

    """
    assert delta > 0,"Spatial resolution must be positive"
    if hasattr(path,'__iter__'):
        path = Trajectory(list(range(len(path))),path)
    constraints,startConfig,endConfig,solver = _make_canonical(robot,constraints,startConfig,endConfig,solver)
    #correct start and goal configurations, if specified
    if startConfig:
        robot.setConfig(startConfig)
        set_cartesian_constraints(path.milestones[0],constraints,solver)
        if not solver.isSolved():
            if not solver.solve():
                warnings.warn("cartesian_path_interpolate(): initial configuration cannot be solved to match initial Cartesian coordinates")
                return None
            warnings.warn("cartesian_path_interpolate(): Warning, initial configuration does not match initial Cartesian coordinates, solving",RuntimeWarning)
            startConfig = robot.getConfig() 
    if endConfig:
        robot.setConfig(endConfig)
        set_cartesian_constraints(path.milestones[-1],constraints,solver)
        if not solver.isSolved():
            if not solver.solve():
                warnings.warn("cartesian_path_interpolate(): final configuration cannot be solved to match final Cartesian coordinates")
                return None
            warnings.warn("cartesian_path_interpolate(): final configuration does not match final Cartesian coordinates, solving",RuntimeWarning)
            endConfig = robot.getConfig()   

    #now we're at a canonical setup
    if method == 'any' or method == 'pointwise':
        #try pointwise resolution first
        if startConfig is None:
            if ik.solve_global(constraints,solver.getMaxIters(),solver.getTolerance(),solver.getActiveDofs(),max(100,numSamples),feasibilityTest):
                startConfig = robot.getConfig()
            else:
                warnings.warn("cartesian_path_interpolate(): could not solve for start configuration")
                return None
        res = RobotTrajectory(robot)
        res.times.append(path.times[0])
        res.milestones.append(startConfig)
        infeasible = False
        for i in range(len(path.milestones)-1):
            if endConfig is None:
                segEnd = None
            else:
                u = (path.times[i+1] - path.times[i])/(path.times[-1] - path.times[i])
                segEnd = robot.interpolate(res.milestones[-1],endConfig,u)
                robot.setConfig(segEnd)
                if solve_cartesian(path.milestones[i+1],constraints,solver):
                    segEnd = robot.getConfig()
            if segEnd is None:
                seg = cartesian_interpolate_linear(robot,path.milestones[i],path.milestones[i+1],constraints,
                    startConfig=res.milestones[-1],endConfig=segEnd,delta=delta,solver=solver,feasibilityTest=feasibilityTest)
            else:
                seg = cartesian_interpolate_bisect(robot,path.milestones[i],path.milestones[i+1],constraints,
                    startConfig=res.milestones[-1],endConfig=segEnd,delta=delta,solver=solver,feasibilityTest=feasibilityTest)
            if not seg:
                warnings.warn("cartesian_path_interpolate(): Found infeasible cartesian interpolation segment at time {}".format(path.times[i+1]))
                infeasible = True
                break
            #concatenate
            dt = path.times[i+1] - path.times[i]
            seg.times = [t*dt for t in seg.times]
            res = res.concat(seg,relative=True)
        if not infeasible:
            #print "Resolved with pointwise interpolation!"
            return res
        if method == 'pointwise' and maximize:
            return res
    if method == 'any' or method == 'roadmap':
        #TODO: sample on continuous parameterization of path
        if path.duration() > 0:
            #manual discretization using config.interpolate
            numdivs = 20
            divpts = [path.startTime() + path.duration()*float(i)/(numdivs-1) for i in range(numdivs)]
            oldseg = 0
            oldu = 0
            times = [0]
            milestones = [path.milestones[0]]
            for t in divpts:
                s,u = path.getSegment(t)
                if s < 0:
                    s = 0
                if s+1 >= len(path.milestones):
                    s = len(path.milestones)-2
                    u = 1
                if s == oldseg:
                    if u != oldu:
                        assert t > times[-1]
                        times.append(t)
                        milestones.append(config.interpolate(constraints,path.milestones[s],path.milestones[s+1],u))
                else:
                    for i in range(oldseg+1,s+1):
                        assert path.times[i] > times[-1]
                        times.append(path.times[i])
                        milestones.append(path.milestones[i])
                    times.append(t)
                    #print(s,u)
                    milestones.append(config.interpolate(constraints,path.milestones[s],path.milestones[s+1],u))
                oldseg,oldu = s,u
            for i in range(len(times)-1):
                assert times[i] < times[i+1]
            path = path.constructor()(times,milestones)
        import random
        #mark whether we need to sample the end or start
        pathIndices = list(range(1,len(path.milestones)-1))
        if startConfig is None:
            pathIndices = [0] + pathIndices
        if endConfig is None:
            pathIndices = pathIndices + [len(path.milestones)-1]
        samp = 0
        if startConfig is None:
            #need to seed a start configuration
            while samp < numSamples:
                samp += 1
                solver.sampleInitial()
                if solve_cartesian(path.milestones[0],constraints,solver):
                    if feasibilityTest is None or feasibilityTest(robot.getConfig()):
                        startConfig = robot.getConfig()
                        break
        if endConfig is None:
            #need to seed an end configuration
            samp = 0
            while samp < numSamples:
                samp += 1
                if samp > 0:
                    solver.sampleInitial()
                else:
                    robot.setConfig(startConfig)
                if solve_cartesian(path.milestones[-1],constraints,solver):
                    if feasibilityTest is None or feasibilityTest(robot.getConfig()):
                        endConfig = robot.getConfig()
                        break
        if startConfig is None or endConfig is None:
            warnings.warn("cartesian_path_interpolate(): Exhausted all samples, perhaps endpoints are unreachable",RuntimeWarning)
            return None
        selfMotionManifolds = [[] for i in path.milestones]
        nodes = []
        configs = []
        ccs = []
        edges = []
        def findpath(depth):
            #start and goal are connected! find a path through the edges list using BFS
            eadj = [[] for n in nodes]
            for (i,j,p) in edges:
                eadj[i].append((j,p))
            q = deque()
            parent = [None]*len(nodes)
            for c in selfMotionManifolds[0]:
                q.append(c)
            #print "Adjacency list"
            #for i,alist in enumerate(eadj):
            #   print nodes[i],": ",' '.join(str(nodes[j]) for (j,p) in alist)

            while len(q) > 0:
                n = q.pop()
                for c,p in eadj[n]:
                    if parent[c] is not None:
                        continue
                    parent[c] = n
                    if nodes[c][0] == depth:
                        warnings.warn("cartesian_path_interpolate(): Found a path using roadmap after {} samples".format(samp),RuntimeWarning)
                        #arrived at goal node, trace parent list back
                        npath = []
                        n = c
                        while c is not None:
                            npath.append(c)
                            c = parent[c]
                        npath = [n for n in reversed(npath)]
                        warnings.warn(' '.join(str(nodes[n]) for n in npath),RuntimeWarning)
                        assert nodes[npath[0]][0] == 0,"Didnt end up at a start configuration?"
                        res = RobotTrajectory(robot)
                        res.times.append(path.times[0])
                        res.milestones.append(configs[npath[0]])
                        for i,n in enumerate(npath[:-1]):
                            found = False
                            for j,p in eadj[n]:
                                if j == npath[i+1]:
                                    #print "Suffix",p.times[0],p.times[-1]
                                    #print res.times[0],res.times[-1]
                                    res = res.concat(p,relative=False)
                                    #print "Resulting range",res.times[0],res.times[-1]
                                    found = True
                                    break
                            assert found,"Internal error? "+str(nodes[npath[i]])+" -> "+str(nodes[npath[i+1]])
                        return res
                    q.append(c)
            warnings.warn("cartesian_path_interpolate(): Path to depth {} could not be found".format(depth),RuntimeWarning)
            return None
        selfMotionManifolds[0].append(0)
        configs.append(startConfig)
        nodes.append((0,0))
        ccs.append(0)
        selfMotionManifolds[-1].append(1)
        configs.append(endConfig)
        nodes.append((len(path.milestones)-1,0))
        ccs.append(1)
        for samp in range(samp,numSamples):
            irand = random.choice(pathIndices)
            solver.sampleInitial()
            #check for successful sample on self motion manifold, test feasibility
            if not solve_cartesian(path.milestones[irand],constraints,solver):
                continue
            x = robot.getConfig()
            if feasibilityTest is not None and not feasibilityTest(x):
                continue
            #add to data structure
            nx = len(nodes)
            nodes.append((irand,len(selfMotionManifolds[irand])))
            ccs.append(nx)
            assert len(ccs) == nx+1
            selfMotionManifolds[irand].append(nx)
            configs.append(x)
            #try connecting to other nodes
            k = int(math.log(samp+2)) + 2
            #brute force k-nearest neighbor
            d = []
            for i,n in enumerate(nodes[:-1]):
                if n[0] == irand:
                    continue
                dist = config.distance(constraints,path.milestones[n[0]],path.milestones[irand])
                dist = robot.distance(x,configs[i])
                d.append((dist,i))
            k = min(k,len(d))
            #warnings.warn("cartesian_path_interpolate(): Sampled at time point {} checking {} potential connections".format(irand,k),RuntimeWarning)
            totest = [v[1] for v in sorted(d)[:k]]
            for n in totest:
                i = irand
                j = nodes[n][0]
                qi = x
                qj = configs[n]
                ni = nx
                nj = n
                if ccs[ni] == ccs[nj]:
                    #same connected component, use visibility graph technique
                    continue
                if i > j:
                    i,j = j,i
                    qi,qj = qj,qi
                    ni,nj = nj,ni
                pij = path.constructor()(path.times[i:j+1],path.milestones[i:j+1])
                #try connecting edges
                t = cartesian_path_interpolate(robot,pij,constraints,
                    startConfig=qi,endConfig=qj,delta=delta,method='pointwise',solver=solver,feasibilityTest=feasibilityTest)
                #t = cartesian_interpolate_bisect(robot,path.milestones[i],path.milestones[j],constraints,qi,qj,delta=delta,solver=solver,feasibilityTest=feasibilityTest)
                if t is None:
                    #warnings.warn("  Failed edge {} -> {}".format(nodes[ni],nodes[nj]),RuntimeWarning)
                    continue
                #t.times = [path.times[i] + v*(path.times[j]-path.times[i]) for v in t.times]
                #warnings.warn("  Added edge {} -> {}".format(nodes[ni],nodes[nj]),RuntimeWarning)
                edges.append((ni,nj,t))
                if ccs[ni] != ccs[nj]:
                    #not in same connected component, collapse ccs
                    src,tgt = ccs[ni],ccs[nj]
                    if src < tgt: src,tgt = tgt,src
                    checkgoal = False
                    for i,cc in enumerate(ccs):
                        if ccs[i] == src:
                            ccs[i] = tgt
                            if nodes[i][0] == 0 or nodes[i][0] == len(path.milestones)-1:
                                checkgoal=True
                    if checkgoal:
                        checkgoal = False
                        for c in selfMotionManifolds[0]:
                            for d in selfMotionManifolds[-1]:
                                if ccs[c] == ccs[d]:
                                    checkgoal = True
                                    break
                            if checkgoal:
                                break
                    if checkgoal:
                        return findpath(len(path.milestones)-1)
            if ccs[-1] != 0 and ccs[-1] != 1 and False:
                #didn't connect to either start or goal... delete isolated points?
                warnings.warn("cartesian_path_interpolate(): Isolated node, removing...",RuntimeWarning)
                edges = [(i,j,t) for (i,j,t) in edges if i != nx and j == nx]
                selfMotionManifolds[irand].pop(-1)
                nodes.pop(-1)
                configs.pop(-1)
                ccs.pop(-1)
            #raw_input()
        if maximize:
            #find the point furthest along the path
            startccs = set()
            for c in selfMotionManifolds[0]:
                startccs.add(ccs[c])
            maxdepth = 0
            maxnode = 0
            for i,cc in enumerate(ccs):
                if nodes[i][0] > maxdepth and cc in startccs:
                    maxdepth = nodes[i][0]
                    maxnode = i
            warnings.warn("cartesian_path_interpolate(): Connected components:",RuntimeWarning)
            for n,cc in zip(nodes,ccs):
                warnings.warn("  {}:{}".format(n,cc),RuntimeWarning)
            warnings.warn("cartesian_path_interpolate(): Got to depth {}".format(maxdepth),RuntimeWarning)
            return findpath(maxdepth)
        warnings.warn("cartesian_path_interpolate(): Unable to find a feasible path within {} iterations".format(numSamples),RuntimeWarning)
        warnings.warn("cartesian_path_interpolate(): Number of feasible samples per time instance:")
        return None
    return None


def cartesian_bump(
        robot: Union[RobotModel,SubRobotModel],
        js_path: Union[Trajectory,RobotTrajectory],
        constraints: Union[int,str,IKObjective,Sequence[int],Sequence[str],Sequence[IKObjective]],
        bump_paths: Union[RigidTransform,SE3Trajectory,Sequence[RigidTransform],Sequence[SE3Trajectory]],
        delta: float = 1e-2,
        solver: Optional[IKSolver] = None,
        ee_relative: bool = False,
        maximize: bool = False,
        closest: bool = False,
        maxDeviation: Optional[List[float]] = None
    ) -> Optional[RobotTrajectory]:
    """Given the robot and a reference joint space trajectory, "bumps" the
    trajectory in Cartesian space using a given relative transform (or
    transform path).  The movement in joint space is approximately minimized
    to follow the bumped Cartesian path.

    For example, to translate the motion of an end effector by [dx,dy,dz]
    in world coordinates,  call::

        cartesian_bump(robot,traj,ik.fixed_objective(link),se3.from_translation([dx,dy,dz]))

    Args:
        robot (RobotModel or SubRobotModel): the robot for which the bump is
            applied.  
        js_path (Trajectory or RobotTrajectory): the reference joint space
            Trajectory of the robot.
        constraints: one or more link indices, link names, or
            :class:`IKObjective` objects specifying the Cartesian task space. 
            giving the manner in which the Cartesian space is defined. 
            Interpreted as follows:

            * int or str: the specified link's entire pose is constrained
            * IKObjective: the links, constraint types, local positions, and
              local axes are used as constraints.  The world space elements
              are considered temporary and will change to match the
              Cartesian trajectory.
            * list of int, list of str, or list of IKObjective: concatenates
              the specified constraints together

        bump_paths: one or more transforms or transform paths specifying the
            world-space relative "bump" of each cartesian goal.  One bump per
            constraint must be given as input.  Each bump can either be a
            static klampt.se3 element or a SE3Trajectory.
        delta (float, optional): the maximum configuration space distance
            between points on the output path
        method: method used.  Can be 'any', 'pointwise', or 'roadmap'.
        solver (IKSolver, optional): if provided, an IKSolver configured with
            the desired parameters for IK constraint solving.
        ee_relative (bool, optional): if False (default), bumps are given in
            world coordinates.  If True, bumps are given in end-effector local
            coordinates.
        maximize (bool, optional): if true, goes as far as possible along the
            path.
        closest (bool, optional): if not resolved and this is True, the
            function returns the robot trajectory
            whose cartesian targets get as close as possible (locally) to the
            bumped trajectory
        maxDeviation (list of floats, optional): if not None, this is a vector
            giving the max joint space distance by which each active joint is
            allowed to move from `js_path`.

    Returns:
        The bumped trajectory, or None if no path can be found.
    """
    #make into canonical form
    if not hasattr(constraints,'__iter__'):
        constraints = [constraints]
    if not hasattr(bump_paths,'__iter__'):
        bump_paths = [bump_paths]
    assert len(constraints) == len(bump_paths),"Must specify one bump per constraint"
    if maxDeviation != None:
        assert len(maxDeviation) == robot.numLinks()
    for c in constraints:
        if isinstance(c,(int,str)):
            newconstraints = []
            for d in constraints:
                if isinstance(d,(int,str)):
                    newconstraints.append(ik.objective(robot.link(d),R=so3.identity(),t=[0,0,0]))
                else:
                    assert isinstance(d,IKObjective)
                    newconstraints.append(d)
    is1xform = any(isinstance(p,(int,float)) for p in bump_paths)
    if is1xform:
        bump_paths = [bump_paths]
    meshpts = []
    for i,p in enumerate(bump_paths):
        #it's a static transform, map it to a path
        if isinstance(p,(list,tuple)):
            bump_paths[i] = SE3Trajectory(times=[js_path.startTime()],milestones=[p])
        else:
            assert hasattr(p,'times'),"bump_paths must contain SE3Trajectory's"
            meshpts += p.times
    if solver is None:
        solver = ik.solver(constraints)
    #now preprocess the joint space so that everything is initially within delta distance
    for i in range(len(js_path.milestones)-1):
        d = robot.distance(js_path.milestones[i],js_path.milestones[i+1])
        if d > delta:
            #add in subdividing mesh points
            a,b = js_path.times[i],js_path.times[i+1]
            numdivs = int(math.ceil(d/delta))
            for j in range(1,numdivs):
                meshpts.append(a + float(j)/float(numdivs)*(b-a))
    #ensure that all the movements of the SE3 trajectories are captured
    if len(meshpts) > 0:
        js_path = js_path.remesh(meshpts)[0]
    qmin0,qmax0 = solver.getJointLimits()
    #OK, now move all the cartesian points
    numsolved = 0
    res = RobotTrajectory(robot)
    res.times = js_path.times
    closeIntervals = set()
    for i,q in enumerate(js_path.milestones):
        t = js_path.times[i]
        robot.setConfig(q)
        solver.clear()
        for c,p in zip(constraints,bump_paths):
            xform0 = robot.link(c.link()).getTransform()
            xformrel = p.eval_se3(t)
            xform = (se3.mul(xform0,xformrel) if ee_relative else se3.mul(xformrel,xform0))
            c.matchDestination(*xform)
            solver.add(c)
        if maxDeviation != None:
            qmin = [max(v-d,vmin) for (v,d,vmin) in zip(q,maxDeviation,qmin0)]
            qmax = [min(v+d,vmax) for (v,d,vmax) in zip(q,maxDeviation,qmax0)]
            solver.setJointLimits(qmin,qmax)
        if not solver.solve():
            warnings.warn("cartesian_bump(): Unable to perform Cartesian solve on milestone at time {}".format(t),RuntimeWarning)
            if not closest:
                if maximize:
                    #going as far as possible, just clip the result
                    res.times = res.times[:len(res.milestones)]
                    break
                else:
                    solver.setJointLimits(qmin0,qmax0)
                    return None
            else:
                closeIntervals.add(i)
                #otherwise soldier on with an imperfect solution
        else:
            numsolved += 1
        res.milestones.append(robot.getConfig())
    warnings.warn("cartesian_bump(): Solved %d/%d milestone configurations along path, now interpolating paths..."%(numsolved,len(res.milestones)),RuntimeWarning)
    numResolved = 0
    numTotalEdges = len(res.milestones)-1
    solver.setJointLimits(qmin0,qmax0)
    i = 0
    while i+1 < len(res.milestones):
        q = res.milestones[i]
        qnext = res.milestones[i+1]
        d = robot.distance(q,qnext)
        if d > delta:
            if i in closeIntervals:
                i += 1
                continue
            #resolve cartesian path
            ta = res.times[i]
            tb = res.times[i+1]
            robot.setConfig(q)
            for c,p in zip(constraints,bump_paths):
                xform0 = robot.link(c.link()).getTransform()
                c.matchDestination(*xform0)
            xa = config.getConfig(constraints)
            robot.setConfig(qnext)
            for c,p in zip(constraints,bump_paths):
                xform0 = robot.link(c.link()).getTransform()
                c.matchDestination(*xform0)
            xb = config.getConfig(constraints)
            #TODO: add joint limits into the solver?
            newseg = cartesian_interpolate_bisect(robot,xa,xb,constraints,
                startConfig=q,endConfig=qnext,
                delta=delta,solver=solver)
            if newseg == None:
                warnings.warn("cartesian_bump(): Unable to complete bump while subdividing segment at time {}".format(ta),RuntimeWarning)
                if closest:
                    i += 1
                    continue
                if maximize:
                    res.times = res.times[:i+1]
                    res.milestones = res.milestones[:i+1]
                return None
            #splice in the results
            assert newseg.times[0] == 0 and newseg.times[-1] == 1
            newseg.times = [ta + float(t)*(tb-ta) for t in newseg.times]
            res.times = res.times[:i+1]+newseg.times[1:-1]+res.times[i+1:]
            #print "Adding",len(newseg.milestones)-2,"intermediate milestones"
            assert res.milestones[i] == newseg.milestones[0]
            assert res.milestones[i+1] == newseg.milestones[-1]
            res.milestones = res.milestones[:i+1]+newseg.milestones[1:-1]+res.milestones[i+1:]
            #adjust close intervals
            newclose = set()
            for c in closeIntervals:
                if c > i:
                    newclose.add(c + len(newseg.times)-2)
            closeIntervals = newclose
            i += len(newseg.milestones)-2
            numResolved += 1
        else:
            #print "Skipping",i
            numResolved += 1
        i += 1
    warnings.warn("cartesian_bump(): Resolved %d/%d bumped edges"%(numResolved,numTotalEdges),RuntimeWarning)
    return res

def cartesian_move_to(
        robot:Union[RobotModel,SubRobotModel],
        constraints: Union[int,str,IKObjective,Sequence[int],Sequence[str],Sequence[IKObjective]],
        delta: float = 1e-2,
        solver: Optional[IKSolver] = None,
        feasibilityTest: Optional[Callable[[Vector],bool]] = None,
        maximize: bool = False
    ) -> Optional[RobotTrajectory]:
    """A convenience function that generates a path that performs a linear
    cartesian interpolation starting from the robot's current configuration
    and ending at the desired IK constraints.

    This is a bit more convenient than :func:`cartesian_interpolate_linear`
    since you only need to pass in the target objective, rather than the
    start and end Cartesian parameters as well. 

    Usage::

        path = cartesian_move_to(robot,goal)

    Other arguments are equivalent to those in cartesian_interpolate_linear.
    """
    if not hasattr(constraints,'__iter__'):
        constraints = [constraints]
    for c in constraints:
        assert isinstance(c,IKObjective)
    #extract the task space coordinates of the constraints
    taskEnd = config.getConfig(constraints)
    #extract the task space coordinates of the current robot
    for c in constraints:
        xforml = robot.link(c.link()).getTransform()
        xformr = robot.link(c.destLink()).getTransform() if c.destLink() >= 0 else se3.identity()
        c.matchDestination(*se3.mul(se3.inv(xformr),xforml))
    taskStart = config.getConfig(constraints)
    #just call the solver
    return cartesian_interpolate_linear(robot,taskStart,taskEnd,constraints,
        delta=delta,solver=solver,feasibilityTest=feasibilityTest,
        maximize=maximize)
