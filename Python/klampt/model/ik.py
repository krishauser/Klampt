"""Inverse kinematics methods.  These use the underlying C++ solver in
Klamp't.  Compared to the SWIG bindings in robotik.h, these helpers
are more convenient (and Pythonic). 

The IK solver is a Newton-Raphson solver, which acts like a local
optimization.  It starts from the robot model's initial configuration
and takes a number of iterations to seek the objective.  The solution
is placed in the robot model's configuration.  If successful, the
solution is also guaranteed to satisfy the robot's joint limits.

For basic IK, the calling sequence is::

    link = robot.link("name")
    goal = ik.objective(link,local=[p1,p2,p3],world=[r1,r2,r3])
    robot.setConfig(startConfig)  #(optional, set the initial configuration)
    if ik.solve(goal):
        print("Hooray, IK solved")
        print("Resulting config:",robot.getConfig())
    else:
        print("IK failed")
        print("Final config:",robot.getConfig())

Here, the points p1,...,p3 on the link (specified in local coordinates)
will be matched to the points r1,...,r3 in the world.  You can also
directly constrain the translation/rotation of the link via the form::
    
    ik.objective(link,R=link.getTransform()[0],t=[x,y,z])
    
which will keep the orientation of the link constant, while setting its
position.

A convenience function is ik.fixed_objective(link) which fixes the position
of the link in its current position/orientation in world coordinates.
Individual points on the link can be constrained to their coordinates using
the local argument, as follows::

    ik.fixed_objective(link,local=[p1,p2,p3])

or if you prefer to give world coordinates, like so::

    ik.fixed_objective(link,world=[q1,q2,q3])

More advanced usage can constrain a link to another link using the ref
argument to ik.objective().  You may also pass multiple goals to ik.solve().

For more control over the solution process, you may use a ik.solver() object.
This object will let you set the active degrees of freedom, get the residual
and Jacobian, and sample random initial solutions.  (see
klampt.robotsim.IKSolver)

    1. By default, the active degrees of freedom are all ancestors of
       the constrained links.  For more control, set them using a solver
       object.
    2. As with any local IK method, Newton-Raphson has a tendency to get
       stuck in local minima.  As a result you may wish to choose random
       initial configurations if the first solve fails.

.. note::
    IK solving with constraints between robot links and rigid objects
    (GeneralizedIKObjective and GeneralizedIKSolver)
    is not yet implemented and will result in an exception.
    This functionality may be supported in the future but is not on the
    current development path.

"""

from ..robotsim import *
from ..math import so3,se3,vectorops
from .subrobot import SubRobotModel
from typing import Union,Optional,List,Sequence,Callable
from .typing import IntArray,Vector,Vector3,Rotation,RigidTransform

def objective(
        body: Union[RobotModelLink,RigidObjectModel],
        ref: Union[None,RobotModelLink,RigidObjectModel] = None,
        local: Union[None,Vector3,List[Vector3]] = None,
        world: Union[None,Vector3,List[Vector3]] = None,
        R: Optional[Rotation] = None,
        t: Optional[Vector3] = None
    ) -> IKObjective:
    """Returns an IKObjective for a given body.

    There are two modes in which this can be used:

    1. If `local` and `world` are provided, then this objective asks to match 
       the local point (s) on the body to the world point(s).
    2. If `R` and `t` are provided, then the objective is set to specify the
       transform of the body.  (In this mode,` local` can be set to a 3-vector
       to match the point on the body to the world point `t` rather than the
       origin).

    If `ref` is None, the target frame is the world frame, otherwise, it
    specifies a target frame bound to a movable object.  In mode 1, the world
    points are bound to this frame; in mode 2, the transformation from body to
    ref is specified as (R,t).

    Args:
        body (RobotModelLink or RigidObjectModel): the link that should be
            constrained.
        ref (RobotModelLink or RigidObjectModel, optional): the link that
            `body` should be constrained to, or None for the world frame.
        local (3-vector, or a list of 3-vectors, optional): the local 
            coordinates on `body` should be constrained to the corresponding
            points in `world`
        world (3-vector, or a list of 3-vectors, optional): the coordinates
            to which the points in `local` should be constrained. These are
            given in world coordinates (if `ref`=None) or in `ref`'s frame.
        R (so3 element; list of 9 floats, optional): the rotation that the
            link should take on.
        t (3-vector, optional): the translation that the link's origin
            should take on.

    Returns:
        An IK objective describing the constraint.

        If body and ref are not on the same robot, then a
        GeneralizedIKObjective may be returned. TODO: not implemented yet.
    """
    generalized = False
    if not hasattr(body,'robot'):
        generalized = True
    else: 
        if ref and (not hasattr(ref,'robot') or ref.robot() != body.robot()):
            generalized=True

    if generalized:
        obj = None
        if ref:
            obj = GeneralizedIKObjective(body,ref)
        else:
            obj = GeneralizedIKObjective(body)
        if (local is not None) and (world is not None):
            assert(len(local)==len(world))
            if hasattr(local[0],'__iter__'):
                #sanity check
                for pt in local:
                    assert(len(pt)==3)
                for pt in world:
                    assert(len(pt)==3)
                obj.setPoints(local,world)
            else:
                obj.setPoint(local,world)
        elif R and t:
            obj.setTransform(R,t)
        else:
            raise RuntimeError("Need to specify either local and world or R and t")
        return obj
    else:
        obj = IKObjective()
        obj.robot = body.robot()
        if (local is not None) and (world is not None):
            assert(len(local)==len(world))
            if hasattr(local[0],'__iter__'):
                #sanity check
                for pt in local:
                    assert(len(pt)==3)
                for pt in world:
                    assert(len(pt)==3)
                if ref:
                    obj.setRelativePoints(body.index,ref.index,local,world)
                else:
                    obj.setFixedPoints(body.index,local,world)
            else:
                if ref:
                    obj.setRelativePoint(body.index,ref.index,local,world)
                else:
                    obj.setFixedPoint(body.index,local,world)
        elif (R is not None) and (t is not None):
            if local is not None:
                if len(local) != 3 or hasattr(local[0],'__iter__'):
                    raise ValueError("If local is provided with R,t arguments, it must be a 3-vector")
                #it's a local reference point
                t = vectorops.sub(t,so3.apply(R,local))
            if ref:
                obj.setRelativeTransform(body.index,ref.index,R,t)
            else:
                obj.setFixedTransform(body.index,R,t)
        else:
            raise RuntimeError("Need to specify either local and world or R and t")
        return obj


def fixed_objective(
        link: RobotModelLink,
        ref: Union[None,RobotModelLink,RigidObjectModel] = None,
        local: Union[None,Vector3,List[Vector3]] = None,
        world: Union[None,Vector3,List[Vector3]] = None
    ) -> IKObjective:
    """Convenience function for fixing the given link at the current position
    in space. 

    The arguments are interpreted as follows:

    - If local and world are not provided, the entire link is
      constrained. 

    - If only local is provided, these points are fixed to their current
      positions in space.  

    - If only world is provided, the points on the link with the given world
      position are constrained in place.

    Args:
        link (RobotModelLink): the link that should be constrained.
        ref (RobotModelLink or RigidObjectModel, optional): the link that
            `link` should be constrained to, or None for the world frame.
        local (3-vector, or a list of 3-vectors, optional): the local 
            coordinates on `body` should be constrained in place
        world (3-vector, or a list of 3-vectors, optional): the world
            coordinates on `body` should be constrained in place

    """
    refcoords = ref.getTransform() if ref is not None else se3.identity()
    Tw = link.getTransform()
    Trel = se3.mul(se3.inv(refcoords),Tw)
    if local is not None and not hasattr(local[0],'__iter__'):
        #just a single point, make it a list of points
        local = [local]
    if world is not None and not hasattr(world[0],'__iter__'):
        #just a single point, make it a list of points
        world = [world]
    if local is None and world is None:
        #fixed rotation/position objective
        return objective(link,ref,R=Trel[0],t=Trel[1])
    elif local is None:
        #fixed point, given by world coordinates
        Trelinv = se3.inv(Trel)
        local = [se3.apply(Trelinv,p) for p in world]
        return objective(link,ref,local=local,world=world)
    elif world is None:
        #fixed point, given by local coordinates
        world = [se3.apply(Trel,p) for p in local]
        return objective(link,ref,local=local,world=world)
    else:
        raise ValueError("ik.fixed_objective does not accept both local and world keyword arguments")

def fixed_rotation_objective(
        link: RobotModelLink,
        ref: Union[None,RobotModelLink,RigidObjectModel] = None,
        local_axis: Optional[Vector3] = None,
        world_axis: Optional[Vector3] = None
    ) -> IKObjective:
    """Convenience function for fixing the given link at its current
    orientation in space. 

    The arguments are interpreted as follows:

    - If `local_axis` and `world_axis` are not provided, the entire link's
      orientation is constrained.

    - If only `local_axis` is provided, the link is constrained
      to rotate about this local axis. 

    - If only `world_axis` is provided,
      the link is constrained to rotate about this world-space axis.


    Args:
        link (RobotModelLink): the link that should be constrained.
        ref (RobotModelLink or RigidObjectModel, optional): the link that
            `link` should be constrained to, or None for the world frame.
        local_axis (3-vector, optional): the local direction on
            on `body` that should be constrained in place.
        world_axis (3-vector, optional): the world coordinates
            of the direction that should be constrained.

    """
    refcoords = ref.getTransform()[0] if ref is not None else so3.identity()
    Rw = link.getTransform()
    Rrel = so3.mul(so3.inv(refcoords),Rw[0])
    obj = IKObjective()
    obj.robot = link.robot()
    if ref:
        assert link.robot()==ref.robot(),"Can't do generalized fixed rotation objectives yet"
    obj.setLinks(link.index,(-1 if ref is None else ref.index))
    if local_axis is None and world_axis is None:
        #fixed rotation objective
        obj.setFixedRotConstraint(Rrel)
    elif local_axis is None:
        #fixed axis, given by world coordinates
        Rrelinv = so3.inv(Rrel)
        local_axis = so3.apply(Rrelinv,world_axis)
        obj.setAxialRotConstraint(local_axis,world_axis)
    elif world_axis is None:
        #fixed point, given by local coordinates
        world_axis = so3.apply(Rrel,local_axis)
        obj.setAxialRotConstraint(local_axis,world_axis)
    else:
        raise ValueError("ik.fixed_rotation_objective does not accept both local_axis and world_axis keyword arguments")
    return obj


def objects(objectives):
    """Returns a list of all objects touched by the given objective(s).
    TODO: Not currently implemented."""
    raise NotImplementedError()


class SubRobotIKSolver(IKSolver):
    """A version of IKSolver for subrobots.  Allows addressing of
    configurations and links according to the indices of the SubRobotModel
    provided upon initialization.
    """
    def __init__(self, r: SubRobotModel):
        assert(isinstance(r, SubRobotModel))
        super().__init__(r._robot)
        super().setActiveDofs(r._links)
        self.subrobot = r

    def copy(self) -> "SubRobotIKSolver":
        return SubRobotIKSolver(self.subrobot)

    def setActiveDofs(self, active: IntArray) -> None:
        r"""
        Sets the active degrees of freedom.  
        Respects subrobotness.

        Args:
            active (:obj:`list of int`)
        """
        return super().setActiveDofs([self.subrobot._links[x] for x in active])

    def getActiveDofs(self) -> List[int]:
        r"""
        Returns the active degrees of freedom.  
        Respects subrobotness.

        """
        return [self.subrobot._inv_links[i] for i in super().getActiveDofs()]

    def setJointLimits(self, qmin: Vector, qmax: Vector) -> None:
        r"""
        Sets limits on the robot's configuration. If empty, this turns off joint limits.  
        Respects subrobotness.

        Args:
            qmin (:obj:`list of floats`)
            qmax (:obj:`list of floats`)
        """
        qmin_old, qmax_old = super().getJointLimits()
        for n, lower, upper in zip(self.subrobot._links, qmin, qmax):
            qmin_old[n] = lower
            qmax_old[n] = upper
        return super().setJointLimits(qmin_old, qmax_old)

    def getJointLimits(self):
        r"""
        Returns the limits on the robot's configuration (by default this is the robot's
        joint limits.  

        Respects subrobotness.

        Returns:
            tuple: (qmin, qmax) giving lists of joint limits.
        """
        qmin_full, qmax_full = super().getJointLimits()
        qmin = [qmin_full[i] for i in self.subrobot._links]
        qmax = [qmax_full[i] for i in self.subrobot._links]
        return qmin, qmax

    def setBiasConfig(self, bias_config: Vector) -> None:
        r"""
        Biases the solver to approach a given configuration. Setting an empty vector
        clears the bias term.  
        Respects subrobotness.

        Args:
            bias_config (:obj:`list of floats`)
        """
        old_config = super().getBiasConfig()
        if len(old_config) == 0:
            old_config = [0.0]*self.robot.numLinks()
        for n, x in zip(self.subrobot._links, bias_config):
            old_config[n] = x
        return super().setBiasConfig(old_config)

    def getBiasConfig(self) ->Vector:
        r"""
        Returns the solvers' bias configuration.  
        Respects subrobotness.

        """
        config = super().getBiasConfig()
        return [config[i] for i in self.subrobot._links]

    def getJacobian(self):
        r"""
        Computes the matrix describing the instantaneous derivative of the objective
        with respect to the active Dofs.  
        Respects subrobotness.

        Returns:
            ndarray: Matrix (6 x N) J; such that J @ qdot = EE velocity.
        """
        return super().getJacobian()[:, self.subrobot._links]



def solver(
        objectives: Union[IKObjective,Sequence[IKObjective]],
        iters: Optional[int] = None,
        tol: Optional[float] = None,
        secondary : Optional[Union[IKObjective,Sequence[IKObjective],Tuple[Callable,Callable]]] = None,
    ) -> IKSolver:
    """Returns a solver for the given objective(s). 

    Args:
        objectives (IKObjective or list of IKObjectives): the objective(s) that
            should be solved.

            Note that these should be result(s) from the :func:`objective`
            function.  OR, if you are making your own goals by calling the
            ``IKObjective`` constructors from the ``robotsim`` module, you must
            set the ``.robot`` member of each objective to the
            :class:`RobotModel` being solved.

        iters (int, optional): if given, the max # of iterations for the solver
            is set to this value.  Otherwise, the default value (100) is used

        tol (float, optional): if given, the constraint solving tolerance for
            the solver is set to this value.  Otherwise, the default value
            (1e-3) is used.
        
        secondary (IKObjective or list of IKObjective, optional): if given,
            tries to optimize a secondary objective.  Make sure to use
            ``solver.minimize()`` rather than ``solver.solve()``

    .. note::
        In rare cases, this may return a list of IKSolver's if you give
        it objectives on different robots.  They should be solved
        independently for efficiency.

    """
    result = None
    if hasattr(objectives,'__iter__'):
        generalized = []
        robs = dict()
        for obj in objectives:
            if isinstance(obj,IKObjective):
                if not hasattr(obj,'robot'):
                    raise ValueError("IKObjective objects must have 'robot' member for use in ik.solver. Either set this manually or use the ik.objective function")
                robot = obj.robot
                key = (robot.getName(),robot.getID())
                robs.setdefault(key,[robot,[]])[1].append(obj)
            elif isinstance(obj,GeneralizedIKObjective):
                generalized.append(obj)
            else:
                raise TypeError("Objective is of wrong type")
        if len(generalized) != 0:
            #need a generalized solver
            world = None
            if generalized[0].isObj1:
                world = WorldModel(generalized[0].obj1.world)
            else:
                world = WorldModel(generalized[0].link1.world)
            result = GeneralizedIKSolver(world)
        else:
            result = []
            for key,(r,objs) in robs.items():
                if isinstance(r,SubRobotModel):
                    s = SubRobotIKSolver(r)
                else:
                    s = IKSolver(r)
                for obj in objs:
                    s.add(obj)
                result.append(s)
    else:
        if isinstance(objectives,IKObjective):
            if not hasattr(objectives,'robot'):
                raise ValueError("IKObjective object must have 'robot' member for use in ik.solver. Either set this manually or use the ik.objective function")
            if isinstance(objectives.robot,SubRobotModel):
                r = objectives.robot
                result = SubRobotIKSolver(r)
            else:
                result = IKSolver(objectives.robot)
            result.add(objectives)
        elif isinstance(objectives,GeneralizedIKObjective):
            world = None
            if objectives.isObj1:
                world = WorldModel(objectives.obj1.world)
            else:
                world = WorldModel(objectives.link1.world)
            result = GeneralizedIKSolver(world)
            result.add(objectives)
        else:
            raise TypeError("Objective is of wrong type")
    
    if not isinstance(result,list):
        result = [result]
    if secondary is not None:
        #add secondary objectives
        if not hasattr(secondary,'__iter__'):
            secondary = [secondary]
        for s in result:        
            for o in secondary:
                s.addSecondary(o)
    for s in result:
        if iters is not None: s.setMaxIters(iters)
        if tol is not None: s.setTolerance(tol)
    if len(result)==1:
        return result[0]
    return result


def solve(
        objectives: Union[IKObjective,Sequence[IKObjective]],
        iters: int = 1000,
        tol: float = 1e-3,
        activeDofs: Optional[List[int]] = None,
        secondary: Optional[Union[IKObjective,Sequence[IKObjective],Tuple[Callable,Callable]]]=None
    ) -> bool:
    """Attempts to solve the given objective(s). Either a single objective
    or a list of simultaneous objectives can be provided.

    Args:
        objectives: either a single IKObjective or a list of IKObjectives.
        iters (int, optional): a maximum number of iterations.
        tol (float, optional): a maximum error tolerance on satisfying the
            objectives
        activeDofs (list, optional): a list of link indices or names to use for
            IK solving.  By default, will determine these automatically.
        secondary (IKObjective or list of IKObjective or tuple (func,gradient),
            optional): if given, tries to optimize a secondary objective.  If a
            tuple, it provides both a function and its gradient.

    .. note::
        You cannot use sub-robots and activeDofs at the same time.  Undefined
        behavior will result.

    Returns:
        True if a solution is successfully found to the given tolerance,
        within the provided number of iterations.  The robot(s) are then set
        to the solution configuration.  If a solution is not found, False is
        returned and the robot(s) are set to the best found configuration.

    This function will be smart about multiple objects / robots.  If the
    objectives are on disjoint robots then disjoint IK problems will be
    solved individually, which is fater.
    
    (The objectives should be a result from the :func:`objective` function.
    Beware that if you are making your own goals by calling the IKObjective
    constructors from the robotsim module, the .robot member of these goals
    must be set).
    """
    minimize = False
    minimize_args = ()
    if secondary is not None:
        if isinstance(secondary,Tuple) and len(secondary)==2 and callable(secondary[0]) and callable(secondary[1]):
            minimize_args = secondary
            secondary = None
    s = solver(objectives,iters,tol,secondary)
    if activeDofs is not None:
        links = activeDofs[:]
        for i,l in enumerate(links):
            if isinstance(l,str):
                links[i] = s.robot.link(l).getIndex()
        s.setActiveDofs(links)

    if hasattr(s,'__iter__'):
        if minimize:
            res = [si.minimize(*minimize_args)[0] for si in s]
            return all(res)
        res = [si.solve()[0] for si in s]
        return all(res)
    else:
        if minimize:
            return s.minimize(*minimize_args)
        return s.solve()

def residual(objectives: Union[IKObjective,Sequence[IKObjective]]) -> Vector:
    """Returns the residual of the given objectives."""
    return solver(objectives).getResidual()

def jacobian(objectives: Union[IKObjective,Sequence[IKObjective]]): 
    """Returns the jacobian of the given objectives."""
    return solver(objectives).getJacobian()


def solve_global(
        objectives: Union[IKObjective,Sequence[IKObjective]],
        iters: int = 1000,
        tol: float = 1e-3,
        activeDofs: Optional[List[int]] = None,
        secondary: Optional[Union[IKObjective,Sequence[IKObjective],Tuple[Callable,Callable]]]=None,
        numRestarts: int=100,
        feasibilityCheck: Optional[Callable[[],bool]] = None,
        startRandom: bool=False
    ) -> bool:
    """Attempts to solve the given objective(s) but avoids local minima
    to some extent using a random-restart technique.  
        
    Args:
        objectives, iters, tol, activeDofs, secondary: same as :func:`solve`
        numRestarts (int, optional): the number of random restarts.  The larger
            this is, the more likely a solution will be found, but if no
            solution exists, the routine will take more time.
        feasibilityCheck (function, optional): if provided, it will be a
            function feasible() that returns True if the robot/world in the
            current configuration is feasible, and False otherwise.
        startRandom (bool, optional): True if you wish to forgo the robot's
            current configuration and start from random initial configurations.

    Returns:
        True if a feasible solution is successfully found to the given
        tolerance, within the provided number of iterations.
    """
    if feasibilityCheck is None: feasibilityCheck=lambda : True
    minimize = False
    minimize_args = ()
    if secondary is not None:
        if isinstance(secondary,Tuple) and len(secondary)==2 and callable(secondary[0]) and callable(secondary[1]):
            minimize_args = secondary
            secondary = None

    s = solver(objectives,iters,tol,secondary)
    if activeDofs is not None:
        links = activeDofs[:]
        for i,l in enumerate(links):
            if isinstance(l,str):
                links[i] = s.robot.link(l).getIndex()
        s.setActiveDofs(links)
    if not hasattr(s,'__iter__'):
        s = [s]
    if startRandom:
        s[0].sampleInitial()
    if minimize:
        res = [si.minimize(*minimize_args) for si in s]
    else:
        res = [si.solve() for si in s]
    if all(res):
        if feasibilityCheck():
            return True
    for i in range(numRestarts):
        for si in s:
            si.sampleInitial()
        if minimize:
            res = [si.minimize(*minimize_args) for si in s]
        else:
            res = [si.solve() for si in s]
        if all(res):
            if feasibilityCheck():
                return True
    return False
    
    
def solve_nearby(
        objectives: Union[IKObjective,Sequence[IKObjective]],
        maxDeviation: float,
        iters: int = 1000,
        tol: float = 1e-3,
        activeDofs: Optional[List[int]] = None,
        secondary: Optional[Union[IKObjective,Sequence[IKObjective],Tuple[Callable,Callable]]]=None,
        numRestarts: int = 0,
        feasibilityCheck: Optional[Callable[[],bool]] = None
    ) -> bool:
    """Solves for an IK solution that does not deviate too far from the
    initial configuration.

    .. note::
        Currently only supports single-robot objectives!
    
    Args: 
        objectives, iters, tol, activeDofs, feasibilityCheck: same as :func:`solve_global` except for...
        maxDeviation (float): the maximum absolute amount in configuration space that
            each configuration element is allowed to change.
        numRestarts (int): same as in :func:`solve_global` , but is by default set to zero.
    """
    if feasibilityCheck is None: feasibilityCheck=lambda : True
    minimize = False
    minimize_args = ()
    if secondary is not None:
        if isinstance(secondary,Tuple) and len(secondary)==2 and callable(secondary[0]) and callable(secondary[1]):
            minimize_args = secondary
            secondary = None

    s = solver(objectives,iters,tol)
    if not isinstance(s,IKSolver):
        raise NotImplementedError("solve_nearby: currently only supports single-robot objectives")
    if hasattr(objectives,'__iter__'):
        robot = objectives[0].robot
    else:
        robot = objectives.robot
    if activeDofs is not None:
        links = activeDofs[:]
        for i,l in enumerate(links):
            if isinstance(l,str):
                links[i] = s.robot.link(l).getIndex()
        s.setActiveDofs(links)
    #set up the joint limits
    dofs = s.getActiveDofs()
    q = robot.getConfig()
    qmin,qmax = robot.getJointLimits()
    for d in dofs:
        qmin[d] = max(qmin[d],q[d]-maxDeviation)
        qmax[d] = min(qmax[d],q[d]+maxDeviation)
    s.setJointLimits(qmin,qmax)
    s.setBiasConfig(q)
    #start solving
    if minimize:
        solved = s.minimize(*minimize_args)
    else:
        solved = s.solve()
    if solved and feasibilityCheck():
        return True
    for i in range(numRestarts):
        s.sampleInitial()
        if minimize:
            solved = s.minimize(*minimize_args)
        else:
            solved = s.solve()
        if solved and feasibilityCheck():
            return True
    return False
