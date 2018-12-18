"""Inverse kinematics methods.  These use the underlying C++ solver in
Klamp't.  Compared to the SWIG bindings in robotik.h, these helpers
are more convenient (and Pythonic). 

The IK solver is a Newton-Raphson solver, which acts like a local
optimization.  It starts from the robot model's initial configuration
and takes a number of iterations to seek the objective.  The solution
is placed in the robot model's configuration.  If successful, the
solution is also guaranteed to satisfy the robot's joint limits.

For basic IK, the calling sequence is

    link = robot.link("name")
    goal = ik.objective(link,local=[p1,p2,p3],world=[r1,r2,r3])
    robot.setConfig(startConfig)  #(optional, set the initial configuration)
    if ik.solve(goal):
        print "Hooray, IK solved"
        print "Resulting config:",robot.getConfig()
    else:
        print "IK failed"
        print "Final config:",robot.getConfig()

Here, the points p1,...,p3 on the link (specified in local coordinates)
will be matched to the points r1,...,r3 in the world.  You can also
directly constrain the translation/rotation of the link via the form:
    
    ik.objective(link,R=link.getTransform()[0],t=[x,y,z])
    
which will keep the orientation of the link constant, while setting its
position.

A convenience function is ik.fixed_objective(link) which fixes the position
of the link in its current position/orientation in world coordinates.
Individual points on the link can be constrained to their coordinates using
the local argument, as follows:

    ik.fixed_objective(link,local=[p1,p2,p3])

or if you prefer to give world coordinates, like so:

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

Note: IK solving with constraints between robot links and rigid objects
is not yet implemented and will result in a thrown exception.
"""

from ..robotsim import *
from ..math import so3,se3
from .subrobot import SubRobotModel
from .coordinates import Point,Direction,Frame,Transform

def objective(body,ref=None,local=None,world=None,R=None,t=None):
    """Returns an IKObjective or GeneralizedIKObjective for a given body.
    If ref is provided, there is a target body.

    If local and world are provided (either as single 3D points, or a list of
    3D points), then this objective will match the local point(s) to the
    world point(s).

    If R and t are provided, then the objective is set to a relative
    transformation between body and ref.
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
        if local and world:
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
        if isinstance(obj.robot,SubRobotModel):
            obj.robot = obj.robot._robot
        if local and world:
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
        elif R and t:
            if ref:
                obj.setRelativeTransform(body.index,ref.index,R,t)
            else:
                obj.setFixedTransform(body.index,R,t)
        else:
            raise RuntimeError("Need to specify either local and world or R and t")
        return obj


def fixed_objective(link,ref=None,local=None,world=None):
    """Convenience function for fixing the given link at the current position
    in space.  If local and world are not provided, the entire link is
    constrained.  If only local is provided, these points are fixed
    to their current positions in space.  If only world is provided,
    the points on the link with the given world position are constrained in
    place."""
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
        local = [se3.apply(trelinv,p) for p in world]
        return objective(link,ref,local=local,world=world)
    elif world is None:
        #fixed point, given by local coordinates
        world = [se3.apply(Trel,p) for p in local]
        return objective(link,ref,local=local,world=world)
    else:
        raise ValueError("ik.fixed_objective does not accept both local and world keyword arguments")

def fixed_rotation_objective(link,ref=None,local_axis=None,world_axis=None):
    """Convenience function for fixing the given link at its current orientation
    in space.  If local_axis and world_axis are not provided, the entire link's orientation
    is constrained.  If only local_axis is provided, the link is constrained
    to rotate about this local axis.  If only world_axis is provided,
    the link is constrained to rotate about this world-space axis."""
    refcoords = ref.getTransform()[0] if ref is not None else so3.identity()
    Rw = link.getTransform()
    Rrel = so3.mul(so3.inv(refcoords),Rw[0])
    obj = IKObjective()
    obj.robot = link.robot()
    if ref:
        assert link.robot()==ref.robot(),"Can't do generalized fixed rotation objectives yet"
    if local_axis is None and world_axis is None:
        #fixed rotation objective
        obj.setLinks(link.index,(-1 if ref is None else ref.index))
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
    Not currently implemented."""
    raise NotImplementedError()
    pass

def solver(objectives,iters=None,tol=None):
    """Returns a solver for the given objective(s). Either a single objective
    or a list of objectives can be provided. 

    If iters != None / tol != None, the max # of iterations / constraint solving tolerance
    for the solver is set.  Otherwise, the default values are used
    
    The result is either an IKSolver or a GeneralizedIKSolver corresponding
    to the given objective(s).  (see klampt.robotsim.IKSolver and
    klampt.robotsim.GeneralizedIKSolver).

    In rare cases, it may return a list of IKSolver's if you give
    it objectives on different robots.  They should be solved
    independently for efficiency.
    
    (The objectives should be a result from the :func:`objective` function.
    Beware that if you are making your own goals by calling the IKObjective
    constructors from the robotsim module, the .robot member of these goals
    must be set).
    """
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
            s = GeneralizedIKSolver(world)
            if iters != None: s.setMaxIters(iters)
            if tol != None: s.setTolerance(tol)
            for obj in generalized:
                s.add(obj)
            for (key,(r,objs)) in robs.items():
                for obj in objs:
                    s.add(GeneralizedIKObjective(r,obj))
            return s
        else:
            res = []
            for key,(r,objs) in robs.items():
                if isinstance(r,SubRobotModel):
                    s = IKSolver(r._robot)
                    s.setActiveDofs(r.links)
                else:
                    s = IKSolver(r)
                if iters != None: s.setMaxIters(iters)
                if tol != None: s.setTolerance(tol)
                for obj in objs:
                    s.add(obj)
                res.append(s)
            if len(res)==1:
                return res[0]
            return res
    else:
        if isinstance(objectives,IKObjective):
            if not hasattr(objectives,'robot'):
                raise ValueError("IKObjective object must have 'robot' member for use in ik.solver. Either set this manually or use the ik.objective function")
            if isinstance(objectives.robot,SubRobotModel):
                s = IKSolver(objectives.robot._robot)
                s.setActiveDofs(r.links)
            else:
                s = IKSolver(objectives.robot)
            if iters != None: s.setMaxIters(iters)
            if tol != None: s.setTolerance(tol)
            s.add(objectives)
            return s
        elif isinstance(objectives,GeneralizedIKObjective):
            world = None
            if objectives.isObj1:
                world = WorldModel(objectives.obj1.world)
            else:
                world = WorldModel(objectives.link1.world)
            s = GeneralizedIKSolver(world)
            if iters != None: s.setMaxIters(iters)
            if tol != None: s.setTolerance(tol)
            s.add(objectives)
            return s
        else:
            raise TypeError("Objective is of wrong type")

def solve(objectives,iters=1000,tol=1e-3,activeDofs=None):
    """Attempts to solve the given objective(s). Either a single objective
    or a list of simultaneous objectives can be provided.

    Arguments:
        - objectives: either a single IKObjective or a list of IKObjectives.
        - iters: a maximum number of iterations.
        - tol: a maximum error tolerance on satisfying the objectives
        - activeDofs: a list of link indices or names to use for IK solving.
          Note: cannot use sub-robots and activeDofs at the same time.  Undefined
          behavior will result.

    Returns True if a solution is successfully found to the given tolerance,
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
    s = solver(objectives,iters,tol)
    if activeDofs is not None:
        links = activeDofs[:]
        for i,l in enumerate(links):
            if isinstance(l,str):
                links[i] = s.robot.link(l).getIndex()
        s.setActiveDofs(links)

    if hasattr(s,'__iter__'):
        res = [si.solve()[0] for si in s]
        return all(res)
    else:
        return s.solve()

def residual(objectives):
    """Returns the residual of the given objectives."""
    return solver(objectives).getResidual()

def jacobian(objectives):
    """Returns the jacobian of the given objectives."""
    return solver(objectives).getJacobian()


def solve_global(objectives,iters=1000,tol=1e-3,activeDofs=None,numRestarts=100,feasibilityCheck=None,startRandom=False):
    """Attempts to solve the given objective(s) but avoids local minima
    to some extent using a random-restart technique.  
        
    Arguments: same as :func:`solve` except for...
    - numRestarts: the number of random restarts.  The larger this is, the more
      likely a solution will be found, but if no solution exists, the routine
      will take more time.
    - feasibilityCheck: if provided, it will be a function that takes no
      arguments and returns True if the robot/world in the current
      configuration is feasible, and False otherwise.
    - startRandom: True if you wish to forgo the robot's current configuration
      and start from random initial configurations

    Returns true if a feasible solution is successfully found to the given
    tolerance, within the provided number of iterations.
    """
    if feasibilityCheck is None: feasibilityCheck=lambda : True
    s = solver(objectives,iters,tol)
    if activeDofs is not None:
        links = activeDofs[:]
        for i,l in enumerate(links):
            if isinstance(l,str):
                links[i] = s.robot.link(l).getIndex()
        s.setActiveDofs(links)
    if hasattr(s,'__iter__'):
        if startRandom:
            s.sampleInitial()
        res = [si.solve() for si in s]
        if all(res):
            if feasibilityCheck():
                return True
        for i in range(numRestarts):
            for si in s:
                si.sampleInitial()
            res = [si.solve() for si in s]
            if all(res):
                if feasibilityCheck():
                    return True
        return False
    else:
        if startRandom:
            s.sampleInitial()
        if s.solve():
            if feasibilityCheck():
                return True
        for i in range(numRestarts):
            s.sampleInitial()
            if s.solve():
                if feasibilityCheck():
                    return True
        return False

def solve_nearby(objectives,maxDeviation,iters=1000,tol=1e-3,activeDofs=None,numRestarts=0,feasibilityCheck=None):
    """Solves for an IK solution that does not deviate too far from the
    initial configuration.

    Note: currently only supports single-robot objectives!
    
    Arguments: same as :func:`solve_global` except for...
    - maxDeviation: the maximum absolute amount in configuration space that
      each configuration element is allowed to change.
    - numRestarts: same as in solve_global, but is by default set to zero.
    """
    if feasibilityCheck is None: feasibilityCheck=lambda : True
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
    if s.solve():
        if feasibilityCheck():
            return True
    for i in range(numRestarts):
        s.sampleInitial()
        if s.solve():
            if feasibilityCheck():
                return True
    return False
