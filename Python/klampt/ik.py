"""Inverse kinematics methods.  These use the underlying C++ solver in
Klamp't.  Compared to the SWIG bindings in robotik.h, these helpers
are more convenient (and Pythonic). 

The IK solver is a Newton-Raphson solver, which acts like a local
optimization.  It starts from the robot model's initial configuration
and takes a number of iterations to seek the objective.  The solution
is placed in the robot model's configuration.  If successful, the
solution is also guaranteed to satisfy the robot's joint limits.

For basic IK, the calling sequence is

    link = robot.getLink("name")
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

from robotsim import *

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
    if not hasattr(body,'getRobot'):
        generalized = True
    else: 
        if ref and (not hasattr(ref,'getRobot') or ref.getRobot() != body.getRobot()):
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
        obj.robot = body.getRobot()
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

def objects(objectives):
    """Returns a list of all objects touched by the given objective(s).
    Not currently implemented."""
    raise NotImplementedError()
    pass

def solver(objectives):
    """Returns a solver for the given objective(s). Either a single objective
    or a list of objectives can be provided. 
    
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
                robs.getdefault(obj.robot,[]).append(obj)
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
            for obj in generalized:
                s.add(obj)
            for (r,objs) in robs.iteritems():
                for obj in objs:
                    s.add(GeneralizedIKObjective(r,obj))
            return s
        else:
            res = []
            for (r,objs) in robs:
                s = IKSolver(r)
                for obj in objs:
                    s.add(obj)
                res.append(s)
            if len(res)==1:
                return res[0]
            return res
    else:
        if isinstance(objectives,IKObjective):
            s = IKSolver(objectives.robot)
            s.add(objectives)
            return s
        elif isinstance(objectives,GeneralizedIKObjective):
            world = None
            if objectives.isObj1:
                world = WorldModel(objectives.obj1.world)
            else:
                world = WorldModel(objectives.link1.world)
            s = GeneralizedIKSolver(world)
            s.add(objectives)
            return s
        else:
            raise TypeError("Objective is of wrong type")

def solve(objectives,iters=1000,tol=1e-3):
    """Attempts to solve the given objective(s). Either a single objective
    or a list of objectives can be provided.

    Returns true if a solution is successfully found to the given tolerance,
    within the provided number of iterations.
    
    (The objectives should be a result from the :func:`objective` function.
    Beware that if you are making your own goals by calling the IKObjective
    constructors from the robotsim module, the .robot member of these goals
    must be set).
    """
    s = solver(objectives)
    if hasattr(s,'__iter__'):
        res = [si.solve(iters,tol)[0] for si in s]
        return all(res)
    else:
        return s.solve(iters,tol)[0]

def residual(objectives):
    """Returns the residual of the given objectives."""
    return solver(objectives).getResidual()

def jacobian(objectives):
    """Returns the jacobian of the given objectives."""
    return solver(objectives).getJacobian()
