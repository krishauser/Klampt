from .cspace import *
from .cspaceutils import EmbeddedCSpace,AffineEmbeddedCSpace,EmbeddedMotionPlan
from . import robotcspace
from ..model import collide
from ..robotsim import IKObjective
import warnings

def preferred_plan_options(robot,movingSubset=None,optimizing=False):
    """Returns some options that might be good for your given robot, and
    whether you want a feasible or just an optimal plan.

    TODO: base this off of info about the robot, such as dimensionality,
    joint ranges, etc.
    """
    if optimizing:
        return { 'type':"rrt", 'perturbationRadius':0.5, 'bidirectional':1, 'shortcut':1, 'restart':1, 'restartTermCond':"{foundSolution:1,maxIters:1000}" }
    else:
        return { 'type':"sbl", 'perturbationRadius':0.5, 'randomizeFrequency':1000, 'shortcut':1 }


def make_space(world,robot,
              edgeCheckResolution=1e-2,
              extraConstraints=[],
              equalityConstraints=[],
              equalityTolerance=1e-3,
              ignoreCollisions=[],
              movingSubset=None):
    """Creates a standard CSpace instance for the robot moving in the given world.

    Args:
        world (WorldModel): the world in which the robot lives, including
            obstacles.
        robot (RobotModel): the moving robot
        edgeCheckResolution (float, optional): the resolution at which edges in
            the path are checked for feasibility
        extraConstraints (list, optional): possible extra constraint functions,
            each of which needs to return True if satisfied. 

            .. note::

                Don't put cartesian constraints here! Instead place your
                function in equalityConstraints.

        equalityConstraints (list, optional): a list of IKObjectives or
            equality constraints f(x)=0 that must be satisfied during the
            motion.  Equality constraints may return a float or a list of
            floats.  In the latter case, this is interpreted as a vector
            function, in which all entries of the vector must be 0.
        equalityTolerance (float, optional): a tolerance to which all the
            equality constraints must be satisfied.
        ignoreCollisions (list): a list of ignored collisions. Each element
            may be a body in the world, or a pair (a,b) where a, b are bodies
            in the world.
        movingSubset (optional): if None, 'all', or 'auto' (default), all
            joints will be allowed to move.  If this is a list, then only
            these joint indices will be allowed to move.

    Returns:
        CSpace: a C-space instance that describes the robot's feasible space.
            This can be used for planning by creating a :class:`cspace.MotionPlan`
            object.  Note that if an EmbeddedCSpace is returned, you should
            create a EmbeddedMotionPlan for greater convenience.
    """
    subset = []
    if movingSubset == 'auto' or movingSubset == 'all' or movingSubset == None:
        subset = None
    else:
        subset = movingSubset
        
    collider = collide.WorldCollider(world,ignore=ignoreCollisions)
    
    implicitManifold = []
    for c in equalityConstraints:
        if not isinstance(c,IKObjective):
            implicitManifold.append(c)
    
    if len(equalityConstraints)==0:
        space = robotcspace.RobotCSpace(robot,collider)
    else:
        if len(implicitManifold) > 0:
            raise NotImplementedError("General inequality constraints")
        else:
            space = robotcspace.ClosedLoopRobotCSpace(robot,equalityConstraints,collider)
            space.tol = equalityTolerance
            if subset is not None and len(subset) < robot.numLinks():
              space.setIKActiveDofs(subset)
    space.eps = edgeCheckResolution

    for c in extraConstraints:
        space.addConstraint(c)

    #New in 0.8.6: configuration spaces with affine drivers
    has_affine = False
    for d in range(robot.numDrivers()):
        dr = robot.driver(d)
        if dr.getType() == 'affine':
            if subset is not None and len(subset) < robot.numLinks():
                if any(l in subset for l in dr.getAffectedLinks()):
                    has_affine = True
                    break
            else:
                has_affine = True
                break
    if has_affine:
        if not isinstance(space,robotcspace.RobotCSpace):
            raise ValueError("Robot is affected by affine links, but closed-chain constraints are specified. Can't handle this combination of constraints yet")
        affected_drivers = []
        affected_links = set(list(range(robot.numLinks())) if subset is None else subset)
        for d in range(robot.numDrivers()):
            dr = robot.driver(d)
            for l in dr.getAffectedLinks():
                if l in affected_links:
                    affected_drivers.append(d)
                    break
        embedded_space = AffineEmbeddedCSpace.fromRobotDrivers(robot,space,affected_drivers)

        #copy and paste from EmbeddedRobotCSpace
        active = [False]*robot.numLinks()
        for i in affected_links:
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
        embedded_space.setup()
        return embedded_space

    if subset is not None and len(subset) < robot.numLinks():
        #choose a subset
        sspace = robotcspace.EmbeddedRobotCSpace(space,subset,xinit=robot.getConfig())
        sspace.disableInactiveCollisions()
        space = sspace
    
    space.setup()
    return space

def plan_to_config(world,robot,target,
                 edgeCheckResolution=1e-2,
                 extraConstraints=[],
                 equalityConstraints=[],
                 equalityTolerance=1e-3,
                 ignoreCollisions=[],
                 movingSubset='auto',
                 verbose=True,
                 **planOptions):
    """Creates a MotionPlan object that can be called to solve a standard motion
    planning problem for a robot in a world.  The plan starts from the robot's
    current configuration and ends in a target configuration.

    Args:
        world (WorldModel): the world in which the robot lives, including
            obstacles
        robot (RobotModel): the moving robot.  The plan start configuration
            is the robot's current configuration `robot.getConfig()`.
        target (list of float): the desired final configuration of the robot.
        edgeCheckResolution (float, optional): the resolution at which edges
            in the path are checked for feasibility
        extraConstraints (list, optional): possible extra constraint functions,
            each of which needs to return True if satisfied.  

            .. note::

                Don't put cartesian constraints here! Instead place your
                function in equalityConstraints.

        equalityConstraints (list, optional): a list of IKObjectives or
            equality constraints f(x)=0 that must be satisfied during the
            motion. Equality constraints may return a float or a list of
            floats.  In the latter case, this is interpreted as a vector
            function, in which all entries of the vector must be 0.
        equalityTolerance (float, optional): a tolerance to which all the
            equality constraints must be satisfied.
        ignoreCollisions (list): a list of ignored collisions. Each element may
            be a body in the world, or a pair (a,b) where a, b are bodies in
            the world.
        movingSubset (optional): if 'auto' (default), only the links that are
            different between the robot's current config and target config will
            be allowed to move.  Otherwise, if this is None or 'all', all
            joints will be allowed to move.  If this is a list, then only these
            joint indices will be allowed to move.
        planOptions (keywords): keyword options that will be sent to the
            planner.  See the documentation for MotionPlan.setOptions for more
            details.
    
    Returns: 
        MotionPlan: a planner instance that can be called to get a
        kinematically-feasible plan. (see :meth:`MotionPlan.planMore`)

        The underlying configuration space (a RobotCSpace, ClosedLoopRobotCSpace, or
        EmbeddedRobotCSpace) can be retrieved using the "space" attribute of the
        resulting MotionPlan object.
    """
    q0 = robot.getConfig()
    assert(len(q0)==len(target)),"target configuration must be of correct size for robot"
    if movingSubset == 'auto':
        subset = []
        for i,(a,b) in enumerate(zip(q0,target)):
            if a != b:
                subset.append(i)
    elif movingSubset == 'all' or movingSubset == None:
        subset = list(range(len(q0)))
    else:
        subset = movingSubset
        for i in range(len(q0)):
            if i not in subset:
                if q0[i] != target[i]:
                    raise ValueError("Error: target configuration value differs from start configuration along a fixed DOF: %s (link %d): %g vs %g"%(robot.link(i).getName(),i,q0[i],target[i]))
    
    space = make_space(world=world,robot=robot,
                      edgeCheckResolution=edgeCheckResolution,
                      extraConstraints=extraConstraints,
                      equalityConstraints=equalityConstraints,
                      equalityTolerance=equalityTolerance,
                      ignoreCollisions=ignoreCollisions,
                      movingSubset=subset)
    
    if hasattr(space,'lift'):  #the planning takes place in a space of lower dimension than #links
        plan = EmbeddedMotionPlan(space,q0,**planOptions)
    else:
        plan = MotionPlan(space,**planOptions)
    try:
        plan.setEndpoints(q0,target)
    except RuntimeError:
        #one of the endpoints is infeasible, print it out
        if space.cspace==None: space.setup()
        sfailures = space.cspace.feasibilityFailures(plan.space.project(q0))
        gfailures = space.cspace.feasibilityFailures(plan.space.project(target))
        if sfailures:
            warnings.warn("Start configuration fails {}".format(sfailures))
            if 'self collision' in sfailures:
                robot.setConfig(q0)
                for i in range(robot.numLinks()):
                    for j in range(i):
                        if robot.selfCollisionEnabled(i,j):
                            if robot.link(i).geometry().collides(robot.link(j).geometry()):
                                print("  Links {} and {} collide".format(robot.link(i).getName(),robot.link(j).getName()))
        if gfailures:
            warnings.warn("Goal configuration fails {}".format(gfailures))
            if 'self collision' in gfailures:
                robot.setConfig(target)
                for i in range(robot.numLinks()):
                    for j in range(i):
                        if robot.selfCollisionEnabled(i,j):
                            if robot.link(i).geometry().collides(robot.link(j).geometry()):
                                print("  Links {} and {} collide".format(robot.link(i).getName(),robot.link(j).getName()))
        return None
    return plan


def plan_to_set(world,robot,target,
              edgeCheckResolution=1e-2,
              extraConstraints=[],
              equalityConstraints=[],
              equalityTolerance=1e-3,
              ignoreCollisions=[],
              movingSubset=None,
              **planOptions):
    """
    Creates a MotionPlan object that can be called to solve a standard motion
    planning problem for a robot in a world.  The plan starts from the robot's
    current configuration and ends in a target set.

    Args:
        world (WorldModel): the world in which the robot lives, including
            obstacles
        robot (RobotModel): the moving robot.  The plan starts from
            robot.getConfig()
        target (function or CSpace): a function f(q) returning a bool which is
            True if the configuration q is a goal, OR an instance of a CSpace
            subclass where sample() generates a sample in the target set and
            feasible(x) tests whether a sample is in the target set. 

            .. note::

                The function should accept vectors of the same dimensionality 
                as the robot, not the moving subset.  Similarly, the CSpace
                should have the same dimensionality as the robot.

        edgeCheckResolution (float, optional): the resolution at which edges in the path are
            checked for feasibility
        extraConstraints (list, optional): possible extra constraint functions, each
            of which needs to return True if satisfied. 

            .. note::

                Don't put cartesian constraints here! Instead place your function in equalityConstraints.
                
        equalityConstraints (list, optional): a list of IKObjectives or equality
            constraints f(x)=0 that must be satisfied during the motion. Equality
            constraints may return a float or a list of floats.  In the latter case, this
            is interpreted as a vector function, in which all entries of the vector must be 0.
        equalityTolerance (float, optional): a tolerance to which all the equality constraints
            must be satisfied.
        ignoreCollisions (list): a list of ignored collisions. Each element may be
            a body in the world, or a pair (a,b) where a, b are bodies in the world.
        movingSubset (optional): if 'auto', 'all', or None (default), all joints
            will be allowed to move.  If this is a list, then only these joint
            indices will be allowed to move.
        planOptions (keywords): keyword options that will be sent to the planner.  See
            the documentation for MotionPlan.setOptions for more details.
    
    Returns: 
        MotionPlan: a planner instance that can be called to get a
        kinematically-feasible plan. (see :meth:`MotionPlan.planMore` )

        The underlying configuration space (a RobotCSpace, ClosedLoopRobotCSpace, or
        EmbeddedRobotCSpace) can be retrieved using the "space" attribute of the
        resulting MotionPlan object.
    """
    q0 = robot.getConfig()
    subset = []
    if movingSubset == 'auto' or movingSubset == 'all' or movingSubset == None:
        subset = list(range(len(q0)))
    else:
        subset = movingSubset

    space = make_space(world=world,robot=robot,
                      edgeCheckResolution=edgeCheckResolution,
                      extraConstraints=extraConstraints,
                      equalityConstraints=equalityConstraints,
                      equalityTolerance=equalityTolerance,
                      ignoreCollisions=ignoreCollisions,
                      movingSubset=subset)

    if hasattr(space,'lift'):  #the planning takes place in a space of lower dimension than #links
        plan = EmbeddedMotionPlan(space,q0,**planOptions)
    else:
        plan = MotionPlan(space,q0,**planOptions)

    #convert target to a (test,sample) pair if it's a cspace
    if isinstance(target,CSpace):
        goal = [(lambda x:target.feasible(x)),(lambda : target.sample())]
    else:
        if not callable(target):
            if not isinstance(target,(tuple,list)) or len(target)!=2 or not callable(target[0]) or not callable(target[1]):
                raise TypeError("target must be a predicate function or CSpace object")
        goal = target

    try:
        plan.setEndpoints(q0,goal)
    except RuntimeError:
        #the start configuration is infeasible, print it out
        if space.cspace==None: space.setup()
        sfailures = space.cspace.feasibilityFailures(plan.space.project(q0))
        warnings.warn("Start configuration fails {}".format(sfailures))
        raise
    return plan

def plan_to_cartesian_objective(world,robot,iktargets,iktolerance=1e-3,
                             extraConstraints=[],
                             equalityConstraints=[],
                             equalityTolerance=1e-3,
                             ignoreCollisions=[],
                             movingSubset=None,
                             **planOptions):
    """
    Plans a path to reach one or more IK targets.

    Args:
        world (WorldModel): same as plan_to_config
        iktargets (list of :class:`IKObjective`): a list of IKObjective
            instances (see the ik module)
        iktolerance (float): a tolerance to which the ik objectives must be
            satisfied

    Returns: 
        MotionPlan: a planner instance that can be called to get a
        kinematically-feasible plan. (see :meth:`MotionPlan.planMore` )

        The underlying configuration space (a RobotCSpace, ClosedLoopRobotCSpace, or
        EmbeddedRobotCSpace) can be retrieved using the "space" attribute of the
        resulting MotionPlan object.
    """
    #TODO: only subselect those links that are affected by the IK target
    goalset = robotcspace.ClosedLoopRobotCSpace(robot,iktargets,None)
    return plan_to_set(world,robot,goalset,
                     extraConstraints=extraConstraints,
                     equalityConstraints=equalityConstraints,
                     equalityTolerance=equalityTolerance,
                     ignoreCollisions=ignoreCollisions,
                     movingSubset=movingSubset,
                     **planOptions)

class _WizardGUI:
    def __init__(self):
        self.world = None
        self.movingObject = None
        self.cspace = None
        self.plannerSettings = preferred_plan_options(None)
        self.startConfig = None
        self.goalConfig = None
        self.goalIKTargets = None
        self.goalSetTest = None
        self.goalSetSampler = None
        self.extraConstraints = []
        self.equalityConstraints = []
        self.movingSubset = None

        #these are temporary objects
        self.activeCSpace = None
        self.planner = None
        self.activeMovingSubset = None
        self.currentRoadmap = None
        self.currentSolution = None

        #visualization settings
        self.draw_end_effectors = None
        self.draw_infeasible = False
        self.debug_plan_time = 30

    def makePlanner(self,use_active_space=False):
        if self.startConfig is None:
            start = self.movingObject.getConfig()
        else:
            start = self.startConfig
        if self.goalConfig is not None:
            goal = self.goalConfig
        elif self.goalIKTargets is not None:
            goal = robotcspace.ClosedLoopRobotCSpace(robot,self.goalIKTargets,None)
        elif self.goalSetSampler is not None:
            goal = (self.goalSetTest,self.goalSetSampler)
        elif self.goalSetTest is not None:
            goal = self.goalSetTest
        else:
            #no goal, can't create the planner
            return None
        if use_active_space:
            plan = MotionPlan(self.activeCSpace,**self.plannerSettings)
        else:
            plan = MotionPlan(self.cspace,**self.plannerSettings)
        plan.setEndpoints(start,goal)
        return plan



def wizard(world_or_space_or_plan,moving_object=None,
    draw_end_effectors=None,draw_infeasible=False,
    debug_plan_time=30):
    """Launches a "wizard" to help set up or debug a planner.  The wizard
    will allow you to configure the planner, including the group of moving
    joints, start and terminal sets, and collision detection settings. 

    The return value is a configured MotionPlan object, ready to be launched.

    The wizard will also allow you to get a Python string that sets up the
    space and/or invokes the planner.

    Arguments:
        world_or_space_or_plan (WorldModel, CSpace, or MotionPlan): the
            world containing the moving robot, or the currently configured
            CSpace or MotionPlan.
        moving_object (RobotModel or RigidObjectModel, optional): if
            world_or_space_or_plan is a WorldModel, this is the moving object
            for which you'd like to plan.  By default, robot 0 is moving.
        draw_end_effectors (list, optional): if provided, initializes the
            links to be drawn in the motion plan debugger.
        draw_infeasible (bool, optional): initializes whether the motion plan
            debugger will show infeasible configurations
        debug_plan_time (float, optional): initializes the planning time in
            the motion plan debugger.

    Returns:
        MotionPlan: a properly configured MotionPlan object that can be called
        to get a motion plan. (see :meth:`MotionPlan.planMore`).
    """
    gui = _WizardGUI()
    if isinstance(world_or_space_or_plan,WorldModel):
        gui.world = world_or_space_or_plan
        if moving_object is None:
            if gui.world.numRobots() == 0:
                if gui.world.numRigidObjects() == 0:
                    raise ValueError("World has no robots or rigid objects")
                moving_object = gui.world.rigidObject(0)
            else:
                moving_object = gui.world.robot(0)
        if not isinstance(moving_object,(RobotModel,RigidObjectModel)):
            raise TypeError("Invalid type of moving_object")
        gui.cspace = make_space(gui.world,moving_object)
    elif isinstance(world_or_space_or_plan,CSpace):
        gui.cspace = world_or_space_or_plan
    elif isinstance(world_or_space_or_plan,MotionPlan):
        plan = world_or_space_or_plan
        gui.cspace = world_or_space_or_plan.space
    if isinstance(gui.cspace,EmbeddedCSpace):
        gui.activeMovingSubset = gui.cspace.mapping
    if plan is not None:
        import json
        gui.plannerSettings = json.loads(plan.planOptions)
        start,goal = plan.planner.getEndpoints()
        gui.startConfig = start
        if hasattr(goal,'__iter__'):
            if len(goal)==2 and callable(goal[0]):
                gui.goalSetTest,gui.goalSetSampler = goal
            else:
                gui.endConfig = goal
        else:
            assert callable(goal)
            gui.goalSetTest = goal
        #TODO: parse IK targets

def _deprecated_func(oldName,newName):
    import sys
    mod = sys.modules[__name__]
    f = getattr(mod,newName)
    def depf(*args,**kwargs):
        warnings.warn("{} will be deprecated in favor of {} in a future version of Klampt".format(oldName,newName),DeprecationWarning)
        return f(*args,**kwargs)
    depf.__doc__ = 'Deprecated in a future version of Klampt. Use {} instead'.format(newName)
    setattr(mod,oldName,depf)

_deprecated_func('preferredPlanOptions','preferred_plan_options')
_deprecated_func('makeSpace','make_space')
_deprecated_func('planToConfig','plan_to_config')
_deprecated_func('planToSet','plan_to_set')
_deprecated_func('planToCartesianObjective','plan_to_cartesian_objective')
