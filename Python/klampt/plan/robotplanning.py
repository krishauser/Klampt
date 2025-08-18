from .cspace import *
from .cspaceutils import EmbeddedCSpace,AffineEmbeddedCSpace,EmbeddedKinematicPlanner
from . import robotcspace
from ..model import collide
from ..robotsim import IKObjective,RobotModel,RobotModelLink,RigidObjectModel,WorldModel
from ..model.typing import Config,Vector,Vector3,RigidTransform
import time
from typing import List,Union,Optional,Callable
import warnings

def preferred_plan_options(robot : RobotModel, movingSubset=None, optimizing=False):
    """Returns some options that might be good for your given robot, and
    whether you want a feasible or just an optimal plan.

    TODO: base this off of info about the robot, such as dimensionality,
    joint ranges, etc.
    """
    if optimizing:
        return { 'type':"rrt", 'perturbationRadius':0.5, 'bidirectional':1, 'shortcut':1, 'restart':1, 'restartTermCond':"{foundSolution:1,maxIters:1000}" }
    else:
        return { 'type':"sbl", 'perturbationRadius':0.5, 'randomizeFrequency':1000, 'shortcut':1 }


def make_space(world : WorldModel, robot : RobotModel,
              edgeCheckResolution : float = 1e-2,
              extraConstraints : List[Callable]= [],
              equalityConstraints : List[Union[Callable,IKObjective]]=[],
              equalityTolerance : float = 1e-3,
              ignoreCollisions=[],
              movingSubset : Optional[Union[str,List[int]]]=None,
              attachedObjects : Optional[List[Tuple[RigidObjectModel,RobotModelLink]]]=None,
              attachedObjectRelTransforms : Optional[List[RigidTransform]]=None):
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
        attachedObjects (list of (RigidObjectModel,RobotModelLink), optional): a
            list of rigid objects and the links to which they are attached. 
            This is used to model grasped objects.
        attachedObjectRelTransforms (list of RigidTransform, optional): if
            any objects are attached, the relative transform from the object to
            the target link.  If not provided, the current transforms are used
            to determine the relative transforms.

    Returns:
        CSpace: a C-space instance that describes the robot's feasible space.
            This can be used for planning by creating a :class:`cspace.KinematicPlanner`
            object.  Note that if an EmbeddedCSpace is returned, you should
            create a EmbeddedKinematicPlanner for greater convenience.
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
    
    if attachedObjects is not None and len(attachedObjects) > 0:
        if len(equalityConstraints) > 0 or len(implicitManifold) > 0:
            raise ValueError("Attached objects not supported with equality constraints")
        if attachedObjectRelTransforms is not None:
            if len(attachedObjects) != len(attachedObjectRelTransforms):
                raise ValueError("attachedObjects and attachedObjectRelTransforms must have the same length")
        space = robotcspace.RobotCSpaceWithObject(robot, collider)
        for i,(obj,link) in enumerate(attachedObjects):
            if attachedObjectRelTransforms is not None:
                relT = attachedObjectRelTransforms[i]
            else:
                relT = None
            space.attachObject(obj,link,relT)
    else:
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

def plan_to_config(world : WorldModel, robot : RobotModel, target : Config,
                 edgeCheckResolution : float = 1e-2,
                 extraConstraints : List[Callable]= [],
                 equalityConstraints : List[Union[Callable,IKObjective]]=[],
                 equalityTolerance : float = 1e-3,
                 ignoreCollisions=[],
                 movingSubset : Optional[Union[str,List[int]]] = 'auto',
                 attachedObjects : Optional[List[Tuple[RigidObjectModel,RobotModelLink]]]=None,
                 attachedObjectRelTransforms : Optional[List[RigidTransform]]=None,
                 verbose=True,
                 **planOptions) -> KinematicPlanner:
    """Creates a KinematicPlanner object that can be called to solve a standard motion
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
        attachedObjects (list of (RigidObjectModel,RobotModelLink), optional): a
            list of rigid objects and the links to which they are attached. 
            This is used to model grasped objects.
        attachedObjectRelTransforms (list of RigidTransform, optional): if
            any objects are attached, the relative transform from the object to
            the target link.  If not provided, the current transforms are used
            to determine the relative transforms.
        planOptions (keywords): keyword options that will be sent to the
            planner.  See the documentation for KinematicPlanner.setOptions for more
            details.
    
    Returns: 
        KinematicPlanner: a planner instance that can be called to get a
        kinematically-feasible plan. (see :meth:`KinematicPlanner.planMore`)

        The underlying configuration space (a RobotCSpace, ClosedLoopRobotCSpace, or
        EmbeddedRobotCSpace) can be retrieved using the "space" attribute of the
        resulting KinematicPlanner object.
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
                      movingSubset=subset,
                      attachedObjects=attachedObjects,
                      attachedObjectRelTransforms=attachedObjectRelTransforms)

    if hasattr(space,'lift'):  #the planning takes place in a space of lower dimension than #links
        plan = EmbeddedKinematicPlanner(space,q0,**planOptions)
    else:
        plan = KinematicPlanner(space,**planOptions)
    try:
        plan.setEndpoints(q0,target)
    except RuntimeError:
        #one of the endpoints is infeasible, print it out
        if space.cspace==None: space.setup()
        q0_raw = q0 if not hasattr(plan.space,'project') else plan.space.project(q0)
        target_raw = target if not hasattr(plan.space,'project') else plan.space.project(target)
        sfailures = space.cspace.feasibilityFailures(q0_raw)
        gfailures = space.cspace.feasibilityFailures(target_raw)
        if sfailures:
            warnings.warn("Start configuration fails {}".format(sfailures))
            if verbose and 'self collision' in sfailures:
                robot.setConfig(q0)
                for i in range(robot.numLinks()):
                    for j in range(i):
                        if robot.selfCollisionEnabled(i,j):
                            if robot.link(i).geometry().collides(robot.link(j).geometry()):
                                print("  Links {} and {} collide".format(robot.link(i).getName(),robot.link(j).getName()))
        if gfailures:
            warnings.warn("Goal configuration fails {}".format(gfailures))
            if verbose and 'self collision' in gfailures:
                robot.setConfig(target)
                for i in range(robot.numLinks()):
                    for j in range(i):
                        if robot.selfCollisionEnabled(i,j):
                            if robot.link(i).geometry().collides(robot.link(j).geometry()):
                                print("  Links {} and {} collide".format(robot.link(i).getName(),robot.link(j).getName()))
        raise
    return plan


def plan_to_set(world : WorldModel, robot : RobotModel, target : Union[Callable,CSpace],
              edgeCheckResolution : float =1e-2,
              extraConstraints : List[Callable]= [],
              equalityConstraints : List[Union[Callable,IKObjective]]=[],
              equalityTolerance : float = 1e-3,
              ignoreCollisions=[],
              movingSubset : Optional[Union[str,List[int]]] = None,
              attachedObjects : Optional[List[Tuple[RigidObjectModel,RobotModelLink]]]=None,
              attachedObjectRelTransforms : Optional[List[RigidTransform]]=None,
              **planOptions) -> KinematicPlanner:
    """
    Creates a KinematicPlanner object that can be called to solve a standard motion
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
        attachedObjects (list of (RigidObjectModel,RobotModelLink), optional): a
            list of rigid objects and the links to which they are attached. 
            This is used to model grasped objects.
        attachedObjectRelTransforms (list of RigidTransform, optional): if
            any objects are attached, the relative transform from the object to
            the target link.  If not provided, the current transforms are used
            to determine the relative transforms.
        planOptions (keywords): keyword options that will be sent to the planner.  See
            the documentation for KinematicPlanner.setOptions for more details.
    
    Returns: 
        KinematicPlanner: a planner instance that can be called to get a
        kinematically-feasible plan. (see :meth:`KinematicPlanner.planMore` )

        The underlying configuration space (a RobotCSpace, ClosedLoopRobotCSpace, or
        EmbeddedRobotCSpace) can be retrieved using the "space" attribute of the
        resulting KinematicPlanner object.
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
                      movingSubset=subset,
                      attachedObjects=attachedObjects,
                      attachedObjectRelTransforms=attachedObjectRelTransforms)

    if hasattr(space,'lift'):  #the planning takes place in a space of lower dimension than #links
        plan = EmbeddedKinematicPlanner(space,q0,**planOptions)
    else:
        plan = KinematicPlanner(space,**planOptions)

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


def plan_to_cartesian_objective(world : WorldModel, robot : RobotModel, iktargets : List[IKObjective], iktolerance : float = 1e-3,
                             edgeCheckResolution : float = 1e-2,
                             extraConstraints : List[Callable]= [],
                             equalityConstraints : List[Union[Callable,IKObjective]]=[],
                             equalityTolerance : float = 1e-3,
                             ignoreCollisions=[],
                             movingSubset : Optional[Union[str,List[int]]] = None,
                             **planOptions) -> KinematicPlanner:
    """
    Creates a KinematicPlanner object that will try to reach one or
    more IK targets. 

    Args:
        world (WorldModel): same as plan_to_config
        iktargets (list of :class:`IKObjective`): a list of IKObjective
            instances (see the ik module) that should be satisfied at the end
            of the motion.
        iktolerance (float): a tolerance to which the ik objectives must be
            satisfied

    Returns: 
        KinematicPlanner: a planner instance that can be called to get a
        kinematically-feasible plan. (see :meth:`KinematicPlanner.planMore` )

        The underlying configuration space (a RobotCSpace, ClosedLoopRobotCSpace, or
        EmbeddedRobotCSpace) can be retrieved using the "space" attribute of the
        resulting KinematicPlanner object.
    """
    #TODO: only subselect those links that are affected by the IK target
    goalset = robotcspace.ClosedLoopRobotCSpace(robot,iktargets,None)
    goalset.tol = iktolerance
    return plan_to_set(world,robot,goalset,
                     edgeCheckResolution=edgeCheckResolution,
                     extraConstraints=extraConstraints,
                     equalityConstraints=equalityConstraints,
                     equalityTolerance=equalityTolerance,
                     ignoreCollisions=ignoreCollisions,
                     movingSubset=movingSubset,
                     **planOptions)


def run_plan(plan : KinematicPlanner,
            maxIters : int = 100000,
            maxTime : float = 10.0,
            endpoints : Tuple = None,
            maxItersToOptimize : int = None,
            maxTimeToOptimize : float = None,
            verbose : int=1,
            checkFrequency:int = 10)  -> Union[None,List[Config]]:
    """A default runner for a planner that works generally in a sane manner for 
    most defaults. Allows debugging by setting verbose >= 1.

    Args:
        plan (KinematicPlanner): a planner object at least partially configured.
        maxIters (int): the maximum number of iterations to run.
        maxTime (float): the maximum seconds to run.
        maxItersToOptimize (int): the maximum number of iterations to run after
            a first feasible path is found.
        maxTimeToOptimize (float): the maximum seconds to run after a first
            feasible path is found.
        endpoints (None or pair): the endpoints of the plan, either Configs
            or goal specifications. If None uses the endpoints configured in
            plan.
        verbose (int): whether to print information about the planning.
        checkFrequency (int): checks termination criteria every checkFrequency
            planner iterations.

    Returns:
        path or None: if successful, returns a feasible path solving the 
        terminal conditions specified in the plan or between the specified
        endpoints.
    """
    if maxItersToOptimize is None:
        maxItersToOptimize = maxIters
    if maxTimeToOptimize is None:
        maxTimeToOptimize = maxTime
    if endpoints is not None:
        if len(endpoints) != 2:
            raise ValueError("Need exactly two endpoints")
        try:
            plan.setEndpoints(*endpoints)
        except RuntimeError:
            #must be invalid configuration
            if verbose:
                if isinstance(endpoints[0],(list,tuple)) and isinstance(endpoints[0][0],(float,int)):
                    print("run_plan: Start configuration fails:",plan.space.cspace.feasibilityFailures(endpoints[0]))
                if isinstance(endpoints[1],(list,tuple)) and isinstance(endpoints[1][0],(float,int)):
                    print("run_plan: Goal configuration fails:",plan.space.cspace.feasibilityFailures(endpoints[1]))
            return None
    
    #this is helpful to slightly speed up collision queries
    assert plan.space.cspace is not None, "plan.space.setup() must be called before running the planner"
    plan.space.cspace.enableAdaptiveQueries(True)

    t0 = time.time()

    #begin planning
    numIters = 0
    feasIters = None
    feasTime = None
    for round in range(maxIters//checkFrequency):
        if verbose:
            print("run_plan: {}/{} iterations".format(numIters,maxIters))
        plan.planMore(min(maxIters-numIters,checkFrequency))
        numIters += checkFrequency

        if not plan.isOptimizing():  #break on first path found
            path = plan.getPath()
            if path is not None and len(path)>0:
                break
        elif feasIters is None:
            path = plan.getPath()
            if path is not None and len(path)>0:
                feasIters = numIters
                feasTime = time.time()
                if len(path) == 2:
                    #straight line path, so no need to optimize
                    break
                if verbose:
                    print("  Found first feasible path with {} milestones, cost {} at time {}".format(len(path),plan.pathCost(path),feasTime))
        else: #optimizing
            if numIters - feasIters >= maxItersToOptimize or time.time()-feasTime > maxTimeToOptimize:
                print("  Done optimizing after {} iterations, time {}".format(numIters-feasIters,time.time()-feasTime))
                break
        if time.time()-t0 > maxTime:
            break
    if verbose:
        print("run_plan: planning took time {}s over {} iterations".format(time.time()-t0,numIters))
        
    #this code just gives some debugging information. it may get expensive
    if verbose >= 2:
        V,E = plan.getRoadmap()
        print(len(V),"feasible milestones sampled,",len(E),"edges connected")
    
    if verbose >= 2:
        print("run_plan: stats:")
        print(plan.getStats())

    path = plan.getPath()
    if path is None or len(path)==0:
        if verbose:
            print("run_plan: Failed to find feasible path")
            stats = plan.space.getStats()
            limiting_constraint = min([(float(p),k) for k,p in stats.items() if k.endswith('probability') and k != 'visible_probability'],key=lambda x: x[0],default=(1,'none'))
            if limiting_constraint[0] < 1:
                print("   Least likely constraint to be satisfied: {} at probability {}".format(limiting_constraint[1],limiting_constraint[0]))
            else:
                print("   All constraints satisfied with probability 1.0")
    
            print("run_plan: stats:")
            print(plan.getStats())

            #debug some sampled configurations
            if verbose >= 2:
                print("  CSpace stats:")
                print(plan.space.getStats())
            
                print("Some sampled configurations:")
                print(V[0:min(10,len(V))])
        return None
    else:
        if verbose:
            print("run_plan: path has {} milestones, length {}, cost {}".format(len(path), plan.pathLength(path), plan.pathCost(path)))

    return path
