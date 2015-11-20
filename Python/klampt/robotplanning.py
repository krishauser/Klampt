import cspace
import robotcspace
import robotcollide
from klampt import IKObjective

def preferredPlanOptions(robot,movingSubset=None,optimizing=False):
    """Returns some options that might be good for your given robot, and
    whether you want a feasible or just an optimal plan.

    TODO: base this off of info about the robot, such as dimensionality,
    joint ranges, etc.
    """
    if optimizing:
        return { 'type':"rrt", 'perturbationRadius':0.5, 'bidirectional':1, 'shortcut':1, 'restart':1, 'restartTermCond':"{foundSolution:1,maxIters:1000}" }
    else:
        return { 'type':"sbl", 'perturbationRadius':0.5, 'randomizeFrequency':1000, shortcut:1 }

def makeSpace(world,robot,
              edgeCheckResolution=1e-2,
              extraConstraints=[],
              equalityConstraints=[],
              equalityTolerance=1e-3,
              ignoreCollisions=[],
              movingSubset=None):
    """Arguments:
    - world: a WorldModel instance
    - robot: a RobotModel in the world
    - edgeCheckResolution: the resolution at which edges in the path are
      checked for feasibility
    - extraConstraints: a list of possible extra constraint functions, each
      of which needs to return True if satisfied.  Note: don't put cartesian
      constraints here! Instead place your function in equalityConstraints.
    - equalityConstraints: a list of IKObjectives or equality
      constraints f(x)=0 that must be satisfied during the motion. Equality
      constraints may return a float or a list of floats, in which case all
      entries of the vector must be 0.
    - equalityTolerance: a tolerance to which all the equality constraints
      must be satisfied.
    - ignoreCollisions: a list of ignored collisions. Each element may be
      a body in the world, or a pair (a,b) where a, b are bodies in the world.
    - movingSubset: if None, 'all', or 'auto' (default), all joints
      will be allowed to move.  If this is a list, then only these joint
      indices will be allowed to move.
    Output: a CSpace instance 
    """
    subset = []
    if movingSubset == 'auto' or movingSubset == 'all' or movingSubset == None:
        subset = None
    else:
        subset = movingSubset
        
    collider = WorldCollider(world,ignore=ignoreCollisions)

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
    space.eps = edgeCheckResolution
        
    if subset is not None and len(subset) <= robot.numLinks():
        #choose a subset
        space = EmbeddedCSpace(space,xinit=robot.getConfig())
        inactive = []
        for i in range(robot.numLinks()):
            if i not in subset: inactive.append(i)
        #disable self-collisions for inactive objects
        for i in inactive:
            rindex = space.collider.robots[robot.index][i]
            space.collider.mask[rindex] = set()
    
    for c in extraConstraints:
        space.addConstraint(c)
    return space

def planToConfig(world,robot,target,
                 edgeCheckResolution=1e-2,
                 extraConstraints=[],
                 equalityConstraints=[],
                 equalityTolerance=1e-3,
                 ignoreCollisions=[],
                 movingSubset='auto',
                 verbose=True,
                 **planOptions):
    """Arguments:
    - world: a WorldModel instance
    - robot: a RobotModel in the world
    - target: the desired final configuration of the robot
    - edgeCheckResolution: the resolution at which edges in the path are
      checked for feasibility
    - extraConstraints: a list of possible extra constraint functions, each
      of which needs to return True if satisfied.  Note: don't put cartesian
      constraints here! Instead place your function in equalityConstraints.
    - equalityConstraints: a list of IKObjectives or equality
      constraints f(x)=0 that must be satisfied during the motion. Equality
      constraints may return a float or a list of floats, in which case all
      entries of the vector must be 0.
    - equalityTolerance: a tolerance to which all the equality constraints
      must be satisfied.
    - ignoreCollisions: a list of ignored collisions. Each element may be
      a body in the world, or a pair (a,b) where a, b are bodies in the world.
    - movingSubset: if 'auto' (default), only the links that are
      different between the robot's current config and target config will
      be allowed to move.  Otherwise, if this is None or 'all', all joints
      will be allowed to move.  If this is a list, then only these joint
      indices will be allowed to move.
    - planOptions: keyword options that will be sent to the planner.
    Output: a cspace.MotionPlan instance that can be called to get a
      kinematically-feasible plan. (see cspace.MotionPlan.planMore(iterations))
    """
    q0 = robot.getConfig()
    assert(len(q0)==len(target)),"target configuration must be of correct size for robot"
    subset = []
    if movingSubset == 'auto':
        subset = []
        for i,(a,b) in enumerate(zip(q0,target)):
            if a != b:
                subset.append(i)
    elif movingSubset == 'all' or movingSubset == None:
        subset = range(len(q0))
    else:
        for i in range(len(q0)):
            if i not in subset:
                if q0[i] != target[i]:
                    raise ValueError("Error: target configuration value differs from start configuration along a fixed DOF")
        subset = movingSubset
    
    space = makeSpace(world=world,robot=robot,
                      edgeCheckResolution=edgeCheckResolution,
                      extraConstraints=extraConstraints,
                      equalityConstraints=equalityConstraints,
                      equalityTolerance=equalityTolerance,
                      ignoreCollisions=ignoreCollisions,
                      movingSubset=subset)
    
    plan = MotionPlan(space,**planOptions)
    plan.setEndpoints([q0[s] for s in subset],
                      [target[s] for s in subset])
    return plan


def planToSet(world,robot,target,
              extraConstraints=[],
              equalityConstraints=[],
              equalityTolerance=1e-3,
              ignoreCollisions=[],
              movingSubset=None,
              **planOptions):
    """Arguments:
    - world: a WorldModel instance
    - robot: a RobotModel in the world
    - target: a function testing whether the given configuration is a goal
    - extraConstraints: a list of possible extra constraint functions, each
      of which needs to return True if satisfied.
    - equalityConstraints: a list of IKObjectives or equality
      constraints f(x)=0 that must be satisfied during the motion. Equality
      constraints may return a float or a list of floats, in which case all
      entries of the vector must be 0.
    - equalityTolerance: a tolerance to which all the equality constraints
      must be satisfied.
    - ignoreCollisions: a list of collision pairs (a,b) where a, b are
      bodies in the world.
    - movingSubset: if 'auto', 'all', or None (default), all joints
      will be allowed to move.  If this is a list, then only these joint
      indices will be allowed to move.
    - planOptions: keywords to be sent to the planner.
    Output: a cspace.MotionPlan instance that can be called to get a
      kinematically-feasible plan. (see cspace.MotionPlan.planMore(iterations))
    """
    q0 = robot.getConfig()
    subset = []
    if movingSubset == 'auto' or movingSubset == 'all' or movingSubset == None:
        subset = range(len(q0))
    else:
        subset = movingSubset

    space = makeSpace(world=world,robot=robot,
                      edgeCheckResolution=edgeCheckResolution,
                      extraConstraints=extraConstraints,
                      equalityConstraints=equalityConstraints,
                      equalityTolerance=equalityTolerance,
                      ignoreCollisions=ignoreCollisions,
                      movingSubset=subset)

    plan = MotionPlan(space,**planOptions)
    plan.setEndpoints([q0[s] for s in subset],target)
    return plan

def planToCartesianObjective(world,robot,iktargets,iktolerance=1e-3,
                             extraConstraints=[],
                             equalityConstraints=[],
                             equalityTolerance=1e-3,
                             ignoreCollisions=[],
                             movingSubset=None,
                             **planOptions):
    """Arguments: all the same as planToConfig, except
    - iktargets: a list of IKObjective instances (see the ik module)
    - iktolerance: a tolerance to which the ik objectives must be satisfied
    Output: a cspace.MotionPlan instance that can be called to get a
      kinematically-feasible plan. (see cspace.MotionPlan.planMore(iterations))
    """
    #TODO: only subselect those links that are affected by the IK target
    goalset = robotcspace.ClosedLoopRobotCSpace(robot,iktargets,None)
    return planToSet(world,robot,goalset,
                     extraConstraints=extraConstraints,
                     equalityConstraints=equalityConstraints,
                     equalityTolerance=equalityTolerance,
                     ignoreCollisions=ignoreCollisions,
                     movingSubset=movingSubset,
                     **planOptions)
