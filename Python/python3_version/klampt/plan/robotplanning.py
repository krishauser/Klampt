from .cspace import *
from .cspaceutils import *
from . import robotcspace
from ..model import collide
from ..robotsim import IKObjective

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

class SubsetMotionPlan (MotionPlan):
    """An adaptor that "lifts" a motion planner in an EmbeddedCSpace to a higher dimensional ambient
    space.  Used for planning in subsets of robot DOFs."""
    def __init__(self,space,subset,q0,type=None,**options):
        MotionPlan.__init__(self,space,type,**options)
        self.subset = subset
        self.q0 = q0
    def getPath(self,milestone1=None,milestone2=None):
        spath = MotionPlan.getPath(self,milestone1,milestone2)
        if spath == None: return None
        path = []
        for sq in spath:
            assert len(sq)==len(self.subset),"Subset must be same size of space dimensionality"
            q = self.q0[:]
            for s,v in zip(self.subset,sq):
                q[s] = v
            path.append(q)
        return path


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

    if subset is not None and len(subset) < robot.numLinks():
        #choose a subset
        sspace = EmbeddedCSpace(space,subset,xinit=robot.getConfig())
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
            rindices = space.collider.robots[robot.index]
            rindex = rindices[i]
            if rindex < 0:
                continue
            newmask = set()
            for j in range(robot.numLinks()):
                if rindices[j] in space.collider.mask[rindex] and active[j]:
                    newmask.add(rindices[j])
            space.collider.mask[rindex] = newmask
        space = sspace
    
    space.setup()
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
    - planOptions: keyword options that will be sent to the planner.  See
      the documentation for MotionPlan.setOptions for more details.
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
        subset = list(range(len(q0)))
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
    
    plan = SubsetMotionPlan(space,subset,q0,**planOptions)
    try:
        plan.setEndpoints([q0[s] for s in subset],
                          [target[s] for s in subset])
    except RuntimeError:
        #one of the endpoints is infeasible, print it out
        if space.cspace==None: space.setup()
        sfailures = space.cspace.feasibilityFailures([q0[s] for s in subset])
        gfailures = space.cspace.feasibilityFailures([target[s] for s in subset])
        print("Start configuration fails",sfailures)
        print("Goal configuration fails",gfailures)
        return None
    return plan


def planToSet(world,robot,target,
              edgeCheckResolution=1e-2,
              extraConstraints=[],
              equalityConstraints=[],
              equalityTolerance=1e-3,
              ignoreCollisions=[],
              movingSubset=None,
              **planOptions):
    """Arguments:
    - world: a WorldModel instance
    - robot: a RobotModel in the world
    - target: a function testing whether the given RobotModel configuration is
      a goal, OR an instance of a CSpace subclass where sample() generates a
      sample in the target set and feasible(x) tests whether a sample is in the
      target set. (The CSpace should be of the same dimensionality as the robot,
      not the moving subset.)
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
        subset = list(range(len(q0)))
    else:
        subset = movingSubset

    space = makeSpace(world=world,robot=robot,
                      edgeCheckResolution=edgeCheckResolution,
                      extraConstraints=extraConstraints,
                      equalityConstraints=equalityConstraints,
                      equalityTolerance=equalityTolerance,
                      ignoreCollisions=ignoreCollisions,
                      movingSubset=subset)

    plan = SubsetMotionPlan(space,subset,q0,**planOptions)

    if isinstance(target,CSpace):
      if isinstance(target,EmbeddedCSpace):
        def goaltest(x):
          return target.feasible(space.lift(x))
        def goalsample():
          qrobot = target.sample()
          qproj = space.project(qrobot)
          return qproj
        goal = [goaltest,goalsample]
      else:
        goal = [(lambda x:target.feasible(x)),(lambda : target.sample())]
    else:
      goal = target
    try:
        print("a",target)
        plan.setEndpoints([q0[s] for s in subset],goal)
    except RuntimeError:
        #the start configuration is infeasible, print it out
        if space.cspace==None: space.setup()
        sfailures = space.cspace.feasibilityFailures([q0[s] for s in subset])
        print("Start configuration fails",sfailures)
        raise
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
