"""This module provides convenient access to the motionplanning module
functionality by defining the :class:`CSpace` and :class:`MotionPlan`
classes.
"""

from . import motionplanning
import random
import warnings

class CSpace:
    """Used alongside :class:`MotionPlan` to define a configuration space for
    motion planning.

    Attributes:
        eps (float): the collision tolerance used for checking edges, in the units
            defined by the distance(a,b) method.  (by default, euclidean distance)
        bound (float): a list of lower and upper bounds on the space [(l1,u1),...,(ld,ud)]
            where d is the dimension of the configuration space.
        properties (dict): a map of properties that may be used by a planner.  See
            below documentation for more detail. 
        cspace (CSpaceInterface, internal): a motionplanning module CSpaceInterface object
        feasibilityTests (list of functions, internal): a list of one-argument functions that test
            feasibility of this configuration space's constraints.  E.g.,
            if you have a collision tester that returns True if a configuration is in
            collision, and also want to check bounds, you can set this to a list:
            [self.in_bounds,lambda x not self.collides(x)]
            You should not write this directly but instead use addFeasibilityTest.
        feasibilityTestNames (list of strs, internal): a list of names of feasibility
            tests.  You should not write this directly but instead use addFeasibilityTest.

    To define a custom CSpace, subclasses may optionally override:
        
    - feasible(x): returns true if the vector x is in the feasible space.  By
      default calls each function in self.feasibilityTests, which by default
      only tests bound constraints.
    - sample(): returns a new vector x from a superset of the feasible space
    - sampleneighborhood(c,r) (optional): returns a new vector x from a neighborhood of c
      with radius r
    - visible(a,b): returns true if the path between a and b is feasible
    - distance(a,b): return a distance between a and b
    - interpolate(a,b,u): interpolate between a, b with parameter u

    To avoid memory leaks, CSpace.close() or motionplanning.destroy() must
    be called when you are done.  (The latter deallocates all previously
    created cspaces and planners)

    If sample() is not defined, then subclasses should set self.bound to be a
    list of pairs defining an axis-aligned bounding box.  The setBounds method
    is a convenient way of defining this.

    If visible() is not defined, then paths are checked by subdivision, with
    the collision tolerance self.eps.

    To help planners know a bit more about the CSpace, you can set the
    self.properties member to a map from strings to values.  Useful values
    are
    
    - euclidean (0 or 1): indicates a euclidean space
    - geodesic (0 or 1): indicates whether the interpolation is along
      geodesics.
    - volume (real): a size of the space
    - minimum, maximum (real array): bounds on the space
    - metric (string): the metric used by distance, can be "euclidean",
      "weighted euclidean", "manhattan", "weighted manhattan", "Linf", etc.
    - metricWeights (real array): weights used by weighted metrics
    
    volume is necessary for Lazy PRM* and Lazy RRG* to work.
    
    metricWeights is necessary for KD-tree point location structures to work,
    for FMM methods to work, etc.

    minimum/maximum can be used by grid-based methods (optional for FMM, FMM*).
    """
    def __init__(self):
        self.cspace = None
        self.feasibilityTests = None
        self.feasibilityTestNames = None
        self.feasibilityTestDependencies = None
        self.eps = 1e-3
        self.bound = [(0,1)]
        self.properties = {}

    def setBounds(self,bound):
        """Convenience function: sets the sampling bound and the
        space properties in one line."""
        self.bound = bound
        self.properties["minimum"] = [b[0] for b in bound]
        self.properties["maximum"] = [b[1] for b in bound]
        volume = 1
        for b in self.bound:
            if b[0] != b[1]: volume *= b[1]-b[0]
        self.properties['volume'] = volume

    def close(self):
        """This method must be called to free the memory associated with the
        planner.  Alternatively, motionplanning.destroy() can be called to
        free all previously constructed CSpace and MotionPlan objects."""
        if self.cspace is not None:
            self.cspace.destroy()
            self.cspace = None
    
    def setup(self,reinit = False):
        """Called internally by the MotionPlan class to set up planning
        hooks. 

        If reinit is not set to True, and the setup() method has been called before,
        a warning message will be printed.  Set it to True to suppress this message."""
        if self.cspace is not None:
            if not reinit:
                warnings.warn("CSpace.setup(): Performance warning, called twice, destroying previous CSpaceInterface object")
            self.cspace.destroy()
        self.cspace = motionplanning.CSpaceInterface()
        if self.feasibilityTests is not None:
            for n,f in zip(self.feasibilityTestNames,self.feasibilityTests):
                self.cspace.addFeasibilityTest(n,f)
            self.cspace.enableAdaptiveQueries()
            for (n,d) in self.feasibilityTestDependencies:
                self.cspace.setFeasibilityDependency(n,d)
        else:
            if hasattr(self,'feasible'):
                self.cspace.setFeasibility(getattr(self,'feasible'))
            else:
                raise RuntimeError('CSpace needs feasible(q) method to be defined, or addFeasibilityTests to be called')
        if hasattr(self,'visible'):
            self.cspace.setVisibility(getattr(self,'visible'))
        else:
            self.cspace.setVisibilityEpsilon(self.eps)
        self.cspace.setSampler(self.sample)
        self.cspace.setNeighborhoodSampler(self.sampleneighborhood)
        if hasattr(self,'distance'):
            self.cspace.setDistance(getattr(self,'distance'))
        if hasattr(self,'interpolate'):
            self.cspace.setInterpolate(getattr(self,'interpolate'))
        for (k,v) in self.properties.items():
            if isinstance(v,(list,tuple)):
                self.cspace.setProperty(k," ".join([str(item) for item in v]))
            else:
                self.cspace.setProperty(k,str(v))

    def sample(self):
        """Overload this to define a nonuniform sampler.
        By default, it will sample from the axis-aligned bounding box
        defined by self.bound. To define a different domain, set self.bound
        to the desired bound.
        """
        return [random.uniform(*b) for b in self.bound]

    def sampleneighborhood(self,c,r):
        """Overload this to define a nonuniform sampler.
        By default, it will sample from the axis-aligned box of radius r
        around c, but clamped to the bound.
        """
        return [random.uniform(max(b[0],ci-r),min(b[1],ci+r)) for ci,b in zip(c,self.bound)]

    def addFeasibilityTest(self,func,name=None,dependencies=None):
        """Adds a new feasibility test with the given function func(x) and the specified name.
        If name is not provided (default) a default name is generated.

        If dependencies is provided, it can be a string or a list of strings, 
        indicating that this test must be called after some other test(s).
        """
        if self.feasibilityTests is None:
            self.feasibilityTests = []
            self.feasibilityTestNames = []
            self.feasibilityTestDependencies = []
        assert name is None or isinstance(name,str),"Name argument 'name' must be a string"
        assert callable(func),"Feasibility test 'func' must be a callable object"
        self.feasibilityTests.append(func)
        if name is None:
            name = "test_"+str(len(self.feasibilityTests)-1)
        self.feasibilityTestNames.append(name)
        if dependencies is not None:
            if isinstance(dependencies,(list,tuple)):
                for d in dependencies:
                    self.feasibilityTestDependencies.append((name,d))
            else:
                self.feasibilityTestDependencies.append((name,dependencies))

    def inBounds(self,x):
        """Returns true if x is within the given bounds"""
        return all(a<=xi<=b for (xi,(a,b)) in zip(x,self.bound))

    def feasible(self,x):
        """Overload this to define your new feasibility test.
        By default the implementation simply tests the bounds constraint, or if self.feasibilityTests
        is not empty, tests each function in self.feasibilityTests."""
        if self.feasibilityTests is None:
            return self.inBounds(x)
        else:
            for test in self.feasibilityTests:
                if not test(x): return False
            return True

    def isFeasible(self,x):
        """An overload for self.cspace.isFeasible.  Use this to test feasibility of a configuration
        (rather than feasible()) if you wish to take advantage of adaptive feasibility testing and
        constraint testing statistics."""
        return self.cspace.isFeasible(x)

    def isVisible(self,x,y):
        """An overload for self.cspace.isVisible.  Use this to test visibility of a line
        (rather than visible()) if you want to use the natural visibility tester, wish to take
        advantage of adaptive visibility testing, or want to use constraint testing statistics."""
        return self.cspace.isVisible(x,y)

    def getStats(self):
        """Returns a dictionary mapping statistic names to values.  Result contains 
        fraction of feasible configurations, edges, etc.  If feasibility tests are
        individually specified, returns stats for individual tests as well. """
        if self.cspace is None: return {}
        return self.cspace.getStats()


class MotionPlan:
    """A motion planner instantiated on a space.  Currently supports
    only kinematic, point-to-point, or point-to-set plans.

    Planner parameters must be set by calling the static
    MotionPlan.setOptions(param1=value1,param2=value2,...) method BEFORE
    calling the MotionPlan(space,type) constructor.

    If type is not specified in the constructor, the planning algorithm
    will be chosen by default.
    
    Note that MotionPlan.close() or motionplanning.destroy() must be called
    to free memory after you are done.

    Multi-query roadmaps are supported for the PRM and SBLPRT algorithms.
    In multi-query mode, you may call addMilestone(q) to add a new milestone.
    addMilestone() returns the milestone's index, which can be used
    in later calls to getPath().

    Cost functions are supported by any restart, shortcutting, or goal set
    planners, as well as PRM, RRT. RRT*, PRM*, Lazy RRG*, Lazy PRM*. However,
    be aware that the X* planners are internally trying to optimize path 
    length, and the result may not be asymptotically optimal for other cost
    functions.
    """
    def __init__(self,space,type=None,**options):
        """Initializes a plan with a given CSpace and a given type.
        Optionally, planner options can be set via keyword arguments.
        
        Valid values for type are:

            * 'prm': the Probabilistic Roadmap algorithm
            * 'rrt': the Rapidly Exploring Random Trees algorithm
            * 'sbl': the Single-Query Bidirectional Lazy planner
            * 'sblprt': the probabilistic roadmap of trees (PRT) algorithm with SBL as the inter-root planner.
            * 'rrt*': the RRT* algorithm for optimal motion planning 
            * 'prm*': the PRM* algorithm for optimal motion planning
            * 'lazyprm*': the Lazy-PRM* algorithm for optimal motion planning
            * 'lazyrrg*': the Lazy-RRG* algorithm for optimal motion planning
            * 'fmm': the fast marching method algorithm for resolution-complete optimal motion planning
            * 'fmm*': an anytime fast marching method algorithm for optimal motion planning

        (this list may be out-of-date; the most current documentation
        is listed in src/motionplanning.h)
        """
        if space.cspace is None:
            space.setup()
        if type != None:
            motionplanning.set_plan_type(type)
        if len(options) > 0:
            MotionPlan.setOptions(**options)
        self.space = space
        self.planOptions = motionplanning.get_plan_json_string()
        self.planner = motionplanning.PlannerInterface(space.cspace)
        self.edgeCost=None
        self.terminalCost=None

    def close(self):
        """This method must be called to free the memory associated with the
        planner.  Alternatively, motionplanning.destroy() can be called to
        free all previously constructed CSpace and MotionPlan objects."""
        self.planner.destroy()

    @staticmethod
    def setOptions(**opts):
        """Sets a numeric or string-valued setting for the next constructed
        planner.  Arguments are specified by key-value pair.
        
        Valid keys are:

            * "knn": k value for the k-nearest neighbor connection strategy
              (used in PRM)
            * "connectionThreshold": a milestone connection threshold
            * "perturbationRadius": (for RRT and SBL)
            * "bidirectional": 1 if bidirectional planning is requested (used
              in RRT)
            * "grid": 1 if a point selection grid should be used (used in SBL)
            * "gridResolution": resolution for the grid, if the grid should
              be used (used in SBL with grid, FMM, FMM*)
            * "suboptimalityFactor": allowable suboptimality (used in RRT*,
              lazy PRM*, lazy RRG*)
            * "randomizeFrequency": a grid randomization frequency (for SBL)
            * "shortcut": nonzero if you wish to perform shortcutting to
              improve a path after a first plan is found.
            * "restart": nonzero if you wish to restart the planner to
              get progressively better paths with the remaining time.
            * "pointLocation": a string designating a point location data
              structure. "kdtree" is supported, optionally followed by a
              weight vector (used in PRM, RRT, RRT*, PRM*, LazyPRM*, LazyRRG*)
            * "restartTermCond": used if the "restart" setting is true.
              This is a JSON string defining the termination condition
              (default value: "{foundSolution:1;maxIters:1000}")

        (this list may be out-of-date; the most current documentation
        is listed in Klampt/Python/klampt/src/motionplanning.h)

        An exception may be thrown if an invalid setting is chosen.
        """
        for (a,b) in list(opts.items()):
            if a=='type':
                motionplanning.set_plan_type(str(b))
            elif isinstance(b,str):
                motionplanning.set_plan_setting(a,b)
            else:
                motionplanning.set_plan_setting(a,float(b))

    def setEndpoints(self,start,goal):
        """Sets the start and goal configuration or goal condition. 

        Args:
            start (list of floats): start configuration
            goal (list of floats, function, or pair of functions): defines the
                goal condition.  Can be:

                - a configuration: performs point-to-point planning.
                - a function: the goal is a set, and goal is a 1-argument
                  function f(q) that returns true if the configuration q is at
                  in the goal set and false otherwise. 
                - a pair of functions (f,s): f(q) tests whether the
                  configuration is at the goal, and s() generates a new
                  sample in the goal set.

        """
        if hasattr(goal,'__call__'):
            self.planner.setEndpointSet(start,goal)
        elif(len(goal) == 2 and hasattr(goal[0],'__call__')):
            self.planner.setEndpointSet(start,goal[0],goal[1])
        else:
            self.planner.setEndpoints(start,goal)

    def setCostFunction(self,edgeCost=None,terminalCost=None):
        """Sets a cost function to be used when retrieving a solution
        path.  Some planners cannot accept objective functions.

        The total cost of a path x0,x1,...,xn is:

            edgeCost(x0,x1) + edgeCost(x1,x2) + ... + edgeCost(xn-1,xn) + terminalCost(xn)

        Args:
            edgeCost (function, optional): has signature f(a,b)->float where 
                a,b are configurations.
            terminalCost (function, optional): has signature f(q)->float where
                q is a configuration.
        """
        if edgeCost is not None and not callable(edgeCost):
            raise TypeError("Need to pass a function into setCostFunction")
        if terminalCost is not None and not callable(terminalCost):
            raise TypeError("Need to pass a function into setCostFunction")
        self.edgeCost = edgeCost
        self.terminalCost = terminalCost
        self.planner.setCostFunction(edgeCost,terminalCost)

    def addMilestone(self,x):
        """Manually adds a milestone at configuration x and returns its index"""
        return self.planner.addMilestone(x)

    def getClosestMilestone(self,x):
        """Returns the index of the closest milestone to configuration x"""
        return self.planner.getClosestMilestone(x)
    
    def planMore(self,iterations):
        """Performs a given number of iterations of planning."""
        self.planner.planMore(iterations)

    def getPath(self,milestone1=None,milestone2=None):
        """Returns the path between the two milestones.  If no
        arguments are provided, this returns the optimal path between
        the start and goal.

        Args:
            milestone1 (int, optional): the start milestone
            milestone2 (int or list of int, optional): the goal milestone.
                If this is a list, the optimal path to some goal milestone
                in this list is returned.

        Returns:
            list of configurations: The result path, given as a list of
            configurations.  Each configuration is a Python list.

            Note that for non-euclidean spaces (e.g., closed loop, SE2,
            or SE3 spaces) the CSpace's interpolate() function must be used
            between milestones to properly interpolate along the path.
        """
        if milestone1 is None and milestone2 is None:
            return self.planner.getSolutionPath();
        else:
            return self.planner.getPath(milestone1,milestone2)

    def getRoadmap(self):
        """Returns a graph (V,E) containing the planner's roadmap.

        V is a list of configurations (each configuration is a list of floats)
        and E is a list of edges (each edge is a pair (i,j) indexing into V).
        """
        return self.planner.getRoadmap()

    def getStats(self):
        """Returns a dictionary mapping statistic names to values.  Result is
        planner-dependent """
        return self.planner.getStats()

    def pathCost(self,path):
        """Helper function to calculate the cost of a path.  If no cost
        function was previously set with setCostFunction, this is just the CSpace
        length.
        """
        c = 0.0 if self.terminalCost is None else self.terminalCost(path[-1])
        if self.edgeCost is None:
            c += sum(self.space.cspace.distance(a,b) for (a,b) in zip(path[:-1],path[1:]))
        else:
            c += sum(self.edgeCost(a,b) for (a,b) in zip(path[:-1],path[1:]))
        return c

optimizingPlanners = set(['fmm*','rrt*','prm*','lazyprm*','lazyrrg*'])
"""set: The set of natively optimizing planners. 

Goal set planners, random-restart, and shortcut planners also support optimization
"""

costAcceptingPlanners = set(['prm','rrt','rrt*','prm*','lazyprm*','lazyrrg*'])
"""set: The set of planners that natively accept costs.

Goal set planners, random-restart, and shortcut planners also support costs.
"""

def configurePlanner(*args,**kwargs):
    warnings.warn("configurePlanner will be renamed configure_planner in a future version of Klampt",DeprecationWarning)
    return configure_planner(*args,**kwargs)

def configure_planner(space,start,goal,edgeCost=None,terminalCost=None,optimizing=True,
    type='auto',stepsize=None,knn=10,
    shortcut='auto',restart='auto',restartIters=1000,pointLocation='auto',
    **otherSettings):
    """Automatically sets up a MotionPlan with reasonable options, double
    checking if the options are compatible with the given inputs.

    Args:
        space (CSpace): the space you'd like to plan for.
        start (list of floats): the start configuration
        goal (list of floats or function or (function,function) tuple): the 
            goal configuration or condition.  See
            :meth:`MotionPlan.setEndpoints`.
        edgeCost (function, optional): the edge cost. See
            :meth:`MotionPlan.setCostFunction`.
        terminalCost (function, optional): the terminal cost. See
            :meth:`MotionPlan.setCostFunction`.
        optimizing (bool, optional): whether you expect to be planning past
            the first path found to obtain a better solution.
        type (str, optional): the planner type string.  If 'auto', the planner
            type is set automatically.
        stepsize (float, optional): if given, sets the growth radius or
            grid resolution of the planner.  If not, this is auto-determined
            by the size of the space.
        knn (int, optional): for prm planner, the number of nearest neighbors
            to test.
        shortcut (bool, optional): whether to shortcut the resulting path
        restart (bool, optional): whether to do random-restarts 
        restartIters (int, optional): how many iterations to run between
            restarts
        pointLocation (str, optional): what point location data structure to
            use.  By default, either 'kdtree' or 'balltree' are selected,
            depending on whether you space is assumed Cartesian or not.
        otherSettings (keyword dict, optional): other MotionPlan keywords 
            can be added to override any of the auto-determined settings.

    Returns:
        (MotionPlan,dict): a pair giving the MotionPlan object that can be
        called to produce a plan, and a dictionary giving the relevant
        settings.
    """
    global optimizingPlanners
    global costAcceptingPlanners
    import math

    if type == 'auto':
        type = "sbl"
        if edgeCost is not None:
            type = "prm"
    
    if stepsize is None:
        #how far across the state space to connect / perturb
        from ..math import vectorops
        radius = vectorops.norm(a-b for (a,b) in space.bound if not math.isinf(a) and not math.isinf(b))
        stepsize = 0.1 *radius
    
    if shortcut == 'auto':
        shortcut = optimizing
    if restart == 'auto':
        restart = optimizing

    cartesian = (not hasattr(space,'interpolate') or space.properties.get('euclidean',0) or space.properties.get('cartesian',0))
    infinite = any(math.isinf(a) or math.isinf(b) for (a,b) in space.bound)

    #ball trees are good default settings
    if pointLocation == 'auto':
        if not cartesian:
            pointLocation = 'balltree'
        else:
            pointLocation = 'kdtree'
    #pointLocation = ''
    restartTermCond="{foundSolution:1,maxIters:%d}"%(restartIters,)

    isgoalset = callable(goal) or callable(goal[0])
    optimizingPlanner = (type in optimizingPlanners) or shortcut or restart or isgoalset
    if optimizingPlanner != optimizing:
        warnings.warn("Returned planner is %soptimizing but requested a %soptimizing planner"%(('' if optimizingPlanner else 'not '),('' if optimizing else 'not ')))
    if edgeCost is not None or terminalCost is not None:
        if not shortcut and not restart and type not in costAcceptingPlanners:
            warnings.warn("Planner %s does not accept cost functions"%(type,))
    if isgoalset:
        if type in ['prm*','rrt*','lazyprm*','lazyrrg*']:
            warnings.warn("Planner {} is fairly inefficient when the goal is a set... have not implemented multi-goal versions".format(type))

    args = { 'type':type }
    #PRM planner
    if type == "prm":
        args['knn']=knn
        args['connectionThreshold']=stepsize
        args['pointLocation']=pointLocation
        if edgeCost is not None:
            #this helps with an objective function
            args['ignoreConnectedComponents']=True
    elif type == 'fmm*':
        #FMM* planner
        args['gridResolution']=stepsize
        if not cartesian:
            warnings.warn("Planner {} does not support the topology of non-Cartesian spaces".format(type))
        if infinite:
            raise ValueError("Cannot use planner "+type+" with infinite spaces")
    elif type == 'fmm':
        #FMM planner
        args['gridResolution']=stepsize
        if not cartesian:
            warnings.warn("Planner {} does not support the topology of non-Cartesian spaces".format(type))
        if infinite:
            raise ValueError("Cannot use planner "+type+" with infinite spaces")
    elif type == 'rrt':
        #RRT planner: bidirectional except for goal sets
        args['type'] = 'rrt'
        args['bidirectional'] = True
        args['perturbationRadius']=stepsize
        args['pointLocation']=pointLocation
        args['shortcut']=shortcut
        args['restart']=restart
        #can't do forward growing only yet
        #if callable(goal):
        #    args['bidirectional'] = False
    elif type == 'sbl':
        #SBL planner
        args['perturbationRadius']=stepsize
        args['gridResolution']=stepsize
        args['shortcut']=shortcut
        args['restart']=restart
    elif type in ['rrt*','prm*','lazyprm*','lazyrrg*']:
        #Lazy-RRG* planner
        args['pointLocation'] = pointLocation
    elif type.startswith('ompl:'):
        #OMPL planners:
        #Tested to work fine with OMPL's prm, lazyprm, prm*, lazyprm*, rrt, rrt*, rrtconnect, lazyrrt, lbtrrt, sbl, bitstar.
        #Note that lbtrrt doesn't seem to continue after first iteration.
        #Note that stride, pdst, and fmt do not work properly...
        args['suboptimalityFactor']=0.0
        args['knn']=knn
        args['connectionThreshold']=stepsize
    if 'restart' in args:
        args['restartTermCond']=restartTermCond
    args.update(otherSettings)

    MotionPlan.setOptions(**args)
    planner = MotionPlan(space)

    #do some checking of the terminal conditions
    if not space.isFeasible(start):
        sfailures = space.cspace.feasibilityFailures(start)
        warnings.warn("Start configuration fails constraints {}".format(sfailures))

    if hasattr(goal,'__iter__'):
        if not callable(goal[0]):
            if not space.isFeasible(goal):
                gfailures = space.cspace.feasibilityFailures(goal)
                warnings.warn("Goal configuration fails constraints {}".format(gfailures))
        else:
            if not callable(goal[1]):
                raise TypeError("goal sampler is not callable")
            try:
                goal[0](start)
            except Exception:
                warnings.warn("Goal test doesn't seem to work properly")
            try:
                qg = goal[1]()
                if len(qg) != len(start):
                    warnings.warn("Goal sampler doesn't seem to produce a properly-sized object")
            except Exception:
                warnings.warn("Goal sampler doesn't seem to work properly")
    else:
        if not callable(goal):
            raise TypeError("goal is not a configuration or callable")
        try:
            goal(start)
        except Exception:
            warnings.warn("Goal test doesn't seem to work properly")
        
    planner.setEndpoints(start,goal)
    if edgeCost or terminalCost:
        planner.setCostFunction(edgeCost,terminalCost)
    return planner,args



def _selfTest():
    c = CSpace()
    c.bound = [(-2,2),(-2,2)]
    c.feasible = lambda x: pow(x[0],2.0)+pow(x[1],2.0) > 1.0
    c.setup()
    MotionPlan.setOptions(type="rrt")
    print("Setup complete")
    p = MotionPlan(c)
    print("Setting endpoints")
    p.setEndpoints([-1.5,0],[1.5,0])
    print("PlanMore")
    p.planMore(100)
    print("GetPath")
    path = p.getPath()
    print("Resulting path:")
    print(path)
    p.close()
    c.close()

if __name__=="__main__":
    _selfTest()