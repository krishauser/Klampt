"""This module provides convenient access to the motionplanning module
   functionality by defining the CSpace and MotionPlan classes."""

import motionplanning
import random

class CSpace:
    """Used alongside MotionPlan to define a configuration space for
    motion planning.

    Attributes:
    - eps: the collision tolerance used for checking edges, in the units
      defined by the distance(a,b) method.  (by default, euclidean distance)
    - bound: a list of lower and upper bounds on the space [(l1,u1),...,(ld,ud)]
      where d is the dimension of the configuration space.
    - properties: a map of properties that may be used by a planner.  See below
      documentation for more detail. 
    Internally used attributes:
    - cspace: a motionplanning module CSpaceInterface object
    - feasibilityTests: a list of one-argument functions that test feasibility
      of this configuration space's constraints.  E.g.,
      if you have a collision tester that returns True if a configuration is in
      collision, and also want to check bounds, you can set this to a list:
      [self.in_bounds,lambda x not self.collides(x)]
      You should not write this directly but instead use addFeasibilityTest.
    - feasibilityTestNames: a list of names of feasibility tests.  You should
      not write this directly but instead use addFeasibilityTest.

    To define a custom CSpace, subclasses will need to override:
        
    - *feasible(x): returns true if the vector x is in the feasible space.  By
      default calls each function in self.feasibilityTests, which by default
      only tests bound constraints.
    - *sample(): returns a new vector x from a superset of the feasible space
    - *sampleneighborhood(c,r): returns a new vector x from a neighborhood of c
      with radius r
    - *visible(a,b): returns true if the path between a and b is feasible
    - *distance(a,b): return a distance between a and b
    - *interpolate(a,b,u): interpolate between a, b with parameter u
    (* indicates an optional override.)

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
                print "CSpace.setup(): Performance warning, called twice, destroying previous CSpaceInterface object"
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
        for (k,v) in self.properties.iteritems():
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
    only kinematic, point-to-point plans.

    Multi-query roadmaps are supported for the PRM and SBLPRT algorithms.
    In multi-query mode, you may call addMilestone(q) to add a new milestone.
    addMilestone() returns the milestone's index, which can be used
    in later calls to getPath().

    Planner parameters must be set by calling the static
    MotionPlan.setOptions(param1=value1,param2=value2,...) method BEFORE
    calling the MotionPlan(space,type) constructor.

    If type is not specified in the constructor, the planning algorithm
    will be chosen by default.
    
    Note that MotionPlan.close() or motionplanning.destroy() must be called
    to free memory after you are done.
    """
    def __init__(self,space,type=None,**options):
        """Initializes a plan with a given CSpace and a given type.
        Optionally, planner options can be set via keyword arguments.
        
        Valid values for type are:
            - prm: the Probabilistic Roadmap algorithm
            - rrt: the Rapidly Exploring Random Trees algorithm
            - sbl: the Single-Query Bidirectional Lazy planner
            - sblprt: the probabilistic roadmap of trees (PRT) algorithm with SBL as the inter-root planner.
            - rrt*: the RRT* algorithm for optimal motion planning 
            - prm*: the PRM* algorithm for optimal motion planning
            - lazyprm*: the Lazy-PRM* algorithm for optimal motion planning
            - lazyrrg*: the Lazy-RRG* algorithm for optimal motion planning
            - fmm: the fast marching method algorithm for resolution-complete optimal motion planning
            - fmm*: an anytime fast marching method algorithm for optimal motion planning
        (this list may be out-of-date; the most current documentation
        is listed in src/motionplanning.h)
        """
        if space.cspace is None:
            space.setup()
        if type != None:
            motionplanning.setPlanType(type)
        if len(options) > 0:
            MotionPlan.setOptions(**options)
        self.space = space
        self.planner = motionplanning.PlannerInterface(space.cspace)

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
            - "knn": k value for the k-nearest neighbor connection strategy
              (used in PRM)
            - "connectionThreshold": a milestone connection threshold
            - "perturbationRadius": (for RRT and SBL)
            - "bidirectional": 1 if bidirectional planning is requested (used
              in RRT)
            - "grid": 1 if a point selection grid should be used (used in SBL)
            - "gridResolution": resolution for the grid, if the grid should
              be used (used in SBL with grid, FMM, FMM*)
            - "suboptimalityFactor": allowable suboptimality (used in RRT*,
              lazy PRM*, lazy RRG*)
            - "randomizeFrequency": a grid randomization frequency (for SBL)
            - "shortcut": nonzero if you wish to perform shortcutting to
              improve a path after a first plan is found.
            - "restart": nonzero if you wish to restart the planner to
              get progressively better paths with the remaining time.
            - "pointLocation": a string designating a point location data
              structure. "kdtree" is supported, optionally followed by a
              weight vector (used in PRM, RRT*, PRM*, LazyPRM*, LazyRRG*)
            - "restartTermCond": used if the "restart" setting is true.
              This is a JSON string defining the termination condition
              (default value:
                "{foundSolution:1;maxIters:1000}")
        (this list may be out-of-date; the most current documentation
        is listed in motionplanning.h)
        """
        for (a,b) in opts.items():
            try:
                if a=='type':
                    motionplanning.setPlanType(str(b))
                elif isinstance(b,str):
                    motionplanning.setPlanSetting(a,b)
                else:
                    motionplanning.setPlanSetting(a,float(b))
            except Exception:
                    print "Invalid MotionPlan setting",a,"=",b
                    print "Valid keys are:"
                    print "  type, knn, connectionThreshold, perturbationRadius, grid, gridResolution"
                    print "  suboptimalityFactor, randomizeFrequency, shortcut, restart, pointLocation"
                    print "  restartTermCond"
    def setEndpoints(self,start,goal):
        """Sets the start and goal configuration.  goal can also be a
        *goal test*, which is a function taking one argument f(q) that
        returns true if the configuration is at the goal and false
        otherwise.  Another representation of a goal test is a pair
        (f,s) where f() tests whether the configuration is at the goal,
        and s() generates a new sample at the goal."""
        if hasattr(goal,'__call__'):
            self.planner.setEndpointSet(start,goal)
        elif(len(goal) == 2 and hasattr(goal[0],'__call__')):
            self.planner.setEndpointSet(start,goal[0],goal[1])
        else:
            self.planner.setEndpoints(start,goal)

    def addMilestone(self,x):
        """Manually adds a milestone and returns its index"""
        return self.planner.addMilestone(x);
    
    def planMore(self,iterations):
        """Performs a given number of iterations of planning."""
        self.planner.planMore(iterations)

    def getPath(self,milestone1=None,milestone2=None):
        """Returns the path between the two milestones.  If no
        arguments are provided, this returns the path between the
        start and goal.

        The path is a list of configurations (each configuration is a Python
        list)."""
        if milestone1==None and milestone2==None:
            return self.planner.getPathEndpoints();
        else:
            return self.planner.getPath(milestone1,milestone2)

    def getRoadmap(self):
        """Returns a graph (V,E) containing the planner's roadmap.
        V is a list of configurations (each configuration is a Python list)
        and E is a list of edges (each edge is a pair (i,j) indexing into V).
        """
        return self.planner.getRoadmap()

    def getStats(self):
        """Returns a dictionary mapping statistic names to values.  Result is
        planner-dependent """
        return self.planner.getStats()

def _selfTest():
    c = CSpace()
    c.bound = [(-2,2),(-2,2)]
    c.feasible = lambda x: pow(x[0],2.0)+pow(x[1],2.0) > 1.0
    c.setup()
    MotionPlan.setOptions(type="rrt")
    print "Setup complete"
    p = MotionPlan(c)
    print "Setting endpoints"
    p.setEndpoints([-1.5,0],[1.5,0])
    print "PlanMore"
    p.planMore(100)
    print "GetPath"
    path = p.getPath()
    print "Resulting path:"
    print path
    p.close()
    c.close()

if __name__=="__main__":
    _selfTest()