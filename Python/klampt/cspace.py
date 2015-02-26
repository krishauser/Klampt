"""This module provides convenient access to the motionplanning module
   functionality by defining the CSpace and MotionPlan classes."""

import motionplanning
import random

class CSpace:
    """Used alongside MotionPlan to define a configuration space for
    motion planning.

    To define a custom CSpace, subclasses will need to override:
        
    - feasible(x): returns true if the vector x is in the feasible space
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
        self.eps = 1e-3
        self.bound = [(0,1)]
        self.properties = {}

    def setBounds(self,bounds):
        """Convenience function: sets the sampling bound and the
        space properties in one line."""
        self.bounds = bounds
        self.properties["minimum"] = [b[0] for b in bounds]
        self.properties["maximum"] = [b[1] for b in bounds]

    def close(self):
        """This method must be called to free the memory associated with the
        planner.  Alternatively, motionplanning.destroy() can be called to
        free all previously constructed CSpace and MotionPlan objects."""
        if self.cspace != None:
            self.cspace.destroy()
            self.cspace = None
    
    def setup(self):
        """Called internally by the MotionPlan class to set up planning
        hooks."""
        assert self.cspace == None
        self.cspace = motionplanning.CSpaceInterface()
        if hasattr(self,'feasible'):
            self.cspace.setFeasibility(getattr(self,'feasible'))
        else:
            raise 'Need feasible method'
        if hasattr(self,'visible'):
            self.cspace.setVisibility(getattr(self,'visible'))
        else:
            self.cspace.setVisibilityEpsilon(self.eps)
        if hasattr(self,'sample'):
            self.cspace.setSampler(getattr(self,'sample'))
        else:
            raise 'Need sample method'
        if hasattr(self,'sampleneighborhood'):
            self.cspace.setNeighborhoodSampler(getattr(self,'sampleneighborhood'))
        if hasattr(self,'distance'):
            self.cspace.setDistance(getattr(self,'distance'))
        if hasattr(self,'interpolate'):
            self.cspace.setInterpolate(getattr(self,'interpolate'))
        for (k,v) in self.properties:
            if isinstance(v,(list,tuple)):
                self.cspace.setPropety(k," ".join([str(item) for item in v]))
            else:
                self.cspace.setProperty(k,str(v))

    def sample(self):
        """Overload this to define a nonuniform sampler.
        By default, it will sample from the axis-aligned bounding box
        defined by self.bound. To define a different domain, set self.bound
        to the desired bound.
        """
        return [random.uniform(*b) for b in self.bound]

    def feasible(self,x):
        """Overload this to define your new feasibility test"""
        return all(a<=xi<=b for (xi,(a,b)) in zip(x,self.bound))


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
    def __init__(self,space,type=None):
        """Initializes a plan with a given CSpace and a given type.
        
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
        is listed in motionplanning.h)
        """
        if space.cspace == None:
            space.setup()
        if type != None:
            motionplanning.setPlanType(type)
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
            if a=='type':
                motionplanning.setPlanType(str(b))
            elif isinstance(b,str):
                motionplanning.setPlanSetting(a,b)
            else:
                motionplanning.setPlanSetting(a,float(b))

    def setEndpoints(self,start,goal):
        """Sets the start and goal configuration."""
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



if __name__=="__main__":
    c = CSpace()
    c.bound = [(-2,2),(-2,2)]
    c.feasible = lambda(x): pow(x[0],2.0)+pow(x[1],2.0) > 1.0
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
