"""This module provides convenient access to the motionplanning module
    functionality by defining the CSpace and MotionPlan classes."""

import motionplanning
import random

class CSpace:
    """To define a custom CSpace, subclasses will need to override:
    - feasible(x): returns true if the vector x is in the feasible space
    - *sample(): returns a new vector x from a superset of the feasible space
    - *sampleneighborhood(c,r): returns a new vector x from a neighborhood of c
      with radius r
    - *visible(a,b): returns true if the path between a and b is feasible
    - *distance(a,b): return a distance between a and b
    - *interpolate(a,b,u): interpolate between a, b with parameter u

    * indicates that the method does not need to be defined.

    To avoid memory leaks, CSpace.close() or motionplanning.destroy() must
    be called when you are done.  (The latter deallocates all previously
    created cspaces and planners)

    If sample is not defined, then subclasses should set self.bound to be a
    list of pairs defining an axis-aligned bounding box.  The setBounds method
    is a convenient way of defining this.

    If visible is not defined, then paths are checked by subdivision, with the
    collision tolerance self.eps.

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
        space properties in one lie."""
        self.bounds = bounds
        self.properties["minimum"] = [b[0] for b in bounds]
        self.properties["maximum"] = [b[1] for b in bounds]

    def close(self):
        if self.cspace != None:
            self.cspace.destroy()
            self.cspace = None
    
    def setup(self):
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
        To define a different domain, set self.bound to the desired axis-aligned
        bound.
        """
        return [random.uniform(*b) for b in self.bound]

    def feasible(self,x):
        """Overload this to define your new feasibility test"""
        return all(a<=xi<=b for (xi,(a,b)) in zip(x,self.bound))


class MotionPlan:
    def __init__(self,space,type=None):
        """Initializes a plan with a given CSpace and a given type.
        Type can be 'prm', 'rrt', 'sbl'.
        """
        if space.cspace == None:
            space.setup()
        if type != None:
            motionplanning.setPlanType(type)
        self.planner = motionplanning.PlannerInterface(space.cspace)

    def close(self):
        self.planner.destroy()

    @staticmethod
    def setOptions(**opts):
        for (a,b) in opts.items():
            if a=='type':
                motionplanning.setPlanType(str(b))
            elif isinstance(b,str):
                motionplanning.setPlanSetting(a,b)
            else:
                motionplanning.setPlanSetting(a,float(b))

    def setEndpoints(self,start,goal):
        self.planner.setEndpoints(start,goal)

    def addMilestone(self,x):
        return self.planner.addMilestone(x);
    
    def planMore(self,iterations):
        self.planner.planMore(iterations)

    def getPath(self,milestone1=None,milestone2=None):
        if milestone1==None and milestone2==None:
            return self.planner.getPathEndpoints();
        else:
            return self.planner.getPath(milestone1,milestone2)

    def getRoadmap(self):
        return self.planner.getRoadmap();



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
