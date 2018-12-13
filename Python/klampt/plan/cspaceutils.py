import math
import time
from ..math import vectorops
from cspace import CSpace

def default_sampleneighborhood(c,r):
    return [ci + random.uniform(-r,r) for ci in c]

def default_visible(a,b):
    raise RuntimeError("Can't check visibility")

def default_distance(a,b):
    return vectorops.distance(a,b)

def default_interpolate(a,b,u):
    return vectorops.interpolate(a,b,u)

def makedefault(space):
    """Helper: makes a space's callbacks perform the default Cartesian space
    operations."""
    space.sampleneighborhood = default_sampleneighborhood
    space.visible = default_visible
    space.distance = default_distance
    space.interpolate = default_interpolate



class CompositeCSpace(CSpace):
    """A cartesian product of multiple spaces, given as a list upon
    construction.  The feasible method can be overloaded to include
    interaction tests."""
    def __init__(self,spaces):
        CSpace.__init__(self)
        self.spaces = spaces

        #construct optional methods
        def sampleneighborhood(c,r):
            return self.join(s.sampleneighborhood(cs,r) for (s,cs) in zip(self.spaces,self.split(c)))
        def visible(a,b):
            return all(s.visible(ai,bi) for (s,ai,bi) in zip(self.spaces,self.split(a),self.split(b)))
        def distance(a,b):
            return sum(s.distance(ai,bi) for (s,ai,bi) in zip(self.spaces,self.split(a),self.split(b)))
        def interpolate(a,b,u):
            return self.join(s.interpolate(ai,bi,u) for (s,ai,bi) in zip(self.spaces,self.split(a),self.split(b)))
        
        if any(hasattr(s,'sampleneighborhood') for s in spaces):
            for s in self.spaces:
                if not hasattr(s,'sampleneighborhood'):
                    s.sampleneighborhood = defaultsampleneighborhood
            self.sampleneighborhood = sampleneighborhood
        if any(hasattr(s,'visible') for s in spaces):
            for s in self.spaces:
                if not hasattr(s,'visible'):
                    s.visible = defaultvisible
            self.visible = visible
        if any(hasattr(s,'distance') for s in spaces):
            for s in self.spaces:
                if not hasattr(s,'distance'):
                    s.distance = defaultdistance
            self.distance = distance
        if any(hasattr(s,'interpolate') for s in spaces):
            for s in self.spaces:
                if not hasattr(s,'interpolate'):
                    s.interpolate = defaultinterpolate
            self.interpolate = interpolate

        #TODO: should add feasibility tests for subspaces -- this will allow the planning module to optimize
        #constraint testing order.

    def subDims(self):
        return [len(s.sample()) for s in self.spaces]

    def split(self,x):
        d = self.subDims()
        res = []
        pos = 0
        for di in d:
            res.append(x[pos:pos+di])
            pos += di
        return res

    def join(self,xs):
        res = []
        for x in xs:
            res += x
        return res

    def feasible(self,x):
        for (xi,si) in zip(self.split(x),self.spaces):
            if not si.feasible(xi):
                return False
        return True
        
    def sample(self):
        return self.join(s.sample() for s in self.spaces)



class EmbeddedCSpace(CSpace):
    """A subspace of an ambient space, with the active DOFs given by a list
    of DOF indices of that ambient space.
    
    Attributes:
    - ambientspace: the ambient configuration space
    - mapping: the list of active indices into the ambient configuration
      space
    - xinit: the initial configuration in the ambient space (by default, 0)
    """
    def __init__(self,ambientspace,subset,xinit=None):
        CSpace.__init__(self)
        self.ambientspace = ambientspace
        n = len(ambientspace.sample())
        self.mapping = subset
        #start at the zero config if no initial configuration is given
        if xinit==None:
            self.xinit = [0.0]*n
        else:
            self.xinit = xinit

        #construct optional methods
        def sampleneighborhood(c,r):
            return self.project(self.ambientspace.sampleneighborhood(self.lift(c),r))
        def visible(a,b):
            return self.ambientspace.visible(self.lift(a),self.lift(b))
        def distance(a,b):
            return self.ambientspace.distance(self.lift(a),self.lift(b))
        def interpolate(a,b,u):
            return self.project(self.ambientspace.interpolate(self.lift(a),self.lift(b),u))

        if hasattr(ambientspace,'sampleneighborhood'):
            self.sampleneighborhood = sampleneighborhood
        if hasattr(ambientspace,'visible'):
            self.visible = visible
        if hasattr(ambientspace,'distance'):
            self.distance = distance
        if hasattr(ambientspace,'interpolate'):
            self.interpolate = interpolate
        self.eps = self.ambientspace.eps
        self.bound = [self.ambientspace.bound[i] for i in self.mapping]
        self.properties = self.ambientspace.properties
        if self.ambientspace.feasibilityTests is not None:
            self.feasibilityTests = [(lambda x,f=f:f(self.lift(x))) for f in self.ambientspace.feasibilityTests]
            self.feasibilityTestNames = self.ambientspace.feasibilityTestNames[:]
            self.feasibilityTestDependencies = self.ambientspace.feasibilityTestDependencies[:]

    def project(self,xamb):
        """Ambient space -> embedded space"""
        if len(xamb) != len(self.xinit):
            raise ValueError("Invalid length of ambient space vector: %d should be %d"%(len(xamb),len(self.xinit)))
        return [xamb[i] for i in self.mapping]

    def lift(self,xemb):
        """Embedded space -> ambient space"""
        if len(xemb) != len(self.mapping):
            raise ValueError("Invalid length of embedded space vector: %d should be %d"%(len(xemb),len(self.mapping)))
        xamb = self.xinit[:]
        for (i,j) in enumerate(self.mapping):
            xamb[j] = xemb[i]
        return xamb

    def liftPath(self,path):
        """Given a CSpace path path, lifts this to the full ambient space configuration"""
        return [self.lift(q) for q in path]

    def feasible(self,x):
        return self.ambientspace.feasible(self.lift(x))
        
    def sample(self):
        return self.project(self.ambientspace.sample())
    
    
