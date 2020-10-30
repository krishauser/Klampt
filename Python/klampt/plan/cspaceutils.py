import math
import time
from ..math import vectorops
from .cspace import CSpace,MotionPlan

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
    
    Note:
        A MotionPlan constructed on this object operates in the embedded space,
        NOT the ambient space.  To push endpoints to the embedded space you 
        will need to call :meth:`EmbeddedCSpace.project`, and to pull a plan 
        back to the ambient space, use the :meth:`EmbeddedCSpace.liftPlan`
        method.

        To make this more convenient, the SubsetMotionPlan class is provided
        for you.

    Note:
        Sampling does not work directly when the ambient space has implicit
        manifold constraints, e.g., closed-loop constraints.  sample
        and sampleneighborhood will need to be customized so that the
        constraint solving is done without perturbing the seed configuration
        except for the dofs in the moving subset.

    Attributes:
        ambientspace (CSpace): the ambient configuration space
        mapping (list): the list of active indices into the ambient configuration
            space
        xinit (list, optional): the initial configuration in the ambient space
            (by default, 0 vector)
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
        """Given a CSpace path, lifts it to a full ambient C-space path"""
        return [self.lift(q) for q in path]

    def projectPath(self,path_amb):
        """Given an ambient C-space path, projects it to a C-space path"""
        return [self.project(q) for q in path_amb]

    def feasible(self,x):
        return self.ambientspace.feasible(self.lift(x))
        
    def sample(self):
        return self.project(self.ambientspace.sample())
    
    
class SubsetMotionPlan (MotionPlan):
    """An adaptor that "lifts" a motion planner in an EmbeddedCSpace to a
    higher dimensional ambient space.  Used for planning in subsets of robot DOFs.
    """
    def __init__(self,space,subset,q0,type=None,**options):
        MotionPlan.__init__(self,space,type,**options)
        self.subset = subset
        self.q0 = q0

    def project(self,xamb):
        """Ambient space -> embedded space"""
        if len(xamb) != len(self.q0):
            raise ValueError("Invalid length of ambient space vector: %d should be %d"%(len(xamb),len(self.q0)))
        return [xamb[i] for i in self.subset]

    def lift(self,xemb):
        """Embedded space -> ambient space"""
        if len(xemb) != len(self.subset):
            raise ValueError("Invalid length of embedded space vector: %d should be %d"%(len(xemb),len(self.subset)))
        xamb = self.q0[:]
        for (i,j) in enumerate(self.subset):
            xamb[j] = xemb[i]
        return xamb

    def setEndpoints(self,start,goal):
        """Takes care of projecting the start and goal (represented in the ambient
        space) down to the subset. Works with both config and set goals.
        """
        #take care of the moving subset -- make sure to lift MP configurations back to
        #space configurations
        embstart = self.project(start)
        if hasattr(goal,'__iter__'):
            if len(goal)==2 and callable(goal[0]) and callable(goal[1]):
                #it's a (test,sample) pair
                def goaltest(x,test=goal[0]):
                    return test(self.lift(x))
                def goalsample(sample=goal[1]):
                    qamb = sample()
                    qproj = self.project(qamb)
                    return qproj
                embgoal = [goaltest,goalsample]
            else:
                #it's a configuration
                embgoal = self.project(goal)
        elif callable(goal):
            def goaltest(x,goal=goal):
                return goal(self.lift(x))
            embgoal = goaltest
        MotionPlan.setEndpoints(self,embstart,embgoal)

    def addMilestone(self,x):
        """Manually adds a milestone from the ambient space, and returns its index"""
        return MotionPlan.addMilestone(self,self.project(x))

    def getPath(self,milestone1=None,milestone2=None):
        """Lifts the motion planner's lower-dimensional path back to the ambient space"""
        spath = MotionPlan.getPath(self,milestone1,milestone2)
        if spath == None: return None
        return [self.lift(q) for q in spath]

    def getRoadmap(self):
        """Lifts the motion planner's lower-dimensional roadmap back to the ambient space"""
        V,E = MotionPlan.getRoadmap(self)
        return [self.lift(v) for v in V],E

