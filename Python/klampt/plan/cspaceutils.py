"""General utilities for creating and operating with C-spaces.
"""

import math
import time
from ..math import vectorops
from .cspace import CSpace,MotionPlan
import warnings
import itertools
import random

def default_sampleneighborhood(c,r):
    return [ci + random.uniform(-r,r) for ci in c]

def default_visible(a,b):
    raise RuntimeError("Can't check visibility")

def default_distance(a,b):
    return vectorops.distance(a,b)

def default_interpolate(a,b,u):
    return vectorops.interpolate(a,b,u)

def make_default(space):
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
                    s.sampleneighborhood = default_sampleneighborhood
            self.sampleneighborhood = sampleneighborhood
        if any(hasattr(s,'visible') for s in spaces):
            for s in self.spaces:
                if not hasattr(s,'visible'):
                    s.visible = default_visible
            self.visible = visible
        if any(hasattr(s,'distance') for s in spaces):
            for s in self.spaces:
                if not hasattr(s,'distance'):
                    s.distance = default_distance
            self.distance = distance
        if any(hasattr(s,'interpolate') for s in spaces):
            for s in self.spaces:
                if not hasattr(s,'interpolate'):
                    s.interpolate = default_interpolate
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
    
    .. note ::

        A MotionPlan constructed on this object operates in the embedded space,
        NOT the ambient space.  To push endpoints to the embedded space you 
        will need to call :meth:`EmbeddedCSpace.project`, and to pull a plan 
        back to the ambient space, use the :meth:`EmbeddedCSpace.liftPlan`
        method.

        To make this more convenient, the :class:`SubsetMotionPlan` class is
        provided for you.  :class:`EmbeddedMotionPlan` does the same thing.

    .. note ::

        Sampling does not work directly when the ambient space has implicit
        manifold constraints, e.g., closed-loop constraints.  ``sample``
        and ``sampleneighborhood`` will need to be customized so that the
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


class AffineEmbeddedCSpace(CSpace):
    """A subspace of an ambient space, with the active DOFs given by variables x
    that are transformed to the ambient space by the transform x_amb = A*x+b.
    Here A is a sparse matrix.
    
    .. note ::

        A MotionPlan constructed on this object operates in the embedded space,
        NOT the ambient space.  To push endpoints to the embedded space you 
        will need to call :meth:`AffineEmbeddedCSpace.project`, and to pull a
        plan back to the ambient space, use the
        :meth:`AffineEmbeddedCSpace.liftPlan` method.

        To make this more convenient, the :class:`EmbeddedMotionPlan` class is
        provided for you.

    .. note ::

        Sampling does not work directly when the ambient space has implicit
        manifold constraints, e.g., closed-loop constraints.  ``sample``
        and ``sampleneighborhood`` will need to be customized so that the
        constraint solving is done without perturbing the seed configuration
        except for the dofs in the moving subset.

    Attributes:
        ambientspace (CSpace): the ambient configuration space
        A (list of dict, numpy array, or scipy sparse matrix): the map from
            embedded space to ambient space
        b (list, optional): the offset in the ambient space (by default, the
            0 vector)
    """
    def __init__(self,ambientspace,A,b=None):
        CSpace.__init__(self)
        self.ambientspace = ambientspace
        n = len(ambientspace.sample())
        self.A = A
        if hasattr(A,'shape'):  #numpy array or scipy.sparse matrix
            m = A.shape[1]
            if self.A.shape[0] != n:
                raise ValueError("Coefficient matrix must have n rows")
        else:
            if len(self.A) != n:
                raise ValueError("Coefficient matrix must have n rows")
            m = 0
            for row in A:
                if not isinstance(row,dict):
                    raise ValueError("Coefficient matrix must be a list of dicts, numpy array, or scipy matrix")
                for j in row:
                    if j < 0:
                        raise ValueError("Invalid entry {} in coeffcient matrix".format(j))
                    m = max(j,m)
            if m >= n:
                warnings.warn("Embedded space has more DOFs than ambient space?")
        self.m = m
        self.n = n

        #start at the zero config if no offset is given
        if b==None:
            self.b = [0.0]*n
        else:
            self.b = b
            if len(b) != n:
                raise ValueError("Offset matrix must have n entries")

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
        inf = float('inf')
        self.bound = [[-inf,inf] for i in range(m)]
        #set naive bounds
        Aentries = []
        if hasattr(A,'tocoo'):
            Acoo = A.tocoo()
            Aentries = zip(Acoo.row,Acoo.col,Acoo.data)
        elif hasattr(A,'dot'):
            import numpy as np
            Aentries = [(i,j,v) for (i,j),v in np.ndenumerate(A) if v != 0]
        else:
            Aentries = [(i,j,v) for (j,v) in row.items() for i,row in enumerate(A)]
        for (i,j,v) in Aentries:
            if v < 0:
                if self.bound[j][0]*v > self.ambientspace.bound[i][1]:
                    self.bound[j][0] = self.ambientspace.bound[i][1]/v
                if self.bound[j][1]*v < self.ambientspace.bound[i][0]:
                    self.bound[j][1] = self.ambientspace.bound[i][0]/v
            elif v > 0:
                if self.bound[j][0]*v < self.ambientspace.bound[i][1]:
                    self.bound[j][0] = self.ambientspace.bound[i][1]/v
                if self.bound[j][1]*v > self.ambientspace.bound[i][0]:
                    self.bound[j][1] = self.ambientspace.bound[i][0]/v

        self.properties = self.ambientspace.properties
        if self.ambientspace.feasibilityTests is not None:
            self.feasibilityTests = [(lambda x,f=f:f(self.lift(x))) for f in self.ambientspace.feasibilityTests]
            self.feasibilityTestNames = self.ambientspace.feasibilityTestNames[:]
            self.feasibilityTestDependencies = self.ambientspace.feasibilityTestDependencies[:]
        print("AffineEmbeddedCSpace projects from dimension",self.m,"to",self.n)

    @staticmethod
    def fromRobotDrivers(robot,ambientspace,drivers=None):
        """Creates an AffineEmbeddedCSpace for a robot's drivers.  

        Args:
            robot (RobotModel): the robot that you'd like to move
            ambientspace (CSpace): an ambient space on the robot's full DOFs,
                e.g., RobotCSpace.
            drivers (list of int or list of str, optional): if provided, gives
                a list of robot's active drivers.
        """
        if drivers is None:
            drivers = range(robot.numDrivers())
            m = robot.numDrivers()
        else:
            m = len(drivers)
        try:
            import scipy.sparse
            data = []
            rows = []
            cols = []
            b = [0]*robot.numLinks()
            for i,dind in enumerate(drivers):
                d = robot.driver(dind)
                links = d.getAffectedLinks()
                if d.getType() != 'affine':
                    data.append(1)
                    rows.append(links[0])
                    cols.append(i)
                else:
                    scale,offset = d.getAffineCoeffs()
                    for j,s,o in zip(links,scale,offset):
                        data.append(s)
                        rows.append(j)
                        cols.append(i)
                        b[j] = o
            A = scipy.sparse.coo_matrix((data,(rows,cols)),shape=(robot.numLinks(),m))
        except ImportError:
            A = [dict() for i in robot.numLinks()]
            b = [0]*robot.numLinks()
            for i,dind in enumerate(drivers):
                d = robot.driver(dind)
                links = d.getAffectedLinks()
                if d.getType() != 'affine':
                    A[links[0]][i] = 1
                else:
                    scale,offset = d.getAffineCoeffs()
                    for j,s,o in zip(links,scale,offset):
                        A[j][i] = s
                        b[j] = o
        return AffineEmbeddedCSpace(ambientspace,A,b)

    def project(self,xamb):
        """Ambient space -> embedded space"""
        if len(xamb) != len(self.b):
            raise ValueError("Invalid length of ambient space vector: %d should be %d"%(len(xamb),len(self.b)))
        import numpy as np
        #xamb = A*xemb + b
        if hasattr(self.A,'todense'):
            import scipy.sparse.linalg
            xemb = scipy.sparse.linalg.lsqr(self.A,np.asarray(xamb)-np.asarray(self.b))[0].tolist()
        elif hasattr(self.A,'dot'):
            xemb = np.linalg.lstsq(self.A,np.asarray(xamb)-np.asarray(self.b))[0].tolist()
        else:
            import scipy.sparse
            A = scipy.sparse.lil_matrix((self.A.shape[0],self.m))
            for i,row in enumerate(self.A):
                for j,v in row.items():
                    A.rows[i].append(j)
                    A.data[i].append(v)
            xemb = scipy.sparse.linalg.lsqr(A.tocsr(),np.asarray(xamb)-np.asarray(self.b))[0].tolist()
        assert len(xemb)==self.m
        return xemb

    def lift(self,xemb):
        """Embedded space -> ambient space"""
        if len(xemb) != self.m:
            raise ValueError("Invalid length of embedded space vector: %d should be %d"%(len(xemb),self.m))
        
        if hasattr(self.A,'dot'):  #numpy array or scipy.sparse matrix
            xamb = vectorops.add(self.b,self.A.dot(xemb))
        else:
            xamb = self.b[:]
            #list of dicts
            for (i,dofs) in enumerate(self.A):
                for j,c in dofs.items():
                    xamb[j] += c*xemb[i]
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
    """An adaptor that "lifts" a motion planner in an :class:`EmbeddedCSpace`
    to a higher dimensional ambient space.  Used for planning in subsets of
    robot DOFs.
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



class EmbeddedMotionPlan (MotionPlan):
    """An adaptor that "lifts" a motion planner in an :class:`EmbeddedCSpace`
    or :class:`AffineEmbeddedCSpace` to a higher dimensional ambient space. 

    Used for planning in robots with frozen DOFs (EmbeddedCSpace) or
    "affine" drivers (AffineEmbeddedCSpace).
    """
    def __init__(self,space,q0,type=None,**options):
        if not isinstance(space,(EmbeddedCSpace,AffineEmbeddedCSpace)):
            if not hasattr(space,'project') or not hasattr(space,'lift'):
                raise ValueError("space argument must have the project and lift methods")
            else:
                warnings.warn("EmbeddedMotionPlan has not been tested with classes other than EmbeddedCSpace and AffineEmbeddedCSpace")
        MotionPlan.__init__(self,space,type,**options)
        
    def setEndpoints(self,start,goal):
        """Takes care of projecting the start and goal (represented in the ambient
        space) down to the subset. Works with both config and set goals.
        """
        #take care of the moving subset -- make sure to lift MP configurations back to
        #space configurations
        embstart = self.space.project(start)
        if hasattr(goal,'__iter__'):
            if len(goal)==2 and callable(goal[0]) and callable(goal[1]):
                #it's a (test,sample) pair
                def goaltest(x,test=goal[0]):
                    return test(self.space.lift(x))
                def goalsample(sample=goal[1]):
                    qamb = sample()
                    qproj = self.space.project(qamb)
                    return qproj
                embgoal = [goaltest,goalsample]
            else:
                #it's a configuration
                embgoal = self.space.project(goal)
        elif callable(goal):
            def goaltest(x,goal=goal):
                return goal(self.space.lift(x))
            embgoal = goaltest
        MotionPlan.setEndpoints(self,embstart,embgoal)
        
    def addMilestone(self,x):
        """Manually adds a milestone from the ambient space, and returns its index"""
        return MotionPlan.addMilestone(self,self.space.project(x))

    def getPath(self,milestone1=None,milestone2=None):
        """Lifts the motion planner's lower-dimensional path back to the ambient space"""
        spath = MotionPlan.getPath(self,milestone1,milestone2)
        if spath == None: return None
        return [self.space.lift(q) for q in spath]

    def getRoadmap(self):
        """Lifts the motion planner's lower-dimensional roadmap back to the ambient space"""
        V,E = MotionPlan.getRoadmap(self)
        return [self.space.lift(v) for v in V],E

