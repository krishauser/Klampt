import math
import time
from cspace import CSpace

def default_sampleneighborhood(self,c,r):
    return [ci + random.uniform(-r,r) for ci in c]

def default_visible(self,a,b):
    raise RuntimeError("Can't check visibility")

def default_distance(self,a,b):
    return math.sqrt(math.pow(ai-bi,2) for (ai,bi) in zip(a,b))

def default_interpolate(self,a,b,u):
    return [ai+u*(bi-ai) for (ai,bi) in zip(a,b)]

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
        def sampleneighborhood(self,c,r):
            return self.join(s.sampleNeighborhood(cs,r) for (s,cs) in zip(self.spaces,self.split(c)))
        def visible(self,a,b):
            return all(s.visible(ai,bi) for (s,ai,bi) in zip(self.spaces,self.split(a),self.split(b)))
        def distance(self,a,b):
            return sum(s.distance(ai,bi) for (s,ai,bi) in zip(self.spaces,self.split(a),self.split(b)))
        def interpolate(self,a,b,u):
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
    """A subspace of an ambient space, with the active DOFs given by the
    mapping list."""
    def __init__(self,ambientspace,xinit=None):
        self.ambientspace = ambientspace
        n = len(ambientspace.sample())
        self.mapping = range(n)
        #start at the zero config if no initial configuration is given
        if xinit==None:
            self.xinit = [0.0]*n
        else:
            self.xinit = xinit

        #construct optional methods
        def sampleneighborhood(self,c,r):
            return self.project(self.ambientspace.sampleNeighborhood(self.lift(c),r))
        def visible(self,a,b):
            return self.ambientspace.visible(self.lift(a),self.lift(b))
        def distance(self,a,b):
            return self.ambientspace.distance(self.lift(a),self.lift(b))
        def interpolate(self,a,b,u):
            return self.project(self.ambientspace.interpolate(self.lift(a)),self.lift(b))

        if hasattr(ambientspace,'sampleneighborhood'):
            self.sampleneighborhood = sampleneighborhood
        if hasattr(s,'visible'):
            self.visible = visible
        if hasattr(s,'distance'):
            self.distance = distance
        if hasattr(s,'interpolate'):
            self.interpolate = interpolate

    def project(self,xamb):
        """Ambient space -> embedded space"""
        return [xamb[i] for i in self.mapping]

    def lift(self,xemb):
        """Embedded space -> ambient space"""
        xamb = xinit[:]
        for (i,j) in enumerate(self.mapping):
            xamb[j] = xemb[i]
        return xamb

    def feasible(self,x):
        return self.ambientspace.feasible(self.lift(x))
        
    def sample(self):
        return self.project(self.ambientspace.sample())
    
    

class ZeroTest:
    """A test that evaluates to 0 at the feasible set"""
    def __init__(self):
        self.name = '0'
        self.type = 'constant'
        self.dist = lambda(x): 1 if x != 0 else 0

    def __call__(self,obj):
        return self.dist(obj)

    def setConstant(self,val):
        self.name = str(val)
        self.type = 'constant'
        if isinstance(val,(int,float)):
            self.dist = lambda(x): x-val
        else:
            self.dist = lambda(x): 1 if x != val else 0

    def setCondition(self,f,name=None):
        """Let this be 0 whenever f evaluates to True"""
        if name: self.name = name
        else: self.name = "f(x)"
        self.f = f
        self.dist = lambda(x): 0 if f(x) else 1
            
    def setComparison(self,f,cmp,rhs,name=None):
        if name: self.name = name+cmp+str(rhs)
        else: self.name = "f(x)"+cmp+str(rhs)
        self.f = f
        self.cmp = cmp
        self.rhs = rhs
        
        def lessPenalty(x,y):
            if isinstance(x,int) and isinstance(y,int):
                return max(1+x-y,0)
            else:
                return max(x-y,0)
        
        def greaterPenalty(x,y):
            if isinstance(x,int) and isinstance(y,int):
                return max(1+y-x,0)
            else:
                return max(y-x,0)
        
        comparisons = {'==':lambda x,y:abs(x-y),
                       '>=':lambda x,y:max(y-x,0),
                       '<=':lambda x,y:max(x-y,0),
                       '<':lessPenalty,
                       '>':greaterPenalty}
        self.dist = comparisons[cmp]

class AdaptiveZeroTester:
    """Tests a set of tests f1(x),...,fn(x) for equality to zero.
    Maintains statistics about evaluation time, success rate,
    max/average deviation from zero.  The stats are then used to determine
    the optimal testing order.
    """
    def __init__(self):
        self.tests = []

    def add_test(self,f):
        self.tests.append(f)
        self.reset_history(self.tests[-1])

    def update_order(self):
        thelist = [(f._sum_cost/f._num_fail,f) for f in self.tests]
        self.tests = [t for (ec,t) in sorted(thelist)]

    def testmax(self,*args):
        """Tests all tests, returning the max absolute deviation from 0."""
        vmax = 0.0
        for f in self.tests:
            t1 = time.time()
            res = f(*args)
            t2 = time.time()
            self.update_stats(f,t2-t1,res)
            vmax = max(vmax,abs(res))
        self.update_order()
        return vmax

    def test(self,*args):
        """Tests whether *args passes all tests.  Updates the stats and
        internal order"""
        for f in self.tests:
            t1 = time.time()
            res = f(*args)
            t2 = time.time()
            self.update_stats(f,t2-t1,res)
            if res != 0:
                self.update_order()
                return False
        self.update_order()
        return True

    def expectation(self):
        """Returns (expected cost, expected success) of testing all tests
        in the current order."""
        c = 0.0
        p = 1.0
        for f in self.tests:
            avgcost = f._sum_cost/(f._num_pass+f._num_fail)
            failrate = float(f._num_fail)/(f._num_pass+f._num_fail)
            c += p*avgcost
            p *= failrate
        return (c,p)

    def reset_history(self,f,avg_cost=1.0,pr_pass=0.5,evidence=2.0):
        f._sum_cost = avg_cost*evidence
        f._num_pass = pr_pass*evidence
        f._num_fail = (1.0-pr_pass)*evidence
        f._max_dist = 0.
        f._sum_dist = 0.

    def update_stats(self,f,cost,res):
        f._sum_cost += cost
        if res==0: f._num_pass += 1
        else:
            f._num_fail += 1
            f._max_dist = max(f._max_dist,abs(res))
            f._sum_dist += abs(res)

    def stats(self,f):
        """Returns a dictionary describing the statistics of f"""
        res = dict()
        res['average cost']=f._sum_cost/(f._num_pass+f._num_fail)
        res['pass rate']=float(f._num_pass)/(f._num_pass+f._num_fail)
        res['evaluations']=f._num_pass+f._num_fail
        res['max distance']=f._max_dist
        res['average distance']=float(f._sum_dist)/(f._num_pass+f._num_fail)
        return res

    def init_stats(self,f,d):
        """Given a dictionary returned by a stats() call, fills in the
        appropriate statistics of f"""
        f._sum_cost = d['average cost']*d['evaluations']
        f._num_pass = d['evaluations']*d['pass rate']
        f._num_fail = d['evaluations']*(1.0-d['pass rate'])
        f._max_dist = d['max distance']
        f._sum_dist = d['average distance']*d['evaluations']




class AdaptiveCSpace(CSpace,AdaptiveZeroTester):
    """A cspace with an adaptive feasibility checker.  Subclasses
    fill out feasibility tests using addFeasibleTest (binary conditions)
    or addFeasibleComp (inequalities).  The cspace will then learn the
    characteristics of each test and find the (near) optimal testing
    order."""
    def __init__(self):
        CSpace.__init__(self)
        AdaptiveZeroTester.__init__(self)

    def addFeasibleTest(self,f,name):
        t = ZeroTest()
        t.setCondition(f,name)
        self.add_test(t)

    def addFeasibleComp(self,f,cmp,val,name):
        t = ZeroTest()
        t.setComparison(f,cmp,val,name)
        self.add_test(t)

    def feasible(self,x):
        return self.test(x)

    def stats(self):
        """Retreives the zero tester stats."""
        res = dict()
        for t in self.tests:
            res[t.name] = AdaptiveZeroTester.stats(self,t)
        return res

    def init_stats(self,d):
        """Given a dictionary of (name,dict) pairs returned from stats(),
        initializes the zero tester stats."""
        for (k,v) in d.iteritems():
            found = False
            for t in self.tests:
                if t.name == k:
                    AdaptiveZeroTester.init_stats(self,t,v)
                    found = True
                    break
            if not Found:
                raise RuntimeError("init_stats: key '"+key+"' not found")
