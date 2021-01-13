"""Helpers to make the rootfind module more convenient and Pythonic.
"""

class VectorFieldFunction:
    """A callback class used with the rootfind module to define a vector
    field :math:`f(x)=0` to be solved for during Newton-Raphson root finding.
    The output is dimension m and the input is dimension n.

    At the minimum, your subclass should fill out the m, n attributes, and
    override the eval(), and jacobian() functions.  The jacobian_numeric
    function is provided for you in case you want to use differencing
    to approximate the jacobian.
    """
    def __init__(self):
        self.n = 0
        self.m = 0

    def eval(self, x):
        pass
    
    def eval_i(self, x, i):
        pass

    def jacobian(self, x):
        pass
    
    def jacobian_ij(self, x, i, j):
        pass
    
    def num_vars(self):
        return self.n
    
    def num_fns(self):
        return self.m

    def jacobian_numeric(self,x,delta):
        """Helper method: returns the centered-difference jacobian
        approximation with stepsize delta."""
        xtemp = x[:]
        J = []
        for i,xi in enumerate(x):
            xtemp[i] = xi - delta
            e1 = self.eval(xtemp)
            xtemp[i] = xi + delta
            e2 = self.eval(xtemp)
            xtemp[i] = xi
            J.append([(ei2-ei1)/(2.0*delta) for (ei1,ei2) in zip(e1,e2)])
        return J
        
class CompositeVectorFieldFunction(VectorFieldFunction):
    """A helper VectorFieldFunction that aggregates multiple
    VectorFieldFunctions into a stacked constraint::
    
        0 = f1(x)
        0 = f2(x)
        ...
        0 = fn(x)
    
    """
    def __init__(self, fns):
        if not fns:
            raise RuntimeError("Must have at least one function for composite")
        self.fns = fns
        self.n = self.fns[0].num_vars()
        for f in self.fns:
            if f.num_vars() != self.n:
                raise RuntimeError("Functions must take the same vector")
        self.m = sum([f.num_fns() for f in self.fns])

    def eval(self, x):
        res = []
        for f in self.fns:
            res += f.eval(x)
        return res
            
    def eval_i(self, x, i):
        for f in self.fns:
            if i < f.num_fns():
                return f.eval_i(x, i)
            i -= f.num_fns()
        raise RuntimeError("eval_i: i must be between 0 and %d" % self.num_fns())

    def jacobian(self, x):
        res = []
        for f in self.fns:
            res += f.jacobian(x)
        return res
        
    def jacobian_ij(self, x, i, j):
        for f in self.fns:
            if i < f.num_fns():
                return f.jacobian_ij(x,i,j)
            i -= f.num_fns()
        raise RuntimeError("jacobian_ij: i must be between 0 and %d" % self.num_fns())
