"""Basic math / linear algebra routines. Defines the following AD functions:

 =================  =============  ===================================
 Function           Derivative     Notes
 =================  =============  ===================================
 exp                1
 log                1
 sqrt               1
 sin                Y
 cos                Y
 dot                Y
 linear             Y              Produces :math:`A x` for a fixed A
 quadratric         Y              Produces :math:`x^T A x` for a fixed A
 bilinear           Y              Produces :math:`x^T A y` for a fixed A
 ShapedDot          Y              Reshapes a vector before performing dot
 norm               1              Standard L2 norm
 normSquared        Y
 distance           1              Standard L2 norm
 distanceSquared    Y
 unit               N
 cross              1 
 interpolate        1
 =================  =============  ===================================

Module contents
~~~~~~~~~~~~~~~

.. autosummary::
    exp
    log
    sqrt
    sin
    cos
    dot
    linear
    quadratric
    bilinear
    ShapedDot
    norm
    normSquared
    distance
    distanceSquared
    unit
    cross
    interpolate

"""

import numpy as np 
from .ad import ADFunctionInterface,function,_scalar,_size


class _ADExp(ADFunctionInterface):
    def __str__(self):
        return 'exp'
    def n_args(self):
        return 1
    def n_in(self,arg):
        return -1
    def n_out(self):
        return -1
    def eval(self,x):
        return np.exp(x)
    def derivative(self,arg,x):
        assert arg == 0
        diag = np.exp(x)
        if _scalar(diag):
            return np.array([[diag]])
        else:
            return np.diag(diag)
    def jvp(self,arg,darg,x):
        assert arg == 0
        return np.exp(x)*darg


class _ADLog(ADFunctionInterface):
    def __str__(self):
        return 'log'
    def n_args(self):
        return 1
    def n_in(self,arg):
        return -1
    def n_out(self):
        return -1
    def eval(self,x):
        return np.log(x)
    def derivative(self,arg,x):
        assert arg == 0
        diag = 1.0/x
        if _scalar(diag):
            return np.array([[diag]])
        else:
            return np.diag(diag)
    def jvp(self,arg,darg,x):
        assert arg == 0
        return darg/x


class _ADSqrt(ADFunctionInterface):
    def __str__(self):
        return 'sqrt'
    def n_args(self):
        return 1
    def n_in(self,arg):
        return -1
    def n_out(self):
        return -1
    def eval(self,x):
        return np.sqrt(x)
    def derivative(self,arg,x):
        assert arg == 0
        return np.diag(0.5/np.sqrt(x))
    def jvp(self,arg,darg,x):
        assert arg == 0
        return 0.5*darg/np.sqrt(x)


class _ADSin(ADFunctionInterface):
    def __str__(self):
        return 'sin'
    def n_args(self):
        return 1
    def n_in(self,arg):
        return -1
    def n_out(self):
        return -1
    def eval(self,x):
        return np.sin(x)
    def derivative(self,arg,x):
        assert arg == 0
        return np.diag(np.cos(x))
    def jvp(self,arg,darg,x):
        assert arg == 0
        return np.cos(x)*darg
    def gen_derivative(self,arg,x):
        if len(arg) == 1:
            return self.derivative(arg[0],x)
        if not _scalar(x):
            raise NotImplementedError()
        if len(arg)%2 == 1:
            base = np.cos(x)
        else:
            base = np.sin(x)
        sign = [1,1,-1,-1][len(arg)%4]
        return sign*np.array(base)


class _ADCos(ADFunctionInterface):
    def __str__(self):
        return 'cos'
    def n_args(self):
        return 1
    def n_in(self,arg):
        return -1
    def n_out(self):
        return -1
    def eval(self,x):
        return np.cos(x)
    def derivative(self,arg,x):
        assert arg == 0
        return np.diag(-np.sin(x))
    def jvp(self,arg,darg,x):
        assert arg == 0
        return -np.sin(x)*darg
    def gen_derivative(self,arg,x):
        if len(arg) == 1:
            return self.derivative(arg[0],x)
        if not _scalar(x):
            raise NotImplementedError()
        if len(arg)%2 == 1:
            base = np.sin(x)
        else:
            base = np.cos(x)
        sign = [-1,-1,1,1][len(arg)%4]
        return sign*np.array(base)


class _ADDot(ADFunctionInterface):
    def __str__(self):
        return 'dot'
    def n_args(self):
        return 2
    def n_in(self,arg):
        return -1
    def n_out(self):
        return 1
    def eval(self,x,y):
        return np.dot(x,y)
    def derivative(self,arg,x,y):
        other = y if arg==0 else x
        return np.asarray(other)[np.newaxis,:]
    def jvp(self,arg,darg,x,y):
        other = y if arg==0 else x
        return np.dot(other,darg)
    def gen_derivative(self,arg,x,y):
        if len(arg) == 1:
            return self.derivative(arg[0],x,y)
        elif len(arg) == 2:
            if arg[0] != arg[1]:
                return np.eye(x.shape[0])[np.newaxis,:,:]
        return 0


class ShapedDot(ADFunctionInterface):
    """This function is used to do matrix-vector products, which are by
    normally not supported by the dot function since all arguments are
    flattened 1D arrays.  This function reshapes either the first or second
    arguments into a 2D array before calling dot.  You must specify the array
    shape upon instantiation.
    """
    def __init__(self,shape1=None,shape2=None):
        if shape1 is not None:
            assert len(shape1)==2,"ShapedDot should only be applied to reshape to matrices"
            assert shape2 is None,"Can't reshape both vectors"
        if shape2 is not None:
            assert len(shape2)==2,"ShapedDot should only be applied to reshape to matrices"
            assert shape1 is None,"Can't reshape both vectors"
        assert shape1 is not None or shape2 is not None,"At least one argument should be reshaped"
        self.shape1 = shape1
        self.shape2 = shape2
    def n_args(self):
        return 2
    def n_in(self,arg):
        if arg == 0 and self.shape1 is not None:
            return self.shape1[0]*self.shape1[1]
        if arg == 1 and self.shape2 is not None:
            return self.shape2[0]*self.shape2[1]
        return -1
    def n_out(self):
        if self.shape1 is not None:
            return self.shape1[0]
        if self.shape2 is not None:
            return self.shape2[1]
        return 1
    def eval(self,x,y):
        if self.shape1 is not None:
            return np.dot(x.reshape(self.shape1),y)
        else:
            return np.dot(x,y.reshape(self.shape2))
    def derivative(self,arg,x,y):
        if arg == 0:
            if self.shape1 is not None:
                assert self.shape1[1] == len(y)
                raise NotImplementedError()
            else:
                return y.reshape(self.shape2).T
        else:
            if self.shape2 is not None:
                assert self.shape2[0] == len(x)
                raise NotImplementedError()
            else:
                return x.reshape(self.shape1)
    def jvp(self,arg,darg,x,y):
        if arg == 0:
            if self.shape1 is not None:
                assert self.shape1[1] == len(y)
                return np.dot(darg.reshape(self.shape1),y)
            else:
                return np.dot(darg,y.reshape(self.shape2))
        else:
            if self.shape2 is not None:
                assert self.shape2[0] == len(x)
                return np.dot(x,darg.reshape(self.shape2))
            else:
                return np.dot(x.reshape(self.shape1),darg)
    def gen_derivative(self,arg,x,y):
        if len(arg) == 1:
            return self.derivative(arg[0],x,y)
        elif len(arg) == 2:
            if arg[0] != arg[1]:
                raise NotImplementedError()
        else:
            return 0


class _ADLinear(ADFunctionInterface):
    def __init__(self,A):
        self.A = A 
        assert len(A.shape) == 2
    def __str__(self):
        return 'linear'
    def n_args(self):
        return 1
    def n_in(self,arg):
        return self.A.shape[1]
    def n_out(self):
        return self.A.shape[0]
    def eval(self,x):
        return self.A.dot(x)
    def derivative(self,arg,x):
        assert arg == 0
        return self.A
    def jvp(self,arg,darg,x):
        return np.dot(self.A,darg)
    def gen_derivative(self,arg,x):
        if len(arg) == 1:
            return self.derivative(arg[0],x)
        else:
            return 0


class _ADQuadratic(ADFunctionInterface):
    def __init__(self,A):
        self.A = A 
        assert len(A.shape) == 2
        assert A.shape[0] == A.shape[1]
    def __str__(self):
        return 'quadratic'
    def n_args(self):
        return 1
    def n_in(self,arg):
        return self.A.shape[1]
    def n_out(self):
        return 1
    def eval(self,x):
        return np.dot(x,(self.A.dot(x)))
    def derivative(self,arg,x):
        assert arg == 0
        return (self.A.dot(x) + self.A.T.dot(x))[np.newaxis,:]
    def jvp(self,arg,darg,x):
        return np.dot(x,self.A.dot(darg)) + np.dot(darg,self.A.dot(x))
    def gen_derivative(self,arg,x):
        if len(arg) == 1:
            return self.derivative(arg[0],x)
        elif len(arg) == 2:
            return 2*self.A[np.newaxis,:,:]
        else:
            return 0

class _ADBilinear(ADFunctionInterface):
    def __init__(self,A):
        self.A = A 
        assert len(A.shape) == 2
    def __str__(self):
        return 'bilinear'
    def n_args(self):
        return 2
    def n_in(self,arg):
        return self.A.shape[arg]
    def n_out(self):
        return 1
    def eval(self,x,y):
        return np.dot(x,(self.A.dot(y)))
    def derivative(self,arg,x,y):
        if arg == 0:
            return self.A.dot(y)[np.newaxis,:]
        else:
            return self.A.T.dot(x)[np.newaxis,:]
    def jvp(self,arg,darg,x,y):
        if arg == 0:
            return np.dot(darg,self.A.dot(y))
        else:
            return np.dot(x,self.A.dot(darg))
    def gen_derivative(self,arg,x,y):
        if len(arg) == 1:
            return self.derivative(arg[0],x,y)
        elif len(arg) == 2:
            if arg[0] == arg[1]:
                return 0
            if arg[0] == 0 and arg[1] == 1:
                return self.A[np.newaxis,:,:]
            else:
                return self.A.T[np.newaxis,:,:]
        else:
            return 0
    

class _ADDistanceL2(ADFunctionInterface):
    def __str__(self):
        return 'distance'
    def n_args(self):
        return 2
    def n_in(self,arg):
        return -1
    def n_out(self):
        return 1
    def eval(self,a,b):
        return np.linalg.norm(a-b)
    def derivative(self,arg,a,b):
        if arg==0:
            return _distance_derivative_a(a,b)[np.newaxis,:]
        else:
            return _distance_derivative_a(b,a)[np.newaxis,:]
    def jvp(self,arg,darg,a,b):
        if arg==0:
            return _distance_jvp_a(darg,a,b)
        else:
            return _distance_jvp_a(darg,b,a)
    

class _ADDistanceSquared(ADFunctionInterface):
    def __str__(self):
        return 'distanceSquared'
    def n_args(self):
        return 2
    def n_in(self,arg):
        return -1
    def n_out(self):
        return 1
    def eval(self,*args):
        d = args[0]-args[1]
        return np.dot(d,d)
    def derivative(self,arg,a,b):
        sign = 1 if arg == 0 else -1
        return (2*sign)*(a - b).reshape((1,_size(a)))
    def jvp(self,arg,darg,a,b):
        sign = 1 if arg == 0 else -1
        return (2*sign)*np.dot(a - b,darg)
    def gen_derivative(self,arg,a,b):
        if len(arg) == 1:
            return self.derivative(arg[0],a,b)
        elif len(arg) == 2:
            if arg[0] == arg[1]:
                return 2*np.eye(_size(a))[np.newaxis,:,:]
            else:
                return -2*np.eye(_size(a))[np.newaxis,:,:]
        else:
            return 0


exp = _ADExp()
"""Autodiff'ed function comparable to np.exp.  First derivative is implemented."""

log = _ADLog()
"""Autodiff'ed function comparable to np.log.  First derivative is implemented."""

sqrt = _ADSqrt()
"""Autodiff'ed function comparable to np.sqrt.  First derivative is implemented."""

sin = _ADSin()
"""Autodiff'ed function comparable to np.sin.  All derivatives are implemented."""

cos = _ADCos()
"""Autodiff'ed function comparable to np.cos.  All derivatives are implemented."""

dot = _ADDot()
"""Autodiff'ed function comparable to np.dot.  All derivatives are implemented."""

def linear(A,x):
    """Autodiff'ed function comparable to np.dot(A,x).  A must be a constant, 
    2D np.array.  x may be an expression."""
    return _ADLinear(A)(x)

def quadratric(A,x):
    """Autodiff'ed function comparable to np.dot(x,np.dot(A,x)).  A must be a 
    constant, 2D np.array.  x may be an expression."""
    return _ADQuadratic(A)(x)

def bilinear(x,A,y):
    """Autodiff'ed function comparable to np.dot(x,np.dot(A,y)).  A must be a 
    constant, 2D np.array.  x and y may be expressions."""
    return _ADBilinear(A)(x,y)

def _norm_derivative(x):
    return np.asarray(x)/np.linalg.norm(x)

def _norm_jvp(dx,x):
    return np.dot(x,dx)/np.linalg.norm(x)

def _distance_derivative_a(a,b):
    a = np.asarray(a)
    b = np.asarray(b)
    return (a-b) / np.linalg.norm(a-b)

def _distance_jvp_a(da,a,b):
    a = np.asarray(a)
    b = np.asarray(b)
    return np.dot(a - b,da) / np.linalg.norm(a-b)

norm = function(np.linalg.norm,'norm',[-1],1,
    derivative=[lambda x:_norm_derivative(x)[np.newaxis,:]],
    jvp=[_norm_jvp])
"""Autodiff'ed function comparable to np.linalg.norm.  First derivative is
implemented."""

normSquared = function(lambda x:np.dot(x,x),'normSquared',[-1],1,
    derivative=[lambda x:2*np.asarray(x)[np.newaxis,:]],jvp=[lambda dx,x:2*np.dot(x,dx)],gen_derivative=[lambda x:2*np.eye(len(x))])
"""Autodiff'ed function comparable to np.dot(x,x).  All derivatives are
implemented."""

distance = _ADDistanceL2()
"""Autodiff'ed function comparable to np.linalg.norm(x-y).  First derivative is
implemented."""

distanceSquared = _ADDistanceSquared()
"""Autodiff'ed function comparable to np.dot(x-y,x-y).  All derivatives are
implemented."""

def _unit(x):
    n = np.linalg.norm(x)
    if n > 1e-7:
        return x/n
    return n*0
def _unit_jvp(dx,x):
    n = np.linalg.norm(x)
    dn = _norm_jvp(dx,x)
    if n > 1e-7:
        return dx/n - x*(dn/n**2)
    return np.linalg.norm(dx)
unit = function(_unit,'unit',[-1],-1,
    jvp=[_unit_jvp])
"""Autodiff'ed function comparable to x/norm(x).  First derivative is
implemented."""

cross = function(np.cross,'cross',[-1,-1],-1,
    jvp=[(lambda da,a,b:np.cross(da,b)),(lambda db,a,b:np.cross(a,db))],
    gen_derivative={(0,0):0,(1,1):0},
    order=2)
"""Autodiff'ed function comparable to np.cross(x,y).  First derivative is
implemented.  Some 2nd derivatives are implemented."""

interpolate = function(lambda a,b,u:(1-u)*a+u*b,'interpolate',[-1,-1,1],-1,['a','b','u'],
    jvp=[(lambda da,a,b,u:(1-u)*da),(lambda db,a,b,u:u*db),(lambda du,a,b,u:du*(b-a))],order=2)
"""Autodiff'ed function comparable to (1-u)*a+u*b.  First derivatives is
implemented."""
