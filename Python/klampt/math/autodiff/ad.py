"""Basic definitions for autodifferentiation.

Overview
========

This module defines a simple autodiff system for Klamp't objects, including
3D math, kinematics, dynamics, and geometry functions. This is useful for 
all sorts of optimization functions, e.g., machine learning, calibration,
and trajectory optimization.

This can be used standalone, but is more powerful in conjunction with
PyTorch or CadADi. Use the functions in :mod:`pytorch` and :mod:`casadi`
to convert these Klamp't functions to PyTorch or CasADi functions.

Calling and evaluating functions
================================

When you call an ADFunctionInterface object on some arguments, the function
isn't immediately called. Instead, it creates an expression DAG that can
either be evaluated using ``eval()``, or a derivative can be taken with
``derivative()``, or the expression itself can be passed as an argument to
another ADFunctionInterface.

Many built-in Python numeric functions and some Numpy functions are made
available in autodiff variants. Standard numeric operators like +, -, *, /, **,
abs, and sum will automatically create autodiff-able expressions. 

Array indexing works properly on expressions.  To create larger arrays from
scalars or smaller arrays, you must use the ``ad.stack`` function, you can't
just write ``[expr1,expr2]``.

Variables are named by strings. If you pass a string to a function, it creates
an expression that is dependent on a variable.  It can later be evaluated by
passing the named variables as arguments to ``eval`()``.  For example::

    from klampt.math.autodiff.ad import *
    import numpy as np
    print(mul('x','y').eval(x=2,y=np.array([3.0,4.0]))) #gives the vector [6,8]

Note that if want to use variables in Python operators, you can't simply do
something like ``'x'+'y'`` since Python sees this as string addition. Instead,
you should explicitly declare a variable using the ``var`` keyword:
``var('x') + var('y')``.

Everything evaluates to a scalar or 1D array
============================================

An important note is that all arguments to functions, return values, and
variable  assignments must either be scalars or 1D numpy arrays.

Derivatives of a function are always of shape (size(f(x)),size(x)).

If you are creating your own function that can accept either scalars or
vectors, the ad._size() and ad._scalar() functions are your friends. The size
of a scalar is 1.


Expression DAG and performance
================================

The expression DAG can be nested arbitrarily deeply. For improved performance,
you can reuse the same sub-expression multiple times in a DAG. For example, in 
the following setup::

    diff = var('x')**2 - var('y')**2
    poly = diff**3 - 2.0*diff**2 + 4.0*diff - 6
    poly.eval(x=2.5,y=1.5)

the term ``diff`` will only be evaluated once and then reused through the
evaluation of ``poly``. Similarly, derivatives will be smart in reusing
common subexpressions constructed in this fashion.  For complex subexpressions,
this can be much faster than evaluating the full expression.

Note, however, it is the user's job to construct expressions such that common
subexpressions are the same Python object.  So the following code::

    x = var('x')
    y = var('y')
    poly = (x**2-y**2)**3 - 2.0*(x**2-y**2)**2 + 4.0*(x**2-y**2) - 6
    poly.eval(x=2.5,y=1.5)

doesn't know that the term ``(x**2-y**2)`` is repeated, and instead it will be
recalculated three times.

Caching is only done within a single eval call. If you would like to have
multiple expressions evaluated with a single eval call, you can either put them
in a ``stack`` expression, or use the :func:`eval_multiple` function.

If you print an expression DAG that contains repeated subexpressions, you will 
see terms like #X and @X.  The term #X after a subexpression indicates that 
this subexpression may be reused, and @X indicates an instance of reuse.


Derivatives
===========

To get the derivative of an expression, call :func:`ADFunctionCall.derivative`.
This will use the chain rule to handle arbitrarily nested expressions.  A
derivative is always a 2D array, even if the expression or the argument is
scalar.

Finite differences is performed automatically if some internal function doesn't
have derivative information defined.  To modify the finite difference step size,
modify ``FINITE_DIFFERENCE_RES``.


Creating your own functions
===========================

The easiest way to create your own function is the :func:`function` function.
Just pass a Python function to it, indicate the input and output sizes (for
better error handling), and any available derivatives as keywords.  This will
then automatically generate an ADFunctionInterface instance that can be called
to produce an auto-differentable function.

You can either provide a ``derivative`` function, ``J(arg,x1,...,xn)``, which
returns the Jacobian :math:`\\frac{df}{dx_{arg}}(x1,..,xn)` , or a ``jvp``
function ``Jdx(arg,dx,x1,...,xn)``, which calculates the jacobian-vector
product :math:`\\frac{df}{dx_{arg}}(x1,...,xn)\cdot dx` .  More conveniently,
you can just provide a list of functions ``Ji(x1,...,xn)``, with i=1,..,n,
with ``Ji`` giving :math:`\\frac{df}{dx_i}(x1,...,xn)` , or in the JVP form,
functions ``Jdxi(dx,x1,...,xn)`` each giving
:math:`\\frac{df}{dx_{i}}(x1,...,xn)\cdot dx`.  See the :func:`function`
documentation for more details.

Example::

    from klampt.math.autodiff import ad

    def my_func(x,y):
        return x**2 - y**2

    my_func_ad = ad.function(my_func)   #does no error checking, no derivatives
    my_func_ad2 = ad.function(my_func,'auto',[1,1],1)  #indicates a scalar function of two scalars
    my_func_ad3 = ad.function(my_func,'auto',[1,1],1,
        derivative=[lambda x,y:np.array(2*x).resize((1,1)),
                    lambda x,y:np.array(-2*y).resize((1,1))])  #gives Jacobians
    my_func_ad4 = ad.function(my_func,'auto',[1,1],1,
        jvp=[lambda dx,x,y:2*x*dx, lambda dy,x,y:-2*y*dy])  #gives Jacobian-vector products

    #now you can do a bunch of things...
    print(my_func_ad4(var('x'),var('x')).derivative('x',x=2))  #evaluates the derivative, which is always [[0]]
    print(my_func_ad4(var('x'),2.0).derivative('x',x=2))  #evaluates the derivative, which is 4
    print(my_func_ad(var('x'),2.0).derivative('x',x=2))   #uses finite differencing to approximate the derivative

For more sophisticated functions, such as those that need to be constructed
with some custom user data, you may subclass the :class:`ADFunctionInterface`
interface class. At the very least, you should overload the eval method,
although you will likely also want to overload n_args, n_in, n_out, and
one of the derivative methods.

The same example as above::

    class MyFunc(ADFunctionInterface):
        def __str__(self):
            return 'my_func'
        def n_args(self):
            return 2
        def n_in(self,arg):
            return 1
        def n_out(self):
            return 1
        def eval(self,x,y):
            return x**2 - y**2
        def jvp(self,arg,darg,x,y):
            if arg == 0:  #x
                return 2*x*darg
            else: #y
                return -2*y*darg

    my_func_ad5 = MyFunc()
    print(my_func_ad4(var('x'),var('x')).derivative('x',x=2))  #evaluates the derivative, which is always [[0]]
    print(my_func_ad4(var('x'),2.0).derivative('x',x=2))  #evaluates the derivative, which is 4


If you are going to be taking a lot of derivatives w.r.t. vector variables,
then the most efficient way to implement your functions is to provide the
``derivative`` method.  Otherwise, the most efficient way is to provide the
``jvp`` method.  The reason is that taking the derivative w.r.t. a vector will 
create intermediate matrices, and to perform the chain rule, ``derivative``
will perform a single call and a matrix multiply.  On the other hand, ``jvp``
will be called once for each column in the matrix.  If you are taking
derivatives w.r.t. scalars, ``jvp`` won't create the intermediate Jacobian
matrix, so it will be faster.  Implementing both will let the system figure out
the best alternative.

When you create a new function with its derivatives, you ought to run it
through check_derivatives to see whether they match with finite differences.
To modify the finite difference step size, modify ``FINITE_DIFFERENCE_RES``.


ad Module
=========

This module defines :func:`var`, :func:`function`, and the helper functions
:func:`check_derivatives`, :func:`eval_multiple`, :func:`derivative_multiple`
and :func:`finite_differences`. 

The operators :data:`stack`, :data:`sum_`, :data:`minimum`, and :data:`maximum`
operate somewhat like their Numpy counterparts:

- stack acts like np.stack, since all items are 1D arrays. 
- sum\_ acts like Numpy's sum if only one item is provided.  Otherwise, it
  acts like the builtin sum.
- minimum/maximum act like ndarray.min/max if only one item is provided, and
  otherwise they act like Numpy's minimum/maximum. If more than 2 items are
  provided, they are applied elementwise and sequentially.

The operators add, sub, mul, div, neg, pow\_, abs\_, and getitem are also
provided here, but you will probably just want to use the standard Python 
operators +, -, *, /, -, **, abs, and [].  On arrays, they act just like
the standard numpy functions.

Another convenience function is :func:`setitem`, which has the signature
``setitem(x,indices,v)`` and is equivalent to::

    xcopy = np.copy(x)
    xcopy[indices] = v
    return xcopy

which is sort of a proper functional version of the normal ``x[indices]=v``.

The function :data:`cond` acts like an if statement, where
``cond(pred,posval,negval)`` returns posval if pred > 0 and negval otherwise.
There's also ``cond3(pred,posval,zeroval,negval)`` that returns posval if
pred > 0, zeroval if pred==0, and negval otherwise.  Conditions can also be 
applied when pred is an array, but it must be of equal size to posval and
negval.  The result in this case is an array with the i'th value taking on
posval[i] when pred[i] > 0 and negval[i] otherwise.


Other Klampt ad Modules
=======================

autodiff versions of many Klamp't functions are found in:

- :mod:`math_ad` : basic vector math, including functions from the
  :mod:`~klampt.math.vectorops` module.
- :mod:`so3_ad` : operations on SO(3), mostly compatible with the
  :mod:`~klampt.math.so3` module.
- :mod:`se3_ad` : operations on SE(3), mostly compatible with the
  :mod:`~klampt.math.se3` module.
- :mod:`kinematics_ad` kinematics functions, e.g., robot Jacobians.
- :mod:`geometry_ad` geometry derivative functions.
- :mod:`dynamics_ad` dynamics derivative functions.
- :mod:`trajectory_ad` trajectory derivative functions.

"""

import numpy as np

FINITE_DIFFERENCE_RES = 1e-6
"""The resolution used for finite differences."""

class ADFunctionInterface:
    """The base class for a new auto-differentiable function. 

    The function has the form :math:`f(x_0,...,x_k)` with ``k+1 ==
    self.n_args()``.  All inputs and outputs are either scalars or numpy
    ndarrays.

    To define a new function, you need to implement ``n_args()``,
    ``eval(*args)``, and optionally ``n_in(arg)``, ``n_out()``,
    ``derivative(arg,*args)``, and ``jvp(arg,darg,*args).``

    A function class can be instantiated with non-numeric data.  For example,
    to perform forward kinematics the call::

         wp = WorldPosition(link,localPos)

    (see :mod:`kinematics_ad`) creates a function wp(q) that will accept a 
    robot configuration q and return the world position of the point 
    ``localPos`` on link ``link``.

    This makes these function classes more expressive, but also leads to a 
    slight annoyance in that to define a function that can be called, you need
    to instantiate a function.  For exaple, the sin function is defined in
    :mod:`math_ad` as ``sin = ADSinFunction()`` where ``ADSinFunction`` is
    the actual class that implements the sin function.  This construction lets
    you call ``math_ad.sin(var('x'))`` as though it were a normal function.

    You can also call the convenience routine
    ``fn_name = function(func,'fn_name',argspec,outspec)`` to do
    just this.  See the documentation of :func:`function` for more inf.

    The general convention is that CapitalizedFunctions are function classes
    that must be instantiated, while lowercase_functions are already
    instantiated.
    """

    def __str__(self):
        """Returns a descriptive string for this function"""
        return self.__class__.__name__

    def argname(self,arg):
        """Returns a descriptive string for argument #arg"""
        return "Arg %d"%(arg,)

    def n_args(self):
        """Returns the number of arguments."""
        raise NotImplementedError()

    def n_in(self,arg):
        """Returns the number of entries in argument #arg.  If 1, this can
        either be a 1-D vector or a scalar.  If -1, the function can accept a
        variable sized argument.
        """
        raise NotImplementedError()

    def n_out(self):
        """Returns the number of entries in the output of the function. If -1,
        this can output a variable sized argument.
        """
        raise NotImplementedError()

    def eval(self,*args):
        """Evaluates the application of the function to the given
        (instantiated) arguments. 

        Args:
            args (list): a list of arguments, which are either ndarrays or
                scalars.
        """
        raise NotImplementedError()

    def derivative(self,arg,*args):
        """Returns the Jacobian of the function w.r.t. argument #arg.

        Args:
            arg (int): A value from 0,...,self.n_args()-1 indicating that we
                wish to take :math:`df/dx_{arg}`. 
            args (list of ndarrays): arguments for the function.

        Returns:
            ndarray: A numpy array of shape ``(self.n_out(),self.n_in(arg))``.
            Keep in mind that even if the argument or result is a scalar, this
            needs to be a 2D array.

            If the derivative is not implemented, raise a NotImplementedError.

            If the derivative is zero, can just return 0 (the integer)
            regardless of the size of the result.
        """
        raise NotImplementedError()

    def gen_derivative(self,arg,*args):
        """Generalized derivative that allows higher-order derivatives to be
        taken.

        Args:
            arg (list): Indicates the order of derivatives to be taken.  For
                example, to take the 2nd derivative w.r.t. x0,x1,
                arg = [0,1] should be specified.
            args (list of ndarrays): arguments for the function.

        Returns:
            ndarray: A tensor of shape ``(n_out,n_in(arg[0]),...,
            n_in(arg[-1])``.

            If the generalized derivative is not implemented, raise a
            NotImplementedError.

            If the generalized derivative is zero, can just return 0 (the
            integer) regardless of the size of the result.
        """
        raise NotImplementedError()


    def jvp(self,arg,darg,*args):
        """Performs a Jacobian-vector product for argument #arg.

        Args:
            arg (int): A value from 0,...,self.n_args()-1 indicating that we
                wish to calculate df/dx_arg * darg. 
            darg (ndarray): the derivative of x_arg w.r.t some other parameter.
                Must have ``darg.shape = (self.n_in(arg),)``. 
            args (list of ndarrays): arguments for the function.

        Returns:
            ndarray: A numpy array of length ``self.n_out()``

            If the derivative is not implemented, raise a NotImplementedError.
        """
        raise NotImplementedError()

    def __call__(self,*args):
        try:
            na = self.n_args()
            if na >= 0 and len(args) != na:
                raise ValueError("Invalid number of arguments to function %s, wanted %d, got %d"%(str(self),na,len(args)))
        except NotImplementedError:
            pass
        return ADFunctionCall(self,args)


class ADTerminal:
    """A symbolic value that will be bound later.

    Consider using :func:`var` rather than calling this explicitly.
    """
    def __init__(self,name):
        self.name = name
    def __str__(self):
        return self.name
    def __neg__(self):
        return neg(self)
    def __add__(self,rhs):
        return add(self,rhs)
    def __sub__(self,rhs):
        return sub(self,rhs)
    def __mul__(self,rhs):
        return mul(self,rhs)
    def __div__(self,rhs):
        return div(self,rhs)
    def __pow__(self,rhs):
        return pow_(self,rhs)
    def __truediv__(self,rhs):
        return div(self,rhs)
    def __pow__(self,rhs):
        return pow_(self,rhs)
    def __radd__(self,lhs):
        return add(lhs,self)
    def __rsub__(self,lhs):
        return sub(lhs,self)
    def __rmul__(self,lhs):
        return mul(lhs,self)
    def __rdiv__(self,lhs):
        return div(lhs,self)
    def __rtruediv__(self,lhs):
        return div(lhs,self)
    def __rpow__(self,lhs):
        return pow_(lhs,self)
    def __getitem__(self,index):
        return getitem(self,index)
    def __abs__(self):
        return abs_(self)


class ADFunctionCall:
    """A function call in an expression tree."""
    def __init__(self,func,args):
        self.func = func
        self.args = [a for a in args]
        for i,arg in enumerate(args):
            if isinstance(arg,str):
                self.args[i] = ADTerminal(arg)
            elif isinstance(arg,ADTerminal):
                pass
            elif isinstance(arg,ADFunctionCall):
                try:
                    narg = arg.func.n_out()
                    nin = func.n_in(i)
                    if nin >= 0 and narg >= 0 and nin != narg:
                        raise ValueError("Invalid size of argument %s=%s to function %s: %d != %d"%(func.argname(i),str(arg),str(func),narg,nin))
                except NotImplementedError:
                    #argument sizes not specified, ignore
                    pass
            elif isinstance(arg,ADFunctionInterface):
                raise ValueError("Can't pass a ADFunctionInterface directly as an argument")
            else:
                #a constant
                try:
                    nin = func.n_in(i)
                    if nin >= 0:
                        if _size(arg) != nin:
                            raise ValueError("Function %s expected %s to have size %d, instead got %d",self.func.name,func.argname(i),nin,_size(arg))
                except NotImplementedError:
                    #argument sizes not specified, ignore
                    pass
                pass
        self._cache = {}

    def __str__(self):
        terms = {}
        dag = self._print_dag(terms)
        self._clear_cache('print_id')
        def _flatten_str_tree(tree):
            flat = []
            for v in tree:
                if isinstance(v,list):
                    flat.append(_flatten_str_tree(v))
                else:
                    flat.append(v)
            return ''.join(flat)
        return _flatten_str_tree(dag)

    def eval(self,**kwargs):
        """Evaluate a function call with bound values given by the keyword
        arguments.  Example::

            print((3*var('x')**2 - 2*var('x')).eval(x=1.5))

        """
        self._clear_cache('eval_result','eval_context','eval_instantiated_args')
        return self._eval([],kwargs)

    def derivative(self,arg,**kwargs):
        """Evaluates the derivative of this computation graph with respect to
        the named variable at the given setings in kwargs.

        If any derivatives are not defined, finite differences will be used
        with step size FINITE_DIFFERENCE_RES.

        If the prior eval() call wasn't with the same arguments, then this will
        call eval() again with kwargs.  So, the sequence ``eval(**kwargs),
        derivative(**kwargs)`` will save some time compared to changing the
        args.
        """
        if 'eval_context' not in self._cache or not _context_equal(self._cache['eval_context'],kwargs):
            self.eval(**kwargs)
            assert _context_equal(self._cache['eval_context'],kwargs)
        self._clear_cache('deriv_result')
        res = self._deriv([],arg,kwargs)
        if res is 0:
            return np.zeros((_size(self._cache['eval_result']),_size(kwargs[arg])))
        if len(res.shape) == 1:
            #deriv w.r.t. scalar
            res = res.reshape((res.shape[0],1))
        return res

    def gen_derivative(self,args,**kwargs):
        """Evaluates the generalized derivative of this computation graph with 
        respect to the named variables in args, at the given settings in
        kwargs.

        Examples:

        - ``(var('x')**3).gen_derivative(['x','x'])`` returns the
           second derivative of :math:`x^3` with respect to x. 
        - ``var('x')**2*var('y').gen_derivative(['x','y'])`` returns
           :math:`\\frac{d^2}{dx dy} x^2 y`.

        If args is [], this is equivalent to ``self.eval(**kwargs)``.

        If args has only one element, this is equivalent to
        ``self.derivative(args[0],**kwargs)``.

        Args:
            args (list of str): the denominators of the derivative to be
                evaluated.
            kwargs (dict): the values at which the derivative is evaluated.

        Result:
            ndarray: A tensor of shape
            ``(_size(this),_size(args[0]),...,_size(args[-1]))`` containing
            the derivatives.
        """
        if len(args)==0:
            return self.eval(**kwargs)
        elif len(args)==1:
            return self.derivative(args[0],**kwargs)
        elif len(args)==2:
            if 'eval_context' not in self._cache or not _context_equal(self._cache['eval_context'],kwargs):
                self.eval(**kwargs)
                assert _context_equal(self._cache['eval_context'],kwargs)
            self._clear_cache('hessian_result','deriv_darg1','deriv_darg2')
            self.derivative(args[0],**kwargs)
            self._move_cache('deriv_result','deriv_darg1')
            self.derivative(args[1],**kwargs)
            self._move_cache('deriv_result','deriv_darg2')
            res = self._hessian([],args[0],args[1],kwargs)
            hess_shape = (_size(self._cache['eval_result']),)+tuple(_size(kwargs[arg]) for arg in args)
            if res is 0:
                return np.zeros(hess_shape)
            return res
        else:
            raise NotImplementedError("TODO: higher-order derivatives")

    def replace(self,**kwargs):
        """Replaces any symbols whose names are keys in kwargs with the
        matching value as an expression.  The result is an ADFunctionCall.

        For example, ``(var('x')**2 + var('y')).replace(x=var('z'),y=3)`` would
        return the expression ``var('z')**2 + 3``.
        """
        res = self._replace(kwargs)
        self._clear_cache('replace_result')
        return res

    def terminals(self,sort=False):
        """Returns the list of terminals in this expression.  If sort=True,
        they are sorted by alphabetical order.

        Returns:
            list of str
        """
        res = []
        resset = set()
        self._terminals(res,resset)
        self._clear_cache('terminals_visited')
        if sort:
            return sorted(res)
        return res

    def _print_dag(self,terms):
        assert 'print_id' not in self._cache
        id = len(terms)
        self._cache['print_id'] = id
        argstr = []
        for v in self.args:
            if isinstance(v,ADTerminal):
                argstr.append(str(v))
            elif isinstance(v,ADFunctionCall):
                if 'print_id' in v._cache:
                    vid = v._cache['print_id']
                    argstr.append('@'+str(vid))
                    if not terms[vid][-1].startswith('#'):
                        terms[vid].append('#'+str(vid))
                else:
                    argstr.append(v._print_dag(terms))
            else:
                argstr.append(str(v))
            argstr.append(',')
        argstr.pop(-1)
        res = [str(self.func),'(',argstr,')']
        terms[id] = res
        return res

    def _eval(self,callStack,kwargs):
        if 'eval_result' in self._cache:
            return self._cache['eval_result']
        self._cache['eval_context'] = kwargs
        instantiated_args = []
        callStack.append(self)
        for i,arg in enumerate(self.args):
            if isinstance(arg,ADTerminal):
                if arg.name not in kwargs:
                    raise ValueError("Function argument "+arg.name+" not present in arguments")
                instantiated_args.append(kwargs[arg.name])
                if np.ndim(instantiated_args[-1]) > 1:
                    raise ValueError("Terminal",arg.name,"is an array with more than 1D? Can only support 1D arrays")
            elif isinstance(arg,ADFunctionCall):
                instantiated_args.append(arg._eval(callStack,kwargs))
                if np.ndim(instantiated_args[-1]) > 1:
                    raise ValueError("Function",arg.func.name,"returned array with more than 1D? Can only support 1D arrays")
            else:
                instantiated_args.append(arg)
            try:
                ni = self.func.n_in(i)
                if ni >= 0:
                    if _size(instantiated_args[-1]) != ni:
                        raise ValueError("Error evaluating %s: argument %d has expected size %d, instead got %d"%(str(self),i,ni,_size(instantiated_args[-1])))
            except NotImplementedError:
                #not implemented, just ignore
                pass

        callStack.pop(-1)
        self._cache['eval_instantiated_args'] = instantiated_args
        try:
            res = self.func.eval(*instantiated_args)
        except Exception:
            context = [str(s) for s in callStack]
            raise RuntimeError("Error running function %s in context %s"%(str(self.func),' -> '.join(context)))
        try:
            no = self.func.n_out()
            if not isinstance(no,int):
                raise ValueError("Error evaluating %s: return # of outputs is not integer? got %s"%(str(self),str(no)))
            if no >= 0:
                if _size(res) != no:
                    raise ValueError("Error evaluating %s: return value has expected size %d, instead got %d"%(str(self),no,_size(res)))
        except NotImplementedError:
            #not implemented, just ignore
            pass
        self._cache['eval_result'] = res
        return res

    def _deriv_jacobian(self,arg,args):
        n = _size(args[arg])
        no = -1
        try:
            no = self.func.n_out()
        except NotImplementedError:
            pass
        try:
            df = self.func.derivative(arg,*args)
            if len(df.shape) != 2:
                raise ValueError("Function %s.derivative doesn't return a 2D array?  Context is %s, result has shape %s"%(str(self.func),str(self),df.shape))
            if no >= 0 and df.shape != (no,n):
                raise ValueError("Function %s.derivative has wrong size?  Context is %s, expects %s, result has shape %s"%(str(self.func),str(self),(no,n),df.shape))
            return df
        except NotImplementedError:
            pass
        try:
            df = 0
            temp = np.zeros(n)
            for col in range(n):
                temp[col] = 1
                dcol = self.func.jvp(arg,temp,*args)
                if df is 0:
                    df = np.empty((_size(dcol),n))
                if no >= 0 and _size(dcol) != no:
                    raise ValueError("Function %s.jvp result has wrong size? Context is %s, expects %d, result has size %d"%(str(self.func),str(self),no,_size(dcol)))
                df[:,col] = dcol
                temp[col] = 0
            return df
        except NotImplementedError:
            pass
        return finite_differences(lambda x:self.func.eval(*(args[:arg]+[x]+args[arg+1:])),args[arg],FINITE_DIFFERENCE_RES)

    def _deriv_jacobian_array_product(self,arg,mat,args):
        #assumes eval was already called
        n = _size(args[arg])
        no = -1
        try:
            no = self.func.n_out()
        except NotImplementedError:
            pass
        if len(mat.shape) == 1 or mat.shape[1] == 1:
            #try 1-D jvp first, then derivative, then finite difference
            if len(mat.shape) == 2:
                mat = mat[:,0]
            if mat.shape[0] != n:
                raise ValueError("1D derivative of arg %s of function %s expected size %d, got %d. Context is %s"%(self.func.argname(arg),str(self.func),n,mat.shape[0],str(self)))
            try:
                res = self.func.jvp(arg,mat,*args)
                if _scalar(res):
                    #for jvps, if the result is a scalar this needs to be resized to a 1d array
                    return np.asarray(res).reshape((1,))
                if len(res.shape) != 1:
                    raise ValueError("Function %s.jvp doesn't return a 1D array or scalar?  Context is %s, result has shape %s"%(str(self.func),str(self),res.shape))
                if no >= 0 and res.shape[0] != no:
                    raise ValueError("Function %s.jvp result has wrong size? Context is %s, expects %d, result has size %d"%(str(self.func),str(self),no,res.shape[0]))
                return res
            except NotImplementedError:
                pass
            try:
                df = self.func.derivative(arg,*args)
                if len(df.shape) != 2:
                    raise ValueError("Function %s.derivative doesn't return a 2D array?  Context is %s, result has shape %s"%(str(self.func),str(self),df.shape))
                if no >= 0 and df.shape != (no,n):
                    raise ValueError("Function %s.derivative has wrong size?  Context is %s, expects %s, result has shape %s"%(str(self.func),str(self),(no,n),df.shape))
                return np.dot(df,mat)
            except NotImplementedError:
                pass
            return finite_differences(lambda t:self.func.eval(*(args[:arg]+[args[arg]+t*mat]+args[arg+1:])),0,FINITE_DIFFERENCE_RES)
        else:
            #try derivative * mat first, then directional jvp, then fd * mat
            try:
                df = self.func.derivative(arg,*args)
                if len(df.shape) != 2:
                    raise ValueError("Function %s.derivative doesn't return a 2D array?  Context is %s, result has shape %s"%(str(self.func),str(self),df.shape))
                return np.dot(df,mat)
            except NotImplementedError:
                pass
            try:
                columns = []
                if mat.shape[0] != n:
                    raise ValueError("Derivative of arg %s of function %s expected size %d, got %d. Context is %s"%(self.func.argname(arg),str(self.func),n,mat.shape[0],str(self)))
                for col in range(mat.shape[1]):
                    columns.append(self.func.jvp(arg,mat[:,col],*args))
                    if no >= 0 and _size(columns[-1]) != no:
                        raise ValueError("Function %s.jvp result has wrong size? Context is %s, expects %d, result has size %d"%(str(self.func),str(self),no,_size(columns[-1])))
                return np.column_stack(columns)
            except NotImplementedError:
                pass
            return np.dot(finite_differences(lambda x:self.func.eval(*(args[:arg]+[x]+args[arg+1:])),args[arg],FINITE_DIFFERENCE_RES),mat)

    def _deriv(self,callStack,deriv_arg,kwargs):
        if 'deriv_result' in self._cache:
            return self._cache['deriv_result']
        assert 'eval_context' in self._cache
        eval_result = self._cache['eval_result']
        eval_args = self._cache['eval_instantiated_args']
        callStack.append(self)
        res = 0
        for i,arg in enumerate(self.args):
            if isinstance(arg,ADTerminal):
                if arg.name == deriv_arg:
                    inc = self._deriv_jacobian(i,eval_args)
                    assert hasattr(inc,'shape') and inc.shape[0] == _size(eval_result),"Function %s has invalid jacobian shape? Context %s"%(str(self.func),str(self))
                    if _scalar(eval_args[i]):
                        #for scalar derivatives, we just use a 1D vector rather than a 2D array
                        inc = inc[:,0]
                else:
                    #not deriv_arg, no derivative
                    continue
            elif isinstance(arg,ADFunctionCall):
                da = arg._deriv(callStack,deriv_arg,kwargs)
                if da is 0:
                    continue
                inc = self._deriv_jacobian_array_product(i,da,eval_args)
                assert hasattr(inc,'shape') and inc.shape[0] == _size(eval_result),"Function %s has invalid jacobian shape? Context %s"%(str(self.func),str(self))
            else:
                #const, no derivative
                continue
            if res is 0:
                res = inc
            else:
                res += inc
        callStack.pop(-1)
        self._cache['deriv_result'] = res
        return res

    def _hessian(self,callStack,darg1,darg2,kwargs):
        #d/dv f(x1,...,xn) = sum_{i=1...,n} d/dxi f(...)* dxi/dv
        #d/du d/dv f(x1,...,xn) = sum_{i=1...,n} (sum_{j=1...,n} (d^2/dxixj) f(...)* dxi/dv * dxj/du) + d/dxi f(...) (d^2 / duv) xi
        if 'hessian_result' in self._cache:
            return self._cache['hessian_result']
        assert 'eval_context' in self._cache
        assert 'deriv_darg1' in self._cache
        assert 'deriv_darg2' in self._cache
        eval_result = self._cache['eval_result']
        eval_args = self._cache['eval_instantiated_args']
        callStack.append(self)
        res = 0
        #add d/dxi f * (d^2/duv) xi to res
        for i,arg in enumerate(self.args):
            if isinstance(arg,ADTerminal):
                #no second derivative of a constant
                continue
            elif isinstance(arg,ADFunctionCall):
                da = arg._hessian(callStack,darg1,darg2,kwargs)
                if da is 0:
                    continue
                inc = np.empty((_size(eval_result),_size(kwargs[darg1]),_size(kwargs[darg2])))
                for i in range(da.shape[-1]):
                    inc[:,:,i] = self._deriv_jacobian_array_product(i,da[:,:,i],eval_args)
                print("Deriv * hessian",arg," = ",inc)
            else:
                #const, no derivative
                continue
            if res is 0:
                res = inc
            else:
                res += inc
        #add (d^2/dxixj) f(...)* dxi/dv * dxj/du
        for i,arg in enumerate(self.args):
            if isinstance(arg,ADTerminal):
                if arg.name != darg1:
                    continue
            elif isinstance(arg,ADFunctionCall):
                if arg._cache['deriv_arg1'] is 0:
                    continue
            else:
                #constant
                continue
            for j,arg2 in enumerate(self.args):
                inc = 0
                if isinstance(arg2,ADTerminal):
                    if arg2.name != darg2:
                        continue
                elif isinstance(arg2,ADFunctionCall):
                    if arg2._cache['deriv_arg2'] is 0:
                        continue
                else:
                    #constant
                    continue
                try:
                    ddf = self.func.gen_derivative([i,j],*eval_args)
                except NotImplementedError:
                    return finite_differences()
                #print("d^2/d",i,j,"of func",self.func,"is",ddf)
                if ddf is 0:
                    continue
                if isinstance(arg,ADTerminal):
                    if isinstance(arg2,ADTerminal):
                        #print("d^2/d",i,j,"of terminal",arg.name,"and terminal",arg2.name)
                        inc = ddf
                    elif isinstance(arg2,ADFunctionCall):
                        #print("d^2/d",darg1,darg2," of terminal",arg.name,"and arg",j)
                        inc = np.tensordot(ddf,(2,0),arg2._cache['deriv_arg2'])
                elif isinstance(arg,ADFunctionCall):
                    if isinstance(arg2,ADTerminal):
                        #print("d^2/d",i,j," of arg",i,"and terminal",arg2.name)
                        inc = np.tensordot(ddf,(1,0),arg._cache['deriv_arg1'])
                    elif isinstance(arg2,ADFunctionCall):
                        #print("d^2/d",darg1,darg2," of arg",i," and arg",j)
                        inc = np.tensordot(np.tensordot(ddf,(1,0),arg._cache['deriv_arg1']),(1,0),arg._cache['deriv_arg2'])
                print("hessian",arg,"of func",self.func,"*d^2/dx",i,"dx",j," = ",inc)
                if res is 0:
                    res = inc
                else:
                    res += inc
        callStack.pop(-1)
        self._cache['hessian_result'] = res
        return res            

    def _replace(self,kwargs):
        if 'replace_result' in self._cache:
            return self._cache['replace_result']
        replaced = False
        newargs = []
        for arg in self.args:
            if isinstance(arg,ADTerminal):
                if arg.name in kwargs:
                    newargs.append(kwargs[arg.name])
                    replaced = True
                else:
                    newargs.append(arg)
            elif isinstance(arg,ADFunctionCall):
                newargs.append(arg._replace(kwargs))
                if newargs[-1] is not arg:
                    replaced = True
            else:
                newargs.append(arg)
        if replaced:
            res = ADFunctionCall(self.func,newargs)
        else:
            #circular reference... can we make sure this won't mess up the GC?
            res = self
        self._cache['replace_result'] = res
        return res

    def _terminals(self,res,resset):
        if 'terminals_visited' in self._cache:
            return
        for arg in self.args:
            if isinstance(arg,ADTerminal):
                if arg.name not in resset:
                    res.append(arg.name)
                    resset.add(arg.name)
            elif isinstance(arg,ADFunctionCall):
                arg._terminals(res,resset)
        self._cache['terminals_visited'] = True
        return

    def _clear_cache(self,*args):
        if args[0] not in self._cache:
            return
        for a in args:
            del self._cache[a]
        for v in self.args:
            if isinstance(v,ADFunctionCall):
                v._clear_cache(*args)

    def _move_cache(self,src,dest):
        if src not in self._cache:
            return
        self._cache[dest] = self._cache[src]
        del self._cache[src]
        for v in self.args:
            if isinstance(v,ADFunctionCall):
                v._move_cache(src,dest)

    def __neg__(self):
        return neg(self)
    def __add__(self,rhs):
        return add(self,rhs)
    def __sub__(self,rhs):
        return sub(self,rhs)
    def __mul__(self,rhs):
        return mul(self,rhs)
    def __div__(self,rhs):
        return div(self,rhs)
    def __pow__(self,rhs):
        return pow_(self,rhs)
    def __truediv__(self,rhs):
        return div(self,rhs)
    def __pow__(self,rhs):
        return pow_(self,rhs)
    def __radd__(self,lhs):
        return add(lhs,self)
    def __rsub__(self,lhs):
        return sub(lhs,self)
    def __rmul__(self,lhs):
        return mul(lhs,self)
    def __rdiv__(self,lhs):
        return div(lhs,self)
    def __rtruediv__(self,lhs):
        return div(lhs,self)
    def __rpow__(self,lhs):
        return pow_(lhs,self)
    def __getitem__(self,index):
        return getitem(self,index)
    def __abs__(self):
        return abs_(self)


def var(x):
    """Creates a new variable symbol with name x."""
    return ADTerminal(x)


def function(func,name='auto',argspec='auto',outspec=None,argnames=None,
    derivative=None,jvp=None,gen_derivative=None,order=None):
    """Declares and instantiates a ADFunctionInterface object that calls the 
    Python function func.

    Args:
        func (callable): a Python function.
        name (str, optional): the string that the ADFunctionInterface prints.
        argspec (list, optional): a list of argument sizes, or 'auto' to auto-
            set the argument list from the function signature.
        outspec (int, optional): a size of the output.
        argnames
        derivative (callable or list of callables, optional): one or more
            derivative functions.  If a callable, takes the index-arguments
            tuple ``(arg,*args)``.  If a list, each element just takes the same
            arguments as func.
        jvp (callable or list of callables, optional): one or more Jacobian-
            vector product functions. If a single callable, has signature
            ``(arg,darg,*args)``.  If a list, each element has the signature
            ``(darg,*args)``.
        gen_derivative (callable, list of callables, or dict, optional): a
            higher-order derivative function. 

            If a callable is given, it has standard signature (arg,*args). 

            If a list of callables is given, they are a list of higher-order
            derivatives, starting at the second derivative.  In this case,
            the function may only have 1 argument, and each callable has the
            signature ``(*args)``.

            If a dict is given, it must map tuples to callables, each of which
            has the signature ``(*args)``.
        order (int, optional): if provided, we know that all derivatives higher
            than this order will be zero.
    """
    if name == 'auto':
        import inspect
        for x in inspect.getmembers(func):
            if x[0] == '__name__':
                name = x[1]
                break
        assert name != None,"Could not get the name of the function?"
    if argspec == 'auto':
        import inspect
        (argnames,varargs,keywords,defaults) = inspect.getargspec(func)
        if varargs != None or keywords != None:
            #variable number of arguments
            raise ValueError("Can't handle variable # of arguments yet")
        else:
            argspec = [-1]*len(argnames)
    if argnames is not None:
        if len(argnames) != len(argspec):
            raise ValueError("Invalid number of argument names provided")
    def run_if_not_none(f,*args):
        if f is None:
            raise NotImplementedError()
        else:
            return f(*args)
    if hasattr(derivative,'__iter__'):
        derivative_list = derivative
        derivative = lambda arg,*args:run_if_not_none(derivative_list[arg],*args)
    if hasattr(jvp,'__iter__'):
        jvp_list = jvp
        jvp = lambda arg,darg,*args:run_if_not_none(jvp_list[arg],darg,*args)
    if gen_derivative is None and order is not None:
        def default_gen_deriv(arg,*args):
            if len(arg) > order:
                return 0
            if len(arg) == 1:
                return derivative(arg[0],*args)
            raise NotImplementedError()
        gen_derivative = default_gen_deriv
    if isinstance(gen_derivative,dict):
        gen_derivative_dict = gen_derivative
        def dict_gen_deriv(arg,*args):
            if len(arg) == 1:
                return derivative(arg[0],*args)
            if tuple(arg) in gen_derivative_dict:
                return gen_derivative_dict(tuple(arg),*args)
            if order is not None and len(tuple) > order:
                return 0
            raise NotImplementedError()
        gen_derivative = dict_gen_deriv
    elif hasattr(gen_derivative,'__iter__'):
        if len(argspec) > 1:
            raise ValueError("Cannot accept a list of higher-order derivatives except for a single-argument function")
        gen_derivative_list = gen_derivative
        def list_gen_deriv(arg,*args):
            if len(arg) == 1:
                return derivative(arg[0],*args)
            if len(arg)-2 < len(gen_derivative_list):
                if gen_derivative_list[len(arg)-1] is None: 
                    raise NotImplementedError()
                return gen_derivative_list[len(arg)-1](*args)
            if order is not None and len(tuple) > order:
                return 0
            raise NotImplementedError()
    members = {}
    members['__str__'] = lambda self:name
    members['eval'] = lambda self,*args:np.asarray(func(*args))
    members['n_args'] = lambda self:len(argspec)
    members['n_in'] = lambda self,arg:argspec[arg]
    if outspec is not None:
        members['n_out'] = lambda self:outspec
    def toarray(res):
        return res if _scalar(res) else np.asarray(res)
    if derivative is not None:
        members['derivative'] = lambda self,arg,*args:toarray(derivative(arg,*args))
    if jvp is not None:
        members['jvp'] = lambda self,arg,darg,*args:toarray(jvp(arg,darg,*args))
    if gen_derivative is not None:
        members['gen_derivative'] = lambda self,arg,*args:toarray(gen_derivative(arg,*args))
    if argnames is not None:
        members['argname'] = lambda self,arg:argnames[arg]
    TempType = type("_ad_"+name,(ADFunctionInterface,),members)
    return TempType()


def finite_differences(f,x,h=1e-6):
    """Performs forward differences to approximate the derivative of f at x.
    f can be a vector or scalar function, and x can be a scalar or vector.

    The result is a Numpy array of shape ``(_size(f(x)),_size(x))``.

    h is the step size.
    """
    f0 = f(x)
    g = np.empty((_size(x),_size(f0)))
    if hasattr(x,'__iter__'):
        temp = x.astype(float,copy=True)        #really frustrating bug: if x is an integral type, then adding h doesn't do anything
        for i in range(len(x)):
            temp[i] += h
            g[i] = f(temp)-f0
            temp[i] = x[i]
    else:
        g[0,:] = f(x+h) - f0
    g *= 1.0/h
    return g.T


def finite_differences_hessian(f,x,h):
    """Performs forward differences to approximate the Hessian of f(x) w.r.t.
    x.
    """
    f0 = f(x)
    g = np.empty((_size(f0),_size(x),_size(x)))
    if hasattr(x,'__iter__'):
        xtemp = x.astype(float,copy=True)
        fx = []
        for i in range(len(x)):
            xtemp[i] += h
            fx.append(np.copy(f(xtemp)))
            xtemp[i] = x[i]
        for i in range(len(x)):
            xtemp[i] -= h
            g[:,i,i] = (fx[i] - 2*f0 + f(xtemp))
            xtemp[i] = x[i]
            xtemp[i] += h
            for j in range(i):
                xtemp[j] += h
                g[:,j,i] = g[:,i,j] = (f(xtemp) - fx[i]) - (fx[j] - f0)
                xtemp[j] = x[j]
            xtemp[i] = x[i]
    else:
        fx = f(x+h)
        g[:,0,0] = (fx - 2*f0 + f(x-h))
    return g/h**2


def finite_differences_hessian2(f,x,y,hx,hy=None):
    """Performs forward differences to approximate the Hessian of f(x,y) w.r.t.
    x and y.
    """
    if hy is None:
        hy = hx
    f0 = f(x,y)
    g = np.empty((_size(f0),_size(x),_size(y)))
    if hasattr(x,'__iter__'):
        xtemp = x.astype(float,copy=True)
        fx = []
        for i in range(len(x)):
            xtemp[i] += hx
            fx.append(np.copy(f(xtemp,y)))
            xtemp[i] = x[i]
        
    else:
        fx = f(x+hx,y)

    if hasattr(y,'__iter__'):
        fy = []
        ytemp = y.astype(float,copy=True)
        for j in range(len(y)):
            ytemp[j] += hy
            fy.append(np.copy(f(x,ytemp)))
            ytemp[j] = y[j]
    else:
        fy = f(x,y+hy)

    if hasattr(x,'__iter__'):
        if hasattr(y,'__iter__'):
            for i in range(len(x)):
                xtemp[i] += hx
                for j in range(len(y)):
                    ytemp[j] += hy
                    g[:,i,j] = (f(xtemp,ytemp)-fx[i]) - (fy[j] - f0)
                    ytemp[j] = y[j]
                xtemp[i] = x[i]
        else:
            for i in range(len(x)):
                xtemp[i] += hx
                g[:,i,0] = (f(xtemp,y+hy)-fx[i]) - (fy - f0)
                xtemp[i] = x[i]
    else:
        if hasattr(y,'__iter__'):
            for j in range(len(y)):
                ytemp[j] += hy
                g[:,0,j] = (f(x+hy,ytemp)-fx) - (fy[j] - f0)
                ytemp[j] = y[j]
        else:
            g[:,0,0] = (f(x+hx,y+hy)-fx) - (fy - f0)
    g *= 1.0/(hx*hy)
    return g


def check_derivatives(f,x,h=1e-6,rtol=1e-2,atol=1e-3):
    """Checks the derivatives of a ADFunctionCall or an ADFunctionInterface.

    If f is an ADFunctionCall, the derivatives of all terminal values are
    checked, and x must be a dict matching terminal values to their settings.
    Example::

        f = var('x')+var('y')**2+10
        check_derivatives(f,{'x':3,'y':2})

    If f is an ADFunctionInterface, x is list of the function's arguments.
    All derivative and jvp functions are tested.

    The derivatives must match the finite differences result by relative
    tolerance rtol and absolute tolerance atol (see np.allclose for details).
    Otherwise, an AssertionError is raised.
    """
    if isinstance(f,ADFunctionCall):
        #check w.r.t. all arguments
        if not isinstance(x,dict):
            raise ValueError("If a function call is provided to check_derivatives, then x must be a dict of keyword arguments")
        if len(x)==0:
            #no derivatives
            return True
        def replace_eval(y,arg):
            oldx = x[arg]
            x[arg] = y
            res = f.eval(**x)
            x[arg] = oldx
            return res
        for k,v in x.items():
            try:
                g = f.derivative(k,**x)
            except NotImplementedError:
                continue
            g_fd = finite_differences(lambda y:replace_eval(y,k),v,h)
            if g is 0:
                g = np.zeros(g_fd.shape)
            elif g.shape != g_fd.shape:
                raise AssertionError("derivative of %s w.r.t. %s did not produce the correct shape: %s vs %s"%(str(f),k,g.shape,g_fd.shape))
            if not np.allclose(g_fd,g,rtol,atol):
                print("check_derivative",f,"failed with args",x,"@ argument",k)
                print("Finite differences:",g_fd)
                print("Derivative:",g)
                raise AssertionError("derivative of %s w.r.t. %s has an error of size %f"%(str(f.func),k,np.linalg.norm(g-g_fd)))
    elif isinstance(f,ADFunctionInterface):
        assert f.n_args() < 0 or f.n_args() == len(x),"Invalid number of arguments provided to %s: %d != %d"%(str(f),len(x),f.n_args())
        has_gen_derivative = True
        for i,v in enumerate(x):
            has_derivative = False
            try:
                g = f.derivative(i,*x)
                has_derivative = True
                g_fd = finite_differences(lambda y:f.eval(*(x[:i]+[y]+x[i+1:])),v,h)
                if g is 0:
                    g = np.zeros(g_fd.shape)
                elif g.shape != g_fd.shape:
                    raise AssertionError("derivative of %s w.r.t. %s did not produce the correct shape: %s vs %s"%(str(f),f.argname(i),g.shape,g_fd.shape))
                if not np.allclose(g_fd,g,rtol,atol):
                    print("check_derivative",f,"failed with args",x,"@ argument",i)
                    print("Finite differences:",g_fd)
                    print("Derivative:",g)
                    raise AssertionError("derivative of %s w.r.t. %s has an error of size %f"%(str(f),f.argname(i),np.linalg.norm(g-g_fd)))
            except NotImplementedError:
                pass
            
            has_jvp = False
            try:
                for j in range(_size(v)):
                    dv = np.zeros(_size(v))
                    dv[j] = 1
                    g = f.jvp(i,dv,*x)
                    has_jvp = True
                    if _scalar(v):
                        dv = dv[0]
                    g_fd = finite_differences(lambda t:f.eval(*(x[:i]+[v+dv*t]+x[i+1:])),0,h)[:,0]
                    if g is 0:
                        g = np.zeros(g_fd.shape)
                    if _scalar(g):
                        g = np.array([g])
                    elif g.shape != g_fd.shape:
                        raise AssertionError("Jacobian-vector product of %s w.r.t. %s did not produce the correct shape: %s vs %s"%(str(f),f.argname(i),g.shape,g_fd.shape))
                    if not np.allclose(g_fd,g,rtol,atol):
                        print("check_derivative",f,"failed with args",x,"@ argument",i)
                        print("Chosen direction",dv)
                        print("jvp Finite differences:",g_fd)
                        print("jvp:",g)
                        raise AssertionError("Jacobian-vector product of %s w.r.t. %s has an error of size %f"%(str(f),f.argname(i),np.linalg.norm(g-g_fd)))
                for j in range(_size(v)):
                    #test with scaling
                    dv = np.zeros(_size(v))
                    dv[j] = np.random.uniform(-0.9,-0.05)
                    g = f.jvp(i,dv,*x)
                    has_jvp = True
                    if _scalar(v):
                        dv = dv[0]
                    g_fd = finite_differences(lambda t:f.eval(*(x[:i]+[v+dv*t]+x[i+1:])),0,h)[:,0]
                    if g is 0:
                        g = np.zeros(g_fd.shape)
                    if _scalar(g):
                        g = np.array([g])
                    elif g.shape != g_fd.shape:
                        raise AssertionError("Jacobian-vector product of %s w.r.t. %s did not produce the correct shape: %s vs %s"%(str(f),f.argname(i),g.shape,g_fd.shape))
                    if not np.allclose(g_fd,g,rtol,atol):
                        print("check_derivative",f,"failed with args",x,"@ argument",i)
                        print("Chosen direction",dv)
                        print("jvp Finite differences:",g_fd)
                        print("jvp:",g)
                        raise AssertionError("Jacobian-vector product of %s w.r.t. %s has an error of size %f"%(str(f),f.argname(i),np.linalg.norm(g-g_fd)))

                dv = np.random.uniform(-1,-1,_size(v))
                g = f.jvp(i,dv,*x)
                has_jvp = True
                if _scalar(v):
                    dv = dv[0]
                g_fd = finite_differences(lambda t:f.eval(*(x[:i]+[v+dv*t]+x[i+1:])),0,h)[:,0]
                if g is 0:
                    g = np.zeros(g_fd.shape)
                if _scalar(g):
                    g = np.array([g])
                elif g.shape != g_fd.shape:
                    raise AssertionError("Jacobian-vector product of %s w.r.t. %s did not produce the correct shape: %s vs %s"%(str(f),f.argname(i),g.shape,g_fd.shape))
                if not np.allclose(g_fd,g,rtol,atol):
                    print("check_derivative",f,"failed with args",x,"@ argument",i)
                    print("Chosen direction",dv)
                    print("jvp Finite differences:",g_fd)
                    print("jvp:",g)
                    raise AssertionError("Jacobian-vector product of %s w.r.t. %s has an error of size %f"%(str(f),f.argname(i),np.linalg.norm(g-g_fd)))
            except NotImplementedError:
                pass
            if not (has_derivative or has_jvp):
                print("check_derivative: function",f,"has no derivative or jvp function defined for arg",i)

            try:
                H = f.gen_derivative([i,i],*x)
                H_fd = finite_differences_hessian(lambda y:f.eval(*(x[:i]+[y]+x[i+1:])),v,h)
                if H is 0:
                    H = np.zeros(H_fd.shape)
                elif H.shape != H_fd.shape:
                    raise AssertionError("Hessian of %s w.r.t. %s did not produce the correct shape: %s vs %s"%(str(f),f.argname(i),H.shape,H_fd.shape))
                if not np.allclose(H_fd,H,rtol,atol):
                    print("check_derivative",f,"failed with args",x,"@ argument",i)
                    print("Finite differences:",H_fd)
                    print("gen_derivatives Hessian:",H)
                    raise AssertionError("gen_derivative of %s w.r.t. %s has an error of size %f"%(str(f),f.argname(i),np.linalg.norm(H-H_fd)))
            except NotImplementedError:
                has_gen_derivative = False
                pass
        if has_gen_derivative:
            for i,v in enumerate(x):
                for j,w in list(enumerate(x))[i+1:]:
                    H = f.gen_derivative([i,j],*x)
                    H_fd = finite_differences_hessian2(lambda y,z:f.eval(*(x[:i]+[y]+x[i+1:j]+[z]+x[j+1:])),v,w,h)
                    if H is 0:
                        H = np.zeros(H_fd.shape)
                    elif H.shape != H_fd.shape:
                        raise AssertionError("Hessian of %s w.r.t. %s,%s did not produce the correct shape: %s vs %s"%(str(f),f.argname(i),f.argname(j),H.shape,H_fd.shape))
                    H2 = f.gen_derivative([j,i],*x)
                    H2_fd = finite_differences_hessian2(lambda z,y:f.eval(*(x[:i]+[y]+x[i+1:j]+[z]+x[j+1:])),w,v,h)
                    if H2 is 0:
                        H2 = np.zeros(H2_fd.shape)
                    elif H2.shape != H2_fd.shape:
                        raise AssertionError("Hessian of %s w.r.t. %s,%s did not produce the correct shape: %s vs %s"%(str(f),f.argname(j),f.argname(i),H.shape,H_fd.shape))
    else:
        raise ValueError("f must be a ADFunctionCall or ADFunctionInterface")
    return True


def eval_multiple(exprs,**kwargs):
    """Given a list of expressions, and keyword arguments that set variable
    values, returns a list of the evaluations of the expressions.

    This can leverage common subexpressions in exprs to speed up running times
    compared to multiple eval() calls.
    """
    for e in exprs:
        e._clear_eval_cache()
    return [e._eval([],kwargs) for e in exprs]


def derivative_multiple(exprs,arg,**kwargs):
    """Given a list of expressions, the argument to which you'd like to take
    the derivatives, and keyword arguments that set variable values, returns
    a list of the derivatives of the expressions.

    This can leverage common subexpressions in exprs to speed up running times
    compared to multiple derivative() calls.
    """
    for e in exprs:
        e._clear_deriv_cache()
    if any(e._eval_context is None or not _context_equal(e._eval_context,kwargs) for e in exprs):
        eval_multiple(exprs,**kwargs)
    res = [e._deriv([],arg,kwargs) for e in exprs]
    for i,der in enumerate(res):
        if der is 0:
            res[i] = np.zeros((_size(exprs[i]._eval_result),_size(kwargs[arg])))
        if len(der.shape) == 1:
            #deriv w.r.t. scalar
            res[i] = der.reshape((der.shape[0],1))
    return res


def _size(a):
    try:
        return a.shape[0]
    except AttributeError:
        if hasattr(a,'__iter__'):
            raise ValueError("Can't pass a list as an argument. Try wrapping args with np.array.")
        return 1
    except IndexError:
        #a numpy 1-parameter array
        return 1

def _scalar(a):
    return np.ndim(a)==0

def _context_equal(a,b):
    for k,v in a.items():
        try:
            if v is not b[k] and not np.array_equal(v,b[k]):
                return False
        except KeyError:
            return False
    for k in b.keys():
        if k not in a:
            return False
    return True

def _zero_derivative(nout,arg,*args):
    if hasattr(arg,'__iter__'):
        return np.zeros([nout]+[_size(args[a]) for a in arg])
    return np.zeros([nout,_size(args[arg])])


class _ADAdd(ADFunctionInterface):
    def __str__(self):
        return 'add'

    def n_args(self):
        return 2

    def n_in(self,arg):
        return -1

    def n_out(self):
        return -1

    def eval(self,x,y):
        return x + y

    def derivative(self,arg,*args):
        size = max(_size(a) for a in args)
        assert 0 <= arg <= 1
        if _scalar(args[arg]):
            return np.ones((size,1))
        return np.eye(size)

    def jvp(self,arg,darg,*args):
        assert 0 <= arg <= 1
        size = max(_size(a) for a in args)
        if _scalar(args[arg]):
            assert darg.shape[0] == 1
            if size == 1:
                return darg
            else:
                return np.hstack([darg]*size)
        else:
            assert darg.shape[0] == size
            return darg

    def gen_derivative(self,arg,*args):
        if len(arg) == 1:
            return self.derivative(arg[0],*args)
        return 0
        


class _ADSub(ADFunctionInterface):
    def __str__(self):
        return 'sub'

    def n_args(self):
        return 2

    def n_in(self,arg):
        return -1

    def n_out(self):
        return -1

    def eval(self,x,y):
        return x - y

    def derivative(self,arg,*args):
        size = max(_size(a) for a in args)
        assert 0 <= arg <= 1
        sign = 1 if arg == 0 else -1
        if _scalar(args[arg]):
            return sign*np.ones((size,1))
        return sign*np.eye(size)

    def jvp(self,arg,darg,*args):
        assert 0 <= arg <= 1
        size = max(_size(a) for a in args)
        sign = 1 if arg == 0 else -1
        if _scalar(args[arg]):
            assert darg.shape[0] == 1
            if size == 1:
                return sign*darg
            else:
                return sign*np.hstack([darg]*size)
        else:
            assert darg.shape[0] == size
            return sign*darg

    def gen_derivative(self,arg,*args):
        if len(arg) > 1:
            return 0
        return self.derivative(arg[0],*args)


class _ADMul(ADFunctionInterface):
    def __str__(self):
        return 'mul'

    def n_args(self):
        return 2

    def n_in(self,arg):
        return -1

    def n_out(self):
        return -1

    def eval(self,x,y):
        return x*y

    def derivative(self,arg,*args):
        assert 0 <= arg <= 1
        x = args[arg]
        y = args[1-arg]
        if _scalar(x):
            res = np.asarray(y)
            return res.reshape((_size(y),1))
        if _scalar(y):
            return np.diag([y]*_size(x))
        return np.diag(np.asarray(y))

    def jvp(self,arg,darg,*args):
        return args[1-arg]*darg

    def gen_derivative(self,arg,*args):
        if len(arg) == 1:
            return self.derivative(arg[0],*args)
        elif len(arg) == 2:
            i,j = arg
            if i == j:
                return 0
            res = args[i]*args[j]
            hess_shape = (_size(res),_size(args[i]),_size(args[j]))
            res = np.zeros(hess_shape)
            if _scalar(args[i]):
                if _scalar(args[j]):
                    res[0,0,0] = 1
                else:
                    for i in range(hess_shape[2]):
                        res[i,0,i] = 1
            else:
                if _scalar(args[j]):
                    for i in range(hess_shape[1]):
                        res[i,i,0] = 1
                else:
                    for i in range(hess_shape[1]):
                        res[i,i,i] = 1
            return res
        return 0


class _ADDiv(ADFunctionInterface):
    def __str__(self):
        return 'div'

    def n_args(self):
        return 2

    def n_in(self,arg):
        return -1

    def n_out(self):
        return -1

    def eval(self,x,y):
        return x/y

    def derivative(self,arg,x,y):
        if arg == 0:
            diag = 1.0/y
            if _scalar(x):
                return diag.reshape((_size(y),1))
            if _scalar(y):
                diag = [diag]*_size(x)
            return np.diag(diag)
        else:
            diag = -x/y**2
            if _scalar(y):
                return diag.reshape((_size(x),1))
            return np.diag(-x/y**2)

    def jvp(self,arg,darg,x,y):
        if arg == 0:
            return darg/y
        else:
            return -(x*darg)/y**2

    def gen_derivative(self,arg,x,y):
        if len(arg) == 1:
            return self.derivative(arg[0],x,y)
        elif len(arg) == 2:
            i,j = arg
            if i == 0 and j == 0:
                return 0
            if i == j:
                #double derivative w.r.t. y
                m = max(_size(x),_size(y))
                n = _size(y)
                hess_shape = (m,n,n)
                res = np.zeros(hess_shape)
                xarr = [x]*n if _scalar(x) else x
                if _scalar(y):
                    for i in range(m):
                        res[i,0,0] = 2*xarr[i]/y**3
                else:
                    for i in range(n):
                        res[i,i,i] = 2*xarr[i]/y[i]**3
                return res
            else:
                swap = (i == 1)
                #double derivative d^2/dxy or d^2/dyx
                m = max(_size(x),_size(y))
                hess_shape = (m,_size(x),_size(y))
                res = np.zeros(hess_shape)
                if _scalar(y):
                    for i in range(hess_shape[1]):
                        res[i,i,0] = -1.0/y**2
                elif _scalar(x):
                    for i in range(hess_shape[2]):
                        res[i,0,i] = -1.0/y[i]**2
                else:
                    for i in range(hess_shape[2]):
                        res[i,i,i] = -1.0/y[i]**2
                if swap:
                    return np.swapaxes(res,1,2)
                else:
                    return res
        raise NotImplementedError()


class _ADNeg(ADFunctionInterface):
    def __str__(self):
        return 'neg'

    def n_args(self):
        return 1

    def n_in(self,arg):
        return -1

    def n_out(self):
        return -1

    def eval(self,x):
        return -x

    def derivative(self,arg,x):
        size = _size(x)
        return -np.eye(size)

    def jvp(self,arg,dx,x):
        return -dx

    def gen_derivative(self,arg,x):
        if len(arg) == 1:
            return self.derivative(arg[0],x)
        return 0
        


class _ADSum(ADFunctionInterface):
    def __str__(self):
        return 'sum'

    def n_args(self):
        return -1

    def n_in(self,arg):
        return -1

    def n_out(self):
        return -1

    def eval(self,*args):
        if len(args) == 1:
            return np.sum(args[0])
        return sum(args)

    def derivative(self,arg,*args):
        if len(args) == 1:
            return np.ones(_size(args[0]))[np.newaxis,:]
        size = max(_size(a) for a in args)
        if _scalar(args[arg]):
            return np.ones((size,1))
        return np.eye(size)

    def jvp(self,arg,darg,*args):
        if len(args) == 1:
            return np.sum(darg)
        size = max(_size(a) for a in args)
        if _scalar(args[arg]):
            if size == 1:
                return darg
            else:
                return np.hstack([darg]*size)
        else:
            return darg

    def gen_derivative(self,arg,*args):
        if len(arg) == 1:
            return self.derivative(arg[0],*args)
        return 0


class _ADPow(ADFunctionInterface):
    def __str__(self):
        return 'pow'

    def argname(self,arg):
        return ['base','exp'][arg]

    def n_args(self):
        return 2

    def n_in(self,arg):
        return -1

    def n_out(self):
        return -1

    def eval(self,base,exp):
        return np.power(base,exp)

    def derivative(self,arg,base,exp):
        size = max(_size(base),_size(exp))
        if arg == 0:
            diag = exp*np.power(base,exp-1)
            if _scalar(base):
                return diag.reshape((_size(diag),1))
            return np.diag(diag)
        else:
            with np.errstate(divide='ignore'):
                diag = np.log(base)
            if _scalar(base):
                if base == 0:
                    diag = np.array(0.0)
            else:
                diag[base==0] = 0
            diag = diag*np.power(base,exp)
            if _scalar(exp):
                return diag.reshape((_size(diag),1))
            return np.diag(diag)

    def jvp(self,arg,darg,base,exp):
        if arg == 0:
            return exp*np.power(base,exp-1)*darg
        else:
            with np.errstate(divide='ignore'):
                scale = np.log(base)
            if _scalar(base):
                if base == 0:
                    scale = 0.0
            else:
                scale[base==0] = 0
            return scale*np.power(base,exp)*darg

    def gen_derivative(self,arg,*args):
        if len(arg)==1:
            return self.derivative(arg[0],*args)
        elif len(arg)==2:
            base,exp = args
            n = max(_size(base),_size(exp))
            hess_shape = (n,_size(args[arg[0]]),_size(args[arg[1]]))
            res = np.zeros(hess_shape)
            if arg[0]==0 and arg[1]==0:
                diag = exp*(exp-1)*np.power(base,exp-2)
                if _scalar(base):
                    return diag.reshape((1,1,1))
                else:
                    for i in range(n):
                        res[i,i,i] = diag[i]
            elif arg[0]==1 and arg[1]==1:
                #first deriv is log(base)*base**exp
                #second deriv is log(base)**2*base**exp
                with np.errstate(divide='ignore'):
                    scale = np.log(base)**2
                if _scalar(base):
                    if base == 0:
                        scale = 0.0
                else:
                    scale[base==0] = 0
                diag = scale*np.power(base,exp)
                if _scalar(exp):
                    return diag.reshape((n,1,1))
                else:
                    for i in range(n):
                        res[i,i,i] = diag[i]
            else:
                #d/dexp log(base)*base**exp
                #d^2/dexp dbase is (1 + log(base)*exp)*base**(exp-1)
                with np.errstate(divide='ignore'):
                    scale = np.log(base)
                if _scalar(base):
                    if base == 0:
                        scale = 0.0
                else:
                    scale[base==0] = 0
                diag = (1 + scale*exp)*np.power(base,exp-1)
                if arg[0] == 1:
                    for i in range(n):
                        j = 0 if hess_shape[1] == 1 else i
                        k = 0 if hess_shape[2] == 1 else i
                        res[i,j,k] = diag[i]
                return res
        raise NotImplementedError()


class _ADAbs(ADFunctionInterface):
    def __str__(self):
        return 'abs'

    def n_args(self):
        return 1

    def n_in(self,arg):
        return -1

    def n_out(self):
        return -1

    def eval(self,x):
        return np.abs(x)

    def derivative(self,arg,x):
        assert arg == 0
        size = _size(x)
        if _scalar(x):
            return np.sign(x).reshape((1,1))
        return np.diag(np.sign(x))

    def jvp(self,arg,dx,x):
        return dx*np.sign(x)

    def gen_derivative(self,arg,x):
        if len(arg) == 1:
            return self.derivative(arg[0],x)
        return 0


class _ADGetItem(ADFunctionInterface):
    def __init__(self,indices):
        self.indices = indices

    def __str__(self):
        return 'getitem['+str(self.indices)+']'

    def n_args(self):
        return 1

    def n_in(self,arg):
        return -1

    def n_out(self,argval=None):
        if argval is None:
            return -1
        if isinstance(self.indices,slice):
            start, stop, step = self.indices.indices(len(argval))
            return (stop-start)//step
        elif hasattr(self.indices,'__iter__'):
            return len(self.indices)
        else:
            return 1

    def eval(self,x):
        return x[self.indices]

    def derivative(self,arg,x):
        assert arg == 0
        size = _size(x)
        if isinstance(self.indices,slice):
            start, stop, step = self.indices.indices(size)
            nres = (stop-start)//step
            res = np.zeros((nres,size))
            for i in range(nres):
                j = start+i*step
                res[i,j] = 1
            return res
        elif hasattr(self.indices,'__iter__'):
            res = np.zeros((len(self.indices),size))
            for i,j in enumerate(self.indices):
                res[i,j] = 1
            return res
        else:
            res = np.zeros((1,size))
            res[0,self.indices] = 1
            return res

    def jvp(self,arg,dx,x):
        return dx[self.indices]

    def gen_derivative(self,arg,x):
        if len(arg) == 1:
            return self.derivative(arg[0],x)
        return 0


class _ADSetItem(ADFunctionInterface):
    def __init__(self,indices):
        self.indices = indices

    def __str__(self):
        return 'setitem['+str(self.indices)+']'

    def n_args(self):
        return 2

    def n_in(self,arg):
        if arg==0: return -1
        if isinstance(self.indices,slice):
            if self.indices.stop is not None:
                start, stop, step = self.indices.start,self.indices.stop,self.indices.step
                if self.indices.step >= 0 and self.indices.stop >= 0:
                    return (stop-start)//step
            return -1
        elif hasattr(self.indices,'__iter__'):
            return len(self.indices)
        else:
            return 1

    def n_out(self):
        return -1

    def eval(self,x,v):
        xcopy = np.copy(x)
        xcopy[self.indices] = v
        return xcopy

    def derivative(self,arg,x,v):
        if arg==0:
            diag = np.ones(x.shape)
            diag[self.indices] = 0
            return np.diag(diag)
        else:
            res = np.zeros((_size(x),_size(v)))
            if isinstance(self.indices,slice):
                start, stop, step = self.indices.indices(res.shape[0])
                nres = (stop-start)//step
                for i in range(nres):
                    j = start+i*step
                    res[j,i] = 1
                return res
            elif hasattr(self.indices,'__iter__'):
                for i,j in enumerate(self.indices):
                    res[j,i] = 1
                return res
            else:
                res[self.indices,0] = 1
                return res

    def jvp(self,arg,darg,x,v):
        if arg==0:
            res = np.copy(darg)
            res[self.indices] = 0
            return res
        else:
            res = np.zeros(x.shape)
            res[self.indices] = darg
            return res

    def gen_derivative(self,arg,x,v):
        if len(arg) == 1:
            return self.derivative(arg[0],x,v)
        return 0


class _ADStack(ADFunctionInterface):
    def __str__(self):
        return 'stack'

    def n_args(self):
        return -1

    def n_in(self,arg):
        return -1

    def n_out(self):
        return -1

    def eval(self,*args):
        return np.hstack(args)

    def derivative(self,arg,*args):
        sizes = [_size(a) for a in args]
        totalsize = sum(sizes)
        res = np.zeros((totalsize,sizes[arg]))
        indices = np.cumsum([0]+sizes)
        base = indices[arg]
        for i in range(sizes[arg]):
            res[base+i,i] = 1
        return res

    def jvp(self,arg,darg,*args):
        sizes = [_size(a) for a in args]
        totalsize = sum(sizes)
        indices = np.cumsum([0]+sizes)
        res = np.zeros(totalsize)
        res[indices[arg]:indices[arg]+sizes[arg]] = darg
        return res

    def gen_derivative(self,arg,*args):
        if len(arg) == 1:
            return self.derivative(arg[0],*args)
        return 0


class _ADMinimum(ADFunctionInterface):
    def __str__(self):
        return 'minimum'

    def n_args(self):
        return -1

    def n_in(self,arg):
        return -1

    def n_out(self):
        return -1

    def eval(self,*args):
        if len(args) == 0:
            raise ValueError("Can't take a minimum with 0 arguments")
        if len(args) == 1:
            assert not _scalar(args[0]),"Can't take a minimum over a single scalar"
            return args[0].min()
        res = np.minimum(args[0],args[1])
        for i in range(2,len(args)):
            res = np.minimum(res,args[i])
        return res

    def derivative(self,arg,*args):
        if len(args)==1:
            res = np.zeros((1,_size(args[0])))
            res[0,args[0].argmax()] = 1
            return res
        sizes = [_size(a) for a in args]
        size = max(sizes)
        amin = np.zeros(size,dtype=int)
        if _scalar(args[0]) and _scalar(args[1]):
            if args[1] < args[0]:
                amin[:] = 1
        else:
            amin[args[1] < args[0]] = 1
        res = np.minimum(args[0],args[1])
        for i in range(2,len(args)):
            if _scalar(res) and _scalar(args[i]):
                if args[i] < res:
                    amin[:] = i
            else:
                amin[args[i] < res] = i
            res = np.minimum(res,args[i])
        J = np.zeros((size,sizes[arg]))
        if sizes[arg] == 1:
            for i,index in enumerate(amin):
                if index == arg:
                    J[i,0] = 1
        else:
            for i,index in enumerate(amin):
                if index == arg:
                    J[i,i] = 1
        return J

    def jvp(self,arg,darg,*args):
        if len(args)==1:
            return darg[args[0].argmin()]
        sizes = [_size(a) for a in args]
        size = max(sizes)
        amin = np.zeros(size,dtype=int)
        if _scalar(args[0]) and _scalar(args[1]):
            if args[1] < args[0]:
                amin[:] = 1
        else:
            amin[args[1] < args[0]] = 1
        res = np.minimum(args[0],args[1])
        for i in range(2,len(args)):
            if _scalar(res) and _scalar(args[i]):
                if args[i] < res:
                    amin[:] = i
            else:
                amin[args[i] < res] = i
            res = np.minimum(res,args[i])
        Jdx = np.zeros(size)
        if sizes[arg] == 1:
            for i,index in enumerate(amin):
                if index == arg:
                    Jdx[i] = darg[0]
        else:
            for i,index in enumerate(amin):
                if index == arg:
                    Jdx[i] = darg[i]
        return Jdx 

    def gen_derivative(self,arg,*args):
        if len(arg) == 1:
            return self.derivative(arg[0],*args)
        return 0



class _ADMaximum(ADFunctionInterface):
    def __str__(self):
        return 'maximum'

    def n_args(self):
        return -1

    def n_in(self,arg):
        return -1

    def n_out(self):
        return -1

    def eval(self,*args):
        if len(args) == 0:
            raise ValueError("Can't take a maximum with 0 arguments")
        if len(args) == 1:
            assert not _scalar(args[0]),"Can't take a minimum over a single scalar"
            return args[0].max()
        res = np.maximum(args[0],args[1])
        for i in range(2,len(args)):
            res = np.maximum(res,args[i])
        return res

    def derivative(self,arg,*args):
        if len(args)==1:
            res = np.zeros((1,_size(args[0])))
            res[0,args[0].argmax()] = 1
            return res
        sizes = [_size(a) for a in args]
        size = max(sizes)
        amin = np.zeros(size,dtype=int)
        if _scalar(args[0]) and _scalar(args[1]):
            if args[1] > args[0]:
                amin[:] = 1
        else:
            amin[args[1] > args[0]] = 1
        res = np.maximum(args[0],args[1])
        for i in range(2,len(args)):
            if _scalar(res) and _scalar(args[i]):
                if args[i] > res:
                    amin[:] = i
            else:
                amin[args[i] > res] = i
            res = np.maximum(res,args[i])
        J = np.zeros((size,sizes[arg]))
        if sizes[arg] == 1:
            for i,index in enumerate(amin):
                if index == arg:
                    J[i,0] = 1
        else:
            for i,index in enumerate(amin):
                if index == arg:
                    J[i,i] = 1
        return J

    def jvp(self,arg,darg,*args):
        if len(args)==1:
            return darg[args[0].argmax()]
        sizes = [_size(a) for a in args]
        size = max(sizes)
        amin = np.zeros(size,dtype=int)
        if _scalar(args[0]) and _scalar(args[1]):
            if args[1] > args[0]:
                amin[:] = 1
        else:
            amin[args[1] > args[0]] = 1
        res = np.maximum(args[0],args[1])
        for i in range(2,len(args)):
            if _scalar(res) and _scalar(args[i]):
                if args[i] > res:
                    amin[:] = i
            else:
                amin[args[i] > res] = i
            res = np.maximum(res,args[i])
        Jdx = np.zeros(size)
        if sizes[arg] == 1:
            for i,index in enumerate(amin):
                if index == arg:
                    Jdx[i] = darg[0]
        else:
            for i,index in enumerate(amin):
                if index == arg:
                    Jdx[i] = darg[i]
        return Jdx 

    def gen_derivative(self,arg,*args):
        if len(arg) == 1:
            return self.derivative(arg[0],*args)
        return 0


class _ADCond(ADFunctionInterface):
    def __str__(self):
        return 'cond'

    def argname(self,arg):
        return ['pred','posval','negval'][arg]

    def n_args(self):
        return 3

    def n_in(self,arg):
        return -1

    def n_out(self):
        return -1

    def eval(self,pred,trueval,falseval):
        if _scalar(pred):
            if pred > 0:
                return trueval
            return falseval
        else:
            assert len(pred)==len(trueval)
            assert len(pred)==len(falseval)
            res = falseval.copy()
            res[pred>0]=trueval[pred>0]
            return res

    def jvp(self,arg,darg,pred,trueval,falseval):
        if arg==0:
            return 0
        if arg==1:
            if _scalar(pred):
                if pred > 0:
                    return darg
                return 0
            else:
                res = np.zeros(trueval.shape)
                res[pred > 0] = darg[pred > 0]
        else:
            if _scalar(pred):
                if pred > 0:
                    return 0
                return darg
            else:
                res = darg.copy()
                res[pred > 0] = 0
                return res


class _ADCond3(ADFunctionInterface):
    def __str__(self):
        return 'cond3'

    def argname(self,arg):
        return ['pred','posval','zeroval','negval'][arg]

    def n_args(self):
        return 4

    def n_in(self,arg):
        return -1

    def n_out(self):
        return -1

    def eval(self,pred,trueval,zeroval,falseval):
        if _scalar(pred):
            if pred > 0:
                return trueval
            elif pred == 0:
                return zeroval
            else:
                return falseval
        else:
            assert len(pred)==len(trueval)
            assert len(pred)==len(zeroval)
            assert len(pred)==len(falseval)
            res = falseval.copy()
            res[pred>0]=trueval[pred>0]
            res[pred==0]=zeroval[pred==0]
            return res

    def jvp(self,arg,darg,pred,trueval,zeroval,falseval):
        if arg==0:
            return 0
        if arg==1:
            if _scalar(pred):
                if pred > 0:
                    return darg
                return 0
            else:
                res = np.zeros(trueval.shape)
                res[pred > 0] = darg[pred > 0]
        elif arg==2:
            if _scalar(pred):
                if pred == 0:
                    return darg
                return 0
            else:
                res = np.zeros(trueval.shape)
                res[pred == 0] = darg[pred == 0]
        else:
            if _scalar(pred):
                if pred >= 0:
                    return 0
                return darg
            else:
                res = darg.copy()
                res[pred >= 0] = 0
                return res



add = _ADAdd()
"""Autodiff'ed function comparable to the addition operator +. Applied 
automatically when the + operator is called on an ADTerminal or an
ADFunctionCall."""

sub = _ADSub()
"""Autodiff'ed function comparable to the subtraction operator -. Applied 
automatically when the - operator is called on an ADTerminal or an
ADFunctionCall."""

mul = _ADMul()
"""Autodiff'ed function comparable to the multiplication operator \*. Applied 
automatically when the * operator is called on an ADTerminal or an
ADFunctionCall.  Multiplication of vectors is performed element-wise."""

div = _ADDiv()
"""Autodiff'ed function comparable to the division operator /. Applied 
automatically when the / operator is called on an ADTerminal or an
ADFunctionCall.  Divison of vectors is performed element-wise."""

neg = _ADNeg()
"""Autodiff'ed function comparable to the negation operator -. Applied 
automatically when the - operator is called on an ADTerminal or an
ADFunctionCall."""

sum_ = _ADSum()
"""Autodiff'ed function comparable to sum.  It acts like np.sum if only one
item is provided.  Otherwise, it acts like the builtin sum."""

pow_ = _ADPow()
"""Autodiff'ed function comparable to np.power. Applied automatically when the
** operator is called on an ADTerminal or an ADFunctionCall."""

abs_ = _ADAbs()
"""Autodiff'ed function comparable to abs. Applied automatically when abs is
called on an ADTerminal or an ADFunctionCall."""

stack = _ADStack()
"""Autodiff'ed function comparable to np.stack."""

def getitem(a,index):
    """Creates an Autodiff'ed function comparable to the [] operator. Note that
    the index argument cannot be a variable."""
    return _ADGetItem(index)(a)

def setitem(x,index,v):
    """Creates an Autodiff'ed function similar to ``x[index] = v, return x``
    but without modifying x.  The index argument cannot be a variable."""
    return _ADSetItem(index)(x,v)

minimum = _ADMinimum()
"""Autodiff'ed function comparable to np.minimum. It acts like ndarray.min if
only one item is provided, and otherwise it acts like np.minimum. If more than
2 items are provided, they are applied elementwise and sequentially.
"""

maximum = _ADMaximum()
"""Autodiff'ed function comparable to np.maximum. It acts like ndarray.max if
only one item is provided, and otherwise it acts like np.maximum. If more than
2 items are provided, they are applied elementwise and sequentially.
"""

cond = _ADCond()
"""Autodiff'ed function that performs a conditional (pred,trueval,falseval).
It returns trueval if pred > 0 and falseval otherwise.   If pred is an array,
then it must have the same size as trueval and falseval, and performs the
conditioning element-wise.
"""

cond3 = _ADCond3()
"""Autodiff'ed function that performs a triple conditional
(pred,trueval,zeroval,falseval). It returns trueval if pred > 0, zeroval if
pred=0, and falseval otherwise.   If pred is an array, then it must have the
same size as trueval, zeroval, and falseval, and performs the conditioning
element-wise.
"""
