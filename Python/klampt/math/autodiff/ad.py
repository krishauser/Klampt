"""Basic definitions for autodifferentiation.

Overview
========

This module defines a simple autodiff system for Klamp't objects, including
3D math, kinematics, dynamics, and geometry functions. This is useful for 
all sorts of optimization functions, e.g., machine learning, calibration,
and trajectory optimization.

This can be used standalone, but is more powerful in conjunction with
PyTorch or CadADi. Use the functions in :module:`pytorch` and :module:`casadi`
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
returns the Jacobian :math:`\frac{df}{dx_{arg}}(x1,..,xn)` , or a ``jvp``
function ``Jdx(arg,dx,x1,...,xn)``, which calculates the jacobian-vector
product :math:`\frac{df}{dx_{arg}}(x1,...,xn)\cdot dx` .  More conveniently,
you can just provide a list of functions ``Ji(x1,...,xn)``, with i=1,..,n,
with ``Ji`` giving :math:`\frac{df}{dx_i}(x1,...,xn)` , or in the JVP form,
functions ``Jdxi(dx,x1,...,xn)`` each giving
:math:`\frac{df}{dx_{i}}(x1,...,xn)\cdot dx`.  See the :func:`function`
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


ad Module
=========

This module defines :func:`var`, :func:`function`, and the helper functions
:func:`check_derivatives`, :func:`eval_multiple`: and
:func:`finite_differences`. 

The operators :func:`stack`, :func:`sum_`, :func:`minimum`, and :func:`maximum`
operate somewhat like their Numpy counterparts:

- stack acts like hstack, since all items are 1D arrays. 
- sum_ acts like Numpy's sum if only one item is provided.  Otherwise, it
  acts like the builtin sum.
- minimum/maximum act like ndarray.min/max if only one item is provided, and
  otherwise they act like Numpy's minimum/maximum. If more than 2 items are
  provided, they are applied elementwise and sequentially.

The operators add, sub, mul, div, neg, pow_, abs_, and getitem are also
provided here, but you will probably just want to use the standard Python 
operators +, -, *, /, -, **, abs, and [].  On arrays, they act just like
the standard numpy functions.

When you create a new function with its derivatives, you ought to run it
through check_derivatives to see whether they match with finite differences.
To modify the finite difference step size, modify ``FINITE_DIFFERENCE_RES``.


Other Klampt ad Modules
=======================

autodiff versions of many Klamp't functions are found in:

- :module:`math_ad` : basic vector math, including functions from the
  :module:`~klampt.math.vectorops` module.
- :module:`so3_ad` : operations on SO(3), mostly compatible with the
  :module:`~klampt.math.so3` module.
- :module:`se3_ad` : operations on SE(3), mostly compatible with the
  :module:`~klampt.math.se3` module.
- :module:`kinematics_ad` kinematics functions, e.g., robot Jacobians.
- :module:`geometry_ad` geometry derivative functions. TODO
- :module:`dynamics_ad` dynamics derivative functions. TODO
- :module:`spline_ad` spline derivative functions. TODO
"""

import numpy as np

#Resolution used for finite differences
FINITE_DIFFERENCE_RES = 1e-6

class ADFunctionInterface:
    """The base class for a new auto-differentiable function. 

    The function has the form f(x_0,...,x_k) with k+1 == self.n_args().  All
    inputs and outputs are either scalars or numpy ndarrays.

    To define a new function, you need to implement n_args(), eval(*args), and
    optionally n_in(arg), n_out(), derivative(arg,*args), and
    jvp(arg,darg,*args).

    A function class can be instantiated with non-numeric data.  For example,
    to perform forward kinematics the call::

         wp = WorldPosition(link,localPos)

    (see :module:`kinematics_ad`) creates a function wp(q) that will accept a 
    robot configuration q and return the world position of the point 
    ``localPos`` on link ``link``.

    This makes these function classes more expressive, but also leads to a 
    slight annoyance in that to define a function that can be called, you need
    to instantiate a function.  For exaple, the sin function is defined in
    :module:`math_ad` as ``sin = ADSinFunction()`` where ``ADSinFunction`` is
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

        Arguments:
            args (list): a list of arguments, which are either ndarrays or
            scalars.
        """
        raise NotImplementedError()

    def derivative(self,arg,*args):
        """Returns the Jacobian of the function w.r.t. argument #arg.

        Arguments:
            arg (int): A value from 0,...,self.n_args()-1 indicating that we
                wish to take df/dx_arg. 
            args (list of ndarrays): arguments for the function.

        Returns:
            ndarray: A numpy array of shape (self.n_out(),self.n_in(arg)).
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

        Arguments:
            arg (list): Indicates the order of derivatives to be taken.  For
                example, to take the 2nd derivative w.r.t. x0,x1,
                arg = [0,1] should be specified.
            args (list of ndarrays): arguments for the function.

        Returns:
            ndarray: A tensor of shape (n_out,n_in(arg[0]),...,n_in(arg[-1]).

            If the generalized derivative is not implemented, raise a
            NotImplementedError.

            If the generalized derivative is zero, can just return 0 (the
            integer) regardless of the size of the result.
        """
        raise NotImplementedError()


    def jvp(self,arg,darg,*args):
        """Performs a Jacobian-vector product for argument #arg.

        Arguments:
            arg (int): A value from 0,...,self.n_args()-1 indicating that we
                wish to calculate df/dx_arg * darg. 
            darg (ndarray): the derivative of x_arg w.r.t some other parameter.
                Must have darg.shape = (self.n_in(arg),). 
            args (list of ndarrays): arguments for the function.

        Returns:
            ndarray: A numpy array of length self.n_out()

            If the derivative is not implemented, raise a NotImplementedError.
        """
        raise NotImplementedError()

    def __call__(self,*args):
        na = self.n_args()
        if na >= 0 and len(args) != na:
            raise ValueError("Invalid number of arguments to function %s, wanted %d, got %d"%(str(self),na,len(args)))
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
        self.terminals = {}
        for i,arg in enumerate(args):
            if isinstance(arg,str):
                self.args[i] = ADTerminal(arg)
                self.terminals[arg] = self.args[i]
            elif isinstance(arg,ADTerminal):
                self.terminals[arg.name] = arg
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
        self._print_id = None
        self._eval_result = None
        self._eval_context = None
        self._eval_instantiated_args = None
        self._deriv_result = None

    def __str__(self):
        terms = {}
        dag = self._print_dag(terms)
        self._clear_print_cache()
        def _flatten_str_tree(tree):
            flat = []
            for v in tree:
                if isinstance(v,list):
                    flat.append(_flatten_str_tree(v))
                else:
                    flat.append(v)
            return ''.join(flat)
        return _flatten_str_tree(dag)

    def _clear_print_cache(self):
        if self._print_id is None:
            return
        self._print_id = None
        for v in self.args:
            if isinstance(v,ADFunctionCall):
                v._clear_print_cache()

    def _print_dag(self,terms):
        assert self._print_id is None
        id = len(terms)
        self._print_id = id
        argstr = []
        for v in self.args:
            if isinstance(v,ADTerminal):
                argstr.append(str(v))
            elif isinstance(v,ADFunctionCall):
                if v._print_id is not None:
                    argstr.append('@'+str(v._print_id))
                    if not terms[v._print_id][-1].startswith('#'):
                        terms[v._print_id].append('#'+str(v._print_id))
                else:
                    argstr.append(v._print_dag(terms))
            else:
                argstr.append(str(v))
            argstr.append(',')
        argstr.pop(-1)
        res = [str(self.func),'(',argstr,')']
        terms[id] = res
        return res

    def eval(self,**kwargs):
        """Evaluate a function call with bound values given by the keyword
        arguments.  Example::

            print((3*var('x')**2 - 2*var('x')).eval(x=1.5))

        """
        self._clear_eval_cache()
        return self._eval([],kwargs)

    def derivative(self,arg,**kwargs):
        """Evaluates the derivative of this computation graph with respect to
        the named variable at the given setings in kwargs.

        If any derivatives are not defined, finite differences will be used
        with step size FINITE_DIFFERENCE_RES.

        If the prior eval() call wasn't with the same arguments, then this will
        call eval() again with kwargs.  So, the sequence eval(**kwargs),
        derivative(**kwargs) will save some time compared to changing the args.
        """
        self._clear_deriv_cache()
        if self._eval_context is None or not _context_equal(self._eval_context,kwargs):
            self.eval(**kwargs)
            assert _context_equal(self._eval_context,kwargs)
        res = self._deriv([],arg,kwargs)
        if res is 0:
            return np.zeros((_size(self._eval_result),_size(kwargs[arg])))
        if len(res.shape) == 1:
            #deriv w.r.t. scalar
            res = res.reshape((res.shape[0],1))
        return res

    def _eval(self,callStack,kwargs):
        if self._eval_result is not None:
            return self._eval_result
        self._eval_context = kwargs
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
        self._eval_instantiated_args = instantiated_args
        try:
            res = self.func.eval(*instantiated_args)
        except Exception:
            context = [str(s) for s in callStack]
            raise RuntimeError("Error running function %s in context %s"%(str(self.func),' -> '.join(context)))
        try:
            no = self.func.n_out()
            if no >= 0:
                if _size(res) != no:
                    raise ValueError("Error evaluating %s: return value has expected size %d, instead got %d"%(str(self),no,_size(res)))
        except NotImplementedError:
            #not implemented, just ignore
            pass
        self._eval_result = res
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
                        raise ValueError("Function %s.jvp result has wrong size? Context is %s, expects %d, result has size %d"%(str(self.func),str(self),no,_size(dcol)))
                return np.column_stack(columns)
            except NotImplementedError:
                pass
            return np.dot(finite_differences(lambda x:self.func.eval(*(args[:arg]+[x]+args[arg+1:])),args[arg],FINITE_DIFFERENCE_RES),mat)

    def _deriv(self,callStack,deriv_arg,kwargs):
        if self._deriv_result is not None:
            return self._deriv_result
        assert self._eval_context is not None
        callStack.append(self)
        res = 0
        for i,arg in enumerate(self.args):
            if isinstance(arg,ADTerminal):
                if arg.name == deriv_arg:
                    inc = self._deriv_jacobian(i,self._eval_instantiated_args)
                    assert hasattr(inc,'shape') and inc.shape[0] == _size(self._eval_result),"Function %s has invalid jacobian shape? Context %s"%(str(self.func),str(self))
                    if _scalar(self._eval_instantiated_args[i]):
                        #for scalar derivatives, we just use a 1D vector rather than a 2D array
                        inc = inc[:,0]
                else:
                    #not deriv_arg, no derivative
                    continue
            elif isinstance(arg,ADFunctionCall):
                da = arg._deriv(callStack,deriv_arg,kwargs)
                if da is 0:
                    continue
                inc = self._deriv_jacobian_array_product(i,da,self._eval_instantiated_args)
                assert hasattr(inc,'shape') and inc.shape[0] == _size(self._eval_result),"Function %s has invalid jacobian shape? Context %s"%(str(self.func),str(self))
            else:
                #const, no derivative
                continue
            if res is 0:
                res = inc
            else:
                res += inc
        callStack.pop(-1)
        self._deriv_result = res
        return res


    def _clear_eval_cache(self):
        if self._eval_result is None:
            return
        self._eval_result = None
        self._eval_context = None
        self._eval_instantiated_args = None
        for arg in self.args:
            if isinstance(arg,ADFunctionCall):
                arg._clear_eval_cache()

    def _clear_deriv_cache(self):
        if self._deriv_result is None:
            return
        self._deriv_result = None
        for arg in self.args:
            if isinstance(arg,ADFunctionCall):
                arg._clear_deriv_cache()

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
            tuple (arg,*args).  If a list, each element just takes the same
            arguments as func.
        jvp (callable or list of callables, optional): one or more Jacobian-
            vector product functions. If a single callable, has signature
            (arg,darg,*args).  If a list, each element has the signature
            (darg,*args)
        gen_derivative (callable, list of callables, or dict, optional): a
            higher-order derivative function. 

            If a callable is given, it has standard signature (arg,*args). 

            If a list of callables is given, they are a list of higher-order
            derivatives, starting at the second derivative.  In this case,
            the function may only have 1 argument, and each callable has the
            signature (*args).

            If a dict is given, it must map tuples to callables, each of which
            has the signature (*args).
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
    if hasattr(derivative,'__iter__'):
        derivative_list = derivative
        derivative = lambda arg,*args:derivative_list[arg](*args)
    if hasattr(jvp,'__iter__'):
        jvp_list = jvp
        jvp = lambda arg,darg,*args:jvp_list[arg](darg,*args)
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
    if derivative is not None:
        members['derivative'] = lambda self,arg,*args:np.asarray(derivative(arg,*args))
    if jvp is not None:
        members['jvp'] = lambda self,arg,darg,*args:np.asarray(jvp(arg,darg,*args))
    if gen_derivative is not None:
        members['gen_derivative'] = lambda self,arg,*args:np.asarray(gen_derivative(arg,*args))
    if argnames is not None:
        members['argname'] = lambda self,arg:argnames[arg]
    TempType = type("_ad_"+name,(ADFunctionInterface,),members)
    return TempType()


def finite_differences(f,x,h=1e-6):
    """Performs forward differences to approximate the derivative of f at x.
    f can be a vector or scalar function, and x can be a scalar or vector.

    The result is a Numpy array of shape (_size(f(x)),_size(x)).

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
        assert f.n_args() < 0 or f.n_args() == len(x),"Invalid number of arguments provided"
        for i,v in enumerate(x):
            try:
                g_fd = finite_differences(lambda y:f.eval(*(x[:i]+[y]+x[i+1:])),v,h)
                g = f.derivative(i,*x)
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
                #try jvp
                try:
                    dv = np.random.uniform(-1,1,_size(v))
                    g = f.jvp(i,dv,*x)
                    g_fd = finite_differences(lambda t:f.eval(*(x[:i]+[v+dv*t]+x[i+1:])),0,h)[:,0]
                    if g is 0:
                        g = np.zeros(g_fd.shape)
                    elif g.shape != g_fd.shape:
                        raise AssertionError("Jacobian-vector product of %s w.r.t. %s did not produce the correct shape: %s vs %s"%(str(f),f.argname(i),g.shape,g_fd.shape))
                    if not np.allclose(g_fd,g,rtol,atol):
                        print("check_derivative",f,"failed with args",x,"@ argument",i)
                        print("Random direction",dv)
                        print("jvp Finite differences:",g_fd)
                        print("jvp:",g)
                        raise AssertionError("Jacobian-vector product of %s w.r.t. %s has an error of size %f"%(str(f),f.argname(i),np.linalg.norm(g-g_fd)))
                except NotImplementedError:
                    print("check_derivative: function",f,"has no derivative or jvp function defined")
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
        return np.sign(x)*dx

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

    def n_args(self):
        return 2

    def n_in(self,arg):
        return -1

    def n_out(self):
        return -1

    def eval(self,base,exp):
        return np.power(base,exp)

    def deriv(self,arg,base,exp):
        size = max(_size(base),_size(exp))
        if arg == 0:
            diag = exp*np.power(base,exp-1)
            if _scalar(base):
                return diag.reshape((diag.shape[0],1))
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
                return diag.reshape((diag.shape[0],1))
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
    def __str__(self):
        return 'getitem['+str(self.indices)+']'

    def __init__(self,indices):
        self.indices = indices

    def n_args(self):
        return 1

    def n_in(self,arg):
        return -1

    def n_out(self,argval=None):
        if argval is None:
            return -1
        if isinstance(self.indices,slice):
            start, stop, step = self.indices.indices(size)
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


add = _ADAdd()
sub = _ADSub()
mul = _ADMul()
div = _ADDiv()
neg = _ADNeg()
sum_ = _ADSum()
pow_ = _ADPow()
abs_ = _ADAbs()
stack = _ADStack()

def getitem(a,index):
    return _ADGetItem(index)(a)

minimum = _ADMinimum()
maximum = _ADMaximum()