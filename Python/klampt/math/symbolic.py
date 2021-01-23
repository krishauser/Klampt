""" symbolic module

===============================================================================
Overview
===============================================================================

A framework for declaring and manipulating symbolic expressions.  Suitable for
automatic differentiation and developing computation graph models of functions
for use in optimization.

Inspired by Sympy, SCIP models, Gurobi models, as well as TensorFlow / PyTorch,
this framework has the following features:

- Saving and loading of expressions is fully supported.  Strings and JSON
  formats are supported.
- Supports black-box functions inside expressions.
- Allows references to arbitrary non-serializable user data.
- Allows new functions to be user-defined in a straightforward way.
- Symbolic auto-differentiation.
- Conversion to and from Sympy.
- Numeric, vector, and matrix variables are allowed.  Many native numpy
  operations are supported.

Could it be used for learning?  Yes, but it is not as fast as GPU-based
learning libraries like TensorFlow / PyTorch.

How does it compare to Sympy?  This module does a better job of handling large
matrix expressions and variable indexing.  For example, the multiplication of 5
symbolic 2x2 matrices takes ~200 microseconds, which is a bit slower than
direct numpy evaluation of ~50 microseconds.  On the other hand, the Sympy
expression takes over 4s to construct, and each entry of the matrix contains
hundreds of terms! 

This module can also take derivatives with respect to vectors using matrix
calculus.  It can even auto-differentiate with nested lists/arrays!  However,
function simplification is not as complete as Sympy, and derivatives for some
functions are not available.  There are conversions to and from Sympy in the
symbolic_sympy module, and these are complete for most "tame" expressions.


===============================================================================
Basic usage
===============================================================================

In standard usage, first create a :class:`Context` and declare any extra
functions to be used in your library. Then, and add variables and declare
expressions as necessary.::

    ctx = Context()
    x = ctx.addVar("x",'N')
    y = ctx.addVar("y",'N')
    print(x + y)  #(x+y) is an Expression object, which has not been evaluated yet
    print(flatten(x,y))   #flatten(x,y) is an Expression object, which has not been evaluated yet
    print((x + y).eval({'x':5,'y':3}))  #prints 8 (a ConstantExpression)
    print(flatten(x,y).eval({'x':5,'y':3}))  #prints [5,3] (a ConstantExpression
    print(flatten(x,y).eval({'y':[1,2]}))  #prints [x,1,2] (an Expression object)
    x.bind(5)  #x is now treated as a constant
    y.bind(3)  #y is now treated as a constant
    print((x+y).eval())  #prints 8 (a ConstantExpression)
    print((x+y).evalf())  #prints 8 (as an integer)
    x.unbind() #x is now a variable
    y.unbind() #y is now a variable

An :class:`Expression` can take on a form ``x OP y`` or ``OP(x,y,z)``, where
x,y, and z are either :class:`Variable`, :class:`Expression`, or constant
values, and ``OP`` is either a builtin function or a declared :class:`Function`
type.  Expressions can be evaluated to other Expressions via
:meth:`Expression.eval`, or to constants using
:meth:`Expression.evalf`.

There are two kinds of :class:`Variable`: 

- *Standard Variable*: This type is managed by :class:`Context` and contains
  information in the :class:`Variable` class.  Such a variable can either be
  numeric, vector, or matrix type.  It will contain size information and will
  be saved to and  loaded from disk with the :class:`Context`. 
- *User-data Variable*. This type is an unmanaged variable.  These have
  unlimited type, and can be complex Python objects, but the user must set
  these up in the context manually.  Hence, if you wish to reuse
  Expressions of user-data variables loaded from disk, you will need to
  first set up the ``Context`` appropriately with a similar   user-data object.

By default, expressions constructed in Python refer to user-data objects with
strings, e.g., ``setConfig("robot",q)`` runs the
:func:`symbolic_klampt.setConfig` Function on the "robot" user data and the
``q`` Variable. 

The :func:`expr` function is run on each argument to a :class:`Function`.
This will leave a :class:`Variable` and :class:`Expression` as is,
and most Python constants will be converted to a :class:`ConstantExpression`. 
However, Python strings will be converted to references to user-data variables.

Constant values can be floats, integers, booleans, strings, lists, and numpy
arrays (Dictionaries are not supported as constant values.)  Constants are
converted to expressions using :func:`const`, or :func:`expr`.

Type specifiers are used for type checking and to determine the type of
expressions, particularly for Jacobians.  The :class:`Type` class is used for
this. The type member specifies the general type of object, and is given by a
character:

- N: generic numeric
- I: integer
- B: boolean
- X: index (either an integer or a slice)
- V: 1D vector
- M: 2D matrix
- A: n-D array
- L: list
- U: user data
- None: indicates an unknown type

as well as an optional size and sub-type.  For L and V types, size is the length
of the array.  For M and A types,  size is the Numpy shape of the array.  For
the U type, the subtype member indicates ``__class__.__name__`` if known.


===============================================================================
Standard functions
===============================================================================

Standard operations on expressions include:

- ``expr(x)``: returns an ``Expression`` corresponding to x, whether x is a
  constant, ``Variable``, or ``Expression``
- ``deriv(expr,var)``: take the derivative of expr with respect to var.  If no
  derivative information is available for some of the expressions in expr,
  returns None.
- ``simplify(expr,context=None)``: simplifies the expression in the optimal given
  context.  Note: this is not terribly  powerful in terms of rearranging
  expressions (like x - x is not converted to 0).
- ``is_const(x)``: returns True if x evaluates to a constant.
- ``to_const(x)``: returns the constant to which x evaluates, if ``is_const(x)``.
- ``is_expr(x)``: returns True if x is an ``Expression``.
- ``is_var(x)``: returns True if x is a ``Variable`` or a ``VariableExpression``
  (i.e., monomial).
- ``to_var(x)``: returns the ``Variable`` corresponding to the ``Variable`` or
  monomial x.
- ``is_op(x,func=None)``: returns True if x is an operator.  If func is given,
  checks whether this is the the name of the function.
- ``is_zero(x)``: returns True if x is the zero(...) operator or a scalar equal to
  0.
- ``is_scalar(x,val=None)``: returns True if x is known to evaluate to a scalar. 
  If val is provided, it checks whether x is equal to val.
- ``type_of(x)``: returns the Type corresponding to x.

.. note::
    These are Python functions, not symbolic Functions.


===============================================================================
Symbolic Functions
===============================================================================

You can build an :class:`Expression` by calling a symbolic :class:`Function`,
many of which duplicate the functions of plain Python, the math module, or
Numpy.  Calling a :class:`Function` does not immediately evaluate
the function on its arguments, but instead builds an :class:`Expression` that 
is the root of a symbolic Expression Graph.

Built-in symbolic Functions include:

Shape functions
---------------

- ``range_(n)``: Evaluates to range(n).  The result can be treated like a
  vector or a list.
- ``dims(x)``: Returns the dimensionality of the input.  Equivalent to
  ``len(shape(x))``.
- ``len_(x)``: Evaluates to len(x), except if x is a scalar then it evaluates
  to 0. If x is a multi-dimensional array, this is the length of its first
  dimension.  Undefined for other forms of user data.
- ``count(x)``: Evaluates the number of numeric parameters in x. Works with
  scalars, arrays, and lists. If x is a scalar then it evaluates to 1.
  Lists are evaluated recursively.  Undefined for other forms of user data.
- ``shape(x)``: Evaluates to ``x.shape`` if x is a Numpy array, ``(len_(x),)``
  if x is a vector, and () if x is a scalar.  If x is a list, this is
  ``(len_(x),)+shape(item)`` if all of the items have the same shape,
  and otherwise it is a hyper-shape. (shortcut: "x.shape")
- ``reshape(x,s)``: Evaluates to x reshaped to the shape s.  If x is a scalar,
  this evaluates to a constant matrix.
- ``transpose(x)``: Evaluates to np.transpose(x).  (shortcut: "x.T")
- ``transpose2(x,axes)``: Tensor transpose; evaluates to np.transpose(x,axes).
- ``basis(i,n)``: Evaluates to the i'th elementary basis vector in dimension n.
- ``eye(n)``: Evaluates to np.eye(n) if n > 0, otherwise returns 1.
- ``zero(s)``: Evaluates to np.zeros(s) if s is a matrix shape or scalar > 0,
  otherwise evaluates to 0.
- ``diag(x)``: Evaluates to np.diag(x).
- ``flatten(*args)``: Evaluates to a vector where all arguments are stacked
  (concatenated) into a single vector.  Arrays are reduced to vectors by
  Numpy's flatten(), and complex objects are reduced via recursive flattening.
- ``row_stack(*args)``: Evaluates to a matrix where all arguments are stacked
  vertically.  1D arrays are treated as row vectors. If only a single list
  element is provided, then all arguments are stacked.
- ``column_stack(*args)``: Evaluates to a matrix where all arguments are stacked
  horizontally.  1D arrays are treated as column vectors. If only a single
  list element is provided, then all arguments are stacked.

Comparisons and logical functions
----------------------------------

- ``eq(lhs,rhs)``: Evaluates to lhs = rhs (shortcut: "lhs = rhs").
- ``ne(lhs,rhs)``: Evaluates to lhs != rhs (shortcut: "lhs != rhs").
- ``le(lhs,rhs)``: Evaluates to lhs <= rhs (shortcut: "lhs <= rhs").
- ``ge(lhs,rhs)``: Evaluates to lhs >= rhs (shortcut: "lhs >= rhs").
- ``not_(x)``: Evaluates to not x (shortcut: "not x").
- ``or_(x,y)``: Evaluates to x or y (shortcut: "x or y").
- ``and_(x,y)``: Evaluates to x and y (shortcut: "x and y").
- ``any_(*args)``: Evaluates to ``any(*args)``
- ``all_(*args)``: Evaluates to ``all(*args)``

Arithmetic functions
---------------------

- ``neg(x)``: Evaluates to -x (shorcut "-x").
- ``abs_(x)``: Evaluates to abs(x) (shortcut: "abs(x)"). Works with arrays too
  (elementwise)
- ``sign(x)``: Evaluates the sign of x.  Works with arrays too (elementwise).
- ``add(x,y)``: Evaluates to x + y (shortcut: "x + y").  Works with arrays too.
- ``sub(x,y)``: Evaluates to x - y (shortcut: "x - y").  Works with arrays too,
  and vector - scalar.
- ``mul(x,y)``: Evaluates to x * y (shortcut: "x * y").  Works with arrays too
  (elementwise multiplication).
- ``div(x,y)``: Evaluates to x / y (shortcut: "x / y").  Works with arrays too
  (elementwise division), and vector / scalar.
- ``pow_(x,y)``: Evaluates to ``pow(x,y)`` (shortcut "x**y").
- ``dot(x,y)``: Evaluates to ``np.dot(x,y)``.
- ``outer(x,y)``: Evaluates to ``np.outer(x,y)``.
- ``tensordot(x,y,axes)``: Evaluates to ``np.tensordot(x,y,axes)``.
- ``max_(*args)``: Evaluates to the maximum of the arguments.
- ``min_(*args)``: Evaluates to the minimum of the arguments.
- ``argmax(*args)``: Evaluates to the index of the maximum of the argments.
- ``argmin(*args)``: Evaluates to the index of the minimum of the argments.
- ``cos(x)``: Evaluates to math.cos(x).
- ``sin(x)``: Evaluates to math.sin(x).
- ``tan(x)``: Evaluates to math.tan(x).
- ``sqrt(x)``: Evaluates to math.sqrt(x).
- ``exp(x)``: Evaluates to math.exp(x).
- ``log(x)``: Evaluates to math.log(x) (base 10).
- ``ln(x)``: Evaluates to math.ln(x) (natural log).
- ``sum_(*args)``: Evaluates to sum(args).  If arguments are vectors or matrices,
  then the result is also a vector or matrix. This is somewhat different
  behavior from sum(x) if x is a list.
- ``weightedsum(v1,...,vn,w1,...,wn)``: Evaluates to :math:`w1*v1+...+wn*vn`. 

Accessors
---------

- ``getitem(vec,index)``: Evaluates to ``vec[index]``.  This also supports slices
  and tuples, as well as lists (Numpy fancy indexing). (shortcut:
  "vec[index]")

- ``setitem(vec,index,val)``: Evaluates to vec except with vec[index] set to val.
  Equivalent to Python code::

      temp = vec[:]
      vec[index]=val
      return temp

- ``getattr_(object,attr)``: returns the value of a given attribute under the
  given object. For example, ``getattr_(traj,const("milestones"))`` gets the
  milestone list of a user-data ``Trajectory`` named ``traj``.

  If the result is a function, it will be called with no arguments.  For
  example, ``getattr_(robot,const("getJointLimits"))`` will return the robot's
  joint limits.

  (shortcut: "object.attr", where object is a UserDataExpression)

- ``setattr_(object,attr,val)``: returns a modified version of the given class
  object, where ``value`` is assigned to the attribute ``attr``. For example,
  ``setattr_(traj,const("times"),[0,0.5,0.1])`` sets the ``times`` attribute
  of a ``Trajectory`` to [0,0.5,1].

  Note: this operation modifies the object itself. 

  If the attribute ``attr`` is a function, it will be called with the argument
  ``val``. This allows setting operations to be called.

Conditionals
------------

- ``if_(cond,trueval,falseval)``: If cond evaluates to True, this expression
  evaluates to trueval. Otherwise, it evaluates to falseval.

Arrays
-------

Arrays are mostly interchangable with vectors (numpy 1-D arrays), except they
can contain objects of varying type/size. Arrays of varying type/size should
only be used as arguments in the flatten or the special looping functions
(``forall``, ``summation``, etc). In many cases the Python construction
``[e1,e2,...,en]`` where en is an ``Expression``, will be interpreted correctly
as an ``array`` Expression. To handle nested lists of Expressions, and
to ensure that your lists can be saved and loaded, you may need to use these
functions.

- ``array(*args)``: Creates a list or numpy array of the given arguments. This can
  accept arbitrary arguments, such as variable size vectors.  A Numpy array is
  produced only if the items have compatible types and dimensions.
- ``list_(*args)``: Creates a list of the given arguments. This can accept
  arbitrary arguments, such as variable size vectors.  No attempt is made to
  convert to a numpy array.
- ``tuple_(*args)``: Creates a tuple of the given arguments. This can accept
  arbitrary arguments, such as variable size vectors.
- ``zip_(collection1,collection1,...)``: Does the same thing as the Python zip
  function, returning a list of tuples.

Looping
--------

Special functions are available for temporary variable substitution and
emulation of ``for`` loops.  In each of the following, var can be a variable
or userData referenced in the ``Expression`` ``expr``:

- ``subs(expr,var,value)``: evaluates expr with var substituted with value.  For
  example, ``subs(const(2)*"i","i",3.5)`` yields 2*3.5
- ``map_(expr,var,values)``: like the Python ``map`` function, evaluates to a
  list where each entry evaluates ``expr`` with ``var`` substituted with
  a value from the list ``values``.  For example, if x is a ``Variable``, then
  ``map_(x**"i","i",range_(3))`` yields the list ``[x**0, x**1, x**2]``
- ``forall(expr,var,values)``: True if, for every value in the list ``values``,
  ``expr`` evaluates to nonzero when ``var`` is substituted with that value.
  Equivalent to ``all_(*[subs(expr,var,value) for value in values])``
- ``forsome(expr,var,values)``: True if, for some value in the list ``values``,
  ``expr`` evaluates to nonzero when ``var`` is substituted with that value.
  Equivalent to ``any_(*[subs(expr,var,value) for value in values])``
- ``summation(expr,var,values)``: The sum of ``expr`` over ``var`` when ``var``
  is substituted with each value in the list ``values``. Equivalent to
  ``sum_(*[subs(expr,var,value) for value in values])``

Nested iteration can be performed, such as
``summation(summation(expr("x")**"y","x",[1,2]),"y",[0,1])``


===============================================================================
Defining your own Functions
===============================================================================

If you want to use your own Python functions, you can use ``Context.declare``.
By default this will use the same function signature that you used to define
the function, or you can provide a new signature (name, arguments).
The convention used in the built-in symbolic libraries is to prepend an
underscore to the underlying Python function, and then declare the function
to the undecorated name.::

    def _f(x,y):
        return 2*x*y
    ctx = Context()
    f = ctx.declare(_f,"f")
    print(f(3,4))   #prints f(3,4)
    print(f(3,4).evalf())  #prints 24, since 2*3*4=12.

At this point, the module does not know how to take derivatives, so any
``deriv(f(...),arg)`` call will return None. To set derivatives, you can
use the ``setDeriv`` or ``setJacobian`` functions::

    f.setDeriv("x",(lambda x,y,dx:2*dx*y))
    f.setDeriv("y",(lambda x,y,dy:2*x*dy))

OR::

    f.setJacobian("x",(lambda x,y:2*y))
    f.setJacobian("y",(lambda x,y:2*x))

Alternatively, you can declare an ``Expression`` as a ``Function``.  Here you
need to define the function name and argument list so that unbound variables in
the ``Expression`` are bound to the arguments.::

    f = ctx.declare(const(2)*expr("x")*expr("y"),"f",["x","y"])

This form will automatically obtain derivatives for you.

Functions can also specify argument types and return types, which are used
for type checking at Expression creation time. This is quite helpful for
debugging, since you do not have to evaluate the ``Expression`` to find
mismatched types, like indexing a vector with a list. To declare types for
custom functions, use the ``setArgType`` and ``setReturnType`` methods::

    #numeric input and output
    f.setArgType(0,'N')
    f.setArgType(1,'N')
    f.setReturnType('N')


===============================================================================
Gotchas
===============================================================================

Most conversions of arguments to Expressions are done for you during the
course of calling operators or calls to symbolic functions, but if you call
operators on Python constants, you will invoke the standard Python operation.
For example, 5 + 7 produces 12 because 5 and 7 are Python constants, but if you
want to build the symbolic ``Expression`` that represents "5 + 7", you will need
to use ``const(5)+const(7)`` or ``expr(5)+expr(7)``.

Since raw strings are interpreted as references to user-data variables, to
create string constants, you will need to wrap the string using the ``const()``
function, e.g., ``const("text")``.  This is mostly relevant for the
``get/setattr_`` functions.  Suppose Point is a class with members Point.x and
Point.y, the expression that accesses the x member of the user data ``Point p``
is built with ``getattr_("p",const("x"))``.  Using ``__getattr__`` operator
overloading you can also use ``expr("p").x``

Python Lists as arguments are handled in somewhat of a tricky way.  If the list
is compatible with a Numpy array  (i.e., each element is numeric or equal-sized
arrays), it will be converted to a Numpy array.  Otherwise, (e.g., a non-
constant Expression is inside or the elements do not have the same size) it will
be converted to an ``array(e1,...,en)`` Expression.  Tuples are NOT
converted in the same way, so that Numpy matrix indices can be preserved.
Tuples of expressions are not supported implicitly, instead you will have to use
``tuple_(e1,...,en)``.

Comparison testing on Expressions, such as ``e != 0`` or ``e1 == e2``, do
not return True or False. Instead, they return  Expressions themselves!  As
a result, standard Python if statements cannot be used on Expressions.  Instead,
you  will need to run ``eval()`` or ``evalf()`` on the result if you want to get
its truth value.  (``eval()`` does return an  Expression, but if the result of
``eval()`` is a ConstantExpression then it may be directly tested for its truth
value.) To help you remember this, an exception will be raised if you try to use
an expression as a condition in a Python ``if`` statement.

Note also that ``e1 == e2`` does NOT test for whether the two expressions are
logically equivalent.  As an example, the expression ``x1+x2 == x2+x1`` does not
test equivalence as you might expect, nor would even ``eval()`` to true unless
x1 and x2 were given.  In order to test whether two expressions are
*syntactically* equivalent, you can use ``e1.match(e2)``.  It is computationally
intractable in general to determine whether two expressions are logically
equivalent, so we don't even try.

Derivatives are handled as usual for scalar/vector quantities, giving Jacobian
matrices in the form::

    [df1/dx1 ... df1/dxn ]
    [...                 ]
    [dfm/dx1 ... dfm/dxn ].

However, if the function or variable has an "exotic" type, like a matrix,
tensor, or nested list, then the Jacobian is taken with respect to the
**flattened** version of the function and the variable.  The way this is
handled can be seen in some complex derivative expressions, which will have
various reshape operations.

Many standard Python operators -- +,-,*,/,**, and, or, not, comparison tests,
the [] indexing operator, and list construction via [a,b] -- are supported
directly on Variables and Expressions.  Compound slices (``matrix[:,:]``) have
not yet been thoroughly tested.  Other standard functions -- len, min, max, sum,
abs, any, and all -- have direct analogues in this package, with a trailing
underscore added. If statements and list comprehensions can be emulated via the
``if_(cond,trueval,falseval)`` statement and the ``map_(expr,var,range)
statement``.  There is no support for the in statement, bitwise operators, or
procedural code (e.g., statements with side-effects).

If-elif-...-else blocks can be emulated using a multiplexer
``array(expr1,expr2,...,exprn)[switch]`` where ``switch`` takes on the values
0,...,n-1.  This expression is lazy-evaluated so that only the selected
expression is expanded.  Standard ``if_`` statements are also lazy-evaluated.

Thread-safety: Expression evaluation, derivatives, and simplification are NOT
thread safe.  Common Expressions used between threads should be deep-copied
before use.


===============================================================================
Performance and Expression DAGs
===============================================================================

If your expression repeatedly performs computationally expensive operations, it
is helpful to gather sub-expressions into the same Python objects.  As an
example, consider a Variable a, and the expression d given by the following
construction::

    b = (a+a)  
    c = (b+b)
    d = (c+c)

Expanded, d = (c+c) = ((b+b)+(b+b)) = (((a+a)+(a+a))+((a+a)+(a+a)). However,
when ``d.eval({'a':1})`` is called, this module is smart enough to evaluate a,
b, and c only once.  This becomes really helpful in derivative evaluation as
well, since the derivative expression will also try to reuse common sub-
expressions.

Now what if we want to evaluate several expressions at once?  This naive code
will evaluate a, b, c, and d 1000 times::

    ds = [(d+i).eval({'a':1}) for i in range(1000)]

In order to take advantage of the sub-expression d, we can put everything into a
single ``array()`` expression::

    ds = array(*[(d+i) for i in range(1000)]).eval({'a':1})

which will only evaluate a,b,c, and d once.


===============================================================================
IO
===============================================================================

There are three forms of Expression IO in symbolic_io:

1. Standard Python printing: ``str(expr)`` (output only)
2. Human-readable strings: :func:`symbolic_io.toStr` (output) /
   :func:`symbolic_io.fromStr` (input)
3. JSON-like objects: :func:`symbolic_io.toJson` (output) /
   :func:`symbolic_io.fromJson` (input)

Method 1 is the most readable when printed to the console, but can't be read.

Method 2 is somewhat readable, and is compatible with the optimization problem
editor.  Note that there may be some subtle problems with IO for lists vs numpy
arrays, since lists are automatically converted to numpy arrays when possible.
It hasn't been tested extremely thoroughly yet.

Method 3 is the least ambiguous but it uses a nested JSON structure to represent
the expression graph.  Hence it is rather verbose.

In Methods 2 and 3, if the expression graph is a DAG rather than a tree, a
special notation is used to represent repeated references to nodes.  In
particular, any common ConstantExpression and OperatorExpression objects are
"tagged" with a unique id string.

In Method 2, the first time a sub-expression appears it's tagged with a suffix
#id.  Then, subsequent references are retrieved with the expression @id.  As an
example, the string ``(x+y)#1*@1`` evaluates to ``(x+y)*(x+y)``, since '#1'
assigns ``(x+y)`` to the id '1' and '@1' retrieves it.


===============================================================================
Code generation
===============================================================================

- context.makePyFunction(expr,varorder=None): creates a Python function that
  evaluates ``expr`` given arguments whose values are to be assigned to each
  ``Variable`` in ``varorder``.
- context.makeFlatFunction(expr,varorder=None): creates a Python function that
  evaluates ``expr`` given a flattened list or vector of parameters, which can
  be unraveled to values to be assigned to each Variable in ``varorder``.
- symbolic_io.latex(expr): produces a LaTex string representing the expression
  (requires Sympy)
- symbolic_io.codegen(exprs,language): produces C / Matlab code to evaluate the
  ``Function`` or named expressions (requires Sympy).


===============================================================================
Sympy integration
===============================================================================

Sympy integration is quite automatic.  See :func:`symbolic_sympy.exprToSympy`
and :func:`symbolic_sympy.exprFromSympy` in symbolic_sympy.py. Built-in
functions are converted mostly bidirectionally and hence support all aspects of
both libraries.

Array operations are converted to Sympy Matrix operations, which expand all
entries into scalar expressions, so array operations like dot() cannot be
converted to Sympy and then back into the equvalent symbolic operation.

Custom symbolic functions are converted to sympy Function(f) objects, and it
should be noted that these do not support code generation.

Special Sympy functions are automatically converted for use in the symbolic
module, including differentiation.  (Note that they cannot be saved to / loaded
from disk, though.)


===============================================================================
Wish list
===============================================================================

- Sparse matrix support when obtaining jacobians -- especially in block matrix
  form.
- Compiled code generation on vectors/matrices -- maybe integration with
  TensorFlow / PyTorch?

===============================================================================
Module summary
===============================================================================


External interface
-----------------------

.. autosummary::
    expr
    Type
    deriv
    simplify
    supertype
    type_of
    const
    is_const
    to_const
    is_scalar
    to_scalar
    is_zero
    is_var
    to_var
    is_expr
    is_op
    is_sparse
    to_monomial
    to_polynomial

Internal implementation
-----------------------

.. autosummary::
    Type
    Context
    Function
    Variable
    Wildcard
    Expression
    ConstantExpression
    UserDataExpression
    VariableExpression
    OperatorExpression


Symbolic standard functions
---------------------------

.. autosummary::
    range_
    len_
    count
    shape
    reshape
    transpose
    transpose2
    dims
    eye
    basis
    zero
    diag
    eq
    ne
    le
    ge
    not_
    or_
    and_
    neg
    abs_
    sign
    add
    sub
    mul
    div
    pow_
    dot
    outer
    tensordot
    if_
    max_
    min_
    argmax
    argmin
    cos
    sin
    tan
    arccos
    arcsin
    arctan
    arctan2
    sqrt
    exp
    log
    ln
    sum_
    any_
    all_
    getitem
    setitem
    getattr_
    setattr_
    flatten
    row_stack
    column_stack
    array
    list_
    tuple_
    zip_
    weightedsum
    subs
    map_
    forall
    forsome
    summation

"""

import weakref
from . import vectorops
import operator
import numpy as np
import math
import copy
import itertools
import warnings
from builtins import object

_DEBUG_CACHE = False
_DEBUG_DERIVATIVES = False
_DEBUG_SIMPLIFY = False
_DEBUG_TRAVERSE = False
#all recursion items to debug. Warning: don't include 'depth' or None
_DEBUG_TRAVERSE_ITEMS = {}
SCALAR_TYPES = 'NIB'
ARRAY_TYPES = 'AVM'
BUILTIN_TYPES = 'NIBAVMX'
VAR_DERIV_PREFIX = 'D'
TYPE_CHECKING = True
SHAPE_CHECKING = True
_PY_INT_TYPES = (int,np.int32,np.int64)
_PY_FLOAT_TYPES = (float,np.float32,np.float64)
_PY_NUMERIC_TYPES = _PY_INT_TYPES + _PY_FLOAT_TYPES
_PY_PRIMITIVE_TYPES = _PY_NUMERIC_TYPES + (bool,str)
_PY_CONST_TYPES = _PY_INT_TYPES + _PY_FLOAT_TYPES + (bool,np.ndarray)

class Type:
    """A specification of a variable/expression type.

    Attributes:
        char (str): a character defining the type. Valid values are
            N, I, B, X, A, V, M, L, U, and None.
        size: None (no size specified), the length of the array (L
            and V types), or the Numpy shape (A, M types)
        subtype (str, Type, or list of Types, optional):

          * U type: the name of the class, if known.
          * L type: either a single Type defining the list's element
            types, or a list of Types defining individual element
            types (of the same length as size).
    """
    def __init__(self,type,size=None,subtype=None):
        if isinstance(type,Type):
            self.char = type.char
            self.size = type.size if size is None else size
            if subtype is None:
                self.subtype = type.subtype
            elif isinstance(subtype,list):
                self.subtype = [Type(s) for s in subtype]
            else:
                self.subtype = Type(subtype)
        else:
            if type is not None:
                assert isinstance(type,str),"Type argument "+str(type)+" must be a string, not "+type.__class__.__name__
            if type is not None and len(type) > 1:
                #assume it's a user type
                self.char = 'U'
                self.size = size
                self.subtype = type
            else:
                self.char = type
                self.size = size
                if subtype is None:
                    self.subtype = subtype
                elif isinstance(subtype,list):
                    self.subtype = [Type(s) for s in subtype]
                else:
                    self.subtype = Type(subtype)
        assert self.size is None or isinstance(self.size,_PY_INT_TYPES) or isinstance(self.size,(list,tuple,dict,np.ndarray)),"Erroneous type of size? "+str(size)+": "+size.__class__.__name__
        if isinstance(self.size,dict):
            assert self.char in 'AL',"Hyper-shape objects must have A or L type"
            self.char = 'L'
            subtype = self.subtype
            size = self.size
            self.size = len(self.size)
            self.subtype = [None]*self.size
            for (k,v) in size.items():
                if isinstance(v,dict):
                    self.subtype[k] = Type('L',v,subtype)
                else:
                    self.subtype[k] = Type('A',v,subtype)
        elif isinstance(self.size,np.ndarray):
            self.size = tuple(self.size.astype(np.int64))
        elif isinstance(self.size,list):
            self.size = tuple(self.size)
        if self.char == 'A' and isinstance(self.size,_PY_INT_TYPES):
            self.size = (self.size,)
        if self.char is not None and self.char in 'AM':
            assert not isinstance(self.size,_PY_INT_TYPES)
                    
    def is_scalar(self):
        if self.char is None: return False
        return self.char in SCALAR_TYPES
    def shape(self,hypershape=True):
        """Returns the Numpy shape or hypershape of this type.  If it's array-
        like, the return value is a tuple. Otherwise, it's a dict.
        
        A ValueError is raised if no size is specified.
        """
        if self.char is None: 
            raise ValueError("Item of unknown type does not map to a Numpy shape")
        if self.char in SCALAR_TYPES: return ()
        if self.size is None:
            raise ValueError("Item of type "+self.char+" does not have a defined size")
        elif self.char == 'V': return (self.size,)
        elif self.char in ['M','A']: return self.size
        if self.char == 'L' and hypershape:
            if self.subtype is not None:
                res = {}
                if isinstance(self.subtype,list):
                    for i,st in enumerate(self.subtype):
                        res[i] = st.shape()
                else:
                    stsh = self.subtype.shape()
                    for i in range(self.size):
                        res[i] = stsh
                return res
        raise ValueError("Item of type "+self.char+" does not map to a Numpy shape")
    def len(self):
        """Returns the number of first-level sub-items of this type. None is returned if no
        size is specified."""
        if self.char is None: 
            raise ValueError("Item of unknown type does not have a len")
        if self.char in SCALAR_TYPES: return 0
        elif self.char == 'V': return self.size
        elif self.char == 'L':
            return self.size
        elif self.char in 'MA':
            return self.size[0]
        else:
            raise ValueError("Type "+self.char+" has no len")
    def count(self):
        """Returns the number of free parameters in this type.  None is returned if no
        size is specified."""
        if self.char is None: 
            raise ValueError("Item of unknown type does not have a count")
        if self.char in SCALAR_TYPES: return 1
        if self.char == 'L':
            try:
                if self.subtype is not None:
                    if isinstance(self.subtype,list):
                        res = sum(s.count() for s in self.subtype)
                        assert isinstance(res,int),"Result of count isn't an integer? %s"%(res.__class__.__name__,)
                        return res
                    elif self.subtype.len() is not None:
                        res = self.size*self.subtype.count()
                        assert isinstance(res,int),"Result of count isn't an integer? %s"%(res.__class__.__name__,)
                        return res
                    return None  #no length specified
            except Exception:
                return None
            return self.size
        if self.char in 'MA':
            if self.size is None: return None
            return int(np.product(self.size))
        assert isinstance(self.len(),int),"Result of count isn't an integer? %s"%(self.len().__class__.__name__,)
        return self.len()
    def dims(self):
        """Returns the number of entries in the shape of this type. For compound list types, this is
        1+subtype.dims()."""
        if self.char is None: 
            raise ValueError("Item of unknown type does not have a dimensionality")
        if self.char in SCALAR_TYPES: return 0
        elif self.char == 'V': return 1
        elif self.char == 'M': return 2
        elif self.char == 'A':
            if self.size is None: raise ValueError("Array of unspecified shape does not have a defined dimensionality")
            return len(self.size)
        elif self.char == 'L':
            if self.subtype is None:
                raise ValueError("List of unspecified subtype does not have a defined dimensionality")
            if isinstance(self.subtype,list):
                return 1 + max(st.dims() for st in self.subtype)
            return 1 + self.subtype.dims()
        raise ValueError("Item of type "+self.char+" does not have a dimensionality")
    def match(self,obj,strict=False):
        """Returns true if the given object type matches all aspects of this specification.
        Integers and booleans match with numeric, vectors and matrices match with array.

        obj is a Type, str, or Python object.  If obj is a str, it is either a character specifier
        or a class name, and sizes aren't checked.  If obj is an object this is equivalent to
        self.match(type_of(obj)).

        If obj doesn't have a size then it matches only strict = False, or if this
        specification also doesn't have a size.
        """
        if isinstance(obj,Type):
            if obj.char is None and not strict: return True
            if self.char is None: return True
            elif obj.char == 'U':
                if self.char == 'U':
                    return self.subtype is None or obj.subtype is None or self.subtype == obj.subtype
                else:
                    return obj.subtype is None or self.match(obj.subtype,strict)
            elif self.char == 'N' and obj.char in SCALAR_TYPES: return True
            elif self.char == 'A' and obj.char in ARRAY_TYPES:
                if self.size is None: return True
                if obj.char == 'V': return self.size[0] == obj.size
                return self.size == obj.size
            elif obj.char == 'A' and self.char in ARRAY_TYPES:
                if obj.size is None: return True
                if self.char == 'V': return obj.size[0] == self.size
                return self.size == obj.size
            elif self.char == 'L':
                if obj.char in ARRAY_TYPES:
                    if self.size is None: return True
                    if obj.size is None: return not strict
                    assert isinstance(self.size,_PY_INT_TYPES),"Invalid size of list: "+str(self.size)
                    if obj.len() != self.size: return False
                    if self.subtype is None: return True
                    if isinstance(self.subtype,list):
                        ost = obj.itemtype()
                        return all(st.match(ost,strict) for st in self.subtype)
                    return self.subtype.match(obj.itemtype(),strict)
                elif obj.char == 'L':
                    if self.size is None: return True
                    if obj.size is None: return not strict
                    if obj.size != self.size: return False
                    if self.subtype is None: return True
                    if isinstance(self.subtype,list):
                        if isinstance(obj.subtype,list):
                            return all(st.match(ost,strict) for st,ost in zip(self.subtype,obj.subtype))
                        return all(st.match(obj.subtype,strict) for st in self.subtype)
                    return self.subtype.match(obj.itemtype(),strict)
                else:
                    return False
            elif self.char == 'X':
                return obj.char in 'VIX' or (not strict and obj.char == 'N')
            if not strict and self.char in SCALAR_TYPES and obj.char in SCALAR_TYPES: return True
            return self.char == obj.char and (self.size is None or (obj.size is None and not strict) or self.size == obj.size)
        elif isinstance(obj,str):
            if self.char is None: return True
            elif self.char == obj: return True
            elif self.char == 'N': return obj in SCALAR_TYPES
            elif self.char == 'A': return obj in ARRAY_TYPES
            elif self.char == 'L': return obj in ARRAY_TYPES
            elif self.char == 'X': return obj in 'VIX'
            elif self.char == 'U': return self.subtype is None or obj == self.subtype
            if not strict and self.char in SCALAR_TYPES and obj in SCALAR_TYPES: return True
            return False
        elif obj is None: return not strict
        else:
            return self.match(type_of(obj),strict)
    def itemtype(self,index=None):
        """If this is an array or list type, returns the Type of each item accessed by x[i].
        Otherwise returns None.
        """
        if self.char == 'V':
            if self.subtype is None:
                return Type('N')
            else:
                return self.subtype
        elif self.char == 'M':
            if self.size is None:
                return Type('V',None,self.subtype)
            else:
                return Type('V',self.size[1],self.subtype)
        elif self.char == 'A':
            if self.size is None:
                return Type(None)
            if len(self.size) == 0: return None
            elif len(self.size) == 1: return Type('N') if self.subtype is None else self.subtype
            else: return Type(self,self.size[1:])
        elif self.char == 'L':
            if isinstance(self.subtype,list):
                if index is not None:
                    return self.subtype[index]
                return supertype(self.subtype)
            return self.subtype
        else:
            return None
    def __str__(self):
        """Returns a string representation of this type, suitable for brief printouts"""
        if self.char is None:
            return 'None'
        if self.size is None and self.subtype is None:
            return self.char
        elif self.subtype is None:
            return self.char+' size '+str(self.size)
        elif self.size is None:
            if isinstance(self.subtype,list):
                return self.char+' (subtypes '+','.join(str(s) for s in self.subtype)+')'
            else:
                return self.char+' (subtype '+str(self.subtype)+')'
        else:
            if isinstance(self.subtype,list):
                return self.char+' size '+str(self.size)+' (subtypes '+','.join(str(s) for s in self.subtype)+')'
            else:
                return self.char+' size '+str(self.size)+' (subtype '+str(self.subtype)+')'

    _TYPE_DESCRIPTIONS = {'N':'numeric','I':'int','B':'bool','X':'index (int or slice)',
        'V':'vector','M':'matrix','A':'array','L':'list','U':'user data'}

    def info(self):
        """Returns a verbose, human readable string representation of this type"""
        if self.char is None:
            return 'unknown'
        typedesc = "%s (%s)"%(self._TYPE_DESCRIPTIONS[self.char],self.char)
        if self.size is not None:
            if isinstance(self.size,(list,tuple)):
                sizedesc = 'x'.join(str(v) for v in self.size)
                typedesc = sizedesc + ' ' + typedesc
            else:
                sizedesc = str(self.size)
                typedesc = sizedesc + '-' + typedesc
        if self.subtype is None:
            return typedesc
        else:
            if self.char == 'U':
                return '%s of type %s'%(typedesc,self.subtype)
            if isinstance(self.subtype,list):
                return '%s of subtypes [%s]'%(typedesc,','.join(str(s) for s in self.subtype))
            else:
                return '%s of subtype (%s)'%(typedesc,self.subtype.info())


Numeric = Type('N')
Integer = Type('I')
Boolean = Type('B')
Index = Type('X')
Vector = Type('V')
Vector2 = Type('V',2)
Vector3 = Type('V',3)
Matrix = Type('M')
Matrix2 = Type('M',(2,2))
Matrix3 = Type('M',(3,3))


def _is_exactly(a,b):
    """Returns True if these are exactly the same class and match.
    Works for numpy arrays, scalars, and class objects (this latter
    case is matched with the 'is' keyword)
    """
    if a.__class__ == b.__class__:
        if isinstance(a,np.ndarray):
            return np.all(a==b)
        elif isinstance(a,_PY_PRIMITIVE_TYPES + _PY_FLOAT_TYPES):
            return a == b
        return a is b
    return False

def _ravel(obj):
    """A hyper-version of numpy's ravel function that also works for nested lists"""
    if isinstance(obj,np.ndarray):
        return np.ravel(obj)
    elif isinstance(obj,(list,tuple)):
        hstack_args = [_ravel(v) for v in obj]
        hstack_args = [v for v in hstack_args if len(v) > 0]
        if len(hstack_args) == 0:
            return np.array([])
        elif len(hstack_args) == 1:
            return hstack_args[0]
        try:
            return np.hstack(hstack_args)
        except ValueError as e:
            return np.array([])
    else:
        return np.array([obj])

def _hyper_count(obj):
    if isinstance(obj,np.ndarray):
        return np.product(obj.shape)
    elif isinstance(obj,(list,tuple)):
        return sum(_hyper_count(v) for v in obj)
    else:
        return 1

def _has_hyper_shape(obj):
    if isinstance(obj,np.ndarray):
        return False
    elif isinstance(obj,(list,tuple)):
        if any(_has_hyper_shape(v) for v in obj): return True
        subshapes = [_hyper_shape(v) for v in obj]
        if len(subshapes) == 0: return False
        if all(isinstance(s,tuple) and s==subshapes[0] for s in subshapes):
            return False
        return True
    else:
        return ()

def _hyper_shape(obj):
    if isinstance(obj,np.ndarray):
        return obj.shape
    elif isinstance(obj,(list,tuple)):
        subshapes = [_hyper_shape(v) for v in obj]
        if len(subshapes) == 0: return ()
        if all(isinstance(s,tuple) and s==subshapes[0] for s in subshapes):
            return (len(obj),)+subshapes[0]
        return dict([(i,s) for i,s in enumerate(subshapes)])
    else:
        return ()

def _is_numpy_shape(sh):
    if isinstance(sh,(tuple,list)):
        return (len(sh)==0 or all(isinstance(x,_PY_INT_TYPES) for x in sh))
    elif isinstance(sh,np.ndarray):
        return len(sh.shape) == 1 and sh.dtype.char != 'O'
    return False

def _is_hyper_shape(sh):
    if isinstance(sh,(tuple,list)):
        return len(sh)==0 or all(isinstance(x,_PY_INT_TYPES) for x in sh)
    elif isinstance(sh,np.ndarray):
        return len(sh.shape) == 1 and sh.dtype.char != 'O'
    elif isinstance(sh,dict):
        for (k,v) in sh.items():
            if not isinstance(k,_PY_INT_TYPES): return False
            if not _is_hyper_shape(v): return False
        return True

def _concat_hyper_shape(sh1,sh2):
    """Describes the shape of the object where every primitive element of the type described by sh1
    is replaced by an element of the type described by sh2.  In other words, performs the Cartesian
    product."""
    if _is_numpy_shape(sh2):
        if isinstance(sh1,(tuple,list,np.ndarray)):
            return tuple(sh1) + tuple(sh2)
        else:
            assert isinstance(sh1,dict),"Erroneous hypershape object class "+sh1.__class__.__name__
            res = {}
            for (k,v) in sh1.items():
                res[k] = _concat_hyper_shape(v,sh2)
            return res
    else:
        if isinstance(sh1,(tuple,list,np.ndarray)):
            if len(sh1) == 0:
                return sh2
            elif len(sh1) == 1:
                return dict((i,sh2) for i in range(sh1[0]))
            else:
                raise NotImplementedError("TODO: cartesian product of multidimensional hyper-shapes")
        else:
            assert isinstance(sh1,dict),"Erroneous hypershape object class "+sh1.__class__.__name__
            res = {}
            for (k,v) in sh1.items():
                res[k] = _concat_hyper_shape(v,sh2)
            return res

def _unravel(obj,shape):
    """The reverse of _ravel:
    - obj is a 1-D Numpy array (containing a sufficient number of items that can be extracted)
    - shape is a Numpy shape or a hyper-shape.

    Returns a nested structure of numpy array or lists in the hyper-shape specified by shape.
    """
    assert isinstance(obj,np.ndarray)
    olen = obj.shape[0]
    if isinstance(shape,dict):
        res = [None]*len(shape)
        ofs = 0
        for (i,s) in shape.items():
            res[i] = _unravel(obj[ofs:],s)
            ofs += _hyper_count(res[i])
            if ofs > olen: raise ValueError("Insufficent sized vector to extract the desired parameters")
        return res
    else:
        if len(shape) == 0:
            if olen == 0: raise ValueError("Insufficent sized vector to extract the desired parameters")
            return obj[0]
        arrlen = np.product(shape)
        if arrlen > olen: raise ValueError("Insufficent sized vector to extract the desired parameters")
        return obj[:arrlen].reshape(shape)

def _addDictPrefix(prefix,items,prefixedKeys=None):
    """Recursively adds the prefix prefix to the keys in items, and any sub-dicts."""
    if not isinstance(items,dict): return items
    newdict = dict()
    for (k,v) in items.items():
        if prefixedKeys is None or k in prefixedKeys:
            if prefix+k in newdict:
                raise RuntimeError("Adding dictionary prefix %s created a name clash with key %s"%(str(prefix),str(k)))
            newdict[prefix+k] = _addDictPrefix(prefix,v,prefixedKeys)
        else:
            newdict[k] = _addDictPrefix(prefix,v,prefixedKeys)
    return newdict

class Context:
    """Base class for all symbolic operations.  Define variables, expressions, and user data here.
    """
    def __init__(self):
        self.variables = []
        self.variableDict = dict()
        self.userData = dict()
        self.expressions = dict()
        self.customFunctions = dict()
    def addVar(self,name,type='V',size=None):
        """Creates a new Variable with the given name, type, and size.

        Valid types include:
        - V: vector (default)
        - M: matrix
        - N: numeric (generic float or integer)
        - B: boolean
        - I: integer
        - A: generic array
        - X: index

        size is a hint for vector variables to help determine how many entries
        the variable should contain.  It can be None, in which case the size is 
        assumed unknown.
        """
        if name in self.variableDict:
            raise RuntimeError("Variable "+name+" already exists")
        if name in self.userData:
            raise RuntimeError("Variable "+name+" conflicts with a user data")
        if name in self.expressions:
            raise RuntimeError("Variable "+name+" conflicts with an expression")
        v = Variable(name,Type(type,size),self)
        self.variables.append(v)
        self.variableDict[name] = v
        return v
    def addVars(self,prefix,N,type='N',size=None):
        """Creates N new Variables with the names prefix+str(i) for i in 1,...,N.
        type is the same as in addVar, but by default it is numeric.
        """
        res = []
        for i in range(N):
            res.append(self.addVar(prefix+str(i+1),type,size))
        return res
    def addUserData(self,name,value):
        """Adds a new item to user data.  These are un-serializable objects that can
        be referred to by functions, but must be restored after loading."""
        if name in self.variableDict:
            raise RuntimeError("User data "+name+" conflicts with a variable")
        if name in self.expressions:
            raise RuntimeError("User data "+name+" conflicts with an expression")
        if name in self.customFunctions or name in _builtin_functions:
            raise RuntimeError("User data "+name+" conflicts with a function")
        self.userData[name] = value
        return UserDataExpression(name)
    def addExpr(self,name,expr):
        """Declares a named expression"""
        if name in self.variableDict:
            raise RuntimeError("Expression "+name+" conflicts with a variable")
        if name in self.expressions:
            raise RuntimeError("Expression "+name+" already exists")
        if name in self.customFunctions or name in _builtin_functions:
            raise RuntimeError("Expression "+name+" conflicts with a function")
        assert isinstance(name,str)
        assert isinstance(expr,Expression)
        self.expressions[name] = expr
        return expr
    def declare(self,func,fname=None,fargs=None):
        """Declares a custom function.  If fname is provided, this will be how
        the function will be referenced.  Otherwise, the Python name of the
        function is used.

        Args:
            func (function or :class:`Expression`): the function / expression
                object
            fname (str, optional): the name of the new Function.  If func is an
                Expression this is mandatory.  Otherwise it is taken from the
                Python function name.
            fargs (list of str, optional): the argument names.  If func is an
                :class:`Expression` this is mandatory. 

        To convert Expressions into Functions, the fargs list declares the
        order in which variables in func will be bound to arguments.  E.g.,
        ``Context.declare(2*expr("x")*expr("y"),"twoxy",["x","y"])``
        produces a two argument function ``twoxy(x,y): 2*x*y``.
        """ 
        if isinstance(func,Function):
            if fname is None:
                fname = func.name
            import copy
            newfunc = copy.copy(func)
            newfunc.name = fname
            self.customFunctions[fname] = newfunc
            return newfunc
        if isinstance(func,Expression):
            assert fname != None,"When declaring an expression as a function, the name must be provided"
            assert fargs != None,"When declaring an expression as a function, the argument order must be provided"
            self.customFunctions[fname] = Function(fname,func,fargs)
            return self.customFunctions[fname]
        if callable(func):
            if fname is None:
                import inspect
                for x in inspect.getmembers(func):
                    if x[0] == '__name__':
                        fname = x[1]
                        break
                assert fname != None
            self.customFunctions[fname] = Function(fname,func,fargs)
            return self.customFunctions[fname]
        else:
            raise ValueError("func should be a symbolic Function, symbolic Expression, or Python function")
            
    def include(self,context,prefix=None,modify=False):
        """Adds all items inside another context as a sub-context.

        If prefix != None, ``self.[prefix]`` is set to ``context``, and all
        names in that context are prepended by the given prefix and a '.'.

        If ``modify=True``, then all the Variable and Function names inside
        the given sub-context are modified to fit the `self` context.  This is
        useful if you are saving/loading expressions and using the convenience
        form self.[prefix].[functionName] to instantiate Functions embedded
        inside the sub-context. 

        .. note::
            When using modify=True, be careful not to re-use the sub-context
            inside multiple super-contexts.

        """
        if prefix is not None:
            assert not hasattr(self,prefix),"Can't include a new context, already have an element named "+prefix
            setattr(self,prefix,context)
            prefix = prefix + '.'
        if prefix is None:
            modify = False
        for v in context.variables:
            n = v.name if prefix is None else prefix+v.name
            self.addVar(n,v.type)
            if modify:
                v.name = n
        if modify:
            context.variableDict = dict((v.name,v) for v in context.variables)
        def emodify(e):
            if isinstance(e,VariableExpression):
                return VariableExpression(self.variableDict[prefix + e.var.name])
            elif isinstance(e,UserDataExpression):
                return UserDataExpression(prefix +e.var.name)
            elif isinstance(e,OperatorExpression):
                return OperatorExpression(e.functionInfo,[emodify(a) for a in e.args],e.op)
            return e
        for n,e in context.expressions.items():
            n = n if prefix is None else prefix+n
            if prefix is None:
                self.addExpr(n,e)
            else:
                #need to modify any user data or variables
                self.addExpr(n,emodify(e))
        for n,d in context.userData.items():
            n = n if prefix is None else prefix+n
            self.addUserData(n,d)
        newfunctions = []
        for n,d in context.customFunctions.items():
            if modify:
                fnew = d
            else:
                fnew = copy.copy(d)
            fnew.simplifierDict = _addDictPrefix(prefix,fnew.simplifierDict,context.customFunctions)
            fnew.presimplifierDict = _addDictPrefix(prefix,fnew.presimplifierDict,context.customFunctions)
            newfunctions.append(fnew)
        for f in newfunctions:
            f.name = prefix + f.name
            self.declare(f)
        if modify:
            context.variableDict = dict((v.name,v) for v in context.variables)
            context.userData = dict((prefix+n,d) for n,d in context.userData.items())
            context.expressions = dict((prefix+n,e) for n,e in context.expressions.items())
            context.customFunctions = dict((d.name,d) for d in context.customFunctions.values())
    def copy(self,copyFunctions=False):
        """Returns a shallow copy of this context."""
        res = Context()
        for v in self.variables:
            res.variables.append(Variable(v.name,v.type,res))
        for v in res.variables:
            res.variableDict[v.name] = v
        res.userData = self.userData.copy()
        #deep-copy expressions so they refer to expressions and variables in the new context
        def rebind(expr,res):
            if isinstance(expr,OperatorExpression):
                newargs = [rebind(a,res) for a in expr.args]
                return OperatorExpression(expr.functionInfo,newargs,expr.op)
            elif isinstance(expr,VariableExpression):
                return VariableExpression(res.variableDict[expr.var.name])
            else:
                return expr
        for n,e in self.expressions.items():
            res.expressions[n] = rebind(e,res)
        if copyFunctions:
            res.customFunctions = self.customFunctions.copy()
        else:
            res.customFunctions = self.customFunctions
        return res
    def bind(self,vars,values):
        """Assigns multiple variables to values"""
        for v,val in zip(vars,values):
            if isinstance(v,str):
                self.variableDict[v].value = val
            else:
                assert isinstance(v,Variable)
                v.value = val
    def bindFunction(self,function,remapping=None):
        """Produces an Expression that evalutes the function, where its
        arguments are bound to variables / user data in the current
        environment.  The argument names should map to similarly named
        variables or user data.

        If remapping is provided, then it maps function arguments to variables
        or values, either as

        - a dictionary mapping a function argument arg to remapping[arg].
        - a list or tuple mapping the i'th function argument to remapping[i].

        """
        if isinstance(function,str):
            function = self.customFunctions[function]
        assert isinstance(function,Function)
        args = []
        for aindex,arg in enumerate(function.argNames):
            if remapping is not None:
                if isinstance(remapping,dict):
                    if arg in remapping:
                        var = remapping[arg]
                        if isinstance(var,str) and var in self.variableDict:
                            #Do we want to map it to the corresponding variable? Or just keep it as a userData reference?
                            pass
                        args.append(var)
                        continue
                else:
                    #its a list
                    var = remapping[aindex]
                    if isinstance(var,str) and var in self.variableDict:
                        #Do we want to map it to the corresponding variable? Or just keep it as a userData reference?
                        pass
                    args.append(var)
                    continue
            if arg in self.variableDict:
                args.append(self.variableDict[arg])
            elif arg in self.userData:
                args.append(arg)
            else:
                raise ValueError("Function %s argument %s does not exist in the current context"%(function.name,arg))
        return function(*args)
    def listVars(self,doprint=True,indent=0):
        if doprint:
            for v in self.variables:
                print(' '*indent + "%s (type %s)"%(v.name,str(v.type)))
        return [v.name for v in self.variables]
    def listExprs(self,doprint=True,indent=0):
        if doprint:
            for n,f in self.expressions.items():
                print(' '*indent + n + '=' + str(f))
        return list(self.expressions.keys())
    def listFunctions(self,doprint=True,indent=0,builtins=False):
        global _builtin_functions
        if doprint:
            for n,f in _builtin_functions.items():
                argstr =  '...' if f.argNames is None else ','.join(f.argNames)
                print(' '*indent + n + '(' + argstr + ')')
            for n,f in self.customFunctions.items():
                argstr =  '...' if f.argNames is None else ','.join(f.argNames)
                print(' '*indent + n + '(' + argstr + ')')
        if builtins:
            return list(self.customFunctions.keys()) + list(_builtin_functions.keys())
        return list(self.customFunctions.keys())
    def expr(self,name):
        """Retrieves a named expression"""
        try:
            return self.expressions[name]
        except KeyError:
            pass
        raise KeyError("Invalid expression name "+name)
    def function(self,name):
        """Retrieves a named function.."""
        global _builtin_functions
        try:
            return _builtin_functions[name]
        except KeyError:
            pass
        try:
            return self.customFunctions[name]
        except KeyError:
            pass
        raise KeyError("Invalid function key "+name)
    def get(self,name,*args):
        """Retrieves a named reference to a userData or variable.

        Can be called as ``get(name)``, in which case a KeyError is raised if 
        the key does not exist, or ``get(name,defaultValue)`` in which case the
        default value is returned if the key does not exist.
        """
        try:
            return self.userData[name]
        except KeyError:
            pass
        try:
            return self.variableDict[name]
        except KeyError:
            pass
        if len(args) == 0:
            raise KeyError("Invalid variable / userData key "+name)
        return args[0]

    def renameVar(self,item,newname):
        """Renames a variable"""
        if isinstance(item,str):
            item = self.variableDict[item]
        if item.name == newname: return
        assert newname not in self.variableDict,"Renamed variable name "+newname+" already exists"

        del self.variableDict[item.name]
        self.variableDict[newname] = item
        item.name = newname
    def renameUserData(self,itemname,newname):
        """Renames a userData"""
        assert itemname in self.userData,"Userdata "+itemname+" does not exist"
        if itemname == newname: return
        assert newname not in self.userData,"Renamed userdata name "+newname+" already exists"
        self.userData[newname] = self.userData[itemname]
        del self.userData[itemname]

    def makePyFunction(self,expr,varorder=None):
        """Converts an Expression or Function to a Python function ``f(x)``
        that takes ``x`` as a list of scalar values, maps those to Variable
        values, and returns the result of evaluating the expression / function.

        Args:
            expr (:class:`Function` or :class:`Expression`): the function or
                expression to evaluate
            varorder (list, optional): If given, the list of Variables that
                should appear in the flattened argument list ``x``.
                
                If this isn't provided, then the Variables in ``expr`` are
                ordered by the order in which were added to this ``Context``.

        Returns:
            tuple: A pair ``(f,varorder)``, where:

                * ``f(x)`` is a 1-argument Python function equivalent to ``expr``
                    but where ``x`` is a list of variable values.

                * ``varorder`` gives the order of variables that should be
                    sent in ``x``

        """
        if isinstance(expr,Function):
            if varorder is None:
                varnames = expr.argNames
                varorder = [self.variableDict[v] for v in varnames]
                iorder = list(range(len(varorder)))
            else:
                varorder = [(self.variableDict[v] if isinstance(v,str) else v) for v in varorder]
                iorder = [expr.argNames.index(v.name) for v in varorder]
                varnames = [v.name for v in varorder]
            def f(*args):
                eargs = expr.argNames[:]
                for i,a in zip(iorder,args):
                    eargs[i] = a
                #print("Arguments",eargs)
                return expr(*eargs).evalf(self)
            return f,varorder
        assert isinstance(expr,Expression)
        res = expr.eval(self)
        if isinstance(res,Expression):
            rvars = res.vars(self,bound=False)
            if varorder is None:
                allorder = dict()
                for i,v in enumerate(self.variables):
                    allorder[v.name] = i
                varorder = sorted(rvars,key=lambda v:allorder[v.name])
            else:
                #convert strings to Variables
                varorder = [(self.variableDict[v] if isinstance(v,str) else v)for v in varorder]
                vnames = set(v.name for v in varorder)
                for v in rvars:
                    if v.name not in vnames:
                        warnings.warn("Error while creating Python function corresponding to",res)
                        raise ValueError("Unbound variable "+v.name+" not in given variable order "+",".join([var.name for var in varorder]))
            def f(*args):
                #print("Evaluating with order",[str(v) for v in varorder],args)
                for (v,val) in zip(varorder,args):
                    v.bind(val)
                res = expr.evalf(self)
                for (v,val) in zip(varorder,args):
                    v.unbind()
                return res
            return (f,varorder)
        else:
            if varorder is None:
                varorder = []
            return ((lambda *args: res),varorder)
    def makeFlatFunction(self,expr,varorder=None,defaultsize=1):
        """Given an expression expr, return ``(f,varorder)``,
        where ``f`` is an equivalent 1-argument Python function that takes a
        list of numbers or numpy array.  The order of variables that should
        be provided to ``f`` in this tuple is returned in ``varorder``.

        If vector variables are not given size hints, then they are assumed
        to have ``defaultsize``.

        See also :meth:`makePyFunction`.
        """
        f,varorder = self.makePyFunction(expr,varorder)
        indices = self.getFlatVarRanges(varorder,defaultsize)
        def fv(x):
            arglist = []
            for i,v in enumerate(varorder):
                if v.type.char in 'AM':
                    arglist.append(x[indices[i]:indices[i+1]].reshape(v.size))
                elif v.type.char == 'V':
                    arglist.append(x[indices[i]:indices[i+1]])
                else:
                    arglist.append(x[indices[i]])
            #print("makeFlatFunction lambda function: Extracted arguments",arglist,"from",x)
            #print("Evaluating expression",expr)
            return f(*arglist)
        return fv,varorder
    def makeFlatFunctionDeriv(self,expr,varorder=None,defaultsize=1):
        """Given a differentiable expression expr, return ``(df,varorder)``,
        where df is an 1-argument Python function that that takes a list of
        numbers or numpy array and outputs the derivative (Jacobian matrix)
        of expression ``expr``.

        The order of variables that should be provided to ``df`` in this tuple
        is returned in ``varorder``.

        If ``expr`` is not differentiable, then ``df=None`` is returned.

        If vector variables are not given size hints, then they are assumed to
        have ``defaultsize``.

        See also :meth:`makeFlatFunction`.
        """
        if varorder is None:
            f,varorder = self.makePyFunction(expr)
        dvs = []
        for v in varorder:
            dv = expr.deriv(v)
            if dv is None:
                warnings.warn("makeFlatFunctionDeriv: Derivative with respect to {} is undefined, returning None".format(v.name))
                return None,varorder
            if not isinstance(dv,Expression):
                if _is_exactly(dv,0) and not v.type.is_scalar():
                    #get the jacobian dimensions right
                    assert v.type.char == 'V',"Can't make flat function of non-numeric or non-vector types: "+str(v.type)
                    dv = zero(count.optimized(v))
                else:
                    dv = ConstantExpression(dv)
            dvs.append(dv)
        if len(dvs) > 1:
            rt = expr.returnType()
            if rt is None or rt.char is None:
                warnings.warn("makeFlatFunctionDeriv: no return type inferred, assuming vector")
                rt = Type('V')
            if rt.is_scalar():
                dflist,varorder = self.makePyFunction(flatten(*dvs),varorder)
                df = lambda *args: np.array(dflist(*args))
            elif rt.char == 'V':
                df,varorder = self.makePyFunction(column_stack(*dvs),varorder)
            else:
                warnings.warn("Expression {} has return type {}".format(expr,rt))
                raise NotImplementedError("Can't make flat function of non-numeric or non-vector types: "+str(rt))
        else:
            df,varorder = self.makePyFunction(dvs[0],varorder)
        indices = self.getFlatVarRanges(varorder,defaultsize)
        def dfv(x):
            arglist = []
            for i,v in enumerate(varorder):
                if v.type.char in 'AM':
                    arglist.append(x[indices[i]:indices[i+1]].reshape(v.size))
                elif v.type.char == 'V':
                    arglist.append(x[indices[i]:indices[i+1]])
                else:
                    arglist.append(x[indices[i]])
            #print("Extracted arguments",arglist)
            return df(*arglist)
        return dfv,varorder
    def getFlatVarRanges(self,varorder,defaultsize=1):
        """If varorder is a variable order returned by makeFlatFunction, returns the list of index ranges
        r=[k0,...,kn] where variable index i corresponds with x[r[i]:r[i+1]].  The x vector must have size
        r[-1]."""
        varorder = [(self.variableDict[v] if isinstance(v,str) else v) for v in varorder]
        indices = [0]
        for v in varorder:
            assert v.type.char in 'AMV' or v.type.is_scalar(),"Can't handle list or user data types in makeFlatFunction"
        for v in varorder:
            if v.type.char in 'AM':
                assert v.type.size is not None,"Matrix and array variables need to have their size specified"
                indices.append(indices[-1]+v.type.len())
            elif v.type.char == 'V':
                size = v.type.size if v.type.size != None else defaultsize
                indices.append(indices[-1]+size)
            else:
                indices.append(indices[-1]+1)
        return indices
    def getFlatVector(self,varorder):
        """If varorder is a variable order returned by makeFlatFunction, returns the x vector corresponding 
        to the current variable values."""
        assert not any(v.value is None for v in varorder),"All variables' values must be set"
        return np.hstack([v.value.flatten() if isinstance(v,np.ndarray) else v for v in varorder])


class Function(object):
    """A symbolic function.  Contains optional specifications of argument and
    return types, as well as derivatives.

    Args:
        name (str):
        func (Python function or :class:`Expression`):
        argNames (list of str, optional):
        returnType (str or function, optional): sets ``self.returnType`` or
            ``self.returnTypeFunc``.

    If ``func`` is a Python function, the argument list will be derived from
    the declaration of the Python function. Otherwise, it must be an
    :class:`Expression`, and ``argNames`` needs to be provided.  The
    expression must be closed, meaning that all unspecified variables in the
    expression must be named in ``argNames``.

    Examples:
        Basic instantiation::

            #standard function method
            def f(x,y):
                ...
            symfunc = Function("f",f)
            a = context.addVar("a")
            b = context.addVar("b")
            c = context.addVar("c")
            print(symfunc(a,b))   # prints f($a,$b)

        Creating a Function from an Expression (sort of like a lambda function)::

            expr = a + 2*b
            symfunc2 = Function("f2", expr, ["a","b"])  #when symfunc2 is called, its first argument will be bound to a, and its second will be bound to b
            print(symfunc2(c,4))   #prints f2(c,4)
            print(symfunc2(c,4).eval({'c':5}))  #prints 13, because c + 2*4 = 5 + 2*4 = 13

    If ``func`` is an Expression, then you can automatically set derivatives
    if all the sub-expressions have derivatives. To do so, use the
    :meth:`autoSetJacobians` method.

    Defining derivatives:
        There are three ways to define derivatives: :meth:`setDeriv`,
        :meth:`setJacobian`, and :meth:`autoSetJacobians` (available if
        ``func`` is an expression.  In :meth:`setDeriv` you provide
        Jacobian-vector products.  In :meth:`setJacobian` you provide
        Jacobian matrices (or tensors).

    Attributes:
        name (str): name of function used in printing and IO.
        description (str, optional): text description of function.
        func (Python function or :class:`Expression`): the function to be
            evaluated.
        argNames (list of strs, optional): names of arguments.
        argTypes (list of Type, optional): list giving each argument's
          :class:`Type`.
        argDescriptions (list of strs, optional): strings describing each
            argument
        returnType (Type, optional): return :class:`Type`
        returnTypeFunc (function, optional): a function that takes argument 
            types and produces a more specific return :class:`Type` than
            ``returnType``
        returnTypeDescription (str, optional): description of the return type
        deriv (optional): Can be either:

            1. a list of Jacobian-vector products with respect to a derivative
               of each argument: ``[df1(*args,darg1),...,dfn(*args,dargn)]``.
               Here, ``dargi`` is an argument of the same shape as
               ``args[i]``, giving the derivative :math:`\\frac{dargs[i]}{dx}` 
               w.r.t. some scalar parameter x.  The function ``dfi`` gives
               :math:`\\frac{df}{dargs[i]}\\cdot\\frac{dargs[i]}{dx}` where
               :math:`\\frac{df}{dargs[i]}` is the partial 
               of ``self`` with respect to the i'th argument.
            2. a function ``df(args,dargs)`` giving the total derivative of f 
               given directional derivatives of each of the argmuments. 
               Here, ``dargs`` is a list of derivatives ``[darg1,...,dargn]``.
               The result should be equal to
               :math:`\\frac{df}{dx} = \\frac{df}{dargs[0]}\\cdot \\frac{dargs[0]}{dx} + ... + \\frac{df}{dargs[n-1]}\\cdot\\frac{dargs[n-1]}{dx}`.

            The return value should have the same shape as
            ``self.returnType()``

        colstackderiv (optional): same as ``deriv``, except that each function
            accepts stacked argument derivatives.  This can be more efficient
            than filling in ``deriv``, since taking derivatives w.r.t. k
            variables x1,...,xk can be done by setting up ``dargi`` as a matrix
            ``[dflatten(argi)/dx1 | ... | dflatten(argi)/dxk]``, i.e.,. the 
            column stacking of each of the flattened argument derivatives.
            `dargi` has shape ``(count(argi),k)``.
        rowstackderiv (optional): same as ``colstackderiv``, except that row-
            wise stacked argument derivatives are accepted.  In other words,
            ``dargi`` is a matrix
            :math:`[\\frac{d\\text{flatten(args[i])}}{dx1}, ... , \\frac{d\\text{flatten(args[i])}}{dxk}]`
            which has shape ``(k,count(argi))``.  This
            is more efficient than ``colstackderiv`` but corresponds less
            directly to standard mathematical notation.
        jacobian (list of functions, optional): list of Jacobian functions
            with respect to each argument.  Has the form
            ``[Jf1(*args),...,Jfn(*args)]`` where `Jfi` returns a matrix of 
            shape ``(count(self),count(args[i]))``.  The total derivative with
            respect to some variable ``x`` is
            :math:`\\frac{df}{dx} =` ``reshape(dot(Jf1,flatten(darg1)) + dot(Jfn,flatten(dargn)),shape(self))``
        presimplifier (function, optional):
        simplifier (function, optional):
        presimplifierDict (dict, optional): a nested dict, mapping argument
            signatures to simplification functions.  See :meth:`addSimplifier`
            for more details.
        simplifierDict (dict, optional): a nested dict, mapping argument
            signatures to simplification functions. See :meth:`addSimplifier`
            for more details.
        properties (dict): possible properties of this function, such as
            associativity, etc.
        printers (dict of functions): a dict containing code generation
            methods.  Each entry, if present, is a function
            ``printer(expr,argstrs)`` that returns a string. Common key values
            are:

            * "str": for printing to console 
            * "parse": for parseCompatible=True exprToStr

    """
    def __init__(self,name,func,argNames=None,returnType=None):
        self.name = name
        self.description = None
        self.func = func
        self.argNames = argNames
        self.argTypes = None
        self.argDescriptions = None
        self.returnType = None
        self.returnTypeFunc = None
        self.returnTypeDescription = None
        self.setReturnType(returnType)
        if callable(func) and argNames is None:
            import inspect
            (argNames,varargs,keywords,defaults) = inspect.getargspec(func)
            if varargs != None or keywords != None:
                #variable number of arguments
                warnings.warn("symbolic.py: Note that Function {} is declared with variable arguments, pass '...' as argNames to suppress this warning".format(name))
                pass
            else:
                self.argNames = argNames
        if self.argNames == '...':
            self.argNames = None
        if isinstance(func,Expression):
            if returnType is None:
                self.returnType = func.returnType()
            if self.argNames is None:
                raise ValueError("Cannot use Expression as a function with variable arguments")
            self.exprArgRefs = []
            for arg in self.argNames:
                uexpr = func.find(UserDataExpression(arg))
                if uexpr is None:
                    vexpr = func.find(VariableExpression(Variable(arg,'V')))
                    if vexpr is None:
                        raise ValueError("Expression-based function %s does not contain specified argument %s"%(str(func),arg))
                    else:
                        self.exprArgRefs.append(vexpr)
                        assert vexpr.var.name == arg
                else:
                    self.exprArgRefs.append(uexpr)
                    assert uexpr.name == arg
            self.argTypes = [e.returnType() if not isinstance(e,UserDataExpression) else Type(None) for e in self.exprArgRefs]
        self.deriv = None
        if isinstance(self.returnType,Type) and self.returnType.char is not None and self.returnType.char in "BI":
            #no derivatives for boolean or integer functions
            self.deriv = 0
        self.colstackderiv = None
        self.rowstackderiv = None
        self.jacobian = None
        self.presimplifier = None
        self.simplifier = None
        self.presimplifierDict = dict()
        self.simplifierDict = dict()
        self.properties = dict()
        self.printers = dict()

    def __getattribute__(self,name):
        if name=='__doc__':
            return self.info()
        else:
            return object.__getattribute__(self, name)
    
    def __call__(self,*args):
        if self.argNames is not None:
            if len(args) != len(self.argNames):
                raise ValueError("Invalid number of arguments passed to "+self.name)
        if TYPE_CHECKING and self.argTypes is not None:
            #print("Calling",self.name,"(",','.join(str(a) for a in args),")")
            for i,(a,t) in enumerate(zip(args,self.argTypes)):
                #print("  Type checking argument",a,"against type",t)
                #print("    ... argument has type",type_of(a))
                #raw_input()
                if t is None: continue
                a = expr(a)
                if not t.match(type_of(a)):
                    raise ValueError("Invalid argument %d (%s) passed to %s: type %s doesn't match %s"%(i,str(simplify(a)),self.name,type_of(a),t))
                #rint "  match."
                #raw_input()
        return self._call(*args)
    
    def _call(self,*args):
        """Internally used version of __call__ -- does not perform type checking"""
        if callable(self.func):
            return OperatorExpression(self,args)
        else:
            assert isinstance(self.func,Expression),"What..."+self.func.__class__.__name__
            assert self.argNames is not None
            assert len(self.exprArgRefs) == len(args)
            return OperatorExpression(self,args,lambda *argvals:subs(self.func,self.exprArgRefs,argvals).eval())
    
    def optimized(self,*args):
        """Similar to ``self(*args).simplify(depth=1)``, but optimized to
        reduce complexity earlier. In particular, this directly applies the
        operation if all arguments are constants, and it will try to apply
        the simplifier immediately."""
        if self.presimplifier != None:
            res = self.presimplifier(*args)
            if res is not None:
                if isinstance(res,Expression):
                    return res.simplify(None,depth=1)
                else:
                    return res
            
        anyVariable = False
        newargs = []
        if self.argNames is not None:
            assert len(args) == len(self.argNames),"Invalid number of arguments passed to "+self.name
        for i,arg in enumerate(args):
            ac = to_const(arg,shallow=True)
            if ac is None:
                anyVariable = True
                newargs.append(arg)
            else:
                newargs.append(ac)

        if not anyVariable:
            #just call the function
            if callable(self.func):
                try:
                    return ConstantExpression(self.func(*newargs))
                except Exception as e:
                    #print("Error evaluating expression",self.name,"arguments",newargs)
                    raise
            else:
                #return self.func.eval(context=dict(self.argNames,newargs))
                return subs(self.func,self.exprArgRefs,newargs).eval()
        else:
            res = self.__call__(*newargs)
            res2 = res._simplify(depth=1,constant_expansion=False)
            if res2 is not None:
                return res2
            return res

    def checkArg(self,arg):
        """Verifies that a named or indexed argument is valid, and normalizes it.  Returns an (index,name) tuple"""
        if isinstance(arg,str):
            if self.argNames is None:
                raise ValueError("Can't get an argument by name for a variable-argument function")
            return self.argNames.index(arg),arg
        if self.argNames is None:
            if arg < 0:
                raise ValueError("Negative argument index specified")
            return arg,'arg_'+str(arg)
        if arg < 0 or arg >= len(self.argNames):
            raise ValueError("Invalid argument specified")
        return arg,self.argNames[arg]

    def setDeriv(self,arg,dfunc,asExpr=False,stackable=False):
        """Declares a (partial) derivative of the function with respect to
        argument arg.  The function ``dfunc`` is a function that takes
        arguments ``(arg1,...,argn,dx)``, where ``dx`` is the
        derivative of ``arg`` with respect to some variable x, and
        returns :math:`df/darg(arg1,...,argn) * darg/dx`.
        
        For vector-valued arguments and functions, the * is a matrix-vector
        product, and ``dfunc`` is required to produce what's commonly known as
        the Jacobian-vector product.
        
        For matrix-valued arguments or functions, the * is a tensor product.

        Args:
            arg (int or str): either an argument index or name.
            dfunc: either:
            
                1. an Expression of variables arg1,...,argn,darg,
                2. a ``Function`` of n+1 variables mapping to ``arg1,...,argn,dx``,
                3. a Python function of n+1 variables ``(arg1,...,argn,dx)`` 
                    that either returns a value (setting ``asExpr=False``) or 
                    an ``Expression`` of the derivative (if ``asExpr=True``).
                4. None, to indicate that the derivative is not defined,
                5. 0, to indicate that the derivative is identically 0.
                
                In the normal case (``stackable=False``), the shape of ``dx`` 
                is the same as ``arg``, and the result has the same shape as
                self.

            asExpr (bool, optional): if ``dfunc`` is a Python function, this
                flag says that it will return an ``Expression``.  Using an
                ``Expression`` allows taking multiple derivatives.
            stackable (bool or str, optional): states whether ``dfunc`` can
                accept stacked derivative arguments.  If ``True`` or ``'col'``,
                then it can accept ``dvar`` as a column-stacked array of
                derivatives.  In other words, ``dvar[...,0]`` is the derivative
                w.r.t. x0, ``dvar[...,1]`` is the derivative w.r.t. x1, etc. 
                
                E.g., for vector arguments, dvar can be thought of as a matrix
                with k=``dvar.shape[-1]`` derivatives in its *columns*. 
                
                In this case, ``dfunc`` should return an array of shape
                shape(self) x k (i.e., f's derivatives are stacked in columns.)

                ``stackable`` can also be ``'row'`` in which case ``dfunc``
                accepts ``dvar`` as a row-stacked array of derivatives. I.e,
                ``dvar[0,...]`` is the derivative w.r.t. x0, ``dvar[1,...]`` is
                the derivative w.r.t., x1, etc.  In this case, ``dfunc`` should
                return a list of derivatives.
                
                If the argument is not an array or its type cannot be
                determined (see ``setArgType``) then ``dvar`` will be passed as
                a ``count(arg)`` x k matrix (for column-stacking) or a
                k x ``count(arg)`` matrix (for row stacking).
                
                If the function does not return an array or the return type
                cannot be determined (see ``setReturnType``), the shape of the
                returned matrix must be ``count(self)`` x k (for column-
                stacking) or k x ``count(self)`` (for row-stacking).
        """
        aindex,arg = self.checkArg(arg)
        if self.deriv is None:
            self.deriv = [None]*len(self.argNames)
        if isinstance(dfunc,Expression):
            self.deriv[aindex] = lambda *args:subs(dfunc,self.exprArgRefs+['d'+arg],args)
        elif isinstance(dfunc,Function):
            self.deriv[aindex] = dfunc
        elif dfunc is None or _is_exactly(dfunc,0):
            self.deriv[aindex] = dfunc
        elif asExpr:
            assert callable(dfunc)
            #print("Setting derivative function with name",self.name,"argument",arg,"as expression")
            self.deriv[aindex] = dfunc
        else:
            assert callable(dfunc)
            #print("Declaring new derivative function with name",self.name + "_deriv_" + arg)
            temp_function_info = Function(self.name + "_deriv_" + arg,dfunc,self.argNames + [VAR_DERIV_PREFIX+arg])
            if not hasattr(self,'deriv_funcs'):
                self.deriv_funcs = [None]*len(self.argNames)
            self.deriv_funcs[aindex] = temp_function_info
            self.deriv[aindex] = temp_function_info #lambda *args:OperatorExpression(temp_function_info,args)
        if stackable == True or stackable == 'col':
            if self.colstackderiv is None:
                self.colstackderiv = [None]*len(self.argNames)
            self.colstackderiv[aindex] = self.deriv[aindex]
        elif stackable == 'row':
            if self.rowstackderiv is None:
                self.rowstackderiv = [None]*len(self.argNames)
            self.rowstackderiv[aindex] = self.deriv[aindex]
    
    def setJacobian(self,arg,dfunc,asExpr=False):
        """Declares a (partial) derivative of the function with respect to
        argument ``arg``. 
        
        This only makes sense when the arguments are all arrays or scalars; no
        complex types are supported.
        
        Args: 
            arg (int or str): same as ``setDeriv``.
            dfunc: similar to ``setDeriv``, but a function that takes n
                arguments (arg1,...,argn) and returns the matrix
                :math:`df/darg(arg1,...,argn)`.
                
        The return value of ``dfunc`` must have shape shape(self) + shape(arg),
        so that:
        
            - If self and arg are scalars, the Jacobian is also a scalar
            - If self and arg are a scalar and a vector (or vice versa) the
              Jacobian is a vector.
            - If self and arg contain a scalar and an N-D array, the Jacobian
              is an N-D array.
            - If self and arg are both vectors, the Jacobian is a matrix.
        
        """
        aindex,arg = self.checkArg(arg)
        if TYPE_CHECKING:
            self_irregular =  isinstance(self.returnType,Type) and self.returnType.char not in SCALAR_TYPES+ARRAY_TYPES
            var_irregular = self.argTypes is not None and isinstance(self.argTypes[aindex],Type) and self.argTypes[aindex].char not in SCALAR_TYPES+ARRAY_TYPES
            if self_irregular:
                warnings.warn("symbolic.autoSetJacobians: setting Jacobian of a non-array function. Will flatten.")
            if var_irregular:
                warnings.warn("symbolic.autoSetJacobians: setting Jacobian of a non-array argument. Will flatten.")
        if self.deriv is None:
            self.deriv = [None]*len(self.argNames)
        if self.jacobian is None:
            self.jacobian = [None]*len(self.argNames)
        if isinstance(dfunc,Expression):
            self.jacobian[aindex] = lambda *args:subs(dfunc,self.exprArgRefs,args)
        elif isinstance(dfunc,Function):
            self.jacobian[aindex] = dfunc
        elif dfunc is None or _is_exactly(dfunc,0):
            self.jacobian[aindex] = dfunc
        elif asExpr:
            if not callable(dfunc):
                raise TypeError("Unexpected type of dfunc %s"%(dfunc.__class__.__name__))
            #print "Setting jacobian function with name",self.name,"argument",arg,"as expression"
            self.jacobian[aindex] = dfunc
        else:
            if not callable(dfunc):
                raise TypeError("Unexpected type of dfunc %s"%(dfunc.__class__.__name__))
            #print "Declaring new derivative function with name",self.name + "_jacobian_" + arg
            temp_function_info = Function(self.name + "_jacobian_" + arg,dfunc,self.argNames)
            self.jacobian[aindex] = temp_function_info  #lambda *args:OperatorExpression(temp_function_info,args)
    
    def autoSetJacobians(self,args=None):
        """For ``Expression``-based functions, can automatically set the
        Jacobians.  If ``args`` is not None, only the arguments in ``args``
        are set.
        """
        if not isinstance(self.func,Expression):
            raise ValueError("Can only auto-set jacobians for Expressions")
        if args is None:
            args = self.argNames
        for arg in args:
            darg = self.func.deriv(arg)
            if _DEBUG_DERIVATIVES:
                print("symbolic.autoSetJacobians: Derivative of",self.func,"w.r.t",arg,"is (type %s)"%(darg.__class__.__name__,),darg)
            if darg is not None:
                darg = expr(darg)
            self.setJacobian(arg,darg,asExpr=True)
    
    def setReturnType(self,type):
        """Sets a return type specifier.

        Args:
           type (:class:`Type`, str, or function): if a Type object, the return
               type is set directly.  If a character type specifier, it is cast
               to Type. If it is a function, it is assumed to be a function 
               that that takes in argument Types and returns a :class:`Type`.
        """
        if callable(type):
            self.returnTypeFunc = type
        else:
            self.returnType = Type(type)
    
    def setArgType(self,arg,type):
        """Sets an argument type specifier.

        Args:
            arg (int or str): an index or string naming an argument.
            type (Type or str): a :class:`Type` object or character type
                specifier for the specified argument.
        """
        if self.argTypes is None:
            self.argTypes = [None]*len(self.argNames)
        index,name = self.checkArg(arg)
        self.argTypes[index] = Type(type)
    
    def getArgType(self,arg):
        """Retrieves an argument type.

        Args:
            arg (int or str): the index or string naming an argument.
        """
        if self.argTypes is None: return None
        index,name = self.checkArg(arg)
        return self.argTypes[index]
    
    def addSimplifier(self,signatures,func,pre=False):
        """For a signature tuple, sets the simplifier to ``func``.

        Args:
            signatures (list): a list of argument signatures. A signature can
                be:

                * the operation name for an OperatorExpression, passing the arg
                  directly to func(...)
                * '_scalar': matches to a constant scalar, and passes that
                  constant to func(...)
                * '_const': matches to constant, and passes that constant to
                  ``func(...)``
                * '_returnType': passes the argument's ``returnType()`` to
                  ``func(...)``
                * None: match all.  The arg is passed directly to func(...)
                
            func (callable): a callable that takes a list of function arguments, 
                possibly transformed by _const or _returnType. This returns a 
                simplified :class:`Expression` or ``None``.

            pre (bool, optional): if True, the simplifier is called before
                arguments are simplified.

        If multiple matches are present then they are tested in order

        1. operation name
        2. _const
        3. _returnType
        4. None

        """
        if self.argNames is not None:
            if len(signatures) != len(self.argNames):
                raise ValueError("Invalid signature length")
        if len(signatures) == 0:
            raise ValueError("Invalid signature tuple")
        root = self.presimplifierDict if pre else self.simplifierDict
        for s in signatures[:-1]:
            if s is not None and not isinstance(s,str):
                raise ValueError("Signatures must be strings or None")
            root = root.setdefault(s,dict())
        root[signatures[-1]] = func
    
    def simplify(self,args,pre=False):
        """Performs simplification of ``OperatorExpression(self,args)``, either
        with the simplifier function or the simplifierDict."""
        if pre:
            simplifier = self.presimplifier
            simplifierDict = self.presimplifierDict
        else:
            simplifier = self.simplifier
            simplifierDict = self.simplifierDict
        for a in args:
            assert isinstance(a,Expression),"Uh... args to simplify need to be Expressions, got an arg %s of type %s"%(str(a),a.__class__.__name__)
        if simplifier is not None:
            res = simplifier(*args)
            if res is not None:
                if SHAPE_CHECKING and isinstance(res,Expression):
                    try:
                        shres = res.returnType().shape()
                        if to_const(shres) is not None:
                            shself = self.__call__(*args).returnType().shape()
                            if to_const(shself) is None or to_const(shres) != to_const(shself):
                                warnings.warn("symbolic.simplify: simplified version of {} doesn't have same size: {} -> {}".format(self.functionInfo.name,shself,shres))
                                warnings.warn("Args: {}".format(args))
                                warnings.warn("Simplified {}".format(res))
                    except Exception:
                        pass
                return res
        if len(simplifierDict) > 0:
            #print "Trying to match",[str(a) for a in args],"to simplifier dict"
            root = simplifierDict
            stack = []
            passedArgs = args[:]
            for i,a in enumerate(args):
                #print "Keys:",root.keys()
                #print "Arg",i,":",a
                if isinstance(a,OperatorExpression) and a.functionInfo.name in root:
                    if _DEBUG_SIMPLIFY:
                        print("Function",self.name,"simplifier list matches arg",a.functionInfo.name)
                        stack.append(a.functionInfo.name)
                    root = root[a.functionInfo.name]
                    continue
                if '_scalar' in root:
                    ac = to_scalar(a)
                    if ac is not None:
                        if _DEBUG_SIMPLIFY:
                            print("Function",self.name,"simplifier list matches _scalar for arg",a)
                            stack.append('_scalar')
                        passedArgs[i] = ac
                        root = root['_scalar']
                        continue
                if '_const' in root:
                    ac = to_const(a)
                    if ac is not None:
                        if _DEBUG_SIMPLIFY:
                            print("Function",self.name,"simplifier list matches _const for arg",a)
                            stack.append('_const')
                        passedArgs[i] = ac
                        root = root['_const']
                        continue
                if '_returnType' in root:
                    if _DEBUG_SIMPLIFY:
                        print("Function",self.name,"simplifier list matches arg",a,"returnType",a.returnType())
                        stack.append('_returnType')
                    passedArgs[i] = a.returnType()
                    root = root['_returnType']
                    continue
                if None in root:
                    #print "  Matches None"
                    if _DEBUG_SIMPLIFY:
                        print("Function",self.name,"simplifier list matches all for arg",a)
                        stack.append(None)
                    root = root[None]
                    continue
                #no match
                if _DEBUG_SIMPLIFY:
                    if isinstance(a,OperatorExpression):
                        print("Function",self.name,"simplifier list",list(root.keys()),"fails to match",a)
                return None
            if callable(root):
                res = root(*passedArgs)
                if _DEBUG_SIMPLIFY:
                    print("Function",self.name,"matches pattern",stack,", simplified to",res)
                if SHAPE_CHECKING and isinstance(res,Expression):
                    try:
                        shres = res.returnType().shape()
                        if to_const(shres) is not None:
                            shself = self.__call__(*args).returnType().shape()
                            if to_const(shself) is None or to_const(shres) != to_const(shself):
                                warnings.warn("symbolic.simplify: simplified version of {} doesn't have same size: {} -> {}".format(self.name,shself,shres))
                                warnings.warn("Args: {}".format(passedArgs))
                                warnings.warn("Simplified {}".format(res))
                    except Exception:
                        pass
                return res
            return None
        return None
    
    def info(self):
        """Returns an text string describing the Function, similar to a docstring"""
        argstr =  '...' if self.argNames is None else ','.join(self.argNames)
        signature = '%s(%s)'%(self.name,argstr)
        if isinstance(self.func,Expression):
            signature = signature + '\n Defined as '+str(self.func)
        argHelp = None
        if self.argTypes is not None or self.argDescriptions is not None:
            if self.argNames is None:
                argHelp = [str(self.argDescriptions)]
            else:
                argHelp= []
                if self.argTypes is not None:
                    assert len(self.argNames) == len(self.argTypes),"invalid arg specification for function %s, %d args != %d types"%(str(self.func),len(self.argNames),len(self.argTypes))
                for i,name in enumerate(self.argNames):
                    type = None if self.argTypes is None else self.argTypes[i]
                    desc = None if self.argDescriptions is None else self.argDescriptions[i]
                    if desc is None:
                        if type is not None:
                            desc = type.info()
                        else:
                            desc = 'unknown'
                    argHelp.append('- %s: %s'%(name,desc))
        returnHelp = None
        if self.returnTypeDescription is not None:
            returnHelp = "- " + self.returnTypeDescription
        elif self.returnType is not None:
            returnHelp = "- " + self.returnType.info()
        elif self.returnTypeFunc is not None:
            if self.returnTypeFunc == _propagate_returnType:
                returnHelp = "- same as arguments"
            elif self.returnTypeFunc == _promote_returnType:
                returnHelp = "- same as arguments"
            elif self.returnTypeFunc == _returnType1:
                returnHelp = "- same as argument 1"
            elif self.returnTypeFunc == _returnType2:
                returnHelp = "- same as argument 2"
            elif self.returnTypeFunc == _returnType3:
                returnHelp = "- same as argument 3"
            else:
                returnHelp = "- dynamic"
        derivHelp = None
        if self.deriv is not None or self.jacobian is not None:
            derivHelp = []
            if _is_exactly(self.deriv,0):
                derivHelp.append('- derivative is 0 everywhere')
            elif callable(self.deriv):
                deval = None
                if self.argNames is not None:
                    argTypes = [Type(None)]*len(self.argNames) if self.argTypes is None else self.argTypes
                    vars = [expr(Variable(a,t)) for a,t in zip(self.argNames,argTypes)]
                    dvars = [expr(Variable('d'+a,t)) for a,t in zip(self.argNames,argTypes)]
                    try:
                        deval = self.deriv(vars,dvars)
                    except Exception:
                        pass
                if deval is not None:
                    derivHelp.append('- derivative is '+str(deval))
                else:
                    derivHelp.append('- derivative is a total derivative function')
            elif callable(self.jacobian):
                derivHelp.append('- jacobian is a total derivative function')
            elif self.argNames is not None:
                argTypes = [Type(None)]*len(self.argNames) if self.argTypes is None else self.argTypes
                vars = [expr(Variable(a,t)) for a,t in zip(self.argNames,argTypes)]
                for i,a in enumerate(self.argNames):
                    if self.deriv is not None and self.deriv[i] is not None:
                        if _is_exactly(self.deriv[i],0):
                            derivHelp.append('- %s: derivative is 0'%(a,))
                        elif isinstance(self.deriv[i],Function):
                            derivHelp.append('- %s: available as Python df/da * da/dx function'%(a,))
                        else:
                            try:
                                deval = self.deriv[i](*(vars+[expr(Variable('d'+a,argTypes[i]))]))
                                if is_op(deval,'subs'):
                                    deval = deval.args[0]
                                derivHelp.append('- %s: available as df/da * da/dx function %s'%(a,str(deval)))
                            except Exception as e:
                                derivHelp.append('- %s: available as df/da * da/dx function, Exception %s'%(a,str(e)))
                    elif self.jacobian is not None and self.jacobian[i] is not None:
                        if _is_exactly(self.jacobian[i],0):
                            derivHelp.append('- %s: jacobian is 0'%(a,))
                        elif isinstance(self.deriv[i],Function):
                            derivHelp.append('- %s: available as Python jacobian df/da'%(a,))
                        else:
                            try:
                                deval = self.jacobian[i](*vars)
                                if is_op(deval,'subs'):
                                    deval = deval.args[0]
                                derivHelp.append('- %s: available as jacobian df/da %s'%(a,str(deval)))
                            except Exception as e:
                                derivHelp.append('- %s: available as jacobian df/da, Exception %s'%(a,str(e)))
        items = [signature]
        if self.description != None:
            items += ['',self.description,'']
        if argHelp is not None and len(argHelp) > 0:
            items += ['','Parameters','---------']+argHelp
        if returnHelp is not None:
            items += ['','Return type','-----------',returnHelp]
        if derivHelp is not None and len(derivHelp) > 0:
          items += ['','Derivatives','-----------']+derivHelp
        return '\n'.join(items)


class Variable:
    def __init__(self,name,type,ctx=None):
        self.name = name
        self.type = Type(type)
        self.ctx = None if ctx is None else weakref.proxy(ctx)
        self.value = None
    def isAssigned(self):
        return (self.value is not None)
    def bind(self,value):
        self.value = value
    def unbind(self):
        self.value = None
    def context(self):
        return self.ctx
    def __str__(self):
        if self.value is None:
            return self.name
        else:
            return self.name+'='+str(self.value)
    def __len__(self):
        if self.type.size == 0:
            raise TypeError("Numeric Variable has no len()")
        if self.type.size is None:
            if self.value is not None:
                return len(self.value)
            raise TypeError("Variable-sized vector Variable has no len()")
        return self.type.size
    def __not__(self):
        return not_(self)
    def __and__(self,rhs):
        return and_(self,rhs)
    def __or__(self,rhs):
        return or_(self,rhs)
    def __eq__(self,rhs):
        return eq(self,rhs)
    def __ne__(self,rhs):
        return ne(self,rhs)
    def __le__(self,rhs):
        return le(self,rhs)
    def __ge__(self,rhs):
        return ge(self,rhs)
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
    def __setitem__(self,index,val):
        return setitem(self,index,val)
    def __getattr__(self,attr):
        if attr == 'T':
            return transpose(self)
        elif attr == 'shape':
            return shape(self)
        else:
            raise AttributeError("'Variable' object has no attribute '%s'" % attr)
    #def __getslice__(self,*args):
    #    return _builtin_functions['getslice'](self,index)

class _TraverseException(Exception):
    def __init__(self,e,node,task,exc_info):
        self.e = e
        self.exc_info = exc_info
        self.path = [node]
        self.parents = []
        n = node
        while n._parent is not None and not isinstance(n._parent,str):
            self.parents.append(n._parent)
            n = n._parent[0]()
            self.path.append(n)
        self.parents = self.parents[::-1]
        self.path = self.path[::-1]
        self.task = task
    def __str__(self):
        print(":", end=' ')
        if isinstance(self.e,_TraverseException):
            #lazy traversal
            print("Expression traversal: Recursive exception during %s"%(str(self.task),))
            elist = [self.e]
            while isinstance(elist[-1].e,_TraverseException):
                elist.append(elist[-1].e)
            self.print_call_stack()
            for i,e in enumerate(elist):
                print(" ","@",e.task,"@:")
                e.print_call_stack((i+1)*2)
            print("Lowest level exception:", end=' ')
            return elist[-1].e.__class__.__name__+": "+str(elist[-1].e)
        else:
            print("Expression traversal: Exception '%s' during %s"%(self.e,str(self.task)))
            self.print_call_stack()
            return ""
    def __repr__(self):
        return self.__str__()
    def reraise(self):
        raise self.e.with_traceback(self.exc_info)
    def print_call_stack(self,indent=0):
        self.path[0]._parent = self.task
        for n,p in zip(self.path[1:],self.parents):
            n._parent = p
        self.path[-1]._print_call_stack(indent)

class Expression(object):
    def __init__(self):
        self._parent = None
        self._id = None
        self._cache = dict()
        self._children = None
    def returnType(self):
        """Returns the output type of this expression."""
        raise NotImplementedError()
    def depth(self,cache=True):
        """Returns the depth of the deepest leaf of this expression tree (i.e.,
        its height). If cache = True, depths are cached as attribute 'depth' to
        speed up subsequent depth() calls."""
        try:
            return self._cache['depth']
        except KeyError:
            pass
        def _depth(node,childdepths):
            if len(childdepths)==0: return (True,1)
            return (True,max(childdepths)+1)
        if not cache:
            return self._traverse(post=_depth,cache=False)
        else:
            return self._traverse(post=_depth,cache=True,clearcache=False,cacheas='depth')
    def isConstant(self):
        """Returns true if this expression is identically a constant.  If so,
        then the expression can be safely replaced with its evalf() value
        without any possible change in meaning.
        """
        raise NotImplementedError()
    def returnConstant(self,context=None):
        """Returns true if the evaluation of the expression in the given
        context results in a constant. If so, then :meth:`evalf` can safely be
        applied without error."""
        raise NotImplementedError()
    def eval(self,context=None):
        """Evaluates the expression while substituting all constant values. 
        The result can be a constant or an Expression if the result is not
        constant.

        Args:
            context (Context or dict, optional): a map of variable names to
                constant values.
        """
        return self._eval(context)
    def evalf(self,context=None):
        """Evaluates the expression while substituting all constant values,
        enforcing that the result is a constant.  If the result is not
        constant, a ValueError is returned.
        """
        r = self.eval(context)
        if not is_const(r):
            from . import symbolic_io
            raise ValueError("Expression.evalf: result of %s is not a constant, result is %s"%(str(symbolic_io.exprToJson(self)),str(r)))
        if isinstance(r,ConstantExpression):
            return r.value
        if isinstance(r,Expression):
            from . import symbolic_io
            raise ValueError("Expression.evalf: result of %s is not a constant, result is %s"%(str(symbolic_io.exprToJson(self)),str(r)))
        return r
    def _eval(self,context):
        """Internally used eval"""
        raise NotImplementedError()
    def _evalf(self,context=None):
        """Internally used evalf.  May be a tiny bit faster than :meth:`evalf`
        but has less informative error messages.
        """
        raise NotImplementedError()
    def deriv(self,var,context=None):
        """Returns a new expression where all bound variables and variables in 
        context are reduced to constant values. If all arguments are constant,
        returns a constant.

        Args:
            var: either a Variable, a string naming a Variable, or a dictionary
                mapping variable names to derivatives. For example, in the
                latter case, if this is an expression f(x,y),and var is a
                dictionary {"x":dx,"y":dy}, then the result is
                ``df/dx(x,y)dx + df/dy(x,y)dy``
            context (Context or dict, optional): a map of variable names to
                constant values.
        """
        return self._deriv(var,context)
    def _deriv(self,var,context,rows):
        """Internally used deriv"""
        return 0
    def __str__(self):
        """Returns a string representing this expression"""
        from . import symbolic_io
        exp = symbolic_io.exprToStr(self,parseCompatible=False)
        assert isinstance(exp,str),"Invalid type returned by exprToStr: "+exp.__class__.__name__
        return exp
    def __repr__(self):
        from . import symbolic_io
        return symbolic_io.exprToStr(self,parseCompatible=True)
    def vars(self,context=None,bound=False):
        """Returns a list of all free Variables under this expression. 

        If bound is True, any Variables currently bound to values are
        returned, otherwise they are not."""
        return []
    def match(self,val):
        """Returns true if self is equivalent to the given expression. Note: no
        rearrangement is attempted, so x+y does not match y+x."""
        if isinstance(val,Wildcard): return True
        return False
    def find(self,val):
        """Returns self if self contains the given expression as a sub-expression, or None otherwise."""
        if self.match(val): return self
        return None
    def replace(self,val,valReplace,error_no_match=False):
        """Returns a copy of this expression where any sub-expression matching val gets replaced
        by valReplace.  If val doesn't exist as a subexpression, self is returned."""
        if self.match(val):
            return valReplace
        if error_no_match:
            raise ValueError("Unable to find term %s in expression %s"%(str(val),str(self)))
        return self
    def simplify(self,context=None,depth=None):
        """Returns a simplified version of self, or self if it cannot be
        simplified.

        Args:
            context (dict or :class:`Context`): any Variable / UserData whose
                name appears in here will be substituted with its value.
            depth (int, optional): None for full-depth simplification, or the
                max depth to explore.

        Returns:
            Expression:
        """
        res = self._simplify(context,depth)
        if res is None: return self
        return res
    def _simplify(self,context=None,depth=None):
        """Returns a new expression, or None if this cannot be simplified.
        - context: a dict or Context for constant substitution
        - depth: None for full-depth simplification, or the max depth to explore
        """
        return None
    def _traverse(self,pre=None,post=None,cache=True,clearcache=True,cacheas=None):
        """Generic traversal function.

        Args:
            pre (function, optional): a function f(expr) of an Expression that
                is called before traversing children. It returns a triple
                ``(descend,cont,value)`` which controls the recursion:

                * If descend is True, then this proceeds to traverse children
                  and call post. value is ignored.
                * If descend is False, value is the return value of the
                  traversal.
                * If cont is True, then the parent of expr continues traversing
                  children.
                * If cont is False, then the parent of expr continues
                  traversing children.

                For example, a find function would return (False,False,*) if an
                item is found, and (True,True,*) otherwise.

            post (function, optional): a two-argument function
                ``f(expr,childvals)`` called after traversing children. 
                ``childvals`` are the return values of traversing children.
                It returns a pair ``(cont,value)``, where value is the return
                value of the traversal under ``expr``, and ``cont`` which
                controls the recursion:

                * If cont is True, the parent of expr continues traversing
                  children.
                * If cont is False, the parent of expr stops traversing
                  children and value is used the return value of the parent.

                If post is evaluated, the cont value here overrides the cont
                value of pre.
            cache (bool, optional): if True, uses the caching functionality.
            clearcache (bool, optional): if True, deletes the cache after
                traversal.  Default uses cache and clears it.
            cacheas (str, optional): if not None, caches the return value as an
                attribute. For this to work, ``cache`` must be True.

        Returns:
            result: The value returned by pre or post.
        """
        if cacheas is None:
            cacheas = 0
        if cache and clearcache:
            if cacheas in self._cache:
                print("Hmm... cache[%s] was not cleared correctly before traverse is called?  Or perhaps reentrant traversal?"%(str(cacheas),))
                print("Myself: %s, cached value %s"%(str(self),str(self._cache[cacheas])))
                import traceback
                traceback.print_stack()
                cache = False
        def _traverse_recurse_cache(node,pre,post):
            try:
                res = node._cache[cacheas]
                if _DEBUG_TRAVERSE and cacheas in _DEBUG_TRAVERSE_ITEMS:
                    print("Found cached value cache[%s]=%s for node %s"%(cacheas,str(res),str(self)))
                return (True,res)
            except KeyError:
                #print "Did not find cached value cache[%s]"%(cacheas,)
                pass
            value = None
            cont = True
            if pre is not None:
                try:
                    (descend,cont,value) = pre(node)
                except Exception as e:
                    import sys
                    return (False,_TraverseException(e,node,'pre-traverse '+cacheas,sys.exc_info()[2]))
                if not descend:
                    if _DEBUG_TRAVERSE and cacheas in _DEBUG_TRAVERSE_ITEMS:
                        print("Not descending under %s, caching as %s and returning value"%(str(node),cacheas,value))
                    node._cache[cacheas] = value
                    return (cont,value)
            cvals = []
            if node._children is not None:
                for i,c in enumerate(node._children):
                    try:
                        res = c._cache[cacheas]
                        if _DEBUG_TRAVERSE and cacheas in _DEBUG_TRAVERSE_ITEMS:
                            print("Found cached child %d cache[%s]=%s for node %s"%(i,cacheas,str(res),str(node)))
                        cvals.append(res)
                    except KeyError:
                        c._parent = (weakref.ref(node),i)
                        (ccont,value) = _traverse_recurse_cache(c,pre,post)
                        c._parent = None
                        if isinstance(value,_TraverseException):
                            return False,value
                        if not ccont:
                            if _DEBUG_TRAVERSE and cacheas in _DEBUG_TRAVERSE_ITEMS:
                                print("Not continuing under %s, caching as %s and returning value"%(str(node),cacheas,value))
                            node._cache[cacheas] = value
                            return (False,value)
                        cvals.append(value)
            if post is not None:
                try:
                    (cont,value) = post(node,cvals)
                except Exception as e:
                    import sys
                    return (False,_TraverseException(e,node,cacheas,sys.exc_info()[2]))
            if _DEBUG_TRAVERSE and cacheas in _DEBUG_TRAVERSE_ITEMS:
                print("Completed traversal under %s, caching as %s and returning value %s"%(str(node),cacheas,value))
            node._cache[cacheas] = value
            return (cont,value)
        def _traverse_recurse_nocache(node,pre,post):
            value = None
            cont = True
            if pre is not None:
                try:
                    (descend,cont,value) = pre(node)
                except Exception as e:
                    import sys
                    return (False,_TraverseException(e,node,'pre-traverse '+cacheas,sys.exc_info()[2]))
                if not descend:
                    if _DEBUG_TRAVERSE and cacheas in _DEBUG_TRAVERSE_ITEMS:
                        print("Not descending under",str(node),", returning value",value)
                    return (cont,value)
            cvals = []
            if node._children is not None:
                for i,c in enumerate(node._children):
                    c._parent = (weakref.ref(node),i)
                    (cont,value) = _traverse_recurse_nocache(c,pre,post)
                    c._parent = None
                    if isinstance(value,_TraverseException):
                        return False,value
                    if not cont:
                        if _DEBUG_TRAVERSE and cacheas in _DEBUG_TRAVERSE_ITEMS:
                            print("Not continuing under",str(node),", returning value",value)
                        return (False,value)
                    cvals.append(value)
            if post is not None:
                try:
                    (cont,value) = post(node,cvals)
                except Exception as e:
                    import sys
                    return (False,_TraverseException(e,node,cacheas,sys.exc_info()[2]))
            if _DEBUG_TRAVERSE and cacheas in _DEBUG_TRAVERSE_ITEMS:
                print("Completed traversal under",str(node),", returning value",value)
            return (cont,value)
        self._parent = str(cacheas)
        if cache:
            cont,value = _traverse_recurse_cache(self,pre,post)
            if clearcache:
                self._clearCache(cacheas)
        else:
            cont,value = _traverse_recurse_nocache(self,pre,post)
        self._parent = None
        if isinstance(value,_TraverseException):
            raise value.with_traceback(value.exc_info)
        return value
    def _clearCache(self,key,deep=False):
        incache = key in self._cache
        if incache or deep:
            if incache: del self._cache[key]
            if self._children is not None:
                for a in self._children:
                    a._clearCache(key,deep)
        if _DEBUG_CACHE:
            def checkcleared(n,key):
                if key in n._cache:
                    raise RuntimeError("clearCache failed to actually clear cache[%s] on %s, value %s, in context %s"%(str(key),str(n),str(n._cache),str(self)))
                if n._children is not None:
                    for a in n._children:
                        checkcleared(a,key)
            if not deep:
                checkcleared(self,key)
    def _print_call_stack(self,indent=0):
        """Prints the traversal stack.  Used for debugging purposes."""
        from . import symbolic_io
        if indent > 0:
            print(""*(indent-1), end=' ')
        sstr = '##'+str(self)+'##'
        if self._parent is None:
            print(sstr)
            return
        if isinstance(self._parent,str):
            print(self._parent,"(",sstr,")")
            return
        def call_stack_str(n,cstr,cindex):
            astr = []
            for i,a in enumerate(n._children):
                if i == cindex:
                    astr.append(cstr)
                else:
                    astr.append('...')
            nstr = symbolic_io._prettyPrintExpr(n,astr,parseCompatible=False)
            if n._parent is None or isinstance(n._parent,str):
                return nstr
            ref = n._parent[0]()
            if ref is not None:
                return call_stack_str(ref,nstr,n._parent[1])
            return nstr
        ref = self._parent[0]()
        if ref is not None:
            print(call_stack_str(ref,sstr,self._parent[1]))
    def _signature(self):
        """Returns a tuple that can be used for comparisons and hashing"""
        return ()
    def __bool__(self):
        print("__bool__ was called on",self)
        raise ValueError("Can't test the truth value of an Expression.... did you mean to use if_(...)?")
    def __not__(self):
        return not_(self)
    def __and__(self,rhs):
        return and_(self,rhs)
    def __or__(self,rhs):
        return or_(self,rhs)
    def __eq__(self,rhs):
        return eq(self,rhs)
    def __ne__(self,rhs):
        return ne(self,rhs)
    def __le__(self,rhs):
        return le(self,rhs)
    def __ge__(self,rhs):
        return ge(self,rhs)
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
    def __setitem__(self,index,val):
        return setitem(self,index,val)
    def __getattr__(self,attr):
        if attr == 'T':
            return transpose(self)
        elif attr == 'shape':
            return shape(self)
        else:
            raise AttributeError("'Expression' object has no attribute '%s'" % attr)
    def __hash__(self):
        return hash(self._signature())

class Wildcard(Expression):
    """Used for matching / finding"""
    def __init__(self,name="*"):
        self.name = name
    def match(self,val):
        return True
    def _eval(self,context):
        raise ValueError("Can't evaluate Wildcard expressions")
    def _evalf(self,context):
        raise ValueError("Can't evaluate Wildcard expressions")
    def _deriv(self,var,context,rows):
        raise ValueError("Can't take derivative of Wildcard expressions")

class ConstantExpression(Expression):
    def __init__(self,value):
        if value is None:
            raise ValueError("Can't initialize a ConstantExpression with None")
        Expression.__init__(self)
        if isinstance(value,ConstantExpression):
            self.value = value.value
        elif isinstance(value,Expression):
            raise ValueError("Can't initialize a ConstantExpression with a non-constant Expression")
        else:
            self.value = value
        def checkType(v):
            if hasattr(v,'__iter__') and not isinstance(v,str):
                for x in v:
                    checkType(x)
            assert not isinstance(v,ConstantExpression)
        checkType(self.value)
    def returnType(self):
        return type_of(self.value)
    def isConstant(self):
        return True
    def returnConstant(self,context=None):
        return True
    def _eval(self,context):
        return self.value
    def _evalf(self,context):
        return self.value
    def match(self,val):
        if not isinstance(val,Expression): val = expr(val)
        if isinstance(val,Wildcard): return True
        return isinstance(val,ConstantExpression) and np.all(self.value == val.value)
    def __len__(self):
        return len(self.value)
    def _signature(self):
        if isinstance(self.value,_PY_PRIMITIVE_TYPES):
            return (self.value,)
        elif isinstance(self.value,np.ndarray):
            return tuple(self.value.flatten())
        else:
            return (id(self.value),)
    def __bool__(self):
        return bool(self.value)
    def __not__(self):
        return ConstantExpression(not self.value)
    def _do_binary_op(self,rhs,func):
        if isinstance(rhs,(Expression,Variable)):
            if isinstance(rhs,ConstantExpression):
                return ConstantExpression(func.func(self.value,rhs.value))
            return func(self,rhs)
        else:
            return ConstantExpression(func.func(self.value,rhs))
    def __eq__(self,rhs):
        return self._do_binary_op(rhs,eq)
    def __ne__(self,rhs):
        return self._do_binary_op(rhs,ne)
    def __le__(self,rhs):
        return self._do_binary_op(rhs,le)
    def __ge__(self,rhs):
        return self._do_binary_op(rhs,ge)
    def __neg__(self):
        return ConstantExpression(-self.value)
    def __getitem__(self,index):
        return self._do_binary_op(index,getitem)
    def __getattr__(self,attr):
        if attr.startswith("_"):
            raise AttributeError("ConstantExpression object has no attribute '%s'" % attr)
        if hasattr(self.value,attr):
            return getattr(self.value,attr)
        raise AttributeError("ConstantExpression object has no attribute '%s'" % attr)

class UserDataExpression(Expression):
    def __init__(self,name):
        Expression.__init__(self)
        self.name = name
    def returnType(self):
        return Type('U')
    def isConstant(self):
        return False
    def returnConstant(self,context=None):
        if context is None: return False
        if isinstance(context,dict):
            return self.name in context
        else:
            return self.name in context.userData
    def _eval(self,context):
        if context is None:
           return self
        if isinstance(context,dict):
            return context.get(self.name,self)
        else:
            return context.userData.get(self.name,self)
    def _evalf(self,context):
        assert context is not None,"Can't evalf UserDataExpressions without a context"
        if isinstance(context,dict):
            return context[self.name]
        else:
            return context.userData[self.name]
    def _deriv(self,var,context,rows):
        print("Attempting derivative of user data expression",self.name,"w.r.t.",var)
        if isinstance(var,dict):
            return var.get(self.name,0)
        else:
            if self.name == var:
                raise ValueError("Cannot take derivatives with respect to user-data variables")
            return 0
    def match(self,val):
        if not isinstance(val,Expression): return isinstance(val,str) and val == self.name
        if isinstance(val,Wildcard): return True
        return isinstance(val,UserDataExpression) and self.name == val.name
    def vars(self,context=None,bound=False):
        if context is None or bound:
            return [self]
        elif isinstance(context,dict):
            if self.name in context: return []
        else:
            if self.name in context.userData: return []
        return [self]
    def _simplify(self,context=None,depth=None):
        if context is not None: 
            res = context.get(self.name,None)
            if isinstance(res,_PY_CONST_TYPES) or isinstance(res,(list,tuple)):
                return ConstantExpression(res)
            return None
        return None
    def _signature(self):
        return ('$'+self.name,)
    def __getattr__(self,attr):
        if attr.startswith("_"):
            raise AttributeError("UserData object has no attribute '%s'" % attr)
        return getattr_(self,attr)

class VariableExpression(Expression):
    def __init__(self,var):
        Expression.__init__(self)
        self.var = var
    def returnType(self):
        return self.var.type
    def isConstant(self):
        return False
    def returnConstant(self,context=None):
        return (self.var.value is not None)
    def _eval(self,context):
        if self.var.value is not None: return self.var.value
        if isinstance(context,dict):
            return context.get(self.var.name,self)
        return self
    def _evalf(self,context):
        if self.var.value is not None: return self.var.value
        if isinstance(context,dict):
            return context[self.var.name]
        raise ValueError("Must provide variable binding or context")
    def _deriv(self,var,context,rows):
        if isinstance(var,Variable): return self.deriv(var.name)
        elif isinstance(var,dict):
            return var.get(self.var.name,0)
        else:
            if not isinstance(var,str):
                raise ValueError("Can't take the derivative of an expression w.r.t. anything except a str, Variable, or dict")
            if self.var.name == var:
                size = self.var.type.size
                if size is None:
                    assert self.var.type.char in SCALAR_TYPES,"Can only take derivative if the size of a variable is known"
                    return 1
                else:
                    assert len(size) == 1,"Can't take derivative with respect to a non-vector Variable yet?"
                    return _eye(size[0])
            return 0
    def match(self,val):
        if isinstance(val,Wildcard): return True
        if isinstance(val,Variable): return self.var.name == val.name
        return isinstance(val,VariableExpression) and self.var.name == val.var.name
    def vars(self,context=None,bound=False):
        if not bound and self.var.value is not None: return []
        return [self.var]
    def _simplify(self,context=None,depth=None):
        if self.var.value is not None: return ConstantExpression(self.var.value)
        if isinstance(context,dict): return ConstantExpression(context.get(self.var.name,None))
        return None
    def _signature(self):
        return (self.var.name,)
    def __len__(self):
        return len(self.var)

class OperatorExpression(Expression):
    """A compound function of individual Expressions"""
    def __init__(self,finfo,args,op=None):
        Expression.__init__(self)
        self.functionInfo = finfo
        if op is not None:
            self.op = op
        else:
            self.op = finfo.func
        self.args = []
        for a in args:
            self.args.append(expr(a))
        self._children = self.args
    #def __del__(self):
    #    #remove references in parents list
    #    #print("OperatorExpression.__del__ called")
    #    for a in self.args:
    #        a._parents = [(p,index) for p,index in a._parents if p() is not None]
    def _do(self,constargs):
        """Calculates function with the given resolved argument tuple"""
        assert not any(isinstance(a,Expression) for a in constargs),"Trying to evaluate an Expression's base function with non-constant inputs?"
        assert self.op is not None
        try:
            return self.op(*constargs)
        except Exception as e:
            #this will be handled during eval's exception handler
            """
            print("Error while evaluating",self.functionInfo.name,"with arguments:")
            if self.functionInfo.argNames:
                for (name,a) in zip(self.functionInfo.argNames,constargs):
                    print(" ",name,"=",a)
            else:
                print(" ",','.join([str(a) for a in constargs]))
            print("Error",e)
            import traceback
            print("Traceback")
            traceback.print_exc()
            try:
                print("Call stack:",)
                self._print_call_stack()
            except Exception as e2:
                print("Hmm... exception occured while calling print_call_stack()?")
                print(e2)
                import traceback
                print("Traceback")
                traceback.print_exc()
            raise ValueError("Exception "+str(e)+" while evaluating function "+self.functionInfo.name+" with args "+",".join(str(v) for v in constargs))
            """
            task = "Evaluating "+self.functionInfo.name+" with args "+",".join(str(v) for v in constargs)
            import sys
            raise _TraverseException(e,self,task,sys.exc_info()[2]).with_traceback(sys.exc_info()[2])
    def returnType(self):
        if 'returnType' in self._cache:
            return self._cache['returnType']
        if self.functionInfo.returnTypeFunc != None:
            try:
                type = self.functionInfo.returnTypeFunc(*self.args)
                self._cache['returnType'] = type
                return type
            except Exception as e:
                print("Exception while evaluating",self.functionInfo.name,"return type with args",','.join(str(v) for v in self.args))
                print("   exception is:",e)
                raise
        if self.functionInfo.returnType:
            self._cache['returnType'] = self.functionInfo.returnType
            return self.functionInfo.returnType
        self._cache['returnType'] = Type(None)
        return Type(None)
    def isConstant(self):
        if 'constant' not in self._cache:
            if len(self.args) == 0:
                self._cache['constant'] = True
            else:
                self._cache['constant'] = all(a.isConstant() for a in self.args)
        return self._cache['constant']
    def returnConstant(self,context=None):
        def _returnconstant(node,childargs):
            if len(childargs)==0: 
                if isinstance(node,OperatorExpression): return (True,True)
                return (True,node.returnConstant(context))
            res = all(childargs)
            if not res:
                return (False,False)
            return (True,True)
        return self._traverse(post=_returnconstant,cache=False)
    def eval(self,context=None):
        """Returns a new expression where all bound variables and variables in context
        are reduced to constant values. If all arguments are constant, returns a
        constant expression."""
        if 'eval' in self._cache:
            print("WARNING: Expression.eval already has cached value?")
            print("  expression",self)
            print("  Cached value",self._cache['eval'])
            raise ValueError()
            input()
        res = self._eval(context)
        self._clearCache('eval')
        return res
    def _eval(self,context):
        def _preeval(node):
            if isinstance(node,OperatorExpression) and hasattr(node.functionInfo,'custom_eval'):
                return (False,True,node.functionInfo.custom_eval(*([context]+node.args)))
            return (True,True,None)
        def _posteval(node,aexprs):
            if not isinstance(node,OperatorExpression):
                return (True,node._eval(context))
            if any(isinstance(a,Expression) for a in aexprs):
                if any(a is not b for a,b in zip(aexprs,node.args)):
                    return (True,OperatorExpression(node.functionInfo,aexprs))
                else:
                    return (True,node)
            else:
                return (True,node._do(aexprs))
        return self._traverse(_preeval,_posteval,cacheas='eval',clearcache=False)
    def _evalf(self,context):
        def _preeval(node):
            if isinstance(node,OperatorExpression) and hasattr(node.functionInfo,'custom_eval'):
                res = node.functionInfo.custom_eval(*([context]+node.args))
                if isinstance(res,ConstantExpression):
                    res = res.value
                assert not isinstance(res,Expression),"Invalid evalf call on expression with custom eval"
                return (False,True,res)
            return (True,True,None)
        def _posteval(node,aexprs):
            if not isinstance(node,OperatorExpression):
                return (True,node._evalf(context))
            return (True,node._do(aexprs))
        return self._traverse(_preeval,_posteval,cacheas='evalf')
    def deriv(self,var,context=None):
        """Returns an expression for the derivative dself/dvar, where all
        bound variables and variables in context are reduced to constant
        values.
        
        Normally, ``var`` is a ``Variable`` or ``str``.

        The derivative/Jacobian size is determined as follows:

        - If both var and self are scalar, the result is a scalar.
        - If self is scalar and var is non-scalar, the result has the same
          shape as var.
        - If self is non-scalar and var is scalar, the result the same shape
          as self.
        - If both self and var are arrays, the result is an n-D array with
          shape shape(self)+shape(var).
        - If other self or var are compound (i.e., unstructured lists), the
          result is an m x n matrix, where m = count(self) and n = count(var).

        If no derivative can be determined, this returns None.

        If the expression evaluates to a constant, the result will be either 0
        or a constant expression.
        
        Another option is for ``var`` to be a dict of variable names to their
        derivatives.
        """
        if not isinstance(var,dict):
            if isinstance(var,Variable):
                name = var.name
            else:
                name = var
                
                vd = self.find(VariableExpression(Variable(var,None)))
                if vd is not None:
                    var = vd.var
                else:
                    ud = self.find(UserDataExpression(name))
                    if ud is not None:
                        warnings.warn("symbolic.deriv: taking the derivative w.r.t. user data {}, assuming a numeric value".format(var))
                        var = VariableExpression(Variable(var,'N'))
                    else:
                        raise ValueError("Can't take the derivative w.r.t. undefined variable "+var)

            var_irregular = var.type.char not in SCALAR_TYPES + ARRAY_TYPES
            try:
                var_shape = var.type.shape()
            except Exception:
                var_shape = shape.optimized(var)
            if not var_irregular:
                var_scalar = False
                if isinstance(var_shape,tuple):
                    #array type, fixed shape
                    var_cols = np.product(var_shape)
                    if var_shape == ():
                        var_cols = 0
                    var_dims = len(var_shape)
                    var_scalar = (var_cols == 0)
                else:
                    #array type, variable shape
                    var_scalar = False
                    var_cols = count.optimized(var)
                    var_dims = dims.optimized(var)
                    assert is_const(var_dims),"Variable dimensions are not constant? "+str(var_dims)+' type '+str(var.type)
                    var_dims = to_const(var_dims)
                #reshape arrays with 2 or more dims 
                var_deriv_reshape = (var_dims > 1)
            else:
                #irregular type
                var_cols = count.optimized(var)
                if is_const(var_cols):
                    var_cols = int(to_const(var_cols))
                var_scalar = False
                var_dims = dims.optimized(var)
                #don't reshape hyper-types, keep them as count
                var_deriv_reshape = False
            #print "Number of columns of deriv variable",var,"is",cols
            #if not is_const(cols):
            #    print "WARNING: variable number of columns",cols,"in derivative Variable type",var.type
            if isinstance(var_cols,int) and var_cols == 0:
                varderivs = {name:1}
                assert var_deriv_reshape == False
            elif isinstance(var_cols,int) and var_cols == 1:
                varderivs = {name:eye.optimized(var_cols)}
                assert var_deriv_reshape == False
            else:
                if var_deriv_reshape:
                    varderivs = {name:reshape._call(eye._call(var_cols),flatten.optimized(var_cols,var_shape))}
                else:
                    varderivs = {name:eye._call(var_cols)}
        else:
            #var is a dict
            varderivs = var
            var_deriv_reshape = False
            #because some functions' derivatives may call Expression.deriv again with the
            #same dict, we need to determine the dict size dynamically
            var_cols = 0
            for (k,v) in varderivs.items():
                if is_op(v,'eye'):
                    var_cols = v.args[0]
                elif is_op(v,'reshape'):
                    var_cols = getitem.optimized(v.args[1],0)
                elif is_const(v):
                    var_cols = 0
                break
            print("CALLING DERIV",self,"WITH ARG DERIV dict INFERRED COLUMNS",var_cols)
            if to_const(var_cols) is not None:
                var_cols = to_const(var_cols)
            var_scalar = is_scalar(var_cols,0)
            var_dims = 0
            var_irregular = False
        
        res = self._deriv(varderivs,context,var_cols)

        self._clearCache('deriv',deep=True)

        if _DEBUG_DERIVATIVES:
            print("symbolic.deriv: Derivative of",self,"w.r.t.",list(varderivs.keys()),"is",res)

        if res is None:
            #error occurred, return None
            return None

        self_type = self.returnType()
        self_irregular = self_type is None or self_type.char not in SCALAR_TYPES + ARRAY_TYPES
        self_dims = dims.optimized(self) if self_type is None else self_type.dims()
        if not is_const(self_dims):
            if _DEBUG_DERIVATIVES:
                warnings.warn("symbolic.deriv: can't determine dimensions of self, trying eval")
            self_dims = dims.optimized(self.eval(context))
            if _DEBUG_DERIVATIVES:
                if not is_const(self_dims):
                    warnings.warn("   ... really cannot determine dimensions of self. Proceeding as though non-scalar")
        self_scalar = is_scalar(self_dims,0)
        if not var_scalar and not self_scalar:
            if to_const(self_dims+1) == 2:
                if _DEBUG_DERIVATIVES:
                    print("symbolic.deriv: doing transpose of",res)
                res = transpose(res)
            else:
                #use the proper transpose for higher dimensional derivatives
                if _DEBUG_DERIVATIVES:
                    print("symbolic.deriv: using tensor transpose for self dims",self_dims,"var dims",var_dims)
                if self_dims > 2:
                    print("WEIRD TENSOR RESULT",self)
                res = transpose2(res,flatten(1+range_(dims.optimized(res)-1),0))
        else:
            #print "symbolic.deriv: not doing transpose of",res
            pass
        
        """
        if self_irregular or var_irregular:
            print("derivative result",res,"with irregular self?",self_irregular,"or var:",var_irregular)
            print("Res shape",shape.optimized(res))
            print("Self shape",shape.optimized(self))
            print("Var shape",var_shape)
            print("Self scalar:",self_scalar,"var scalar:",var_scalar)
        """
        if self_scalar and (var_irregular or var_deriv_reshape):
            if _DEBUG_DERIVATIVES:
                print("symbolic.deriv: reshaping result to",var_shape)
            res = reshape(res,var_shape)
        if not self_scalar and var_deriv_reshape:
            if _DEBUG_DERIVATIVES:
                print("symbolic.deriv: reshaping result to",simplify(flatten(shape(res)[0:-1],var_shape)))
            res = reshape(res,simplify(flatten(shape(res)[0:-1],var_shape)))
        
        if isinstance(res,OperatorExpression):
            rsimp = res._postsimplify(depth=None)
            res._clearCache('simplified',deep=True)
            if rsimp is not None:
                if _DEBUG_DERIVATIVES:
                    print("symbolic.deriv: Post-simplified derivative from",res,"to",rsimp)
                res = rsimp
            else:
                if _DEBUG_DERIVATIVES:
                    print("symbolic.deriv: Couldn't simplify")
            if isinstance(res,OperatorExpression):
                rsimp = res._constant_expansion(context,depth=None)
                res._clearCache('simplified',deep=True)
                if _DEBUG_DERIVATIVES:
                    if rsimp is not None:
                        print("symbolic.deriv: Constant expansion of derivative from",res,"to",rsimp)
            if rsimp is not None: return rsimp
            return res
        return res

    def _deriv(self,varderivs,context,rows=None):
        #varderivs must contain a map from variable names var to derivatives dvar.
        #
        #If rows == 0, then each dvar is a derivative of var w.r.t. a scalar parameter x.
        #
        #If rows > 0, then each dvar is a row-stacked array of shape [rows] + shape(var)
        #if var is array-like, or (rows,count(var)) otherwise.
        assert len(varderivs) > 0,"Must provide at least one variable derivative"
        assert rows is not None
        var_scalar = is_scalar(rows,0)
        if var_scalar:
            assert isinstance(to_const(rows),int),"NUMBER OF ROWS ISNT INTEGER? %s %s"%(str(rows),rows.__class__.__name__)
            def _jacobian_shape(node):
                return shape.optimized(node)
        else:
            def _jacobian_shape(node):
                nt = node.returnType()
                if nt is not None and nt.char in SCALAR_TYPES+ARRAY_TYPES:
                    return flatten.optimized(rows,shape.optimized(node))
                else:
                    return array.optimized(rows,count.optimized(node))

        def _deriv_pre(node):
            if isinstance(node,OperatorExpression) and hasattr(node.functionInfo,'custom_eval'):
                if node.functionInfo.deriv is None:
                    return (False,True,None)
                if _is_exactly(node.functionInfo.deriv,0):
                    return (False,True,None)
                assert callable(node.functionInfo.deriv),"custom_eval functions needs to define a callable deriv function"
                if var_scalar:
                    assert isinstance(to_const(rows),int),"NUMBER OF ROWS ISNT INTEGER? %s %s"%(str(rows),rows.__class__.__name__)
                res = node.functionInfo.deriv(*([context]+node.args+[varderivs,rows]))
                if _is_exactly(res,0):
                    try:
                        res = zero._call(_jacobian_shape(node))
                    except Exception:
                        #there's a problem getting the shape... let's hope that it's ok to return 0
                        pass
                elif res is None:
                    pass
                else:
                    pass
                    #print("In branch",node,"deriv",res)
                    #raise NotImplementedError("Haven't tested this branch... why do we need a transpose?")
                    #res = transpose.optimized(res)
                return (False,True,res)
            return (True,True,None)
        def _deriv_post(node,cvals):
            res = 0
            if isinstance(node,OperatorExpression):
                if _is_exactly(node.functionInfo.deriv,0):
                    res = 0
                elif node.functionInfo.deriv is None:
                    if all(_is_exactly(v,0) or is_zero(v) for v in cvals): 
                        if _DEBUG_DERIVATIVES:
                            print("symbolic.deriv: No derivative for function",node.functionInfo.name,"but we happily had no variables to consider")
                            print("  Values of arguments")
                            for v in node.args:
                                print("    ",v)
                            print("  Derivatives of arguments")
                            for v in cvals:
                                print("    ",simplify(v))
                        res = 0
                    else:
                        warnings.warn("symbolic.deriv: No derivative for function {}".format(node.functionInfo.name))
                        return True,None
                elif all(_is_exactly(v,0) or is_zero(v) for v in cvals): 
                    if _DEBUG_DERIVATIVES:
                        print("symbolic.deriv: zero derivatives of arguments for",node.functionInfo.name)
                    res = 0
                else:
                    if _DEBUG_DERIVATIVES:
                        print("symbolic.deriv: Computing raw derivative of %s(%s)"%(node.functionInfo.name,','.join(str(a) for a in node.args)))
                        print("   w.r.t.",', '.join('d/d'+k+'='+str(v) for (k,v) in varderivs.items()))
                        if len(cvals) <= 1:
                            print("   with argument derivatives",','.join([str(v) for v in cvals]))
                        else:
                            print("   with argument derivatives")
                            for v in cvals:
                                print("   -",v)
                    #ACTUALLY DO THE DERIVATIVE
                    res = node._do_deriv(cvals,rows)
                    #if _DEBUG_DERIVATIVES:
                    #    print("  Result is %s (type %s)"%(str(simplify(res)),res.__class__.__name__))
                    #    print("  unsimplified",res)
                    #print "Result of derivative of",node.functionInfo.name,"w.r.t.",[(k,str(v)) for (k,v) in varderivs.iteritems()],"is",str(res)
            elif isinstance(node,VariableExpression):
                if node.var.name in varderivs:
                    res = varderivs[node.var.name]
            elif isinstance(node,UserDataExpression):
                if node.name in varderivs:
                    res = varderivs[node.name]
            elif isinstance(node,ConstantExpression):
                res = 0
            if res is not None and not _is_exactly(res,0):
                dconst = to_const(dims.optimized(node))
                if dconst is None:
                    raise ValueError("Odd, dimensions of node "+str(node)+" don't have constant # of dimensions?  dims="+str(dims.optimized(node)))
                if not var_scalar and dconst > 1:
                    rshape = shape.optimized(res)
                    outshape = _jacobian_shape(node)
                    if not is_const(rshape) or not is_const(outshape):
                        if not rshape.match(outshape):
                            warnings.warn("symbolic.deriv: had to reshape result of derivative of {}\n   had type {} need shape {}".format(node,res.returnType(),outshape))
                    elif not np.array_equal(to_const(rshape),to_const(outshape)):
                        warnings.warn("symbolic.deriv: had to reshape result of derivative of {}\n   had type {} need shape {}".format(node,rshape,outshape))
                        res = reshape.optimized(res,outshape)
            if _is_exactly(res,0) and not _is_exactly(_jacobian_shape(node),()):
                #no derivative
                try:
                    res = zero._call(_jacobian_shape(node))
                    if _DEBUG_DERIVATIVES:
                        print("symbolic.deriv: Converted zero jacobian of",node,"to size",_jacobian_shape(node))
                except Exception as e:
                    #there's a problem getting the shape... let's hope that it's ok to return 0
                    if _DEBUG_DERIVATIVES:
                        print("Uh... couldn't get the jacobian shape of",node)
                        print("=== REASON ===")
                        print(e)
                        print("=== END REASON ===")
                        pass
            if _DEBUG_DERIVATIVES:
                print("symbolic.deriv: Derivative of",node,"with arg derivs",cvals,"is (unsimplified)",res)
            return True,res
        return self._traverse(pre=_deriv_pre,post=_deriv_post,cacheas='deriv',clearcache=False)
        

    def _do_deriv(self,dargs,stackcount):
        """Returns the derivative expression of the output with respect to some
        variable x, when dargs is a list [dargs[i]/dx for i = 0,...,nargs-1]

        If a derivative is not available, returns None
        If a derivative is 0 or a zero vector/matrix, should return 0

        If stackcount>0, each entry dargs[i] is an array with shape [stackcount]+shape(args[i]))
        if args[i] is array-like, or a matrix with shape (stackcount,count(args[i])) if args[i]
        is complex.

        If stackcount==0, the result of this operation has shape shape(self). 
        If stackcount > 0, the result is a matrix with shape [stackcount]+shape(self) if
        self is array-like, or a matrix with shape (stackcount,count(self)) if self is complex.
        """
        if self.functionInfo.deriv is None: return None
        if _is_exactly(self.functionInfo.deriv,0): return 0
        assert len(dargs) == len(self.args),"Invalid argument to deriv: not enough arguments"
        #short-circuit termination: no derivative available for sub-argument
        if any(v is None for v in dargs): return None

        #check what kind of derivative / jacobian we should produce
        cstackcount = to_const(stackcount)
        do_stack = cstackcount is None or cstackcount > 0
        self_dims = dims.optimized(self)
        self_type = self.returnType()

        if not is_const(self_dims):
            if _DEBUG_DERIVATIVES:
                print("Can't determine dimensionality of expression",self,"type",self.returnType())
                print("  dims type",self_dims.__class__.__name__)
                print("  Proceeding by assuming non-scalar result")
                input("Press enter to continue...")
        else:
            self_dims = to_const(self_dims)

        self_irregular = self_type is None or self_type.char not in SCALAR_TYPES + ARRAY_TYPES
        if not self_irregular:
            try:
                self_shape = self_type.shape()
            except Exception:
                self_shape = shape.optimized(self)
            if do_stack:
                jacobian_shape = flatten.optimized(stackcount,self_shape)
                jacobian_dims = 1 + self_dims
            else:
                jacobian_shape = self_shape
                jacobian_dims = self_dims
        else:
            if do_stack:
                self_shape = None
                self_count = count.optimized(self)
                jacobian_shape = array.optimized(stackcount,self_count)
                jacobian_dims = 2
            else:
                self_shape = shape.optimized(self)
                jacobian_shape = self_shape
                jacobian_dims = self_dims

        if do_stack:
            #determine whether the arguments' are array-like or not.  None indicates non-array like
            arg_dims = []
            for (a,da) in zip(self.args,dargs):
                art = a.returnType()
                if art is not None and art.char in SCALAR_TYPES + ARRAY_TYPES:
                    arg_dims.append(art.dims())
                else:
                    arg_dims.append(None)
            res = None
            if _DEBUG_DERIVATIVES:
                print("symbolic.deriv: Reshaping arguments to",self,"have dims",arg_dims)
            if callable(self.functionInfo.rowstackderiv):
                if _DEBUG_DERIVATIVES:
                    print("symbolic.deriv: Using row-stack derivative to",self.functionInfo.name)
                if any(v is None for v in dargs): return None
                #print("callback rowstackderiv: Shapes",[simplify(shape(da)) for da in dargs])
                #print("Needs reshaping?",needs_reshaping)
                res = self.functionInfo.rowstackderiv(self.args,dargs)
                #print("rowstackderiv,",self.functionInfo.name,"result",res)
            elif callable(self.functionInfo.colstackderiv):
                if _DEBUG_DERIVATIVES:
                    print("symbolic.deriv: Using column-stack derivative to",self.functionInfo.name)
                daresized = []
                for (a,da,adims) in zip(self.args,dargs,arg_dims):
                    if da is 0:
                        daresized.append(0)
                    elif adims is None:
                        #can't determine dimensions: prepare for the worst case tensor
                        daresized.append(transpose2(da,flatten(1+range_(adims-1),0)))
                    elif adims == 0:
                        daresized.append(da)
                    elif adims == 1:
                        daresized.append(transpose.optimized(da))
                    else:
                        daresized.append(transpose2(da,list(range(1,adims))+[0]))
                res = self.functionInfo.colstackderiv(self.args,daresized)
                if res is not None:
                    if jacobian_dims <= 1:
                        pass
                    elif jacobian_dims == 2:
                        res = transpose.optimized(res)
                    else:
                        raise ValueError("Can't rearrange column-stacked jacobians for matrix functions yet")
            elif callable(self.functionInfo.deriv):
                assert is_const(stackcount),"Can't do functional derivatives yet with variable stack size"
                row_derivs = []
                #print("DERIV",self.functionInfo.name,"WITH STACK COUNT",stackcount)
                #print([str(da) for da in dargs])
                for i in range(stackcount):
                    daresized = []
                    for a,da,adims in zip(self.args,dargs,arg_dims):
                        if da is 0:
                            daresized.append(0)
                        elif is_op(da,'zero'):
                            if adims == 0:
                                daresized.append(0)
                            else:
                                daresized.append(zero(shape.optimized(a)))
                        elif adims is None:
                            #non-array-like, needs reshaping
                            daresized.append(reshape.optimized(getitem.optimized(da,i),shape.optimized(a)))
                        else:
                            daresized.append(getitem.optimized(da,i))
                    #print "  Reshaped i'th derivatives",[str(da) for da in daresized]
                    di = self.functionInfo.deriv(self.args,daresized)
                    dishape = to_const(shape.optimized(di))
                    if dishape is not None and to_const(self_shape) is not None:
                        assert dishape == self_shape,"Result of derivative %s should have shape %s, instead has %s"%(self.functionInfo.name,str(self_shape),str(dishape))
                    if self_irregular:
                        di = flatten.optimized(di)
                    row_derivs.append(di)
                    
                res = array(*row_derivs)
            else:
                #individual deriv, rowstackderiv, colstackderiv, or jacobian functions
                #order from most efficient to least: rowstackderiv, colstackderiv, jacobian (if stackcount > count(a), deriv
                #print("INDIVIDUAL DERIVS",self.functionInfo.name,"WITH STACK COUNT",stackcount)
                #print([str(simplify(da)) for da in dargs])
                res = 0
                for index,da in enumerate(dargs):
                    if _is_exactly(self.functionInfo.deriv[index],0): continue
                    if _is_exactly(da,0) or is_op(da,'zero'): continue
                    adims = arg_dims[index]
                    needs_reshaping_i = (arg_dims[index] is None)
                    if self.functionInfo.rowstackderiv is not None and self.functionInfo.rowstackderiv[index] is not None:
                        inc = self.functionInfo.rowstackderiv[index](*(self.args+[da]))
                    elif self.functionInfo.colstackderiv is not None and self.functionInfo.colstackderiv[index] is not None:
                        if adims is None:
                            daresized = transpose2(da,flatten(1+range_(adims-1),0))
                        elif adims == 0:
                            daresized = da
                        elif adims == 1:
                            daresized = transpose.optimized(da)
                        else:
                            daresized = transpose2(da,list(range(1,adims))+[0])
                        #print("Stack",simplify(da_stack),"has shape",simplify(shape(da_stack)))
                        inc = self.functionInfo.colstackderiv[index](*(self.args+[daresized]))
                        if _DEBUG_DERIVATIVES:
                            print("symbolic.deriv: Column stack form",index," dargs:",daresized)
                            print("   raw derivative",inc)
                        if inc is not None:
                            #convert from column to row form
                            if jacobian_dims <= 1:
                                pass
                            elif jacobian_dims == 2:
                                if _DEBUG_DERIVATIVES:
                                    print("symbolic.deriv: doing transpose to convert",inc,"into row form")
                                inc = transpose.optimized(inc)
                            else:
                                raise ValueError("Can't rearrange column-stacked jacobians for matrix functions yet")
                    else:
                        use_jacobian = self.functionInfo.jacobian is not None and self.functionInfo.jacobian[index] is not None
                        if self.functionInfo.deriv is not None and self.functionInfo.deriv[index] is not None:
                            if adims != 2 and adims != None:
                                #not flattened?
                                use_jacobian = False
                            nargs = to_const(count.optimized(self.args[index]))
                            if use_jacobian and nargs is not None and (cstackcount is not None and nargs > cstackcount):
                                #jacobian is less efficient
                                use_jacobian = False
                        if use_jacobian:
                            #print("symbolic.deriv: Using jacobian for",self.functionInfo.name,"arg",index)
                            J = self.functionInfo.jacobian[index](*self.args)
                            assert J is not None
                            Jconst = to_const(J)
                            if Jconst is not None:
                                da_cols = shape.optimized(da)[-1]
                                if is_const(da_cols):
                                    assert Jconst.shape[-1] == to_const(da_cols),"Invalid jacobian size for %s argument %d"%(self.functionInfo.name,index)
                            inc = dot(da,transpose(J))
                        else:
                            #da is stacked, but deriv can't handle them
                            if is_const(stackcount):
                                """
                                print("arg_shape",arg_shape,"needs reshaping?",needs_reshaping_i)
                                for i in range(stackcount):
                                    print("Argument derivative",i,":",simplify(reshape.optimized(da[i],arg_shape)))
                                    print("Orig shape",shape.optimized(da[i]))
                                    print("Shape",shape.optimized(reshape.optimized(da[i],arg_shape)))
                                    print("Shape",shape.optimized(reshape(da[i],arg_shape)))
                                """
                                darows = [getitem.optimized(da,i) for i in range(stackcount)]
                                ashape = shape.optimized(self.args[index])
                                rows = [self.functionInfo.deriv[index](*(self.args+[reshape.optimized(darow,ashape) if needs_reshaping_i else darow])) for darow in darows]
                                inc = expr(rows)
                            else:
                                #Need an apply_ and concat function
                                #inc = array(*map_(apply_(self.functionInfo.deriv[index],flatten(self.args,[reshaper(da['i'])])),'i',range_(stackcount))
                                raise NotImplementedError("Dynamically sized dispatch to stacked derivatives of function %s?"%(self.functionInfo.name,))
                    if inc is None:
                        return None
                    if _is_exactly(res,0):
                        res = inc
                    else:
                        if to_const(shape.optimized(inc)) is not None:
                            if self_irregular:
                                assert to_const(shape.optimized(inc)) == (to_const(stackcount),to_const(self_count)),"Invalid (irregular)jacobian size %s, should be %s"%(shape.optimized(inc),(stackcount,self_count))
                            else:
                                assert to_const(shape.optimized(inc)) == to_const(jacobian_shape),"Invalid jacobian size %s, should be %s"%(shape.optimized(inc),jacobian_shape)
                        res = add(res,inc)
                    #res += inc
            if not _is_exactly(res,0):
                if res is None:
                    return res
                if self_irregular:
                    if _DEBUG_DERIVATIVES:
                        print("symbolic.deriv: I think I need to reshape the result of",self,"to",jacobian_shape)
                    res = reshape(res,jacobian_shape)
            return res

        #normal derivative, stackcount = 0
        if callable(self.functionInfo.deriv):
            return self.functionInfo.deriv(self.args,dargs)
        res = 0
        assert len(self.functionInfo.deriv) == len(self.args)
        for index,da in enumerate(dargs):
            if _is_exactly(self.functionInfo.deriv[index],0): continue
            if _is_exactly(da,0) or is_op(da,'zero'): continue
            if self.functionInfo.deriv[index] is None:
                if self.functionInfo.jacobian is not None and self.functionInfo.jacobian[index] is not None:
                    J = self.functionInfo.jacobian[index](*self.args)
                    assert J is not None
                    Jconst = to_const(J)
                    if Jconst is not None:
                        da_cols = shape.optimized(da)[-1]
                        if is_const(da_cols):
                            assert Jconst.shape[-1] == to_const(da_cols),"Invalid jacobian size for %s argument %d"%(self.functionInfo.name,index)
                    inc = dot(J,da)
                else:
                    warnings.warn("symbolic.deriv: No partial derivative for function %s argument %d (%s)"%(self.functionInfo.name,index+1,self.functionInfo.argNames[index]))
                    warnings.warn("  Derivative with respect to argument is {}".format(da))
                    return None
            else:
                arg = self.args[index]
                # and not is_const(stackcount) or to_const(stackcount) > 0?
                if not callable(self.functionInfo.deriv[index]):
                    #df/di = const (usually 0)
                    inc = self.functionInfo.deriv[index]
                else:
                    #da is not stacked
                    #reshape da to the format of self.args[index]
                    inc = self.functionInfo.deriv[index](*(self.args+[da]))
            if _DEBUG_DERIVATIVES:
                print("symbolic.deriv: Partial derivative for function",self.functionInfo.name,"argument",index+1,str(simplify(inc)))
            if inc is None:
                return None
            if _is_exactly(res,0):
                res = inc
            else:
                res = add(res,inc)
            #res += inc
        if SHAPE_CHECKING and not _is_exactly(res,0):
            myshape = to_const(shape.optimized(self))
            resshape = to_const(shape.optimized(res))
            if myshape is not None and resshape is not None:
                if myshape != resshape:
                    warnings.warn("symbolic.deriv: derivative doesn't match my shape? derivative shape %s = shape(%s)"%(resshape,simplify(res)))
                    warnings.warn("  compared to self's shape %s = shape(%s)"%(myshape,simplify(self)))
        return res

    def vars(self,context=None,bound=False):
        varset = set()
        varlist = []
        def _vars_pre(node):
            if isinstance(node,OperatorExpression) and hasattr(node.functionInfo,"custom_eval"):
                #has a bound inner expression -- don't count temporary variables inside
                if node.functionInfo.name in ['subs','map','forall','forsome','summation']:
                    exprvars = node.args[0].vars(context,bound)
                    loopvars = node.args[1]
                    replvars = node.args[2].vars(context,bound)
                    if is_op(loopvars,'array'):
                        for v in loopvars.args:
                            assert isinstance(v,UserDataExpression)
                            exprvars = [x for x in exprvars if x.name != v.name]
                    else:
                        assert isinstance(loopvars,UserDataExpression)
                        exprvars = [x for x in exprvars if x.name !=  loopvars.name]
                    
                    for v in exprvars + replvars:
                        if v.name not in varset:
                            varlist.append(v)
                            varset.add(v.name)
                    """
                    #DEBUGGING
                    print("Expression free variables",[x.name for x in exprvars],"replaced variables",[x.name for x in replvars])
                    exprvars += replvars
                    evars = node.functionInfo.custom_eval(context,*node.args).vars()
                    print("Custom eval",node.functionInfo.custom_eval(context,*node.args))
                    print("Test free variables",[x.name for x in evars])
                    assert all(x.name in [y.name for y in evars] for x in exprvars)
                    assert all(x.name in [y.name for y in exprvars] for x in evars)
                    """
                    return False,True,None
                    
            return True,True,None
        def _vars(node,cvals):
            if isinstance(node,OperatorExpression):
                return True,None
            vlist = node.vars(context,bound)
            for v in vlist:
                if v.name not in varset:
                    varlist.append(v)
                    varset.add(v.name)
            return True,None
        self._traverse(pre=_vars_pre,post=_vars,cacheas='vars')
        return varlist
    def match(self,expr):
        if isinstance(expr,Wildcard): return True
        if not isinstance(expr,OperatorExpression): return False
        if self.functionInfo.name != expr.functionInfo.name: return False
        if len(self.args) != len(expr.args): return False
        if 'depth' in self._cache:
            if self._cache['depth'] != expr._cache.get('depth',None): return False
        for (a,b) in zip(self.args,expr.args):
            if not a.match(b): return False
        return True
    def find(self,term):
        if not isinstance(term,Expression):
            term = expr(term)
        #TODO: don't use depth caching when term contains a wildcard?
        self.depth(cache=True)
        term.depth(cache=True)
        tdepth = term._cache['depth']
        def _find_pre(node):
            nd = node._cache['depth']
            if nd > tdepth: return (True,True,None)
            elif nd == tdepth: 
                if node.match(term):
                    return (False,False,node)
                else:
                    return (True,True,None)
            #break of search if the tree is too shallow to contain expr
            return (False,True,None)
        def _find_post(node,cmatches):
            matched = [v for v in cmatches if v is not None]
            if len(matched) > 0: return False,matched[0]
            return True,None
        res = self._traverse(pre=_find_pre,post=_find_post,cacheas='find')
        return res
    def replace(self,term,termReplace,error_no_match=True):
        if not isinstance(term,Expression):
            term = expr(term)
        if not isinstance(termReplace,Expression):
            termReplace = expr(termReplace)
        self.depth(cache=True)
        term.depth(cache=True)
        tdepth = term._cache['depth']
        matched = [False]
        def _replace_post(node,cres):
            nd = node._cache['depth']
            if nd > tdepth:
                assert isinstance(node,OperatorExpression)
                return (True,OperatorExpression(node.functionInfo,cres,node.op))
            elif nd == tdepth:
                if node.match(term):
                    #can't just say matched = True since matched is out of scope
                    matched[0] = True
                    return (True,termReplace)
                return (True,node)
            return (True,node)
        res = self._traverse(post=_replace_post,cacheas='replace')
        if not matched[0] and error_no_match:
            raise ValueError("Unable to find term %s in expression %s"%(str(term),str(self)))
        return res

    def _presimplify(self,depth):
        if _is_exactly(depth,0): return None
        newdepth = None if depth is None else depth-1
        
        changed = False
        argsChanged = False
        expr = self
        for a in self.args:
            assert isinstance(a,Expression),"Uh... args need to be Expressions, got an arg %s of type %s"%(str(a),a.__class__.__name__)
        res = self.functionInfo.simplify(self.args,pre=True)
        if res is not None:
            expr = res
            changed = True
            if not isinstance(expr,OperatorExpression): return expr
        newargs = []
        for i,a in enumerate(expr.args):
            a._parent = (weakref.ref(expr),i)
            ares = (a._presimplify(newdepth) if isinstance(a,OperatorExpression) else None)
            a._parent = None
            if ares is not None:
                newargs.append(ares)
                argsChanged = True
            else:
                newargs.append(a)
        if argsChanged:
            return OperatorExpression(expr.functionInfo,newargs,expr.op)
        else:
            return expr
        return None
    def _postsimplify(self,depth,mydepth=0):
        if 'simplified' in self._cache:
            if _DEBUG_SIMPLIFY:
                print(" "*mydepth,"Postsimplify",self,"cached to",self._cache["simplified"])
            return self._cache['simplified']
        if _is_exactly(depth,0):
            return None
        newdepth = None if depth is None else depth-1

        simplified = False
        newargs = []
        if _DEBUG_SIMPLIFY:
            print(" "*mydepth,"Postsimplify",self.functionInfo.name,":",self)
        if self.functionInfo.name == 'subs':
            #DON'T simplify variable lists or values-as-list
            newargs= [self.args[0],self.args[1],self.args[2]]
            values = self.args[2]
            if is_op(values,'array') or is_op(values,'list'):
                newvalues = []
                for i,v in enumerate(values.args):
                    if isinstance(v,OperatorExpression):
                        v._parent = (weakref.ref(values),i)
                        vsimp = v._postsimplify(newdepth,mydepth+1)
                        v._parent = None
                    else:
                        vsimp = None
                    if vsimp is None:
                        newvalues.append(v)
                    else:
                        if _DEBUG_SIMPLIFY:
                            print(" "*mydepth,"  Simplified subs value",v,"to",vsimp)
                        newvalues.append(vsimp)
                        simplified = True
                newargs[-1] = array(*newvalues)
        else:
            for i,a in enumerate(self.args):
                if isinstance(a,OperatorExpression):
                    a._parent = (weakref.ref(self),i)
                    asimp = a._postsimplify(newdepth,mydepth+1)
                    a._parent = None
                else:
                    asimp = None
                if asimp is None:
                    newargs.append(a)
                else:
                    if _DEBUG_SIMPLIFY:
                        print(" "*mydepth,"  Simplified arg",a,"to",asimp)
                    newargs.append(asimp)
                    simplified = True
        for a in newargs:
            assert isinstance(a,Expression),"Uh... args need to be Expressions, got an arg %s of type %s"%(str(a),a.__class__.__name__)
        #default simplifications
        #1. inversion
        if len(newargs) == 1 and 'inverse' in self.functionInfo.properties:
            if isinstance(newargs[0],OperatorExpression) and newargs[0].functionInfo.name == self.functionInfo.properties['inverse'].name:
                res = newargs[0].args[0]
                self._cache['simplified'] = res
                if _DEBUG_SIMPLIFY:
                    print(" "*mydepth,"Simplified operation",self.functionInfo.name,"in context",OperatorExpression(self.functionInfo,newargs),"via inverse rule")
                if isinstance(res,ConstantExpression):
                    assert res.value is not None
                return res
        #2. foldable variable-argument operators
        if self.functionInfo.properties.get('foldable',False):
            associative = self.functionInfo.properties.get('associative',False)
            assert associative,"Foldable functions need to also be associative"
            newargs2 = []
            for a in newargs:
                if isinstance(a,OperatorExpression) and a.functionInfo is self.functionInfo:
                    newargs2 += a.args
                else:
                    newargs2.append(a)
            if len(newargs2) > len(newargs):
                simplified = True
                if _DEBUG_SIMPLIFY:
                    print(' '*mydepth,"Folded variable argument operator",self.functionInfo.name,"args from",[str(e) for e in newargs],"to",[str(e) for e in newargs2])
                newargs = newargs2
        #custom simplification
        assert not any(a is None or (isinstance(a,ConstantExpression) and a.value is None )for a in newargs)
        for a in newargs:
            assert isinstance(a,Expression),"Uh... args need to be Expressions, got an arg %s of type %s"%(str(a),a.__class__.__name__)
        try:
            res = self.functionInfo.simplify(newargs)
            if res is not None:
                if _DEBUG_SIMPLIFY:
                    print(" "*mydepth,"Simplified operation",self.functionInfo.name,"in context",OperatorExpression(self.functionInfo,newargs),"via simplifier to",res)
                    #print "Args",[str(a) for a in newargs]
                if isinstance(res,OperatorExpression):
                    res2 = res._postsimplify(depth,mydepth)
                    if res2 is not None:
                        #should we clear the cache? less efficient for future operations, but better to be on the safe side
                        res._clearCache('simplified',deep=True)
                        if _DEBUG_SIMPLIFY:
                            print("... re-simplified to",res2)
                        res = res2
                if not isinstance(res,Expression):
                    res = expr(res)
                self._cache['simplified'] = res
                return res
            else:
                pass
                #print "Unable to simplify",self
                #print "Call stack",self._print_call_stack()
        except Exception as e:
            print(" "*mydepth,"  Call stack:", end=' ')
            self._print_call_stack()
            print(" "*mydepth,"Exception:",e)
            import traceback
            traceback.print_exc()
            print(" "*mydepth,"Error post-simplifying function",OperatorExpression(self.functionInfo,newargs,self.op))
            raise
            return None
        #5. If-collapsing: pull all equivalent conditions to front
        #op(if(cond,arg1,arg1'),arg2) => if(cond,op(arg1,arg2),op(arg1',arg2)) if the inner op's can be simplified
        #op(if(cond,arg1,arg1')) => if(cond,op(arg1),op(arg1'))
        #or op(if(cond,arg1,arg1'),if(cond,arg2,arg2')) => if(cond,op(arg1,arg2),op(arg1',arg2'))
        if len(newargs) > 0 and any(is_op(a,'if') for a in newargs):
            ifcond = None
            for a in newargs:
                if is_op(a,'if'):
                    ifcond = a.args[0]
                    break
            docollapse = True
            numifs = 0
            for i in range(1,len(newargs)):
                if is_op(newargs[i],'if'):
                    numifs += 1
                    if not ifcond.match(newargs[i].args[0]):
                        docollapse = False
                        break
            if _DEBUG_SIMPLIFY:
                print(" "*mydepth,"Considering if-folding of operator",self.functionInfo.name,"judgement:",docollapse)
            if docollapse:
                trueargs = []
                falseargs = []
                for a in newargs:
                    if is_op(a,'if'):
                        trueargs.append(a.args[1])
                        falseargs.append(a.args[2])
                    else:
                        trueargs.append(a)
                        falseargs.append(a)
                trueres = OperatorExpression(self.functionInfo,trueargs)
                trueres2 = trueres._postsimplify(depth,mydepth+1)
                if trueres2 is not None: trueres = trueres2
                if _DEBUG_SIMPLIFY:
                    if trueres2 is not None:
                        print(" "*mydepth,"True condition simplified to",trueres2)
                    else:
                        print(" "*mydepth,"True condition remains",trueres)
                falseres = OperatorExpression(self.functionInfo,falseargs)
                falseres2 = falseres._postsimplify(depth,mydepth+1)
                if falseres2 is not None: falseres = falseres2
                if _DEBUG_SIMPLIFY:
                    if falseres2 is not None:
                        print(" "*mydepth,"False condition simplified to",falseres2)
                    else:
                        print(" "*mydepth,"False condition remains",falseres)
                if numifs > 1 or len(newargs)==1 or trueres2 is not None or falseres2 is not None:
                    res = if_(ifcond,trueres,falseres)
                    self._cache['simplified'] = res
                    return res
                else:
                    if _DEBUG_SIMPLIFY:
                        print(" "*mydepth,"If-folding of operator",self.functionInfo.name,"ignored, no improvement in expression size")
                
        #expand Functions defined as expressions
        if isinstance(self.functionInfo.func,Expression):
            res = _subs(None,self.functionInfo.func,self.functionInfo.exprArgRefs,newargs,False,False,False)
            if isinstance(res,OperatorExpression):
                res2 = res._postsimplify(depth,mydepth+1)
                if res2 is not None:
                    res._clearCache('simplified',deep=True)
                    res = res2
            self._cache['simplified'] = res
            if _DEBUG_SIMPLIFY:
                print(" "*mydepth,"Simplied via Expression espansion to",res)
            return res
        if simplified:
            self._cache['simplified'] = OperatorExpression(self.functionInfo,newargs,self.op)
            if _DEBUG_SIMPLIFY:
                if len(self.args) > 1:
                    print(" "*mydepth,"Simplified one or more arguments of operation",self.functionInfo.name,"in context",self)
                else:
                    print(" "*mydepth,"Simplified argument of operation",self.functionInfo.name,"in context",self)
            #print "Simplified args result",self._cache
            return self._cache['simplified']
        self._cache['simplified'] = None
        return None
    def _constant_expansion(self,context,depth): 
        if 'simplified' in self._cache:
            return self._cache['simplified']
        if _is_exactly(depth,0): return None
        newdepth = None if depth is None else depth-1
        #constant replacement
        simplified = False
        aconst = []
        newargs = []
        #print "Simplifying operation",self.functionInfo.name,"in context",self
        for i,a in enumerate(self.args):
            a._parent = (weakref.ref(self),i)
            asimp = (a._constant_expansion(context,newdepth) if isinstance(a,OperatorExpression) else a._simplify(context))
            a._parent = None
            if asimp is None:
                newargs.append(a)
            else:
                simplified = True
                newargs.append(asimp)
            if isinstance(newargs[-1],UserDataExpression):
                aconst.append(newargs[-1].eval(context))
                if aconst[-1] is newargs[-1]:
                    aconst[-1] = None
            else:
                aconst.append(to_const(newargs[-1]))
        if not any(a is None for a in aconst):
            #print "Constant expansion of",self
            #print "Arguments",[str(a) for a in newargs]
            #print "Evaluated arguments",[str(a) for a in aconst]
            #constant
            assert not any(isinstance(a,Expression) for a in aconst),"Hmm... we should have gotten rid of any Expressions"
            try:
                return ConstantExpression(self._do(aconst))
            except Exception as e:
                print("Error simplifying function",self.functionInfo.name)
                print("  Call stack:", end=' ')
                self._print_call_stack()
                print("Exception:",e)
                raise 
                return None
        if self.functionInfo.properties.get('foldable',False):
            commutative = self.functionInfo.properties.get('commutative',False)
            if commutative:
                avar = []
                #check constants that can be folded in
                for ac,v in zip(aconst,newargs):
                    if ac is None:
                        avar.append(v)
                aconst = [ac for ac in aconst if ac is not None and not _is_exactly(ac,0)]
                if len(aconst) > 1:
                    const = self.op(*aconst)
                    if 'foldfunc' in self.functionInfo.properties:
                        const,stop = self.functionInfo.properties['foldfunc'](const)
                        if stop:
                            return ConstantExpression(const)
                    if const is not None:
                        newargs = avar + [ConstantExpression(const)]
                    simplified = True
            else:
                #fold streaks of constant arguments together
                newargs2 = []
                istart = None
                for i in range(len(aconst)):
                    if aconst[i] is None:
                        #non-constant
                        if istart is not None:
                            if i-istart > 1:
                                #fold istart...i-1 into a constant
                                const = self.op(*[aconst[j] for j in range(istart,i)])
                                newargs2.append(ConstantExpression(const))
                            else:
                                newargs2.append(newargs[istart])
                        istart = None
                        newargs2.append(newargs[i])
                    else:
                        if istart is None:
                            istart = i
                if istart is not None:
                    i = len(aconst)
                    if i-istart > 1:
                        #fold istart...i-1 into a constant
                        const = self.op(*[aconst[j] for j in range(istart,i)])
                        newargs2.append(ConstantExpression(const))
                    else:
                        newargs2.append(newargs[istart])
                if len(newargs2) < len(newargs):
                    #print "Simplified",[str(e) for e in newargs],"to",[str(e) for e in newargs2],"by non-commutative folding"
                    #raw_input()
                    newargs = newargs2
        if simplified:
            if _DEBUG_SIMPLIFY:
                print(" "*depth,"Constant expansion of",self)
                print(" "*depth,"   with arguments",[str(a) for a in newargs])
                #print "Evaluated arguments",[str(a) for a in aconst]
            if self.functionInfo.simplifier != None:
                for i,a in enumerate(newargs):
                    if not isinstance(a,Expression):
                        if a is not 0:
                            warnings.warn("symbolic.simplify: function %s shouldn't simplify to constant: %s"%(self.functionInfo.name,a))
                        newargs[i] = expr(a)
                try:
                    res = self.functionInfo.simplifier(*newargs)
                    if res is not None:
                        self._cache['simplified'] = res
                        if _DEBUG_SIMPLIFY:
                            print("=>",res)
                        if isinstance(res,ConstantExpression):
                            assert res.value is not None
                        return res
                    else:
                        pass
                        #print "Unable to simplify",self
                        #print "Call stack",self._print_call_stack()
                except Exception as e:
                    #print "Error simplifying function",self.functionInfo.name
                    #print "  Call stack:",
                    #self._print_call_stack()
                    #print "Exception:",e
                    raise
                    return None
            if _DEBUG_SIMPLIFY:
                print(" "*depth,"=>",OperatorExpression(self.functionInfo,newargs,self.op))
            self._cache['simplified'] = OperatorExpression(self.functionInfo,newargs,self.op)
            return self._cache['simplified']
        return None

    def _simplify(self,context=None,depth=None,constant_expansion=True):
        """Returns a new expression, or None if this cannot be simplified.

        Simplification has three phases:

        - pre-simplification (top-down): each expression examines its immediate children for
          possible symbolic simplification.  Then, it pre-simplifies its descendents.
        - post-simplification (bottom-up): each expression post-simplifies its descendants, then
          examines its new children for possible symbolic simplification.
        - constant expansion (bottom-up): any descendants are replaced with constants, and if any
          immediate children are changed, symbolic simplification is tried again.

        """
        if _DEBUG_SIMPLIFY:
            print("Simplifying expression",self,"to depth",depth)
        simplest = self
        res = self._presimplify(depth)
        if res is not None:
            if not isinstance(res,OperatorExpression):
                if not isinstance(res,Expression): return ConstantExpression(res)
                return res
            simplest = res
        res = simplest._postsimplify(depth)
        if _DEBUG_SIMPLIFY:
            print("After pre/postsimplify:",res)
        simplest._clearCache('simplified',deep=True)
        if res is not None:
            if not isinstance(res,OperatorExpression): 
                if not isinstance(res,Expression): return ConstantExpression(res)
                return res
            simplest = res
        if constant_expansion:
            res = simplest._constant_expansion(context,depth)
            if _DEBUG_SIMPLIFY: print("After constant expansion:",res)
            simplest._clearCache('simplified',deep=True)
            if res is not None:
                simplest = res
        if simplest is self:
            if _DEBUG_SIMPLIFY: print("... no simplification possible")
            return None
        if _DEBUG_SIMPLIFY: print("... simplified to",simplest)
        if not isinstance(simplest,Expression): return ConstantExpression(simplest)
        return simplest

    def _signature(self):
        asig = [a._signature() for a in self.args]
        return (self.functionInfo.name,sum(asig,()))

def supertype(types):
    assert len(types) > 0
    tres = Type(types[0])
    for t in types[1:]:
        if tres.match(t): continue
        if tres.is_scalar() and t.is_scalar():
            #promote to N
            tres.char = 'N'
        elif not tres.is_scalar() and not t.is_scalar():
            if tres.char == t.char:
                if tres.char == 'L':
                    if tres.size != t.size:
                        tres.size = None
                    if tres.subtype is not None:
                        tres.subtype = supertype([tres.subtype,t.subtype])
                else:
                    if tres.size != t.size:
                        tres.size = None
                    if tres.subtype is not None:
                        tres.subtype = supertype([tres.subtype,t.subtype])
            else:
                #mixed types
                tres.char = 'L'
                tres.subtype = None
        else:
            #mix of scalars and array types
            return Type(None)
    return tres


def type_of(x):
    if isinstance(x,Type): return x
    elif isinstance(x,Variable): return x.type
    elif isinstance(x,Expression): return x.returnType()
    else:
        if isinstance(x,bool): return Type('B')
        elif isinstance(x,_PY_INT_TYPES): return Type('I')
        elif isinstance(x,_PY_FLOAT_TYPES): return Type('N')
        elif isinstance(x,slice): return Type('X')
        elif isinstance(x,np.ndarray): 
            if len(x.shape) == 1:
                return Type('V',x.shape[0])
            elif len(x.shape) == 2:
                return Type('M',x.shape)
            else:
                return Type('A',x.shape)
        elif hasattr(x,'__iter__'): 
            #determine whether to turn this to a vector, matrix, array, or list
            if len(x) == 0:
                return Type('V',0)
            itemtype = supertype([type_of(v) for v in x])
            if itemtype.is_scalar():
                return Type('V',len(x))
            elif itemtype.char is None:
                return Type('L',len(x))
            else:
                if itemtype.char in 'AVM' and itemtype.size is not None:
                    if itemtype.char == 'V':
                        return Type('M',(len(x),itemtype.size))
                    else:
                        return Type('A',(len(x),) + itemtype.size)
                else:
                    return Type('L',len(x),itemtype)
        else:
            #print("object ",x,"is not of known type, setting as user data of type",x.__class__.__name__)
            return Type(x.__class__.__name__)

def const(v):
    return ConstantExpression(v)

def is_const(v,context=None,shallow=False):
    """Returns True if v is a constant value or constant expression.

    If context is not None, then Variables and user data that are bound to values are considered
    to be constants. Otherwise, they are not considered to be constants.

    If shallow=True, this doesn't try to convert Expressions.
    """
    if isinstance(v,Variable):
        if v.value is not None:
            return True
        return False
    elif isinstance(v,Expression):
        if shallow and isinstance(v,OperatorExpression) and not is_zero(v): return False
        if context is None: return v.isConstant()
        else: return v.isConstant() or v.returnConstant(context)
    elif isinstance(v,_PY_CONST_TYPES):
        return True
    elif isinstance(v,dict):
        return len(v) == 0 or all(is_const(x,context) for x in v.values())
    elif hasattr(v,'__iter__'):
        try:
            return len(v) == 0 or all(is_const(x,context) for x in v)
        except TypeError:
            raise ValueError("Object of type %s has iter but not len"%(v.__class__.__name__))
            return False
    else:
        return True

def to_const(v,context=None,shallow=False):
    """Returns the value corresponding to a constant value or expression v. Otherwise returns None.

    If context is not None, then Variables and user data that are bound to values are considered
    to be constants. Otherwise, they are not considered to be constants.

    If shallow=True, this doesn't try to convert Expressions.
    """
    if isinstance(v,Variable):
        if v.value is not None:
            return v.value
        return None
    elif isinstance(v,ConstantExpression):
        return v.value
    elif isinstance(v,Expression):
        if shallow and isinstance(v,OperatorExpression) and not is_zero(v): return None
        if context is None:
            if v.isConstant():
                return v.evalf()
        else:
            if v.isConstant() or v.returnConstant(context):
                return v.evalf(context)
        return None
    elif isinstance(v,_PY_CONST_TYPES):
        return v
    elif isinstance(v,dict):
        vc = {}
        for k,x in v.items():
            kc = to_const(x,context,shallow)
            if kc is None: return None
            vc[k] = kc
        return vc
    elif hasattr(v,'__iter__'):
        if len(v) == 0: return v
        vc = v.__class__(to_const(x,context,shallow) for x in v)
        if all(x is not None for x in vc):
            return vc
        return None
    else:
        return v

def is_scalar(v,value=None):
    """Returns True if v evaluates to a scalar.  If value is provided, then
    returns True only if v evaluates to be equal to value"""
    if isinstance(v,Variable):
        if not v.type.is_scalar(): return False
        return value is None or v.value == value
    elif isinstance(v,ConstantExpression):
        return is_scalar(v.value,value)
    elif isinstance(v,Expression):
        rt = v.returnType()
        if rt is None: return False
        if not rt.is_scalar(): return False
        return value is None
    else:
        if not (isinstance(v,(bool,)+_PY_NUMERIC_TYPES) or (hasattr(v,'shape') and v.shape == ())): return False
        return value is None or v == value

def to_scalar(v):
    """Returns the value of v if v is a scalar"""
    if isinstance(v,Variable):
        if not v.type.is_scalar(): return None
        return None
    elif isinstance(v,ConstantExpression):
        return to_scalar(v.value)
    elif isinstance(v,Expression):
        return None
    else:
        if not (isinstance(v,(bool,)+_PY_NUMERIC_TYPES) or (hasattr(v,'shape') and v.shape == ())): return None
        return v

def is_zero(v):   
    """Returns true if v represents a 0 value"""
    if is_op(v,'zero'): return True
    if isinstance(v,Expression): return False
    vc = to_const(v)
    if vc is not None:
        if isinstance(vc,np.ndarray):
            return np.all(vc==0)
        else:
            return vc == 0
    return False

def is_var(v):
    """Returns True if v is equivalent to a stand-alone variable."""
    return isinstance(v,Variable) or isinstance(v,VariableExpression)

def to_var(v):
    """If v is equivalent to a stand-alone variable, returns the Variable. Otherwise returns None."""
    if isinstance(v,Variable):
        return v
    elif isinstance(v,VariableExpression):
        return v.var
    return None

def is_sparse(v,threshold='auto'):
    """Returns true if v is a sparse array, with #nonzeros(v) < threshold(shape(v)).  

    Args: 
        v (Expression): the array
        threshold (optional): either 'auto', a constant, or a function of a
            shape.  If threshold=auto, the threshold is sqrt(product(shape))
    """
    if isinstance(v,ConstantExpression):
        v = v.value
    if isinstance(v,Expression): 
        raise ValueError("is_sparse can only be run on constants")
    sh = _shape(v)
    nnz = np.count_nonzero(v)
    if callable(threshold):
        threshold = threshold(sh)
    elif threshold == 'auto':
        threshold = 3*math.sqrt(np.product(sh))
    return (nnz < threshold)

def expr(x):
    """Converts x to an appropriate Expression as follows:

    - If x is a Variable, returns a VariableExpression.
    - If x is a non-str constant, returns a ConstantExpression.
    - If x is a str, then a UserDataExpression is returned. 
    - If x is already an Expression, x is returned.
    - If x is a list containing expressions, then ``array(*x)`` is returned. 
        This adds some ambiguity to lists containing strings, like
        ``expr(['x','y'])``.  In this case, strings are recursively converted
        to UserDataExpressions.

    """
    if isinstance(x,Variable):
        return VariableExpression(x)
    elif isinstance(x,str):
        return UserDataExpression(x)
    elif isinstance(x,Expression):
        return x
    elif isinstance(x,(np.ndarray,dict)):
        return ConstantExpression(x)
    elif hasattr(x,'__iter__'):
        try:
            xc = x.__class__(to_const(v) if not isinstance(v,str) else None for v in x)
            if all(v is not None for v in xc):
                return ConstantExpression(xc)
        except Exception as e:
            print("symbolic.expr: Error doing conversion to list of constants?",x.__class__.__name__,x)
            print("=== EXCEPTION ===")
            print(e)
            print("=== END EXCEPTION ===")
            raise
            input()
        return array(*[expr(v) for v in x])
    return ConstantExpression(x)

def is_expr(x):
    return isinstance(x,Expression)

def is_op(x,opname=None):
    if not isinstance(x,OperatorExpression): return False
    if opname is not None: return (x.functionInfo.name == opname)
    return True

def deriv(e,var):
    return expr(e).deriv(var)

def simplify(e,context=None,depth=None):
    return expr(e).simplify(context,depth)

def to_monomial(v):
    """If v is a monomial of a single variable, returns (x,d) where x is the Variable and d is the degree.
    Otherwise returns None."""
    if isinstance(v,Variable):
        return (v,1)
    elif isinstance(v,Expression):
        if isinstance(v,VariableExpression):
            return (v.var,1)
        if isinstance(v,OperatorExpression):
            if v.functionInfo.name == 'pow':
                res = to_monomial(v.args[0])
                if res is None: return None
                return (res[0],mul.optimized(v.args[1],res[1]))
            elif v.functionInfo.name == 'mul':
                if len(v.args) != 2:
                    #print("NOT MONOMIAL, ",str(v),"HAS",len(v.args),"ARGS")
                    return None
                assert len(v.args) == 2
                res1 = to_monomial(v.args[0])
                res2 = to_monomial(v.args[1])
                if res1 is None or res2 is None:
                    return None
                if res1[0].name != res2[0].name:
                    return None
                return (res1[0],add.optimized(res1[1],res2[1]))
    return None

def to_polynomial(expr,x=None,const_only=True):
    """If expr is a polynomial of a single variable, returns x,[c1,...,cn],[d1,...,dn] where
    expr = c1 x^d1 + ... + cn x^dn. Otherwise returns None.

    If x is given, the polynomial must be a polynomial in x.  If const_only=True, the coefficients
    and degrees must be constants, 
    """
    if not const_only:
        raise NotImplementedError("TODO: extract polynomial with non-constant coefficients")
    econst = to_const(expr)
    if x is not None and econst is not None:
        return x,[econst],[0]
    mon = to_monomial(expr)
    if mon != None:
        if x is None or mon[0].name == x.name:
            return (mon[0],[1],[mon[1]])
        else:
            return None
    uvars = expr.vars()
    if len(uvars) == 1:
        if x is None or uvars[0].name == x.name:
            #must be a combination of +,-,*, and pow
            if is_op(expr) and expr.functionInfo.name in ['add','sum','sub','neg','mul','pow']:
                x = uvars[0]
                pargs = []
                for a in expr.args:
                    pa = to_polynomial(a,x)
                    if pa is None: return None
                    pargs.append(pa)
                if expr.functionInfo.name in ['neg']:
                    return (x,[-c for c in pargs[0][1]],pargs[0][2])
                elif expr.functionInfo.name in ['add','sum','sub']:
                    if expr.functionInfo.name == 'sub':
                        pargs[1] = (pargs[1][0],[-c for c in pargs[1][1]],pargs[1][2])
                    dtoc = {}
                    for (x,cs,ds) in pargs:
                        for c,d in zip(cs,ds):
                            if d in dtoc:
                                dtoc[d] += c
                            else:
                                dtoc[d] = c
                    ds = sorted(dtoc.keys())
                    cs = [dtoc[d] for d in ds]
                    return (x,cs,ds)
                elif expr.functionInfo.name == 'mul':
                    (x,cas,das) = pargs[0]
                    (x,cbs,dbs) = pargs[1]
                    dtoc = {}
                    for (ca,da) in zip(cas,das):
                        for (cb,db) in zip(cbs,dbs):
                            d = da + db
                            if d in dtoc:
                                dtoc[d] += cb*ca
                            else:
                                dtoc[d] = cb*ca
                    ds = sorted(dtoc.keys())
                    cs = [dtoc[d] for d in ds]
                    return (x,cs,ds)
                else:
                    #TODO: raise powers with binomial 
                    return None
    return None



def _dims(v):
    if hasattr(v,'shape'): return len(v.shape)
    if hasattr(v,'__len__'):
        return 1
    raise ValueError("Computing dims of some unknown thing? "+v.__class__.__name__)
    return 0

def _len(v):
    assert not isinstance(v,Variable),"Did you mean to use len_?"
    if hasattr(v,'__len__'): return len(v)
    return 0

def _count(v):
    assert not isinstance(v,Variable),"Did you mean to use count?"
    return _hyper_count(v)

def _shape(v):
    assert not isinstance(v,Variable),"Did you mean to use shape?"
    if hasattr(v,'shape'): return v.shape
    if hasattr(v,'__len__'):
        return _hyper_shape(v)
    return ()

def _basis(i,n):
    assert n > 0,"basis must be given a positive integer for its second argument"
    assert i < n,"basis index must be less than the size"
    res = np.zeros(n)
    res[i] = 1.0
    return res

def _eye(n):
    assert not isinstance(n,Variable),"Did you mean to use eye?"
    if n == 0: return 1
    return np.eye(int(n))

def _zero(n):
    assert not isinstance(n,Variable),"Did you mean to use zero?"
    if isinstance(n,dict):
        #hyper array
        res = [None]*len(n)
        for (k,v) in n.items():
            res[k] = _zero(v)
        return res
    if hasattr(n,'__iter__'):
        if len(n) == 0: return 0
        if isinstance(n,np.ndarray):
            n = n.astype(np.int64)
        else:
            n = tuple(int(v) for v in n)
    else:
        if n == 0: return 0
        n = int(n)
    return np.zeros(n)

def _reshape(x,s):
    """Reshapes x to the shape s"""
    if x is None: return None
    if isinstance(s,np.ndarray):
        s = tuple(s.astype(np.int64).tolist())
    elif isinstance(s,list):
        s = tuple(s)
    if _shape(x) == s:
        return x
    assert s != ()
    if _is_exactly(_shape(x),()):
        #scalar, just return ones
        return np.ones(s)*x
    if not _is_numpy_shape(s):
        return _unravel(_ravel(x),s)
    try:
        return np.array(x).reshape(s)
    except Exception:
        flattened = _ravel(x)
        return np.array(flattened).reshape(s)

def _transpose(v):
    if not hasattr(v,'__iter__'): return v
    if hasattr(v,'shape') and len(v.shape) >= 3:
        warnings.warn("symbolic.transpose: being applied to tensor? You probably want transpose2")
    return np.transpose(v)

def _diag(v):
    if not hasattr(v,'__iter__'): return v
    return np.diag(v)

def _if(cond,trueval,falseval):
    return trueval if cond else falseval

def _max(*args):
    assert len(args) > 0,"Cannot perform max with 0 arguments"
    if len(args) == 1: return args[0]
    return max(args)

def _min(*args):
    assert len(args) > 0,"Cannot perform min with 0 arguments"
    if len(args) == 1: return args[0]
    return min(args)

def _argmax(*args):
    assert len(args) > 0,"Cannot perform argmax with 0 arguments"
    if len(args) == 1: return 0
    return max((v,i) for i,v in enumerate(args))[1]

def _argmin(*args):
    assert len(args) > 0,"Cannot perform argmin with 0 arguments"
    if len(args) == 1: return 0
    return min((v,i) for i,v in enumerate(args))[1]

def _add(*args):
    """Generalized add"""
    if len(args) == 0: return 0
    try:
        return np.sum(args,axis=0)
    except ValueError:
        pass
    res = args[0]
    for v in args[1:]:
        if isinstance(res,np.ndarray):
            res += v
        elif hasattr(res,'__iter__'):
            if isinstance(v,np.ndarray):
                res = np.array(res)+v
            else:
                if hasattr(v,'__iter__'):
                    assert len(v) == len(res)
                    res = vectorops.add(res,v)
                else:
                    res = vectorops.add(res,[v]*len(res))
        else:
            if isinstance(v,np.ndarray):
                res = v + res
            elif hasattr(v,'__iter__'):
                res = vectorops.add([res]*len(v),v)
            else:
                res += v
    return res    

def _sub(a1,a2):
    """Generalized subtraction"""
    return np.subtract(a1,a2)

def _mul(*args):
    """Generalized product"""
    assert len(args) > 0,"mul needs at least one argument"
    if len(args) == 2:
        return np.multiply(args[0],args[1])
    elif len(args) > 2:
        return np.multiply(args[0],_mul(*args[1:]))
    else:
        return args[0]
    

def _div(a1,a2):
    """Generalized product"""
    return np.divide(a1,a2)

def _pow(x,y):
    return np.power(x,y)

def _weightedsum(*arglist):
    if len(arglist) == 0: return 0.0
    vals = arglist[:len(arglist)/2]
    weights = arglist[len(arglist)/2:]
    res = _mul(vals[0],weights[0])
    for (v,w) in zip(vals,weights)[1:]:
        res += _mul(v,w)
    return res

def _getitem(vector,index):
    if isinstance(index,slice):
        start,stop,step =index.start,index.stop,index.step
        if isinstance(start,ConstantExpression):
            start = start.value
        if isinstance(stop,ConstantExpression):
            stop = stop.value
        if isinstance(step,ConstantExpression):
            step = step.value
        index = slice(start,stop,step)
    if hasattr(index,'__iter__'):
        return [vector[i] for i in index]
    else:
        return vector[index]

def _lazy_getitem(context,vector,index):
    #lazy evaluation of list entries
    eindex = index.eval(context)
    if isinstance(eindex,slice):
        start,stop,step = eindex.start,eindex.stop,eindex.step
        if isinstance(start,ConstantExpression):
            start = start.value
        if isinstance(stop,ConstantExpression):
            stop = stop.value
        if isinstance(step,ConstantExpression):
            step = step.value
        eindex = slice(start,stop,step)
    if is_op(vector,'array'):
        #special lazy evaluation for array
        if is_const(eindex):
            if hasattr(eindex,'__iter__'):
                return array([vector.args[i] for i in eindex]).eval(context)
            else:
                return vector.args[eindex]._eval(context)
        return vector._eval(context)[eindex]
    else:
        evector = vector._eval(context)
        if is_const(eindex):
            if is_const(evector):
                evector = to_const(evector)
                if isinstance(eindex,(list,tuple,np.ndarray)):
                    if isinstance(evector,np.ndarray):
                        return evector[eindex]
                    elif isinstance(evector,(list,tuple)):
                        return np.array(evector)[eindex]
            if isinstance(eindex,(list,np.ndarray)):
                #fancy indexing
                if isinstance(evector,Expression):
                    return evector[eindex]
                    #return flatten(*[vector[i] for i in eindex]).eval(context)
                return [evector[i] for i in eindex]
            return evector[eindex]
        warnings.warn("Non-constant getitem? {} class {}".format(eindex,eindex.__class__.__name__))
        return evector[eindex]

def _elementary_basis(n,index):
    #print "n=",n,"index=",index
    d = np.zeros(n)
    d[to_const(index)] = 1
    return d

def _array(*args):
    if len(args)==0: return []
    try:
        arr = np.array(args)
        if arr.dtype.char == 'O':  #couldn't make a proper array
            return list(args)
        return arr
    except Exception:
        return list(args)

def _array_returnType(*args):
    if len(args) == 0:
        return Type('L',0)
    atypes = [type_of(a) for a in args]
    atype = supertype(atypes)
    if atype.is_scalar():
        res = Type('V',len(args))
        res.subtype = atype
        return res
    if atype.char is not None and atype.char in 'VMA' and atype.size is not None:
        res = Type('A',(len(args),)+atype.shape())
        res.subtype = atype.subtype
        return res
    #default
    #print("Could not deduce return type of array of arg types",[str(t) for t in atypes])
    #if len(atypes) >= 2:
    #    for i in range(1,len(atypes)):
    #        print("match",i,"?",atypes[0].match(atypes[1]))
    return Type('L',len(args),atypes)

def _array_simplifier(*args):
    #reassemble
    if len(args) > 0 and all(is_op(a,'getitem') for a in args):
        v = args[0].args[0]
        i0 = to_const(args[0].args[1])
        if i0 is None or not isinstance(i0,_PY_INT_TYPES):
            return None
        for (i,a) in enumerate(args):
            if not is_scalar(a.args[1],i+i0):
                return None
        lv = to_const(len_.optimized(v))
        if lv is not None and i0 == 0 and lv == len(args):
            return v
        else:
            assert lv is None or i0+len(args) <= lv
            return v[i0:i0+len(args)]
    if len(args) > 0 and all(is_op(a,'zero') for a in args):
        ash = [shape.optimized(a) for a in args]
        sconst = to_const(ash[0])
        if sconst:
            convertable = all(to_const(a) == sconst for a in ash[1:])
            if convertable:
                return zero([len(args),] + list(sconst))

def _list_simplifier(*args):
    return _array_simplifier(*args)

def _tuple_simplifier(*args):
    return tuple_(_array_simplifier(*args))

def _list_returnType(*args):
    atypes = [type_of(a) for a in args]
    atype = supertype(atypes)
    return Type('L',len(args),atypes)

def _flatten(*args):
    if len(args)==0: return []
    return _ravel(args)

def _row_stack(*args):
    if len(args)==0: return []
    if len(args)==1 and hasattr(args[0],'__iter__'):
        return np.row_stack(args[0])
    return np.row_stack(args)

def _column_stack(*args):
    if len(args)==0: return []
    if len(args)==1 and hasattr(args[0],'__iter__'):
        return np.column_stack(args[0])
    return np.column_stack(args)

def _setitem(x,indices,rhs):
    if isinstance(indices,(list,tuple)):
        xcopy = np.array(x)
    elif isinstance(x,(list,tuple)):
        xcopy = list(x)
    else:
        xcopy = np.array(x)
    xcopy[indices] = rhs
    return xcopy

def _getattr(object,attr):
    assert isinstance(attr,str)
    res = getattr(object,attr)
    if callable(res):
        return res()
    return res

def _setattr(object,attr,val):
    assert isinstance(attr,str) or isinstance(attr,_PY_INT_TYPES)
    if callable(getattr(object,attr)):
        getattr(object,attr)(val)
    else:
        setattr(object,attr,val)
    return object

def _subs(context,expr,var,value,evalexpr=True,evalvar=True,evalvalue=True):
    if not isinstance(expr,Expression):
        return expr
    if is_op(var,'array') or is_op(var,'list'):
        var = var.args
    if isinstance(var,ConstantExpression):
        #may be an empty list
        var = var.value
    if hasattr(var,'__iter__'):
        if isinstance(value,ConstantExpression):
            value = value.value
            evalvalue = False
        assert hasattr(value,'__iter__') or is_op(value,'array'),"Variable list given, but value is not a list, %s has type %s"%(str(value),value.__class__.__name__)
        if is_op(value,'array') or is_op(value,'list'):
            value = value.args
        assert len(var) == len(value)
        #if evalexpr: expr = expr._eval(context)
        if evalvar: var = [v._eval(context) for v in var]
        if evalvalue: value = [v._eval(context) for v in value]
        if all(is_const(x,context) for x in value):
            if isinstance(context,Context):
                context = context.userData
            elif context is None:
                context = {}
            dummy = _setattr
            #this is a bit more efficient than expr.replace(var,value)
            oldvals = []
            for v,x in zip(var,value):
                if not isinstance(v,(VariableExpression,UserDataExpression)):
                    raise ValueError("var argument must be a list of variables or userDatas, got an instance of "+v.__class__.__name__)
                if isinstance(v,VariableExpression):
                    oldvals.append(v.var.value)
                    v.var.value = x
                elif isinstance(v,UserDataExpression):
                    oldval = context.get(v.name,dummy) if context is not None else None
                    context[v.name] = x
                    oldvals.append(oldval)
            res = expr._eval(context)
            expr._clearCache('eval')
            #restore previous values in context
            for v,x in zip(var,oldvals):
                if isinstance(v,VariableExpression):
                    v.var.value = x
                else:
                    if x is dummy:
                        del context[v.name]
                    else:
                        context[v.name] = x
            return res
        else:
            #slow version: replacing variables with expressions
            for v,x in zip(var,value):
                expr = expr.replace(v,x,error_no_match=False)
            return expr
    #if evalexpr: expr = expr._eval(context)
    #print("original var argument:",var,var.__class__.__name__)
    if evalvar: var = var._eval(context)
    if evalvalue: value = value._eval(context)
    if not isinstance(var,(VariableExpression,UserDataExpression)):
        #print("var argument:",var)
        raise ValueError("var argument must be a variable or a userData, got an instance of "+var.__class__.__name__)
    if is_const(value,context):
        if isinstance(var,VariableExpression):
            #this is a bit more efficient than expr.replace(var,value)
            oldval = var.var.value
            var.var.value = to_const(value,context)
            assert var.var.value is not None
            res = expr._eval(context)
            expr._clearCache('eval')
            var.var.value = oldval
            return res
        if isinstance(var,UserDataExpression):
            #this is a bit more efficient than expr.replace(var,value)
            if isinstance(context,Context):
                context = context.userData
            elif context is None:
                context = {}
            dummy = _setattr
            oldval = context.get(var.name,dummy) if context is not None else None
            context[var.name] = value
            res = expr.eval(context)
            expr._clearCache('eval')
            if oldval is dummy:
                del context[var.name]
            else:
                context[var.name] = oldval
            return res
    return expr.replace(var,value,error_no_match=False)

def _map(context,expr,var,values):
    if isinstance(values,Expression):
        values = values._eval(context)
    var = var._eval(context)
    #expr = expr._eval(context)
    if not isinstance(var,(VariableExpression,UserDataExpression)):
        raise ValueError("var argument must be a variable or a userData")
    if is_op(values,'array'):
        values = values.args  
    if is_op(values):
        expr = expr.eval(context)
        return map_(expr,var,values)
    resargs = [_subs(context,expr,var,value,evalexpr=False,evalvar=False,evalvalue=False) for value in values]
    if all(is_const(e,context) for e in resargs):
        return array(*resargs).evalf()
    else:
        return array(*resargs)

def _lazy_all(context,*args):
    for i,a in enumerate(args):
        a = a._eval(context,clearcache=False)
        ac = to_const(a)
        if ac is not None and not ac:
            for j in range(i+1):
                args[j]._clearCache('eval')
            return False
    for j in range(len(args)):
        args[j]._clearCache('eval')
    return True

def _lazy_any(context,*args):
    for i,a in enumerate(args):
        a = a._eval(context,clearcache=False)
        ac = to_const(a)
        if ac is not None and ac:
            for j in range(i+1):
                args[j]._clearCache('eval')
            return True
    for j in range(len(args)):
        args[j]._clearCache('eval')
    return False

def _forall(context,expr,var,values):
    if isinstance(values,Expression):
        values = values._eval(context)
    var = var._eval(context)
    #expr = expr._eval(context)
    if not isinstance(var,(VariableExpression,UserDataExpression)):
        raise ValueError("var argument must be a variable or a userData")
    if is_op(values,'array'):
        values = values.args
    if is_op(values):
        return forall(expr,var,values)
    anyexpr = False
    resargs = []
    for value in values:
        e = _subs(context,expr,var,value,evalexpr=False,evalvar=False,evalvalue=False)
        ec = to_const(e)
        if ec is not None:
            if not ec:
                return ConstantExpression(False)
        else:
            anyexpr = True
        resargs.append(e)
    if anyexpr:
        return all_(*resargs)
    return ConstantExpression(True)
        
def _forsome(context,expr,var,values):
    if isinstance(values,Expression):
        values = values._eval(context)
    var = var._eval(context)
    #expr = expr._eval(context)
    if not isinstance(var,(VariableExpression,UserDataExpression)):
        raise ValueError("var argument must be a variable or a userData")
    if is_op(values,'array'):
        values = values.args
    if is_op(values):
        return forsome(expr,var,values)
    anyexpr = False
    resargs = []
    for value in values:
        e = _subs(context,expr,var,value,evalexpr=False,evalvar=False,evalvalue=False)
        ec = to_const(e)
        if ec is not None:
            if ec:
                return ConstantExpression(True)
        else:
            anyexpr = True
        resargs.append(e)
    if anyexpr:
        return all_(*resargs)
    return ConstantExpression(False)

def _summation(context,expr,var,values):
    if isinstance(values,Expression):
        values = values._eval(context)
    var = var._eval(context)
    #expr = expr._eval(context)
    if not isinstance(var,(VariableExpression,UserDataExpression)):
        raise ValueError("var argument must be a variable or a userData")
    if is_op(values,'array'):
        values = values.args
    if is_op(values):
        expr = expr.eval(context)
        return summation(expr,var,values)
    anyexpr = False
    resargs = []
    for value in values:
        e = _subs(context,expr,var,value,evalexpr=False,evalvar=False,evalvalue=False)
        resargs.append(e)
        if not is_const(e,context):
            anyexpr = True
    if anyexpr:
        return sum_(*resargs)
    return sum_(*resargs).eval()

def _propagate_returnType(*args):
    return supertype([type_of(a) for a in args])

def _promote_returnType(*args):
    if len(args) == 0: return Type('N')
    types = [type_of(a) for a in args]
    assert len(types) > 0
    tres = Type(types[0])
    for t in types[1:]:
        if tres.match(t): continue
        if tres.is_scalar():
            if t.is_scalar():
                #promote to N
                tres.char = 'N'
            else:
                tres = t
        else:
            if t.is_scalar():
                pass
            else:
                raise ValueError("Incompatible types %s"%(','.join(str(t) for t in types),))
    return tres

def _if_returnType(cond,trueval,falseval):
    return supertype([type_of(trueval),type_of(falseval)])

def _returnType1(*args):
    return args[0].returnType()

def _returnType2(*args):
    return args[1].returnType()

def _returnType3(*args):
    return args[2].returnType()

#all of these are symbolic Functions in the default namespace
range_ = Function('range',lambda x:np.array(list(range(x))),['n'],returnType='V')
len_ = Function('len',_len,['x'],returnType='I')
count = Function('count',_count,['x'],returnType='I')
shape = Function('shape',_shape,['x'],returnType='V')
reshape = Function('reshape',_reshape,['x','s'])
transpose = Function('transpose',_transpose,['x'])
transpose2 = Function('transpose2',lambda x,axes:np.transpose(x,axes),['x','axes'])
dims = Function('dims',_dims,['x'],returnType='I')
eye = Function('eye',_eye,['n'],returnType='M')
basis = Function('basis',_basis,['i','n'],returnType='V')
zero = Function('zero',_zero,['n'],returnType='A')
diag = Function('diag',_diag,['x'],returnType='M')
eq = Function('eq',operator.eq,['lhs','rhs'],returnType='B')
ne = Function('ne',operator.ne,['lhs','rhs'],returnType='B')
le = Function('le',operator.le,['lhs','rhs'],returnType='B')
ge = Function('ge',operator.ge,['lhs','rhs'],returnType='B')
not_ = Function('not',operator.not_,['x'],returnType='B')
or_ = Function('or',operator.or_,['x','y'],returnType='B')
and_ = Function('and',operator.and_,['x','y'],returnType='B')
neg = Function('neg',operator.neg,['x'],returnType=_propagate_returnType)
abs_ = Function('abs',np.abs,['x'],returnType=_propagate_returnType)
sign = Function('sign',np.sign,['x'],returnType=_propagate_returnType)
add = Function('add',_add,['x','y'],returnType=_promote_returnType)
sub = Function('sub',_sub,['x','y'],returnType=_promote_returnType)
mul = Function('mul',_mul,'...',returnType=_promote_returnType)
div = Function('div',_div,['x','y'],returnType=_promote_returnType)
pow_ = Function('pow',_pow,['x','y'],returnType=_promote_returnType)
dot = Function('dot',np.dot,['x','y'],'A')
outer = Function('outer',np.outer,['x','y'],'A')
tensordot = Function('tensordot',np.tensordot,['x','y','axes'],'A')
if_ = Function('if',_if,['cond','trueval','falseval'],returnType=_if_returnType)
if_.returnTypeDescription = "either the type of trueval or falseval"
max_ = Function('max',_max,'...',returnType=_propagate_returnType)
min_ = Function('min',_min,'...',returnType=_propagate_returnType)
argmax = Function('argmax',_argmax,'...',returnType='I')
argmin = Function('argmin',_argmin,'...',returnType='I')
cos = Function('cos',np.cos,['x'],returnType=_propagate_returnType)
sin = Function('sin',np.sin,['x'],returnType=_propagate_returnType)
tan = Function('tan',np.tan,['x'],returnType=_propagate_returnType)
arccos = Function('arccos',np.arccos,['x'],returnType=_propagate_returnType)
arcsin = Function('arcsin',np.arcsin,['x'],returnType=_propagate_returnType)
arctan = Function('arctan',np.arctan,['x'],returnType=_propagate_returnType)
arctan2 = Function('arctan2',np.arctan2,['x'],returnType=_propagate_returnType)
sqrt = Function('sqrt',np.sqrt,['x'],returnType=_propagate_returnType)
exp = Function('exp',np.exp,['x'],returnType=_propagate_returnType)
log = Function('log',np.log,['x'],returnType=_propagate_returnType)
ln = Function('ln',np.log,['x'],returnType=_propagate_returnType)
sum_ = Function('sum',_add,'...',returnType=_promote_returnType)
any_ = Function('any',any,'...',returnType='B')
all_ = Function('all',all,'...',returnType='B')
getitem = Function('getitem',_getitem,['vec','index'])
setitem = Function('setitem',_setitem,['vec','index','val'])
#Deprecated in Python 3...
#getslice = Function('getslice',operator.getslice,'...',returnType='L')
getattr_ = Function('getattr',_getattr)
setattr_ = Function('setattr',_setattr)
flatten = Function('flatten',_flatten,'...',returnType='V')
row_stack = Function('row_stack',_row_stack,'...',returnType='M')
column_stack = Function('column_stack',_column_stack,'...',returnType='M')
array = Function('array',_array,'...',returnType='A')
list_ = Function('list',lambda *args:args,'...',returnType='L')
tuple_ = Function('tuple',lambda *args:tuple(args),'...',returnType='L')
zip_ = Function('zip',lambda *args:list(zip(*args)),'...',returnType='L')
weightedsum = Function('weightedsum',_weightedsum,'...',returnType=_propagate_returnType)
subs = Function('subs',_subs,['expr','var','value'],returnType=_returnType1)
map_ = Function('map',_map,['expr','var','values'],returnType='L')
forall = Function('forall',_forall,['expr','var','values'],returnType='B')
forsome = Function('forsome',_forsome,['expr','var','values'],returnType='B')
summation = Function('summation',_summation,['expr','var','values'])
getitem.custom_eval = _lazy_getitem
any_.custom_eval = _lazy_any
all_.custom_eval = _lazy_all
subs.custom_eval = _subs
map_.custom_eval = _map
forall.custom_eval = _forall
forsome.custom_eval = _forsome
summation.custom_eval = _summation
range_.description = """Equivalent to Python's range(n) function."""
dims.description = """Returns the dimensionality of the input.  Equivalent to len(shape(x))."""
len_.description = """Equivalent to Python's len(x) function, except if x is a scalar then it evaluates to 0. 

If x is a multi-dimensional array, this is the length of its first dimension.  Undefined for other
forms of user data."""
count.description = """Evaluates the number of numeric parameters in x. Works with scalars, arrays, and lists.

If x is a scalar then it evaluates to 1.  Lists are evaluated recursively.  Undefined for other forms of
user data."""
shape.description = """Evaluates to x.shape if x is a Numpy array, (len_(x),) if x is a vector, and () if x is
a scalar.  If x is a list, this is (len_(x),)+shape(item) if all of the items have the same shape,
otherwise it is a hyper-shape. (shortcut: "x.shape")"""
reshape.description = """Evaluates to x reshaped to the shape s.  If x is a scalar, this evaluates to a constant
matrix."""
transpose.description = """Evaluates to np.transpose(x).  (shortcut: "x.T")"""
transpose2.description = """Evaluates to np.transpose(x,axes). """
basis.description = """Evaluates to the i'th elementary basis vector in dimension n."""
eye.description = """Evaluates to np.eye(n) if n > 0, otherwise returns 1."""
zero.description = """Evaluates to np.zeros(s) if s is a matrix shape or scalar > 0, otherwise evaluates to 0."""
diag.description = """Evaluates to np.diag(x)."""
flatten.description = """Evaluates to a vector where all arguments are stacked (concatenated) into a single
vector. Arrays are reduced to vectors by Numpy's flatten(), and complex objects are reduced via
recursive flattening."""
row_stack.description = """Evaluates to a matrix where all arguments are stacked vertically.  1D arrays are
treated as row vectors.  If only a single list element is provided, then all arguments are stacked."""
column_stack.description = """Evaluates to a matrix where all arguments are stacked horizontally.  1D arrays are
treated as column vectors. If only a single list element is provided, then all arguments are stacked."""

eq.description = """Evaluates to lhs = rhs (shortcut: "lhs = rhs")."""
ne.description = """Evaluates to lhs != rhs (shortcut: "lhs != rhs")."""
le.description = """Evaluates to lhs <= rhs (shortcut: "lhs <= rhs")."""
ge.description = """Evaluates to lhs >= rhs (shortcut: "lhs >= rhs")."""
not_.description = """Evaluates to not x (shortcut: "not x")."""
or_.description = """Evaluates to x or y (shortcut: "x or y")."""
and_.description = """Evaluates to x and y (shortcut: "x and y")."""
any_.description = """Evaluates to any(*args)"""
all_.description = """Evaluates to all(*args)"""
neg.description = """Evaluates to -x (shorcut "-x")."""
abs_.description = """Evaluates to abs(x) (shortcut: "abs(x)"). Works with arrays too (elementwise)"""
sign.description = """Evaluates the sign of x.  Works with arrays too (elementwise)."""
add.description = """Evaluates to x + y (shortcut: "x + y").  Works with arrays too."""
sub.description = """Evaluates to x y (shortcut: "x - y").  Works with arrays too, and vector - scalar."""
mul.description = """Evaluates to x * y (shortcut: "x * y").  Works with arrays too (elementwise multiplication)."""
div.description = """Evaluates to x / y (shortcut: "x / y").  Works with arrays too (elementwise division), and
vector / scalar."""
pow_.description = """Evaluates to pow(x,y) (shortcut "x**y")."""
dot.description = """Evaluates to np.dot(x,y)."""
outer.description = """Evaluates to np.outer(x,y)."""
tensordot.description = """Evaluates to np.tensordot(x,y,axes)."""
max_.description = """Evaluates to the maximum of the arguments."""
min_.description = """Evaluates to the minimum of the arguments."""
argmax.description = """Evaluates to the index of the maximum of the argments."""
argmin.description = """Evaluates to the index of the minimum of the argments."""
cos.description = """Evaluates to np.cos(x)."""
sin.description = """Evaluates to np.sin(x)."""
tan.description = """Evaluates to np.tan(x)."""
arccos.description = """Evaluates to np.arccos(x)."""
arcsin.description = """Evaluates to np.arcsin(x)."""
arctan.description = """Evaluates to np.arctan(x)."""
arctan2.description = """Evaluates to np.arctan2(x)."""
sqrt.description = """Evaluates to math.sqrt(x)."""
exp.description = """Evaluates to math.exp(x)."""
log.description = """Evaluates to math.log(x) (base 10)."""
ln.description = """Evaluates to math.ln(x) (natural log)."""
sum_.description = """Evaluates to sum(args).  If arguments are vectors or matrices, then the result is also a vector
or matrix. This is somewhat different behavior from sum(x) if x is a list."""
weightedsum.description = """Evaluates to w1*v1+...+wn*vn. """
getitem.description = """Evaluates to vec[index].  This also supports slices and tuples, as well as lists (Numpy fancy
indexing). (shortcut: "vec[index]")"""
setitem.description = """Evaluates to vec except with vec[index] set to val.  Equivalent to Python code "temp = vec[:];
vec[index]=val; return temp" """
getattr_.description = """returns the value of a given attribute under the given object. For example,
getattr_(traj,const("milestones")) gets the milestone list of a user-data Trajectory named traj.

If the result is a function (e.g., a getter method), it will be called with no arguments.  For example,
getattr_(robot,const("getJointLimits")) will return the robot's joint limits.  (shortcut:
"object.attr", where object is a UserDataExpression)"""
setattr_.description = """returns a modified version of the given class object, where value is assigned to the
attribute attr. For example, setattr_(traj,const("times"),[0,0.5,0.1]) sets the times attribute of a
Trajectory to [0,0.5,1].  Note: this operation modifies the object itself and returns it. 

If the attribute is a function (e.g., a setter method), it will be called with the argument val."""
if_.description = """If cond evaluates to True, this expression evaluates to trueval. Otherwise, it evaluates to
falseval."""
array.description = """Creates a list or numpy array of the given arguments. This can accept arbitrary arguments,
such as variable size vectors.  A Numpy array is produced only if the items have compatible types
and dimensions."""
list_.description = """Creates a list of the given arguments. This can accept arbitrary arguments, such as variable
size vectors.  No attempt is made to convert to a numpy array."""
tuple_.description = """Creates a tuple of the given arguments. This can accept arbitrary arguments, such as variable
size vectors."""
zip_.description = """Equivalent to the Python zip function, returning a list of tuples."""
subs.description = """evaluates expr with var substituted with value.  For example, subs(const(2)*"i","i",3.5)
yields 2*3.5"""
map_.description = """like the Python map function, evaluates to a list where each entry evaluates expr with var
substituted with a value from the list values.  For example, if x is a Variable, map_(x**"i","i",range(3))
yields the list [x**0, x**1, x**2]"""
forall.description = """True if, for every value in the list values, expr evaluates to nonzero when var is substituted
with that value. Equivalent to all_(*[subs(expr,var,value) for value in values])"""
forsome.description = """True if, for some value in the list values, expr evaluates to nonzero when var is substituted
with that value. Equivalent to any_(*[subs(expr,var,value) for value in values])"""
summation.description = """The sum of expr over var when var is substitued with each value in the list values.
Equivalent to sum_(*[subs(expr,var,value) for value in values])"""

_builtin_functions = {'dims':dims,'len':len_,'count':count,'shape':shape,'reshape':reshape,'transpose':transpose,'transpose2':transpose2,
                    'range':range_,
                    'eye':eye,'basis':basis,'zero':zero,'diag':diag,
                    'eq':eq,'ne':ne,'ge':ge,'le':le,
                    'not':not_,'or':or_,'and':and_,
                    'neg':neg,'abs':abs_,'sign':sign,
                    'add':add,'sub':sub,'mul':mul,'div':div,
                    'pow':pow_,
                    'dot':dot,
                    'if':if_,
                    'max':max_,'min':min_,
                    'argmax':argmax,
                    'argmin':argmin,
                    'cos':cos,'sin':sin,'tan':tan,'sqrt':sqrt,
                    'arccos':arccos,'arcsin':arcsin,'arctan':arctan,'arctan2':arctan2,
                    'sum':sum_,
                    'any':any_,'all':all_,
                    'getitem':getitem, 'setitem':setitem,
                    #Deprecated in Python 3
                    #'getslice':getslice,
                    'getattr':getattr_,'setattr':setattr_,
                    'flatten':flatten,'row_stack':row_stack,'column_stack':column_stack,
                    'array':array,'list':list_,'tuple':tuple_,'zip':zip_,
                    'weightedsum':weightedsum,
                    'subs':subs,'map':map_,'forall':forall,'forsome':forsome,'summation':summation
                    }
_infix_operators = {'and':' and ','or':' or ','sub':'-','div':'/','pow':'**','ge':'>=','le':'<=','eq':'==','ne':'!='}
_prefix_operators = {'not':'not ','neg':'-'}

_python_naming_changes = {'and':'and_',
                            'or':'or_',
                            'not':'not_',
                            'len':'len_',
                            'abs':'abs_',
                            'if':'if_',
                            'pow':'pow_',
                            'sum':'sum_',
                            'any':'any_',
                            'max':'max_',
                            'min':'min_',
                            'all':'all_',
                            'list':'list_',
                            'tuple':'tuple_',
                            'zip':'zip_'
                            }

#define simplifiers and derivatives

def _dims_simplifier(x):
    try:
        res = x.returnType().dims()
        if res is not None: return res
    except Exception as e:
        pass
    if isinstance(x,Variable):
        try:
            res = x.type.dims()
            if res is not None: return res
        except Exception as e:
            pass
    if is_op(x):
        if is_op(x,'getitem'):
            assert NotImplementedError("Shouldn't get here")
            if isinstance(to_const(x.args[1]),(list,tuple,slice)):
                return dims(x.args[0])
            else:
                return dims(x.args[0])-1
        if is_op(x,'setitem'):
            assert NotImplementedError("Shouldn't get here")
            return dims(x.args[0])
        if is_op(x,'reshape'):
            if is_op(x.args[1],'shape'):
                return dims(x.args[1].args[0])
            return len_(x.args[1])
    elif is_const(x):
        if isinstance(x,np.ndarray):
            return len(x.shape)
        try:
            return type_of(x).dims()
        except Exception:
            #no way to determine dimensions of unknown type
            return None
    return None
def _len_simplifier(x):
    if is_scalar(x): return 0
    if isinstance(x,OperatorExpression): 
        try:
            res = x.returnType().len()
            if res is not None: return res
        except Exception:
            pass
        if is_op(x,'zero'):
            if hasattr(x.args[0],'__iter__'):
                return x.args[0][0]
            return x.args[0]
        if is_op(x,'basis'):
            return x.args[1]
        if is_op(x,'eye'):
            return x.args[0]
        if is_op(x,'diag'):
            return len_(x.args[0])
        if is_op(x,'getitem'):
            if isinstance(to_const(x.args[1]),(list,tuple)):
                return (len(x.args[1]),)
        if is_op(x,'if'):
            cond,trueval,falseval = x.args
            return if_(cond,len_(trueval),len_(falseval))
        if is_op(x,'flatten'):
            return sum_(*[count._call(a) for a in x.args])
        if is_op(x,'shape'):
            rt = x.args[0].returnType()
            if rt is None: return None
            if rt.char == 'A': return None if rt.size is None else len(rt.size)
            elif rt.char == 'V': return 1
            elif rt.char == 'M': return 2
            elif rt.char == 'L': return rt.size
            else: return 0
    elif isinstance(x,VariableExpression):
        if x.var.type.size is not None:
            return x.var.type.len()
    elif isinstance(x,Variable):
        try:
            return x.type.len()
        except Exception as e:
            pass
    elif hasattr(x,'__len__'):
        return len(x)
    #print "Can't yet simplify len(",str(x),")","return type",x.returnType()
    return None
def _count_simplifier(x):
    if is_scalar(x): return 1
    if is_op(x):
        try:
            c = x.returnType().count()
            if c is not None:
                return c
        except Exception as e:
            pass
    elif isinstance(x,VariableExpression):
        if x.var.type.size is not None:
            return x.var.type.count()
    elif isinstance(x,Variable):
        if x.type.size is not None:
            return x.type.count()
    if is_op(x,'shape'):
        return dims(x.args[0])
    return None
def _shape_simplifier(x):
    if is_scalar(x): return ()
    try:
        res = x.returnType().shape()
        if res is not None:
            return res
    except Exception as e:
        pass
    if isinstance(x,VariableExpression):
        if x.var.type.size is not None:
            return x.var.type.shape()
    elif isinstance(x,Variable):
        if x.type.size is not None:
            return x.type.shape()
    #TEST: new simplifier
    """
    if is_op(x):
        if is_op(x,'zero'):
            return x.args[0]
        if is_op(x,'basis'):
            return x.args[1]
        if is_op(x,'eye'):
            return flatten(x.args[0],x.args[0])
        if is_op(x,'diag'):
            return flatten(len_(x.args[0]),len_(x.args[0]))
        if is_op(x,'dot'):
            return dotshape(x.args[0],x.args[1])
        if is_op(x,'getitem') and isinstance(to_const(x.args[1]),(list,tuple)):
            return (len_(x.args[1]),)
        if is_op(x,'setitem'):
            return shape(x.args[0])
        if is_op(x,'reshape'):
            return x.args[1]
        if is_op(x,'flatten'):
            counts = [count(a) for a in x.args]
            return array(sum(counts))
        #print "Can't yet simplify shape(",str(x),")","return type",x.returnType()
    """
    return None
def _eye_returnType(N):
    Nc = to_const(N)
    if Nc is not None:
        return Type('M',(Nc,Nc))
    return Type('M')
def _basis_returnType(i,n):
    Nc = to_const(n)
    if Nc is not None:
        return Type('V',Nc)
    return Type('V')
def _zero_returnType(sh):
    shc = to_const(sh)
    if shc is not None:
        if isinstance(shc,int):
            if shc == 0:
                return Type('N')
            else:
                return Type('V',shc)
        if len(shc) == 0:
            return Type('N')
        elif len(shc) == 1:
            return Type('V',shc[0])
        elif len(shc) == 2:
            return Type('M',shc)
        else:
            return Type('A',shc)
    try:
        d = sh.returnType().dims()
        if d <= 1:
            return Type('V')
        elif d == 2:
            return Type('M')
        else:
            return Type('A')
    except Exception:
        return Type('A')
def _reshape_returnType(x,sh):
    shc = to_const(sh)
    if shc is not None:
        return Type('A',shc)
    l = simplify(len_(sh))
    lc = to_const(l)
    if lc is not None:
        if lc == 0:
            return Type('N')
        elif lc == 1:
            return Type('V')
    return Type('A')
    """
    #is there some way to getting the length of the shape?
    try:
        print("reshape returntype not const",sh)
        d = sh.returnType().dims()
        if d <= 1:
            return Type('V')
        else:
            #might have a hyper-shape?
            return Type('L')
    except Exception:
        return Type('A')
    """

def _reshape_presimplifier(x,sh):
    if is_op(x,'reshape'):
        return reshape(x.args[0],sh)
    if is_op(x,'flatten'):
        return reshape(x.args[0],sh)
    if is_op(x,'zero'):
        return zero(sh)
    try:
        xsh = x.returnType().shape()
    except Exception:
        return None
    if is_const(sh):
        shc = to_const(sh)
        if shc is not None:
            if np.array_equal(xsh,shc):
                return x
            if len(shc) == 1:
                return flatten(x)
            elif len(shc) == 0:
                assert xsh==(1,)
                return x[0]
        return None
    try:
        if sh.returnType().dims() == 1:
            return flatten(x)
    except Exception:
        pass

def _reshape_simplifier(x,sh):
    if is_op(x,'reshape'):
        return reshape(x.args[0],sh)
    if is_op(x,'zero'):
        return zero(sh)
    if is_op(x,'eye'):
        #print("RESHAPE EYE",x.args[0],"TO",sh)
        if to_const(len_.optimized(sh)) == 2:
            try:
                #print("It's square")
                if x.args[0].match(simplify(sh[0])) and x.args[0].match(simplify(sh[1])):
                    #print("It matches",simplify(sh[0]),"and",simplify(sh[1]))
                    return x
            except Exception:
                pass
    try:
        xsh = x.returnType().shape()
    except Exception:
        return None
    if is_const(sh):
        shc = to_const(sh)
        if shc is not None:
            if np.array_equal(xsh,shc):
                return x
            if len(shc) == 1:
                return flatten(x)
            if len(xsh) > 1 and xsh[0] == 1:
                return getitem.optimized(x,0)
            #try flattening and reshaping 
            size = np.prod(shc)
            if is_op(x,'array') or is_op(x,'list'):
                def to_nparray(v):
                    if is_op(v,'array') or is_op(v,'list'):
                        try:
                            return np.array([to_nparray(a) for a in v.args])
                        except ValueError:
                            raise
                    return v
                try:
                    xarr = to_nparray(x)
                    if np.prod(xarr.shape) == size:
                        #print("reshape: converted",x,"to array",xarr)
                        #print("Trying to flatten and reshape from",xsh,"to shape",shc)
                        res = xarr.reshape(shc)
                        def to_array(v):
                            if isinstance(v,np.ndarray) and len(v.shape) >= 1:
                                return array(*[to_array(w) for w in v])
                            return v
                        #print("Result as np array",res)
                        res_array = to_array(res)
                        #print("Result as symbolic array:",res_array)
                        return res_array
                except ValueError as e:
                    #import traceback
                    #print("Exception encountered:",e)
                    #traceback.print_exc()
                    #print("context is trying to convert",x,"into an numpy array")
                    pass
        return None
    try:
        if len(sh.returnType().shape()) == 1:
            return flatten(x)
    except Exception:
        pass

def _transpose_simplifier(x):
    d = to_const(dims.optimized(x))
    if d is not None and d <= 1:
        return x
    #handled by inverse rule
    #if is_op(x,'transpose'):
    #    return x.args[0]
    #TEST: new simplifiers
    """
    if is_op(x,'eye') or is_op(x,'basis'):
        return x
    if is_op(x,'zero'):
        s = to_const(x.args[0])
        if s is not None:
            s = list(s)
            if len(s) >= 2:
                s[0],s[-1] = s[-1],s[0]
            return zero(tuple(s))
    if is_op(x,'neg'):
        return -transpose(x.args[0])
    """
    return None

def _transpose_returnType(x):
    xt = x.returnType()
    if xt is None: return None
    elif xt.char is None: return None
    elif xt.is_scalar() or xt.char == 'V': return xt
    elif xt.char in 'MA':
        if xt.size is None: return xt
        s = list(xt.size)
        s[0],s[-1] = s[-1],s[0]
        return Type(xt,tuple(s))
    elif xt.char == 'L':
        if xt.subtype is not None:
            if isinstance(xt.subtype,Type) and xt.subtype.char in 'AMLV':
                return Type('A')
        raise ValueError("transpose: Can't do a transpose on a list object with subtype "+str(xt.subtype))
    return None

def _transpose2_simplifier(x,axes):
    da = to_const(len_.optimized(axes))
    if da is not None:
        if da <= 1:
            return x
        if da == 2:
            a0 = to_const(axes[0])
            if a0 == 0:
                return x
            elif a0 == 1:
                return transpose(x)
    return None

def _transpose2_returnType(x,axes):
    xt = x.returnType()
    if xt is None: return None
    elif xt.char is None: return None
    elif xt.size is None: return xt
    caxes = to_const(axes)
    if caxes is not None:
        if any(i < 0 or i >= len(xt.size) for i in caxes):
            raise ValueError("transpose2: invalid axis index")
        return Type(xt.char,tuple(xt.size[i] for i in caxes),xt.subtype)
    return Type(xt.char,None,xt.subtype)



"""
def _neg_simplifier(arg):
    #handled by inverse rule
    #if is_op(arg,'neg'):
    #    return arg.args[0]
    if is_op(arg,'zero'):
        return arg.args[0]
    return None
"""

def _sum_simplifier(*args):
    aconst = [to_const(a,shallow=True) for a in args]
    avar = []
    #check constants that can be folded in
    for ac,v in zip(aconst,args):
        if ac is None:
            if is_op(v,'zero'):
                continue
            avar.append(v)
    aconst = [ac for ac in aconst if ac is not None and not is_scalar(ac,0)]
    #print "Constants to sum",aconst
    sumconst = 0
    if len(aconst) == 1:
        sumconst = aconst[0]
        if isinstance(aconst[0],(list,tuple)):
            sumconst = np.array(aconst[0])
    elif len(aconst) > 1:
        sumconst = _add(*aconst)
    #print "Variable",avar
    #print "Sum of constants",sumconst
    changed = False
    if isinstance(sumconst,np.ndarray):
        if np.all(sumconst == 0):
            sumconst = 0
    elif hasattr(sumconst,'__iter__'):
        if all(v == 0 for v in sumconst):
            sumconst = 0
    else:
        if sumconst == 0:
            sumconst = 0
    if len(avar) == 0:
        return sumconst
    def match(a,b):
        if a is b:
            return True
        if isinstance(a,VariableExpression) and isinstance(b,VariableExpression):
            if a.var.name == b.var.name:
                return True
        if isinstance(a,OperatorExpression) and isinstance(b,OperatorExpression):
            if a.functionInfo.name == b.functionInfo.name:
                if len(a.args) == len(b.args):
                    for va,vb in zip(a.args,b.args):
                        if not match(va,vb): return False
                    return True
        return False
    #now check for duplicate arguments that can be converted to multiplies
    for i in range(len(avar)):
        if avar[i] is None: continue
        matches = [j for j in range(len(avar)) if j != i and match(avar[i],avar[j])]
        if len(matches) >= 1:
            for j in matches:
                avar[j] = None
            avar[i] = mul(1+len(matches),avar[i])
    avar = [v for v in avar if v is not None]
    if len(avar) > 1 and all(is_op(v,'array') for v in avar):
        #add the entries in the list
        match = True
        for v in avar[1:]:
            if len(v.args) != len(avar[0].args):
                match = False
                break
        if match:
            sargs = []
            for i in range(len(avar[0].args)):
                sargs.append(sum_(*[v.args[i] for v in avar]))
            return array(*sargs)
    if len(avar) > 1 and all(is_op(v,'flatten') for v in avar):
        #add the stacked vectors
        match = True
        for v in avar[1:]:
            if len(v.args) != len(avar[0].args):
                match = False
                break
            if not all(simplify(shape(a)).evalf() == simplify(shape(b)).evalf() for (a,b) in zip(avar[0].args,v.args)):
                match=False
                break
        if match:
            sargs = []
            for i in range(len(avar[0].args)):
                sargs.append(sum_(*[v.args[i] for v in avar]))
            return flatten(*sargs)
    if len(avar) > 1 and all(is_op(v,'reshape') for v in avar):
        #add the internal vectors
        sargs = [v.args[0] for v in avar]
        return reshape(sum_(*sargs),avar[0].args[1])
    start = 0
    if not _is_exactly(sumconst,0):
        start += 1
    if start+len(avar) < len(args):
        #simplified some constants
        changed = True
        if not _is_exactly(sumconst,0):
            args = [sumconst]+avar
        else:
            args = avar
    if len(args)==1:
        return args[0]
    if changed:
        return sum_(*args)
    return None

"""
def _sub_simplifier(x,y):
    if is_op(y,'neg'):
        return add(x,y.args[0])
    if is_zero(y):
        return x    
    if is_zero(x):
        return neg(y)
    return None
"""

def _mul_deriv(args,dargs):
    terms = []
    for i,dx in enumerate(dargs):
        if not is_zero(dx):
            dxargs = args[:i]+[dx]+args[i+1:]
            inc = mul.optimized(*dxargs)
            terms.append(inc)
    if len(terms) == 0:
        return 0
    return sum_.optimized(*terms)

def _mul_presimplifier(*args):
    if any(is_op(x,'basis') for x in args):
        #pull out the basis term
        #print("TRYING TO PULL OUT MUL BASIS TERM")
        entry = None
        basisTerm = None
        for x in args:
            if is_op(x,'basis'):
                if entry is not None:
                    if not entry.matches(x.args[0]):
                        #print("mul(basis(%s),basis(%s)) simplifies to zero?"%(entry,x.args[0]))
                        return zero(x.args[1])
                entry = x.args[0]
                basisTerm = x
        anyVec = False
        eargs = []
        for x in args:
            if not is_op(x,'basis'):
                if is_scalar(x):
                    eargs.append(x)
                else:
                    eargs.append(getitem._call(x,entry))
                    anyVec = True
        if anyVec:
            return mul(basisTerm,*eargs)
            

def _mul_simplifier(*args):
    if len(args) == 1:
        return args[0]
    
    aconst = [to_const(x) for x in args]
    avar = []
    for xc,x in zip(aconst,args):
        if xc is None:
            if is_op(x,'zero'):
                return zero(x.args[0])
            avar.append(x)
        elif is_scalar(xc,0):
            return 0
    aconst = [x for x in aconst if x is not None]
    if len(aconst) > 0:
        aconst = mul.optimized(*aconst)
    else:
        aconst = 1
    if any(is_op(x,'neg') for x in avar):
        #print("Pulling out negative")
        inner_args = [(x.args[0] if is_op(x,'neg') else x) for x in avar]
        #print("Inner args",inner_args)
        odd = sum(is_op(x,'neg') for x in avar)%2
        #print("Odd-ness",odd)
        if odd:
            if aconst == 1:
                return neg(mul.optimized(*inner_args))
            elif aconst == -1:
                return mul.optimized(*inner_args)
            return mul.optimized(*([neg.optimized(aconst)]+inner_args))
        else:
            if aconst == 1:
                return mul.optimized(*inner_args)
            elif aconst == -1:
                return neg(mul.optimized(*inner_args))
            return mul.optimized(*([aconst]+inner_args))

    if len(avar) == 0: return aconst
    if not is_scalar(aconst,1):
        newargs = [aconst]+avar
    else:
        newargs = avar
    if len(newargs) < len(args):
        if len(newargs) == 0:
            return 1
        elif len(newargs) == 1:
            return newargs[0]
        return mul(*newargs)

    #combine polynomial factors
    asort = sorted(args,key=lambda x: str(x))
    apoly = [to_monomial(x) for x in asort]
    newargs = [asort[0]]
    for i in range(len(apoly)-1):
        if apoly[i] and apoly[i+1]:
            base1,exp1 = apoly[i]
            base2,exp2 = apoly[i+1]
            if base1.name == base2.name:
                newargs[-1] = pow_(base1,add.optimized(exp1,exp2))
                apoly[i+1] = (base2,exp1+exp2)
            else:
                newargs.append(asort[i+1])
        else:
            newargs.append(asort[i+1])
    if len(newargs) < len(args):
        if len(newargs) == 0:
            return 1
        elif len(newargs) == 1:
            return newargs[0]
        return mul(*newargs)

    if len(args) == 2:
        x,y = args
        xconst = to_const(x)
        yconst = to_const(y)
        if isinstance(xconst,_PY_NUMERIC_TYPES):
            if xconst == -1: 
                return -y
            #if is_op(y,'mul'):
            #    #look for other constants
            #    y0const = to_const(y.args[0])
            #    if y0const is not None:
            #        return mul.optimized(_mul(xconst,y0const),y.args[1])
            if is_op(y,'div'):
                #look for other constants
                y0const = to_const(y.args[0])
                if y0const is not None:
                    return div(_div(xconst,y0const),y.args[1])
            if is_op(y,'neg'):
                return mul(_mul(xconst,-1),y.args[0])
        elif isinstance(yconst,_PY_NUMERIC_TYPES):
            #put constants in leading terms
            return mul(y,x)
        elif isinstance(xconst,np.ndarray):
            return reshape([const(v)*y for v in xconst],xconst.shape)
        elif isinstance(yconst,np.ndarray):
            return reshape([const(v)*x for v in yconst],yconst.shape)
    return None

def _div_simplifier(x,y):
    xconst = to_const(x)
    yconst = to_const(y)
    if isinstance(xconst,_PY_NUMERIC_TYPES):
        if xconst == 0: return 0
        if is_op(y,'mul'):
            #look for other constants
            y0const = to_const(y.args[0])
            if y0const is not None:
                return div.optimized(_div(xconst,y0const),y.args[1])
        if is_op(y,'div'):
            #look for other constants
            y0const = to_const(y.args[0])
            if y0const is not None:
                return mul.optimized(_div(xconst,y0const),y.args[1])
        if is_op(y,'neg'):
            return div.optimized(_mul(xconst,-1),y.args[0])
    elif isinstance(yconst,_PY_NUMERIC_TYPES):
        #put constants in leading terms
        return mul(1.0/y,x)
    elif is_op(x,'mul') and to_const(x.args[0]):
        #put constants in leading terms
        if len(x.args) > 2:
            return mul(x.args[0],div.optimized(mul.optimized(*x.args[1:]),y))
        return mul(x.args[0],div.optimized(x.args[1],y))
    elif is_op(x,'neg'):
        #take out negatives
        if is_op(y,'neg'):
            return div.optimized(x.args[0],y.args[0])
        else:
            return neg(div.optimized(x.args[0],y))
    elif is_op(y,'neg'):
        return neg(div.optimized(x,y.args[0]))
    else:
        xpoly = to_monomial(x)
        ypoly = to_monomial(y)
        if xpoly and ypoly:
            return pow_(xpoly[0],sub.optimized(xpoly[1],ypoly[1]))
    if is_op(y,'div'):
        return div(mul.optimized(x,y.args[1]),y.args[0])
    return None

def _pow_simplifier(x,y):
    xconst = to_scalar(x)
    yconst = to_scalar(y)
    if xconst is not None:
        if xconst == 0: return 0
        if xconst == 1: return 1
    if yconst is not None:
        if yconst == 0: return 1
        if yconst == 1: return x
        if isinstance(x,OperatorExpression) and x.functionInfo.name == 'neg':
            if yconst % 2 == 0:
                return pow_(x.arg[0],y)
            else:
                return -pow_(x.arg[0],y)
        xpoly = to_monomial(x)
        if xpoly and xpoly[1] != 1:
            return pow_(xpoly[0],xpoly[1]*yconst)
    return None

"""
def _abs_simplifier(x):
    if is_op(x,'neg'):
        return abs_(x.args[0])
    if is_op(x,'abs'):
        return x
    if is_op(x,'zero') or is_op(x,'eye') or is_op(x,'basis'):
        return x

def _sign_simplifier(x):
    if is_op(x,'neg'):
        return -sign(x.args[0])
    if is_op(x,'zero') or is_op(x,'eye') or is_op(x,'basis'):
        return x
"""

def dotshape(x,y):
    """A symbolic expression producing the shape of the dot product of expressions x and y"""
    sx = shape.optimized(x)
    sy = shape.optimized(y)
    lx = dims.optimized(x)
    ly = dims.optimized(y)
    #assert is_const(lx),"The dimension of expression %s cannot be determined"%(str(x))
    #assert is_const(ly),"The dimension of expression %s cannot be determined"%(str(y))
    if is_const(lx) and is_const(ly): 
        lx = to_const(lx)
        ly = to_const(ly)
        if lx > 0 and ly > 0:
            if to_const(sx) and to_const(sy):
                assert to_const(sx)[-1]==to_const(sy)[0],"Error forming dot product: dimensions don't match"
            return flatten.optimized(sx[0:lx-1],sy[1:ly])
        else:
            return flatten.optimized(sx,sy)
    else:
        res = if_._call(or_._call(lx == 0,ly == 0),
                flatten._call(sx,sy),           #scalar - array
                flatten._call(getitem._call(sx,range_._call(lx-1)),getitem._call(sy,1+range_._call(ly-1))))  #array - array
        return simplify(res)

def dottype(x,y):
    dx = dims.optimized(x)
    dy = dims.optimized(y)
    try:
        assert is_const(dx)
        assert is_const(dy)
        dx = to_const(dx)
        dy = to_const(dy)
        resdims = dx + dy - 2
        if resdims <= 0:
            basetype = 'N'
        elif resdims == 1:
            basetype = 'V'
        elif resdims == 2:
            basetype = 'M'
        else:
            basetype = 'A'
    except:
        basetype = 'A'
    s = to_const(dotshape(x,y))
    if s is None: return Type(basetype)
    if len(s) == 0:
        return Type('N')
    if basetype == 'V':
        s = s[0]
    return Type(basetype,s)

def _tensordot_shape(x,y,axes):
    try:
        xs = x.returnType().shape()
        ys = y.returnType().shape()
        ac = to_const(axes)
    except Exception:
        return None
    if ac is None:
        return None
    if isinstance(ac,int):
        return xs[:ac] + ys[ac:]
    else:
        assert hasattr(ac,'__iter__'),"tensordot axes must be an index or pair of indices"
        assert len(ac) == 2,"too many items in tensordot axes"
        assert hasattr(ac[0],'__iter__') and hasattr(ac[1],'__iter__'),"tensordot axes must be a pair of lists"
        xused = [False]*len(xs)
        yused = [False]*len(ys)
        for i in ac[0]:
            assert i >= 0 and i < len(xs),"Invalid index in tensordot"
            xused[i] = True
        for i in ac[1]:
            assert i >= 0 and i < len(ys),"Invalid index in tensordot"
            yused[i] = True
        return [s for s,u in zip(xs,xused) if not u] + [s for s,u in zip(ys,yused) if not u]

def _tensordot_returnType(x,y,axes):
    sh = _tensordot_shape(x,y,axes)
    if sh is None:
        return Type('A')
    return Type('A',sh)

def _tensordot_const_simplifier(x,y,axes):
    if axes == 1:
        return dot(x,y)
    try:
        xd = x.returnType().dims()
        yd = y.returnType().dims()    
        if xd == 1 and yd == 1:
            if axes == 0 or axes==[[],[]]:
                return outer(x,y)
            elif axes == 1 or axes == [[0],[0]]:
                return dot(x,y)
            else:
                raise ValueError("Invalid value for axes in tensordot of two vectors:",axes)
    except Exception:
        return None
    return None

def _dot_simplifier(x,y):
    try:
        xd = x.returnType().dims()
        yd = y.returnType().dims()
        if xd == 1 and yd == 2:
            if is_op(y,'transpose'):
                return dot(y.args[0],x)
    except Exception:
        pass

"""
def _dot_simplifier(x,y):
    if is_op(x,'eye'):
        return y
    elif is_op(y,'eye'):
        return x
    elif is_op(x,'basis'):
        return getitem._call(y,x.args[0])
    elif is_op(y,'basis'):
        return getitem._call(x,y.args[0])
    elif is_op(x,'zero'):
        return zero._call(dotshape(x,y))
    elif is_op(y,'zero'):
        return zero._call(dotshape(x,y))
    elif is_op(x,'dot') or is_op(y,'dot'):
        if is_op(x,'dot'):
            #consider (x0*x1)*y vs x0*(x1*y)
            a,b,c=x.args[0],x.args[1],y
        else:
            a,b,c=x,y.args[0],y.args[1]
        sa = shape.optimized(a)
        sb = shape.optimized(b)
        sc = shape.optimized(c)
        abfirst = None
        if is_const(a) and is_const(b) and not is_const(c):
            abfirst = True
        elif is_const(b) and is_const(c) and not is_const(a):
            abfirst = False
        elif is_const(sa) and is_const(sb) and is_const(sc):
            minlen = len(sa)
            if len(sa) < len(sb):
                if len(sc) < len(sa):
                    abfirst = False
                    minlen = len(sc)
                elif len(sa) < len(sc):
                    abfirst = True
            elif len(sb) < len(sa):
                minlen = len(sb)
                abfirst = False
                if len(sc) < len(sb):
                    abfirst = False
                    minlen = len(sc)
                elif len(sb) < len(sc):
                    #weird, M*V*M?
                    pass
            else:
                #len(sb) == len(sa) 
               if len(sc) < len(sb):
                    #M * M * V
                    minlen = len(sc)
                    abfirst = False
            if abfirst is None:
                if len(sc) == len(sa) and len(sa) < len(sb):
                    #V * M * V, complexity really doesn't matter
                    if sb[0] > sb[1]:
                        abfirst = True
                    else:
                        abfirst = False
                else:
                    #all the same length, still need to determine order
                    assert len(sa) == len(sb) == len(sc) == 2,"Hmm... can only simplify M*M*M dot products"
                    assert sa[1] == sb[0],"Invalid sizes for dot product"
                    assert sb[1] == sc[0],"Invalid sizes for dot product"
                    abcount = sa[0]*sb[1]
                    bccount = sb[0]*sc[1]
                    if abcount < bccount:
                        abfirst = True
                    elif bccount < abcount:
                        abfirst = False
        if abfirst is None:
            return None
        elif abfirst:
            if not is_op(x,'dot'):
                return dot._call(dot._call(a,b),c)
        else:
            if is_op(x,'dot'):
                return dot._call(a,dot._call(b,c))
    elif is_op(x,'setitem'):
        x0,indices,v = x.args
        if is_const(indices):
            iconst = to_const(indices)
            nonindices = []
            iset = set(iconst)
            lx = len_.optimized(x0)
            for i in range(lx):
                if i not in iset:
                    nonindices.append(i)
            #print "dot-setitem returning",dot(x0[nonindices],y[nonindices])+dot(v,y[iconst])
            return dot._call(x0[nonindices],y[nonindices])+dot._call(v,y[iconst])
    elif is_op(y,'setitem'):
        y0,indices,v = y.args
        if is_const(indices):
            iconst = to_const(indices)
            nonindices = []
            iset = set(iconst)
            ly = len_.optimized(y0)
            for i in range(ly):
                if i not in iset:
                    nonindices.append(i)
            #print "dot-setitem returning",dot(x[nonindices],y0[nonindices])+dot(x[iconst],v)
            return dot._call(getitem.optimized(x,nonindices),getitem.optimized(y0,nonindices))+dot._call(getitem.optimized(x,iconst),v)
    xconst = to_const(x)
    yconst = to_const(y)
    if isinstance(xconst,np.ndarray):
        if (xconst==0).all():
            return zero(dotshape(x,y))
        if is_sparse(xconst):
            #THIS CODE APPEARS BUGGY
            return None
            #print "SPARSIFYING DOT PRODUCT",np.count_nonzero(xconst),"NONZEROS"
            nzs = np.nonzero(xconst)
            if len(xconst.shape) == 1:
                #it's a sum-product
                args = [xconst[ind]*y[ind] for ind in nzs[0]]
                dp = sum_(args)
                return dp
            elif is_const(shape.optimized(y)):
                yshape = to_const(shape.optimized(y))
                if len(yshape) == 1:
                    yentries = [getitem._call(y,i) for i in range(yshape[0])]
                    newx = np.zeros(xconst.shape[:-1],dtype='O')
                    #print "dot",x,y
                    #print "Nonzeros:",nzs
                    #print "Result shape",newx.shape
                    for i,ind in enumerate(nzs[-1]):
                        xindex = tuple(nzdim[i] for nzdim in nzs)
                        dpindex = tuple(nzdim[i] for nzdim in nzs[:-1])
                        #print "SETTING RESULT INDEX",dpindex,"to",xconst[xindex]*y[ind]
                        newx[dpindex] = xconst[xindex]*yentries[ind]
                    #print "RESULT",array(*newx.tolist())
                    #raw_input()
                    return array(*newx.tolist())
                else:
                    yentries = [getitem._call(y,i) for i in range(yshape[0])]
                    newx = np.zeros(xconst.shape[:-1]+tuple(yshape[1:]),dtype='O')
                    #print "dot",x,y
                    #print "Nonzeros:",nzs
                    #print "xshape",xconst.shape,"yshape",yshape
                    #print "Result shape",newx.shape
                    for i,ind in enumerate(nzs[-1]):
                        xindex = tuple(nzdim[i] for nzdim in nzs)
                        dpindex = tuple(nzdim[i] for nzdim in nzs[:-1])
                        val = simplify(xconst[xindex]*yentries[ind])
                        #print "SETTING RESULT INDEX",dpindex,"to",val
                        for yind in itertools.product(*[range(d) for d in yshape[1:]]):
                            #print "Index",dpindex,yind
                            #print dpindex+yind,"in shape",newx.shape
                            newx[d+yind] = val[yind]
                    #print "RESULT",array(*newx.tolist())
                    #raw_input()
                    return array(*newx.tolist())
    elif is_scalar(xconst):
        return mul.optimized(xconst,y)
    if isinstance(yconst,np.ndarray):
        if (yconst==0).all():
            return zero(dotshape(x,y))
    elif is_scalar(yconst):
        return mul.optimized(x,yconst)
    return None
"""

def _max_deriv(args,dargs):
    return array(*dargs)[argmax(*args)]

def _min_deriv(args,dargs):
    return array(*dargs)[argmin(*args)]

"""
def _if_presimplifier(cond,trueval,falseval):
    condconst = to_const(cond)
    if condconst is True:
        return trueval
    if condconst is False:
        return falseval
    return None

def _if_simplifier(cond,trueval,falseval):
    condconst = to_const(cond)
    if condconst is True:
        return trueval
    if condconst is False:
        return falseval
    return None
"""

def _argmax_simplifier(*args):
    if len(args) == 2:
        return if_._call((args[0] >= args[1]),0,1)
    return None

def _argmin_simplifier(*args):
    if len(args) == 2:
        return if_._call((args[0] <= args[1]),0,1)
    return None


def _all_simplifier(*args):
    nonconst = []
    for a in args:
        aconst = to_const(a)
        if aconst is False:
            return False
        elif aconst is None:
            nonconst.append(a)
    if len(nonconst) == 0:
        return False
    if len(nonconst) < len(args):
        if len(nonconst) == 1:
            return nonconst[0]
        return all_(*nonconst)
    return None

def _any_simplifier(*args):
    nonconst = []
    for a in args:
        aconst = to_const(a)
        if aconst is True:
            return True
        elif aconst is None:
            nonconst.append(a)
    if len(nonconst) == 0:
        return False
    if len(nonconst) < len(args):
        if len(nonconst) == 1:
            return nonconst[0]
        return any_(*nonconst)
    return None

def _flatten_simplifier(*args):
    #sanity check
    for a in args:
        if isinstance(a,int):
            raise ValueError("Args need to be ConstantExpressions")
    if len(args)==1:
        if is_op(args[0],'array'):
            return flatten.optimized(*args[0].args)
        #elif is_const(args[0]):
        #    return _flatten(args[0])
        elif args[0].returnType().char == 'N':
            return array(args[0])
        elif args[0].returnType().char == 'V':
            return args[0]
    newargs = []
    changed = False
    for a in args:
        if is_op(a,'flatten'):
            newargs += a.args
            changed = True
        elif is_op(a,'array') or is_op(a,'list'):
            newargs += a.args
            changed = True
        elif is_op(a,'reshape'):
            #ignore shape
            newargs.append(a.args[0])
            changed = True
        else:
            cnt = count.optimized(a)
            if is_zero(cnt):
                #skip
                changed = True
            elif is_const(cnt,1) and to_const(len_.optimized(a)) == 1:
                newargs.append(a[0])
                changed = True
            else:
                newargs.append(a)
    if len(newargs) == 1:
        #if is_const(newargs[0]):
        #    return _flatten(newargs[0])
        #el
        if newargs[0].returnType().char == 'N':
            return array(newargs[0])
        elif newargs[0].returnType().char == 'V':
            return newargs[0]
    if all(is_scalar(dims.optimized(a),0) for a in newargs):
        #all scalars
        return array(*newargs)
    if changed:
        return flatten(*newargs)
    return None

def _flatten_returnType(*args):
    size = to_const(simplify(sum_._call(*[count._call(v) for v in args])))
    if size is not None: size = int(size)
    return Type('V',size)

def _flatten_deriv(args,dargs):
    if len(args)==0: return []
    if len(args)==1 and hasattr(args[0],'__iter__'):
        if not any(hasattr(v,'__iter__') for v in args[0]):
            return dargs[0]
        return _flatten_deriv(args[0],dargs[0])
    return flatten(*dargs)

def _weightedsum_deriv(args,dargs):
    vals = args[len(args)/2:]
    weights = args[:len(args)/2]
    dvals = dargs[len(args)/2:]
    dweights = dargs[:len(args)/2]
    newvals = []
    newweights = []
    for (v,w) in zip(vals,dweights):
        if w is not None:
            newvals.append(v)
            newweights.append(w)
    for (v,w) in zip(dvals,weights):
        if v is not None:
            newvals.append(v)
            newweights.append(w)
    return weightedsum.optimized(*(newvals+newweights))

def _weightedsum_simplifier(*args):
    if len(args)==2:
        return mul.optimized(args[0],args[1])
    if len(args)==4:
        return mul.optimized(args[0],args[2])+mul.optimized(args[1],args[3])
    vals = args[:len(args)/2]
    weights = args[len(args)/2:]
    newvals = []
    newweights = []
    notone = False
    for (v,w) in zip(vals,weights):
        wconst = to_const(w) 
        if wconst != 1:
            notone = True
        else:
            notone = True
        if is_zero(w):
            continue
        if is_zero(v):
            continue
        newvals.append(v)
        newweights.append(w)
    if not notone:
        return sum_(*newvals)
    if len(newvals) == len(vals):
        return None
    return weightedsum.optimized(*(newvals+newweights))

def _getitem_deriv(context,v,index,dvars,rows):
    eindex = index.eval(context)
    if is_op(v,'array') or is_op(v,'list'):
        iconst = to_const(eindex)
        if iconst is not None:
            if isinstance(iconst,(list,np.ndarray)):
                if is_op(v,'list'):
                    return list_(*[v.args[i]._deriv(dvars,context,rows) for i in iconst])
                else:
                    return array(*[v.args[i]._deriv(dvars,context,rows) for i in iconst])
            else:
                return v.args[iconst]._deriv(dvars,context,rows)
        res = v._deriv(dvars,context,rows)
        if _is_exactly(res,0): return 0
        if res is None: return None
        return res[eindex]
    else:
        res = v._deriv(dvars,context,rows)
        vtype = v.returnType()
        if vtype is not None and vtype.char not in SCALAR_TYPES + ARRAY_TYPES:
            #irregular type: need to extract out the flattened indices corresponding to the original eindex
            orig_indices = []
            vcount = 0
            if isinstance(eindex,int):
                if eindex == 0:
                    estart = 0
                    orig_indices = range_(count.optimized(v[eindex]))
                else:
                    estart = count.optimized(v[:eindex])
                    orig_indices = estart+range_(count.optimized(v[eindex]))
                #print("IRREGULAR INDEX",eindex,"CONVERTED TO",orig_indices)
                eindex = orig_indices
            elif isinstance(eindex,(list,tuple)):  #it's a range
                estart = 0
                ilast = 0
                for ei in eindex:
                    if ei > ilast:
                        estart += count.optimized(v[ilast:ei])
                    vicount = count.optimized(v[ei])
                    orig_indices.append(estart + range_(vicount))
                    estart += vicount
                    ilast = ei+1
                #print("IRREGULAR INDEX",eindex,"CONVERTED TO",orig_indices)
                eindex = flatten(*orig_indices)
            else:
                raise ValueError("symbolic.deriv: Don't know how to take derivative with respect to variable index yet")
        if is_op(res,'eye') and res.args[0].match(rows):
            return transpose.optimized(getitem.optimized(res,eindex))
        if _DEBUG_DERIVATIVES:
            print("GETITEM DERIV",index,"v=",v,"deriv",res)
        if _is_exactly(res,0): return 0
        if res is None: return None
        crows = to_const(rows)
        if crows is None or crows > 0:
            #the jacobian has shape rows * shape(v)
            #want to extract out entry [:,eindex,...]
            if crows is not None:
                return array(*[getitem.optimized(res,i)[eindex] for i in range(int(crows))])
            else:   
                #this returns a list...
                #print("Mapping derivative with",rows,"rows and indexed columns",eindex)
                return getitem(array.optimized(map_(getitem.optimized(res,"i")[eindex],"i",range_(rows))),0)
        else:
            return res[eindex]

"""
def _getitem_simplifier(v,indices):
    iconst = to_const(indices)
    if isinstance(iconst,slice):
        vlen = len_.optimized(v)
        vlen = to_const(vlen)
        if vlen is None:
            #can't simplify a slice of a variable-length object
            return None
        indices = iconst = range(iconst.start,min(vlen,iconst.stop if iconst.stop is not None else 1),(iconst.step if iconst.step is not None else 1))
    li = len_.optimized(indices)
    if not is_const(li):
        return None
    if iconst is not None and isinstance(iconst,(list,tuple)) and len(iconst)==0:
        return []
    if is_op(v,'eye'):
        if li == 0:
            return basis._call(indices,getitem.optimized(shape.optimized(v),0))
        elif iconst is None:
            return row_stack(*[basis._call(i,getitem.optimized(shape.optimized(v),0)) for i in iconst])
        return None
    if is_op(v,'zero'):
        if li == 0:
            return zero.optimized(getitem.optimized(shape.optimized(v),-1))
        else:
            return zero.optimized((getitem.optimized(shape.optimized(v),-1),li))
    if is_op(v,'array') and iconst is not None:
        if hasattr(iconst,'__iter__'):
            return array(*[v.args[i] for i in iconst])
        return v.args[iconst]
    if is_op(v,'add') or is_op(v,'sum'):
        return sum_(*[a[indices] for a in v.args])
    if is_op(v,'sub'):
        return sub(*[a[indices] for a in v.args])
    if is_op(indices,'if'):
        cond,trueval,falseval = indices.args
        return if_._call(cond,v[trueval],v[falseval])
    if iconst is not None and isinstance(iconst,_PY_INT_TYPES+(list,slice)) and is_op(v,'dot'):
        A,B = v.args
        return dot._call(A[iconst],B)
    if iconst is not None and isinstance(iconst,_PY_INT_TYPES) and is_op(v,'row_stack'):
        if li == 0:
            for a in v.args:
                sa = to_const(shape.optimized(a))
                if sa is None:
                    return None
                if len(sa) == 0:
                    if iconst == 0: return a
                    iconst -= 1
                else:
                    if iconst < sa[0]: return a[iconst]
                    iconst -= sa[0]
    if iconst is not None and isinstance(iconst,_PY_INT_TYPES) and is_op(v,'flatten'):
        if li == 0:
            for a in v.args:
                ca = to_const(count.optimized(a))
                if ca is None:
                    return None
                if ca == 1:
                    if iconst == 0: return a
                    iconst -= 1
                else:
                    if iconst < ca:
                        if a.returnType().dims() == 1:
                            return a[iconst]
                        else:
                            #TODO: figure this out for matrices
                            return None
                    iconst -= ca
    if iconst is not None and is_op(v,'setitem'):
        x,indices,rhs = v.args
        indices = to_const(indices)
        if indices is not None:
            if iconst == indices:
                return rhs
            if not hasattr(iconst,'__iter__'):
                if hasattr(indices,'__iter__'):
                    if iconst in indices:
                        return rhs[indices.index(iconst)]
                    return x[iconst]
                else:
                    return x[iconst]
            else:
                fromx = []
                resindices = []
                if hasattr(indices,'__iter__'):
                    for i in iconst:
                        if i in indices:
                            fromx.append(False)
                            resindices.append(indices.index(i))
                        else:
                            fromx.append(True)
                            resindices.append(i)
                    if all(fromx):
                        return x[resindices]
                    elif not any(fromx):
                        return rhs[resindices]
                    else:
                        return row_stack(*[x[i] if ifromx else rhs[i] for (i,ifromx) in zip(resindices,fromx)])
                else:
                    for i in iconst:
                        if i == indices:
                            fromx.append(False)
                            resindices.append(0)
                        else:
                            fromx.append(True)
                            resindices.append(i)
                    if all(fromx):
                        return x[resindices]
                    elif not any(fromx):
                        return rhs
                    else:
                        return row_stack(*[x[i] if ifromx else rhs for (i,ifromx) in zip(resindices,fromx)])
    return None
"""

def _getitem_returnType(x,indices):
    iconst = to_const(indices)
    xt = x.returnType()
    if xt is None or xt.char is None: return xt
    if iconst is not None:
        if isinstance(iconst,slice):
            start,stop,step = iconst.start,iconst.stop,iconst.step
            if start is None:
                start = 0
            if stop is None or stop > 90000000000:
                size = None
            else:
                if step is not None:
                    size =  (stop-start)//step
                else:
                    size = stop-start
            if isinstance(xt.subtype,list):
                return Type(xt.char,size,xt.subtype[iconst])
            return Type(xt.char,size,xt.itemtype())
        if hasattr(iconst,'__iter__'):
            if xt.char in 'MA':
                return Type(xt.char,(len(iconst),)+xt.size[1:],xt.itemtype())
            return Type(xt.char,len(iconst),xt.itemtype())
        else:
            return xt.itemtype(iconst)
    si = shape.optimized(indices) 
    if is_const(si):
        si = to_const(si)
        assert len(si)<=1,"Can't use a matrix as an index?"
        if len(si) == 1:
            return Type(xt.char,si[0],xt.itemtype())
        else:
            return xt.itemtype(iconst)
    return Type(xt,size=None)

def _setitem_simplifier(x,indices,rhs):
    li = count.optimized(indices)
    if is_zero(li):
        return x
    li = len_.optimized(indices)
    lx = len_.optimized(x)
    if to_const(lx) == to_const(li):
        return rhs
    return None

def _subs_deriv(context,expr,var,value,derivs,rows):
    #chain rule d/dx expr(x,value(x)) = d/dx expr + d/dvalue expr dvalue/dx
    if isinstance(context,Context):
        context = context.userData
    elif context is None:
        context = {}
    if hasattr(var,'__iter__'):
        assert hasattr(value,'__iter__')
        assert len(var) == len(value)
        vderivs = [val._deriv(derivs,context,rows) for val in value]
        varnames = [v.name if isinstance(v,(Variable,UserDataExpression)) else v for v in var]
        dxexpr = expr._deriv(derivs,context,rows)
        dvexprs = []
        #NEED TO DO A CACHE CLEAR SINCE THIS IS REENTRANT 
        for (varname,vderiv) in zip(varnames,vderivs):
             expr._clearCache('deriv',deep=True)
             dvexprs.append(expr._deriv({varname:vderiv},context=context,rows=rows))
        expr._clearCache('deriv',deep=True)
        if dxexpr is None or any(v is None for v in dvexprs):
            return None
        return subs(dxexpr+sum_(dvexprs),var,value)
    else:
        vderiv = value._deriv(derivs,context,rows)
        varname = var.name if isinstance(var,(Variable,UserDataExpression)) else var
        dxexpr = expr._deriv(derivs,context=context)
        #NEED TO DO A CACHE CLEAR SINCE THIS IS REENTRANT 
        expr._clearCache('deriv',deep=True)
        dvexpr = expr._deriv({varname:vderiv},context=context)
        expr._clearCache('deriv',deep=True)
        if dxexpr is None or dvexpr is None:
            return None
        return subs(dxexpr + dvexpr,var,value)

def _map_deriv(context,expr,var,values,derivs,rows):
    if isinstance(context,Context):
        context = context.userData
    elif context is None:
        context = {}
    varname = var.name if isinstance(var,(Variable,UserDataExpression)) else var
    vderivs = values._deriv(derivs,context=context,rows=rows)
    if vderivs is None:
        return None
    dxexpr = expr._deriv(derivs,context=context,rows=rows)
    if dxexpr is None:
        return None
    if is_zero(vderivs):
        return map_(dxexpr,var,values)
    dvname = '_d'+varname
    vdvname = '_xdx'+varname
    #TODO: determine whether the values are integers or arrays
    dvvar = Variable(dvname,values[0].returnType())
    vdv = UserDataExpression(vdvname)
    expr._clearCache('deriv',deep=True)
    dvexpr = expr._deriv({varname:dvvar},context=context,rows=rows)
    expr._clearCache('deriv',deep=True)
    if dvexpr is None:
        return None
    return map_(subs(dxexpr + dvexpr,[var,dvvar],[vdv[0],vdv[1]]),vdvname,zip_(values,vderivs))

def _map_returnType(expr,var,values):
    if hasattr(values,'__len__'):
        return Type('L',len(values),expr.returnType())
    else:
        return Type('L',None,expr.returnType())

def _summation_deriv(context,expr,var,values,derivs,rows):
    if isinstance(context,Context):
        context = context.userData
    elif context is None:
        context = {}
    varname = var.name if isinstance(var,(Variable,UserDataExpression)) else var
    vderivs = values._deriv(derivs,context=context,rows=rows)
    if vderivs is None:
        return None
    dxexpr = expr._deriv(derivs,context=context,rows=rows)
    if dxexpr is None:
        return None
    if is_zero(vderivs):
        return summation(dxexpr,var,values)
    dvname = '_d'+varname
    vdvname = '_xdx'+varname
    #TODO: determine whether the values are integers or arrays
    dvvar = Variable(dvname,values[0].returnType())
    vdv = UserDataExpression(vdvname)
    expr._clearCache('deriv',deep=True)
    dvexpr = expr._deriv({varname:dvvar},context=context)
    expr._clearCache('deriv',deep=True)
    if dvexpr is None:
        return None
    return summation(subs(dxexpr + dvexpr,[var,dvvar],[vdv[0],vdv[1]]),vdvname,zip_(values,vderivs))



#no derivative functions
for fn in ['and','or','not','dims','count','len','shape','range','eye','diag','ge','eq','le','ne','any','all']:
    _builtin_functions[fn].deriv = 0
dims.simplifier = _dims_simplifier
dims.presimplifier = _dims_simplifier
len_.presimplifier = _len_simplifier
len_.simplifier = _len_simplifier
count.presimplifier = _count_simplifier
count.simplifier = _count_simplifier
def _count_dot_simplifier(x):
    A,B = x.args
    Ad = dims.optimized(A)
    Bd = dims.optimized(B)
    if is_scalar(Ad,1) and is_scalar(Bd,1):
        return 1
    As = to_const(shape.optimized(A))
    Bs = to_const(shape.optimized(B))
    if As is not None and Bs is not None:
        return np.prod(tuple(As[:-1])+tuple(Bs[1:]))
    return None
count.addSimplifier(['dot'],_count_dot_simplifier)
shape.presimplifier = _shape_simplifier
shape.simplifier = _shape_simplifier
def _shape_from_returnType(rt):
    try:
        return rt.shape()
    except Exception:
        return None
shape.addSimplifier(['_returnType'],_shape_from_returnType)
shape.addSimplifier(['zero'],lambda x:x.args[0])
shape.addSimplifier(['basis'],lambda x:flatten(x.args[1]))
shape.addSimplifier(['eye'],lambda x:flatten(x.args[0],x.args[0]))
shape.addSimplifier(['diag'],lambda x:array(len_(x.args[0]),len_(x.args[0])))
shape.addSimplifier(['dot'],lambda x:dotshape(x.args[0],x.args[1]))
shape.addSimplifier(['outer'],lambda x:array(len_(x.args[0]),len_(x.args[1])))
shape.addSimplifier(['tensordot'],lambda x,axes:_tensordot_shape(x.args[0],x.args[1],x.args[2]))
def _shape_getitem(x):
    if isinstance(to_const(x.args[1]),(list,tuple)):
        return (len_(x.args[1]),)
    return None
shape.addSimplifier(['getitem'],_shape_getitem)
shape.addSimplifier(['setitem'],lambda x:x.args[0])
shape.addSimplifier(['reshape'],lambda x:x.args[1])
shape.addSimplifier(['flatten'],lambda x:array(sum_(*[count(a) for a in x.args])))
shape.addSimplifier(['neg'],lambda x:shape(x.args[0]))
shape.addSimplifier(['transpose'],lambda x:array(shape(x.args[0])[1],shape(x.args[0])[0]))
eye.returnTypeFunc = _eye_returnType
basis.returnTypeFunc = _basis_returnType
zero.returnTypeFunc = _zero_returnType
reshape.returnTypeFunc = _reshape_returnType
reshape.returnTypeDescription = "array, or a list if size is a nested tuple"
reshape.presimplifier = _reshape_presimplifier
reshape.simplifier = _reshape_simplifier
transpose.deriv = [lambda x,dx:transpose(dx)]
#TEST: new simplifiers
transpose.simplifier = _transpose_simplifier 
transpose.addSimplifier(['eye'],lambda x:x)
transpose.addSimplifier(['basis'],lambda x:x)
def _zerotranspose(sh):
    s = to_const(sh)
    if s is not None:
        s = list(s)
        if len(s) >= 2:
            s[0],s[-1] = s[-1],s[0]
        return zero(tuple(s))
    return None
transpose.addSimplifier(['zero'],lambda x:_zerotranspose(x.args[0]))
transpose.addSimplifier(['neg'],lambda x:-transpose(x.args[0]))
transpose.addSimplifier(['row_stack'],lambda x:column_stack(transpose.optimized(v) for v in x.args))
def _transpose_array_simplifier(x):
    vsize = None
    for v in x.args:
        if is_op(v,'array') or is_op(v,'list'):
            vlen = to_const(shape.optimized(v))
            if vlen is None:
                return None
            if vsize is None:
                vsize = vlen
            else:
                if vsize != vlen:
                    return None
    if not isinstance(vsize,(list,tuple)) or len(vsize) != 1:
        return None
    #everything is good for a 
    n = vsize[0]
    m = len(x.args)
    inner = []
    for i in range(n):
        inner.append(array(*[v[i] for v in x.args]))
    return array(*inner)
transpose.addSimplifier(['array'],_transpose_array_simplifier)
transpose.addSimplifier(['column_stack'],lambda x:row_stack(transpose.optimized(v) for v in x.args))
transpose.returnTypeFunc = _transpose_returnType
transpose.returnTypeDescription = "Same as input, but with the first and second axes transposed"
transpose.properties['inverse'] = weakref.proxy(transpose)
transpose.printers['str'] = lambda expr,astr:'('+astr[0]+').T' if isinstance(expr.args[0],OperatorExpression) else astr[0]+'.T'
transpose2.argTypes = [Type('A'),Type('L')]
transpose2.returnTypeFunc = _transpose2_returnType
transpose2.returnTypeDescription = "array with shuffled axes"
transpose2.deriv = [lambda x,axes,dx:transpose(dx,axes),None]
transpose2.simplifier = _transpose2_simplifier
transpose2.addSimplifier(['neg',None],lambda x,axes:-transpose2(x.args[0],axes))
range_.argTypes = [Type('I')]
eye.argTypes = [Type('I')]
basis.argTypes = [Type('I'),Type('I')]
neg.deriv = [lambda x,dx:-dx]
neg.colstackderiv = neg.deriv
neg.rowstackderiv = neg.deriv
#TEST: new simplifiers
#neg.simplifier = _neg_simplifier  
neg.addSimplifier(['zero'],lambda x:x.args[0])
neg.addSimplifier(['sum'],lambda x:sum_(*[-a for a in x.args]))
neg.addSimplifier(['add'],lambda x:add(*[-a for a in x.args]))
neg.properties['inverse'] = weakref.proxy(neg)
add.deriv = lambda args,dargs:sum_.optimized(*dargs)
add.colstackderiv = add.deriv
add.rowstackderiv = add.deriv
add.simplifier = _sum_simplifier
add.properties['commutative'] = True
add.properties['associative'] = True
add.properties['foldable'] = True
add.properties['foldfunc'] = lambda const: (const,False) if not is_zero(const) else (None,False)
add.printers['str'] = lambda expr,astr: ' + '.join(astr)
add.printers['parse'] = lambda expr,astr: ' + '.join(astr)
sum_.deriv = add.deriv
sum_.colstackderiv = add.colstackderiv
sum_.rowstackderiv = add.rowstackderiv
sum_.simplifier = add.simplifier
sum_.properties = add.properties
sum_.printers = add.printers
sub.deriv = [(lambda x,y,dx:dx),(lambda x,y,dy:-dy)]
sub.colstackderiv = sub.deriv
sub.rowstackderiv = sub.deriv
#TEST: new simplifiers
#sub.simplifier = _sub_simplifier
sub.addSimplifier([None,'neg'],lambda x,y:add(x,y.args[0]))
sub.addSimplifier([None,'zero'],lambda x,y:x)
sub.addSimplifier(['zero',None],lambda x,y:neg(y))
sub.addSimplifier(['add',None],lambda x,y:sum_(*(x.args+[neg(y)])))
sub.addSimplifier(['sum',None],lambda x,y:sum_(*(x.args+[neg(y)])))
mul.deriv = _mul_deriv
mul.presimplifier = _mul_presimplifier
mul.simplifier = _mul_simplifier
mul.properties['commutative'] = True
mul.properties['associative'] = True
mul.properties['foldable'] = True
mul.properties['foldfunc'] = lambda const: (const,False) if const != 1 else (None,False)
mul.printers['str'] = lambda expr,astr: '*'.join(astr)
mul.printers['parse'] = lambda expr,astr: '*'.join(astr)
div.deriv = [(lambda x,y,dx:div(dx,y)),(lambda x,y,dy:-mul(dy,div(x,y**2)))]
div.simplifier = _div_simplifier
abs_.deriv = [lambda x,dx:sign(x)*dx]
abs_.rowstackderiv = [lambda x,dx:sign(x)*dx]
#TEST: new simplifiers
#abs_.simplifier = _abs_simplifier
abs_.addSimplifier(['neg'],lambda x:abs_(x.args[0]))
abs_.addSimplifier(['abs'],lambda x:x)
abs_.addSimplifier(['zero'],lambda x:x)
abs_.addSimplifier(['eye'],lambda x:x)
abs_.addSimplifier(['basis'],lambda x:x)
def _abs_default_simplify(x):
    if isinstance(x,OperatorExpression):
        #print "Doing default simplify abs",x,"properties",x.functionInfo.properties
        if 'nonnegative' in x.functionInfo.properties or 'positive' in x.functionInfo.properties:
            return x
        if 'negative' in x.functionInfo.properties or 'nonpositive' in x.functionInfo.properties:
            return -x
    return None
abs_.addSimplifier([None],_abs_default_simplify)
sign.deriv = 0
#TEST: new simplifiers
#sign.simplifier = _sign_simplifier
sign.addSimplifier(['neg'],lambda x:-sign(x.args[0]))
sign.addSimplifier(['zero'],lambda x:x)
sign.addSimplifier(['eye'],lambda x:x)
sign.addSimplifier(['basis'],lambda x:x)
dot.deriv = [(lambda x,y,dx:dot(dx,y)),(lambda x,y,dy:dot(x,dy))]
dot.rowstackderiv = [lambda x,y,dx:dot(dx,y),None]
dot.colstackderiv = [None,lambda x,y,dy:dot(x,dy)]
dot.returnTypeFunc = dottype
#TEST: new simplifiers
dot.simplifier = _dot_simplifier
dot.addSimplifier(['neg','neg'],lambda x,y:dot(x.args[0],y.args[0]))
dot.addSimplifier([None,'neg'],lambda x,y:-dot(x,y.args[0]))
dot.addSimplifier(['neg',None],lambda x,y:-dot(x.args[0],y))
dot.addSimplifier(['eye',None],lambda x,y:y)
dot.addSimplifier([None,'eye'],lambda x,y:x)
#TODO: these only work for vector-vector dot products
dot.addSimplifier(['basis',None],lambda x,y:getitem._call(y,x.args[0]))
dot.addSimplifier([None,'basis'],lambda x,y:getitem._call(x,y.args[0]))
dot.addSimplifier(['zero',None],lambda x,y:zero._call(dotshape(x,y)))
dot.addSimplifier([None,'zero'],lambda x,y:zero._call(dotshape(x,y)))
def _dot_dot_simplifier(x,y):
    if is_op(x,'dot'):
        #consider (x0*x1)*y vs x0*(x1*y)
        a,b,c=x.args[0],x.args[1],y
    else:
        a,b,c=x,y.args[0],y.args[1]
    sa = shape.optimized(a)
    sb = shape.optimized(b)
    sc = shape.optimized(c)
    abfirst = None
    if is_const(a) and is_const(b) and not is_const(c):
        abfirst = True
    elif is_const(b) and is_const(c) and not is_const(a):
        abfirst = False
    elif is_const(sa) and is_const(sb) and is_const(sc):
        minlen = len(sa)
        if len(sa) < len(sb):
            if len(sc) < len(sa):
                abfirst = False
                minlen = len(sc)
            elif len(sa) < len(sc):
                abfirst = True
        elif len(sb) < len(sa):
            minlen = len(sb)
            abfirst = False
            if len(sc) < len(sb):
                abfirst = False
                minlen = len(sc)
            elif len(sb) < len(sc):
                #weird, M*V*M?
                pass
        else:
            #len(sb) == len(sa) 
           if len(sc) < len(sb):
                #M * M * V
                minlen = len(sc)
                abfirst = False
        if abfirst is None:
            if len(sc) == len(sa) and len(sa) < len(sb):
                #V * M * V, complexity really doesn't matter
                if sb[0] > sb[1]:
                    abfirst = True
                else:
                    abfirst = False
            else:
                #all the same length, still need to determine order
                assert len(sa) == len(sb) == len(sc) == 2,"Hmm... can only simplify M*M*M dot products"
                assert sa[1] == sb[0],"Invalid sizes for dot product"
                assert sb[1] == sc[0],"Invalid sizes for dot product"
                abcount = sa[0]*sb[1]
                bccount = sb[0]*sc[1]
                if abcount < bccount:
                    abfirst = True
                elif bccount < abcount:
                    abfirst = False
    if abfirst is None:
        return None
    elif abfirst:
        if not is_op(x,'dot'):
            return dot._call(dot._call(a,b),c)
    else:
        if is_op(x,'dot'):
            return dot._call(a,dot._call(b,c))
dot.addSimplifier(['dot',None],_dot_dot_simplifier)
def dot_setitem_simplifier1(x,y):
    x0,indices,v = x.args
    if is_const(indices):
        iconst = to_const(indices)
        nonindices = []
        iset = set(iconst)
        lx = len_.optimized(x0)
        for i in range(lx):
            if i not in iset:
                nonindices.append(i)
        #print "dot-setitem returning",dot(x0[nonindices],y[nonindices])+dot(v,y[iconst])
        return dot._call(x0[nonindices],y[nonindices])+dot._call(v,y[iconst])
dot.addSimplifier(['setitem',None],dot_setitem_simplifier1)
def dot_setitem_simplifier2(x,y):
    y0,indices,v = y.args
    if is_const(indices):
        iconst = to_const(indices)
        nonindices = []
        iset = set(iconst)
        ly = len_.optimized(y0)
        for i in range(ly):
            if i not in iset:
                nonindices.append(i)
        #print "dot-setitem returning",dot(x[nonindices],y0[nonindices])+dot(x[iconst],v)
        return dot._call(getitem.optimized(x,nonindices),getitem.optimized(y0,nonindices))+dot._call(getitem.optimized(x,iconst),v)
dot.addSimplifier([None,'setitem'],dot_setitem_simplifier2)
dot.addSimplifier(['_scalar',None],lambda x,y:mul._call(x,y))
dot.addSimplifier([None,'_scalar'],lambda x,y:mul._call(y,x))
def _dot_const_simplify(x,y):
    x = np.asarray(x)
    try:
        iszero = np.all(x==0)
    except Exception:
        warnings.warn("dot(constant,y): Weird case, x can't be tested for equality to 0?")
        warnings.warn(x.__class__.__name__)
        warnings.warn(str(x))
        raise
    if iszero:
        return zero(dotshape(x,y))
    if is_sparse(x):
        #print("SPARSIFYING DOT PRODUCT x with",np.count_nonzero(x),"NONZEROS / ",count.optimized(x))
        nzs = np.nonzero(x)
        yshape = to_const(shape.optimized(y))
        if len(x.shape) == 1:
            #it's a sum-product
            args = [x[ind]*y[ind] for ind in nzs[0]]
            dp = sum_(args)
            return dp
        elif yshape is not None:
            """
            newx = np.zeros(x.shape[:-1] + tuple(yshape[1:]),dtype='O')
            for xind in itertools.product(*[range(d) for d in x.shape[:-1]]):
                for yind in itertools.product(*[range(d) for d in tuple(yshape[1:])]):
                    res = 0
                    for k in range(x.shape[-1]):
                        if x[xind+(k,)] != 0:
                            res += y[(k,)+yind]
                    if not _is_exactly(res,0):
                        newx[xind+yind] = res
                  newx[ind] = res
            """
            assert x.shape[-1] == yshape[0],"Uh... dot is between two arrays of mismatched shape? %s vs %s"%(str(x.shape),str(yshape))
            if len(yshape) == 1:
                yentries = [getitem._call(y,i) for i in range(yshape[0])]
                newx = np.zeros(x.shape[:-1],dtype='O')
                #print "dot",x,y
                #print "Nonzeros:",nzs
                #print "Result shape",newx.shape
                for i,ind in enumerate(nzs[-1]):
                    xindex = tuple(nzdim[i] for nzdim in nzs)
                    dpindex = tuple(nzdim[i] for nzdim in nzs[:-1])
                    #print "SETTING RESULT INDEX",dpindex,"to",x[xindex]*y[ind]
                    newx[dpindex] += x[xindex]*yentries[ind]
                #print "RESULT",array(*newx.tolist())
                #raw_input()
                return array(*newx.tolist())
            else:
                yentries = [getitem._call(y,i) for i in range(yshape[0])]
                newx = np.zeros(x.shape[:-1]+tuple(yshape[1:]),dtype='O')
                for i in range(len(nzs[0])):
                    xindex = tuple(nzdim[i] for nzdim in nzs)
                    dpindex = xindex[:-1]
                    yindex = xindex[-1]
                    val = simplify(x[xindex]*yentries[yindex])
                    if len(yshape) == 2:
                        for yind in range(yshape[1]):
                            entry = val[yind]
                            if isinstance(entry,ConstantExpression):
                                entry = entry.value
                            newx[dpindex+(yind,)] += entry
                    else:
                        for yind in itertools.product(*[list(range(d)) for d in yshape[1:]]):
                            entry = val[yind]
                            if isinstance(entry,ConstantExpression):
                                entry = entry.value
                            newx[dpindex+yind] += entry
                #raw_input()
                return array(*newx.tolist())
def _dot_const_simplify2(x,y):
    y = np.asarray(y)
    if np.all(y==0):
        return zero(dotshape(x,y))
    if is_sparse(y):
        #print("SPARSIFYING DOT PRODUCT y with",np.count_nonzero(y),"NONZEROS / ",count.optimized(x))
        nzs = np.nonzero(y)
        xshape = to_const(shape.optimized(x))
        if xshape is None:
            #Not sure how to handle this
            return None
        if len(y.shape) == 1:
            #matrix-vector product
            rows = xshape[0]
            res = []
            for i in range(rows):
                xrow = getitem.optimized(x,i)
                elems = [mul.optimized(xrow[j],y[j]) for j in nzs[0]]
                if len(elems) > 1:
                    res.append(sum_.optimized(*elems))
                elif len(elems) == 1:
                    res.append(elems[0])
                else:
                    res.append(0)
            return array(*res)
        elif len(xshape) == 1:
            xentries = [getitem.optimized(x,i) for i in range(xshape[-1])]
            newy = np.zeros(y.shape[1:],dtype='O')
            #print "dot",x,y
            #print "Nonzeros:",nzs
            #print "Result shape",newy.shape
            for i,ind in enumerate(nzs[0]):
                yindex = tuple(nzdim[i] for nzdim in nzs)
                dpindex = tuple(nzdim[i] for nzdim in nzs[1:])
                #print "SETTING RESULT INDEX",dpindex,"to",y[yindex]*x[ind]
                newy[dpindex] += y[yindex]*xentries[ind]
            #print "RESULT",array(*newy.tolist())
            #raw_input()
            return array(*newy.tolist())
        elif len(xshape) == 2 and len(y.shape)==2:
            #matrix-matrix product
            rows = xshape[0]
            cols = y.shape[1]
            res = []
            for i in range(rows):
                xrow = getitem.optimized(x,i)
                elems = [0]*cols                
                for (j,k) in zip(*nzs):
                    elems[k] += mul.optimized(xrow[j],y[j,k])
                res.append(elems)
            return array(*res)
    return None
dot.addSimplifier(['_const',None],_dot_const_simplify)
dot.addSimplifier([None,'_const'],_dot_const_simplify2)
dot.properties['associative'] = True
#dot.properties['foldable'] = True
outer.argTypes = [Type('V'),Type('V')]
outer.deriv = [lambda x,y,dx:outer(dx,y),lambda x,y,dy:outer(y,dy)]
outer.rowstackderiv = [lambda x,y,dx:dot(dx,y),None]
outer.colstackderiv = [None,lambda x,y,dy:outer(x,dy)]
def _outer_returnType(x,y):
    try:
        xs = x.returnType().count()
        ys = y.returnType().count()
        if xs is not None and ys is not None:
            return Type('M',(xs,ys))
    except Exception:
        return Type('M')
outer.returnTypeFunc = _outer_returnType
outer.addSimplifier(['neg','neg'],lambda x,y:outer(x.args[0],y.args[0]))
outer.addSimplifier([None,'neg'],lambda x,y:-outer(x,y.args[0]))
outer.addSimplifier(['neg',None],lambda x,y:-outer(x.args[0],y))
outer.addSimplifier(['zero',None],lambda x,y:zero._call(array._call(len_._call(x),len_._call(y))))
outer.addSimplifier([None,'zero'],lambda x,y:zero._call(array._call(len_._call(x),len_._call(y))))
tensordot.argTypes = [Type('V'),Type('V'),Type('X')]
tensordot.deriv = [lambda x,y,axes,dx:tensordot(dx,y,axes),lambda x,y,axes,dy:tensordot(y,dy,axes),None]
tensordot.rowstackderiv = [lambda x,y,axes,dx:tensordot(dx,y,axes),None,None]
tensordot.colstackderiv = [None,lambda x,y,axes,dy:tensordot(x,dy,axes),None]        
tensordot.returnTypeFunc = _tensordot_returnType
tensordot.properties['associative'] = True
tensordot.addSimplifier(['neg','neg',None],lambda x,y,axes:tensordot(x.args[0],y.args[0],axes))
tensordot.addSimplifier([None,'neg',None],lambda x,y,axes:-tensordot(x,y.args[0],axes))
tensordot.addSimplifier(['neg',None,None],lambda x,y,axes:-tensordot(x.args[0],y,axes))
tensordot.addSimplifier([None,None,'_const'],_tensordot_const_simplifier)


pow_.deriv = [(lambda b,e,db:mul(mul.optimized(db,e),pow_(b,e-1))),(lambda b,e,de:mul(mul.optimized(de,ln(b)),pow_(b,e)))]
pow_.simplifier = _pow_simplifier
array.returnTypeFunc = _array_returnType
array.simplifier = _array_simplifier
list_.returnTypeFunc = _list_returnType
list_.simplifier = _list_simplifier
tuple_.returnTypeFunc = _list_returnType
tuple_.simplifier = _tuple_simplifier
if_.deriv = [None,(lambda cond,trueval,falseval,dtrue:if_(cond,dtrue,0)),(lambda cond,trueval,falseval,dfalse:if_(cond,0,dfalse))]
if_.stackderiv = if_.deriv
#TEST: new simplifiers
#if_.presimplifier = _if_presimplifier
#if_.simplifier = _if_simplifier
if_.addSimplifier(['_const',None,None],(lambda cond,trueval,falseval:(trueval if cond else falseval)),pre=True)
if_.addSimplifier(['_const',None,None],(lambda cond,trueval,falseval:(trueval if cond else falseval)),pre=False)
if_.addSimplifier([None,'_scalar','_scalar'],(lambda cond,trueval,falseval:trueval if trueval == falseval else None),pre=False)
def _if_simplifier(cond,trueval,falseval):
    if trueval.match(falseval):
        return trueval
    return None
if_.addSimplifier([None,None,None],_if_simplifier,pre=False)
argmax.simplifier = _argmax_simplifier
argmin.simplifier = _argmin_simplifier
all_.properties['foldable'] = True
all_.properties['foldfunc'] = lambda const: (None,False) if const else (False,True)
any_.properties['foldable'] = True
any_.properties['foldfunc'] = lambda const: (True,True) if const else (None,True)
all_.simplifier = _all_simplifier
and_.simplifier = _all_simplifier
any_.simplifier = _any_simplifier
or_.simplifier = _any_simplifier
all_.properties['commutative'] = True
any_.properties['commutative'] = True
and_.properties['commutative'] = True
or_.properties['commutative'] = True
all_.properties['associative'] = True
any_.properties['associative'] = True
and_.properties['associative'] = True
or_.properties['associative'] = True
max_.deriv = _max_deriv
min_.deriv = _min_deriv
max_.properties['commutative'] = True
min_.properties['commutative'] = True
max_.properties['associative'] = True
min_.properties['associative'] = True
cos.deriv = [lambda x,dx:mul(-sin(x),dx)]
cos.colstackderiv = cos.deriv
cos.addSimplifier(['neg'],lambda x:cos(x.args[0]))
sin.deriv = [lambda x,dx:mul(cos(x),dx)]
sin.colstackderiv = sin.deriv
sin.addSimplifier(['neg'],lambda x:-sin(x.args[0]))
tan.deriv = [lambda x,dx:dx/sin(x)**2]
tan.colstackderiv = tan.deriv
tan.addSimplifier(['neg'],lambda x:tan(x.args[0]))
arccos.deriv = [lambda x,dx:-dx/sqrt(1.0-x**2)]
arccos.colstackderiv = arccos.deriv
arcsin.deriv = [lambda x,dx:dx/sqrt(1.0-x**2)]
arcsin.colstackderiv = arcsin.deriv
arctan.deriv = [lambda x,dx:dx/(1.0+x**2)]
arctan.colstackderiv = arctan.deriv
sqrt.deriv = [lambda x,dx:0.5*dx/sqrt(x)]
sqrt.colstackderiv = sqrt.deriv
sqrt.addSimplifier(['pow'],lambda x:pow_(x.args[0],x.args[1]*0.5))
sqrt.properties['nonnegative'] = True
exp.deriv = [lambda x,dx:exp(x)*dx]
exp.colstackderiv = exp.deriv
exp.properties['inverse'] = weakref.proxy(ln)
exp.properties['positive'] = True
ln.deriv = [lambda x,dx:dx/x]
ln.colstackderiv = ln.deriv
ln.properties['inverse'] = weakref.proxy(exp)
getitem.argTypes = [Type('L'),Type('X')]
getitem.deriv = _getitem_deriv
getitem.returnTypeFunc = _getitem_returnType
#TEST: new simplifier
#getitem.simplifier = _getitem_simplifier
def _to_indices(v,slice_or_list):
    if isinstance(slice_or_list,slice):
        sl = slice_or_list
        vlen = len_.optimized(v)
        vlen = to_const(vlen)
        if vlen is None:
            if sl.stop is not None and sl.stop < 900000000:
                return list(range(sl.start,sl.stop,(sl.step if sl.step is not None else 1)))
            #can't simplify an open-ended slice of a variable-length object
            raise ValueError("Can't get a slice of a variable-length object")
        return list(range(sl.start,min(vlen,sl.stop if sl.stop is not None else None),(sl.step if sl.step is not None else 1)))
    else:
        return slice_or_list
getitem.addSimplifier(['eye','_scalar'],lambda v,index:basis._call(index,getitem.optimized(shape.optimized(v),0)),pre=True)
getitem.addSimplifier(['eye','_scalar'],lambda v,index:basis._call(index,getitem.optimized(shape.optimized(v),0)))
getitem.addSimplifier(['eye','_const'],lambda v,indices:array(*[basis._call(i,getitem.optimized(shape.optimized(v),0)) for i in _to_indices(v,indices)]),pre=True)
getitem.addSimplifier(['eye','_const'],lambda v,indices:array(*[basis._call(i,getitem.optimized(shape.optimized(v),0)) for i in _to_indices(v,indices)]))
getitem.addSimplifier(['zero','_scalar'],lambda v,index:zero(getitem.optimized(v.args[0],slice(1,None,None))),pre=True)
getitem.addSimplifier(['zero','_const'],lambda v,indices:zero(flatten.optimized(v.args[0],slice(1,None,None)),len_.optimized(_to_indices(v,indices))),pre=True)
getitem.addSimplifier(['array','_scalar'],lambda v,index:v.args[index],pre=True)
getitem.addSimplifier(['array','_scalar'],lambda v,index:v.args[index])
getitem.addSimplifier(['array','_const'],lambda v,indices:array(*[v.args[i] for i in _to_indices(v,indices)]),pre=True)
getitem.addSimplifier(['array','_const'],lambda v,indices:array(*[v.args[i] for i in _to_indices(v,indices)]))
getitem.addSimplifier(['list','_scalar'],lambda v,index:v.args[index],pre=True)
getitem.addSimplifier(['list','_scalar'],lambda v,index:v.args[index])
getitem.addSimplifier(['list','_const'],lambda v,indices:list(*[v.args[i] for i in _to_indices(v,indices)]),pre=True)
getitem.addSimplifier(['list','_const'],lambda v,indices:list(*[v.args[i] for i in _to_indices(v,indices)]))
getitem.addSimplifier(['row_stack','_scalar'],lambda v,index:v.args[index],pre=True)
getitem.addSimplifier(['row_stack','_const'],lambda v,indices:list(*[v.args[i] for i in _to_indices(v,indices)]),pre=True)
getitem.addSimplifier(['add',None],lambda v,indices:add(*[a[indices] for a in v.args]),pre=True)
getitem.addSimplifier(['sum',None],lambda v,indices:sum_(*[a[indices] for a in v.args]),pre=True)
getitem.addSimplifier(['sub',None],lambda v,indices:sub(*[a[indices] for a in v.args]),pre=True)
getitem.addSimplifier(['mul',None],lambda v,indices:mul(*[a if is_scalar(a) else a[indices] for a in v.args]),pre=True)
getitem.addSimplifier(['neg',None],lambda v,indices:neg(v.args[0][indices]),pre=True)
getitem.addSimplifier(['dot','_const'],lambda v,indices:(dot._call(v.args[0][indices],v.args[1]) if isinstance(indices,_PY_INT_TYPES+(list,slice)) else None),pre=True)
getitem.addSimplifier(['basis','_scalar'],lambda v,index:if_.optimized(v.args[0]==index,1,0),pre=True)
getitem.addSimplifier(['if',None],lambda v,indices:if_(v.args[0],v.args[1][indices],v.args[2][indices]),pre=True)
def _getitem_basis_simplifier(v,indices):
    if not is_const(v.args[0]):
        return None
    i = to_const(v.args[0])
    try:
        eindices = _to_indices(v,indices)
        if i in eindices:
            return basis(eindices.index(i),len(eindices))
        else:
            return zero(len(eindices))
    except Exception:
        return None
getitem.addSimplifier(['basis','_const'],_getitem_basis_simplifier,pre=True)
getitem.addSimplifier(['basis','_const'],_getitem_basis_simplifier)
getitem.addSimplifier(['flatten','_scalar'],lambda v,index:v.args[0][0] if index==0 else (v.args[-1][-1] if index == -1 else None),pre=True)
getitem.addSimplifier(['flatten','_scalar'],lambda v,index:v.args[0][0] if index==0 else (v.args[-1][-1] if index == -1 else None))
getitem.addSimplifier(['range','_scalar'],lambda v,index:v.args[0]+index,pre=True)
getitem.addSimplifier(['range','_scalar'],lambda v,index:v.args[0]+index)
#TODO: something mistakes variable slices as being constant
#getitem.addSimplifier(['range','_const'],lambda v,indices:[v.args[0]+i for i in _to_indices(v,indices)])

def _getitem_row_stack(v,index):
    for a in v.args:
        sa = to_const(shape.optimized(a))
        if sa is None:
            return None
        if len(sa) == 0:
            if index == 0: return a
            index -= 1
        else:
            if index < sa[0]: return a[index]
            index -= sa[0]
    return None
getitem.addSimplifier(['row_stack','_scalar'],_getitem_row_stack)
def _getitem_flatten(v,index):
    for a in v.args:
        ca = to_const(count.optimized(a))
        if ca is None:
            return None
        if ca == 1:
            if index == 0: return a
            index -= 1
        else:
            if index < ca:
                if a.returnType().dims() == 1:
                    return a[index]
                else:
                    #TODO: figure this out for matrices
                    return None
            index -= ca
getitem.addSimplifier(['flatten','_scalar'],_getitem_flatten)
def _getitem_setitem_scalar(v,index):
    x,setindices,rhs = v.args
    setindices = to_const(setindices)
    if setindices is None:
        return None
    if hasattr(setindices,'__iter__'):
        if index in setindices:
            return rhs[setindices.index(index)]
        return x[index]
    else:
        if index == setindices:
            return rhs
        return x[index]
def _getitem_setitem_list(v,indices):
    x,setindices,rhs = v.args
    setindices = to_const(setindices)
    if setindices is None:
        return None
    fromx = []
    resindices = []
    if hasattr(setindices,'__iter__'):
        for i in indices:
            if i in setindices:
                fromx.append(False)
                resindices.append(setindices.index(i))
            else:
                fromx.append(True)
                resindices.append(i)
        if all(fromx):
            return x[resindices]
        elif not any(fromx):
            return rhs[resindices]
        else:
            return row_stack(*[x[i] if ifromx else rhs[i] for (i,ifromx) in zip(resindices,fromx)])
    else:
        for i in indices:
            if i == setindices:
                fromx.append(False)
                resindices.append(0)
            else:
                fromx.append(True)
                resindices.append(i)
        if all(fromx):
            return x[resindices]
        elif not any(fromx):
            return rhs
        else:
            return row_stack(*[x[i] if ifromx else rhs for (i,ifromx) in zip(resindices,fromx)])
    return
getitem.addSimplifier(['setitem','_scalar'],_getitem_setitem_scalar,pre=True)
getitem.addSimplifier(['setitem','_const'],_getitem_setitem_list,pre=True)
def _getitem_getitem_scalar(v,index):
    x,innerindices = v.args
    if is_op(x,'eye'):
        cinnerindices = to_const(innerindices)
        if isinstance(cinnerindices,int):
            return 1 if cinnerindices == index else 0
        elif isinstance(cinnerindices,slice):
            start,stop,step = cinnerindices.start,cinnerindices.stop,cinnerindices.step
            if start is None:
                start = 0
            if step is None:
                step = 1
            if stop is None:
                return 1 if (index - to_const(start))%step == 0 else 0
            else:
                return 1 if ((index - to_const(start))%step == 0 and index - to_const(start) < to_const(stop)) else 0
        if dims.optimized(innerindices)==0:
            return if_(innerindices ==index,1,0)
    elif is_op(x,'transpose'):
        cinnerindices = to_const(innerindices)
        cindex = to_const(index)
        if isinstance(cinnerindices,int) and cindex is not None:
            Ash = to_const(shape.optimized(x.args[0]))
            if Ash is not None:
                assert len(Ash) == 2 and index < Ash[0] and cinnerindices < Ash[1],"Invalid indices to transposed matrix: shape %s, indices %s %s"%(Ash,innerindices,index)
        return x.args[0][index][innerindices]
    return None
def _getitem_getitem(v,indices):
    x,innerindices = v.args
    if is_op(x,'transpose'):
        return x.args[0][indices][innerindices]
    if is_op(x,'column_stack'):
        return x.args[indices][innerindices]
    return None

getitem.addSimplifier(['getitem','_scalar'],_getitem_getitem_scalar,pre=True)
getitem.addSimplifier(['getitem',None],_getitem_getitem,pre=True)
getitem.returnTypeDescription = "numeric, or an array if a slice is given"
setitem.returnTypeFunc = _returnType1
#getslice.deriv = [lambda v,start,stop,dv:getslice(dv,start,stop),None]
flatten.deriv = _flatten_deriv
flatten.simplifier = _flatten_simplifier
flatten.returnTypeFunc = _flatten_returnType
flatten.properties['associative'] = True
flatten.properties['foldable'] = True
weightedsum.simplifier = _weightedsum_simplifier
weightedsum.deriv = _weightedsum_deriv
weightedsum.rowstackderiv = weightedsum.deriv
setitem.argTypes = [Type('L'),Type('X'),Type(None)]
def _setitem_deriv_vec(x,indices,val,dx):
    return setitem(dx,indices,zero(shape(val)))
def _setitem_deriv_val(x,indices,val,dval):
    return setitem(zero(shape(x)),indices,dval)
setitem.setDeriv('vec',_setitem_deriv_vec,asExpr=True)
setitem.setDeriv('val',_setitem_deriv_val,asExpr=True)
setitem.simplifier = _setitem_simplifier
array.deriv = lambda args,dargs:array(*dargs)
array.printers['str'] = lambda expr,astr:'['+','.join(astr)+']'
array.printers['parse'] = lambda expr,astr:'['+','.join(astr)+']'
list_.deriv = lambda args,dargs:dargs
list_.printers = array.printers
tuple_.deriv = lambda args,dargs:tuple(dargs)
tuple_.printers['str'] = lambda expr,astr:'('+','.join(astr)+')' if len(astr) != 1 else '('+astr[0]+',)'
zip_.deriv = lambda args,dargs:zip_(*dargs)
subs.deriv = _subs_deriv
def _subs_simplifier(expr,var,val):
    return _subs(None,expr,var,val,False,False,False)
subs.simplifier = _subs_simplifier
map_.argTypes = [Type(None),Type('U'),Type('L')]
map_.returnTypeFunc = _map_returnType
map_.deriv = _map_deriv
summation.argTypes = [Type(None),Type('U'),Type('L')]
summation.deriv = _summation_deriv


def _run_basic_test():
    print("Type of 0.4",type_of(0.4))
    print("Type of [0.5,0.2]",type_of([0.5,0.2]))
    print("Type of [1,0]",type_of([1,0]))
    print("Type of [1,[0,1]]",type_of([1,[0,1]]))
    print("Type of [[1,0],[0,1]]",type_of([[1,0],[0,1]]))
    print("Type of [[1,0],[0,[1,2]]]",type_of([[1,0],[0,[1,2]]]))
    e = Context()
    x = e.addVar("x",'V',size=2)
    y = e.addVar("y",'N')
    subexpr = y+y
    print("x[1] + y:",x[1] + y)
    print("flatten(x[1],y):",flatten(x[1],y))
    print("flatten(x[1],y) at x=[3,4], y=[5,6]:",flatten(x[1],y).eval({'x':[3,4],'y':[5,6]}))
    print("d/dy stack(x[1],y):",deriv(flatten(x[1],y),y),"= [0,1]")
    #print "deriv(flatten(x[1],y)) at x=[3,4], y=[5,6]:",deriv(flatten(x[1],y),y).eval({'x':[3,4],'y':[5,6]})
    print("Simplification of 0+y:",simplify(0+y))
    print("Simplification of 0*y:",simplify(0*y))
    print("Simplification of 1*y:",simplify(1.0*y))
    print("Simplification of y+y:",simplify(y+y))
    print("Simplification of zero(2):",simplify(zero(2)))
    print("Simplification of zero(100):",simplify(zero(100)))
    print("Simplification of y/x[1]:",simplify(y/x[1]))
    print("Simplification of d/dy cos(-y):",simplify(cos(-y).deriv(y)))
    print("Derivative of y^3:",(y**3).deriv(y))
    print("Derivative of y^4:",(y**4).deriv(y))
    print("Derivative of (expanded) y^3:",(y*y*y).deriv(y))
    print("Derivative of (expanded) y^4:",(y*y*y*y).deriv(y))
    print("Derivative of (y+y):",deriv(subexpr,y))
    print("Derivative of (y+y)^3:",deriv(subexpr*subexpr*subexpr,y))
    print("Derivative of (expanded) (y+y)^3, simplified:",simplify(deriv(subexpr*subexpr*subexpr,y)))
    print("eye(2)[1]",(eye(2)[1]).evalf())
    print("0+eye(2)[1] (simplified)",simplify(0+eye(2)[1]))
    print("d/dx (x[1])",(x[1]).deriv(x))
    print("d/dx (x[1] + 1)",(x[1] + 1).deriv(x))
    print("d/dx (x[1] + y)",(x[1] + y).deriv(x))
    print("d/dx (x[0] + 3*x[1] + y)",(x[0] + 3*x[1] + y).deriv(x))
    print("d/dy (x[1] + y)",(x[1] + y).deriv(y))
    print("d/dx (x[0]*x[1])",(x[0]*x[1]).deriv(x))
    print("d/dy (y^2/x[1]):",deriv(y**2/x[1],y))
    print("d/dx (y^2/x[1]):",deriv(y**2/x[1],x))
    print("d/dx (x[0]*x[1])",(x[0]*x[1]).deriv(x))
    print("d/dx (x[0]*x[1]) at [3,4]",(x[0]*x[1]).deriv(x).evalf({'x':[3,4]}))
    print("d/dx max(x[0],x[1]):",(max_(x[0],x[1])).deriv(x))
    print("range(10)[3:6]:",range_(10)[3:6].eval())
    depth = 20
    expr = const(1)
    for i in range(depth):
        expr = expr + expr
    expr2 = sum_(*[const(1)]*pow(2,depth))
    import time
    print("Timing test,",pow(2,depth)," adds")
    print("Expression depth",expr.depth())
    t0=time.time()
    print("  nested eval",expr.eval(),time.time()-t0)
    #raw_input("Press enter to continue...")
    t0=time.time()
    print("  nested evalf",expr.evalf(),time.time()-t0)
    #raw_input("Press enter to continue...")
    t0=time.time()
    print("  nested _evalf",expr._evalf(None),time.time()-t0)
    #raw_input("Press enter to continue...")
    t0=time.time()
    print("  sum eval",expr2.eval(),time.time()-t0)
    #raw_input("Press enter to continue...")
    t0=time.time()
    print("  plain sum",sum([1]*pow(2,depth)),time.time()-t0)
    #raw_input("Press enter to continue...")
    
def _run_function_test():
    e = Context()
    x = e.addVar("x",'V',size=2)
    y = e.addVar("y",'N')
    f = lambda a,b:3*a*b
    ef = e.declare(f,"myFunc")
    e.addExpr("z",ef(x[1],2.0))
    z = e.expr('z')
    print("Unbound variables",[v.name for v in z.vars()],"= [x]")
    print(z.eval({'x':[3.0,4.0]}),'=',f(4.0,2.0))
    fz,vorder = e.makeFlatFunction(z)
    print(fz([3.0,4.0]),'=',f(4.0,2.0))
    fw,vorder = e.makeFlatFunction(x[1]+y)
    print(fw([3.0,4.0,1.0]),"= 5")
    fw2,vorder = e.makeFlatFunction(x[1]+y,varorder=['y','x'])
    print(fw2([1.0,3.0,4.0]),"= 5")
    dfw,vorder = e.makeFlatFunctionDeriv(x[1]*x[0]+y)
    print(dfw([3.0,4.0,1.0]),"= [4,3,1]")

    context = dict()
    context['some_param'] = 4.0
    g = lambda a,some_param:a+some_param
    eg = e.declare(g,"myFunc2")
    print("Custom function is printed as",eg(y,'some_param'))
    print("Evaluation in context:",eg(y,'some_param').eval(context))
    print("Unbound variables",[v.name for v in eg(y,'some_param').vars()])
    y.bind(4)
    print("Unbound variables with y bound to 4",[v.name for v in eg(y,'some_param').vars()])
    print("All variables with y bound to 4",[v.name for v in eg(y,'some_param').vars(bound=True)])
    y.unbind()
    print("Unbound variables with context",[v.name for v in eg(y,'some_param').vars(context)])

    print("Unbound variables of subs",[v.name for v in subs(expr("x")**2,"x",y).vars()])
    print("Unbound variables of subs",[v.name for v in subs(expr("x")**2*expr("z"),"x",y).vars()])
    print("Unbound variables of multiple-subs",[v.name for v in subs(expr("x")**2*expr("y"),["x","y"],["z","w"]).vars()])

def _run_simplify_test():
    e = Context()
    x = e.addVar("x",'N')
    y = e.addVar("y",'N')
    z = e.addVar("z",'V',2)
    print("x*1 simplifies to",simplify(x*1))
    print("x*0 simplifies to",simplify(x*0))
    print("x^2 * x^3 simplifies to",simplify(x**2 * x**3))
    print("x+x simplifies to",simplify(x + x))
    print("(x+x)+(x+x) simplifies to",simplify((x + x) + (x + x)))
    print("x-x simplifies to",simplify(x - x))
    print("-(-x) simplifies to",simplify(-(-x)))
    print("4*(-x) simplifies to",simplify(const(4)*(-x)))
    print("x+(-x) simplifies to",simplify(x + (-x)))
    v,cs,ds = to_polynomial(x*(x+3)+1)
    print("Polynomial x*(x+3)+1:",simplify(sum_(*[const(c)*v**d for (c,d) in zip(cs,ds)])))
    print("setitem([0]*10,[1,3],z)[1] simplifies to",simplify(setitem([0]*10,[1,3],z)[1])," (should be z[0])")
    print("flatten(z,y)[2] simplifies to",simplify(flatten(z,y)[2]))
    print("dot(setitem([0]*10,[1,3],z),setitem([0]*10,[1,3],z)) simplifies to",simplify(dot(setitem([0]*10,[1,3],z),setitem([0]*10,[1,3],z)))," (should be dot(z,z))")

def _run_arraytest():
    e = Context()
    x = e.addVar("x",'V',size=2)
    y = e.addVar("y",'N')
    f = lambda a,b:3*a*b
    ef = e.declare(f,"myFunc")
    e.addExpr("z",ef(x[1],2.0))
    print("Variables:")
    e.listVars(indent=2)
    print("Expressions:")
    e.listExprs(indent=2)
    print("Functions:")
    e.listFunctions(indent=2)

def _run_io_test():
    from . import symbolic_io
    e = Context()
    x = e.addVar("x",'V',size=2)
    y = e.addVar("y",'N')
    f = lambda a,b:3*a*b
    ef = e.declare(f,"myFunc")
    e.addExpr("z",ef(x[1],2.0))
    #test saving and loading
    print("JSON Context output   :",symbolic_io.toJson(e))
    e2 = Context()
    e2.declare(f,"myFunc")
    e2 = symbolic_io.contextFromJson(e2,symbolic_io.toJson(e))
    print("JSON re-loaded Context:",symbolic_io.toJson(e2))

def _run_jacobian_test():
    x = np.zeros(2)
    y = np.zeros(3)
    z = np.zeros(3)
    A = np.zeros((2,3))
    B = np.zeros((3,4))
    print(dotshape(y,z), end=' ')
    print("==",np.dot(y,z).shape)
    print(dotshape(x,A), end=' ')
    print("==",np.dot(x,A).shape)
    print(dotshape(A,y), end=' ')
    print("==",np.dot(A,y).shape)
    print(dotshape(A,B), end=' ')
    print("==",np.dot(A,B).shape)

    e = Context()
    vs = e.addVars("v",5,'N')
    s = flatten(*vs)
    print(s.deriv(vs[3]),"should be [0,0,0,1,0]")
    x = e.addVar("x",'V',size=5)
    y = e.addVar("y",'V',size=3)
    z = e.addVar("z",'N')
    s = flatten(x,y,z)
    print(s.eval({'x':[1,2,3,4,5],'y':[6,7,8],'z':9}),"= [1,2,...,9]")
    print(s.deriv(z),"= [0,0,0,0,0,0,0,0,1]")
    print(s.deriv(y)," should have 3 columns, an identity matrix in rows 5-7")
    dp1 = dot([[1,2,3],[4,5,6]],dot(eye(3),y))
    dp2 = dot(flatten(vs[0],vs[1]),dp1)
    print("M*ident*V product",dp1)
    print("M*ident*V simplified",simplify(dp1))
    print("Deriv:",dp1.deriv(y),"(should be [[1,2,3],[4,5,6]])")
    print("Deriv w.r.t. irrelevant variable:",dp1.deriv(vs[0]),"(should be [0,0])")
    print("Deriv of [v0,v1]*M*ident*V w.r.t. y",dp2.deriv(y),"(should be [v[0]+4v[1],2v[0]+5v[1],3v[0]+6v[1]])")
    print("Deriv of [v0,v1]*M*ident*V w.r.t. v0",dp2.deriv(vs[0]))
    A = np.zeros((5,3))
    A[0,0] = 1
    A[3,1] = 2
    xAy = dot(x,dot(A,dot(eye(3),y)))
    print("Deriv of x^T A y w.r.t x",xAy.deriv(x),"(should be dot(A,y)=[y[0],0,0,2y[1],0])")
    print("Deriv of x^T A y w.r.t y",xAy.deriv(y),"(should be dot(x,A)=[x[0],2x[3],0])")
    print("Deriv of dot(x,x) w.r.t x",dot(x,x).deriv(x),"(should be 2x)")
    print() 
    print("(Beginning tests with more sophisticated matrix calculus)")
    A = e.addVar("A",'M',size=(3,3))
    B = e.addVar("B",'M',size=(3,3))
    print("Derivative of A+B matrix addition",(A+B).deriv(A))
    print("Derivative of y*^T*A*y matrix-vector multiplication w.r.t. A should be outer product",dot(y,dot(A,y)).deriv(A))
    print("  Evaluated ",(dot(y,dot(A,y)).deriv(A)).evalf({'y':[1,2,3]}))
    print("Derivative of A*y matrix-vector multiplication w.r.t. A",dot(A,y).deriv(A))
    print("Derivative of A*B matrix multiplication w.r.t. A",dot(A,B).deriv(A))
    print() 
    print("(Beginning tests with more sophisticated reshaping)")
    somelist = array(z,y)
    print("Derivative of uneven list w.r.t. scalar first argument:",somelist.deriv(z),"(should be [1,[0,0,0]])")
    print("Derivative of uneven list access w.r.t. vector second argument:",somelist[1][2].deriv(y),"(should be [0,0,1])")
    print("Derivative of uneven list w.r.t. vector second argument:",somelist.deriv(y),"(should be [[0,0,0],3x3 identity])")
    Ltype = Type('L',{0:(),1:(3,)})
    L = e.addVar('Lvar',Ltype)
    print("Derivative of list scalar access w.r.t. uneven list:",L[0].deriv(L),"(should be [1,[0,0,0]])")
    global _DEBUG_DERIVATIVES
    _DEBUG_DERIVATIVES = True
    print("Derivative of list vector access w.r.t. uneven list:",L[1].deriv(L),"(should be [[0,1,0,0],[0,0,1,0],[0,0,0,1]])")
    _DEBUG_DERIVATIVES = False
    #raw_input("Press enter to continue...")

def _run_subs_test():
    print("2*i with i=3.5:",subs(const(2)*"i","i",3.5))
    print("  evaluates to",subs(const(2)*"i","i",3.5).eval())
    ctx = Context()
    x = ctx.addVar("x",'N')
    print("map(x**i with i in range(3))",map_(x**"i","i",range_(3)))
    print("  evaluates to",map_(x**"i","i",range_(3)).eval())
    print("  with x=2 evalutes to",map_(x**"i","i",range_(3)).eval({'x':2}))
    print("  stacked (simplfied)",simplify(flatten(map_(x**"i","i",range_(3)).eval())))
    print("summation(x**i with i in range(3))",summation(x**"i","i",range_(3)))
    print("  evaluates to",summation(x**"i","i",range_(3)).eval())
    print("  evaluates to (simplified)",simplify(summation(x**"i","i",range_(3)).eval()))
    print("  with x=2 evalutes to",summation(x**"i","i",range_(3)).eval({'x':2}))
    N = ctx.addVar("N",'N')
    print("summation((x+x)**i with i in range(N))",summation((x+x)**"i","i",range_(N)))
    print("  simplifies to",simplify(summation((x+x)**"i","i",range_(N))))
    print("  with x=2 evalutes to",summation((x+x)**"i","i",range_(N)).eval({'x':2}))

if __name__ == '__main__':
    print("Beginning self-test")
    _run_basic_test()
    _run_function_test()
    #_run_print_test()
    _run_simplify_test()
    #_run_io_test()
    _run_jacobian_test()
    #_run_subs_test()
