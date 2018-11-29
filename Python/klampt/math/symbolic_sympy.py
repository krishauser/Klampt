from symbolic import *
from symbolic import _column_stack,_row_stack,_builtin_functions
import sympy
from sympy.matrices import Matrix
from sympy.core.sympify import sympify
from sympy.utilities.lambdify import implemented_function
import operator

def _sympy_zeros(args,sargs):
    return sympy.zeros(*sargs[0])

def _sympy_diag(args,sargs):
    vecmat = args[0]
    if vecmat.returnType().type in ['N','V']:
        return sympy.diag(*sargs[0])
    elif vecmat.returnType().type == 'M':
        raise NotImplementedError("TODO: Get diagonal of matrix in Sympy")
    else:
        raise ValueError("Unknown return type from symbolic Expression")

def _sympy_dot(args,sargs):
    a,b = args
    sa,sb = sargs
    if a.returnType().type == 'V' and b.returnType().type == 'V':
        return (sa.T*sb)[0]
    return sa*sb

def _sympy_transpose(args,sargs):
    return sargs[0].T

def _sympy_list(args,sargs):
    return np.array(sargs,dtype=np.object)

def _sympy_column_stack(args,sargs):
    sargs = [sa.tolist() if isinstance(sa,Matrix) else sa for sa in sargs]
    return sympy.Matrix(_column_stack(*sargs))

def _sympy_row_stack(args,sargs):
    sargs = [sa.tolist() if isinstance(sa,Matrix) else sa for sa in sargs]
    #print "Rows:",sargs
    return sympy.Matrix(_row_stack(*sargs))

def _sympy_summation(args,sargs):
    expr,var,vrange = args
    sexpr,svar,srange = sargs
    if is_op(vrange,'range'):
        start = 0
        stop = exprToSympy(vrange.args[0])
    elif is_const(vrange):
        vcrange = to_const(vrange)
        assert hasattr(vcrange,'__iter__')
        if len(vcrange) == 0:
            return sympy.Sum(sexpr,(svar,0,1))
        if not all(isinstance(v) for v in vcrange):
            raise ValueError("Unable to perform summation of non-integer range")
        for i,v in enumerate(vcrange):
            if v != vcrange[0] + i:
                raise ValueError("Unable to perform summation of non-contiguous range")
        start = vcrange[0]
        stop = start + len(vcrange)
    else:
        raise ValueError("Not yet able to perform summation over non-range objects... try simplifying expression")
    return sympy.Sum(sexpr,(svar,start,stop))

def _sympy_weightedsum(args,sargs):
    if len(args) == 0: return 0
    vals = sargs[:len(sargs)/2]
    weights = sargs[len(sargs)/2:]
    res = vals[0]*weights[0]
    for (v,w) in zip(vals,weights)[1:]:
        res += v*w
    return res

_sympyOperators = set(['neg','add','sub','mul','div','pow',
    'and','or','not','le','ge','getitem'])

_sympySpecialConstructors = {
    'zeros': _sympy_zeros,
    'diag': _sympy_diag,
    'dot': _sympy_dot,
    'transpose': _sympy_transpose,
    'row_stack': _sympy_row_stack,
    'column_stack': _sympy_column_stack,
    'list': _sympy_list,
    'sum':lambda args,sargs:sympy.Add(*sargs),
    'weightedsum':_sympy_weightedsum,
    'summation':_sympy_summation,
}

def exprToSympy(expr):
    if isinstance(expr,Variable):
        if expr.type.is_scalar():
            return sympy.Symbol(expr.name)
        elif expr.type.type in ARRAY_TYPES: 
            #1D column vector
            assert expr.type.size is not None,"Can't convert variable-sized arrays to Sympy Matrix's"
            entries = sympy.symarray(expr.name,expr.type.size)
            return sympy.Matrix(entries)
        else: 
            raise ValueError("Invalid Variable")
    elif isinstance(expr,VariableExpression):
        return exprToSympy(expr.var)
    elif isinstance(expr,UserDataExpression):
        return sympy.Symbol(expr.name)
    elif isinstance(expr,ConstantExpression):
        return exprToSympy(expr.value)
    elif isinstance(expr,OperatorExpression):
        fname = expr.functionInfo.name
        sname = fname.capitalize()
        sargs = [exprToSympy(a) for a in expr.args]
        if fname in _sympySpecialConstructors:
            return _sympySpecialConstructors[fname](expr.args,sargs)
        if fname in _sympyOperators:
            try:
                return getattr(operator,fname)(*sargs)
            except Exception as e:
                print "exprToSympy: Error raised while performing operator %s on arguments %s"%(fname,str(expr))
                raise
        if hasattr(sympy,sname):
            #try capitalized version first
            try:
                return getattr(sympy,sname)(*sargs)
            except Exception:
                print "exprToSympy: Error raised while trying sympy.%s on arguments %s"%(sname,str(expr))
        if hasattr(sympy,fname):
            #numpy equivalents, like eye
            try:
                return getattr(sympy,fname)(*sargs)
            except Exception:
                print "exprToSympy: Error raised while trying sympy.%s on arguments %s"%(fname,str(expr))
        if callable(expr.functionInfo.func):
            print "exprToSympy: Function %s does not have Sympy equivalent, returning adaptor "%(fname,)
            return _make_sympy_adaptor(expr.functionInfo)(*sargs)
        else:
            print "exprToSympy: Function %s does not have Sympy equivalent, expanding expression"%(fname,)
            assert isinstance(expr.functionInfo.func,Expression)
            sfunc = exprToSympy(expr.functionInfo.func)
            return sfunc.subs(zip(expr.functionInfo.argNames,sargs))
        print "exprToSympy: Function %s does not have Sympy equivalent, returning generic Function"%(fname,)
        return sympy.Function(fname)(*sargs)
    else:
        if hasattr(expr,'__iter__'):
            if hasattr(expr[0],'__iter__'):
                #Matrix
                assert not hasattr(expr[0][0],'__iter__'),"Sympy can't handle tensors yet"
                return sympy.Matrix(expr)
            else:
                #1-D vector -- treat as column vector
                return sympy.Matrix(expr)
        if isinstance(expr,(float,int,bool)):
            return sympify(expr)

def _make_sympy_adaptor(func):
    """Adapts a symbolic Function to a sympy Function"""
    assert isinstance(func,Function)
    def _eval_evalf(self,prec):
        fargs = [a._to_mpmath(prec) for a in self.args]
        res = self._symbolic_func(*fargs).evalf()
        return sympy.S(res)
    def fdiff(self, argindex):
        from sympy.core.function import ArgumentIndexError
        f = self._symbolic_func
        if f.deriv is None: 
            raise ArgumentIndexError(self, argindex)
        if f.deriv is 0:
            return S(0)
        argindex -= 1
        if f.jacobian is not None and f.jacobian[argindex] is not None:
            assert isinstance(f.jacobian[argindex],Function)
            return make_sympy_adaptor(f.jacobian[argindex])(*self.args)
        if callable(f.deriv):
            raise NotImplementedError("Can't adapt a callable derivative to sympy yet")
        assert argindex >= 0 and argindex < len(f.deriv),"Invalid derivative argument index? 0 <= %d < %d"%(argindex,len(f.deriv))
        if f.deriv[argindex] is 0:
            return S(0)
        if f.deriv[argindex] is None:
            raise ArgumentIndexError(self, argindex)
        return _make_sympy_adaptor(f.deriv[argindex])(*(self.args+(1,)))

    attributes = {
        '_symbolic_func':func,
        '_eval_evalf':_eval_evalf,
        'fdiff':fdiff
    }
    if func.argNames is not None:
        attributes['nargs'] = len(func.argNames)
    return type(func.name+"_sympy_adaptor",(sympy.Function,),attributes)
    
class SympyFunction(Function):
    """Defines a Function from a Sympy expression.

    Example:
    x,y = sympy.symbols("x y")
    twoxy = SympyFunction("twoxy",2*x*y)
    """
    def __init__(self,name,expr,symbol_order=None):
        """
        - name: the symbolic module name of the function.
        - expr: the Sympy function
        - symbol_order: if you don't want to use lexicographical order for the unbound variables in expr,
          this will contain the desired argument order.
        """
        if symbol_order is None:
            symbol_order = sorted([s.name for s in expr.free_symbols])
        else:
            if not all(v.name in set(symbol_order) for v in expr.free_symbols):
                raise ValueError("symbol_order does not contain some free variables: %s vs %s"%(str(symbol_order),str([s.name for s in expr.free_symbols])))

        func = (lambda *args: expr.evalf(subs=dict(zip(symbol_order,args))))
        Function.__init__(self,name,func,symbol_order)
        self.sympy_expr = expr

        self.deriv = [None]*len(self.argNames)
        self.jacobian = [None]*len(symbol_order)
        self.sympy_jacobian = [None]*len(symbol_order)
        self.sympy_jacobian_funcs = [None]*len(symbol_order)
        if isinstance(expr,Matrix):
            for i,arg in enumerate(symbol_order):
                self.sympy_jacobian[i] = expr.diff(arg)
        else:
            for i,arg in enumerate(symbol_order):
                self.sympy_jacobian[i] = sympy.diff(expr,arg)
        for i in xrange(len(symbol_order)):
            def cache_jacobian(*args):
                if self.sympy_jacobian_funcs[i] is None:
                    #print "Creating jacobian function",name + "_jac_" + symbol_order[i]
                    self.sympy_jacobian_funcs[i] = SympyFunction(name + "_jac_" + symbol_order[i], self.sympy_jacobian[i],symbol_order)
                return OperatorExpression(self.sympy_jacobian_funcs[i],args)
            self.jacobian[i] = cache_jacobian

class SympyFunctionAdaptor(Function):
    """Defines a Function from a Sympy function.

    Example:
    heaviside = SympyFunctionAdaptor("heaviside",sympy.Heaviside,["x"])
    """
    def __init__(self,name,func,argnames=None):
        """
        - name: the symbolic module name of the function.
        - func: the Sympy function
        - argnames: provided if you don't want to use 'x','y', 'z' for the argument names.
        """
        assert isinstance(func,sympy.FunctionClass)
        if hasattr(func,'nargs'):
            if len(func.nargs) > 1:
                print "SympyFunctionAdaptor: can't yet handle multi-argument functions"
            nargs = None
            for i in func.nargs:
                nargs = i
        else:
            nargs = 1
        if argnames is None:
            if nargs == 1:
                argnames = ['x']
            elif nargs <= 3:
                argnames = [['x','y','z'][i] for i in len(nargs)]
            else:
                argnames = ['arg'+str(i+1) for i in len(nargs)]
        Function.__init__(self,name,func,argnames)

        self.deriv = [None]*len(argnames)
        self.jacobian = [None]*len(argnames)
        self.sympy_jacobian = [None]*len(argnames)
        self.sympy_jacobian_funcs = [None]*len(argnames)

        xs = sympy.symarray('x',nargs)
        for i,arg in enumerate(argnames):
            self.sympy_jacobian[i] = func(*xs).diff(xs[i])
        
        for i in xrange(len(argnames)):
            def cache_jacobian(*args):
                if self.sympy_jacobian_funcs[i] is None:
                    #print "Creating jacobian function",name + "_jac_" + argnames[i]
                    self.sympy_jacobian_funcs[i] = SympyFunction(name + "_jac_" + argnames[i], self.sympy_jacobian[i],argnames)
                return OperatorExpression(self.sympy_jacobian_funcs[i],args)
            self.jacobian[i] = cache_jacobian

def exprFromSympy(context,sexpr,addFuncs=True):
    """Converts a Sympy expression to a symbolic.py expression.
    - context: a Context object that captures variable references and custom functions. This may be None.
    - sexpr: the Sympy expression.
    - addFuncs: if True, any Sympy functions without a direct match to symbolic functions are added to
      sexpr's customFunctions list.
    """
    if isinstance(sexpr,sympy.Symbol):
        if context is not None and sexpr.name in context.variableDict:
            return VariableExpression(context.variableDict[sexpr.name])
        return UserDataExpression(sexpr.name)
    elif isinstance(sexpr,Matrix):
        rows,cols = sexpr.rows,sexpr.cols
        sentries = sexpr._mat
        if cols == 1: #interpret as a vector
            entries = [exprFromSympy(context,s) for s in sentries]
            return stack(*entries)
        else:
            #it's a matrix
            k = 0
            erows = []
            for row in xrange(rows):
                erows.append(sentries[k:k+cols])
                k += cols
            return row_stack(*erows)
    elif isinstance(sexpr,sympy.Atom):
        if isinstance(sexpr,sympy.Integer):
            return int(sexpr)
        return float(sexpr)
    elif isinstance(sexpr,(sympy.Function,sympy.Basic)):
        #expression
        sname = sexpr.__class__.__name__
        fname = sname.lower()
        args = [exprFromSympy(context,s) for s in sexpr.args]
        if context is None:
            if fname not in _builtin_functions:
                print "exprFromSympy: Sympy function %s does not have symbolic.py equivalent, creating adaptor with name %s"%(sname,fname)
                f = SympyFunctionAdaptor(fname,sexpr.func)
                #raise ValueError("Unknown Sympy function %s"%(sname,))
            else:
                f = _builtin_functions[fname]
        else:
            try:
                f = context.function(fname)
            except KeyError as e:
                print "exprFromSympy: Sympy function %s does not have symbolic.py equivalent, creating adaptor with name %s"%(sname,fname)
                f = SympyFunctionAdaptor(fname,sexpr.func)
                if addFuncs:
                    context.declare(f)
                #raise ValueError("Unknown Sympy function %s"%(sname,))
        if f.argNames is not None and len(args) != len(f.argNames):
            if len(f.argNames) == 2 and f.properties.get('associative',False):
                #can cascade arguments
                if len(args) == 1:
                    raise ValueError("Invalid number of arguments to function %s"%(fname,))
                res = f(args[0],args[1])
                for a in args[2:]:
                    res = f(res,a)
                return res
        return f(*args)
    else:
        raise ValueError("Can't convert Sympy object %s to symbolic Expression"%(sexpr.__class__.__name__,))
