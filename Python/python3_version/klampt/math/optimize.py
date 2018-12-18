import numpy as np
import math,random
from . import symbolic,symbolic_io,symbolic_linalg
from ..io import loader

class OptimizationProblem:
    """A holder for optimization problem data.  All attributes are optional,
    and some solvers can't handle certain types of constraints and costs.

    The objective function must return a float. All equality and inequality
    functions are required to return a list of floats.
    
    - objective: an objective function f(x)
    - objectiveGrad: a function df/dx(x) giving the gradient of f.
    - bounds: a pair (l,u) giving lower and upper bounds on the search space.
    - equalities: a list of functions g(x)=0 required of a feasible solution.
      In practice, |g(x)| <= tol is required.
    - equalityGrads: a list of gradient/Jacobian functions dg/dx(x) of the 
      equality functions.
    - inequalities: a list of functions h(x)<=0 required of a feasible
      solution.
    - inequalityGrads: a list of gradient/Jacobian functions dh/dx(x) of the 
      inequality functions.
    - feasibilityTests: a list of boolean black-box predicates that must be true 
      of the solution

    Suitable for use with the symbolic module.  Once a Context is created, and
    appropriate Variables, Functions, Expressions are declared, the
    setSymbolicObjective and addSymbolicConstraint methods automatically determine
    the standard Python function forms. I.e., context.makeFlatFunction(f,varorder)
    where varorder = None for the default variable ordering. 

    The OptimizationProblemBuilder class is more closely tied with the symbolic
    module and is more convenient to use.  It performs automatic simplification
    and differentiation, and can be saved / loaded to disk.
    """
    def __init__(self):
        self.objective = None
        self.objectiveGrad = None
        self.bounds = None
        self.equalities = []
        self.inequalities = []
        self.equalityGrads = []
        self.inequalityGrads = []
        self.feasibilityTests = []
    def setObjective(self,func,funcGrad=None):
        self.objective = func
        self.objectiveGrad = funcGrad
    def addEquality(self,func,funcGrad=None):
        self.equalities.append(func)
        self.equalityGrads.append(funcGrad)
    def addInequality(self,func,funcGrad=None):
        self.inequalities.append(func)
        self.inequalityGrads.append(funcGrad)
    def setBounds(self,xmin,xmax):
        self.bounds = (xmin,xmax)
    def setFeasibilityTest(self,test):
        self.feasibilityTests = [test]
    def addFeasibilityTest(self,test):
        self.feasibilityTests.append(test)
    def setSymbolicObjective(self,func,context,varorder=None):
        """Sets an objective function from a symbolic Function or Expression
        (see symbolic module)."""
        if varorder is None: varorder = context.variables
        fpy,varorder = context.makeFlatFunction(func,varorder)
        dfpy,varorder = context.makeFlatFunctionDeriv(func,varorder)
        self.setObjective(fpy,dfpy)
    def addSymbolicConstraint(self,func,context,varorder=None,blackbox=False):
        """adds a constraint from a symbolic Function or Expression
        (see symbolic module).  This will be "smart" in that AND Expressions will be
        converted to multiple constraints, inequalities will be converted to inequality
        constraints, and bounds will be converted to bound constraints.  All other
        constraints will be treated as feasibility constraints"""
        if varorder is None: varorder = context.variables
        if symbolic.is_op(func,"and"):
            for a in func.args:
                self.addSymbolicConstraint(self,a,context,varorder)
        elif symbolic.is_op(func,"le"):
            if symbolic.is_var(func.args[0]) and symbolic.is_const(func.args[1]):
                #x <= c
                x = symbolic.to_var(func.args[0])
                xmax = symbolic.to_const(func.args[1])
                indices = context.getFlatVarRanges(varorder)
                xindex = [i for i,v in enumerate(varorder) if v.name == x.name][0]
                ai,bi = indices[xindex],indices[xindex+1]
                n = indices[-1]
                if self.bounds is None:
                    self.bounds = (np.array([-float('inf')]*n),np.array([float('inf')]*n))
                self.bounds[1][ai:bi] = np.minimum(self.bounds[1][ai:bi],xmax)
            elif symbolic.is_var(func.args[1]) and symbolic.is_const(func.args[0]):
                #c <= x
                xmin = symbolic.to_const(func.args[0])
                x = symbolic.to_var(func.args[1])
                indices = context.getFlatVarRanges(varorder)
                xindex = [i for i,v in enumerate(varorder) if v.name == x.name][0]
                ai,bi = indices[xindex],indices[xindex+1]
                n = indices[-1]
                if self.bounds is None:
                    self.bounds = (np.array([-float('inf')]*n),np.array([float('inf')]*n))
                self.bounds[0][ai:bi] = np.maximum(self.bounds[0][ai:bi],a)
            else:
                h = symbolic.simplify(func.args[0]-func.args[1])
                if func.args[0].returnType().is_scalar() and func.args[1].returnType().is_scalar():
                    #need to convert to a vector
                    h = symbolic.flatten(h)
                hpy,varorder = context.makeFlatFunction(h,varorder)
                dhpy,varorder = context.makeFlatFunctionDeriv(h,varorder)
                self.addInequality(hpy,dhpy)
        elif symbolic.is_op(func,"ge"):
            c = (func.args[1] <= func.args[0])
            self.addSymbolicConstraint(c,context,varorder)
        elif symbolic.is_op(func,"eq"):
            g = symbolic.simplify(func.args[0]-func.args[1])
            if func.args[0].returnType().is_scalar() and func.args[1].returnType().is_scalar():
                #need to convert to a vector
                g = symbolic.flatten(g)
            gpy,varorder = context.makeFlatFunction(g,varorder)
            dgpy,varorder = context.makeFlatFunctionDeriv(g,varorder)
            self.addEquality(gpy,dgpy)
        elif symbolic.is_op(func):
            if func.functionInfo is symbolic_linalg.bound_contains and symbolic.is_const(func.args[0]) and symbolic.is_const(func.args[1]) and symbolic.is_var(func.args[2]):
                #bound constraint
                xmin = symbolic.to_const(func.args[0])
                xmax = symbolic.to_const(func.args[1])
                x = symbolic.to_var(func.args[2])
                indices = context.getFlatVarRanges(varorder)
                xindex = [i for i,v in enumerate(varorder) if v.name == x.name][0]
                ai,bi = indices[xindex],indices[xindex+1]
                n = indices[-1]
                if self.bounds is None:
                    self.bounds = ([-float('inf')]*n,[float('inf')]*n)
                for i,a,b in zip(list(range(ai,bi)),xmin,xmax):
                    self.bounds[0][i] = max(self.bounds[0][i],a)
                    self.bounds[1][i] = min(self.bounds[1][i],b)
            else:
                #it's a generic boolean
                if not blackbox:
                    print("OptimizationProblem.addSymbolicConstraint(): Warning, turning function",func,"into black box function")
                fpy,varorder = context.makeFlatFunction(func,varorder)
                self.addFeasibilityTest(fpy)
        else:
            #it's a generic boolean
            if not blackbox:
                print("OptimizationProblem.addSymbolicConstraint(): Warning, turning function",func,"into black box function")
            fpy,varorder = context.makeFlatFunction(func,varorder)
            self.addFeasibilityTest(fpy)
    def objectiveValue(self,x):
        """Returns the objective function value f(x)."""
        return self.objective(x)
    def feasible(self,x,equalityTol=1e-6):
        """Returns true if x is a feasible point."""
        for g in self.equalities:
            gx = g(x)
            if any(abs(v) > equalityTol for v in gx): return False
        for h in self.inequalities:
            hx = h(x)
            if any(v > 0 for v in hx): return False
        for f in self.feasibilityTests:
            if not f(x): return False
        return True
    def equalityResidual(self,x):
        """Returns the stacked vector g(x) where g(x)=0 is the equality constraint."""
        if len(self.equalities) == 0: return []
        return np.hstack([g(x) for g in self.equalities])
    def inequalityResidual(self,x):
        """Returns the stacked vector h(x) where h(x)<=0 is the inequality constraint."""
        if len(self.inequalities) == 0: return []
        return np.hstack([h(x) for h in self.inequalities])
    def makeUnconstrained(self,objective_scale,keep_bounds=True):
        """If this problem is constrained, returns a new problem in which
        the objective function is a scoring function that sums all of
        the equality / inequality errors at x plus
        objective_scale*objective function(x).  If objective_scale is small,
        then the scoring function is approximately minimized at a feasible
        minimum.

        If the problem is unconstrained, this just returns self.

        If keep_bounds = true, this does not add the bounds to the
        inequality errors.
        """
        #create a scoring function that is approximately minimized at
        #a feasible minimum
        if keep_bounds == False:
            raise NotImplementedError("getting rid of bounds is not implemented yet")
        if len(self.feasibilityTests) == 0 and len(self.inequalities) == 0 and len(self.equalities) == 0:
            #already unconstrained
            return self

        if len(self.inequalities) == 0 and len(self.equalities) == 0:
            #just have a feasibility test
            def flatObjective(x):
                if any(not f(x) for f in self.feasibilityTests):
                    return float('inf')
                return self.objective(x)
            res = Problem()
            res.setObjective(flatObjective,self.objectiveGrad)
            res.bounds = self.bounds
            return res

        def flatObjective(x):
            if any(not f(x) for f in self.feasibilityTests):
                return float('inf')
            f = 0
            #add sum of squared equalities
            for g in self.equalities:
                gx = g(x)
                f += max(abs(v) for v in gx)
            for h in self.inequalities:
                hx = h(x)
                f += sum(max(v,0) for v in hx)
            if self.objective is not None:
                f += objective_scale*self.objective(x)
            return f
        
        res = Problem()
        res.setObjective(flatObjective,None)
        res.bounds = self.bounds
        return res

class LocalOptimizer:
    """A wrapper around different local optimization libraries. Only
    minimization is supported, and only scipy and pyOpt are supported.
    
    The method is specified using the method string, which can be:
    - auto: picks between scipy and pyOpt, whatever is available.
    - scipy: uses scipy.optimize.minimize with default settings.
    - scipy.[METHOD]: uses scipy.optimize.minimize with the argument
      method=[METHOD].
    - pyOpt: uses pyOpt with SLSQP.
    - pyOpt.[METHOD]: uses pyOpt with the given method.
    """
    def __init__(self,method='auto'):
        if method == 'auto':
            try:
                import pyopt
                method = 'pyOpt'
            except ImportError:
                method = 'scipy'

        self.method = method
        self.seed = None

    @staticmethod
    def methodsAvailable():
        """Returns a list of methods that are available on this system"""
        methods = []
        try:
            import pyOpt
            methods.append('pyOpt')
            pyoptmethods = ['SLSQP','PSQP','SNOPT','COBYLA','NLPQL','NLPQLP','MMA','GCMMA','KSOPT']
            for m in pyoptmethods:
                try:
                    x = getattr(pyOpt,m)
                    methods.append('pyOpt.'+m)
                except AttributeError:
                    pass
        except ImportError:
            pass
        try:
            import scipy
            methods.append('scipy')
            methods.append('scipy.Nelder-Mead')
            methods.append('scipy.Powell')
            methods.append('scipy.CG')
            methods.append('scipy.BFGS')
            methods.append('scipy.TNC')
            methods.append('scipy.COBYLA')
            methods.append('scipy.L-BFGS-B')
            methods.append('scipy.SLSQP')
        except ImportError:
            pass
        return methods

    @staticmethod
    def methodsAppropriate(problem):
        """Returns a list of available methods that are appropriate to use for the given problem"""
        allmethods = LocalOptimizer.methodsAvailable()
        if len(problem.inequalities) > 0 or len(problem.equalities) > 0:
            #can only do SLSQP, PSQP, and SNOPT
            methods = []
            for m in allmethods:
                if m=='scipy' or m=='pyOpt' or m.endswith('SQP') or m.endswith('SNOPT'):
                    methods.append(m)
            return methods
        elif problem.bounds is not None:
            #only can do bounded problems
            for m in LocalOptimizer.methodsAvailable():
                if m=='scipy' or m=='pyOpt':
                    methods.append(m)
                else:
                    if not any(m.endswith(x) for x in ['Nelder-Mead','Powell','CG','BFGS']):
                        methods.append(m)
            return methods
        else:
            return allmethods

    def setSeed(self,x):
        self.seed = x

    def solve(self,problem,numIters=100,tol=1e-6):
        """Returns a tuple (success,result)"""
        if self.seed is None:
            raise RuntimeError("Need to provide a seed state")
        if problem.objective is None:
            raise RuntimeError("Need to provide an objective function")
        if self.method.startswith('scipy'):
            from scipy import optimize
            items = self.method.split('.')
            scipyMethod = 'SLSQP'
            if len(items)>1:
                scipyMethod = items[1]
            jac = False
            if problem.objectiveGrad:
                jac = problem.objectiveGrad
            bounds = None
            if problem.bounds:
                bmin = [v if not math.isinf(v) else None for v in problem.bounds[0]]
                bmax = [v if not math.isinf(v) else None for v in problem.bounds[1]]
                bounds = list(zip(bmin,bmax))
            constraintDicts = []
            for i in range(len(problem.equalities)):
                constraintDicts.append({'type':'eq','fun':problem.equalities[i]})
                if problem.equalityGrads[i] is not None:
                    constraintDicts[-1]['jac'] = problem.equalityGrads[i]
            for i in range(len(problem.inequalities)):
                #scipy asks for inequalities to be positive g(x) >= 0, which requires a flip of sign
                constraintDicts.append({'type':'ineq','fun':lambda x:-np.array(problem.inequalities[i](x))})
                if problem.inequalityGrads[i] is not None:
                    constraintDicts[-1]['jac'] = lambda x:-np.array(problem.inequalityGrads[i](x))
            if len(constraintDicts) > 0 and scipyMethod not in ['SLSQP','COBYLA']:
                print("LocalOptimizer.solve(): warning, can't use method",scipyMethod,"with constraints")
                input("Press enter to continue > ")
            #print "Scipy constraints",constraintDicts
            #print "Scipy bounds",bounds
            #print "Objective jacobian",jac
            res = optimize.minimize(problem.objective,x0=self.seed,method=scipyMethod,
                                    jac=jac,bounds=bounds,
                                    constraints=constraintDicts,tol=tol,options={'maxiter':numIters,'disp':True})
            if res.success:
                print("***********************************************************")
                print("LocalOptimizer.solve(): Scipy solver result",res.message)
                print(res)
                x = res.x
                print("My objective value:",problem.objective(x))
                if len(problem.equalities) > 0:
                    h = np.hstack([f(x) for f in problem.equalities])
                else:
                    h = [0]
                if len(problem.inequalities) > 0:
                    g = np.hstack([f(x) for f in problem.inequalities])
                else:
                    g = [0]
                boundfeasible = all(a<=v and v<=b for v,a,b in zip(x,problem.bounds[0],problem.bounds[1])) if problem.bounds is not None else True
                eqfeasible = all(abs(v)<tol for v in h)
                ineqfeasible = all(v<=0 for v in g)
                feasible = eqfeasible and ineqfeasible and boundfeasible
                if not feasible:
                    if not boundfeasible:
                        #try clamping
                        for i in range(len(x)):
                            x[i] = min(max(x[i],problem.bounds[0][i]),problem.bounds[1][i])
                        boundfeasible = True
                        if len(problem.equalities) > 0:
                            h = np.hstack([f(x) for f in problem.equalities])
                        else:
                            h = [0]
                        if len(problem.inequalities) > 0:
                            g = np.hstack([f(x) for f in problem.inequalities])
                        else:
                            g = [0]
                        eqfeasible = all(abs(v)<tol for v in h)
                        ineqfeasible = all(v<=0 for v in g)
                        feasible = eqfeasible and ineqfeasible and boundfeasible
                        print("LocalOptimizer: solution not in bounds, clamped.")
                        print("  Bound-corrected equality residual",h)
                        print("  Bound-corrected inequality residual",g)
                        print("  Feasible?",eqfeasible,ineqfeasible)
                if not feasible:
                    print("LocalOptimizer: Strange, Scipy optimizer says successful and came up with an infeasible solution")
                    if not eqfeasible:
                        print("  Equality has max residual",max(abs(v) for v in h),"> tolerance",tol)
                        print("  Residual vector",h)
                    if not ineqfeasible:
                        print("  Inequality has residual",max(v for v in h),"> 0")
                        print("  Residual vector",g)
                    if not boundfeasible:
                        for i,(v,a,b) in enumerate(zip(x,problem.bounds[0],problem.bounds[1])):
                            if v < a or v > b:
                                print("  Bound %d: %f <= %f <= %f violated"%(i,a,v,b))
                    input("Press enter to continue >")
                print("***********************************************************")
            return res.success,res.x.tolist()
        elif self.method.startswith('pyOpt'):
            import pyOpt
            import warnings
            warnings.filterwarnings("ignore", category=DeprecationWarning) 
            items = self.method.split('.')
            pyOptMethod = 'SLSQP'
            if len(items)>1:
                pyOptMethod = items[1]

            if problem.bounds is not None:
                bmin = np.array(problem.bounds[0][:])
                bmax = np.array(problem.bounds[1][:])
                #for some reason PyOpt doesn't do well with infinite bounds
                for i,v in enumerate(problem.bounds[0]):
                    if math.isinf(v): bmin[i] = -1e20
                for i,v in enumerate(problem.bounds[1]):
                    if math.isinf(v): bmax[i] = 1e20
                ubIndices = [i for i,v in enumerate(bmax) if not math.isinf(v)]
                lbIndices = [i for i,v in enumerate(bmin) if not math.isinf(v)]
            else:
                ubIndices = []
                lbIndices = []
            def objfunc(x):
                #print "EVALUATING OBJECTIVE AT",x
                fx = problem.objective(x)
                eqs = [f(x) for f in problem.equalities]+[f(x) for f in problem.inequalities]
                if len(eqs) == 0:
                    gx = []
                else:
                    gx = np.hstack(eqs)
                    assert len(gx.shape)==1
                    gx = gx.tolist()
                if problem.bounds is not None:
                    ub = (x-bmax)[ubIndices]
                    lb = (bmin-x)[lbIndices]
                    if len(gx) == 0:
                        gx = ub.tolist() + lb.tolist()
                    else:
                        gx = gx + ub.tolist() + lb.tolist()
                #for f in problem.equalities:
                #    print "EQUALITY VALUE",f(x)
                #for f in problem.inequalities:
                #    print "INEQUALITY VALUE",f(x)
                flag = not any(not f(x) for f in problem.feasibilityTests)
                #print "CONSTRAINTS",gx
                #print "FUNCTION VALUE IS",fx
                assert len(gx) == hlen+glen+len(ubIndices)+len(lbIndices)
                flag = True
                if any(math.isnan(v) for v in x):
                    return 0,[0]*len(gx),flag
                return fx,gx,flag
            opt_prob = pyOpt.Optimization('',objfunc)
            opt_prob.addObj('f')
            for i in range(len(self.seed)):
                if problem.bounds is not None:
                    opt_prob.addVar('x'+str(i),'c',lower=bmin[i],upper=bmax[i],value=self.seed[i])
                else:
                    opt_prob.addVar('x'+str(i),'c',value=self.seed[i])
            hlen = sum(len(f(self.seed)) for f in problem.equalities)
            glen = sum(len(f(self.seed)) for f in problem.inequalities)
            opt_prob.addConGroup('eq',hlen,'e')
            opt_prob.addConGroup('ineq',glen,'i')
            #expressing bounds as inequalities
            opt_prob.addConGroup('bnd',len(ubIndices)+len(lbIndices),'i')

            opt = getattr(pyOpt,pyOptMethod)()
            #opt.setOption('IPRINT', -1)
            opt.setOption('IPRINT', -2)
            opt.setOption('MAXIT',numIters)
            opt.setOption('ACC',tol)
            sens_type = 'FD'
            if problem.objectiveGrad is not None:
                #user provided gradients
                if all(f is not None for f in problem.equalityGrads) and all(f is not None for f in problem.inequalityGrads):
                    #print "RETURNING GRADIENTS"
                    def objfuncgrad(x):
                        fx = problem.objectiveGrad(x)
                        gx = sum([f(x) for f in problem.equalityGrads]+[f(x) for f in problem.inequalityGrads],[])
                        for i in ubIndices:
                            zero = [0]*len(x)
                            zero[i] = 1
                            gx.append(zero)
                        for i in lbIndices:
                            zero = [0]*len(x)
                            zero[i] = -1
                            gx.append(zero)
                        flag = True
                        return fx,gx,flag
                    sens_type = objfuncgrad
                else:
                    print("LocalOptimizer.solve(): Warning, currently need all or no gradients provided. Assuming no gradients.")
            [fstr, xstr, inform] = opt(opt_prob,sens_type=sens_type)
            if inform['value'] != 0:
                return False,xstr.tolist()
            f,g,flag = objfunc(xstr)
            #flag doesn't check?
            eqfeasible = all(abs(v)<tol for v in g[:hlen])
            ineqfeasible = all(v <= 0 for v in g[hlen:hlen+glen])
            boundfeasible = all(a<=x and x<=b for x,a,b in zip(xstr,problem.bounds[0],problem.bounds[1])) if problem.bounds is not None else True
            feasible = eqfeasible and ineqfeasible and boundfeasible
            if not feasible:
                if not boundfeasible:
                    #try clamping
                    for i in range(len(xstr)):
                        xstr[i] = min(max(xstr[i],bmin[i]),bmax[i])
                    f,g,flag = objfunc(xstr)
                    boundfeasible = True
                    eqfeasible = all(abs(v)<tol for v in g[:hlen])
                    ineqfeasible = all(v <= 0 for v in g[hlen:hlen+glen])
                    feasible = eqfeasible and ineqfeasible and boundfeasible
                if not feasible:
                    print("LocalOptimizer: Strange, pyOpt optimizer says successful and came up with an infeasible solution")
                    h = g[:hlen]
                    g = g[hlen:hlen+glen]
                    if not eqfeasible:
                        print("  Equality has max residual",max(abs(v) for v in h),"> tolerance",tol)
                        print("  Residual vector",h)
                    if not ineqfeasible:
                        print("  Inequality has residual",max(v for v in h),"> 0")
                        print("  Residual vector",g)
                    if not boundfeasible:
                        for i,(v,a,b) in enumerate(zip(x,bmin,bmax)):
                            if v < a or v > b:
                                print("  Bound %d: %f <= %f <= %f violated"%(i,a,v,b))
                    input("Press enter to continue >")
            return feasible,xstr.tolist()
        else:
            raise RuntimeError('Invalid method specified: '+self.method)

def sample_range(a,b):
    """Samples x in the range [a,b].
    If the range is bounded, the uniform distribution x~U(a,b) is used.
    If the range is unbounded, then this uses the log transform to sample a distribution.
    Specifically, if a=-inf and b is finite, then x ~ b + log(y) where y~U(0,1).  A similar
    formula holds for a finite and b=inf.
    If a=-inf and b=inf, then x ~ s*log(y), where y~U(0,1) and the sign s takes on {-1,1} each
    with probability 0.5.
    """
    x = random.uniform(a,b)
    if math.isinf(x) or math.isnan(x):
        try:
            if math.isinf(a):
                if math.isinf(b):
                    s = math.randint(0,1)*2-1
                    y = math.log(random.random())
                    return s*y
                else:
                    y = math.log(random.random())
                    return b + y
            elif math.isinf(b):
                y = math.log(random.random())
                return a - y
        except ValueError:
            #very, very small chance of this happening (2^-48)
            return sample_range(a,b)
    return x


class GlobalOptimizer:
    """A wrapper around different global optimization libraries. Only
    minimization is supported, and only DIRECT, scipy, and pyOpt are supported.
    
    The optimization technique is specified using the method string, which can be:
    - 'auto': picks between DIRECT and random-restart
    - 'random-restart.METHOD': random restarts using the local optimizer METHOD.
    - 'DIRECT': the DIRECT global optimizer
    - 'scipy': uses scipy.optimize.minimize with default settings.
    - 'scipy.METHOD': uses scipy.optimize.minimize with the argument
      method=METHOD.
    - 'pyOpt': uses pyOpt with SLSQP.
    - 'pyOpt.METHOD': uses pyOpt with the given method.

    The method attribute can also be a list, which does a cascading solver
    in which the previous solution point is used as a seed for the next
    solver.

    Examples:
    - 'DIRECT': Run the DIRECT method
    - 'scipy.differential_evolution': Runs the scipy differential evolution technique
    - 'random-restart.scipy': Runs random restarts using scipy's default local optimizer
    - 'random-restart.pyOpt.SLSQP': Runs random restarts using pyOpt as a local optimizer
    - ['DIRECT','auto']: Run the DIRECT method then clean it up with the default local optimizer
            
    Random restarts picks each component x of the seed state randomly using sample_range(a,b)
    where [a,b] is the range of x given by problem.bounds.

    DIRECT and scipy.differential_evolution require a bounded state space.
    """
    def __init__(self,method='auto'):
        if method == 'auto':            
            method = 'random-restart.scipy'
        self.method = method
        self.seed = None

    def setSeed(self,x):
        self.seed = x

    def solve(self,problem,numIters=100,tol=1e-6):
        """Returns a pair (solved,x) where solved is True if the solver
        found a valid solution, and x is the solution vector."""
        if isinstance(self.method,(list,tuple)):
            #sequential solve
            seed = self.seed
            for i,m in enumerate(self.method):
                if hasattr(numIters,'__iter__'):
                    itersi = numIters[i]
                else:
                    itersi = numIters
                if hasattr(tol,'__iter__'):
                    toli = tol[i]
                else:
                    toli = tol
                print("GlobalOptimizer.solve(): Step",i,"method",m,'iters',itersi,'tol',toli)
                if m == 'auto':
                    opt = LocalOptimizer(m)
                else:
                    opt = GlobalOptimizer(m)
                #seed with previous seed, if necessary
                opt.setSeed(seed)
                (succ,xsol)=opt.solve(problem,itersi,toli)
                if not succ: return (False,xsol)
                seed = xsol[:]
            return ((seed is not None),seed)
        elif self.method == 'scipy.differential_evolution':
            from scipy import optimize
            if problem.bounds == None:
                raise RuntimeError("Cannot use scipy differential_evolution method without a bounded search space")
            flattenedProblem = problem.makeUnconstrained(objective_scale = 1e-5)
            res = optimize.differential_evolution(flattenedProblem.objective,list(zip(*flattenedProblem.bounds)))
            print("GlobalOptimizer.solve(): scipy.differential_evolution solution:",res.x)
            print("  Objective value",res.fun)
            print("  Equality error:",[gx(res.x) for gx in problem.equalities])
            return (True,res.x)
        elif self.method == 'DIRECT':
            import DIRECT
            if problem.bounds == None:
                raise RuntimeError("Cannot use DIRECT method without a bounded search space")
            flattenedProblem = problem.makeUnconstrained(objective_scale = 1e-5)
            minval = [float('inf'),None]
            def objfunc(x,userdata):
                v = flattenedProblem.objective(x)
                if v < userdata[0]:
                    userdata[0] = v
                    userdata[1] = [float(xi) for xi in x]
                return v
            (x,fmin,ierror)=DIRECT.solve(objfunc,problem.bounds[0],problem.bounds[1],eps=tol,maxT=numIters,maxf=40000,algmethod=1,user_data=minval)
            print("GlobalOptimizer.solve(): DIRECT solution:",x)
            print("  Objective value",fmin)
            print("  Minimum value",minval[0],minval[1])
            print("  Error:",ierror)
            print("  Equality error:",[gx(x) for gx in problem.equalities])
            return (True,minval[1])
        elif self.method.startswith('random-restart'):
            import random
            if problem.bounds == None:
                raise RuntimeError("Cannot use method %s without a bounded search space"%(self.method,))
            localmethod = self.method[15:]
            lopt = LocalOptimizer(localmethod)
            seed = self.seed
            best = self.seed
            print("GlobalOptimizer.solve(): Random restart seed is:",best)
            fbest = (problem.objective(best) if (best is not None and problem.feasible(best)) else float('inf'))
            for it in range(numIters[0]):
                if seed is not None:
                    x = seed
                    seed = None
                else:
                    x = [sample_range(a,b) for a,b in zip(*problem.bounds)]
                print("  Solving from",x)
                lopt.setSeed(x)
                succ,x = lopt.solve(problem,numIters[1],tol)
                print("  Result is",succ,x)
                print("  Equality:",problem.equalityResidual(x))
                if succ:
                    fx = problem.objective(x)
                    if fx < fbest:
                        fbest = fx
                        best = x
            return (best is not None, best)
        else:
            assert self.seed is not None,"Pure local optimization requires a seed to be set"
            opt = LocalOptimizer(self.method)
            opt.setSeed(self.seed)
            return opt.solve(problem,numIters,tol)

class OptimizerParams:
    def __init__(self,numIters=50,tol=1e-3,
                 startRandom=False,numRestarts=1,
                 timeout=10,globalMethod=None,localMethod=None):
        self.numIters=numIters
        self.tol=tol
        self.startRandom=startRandom
        self.numRestarts=numRestarts
        self.timeout = timeout
        self.globalMethod = globalMethod
        self.localMethod = localMethod
    def toJson(self):
        obj = dict()
        for attr in ['numIters','tol','startRandom','numRestarts','timeout','globalMethod','localMethod']:
            obj[attr] = getattr(self,attr)
        return obj
    def fromJson(self,obj):
        for attr in ['numIters','tol','startRandom','numRestarts','timeout','globalMethod','localMethod']:
            if attr in obj:
                setattr(self,attr,obj[attr])
    def solve(self,optProblem,seed=None):
        """Globally or locally solves an OptimizationProblem instance with the given parameters.
        Optionally takes a seed as well.  Basically, this is a thin wrapper around GlobalOptimizer
        that converts the OptimizerParams to the appropriate format.

        Returns (success,x) where success is True or False and x is the solution.
        """
        method = self.globalMethod
        numIters = self.numIters
        if self.globalMethod == 'random-restart' or (self.globalMethod is None and (self.numRestarts > 1 or self.startRandom == False)):
            #use the GlobalOptimize version of random restarts
            assert self.localMethod is not None,"Need a localMethod for random-restart to work ('auto' is OK)"
            if self.globalMethod is None:
                method = 'random-restart' + '.' + self.localMethod
            else:
                method = self.globalMethod + '.' + self.localMethod
            numIters = [self.numRestarts,self.numIters]
        elif self.localMethod is not None:
            if self.globalMethod is None:
                method = self.localMethod
            else:
                #do a sequential optimization
                method = [self.globalMethod,self.localMethod]
                #co-opt self.numRestarts for the number of outer iterations?
                numIters = [self.numRestarts,self.numIters]
        optSolver = GlobalOptimizer(method=method)
        if seed is not None:
            optSolver.setSeed(seed)
        (succ,res) = optSolver.solve(optProblem,numIters=numIters,tol=self.tol)
        return (succ,res)


class OptimizationObjective:
    """
    Describes an optimization cost function or constraint.

    Members
    - expr: a symbolic.Expression object f(x)
    - type: string describing what the objective does:
      - 'cost': added to the cost.  Must be scalar.
      - 'eq': an equality f(x)=0 that must be met exactly (up to a given equality tolerance)
      - 'ineq': an inequality constraint f(x)<=0
      - 'feas': a black-box boolean feasibility test f(x) = True
    - soft: if true, this is penalized as part of the cost function.  Specifically ||f(x)||^2 
      is the penalty for eq types, and I[f(x)!=True] for feas types.
    - weight: a weight weight
    - name: an optional name
    """
    def __init__(self,expr,type,weight=None):
        self.expr = expr
        self.type = type
        self.weight = weight
        self.name = None
        if weight is None or type == 'cost':
            self.weight = 1
            self.soft = False
        else:
            self.soft = True
        

class OptimizationProblemBuilder:
    """Defines a generalized optimization problem that can be saved/loaded from
    a JSON string.  Allows custom lists of objectives, feasibility tests, andcost functions.
    Multiple variables can be optimized at once.

    - context: a symbolic.Context that stores the optimization variables and any user data.
    - objectives: a list of OptimizationObjectives used in the optimization
    - optimizationVariables: A list of Variables used for optimization.  If not set, this will
      try to find the variable 'x'.  If not found, this will use all unbound variables in the
      objectives.

    Note that objectives must be symbolic.Function
    objects, so that they are savable/loadable.  See the documentation of the symbolic
    module for more detail.
    """
    def __init__(self,context=None):
        if context is None:
            context = symbolic.Context()
        self.context = context
        self.objectives = []
        self.variableBounds = {}
    def addEquality(self,f,weight=None):
        """If f is a symbolic.Function it's a function f(x) that evaluates to 0 for a
        feasible solution.  If it is a symbolic.Expression it's an expresion over
        the optimization variables

        If weight = None then this is an equality constraint, Otherwise
        it gets added to the objective weight*||f(x)||^2."""
        if isinstance(f,symbolic.Function):
            assert len(self.optimizationVariables) > 0,"To add functions to constraints, the optimizationVariables object must be set"
            return self.addEquality(f.context.bindFunction(f,self.optimizationVariables),weight)
        else:
            assert isinstance(f,symbolic.Expression)
            self.objectives.append(OptimizationObjective(f,"eq",weight))
            return self.objectives[-1]
    def addInequality(self,f,weight=None):
        """Adds an inequality f(x) <= 0."""
        if isinstance(obj,symbolic.Function):
            assert len(self.optimizationVariables) > 0,"To add functions to constraints, the optimizationVariables object must be set"
            return self.addInequality(self.context.bindFunction(f,self.optimizationVariables),weight)
        else:
            assert isinstance(f,symbolic.Expression)
            self.objectives.append(OptimizationObjective(f,"eq",weight))
            return self.objectives[-1]
    def addCost(self,f,weight=1):
        """Adds a cost function f(q)."""
        if isinstance(f,symbolic.Function):
            assert len(self.optimizationVariables) > 0,"To add functions to constraints, the optimizationVariables object must be set"
            return self.addCost(self.context.bindFunction(f,self.optimizationVariables))
        else:
            assert isinstance(f,symbolic.Expression)
            self.objectives.append(OptimizationObjective(f,'cost',weight))
            return self.objectives[-1]
    def addFeasibilityTest(self,f,weight=None):
        """Adds an additional feasibility test."""
        if isinstance(f,symbolic.Function):
            return self.addFeasibilityTest(self.context.bindFunction(f,self.optimizationVariables),weight)
        else:
            assert isinstance(f,symbolic.Expression)
            self.objectives.append(OptimizationObjective(f,'feas',weight))
            return self.objectives[-1]
    def setBounds(self,var,xmin=None,xmax=None):
        """Bounds the optimization variable var"""
        if isinstance(var,symbolic.Variable):
            var = var.name
        if xmin is None and xmax is None:
            if var in self.variableBounds:
                del self.variableBounds[var]
        else:
            self.variableBounds[var] = (xmin,xmax)

    def bind(self,**kwargs):
        """Binds the variables specified by the keyword arguments"""
        for (k,v) in kwargs:
            self.context.variableDict[k].bind(v)
    def unbind(self,**kwargs):
        """Binds the variables specified by the keyword arguments"""
        for (k,v) in kwargs:
            self.context.variableDict[k].unbind()
        return res
    def bindVars(self,*args):
        for x,v in zip(self.optimizationVariables,args):
            x.bind(v)
    def unbindVars(self):
        for x in self.optimizationVariables:
            x.unbind()
    def getVarValues(self):
        """Saves the bindings for optimization variables in the current context into a list."""
        return [v.value for v in self.optimizationVariables]
    def setVarValues(self,s):
        """Converts a state into bindings for the optimization variables in the current context."""
        for (v,var) in zip(s,self.optimizationVariables):
            var.bind(v)
    def getVarVector(self):
        """Flattens the bindings for optimization variables in the current context into a vector x."""
        return symbolic._flatten(*[v.value for v in self.optimizationVariables])
    def setVarVector(self,x):
        """Turns a vector x into bindings for the optimization variables in the current context."""
        ofs=0
        for v in self.optimizationVariables:
            if v.type.is_scalar():
                v.bind(x[ofs])
                ofs += 1
            else:
                assert v.type.char == 'V',"TODO: handle matrix/array variables"
                v.bind(x[ofs:ofs+v.type.size])
                ofs += v.type.size
    def randomVarBinding(self):
        """Samples values for all optimization variables, sampling uniformly according to their bounds"""
        for k,bnds in self.variableBounds.items():
            var = self.context.variableDict[k]
            if var.type.is_scalar():
                var.bind(sample_range(*bnds))
            else:
                var.bind([sample_range(a,b) for (a,b) in zip(*bnds)])
        for var in self.optimizationVariables:
            if var.name not in self.variableBounds:
                infbnd = (-float('inf'),float('inf'))
                if var.type.is_scalar():
                    var.bind(sample_range(*infbnd))
                else:
                    assert v.type.char == 'V',"TODO: handle matrix/array variables"
                    assert var.type.size >= 0
                    var.bind([sample_range(*infbnd) for i in range(var.type.size)])

    def cost(self):
        """Evaluates the cost function with the variables already bound."""
        for v in self.optimizationVariables:
            assert v.value is not None,"All optimization variables must be bound"
        robset = False
        csum = 0.0
        for obj in self.objectives:
            if obj.type == 'cost':
                #print obj.weight,obj.expr.evalf(self.context)
                csum += obj.weight*obj.expr.evalf(self.context)
            elif obj.soft:
                if obj.type == 'eq':
                    r = obj.expr.evalf(self.context)
                    csum += obj.weight*np.linalg.dot(r,r)
                elif obj.type == 'feas':
                    if not obj.expr.evalf(self.context):
                        csum += obj.weight
                elif obj.type == 'ineq':
                    raise NotImplementedError("Soft inequalities")
        return csum

    def equalityResidual(self,soft=True):
        """Evaluates the equality + ik functions at the currently bound state x, stacking the results
        into a single vector.  The residual should equal 0 (to a small tolerance) at a feasible
        solution.

        If soft=True, also stacks the soft equalities.
        """
        for v in self.optimizationVariables:
            assert v.value is not None,"All optimization variables must be bound"
        robset = False
        esum = []
        for obj in self.objectives:
            if obj.type == 'eq' and (not obj.soft or soft):
                esum.append(obj.expr.evalf(self.context)*obj.weight)
        return symbolic._flatten(*esum)

    def satisfiesEqualities(self,tol=1e-3):
        """Returns True if every entry of the (hard) equality + IK residual equals 0 (to the tolerance tol)."""
        res = self.equalityResidual()
        if len(res) == 0: return True
        return all(abs(r) <= tol for r in res)

    def inequalityResidual(self,soft=False):
        """Evaluates the inequality functions at the currently bound state x, stacking the results
        into a single vector.  The residual should be <= 0 at a feasible
        solution.

        If soft=True then this includes the soft inequality residuals.
        """
        for v in self.optimizationVariables:
            assert v.value is not None,"All optimization variables must be bound"
        robset = False
        esum = []
        for obj in self.objectives:
            if obj.type == 'ineq' and (not obj.soft or soft):
                esum.append(obj.expr.evalf(self.context)*obj.weight)
        return symbolic._flatten(*esum)

    def satisfiesInequalities(self,margin=0):
        """Returns True if the for currently bound state x, every entry of the (hard) inequality residuals is
        <= -margin (default 0)."""
        for v in self.optimizationVariables:
            assert v.value is not None,"All optimization variables must be bound"
        res = self.inequalityResidual()
        if len(res) == 0: return True
        return all(r <= -margin for r in res)

    def feasibilityTestsPass(self,soft=False):
        """Returns True if the currently bound state passes all black-box feasibility tests."""
        for v in self.optimizationVariables:
            assert v.value is not None,"All optimization variables must be bound"
        for obj in self.objectives:
            if obj == 'feas' and (not obj.soft or soft):
                r = obj.expr.evalf(self.context)
                if not r: return False
        return True

    def inBounds(self):
        """Returns True if all bounded variables are within their ranges at the currently bound state x"""
        for k,bnds in self.variableBounds.items():
            var = self.context.variableDict[k]
            assert var.value is not None,"All optimization variables must be bound"
            xmin,xmax = bnds
            if not symbolic_linalg.bound_contains(xmin,xmax,var.value).evalf():
                return False
        return True

    def isFeasible(self,eqTol=1e-3):
        """Returns True if the currently bound state passes all equality, inequality, joint limit, and black-box feasibility
        tests.  Equality and IK constraints mut be met with equality tolerance eqTol."""
        
        if not self.inBounds(): return False
        res = self.equalityResidual()
        if any(abs(r) > eqTol for r in res):
            return False
        res = self.inequalityResidual()
        if any(r > 0 for r in res):
            return False
        if not self.feasibilityTestsPass(): return False
        return True

    def costSymbolic(self):
        """Returns a symbolic.Expression, over variables in self.context, that
        evaluates to the cost"""
        components = []
        weights = []
        for obj in self.objectives:
            if obj.type == 'cost':
                components.append(obj.expr)
                weights.append(obj.weight)
            elif obj.soft:
                if obj.type == 'eq':
                    components.append(symbolic_linalg.dot(obj.expr,obj.expr))
                    weights.append(obj.weight)
                elif obj.type == 'feas':
                    components.append(symbolic.if_(obj.expr,1,0))
                    weights.append(obj.weight)
                else:
                    raise NotImplementedError("Soft inequalities")
        if len(components)==0:
            return None
        oldvals = self.getVarValues()
        self.unbindVars()
        res = symbolic.simplify(symbolic.weightedsum(*(components + weights)),self.context)
        self.setVarValues(oldvals)
        return res
        #return symbolic.weightedsum(*(components + weights))

    def equalityResidualSymbolic(self,soft=False):
        """Returns a symbolic.Expression, over variables in self.context, that
        evaluates to the equality residual"""
        components = []
        for obj in self.objectives:
            if obj.type == 'eq' and (not obj.soft or soft):
                components.append(obj.expr*obj.weight)
        if len(components) == 0: return None
        oldvals = self.getVarValues()
        self.unbindVars()
        res = symbolic.simplify(symbolic.flatten(*components),self.context)
        self.setVarValues(oldvals)
        return res

    def inequalityResidualSymbolic(self,soft=False):
        """Returns a symbolic.Expression,  over variables in self.context, that
        evaluates to the inequality residual"""
        components = []
        for obj in self.objectives:
            if obj.type == 'ineq' and (not obj.soft or soft):
                components.append(obj.expr*obj.weight)
        if len(components) == 0: return None
        oldvals = self.getVarValues()
        self.unbindVars()
        res = symbolic.simplify(symbolic.flatten(*components),self.context)
        self.setVarValues(oldvals)
        return res

    def equalitySatisfiedSymbolic(self,tol=1e-3,soft=False):
        """Returns a symbolic.Expression, over variables in self.context, that
        evaluates to True if the equality constraint is met with tolerance tol"""
        res = self.equalityResidualSymbolic(soft)
        if res is None: return None
        return symbolic.abs_(res) <= tol

    def inequalitySatisfiedSymbolic(self,soft=False):
        """Returns a symbolic.Expression, over variables in self.context, that
        evaluates to True if the inequality constraint is met"""
        res = self.inequalityResidualSymbolic(soft)
        if res is None: return None
        return res <= 0

    def feasibilityTestsPassSymbolic(self,soft=False):
        """Returns a symbolic.Expression, over variables in self.context, that
        evaluates to True if the black-box feasibility constraints are met"""
        components = []
        for obj in self.objectives:
            if obj == 'feas' and (not obj.soft or soft):
                components.append(obj.expr)
        if len(components) == 0: return None
        oldvals = self.getVarValues()
        self.unbindVars()
        res = symbolic.simplify(symbolic.all_(*components),self.context)
        self.setVarValues(oldvals)
        return res

    def inBoundsSymbolic(self):
        """Returns a symbolic.Expression, over variables in self.context, that
        evaluates to True the configuration meets bound constraints"""
        exprs = []
        for k,bnd in self.variableBounds.items():
            exprs.append(self.context.linalg.bound_contains(qmin,qmax,self.context.get(k)))
        return symbolic.all_(*exprs)

    def isFeasibleSymbolic(self,eqTol=1e-3):
        """Returns a symbolic.Expression, over $q and other user data variables, that
        evaluates to True if the configuration meets all feasibility tests"""
        tests = [self.inBoundsSymbolic(),self.equalitySatisfiedSymbolic(eqTol),self.inequalitySatisfiedSymbolic(),self.feasibilityTestsPassSymbolic()]
        return symbolic.all_(*[t for t in tests if t is not None])

    def score(self,eqWeight=1.0,ineqWeight=1.0,infeasWeight=1.0):
        """Returns an error score that is equal to the optimum at a feasible
        solution. Evaluated at the currently bound state x."""
        c = self.cost()
        if eqWeight != 0:
            res = self.equalityResidual()
            if len(res) > 0:
                c += eqWeight*vectorops.norm(res)
        if ineqWeight != 0:
            res = self.inequalityResidual()
            if len(res) > 0:
                c += ineqWeight*vectorops.norm(res)
        if infeasWeight != 0:
            for obj in self.objectives:
                if obj == 'feas' and not obj.soft:
                    if not obj.expr.eval():
                        c += infeasWeight
            if not self.inBounds():
                c += infeasWeight
        return c

    def pprint(self,indent=0):
        ncost = len([obj for obj in self.objectives if obj.type == "cost" or obj.soft])
        istring = " "*indent
        if ncost == 0:
            print("%sfind[%s]"%(istring,",".join([v.name for v in self.optimizationVariables])))
        else:
            print("%smin[%s] %s"%(istring,",".join([v.name for v in self.optimizationVariables]),str(self.costSymbolic())))
        if ncost < len(self.objectives) or len(self.variableBounds) > 0:
            print("%s    such that"%(istring,))
            for obj in self.objectives:
                if not(obj.type == "cost" or obj.soft):
                    print("%s%s%s"%(istring,("" if obj.name is None else obj.name+": "),str(obj.expr)), end=' ')
                    if obj.type == "eq":
                        print("= 0")
                    elif obj.type == "ineq":
                        print("<= 0")
                    else:
                        print("holds")
            for k,v in self.variableBounds.items():
                if hasattr(v[0],'__iter__'):
                    for i in range(len(v[0])):
                        print("%s[%f]\t"%(istring,v[0][i]), end=' ')
                        if i == len(v[0])/2:
                            print("<=",k,"<=\t", end=' ')
                        else:
                            print("\t", end=' ')
                        print("[%f]"%(v[1][i],))
                else:
                    print("%s%f <= %s <= %f"%(istring,v[0],k,v[1]))

    def toJson(self,saveContextFunctions=False,prettyPrintExprs=False):
        """Returns a JSON object representing this optimization problem.

        Arguments:
        - saveContextFunctions: if True, saves all custom functions in self.context.
          If they are saved, then the current context is required to be the same context
          in which the problem is loaded.
        - prettyPrintExprs: if True, prints expressions more nicely as more human-readable
          strings rather than JSON objects. These strings are parsed on load, which is a
          little slower than pure JSON. 
        """
        res = dict()
        res['type'] = 'OptimizationProblemBuilder'
        res['context'] = symbolic_io.contextToJson(self.context,saveFunctions=saveContextFunctions)
        objectivesJson = []
        for o in self.objectives:
            if prettyPrintExprs:
                ojson = {'expr':symbolic_io.exprToStr(o.expr,parseCompatible=True),'type':o.type,'soft':o.soft,'weight':o.weight,'name':o.name} 
            else:
                ojson = {'expr':symbolic_io.exprToJson(expr),'type':o.type,'soft':o.soft,'weight':o.weight,'name':o.name} 
            objectivesJson.append(ojson)
        res['objectives'] = objectivesJson
        res['optimizationVariables'] = [v.name for v in self.optimizationVariables]
        res['variableBounds'] = self.variableBounds
        return res

    def fromJson(self,object,context=None):
        """Sets this IK problem to a JSON object representing it. A ValueError
        is raised if it is not the correct type."""
        if object['type'] != 'OptimizationProblemBuilder':
            raise ValueError("Object must have type OptimizationProblemBuilder")

        if context is not None:
            self.context = context
        else:
            symbolic_io.contextFromJson(self.context,object['context'])

        self.objectives = []
        for ojson in object['objectives']:
            if isinstance(ojson['expr'],str):
                expr = symbolic_io.exprFromStr(self.context,ojson['expr'])
            else:
                expr = symbolic_io.exprFromJson(self.context,ojson['expr'])
            self.objectives.append(OptimizationObjective(expr,ojson['type'],(ojson['weight'] if ojson['soft'] else None)))
            self.objectives[-1].name = ojson['name']
        self.optimizationVariables = []
        for n in object['optimizationVariables']:
            assert n in self.context.variableDict,"Context does not contain optimization variable "+n
            self.optimizationVariables.append(self.context.variableDict[n])
        self.variableBounds = object['variableBounds']
        return

    def preprocess(self,steps='all'):
        """Returns a triple (opt,optToSelf,selfToOpt) where
        - opt: a simplified version of this optimization problem. If no simplfication can be performed, opt = self
        - optToSelf: a map of opt's variables to self's variables. If no simplification can be performed, optToSelf = None
        - selfToOpt: a map of self's variables to opts's variables. If no simplification can be performed, selfToOpt = None

        Specific steps include:
        - delete any objectives with 0 weight
        - delete any optimization variables not appearing in expressions
        - fixed-bound (x in [a,b], with a=b) variables are replaced with fixed values.
        - simplify objectives
        - TODO: replace equalities of the form var = expr by matching var to expr?

        If optToSelf is not None, then it is a list of Expressions that, when eval'ed, produce the values of the corresponding
        optimizationVariables in the original optimization problem.  selfToOpt performs the converse mapping.
        In other words, if opt has bound values to all of its optimizationVariables, the code
        for var,expr in zip(self.optimizationVariables,optToSelf):
            var.bind(expr.eval(opt.context))
        binds all optimization variables in self appropriately.
        """
        modified = False
        result = OptimizationProblemBuilder(self.context)
        result.optimizationVariables = []
        optToSelf = []
        selfToOpt = []
        for v in self.optimizationVariables:
            if v.name not in self.variableBounds:
                result.optimizationVariables.append(v.name)
                optToSelf.append(symbolic.expr(result.context.variableDict[v.name]))
                selfToOpt.append(symbolic.expr(v))
            else:
                xmin,xmax = self.variableBounds[v.name]
                if v.type.is_scalar():
                    if xmin == xmax:
                        #remove from optimization
                        if not modified:
                            result.context = self.context.copy()
                            modified = True
                        result.variableDict[v.name].bind(xmin)
                else:
                    assert v.type.char == 'V',"TODO: handle non numeric/vector valued variables, type %s"%(v.type,)
                    activeDofs = []
                    inactiveDofs = []
                    for i in range(len(xmin)):
                        if xmin[i] == xmax[i]:
                            inactiveDofs.append(i)
                        else:
                            activeDofs.append(i)
                    if len(activeDofs) == 0:
                        if not modified:
                            result.context = self.context.copy()
                            modified = True
                        result.context.variableDict[v.name].bind(xmin)
                        #don't add to optimization variables
                        print("OptimizationProblemBuilder.preprocess(): No active DOFS on",v.name,"removing from optimization variables")
                        print("  xmin",xmin,"xmax",xmax)
                    elif len(inactiveDofs) > 0:
                        if not modified:
                            result.context = self.context.copy()
                            modified = True
                        vact = result.context.variableDict[v.name]
                        vact.type.size = len(activeDofs)
                        assert any(vact is v for v in result.context.variables)
                        if v.value is not None:
                            vact.value = [v.value[d] for d in activeDofs]
                        vlift = symbolic.setitem(xmin,activeDofs,vact)
                        result.optimizationVariables.append(v.name)
                        optToSelf.append(vlift)
                        selfToOpt.append(symbolic.getitem(v,activeDofs))
                        if v.name in self.variableBounds:
                            vmin,vmax = self.variableBounds[v.name]
                            result.setBounds(v.name,[vmin[d] for d in activeDofs],[vmax[d] for d in activeDofs])
                    else:
                        result.optimizationVariables.append(v.name)
                        if v.name in self.variableBounds:
                            vmin,vmax = self.variableBounds[v.name]
                            result.setBounds(v.name,vmin,vmax)
                        optToSelf.append(symbolic.expr(result.context.variableDict[v.name]))
                        selfToOpt.append(symbolic.expr(v))
        #print "OptimizationProblemBuilder.preprocess(): optimization variables",[str(v) for v in self.optimizationVariables],"->",[str(v) for v in result.optimizationVariables]
        assert modified != (len(optToSelf) == 0 and len(result.optimizationVariables) == len(self.optimizationVariables))
        #delete any objectives with 0 weight
        sourceObjectives = self.objectives
        if any(obj.weight==0 for obj in self.objectives):
            modified = True
            sourceObjectives = [obj for obj in self.objectives if obj.weight != 0]

        if not modified:
            return self,None,None

        #convert names to Variables
        result.optimizationVariables = [result.context.variableDict[vname] for vname in result.optimizationVariables]
        #simplify and remap expressions
        oldVals = self.getVarValues()
        for var in self.optimizationVariables:
            var.unbind()
        for i,obj in enumerate(sourceObjectives):
            expr = symbolic.simplify(obj.expr,result.context)
            for var,vexpr in zip(result.optimizationVariables,optToSelf):
                try:
                    expr = expr.replace(var,vexpr)
                except ValueError:
                    pass
            #print "Replacement for",obj.type,"objective",obj.expr,"is",expr
            expr = symbolic.simplify(expr)
            #print "  simplified to",expr
            #raw_input()
            result.objectives.append(OptimizationObjective(expr,obj.type,obj.weight))
            result.objectives[-1].soft = obj.soft
            result.objectives[-1].name = obj.name
        self.setVarValues(oldVals)
        return (result,optToSelf,selfToOpt)

    def getBounds(self):
        """Returns optimization varable bounds as a list of (xmin,xmax) pairs. None is returned if the
        problem is unconstrained"""
        inf = float('inf')
        if len(self.variableBounds) == 0 or not any(v.name in self.variableBounds for v in self.optimizationVariables):
            return None
        return [self.variableBounds.get(v.name,((-inf,inf) if v.type.is_scalar() else ([-inf]*v.type.size,[inf]*v.type.size))) for v in self.optimizationVariables]

    def getProblem(self):
        """Returns an OptimizationProblem instance over the optimization variables.
        """
        optProblem = OptimizationProblem()
        eq = self.equalityResidualSymbolic()
        ineq = self.inequalityResidualSymbolic()
        feas = self.feasibilityTestsPassSymbolic()
        cost = self.costSymbolic()
        if len(self.optimizationVariables) == 0:
            if 'x' in self.context.variableDict:
                self.optimizationVariables = self.context.variableDict['x']
            else:
                raise NotImplementedError("No optimization variables set; dynamic interpretation not complete yet")

        #to prevent simplification from destroying variable references, save the values and unbind them...
        oldValues = self.getVarValues()
        self.unbindVars()
        
        if eq is not None: optProblem.addSymbolicConstraint((symbolic.simplify(eq,self.context) == 0),self.context,self.optimizationVariables)
        if ineq is not None: optProblem.addSymbolicConstraint((symbolic.simplify(ineq,self.context) <= 0),self.context,self.optimizationVariables)
        if feas is not None: optProblem.addSymbolicConstraint(symbolic.simplify(feas,self.context),self.context,self.optimizationVariables)
        if cost is not None: optProblem.setSymbolicObjective(symbolic.simplify(cost,self.context),self.context,self.optimizationVariables)
        vbounds = self.getBounds()
        if vbounds is not None:
            aggxmin = symbolic._flatten(*[bmin for (bmin,bmax) in vbounds])
            aggxmax = symbolic._flatten(*[bmax for (bmin,bmax) in vbounds])
            optProblem.setBounds(aggxmin,aggxmax)

        #restore unbound variables
        self.setVarValues(oldValues)
        return optProblem

    def solve(self,params=OptimizerParams(),preprocess=True,cache=False):
        """Solves the optimization problem.  The result is stored in the bound optimizationVariables.

        If you will be solving the problem several times without modification (except for user data and
        initial values of optimizationVariables), you may set cache=True to eliminate some overhead. 
        Note that caching does not work properly if you change constraints or non-optimization variables.
        """
        print("OptimizationProblemBuilder.solve(): My optimization variables",[v.name for v in self.optimizationVariables])
        #first check for cached values
        if cache and hasattr(self,'_cached_problem'):
            p,pToSelf,selfToP,optp = self._cached_problem
        else:
            #if not, do the preprocessing
            if preprocess:
                p,pToSelf,selfToP = self.preprocess()
            else:
                p = self
                pToSelf,selfToP = None,None
            optp = p.getProblem()
            if cache:
                self._cached_problem = p,pToSelf,selfToP,optp

        seed = None
        if params.globalMethod is None or params.globalMethod == 'random-restart':
            #ensure that you have a seed
            vseed = [v.value for v in p.optimizationVariables]
            if all(v is not None for v in vseed):
                seed = symbolic._flatten(vseed)
            else:
                assert all(v is None for v in vseed),"TODO: instantiation of partially bound values"
                assert params.globalMethod is not None,"Only doing local optimization, need to provide a seed value"
        (success,res) = params.solve(optp,seed)
        if res is not None:
            p.setVarVector(res)
        if p is not self:
            #the vector was set in a different context, now map p's variables to self's variables
            for v,expr in zip(self.optimizationVariables,pToSelf):
                v.bind(expr.eval(p.context))
        return success
