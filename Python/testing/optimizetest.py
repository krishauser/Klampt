from klampt import *
from klampt.model import ik
from klampt.math import symbolic,symbolic_io,symbolic_linalg,symbolic_klampt
from klampt.math.optimize import *
from klampt.plan import robotoptimize
import pprint
import time

def rosenbrock(N=2):
    """Returns a symbolic.Context and symbolic.Expression representing the Rosenbrock function
    in N dimensions.
    """
    ctx = symbolic.Context()
    x = []
    assert N >= 2
    if N == 2:
        x.append(ctx.addVar("x","N"))
        x.append(ctx.addVar("y","N"))
    else:
        for i in range(N):
            x.append(ctx.addVar("x"+str(i+1),"N"))
    terms = []
    for i in range(N-1):
        terms.append(symbolic.const(100)*(x[i+1] - x[i]**2)**2 + (x[i]-1)**2)
    return (ctx,symbolic.sum_(*terms))

def run_opt(problem,solver='auto',seed=None,knownOptimum=None):
    if isinstance(solver,str):
        solver = LocalOptimizer(solver)
    if seed is None:
        raise NotImplementedError("TODO: determine dimensionality from problem")
    solver.setSeed(seed)
    print "Seed objective value:",problem.objectiveValue(solver.seed)
    if len(problem.equalities) > 0:
        print "  Equality residual:",problem.equalityResidual(solver.seed)
    if len(problem.inequalities) > 0:
        print "  Inequality residual:",problem.inequalityResidual(solver.seed)
    t0 = time.time()
    res,xopt = solver.solve(problem)
    t1 = time.time()
    print "Solver completed in time",t1-t0
    print "Solution",xopt,
    if knownOptimum is not None:
        print "Optimal point is",knownOptimum
    else:
        print
    print "  Solved objective value:",problem.objectiveValue(xopt)
    if len(problem.equalities) > 0:
        print "  Equality residual:",problem.equalityResidual(xopt)
    if len(problem.inequalities) > 0:
        print "  Inequality residual:",problem.inequalityResidual(xopt)

def test_symbolic():
    """Runs the symbolic module self-tests"""
    symbolic._run_basic_test()
    symbolic._run_subs_test()
    symbolic._run_function_test()
    symbolic._run_jacobian_test()
    symbolic._run_simplify_test()
    print symbolic.expr('x').__class__.__name__
    print symbolic.expr([1,'y',3])['x']
    print symbolic.expr([1,symbolic.expr('y'),3])['x'].eval({'x':1})
    from klampt.math.symbolic_klampt import _so3_rotation

    #test linear algebra
    ctx = symbolic_linalg.LinAlgContext()
    x = ctx.addVar("x","V",2)
    y = ctx.addVar("y","V",4)
    print "Lift:",symbolic.setitem(y,[1,2],x)
    print "Derivative of lift should be [[0 0],[1,0],[0,1],[0,0]]",symbolic.deriv(symbolic.setitem(y,[1,2],x),x)
    print "L2 norm:",ctx.norm(x),"evaluated at [-3,2]",ctx.norm(x).eval({"x":[-3,2]})
    print "  deriv",symbolic.deriv(ctx.norm(x),x),"evaluated at [-3,2]",symbolic.deriv(ctx.norm(x),x).eval({"x":[-3,2]})
    print "L1 norm:",ctx.norm_L1(x),"evaluated at [-3,2]",ctx.norm_L1(x).eval({"x":[-3,2]})
    print "  deriv",symbolic.deriv(ctx.norm_L1(x),x),"evaluated at [-3,2]",symbolic.deriv(ctx.norm_L1(x),x).eval({"x":[-3,2]})
    print "Linf norm:",ctx.norm_Linf(x),"evaluated at [-3,2]",ctx.norm_Linf(x).eval({"x":[-3,2]})
    print "  deriv",symbolic.deriv(ctx.norm_Linf(x),x),"evaluated at [-3,2]",symbolic.deriv(ctx.norm_Linf(x),x).eval({"x":[-3,2]})

    #test SE3 and SO3
    import math
    from klampt.math.symbolic import _ravel,_unravel,_hyper_shape
    from klampt.math import se3
    print "Hyper shape of transform",_hyper_shape(se3.identity())
    print "_ravel  transform",_ravel(se3.identity())
    print "_unravel transform",_unravel(_ravel(se3.identity()),_hyper_shape(se3.identity()))
    expr = _so3_rotation(symbolic.expr("x"),math.pi*0.5)
    print "Rotation matrix:",
    print symbolic.simplify(expr)
    ctx = symbolic_klampt.KlamptContext()
    T = ctx.addVar('T',ctx.se3.type)
    R = ctx.addVar('R',ctx.so3.type)
    x = ctx.addVar('x','V',3)
    t = ctx.addVar('t','V',3)
    print "Transform set derivatives:"
    print "  d/dt:",symbolic_io.exprToStr(symbolic.deriv(symbolic.setitem(T,1,t),t))
    print "  d/dR:",symbolic_io.exprToStr(symbolic.deriv(symbolic.setitem(T,0,R),R))
    print "Homogeneous transform:",ctx.se3.matrix(T)
    print "  d/dt:",symbolic_io.exprToStr(symbolic.deriv(ctx.se3.matrix(symbolic.setitem(T,1,t)),t))
    print "  d/dR:",symbolic_io.exprToStr(symbolic.deriv(ctx.se3.matrix(symbolic.setitem(T,0,R)),R))
    print "Apply transform:",ctx.se3.apply(T,x)
    print "  d/dx:",symbolic_io.exprToStr(symbolic.deriv(ctx.se3.apply(T,x),x))
    print "  d/dt:",symbolic_io.exprToStr(symbolic.deriv(ctx.se3.apply(symbolic.setitem(T,1,t),x),t))
    print "  d/dR:",symbolic_io.exprToStr(symbolic.deriv(ctx.se3.apply(symbolic.setitem(T,0,R),x),R))
    

def test_sympy():
    """Tests integration of symbolic module with sympy"""
    import sympy
    from klampt.math.symbolic_sympy import exprToSympy,exprFromSympy,SympyFunction

    ctx = symbolic.Context()
    x = ctx.addVar("x","N")
    y = ctx.addVar("y","N")

    #Sympy testing
    sx,sy = sympy.symbols("x y")
    sympy_expr = 2*sx*sy
    twoxy = SympyFunction("twoxy",sympy_expr)
    ctx.declare(twoxy)
    f = twoxy(x,y)
    print "Conversion from sympy function to symbolic Function?",f,"=",f.eval({'x':5,'y':2})
    df = symbolic.deriv(f,x)
    print "Derivative of sympy function?",df,"=",df.eval({'x':5,'y':2})
    ddf = symbolic.deriv(df,x)
    print "Second derivative of sympy function?",ddf,"=",ddf.eval({'x':5,'y':2})
    standard_expr = symbolic.sqrt(symbolic.expr(1)/'x')
    vector_expr = symbolic.sqrt(symbolic.expr([1,0])+symbolic.list_(symbolic.const(0.0),symbolic.expr('x')))
    custom_expr = symbolic_linalg.bound_contains('xmin','xmax','x')

    print "Sympy (x+y):",exprToSympy(x+y)
    print "Sympy [x,y]:",exprToSympy(symbolic.stack(x,y))
    print "Sympy [x,y,1][0]:",exprToSympy(symbolic.stack(x,y,1)[0])
    print "Sympy dot([x,y],[x,3]):",exprToSympy(symbolic.dot(symbolic.stack(x,y),symbolic.stack(x,3)))
    print "Sympy row_stack([x,y],[x,3]):",exprToSympy(symbolic.row_stack(symbolic.stack(x,y),symbolic.stack(x,3)))
    print "Sympy dot(eye(2)*2,[x,y]):",exprToSympy(symbolic.dot(symbolic.eye(2)*2,symbolic.stack(x,y)))
    try:
        print "Sympy getitem([1,2,3],x):",exprToSympy(symbolic.getitem([1,2,3],x))
        print "  Hmm... shouldn't get here due to exception"
    except Exception:
        print "  raised exception (properly)"
    print "Sympy custom function:",exprToSympy(symbolic_linalg.bound_contains('xmin','xmax','x'))
    def _twoxy_py(x,y):
        return 2*x*y
    twoxy_py = ctx.declare(_twoxy_py,"twoxy")
    custom_expr2 = twoxy_py(3,"x")
    scustom_expr2 = exprToSympy(custom_expr2)

    print "Sympy custom function 2:",scustom_expr2
    print "Sympy custom function 2 func:",scustom_expr2.func
    print "Sympy custom function 2 evalf:",scustom_expr2.subs("x",2).evalf()
    print "Sympy custom function derivative:",sympy.diff(scustom_expr2,"x")
    twoxy_py.setDeriv("x",lambda x,y:2*y)
    twoxy_py.setDeriv("y",lambda x,y:2*x)
    print "Sympy custom function derivative now defined:",sympy.diff(scustom_expr2,"x")
    print "Sympy custom function derivative now defined:",sympy.diff(exprToSympy(twoxy_py("x","y")),"x")

    print "From sympy (x+y):",exprFromSympy(ctx,sx+sy)
    print "From sympy (x+y+z) (with z not declared):",exprFromSympy(ctx,sx+sy+sympy.Symbol('z'))
    print "From sympy (x+3):",exprFromSympy(ctx,sx+3)
    print "From sympy 2*(x+3):",exprFromSympy(ctx,2*(sx+3))
    print "From sympy cos(x**3):",exprFromSympy(ctx,sympy.cos(sx**3))
    print "  symbolic derivative:",exprFromSympy(ctx,sympy.cos(sx**3)).deriv("x")
    print "  sympy derivative",sympy.diff(sympy.cos(sx**3))
    print "From sympy d/dx cos(x**3):",exprFromSympy(ctx,sympy.diff(sympy.cos(sx**3)))
    heaviside = exprFromSympy(ctx,sympy.Heaviside(sx))
    print "From sympy Heaviside(x):",heaviside
    print "  Evaluated: Heaviside(1) = ",heaviside.eval({"x":1})," Heaviside(-1)=",heaviside.eval({"x":-1})
    print "From sympy Matrix([[x,y]]):",exprFromSympy(ctx,sympy.Matrix([[sx,sy]]))

    print
    print "Sympy-based pretty printing:"
    
    symbolic_io.pprint(standard_expr)
    symbolic_io.pprint(vector_expr)
    symbolic_io.pprint(custom_expr)

    print
    try:
        print "Sympy-based latex:",symbolic_io.exprToLatex(standard_expr)
    except Exception:
        print "FAILED"
    try:
        print "Sympy-based codegen (C):",symbolic_io.codegen(("testfun",standard_expr),"C")[0][1]
    except Exception:
        print "FAILED"
    try:
        print "Sympy-based codegen (Octave/Matlab):",symbolic_io.codegen(("testfun",standard_expr),"Octave")[0][1]
    except Exception:
        print "FAILED"

    print
    print "With arrays:"
    try:
        print "Sympy-based latex:",symbolic_io.exprToLatex(vector_expr)
    except Exception:
        print "FAILED"
    try:
        print "Sympy-based codegen (C):",symbolic_io.codegen(("testfun",vector_expr),"C")[0][1]
    except Exception:
        print "FAILED"
    try:
        print "Sympy-based codegen (Octave/Matlab):",symbolic_io.codegen(("testfun",vector_expr),"Octave")[0][1]
    except Exception:
        print "FAILED"

    print
    print "With custom function:"
    try:
        print "Sympy-based latex:",symbolic_io.exprToLatex(custom_expr)
    except Exception:
        print "FAILED"
    try:
        print "Sympy-based codegen:",symbolic_io.codegen(("testfun",custom_expr),"C")[0][1]
    except Exception:
        print "FAILED"

    print 
    print "Matrix expression testing..."
    M = symbolic.row_stack([x,y],[x**2,y**2])
    expr = M
    for i in range(5):
        expr = symbolic.dot(expr,M)
    print "Long matrix expression",expr
    t0 = time.time()
    xv = 1.0
    yv = 0.5
    Ma = np.array([[xv,yv],[xv**2,yv**2]])
    res = Ma
    for i in range(5):
        res = np.dot(res,Ma)
    t1 = time.time()
    print "Raw numpy time:",t1-t0,"result",res
    t0 = time.time()
    res = expr.evalf({"x":xv,"y":yv})
    t1 = time.time()
    print "Symbolic evalf time:",t1-t0,"result",res
    sexpr = exprToSympy(expr)
    print "Sympy equivalent",sexpr
    t0 = time.time()
    res = sexpr.evalf(subs={"x":xv,"y":yv})
    t1 = time.time()
    print "Sympy evalf time:",t1-t0,"result",res

    from sympy.utilities.lambdify import lambdify,lambdastr
    tstart = time.time()
    fexpr = lambdify(["x","y"],sexpr)
    t0 = time.time()
    res = fexpr(xv,yv)
    t1 = time.time()
    print "Sympy lambdify creation time",t0-tstart,", eval time:",t1-t0,"result",res
    print "  String",lambdastr(["x","y"],sexpr)

def test_parse():
    """Tests parsing of expression strings"""
    ctx = symbolic.Context()
    x = ctx.addVar("x","N")
    y = ctx.addVar("y","N")

    res = symbolic_io.exprFromStr(ctx,"x + y")
    print "Parsed expression x + y:",res
    res = symbolic_io.exprFromStr(ctx,"$x + $y")
    print "Parsed expression $x + $y:",res
    res = symbolic_io.exprFromStr(ctx,"flatten($x,$y)")
    print "Parsed expression flatten($x,$y):",res
    try:
        res = symbolic_io.exprFromStr(ctx,"f(x + y) + g(y)")
        print "Parsed expression f(x + y) + g(y):",res
    except Exception:
        pass
    res = symbolic_io.exprFromStr(ctx,"3 + [1,2]")
    print "Parsed expression 3 + [1,2]:",res

    res = symbolic_io.exprFromStr(ctx,"[1,2]#a + @a")
    print "Parsed expression [1,2]#a + @a:",res

    a = x+y
    b = a+a
    print "Parse-compatible:",symbolic_io.exprToStr(b,parseCompatible=True)
    print "Expanding:",symbolic_io.exprToStr(b,parseCompatible=True,expandSubexprs=True)
    print "Print with labels:",symbolic_io.exprToStr(b,parseCompatible=False,expandSubexprs=False)

    jsonobj = symbolic_io.exprToJson(b)
    print "JSON output:",jsonobj
    print "JSON input:",symbolic_io.exprFromJson(ctx,jsonobj)
    print "JSON re-input, parse compatible:",symbolic_io.exprToStr(symbolic_io.exprFromJson(ctx,jsonobj),parseCompatible=True)

def test_simple():
    """Tests basic optimization API"""
    print 
    print "**** BASIC TEST *****"
    ctx = symbolic.Context()
    x = ctx.addVar("x","N")
    y = ctx.addVar("y","N")
    #simple expression (x-3)^^2 + 10*y**2
    e = (x-3)**2 + symbolic.const(10)*y**2

    #test unconstrained optimization
    opt = OptimizationProblem()
    opt.setSymbolicObjective(e,ctx)
    run_opt(opt,'scipy.CG',[0.0,1.5],[3,0])

    #test constrained optimization
    opt.addSymbolicConstraint((x <= 1),ctx)
    run_opt(opt,'scipy.SLSQP',[0.0,1.5],[1,0])

    opt2 = OptimizationProblem()
    opt2.setSymbolicObjective(e,ctx)
    opt2.addSymbolicConstraint((x**2 + y**2 == 2),ctx)
    run_opt(opt,'scipy.SLSQP',[1.0,1.5],[2,0])

def test_rosenbrock():
    """Tests basic optimization API with the rosenbrock function"""
    print 
    print "**** ROSENBROCK TEST *****"
    ctx,e = rosenbrock()    
    opt = OptimizationProblem()
    opt.setSymbolicObjective(e,ctx)
    run_opt(opt,'scipy.CG',[0.0,0.0],[1,1])

    #constrained test
    x = ctx.get("x")
    y = ctx.get("y")
    opt.addSymbolicConstraint(((x-1)**3 - y + 1 <= 0),ctx)
    opt.addSymbolicConstraint((x + y - 2 <= 0),ctx)
    run_opt(opt,'scipy.SLSQP',[0.0,0.0],[1,1])



def test_ik():
    """Tests optimization with inverse kinematics constraints"""
    print 
    print "**** IK TEST *****"
    world = WorldModel()
    world.readFile("../../data/tx90scenario0.xml")
    robot = world.robot(0)
    ctx = symbolic_klampt.KlamptContext(world)
    q = ctx.addVar('q','V',world.robot(0).numLinks())

    ikgoal = ik.objective(world.robot(0).link(6),local=[0,0,0],world=[1,0,1])
    symgoal = ctx.addUserData("ik_goal",ikgoal)
    q.bind(robot.getConfig())
    print "Evaluating residual",ctx.ik.residual(symgoal,ctx.setConfig("robot",q)).eval(ctx)
    q.unbind()
    opt = OptimizationProblem()
    #opt.setSymbolicObjective(symbolic.const(0),ctx)
    #opt.setSymbolicObjective(ctx.linalg.distance2(q,[0.0]*7),ctx)
    opt.setSymbolicObjective(ctx.linalg.norm2(q),ctx)
    opt.addSymbolicConstraint(ctx.ik.residual(symgoal,ctx.setConfig("robot",q)) == 0,ctx)
    #opt.addSymbolicConstraint(ctx.inJointLimits(q,"robot"),ctx)
    print "Standard joint limits",ctx.inJointLimits(q,"robot")
    print "Simplified joint limits",symbolic.simplify(ctx.inJointLimits(q,"robot"),ctx)
    opt.addSymbolicConstraint(symbolic.simplify(ctx.inJointLimits(q,"robot"),ctx),ctx)
    #opt.setBounds(*robot.getJointLimits())
    for i in range(10):
        robot.randomizeConfig()
        run_opt(opt,'scipy.SLSQP',robot.getConfig())

    
def test_robotoptimize():
    """Tests the robotoptimize module"""
    world = WorldModel()
    world.readFile("../../data/tx90scenario0.xml")
    opt = robotoptimize.RobotOptimizationProblem(world=world)
    rot = opt.addKlamptVar("rot","Rotation")
    opt.addCost(rot.expr[2])
    #opt.addEquality(opt.q[3])
    opt.q.bind(world.robot(0).getConfig())
    print "Initial cost",opt.cost()
    opt.q.unbind()
    opt.pprint()
    print "Solve result:",opt.solve(optimize.OptimizerParams(localMethod='auto'))
    raw_input()

    ikgoal = ik.objective(world.robot(0).link(6),local=[0,0,0],world=[1,0,1])
    opt.addIKObjective(ikgoal)
    opt.objectives[-1].name = "IK constraint"
    #expr = symbolic.getattr_("robot",symbolic.const("getJointLimits"))[0][3]-opt.q[3]
    jlimits = symbolic.getattr_("robot",symbolic.const("getJointLimits"))
    expr = 0.5*(jlimits[0][4]+jlimits[1][4])-opt.q[4]
    opt.addEquality(expr)
    opt.objectives[-1].name = "Meet joint middle 4"
    #opt.addCost(opt.context.linalg.norm2)
    opt.addCost(symbolic.dot(opt.q,opt.q))
    opt.q.bind(world.robot(0).getConfig())
    print "Initial cost",opt.cost()
    opt.q.unbind()
    print "Cost derivative:",symbolic.deriv(opt.costSymbolic(),opt.q)
    eq = opt.equalityResidualSymbolic()
    y = opt.context.addVar("y","V",2)
    q = world.robot(0).getConfig()
    eq2 = eq.replace(opt.q,symbolic.setitem(q,[1,2],y))
    print "Replaced equality:",eq2
    print "Simplified:",symbolic.simplify(eq2).__repr__()
    print "Equality derivative:",symbolic.deriv(eq2,y).__repr__()
    print "Evaluated",symbolic.deriv(eq2,y).eval({"y":[0.1,0.5]}).__repr__()
    raw_input()
    print
    opt.pprint()
    print
    raw_input()

    """
    jsonString = opt.toJson()
    print "JSON representation",
    pprint.pprint(jsonString)
    """
    print "Solve result:",opt.solve(optimize.OptimizerParams(localMethod='scipy'))

    """
    opt2 = robotoptimize.RobotOptimizationProblem(world=world)
    print "Loading json representation..."
    opt2.fromJson(jsonString)
    print "Loaded JSON representation",
    pprint.pprint(opt2.toJson())
    """

def test_error():
    """Tests what happens when an erroneous expression is given"""
    ctx = symbolic.Context()
    x = ctx.addVar("x",'V',2)
    print "eye(3)[2] simplified",symbolic.simplify(symbolic.eye(3)[2]),"=basis(2)"
    #print "The next line should raise an error..."
    #print x[3]
    print "The next line should raise an error..."
    print symbolic.deriv(x[1]*(x[3]+1),x)

from klampt import *
from klampt.plan import robotoptimize
from klampt.math import symbolic,optimize,vectorops,so3,se3

from klampt.vis import editors
from klampt.model import types
from klampt.io import loader,resource
from klampt import RobotPoser,PointPoser,TransformPoser
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtGui import QFileDialog
import json
import unicodedata
from OpenGL.GL import *

class SmallListWidget(QListWidget):
    def sizeHint(self):
        s = QSize()
        s.setHeight(80)
        #s.setWidth(QListWidget.sizeHint(self).width())
        s.setWidth(150)
        return s

class SmallTextEdit(QTextEdit):
    def sizeHint(self):
        s = QSize()
        s.setHeight(80)
        s.setWidth(QTextEdit.sizeHint(self).width())
        return s

class ShowTextDialog(QDialog):
    def __init__(self,title,text,parent=None):
        QDialog.__init__(self,parent)
        layout = QHBoxLayout()
        textEdit = QTextEdit()
        textEdit.setText(text)
        layout.addWidget(textEdit)
        self.setLayout(layout)
        self.setWindowTitle(title)

class OptimizationProblemEditor(editors.WorldEditor):
    def __init__(self,name,value,description,world):
        editors.WorldEditor.__init__(self,name,world,description)
        self.typelist = ['N','V','M','A',
                         'Vector3','Config','Rotation','RigidTransform',
                         'U']
        self.typenamelist = ["Numeric","Vector","Matrix","N-D array",
                             "3D point","Robot configuration","3D rotation","3D transform",
                             "User data"]
        self.typedefaultvalues = [0.0,[0.0,0.0],[[1.0,0.0],[0.0,1.0]],[[1.0,0.0],[0.0,1.0]],
            None,None,None,None,
            None]
        self.visuallyEditableTypes = ['Config','Vector','Vector3','Point','Matrix3','Rotation','RigidTransform']
        self.value = value
        self.allFunctions = sorted(value.context.listFunctions(doprint=False))
        self.argumentWidget = None
        self.worldeditmaster = self.klamptwidgetmaster
        self.variableWidgets = dict()
    def instructions(self):
        return "Edit the optimization problem below.\nRight-click on items in the world to change starting values."
    def addDialogItems(self,parent,ui='qt'):
        self.parent = parent
        playout = QVBoxLayout(parent)
        self.tabWidget = QTabWidget()
        playout.addWidget(self.tabWidget)
        firstTab = QWidget()
        secondTab = QWidget()
        thirdTab = QWidget()
        self.tabWidget.addTab(firstTab,"Objectives")
        self.tabWidget.addTab(secondTab,"Variables")
        self.tabWidget.addTab(thirdTab,"I/O")
        self.tabWidget.setSizePolicy(QSizePolicy(QSizePolicy.Expanding,QSizePolicy.Expanding))

        #constraint editing
        vlayout = QVBoxLayout(firstTab)
        #left.setSizePolicy(QSizePolicy(QSizePolicy.Expanding,QSizePolicy.Fixed))
        #left.resize(350,200)
        #right.setSizePolicy(QSizePolicy(QSizePolicy.Expanding,QSizePolicy.Fixed))
        #right.resize(350,200)
        self.typeCombo = QComboBox()
        self.typeCombo.setToolTip("Filter by constraint type")
        self.typeCombo.addItem("Cost")
        self.typeCombo.addItem("IK objective")
        self.typeCombo.addItem("Equality")
        self.typeCombo.addItem("Inequality")
        self.typeCombo.addItem("Feasibility test")
        self.typeMap = ['cost','ik','eq','ineq','feas']
        self.constraintList = SmallListWidget()
        self.constraintList.setToolTip("Costs and constraints")
        vlayout.addWidget(self.typeCombo)
        vlayout.addWidget(self.constraintList)
        self.addButton = QPushButton("Add")
        self.addButton.setToolTip("Add a new cost/constraint")
        self.deleteButton = QPushButton("Delete")
        self.deleteButton.setToolTip("Delete selected cost/constraint")
        self.weightLabel = QLabel("Weight")
        self.weightSpin = QDoubleSpinBox()
        self.weightSpin.setToolTip("The weight of the selected cost/constraint")
        self.valueLabel = QLabel("Value: ")
        self.solveButton = QPushButton("Solve")
        self.solveButton.setToolTip("Solve the current optimization problem")
        self.objectiveNameEdit = QLineEdit("Name")
        self.objectiveNameEdit.setToolTip("An optional name of the cost/constraint")
        
        hlayout = QHBoxLayout()
        hlayout.addWidget(self.addButton)
        hlayout.addWidget(self.deleteButton)
        vlayout.addLayout(hlayout)
        
        hlayout = QHBoxLayout()
        hlayout.addWidget(self.valueLabel)
        hlayout.addWidget(self.weightLabel)
        hlayout.addWidget(self.weightSpin)
        vlayout.addLayout(hlayout)
        hlayout = QHBoxLayout()
        hlayout.addWidget(QLabel("Name:"))
        hlayout.addWidget(self.objectiveNameEdit)
        vlayout.addLayout(hlayout)
        playout.addWidget(self.solveButton)

        self.functionCombo = QComboBox()
        for f in self.allFunctions:
            self.functionCombo.addItem(f)
        self.insertFunctionButton = QPushButton("Insert Func")
        self.insertFunctionButton.setToolTip("Adds the selected function to the code box")
        self.codeEdit = SmallTextEdit()
        self.codeEdit.setToolTip("The code box. Edit the constraint expression here")
        self.simplifyButton = QPushButton("Simplify")
        self.simplifyButton.setToolTip("Simplify the constraint expression")
        self.derivativeButton = QPushButton("Derivative")
        self.derivativeButton.setToolTip("Calculate the derivative of the constraint")
        self.editVisualButton = QPushButton("Visual Edit")
        self.editVisualButton.setCheckable(True)
        self.editVisualButton.setToolTip("Show the selected expression in the world")

        hlayout = QHBoxLayout()
        hlayout.addWidget(self.functionCombo)
        hlayout.addWidget(self.insertFunctionButton)
        vlayout.addLayout(hlayout)
        vlayout.addWidget(self.codeEdit)
        hlayout = QHBoxLayout()
        hlayout.addWidget(self.simplifyButton)
        hlayout.addWidget(self.derivativeButton)
        vlayout.addLayout(hlayout)
        vlayout.addWidget(self.editVisualButton)
        self.solveButton.clicked.connect(self.solve)
        self.addButton.clicked.connect(self.insert)
        self.deleteButton.clicked.connect(self.delete)
        self.objectiveNameEdit.editingFinished.connect(self.onobjectiveNameChange)
        self.insertFunctionButton.clicked.connect(self.insertFunction)
        self.constraintList.itemSelectionChanged.connect(self.onConstraintSelectionChange)
        self.typeCombo.activated.connect(self.onTypeChange)
        self.simplifyButton.clicked.connect(self.onClickSimplify)
        self.derivativeButton.clicked.connect(self.onClickDerivative)
        self.editVisualButton.clicked.connect(self.onClickVisualEdit)
        self.codeEdit.textChanged.connect(self.onCodeChange)
        self.codeEdit.selectionChanged.connect(self.onCodeSelectionChange)
        self.weightSpin.valueChanged.connect(self.onWeightChange)
        """
        self.pasteShortcut = QShortcut(QKeySequence(QKeySequence.Paste), self.codeEdit, context=Qt.WidgetShortcut)
        self.pasteShortcut.activated.connect(self.codeEdit.paste)
        self.copyShortcut = QShortcut(QKeySequence(QKeySequence.Copy), self.codeEdit, context=Qt.WidgetShortcut)
        self.copyShortcut.activated.connect(self.codeEdit.copy)
        self.cutShortcut = QShortcut(QKeySequence(QKeySequence.Cut), self.codeEdit, context=Qt.WidgetShortcut)
        self.cutShortcut.activated.connect(self.codeEdit.cut)
        """

        #variable editing
        vlayout = QVBoxLayout(secondTab)
        self.variableList = SmallListWidget()
        self.variableList.setToolTip("List of all variables and user data")
        self.addVarButton = QPushButton("Add")
        self.addVarButton.setToolTip("Add a new variable")
        self.deleteVarButton = QPushButton("Delete")
        self.deleteVarButton.setToolTip("Delete the selected variable")
        self.varTypeCombo = QComboBox()
        self.varTypeCombo.setToolTip("The type of the new variable")
        for t in self.typenamelist:
            self.varTypeCombo.addItem(t)
        hlayout2 = QHBoxLayout()
        hlayout2.addWidget(self.addVarButton)
        hlayout2.addWidget(self.varTypeCombo)
        vlayout.addLayout(hlayout2)
        vlayout.addWidget(self.deleteVarButton)
        hlayout = QHBoxLayout()
        hlayout.addWidget(self.variableList)
        vlayout2 = QVBoxLayout()
        hlayout.addLayout(vlayout2)
        self.varNameEdit = QLineEdit("Name")
        self.varNameEdit.setToolTip("Edit the variable name")
        self.varTypeLabel = QLabel("Type:")
        self.varOptimizeCheck = QCheckBox("Optimize")
        self.varOptimizeCheck.setToolTip("Should this variable be optimized?")
        self.varSizeEdit = QLineEdit("Size")
        self.varSizeEdit.setToolTip("The size of the array")
        self.varValueCombo = QComboBox()
        for name in ['Value','Minimum','Maximum']:
            self.varValueCombo.addItem(name)
        self.varValueCombo.setToolTip("Which value to edit?")
        self.varValueEdit = SmallTextEdit()
        self.varValueEdit.setToolTip("Edit the value of the variable here")
        self.varLoad = QPushButton("Load...")
        self.varLoad.setToolTip("Load a variable value from file")
        self.varShowVisual = QCheckBox("Show in visualization")
        for k,v in self.value.context.variableDict.iteritems():
            self.variableList.addItem(k)
        for k,v in self.value.context.userData.iteritems():
            self.variableList.addItem(symbolic_io.USER_DATA_PREFIX+k)
        vlayout2.addWidget(self.varNameEdit)
        vlayout2.addWidget(self.varOptimizeCheck)
        vlayout2.addWidget(self.varTypeLabel)
        sizelayout = QHBoxLayout()
        sizelayout.addWidget(QLabel("Size:"))
        sizelayout.addWidget(self.varSizeEdit)
        vlayout2.addLayout(sizelayout)
        vlayout.addLayout(hlayout)
        valueLayout = QHBoxLayout()
        valueLayout.addWidget(QLabel("Value"))
        valueLayout.addWidget(self.varValueCombo)
        vlayout.addLayout(valueLayout)
        vlayout.addWidget(self.varValueEdit)
        vlayout.addWidget(self.varLoad)
        vlayout.addWidget(self.varShowVisual)
        self.addVarButton.clicked.connect(self.onVarAdd)
        self.deleteVarButton.clicked.connect(self.onVarDelete)
        self.varNameEdit.editingFinished.connect(self.onVarNameChange)
        self.variableList.itemSelectionChanged.connect(self.onVarSelectionChange)
        self.varValueCombo.activated.connect(self.onVarValueComboChange)
        self.varSizeEdit.textChanged.connect(self.onVarSizeChange)
        self.varOptimizeCheck.clicked.connect(self.onVarOptimizeChange)
        self.varLoad.clicked.connect(self.onVarLoad)
        self.varShowVisual.clicked.connect(self.onVarShowVisualChange)

        #problem 
        vlayout = QVBoxLayout(thirdTab)
        self.loadProblemButton = QPushButton("Load problem...")
        self.saveProblemButton = QPushButton("Save problem...")
        self.printProblemButton = QPushButton("Print problem")
        self.printCodeButton = QPushButton("Print code")
        self.prettyPrintCheck = QCheckBox("Pretty-print expressions")
        self.saveContextFunctionsCheck = QCheckBox("Save context functions")
        self.prettyPrintCheck.setChecked(True)
        self.saveContextFunctionsCheck.setChecked(False)
        vlayout.addWidget(self.loadProblemButton)
        vlayout.addWidget(self.saveProblemButton)
        vlayout.addWidget(self.printProblemButton)
        vlayout.addWidget(self.printCodeButton)
        vlayout.addWidget(self.prettyPrintCheck)
        vlayout.addWidget(self.saveContextFunctionsCheck)
        self.loadProblemButton.clicked.connect(self.loadProblem)
        self.saveProblemButton.clicked.connect(self.saveProblem)
        self.printProblemButton.clicked.connect(self.printProblem)
        self.printCodeButton.clicked.connect(self.printCode)

        self.oldSelection = 0,0
        self.suppressSelectionChange = False
        self.onTypeChange(0)
        self.onVarSelectionChange()
        #print "Tab widget size",self.tabWidget.minimumSizeHint()
        self.tabWidget.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        self.tabWidget.resize(self.tabWidget.minimumSizeHint())
        self.tabWidget.adjustSize()

    def onCodeChange(self,updateSelectionList=True):
        type = self.typeCombo.currentIndex()
        index = self.constraintList.currentRow()
        val = str(self.codeEdit.toPlainText()).replace("\n","")
        typename = self.typeMap[type]
        if len(val) == 0:
            return

        if index < 0 or index > len(self.selectionItems):
            print "Invalid constraint selection"
            return
        objindex = self.selectionItems[index]
        if updateSelectionList:
            if self.value.objectives[objindex].name is None:
                self.constraintList.item(index).setText(self.shortName(self.value.objectives[objindex]))
        try:
            if typename == 'ik':
                try:
                    jsonobj = json.loads(val)
                except Exception as e:
                    print "Invalid JSON",val
                    raise
                try:
                    obj = loader.fromJson(jsonobj)
                except Exception as e:
                    print "Invalid Klamp't object:",e
                    raise
                expr = self.value.context.ik.residual(obj,self.value.context.setConfig("robot",self.value.q))
            else:
                expr = symbolic_io.exprFromStr(self.value.context,val)
                print "Parse successful!"
            self.value.objectives[objindex].expr = expr
            pal = self.functionCombo.palette()
            pal.setColor(QPalette.Text, QColor('black'))
            self.codeEdit.setPalette(pal)
            self.updateValueLabel()
        except Exception as e:
            print "Unable to parse:",'"'+val+'"'
            print "Error:",e
            pal = self.functionCombo.palette()
            pal.setColor(QPalette.Text, QColor('red'))
            self.codeEdit.setPalette(pal)
            return

    def onobjectiveNameChange(self):
        index = self.constraintList.currentRow()
        if index < 0 or index > len(self.selectionItems):
            print "Invalid constraint selection"
            return
        objindex = self.selectionItems[index]
        obj = self.value.objectives[objindex]
        obj.name = str(self.objectiveNameEdit.text()).strip()
        if len(obj.name) == 0:
            obj.name = None
        self.constraintList.item(index).setText(self.shortName(obj))

    def onWeightChange(self):
        index = self.constraintList.currentRow()
        objindex = self.selectionItems[index]
        obj = self.value.objectives[objindex]
        obj.weight = self.weightSpin.value()
        if obj.weight == 0:
            obj.soft = False
            obj.weight = 1
        else:
            obj.soft = True
        
    def onTypeChange(self,type):
        self.selectionItems = None
        self.selectionWeights = None
        typename = self.typeMap[type]
        if typename == 'ik':
            self.insertFunctionButton.setEnabled(False)
        else:
            self.insertFunctionButton.setEnabled(True)
        self.selectionItems = []
        for i,obj in enumerate(self.value.objectives):
            if obj.type.startswith(typename):
                self.selectionItems.append(i)
        self.constraintList.clear()
        for item in self.selectionItems:
            self.constraintList.addItem(self.shortName(self.value.objectives[item]))
        if len(self.selectionItems) == 0:
            self.onConstraintSelectionChange()
        else:
            self.constraintList.setCurrentRow(0)
            self.onConstraintSelectionChange()

    def shortName(self,obj):
        if obj.name is not None:
            return obj.name
        elif symbolic.is_op(obj.expr,'ik.residual'):
            res = obj.expr.args[0]
            assert isinstance(res,symbolic.ConstantExpression) and isinstance(res.value,IKObjective),"Not an IK objective: "+str(expr)
            obj = res.value
            l = obj.link()
            p = obj.destLink()
            npos = obj.numPosDims()
            nrot = obj.numRotDims()
            res = []
            if npos > 0:
                if npos == 3:
                    res = ['Position']
                elif npos == 2:
                    res = ['Planar']
                else:
                    res = ['Linear']
                if nrot == 1:
                    res.append('and axis')
                elif nrot == 3:
                    res.append('and orientation')

            elif npos == 0:
                if nrot == 1:
                    res.append('Axis')
                elif nrot == 3:
                    res.append('Orientation')
            res.append('on')
            res.append(self.value.robot.link(l).getName())
            if p >= 0:
                res.append('w.r.t.')
                res.append(self.value.robot.link(p).getName())
            return ' '.join(res)
        else:
            res = str(obj.expr)
            if len(res) > 30:
                return res[:15]+'...'+res[-11:]
            return res

    def onConstraintSelectionChange(self):
        index = self.constraintList.currentRow()
        type = self.typeMap[self.typeCombo.currentIndex()]
        if index >= 0 and type != 'feas':
            self.weightSpin.setEnabled(True)
        else:
            self.weightSpin.setEnabled(False)
        if index >= 0 and index < len(self.selectionItems): 
            self.deleteButton.setEnabled(True)
            self.insertFunctionButton.setEnabled(True)
            self.codeEdit.setEnabled(True)
            self.objectiveNameEdit.setEnabled(True)
            if type != 'ik':
                self.simplifyButton.setEnabled(True)
                self.derivativeButton.setEnabled(True)
            else:
                self.simplifyButton.setEnabled(False)
                self.derivativeButton.setEnabled(False)
        else:
            self.deleteButton.setEnabled(False)
            self.insertFunctionButton.setEnabled(False)
            self.codeEdit.setEnabled(False)
            self.objectiveNameEdit.setEnabled(False)
            self.updateValueLabel()
            self.simplifyButton.setEnabled(False)
            self.derivativeButton.setEnabled(False)
            return
        objindex = self.selectionItems[index]
        obj = self.value.objectives[objindex]
        if obj.name is None:
            self.objectiveNameEdit.setText("")
        else:
            self.objectiveNameEdit.setText(obj.name)
        #update the code box to match the available arguments
        self.codeEdit.clear()
        if obj.type == 'cost' or obj.soft:
            self.weightSpin.setValue(obj.weight)
        else:
            self.weightSpin.setValue(0)
        if self.value.isIKObjective(objindex):
            self.codeEdit.setText(json.dumps(loader.toJson(self.value.getIKObjective(objindex)),sort_keys=True, indent=4, separators=(',', ': ')))
        else:
            self.codeEdit.setText(symbolic_io.exprToStr(obj.expr,parseCompatible=True))
        self.updateValueLabel()

    def setCurrentValue(self,obj,updateText=False,updateWidget=False):
        index = self.constraintList.currentRow()
        if index < 0 or index >= len(self.selectionItems):
            self.codeEdit.setEnabled(False)
            self.codeEdit.setText("")
            self.editVisualButton.setEnabled(False)
            return
        objindex = self.selectionItems[index]
        if updateText:
            jsonobj = loader.toJson(obj,self.argtype)
            s = json.dumps(jsonobj)
            self.insertCode(s)
            self.onCodeChange()
        if updateWidget:
            if self.argumentWidget is not None:
                self.argumentWidget.set(obj)
        self.updateValueLabel()

    def insertCode(self,text):
        self.suppressSelectionChange = True
        cursor = self.codeEdit.textCursor()
        start = cursor.selectionStart()
        cursor.insertText(text)
        end = start+len(text)
        cursor.setPosition(start)
        cursor.setPosition(end, QTextCursor.KeepAnchor)
        self.codeEdit.setTextCursor(cursor)
        self.suppressSelectionChange = False

    def onCodeSelectionChange(self):
        if self.suppressSelectionChange:
            return
        index = self.constraintList.currentRow()
        if index < 0 or index >= len(self.selectionItems):
            self.codeEdit.setEnabled(False)
            self.codeEdit.setText("")
            self.editVisualButton.setEnabled(False)
            return
        cursor = self.codeEdit.textCursor()
        seltext = unicodedata.normalize('NFKD',unicode(cursor.selectedText())).encode('ascii','ignore')
        sel = cursor.selectionStart(),cursor.selectionEnd()
        if sel == self.oldSelection:
            return
        self.oldSelection = sel
        if len(seltext) > 0:
            try:
                jsonobj = json.loads(seltext)
                obj = loader.fromJson(jsonobj)
                valtypes = types.objectToTypes(obj,self.value.robot)
                if len(valtypes) > 0:
                    #TODO: launch new window for isExternallyEditable types
                    print "Editing widget type",valtypes[-1]
                    self.argumentWidget = self.makeEditingWidget(valtypes[-1],obj)
                    if self.argumentWidget is not None:
                        self.argtype = valtypes[-1]
                        print "Editing widget type",self.argtype
                        self.editVisualButton.setEnabled(True)
                        self.editVisualButton.setChecked(False)
                        self.klamptwidgetmaster = self.worldeditmaster
                        self.refresh()
                        return
            except Exception as e:
                print "Can't edit argument, error parsing",seltext
                print "Exception is",e

        if self.editVisualButton.isEnabled():
            self.editVisualButton.setEnabled(False)
            self.editVisualButton.setChecked(False)
            self.klamptwidgetmaster = self.worldeditmaster        
            self.refresh()

    def onClickSimplify(self):
        type = self.typeCombo.currentIndex()
        val = str(self.codeEdit.toPlainText()).replace("\n","")
        typename = self.typeMap[type]
        if len(val) == 0:
            return
        if typename == 'ik':
            return
        else:
            expr = symbolic_io.exprFromStr(self.value.context,val)
            sexpr = expr.simplify()
            if sexpr is None or sexpr is expr:
                print "Unable to simplify expression"
            else:
                sstr = symbolic_io.exprToStr(sexpr,parseCompatible=True)
                print "Simplified expression",sexpr
                dialog = ShowTextDialog("Simplified expression",sstr)
                dialog.exec_()

    def onClickDerivative(self):
        type = self.typeCombo.currentIndex()
        val = str(self.codeEdit.toPlainText()).replace("\n","")
        typename = self.typeMap[type]
        if len(val) == 0:
            return
        if typename == 'ik':
            return
        expr = symbolic_io.exprFromStr(self.value.context,val)

        res = []
        for var in self.value.optimizationVariables:
            dexpr = symbolic.deriv(expr,var)
            dstr = symbolic_io.exprToStr(dexpr,parseCompatible=True)
            res.append(var.name+":")
            res.append("  "+dstr)
        dialog = ShowTextDialog("Derivatives",'\n'.join(res))
        dialog.exec_()

    def makeEditingWidget(self,type,params):
        if type not in self.visuallyEditableTypes:
            return None
        if type in ['Config','Vector']:
            res = RobotPoser(self.world.robot(0))
        elif type in ['Vector3','Point']:
            res = PointPoser()
        elif type in ['Matrix3','Rotation']:
            res = TransformPoser()
            res.enableTranslation(False)
            if params is not None:
                res.set(params,[0.0]*3)
            return res
        else:
            res = TransformPoser()
            if params is not None:
                res.set(*params)
            return res
        if params is not None:
            res.set(params)
        return res

    def isExternallyEditable(self,type,params):
        return type in ['Configs','Trajectory']

    def onClickVisualEdit(self):
        if self.editVisualButton.isChecked():
            self.worldRobotConfig = self.world.robot(0).getConfig()
            self.klamptwidgetmaster = self.argumentWidget
        else:
            self.klamptwidgetmaster = self.worldeditmaster
        self.refresh()

    def updateValueLabel(self):
        typename = self.typeMap[self.typeCombo.currentIndex()]
        if self.constraintList.currentRow() >= 0 and self.constraintList.currentRow() < len(self.selectionItems): 
            objindex = self.selectionItems[self.constraintList.currentRow()]
            if typename == 'ik':
                s = IKSolver(self.value.robot)
                s.add(self.value.getIKObjective(objindex))
                res = s.getResidual()
                self.valueLabel.setText("Error: %0.3f"%(vectorops.norm(res),))
            else:
                self.value.q.bind(self.value.robot.getConfig())
                try:
                    print "Evaluating expression",self.value.objectives[objindex].expr
                    value = self.value.objectives[objindex].expr.evalf(self.value.context)
                    print "Done."
                    if hasattr(value,'__iter__'):
                        if typename == 'ineq':
                            value = max(value)
                            name = 'Max value'
                        else:
                            value = vectorops.norm(value)
                            name = 'Norm'
                        self.valueLabel.setText("%s: %0.3f"%(name,value))
                    else:
                        if isinstance(value,symbolic.Expression):
                            s = str(value)
                            if len(s) > 20:
                                s = s[:12]+'...'+s[-4:]
                            self.valueLabel.setText("Value: "+s)
                        else:
                            self.valueLabel.setText("Value: %0.3f"%(value,))
                except Exception as e:
                    print "Exception",e
                    import traceback
                    traceback.print_exc()
                    self.valueLabel.setText("Value: ERR")
                self.value.q.unbind()
        else:
            self.valueLabel.setText("")
    def insert(self):
        typename = self.typeMap[self.typeCombo.currentIndex()]
        self.selectionItems.append(len(self.value.objectives))
        if typename == 'ik':
            self.value.addIKObjective(ik.fixed_objective(self.value.robot.link(self.value.robot.numLinks()-1)))
        else:
            self.value.objectives.append(OptimizationObjective(symbolic.const(0),typename,1.0))
        self.constraintList.addItem(self.shortName(self.value.objectives[-1]))
    def insertFunction(self):
        fnname = str(self.functionCombo.currentText())
        print "insert function",fnname
        fn = self.value.context.function(fnname)
        if fn is None:
            print "Unable to get function",fnname,"?"
            return
        argnames = fn.argNames[:]
        print self.value.context.userData.keys()
        if fn.argTypes is not None:
            for i,t in enumerate(fn.argTypes):
                if argnames[i] in self.value.context.userData:
                    argnames[i] = symbolic_io.USER_DATA_PREFIX + argnames[i]
                elif t.char == 'U':
                    try:
                        res = types.make(t.subtype,self.value.robot)
                        if res is not None:
                            argnames[i] = json.dumps(loader.toJson(res), sort_keys=True, indent=4, separators=(',', ': '))
                        else:
                            #print "Could not make default item of type",t.subtype,"for argument",argnames[i]
                            pass
                    except Exception:
                        pass

        self.insertCode(fnname+'('+','.join(argnames)+')')
        self.onCodeChange()
    def delete(self):
        print "delete constraint"
        index = self.constraintList.currentRow()
        if index < 0:
            return
        objindex = self.selectionItems[index]
        self.selectionItems = self.selectionItems[:index] + self.selectionItems[index+1:]
        for i,v in enumerate(self.selectionItems):
            if v > objindex:
                self.selectionItems[i] -= 1
        self.value.objectives = self.value.objectives[:objindex] + self.value.objectives[objindex+1:]
        self.constraintList.clear()
        for item in self.selectionItems:
            self.constraintList.addItem(self.shortName(self.value.objectives[item]))
        self.constraintList.setCurrentRow(index-1)
        self.onConstraintSelectionChange()
    def solve(self):
        params = optimize.OptimizerParams()
        params.localMethod = 'auto'
        cost = self.value.costSymbolic()
        print "Q is bound to:",self.value.q.value
        print "Initial cost",cost
        print "Initial cost value",symbolic.expr(cost).eval({'q':self.value.robot.getConfig()})
        print "Optimization variables",[v.name for v in self.value.optimizationVariables]
        print "Q is bound to:",self.value.q.value
        print "Cost derivative",symbolic.deriv(cost,self.value.q)
        print "Cost derivative value:",symbolic.expr(symbolic.deriv(cost,self.value.q)).eval({'q':self.value.robot.getConfig()})
        try:
            res = self.value.solve(params)
            print "Solved value:",res
            print "q value",self.value.q.value
            self.value.robot.setConfig(res)
            self.robotPosers[0].set(res)
            self.worldRobotConfig = res
        except Exception as e:
            import traceback
            traceback.print_exc()
            print "Unable to solve, exception",e
        self.updateValueLabel()
        self.refresh()

    def currentVar(self):
        index = self.variableList.currentRow()
        if index < 0 or index >= self.variableList.count():
            return None,None
        newname = str(self.varNameEdit.text())
        itemname = str(self.variableList.item(index).text())
        if itemname[0] in [symbolic_io.USER_DATA_PREFIX,symbolic_io.NAMED_EXPRESSION_PREFIX]:
            itemname = itemname[1:]
            item = self.value.get(itemname)
        else:
            item = self.value.get(itemname)
        return itemname,item

    def onVarAdd(self):
        import copy
        type = self.typelist[self.varTypeCombo.currentIndex()]
        if len(type) > 1:
            var = self.value.addKlamptVar("unnamed",type)
            print "REFER TO THIS VARIABLE AS",symbolic_io.NAMED_EXPRESSION_PREFIX+"unnamed"
            print "IT SHOULD BE REPLACED WITH",symbolic_io.toStr(var.expr,parseCompatible=False)
            self.variableList.addItem(symbolic_io.NAMED_EXPRESSION_PREFIX+"unnamed")
        else:
            var = self.value.context.addVar("unnamed",type)
            var.value = copy.copy(self.typedefaultvalues[self.varTypeCombo.currentIndex()])
            self.variableList.addItem("unnamed")
        self.variableList.setCurrentRow(self.variableList.count()-1)

    def onVarDelete(self):
        print "TODO: delete variable"

    def onVarNameChange(self):
        index = self.variableList.currentRow()
        itemname,item = self.currentVar()
        if itemname == None: return
        newname = str(self.varNameEdit.text())

        #validation
        if len(newname) == 0:
            QMessageBox.information( 
                self.parent, 
                "Application Name", 
                "Names cannot be empty.")
            self.varNameEdit.setText(itemname)
            return

        if newname == itemname:
            return

        if self.value.get(newname,None):
            QMessageBox.information( 
                self.parent, 
                "Application Name", 
                "A variable with the name "+newname+" already exists." )
            self.varNameEdit.setText(itemname)
            return

        print "Variable name changed from",itemname,"to",newname
        if isinstance(item,(symbolic.Variable,robotoptimize.KlamptVariable)):
            if isinstance(item,robotoptimize.KlamptVariable):
                self.variableList.item(index).setText(symbolic_io.NAMED_EXPRESSION_PREFIX+newname)
            else:
                self.variableList.item(index).setText(newname)
            self.value.rename(itemname,newname)
            #all constraints will be automatically renamed
        else:
            self.variableList.item(index).setText(symbolic_io.USER_DATA_PREFIX+newname)
            self.value.rename(itemname,newname)
            #constraints will NOT be automatically renamed
            for obj in self.value.objectives:
                obj.expr = obj.expr.replace(symbolic.UserDataExpression(itemname),symbolic.UserDataExpression(newname))

    def onVarSelectionChange(self):
        itemname,item = self.currentVar()
        if itemname is None:
            self.varNameEdit.setEnabled(False)
            self.varSizeEdit.setEnabled(False)
            self.varOptimizeCheck.setEnabled(False)
            self.varValueEdit.setEnabled(False)
            return
        else:
            self.varNameEdit.setEnabled(True)
            self.varSizeEdit.setEnabled(True)
            self.varOptimizeCheck.setEnabled(True)
            self.varValueEdit.setEnabled(True)
        if isinstance(item,symbolic.Variable):
            self.varNameEdit.setText(item.name)
            stemp = item.type.size
            item.type.size = None
            self.varTypeLabel.setText("Type: "+str(item.type))
            item.type.size = stemp
            if item.type.char == 'N':
                self.varSizeEdit.setEnabled(False)
            else:
                self.varSizeEdit.setText(str(item.type.size))
            self.varOptimizeCheck.setChecked(any(v is item for v  in self.value.optimizationVariables))
            self.onVarValueComboChange()
        elif isinstance(item,robotoptimize.KlamptVariable):
            self.varNameEdit.setText(item.name)
            self.varTypeLabel.setText("Type: "+item.type)
            self.varSizeEdit.setEnabled(False)
            contextvariable = item.variables[0]
            self.varOptimizeCheck.setChecked(any(v is contextvariable for v in self.value.optimizationVariables))
            self.onVarValueComboChange()
        else:
            self.varOptimizeCheck.setEnabled(False)
            self.varNameEdit.setText(itemname)
            self.varTypeLabel.setText("User data")
            self.varSizeEdit.setEnabled(False)
            self.varValueCombo.setCurrentIndex(0)
            self.varValueCombo.setEnabled(False)
            try:
                jsonobj = loader.toJson(item)
                self.varValueEdit.setText(json.dumps(jsonobj))
            except Exception as e:
                print "Can't dump item",item,"exception",e
                self.varValueEdit.setText('')
        self.varShowVisual.setChecked(itemname in self.variableWidgets)

    def onVarSizeChange(self):
        itemname,item = self.currentVar()
        if itemname is None: return
        sizetext = str(self.varSizeEdit.text())
        if sizetext == 'None' or sizetext == '':
            item.type.size = None
        else:
            try:
                obj = json.loads(sizetext)
                if isinstance(obj,int) or (isinstance(obj,(tuple,list)) and all(isinstance(v) for v in obj)):
                    item.type.size = obj
                else:
                    print "Unable to resize, invalid size specified (must be integer or tuple, got %s)"%(sizetext,)
            except Exception as e:
                print "Unable to resize, invalid size %s specified"%(sizetext,)
                print e
                return
        print "Changed variable size to",sizetext
        if item.value is not None:
            print "TODO: modify the value to reflect the new size"

    def onVarOptimizeChange(self):
        itemname,item = self.currentVar()
        if itemname is None: return
        if isinstance(item,symbolic.Variable):
            if self.varOptimizeCheck.isChecked():
                self.value.optimizationVariables.append(item)
            else:
                for i,v in enumerate(self.value.optimizationVariables):
                    if v is item:
                        self.value.optimizationVariables.pop(i)
                        break
            print "Changed optimize to",self.varOptimizeCheck.isChecked()
        elif isinstance(item,robotoptimize.KlamptVariable):
            for v in item.variables:
                if self.varOptimizeCheck.isChecked():
                    self.value.optimizationVariables.append(v)
                else:
                    for i,v2 in enumerate(self.value.optimizationVariables):
                        if v2 is v:
                            self.value.optimizationVariables.pop(i)
                            break

    def onVarValueComboChange(self):
        itemname,item = self.currentVar()
        if itemname is None: return
        if isinstance(item,symbolic.Variable):
            self.varValueEdit.setEnabled(True)
            if self.varValueCombo.currentIndex() == 0:  #value
                if item.value is None:
                    if itemname == 'q':
                        self.varValueEdit.setText('Current robot config')
                        self.varValueEdit.setEnabled(False)
                    else:
                        self.varValueEdit.setText('')
                else:
                    self.varValueEdit.setText(str(item.value))
            else:
                ind = self.varValueCombo.currentIndex()-1
                if itemname in self.value.variableBounds:
                    limit = self.value.variableBounds[itemname][ind]
                    self.varValueEdit.setText(str(limit))
                else:
                    self.varValueEdit.setText('')
        elif isinstance(item,robotoptimize.KlamptVariable):
            self.varValueEdit.setEnabled(True)
            values = []
            if self.varValueCombo.currentIndex() == 0:  #value
                for var in item.variables:
                    if var.value is None:
                        values.append(None)
                    else:
                        values.append(var.value)
            else:
                ind = self.varValueCombo.currentIndex()-1
                for var in item.variables:
                    if var.name in self.value.variableBounds:
                        limit = self.value.variableBounds[var.name][ind]
                        values.append(limit)
                    else:
                        values.append(None)
            if any(v is None for v in values):
                #unbounded
                self.varValueEdit.setText('')
            else:
                if len(values) == 1:
                    value = item.decode(values[0])
                else:
                    value = item.decode(values)
                self.varValueEdit.setText(str(value))

    def onVarLoad(self):
        print "Clicked variable Load"
        itemname,item = self.currentVar()
        allowedTypes = None
        if isinstance(item,robotoptimize.KlamptVariable):
            allowedTypes = item.type
        res = resource.load(type=allowedTypes)
        if res == None: return  #cancel clicked

        fn,value = res
        types = type.objectToTypes(value,world=self.world)
        if isinstance(types,list):
            types = types[0]
        if itemname is None:
            defaultname = os.path.basename(fn)
            if types in ['Config','RigidTransform','Rotation','Matrix3','Vector3','Point']:
                self.value.addKlamptVar(defaultname,types,initialValue = value)
                self.variableList.addItem(defaultname)
            else:
                self.value.context.userData[defaultname] = value
                self.variableList.addItem(symbolic_io.USER_DATA_PREFIX+defaultname)
        elif isinstance(item,symbolic.Variable):
            #verify that the value is compatible?
            if not item.type.match(value):
                print "Loaded value is not compatible with variable of type",item.type
            else:
                item.bind(value)
        elif isinstance(item,robotoptimize.KlamptVariable):
            #TODO: verify that the value is compatible?
            item.bind(value)
        else:
            self.value.context.userData[itemname] = value

        
    def onVarShowVisualChange(self):
        itemname,item = self.currentVar()
        if item is None: return
        if itemname in ['q','robot','world']:
            return  #can't toggle q, robot, or the world
        if self.varShowVisual.isChecked():
            if isinstance(item,symbolic.Variable):  #can't show a vector or number
                return
            elif isinstance(item,robotoptimize.KlamptVariable): 
                if item.type not in self.visuallyEditableTypes:
                    return
                self.variableWidgets[itemname] = self.makeEditingWidget(item.type,item.getValue())
                self.klamptwidgetmaster.add(self.variableWidgets[itemname])
        else:
            #hide
            self.klamptwidgetmaster.remove(self.variableWidgets[itemname])
            del self.variableWidgets[itemname]

    def loadProblem(self):
        fn = QFileDialog.getOpenFileName(self, "Load optimization problem from...", "problem.json", "JSON file (*.json)")
        if fn is not None:
            with open(fn,'r') as f:
                jsonobj = json.load(f)
                self.value.fromJson(jsonobj)
                self.refresh()
    def saveProblem(self):
        fn = QFileDialog.getSaveFileName(self, "Save optimization problem to...", "problem.json", "JSON file (*.json)")
        if fn is not None:
            saveContextFunctions = self.saveContextFunctionsCheck.isChecked()
            prettyPrint = self.prettyPrintCheck.isChecked()
            with open(fn,'w') as f:
                jsonobj = self.value.toJson(saveContextFunctions=saveContextFunctions,prettyPrintExprs=prettyPrint)
                json.dump(jsonobj,f,sort_keys=True, indent=4, separators=(',', ': '))
                f.write('\n')
    def printProblem(self):
        saveContextFunctions = self.saveContextFunctionsCheck.isChecked()
        prettyPrint = self.prettyPrintCheck.isChecked()
        string = json.dumps(self.value.toJson(saveContextFunctions=saveContextFunctions,prettyPrintExprs=prettyPrint),sort_keys=True, indent=4, separators=(',', ': '))
        dialog = ShowTextDialog("Optimization problem",string)
        dialog.exec_()
        #QInputDialog.getMultiLineText(self.parent,"Optimization problem","JSON",string)
    def printCode(self):
        saveContextFunctions = self.saveContextFunctionsCheck.isChecked()
        prettyPrint = self.prettyPrintCheck.isChecked()
        problem_string = json.dumps(self.value.toJson(saveContextFunctions=saveContextFunctions,prettyPrintExprs=prettyPrint),sort_keys=True, indent=4, separators=(',', ': '))
        userdata_nondefault = [k for k in self.value.context.userData.keys() if k not in ['robot','world']]
        variables_nondefault = [v.name for v in self.value.context.variables if v.name != 'q']
        if len(userdata_nondefault) > 0:
            userdata_args = ','+','.join(userdata_nondefault)
        else:
            userdata_args = ''
        if len(variables_nondefault) > 0:
            variable_args = ','+','.join([k+'=None' for k in variables_nondefault])
        else:
            variable_args = ''
        setup_userdata = '    \n'.join(["res.context.userData['%s'] = %s"%(k,k) for k in userdata_nondefault])
        setup_variables = '    \n'.join(["res.bind('%s',%s)"%(k,k) for k in variables_nondefault])
        extract_variables = '    \n'.join(["res['%s'] = problem.context.variableDict['%s'].value"%(k,k) for k in variables_nondefault])
        boilerplate = """from klampt.plan.robotoptimize import *
from klampt.math.optimize import OptimizerParams
import json

#automatically generated from OptimizationProblemEditor
problemstring = \"\"\"{problem_string}\"\"\"

def optimization_setup(world{userdata_args}):
    \"\"\"Creates a RobotOptimizationProblem from the given user data items.  Calling sequence is:
        from klampt import *
        world = WorldModel()
        world.readFile([the world file])
        opt_problem = optimization_setup(world,...[userdata]...)
        qopt = optimization_solve(opt_problem,...[variables]...)
        if qopt is not None:
            #qopt is the optimal configuration
            print optimization_results(opt_problem)
            ...
    \"\"\"
    #create optimization problem
    res = RobotOptimizationProblem(world=world)
    res.fromJson(json.loads(problemstring))
    #set userdata variables to arguments provided
    {setup_userdata}
    return res

def optimization_solve(problem,params=OptimizerParams(),q=None{variable_args}):
    if q is not None:
        problem.bind('q',q)
    {setup_variables}
    return problem.solve(params)

def optimization_results(problem):
    \"\"\"Returns a dictionary mapping variable names to optimized values\"\"\"
    res = dict()
    res['q'] = problem.q.value
    {extract_variables}
    return res
"""
        formatted = boilerplate.format(userdata_args=userdata_args,setup_userdata=setup_userdata,variable_args=variable_args,setup_variables=setup_variables,extract_variables=extract_variables,problem_string=problem_string)
        dialog = ShowTextDialog("Optimization Python code",formatted)
        dialog.exec_()
    def display(self):
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        if self.klamptwidgetmaster == self.argumentWidget:
            self.world.robot(0).setConfig(self.worldRobotConfig)
        editors.WorldEditor.display(self)
    def mousefunc(self,button,state,x,y):
        if self.klamptwidgetmaster is self.argumentWidget:
            print "New value",self.argumentWidget.get()
            self.setCurrentValue(self.argumentWidget.get(),updateText=True)
        for k,widget in self.variableWidgets.iteritems():
            if widget.hasFocus():
                newval = widget.get()
                var = self.value.get(k)
                assert isinstance(var,robotoptimize.KlamptVariable)
                var.bind(newval)
                self.onVarSelectionChange()
        self.updateValueLabel()
        return editors.WorldEditor.mousefunc(self,button,state,x,y)

class _EditWindow(QMainWindow):
    def __init__(self,glwidget):
        QMainWindow.__init__(self)
        self.glwidget = glwidget
        #glwidget.setMinimumSize(glwidget.width,glwidget.height)
        glwidget.setMaximumSize(4000,4000)
        #glwidget.setSizePolicy(QSizePolicy(QSizePolicy.Maximum,QSizePolicy.Maximum))
        self.instructions = QLabel()
        self.description = QLabel()
        self.topBoxLayout = QVBoxLayout()
        self.topBoxLayout.addWidget(self.description)
        self.topBoxLayout.addWidget(self.instructions)
        self.extraDialog = QFrame()
        self.extraDialog.setSizePolicy(QSizePolicy(QSizePolicy.Minimum,QSizePolicy.Minimum))
        self.topBoxLayout.addWidget(self.extraDialog)
        self.topBoxLayout.setStretchFactor(self.description,1)
        self.topBoxLayout.setStretchFactor(self.instructions,1)
        self.topBoxLayout.setStretchFactor(self.extraDialog,10)
        self.layout = QVBoxLayout()
        #self.layout.addWidget(self.topBox)
        self.layout.addWidget(glwidget)
        #self.layout.setStretchFactor(glwidget,10)
        #self.layout.setStretchFactor(self.topBox,0)
        self.splitter = QSplitter(Qt.Horizontal)
        left = QFrame()
        right = QFrame()
        left.setLayout(self.topBoxLayout)
        right.setLayout(self.layout)
        self.splitter.setHandleWidth(7)
        self.splitter.addWidget(left)
        self.splitter.addWidget(right)
        self.setCentralWidget(self.splitter)
        #self.splitter.setSizes([self.topBoxLayout.sizeHint().height(),self.layout.sizeHint().height()])

    def setEditor(self,editorObject):
        self.editorObject = editorObject
        self.setWindowTitle("Editing "+editorObject.name)
        if editorObject.description==None:
            self.description.setText("")
        else:
            self.description.setText(editorObject.description)
        self.instructions.setText(editorObject.instructions())
        editorObject.addDialogItems(self.extraDialog,ui='qt')

    def closeEvent(self,event):
        from klampt import vis
        reply = QMessageBox.question(self, 'Message',
             "Are you sure to quit the program?", QMessageBox.Yes | 
             QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            res = QMainWindow.closeEvent(self,event)
            vis.show(False)
            return
        return

def test_editor():
    """Tests the visual optimization editor"""
    from klampt import vis
    world = WorldModel()
    world.readFile("../../data/tx90scenario0.xml")
    robot = world.robot(0)

    print "Optimization methods available:",optimize.LocalOptimizer.methodsAvailable()

    opt = robotoptimize.RobotOptimizationProblem(world=world)

    editorObject = OptimizationProblemEditor('opt',opt,'Optimization problem',world) 
    #editors.run(editorObject)
    old_vis_window = vis.getWindow()
    editor_window = vis.createWindow("Optimization Problem Editor")
    vis.setPlugin(editorObject)
    def makefunc(gl_backend):
        assert gl_backend is not None
        res = _EditWindow(gl_backend)
        res.setEditor(editorObject)
        return res
    vis.customUI(makefunc)
    vis.show()
    while vis.shown():
        time.sleep(0.2)

    opt = editorObject.value
    vis.setPlugin(None)
    vis.customUI(None)
    vis.setWindow(old_vis_window)

    print "Testing random restart 10..."
    params = optimize.OptimizerParams()
    params.localMethod = 'auto'
    params.globalMethod = 'random-restart'
    params.numRestarts = 10
    res = opt.solve(params)
    print "Optimization result:",res
    if res:
        robot.setConfig(opt.q.value)
        opt.q.bind(robot.getConfig())
        print "  Cost:",opt.cost()
        print "  Feasible?",opt.isFeasible()
    

    #test saving and loading
    """
    opt2 = robotoptimize.RobotOptimizationProblem(world.robot(0))
    opt2.fromJson(opt.toJson(),context)
    print "Cost:",opt2.cost(res)
    print "Feasible?",opt2.isFeasible(res)
    """


    
        
if __name__ == '__main__':
    templates = {'1':test_symbolic,'2':test_parse,'3':test_simple,
                '4':test_rosenbrock,'5':test_ik,'6':test_robotoptimize,
                '7':test_editor,'8':test_error,'9':test_sympy}
    print "Available templates"
    import inspect
    for k in sorted(templates.keys()):
        fname = 'untitled'
        for x in inspect.getmembers(templates[k]):
            if x[0] == '__name__':
                fname = x[1]
        print " %s) %s: %s"%(k,fname,inspect.getdoc(templates[k]))
    entry = raw_input("Which template do you want to run? (1-%d) > "%(len(templates),))
    if entry not in templates:
        print "Invalid selection"
        exit(1)
    templates[entry]()
