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
from klampt.math import symbolic,optimize,vectorops

from klampt.vis import editors
from klampt.model import types
from klampt.io import loader
from klampt import RobotPoser,PointPoser,TransformPoser
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import json
import unicodedata
from OpenGL.GL import *

class SmallListWidget(QListWidget):
    def sizeHint(self):
        s = QSize()
        s.setHeight(80)
        s.setWidth(QListWidget.sizeHint(self).width())
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
        self.value = value
        self.allFunctions = sorted(value.context.listFunctions(doprint=False))
        self.argumentWidget = None
        self.worldeditmaster = self.klamptwidgetmaster
    def instructions(self):
        return "Use the controls to change the optimization problem.\nRight-click on items in the world to change the solver starting configuration."
    def addDialogItems(self,parent,ui='qt'):
        self.parent = parent
        playout = QHBoxLayout(parent)
        self.tabWidget = QTabWidget()
        playout.addWidget(self.tabWidget)
        firstTab = QWidget()
        secondTab = QWidget()
        thirdTab = QWidget()
        self.tabWidget.addTab(firstTab,"Objectives")
        self.tabWidget.addTab(secondTab,"Variables")
        self.tabWidget.addTab(thirdTab,"Problem")
        self.tabWidget.setSizePolicy(QSizePolicy(QSizePolicy.Expanding,QSizePolicy.Expanding))

        #constraint editing
        hlayout = QHBoxLayout(firstTab)
        left = QGridLayout()
        right = QGridLayout()
        hlayout.addLayout(left)
        hlayout.addLayout(right)
        #left.setSizePolicy(QSizePolicy(QSizePolicy.Expanding,QSizePolicy.Fixed))
        #left.resize(350,200)
        #right.setSizePolicy(QSizePolicy(QSizePolicy.Expanding,QSizePolicy.Fixed))
        #right.resize(350,200)
        self.typeCombo = QComboBox()
        self.typeCombo.addItem("Cost")
        self.typeCombo.addItem("IK objective")
        self.typeCombo.addItem("Equality")
        self.typeCombo.addItem("Inequality")
        self.typeCombo.addItem("Feasibility test")
        self.typeMap = ['cost','ik','eq','ineq','feas']
        self.constraintList = SmallListWidget()
        left.addWidget(self.typeCombo,0,0,1,3)
        left.addWidget(self.constraintList,1,0,1,3)
        self.addButton = QPushButton("Add")
        self.deleteButton = QPushButton("Delete")
        self.weightLabel = QLabel("Weight")
        self.weightSpin = QDoubleSpinBox()
        self.valueLabel = QLabel("Value: ")
        self.solveButton = QPushButton("Solve")
        self.objectiveNameEdit = QLineEdit("Name")
        
        left.addWidget(self.addButton,2,0,1,2)
        left.addWidget(self.deleteButton,2,2)
        
        left.addWidget(self.valueLabel,3,0)
        left.addWidget(self.weightLabel,3,1)
        left.addWidget(self.weightSpin,3,2)
        left.addWidget(self.objectiveNameEdit,4,0,1,3)
        left.addWidget(self.solveButton,5,0,1,3)

        self.functionCombo = QComboBox()
        for f in self.allFunctions:
            self.functionCombo.addItem(f)
        self.insertFunctionButton = QPushButton("Insert Func")
        self.codeEdit = SmallTextEdit()
        self.simplifyButton = QPushButton("Simplify")
        self.derivativeButton = QPushButton("Derivative")
        self.editVisualButton = QPushButton("Visual Edit")
        self.editVisualButton.setCheckable(True)

        right.addWidget(self.functionCombo,0,0,1,2)
        right.addWidget(self.insertFunctionButton,1,0,1,2)
        right.addWidget(self.codeEdit,2,0,1,2)
        right.addWidget(self.simplifyButton,3,0,1,1)
        right.addWidget(self.derivativeButton,3,1,1,1)
        right.addWidget(self.editVisualButton,4,0,1,2)
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
        hlayout = QHBoxLayout(secondTab)
        self.variableList = SmallListWidget()
        self.addVarButton = QPushButton("Add")
        self.deleteVarButton = QPushButton("Delete")
        vlayout = QVBoxLayout()
        vlayout.addWidget(self.variableList)
        vlayout.addWidget(self.addVarButton)
        vlayout.addWidget(self.deleteVarButton)
        hlayout.addLayout(vlayout)
        vlayout = QVBoxLayout()
        self.varNameEdit = QLineEdit("Name")
        self.varTypeCombo = QComboBox()
        self.varTypeCombo.addItem("Numeric")
        self.varTypeCombo.addItem("Vector")
        self.varTypeCombo.addItem("Matrix")
        self.varTypeCombo.addItem("User data")
        self.varOptimizeCheck = QCheckBox("Optimize")
        self.varSizeEdit = QLineEdit("Size")
        self.varValueCombo = QComboBox()
        for name in ['Value','Minimum','Maximum']:
            self.varValueCombo.addItem(name)
        self.varValueEdit = QLineEdit("Value")
        self.varLoad = QPushButton("Load...")
        self.varShowVisual = QCheckBox("Show in visualization")
        for k,v in self.value.context.variableDict.iteritems():
            self.variableList.addItem("$"+k)
        for k,v in self.value.context.userData.iteritems():
            self.variableList.addItem(k)
        vlayout.addWidget(self.varNameEdit)
        vlayout.addWidget(self.varOptimizeCheck)
        sizelayout = QHBoxLayout()
        sizelayout.addWidget(self.varTypeCombo)
        sizelayout.addWidget(QLabel("Size:"))
        sizelayout.addWidget(self.varSizeEdit)
        vlayout.addLayout(sizelayout)
        valueLayout = QHBoxLayout()
        valueLayout.addWidget(self.varValueCombo)
        valueLayout.addWidget(self.varValueEdit)
        vlayout.addLayout(valueLayout)
        vlayout.addWidget(self.varLoad)
        vlayout.addWidget(self.varShowVisual)
        hlayout.addLayout(vlayout)
        self.varNameEdit.editingFinished.connect(self.onVarNameChange)
        self.variableList.itemSelectionChanged.connect(self.onVarSelectionChange)
        self.varTypeCombo.activated.connect(self.onVarTypeChange)
        self.varSizeEdit.textChanged.connect(self.onVarSizeChange)
        self.varOptimizeCheck.clicked.connect(self.onVarOptimizeChange)
        self.varLoad.clicked.connect(self.onVarLoad)
        self.varShowVisual.clicked.connect(self.onVarShowVisualChange)

        #problem 
        hlayout = QHBoxLayout(thirdTab)
        column1 = QVBoxLayout()
        column2 = QVBoxLayout()
        hlayout.addLayout(column1)
        hlayout.addLayout(column2)
        self.loadProblemButton = QPushButton("Load problem...")
        self.saveProblemButton = QPushButton("Save problem...")
        self.printProblemButton = QPushButton("Print problem")
        self.prettyPrintCheck = QCheckBox("Pretty-print expressions")
        self.saveContextFunctionsCheck = QCheckBox("Save context functions")
        self.prettyPrintCheck.setChecked(True)
        self.saveContextFunctionsCheck.setChecked(False)
        column1.addWidget(self.loadProblemButton)
        column1.addWidget(self.saveProblemButton)
        column1.addWidget(self.printProblemButton)
        column2.addWidget(self.prettyPrintCheck)
        column2.addWidget(self.saveContextFunctionsCheck)
        self.loadProblemButton.clicked.connect(self.loadProblem)
        self.saveProblemButton.clicked.connect(self.saveProblem)
        self.printProblemButton.clicked.connect(self.printProblem)

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
        for var in self.value.context.variables:
            dexpr = symbolic.deriv(expr,var)
            dstr = symbolic_io.exprToStr(dexpr,parseCompatible=True)
            res.append(var.name+":")
            res.append("  "+dstr)
        dialog = ShowTextDialog("Derivatives",'\n'.join(res))
        dialog.exec_()

    def makeEditingWidget(self,type,params):
        if type not in ['Config','Vector','Vector3','Point','Matrix3','Rotation','RigidTransform']:
            return None
        if type in ['Config','Vector']:
            res = RobotPoser(self.world.robot(0))
        elif type in ['Vector3','Point']:
            res = PointPoser()
        elif type in ['Matrix3','Rotation']:
            res = TransformPoser()
            res.enableTranslation(False)
        else:
            res = RigidTransform()
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
        print "insert",fnname
        fn = self.value.context.function(fnname)
        if fn is None:
            print "Unable to get function",fnname,"?"
            return
        argnames = fn.argNames[:]
        if fn.argTypes is not None:
            for i,t in enumerate(fn.argTypes):
                if len(t) > 1:
                    res = types.make(t,self.value.robot)
                    if res is not None:
                        argnames[i] = json.dumps(loader.toJson(res), sort_keys=True, indent=4, separators=(',', ': '))
                    else:
                        print "Could not make default item of type",t,"for argument",argnames[i]

        self.insertCode(fnname+'('+','.join(argnames)+')')
        self.onCodeChange()
    def delete(self):
        print "delete"
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

    def onVarNameChange(self):
        index = self.variableList.currentRow()
        if index < 0 or index >= self.variableList.count():
            return
        newname = str(self.varNameEdit.text())
        itemname = str(self.variableList.item(index).text())
        if itemname[0] == '$':
            itemname = itemname[1:]
            item = self.value.context.get(itemname)
        else:
            item = self.value.context.get(itemname)

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

        if self.value.context.get(newname,None):
            QMessageBox.information( 
                self.parent, 
                "Application Name", 
                "A variable with the name "+newname+" already exists." )
            self.varNameEdit.setText(itemname)
            return

        print "Variable name changed from",itemname,"to",newname
        if isinstance(item,symbolic.Variable):
            self.variableList.item(index).setText('$'+newname)
            self.value.context.renameVar(item,newname)
            #all constraints will be automatically renamed
        else:
            self.variableList.item(index).setText(newname)
            self.value.context.renameUserData(itemname,newname)
            #constraints will NOT be automatically renamed
            for obj in self.value.objectives:
                obj.expr = obj.expr.replace(symbolic.UserDataExpression(itemname),symbolic.UserDataExpression(newname))

    def onVarSelectionChange(self):
        index = self.variableList.currentRow()
        if index < 0 or index >= self.variableList.count():
            self.varNameEdit.setEnabled(False)
            self.varTypeCombo.setEnabled(False)
            self.varSizeEdit.setEnabled(False)
            self.varOptimizeCheck.setEnabled(False)
            return
        else:
            self.varNameEdit.setEnabled(True)
            self.varTypeCombo.setEnabled(True)
            self.varSizeEdit.setEnabled(True)
            self.varOptimizeCheck.setEnabled(True)
        itemname = str(self.variableList.item(index).text())
        if itemname[0] == '$':
            itemname = itemname[1:]
            item = self.value.context.get(itemname)
        else:
            item = self.value.context.get(itemname)
            self.varSizeEdit.setEnabled(False)
            self.varOptimizeCheck.setEnabled(False)
        tmap = {'N':0,'V':1,'M':2,'U':3}
        if isinstance(item,symbolic.Variable):
            self.varNameEdit.setText(item.name)
            self.varTypeCombo.setCurrentIndex(tmap[item.type])
            if item.type == 'N':
                self.varSizeEdit.setEnabled(False)
            else:
                self.varSizeEdit.setText(str(item.size))
            self.varOptimizeCheck.setChecked((item in self.value.optimizationVariables))
        else:
            self.varNameEdit.setText(itemname)
            self.varTypeCombo.setCurrentIndex(tmap['U'])

    def onVarTypeChange(self):
        print "Changed variable type to",self.varTypeCombo.currentText()
        print "TODO"
    def onVarSizeChange(self):
        print "Changed variable size to",self.varSizeEdit.text()
        print "TODO"
    def onVarOptimizeChange(self):
        print "Changed optimize to",self.varOptimizeCheck.isChecked()
        print "TODO"
    def onVarLoad(self):
        print "Clicked variable Load"
        print "TODO"
    def onVarShowVisualChange(self):
        print "Changed variable show visual"
        print "TODO"
    def loadProblem(self):
        print "TODO"
    def saveProblem(self):
        print "TODO"
    def printProblem(self):
        saveContextFunctions = self.saveContextFunctionsCheck.isChecked()
        prettyPrint = self.prettyPrintCheck.isChecked()
        print "Save context functions",saveContextFunctions
        print "Pretty print",prettyPrint
        string = json.dumps(self.value.toJson(saveContextFunctions=saveContextFunctions,prettyPrintExprs=prettyPrint),sort_keys=True, indent=4, separators=(',', ': '))
        dialog = ShowTextDialog("Optimization problem",string)
        dialog.exec_()
        #QInputDialog.getMultiLineText(self.parent,"Optimization problem","JSON",string)
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
        self.updateValueLabel()
        return editors.WorldEditor.mousefunc(self,button,state,x,y)

def test_editor():
    """Tests the visual optimization editor"""
    world = WorldModel()
    world.readFile("../../data/tx90scenario0.xml")
    robot = world.robot(0)

    print "Methods available:",optimize.LocalOptimizer.methodsAvailable()

    opt = robotoptimize.RobotOptimizationProblem(world=world)

    editorObject = OptimizationProblemEditor('opt',opt,'Optimization problem',world) 
    editors.run(editorObject)

    params = optimize.OptimizerParams()
    params.localMethod = 'auto'
    params.globalMethod = 'random-restart'
    res = opt.solve(params)
    print "Cost:",opt.cost(res)
    print "Feasible?",opt.isFeasible(res)
    print "Result:",res

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
