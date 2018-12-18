from ..model import ik,types,config
from ..math import vectorops
from ..robotsim import IKSolver,IKObjective
from ..io import loader
import time
import random
from ..math import optimize,symbolic,symbolic_klampt,so3,se3
import numpy as np

class KlamptVariable:
    """
    Members:
    - name: the Klamp't item's name
    - type: the Klamp't item's type
    - encoding: the way in which the item is encoded in the optimization
    - variables: the list of Variables encoding this Klamp't item 
    - expr: the Expression that will be used to replace the symbolic mainVariable via appropriate variables
    - constraints, encoder, decoder: internally used
    """
    def __init__(self,name,type):
        self.name = name
        self.type = type
        self.encoding = None
        self.variables = None
        self.expr = None
        self.constraints = []
        self.encoder = None
        self.decoder = None
    def bind(self,obj):
        """Binds all Variables associated with this to the value of Klamp't object obj"""
        if self.type in ['Config','Vector','Vector3','Point']:
            self.variables[0].bind(obj)
        elif self.type == 'Configs':
            assert len(obj) == len(self.variables),"Invalid number of configs in Configs object"
            for i,v in enumerate(obj):
                self.variables[i].bind(v)
        elif self.type == 'Rotation':
            if self.encoder is None:
                self.variables[0].bind(obj)
            else:
                self.variables[0].bind(self.encoder(obj))
        elif self.type == 'RigidTransform':
            if self.encoder is None:
                self.variables[0].bind(obj[0])
                self.variables[1].bind(obj[1])
            else:
                T = self.encoder(obj)
                self.variables[0].bind(T[0])
                self.variables[1].bind(T[1])
        else:
            raise ValueError("Unsupported object type "+self.type)
    def getParams(self):
        """Returns the list of current parameters bound to the symbolic Variables."""
        if len(self.variables) > 1:
            return [v.value for v in self.variables]
        else:
            return self.variables[0].value
    def getValue(self):
        """Returns the Klamp't value corresponding to the current bound parameters."""
        return self.decode(self.getParams())
    def unbind(self):
        """Unbinds all Variables associated with this."""
        for v in self.variables:
            v.unbind()
    def encode(self,obj):
        """Returns the parameters giving the encoding of the Klamp't object obj"""
        if self.encoder is None:
            return obj
        else:
            return self.encoder(obj)
    def decode(self,params):
        """Returns the Klamp't object given a parameters encoding it"""
        if self.decoder is None:
            return params
        else:
            return self.decoder(params)

class RobotOptimizationProblem(optimize.OptimizationProblemBuilder):
    """Defines a generalized optimization problem for a robot, which is a subclass of
    OptimizationProblemBuilder. This may easily incorporate IK constraints, and may
    have additional specifications of active DOF.

    Members:
    - robot, world: the RobotModel and WorldModel for this optimization
    - context (inherited): a symbolic.KlamptContext that stores the variable $q denoting
      the robot configuration as well as any user data.  User data "robot" and "world" are
      available by default.
    - q: the symbolic.Variable that is the primary optimization variable.
    - activeDofs: the list of active robot DOFs.
    - autoLoad: a dictionary of (userDataName:fileName) pairs that are stored so that user data
      is automatically loaded from files. I.e., upon self.loadJson(), for each pair in autoLoad
      the command self.context.userData[userDataName] = loader.load(fileName) is executed.
    - managedVariables: a dictionary of KlamptVariables like rotations and rigid transforms.
      Managed variables should be referred to in parsed expressions with the prefix @name, 
      and are encoded into optimization form and decoded from optimization form
      using KlamptVariable.bind / KlamptVariable.unbind.  You can also retrieve the Klampt value
      by KlamptVariable.getValue().
    
    If you would like to find the configuration *closest* to solving the
    IK constraints, either add the IK constraints one by one with weight=1 (or some other
    numeric value), or call enableSoftIK() after the constraints have been added.  In this
    case, solve will always return a solution, as long as it finds a configuration that
    passes the feasibility tests. The optimization method changes so that it 1) optimizes
    the residual norm, and then 2) optimizes the cost function to maintain the residual
    norm at its current value.  In other words, minimizing error is the first priority and
    minimizing cost is the second priority.
    """
    def __init__(self,robot=None,world=None,*ikgoals):
        self.robot = robot
        self.world = world
        if self.world is not None and robot is None and self.world.numRobots() > 0:
            robot = self.world.robot(0)
            self.robot = robot
        context = symbolic_klampt.KlamptContext()
        context.addUserData("robot",self.robot)
        if self.world:
            context.addUserData("world",self.world)
        optimize.OptimizationProblemBuilder.__init__(self,context)

        self.activeDofs = None
        self.autoLoad = dict()
        nlinks = robot.numLinks() if robot is not None else None
        self.q = self.context.addVar('q','V',nlinks)
        self.managedVariables = dict()
        self.optimizationVariables = [self.q]
        self.setJointLimits()
        for goal in ikgoals:
            self.addIKObjective(goal)

    def isIKObjective(self,index):
        """Returns True if the indexed constraint is an IKObjective"""
        if self.objectives[index].type != "eq":
            return False
        return symbolic.is_op(self.objectives[index].expr,'ik.residual')
    def getIKObjective(self,index):
        """Returns the IKObjective the indexed constraint is an IKObjective"""
        res = self.objectives[index].expr.args[0]
        assert isinstance(res,symbolic.ConstantExpression) and isinstance(res.value,IKObjective),"Not an IK objective: "+str(self.objectives[index].expr)
        return res.value
    def enableSoftIK(self,enabled=True):
        """Turns on soft IK solving.  This is the same as hard IK solving if all
        constraints can be reached, but if the constraints cannot be reached, it will
        try to optimize the error.
        """
        for i,o in enumerate(self.objective):
            if self.isIKObjective(i):
                o.soft = not o.soft
    def addIKObjective(self,obj,weight=None):
        """Adds a new IKObjective to the problem.  If weight is not None, it is 
        added as a soft constraint."""
        assert isinstance(obj,IKObjective)
        self.addEquality(self.context.ik.residual(obj,self.context.setConfig("robot",self.q)),weight)
        if hasattr(obj,'robot'):
            if self.robot is None:
                self.robot = obj.robot
            else:
                assert self.robot.index == obj.robot.index,"All objectives must be on the same robot"
    def addUserData(self,name,fn):
        """Adds an auto-loaded userData.  Raises an exception if fn cannot be loaded.

        Arguments:
        - name: the name of the userData.
        - fn: the file from which it is loaded.  It must be loadable with loader.load.
        """
        assert isinstance(fn,str)
        obj = loader.load(fn)
        self.context.addUserData(name,obj)
        self.autoLoad[name] = fn
    def addKlamptVar(self,name,type=None,initialValue=None,encoding='auto',constraints=True,optimize=True):
        """Adds one or more variables of a given Klamp't type (e.g., "Config", "Rotation", "RigidTransform"). 
        If necessary, constraints on the object will also be added, e.g., joint limits, or a quaternion unit
        norm constraint.

        At least one of type / initialValue must be provided.

        Arguments:
        - name: a name for the variable.
        - type: a supported variable type (default None determines the type by initialValue).  Supported
          types include "Config", "Configs", Rotation", "RigidTransform", "Vector3".  Future work may support
          Trajectory and other types.
        - initialValue: the configuration of the variable.  If it's a float, the type will be set to 
          numeric, if it's a list it will be set to a vector, or if its a supported object, the type will be set
          appropriately and config.getConfig(initialValue) will be used for its parameter setting.
        - encoding: only supported for Rotation and RigidTransform types, and defines how the variable will be
          parameterized.  Can be:
            - 'rotation_vector' (default) for rotation vector,  3 parameters
            - 'quaternion' for quaternion encoding, 4 parameters + 1 constraint
            - 'rpy' for roll-pitch-yaw euler angles, 3 parameters
            - None for full rotation matrix (9 parameters, 6 constraints)
            - 'auto' (equivalent to to 'rotation_vector')
        - constraints: True if all default constraints are to be added.  For Config / Configs types,
          bound constraints at the robot's joint limits are added.
        - optimize: If True, adds the variables to the list of optimization variables.

        Returns a KlamptVariable containing information about the encoding of the object. 
        Note that extra symbolic Variable names may be decorated with extensions in the form of "_ext" if
        the encoding is not direct.
        """
        if type is None:
            assert initialValue is not None,"Either type or initialValue must be provided"
            type = types.objectToTypes(initialValue)
        if type in ['Vector3','Point']:
            if initialValue is None:
                initialValue = [0.0]*3
            else:
                assert len(initialValue)==3
            type = 'Vector'

        def default(name,value):
            v = self.context.addVar(name,"V",len(value))
            v.value = value[:]
            return v

        if name in self.managedVariables:
            raise ValueError("Klamp't variable name "+name+" already defined")
        kv = KlamptVariable(name,type)

        if type == 'Config':
            if initialValue is None:
                initialValue = self.robot.getConfig()
            else:
                assert len(initialValue) == self.robot.numLinks()
            v = default(name,initialValue)
            if constraints:
                self.setBounds(v.name,*self.robot.getJointLimits())
                kv.constraints = [self.robot.getJointLimits()]
        elif type == 'Vector':
            assert initialValue is not None,"Need to provide initialValue for "+type+" type variables"
            v = default(name,initialValue)
            kv.expr = VariableExpression(v)
        elif type == 'Configs':
            assert initialValue is not None,"Need to provide initialValue for "+type+" type variables"
            vals = []
            for i,v in enumerate(initialValue):
                vals.append(default(name+"_"+str(i),v))
                if constraints:
                    self.setBounds(vals[-1].name,*self.robot.getJointLimits())
                    kv.constraints.append(self.robot.getJointLimits())
            kv.variables = vals
            kv.expr = symbolic.list_(*vals)
        elif type == 'Rotation':
            if encoding == 'auto': encoding='rotation_vector'
            if encoding == 'rotation_vector':
                if initialValue is not None:
                    initialValue2 = so3.rotation_vector(initialValue)
                else:
                    initialValue = so3.identity()
                    initialValue2 = [0.0]*3
                v = default(name+"_rv",initialValue2)
                kv.expr = self.context.so3.from_rotation_vector(v)
                kv.decoder = so3.from_rotation_vector
                kv.encoder = so3.rotation_vector
            elif encoding == 'quaternion':
                if initialValue is not None:
                    initialValue2 = so3.quaternion(initialValue)
                else:
                    initialValue = so3.identity()
                    initialValue2 = [1,0,0,0]
                v = default(name+"_q",initialValue2)
                kv.expr = self.context.so3.from_quaternion(v)
                kv.decoder = so3.from_quaternion
                kv.encoder = so3.quaternion
                if constraints:
                    f = self.addEquality(self.context.so3.quaternion_constraint(v))
                    f.name = name+"_q_constraint"
                    kv.constraints = [f]
            elif encoding == 'rpy':
                if initialValue is not None:
                    initialValue2 = so3.rpy(initialValue)
                else:
                    initialValue = so3.identity()
                    initialValue2 = [0.0]*3
                v = default(name+"_rpy",initialValue2)
                kv.expr = self.context.so3.from_rpy(v)
                kv.decoder = so3.from_rpy
                kv.encoder = so3.rpy
            elif encoding is None:
                if initialValue is None:
                    initialValue = so3.identity()
                v = self.addVar(name,"Vector",initialValue)
                if constraints:
                    f = self.addEquality(self.context.so3.eq_constraint(v))
                    f.name = name+"_constraint"
                    kv.constraints = [f]
            else:
                raise ValueError("Invalid encoding "+str(encoding))
            kv.encoding = encoding
        elif type == 'RigidTransform':
            if initialValue is None:
                Ri,ti = None,[0.0]*3
            else:
                Ri,ti = initialValue
            kR = self.addKlamptVar(name+'_R','Rotation',Ri,constraints=constraints,encoding=encoding)
            t = default(name+'_t',ti)
            kv.variables = kR.variables+[t]
            kv.constraints = kR.constraints
            kv.expr = symbolic.list_(kR.expr,t)
            kv.encoding = encoding
            if kR.encoder is not None:
                kv.encoder = lambda T:(kR.encoder(T[0]),T[1])
                kv.decoder = lambda T:(kR.decoder(T[0]),T[1])
            del self.managedVariables[kR.name]
        else:
            raise ValueError("Unsupported object type "+type)

        if kv.variables is None:
            kv.variables = [v]
        if kv.expr is None:
            kv.expr = symbolic.VariableExpression(v)
        self.context.addExpr(name,kv.expr)
        if optimize:
            for v in kv.variables:
                self.optimizationVariables.append(v)
        self.managedVariables[name] = kv
        return kv

    def get(self,name,defaultValue=None):
        """Returns a Variable or UserData in the context, or a managed KlamptVariable.  If the item
        does not exist, defaultValue is returned.
        """
        if name in self.managedVariables:
            return self.managedVariables[name]
        else:
            return self.context.get(name,defaultValue)

    def rename(self,itemname,newname):
        """Renames a Variable, UserData, or managed KlamptVariable."""
        if itemname in self.managedVariables:
            item = self.managedVariables[itemname]
            del self.managedVariables[itemname]
            item.name = newname
            print("Renaming KlamptVariable",itemname)
            self.context.expressions[newname] = self.context.expressions[itemname]
            del self.context.expressions[itemname]
            for var in item.variables:
                varnewname = newname + var.name[len(itemname):]
                print("  Renaming internal variable",var.name,"to",varnewname)
                if var.name in self.variableBounds:
                    self.variableBounds[varnewname] = self.variableBounds[var.name]
                    del self.variableBounds[var.name]
                self.context.renameVar(var,varnewname)
            self.managedVariables[newname] = item
        elif itemname in self.context.userData:
            self.context.renameUserData(itemname,newname)
        else:
            var = self.context.variableDict[itemname]
            if var.name in self.variableBounds:
                self.variableBounds[newname] = self.variableBounds[var.name]
                del self.variableBounds[var.name]
            self.context.renameVar(var,newname)
    
    def setActiveDofs(self,links):
        """Sets the list of active DOFs.  These may be indices, RobotModelLinks, or strings."""
        self.activeDofs = []
        for v in links:
            if isinstance(v,str):
                self.activeDofs.append(self.robot.link(v).index)
            elif isinstance(v,RobotModelLink):
                self.activeDofs.append(v.index)
            else:
                assert isinstance(v,int)
                self.activeDofs.append(v)
    def enableDof(self,link):
        """Enables an active DOF.  If this is the first time enableDof is called,
        this initializes the list of active DOFs to the single link.  Otherwise
        it appends it to the list.  (By default, all DOFs are enabled)"""
        if isinstance(link,str):
            link = self.robot.link(link).index
        elif isinstance(link,RobotModelLink):
            self.activeDofs.append(link.index)
        else:
            assert isinstance(link,int)
        if self.activeDofs is None:
            self.activeDofs = [link]
        else:
            if link not in self.activeDofs:
                self.activeDofs.append(link)

    def disableJointLimits(self):
        """Disables joint limits.  By default, the robot's joint limits are
        used."""
        self.setBounds("q",None,None)

    def setJointLimits(self,qmin=None,qmax=None):
        """Sets the joint limits to the given lists qmin,qmax.  By default,
        the robot's joint limits are used."""
        if qmin is None:
            self.setBounds("q",*self.robot.getJointLimits())
            return
        #error checking
        assert(len(qmin)==len(qmax))
        if len(qmin)==0:
            #disabled bounds
            self.setBounds("q",None,None)
        else:
            if self.activeDofs is not None:
                assert(len(qmin)==len(self.activeDofs))
                raise NotImplementedError("What to do when you set joint limits on a subset of DOFS?")
            else:
                if self.robot is not None:
                    assert(len(qmin) == self.robot.numLinks())
        self.setBounds("q",qmin,qmax)
    def inJointLimits(self,q):
        """Returns True if config q is in the currently set joint limits."""
        qmin,qmax = self.variableBounds.get('q',self.robot.getJointLimits())
        if len(qmin) == 0:
            return True
        if len(qmin) > 0:
            for v,a,b in zip(q,qmin,qmax):
                if v < a or v > b:
                    return False
        return True    

    def toJson(self,saveContextFunctions=False,prettyPrintExprs=False):
        res = optimize.OptimizationProblemBuilder.toJson(self,saveContextFunctions,prettyPrintExprs)
        if self.activeDofs is not None:
            res['activeDofs'] = self.activeDofs
        if len(self.managedVariables) > 0:
            varobjs = []
            for (k,v) in self.managedVariables.items():
                varobj = dict()
                assert k == v.name
                varobj['name'] = v.name
                varobj['type'] = v.type
                varobj['encoding'] = v.encoding
                varobjs.append(varobj)
            res['managedVariables'] = varobjs
        if len(self.autoLoad) > 0:
            res['autoLoad'] = self.autoLoad
        return res

    def fromJson(self,obj,doAutoLoad=True):
        """Loads from a JSON-compatible object.

        - obj: the JSON-compatible object
        - doAutoLoad: if True, performs the auto-loading step. An IOError is raised if any item can't
          be loaded.
        """
        optimize.OptimizationProblemBuilder.fromJson(self,obj)
        if 'activeDofs' in obj:
            self.activeDofs = obj['activeDofs']
        else:
            self.activeDofs = None
        assert 'q'in self.context.variableDict,'Strange, the loaded JSON file does not have a configuration q variable?'
        self.q = self.context.variableDict['q']
        if 'managedVariables' in obj:
            self.managedVariables = dict()
            for v in obj['managedVariables']:
                name = v['name']
                type = v['type']
                encoding = v['encoding']
                raise NotImplementedError("TODO: load managed variables from disk properly")
                self.managedVariables[name] = self.addKlamptVar(name,type,encoding)
        if doAutoLoad:
            self.autoLoad = obj.get('autoLoad',dict())
            for (name,fn) in self.autoLoad.items():
                try:
                    obj = loader.load(fn)
                except Exception:
                    raise IOError("Auto-load item "+name+": "+fn+" could not be loaded")
                self.context.addUserData(name,obj)

    def solve(self,params=optimize.OptimizerParams()):
        """Locally or globally solves the given problem (using the robot's current configuration
        as a seed if params.startRandom=False).  Returns the solution configuration or
        None if failed.

        Arguments:
        - params: an OptimizerParams object.
        """
        if len(self.objectives) == 0:
            print("Warning, calling solve without setting any constraints?")
            return self.robot.getConfig()
        robot = self.robot
        solver = IKSolver(robot)
        for i,obj in enumerate(self.objectives):
            if self.isIKObjective(i):
                ikobj = self.getIKObjective(i)
                ikobj.robot = self.robot
                solver.add(ikobj)
        if self.activeDofs is not None:
            solver.setActiveDofs(self.activeDofs)
            ikActiveDofs = self.activeDofs
        if 'q' in self.variableBounds:
            solver.setJointLimits(*self.variableBounds['q'])
        qmin,qmax = solver.getJointLimits()
        if len(qmin)==0:
            qmin,qmax = self.robot.getJointLimits()
        backupJointLimits = None
        if self.activeDofs is None:
            #need to distinguish between dofs that affect feasibility vs IK
            ikActiveDofs = solver.getActiveDofs()
            if any(obj.type != 'ik' for obj in self.objectives):
                activeDofs = [i for i in range(len(qmin)) if qmin[i] != qmax[i]]
                activeNonIKDofs = [i for i in activeDofs if i not in ikActiveDofs]
                ikToActive = [activeDofs.index(i) for i in ikActiveDofs]
            else:
                activeDofs = ikActiveDofs
                nonIKDofs = []
                ikToActive = list(range(len(activeDofs)))
        else:
            activeDofs = ikActiveDofs
            activeNonIKDofs = []
            ikToActive = list(range(len(ikActiveDofs)))
        anyIKProblems = False
        anyCosts = False
        softIK = False
        for obj in self.objectives:
            if obj.type == 'ik':
                anyIKProblems = True
                if obj.soft:
                    softIK = True
            elif obj.type == 'cost' or obj.soft:
                anyCosts = True

        #sample random start point
        if params.startRandom:
            self.randomVarBinding()
            solver.sampleInitial()
            if len(activeNonIKDofs)>0:
                q = robot.getConfig()
                for i in activeNonIKDofs:
                    q[i] = random.uniform(qmin[i],qmax[i])
                robot.setConfig(q)
        if params.localMethod is not None or params.globalMethod is not None or (anyCosts or not anyIKProblems):
            #set up optProblem, an instance of optimize.Problem
            assert self.optimizationVariables[0] is self.q
            if len(activeDofs) < self.robot.numLinks():
                #freeze those inactive DOFs
                q = self.robot.getConfig()
                backupJointLimits = qmin[:],qmax[:]
                inactiveDofs = set(range(len(q))) - set(activeDofs)
                for i in inactiveDofs:
                    qmin[i] = q[i]
                    qmax[i] = q[i]
                self.setBounds("q",qmin,qmax)

                reducedProblem,reducedToFullMapping,fullToReducedMapping = self.preprocess()

                optq = reducedProblem.context.variableDict['q']

                print("Preprocessed problem:")
                reducedProblem.pprint()

                optProblem = reducedProblem.getProblem()

                assert backupJointLimits is not None
                self.setBounds("q",*backupJointLimits)
            else:
                optq = self.q
                optProblem = self.getProblem()
                reducedToFullMapping = fullToReducedMapping = None
            #optProblem is now ready to use

        if params.globalMethod is not None:
            #set seed = robot configuration
            if self.q.value is None:
                self.q.bind(robot.getConfig())
            if reducedToFullMapping is None:
                x0 = self.getVarVector()
            else:
                for var,vexpr in zip(reducedProblem.optimizationVariables,fullToReducedMapping):
                    var.bind(vexpr.eval(self.context))
                x0 = reducedProblem.getVarVector()

            #do global optimization of the cost function and return
            (succ,res) = params.solve(optProblem,x0)
            if not succ:
                print("Global optimize returned failure")
                return None
            if reducedToFullMapping is not None:
                reducedProblem.setVarVector(res)
                for var,vexpr in zip(self.optimizationVariables,reducedToFullMapping):
                    var.bind(vexpr.eval(reducedProblem.context))
            else:
                self.setVarVector(res)
            #check feasibility if desired
            if not self.inJointLimits(self.q.value):
                print("Result from global optimize is out of joint limits")
                return None
            if not self.feasibilityTestsPass():
                print("Result from global optimize isn't feasible")
                return None
            if not self.satisfiesEqualities(params.tol):
                print("Result from global optimize doesn't satisfy tolerance.")
                return None
            #passed
            print("Global optimize succeeded! Cost",self.cost())
            q = self.q.value
            return q

        if anyIKProblems:
            print("Performing random-restart newton raphson")
            #random-restart newton-raphson
            solver.setMaxIters(params.numIters)
            solver.setTolerance(params.tol)
            best = None
            bestQuality = float('inf')
            if softIK:
                #quality is a tuple
                bestQuality = bestQuality,bestQuality
            quality = None
            for restart in range(params.numRestarts):
                if time.time() - t0 > params.timeout:
                    return best
                t0 = time.time()
                res = solver.solve()
                if res or self.softObjectives:
                    q = robot.getConfig()
                    print("Got a solve, checking feasibility...")
                    #check feasibility if desired
                    t0 = time.time()
                    self.q.bind(q)
                    if not self.feasibilityTestsPass():
                        print("Failed feasibility")
                        #TODO: resample other non-robot optimization variables
                        if len(nonIKDofs) > 0:
                            u = float(restart+0.5)/params.numRestarts
                            q = robot.getConfig()
                            #perturbation sampling for non-IK dofs
                            for i in nonIKDofs:
                                delta = u*(qmax[i]-qmin[i])*0.5
                                q[i] = random.uniform(max(q[i]-delta,qmin[i]),min(q[i]+delta,qmax[i]))
                            robot.setConfig(q)
                            self.q.bind(q)
                            if not self.feasibilityTestsPass():
                                solver.sampleInitial()
                                continue
                        else:
                            solver.sampleInitial()
                            continue
                    print("Found a feasible config")
                    if softIK:
                        residual = solver.getResidual()
                        ikerr = max(abs(v) for v in residual)
                        if ikerr < params.tol:
                            ikerr = 0
                        else:
                            #minimize squared error
                            ikerr = vectorops.normSquared(residual)
                        if not anyCosts:
                            cost = 0
                            if ikerr == 0:
                                #feasible and no cost
                                return q
                        else:
                            cost = self.cost()
                        quality = ikerr,cost
                    else:
                        if not anyCosts:
                            #feasible, no costs, so we're done
                            print("Feasible and no costs, we're done")
                            return q
                        else:
                            #optimize
                            quality = self.cost(q)
                            print("Quality of solution",quality)
                    if quality < bestQuality:
                        best = self.getVarValues()
                        bestQuality = quality
                #sample a new ik seed
                solver.sampleInitial()

            if best is None or params.localMethod is None:
                return best[0]
            print("Performing post-optimization")
            #post-optimize using local optimizer
            self.setVarValues(best)
            if softIK:
                if not self.satisfiesEqualities(params.tol):
                    raise NotImplementedError("TODO: add soft IK inequality constraint |ik residual| <= |current ik residual|")
            optSolver = optimize.LocalOptimizer(method=params.localMethod)

            if reducedToFullMapping is not None:
                for var,vexpr in zip(reducedProblem.optimizationVariables,fullToReducedMapping):
                    var.bind(vexpr.eval(self.context))
                x0 = reducedProblem.getVarVector()
            else:
                x0 = self.getVarVector()
            optSolver.setSeed(x0)
            res = optSolver.solve(optProblem,params.numIters,params.tol)
            if res[0]:
                if reducedToFullMapping is not None:
                    reducedProblem.setVarVector(res[1])
                    for var,vexpr in zip(self.optimizationVariables,reducedToFullMapping):
                        var.bind(vexpr.eval(reducedProblem.context))
                else:
                    self.setVarVector(res[1])
                
                #check feasibility if desired
                if not self.feasibilityTestsPass():
                    pass
                elif not anyCosts:
                    #feasible
                    best = self.getVarValues()
                else:
                    #optimize
                    quality = self.cost()
                    if quality < bestQuality:
                        #print "Optimization improvement",bestQuality,"->",quality
                        best = self.getVarValues()
                        bestQuality = quality
                    elif quality > bestQuality + 1e-2:
                        print("Got worse solution by local optimizing?",bestQuality,"->",quality)
                self.getVarValues(best)
            print("Resulting quality",bestQuality)
            return best[0]
        else:
            #no IK problems, no global method set -- for now, just perform random restarts
            #
            #set seed = robot configuration
            if self.q.value is None:
                self.q.bind(robot.getConfig())
            if reducedToFullMapping is None:
                x0 = self.getVarVector()
            else:
                for var,vexpr in zip(reducedProblem.optimizationVariables,fullToReducedMapping):
                    var.bind(vexpr.eval(self.context))
                x0 = reducedProblem.getVarVector()

            #do global optimization of the cost function and return
            print("Current optimization variable vector is",x0)
            (succ,res) = params.solve(optProblem,x0)
            if not succ:
                print("Global optimize returned failure")
                return None
            if reducedToFullMapping is not None:
                reducedProblem.setVarVector(res)
                for var,vexpr in zip(self.optimizationVariables,reducedToFullMapping):
                    var.bind(vexpr.eval(reducedProblem.context))
            else:
                self.setVarVector(res)
            #check feasibility if desired
            if not self.inJointLimits(self.q.value):
                print("Result from global optimize is out of joint limits")
                return None
            if not self.feasibilityTestsPass():
                print("Result from global optimize isn't feasible")
                return None
            if not self.satisfiesEqualities(params.tol):
                print("Result from global optimize doesn't satisfy tolerance: result %s"%(str(self.equalityResidual()),))
                for obj in self.objectives:
                    if obj.type == 'eq':
                        print("  ",obj.expr,":",obj.expr.eval(self.context))
                return None
            #passed
            print("Global optimize succeeded! Cost",self.cost())
            q = self.q.value
            return q
