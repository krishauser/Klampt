"""Contains utilities for the Klampt Robot Interface Layer.  These help
implement:

- Generalization of simple interfaces to handle advanced functions, e.g.,
  motion queues and Cartesian controllers (:class:`RobotInterfaceCompleter` ).
  This is useful when you have a position controlled robot (e.g., an Arduino-
  controlled set of motors) and want to do more sophisticated work with it.
- Robots composed of separate independent parts :class:`MultiRobotInterface`,
  such as a robot arm + gripper, each working on separate interfaces.
- Logging... TODO
- Interface with the Klampt :class:`ControllerBase` controller API.

"""

from .robotinterface import *
from ..math import vectorops,spline
from ..plan import motionplanning
from ..model.trajectory import HermiteTrajectory
from .cartesian_drive import CartesianDriveSolver
import bisect
import math

class RobotInterfaceCompleter(RobotInterfaceBase):
    """Completes as much of the RobotInterfaceBase API as possible from a
    partially specified RobotInterfaceBase.  For example, if your
    RobotInterfaceBase implements position control, then the Completer
    will provide velocity, piecewise linear, and piecewise cubic, and Cartesian
    control commands.  

    Note: Completing the base's part interfaces is not implemented yet.

    At a minimum, a base must implement :meth:`sensedPosition` and one of the
    following:

        - :meth:`setPosition`
        - :meth:`moveToPosition`
        - :meth:`setVelocity`
        - :meth:`setPID`
        - :meth:`setTorque`

    This class will *not* emulate:

        - :meth:`status`
        - :meth:`sensedTorque`
        - :meth:`klamptModel`

    This class will also not emulate the following, although the basic
    RobotInterfaceBase class has default implementations that work OK:

        - :meth:`reset`
        - :meth:`estop`
        - :meth:`softStop`

    It will emulate:

        - Clock time from control rate using integration
        - Control rate from clock time using differences
        - Joint velocities from joint positions using finite differences
        - Joint positions from joint velocities using integration
        - PID control from torque commands
        - Torque commands from PID control
        - Piecewise linear / piecewise cubic control from position / velocity
          / PID control.
        - Cartesian positions / velocities / forces from joint positions /
          velocities / torques using the Klamp't model.
        - Cartesian control using the Klamp't model and IK

    Args:
        baseInterface (RobotInterfaceBase): the partially-implemented
            interface.
    """
    def __init__(self,baseInterface):
        RobotInterfaceBase.__init__(self)
        self._base = baseInterface
        self._parent = None
        self._subRobot = False
        self._parts = None
        self._baseParts = None
        self._has = dict()
        self._indices = None
        self._emulator = None
        self._emulatorControlMode = None
        self._baseControlMode = None

    def __str__(self):
        return "Completer("+str(self._base)+')'

    def numDOFs(self,part=None):
        if self._parts is not None and part in self._parts:
            return len(self._parts[part])
        return self._base.numDOFs(part)

    def parts(self):
        if self._parts is None: #pre-initialization
            return self._base.parts()
        return self._parts

    def addPart(self,name,indices):
        """Can add new parts, e.g., for Cartesian control"""
        if self._parts is None:
            raise RuntimeError("Need to call initialize before addPart... (this may change in future versions)")
        assert name not in self._parts
        self._parts[name] = indices

    def _try(self,fn,args,fallback=None):
        """Tries calling a function implemented in the base class, with potential
        alternatives.

        Returns the result of calling getattr(_base,fn)(*args), or if that raises a
        NotImplementedError, returns fallback(*args).

        Also, fn and args can be lists. In this case, fallback is ignored, and
        instead each function and each argument is tried in turn.  The i'th arg
        can be a callable (e.g., a lambda function), which produces the argument
        tuple for the i'th function.  The return value is a pair (res,fn) where
        res is the result of a successful call, and fn is the function name.

        Results of prior calls are cached so that we only explore the base
        interfaces' capabilities once.
        """
        if isinstance(fn,list):
            #go through functions
            assert len(fn) == len(args)
            for i,(f,a) in enumerate(zip(fn,args)):
                hasf = False
                try:
                    hasf = self._has[f]
                except KeyError:
                    #test whether it exists
                    if callable(a):
                        a = a()
                        args[i] = a
                    assert isinstance(args[i],(list,tuple)),"Invalid type of argument list, must be tuple: "+str(args[i])
                    try:
                        res = getattr(self._base,f)(*args[i])
                        self._has[f] = True
                        return res,f
                    except NotImplementedError:
                        self._has[f] = False
                    continue
                if hasf:
                    if callable(a):
                        a = a()
                        args[i] = a
                    assert isinstance(args[i],(list,tuple)),"Invalid type of argument list, must be tuple: "+str(args[i])
                    return getattr(self._base,f)(*args[i]),f
                else:
                    #fall through to next iterations
                    pass
            raise NotImplementedError("Functions "+','.join(fn)+" are not implemented by base interface")
        try:
            #print("Trying to get native",fn)
            if self._has[fn]:
                #print("...Cached")
                return getattr(self._base,fn)(*args)
            else:
                #print("... dont got it, using fallback result",fallback(*args))
                return fallback(*args)
        except KeyError:
            try:
                #print("...first time...")
                res = getattr(self._base,fn)(*args)
                #print("...available")
                self._has[fn] = True
                return res
            except NotImplementedError:
                self._has[fn] = False
                if fallback is None:
                    raise NotImplementedError("Function "+fn+" is not implemented by base interface, no fallback available")
                #print("...not available, returning",fallback(*args))
                return fallback(*args)

    def initialize(self):
        assert self._indices is None,"Can only call initialize() once.."
        if not self._base.initialize():
            return False
        #discover capabilities
        self._baseParts = self._base.parts()
        self._parts = self._baseParts.copy()
        self._indices = self.indices()
        self._try('controlRate',[],lambda :0)
        curclock = self._try('clock',[],lambda :0)
        if not self._has['controlRate'] and not self._has['clock']:
            print ("RobotInterfaceCompleter(%s): Need at least one of controlRate() and clock() to be implemented"%(str(self._base),))
            return False
        self._try('sensedPosition',[],lambda *args:0)
        self._try('sensedVelocity',[],lambda *args:0)
        self._try('sensedTorque',[],lambda *args:0)
        self._try('commandedPosition',[],lambda *args:0)
        self._try('commandedVelocity',[],lambda *args:0)
        self._try('commandedTorque',[],lambda *args:0)
        self._try('destinationPosition',[],lambda *args:0)
        self._try('destinationVelocity',[],lambda *args:0)
        self._try('destinationTime',[],lambda *args:0)
        self._try('queuedTrajectory',[],lambda *args:0)
        if not self._has['sensedPosition'] and not self._has['commandedPosition']:
            print ("RobotInterfaceCompleter(%s): Need at least one of sensedPosition() and commandedPosition() to be implemented"%(str(self._base),))
            return False
        self._emulator = _RobotInterfaceEmulatorData(self._base.numDOFs(),self._base.klamptModel())
        self._emulator.curClock = curclock
        assert curclock is not None
        self._emulator.lastClock = None
        return True

    def startStep(self):
        assert not self._subRobot,"Can't do startStep on a sub-interface"
        self._try('startStep',[],lambda *args:0)
        if self._emulator.lastClock is None:
            qcmd = self._try('commandedPosition',[],lambda *args:None)
            vcmd = self._try('commandedVelocity',[],lambda *args:None)
            tcmd = self._try('commandedTorque',[],lambda *args:None)
            self._emulator.updateCommand(qcmd,vcmd,tcmd)
        if not self._has['controlRate']:
            self._emulator.pendingClock = self.clock()
            if self._emulator.lastClock != self._emulator.pendingClock:
                self._emulator.dt = self._emulator.pendingClock - self._emulator.lastClock
        elif not self._has['clock']:
            self._emulator.dt = self.controlRate()
            self._emulator.pendingClock = self._emulator.curClock + 1.0/self._emulator.dt
        else:
            self._emulator.dt = self.controlRate()
            self._emulator.pendingClock = self.clock()

    _FUNC_TO_CONTROL_MODE = {'setPosition':'p','moveToPosition':'m','setVelocity':'v','setTorque':'t','setPID':'pid','setPiecewiseLinear':'pwl','setPiecewiseCubic':'pwc'}

    def endStep(self):
        assert not self._subRobot,"Can't do endStep on a sub-interface"
        if not self._has['controlRate']:
            self._emulator.lastClock = self._emulator.curClock
            
        q = self.sensedPosition()
        v = self.sensedVelocity()
        try:
            self._emulator.update(self._emulator.pendingClock,q,v)
        except Exception as e:
            self.softStop()
            raise e

        desiredControlMode = self._emulator.desiredControlMode()
        self._emulatorControlMode = desiredControlMode
        if desiredControlMode is None:
            #no control needed
            pass
        elif desiredControlMode == 'pwc':
            res=self._try(['setPiecewiseCubic','setPiecewiseLinear','setPID','setPosition','setVelocity','moveToPosition','setTorque'],[self._emulator.getCommand('pwc'),
                      lambda :self._emulator.getCommand('pwl'),
                      lambda :self._emulator.getCommand('pid'),
                      lambda :self._emulator.getCommand('p'),
                      lambda :self._emulator.getCommand('v'),
                      lambda :self._emulator.getCommand('m'),
                      lambda :self._emulator.getCommand('t')])
            self._baseControlMode = self._FUNC_TO_CONTROL_MODE[res[1]]
            if self._baseControlMode in ['pwc','pwl']:
                self._emulator.commandSent = True
        elif desiredControlMode == 'pwl':
            res=self._try(['setPiecewiseLinear','setPID','setPosition','setVelocity','moveToPosition','setTorque'],[self._emulator.getCommand('pwl'),
                      lambda :self._emulator.getCommand('pid'),
                      lambda :self._emulator.getCommand('p'),
                      lambda :self._emulator.getCommand('v'),
                      lambda :self._emulator.getCommand('m'),
                      lambda :self._emulator.getCommand('t')])
            self._baseControlMode = self._FUNC_TO_CONTROL_MODE[res[1]]
            if self._baseControlMode == 'pwl':
                self._emulator.commandSent = True
        elif desiredControlMode == 'pid':
            res=self._try(['setPID','setTorque','setPosition','setVelocity','moveToPosition'],[self.emulator.getCommand('pid'),
                      lambda :self._emulator.getCommand('t'),
                      lambda :self._emulator.getCommand('p'),
                      lambda :self._emulator.getCommand('v'),
                      lambda :self._emulator.getCommand('m')])
            self._baseControlMode = self._FUNC_TO_CONTROL_MODE[res[1]]
            if self._baseControlMode in ['pid','p','m']:
                self._emulator.commandSent = True
        elif desiredControlMode == 'p':
            res=self._try(['setPosition','setVelocity','setPID','moveToPosition','setPiecewiseLinear','setTorque'],[self._emulator.getCommand('p'),
                      lambda :self._emulator.getCommand('v'),
                      lambda :self._emulator.getCommand('pid'),
                      lambda :self._emulator.getCommand('m'),
                      lambda :self._emulator.getCommand('pwl'),
                      lambda :self._emulator.getCommand('t')])
            self._baseControlMode = self._FUNC_TO_CONTROL_MODE[res[1]]
            if self._baseControlMode in ['p','m','pwl']:
                self._emulator.commandSent = True
        elif desiredControlMode == 'v':
            res=self._try(['setVelocity','setPosition','setPID','moveToPosition','setPiecewiseLinear','setTorque'],[self._emulator.getCommand('v'),
                      lambda :self._emulator.getCommand('p'),
                      lambda :self._emulator.getCommand('pid'),
                      lambda :self._emulator.getCommand('m'),
                      lambda :self._emulator.getCommand('pwl'),
                      lambda :self._emulator.getCommand('t')])
            self._baseControlMode = self._FUNC_TO_CONTROL_MODE[res[1]]
            if self._baseControlMode == ['v','pwl']:
                self._emulator.commandSent = True
        elif desiredControlMode == 't':
            res=self._try(['setTorque','setPID'],[self._emulator.getCommand('t'),
                      lambda :self._emulator.getCommand('pid')])
            self._baseControlMode = self._FUNC_TO_CONTROL_MODE[res[1]]
            if self._baseControlMode in ['t','pid']:
                self._emulator.commandSent = True
        else:
            raise RuntimeError("Invalid emulator control type? "+self._emulator.controlType)
        self._try('endStep',[],lambda *args:0)

    def controlRate(self):
        def _controlRate_backup(self):
            if self._emulator.dt is None:
                raise RuntimeError("Base interface doesn't have controlRate, and need to have at least one time step before determining backup controlRate")
            return 1.0/self._emulator.dt
        return self._try('controlRate',[],lambda :_controlRate_backup(self))

    def clock(self):
        return self._try('clock',[],lambda: self._emulator.curClock)

    def estop(self):
        self._base.estop()

    def softStop(self):
        self._base.softStop()

    def reset(self):
        return self._base.reset()

    def getPartInterface(self,part=None,joint_idx=None):
        return _SubRobotInterfaceCompleter(self,part,joint_idx)

    def jointName(self,joint_idx):
        return self._try('jointName',[self.joint_idx],lambda joint_idx: 'Joint '+str(joint_idx))

    def sensors(self):
        return self._try('sensors',[],lambda *args:[])

    def enabledSensors(self):
        return self._try('enabledSensors',[],lambda *args:[])

    def hasSensor(self,sensor):
        return self._try('hasSensor',[],lambda *args:False)

    def enableSensor(self,sensor):
        return self._try('enableSensor',[sensor],lambda *args:False)

    def setPosition(self,q):
        assert len(q) == len(self._indices)
        self._emulator.setPosition(self._indices,q)

    def setVelocity(self,v,ttl=None):
        assert len(v) == len(self._indices)
        self._emulator.setVelocity(self._indices,v,ttl)

    def setTorque(self,t,ttl=None):
        assert len(t) == len(self._indices)
        self._emulator.setTorque(self._indices,t,ttl)
        
    def setPID(self,q,dq,t=None):
        assert len(q) == len(self._indices)
        assert len(dq) == len(self._indices)
        self._emulator.setPID(self._indices,q,dq,t)

    def setPIDGains(self,kP,kI,kD):
        try:
            self._base.setPIDGains(kP,kI,kD)
        except NotImplementedError:
            pass
        if not hasattr(kP,'__iter__'):
            kP = [kP]*len(self._indices)
        if not hasattr(kD,'__iter__'):
            kD = [kD]*len(self._indices)
        if not hasattr(kI,'__iter__'):
            kI = [kI]*len(self._indices)
        self._emulator.setPIDGains(self._indices,kP,kI,kD)

    def moveToPosition(self,q,speed=1):
        assert len(q) == len(self._indices)
        self._emulator.moveToPosition(self._indices,q,speed)

    def setPiecewiseLinear(self,ts,qs,relative=True):
        self._emulator.setPiecewiseLinear(self._indices,ts,qs,relative)

    def setPiecewiseCubic(self,ts,qs,vs,relative=True):
        self._emulator.setPiecewiseCubic(self._indices,ts,qs,vs,relative)

    def setToolCoordinates(self,xtool_local):
        self._emulator.setToolCoordinates(xtool_local,self._indices)

    def getToolCoordinates(self):
        self._emulator.getToolCoordinates(self._indices)

    def setGravityCompensation(self,gravity=[0,0,-9.8],load=0.0,load_com=[0,0,0]):
        try:
            self._base.setGravityCompensation(gravity,load,load_com)
        except NotImplementedError:
            pass
        self._emulator.setGravityCompensation(gravity,load,load_com,self._indices)

    def setCartesianPosition(self,xparams):
        self._emulator.setCartesianPosition(xparams,self._indices)

    def moveToCartesianPosition(self,xparams,speed=1.0):
        self._emulator.moveToCartesianPosition(xparams,speed,self._indices)

    def setCartesianVelocity(self,dxparams,ttl=None):
        self._emulator.setCartesianVelocity(dxparams,ttl,self._indices)

    def setCartesianForce(self,fparams,ttl=None):
        self._emulator.setCartesianForce(fparams,ttl,self._indices)

    def status(self):
        return self._base.status()

    def isMoving(self):
        if self._emulator.isMoving(self._indices):
            return True
        try:
            return self._base.isMoving()
        except NotImplementedError:
            return False

    def sensedPosition(self):
        #need to have this implemented on base
        return self._base.sensedPosition()

    def sensedVelocity(self):
        return self._try('sensedVelocity',[],self._emulator.sensedVelocity)

    def sensedTorque(self):
        #need to have this implemented on base
        return self._base.sensedTorque()

    def commandedPosition(self):
        return self._try('commandedPosition',[],self._emulator.commandedPosition)

    def commandedVelocity(self):
        return self._try('commandedVelocity',[],self._emulator.commandedVelocity)

    def commandedTorque(self):
        return self._try('commandedTorque',[],self._emulator.commandedTorque)

    def destinationPosition(self):
        if self._baseControlMode == self._emulatorControlMode:
            #using base interface
            return self._try('destinationPosition',[],lambda : self._emulator.destinationPosition())
        else:
            return self._emulator.destinationPosition()

    def destinationVelocity(self):
        if self._baseControlMode == self._emulatorControlMode:
            #using base interface
            return self._try('destinationVelocity',[],lambda : self._emulator.destinationVelocity())
        else:
            return self._emulator.destinationVelocity()

    def destinationTime(self):
        if self._baseControlMode == self._emulatorControlMode:
            #using base interface
            return self._try('destinationTime',[],lambda : self._emulator.destinationTime())
        else:
            return self._emulator.destinationTime()

    def cartesianPosition(self,q):
        if self._baseControlMode == self._emulatorControlMode:
            #using base interface
            return self._try('cartesianPosition',[q],lambda q: self._emulator.cartesianPosition(q,self._indices))
        else:
            return self._emulator.cartesianPosition(q,self._indices)

    def cartesianVelocity(self,q,dq):
        if self._baseControlMode == self._emulatorControlMode:
            #using base interface
            return self._try('cartesianVelocity',[q,dq],lambda q,dq: self._emulator.cartesianPosition(q,dq,self._indices))
        else:
            return self._emulator.cartesianVelocity(q,dq,self._indices)

    def cartesianForce(self,q,t):
        if self._baseControlMode == self._emulatorControlMode:
            #using base interface
            return self._try('cartesianForce',[q,t],lambda q,t: self._emulator.cartesianForce(q,t,self._indices))
        else:
            return self._emulator.cartesianForce(q,t,self._indices)

    def sensedCartesianPosition(self):
        return self._try('sensedCartesianPosition',[],lambda : RobotInterfaceBase.sensedCartesianPosition(self))

    def sensedCartesianVelocity(self):
        return self._try('sensedCartesianVelocity',[],lambda : RobotInterfaceBase.sensedCartesianVelocity(self))

    def sensedCartesianForce(self):
        return self._try('sensedCartesianForce',[],lambda : RobotInterfaceBase.sensedCartesianForce(self))

    def commandedCartesianPosition(self):
        return self._try('commandedCartesianPosition',[],lambda : RobotInterfaceBase.commandedCartesianPosition(self))

    def commandedCartesianVelocity(self):
        return self._try('commandedCartesianVelocity',[],lambda : RobotInterfaceBase.commandedCartesianVelocity(self))

    def commandedCartesianForce(self):
        return self._try('destinationCartesianForce',[],lambda : RobotInterfaceBase.destinationCartesianForce(self))

    def destinationCartesianPosition(self):
        return self._try('destinationCartesianPosition',[],lambda : RobotInterfaceBase.destinationCartesianPosition(self))

    def destinationCartesianVelocity(self):
        return self._try('destinationCartesianVelocity',[],lambda : RobotInterfaceBase.destinationCartesianVelocity(self))

    def partToRobotConfig(self,pconfig,part,robotConfig):
        return self._base.partToRobotConfig(pconfig,part,robotConfig)

    def klamptModel(self):
        return self._base.klamptModel()

    def fromKlamptConfig(self,klamptConfig,part=None,joint_idx=None):
        return self._base.fromKlamptConfig(klamptConfig,part=None,joint_idx=None)

    def fromKlamptVelocity(self,klamptVelocity,part=None,joint_idx=None):
        return self._base.fromKlamptVelocity(klamptConfig,part=None,joint_idx=None)

    def toKlamptConfig(self,config,klamptConfig=None,part=None,joint_idx=None):
        return self._base.toKlamptConfig(config,klamptConfig,part=None,joint_idx=None)

    def toKlamptVelocity(self,config,klamptVelocity,part=None,joint_idx=None):
        return self._base.toKlamptVelocity(velocity,klamptVelocity,part=None,joint_idx=None)

    def print_status(self):
        print("****** Status of RobotInterfaceCompleter *********")
        print("Base interface",str(self._base))
        if self._indices is None:
            print("Not initialized")
            print("**************************************************")
            return
        print("Status",self.status())
        if len(self._parts) != len(self._baseParts):
            newparts = [k for k in self._parts.keys() if k not in self._baseParts]
            print("Added parts:",','.join(newparts))
        
        if self._baseControlMode == self._emulatorControlMode:
            print("Using base control mode",self._baseControlMode)
        else:
            print("Using emulator control mode",self._emulatorControlMode,"and base mode",self._baseControlMode)
        self._emulator.print_status()

        print("**************************************************")



class MultiRobotInterface(RobotInterfaceBase):
    """A RobotInterfaceBase that consists of multiple parts that are
    addressed separately.  For example, a mobile manipulator can consist of
    a base, arms, and grippers.

    On startup, call ``addPart(name,interface,klamptModel,klamptIndices)`` for
    each part of the robot.

    .. note::
        TIP: wrap your interface with a :class:`RobotInterfaceCompleter`
        before adding it so that all parts get a uniform interface.

    The total configuration for the robot is a list of floats, segmented into
    parts.  The ordering of the parts' configurations is the same as how they
    were added.
    """
    def __init__(self):
        self._partNames = []
        self._partInterfaces = dict()
        self._parts = dict()
        self._parts[None] = []
        self._jointToPart = []
        self._klamptModel = None
        self._klamptParts = dict()

    def addPart(self,partName,partInterface,klamptModel=None,klamptIndices=None):
        """Adds a part `partName` with the given interface `partInterface`. 
        The DOFs of the unified robot range from N to N+Np where N is the current
        number of robot DOFs and Np is the number of part DOFs.

        If klamptModel / klamptIndices are given, The part is associated with the given klamptModel, and the indices of the
        part's sub-model are given by klamptIndices.  (The indices of partInterface
        are indices into klamptIndices, not klamptModel )
        """
        self._partNames.append(partName)
        self._partInterfaces[partName] = partInterface

        part_ndof = partInterface.numDOFs()
        partdofs = list(range(len(self._jointToPart),len(self._jointToPart)+part_ndof))
        for i,d in enumerate(partdofs):
            self._jointToPart.append((partName,i))

        for cp,dofs in partInterface.parts().items():
            assert all(d >= 0 and d < len(partdofs) for d in dofs),"Invalid DOF of part %s, must be between 0 and %d"%(str(cp),str(len(partdofs)))
            bigdofs = [partdofs[d] for d in dofs]
            if cp is None:
                self._parts[partName] = bigdofs
            else:
                self._parts[partName+' '+cp] = bigdofs
        if partName not in self._parts:
            self._parts[partName] = partdofs
        self._parts[None] = range(len(self._jointToPart))

        if klamptModel is not None:
            if self._klamptModel is None:
                self._klamptModel = klamptModel
            else:
                assert self._klamptModel is klamptModel, "Can't add parts from multiple Klamp't models, please create a merged robot model"
        else:
            klamptModel = self._klamptModel
        
        if klamptIndices is not None:
            assert klamptModel is not None,"Need to specify a Klamp't model"
            self._klamptParts[partName] = klamptIndices

    def numDOFs(self,part=None):
        return len(self._parts[part])

    def parts(self):
        return self._parts

    def controlRate(self):
        return max(c.controlRate() for (p,c) in self._partInterfaces.items())

    def initialize(self):
        for (p,c) in self._partInterfaces.items():
            if not c.initialize():
                print ("MultiRobotInterface: Part",p,"failed to initialize")
                return False
        return True

    def startStep(self):
        for (p,c) in self._partInterfaces.items():
            c.startStep()

    def endStep(self):
        for (p,c) in self._partInterfaces.items():
            c.endStep()

    def clock(self):
        for (p,c) in self._partInterfaces.items():
            return c.clock()
        raise ValueError("No parts defined, so clock isn't well defined")

    def reset(self):
        for (p,c) in self._partInterfaces.items():
            if c.status() != 'ok':
                if not c.reset():
                    return False
        return True

    def getPartInterface(self,part=None,joint_idx=None):
        if part is None and joint_idx is None:
            return self
        assert part in self._partInterfaces
        if joint_idx is None:
            return self._partInterfaces[part]
        else:
            return self._partInterfaces[part].getPartInterface(None,joint_idx)

    def jointName(self,joint_idx,part=None):
        if part is None:
            part = self._jointToPart[joint_idx]
        return part + ' ' + self._partInterfaces[part].jointName(joint_idx)

    def sensors(self):
        s = []
        for (p,c) in self._partInterfaces.items():
            s += [(p,n) for n in c.sensors()]
        return s

    def enabledSensors(self):
        s = []
        for (p,c) in self._partInterfaces.items():
            s += [(p,n) for n in c.enabledSensors()]
        return s

    def hasSensor(self,sensor):
        return sensor in self.sensors()

    def enableSensor(self,sensor):
        assert isinstance(sensor,(list,tuple))
        p = sensor[0]
        return self._partInterfaces[p].enableSensor(sensor[1])

    def split(self,q):
        """Splits a whole-body robot to parts (one per listed item)."""
        res = []
        for p in self._partNames:
            indices = self._parts[p]
            res.append([q[i] for i in indices])
        return res

    def join(self,qparts):
        """Joins a bunch of parts into a whole-body robot."""
        assert len(qparts) == len(self._partNames)
        res = [0]*len(self._parts[None])
        for q,p in zip(qparts,self._partNames):
            indices = self._parts[p]
            assert len(indices)==len(q)
            for i,j in enumerate(indices):
                res[j] = q[i]
        return res

    def _setSplit(self,q,cmd,*otherArgs):
        qparts = self.split(q)
        for qp,p in zip(qparts,self._partNames):
            getattr(self._partInterfaces[p],cmd)(qp,*otherArgs)

    def _getJoin(self,cmd):
        qparts = [getattr(self._partInterfaces[p],cmd)() for p in self._partNames]
        return self.join(qparts)

    def setPosition(self,q,immediate=False):
        self._setSplit(q,'setPosition',immediate)
        
    def setVelocity(self,v,ttl=None):
        self._setSplit(v,'setVelocity',ttl)

    def setTorque(self,t,ttl=None):
        self._setSplit(t,'setTorque',ttl)

    def setPID(self,q,dq,t=None):
        qs = self.split(q)
        dqs = self.split(dq)
        if t is None:
            ts = [None]*len(self._partNames)
        else:
            ts = self.split(t)
        for (p,q,dq,t) in zip(self._partNames,qs,dqs,ts):
            self._partInterfaces[p].setPID(q,dq,t)

    def setPIDGains(self,kP,kI,kD):
        kPs = self.split(kP)
        kIs = self.split(kI)
        kDs = self.split(kD)
        for (p,kP,kI,kD) in zip(self._partNames,kPs,kIs,kDs):
            self._partInterfaces[p].setPID(kP,kI,kD)

    def setPiecewiseLinear(self):
        raise NotImplementedError()

    def setPiecewiseCubic(self):
        raise NotImplementedError()

    def setCartesianPosition(self,xparams,immediate=False):
        raise ValueError("Can't do cartesian control without specifying a part")

    def setCartesianVelocity(self,dxparams,ttl=None):
        raise ValueError("Can't do cartesian control without specifying a part")

    def setCartesianForce(self,fparams,ttl=None):
        raise ValueError("Can't do cartesian control without specifying a part")

    def status(self):
        for p,c in self._partInterfaces.items():
            s = c.status()
            if s != 'ok':
                return s
        return 'ok'

    def isMoving(self):
        return any(c.isMoving() for p,c in self._partInterfaces.items())

    def sensedPosition(self):
        return self._getJoin('sensedPosition')

    def sensedVelocity(self):
        return self._getJoin('sensedVelocity')

    def sensedTorque(self):
        return self._getJoin('sensedTorque')

    def commandedPosition(self):
        return self._getJoin('commandedPosition')

    def commandedVelocity(self):
        return self._getJoin('commandedVelocity')

    def destinationPosition(self):
        return self._getJoin('destinationPosition')

    def destinationVelocity(self):
        return self._getJoin('destinationVelocity')

    def cartesianPosition(self,q):
        raise ValueError("Can't do cartesian get without specifying a part")

    def cartesianVelocity(self,q,dq):
        raise ValueError("Can't do cartesian get without specifying a part")

    def cartesianForce(self,q,t):
        raise ValueError("Can't do cartesian get without specifying a part")

    def sensedCartesianPosition(self):
        raise ValueError("Can't do cartesian get without specifying a part")

    def sensedCartesianVelocity(self):
        raise ValueError("Can't do cartesian get without specifying a part")

    def sensedCartesianForce(self):
        raise ValueError("Can't do cartesian get without specifying a part")

    def commandedCartesianPosition(self):
        raise ValueError("Can't do cartesian get without specifying a part")

    def commandedCartesianVelocity(self):
        raise ValueError("Can't do cartesian get without specifying a part")

    def commandedCartesianForce(self):
        raise ValueError("Can't do cartesian get without specifying a part")

    def destinationCartesianPosition(self):
        raise ValueError("Can't do cartesian get without specifying a part")

    def destinationCartesianVelocity(self):
        raise ValueError("Can't do cartesian get without specifying a part")


class _JointInterfaceEmulatorData:
    CONTROL_MODES = ['pid','v','p','m','pwl','pwc']
    CONTROL_MODE_PRECEDENCE = {'pid':0,'v':1,'p':2,'pwl':3,'pwc':4}

    def __init__(self):
        self.dt = None
        self.controlMode = None
        self.sensedPosition = None
        self.sensedVelocity = None
        self.commandedPosition = None
        self.commandedVelocity = None
        self.commandedTorque = None
        self.commandTTL = None
        self.lastCommandedPosition = None
        self.commandParametersChanged = False
        self.pidCmd = None
        self.pidGains = None
        self.pidIntegralError = None
        self.trajectoryTimes = None
        self.trajectoryMilestones = None
        self.trajectoryVelocities = None
        self.externalController = None

    def update(self,t,q,v,dt):
        if v is None:
            if self.sensedPosition is None:
                self.sensedVelocity = 0
            else:
                self.sensedVelocity = (q-self.sensedPosition)/dt
            v = self.sensedVelocity
        else:
            self.sensedVelocity = v
        self.sensedPosition = q
        if self.commandTTL is not None:
            self.commandTTL -= dt

        if self.controlMode is None:
            return 
        elif self.controlMode == 'pid':
            self.commandedPosition = self.pidCmd[0]
            self.commandedVelocity = self.pidCmd[1]
            self.commandedTorque = self.pidCmd[2]
            self.pidIntegralError[i] += (self.commandedPosition-q)*dt
            self.commandTTL = dt*5
        elif self.controlMode == 'pwl' or self.controlMode == 'pwc':
            self.commandedPosition,self.commandedVelocity = self.evalTrajectory(t)
            self.commandTTL = dt*5
            self.updateTrajectory(t)
            self.commandedTorque = 0
        elif self.controlMode == 'p' or self.controlMode == 'm':
            if self.lastCommandedPosition is None:
                self.commandedVelocity = 0
            else:
                self.commandedVelocity = (self.commandedPosition-self.lastCommandedPosition)/dt
            self.lastCommandedPosition = self.commandedPosition
        elif self.controlMode == 'v':
            self.commandedPosition += self.commandedVelocity*dt
            if self.commandTTL is not None and self.commandTTL <= 0:
                self.commandedVelocity = 0
                self.commandTTL = None
                self.controlMode = None
        elif self.controlMode == 't':
            if self.commandTTL is not None and self.commandTTL <= 0:
                self.commandedTorque = 0
                self.commandTTL = None
                self.controlMode = None
        else:
            raise RuntimeError("Invalid control mode?")

    def getCommand(self,commandType):
        assert self.controlMode is not None
        if commandType == 'pwc':
            return self.trajectoryTimes,self.trajectoryMilestones,self.trajectoryVelocities
        elif commandType == 'pwl':
            if self.controlMode == 'pwc':
                traj = HermiteTrajectory(self.trajectoryTimes,[[m] for m in self.trajectoryMilestones],[[v] for v in self.trajectoryVelocities])
                assert self.dt is not None
                configTraj = traj.discretize(self.dt)
                return configTraj.times,[m[0] for m in configTraj.milestones]
            if self.controlMode == 'pwl':
                return self.trajectoryTimes,self.trajectoryMilestones
            elif self.controlMode == 'p':
                #construct interpolant... should we do it in 1 time step or stretch it out
                dt = (self.commandedPosition - self.lastCommandedPosition)/self.commandedVelocity
                return [dt],[self.commandedPosition]
            elif self.controlMode == 'v':
                #construct interpolant
                raise NotImplementedError("Shouldn't ever be in v control model")
            else:
                raise NotImplementedError("TODO: convert other control types to linear path")
        if commandType == 'pid':
            return self.commandedPosition,self.commandedVelocity,self.commandedTorque
        elif commandType == 't':
            if self.controlMode == 'pid':
                if self.pidGains is None:
                    raise RuntimeError("Can't emulate PID control using torque control, no gains are set")
                qdes,vdes,tdes = self.pidCmd
                kp,ki,kd = self.pidGains
                t_pid = kp*(qdes-self.sensedPosition) + kd*(vdes-self.sensedVelocity) + ki*self.pidIntegralError + tdes
                #if abs(self.pidIntegralError[i]*ki) > tmax:
                #cap integral error to prevent wind-up
                return self.commandedTorque,self.commandTTL
            else:
                assert self.controlMode == 't',"Can't emulate torque control with any command type except for PID and torque control"
                return self.commandedTorque,self.commandTTL
        elif commandType == 'p' or commandType == 'm':
            return self.commandedPosition,
        elif commandType == 'v':
            return self.commandedVelocity,self.commandTTL

    def promote(self,controlType):
        if self.controlMode == controlType:
            return
        if controlType == 'pid':
            self.pidIntegralError = 0.0
            self.pidCmd = (self.commandedPosition,(0 if self.commandedVelocity is None else self.commandedVelocity),0.0)
        elif controlType == 'pwc':
            if controlType == 'pwl':
                self.trajectoryVelocities = [0]*len(self.trajectoryTimes)
            elif self.controlMode in ['p','v','t']:
                if self.commandedPosition is None:
                    q = self.sensedPosition
                else:
                    q = self.commandedPosition
                if self.commandedVelocity is None:
                    if self.sensedVelocity is None:
                        v = 0
                    else:
                        v = self.sensedVelocity
                else:
                    v = self.commandedVelocity
                self.trajectoryTimes = [0]
                self.trajectoryMilestones = [q]
                self.trajectoryVelocities = [v]
            elif self.controlMode == 'pid':
                self.trajectoryTimes = [0]
                self.trajectoryMilestones = [self.pidCmd[0]]
                self.trajectoryVelocities = [0]
        elif controlType == 'pwl':
            if self.controlMode in ['p','v','t']:
                if self.commandedPosition is None:
                    q = self.sensedPosition
                else:
                    q = self.commandedPosition
                self.trajectoryTimes = [0]
                self.trajectoryMilestones = [q]
            elif self.controlMode == 'pid':
                self.trajectoryTimes = [0]
                self.trajectoryMilestones = [self.pidCmd[0]]
            self.trajectoryVelocities = None
        self.controlMode = controlType

    def destinationTime(self,t):
        if self.controlMode not in ['pwl','pwc']:
            return t
        return self.trajectoryTimes[-1]

    def destinationPosition(self):
        if self.controlMode not in ['pwl','pwc']:
            return self.commandedPosition
        return self.trajectoryMilestones[-1]

    def destinationVelocity(self):
        if self.controlMode not in ['pwl','pwc']:
            return self.commandedVelocity
        if self.controlMode == 'pwl':
            return 0
        return self.trajectoryVelocities[-1]

    def _getSegment(self,t):
        """Returns the index and interpolation parameter for the
        segment at time t. 

        Running time is O(log n) time where n is the number of segments.
        """
        if len(self.trajectoryTimes)==0:
            raise ValueError("Empty trajectory")
        if len(self.trajectoryTimes)==1:
            return (-1,0)
        if t >= self.trajectoryTimes[-1]:
            return (len(self.trajectoryTimes)-1,0)
        if t <= self.trajectoryTimes[0]:
            return (0,0)
        i = bisect.bisect_right(self.trajectoryTimes,t)
        p=i-1
        assert i > 0 and i < len(self.trajectoryTimes),"Invalid time index "+str(t)+" in "+str(self.trajectoryTimes)
        u=(t-self.trajectoryTimes[p])/(self.trajectoryTimes[i]-self.trajectoryTimes[p])
        if i==0:
            return (-1,0)
        assert u >= 0 and u <= 1
        return (p,u)

    def evalTrajectory(self,t):
        """Returns (position,velocity) tuple"""
        assert len(self.trajectoryTimes) == len(self.trajectoryMilestones)
        i,u = self._getSegment(t)
        if i<0:
            return self.trajectoryMilestones[0],0
        if i+1 >= len(self.trajectoryTimes):
            return self.trajectoryMilestones[-1],0
        
        dt = self.trajectoryTimes[i+1]-self.trajectoryTimes[i]
        if self.trajectoryVelocities is None:
            #piecewise linear
            dp = self.trajectoryMilestones[i+1]-self.trajectoryMilestones[i]
            pos = self.trajectoryMilestones[i] + u*dp
            if dt == 0:
                #discontinuity?
                vel = 0
            else:
                vel = dp/dt
            return pos,vel
        else:
            #piecewise cubic
            assert len(self.trajectoryTimes) == len(self.trajectoryVelocities)
            x1,v1 = [self.trajectoryMilestones[i]],[self.trajectoryVelocities[i]*dt]
            x2,v2 = [self.trajectoryMilestones[i+1]],[self.trajectoryVelocities[i+1]*dt]
            x = spline.hermite_eval(x1,v1,x2,v2,u)
            dx = vectorops.mul(spline.hermite_deriv(x1,v1,x2,v2,u),1.0/dt)
            return x[0],dx[0]

    def updateTrajectory(self,t):
        i,u = self._getSegment(t)
        if i < 0:
            return
        if i+1 >= len(self.trajectoryTimes):
            if len(self.trajectoryTimes) > 0:
                self.trajectoryTimes = self.trajectoryTimes[-1:]
                self.trajectoryMilestones = self.trajectoryMilestones[-1:]
                if self.trajectoryVelocities is not None:
                    self.trajectoryVelocities = self.trajectoryVelocities[-1:]
                #print("STOP",self.trajectoryTimes,self.trajectoryMilestones,self.trajectoryVelocities)
            return
        #math.log2 is available only in Python 3... convert to math.log(x,2) in Python 2
        if i > math.log2(len(self.trajectoryTimes)):
            self.trajectoryTimes = self.trajectoryTimes[i:]
            self.trajectoryMilestones = self.trajectoryMilestones[i:]
            if self.trajectoryVelocities is not None:
                self.trajectoryVelocities = self.trajectoryVelocities[i:]
            #print("PWL now have",len(self.trajectoryTimes),"milestnoes ending in",self.trajectoryMilestones[-1])


class _CartesianEmulatorData:
    def __init__(self,robot,indices):
        self.robot = robot
        self.indices = indices
        self.driver = CartesianDriveSolver(robot)
        assert indices[-1] == max(indices),"Indices must be in sorted order"
        self.eeLink = robot.link(indices[-1])
        self.toolCoordinates = [0,0,0]
        self.orientationConstrained = True
        self.t = None
        self.active = False
        self.driveCommand = None,None
        self.endDriveTime = None

    def setToolCoordinates(self,xtool_local):
        if self.active:
            raise RuntimeError("Can't set tool coordinates while a cartesian velocity command is active")
        self.toolCoordinates = xtool_local

    def getToolCoordinates(self):
        return self.toolCoordinates

    def solveIK(self,xparams):
        qorig = self.robot.getConfig()
        if not self.active:
            self.driver.start(qorig,self.indices[-1],endEffectorPositions=self.toolCoordinates)
        goal = self.driver.ikGoals[0]
        if len(xparams) == 3:
            #point-to-point constraint
            goal.setFixedPosConstraint(self.toolCoordinates,xparams)
            goal.setFreeRotConstraint()
            self.orientationConstrained = False
        elif len(xparams) == 2:
            if len(xparams[0]) != 9 or len(xparams[1]) != 3:
                raise ValueError("Invalid IK parameters, must be a point or se3 element")
            goal.setFixedPosConstraint(self.toolCoordinates,xparams[1])
            goal.setFixedRotConstraint(xparams[0]);
            self.orientationConstrained = True
        else:
            raise ValueError("Invalid IK parameters, must be a point or se3 element")
        self.driver.solver.set(0,goal)
        err = self.driver.solver.getResidual()
        res = self.driver.solver.solve()
        if res:
            return self.robot.getConfig()
        err2 = self.driver.solver.getResidual()
        if vectorops.normSquared(err2) < vectorops.normSquared(err):
            return self.robot.getConfig()
        else:
            return qorig

    def setCartesianVelocity(self,qcur,dxparams,ttl):
        assert len(qcur) == self.robot.numDrivers()
        for i,v in enumerate(qcur):
            self.robot.driver(i).setValue(v)
        qcur = self.robot.getConfig()

        if not self.active:
            self.driver.start(qcur,self.indices[-1],endEffectorPositions=self.toolCoordinates)
            self.active = True

        if self.t is None:
            self.endDriveTime = ttl
        else:
            self.endDriveTime = self.t + ttl
        if len(dxparams) == 2:
            if len(dxparams[0]) != 3 or len(dxparams[1]) != 3:
                raise ValueError("Invalid IK parameters, must be a 3 vector or a pair of angular velocity / velocity vectors")
            self.driveCommand = dxparams
            self.orientationConstrained = True
        else:
            if len(dxparams) != 3:
                raise ValueError("Invalid IK parameters, must be a 3 vector or a pair of angular velocity / velocity vectors")
            self.driveCommand = None,dxparams
            self.orientationConstrained = False

    def update(self,qcur,t,dt):
        if self.t is None:
            if self.endDriveTime is not None:
                self.endDriveTime = t + self.endDriveTime
        self.t = t
        if not self.active:
            return
        #print("CartesianEmulatorData update",t)
        if t > self.endDriveTime:
            self.active = False
            return
        assert len(qcur) == self.robot.numDrivers()
        for i,v in enumerate(qcur):
            self.robot.driver(i).setValue(v)
        qcur = self.robot.getConfig()
        #print("Drive command",self.driveCommand,"dt",dt)
        (amt,q) = self.driver.drive(qcur,self.driveCommand[0],self.driveCommand[1],dt)
        self.robot.setConfig(q)
        #print("Result",amt,q)
        return [self.robot.driver(i).getValue() for i in self.indices]

    def cartesianPosition(self,q):
        assert len(q) == self.robot.numDrivers()
        model = self.robot
        for i,v in enumerate(q):
            model.driver(i).setValue(v)
        model.setConfig(model.getConfig())
        T = self.eeLink.getTransform()
        t = T.apply(self.toolCoordinates)
        if self.orientationConstrained:
            return (T[0],t)
        else:
            return t
        
    def cartesianVelocity(self,q,dq):
        assert len(q) == self.robot.numDrivers()
        assert len(dq) == self.robot.numDrivers()
        assert len(q) == self.robot.numDrivers()
        model = self.robot
        for i,v in enumerate(q):
            model.driver(i).setValue(v)
            model.driver(i).setVelocity(dq[i])
        model.setConfig(model.getConfig())
        local = self.toolCoordinates
        v = self.eeLink.getPointVelocity(local)
        if self.orientationConstrained:
            w = self.eeLink.getAngularVelocity()
            return w,v
        return v

    def cartesianForce(self,q,t,indices):
        assert len(q) == self.robot.numDrivers()
        model = self.robot
        for i,v in enumerate(q):
            model.driver(i).setValue(v)
        model.setConfig(model.getConfig())
        local = self.toolCoordinates
        if self.orientationConstrained:
            J = self.eeLink.getJacobian(local)
            wrench = [vectorops.dot(Jrow,t) for Jrow in J]
            return wrench[:3],wrench[3:]
        else:
            J = self.eeLink.getPositionJacobian(local)
            return [vectorops.dot(Jrow,t) for Jrow in J]


class _RobotInterfaceEmulatorData:
    def __init__(self,nd,klamptModel):
        self.klamptModel = klamptModel
        self.curClock = None
        self.lastClock = None
        self.dt = None
        self.jointData = [_JointInterfaceEmulatorData() for i in range(nd)]
        self.cartesianInterfaces = dict()
        self.commandSent = False

    def update(self,t,q,v):
        """Advances the interface"""
        self.lastClock = self.curClock
        self.curClock = t
        self.dt = t - self.lastClock
        for inds,c in self.cartesianInterfaces.items():
            qdes = c.update(q,self.curClock,self.dt)
            if qdes is not None:
                assert len(qdes) == len(c.indices)
                self.commandSent = False
                for i,x in zip(inds,qdes):
                    self.jointData[i].promote('p')
                    self.jointData[i].commandedPosition = x
                    self.jointData[i].commandedVelocity = (x-self.jointData[i].commandedPosition)/self.dt
                    self.jointData[i].commandTTL = 5.0*self.dt
        for i,j in enumerate(self.jointData):
            j.dt = self.dt
            j.update(self.curClock,q[i],v[i],self.dt)

    def updateCommand(self,qcmd,vcmd,tcmd):
        """Could be called before the emulator starts running to initialize the
        commanded joint positions before the emulator takes over.
        """
        assert qcmd is None or len(qcmd) == len(self.jointData)
        assert vcmd is None or len(vcmd) == len(self.jointData)
        assert tcmd is None or len(tcmd) == len(self.jointData)
        for i,j in enumerate(self.jointData):
            if qcmd is not None:
                j.commandedPosition = qcmd[i]
            if vcmd is not None:
                j.commandedVelocity = vcmd[i]
            if tcmd is not None:
                j.commandedTorque = tcmd[i]

    def desiredControlMode(self):
        if self.commandSent:
            return None
        commonMode = None
        precedence = 10000
        for j in self.jointData:
            if j.controlMode != commonMode:
                if commonMode is None:
                    commonMode = j.controlMode
                elif j.controlMode is None:
                    pass
                elif _JointInterfaceEmulatorData.CONTROL_MODE_PRECEDENCE[j.controlMode] < precedence:
                    precedence = _JointInterfaceEmulatorData.CONTROL_MODE_PRECEDENCE[j.controlMode]
                    commonMode = j.controlMode
        return commonMode

    def getCommand(self,commandType):
        res = [j.getCommand(commandType) for j in self.jointData]
        if commandType in ['v','t']:
            #extract out the TTL, take the max
            ttl = None
            if any(x[1] is not None for x in res):
                ttl = max(x[1] for x in res if x[1] is not None)
            return list(x[0] for x in res),ttl
        if commandType == 'pwl':
            unifiedTimes = None
            for times,positions in res:
                if unifiedTimes is None:
                    unifiedTimes = times
                elif unifiedTimes != times:
                    raise NotImplementedError("TODO: convert nonuniform piecewise linear times to position control")
            unifiedMilestones = []
            for i in range(len(unifiedTimes)):
                unifiedMilestones.append([traj[1][i] for traj in res])
            t0 = self.curClock-self.dt if self.curClock is not None else 0
            if any(t < t0 for t in unifiedTimes[1:]):
                print("Uh... have some times that are before current time",t0,"?",min(unifiedTimes[1:]))
            return [t-t0 for t in unifiedTimes][1:],unifiedMilestones[1:]
        if commandType == 'pwc':
            unifiedTimes = None
            for times,positions,velocities in res:
                if unifiedTimes is None:
                    unifiedTimes = times
                elif unifiedTimes != times:
                    raise NotImplementedError("TODO: convert nonuniform piecewise cubic times to position control")
            unifiedMilestones = []
            for i in range(len(unifiedTimes)):
                unifiedMilestones.append([traj[1][i] for traj in res])
            unifiedVelocities = []
            for i in range(len(unifiedTimes)):
                unifiedVelocities.append([traj[2][i] for traj in res])
            t0 = self.curClock if self.curClock is not None else 0
            istart = 0
            while istart < len(unifiedTimes) and unifiedTimes[istart] < t0:
                istart += 1
            if istart==len(unifiedTimes):
                print("WARNING: getCommand is returning empty command because no times are after current time???")
            return [t-t0 for t in unifiedTimes][istart:],unifiedMilestones[istart:],unifiedVelocities[istart:]
        return list(zip(*res))

    def promote(self,indices,controlType):
        """To accept commands of the given controlMode, switches over the
        current state to the controlMode"""
        self.commandSent = False
        if controlType == 'pwl':
            if any(self.jointData[i].controlMode == 'pwc' and len(self.jointData[i].trajectoryTimes) > 1 for i in indices):
                print("RobotInterfaceCompleter: Warning, converting from piecewise cubic to piecewise linear trajectory")
        for i in indices:
            self.jointData[i].promote(controlType)
        for i in indices:
            if self.jointData[i].externalController:
                self.jointData[i].externalController.active = False
                self.jointData[i].externalController = None

    def setPosition(self,indices,q):
        """Backup: runs an immediate position command"""
        self.promote(indices,'p')
        for (i,v) in zip(indices,q):
            self.jointData[i].commandedPosition = v

    def moveToPosition(self,indices,q,speed):
        """Backup: runs a position command using a piecewise linear trajectory"""
        model = self.klamptModel
        if model is None:
            #TODO: warn that move-to is unavailable?
            self.promote(indices,'p')
            for (i,v) in zip(indices,q):
                self.jointData[i].commandedPosition = v
        else:
            #move to command emulation using fixed-velocity
            qmin,qmax = model.getJointLimits()
            xmin = model.configToDrivers(qmin)
            xmax = model.configToDrivers(qmax)
            vmax = model.velocityToDrivers(model.getVelocityLimits())
            amax = model.velocityToDrivers(model.getAccelerationLimits())
            xmin = [xmin[i] for i in indices]
            xmax = [xmax[i] for i in indices]
            vmax = [vmax[i] for i in indices]
            amax = [amax[i] for i in indices]
            qcmd = [self.jointData[i].commandedPosition if self.jointData[i].commandedPosition is not None else self.jointData[i].sensedPosition for i in indices]
            dqcmd = [self.jointData[i].commandedVelocity if self.jointData[i].commandedVelocity is not None else self.jointData[i].sensedVelocity for i in indices]
            ts,xs,vs = motionplanning.interpolateNDMinTime(qcmd,dqcmd,q,[0]*len(q),xmin,xmax,vmax,amax)
            ts,xs,vs = motionplanning.combineNDCubic(ts,xs,vs)
            self.setPiecewiseCubic(indices,ts,xs,vs,True)
            #t = max(abs(qi-qi0)/vimax for (qi,qi0,vimax) in zip(q,qcmd,vmax))
            #self.setPiecewiseLinear(indices,[t],[q],True)

    def setVelocity(self,indices,v,ttl):
        """Backup: runs a velocity command for ttl seconds using a piecewise linear
        trajectory.  If ttl is not specified, uses ttl=1."""
        qcmd = [self.jointData[i].commandedPosition if self.jointData[i].commandedPosition is not None else self.jointData[i].sensedPosition for i in indices]
        if ttl is None:
            ttl = 1.0
            model = self.klamptModel
            if model is not None:
                qmin,qmax = model.getJointLimits()
                xmin = model.configToDrivers(qmin)
                xmax = model.configToDrivers(qmax)
                xmin = [xmin[i] for i in indices]
                xmax = [xmax[i] for i in indices]
                #stop when the first joint limit is hit
                ttl = 1.0
                for i in range(len(indices)):
                    if qcmd[i] < xmin[i] or qcmd[i] > xmax[i]:
                        raise ValueError("Current position %d is out of joint limits: %f <= %f <= %f"%(i,xmin[i],qcmd[i],xmax[i]))
                    if qcmd[i] + ttl*v[i] < xmin[i]:
                        ttl = (xmin[i]+1e-3-qcmd[i])/v[i]
                    if qcmd[i] + ttl*v[i] > xmax[i]:
                        ttl = (xmax[i]-1e-3-qcmd[i])/v[i]
        q0 = qcmd
        q = vectorops.madd(q0,v,ttl)
        self.setPiecewiseLinear(indices,[ttl],[q],True)

    def setTorque(self,indices,t,ttl):
        self.promote(indices,'t')
        for i,v in zip(indices,t):
            self.jointIndices[i].commandedTorque = v
            self.jointIndices[i].commandTTL = ttl

    def setPID(self,indices,q,dq,t):
        if t is None:
            t = [0.0]*len(q)
        self.promote(indices,'pid')
        for i,qi,dqi,ti in zip(indices,q,dq,t):
            self.jointIndices[i].pidCmd = (qi,dqi,ti)

    def setPiecewiseLinear(self,indices,ts,qs,relative):
        assert self.curClock is not None
        assert len(ts) == len(qs)
        if relative:
            if any(t < 0 for t in ts):
                raise ValueError("Can't set a trajectory with negative relative times")
            ts = [t+self.curClock for t in ts]
        else:
            if any(t < self.curClock for t in ts):
                raise ValueError("Can't set a trajectory with absolute times < clock time")
        for i,q in enumerate(qs):
            assert len(q) == len(indices),"Trajectory milestone %d is of incorrect size %d != %d"%(i,len(q),len(indices))
        if ts[0] > self.curClock:
            ts = [self.curClock] + ts
            qs = [[]] + qs
            for i in indices:
                if self.jointData[i].commandedPosition is None:
                    if self.jointData[i].sensedPosition is None:
                        raise RuntimeError("Can't set a trajectory before first controller clock cycle")
                    qs[0].append(self.jointData[i].sensedPosition)
                else:
                    qs[0].append(self.jointData[i].commandedPosition)

        self.promote(indices,'pwl')
        for k,i in enumerate(indices):
            self.jointData[i].trajectoryTimes = ts
            self.jointData[i].trajectoryMilestones = [q[k] for q in qs]

    def setPiecewiseCubic(self,indices,ts,qs,vs,relative):
        assert self.curClock is not None
        assert len(ts) == len(qs)
        assert len(ts) == len(vs)
        if relative:
            if any(t < 0 for t in ts):
                raise ValueError("Can't set a trajectory with negative relative times")
            ts = [t+self.curClock for t in ts]
        else:
            if any(t < self.curClock for t in ts):
                raise ValueError("Can't set a trajectory with absolute times < clock time")
        for q in qs:
            assert len(q) == len(indices),"Trajectory milestone is of incorrect size"
        for v in vs:
            assert len(v) == len(indices),"Trajectory milestone velocity is of incorrect size"
        if ts[0] > self.curClock:
            ts = [self.curClock] + ts
            qs = [[]] + qs
            vs = [[]] + vs
            for i in indices:
                if self.jointData[i].commandedPosition is None:
                    if self.jointData[i].sensedPosition is None:
                        raise RuntimeError("Can't set a trajectory before first controller clock cycle")
                    qs[0].append(self.jointData[i].sensedPosition)
                else:
                    qs[0].append(self.jointData[i].commandedPosition)
                if self.jointData[i].commandedVelocity is None:
                    if self.jointData[i].sensedVelocity is None:
                        vs[0].append(0)
                    else:
                        vs[0].append(self.jointData[i].sensedVelocity)
                else:
                    vs[0].append(self.jointData[i].commandedVelocity)

        self.promote(indices,'pwc')
        for k,i in enumerate(indices):
            self.jointData[i].trajectoryTimes = ts
            self.jointData[i].trajectoryMilestones = [q[k] for q in qs]
            self.jointData[i].trajectoryVelocities = [v[k] for v in vs]

    def isMoving(self,indices):
        for i in indices:
            j = self.jointData[i]
            if j.controlMode == 'v':
                if j.commandedVelocity != 0:
                    return True
            elif j.controlMode == 'p':
                if abs(j.commandedPosition - j.sensedPosition) > 1e-3:
                    return True
            elif j.controlMode == 'pid':
                if j.commandedVelocity != 0:
                    return True
                if abs(j.commandedPosition - j.sensedPosition) > 1e-3:
                    return True
            elif j.controlMode in ['pwl','pwc']:
                if self.curClock < j.destinationTime(self.curClock):
                    return True
        return False

    def commandedPosition(self):
        return [j.commandedPosition for j in self.jointData]

    def commandedVelocity(self):
        return [j.commandedVelocity for j in self.jointData]

    def commandedTorque(self):
        return [j.commandedTorque for j in self.jointData]

    def sensedPosition(self):
        return [j.sensedPosition for j in self.jointData]

    def sensedVelocity(self):
        return [j.sensedVelocity for j in self.jointData]

    def destinationPosition(self):
        return [j.destinationPosition() for j in self.jointData]

    def destinationVelocity(self):
        return [j.destinationVelocity() for j in self.jointData]

    def destinationTime(self):
        return max(j.destinationTime(self.curClock) for j in self.jointData)

    def setToolCoordinates(self,xtool_local,indices):
        try:
            c = self.cartesianInterfaces[tuple(indices)]
        except KeyError:
            c = _CartesianEmulatorData(self.klamptModel,indices)
            self.cartesianInterfaces[tuple(indices)] = c
        c.setToolCoordinates(xtool_local)

    def getToolCoordinates(self,indices):
        try:
            c = self.cartesianInterfaces[tuple(indices)]
        except KeyError:
            return [0,0,0]
        return c.getToolCoordinates()

    def setGravityCompensation(self,gravity,load,load_com,indices):
        raise NotImplementedError("TODO: implement gravity compensation?")

    def setCartesianPosition(self,xparams,indices):
        try:
            c = self.cartesianInterfaces[tuple(indices)]
        except KeyError:
            c = _CartesianEmulatorData(self.klamptModel,indices)
            self.cartesianInterfaces[tuple(indices)] = c
        for i,j in enumerate(self.jointData):
            self.klamptModel.driver(i).setValue(j.sensedPosition if j.commandedPosition is None else j.commandedPosition)
        c.active = False
        qKlamptNext = c.solveIK(xparams)
        self.klamptModel.setConfig(qKlamptNext)
        self.setPosition(indices,[self.klamptModel.driver(i).getValue() for i in indices])

    def moveToCartesianPosition(self,xparams,speed,indices):
        try:
            c = self.cartesianInterfaces[tuple(indices)]
        except KeyError:
            c = _CartesianEmulatorData(self.klamptModel,indices)
            self.cartesianInterfaces[tuple(indices)] = c
        for i,j in enumerate(self.jointData):
            self.klamptModel.driver(i).setValue(j.sensedPosition if j.commandedPosition is None else j.commandedPosition)
        c.active = False
        qKlamptNext = c.solveIK(xparams)
        self.klamptModel.setConfig(qKlamptNext)
        self.moveToPosition(indices,[self.klamptModel.driver(i).getValue() for i in indices],speed)

    def setCartesianVelocity(self,dxparams,ttl,indices):
        try:
            c = self.cartesianInterfaces[tuple(indices)]
        except KeyError:
            c = _CartesianEmulatorData(self.klamptModel,indices)
            self.cartesianInterfaces[tuple(indices)] = c
        qstart = [j.sensedPosition if j.commandedPosition is None else j.commandedPosition for j in self.jointData]
        c.setCartesianVelocity(qstart,dxparams,ttl)
        for i in indices:
            self.jointData[i].externalController = c

    def setCartesianForce(self,fparams,ttl,indices):
        try:
            c = self.cartesianInterfaces[tuple(indices)]
        except KeyError:
            c = _CartesianEmulatorData(self.klamptModel,indices)
            self.cartesianInterfaces[tuple(indices)] = c
        raise NotImplementedError("TODO: implement cartesian force?")
        qstart = [j.sensedPosition if j.commandedPosition is None else j.commandedPosition for j in self.jointData]
        c.setCartesianForce(qstart,fparams,ttl)

    def cartesianPosition(self,q,indices):
        assert len(q) == len(self.jointData),"cartesianPosition: must use full-body configuration"
        try:
            c = self.cartesianInterfaces[tuple(indices)]
            return c.cartesianPosition(q)
        except KeyError:
            raise ValueError("Invalid Cartesian index set for emulator: no command currently set")

    def cartesianVelocity(self,q,dq,indices):
        assert len(q) == len(self.jointData),"cartesianVelocity: must use full-body configuration"
        assert len(dq) == len(self.jointData),"cartesianVelocity: must use full-body configuration"
        try:
            c = self.cartesianInterfaces[tuple(indices)]
            return c.cartesianVelocity(q,dq)
        except KeyError:
            raise ValueError("Invalid Cartesian index set for emulator: no command currently set")

    def cartesianForce(self,q,t,indices):
        assert len(q) == len(self.jointData),"cartesianForce: must use full-body configuration"
        assert len(t) == len(self.jointData),"cartesianForce: must use full-body configuration"
        try:
            c = self.cartesianInterfaces[tuple(indices)]
            return c.cartesianForce(q,t)
        except KeyError:
            raise ValueError("Invalid Cartesian index set for emulator: no command currently set")

    def print_status(self):
        joint_control = [True]*len(self.jointData)
        for k,c in self.cartesianInterfaces.items():
            if c.active:
                print("Cartesian interface active on joints",k)
                print("  Tool coordinates",c.toolCoordinates)
                print("  Drive command",c.driveCommand)
                for j in k:
                    joint_control[j] = False
        for i,j in enumerate(self.jointData):
            if not joint_control[i]: continue
            print("Joint",i,"control mode",j.controlMode)



class _SubRobotInterfaceCompleter(RobotInterfaceCompleter):
    def __init__(self,interface,part,joint_idx):
        RobotInterfaceCompleter.__init__(self,interface._base)
        #it's a sub-interface
        assert isinstance(interface,RobotInterfaceBase)
        assert not isinstance(interface._base,RobotInterfaceBase),"Can't do nested sub-interfaces"
        assert interface._emulator._curClock is not None,"Interface needs to be initialized before accessing a part"
        self._parent = interface
        self.properties = interface.properties.copy()
        if part is not None:
            if joint_idx is None:
                self.properties['name'] = part
            else:
                self.properties['name'] = part + '[' + str(joint_idx) + ']'
        else:
            self.properties['name'] = str(joint_idx)
        self._klamptModel = interface._klamptModel
        self._worldModel = interface._worldModel
        self._subRobot  = True
        self._indices = interface._base.indices(part,joint_idx)
        self._parts = dict()
        self._parts[None] = self._indices
        self._has = interface._has
        self._emulator = interface._emulator

    def __str__(self):
        return str(self._parent)+'['+self.properties['name']+']'

    def addPart(self,name,indices):
        raise RuntimeError("Can't add sub-parts to part")

    def initialize(self):
        raise RuntimeError("Can't call initialize() on a sub-robot interface.")

    def startStep(self):
        raise RuntimeError("Can't call startStep() on a sub-robot interface.")

    def endStep(self):
        raise RuntimeError("Can't call endStep() on a sub-robot interface.")

    def _toPart(self,q):
        return [q[i] for i in self._indices]

    def sensedPosition(self):
        return self._toPart(self._parent.sensedPosition())

    def sensedVelocity(self):
        return self._toPart(self._parent.sensedVelocity())

    def sensedTorque(self):
        return self._toPart(self._parent.sensedTorque())

    def commandedPosition(self):
        return self._toPart(self._parent.commandedPosition())

    def commandedVelocity(self):
        return self._toPart(self._parent.commandedPosition())

    def commandedTorque(self):
        return self._toPart(self._parent.commandedTorque())

    def destinationPosition(self):
        return self._toPart(self._parent.destinationPosition())

    def destinationVelocity(self):
        return self._toPart(self._parent.destinationVelocity())
