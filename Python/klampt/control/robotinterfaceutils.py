"""Contains utilities for the Klampt Robot Interface Layer. 

The :class:`RobotInterfaceCompleter` class is extremely widely used to
standardize the capabilities of :class:`RobotInterfaceBase` to handle
advanced functions. For example, if you have a position controlled robot
(e.g., an Arduino-controlled set of motors) and want to do more
sophisticated work with it, you can create a completer and get access to
motion queues and Cartesian control.

You can also assemble unified interfaces to robots composed of separate
independent parts via :class:`MultiRobotInterface`. For example, if you
have a robot arm + gripper, each working on separate interfaces, creating
a MultiRobotInterface will let you simultaneously control both.

Logging... TODO

"""

from .robotinterface import *
from ..math import vectorops,so2,so3,se3,spline
from ..plan import motionplanning
from ..model.trajectory import Trajectory,HermiteTrajectory
from .cartesian_drive import CartesianDriveSolver
import bisect
import math
import warnings

class RobotInterfaceCompleter(RobotInterfaceBase):
    """Completes as much of the RobotInterfaceBase API as possible from a
    partially specified RobotInterfaceBase.  For example, if your
    RobotInterfaceBase implements position control, then the Completer
    will provide velocity, piecewise linear, and piecewise cubic, and Cartesian
    control commands.  

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
        - Optional safety checks (joint limits, velocity limits, discontinuous
          commands, and tracking errors)

    .. note::
        The base object's klamptModel() method must be implemented for
        Cartesian control and acceleration-bounded control to work properly.

    Args:
        base_interface (RobotInterfaceBase): the partially-implemented
            interface.
        base_initialized (bool, optional): if True, initialize() will not
            initialize the base.
        joint_limit_checking (bool or pair, optional): if True, enables joint
            limit checking using the robot model in klamptModel. Otherwise,
            this is a pair of joint minima and maxima (radians). 
        velocity_limit_checking (bool or pair, optional): if True, enables
            joint velocity limit checking using the robot model in klamptModel.
            Otherwise, this is a pair of joint velocity minima and maxima
            (radians/s).
        discontinuity_checking (float or list, optional): raises an error
            if the commanded position has a discontinuity above this threshold
            (radians).  A per-joint threshold can also be given.
        tracking_error_checking (float, or list, optional): raises an
            error if the commanded and sensed joint positions deviate beyond
            this threshold (a sort of collision warning).  A per-joint threshold
            can be given.
        self_collision_checking (bool, float, or list of floats, optional): if
            True or a float, performs self-collision checking with the given
            margin.  A per-link margin can also be given.  (klamptModel() must
            be defined)
        obstacle_collision_checking (WorldModel, optional): if given, performs
            collision between the object and the given world.  (klamptModel()
            must be defined)
    
    If any of the checkers are raised, the robot is stopped using a soft stop,
    and status() is turned to "joint limit violation", "velocity limit
    violation", "discontinuity violation", etc.
    """
    def __init__(self,base_interface,base_initialized=False,
        joint_limit_checking=False,velocity_limit_checking=False,discontinuity_checking=math.radians(10),
        tracking_error_checking=math.radians(30),self_collision_checking=False,obstacle_collision_checking=None):
        RobotInterfaceBase.__init__(self)
        self._base = base_interface
        self._baseInitialized  = base_initialized
        self._parent = None
        self._subRobot = False
        self._parts = None
        self._baseParts = None
        self._has = dict()
        self._indices = None
        self._emulator = None
        self._emulatorControlMode = None
        self._baseControlMode = None
        self._inStep = False
        self._joint_limit_checking = joint_limit_checking
        self._velocity_limit_checking=velocity_limit_checking
        self._discontinuity_checking=discontinuity_checking
        self._tracking_error_checking=tracking_error_checking
        self._self_collision_checking=self_collision_checking
        self._obstacle_collision_checking=obstacle_collision_checking

    def __str__(self):
        return "Completer("+str(self._base)+')'

    def numJoints(self,part=None):
        if self._parts is not None and part in self._parts:
            return len(self._parts[part])
        return self._base.numJoints(part)

    def parts(self):
        if self._parts is None: #pre-initialization
            return self._base.parts()
        return self._parts

    def addPart(self,name,indices):
        """Can add new parts, e.g., for Cartesian control. 
        
        As an example, suppose we've created a RobotInterfaceCompleter for a
        6DOF robot with 1DOF gripper, with the base controller using position
        control of all 7 DOFs. We can then create an arm and a gripper via::

            robotinterface.addPart("arm",list(range(6)))
            robotinterface.addPart("gripper",[6])
            #start moving the arm upward
            robotinterface.partInterface("arm").setCartesianVelocity([0,0,0],[0,0,0.1])

        """
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
                        try:
                            a = a()
                        except Exception:
                            print("Exception raised while evaluating argument of function",f,"of base",str(self._base))
                            raise
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
                    raise NotImplementedError("Function {} is not implemented by base interface {}, no fallback available".format(fn,self._base))
                #print("...not available, returning",fallback(*args))
                return fallback(*args)
            except Exception:
                print("Error received during first call of ",fn,"of base",str(self._base))
                raise
        except Exception:
                print("Error received during later call of ",fn,"of base",str(self._base))
                raise

    def initialize(self):
        if self._indices is not None:
            warnings.warn("RobotInterfaceCompleter(%s): Should only call initialize() once."%(str(self._base),))
            return True
        if not self._baseInitialized:
            if not self._base.initialize():
                warnings.warn("RobotInterfaceCompleter(%s): base controller did not initialize"%(str(self._base),))
                return False
        self._baseInitialized = True
        #discover capabilities
        self._baseParts = self._base.parts()
        self._parts = self._baseParts.copy()
        self._indices = self.indices()
        self._try('controlRate',[],lambda :0)
        curclock = self._try('clock',[],lambda :0)
        if not self._has['controlRate'] and not self._has['clock']:
            warnings.warn("RobotInterfaceCompleter(%s): Need at least one of controlRate() and clock() to be implemented"%(str(self._base),))
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
            warnings.warn("RobotInterfaceCompleter(%s): Need at least one of sensedPosition() and commandedPosition() to be implemented"%(str(self._base),))
            return False
        self._emulator = _RobotInterfaceEmulatorData(self._base.numJoints(),self._base.klamptModel())
        self._emulator.curClock = curclock
        print("Starting with clock",curclock)
        assert curclock is not None
        self._emulator.lastClock = None
        return True

    def close(self):
        self._base_initialized=False
        return self._base.close()

    def startStep(self):
        assert self._indices is not None,"RobotInterface not initialized yet"
        assert not self._subRobot,"Can't do startStep on a sub-interface"
        assert not self._inStep,"startStep called twice in a row?"
        self._inStep = True
        self._try('startStep',[],lambda *args:0)
        if self._emulator.lastClock is None:
            qcmd = self._try('commandedPosition',[],lambda *args:None)
            vcmd = self._try('commandedVelocity',[],lambda *args:None)
            tcmd = self._try('commandedTorque',[],lambda *args:None)
            self._emulator.updateCommand(qcmd,vcmd,tcmd)
        if not self._has['controlRate']:
            self._emulator.pendingClock = self.clock()
            if self._emulator.lastClock is not None and self._emulator.lastClock != self._emulator.pendingClock:
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
        assert self._inStep,"endStep called outside of a step?"
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
            res=self._try(['setPID','setTorque','setPosition','setVelocity','moveToPosition'],[self._emulator.getCommand('pid'),
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
        self._inStep = False

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

    def partInterface(self,part=None,joint_idx=None):
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

    def sensorMeasurements(self,sensor):
        return self._base.sensorMeasurements(sensor)

    def sensorUpdateTime(self,sensor):
        return self._base.sensorUpdateTime(sensor)

    def setPosition(self,q):
        assert len(q) == len(self._indices),"Position vector has incorrect length: {} != {}".format(len(q),len(self._indices))
        self._emulator.setPosition(self._indices,q)

    def setVelocity(self,v,ttl=None):
        assert len(v) == len(self._indices),"Velocity vector has incorrect length: {} != {}".format(len(v),len(self._indices))
        self._emulator.setVelocity(self._indices,v,ttl)

    def setTorque(self,t,ttl=None):
        assert len(t) == len(self._indices),"Torque vector has incorrect length: {} != {}".format(len(t),len(self._indices))
        assert ttl is None or isinstance(ttl,(float,int)),"ttl must be a number"
        self._emulator.setTorque(self._indices,t,ttl)
        
    def setPID(self,q,dq,t=None):
        assert len(q) == len(self._indices),"Position vector has incorrect length: {} != {}".format(len(q),len(self._indices))
        assert len(dq) == len(self._indices),"Velocity vector has incorrect length: {} != {}".format(len(dq),len(self._indices))
        self._emulator.setPID(self._indices,q,dq,t)

    def setPIDGains(self,kP,kI,kD):
        assert len(kP) == len(self._indices),"kP vector has incorrect length: {} != {}".format(len(kP),len(self._indices))
        assert len(kI) == len(self._indices),"kD vector has incorrect length: {} != {}".format(len(kI),len(self._indices))
        assert len(kD) == len(self._indices),"kD vector has incorrect length: {} != {}".format(len(kD),len(self._indices))
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
        assert len(q) == len(self._indices),"Position vector has incorrect length: {} != {}".format(len(q),len(self._indices))
        assert isinstance(speed,(float,int))
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

    def setCartesianPosition(self,xparams,frame='world'):
        assert len(xparams)==2 or len(xparams)==3, "Cartesian target must be a point or (R,t) transform"
        self._emulator.setCartesianPosition(xparams,frame,self._indices)

    def moveToCartesianPosition(self,xparams,speed=1.0,frame='world'):
        assert len(xparams)==2 or len(xparams)==3, "Cartesian target must be a point or (R,t) transform"
        assert isinstance(speed,(float,int)),"Speed must be a number"
        self._emulator.moveToCartesianPosition(xparams,speed,frame,self._indices)

    def setCartesianVelocity(self,dxparams,ttl=None,frame='world'):
        assert len(dxparams)==2 or len(dxparams)==3, "Cartesian velocity must be a velocity or (angularVelocity,velocity) pair"
        assert ttl is None or isinstance(ttl,(int,float)), "ttl must be None or a number"
        self._emulator.setCartesianVelocity(dxparams,ttl,frame,self._indices)

    def setCartesianForce(self,fparams,ttl=None,frame='world'):
        assert len(fparams)==2 or len(fparams)==3, "Cartesian force must be a force or (torque,force) pair"
        assert ttl is None or isinstance(ttl,(int,float)), "ttl must be None or a number"
        self._emulator.setCartesianForce(fparams,ttl,frame,self._indices)

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

    def cartesianPosition(self,q,frame='world'):
        if self._baseControlMode == self._emulatorControlMode:
            #using base interface
            return self._try('cartesianPosition',[q,frame],lambda q,frame: self._emulator.cartesianPosition(q,frame,self._indices))
        else:
            return self._emulator.cartesianPosition(q,frame,self._indices)

    def cartesianVelocity(self,q,dq,frame='world'):
        if self._baseControlMode == self._emulatorControlMode:
            #using base interface
            return self._try('cartesianVelocity',[q,dq,frame],lambda q,dq,frame: self._emulator.cartesianPosition(q,dq,frame,self._indices))
        else:
            return self._emulator.cartesianVelocity(q,dq,frame,self._indices)

    def cartesianForce(self,q,t,frame='world'):
        if self._baseControlMode == self._emulatorControlMode:
            #using base interface
            return self._try('cartesianForce',[q,t,frame],lambda q,t,frame: self._emulator.cartesianForce(q,t,frame,self._indices))
        else:
            return self._emulator.cartesianForce(q,t,self._indices)

    def sensedCartesianPosition(self,frame='world'):
        return self._try('sensedCartesianPosition',[frame],lambda frame: RobotInterfaceBase.sensedCartesianPosition(self,frame))

    def sensedCartesianVelocity(self,frame='world'):
        return self._try('sensedCartesianVelocity',[frame],lambda frame: RobotInterfaceBase.sensedCartesianVelocity(self,frame))

    def sensedCartesianForce(self,frame='world'):
        return self._try('sensedCartesianForce',[frame],lambda frame: RobotInterfaceBase.sensedCartesianForce(self,frame))

    def commandedCartesianPosition(self,frame='world'):
        return self._try('commandedCartesianPosition',[frame],lambda frame: RobotInterfaceBase.commandedCartesianPosition(self,frame))

    def commandedCartesianVelocity(self,frame='world'):
        return self._try('commandedCartesianVelocity',[frame],lambda frame: RobotInterfaceBase.commandedCartesianVelocity(self,frame))

    def commandedCartesianForce(self,frame='world'):
        return self._try('destinationCartesianForce',[frame],lambda frame: RobotInterfaceBase.destinationCartesianForce(self,frame))

    def destinationCartesianPosition(self,frame='world'):
        return self._try('destinationCartesianPosition',[frame],lambda frame: RobotInterfaceBase.destinationCartesianPosition(self,frame))

    def destinationCartesianVelocity(self,frame='world'):
        return self._try('destinationCartesianVelocity',[frame],lambda frame: RobotInterfaceBase.destinationCartesianVelocity(self,frame))

    def partToRobotConfig(self,pconfig,part,robotConfig):
        return self._base.partToRobotConfig(pconfig,part,robotConfig)

    def klamptModel(self):
        return self._base.klamptModel()

    def fromKlamptConfig(self,klamptConfig,part=None,joint_idx=None):
        return self._base.fromKlamptConfig(klamptConfig,part=None,joint_idx=None)

    def fromKlamptVelocity(self,klamptVelocity,part=None,joint_idx=None):
        return self._base.fromKlamptVelocity(klamptVelocity,part=None,joint_idx=None)

    def toKlamptConfig(self,config,klamptConfig=None,part=None,joint_idx=None):
        return self._base.toKlamptConfig(config,klamptConfig,part=None,joint_idx=None)

    def toKlamptVelocity(self,velocity,klamptVelocity,part=None,joint_idx=None):
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
    """A RobotInterfaceBase that consists of multiple parts, which are
    communicated with separately.  For example, a mobile manipulator can
    consist of a base, arms, and grippers.

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
        RobotInterfaceBase.__init__(self)
        self._initialized = False
        self._partNames = []
        self._partInterfaces = dict()
        self._parts = dict()
        self._parts[None] = []
        self._jointToPart = []   #before initialization, a list of (part,index) pairs
        self._topology = dict()  #maps parts to parent (part,klampt link index)
        self._klamptModel = None
        self._klamptParts = dict()

    def addPart(self,partName,partInterface,klamptModel=None,klamptIndices=None):
        """Adds a part `partName` with the given interface `partInterface`. 
        The DOFs of the unified robot range from N to N+Np where N is the 
        current number of robot DOFs and Np is the number of part DOFs.

        If `klamptModel` / `klamptIndices` are given, the part is
        associated with the given `klamptModel`, and the indices of the
        part's sub-model are given by klamptIndices.  A MultiRobotInterface
        must be provided with a unified klamptModel with all parts mounted as
        appropriate.

        For example, if you want to assemble a unified interface / klamptModel
        from two interfaces, with part2 (e.g., a gripper) mounted on part1
        (e.g., an arm), you'd run::

            world = WorldModel()
            world.readFile(part1_interface.properties['klamptModelFile'])
            world.readFile(part2_interface.properties['klamptModelFile'])
            part1_model = world.robot(0)
            part2_model = world.robot(1)
            part1_indices = list(range(part1_model.numLinks()))
            part2_indices = list(range(part1_model.numLinks(),part1_model.numLinks()+part2_model.numLinks()))
            ee_link = part1_model.numLinks()-1
            mount_rotation = ... some so3 element...
            mount_translation = ... some translation vector...
            part1_model.mount(ee_link,part2_model,mount_rotation,mount_translation)  #part1 model is now the whole robot
            multi_robot_model = world.robot(0)
            world.remove(part2_model)  #not strictly necessary to remove the model from the world...
            voltron_controller = MultiRobotInterface()
            voltron_controller.addPart("part1",part1_interface,multi_robot_model,part1_indices)
            voltron_controller.addPart("part2",part2_interface,multi_robot_model,part2_indices)
            if not voltron_controller.initailize():
                print("hmm... error on initialize?")
            ... do stuff, treating voltron_controller as a unified robot ...

        For example, if your robot has a RobotInfo structure associated with
        it, you'd run::

            from klampt.control.robotinterfaceutils import RobotInterfaceCompleter,MultiRobotInterface
            voltron_controller = MultiRobotInterface()
            for part in robotInfo.parts:
                part_controller = RobotInterfaceCompleter([create the part controller])
                voltron_controller.addPart(part,part_controller,robotInfo.klamptModel(),robotInfo.partLinkIndices(part))
            if not voltron_controller.initialize():
                print("hmm... error on initialize?")
            ... do stuff, treating voltron_controller as a unified robot ...
        
        Args:
            partName (str): the identifier to be used when getting parts.
            partInterface (RobotInterfaceBase): the interface to control this
                part. Highly recommended to wrap your controller in
                :class:`RobotInterfaceCompleter`.
            klamptModel (RobotModel): the unified Klampt model, for which this
                is a part.
            klamptIndices (list of int): the indices of the Klampt model
                corresponding to the degrees of 

        """
        self._partNames.append(partName)
        self._partInterfaces[partName] = partInterface

        part_ndof = partInterface.numJoints()
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
            from ..model.subrobot import SubRobotModel
            assert klamptModel is not None,"Need to specify a Klamp't model"
            self._klamptParts[partName] = klamptIndices
            self._partInterfaces[partName]._klamptModel = SubRobotModel(self._klamptModel,klamptIndices)

            p = self._klamptModel.link(klamptIndices[0]).getParent()
            if p >= 0:
                # TODO: when a part is attached to another part, its cartesian position will depend
                # on that other part's position.  How do we make sure that the model is updated properly,
                # before any IK solves, the 'base' frame is known to the part, etc?
                parent_part = None
                for k,iface in self._partInterfaces.items():
                    if p in iface._klamptModel._links:
                        parent_part = k
                        break
                if parent_part is not None:
                    #TODO: what of this?
                    self._topology[partName] = (parent_part,p)
                    pass
                else:
                    #assume it's not controlled?
                    self._topology[partName] = (None,p)
                    pass

    def numJoints(self,part=None):
        return len(self._parts[part])

    def parts(self):
        return self._parts

    def controlRate(self):
        return max(c.controlRate() for (p,c) in self._partInterfaces.items())

    def initialize(self):
        if self._initialized:
            return True
        self._initialized = True
        for (p,c) in self._partInterfaces.items():
            if not c.initialize():
                warnings.warn("MultiRobotInterface: Part {} failed to initialize".format(p))
                return False
        self._jointToPart = dict((j,p) for (p,j) in self._jointToPart)
        return True

    def close(self):
        res = True
        for (p,c) in self._partInterfaces.items():
            if not c.close():
                warnings.warn("MultiRobotInterface: Part {} failed to close".format(p))
                res = False
        self._initialized = False
        return res

    def startStep(self):
        for (p,c) in self._partInterfaces.items():
            c.startStep()
            # TODO: If there are nested Cartesian commands or the klamptModel() object
            # is accessed, then the reference model used by sub-robots may be invalidated!
            if p in self._klamptParts:
                try:
                    c._klamptModel.setConfig(c.configToKlampt(c.commandedPosition()))
                    c._klamptModel.setVelocity(c.configToKlampt(c.commandedVelocity()))
                except NotImplementedError:
                    c._klamptModel.setConfig(c.configToKlampt(c.sensedPosition()))
                    try:
                        c._klamptModel.setVelocity(c.configToKlampt(c.sensedVelocity()))
                    except NotImplementedError:
                        pass
    
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

    def partInterface(self,part=None,joint_idx=None):
        if part is None and joint_idx is None:
            return self
        assert part in self._partInterfaces
        if joint_idx is None:
            return self._partInterfaces[part]
        else:
            return self._partInterfaces[part].partInterface(None,joint_idx)

    def jointName(self,joint_idx,part=None):
        if part is None:
            part = self._jointToPart[joint_idx]
        return part + ' ' + self._partInterfaces[part].jointName(joint_idx)

    def sensors(self):
        s = []
        for (p,c) in self._partInterfaces.items():
            s += [p+'__'+n for n in c.sensors()]
        return s

    def enabledSensors(self):
        s = []
        for (p,c) in self._partInterfaces.items():
            s += [p+'__'+n for n in c.enabledSensors()]
        return s

    def hasSensor(self,sensor):
        return sensor in self.sensors()

    def enableSensor(self,sensor):
        assert isinstance(sensor,(list,tuple))
        part,partsensor = sensor.split('__',1)
        return self._partInterfaces[part].enableSensor(partsensor)

    def sensorMeasurements(self,sensor):
        part,partsensor = sensor.split('__',1)
        return self._partInterfaces[part].sensorMeasurements(partsensor)

    def sensorUpdateTime(self,sensor):
        part,partsensor = sensor.split('__',1)
        return self._partInterfaces[part].sensorUpdateTime(partsensor)

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

    def setPosition(self,q):
        self._setSplit(q,'setPosition')
    
    def moveToPosition(self,q,speed=1.0):
        self._setSplit(q,'moveToPosition',speed)
        
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

    def setCartesianPosition(self,xparams,frame='world'):
        raise ValueError("Can't do cartesian control without specifying a part")

    def moveToCartesianPosition(self,xparams,speed=1.0,frame='world'):
        raise ValueError("Can't do cartesian control without specifying a part")

    def setCartesianVelocity(self,dxparams,ttl=None,frame='world'):
        raise ValueError("Can't do cartesian control without specifying a part")

    def setCartesianForce(self,fparams,ttl=None,frame='world'):
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

    def cartesianPosition(self,q,frame='world'):
        raise ValueError("Can't do cartesian get without specifying a part")

    def cartesianVelocity(self,q,dq,frame='world'):
        raise ValueError("Can't do cartesian get without specifying a part")

    def cartesianForce(self,q,t,frame='world'):
        raise ValueError("Can't do cartesian get without specifying a part")

    def sensedCartesianPosition(self,frame='world'):
        raise ValueError("Can't do cartesian get without specifying a part")

    def sensedCartesianVelocity(self,frame='world'):
        raise ValueError("Can't do cartesian get without specifying a part")

    def sensedCartesianForce(self,frame='world'):
        raise ValueError("Can't do cartesian get without specifying a part")

    def commandedCartesianPosition(self,frame='world'):
        raise ValueError("Can't do cartesian get without specifying a part")

    def commandedCartesianVelocity(self,frame='world'):
        raise ValueError("Can't do cartesian get without specifying a part")

    def commandedCartesianForce(self,frame='world'):
        raise ValueError("Can't do cartesian get without specifying a part")

    def destinationCartesianPosition(self,frame='world'):
        raise ValueError("Can't do cartesian get without specifying a part")

    def destinationCartesianVelocity(self,frame='world'):
        raise ValueError("Can't do cartesian get without specifying a part")


class _JointInterfaceEmulatorData:
    CONTROL_MODES = ['pid','v','p','m','pwl','pwc']
    CONTROL_MODE_PRECEDENCE = {'pid':0,'v':1,'p':2,'pwl':3,'pwc':4}

    def __init__(self,name):
        self.name = name
        self.dt = None
        self.controlMode = None
        self.sensedPosition = None
        self.sensedVelocity = None
        self.commandedPosition = None
        self.commandedVelocity = None
        self.commandedTorque = None
        self.commandTTL = None
        self.continuousRotation = False
        self.lastCommandedPosition = None
        self.commandParametersChanged = False
        self.pidCmd = None
        self.pidGains = None
        self.pidIntegralError = None
        self.trajectoryTimes = None
        self.trajectoryMilestones = None
        self.trajectoryVelocities = None
        self.externalController = None

    def _positionDifference(self,a,b):
        if self.continuousRotation:
            return so2.diff(a,b)
        else:
            return a-b

    def update(self,t,q,v,dt):
        if v is None:
            if self.sensedPosition is None:
                self.sensedVelocity = 0
            else:
                self.sensedVelocity = self._positionDifference(q,self.sensedPosition)/dt
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
            self.pidIntegralError += self._positionDifference(self.commandedPosition,q)*dt
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
            raise RuntimeError("Invalid control mode, joint {}?".format(self.name))

    def getCommand(self,commandType):
        if self.controlMode is None:
            #getting a command for a joint that hasn't had any commands sent yet
            if self.commandedPosition is None:
                self.commandedPosition = self.sensedPosition
                self.commandedVelocity = 0
                if self.sensedPosition is None:
                    raise ValueError("Trying to get a command for joint {} on the first timestep before sensors have arrived".format(self.name))
                self.controlMode = 'p'
        if commandType == 'pwc':
            assert self.trajectoryTimes is not None,"Can't get piecewise cubic trajectory except in piecewise cubic mode"
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
                #construct interpolant... should we do it in 1 time step or stretch it out?
                if self.commandedVelocity == 0:
                    return [0],[self.commandedPosition]
                dt = self.positionDifference(self.commandedPosition,self.lastCommandedPosition)/self.commandedVelocity
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
                    raise RuntimeError("Can't emulate PID control for joint {} using torque control, no gains are set".format(self.name))
                qdes,vdes,tdes = self.pidCmd
                kp,ki,kd = self.pidGains
                t_pid = kp*self._positionDifference(qdes,self.sensedPosition) + kd*(vdes-self.sensedVelocity) + ki*self.pidIntegralError + tdes
                #if abs(self.pidIntegralError[i]*ki) > tmax:
                #cap integral error to prevent wind-up
                return t_pid,self.commandTTL
            else:
                assert self.controlMode == 't',"Can't emulate torque control with any command type except for PID and torque control"
                return self.commandedTorque,self.commandTTL
        elif commandType == 'p' or commandType == 'm':
            return self.commandedPosition,
        elif commandType == 'v':
            assert self.controlMode != 'pid',"Can't emulate PID control mode with velocity control"
            if self.controlMode == 'v':
                return self.commandedVelocity,self.commandTTL
            else:
                #TODO: make these tunable per-joint?
                kFeedforward = 0.8
                kTrack = 1.0
                vel = kFeedforward*self.commandedVelocity + kTrack*self._positionDifference(self.commandedPosition,self.sensedPosition)
                return vel,self.commandTTL

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
            dp = self._positionDifference(self.trajectoryMilestones[i+1],self.trajectoryMilestones[i])
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
            x2[0] = x1[0] + self._positionDifference(x2[0],x1[0])  #handle continuous rotation joints
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

    def solveIK(self,xparams,frame='world'):
        if frame != 'world':
            raise NotImplementedError("Can only handle world frame, for now")
        qorig = self.robot.getConfig()
        if not self.active:
            self.driver.start(qorig,self.indices[-1],endEffectorPositions=self.toolCoordinates)
        goal = self.driver.ikGoals[0]
        if len(xparams) == 3:
            #point-to-point constraint
            goal.setFixedPosConstraint(self.toolCoordinates,xparams)
            goal.setFreeRotConstraint()
        elif len(xparams) == 2:
            if len(xparams[0]) != 9 or len(xparams[1]) != 3:
                raise ValueError("Invalid IK parameters, must be a point or se3 element")
            goal.setFixedPosConstraint(self.toolCoordinates,xparams[1])
            goal.setFixedRotConstraint(xparams[0]);
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

    def setCartesianVelocity(self,qcur,dxparams,ttl,frame='world'):
        assert len(qcur) == self.robot.numDrivers(),"Invalid length of current configuration: {}!={}".format(len(qcur),self.robot.numDrivers())
        assert ttl is None or isinstance(ttl,(int,float)),"Invalid value for ttl: {}".format(ttl)
        if frame!='world':
            raise NotImplementedError("Can only handle world frame, for now")
        for i,v in enumerate(qcur):
            self.robot.driver(i).setValue(v)
        qcur = self.robot.getConfig()

        if not self.active:
            self.driver.start(qcur,self.indices[-1],endEffectorPositions=self.toolCoordinates)
            self.active = True

        if self.t is None:
            self.endDriveTime = ttl
        elif ttl is not None:
            self.endDriveTime = self.t + ttl
        else:
            self.endDriveTime = None
        if len(dxparams) == 2:
            if len(dxparams[0]) != 3 or len(dxparams[1]) != 3:
                raise ValueError("Invalid IK parameters, must be a 3 vector or a pair of angular velocity / velocity vectors")
            self.driveCommand = dxparams
        else:
            if len(dxparams) != 3:
                raise ValueError("Invalid IK parameters, must be a 3 vector or a pair of angular velocity / velocity vectors")
            self.driveCommand = None,dxparams

    def update(self,qcur,t,dt):
        if self.t is None:
            if self.endDriveTime is not None:
                self.endDriveTime = t + self.endDriveTime
        self.t = t
        if not self.active:
            return
        #print("CartesianEmulatorData update",t)
        if self.endDriveTime is not None and t > self.endDriveTime:
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

    def cartesianPosition(self,q,frame='world'):
        assert len(q) == self.robot.numDrivers()
        if frame == 'tool':
            return se3.identity()
        elif frame == 'end effector':
            return (so3.identity(),self.toolCoordinates)
        model = self.robot
        for i,v in enumerate(q):
            model.driver(i).setValue(v)
        model.setConfig(model.getConfig())
        T = self.eeLink.getTransform()
        t = T.apply(self.toolCoordinates)
        Ttool_world = (T[0],t)
        if frame == 'world':
            return Ttool_world
        elif frame == 'base':
            baseLink = self.robot.link(self.indices[0])
            return se3.apply(se3.inv(baseLink.getTransform()),Ttool_world)
        else:
            raise ValueError("Invalid frame specified")
        
    def cartesianVelocity(self,q,dq,frame='world'):
        assert len(q) == self.robot.numDrivers()
        assert len(dq) == self.robot.numDrivers()
        assert len(q) == self.robot.numDrivers()
        model = self.robot
        for i,v in enumerate(q):
            model.driver(i).setValue(v)
            if frame == 'base' and i < self.indices[0]:
                model.driver(i).setVelocity(0)  #get relative velocity to base
            else:
                model.driver(i).setVelocity(dq[i])
        model.setConfig(model.getConfig())
        local = self.toolCoordinates
        v = self.eeLink.getPointVelocity(local)
        w = self.eeLink.getAngularVelocity()
        if frame == 'world':
            return w,v
        elif frame == 'base':
            baseLink = self.robot.link(self.indices[0])
            Tbase = baseLink.getTransform()
            Rworld_base = so3.inv(Tbase[0])
            return (so3.apply(Rworld_base,w),so3.apply(Rworld_base,v))
        elif frame == 'end effector' or frame == 'tool':
            eeLink = self.eeLink
            Tee = eeLink.getTransform()
            Rworld_ee = so3.inv(Tee[0])
            return (so3.apply(Rworld_ee,w),so3.apply(Rworld_ee,v))
        else:
            raise ValueError("Invalid frame specified")

    def cartesianForce(self,q,t,frame='world'):
        assert len(q) == self.robot.numDrivers()
        model = self.robot
        for i,v in enumerate(q):
            model.driver(i).setValue(v)
        model.setConfig(model.getConfig())
        local = self.toolCoordinates
        J = self.eeLink.getJacobian(local)
        wrench = [vectorops.dot(Jrow,t) for Jrow in J]
        torque,force = wrench[:3],wrench[3:]
        if frame == 'world':
            return (torque,force)
        elif frame == 'base':
            baseLink = self.robot.link(self.indices[0])
            Tbase = baseLink.getTransform()
            Rworld_base = so3.inv(Tbase[0])
            return (so3.apply(Rworld_base,torque),so3.apply(Rworld_base,force))
        elif frame == 'end effector' or frame == 'tool':
            eeLink = self.eeLink
            Tee = eeLink.getTransform()
            Rworld_ee = so3.inv(Tee[0])
            return (so3.apply(Rworld_ee,torque),so3.apply(Rworld_ee,force))
        raise ValueError("Invalid frame specified")


class _RobotInterfaceEmulatorData:
    def __init__(self,nd,klamptModel):
        self.klamptModel = klamptModel
        self.curClock = None
        self.lastClock = None
        self.dt = None
        self.jointData = [_JointInterfaceEmulatorData('Joint '+str(i)) for i in range(nd)]
        if klamptModel is not None:
            #find any continuous rotation joints
            for i in range(nd):
                d = klamptModel.driver(i)
                if len(d.getAffectedLinks())==1:
                    link = d.getAffectedLinks()[0]
                    if klamptModel.getJointType(link)=='spin':
                        #print("_RobotInterfaceEmulatorData: Interpreting joint",i," as a spin joint")
                        self.jointData[i].continuousRotation = True
        self.cartesianInterfaces = dict()
        self.commandSent = False

    def update(self,t,q,v):
        """Advances the interface"""
        self.lastClock = self.curClock
        self.curClock = t
        self.dt = t - self.lastClock
        qcmd = q
        try:
            qcmd = self.getCommand('p')[0]
        except Exception:
            pass
        for inds,c in self.cartesianInterfaces.items():
            qdes = c.update(qcmd,self.curClock,self.dt)
            if qdes is not None:
                assert len(qdes) == len(c.indices)
                self.commandSent = False
                for i,x in zip(inds,qdes):
                    self.jointData[i].promote('p')
                    self.jointData[i].commandedVelocity = (x-self.jointData[i].commandedPosition)/self.dt
                    self.jointData[i].commandedPosition = x
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
            nonuniform = False
            for times,positions in res:
                if unifiedTimes is None:
                    unifiedTimes = times
                elif unifiedTimes != times:
                    nonuniform = True
                    unifiedTimes = list(sorted(set(unifiedTimes)|set(times)))
            unifiedMilestones = []
            joint_trajs = []
            if nonuniform:
                for traj in res:
                    remeshed = Trajectory(traj[0],[[v] for v in traj[1]]).remesh(unifiedTimes)[0]
                    joint_trajs.append([m[0] for m in remeshed.milestones])
                    assert len(joint_trajs[-1])==len(unifiedTimes)
            else:
                joint_trajs = [traj[1] for traj in res]
                assert all(len(traj)==len(unifiedTimes) for traj in joint_trajs)
            for i in range(len(unifiedTimes)):
                unifiedMilestones.append([traj[i] for traj in joint_trajs])
            t0 = self.curClock-self.dt if self.curClock is not None else 0
            futureTimes = []
            futureMilestones = []
            for i,t in enumerate(unifiedTimes):
                if t > t0:
                    futureTimes.append(t-t0)
                    futureMilestones.append(unifiedMilestones[i])
            if len(futureTimes)==0:
                warnings.warn("getCommand is returning empty command because no times are after current time???")
            return futureTimes,futureMilestones
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
            futureTimes = []
            futureMilestones = []
            futureVelocities = []
            for i,t in enumerate(unifiedTimes):
                if t > t0:
                    futureTimes.append(t-t0)
                    futureMilestones.append(unifiedMilestones[i])
                    futureVelocities.append(unifiedVelocities[i])
            if len(futureTimes)==0:
                warnings.warn("getCommand is returning empty command because no times are after current time???")
            return futureTimes,futureMilestones,futureVelocities
        return list(zip(*res))

    def promote(self,indices,controlType):
        """To accept commands of the given controlMode, switches over the
        current state to the controlMode"""
        self.commandSent = False
        if controlType == 'pwl':
            if any(self.jointData[i].controlMode == 'pwc' and len(self.jointData[i].trajectoryTimes) > 1 for i in indices):
                warnings.warn("RobotInterfaceCompleter: Warning, converting from piecewise cubic to piecewise linear trajectory")
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
            warnings.warn("moveToPosition is not available because base controller's klamptModel() is not implemented.")
            self.promote(indices,'p')
            for (i,v) in zip(indices,q):
                self.jointData[i].commandedPosition = v
        else:
            #move to command emulation using bounded velocity curve 
            qmin,qmax = model.getJointLimits()
            vmax = model.getVelocityLimits()
            amax = model.getAccelerationLimits()
            for i in range(model.numDrivers()):
                if model.driver(i).getType() == 'affine':
                    links = model.driver(i).getAffectedLinks()
                    scale,offset = model.driver(i).getAffineCoeffs()
                    for l,s in zip(links,scale):
                        if s < 0:
                            qmin[l],qmax[l] = qmax[l],qmin[l]
                            vmax[l] *= -1
                            amax[l] *= -1
            xmin = model.configToDrivers(qmin)
            xmax = model.configToDrivers(qmax)
            vmax = model.velocityToDrivers(vmax)
            amax = model.velocityToDrivers(amax)
            xmin = [xmin[i] for i in indices]
            xmax = [xmax[i] for i in indices]
            vmax = [vmax[i]*speed for i in indices]
            amax = [amax[i]*speed**2 for i in indices]
            assert all(v >= 0 for v in vmax)
            assert all(v >= 0 for v in amax)
            qcmd = [(self.jointData[i].commandedPosition if self.jointData[i].commandedPosition is not None else self.jointData[i].sensedPosition) for i in indices]
            dqcmd = [(self.jointData[i].commandedVelocity if self.jointData[i].commandedVelocity is not None else self.jointData[i].sensedVelocity) for i in indices]
            #handle continuous rotations...
            if any(self.jointData[j].continuousRotation for j in indices):
                q = q[:]
                for i,j in enumerate(indices):
                    if self.jointData[j].continuousRotation:
                        q[i] = qcmd[i] + self.jointData[j]._positionDifference(q[i],qcmd[i])
            for i in range(len(dqcmd)):
                if qcmd[i] < xmin[i]:
                    xmin[i] = qcmd[i]
                if qcmd[i] > xmax[i]:
                    xmax[i] = qcmd[i]
                    #TODO: warn if this is way out of bounds
                if abs(dqcmd[i]) > vmax[i]:
                    vmax[i] = dqcmd[i]
            try:
                ts,xs,vs = motionplanning.interpolateNDMinTime(qcmd,dqcmd,q,[0]*len(q),xmin,xmax,vmax,amax)
            except Exception as e:
                print("Couldn't solve for move to?")
                print(e)
                if "velocity" in str(e):
                    print("vcmd:",dqcmd,"vmax:",vmax)
                else:
                    print("qcmd",qcmd,"q:",q)
                    print("qmin:",xmin,"qmax:",xmax)
                    print("vcmd:",dqcmd,"vmax:",vmax)
                    print("amax:",amax)
                raise
            ts,xs,vs = motionplanning.combineNDCubic(ts,xs,vs)
            self.setPiecewiseCubic(indices,ts,xs,vs,True)

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
            self.jointData[i].commandedTorque = v
            self.jointData[i].commandTTL = ttl

    def setPID(self,indices,q,dq,t):
        if t is None:
            t = [0.0]*len(q)
        self.promote(indices,'pid')
        for i,qi,dqi,ti in zip(indices,q,dq,t):
            self.jointData[i].pidCmd = (qi,dqi,ti)

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

    def setCartesianPosition(self,xparams,frame,indices):
        try:
            c = self.cartesianInterfaces[tuple(indices)]
        except KeyError:
            c = _CartesianEmulatorData(self.klamptModel,indices)
            self.cartesianInterfaces[tuple(indices)] = c
        for i,j in enumerate(self.jointData):
            self.klamptModel.driver(i).setValue(j.sensedPosition if j.commandedPosition is None else j.commandedPosition)
        c.active = False
        qKlamptNext = c.solveIK(xparams,frame)
        self.klamptModel.setConfig(qKlamptNext)
        self.setPosition(indices,[self.klamptModel.driver(i).getValue() for i in indices])

    def moveToCartesianPosition(self,xparams,speed,frame,indices):
        try:
            c = self.cartesianInterfaces[tuple(indices)]
        except KeyError:
            c = _CartesianEmulatorData(self.klamptModel,indices)
            self.cartesianInterfaces[tuple(indices)] = c
        for i,j in enumerate(self.jointData):
            self.klamptModel.driver(i).setValue(j.sensedPosition if j.commandedPosition is None else j.commandedPosition)
        c.active = False
        qKlamptNext = c.solveIK(xparams,frame)
        self.klamptModel.setConfig(qKlamptNext)
        self.moveToPosition(indices,[self.klamptModel.driver(i).getValue() for i in indices],speed)

    def setCartesianVelocity(self,dxparams,ttl,frame,indices):
        try:
            c = self.cartesianInterfaces[tuple(indices)]
        except KeyError:
            c = _CartesianEmulatorData(self.klamptModel,indices)
            self.cartesianInterfaces[tuple(indices)] = c
        qstart = [j.sensedPosition if j.commandedPosition is None else j.commandedPosition for j in self.jointData]
        c.setCartesianVelocity(qstart,dxparams,ttl,frame)
        for i in indices:
            self.jointData[i].externalController = c

    def setCartesianForce(self,fparams,ttl,frame,indices):
        try:
            c = self.cartesianInterfaces[tuple(indices)]
        except KeyError:
            c = _CartesianEmulatorData(self.klamptModel,indices)
            self.cartesianInterfaces[tuple(indices)] = c
        raise NotImplementedError("TODO: implement cartesian force?")
        qstart = [j.sensedPosition if j.commandedPosition is None else j.commandedPosition for j in self.jointData]
        c.setCartesianForce(qstart,fparams,ttl,frame)

    def cartesianPosition(self,q,frame,indices):
        assert len(q) == len(self.jointData),"cartesianPosition: must use full-body configuration"
        try:
            c = self.cartesianInterfaces[tuple(indices)]
            return c.cartesianPosition(q,frame)
        except KeyError:
            raise ValueError("Invalid Cartesian index set for emulator: no command currently set")

    def cartesianVelocity(self,q,dq,frame,indices):
        assert len(q) == len(self.jointData),"cartesianVelocity: must use full-body configuration"
        assert len(dq) == len(self.jointData),"cartesianVelocity: must use full-body configuration"
        try:
            c = self.cartesianInterfaces[tuple(indices)]
            return c.cartesianVelocity(q,dq,frame)
        except KeyError:
            raise ValueError("Invalid Cartesian index set for emulator: no command currently set")

    def cartesianForce(self,q,t,frame,indices):
        assert len(q) == len(self.jointData),"cartesianForce: must use full-body configuration"
        assert len(t) == len(self.jointData),"cartesianForce: must use full-body configuration"
        try:
            c = self.cartesianInterfaces[tuple(indices)]
            return c.cartesianForce(q,t,frame)
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
        assert isinstance(interface,RobotInterfaceCompleter)
        assert not isinstance(interface._base,RobotInterfaceCompleter),"Can't do nested sub-interfaces"
        assert interface._emulator.curClock is not None,"Interface needs to be initialized before accessing a part"
        self._parent = interface
        self._base_part_interface = None
        try:
            self._base_part_interface = interface._base.partInterface(part,joint_idx)
        except Exception:
            pass            
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
        #self._indices = interface._base.indices(part,joint_idx)
        self._indices = interface.indices(part,joint_idx)
        self._parts = dict()
        self._parts[None] = self._indices
        self._has = interface._has
        self._emulator = interface._emulator

    def __str__(self):
        return str(self._parent)+'['+self.properties['name']+']'

    def status(self):
        if self._base_part_interface is not None:
            return self._base_part_interface.status()
        return RobotInterfaceCompleter.status(self)

    def estop(self):
        #if self._base_part_interface is not None:
        #    self._base_part_interface.estop()
        #Do we want the whole robot to stop, or just the part?
        return RobotInterfaceCompleter.estop(self)

    def softStop(self):
        """Calls a software E-stop on the robot (braking as quickly as
        possible).  Default implementation stops robot at current position; a 
        better solution would slow the robot down.
        """
        self.setPosition(self.commandedPosition())

    def reset(self):
        if self._base_part_interface is not None:
            return self._base_part_interface.reset()
        return RobotInterfaceCompleter.reset(self)

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

