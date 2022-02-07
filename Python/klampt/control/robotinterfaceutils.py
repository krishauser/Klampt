"""Contains utilities for the Klampt Robot Interface Layer. 

The :class:`RobotInterfaceCompleter` class is extremely widely used to
standardize the capabilities of
:class:`~.robotinterface.RobotInterfaceBase` to handle
advanced functions. For example, if you have a position controlled robot
(e.g., an Arduino-controlled set of motors) and want to do more
sophisticated work with it, you can create a completer and get access to
motion queues, Cartesian control, collision filters, etc.

You can also assemble unified interfaces to robots composed of separate
independent parts via :class:`OmniRobotInterface`. For example, if you
have a robot arm + gripper, each working on separate physical interfaces,
creating an OmniRobotInterface will let you simultaneously control both.

Helper classes:

 * :class:`StepContext`: uses a Python context to guard beginStep()/endStep()
   pairs.
 * :class:`ThreadedRobotInterface`: adapts a synchronous interface to an
   asynchronous one via threading.
 * :class:`MultiprocessingRobotInterface`: adapts a synchronous interface to an
   asynchronous one via multiprocessing.
 * :class:`RobotInterfaceRecorder`: records or plays back commands to a
   RIL object.

Hints on wrapping to get the best performance:

 * RobotInterfaceCompleter sends immediate state queries and commands to the
   base interface, so it is more efficient than OmniRobotInterface when the
   base interface implements the given commands and queries. 
 * OmniRobotInterface makes all state queries at the start of the step and
   delays sending the actual commands at the end of the step. 
 * For asynchronous or client-server interfaces, the user has the choice of
   completing the base interface then wrapping it (e.g.,
   ``ThreadedRobotInterface(RobotInterfaceCompleter(MyInterface)))``, or
   wrapping the base interface then completing it (e.g., 
   ``RobotInterfaceCompleter(ThreadedRobotInterface(MyInterface)))``. In the
   former case, more information is transmitted over the network, but the
   caller thread does less work.  In the latter case, less information is
   transmitted, but the caller thread must do more work (including, for
   example, Cartesian control and collision detection.)
 * Collision detection is usually the most computationally complex step.
   Configure your klamptModel's geometries to be as simple as possible and
   set the trajectory collision checking to... TODO

This module also provides :func:`make_from_file` which can either take a
.py / .pyc file, or a module string naming a Python file with a `make`
method.

"""

"""

TODO: Multi-part controllers that run at different rates

TODO: figure out how multi-part FILTERS will work
    - should command filters be checked "caller side" (i.e., right when command is issued)
        - pros: less likelihood to induce timing overruns inside server, easier multithreaded / distributed implementation because the filter doesn't need to be transferred
        - cons: less information available because commands have not been collected
    - should filters be set per part and only affect commands / states on that part, or should they propagate? 
      Resolved: only affect the part.  It is up to user to create multi-part filters on parent.
    - should filters check a whole commanded trajectory, or just run in real time on the current command?


TODO: figure out how emulated gravity compensation will work -- need full-body sensed configuration


Complex example:

* robot: OmniRobotInterface
    * base - MobileBaseInterface
    * left - OmniRobotInterface         #TODO: figure how nested multi-robot interfaces should work
        - arm - ArmInterface1
        - gripper - GripperInterface1
    * right - GripperArmInterface
        - arm: standard part
        - gripper - standard part
    * head - HeadInterface

left_arm = robot.partInterface("left").partInterface("arm")    # wouldn't work, only one layer of parts
right_arm = robot.partInterface("right").partInterface("arm")  # wouldn't work, only one layer of parts
left_arm.configFromKlampt()                                    # world work
left_arm.moveToCartesian(T_target,frame='base')                # works, frame='base' is accessible in part interface
left_arm.moveToCartesian(T_target,frame='world')               # works, frame='world' is accessible in state thanks to emulator
robot.setFilter("self_collision",SelfCollisionFilter(robot.klamptModel()))  #does not work, filters not working properly
right_arm.moveToPosition(colliding_right_arm_config)           # would not raise collision, left_arm and right_arm are not filtered
left_arm.moveToPosition(colliding_left_arm_config)
right_arm.moveToPosition(colliding_right_arm_config)           # TODO: should this raise a collision error? Or should it wait
left_arm.moveToPosition(non_colliding_left_arm_config)

OmniRobotInterface:
    'part' means 'a legitimate part corresponding to a physical RIL'
    'subpart' means some collection of indices corresponding to a whole

    Parts must have disjoint set of indices whose union is the whole robot.
    The klamptModel for a part/sub-part is a SubRobotModel of the whole robot.

    beginStep():
        for all parts:
            part.beginStep()
            gather state / status information into unified state / status
            emulate parts of the state that are unavailable
        for all parts / subparts:
            distribute unified state
            emulate base transforms and cartesian transforms 
        store a prevstatus structure
        run filters affected by state    -- TODO: do we go through tree of interfaces to find part and sub-part filters?
    whole, part, and sub-part queries:
        retrieve items from distributed state / settings
        for cartesian queries w.r.t. frame='world', klamptModel is set to whole robot's sensed, commanded, or destination position
    whole, part, and sub-part commands:
        status change updates immediately
        run filters affected by command    -- TODO: how do we run filter for a part or a sub-part?
        command goes into queue
        internally, can do some immediate changes of unified state e.g., for commanded position and joint control mode
    endStep():
        distribute commands to emulators
        for emulated joints and multi-axis controllers:
            update emulated joint state
            update multi-axis controller state
        distribute emulator commands to parts
        if any item in status was changed by user (status.x!=prevstatus.x), distribute to relevant part(s)
        for all parts:
            part.endStep()

Multi-part Cartesian commands and queries w.r.t. base/world work because each
interface is aware of the parent and its Cartesian transform.

"""


from .robotinterface import RobotInterfaceBase
from ..math import vectorops,so2,so3,se3,spline
from ..plan import motionplanning
from ..model.trajectory import Trajectory,HermiteTrajectory
from ..model.robotinfo import RobotInfo
from ..robotsim import WorldModel,RobotModel
from ..model.subrobot import SubRobotModel
from .cartesian_drive import CartesianDriveSolver
from . import blocks
from .utils import Promise
import bisect
import math
import warnings
import time
from functools import wraps
from ..model.typing import Vector,Vector3,RigidTransform
from typing import Dict,Any,Union,Optional,List,Tuple,Sequence,Callable,TextIO
import copy
import numpy as np

PART_NAME_SEPARATOR = '__'

SETTING_COMMAND_METHODS = ['reset','enableSensor','setPIDGains','setToolCoordinates','setGravityCompensation']
MOTION_COMMAND_METHODS = ['estop','softStop',
                          'setPosition','setVelocity','setTorque','setPID',
                          'moveToPosition','setPiecewiseLinear','setPiecewiseCubic',
                          'setCartesianPosition','moveToCartesianPosition','setCartesianVelocity','setCartesianForce',
                          'setControlMode']
STRUCTURE_QUERY_METHODS = ['numJoints','parts','controlRate','jointName','sensors','hasSensor','klamptModel']
SETTING_QUERY_METHODS = ['status','clock','getPIDGains','getToolCoordinates','getGravityCompensation']
SENSOR_QUERY_METHODS = ['enabledSensors','sensorMeasurements','sensorUpdateTime']
MOTION_QUERY_METHODS = ['isMoving',
                        'sensedPosition','sensedVelocity','sensedTorque',
                        'commandedPosition','commandedVelocity','commandedTorque',
                        'destinationPosition','destinationVelocity','destinationTime',
                        'queuedTrajectory',
                        'sensedCartesianPosition','sensedCartesianVelocity','sensedCartesianForce',
                        'commandedCartesianPosition','commandedCartesianVelocity','commandedCartesianForce',
                        'destinationCartesianPosition','destinationCartesianVelocity',
                        'queuedCartesianTrajectory']
UTILITY_METHODS = ['indices','cartesianPosition','cartesianVelocity','cartesianForce',
                    'partToRobotConfig','robotToPartConfig',
                    'configFromKlampt','velocityFromKlampt','configToKlampt','velocityToKlampt']
COMMAND_METHODS = SETTING_COMMAND_METHODS+MOTION_COMMAND_METHODS
QUERY_METHODS = STRUCTURE_QUERY_METHODS+SETTING_QUERY_METHODS+SENSOR_QUERY_METHODS+MOTION_QUERY_METHODS
CUSTOM_METHODS = ['functionCall','setSetting','getSetting','setFilter','stateValue']
ALL_METHODS = COMMAND_METHODS + QUERY_METHODS + CUSTOM_METHODS


class StepContext:
    """Makes it easier to handle exceptions in RIL control code. Flags allow
    you to handle exceptions manually, print exceptions, or silence them
    completely.

    Args:
        interface (RobotInterfaceBase): the interface to step.
        ignore (bool): if False, raises the exception.  Otherwise, prints
            an error message.
        silent (bool): if True, doesn't print error messages if ignore=True.

    Members:
        numExceptions (int): counts the # of exceptions raised

    Usage::

        interface = MyRobotInterface()
        interface.initialize()
        looper = TimedLooper(1.0/interface.controlRate())
        #declare context outside of loop so numExceptions can be counted
        context = StepContext(interface,ignore=True) 
        while looper:
            with context:
                interface.setCartesianPosition([0.4])  #incorrect arguments, exceptions will be printed
            if context.numExceptions > 10:
                print("Breaking loop, too many exceptions.")
                looper.stop()

    """
    def __init__(self, interface : RobotInterfaceBase, ignore=False, silent=False):
        self.interface = interface
        self.ignore = ignore
        self.silent = silent
        self.numExceptions = 0
        self._beginStep_exception = None
        self._beginStep_traceback = None
    
    def __enter__(self):
        try:
            self.interface.beginStep()
        except Exception as e:
            if self.ignore:
                import traceback
                self._beginStep_exception = e
                self._beginStep_traceback = traceback.extract_tb()
                self.numExceptions += 1
            else:
                raise
        return self
    
    def __exit__(self, type, value, tb):
        if type is not None:
            self.numExceptions += 1
        if self.ignore and not self.silent:
            if self._beginStep_exception is not None:
                print("Exception during {}.beginStep()".format(str(self.interface)))
                print("  Value:", self._beginStep_exception)
                print("  Traceback:", self._beginStep_traceback)
                self._beginStep_exception = None
                self._beginStep_traceback = None
            elif type is not None:
                print("Exception between {}.beginStep/endStep()".format(str(self.interface)))
                print("  Type:", type)
                print("  Value:", value)
                print("  Traceback:", tb)
        try:
            self.interface.endStep()
        except Exception as e:
            self.numExceptions += 1
            if self.ignore:
                import traceback
                print("Exception during {}.endStep()".format(str(self.interface)))
                print("  Value:", e)
                print("  Traceback:", traceback.extract_tb())
            else:
                raise
        if self.ignore:
            return True


def make_from_file(fn : str, robotModel : RobotModel, *args,**kwargs) -> RobotInterfaceBase:
    """Create a RobotInterfaceBase from a Python file or module containing the
    ``make()`` function.

    args and kwargs will be passed to ``make``.

    Example::
        iface = make_from_file('klampt.control.simrobotcontroller', robot)
    
    """
    import importlib
    if fn.endswith('py') or fn.endswith('pyc'):
        import os
        import sys
        path,base = os.path.split(fn)
        mod_name,file_ext = os.path.splitext(base)
        sys.path.append(os.path.abspath(path))
        mod = importlib.import_module(mod_name,base)
        sys.path.pop(-1)
    else:
        mod = importlib.import_module(fn)
    try:
        maker = mod.make
    except AttributeError:
        print("Module",mod.__name__,"must have a make() method")
        raise
    return maker(robotModel,*args,**kwargs)
    


class _Struct:
    """Base class for stateful items. 
    
    If the struct has a member ``children`` it is assumed to be hierarchical.

    Fields in class variable DO_NOT_SERIALIZE are not serialized
    """
    DO_NOT_SERIALIZE = []

    def __init__(self,rhs=None):
        if rhs is not None:  #copy constructor
            for k in self.__dict__:
                if k=='children' and rhs.children is not None:
                    self.children = dict()
                    for (name,value) in rhs.children.items():
                        assert isinstance(value,_Struct)
                        self.children[name] = self.__class__(value)
                else:
                    setattr(self,k,copy.copy(getattr(rhs,k)))

    def to_json(self):
        def _to_json(obj,name):
            if isinstance(obj,np.ndarray):
                return obj.tolist()
            elif isinstance(obj,_Struct):
                return obj.to_json()
            elif isinstance(obj,dict):
                assert all(isinstance(k,str) for k in obj),"All dictionary keys of {} must be str".format(name)
                return dict((k,_to_json(v,name)) for (k,v) in obj.items())
            elif isinstance(obj,(tuple,list)):
                return [_to_json(v,name) for v in obj]
            return obj

        res = dict()
        for (k,v) in self.__dict__.items():
            if v is not None and k not in self.__class__.DO_NOT_SERIALIZE:
                res[k] = _to_json(v,k)
        return res
    
    def from_json(self,jsonobj):
        for k in self.__dict__:
            if k not in self.__class__.DO_NOT_SERIALIZE:
                self.__dict__[k] = None
        for (k,v) in jsonobj.items():
            self.__dict__[k] = v
        if 'children' in jsonobj and jsonobj['children'] is not None:
            for (k,v) in self.children.items():
                self.children[k] = self.__class__()
                self.children[k].from_json(v)

    def complete(self) -> bool:
        """Returns true if all items are present in state"""
        if not all(v is not None for k,v in self.__dict__.items()):
            return False
        if hasattr(self,'children') and self.children is not None:
            for (k,v) in self.children.items():
                if not v.complete():
                    return False
        return True
    
    def update(self,rhs) -> None:
        """Adds any non-None items from rhs to self."""
        for k,v in rhs.__dict__.items():
            if v is None: continue
            if k == 'children':
                if self.children is None:
                    self.children = dict()
                for ck,cv in v.items():
                    if ck not in self.children:
                        self.children[ck] = cv
                    else:
                        self.children[ck].update(cv)
            else:
                self.__dict__[k] = v


class _RobotInterfaceStructure(_Struct):
    """Constant information over the life of an interface."""
    DO_NOT_SERIALIZE = ["robotModel"]

    def __init__(self):
        self.controlRate = None            #type: float
        self.numJoints = None              #type: int
        self.jointNames = None             #type: List[str]
        self.parts = None                  #type: Dict[Union[None,int],List[int]]
        self.sensors = None                #type: List[str]
        self.klamptModel = None            #type: Union[RobotModel,SubRobotModel]
    
    def to_json(self):
        #can't serialize None
        if self.parts is not None and None in self.parts:
            ROOT_PART_TOKEN = '__'
            rootpart = self.parts[None]
            self.parts[ROOT_PART_TOKEN] = rootpart
            del self.parts[None]
            res = _Struct.to_json(self)
            del self.parts[ROOT_PART_TOKEN]
            self.parts[None] = rootpart
            return res
        return _Struct.to_json(self)
    
    def from_json(self,jsonobj):
        _Struct.from_json(self,jsonobj)
        ROOT_PART_TOKEN = '__'
        if self.parts is not None and ROOT_PART_TOKEN in self.parts:
            rootpart = self.parts[ROOT_PART_TOKEN]
            self.parts[None] = rootpart
            del self.parts[ROOT_PART_TOKEN]


class _RobotInterfacePartSettings(_Struct):
    """Per-part settings structure, with nesting.

    Includes settings for Cartesian control, gravity compensation,
    sensors, and custom settings.
    """
    def __init__(self, rhs : '_RobotInterfacePartSettings' = None):
        self.toolCoordinates = None         #type: Vector3
        self.gravity = None                 #type: Vector3
        self.load = None                    #type: float
        self.load_com = None                #type: Vector3
        self.settings = None                #type: Dict[str,Any]
        self.sensorEnabled = None           #type: Dict[str,bool]
        self.children = None                #type: Dict[str,_RobotInterfacePartSettings]
        _Struct.__init__(self,rhs)
    

class _RobotInterfaceSettings(_RobotInterfacePartSettings):
    """Global settings structure, treated as read/write.  Inherits per-part
    settings, adds PID gains.
    """
    def __init__(self):
        _RobotInterfacePartSettings.__init__(self)
        self.kP = None                      #type: Vector
        self.kI = None                      #type: Vector
        self.kD = None                      #type: Vector


class _RobotInterfaceCartesianState(_Struct):
    """Cartesian control state, represented as coordinates in the
    part's base frame. 

    If this is a sub-part interface, base elements should be given.
    These map from the part's base to the world frame.
    """
    def __init__(self):
        self.sensedPosition = None          #type: RigidTransform
        self.sensedVelocity = None          #type: Tuple[Vector3,Vector3]
        self.sensedForce = None             #type: Tuple[Vector3,Vector3]
        self.commandedPosition = None       #type: RigidTransform
        self.commandedVelocity = None       #type: Tuple[Vector3,Vector3]
        self.commandedForce = None          #type: Tuple[Vector3,Vector3]
        self.destinationPosition = None     #type: RigidTransform
        self.destinationVelocity = None     #type: Tuple[Vector3,Vector3]
        self.baseSensedPosition = None      #type: RigidTransform
        self.baseSensedVelocity = None      #type: Tuple[Vector3,Vector3]
        self.baseCommandedPosition = None   #type: RigidTransform
        self.baseCommandedVelocity = None   #type: Tuple[Vector3,Vector3]
        self.baseDestinationPosition = None #type: RigidTransform
        self.baseDestinationVelocity = None #type: Tuple[Vector3,Vector3]


class _RobotInterfacePartState(_Struct):
    """State for individual parts and sub-parts"""
    def __init__(self, rhs : '_RobotInterfacePartState' = None):
        self.cartesianState = None          #type: _RobotInterfaceCartesianState 
        self.sensorState = None             #type: Dict[str,Tuple[float,Vector]]
        self.children = None                #type: Dict[str,_RobotInterfacePartState]
        _Struct.__init__(self,rhs)

    def from_json(self,jsonobj):
        _Struct.from_json(self,jsonobj)
        if self.cartesianState is not None:
            cartesian_obj = self.cartesianState
            self.cartesianState = _RobotInterfaceCartesianState()
            self.cartesianState.from_json(cartesian_obj)


class _RobotInterfaceState(_RobotInterfacePartState):
    """Internal state of the robot, treated as read-only.  Inherits per-part
    state, adds joint-wise values."""
    def __init__(self):
        _RobotInterfacePartState.__init__(self)
        self.clock = None                   #type: float
        self.rate = None                    #type: float
        self.status = None                  #type: str
        self.isMoving = None                #type: bool
        self.sensedPosition = None          #type: Vector
        self.sensedVelocity = None          #type: Vector
        self.sensedTorque = None            #type: Vector
        self.commandedPosition = None       #type: Vector
        self.commandedVelocity = None       #type: Vector
        self.commandedTorque = None         #type: Vector
        self.destinationPosition = None     #type: Vector
        self.destinationVelocity = None     #type: Vector
        self.destinationTime = None         #type: float
        self.queuedTrajectory = None        #type: List[Tuple[List[float],List[float],List[float]]]
        self.jointControlModes = None       #type: List[str]
        self.stateValue = {}                #type: Dict[str,Vector]


class _RobotInterfaceCommand(_Struct):
    """Command from the user to the robot."""
    DO_NOT_SERIALIZE = ["promise"]

    def __init__(self):
        self.time = None                    #type: float
        self.indices = None                 #type: List[int]
        self.func = None                    #type: str
        self.args = None                    #type: Tuple[Any]
        self.promise = None                 #type: Promise


def _valid_vector(v,length=None) -> bool:
    if v is None: return False
    if length is not None and len(v) != length: return False
    if all(x is not None for x in v):
        if any(x is None for x in v):
            print("WARNING: vector has mized values and None: {}".format(v))
            return False
        return True
    return False

def _gather_state_var(res,state,state_inds,attr,n):
    stateattr = getattr(state,attr)
    if stateattr is not None:
        resattr = getattr(res,attr)
        if resattr is None:
            resattr = [None]*n
            setattr(res,attr,resattr)
        for i,j in enumerate(state_inds):
            #assert stateattr[i] is not None,"Part reported vector of None for state attribute for {}".format(attr)
            resattr[j] = stateattr[i]

def _split_state_var(unified,state,state_inds,attr):
    unifiedattr = getattr(unified,attr)
    if unifiedattr is not None:
        state_vals = [None]*len(state_inds)
        for i,j in enumerate(state_inds):
            state_vals[i] = unifiedattr[j]
        if not all(v is None for v in state_vals):
            setattr(state,attr,state_vals)
        
def _gather_state(parts : Dict[str,Sequence[int]],
                  part_states : Dict[str,_RobotInterfaceState],
                  unified_state: _RobotInterfaceState) -> None:
    """Merges sub-parts' states into a macro state"""
    assert len(parts) == len(part_states)
    assert len(parts) > 0
    n = max(max(inds) for inds in parts.values())+1
    for (name,inds) in parts.items():
        if name is None: continue
        state = part_states[name]
        if state.clock is not None:
            if unified_state.clock is None:
                unified_state.clock = state.clock
        if unified_state.rate is not None:
            if unified_state.rate is None:
                unified_state.rate = state.rate
            else:
                unified_state.rate = max(state.rate,unified_state.rate)
        if state.status is not None:
            if unified_state.status is None:
                unified_state.status = state.status
            else:
                if state.status == 'ok':
                    pass
                elif unified_state.status == 'ok':
                    unified_state.status = state.status
                else:
                    unified_state.status = unified_state.status + ", " + state.status
        if state.isMoving is not None:
            if unified_state.isMoving is None:
                unified_state.isMoving = state.isMoving
            else:
                unified_state.isMoving = unified_state.isMoving or state.isMoving
        _gather_state_var(unified_state,state,inds,'sensedPosition',n)
        _gather_state_var(unified_state,state,inds,'sensedVelocity',n)
        _gather_state_var(unified_state,state,inds,'commandedPosition',n)
        _gather_state_var(unified_state,state,inds,'commandedVelocity',n)
        _gather_state_var(unified_state,state,inds,'commandedTorque',n)
        _gather_state_var(unified_state,state,inds,'destinationPosition',n)
        _gather_state_var(unified_state,state,inds,'destinationVelocity',n)

        if state.destinationTime is not None:
            if unified_state.destinationTime is None:
                unified_state.destinationTime = state.destinationTime
            else:
                unified_state.destinationTime = max(unified_state.destinationTime,state.destinationTime)
        if state.queuedTrajectory is not None:
            if unified_state.queuedTrajectory is None:
                unified_state.queuedTrajectory = [(None,None,None)]*n
            for i,j in enumerate(inds):
                unified_state.queuedTrajectory[j] = state.queuedTrajectory[i]
        
        _gather_state_var(unified_state,state,inds,'jointControlModes',n)
        for k,v in state.stateValue.items():
            if k not in unified_state.stateValue:
                unified_state.stateValue[k] = [None]*n
            for i,j in enumerate(inds):
                unified_state.stateValue[k][j] = v[i]

        if state.cartesianState is not None or state.sensorState is not None or state.children is not None:
            if unified_state.children is None:
                unified_state.children = dict()
            unified_state.children[name] = _RobotInterfacePartState(state)


def _split_state(unified_state: _RobotInterfaceState,
                 parts : Dict[str,List[int]]) -> Dict[str,_RobotInterfaceState]:
    """Splits a macro-state into a list of sub-parts' states"""
    res = dict()
    for (name,inds) in parts.items():
        if name is None: continue
        state = _RobotInterfaceState()
        res[name] = state
        state.clock = unified_state.clock
        state.rate = unified_state.rate
        state.status = unified_state.status
        state.isMoving = unified_state.isMoving
        _split_state_var(unified_state,state,inds,'sensedPosition')
        _split_state_var(unified_state,state,inds,'sensedPosition')
        _split_state_var(unified_state,state,inds,'sensedVelocity')
        _split_state_var(unified_state,state,inds,'commandedPosition')
        _split_state_var(unified_state,state,inds,'commandedVelocity')
        _split_state_var(unified_state,state,inds,'commandedTorque')
        _split_state_var(unified_state,state,inds,'destinationPosition')
        _split_state_var(unified_state,state,inds,'destinationVelocity')

        state.destinationTime = unified_state.destinationTime
        if unified_state.queuedTrajectory is not None:
            state.queuedTrajectory = [unified_state.queuedTrajectory[j] for j in inds]
        
        _split_state_var(unified_state,state,inds,'jointControlModes')
        for k,v in unified_state.stateValue.items():
            state.stateValue[k] = [v[j] for j in inds]
        
        if name in unified_state.children:
            partState = unified_state.children[name]
            state.sensorState = copy.copy(partState.sensorState)
            state.cartesianState = copy.copy(partState.cartesianState)
            state.children = copy.deepcopy(partState.children)
    return res

def _gather_settings(parts : Dict[str,List[int]],
                     part_settings : Dict[str,_RobotInterfaceSettings],
                     unified_settings : _RobotInterfaceSettings) -> None:
    if None in parts:
        assert len(part_settings)+1 == len(parts),"part settings isn't the right size? {} vs {}".format(list(part_settings.keys()),list(parts.keys()))
    else:
        assert len(part_settings) == len(parts),"part settings isn't the right size? {} vs {}".format(list(part_settings.keys()),list(parts.keys()))
    assert len(parts) > 0
    n = max(max(inds) for inds in parts.values())+1
    for (partname,inds) in parts.items():
        if partname is None: continue
        settings = part_settings[partname]
        _gather_state_var(unified_settings,settings,inds,'kP',n)
        _gather_state_var(unified_settings,settings,inds,'kI',n)
        _gather_state_var(unified_settings,settings,inds,'kD',n)
        if any(getattr(settings,n) is not None for n in ['toolCoordinates','gravity','settings','sensorEnabled','children']):
            if unified_settings.children is None:
                unified_settings.children = dict()
            unified_settings.children[partname] = _RobotInterfacePartSettings(settings)
            
def _split_settings(unified_settings : _RobotInterfaceSettings,
                    parts : Dict[str,List[int]]) -> Dict[str,_RobotInterfaceSettings]:
    res = dict()
    for (partname,inds) in parts.items():
        if partname is None: continue
        settings = _RobotInterfaceSettings()
        res[partname] = settings
        _split_state_var(unified_settings,settings,inds,'kP')
        _split_state_var(unified_settings,settings,inds,'kI')
        _split_state_var(unified_settings,settings,inds,'kD')
        if unified_settings.children is not None and partname in unified_settings.children:
            partsetting = unified_settings.children[partname]
            for n in ['toolCoordinates','gravity','load','load_com','settings','sensorEnabled','children']:
                setattr(settings,n,copy.copy(getattr(partsetting,n)))
    return res

def _gather_commands(part_commands : Sequence[Sequence[_RobotInterfaceCommand]],
                     part_indices : Sequence[Sequence[int]]) -> List[_RobotInterfaceCommand]:
    assert len(part_commands)==len(part_indices)
    res = dict()
    for (cmdqueue,inds) in zip(part_commands,part_indices):
        for command in cmdqueue:
            if command.indices is None: whole_inds = inds
            else: whole_inds = [inds[i] for i in command.indices]
            if command.func not in res:
                cmd = _RobotInterfaceCommand()
                cmd.time = command.time
                cmd.indices = whole_inds
                cmd.func = command.func
                cmd.args = [copy.copy(a) for a in command.args]
                res[command.func] = cmd
            else:
                #func already exists, now add to it
                cmd = res[command.func]
                cmd.time = max(cmd.time,command.time)
                if cmd.indices == whole_inds:
                    #more up-to-date version of same indices
                    cmd.args = [copy.copy(a) for a in command.args]
                else:
                    #merge
                    if cmd.indices is not None:
                        for i in cmd.indices:
                            if i in whole_inds:
                                raise NotImplementedError("TODO: command index clashes")
                    else:
                        cmd.indices = inds
                    for i,(a,a2) in enumerate(zip(cmd.args,command.args)):
                        if hasattr(a2,'__iter__') and len(a2)==len(inds):
                            assert len(a) == len(cmd.indices)
                        cmd.args[i].extend(a2)
                    cmd.indices += whole_inds
    return sorted(list(res.values()), key = lambda x:x.time)

def _update_from_part_settings(interface : RobotInterfaceBase, settings : _RobotInterfacePartSettings, delta_settings : _RobotInterfacePartSettings):
    if delta_settings.toolCoordinates is not None:
        if interface is not None:
            try:
                interface.setToolCoordinates(delta_settings.toolCoordinates)
            except NotImplementedError as e:
                return e
        settings.toolCoordinates = delta_settings.toolCoordinates
    if delta_settings.gravity is not None:
        assert delta_settings.load is not None
        assert delta_settings.load_com is not None
        if interface is not None:
            try:
                interface.setGravityCompensation(delta_settings.gravity,delta_settings.load,delta_settings.load_com)
            except NotImplementedError as e:
                return e
        settings.gravity = delta_settings.gravity
        settings.load = delta_settings.load
        settings.load_com = delta_settings.load_com
    if delta_settings.settings is not None:
        if settings.settings is None:
            settings.settings = dict()
        for k,val in delta_settings.settings.items():
            if k not in settings.settings or val != settings.settings[k]:
                if interface is not None:
                    try:
                        interface.setSetting(k,val)
                    except NotImplementedError as e:
                        return e
                settings.settings[k] = val
    if delta_settings.sensorEnabled is not None:
        if settings.sensorEnabled is None:
            settings.sensorEnabled = dict()
        for k,val in delta_settings.sensorEnabled.items():
            if k not in settings.sensorEnabled or val != settings.sensorEnabled[k]:
                if interface is not None:
                    interface.enableSensor(k,val)
                settings.sensorEnabled[k] = val

def _update_from_settings(interface : RobotInterfaceBase,
                          settings : _RobotInterfaceSettings,
                          delta_settings : _RobotInterfaceSettings):
    errors = []
    if delta_settings.kP is not None:
        assert delta_settings.kI is not None
        assert delta_settings.kD is not None
        #fill with prior settings
        if settings.kP is not None:
            for i,v in enumerate(delta_settings.kP):
                if v is not None: settings.kP[i] = v
            for i,v in enumerate(delta_settings.kI):
                if v is not None: settings.kI[i] = v
            for i,v in enumerate(delta_settings.kD):
                if v is not None: settings.kD[i] = v
        else:
            settings.kP = delta_settings.kP
            settings.kI = delta_settings.kI
            settings.kD = delta_settings.kD
        if any(v is None for v in settings.kP):
            raise ValueError("kP has a null value")
        if any(v is None for v in settings.kI):
            raise ValueError("kI has a null value")
        if any(v is None for v in settings.kD):
            raise ValueError("kD has a null value")
        if interface is not None:
            try:
                interface.setPIDGains(settings.kP,settings.kD,settings.kI)
            except NotImplementedError as e:
                errors.append(e)
    res = _update_from_part_settings(interface,settings,delta_settings)
    if res is not None:
        errors.append(res)
    return errors

def _update_parts_from_settings(part_interfaces : Dict[str,RobotInterfaceBase],
                                part_indices : Dict[str,List[int]],
                                settings : _RobotInterfaceSettings,
                                delta_settings : _RobotInterfaceSettings,
                                full_update = False):
    errors = []
    for part,iface in part_interfaces.items():
        if delta_settings.children is None or part not in delta_settings.children: continue
        assert part is not None,"Shouldn't have None as a partInterface"
        res = _update_from_part_settings(iface,settings.get(part,_RobotInterfacePartSettings()),delta_settings.children[part])
        errors += res
    if full_update:
        delta_sub_settings = _split_settings(delta_settings,[part_indices[part] for part in part_interfaces])
        sub_settings = _split_settings(settings,[part_indices[part] for part in part_interfaces])
        for ((part,iface),sub_setting,delta_sub_setting) in zip(part_interfaces.items(),sub_settings,delta_sub_settings):
            _update_from_settings(iface, sub_setting, delta_sub_setting)
    return errors


def _try_methods(obj, fn: Union[str,List[str]], args, fallback : Callable = None, cache : Dict = None):
    """Tries calling a method implemented in ``obj``, with potential
    alternatives.

    Returns the result of calling ``getattr(obj,fn)(*args)``, or if that raises a
    ``NotImplementedError``, returns ``fallback(*args)``.

    Also, fn and args can be lists. In this case, fallback is ignored, and
    instead each function and each argument is tried in turn.  The i'th arg
    can be a callable (e.g., a lambda function), which produces the argument
    tuple for the i'th function. 
    
    If cache is not None, it must be a dict, and will speed up the overhead of
    evaluation by about 5x or so on multiple calls. If the item is cached, the
    function will not call the method and test for a NotImplementedError to
    determine if the method is implemented. By passing the same cache to
    multiple calls, this function will only explore the object's methods once.

    Returns: 
        multiple: If fn is a list, returns a pair (res,fn) where res is the
        result of a successful call, and fn is the function name. 
        
        If fn is a str, returns the result of a successful call or the
        fallback.
    """
    if isinstance(fn,list):
        #go through functions
        assert len(fn) == len(args)
        for i,(f,a) in enumerate(zip(fn,args)):
            docall = False
            testcall = True
            if cache is not None:
                try:
                    if not cache[f]: continue
                    docall = True
                    testcall = False
                except KeyError:
                    #test whether it exists
                    testcall = True
            if docall or testcall:
                try:
                    resolved = getattr(obj,f)
                except AttributeError:
                    if cache is not None:
                        cache[f] = False
                    continue
                if callable(a):
                    try:
                        a = a()
                    except NotImplementedError:
                        #used to mark something is currently invalid and should cause a fallback
                        continue
                    except Exception:
                        print("Exception raised while evaluating argument of method",f,"of object",str(obj))
                        raise
                    args[i] = a
                assert isinstance(args[i],(list,tuple)),"Invalid type of argument list, must be tuple: "+str(args[i])
            if testcall:
                try:
                    res = resolved(*args[i])
                    if cache is not None:
                        cache[f] = True
                    return res,f
                except NotImplementedError:
                    if cache is not None:
                        cache[f] = False
                except Exception:
                    print("Exception raised while evaluating method",f,"of object",str(obj))
                    raise
                continue
            if docall:  #known to exist
                return resolved(*args[i]),f
            #fall through to next iterations
            pass
        raise NotImplementedError("Functions "+','.join(fn)+" are not implemented by base interface")
    
    #one function only
    docall = False
    trycall = True
    if cache is not None:
        #print("Trying to get native",fn)
        try:
            docall = cache[fn]
            trycall = False
        except KeyError:
            trycall = True
    if trycall:
        try:
            resolved = getattr(obj,fn)
        except AttributeError:
            if cache is not None:
                cache[fn] = False
            if fallback is None:
                raise NotImplementedError("Function {} is not defined by base interface {}, no fallback available".format(fn,obj))
            if callable(fallback):
                return fallback(*args)
            return fallback
        try:
            #print("...first time...")
            res = resolved(*args)
            #print("...available")
            if cache is not None:
                cache[fn] = True
            return res
        except NotImplementedError:
            if cache is not None:
                cache[fn] = False
        except Exception as e:
            print("Error",e,"received during call of",fn,"of base",str(obj))
            raise
    if docall:
        try:
            res = getattr(obj,fn)(*args)
            return res
        except Exception as e:
            print("Error",e,"received during later call of",fn,"of base",str(obj))
            raise
    if fallback is None:
        raise NotImplementedError("Method {} is not defined by object {}, no fallback available".format(fn,obj))
    if callable(fallback):
        return fallback(*args)
    return fallback


class _LockedCallable:
    """Helper"""
    def __init__(self,func,lock):
        self._func = func
        self._lock = lock
    
    def __call__(self,*args,**kwargs):
        with self._lock:
            return self._func(*args,**kwargs)


class _RobotInterfaceStatefulBase(RobotInterfaceBase):
    """A helper class that accesses as much as possible from settings / state.
    Used to avoid lots of boilerplate code when building more useful wrappers.

    Child classes must implement initialize(), beginStep(), and endStep().

    Child classes should also clear out _delta_settings at beginStep(), then 
    check it at endStep() to see settings that have changed. The
    :func:`_update_from_settings` helper function propagates changes to the 
    new settings and any associated robot interface.
    """
    def __init__(self):
        RobotInterfaceBase.__init__(self)
        self._structure = _RobotInterfaceStructure()
        self._settings = _RobotInterfaceSettings()
        self._delta_settings = None                 #type: _RobotInterfaceSettings
        self._commands = []                         #type: List[_RobotInterfaceCommand]
        self._state = _RobotInterfaceState()
        self._shortcut_update = True                #if True, the corresponding query to a setX command will be updated immediately
        self._partInterfaces = dict()               #type: Dict[str,'_RobotInterfaceStatefulBase']
        self._has = dict()                          #type: Dict[str,bool]

    COMPLEX_METHODS = ['_queueCommand','setSetting','getSetting',
            'sensors','enabledSensors','hasSensor','enableSensor',
            'getGravityCompensation','setGravityCompensation',
            'getPIDGains','setPIDGains',
            'sensedCartesianPosition','sensedCartesianVelocity','sensedCartesianForce',
            'commandedCartesianPosition','commandedCartesianVelocity','commandedCartesianForce',
            'destinationCartesianPosition','destinationCartesianVelocity']

    def _queueCommand(self,func,args=()):
        if not self._has.get(func,True):
            raise NotImplementedError("Interface does not implement {}".format(func))
        cmd = _RobotInterfaceCommand()
        cmd.time = time.time()
        cmd.func = func
        cmd.args = args
        cmd.promise = Promise()
        self._commands.append(cmd)
        if len(self._commands) > 1000:
            raise RuntimeError("Queue has grown abnormally large, must not be correctly processed?")
        return cmd.promise
    
    def _init_part_interfaces(self):
        for (k,inds) in self._structure.parts.items():
            if k is None: continue
            partInterface = _RobotInterfaceStatefulBase()
            partInterface._structure.controlRate = self._structure.controlRate
            partInterface._structure.numJoints = len(inds)
            if self._structure.jointNames is not None:
                partInterface._structure.jointNames = [self._structure.jointNames[i] for i in inds]
            if self._structure.klamptModel is not None:
                link_inds = []
                for i in inds:
                    link_inds += self._structure.klamptModel.driver(i).getAffectedLinks()
                partInterface._structure.klamptModel = SubRobotModel(self._structure.klamptModel,link_inds)
            partInterface.properties['name'] = k
            partInterface.properties['part'] = True
            self._partInterfaces[k] = partInterface

    def _split_settings(self):
        for part,iface in self._partInterfaces.items():   #initial time
            assert part is not None,"Shouldn't have None as a partInterface"
            if part not in self._settings.children:
                self._settings.children[part] = _RobotInterfacePartSettings(iface._settings)
        sub_settings = _split_settings(self._settings,self._structure.parts)
        for part,iface in self._partInterfaces.items():
            self._partInterfaces[part]._settings = sub_settings[part]

    def _split_state(self):
        sub_states = _split_state(self._state, self._structure.parts)
        for (part,iface) in self._partInterfaces.items():
            iface._state = sub_states[part]
        
    def _clear_deltas(self):
        self._delta_settings = _RobotInterfaceSettings()
        for (part,iface) in self._partInterfaces.items():
            iface._delta_settings = _RobotInterfaceSettings()
    
    def _advance_settings_and_commands(self) -> Tuple[_RobotInterfaceSettings,List[_RobotInterfaceCommand]]:
        """Returns the settings delta and queue of new commands. 
        Clears the settings delta and command queue.
        """
        _gather_settings(self._structure.parts,dict((k,v._delta_settings) for (k,v) in self._partInterfaces.items()),self._delta_settings)
        _update_from_settings(None,self._settings,self._delta_settings)
        settings = _split_settings(self._settings,self._structure.parts)
        for (k,setting) in settings.items():
            self._partInterfaces[k]._settings = setting
        cmds = _gather_commands([self._commands]+[iface._commands for iface in self._partInterfaces.values()],
                                [self.indices()]+list(v for (k,v) in self._structure.parts.items() if k is not None))
        res = self._delta_settings
        #clear settings deltas queues
        self._delta_settings = _RobotInterfaceSettings()
        for (k,iface) in self._partInterfaces.items():
            iface._delta_settings = _RobotInterfaceSettings()
        #clear part command queues
        for (k,iface) in self._partInterfaces.items():
            iface._commands = []
        self._commands = []
        return res,cmds
        
    def controlRate(self):
        if self._structure.controlRate is not None: return self._structure.controlRate
        if self._state.rate is None: raise NotImplementedError()
        return self._state.rate

    def parts(self):
        if self._structure.parts is None: raise NotImplementedError()
        return self._structure.parts
    
    def partInterface(self,part,joint_idx=None):
        if joint_idx is not None:
            raise NotImplementedError("TODO: part interfaces for individual joint indices")
        return self._partInterfaces[part]
    
    def numJoints(self,part = None):
        if part is None:
            return self._structure.numJoints
        return len(self._structure.parts[part])

    def jointName(self,joint_idx):
        if self._structure.jointNames is None:
            raise NotImplementedError()
        return self._structure.jointNames[joint_idx]

    def clock(self):
        if self._state.clock is None: raise NotImplementedError("clock() not implemented by base or beginStep() not called")
        return self._state.clock

    def estop(self):
        return self._queueCommand('estop')

    def softStop(self):
        return self._queueCommand('softStop')

    def reset(self):
        return self._queueCommand('reset')

    def setSetting(self,key,value):
        if self._delta_settings is None: raise RuntimeError("Settings query called outside of beginStep()/endStep()")
        if self._delta_settings.settings is None: self._delta_settings.settings = dict()
        self._delta_settings.settings[key] = value
        
    def getSetting(self,key):
        if self._delta_settings is None: raise RuntimeError("Settings query called outside of beginStep()/endStep()")
        if self._delta_settings.settings is not None and key in self._delta_settings.settings:
            return self._delta_settings.settings[key]
        if self._settings.settings is None: raise KeyError("Settings has not been initialized")
        return copy.copy(self._settings.settings[key])

    def sensors(self):
        if self._settings.sensorEnabled is None: raise NotImplementedError()
        return list(self._settings.sensorEnabled.keys())

    def enabledSensors(self):
        if self._delta_settings is None: raise RuntimeError("Settings query called outside of beginStep()/endStep()")
        if self._delta_settings.sensorEnabled is not None: 
            if self._settings.sensorEnabled is not None: 
                #return union of previously and recently enabled sensors
                return [s for (s,e) in self._settings.sensorEnabled.items() if e or self._delta_settings.sensorEnabled[s]]
            return [s for (s,e) in self._delta_settings.sensorEnabled.items() if e]
        if self._settings.sensorEnabled is None: return []
        return [s for (s,e) in self._settings.sensorEnabled.items() if e]

    def hasSensor(self,sensor):
        return self._settings.sensorEnabled and sensor in self._settings.sensorEnabled

    def enableSensor(self,sensor,enabled=True):
        if self._settings.sensorEnabled is None: raise NotImplementedError()
        if self._delta_settings is None: raise RuntimeError("Settings query called outside of beginStep()/endStep()")
        if self._delta_settings.sensorEnabled is None:
            self._delta_settings.sensorEnabled = dict((s,False) for s in self._settings.sensorEnabled)
        self._delta_settings.sensorEnabled[sensor] = enabled

    def sensorMeasurements(self,sensor):
        if self._state.sensorState is None: raise NotImplementedError()
        assert sensor in self._state.sensorState
        return self._state.sensorState[sensor][1]

    def sensorUpdateTime(self,sensor):
        if self._state.sensorState is None: raise NotImplementedError()
        assert sensor in self._state.sensorState
        return self._state.sensorState[sensor][0]

    def setPosition(self,q):
        assert len(q)==self._structure.numJoints,"Invalid size of position command: {} != {}".format(len(q),self._structure.numJoints)
        res = self._queueCommand('setPosition',(q,))
        if self._shortcut_update:
            self._state.commandedPosition = copy.copy(q)
        return res

    def setVelocity(self,v,ttl=None):
        assert len(v)==self._structure.numJoints,"Invalid size of velocity command: {} != {}".format(len(v),self._structure.numJoints)
        res = self._queueCommand('setVelocity',(v,ttl))
        if self._shortcut_update:
            self._state.commandedVelocity = copy.copy(v)
        return res

    def setTorque(self,t,ttl=None):
        assert len(t)==self._structure.numJoints,"Invalid size of torque command: {} != {}".format(len(t),self._structure.numJoints)
        res = self._queueCommand('setTorque',(t,ttl))
        if self._shortcut_update:
            self._state.commandedTorque = copy.copy(t)
        return res
        
    def setPID(self,q,dq,t=None):
        assert len(q)==self._structure.numJoints,"Invalid size of position command: {} != {}".format(len(q),self._structure.numJoints)
        assert len(dq)==self._structure.numJoints,"Invalid size of velocity command: {} != {}".format(len(dq),self._structure.numJoints)
        if t is not None:
            assert len(t)==self._structure.numJoints,"Invalid size of torque command: {} != {}".format(len(t),self._structure.numJoints)
        res = self._queueCommand('setPID',(q,dq,t))
        if self._shortcut_update:
            self._state.commandedPosition = copy.copy(q)
            self._state.commandedVelocity = copy.copy(dq)
        return res    

    def setPIDGains(self,kP,kI,kD):
        if self._delta_settings is None: raise RuntimeError("Settings query called outside of beginStep()/endStep()")
        if not hasattr(kP,'__iter__'):
            kP = [kP]*len(self._structure.numJoints)
        if not hasattr(kD,'__iter__'):
            kD = [kD]*len(self._structure.numJoints)
        if not hasattr(kI,'__iter__'):
            kI = [kI]*len(self._structure.numJoints)
        assert len(kP)==self._structure.numJoints,"Invalid size of kP: {} != {}".format(len(kP),self._structure.numJoints)
        assert len(kI)==self._structure.numJoints,"Invalid size of kI: {} != {}".format(len(kI),self._structure.numJoints)
        assert len(kD)==self._structure.numJoints,"Invalid size of kD: {} != {}".format(len(kD),self._structure.numJoints)
        assert all(v >= 0 for v in kP),"kP must be nonnegative"
        assert all(v >= 0 for v in kI),"kI must be nonnegative"
        assert all(v >= 0 for v in kD),"kD must be nonnegative"
        self._delta_settings.kP = kP
        self._delta_settings.kI = kI
        self._delta_settings.kD = kD

    def getPIDGains(self):
        if self._delta_settings is None: raise RuntimeError("Settings query called outside of beginStep()/endStep()")
        if self._delta_settings.kP is not None:
            return (copy.copy(self._delta_settings.kP),copy.copy(self._delta_settings.kI),copy.copy(self._delta_settings.kD))
        if self._settings.kP is None: raise NotImplementedError()
        return (copy.copy(self._settings.kP),copy.copy(self._settings.kI),copy.copy(self._settings.kD))

    def moveToPosition(self,q,speed=1):
        assert len(q)==self._structure.numJoints,"Invalid size of position command: {} != {}".format(len(q),self._structure.numJoints)
        res = self._queueCommand('moveToPosition',(q,speed))
        if self._shortcut_update:
            self._state.destinationPosition = copy.copy(q)
        return res

    def setPiecewiseLinear(self,ts,qs,relative=True):
        for q in qs:
            assert len(q)==self._structure.numJoints,"Invalid size of position milestone: {} != {}".format(len(q),self._structure.numJoints)
        assert len(qs) == len(ts)
        res = self._queueCommand('setPiecewiseLinear',(ts,qs,relative))
        if self._shortcut_update and len(ts):
            self._state.destinationPosition = copy.copy(qs[-1])
        return res

    def setPiecewiseCubic(self,ts,qs,vs,relative=True):
        for q in qs:
            assert len(q)==self._structure.numJoints,"Invalid size of position milestone: {} != {}".format(len(q),self._structure.numJoints)
        for v in vs:
            assert len(v)==self._structure.numJoints,"Invalid size of velocity milestone: {} != {}".format(len(v),self._structure.numJoints)
        assert len(qs) == len(ts)
        assert len(vs) == len(ts)
        res = self._queueCommand('setPiecewiseCubic',(ts,qs,vs,relative))
        if self._shortcut_update and len(ts):
            self._state.destinationPosition = copy.copy(qs[-1])
            self._state.destinationVelocity = copy.copy(vs[-1])
        return res

    def setToolCoordinates(self,xtool_local):
        if self._delta_settings is None: raise RuntimeError("Settings query called outside of beginStep()/endStep()")
        self._delta_settings.toolCoordinates = xtool_local

    def getToolCoordinates(self):
        if self._delta_settings is None: raise RuntimeError("Settings query called outside of beginStep()/endStep()")
        if self._delta_settings.toolCoordinates:
            return self._delta_settings.toolCoordinates
        if self._settings.toolCoordinates is None: raise NotImplementedError()
        return self._settings.toolCoordinates

    def setGravityCompensation(self,gravity=[0,0,-9.8],load=0.0,load_com=[0,0,0]):
        if self._delta_settings is None: raise RuntimeError("Settings query called outside of beginStep()/endStep()")
        self._delta_settings.gravity = gravity
        self._delta_settings.load = load
        self._delta_settings.load_com = load_com

    def getGravityCompensation(self):
        if self._delta_settings is None: raise RuntimeError("Settings query called outside of beginStep()/endStep()")
        if self._delta_settings.gravity:
            return (copy.copy(self._delta_settings.gravity),self._delta_settings.load,copy.copy(self._delta_settings.load_com))
        if self._settings.gravity is None: raise NotImplementedError()
        return (copy.copy(self._settings.gravity),self._settings.load,copy.copy(self._settings.load_com))

    def setCartesianPosition(self,xparams,frame='world'):
        res = self._queueCommand('setCartesianPosition',(xparams,frame))
        if self._shortcut_update and frame == 'base':
            self._state.cartesianState.commandedPosition = xparams
        return res

    def moveToCartesianPosition(self,xparams,speed=1.0,frame='world'):
        res = self._queueCommand('moveToCartesianPosition',(xparams,speed,frame))
        if self._shortcut_update and frame == 'base':
            self._state.cartesianState.destinationPosition = xparams
        return res
    
    def moveToCartesianPositionLinear(self,xparams,speed=1.0,frame='world'):
        res = self._queueCommand('moveToCartesianPositionLinear',(xparams,speed,frame))
        if self._shortcut_update and frame == 'base':
            self._state.cartesianState.destinationPosition = xparams
        return res

    def setCartesianVelocity(self,dxparams,ttl=None,frame='world'):
        res = self._queueCommand('setCartesianVelocity',(dxparams,ttl,frame))
        if self._shortcut_update and frame == 'base':
            self._state.cartesianState.commandedVelocity = dxparams
        return res

    def setCartesianForce(self,fparams,ttl=None,frame='world'):
        res = self._queueCommand('setCartesianForce',(fparams,ttl,frame))
        if self._shortcut_update and frame == 'base':
            self._state.cartesianState.commandedForce = fparams
        return res

    def status(self,joint_idx=None):
        if self._state.status is None: raise NotImplementedError()
        return self._state.status

    def isMoving(self,joint_idx=None):
        if self._state.isMoving is None: raise NotImplementedError()
        return self._state.isMoving

    def sensedPosition(self):
        if self._state.sensedPosition is None: raise NotImplementedError()
        return copy.copy(self._state.sensedPosition)

    def sensedVelocity(self):
        if self._state.sensedVelocity is None: raise NotImplementedError()
        return copy.copy(self._state.sensedVelocity)

    def sensedTorque(self):
        if self._state.sensedTorque is None: raise NotImplementedError()
        return copy.copy(self._state.sensedTorque)

    def commandedPosition(self):
        if self._state.commandedPosition is None: raise NotImplementedError()
        return copy.copy(self._state.commandedPosition)

    def commandedVelocity(self):
        if self._state.commandedVelocity is None: raise NotImplementedError()
        return copy.copy(self._state.commandedVelocity)

    def commandedTorque(self):
        if self._state.commandedTorque is None: raise NotImplementedError()
        return copy.copy(self._state.commandedTorque)

    def destinationPosition(self):
        if self._state.destinationPosition is None: raise NotImplementedError()
        return copy.copy(self._state.destinationPosition)

    def destinationVelocity(self):
        if self._state.destinationVelocity is None: raise NotImplementedError()
        return copy.copy(self._state.destinationVelocity)

    def destinationTime(self):
        if self._state.destinationTime is None: raise NotImplementedError()
        return copy.copy(self._state.destinationTime)
    
    def stateValue(self,name):
        if self._state.stateValue is None: raise NotImplementedError()
        return copy.copy(self._state.stateValue.get(name,None))
    
    def cartesianPosition(self,q,frame='world'):
        assert len(q) == self._structure.numJoints,"Invalid length of configuration: {} != {}".format(len(q),self._structure.numJoints)
        toolCoords = self.getToolCoordinates()
        model = self.klamptModel()
        if toolCoords is None or model is None: raise NotImplementedError()
        indices = list(range(model.numDrivers()))
        assert(self._structure.numJoints == model.numDrivers())
        if frame == 'world' and isinstance(model,SubRobotModel):
            raise NotImplementedError("cartesianPosition can't calculate kinematics because it doesn't know whole-robot configuration")
        return klamptCartesianPosition(model,q,indices,toolCoords,frame)
    
    def cartesianVelocity(self,q,dq,frame='world'):
        assert len(q) == self._structure.numJoints,"Invalid length of configuration: {} != {}".format(len(q),self._structure.numJoints)
        assert len(dq) == self._structure.numJoints,"Invalid length of velocity: {} != {}".format(len(dq),self._structure.numJoints)
        toolCoords = self.getToolCoordinates()
        model = self.klamptModel()
        if toolCoords is None or model is None: raise NotImplementedError()
        indices = list(range(model.numDrivers()))
        assert(self._structure.numJoints == model.numDrivers())
        if frame == 'world' and isinstance(model,SubRobotModel):
            raise NotImplementedError("cartesianVelocity can't calculate kinematics because it doesn't know whole-robot configuration")
        return klamptCartesianVelocity(model,q,dq,indices,toolCoords,frame)

    def cartesianForce(self,q,t,frame='world'):
        assert len(q) == self._structure.numJoints,"Invalid length of configuration: {} != {}".format(len(q),self._structure.numJoints)
        assert len(t) == self._structure.numJoints,"Invalid length of torques: {} != {}".format(len(t),self._structure.numJoints)
        toolCoords = self.getToolCoordinates()
        model = self.klamptModel()
        if toolCoords is None or model is None: raise NotImplementedError()
        indices = list(range(model.numDrivers()))
        assert(self._structure.numJoints == model.numDrivers())
        if frame == 'world' and isinstance(model,SubRobotModel):
            raise NotImplementedError("cartesianForce can't calculate kinematics because it doesn't know whole-robot configuration")
        return klamptCartesianForce(model,q,t,indices,toolCoords,frame)

    def sensedCartesianPosition(self,frame='world'):
        toolCoordinates = self.getToolCoordinates()
        if toolCoordinates is None: raise NotImplementedError("Cartesian control not available")
        if self._state.cartesianState is None or self._state.cartesianState.sensedPosition is None:
            raise NotImplementedError("Cartesian control not available")
        if frame=='base': return copy.copy(self._state.cartesianState.sensedPosition)
        elif frame == 'tool': return se3.identity()
        elif frame == 'end effector': return (so3.identity(),copy.copy(toolCoordinates))
        else: #base -> world
            Tbase = self._state.cartesianState.baseSensedPosition
            if Tbase is None: return self._state.cartesianState.sensedPosition
            return se3.mul(Tbase,self._state.cartesianState.sensedPosition)

    def sensedCartesianVelocity(self,frame='world'):
        toolCoordinates = self.getToolCoordinates()
        if toolCoordinates is None: raise NotImplementedError("Cartesian control not available")
        if self._state.cartesianState is None or self._state.cartesianState.sensedVelocity is None:
            raise NotImplementedError("Cartesian control not available")
        wtool_base,vtool_base = self._state.cartesianState.sensedVelocity
        Ttool_base = self._state.cartesianState.sensedPosition
        if frame=='base': return copy.copy(self._state.cartesianState.sensedVelocity)
        elif frame == 'tool': 
            if Ttool_base is None: Rbase_tool = so3.identity()
            else: Rbase_tool = so3.inv(Ttool_base[0])
            return (so3.apply(Rbase_tool,wtool_base),so3.apply(Rbase_tool,vtool_base))
        elif frame == 'end effector':
            if Ttool_base is None: Rbase_tool = so3.identity()
            else: Rbase_tool = so3.inv(Ttool_base[0])
            wtool_tool = so3.apply(Rbase_tool,wtool_base)
            vtool_tool = so3.apply(Rbase_tool,vtool_base)
            vtool_ee = vectorops.cross(wtool_tool,toolCoordinates)
            return (wtool_tool,vectorops.add(vtool_tool,vtool_ee))
        else: #base -> world
            Tbase = self._state.cartesianState.baseSensedPosition
            if Tbase is None: return copy.copy(self._state.cartesianState.sensedVelocity)
            Rbase = Tbase[0]
            wtool = so3.apply(Rbase,wtool_base)
            vtool = so3.apply(Rbase,vtool_base)
            if self._state.cartesianState.sensedVelocity is None:
                return (wtool,vtool)
            else:
                wbase,vbase = self._state.cartesianState.sensedVelocity
                #add on base angular velocity x relative position of tool
                return (vectorops.add(wbase,wtool),vectorops.add(vbase,vtool,vectorops.cross(wbase,Ttool_base[1])))

    def sensedCartesianForce(self,frame='world'):
        toolCoordinates = self.getToolCoordinates()
        if toolCoordinates is None: raise NotImplementedError("Cartesian control not available")
        if self._state.cartesianState is None or self._state.cartesianState.sensedForce is None:
            raise NotImplementedError("Cartesian control not available")
        ttool_base,ftool_base = self._state.cartesianState.sensedForce
        Ttool_base = self._state.cartesianState.sensedPosition
        if frame=='base': return copy.copy(self._state.cartesianState.sensedForce)
        elif frame == 'tool': 
            if Ttool_base is None: Rbase_tool = so3.identity()
            else: Rbase_tool = so3.inv(Ttool_base[0])
            return (so3.apply(Rbase_tool,ttool_base),so3.apply(Rbase_tool,ftool_base))
        elif frame == 'end effector':
            if Ttool_base is None: Rbase_tool = so3.identity()
            else: Rbase_tool = so3.inv(Ttool_base[0])
            ttool_tool = so3.apply(Rbase_tool,ttool_base)
            ftool_tool = so3.apply(Rbase_tool,ftool_base)
            ftool_ee = vectorops.cross(ttool_tool,toolCoordinates)
            return (ttool_tool,vectorops.add(ftool_tool,ftool_ee))
        else: #base -> world
            Tbase = self._state.cartesianState.baseSensedPosition
            if Tbase is None: return copy.copy(self._state.cartesianState.sensedForce)
            Rbase = Tbase[0]
            ttool = so3.apply(Rbase,ttool_base)
            ftool = so3.apply(Rbase,ftool_base)
            return (ttool,ftool)

    def commandedCartesianPosition(self,frame='world'):
        toolCoordinates = self.getToolCoordinates()
        if toolCoordinates is None: raise NotImplementedError("Cartesian control not available")
        if self._state.cartesianState is None or self._state.cartesianState.commandedPosition is None:
            raise NotImplementedError("Cartesian control not available")
        if frame=='base': return copy.copy(self._state.cartesianState.commandedPosition)
        elif frame == 'tool': return se3.identity()
        elif frame == 'end effector': return (so3.identity(),toolCoordinates)
        else: #base -> world
            Tbase = self._state.cartesianState.baseCommandedPosition
            if Tbase is None:
                return copy.copy(self._state.cartesianState.commandedPosition)
            return se3.mul(Tbase,self._state.cartesianState.commandedPosition)

    def commandedCartesianVelocity(self,frame='world'):
        toolCoordinates = self.getToolCoordinates()
        if toolCoordinates is None: raise NotImplementedError("Cartesian control not available")
        if self._state.cartesianState is None or self._state.cartesianState.commandedVelocity is None:
            raise NotImplementedError("Cartesian control not available")
        wtool_base,vtool_base = self._state.cartesianState.commandedVelocity
        Ttool_base = self._state.cartesianState.commandedPosition
        if frame=='base': return copy.copy(self._state.cartesianState.commandedVelocity)
        elif frame == 'tool': 
            if Ttool_base is None: Rbase_tool = so3.identity()
            else: Rbase_tool = so3.inv(Ttool_base[0])
            return (so3.apply(Rbase_tool,wtool_base),so3.apply(Rbase_tool,vtool_base))
        elif frame == 'end effector':
            if Ttool_base is None: Rbase_tool = so3.identity()
            else: Rbase_tool = so3.inv(Ttool_base[0])
            wtool_tool = so3.apply(Rbase_tool,wtool_base)
            vtool_tool = so3.apply(Rbase_tool,vtool_base)
            vtool_ee = vectorops.cross(wtool_tool,toolCoordinates)
            return (wtool_tool,vectorops.add(vtool_tool,vtool_ee))
        else: #base -> world
            Tbase = copy.copy(self._state.cartesianState.baseCommandedPosition)
            if Tbase is None: return self._state.cartesianState.commandedVelocity
            Rbase = Tbase[0]
            wtool = so3.apply(Rbase,wtool_base)
            vtool = so3.apply(Rbase,vtool_base)
            if self._state.cartesianState.commandedVelocity is None:
                return (wtool,vtool)
            else:
                wbase,vbase = self._state.cartesianState.commandedVelocity
                #add on base angular velocity x relative position of tool
                return (vectorops.add(wbase,wtool),vectorops.add(vbase,vtool,vectorops.cross(wbase,Ttool_base[1])))

    def commandedCartesianForce(self,frame='world'):
        toolCoordinates = self.getToolCoordinates()
        if toolCoordinates is None: raise NotImplementedError("Cartesian control not available")
        if self._state.cartesianState is None or self._state.cartesianState.commandedForce is None:
            raise NotImplementedError("Cartesian control not available")
        ttool_base,ftool_base = self._state.cartesianState.commandedForce
        Ttool_base = self._state.cartesianState.commandedPosition
        if frame=='base': return copy.copy(self._state.cartesianState.commandedForce)
        elif frame == 'tool': 
            if Ttool_base is None: Rbase_tool = so3.identity()
            else: Rbase_tool = so3.inv(Ttool_base[0])
            return (so3.apply(Rbase_tool,ttool_base),so3.apply(Rbase_tool,ftool_base))
        elif frame == 'end effector':
            if Ttool_base is None: Rbase_tool = so3.identity()
            else: Rbase_tool = so3.inv(Ttool_base[0])
            ttool_tool = so3.apply(Rbase_tool,ttool_base)
            ftool_tool = so3.apply(Rbase_tool,ftool_base)
            ftool_ee = vectorops.cross(ttool_tool,toolCoordinates)
            return (ttool_tool,vectorops.add(ftool_tool,ftool_ee))
        else: #base -> world
            Tbase = self._state.cartesianState.baseCommandedPosition
            if Tbase is None: return copy.copy(self._state.cartesianState.commandedForce)
            Rbase = Tbase[0]
            ttool = so3.apply(Rbase,ttool_base)
            ftool = so3.apply(Rbase,ftool_base)
            return (ttool,ftool)

    def destinationCartesianPosition(self,frame='world'):
        toolCoordinates = self.getToolCoordinates()
        if toolCoordinates is None: raise NotImplementedError("Cartesian control not available")
        if self._state.cartesianState is None or self._state.cartesianState.destinationPosition is None:
            raise NotImplementedError("Cartesian control not available")
        if frame=='base': return copy.copy(self._state.cartesianState.destinationPosition)
        elif frame == 'tool': return se3.identity()
        elif frame == 'end effector': return (so3.identity(),toolCoordinates)
        else: #base -> world
            Tbase = self._state.cartesianState.baseDestinationPosition
            if Tbase is None: return self._state.cartesianState.destinationPosition
            return se3.mul(Tbase,self._state.cartesianState.destinationPosition)

    def destinationCartesianVelocity(self,frame='world'):
        toolCoordinates = self.getToolCoordinates()
        if toolCoordinates is None: raise NotImplementedError("Cartesian control not available")
        if self._state.cartesianState is None or self._state.cartesianState.destinationVelocity is None:
            raise NotImplementedError("Cartesian control not available")
        wtool_base,vtool_base = self._state.cartesianState.destinationVelocity
        Ttool_base = self._state.cartesianState.destinationPosition
        if frame=='base': return copy.copy(self._state.cartesianState.destinationVelocity)
        elif frame == 'tool': 
            if Ttool_base is None: Rbase_tool = so3.identity()
            else: Rbase_tool = so3.inv(Ttool_base[0])
            return (so3.apply(Rbase_tool,wtool_base),so3.apply(Rbase_tool,vtool_base))
        elif frame == 'end effector':
            if Ttool_base is None: Rbase_tool = so3.identity()
            else: Rbase_tool = so3.inv(Ttool_base[0])
            wtool_tool = so3.apply(Rbase_tool,wtool_base)
            vtool_tool = so3.apply(Rbase_tool,vtool_base)
            vtool_ee = vectorops.cross(wtool_tool,toolCoordinates)
            return (wtool_tool,vectorops.add(vtool_tool,vtool_ee))
        else: #base -> world
            Tbase = self._state.cartesianState.baseDestinationPosition
            if Tbase is None: return copy.copy(self._state.cartesianState.destinationVelocity)
            Rbase = Tbase[0]
            wtool = so3.apply(Rbase,wtool_base)
            vtool = so3.apply(Rbase,vtool_base)
            if self._state.cartesianState.destinationVelocity is None:
                return (wtool,vtool)
            else:
                wbase,vbase = self._state.cartesianState.destinationVelocity
                #add on base angular velocity x relative position of tool
                return (vectorops.add(wbase,wtool),vectorops.add(vbase,vtool,vectorops.cross(wbase,Ttool_base[1])))

    def klamptModel(self):
        if self._structure.klamptModel is None:
            res = RobotInterfaceBase.klamptModel(self)
            self._structure.klamptModel = res
        return self._structure.klamptModel


class _RobotInterfaceStatefulWrapper(_RobotInterfaceStatefulBase):
    """A helper class that wraps another RobotInterfaceBase. It follows
    the paradigm:

    0. Query the base's settings.
    1. beginStep(): queries the base's state and stores it.
    2. User can
        * Change the settings.
        * Query state (latched on step 1 and incurs no overhead).
        * Issue commands (recorded).
    3. endStep():
        * If the settings changed, pass them onto the base.
        * If commands were issued, pass them to the base.
    
    The main use of this class is so that managing interfaces can
    read state/commmands and modify commands before endStep() is called.

    It is also helpful for threaded / multiprocessing interfaces because
    all settings/state/commands are stored in one place.
    """
    def __init__(self, base: RobotInterfaceBase, base_initialized:bool = False):
        _RobotInterfaceStatefulBase.__init__(self)
        self._base = base  #type: RobotInterfaceBase
        self._base_initialized = base_initialized
        self._has = dict()
        self._first_step = True
        self._partInterfaces = dict()               #type: Dict[str,'_RobotInterfaceStatefulWrapper']
        self._in_step = False
        self.properties = copy.deepcopy(base.properties)
    
    def _try(self,fn,args,defaultValue=None,strict=True):
        try:
            if self._has[fn]:
                return getattr(self._base,fn)(*args)
            return defaultValue
        except KeyError:   #first time
            try:
                res = getattr(self._base,fn)(*args)
                #print("...available")
                self._has[fn] = True
                return res
            except NotImplementedError:
                self._has[fn] = False
                return defaultValue
            except Exception as e:
                if strict:
                    print("Error",e,"received during first call of",fn,"of base",str(self._base))
                    raise
                else:
                    self._has[fn] = False
                    return defaultValue
        except Exception as e:
            print("Error",e,"received during later call of",fn,"of base",str(self._base))
            raise

    def __str__(self):
        return "StatefulWrapper("+str(self._base)+')'
    
    def initialize(self):
        if not self._base_initialized:
            if not self._base.initialize(): return False
            self._base_initialized = True
        self._try('beginStep',())
        try:
            self._structure.controlRate = self._base.controlRate()
        except Exception:
            self._structure.controlRate = None
        self._structure.parts = self._base.parts()
        self._structure.numJoints = len(self._structure.parts[None])
        self._structure.klamptModel = self._base.klamptModel()
        try:
            self._structure.jointNames = [None]*self._structure.numJoints
            for i in range(self._structure.numJoints):
                self._structure.jointNames[i] = self._base.jointName(i)
        except Exception:
            self._structure.jointNames = None
        for p in self._structure.parts:
            if p is None: continue
            self._partInterfaces[p] = _RobotInterfaceStatefulWrapper(self._base.partInterface(p),base_initialized=True)
            self._partInterfaces[p].properties['name'] = p
            self._partInterfaces[p].properties['part'] = True
            self._partInterfaces[p].initialize()
            if len(self._partInterfaces[p].parts()) > 1:
                raise NotImplementedError("TODO: parts that have sub-parts")
        #read settings
        sensors = self._try('sensors',(),[])
        self._structure.sensors = sensors if sensors else None
        if sensors:
            self._settings.sensorEnabled = dict()
        for s in sensors:
            self._settings.sensorEnabled[s] = False
        for s in self._try('enabledSensors',(),[]):
            self._settings.sensorEnabled[s] = True
        res = self._try('getPIDGains',(),strict=False)
        if res is not None:
            self._settings.kP,self._settings.kI,self._settings.kD = res
        res = self._try('getToolCoordinates',(),strict=False)
        if res is not None:
            self._settings.toolCoordinates = res
        res = self._try('getGravityCompensation',(),strict=False)
        if res is not None:
            self._settings.gravity = res[0]
            self._settings.load = res[1]
            self._settings.load_com = res[2]
        if len(self._structure.parts) > 1:
            self._settings.children = dict()
        res = self._try('getSettings',(),strict=False)
        if res is not None:
            self._settings.settings = res
        self._split_settings()

        #discover capabilities by commanding no-ops
        self._state.commandedPosition = self._try('commandedPosition',(),self._state.commandedPosition)
        self._state.commandedVelocity = self._try('commandedVelocity',(),self._state.commandedVelocity,strict=False)
        self._state.commandedTorque = self._try('commandedTorque',(),self._state.commandedTorque,strict=False)
        if _valid_vector(self._state.commandedTorque):
            self._try('setTorque',(self._state.commandedTorque,))
        if _valid_vector(self._state.commandedVelocity):
            self._try('setVelocity',(self._state.commandedVelocity,))
            self._try('setPiecewiseCubic',([0],[self._state.commandedPosition],[self._state.commandedVelocity]))
            self._try('setPID',(self._state.commandedPosition,self._state.commandedVelocity))
        else:
            vel = [0]*self._structure.numJoints
            self._try('setVelocity',(vel,))
            self._try('setPiecewiseCubic',([0],[self._state.commandedPosition],[vel]),strict=False)
            self._try('setPID',(self._state.commandedPosition,vel),strict=False)
        if _valid_vector(self._state.commandedPosition):
            self._try('setPosition',(self._state.commandedPosition,))
            self._try('moveToPosition',(self._state.commandedPosition,),strict=False)
            self._try('setPiecewiseLinear',([0],[self._state.commandedPosition]))
        else:
             print("strange commanded position?",self._state.commandedPosition)
             input()
        #print("My base class",self._base,"implements",self._has)
        #input("Press enter to continue")

        self._try('endStep',())
        return True

    def close(self):
        return self._base.close()

    def beginStep(self):
        if self._in_step:
            raise RuntimeError("beginStep called twice without endStep")
        self._try('beginStep',())
        self._state.rate = self._try('controlRate',())
        self._state.clock = self._try('clock',())
        self._state.status = self._try('status',())
        self._state.sensedPosition = self._try('sensedPosition',(),self._state.sensedPosition)
        self._state.sensedVelocity = self._try('sensedVelocity',(),self._state.sensedVelocity)
        self._state.sensedTorque = self._try('sensedTorque',(),self._state.sensedTorque)
        self._state.commandedPosition = self._try('commandedPosition',(),self._state.commandedPosition)
        self._state.commandedVelocity = self._try('commandedVelocity',(),self._state.commandedVelocity)
        self._state.commandedTorque = self._try('commandedTorque',(),self._state.commandedTorque)
        self._state.destinationPosition = self._try('destinationPosition',(),self._state.destinationPosition)
        self._state.destinationVelocity = self._try('destinationVelocity',(),self._state.destinationVelocity)
        self._state.destinationTime = self._try('destinationTime',(),self._state.destinationTime)
        self._state.queuedTrajectory = self._try('queuedTrajectory',(),self._state.queuedTrajectory)
        if self._settings.toolCoordinates is not None:
            if self._state.cartesianState is None:
                self._state.cartesianState = _RobotInterfaceCartesianState()
            self._state.cartesianState.sensedPosition = self._try('sensedCartesianPosition',('base',),self._state.cartesianState.sensedPosition,strict=False)
            self._state.cartesianState.sensedVelocity = self._try('sensedCartesianVelocity',('base',),self._state.cartesianState.sensedVelocity,strict=False)
            self._state.cartesianState.sensedForce = self._try('sensedCartesianForce',('base',),self._state.cartesianState.sensedForce,strict=False)
            self._state.cartesianState.commandedPosition = self._try('commandedCartesianPosition',('base',),self._state.cartesianState.commandedPosition,strict=False)
            self._state.cartesianState.commandedVelocity = self._try('commandedCartesianVelocity',('base',),self._state.cartesianState.commandedVelocity,strict=False)
            self._state.cartesianState.commandedForce = self._try('commandedCartesianForce',('base',),self._state.cartesianState.commandedForce,strict=False)
            self._state.cartesianState.destinationPosition = self._try('destinationCartesianPosition',('base',),self._state.cartesianState.destinationPosition,strict=False)
            self._state.cartesianState.destinationVelocity = self._try('destinationCartesianVelocity',('base',),self._state.cartesianState.destinationVelocity,strict=False)
        for sensor in self._try('enabledSensors',(),[]):
            if self._state.sensorState is None:
                self._state.sensorState = dict()
            t_update = self._try('sensorUpdateTime',(sensor,),0)
            if sensor not in self._state.sensorState or t_update > self._state.sensorState[sensor][0]:
                measurements = self._try('sensorMeasurements',(sensor,),[])
                self._state.sensorState[sensor] = (t_update,measurements)
        self._split_state()
        self._clear_deltas()
        for (part,iface) in self._partInterfaces.items():
            iface.beginStep()
        self._in_step = True
    
    def endStep(self):
        if not self._in_step:
            raise RuntimeError("endStep called without corresponding beginStep")
        errors = []
        #update settings from deltas
        _gather_settings(self._structure.parts,dict((k,iface._delta_settings) for k,iface in self._partInterfaces.items()),self._delta_settings)
        _update_from_settings(self._base,self._settings,self._delta_settings)
        _update_parts_from_settings(self._partInterfaces,self._structure.parts,self._settings,self._delta_settings)
        self._delta_settings = None
        #update child settings
        sub_settings = _split_settings(self._settings,self._structure.parts)
        for (part,iface) in self._partInterfaces.items():
            iface._settings = sub_settings[part]

        #read through command queue
        for part,iface in self._partInterfaces.items(): 
            for cmd in iface._commands:
                try:
                    getattr(iface,cmd.func)(*cmd.args)
                except Exception as e:
                    if cmd.func in ['reset','estop','softStop']:   #bubble-up
                        getattr(self._base,cmd.func)(*cmd.args)
                    else:
                        cmd.promise.errback(e)   #TODO: errbacks can add to command queue
                        errors.append(e)
            iface._commands = []
            iface._in_step = False
        for cmd in self._commands:
            if cmd.indices is not None and len(cmd.indices) != self._structure.numJoints:
                errors.append("Invalid number of indices")
            try:
                getattr(self._base,cmd.func)(*cmd.args)
            except Exception as e:
                cmd.promise.errback(e)   #TODO: errbacks can add to command queue
                errors.append(e)
            
        #finish
        self._commands = []
        self._try('endStep',())
        self._in_step = False
        if errors:
            raise RuntimeError("Runtime errors: {}".format(','.join(str(e) for e in errors)))
    
    def partInterface(self,part,joint_idx=None):
        if joint_idx is None:
            if part not in self._partInterfaces:
                raise RuntimeError("Tried to retrieve part interface without initializing")
            return self._partInterfaces[part]
        raise NotImplementedError("TODO: single-joint interfaces")

    def jointName(self,joint_idx):
        res = self._try('jointName',[joint_idx])
        if res is not None: return res
        robot = self.klamptModel()
        if robot is None:
            return 'Joint '+str(joint_idx)
        return robot.link(robot.driver(joint_idx).getAffectedLink()).getName()
    
    def setControlMode(self,mode,*args,**kwargs):
        self._queueCommand('setControlMode',[mode,args,kwargs])

    def functionCall(self,proc,*args,**kwargs):
        self._base.functionCall(proc,*args,**kwargs)

    def cartesianPosition(self,q,frame='world'):
        return self._try('cartesianPosition',[q,frame])

    def cartesianVelocity(self,q,dq,frame='world'):
        return self._try('cartesianVelocity',[q,dq,frame])

    def cartesianForce(self,q,t,frame='world'):
        return self._try('cartesianPosition',[q,t,frame])

    def klamptModel(self):
        return self._base.klamptModel()

    def configFromKlampt(self,klamptConfig,part=None,joint_idx=None):
        return self._base.configFromKlampt(klamptConfig,part=None,joint_idx=None)

    def velocityFromKlampt(self,klamptVelocity,part=None,joint_idx=None):
        return self._base.velocityFromKlampt(klamptVelocity,part=None,joint_idx=None)

    def configToKlampt(self,config,klamptConfig=None,part=None,joint_idx=None):
        return self._base.configToKlampt(config,klamptConfig,part=None,joint_idx=None)

    def velocityToKlampt(self,velocity,klamptVelocity=None,part=None,joint_idx=None):
        return self._base.velocityToKlampt(velocity,klamptVelocity=None,part=None,joint_idx=None)


class OmniRobotInterface(_RobotInterfaceStatefulBase):
    """A RobotInterfaceBase that aggregates one or more parts, which are
    communicated with separately.  For example, a mobile manipulator can
    consist of a base, arms, and grippers.

    On startup, call ``addPhysicalPart(name,driver_indices,interface)`` for
    each part of the robot.  ``driver_indices`` must correspond to the indices
    of drivers corresponding to that part in the Klamp't model.

    The total configuration for the robot is a list of floats, segmented into
    physical parts.  The ordering of the parts' configurations, velocities, etc
    is given by the driver_indices as they were added.

    Allows for complex arrangements like a physical part that has sub-parts,
    such as an arm with an integrated gripper handled with the same physical
    interface.  Sub-parts are named 'PARENT__CHILD'.

    TODO: handle more than one layer of nesting.

    For example, if you want to assemble a unified interface / klamptModel
    from two interfaces, with part2 (e.g., a gripper) mounted on part1
    (e.g., an arm), you'd run::

        voltron_controller = OmniRobotInterface(klampt_model)
        voltron_controller.addPhysicalPart("part1",part1_interface,part1_indices)
        voltron_controller.addPhysicalPart("part2",part2_interface,part2_indices)
        if not voltron_controller.initailize():
            print("hmm... error on initialize?")
        ... do stuff, treating voltron_controller as a unified robot ...

    If you have separate parts with their own Klamp't models, and wish to
    dynamically construct a unified model, you may use the ``mount`` command
    as follows::

        world = WorldModel()
        world.readFile(part1_interface.properties['klamptModelFile'])
        world.readFile(part2_interface.properties['klamptModelFile'])
        part1_model = world.robot(0)
        part2_model = world.robot(1)
        ee_link = part1_model.numLinks()-1
        mount_rotation = ... some so3 element...
        mount_translation = ... some translation vector...
        part1_model.mount(ee_link,part2_model,mount_rotation,mount_translation)  #part1 model is now the whole robot
        world.remove(part2_model)  #not strictly necessary to remove the model from the world...

        klampt_model = world.robot(0)
        part1_indices = list(range(part1_model.numDrivers()))
        part2_indices = list(range(part1_model.numDrivers(),part1_model.numDrivers()+part2_model.numDrivers()))

        #code above should follow

    If your robot has a RobotInfo structure associated with it, you'd point its
    ``controllerFile`` attribute to a file containing code like::

        from klampt.control.robotinterfaceutils import OmniRobotInterface
        #import Part1RobotInterface and Part2RobotInterface from somewhere

        def make(robotModel):
            voltron_controller = OmniRobotInterface(robotModel)
            part1_controller = Part1RobotInterface()
            part2_controller = Part2RobotInterface()
            n1 = part1_controller.numJoints()
            n2 = part2_controller.numJoints()
            assert n1 + n2 == robotModel.numDrivers()
            voltron_controller.addPhysicalPart("part1",part1_controller,list(range(n1)))
            voltron_controller.addPhysicalPart("part2",part2_controller,list(range(n1,n1+n2)))
            #add any virtual parts if your RobotInfo defines them
            return voltron_controller
    
    """
    def __init__(self, klamptModel : RobotModel=None):
        _RobotInterfaceStatefulBase.__init__(self)
        self._structure.parts = dict()
        if klamptModel is None:
            self._structure.numJoints = 0
            self._structure.parts[None] = []
        else:
            self._structure.klamptModel = klamptModel
            self._structure.numJoints = klamptModel.numDrivers()
            self._structure.parts[None] = list(range(self._structure.numJoints))
        self._physicalParts = dict()         # type: Dict[str,_RobotInterfaceStatefulWrapper]
        self._physicalPartIndices = dict()   # type: Dict[str,List[int]]
        self._physicalPartUpdateTimes = dict()  # type: Dict[str,float]
        self._childPhysicalParts = dict()    # type: Dict[str,_RobotInterfaceStatefulWrapper]
        self._childPhysicalPartIndices = dict() # type: Dict[str,List[int]]
        self._virtualParts = dict()          # type: Dict[str,_RobotInterfaceStatefulBase]
        self._virtualPartIndices = dict()    # type: Dict[str,List[int]]
        self._emulator = None
        self._filters = dict()               # type: Dict[str,Dict[str,Callable]]
        self._in_step = False
        del self._partInterfaces
        self.properties['complete'] = True

    def addPhysicalPart(self, name : str, interface : RobotInterfaceBase, indices : Sequence[int]=None) -> None:
        """Adds a new part corresponding to a physical RIL implementation.
        
        If klamptModel was not provided to the initializer, then this part will
        be appended to the list of parts, and indices=None is allowed. 
        Otherwise, indices needs to be a list of driver indices corresponding
        to the part in klamptModel.
        """
        if name in self._structure.parts:
            raise ValueError("Part named {} already exists".format(name))
        n = self.numJoints()
        if indices is None:
            if self._structure.klamptModel is not None:
                raise ValueError("Must specify indices for a physical part when a unified klamptModel has been defined")
            try:
                ni = interface.numJoints()
            except NotImplementedError:
                raise ValueError("When building up an OmniRobotInterface, the physical interface must implement numJoints()")
            indices = list(range(n,n+ni))
            self._structure.numJoints += ni
            self._structure.parts[None] = list(range(self._structure.numJoints))
        else:
            if self._structure.klamptModel is None:
                if any(i < n for i in indices):
                    raise ValueError("When building up an OmniRobotInterface, all indices must be contiguous and added in order")
            else:
                for i in indices:
                    assert i >= 0 and i < n,"Invalid index {} must be in range [0,{}]".format(i,n-1)
        if not isinstance(interface,_RobotInterfaceStatefulWrapper):
            interface = _RobotInterfaceStatefulWrapper(interface)
        self._physicalParts[name] = interface
        interface.properties['name'] = name
        interface.properties['part'] = True
        self._physicalPartIndices[name] = indices
        self._physicalPartUpdateTimes[name] = 0
        self._structure.parts[name] = self._physicalPartIndices[name]
        
    def addVirtualPart(self, name : str, indices : Sequence[int]) -> None:
        if name in self._structure.parts:
            raise ValueError("Part named {} already exists".format(name))
        n = self.numJoints()
        for i in indices:
            assert i >= 0 and i < n,"Invalid index {} must be in range [0,{}]".format(i,n-1)
        interface = _RobotInterfaceStatefulBase()
        interface.properties['name'] = name
        interface.properties['part'] = True
        interface._structure.controlRate = self._structure.controlRate
        interface._structure.numJoints = len(indices)
        interface._structure.parts = dict()
        interface._structure.parts[None] = list(range(interface._structure.numJoints))
        self._virtualParts[name] = interface
        self._virtualPartIndices[name] = indices
        self._structure.parts[name] = self._virtualPartIndices[name]
    
    def setFilter(self, name : str, filter : Optional[Callable], input='auto', output='auto') -> None:
        """Enables or disables a filter. The filter is attached to the given
        name, which can be a standard filter item or sensor.

        Standard filters include:

        * 'joint_limit': adjusts position commands
        * 'velocity_limit': adjusts velocity commands
        * 'acceleration_limit': adjusts acceleration commands
        * 'torque_limit': adjusts torque commands
        * 'tracking_monitor': monitors sensed vs commanded positions
        * 'self_collision': adjusts position commands to avoid collisions
        * 'obstacle_collision': adjusts position commands to avoid collisions
        * SENSOR_NAME: some sensor for which ``hasSensor(SENSOR_NAME)=True``.

        Valid operands include:

        * 'sensedPosition', 'sensedVelocity', 'sensedAcceleration', and
          'sensedTorque': updated on every time step
        * 'commandedPosition', 'commandedVelocity', 'commandedAcceleration',
          and 'commandedTorque': updated on every time step
        * 'positionCommand', 'velocityCommand', 'torqueCommand': updated when a
          position, velocity, or torque command are scheduled for sending to the
          robot. 
        * SENSOR_NAME: some sensor for which ``hasSensor(SENSOR_NAME)=True``.
        * 'status': the input / output status

        Args:
            name (str): the filter identifier
            filter (Callable, or None): the filter operator. If None, disables
                the filter. 
                
                Otherwise, the filter is callable and takes #inputs arguments
                (usually 1) and returns #outputs arguments to be stored in the
                output (a tuple if #outputs > 1). 
                
                Note that to implement a stateful filter, you would create a
                class that updates internal state on __call__. 
                RobotInterfaceCompleter also accepts
                :class:`klampt.control.blocks.Block` objects.
            input (str or list of str): one or more input operands.  If 'auto',
                the input operands are auto-determined from the name.
            output (str or list of str): one or more output operands.  If
                'auto', the output operands are auto-determined from the name.
        """
        self._emulator.setFilter(self.indices(), name, filter, input, output)

    def setPartFilter(self, part : str, name : str, filter : Optional[Callable], input='auto', output='auto') -> None:
        self._emulator.setFilter(self.indices(part), name, filter, input, output)

    def setJointLimits(self, qmin='auto', qmax='auto', op='clamp'):
        """Activates a joint limit filter.
        
        If qmin/qmax are 'auto', these are read from the klampt robot model
        or the properties.

        If op is...

        * 'clamp' then commands are silently clamped to their limits.
        * 'stop' a soft-stop is raised.
        * 'warn' a warning is printed and the robot silently ignores the
          command.

        """
        #need to limit to hardware values
        hw_qmin,hw_qmax = self.properties.get('joint_limits',(None,None))
        self._emulator.setJointLimits(self.indices(),qmin,qmax,op,hw_qmin,hw_qmax)
    
    def setPartJointLimits(self, part : str, qmin='auto', qmax='auto', op='clamp'):
        """Activates a joint limit filter.
        
        If qmin/qmax are 'auto', these are read from the klampt robot model
        or the properties.

        If op is...

        * 'clamp' then commands are silently clamped to their limits.
        * 'stop' a soft-stop is raised.
        * 'warn' a warning is printed and the robot silently ignores the
          command.

        """
        indices = self.indices(part)
        #need to limit to hardware values
        hw_qmin,hw_qmax = self.properties.get('joint_limits',(None,None))
        if hw_qmin is not None:
            hw_qmin = [hw_qmin[i] for i in indices]
            hw_qmax = [hw_qmax[i] for i in indices]
        self._emulator.setJointLimits(indices,qmin,qmax,op,hw_qmin,hw_qmax)
    
    def setCollisionFilter(self, world : WorldModel=None, op = 'warn'):
        """Activates a collision filter.
        
        If world=None, only does self collision. Otherwise, does self and
        world collision.

        If op = 'stop', a soft stop is raised if a collision is predicted.

        If op = 'warn', a warning is printed and the robot silently ignores
        the command.
        
        """
        self._emulator.setCollisionFilter(world,op)
    
    def initialize(self) -> bool:
        if len(self._physicalParts) == 0:
            warnings.warn("No physical parts defined")
            return False
        for k,iface in self._virtualParts.items():
            link_inds = []
            for i in self._virtualPartIndices[k]:
                link_inds += self._structure.klamptModel.driver(i).getAffectedLinks()
            iface._structure.klamptModel = SubRobotModel(self._structure.klamptModel,link_inds)
        for k,iface in self._physicalParts.items():
            link_inds = []
            for i in self._physicalPartIndices[k]:
                link_inds += self._structure.klamptModel.driver(i).getAffectedLinks()
            iface._structure.klamptModel = SubRobotModel(self._structure.klamptModel,link_inds)
        
        n = self._structure.klamptModel.numDrivers()
        self._structure.numJoints = n
        self._structure.jointNames = [None]*n
        
        #initialize physical interfaces
        controlled = [False]*n
        for (k,indices) in self._physicalPartIndices.items():
            for i in indices:
                controlled[i] = True
        if not all(controlled):
            warnings.error("Not all joints are covered by a physical interface")
            return False
        for (k,iface) in self._physicalParts.items():
            if not iface.initialize():
                return False
            
            #update the structure
            pindices = self._physicalPartIndices[k]
            ni = len(pindices)
            for i in range(ni):
                try:
                    ji = iface.jointName(i)
                except NotImplementedError:
                    ji = self._structure.klamptModel.link(self._structure.klamptModel.driver(pindices[i]).getAffectedLink()).getName()
                self._structure.jointNames[pindices[i]] = ji
            #take the max control rate
            if self._structure.controlRate is None:
                self._structure.controlRate = iface._structure.controlRate
            elif iface._structure.controlRate is not None:
                self._structure.controlRate = max(self._structure.controlRate,iface._structure.controlRate)

            #discover child physical interfaces
            for sub_part,sub_indices in iface.parts().items():
                if sub_part is not None:
                    sub_iface = iface.partInterface(sub_part)
                    my_indices = [indices[i] for i in sub_indices]
                    self._childPhysicalParts[k+PART_NAME_SEPARATOR+sub_part] = sub_iface
                    self._childPhysicalPartIndices[k+PART_NAME_SEPARATOR+sub_part] = my_indices
                    link_indices = []
                    for i in my_indices:
                        link_indices += self._structure.klamptModel.driver(i).getAffectedIndices()
                    sub_iface.klamptModel = SubRobotModel(self._structure.klamptModel,link_indices)
        
        indices = dict()
        indices.update(self._physicalPartIndices)
        indices.update(self._childPhysicalPartIndices)
        indices.update(self._virtualPartIndices)
        settings = dict()
        for (k,iface) in self._physicalParts.items():
            settings[k] = iface._settings
        for (k,iface) in self._childPhysicalParts.items():
            settings[k] = iface._settings
        for (k,iface) in self._virtualParts.items():
            settings[k] = iface._settings
        _gather_settings(indices,settings,self._settings)
        #initialize emulator with settings
        self._emulator = RobotInterfaceEmulator(n,self._structure.klamptModel)
        if self._settings.kP is not None and self._settings.kI is not None and self._settings.kD is not None:
            self._emulator.setPIDGains(self._settings.kP,self._settings.kI,self._settings.kD)
        all_indices = list(range(self.numJoints()))
        if self._settings.toolCoordinates is not None:
            self._emulator.setToolCoordinates(all_indices,self._settings.toolCoordinates)
        if self._settings.gravity is not None:
            self._emulator.setGravityCompensation(all_indices,self._settings.gravity,self._settings.load,self._settings.load_com)
        for (k,settings) in self._settings.children.items():
            assert k in self._structure.parts
            if settings.toolCoordinates is not None:
                self._emulator.setToolCoordinates(self._structure.parts[k],settings.toolCoordinates)
            if settings.gravity is not None:
                self._emulator.setGravityCompensation(self._structure.parts[k],settings.gravity,settings.load,settings.load_com)
        #can only check filters at this point
        for (k,iface) in self._physicalParts.items():
            self._emulator._validateFilters(iface,self._physicalPartIndices[k])
        for (k,iface) in self._childPhysicalParts.items():
            self._emulator._validateFilters(iface,self._childPhysicalPartIndices[k])
        for (k,iface) in self._virtualParts.items():
            self._emulator._validateFilters(iface,self._virtualPartIndices[k])
        #distribute structure and settings across virtual parts -- physical and child physical parts are already updated
        settings = _split_settings(self._settings,self._virtualPartIndices)
        for (k,iface) in self._virtualParts.items():
            iface._settings = settings[k]
            iface._structure.jointNames = [self._structure.jointNames[i] for i in self._virtualPartIndices[k]]
            link_indices = []
            for i in self._virtualPartIndices[k]:
                link_indices += self._structure.klamptModel.driver(i).getAffectedLinks()
            iface._structure.klamptModel = SubRobotModel(self._structure.klamptModel,link_indices)
        return True

    def beginStep(self):
        if self._in_step:
            raise RuntimeError("beginStep() called without prior endStep()")
        
        for (k,iface) in self._physicalParts.items():
            iface.beginStep()
        
        try:
            prev_state = self._state
            self._state = _RobotInterfaceState()
            _gather_state(self._physicalPartIndices,dict((k,iface._state) for (k,iface) in self._physicalParts.items()),self._state)
            
            #initialize emulated state on first try
            assert self._structure.controlRate is not None or self._state.rate is not None or self._state.clock is not None,"None of the physical parts implement a clock or rate"
            assert self._state.sensedPosition is not None or self._state.commandedPosition is not None
            if self._emulator.numUpdates < 0:
                if self._state.sensedPosition is None:
                    self._state.sensedPosition = copy.copy(self._state.commandedPosition)
                if self._state.commandedPosition is None:
                    self._state.commandedPosition = copy.copy(self._state.sensedPosition)
                if self._state.commandedVelocity is None:
                    self._state.commandedVelocity = copy.copy(self._state.sensedVelocity)
                if self._state.commandedTorque is None:
                    self._state.commandedTorque = copy.copy(self._state.sensedTorque)
                self._emulator.initialize(self._state.sensedPosition,self._state.sensedVelocity,self._state.sensedTorque,
                    self._state.commandedPosition,self._state.commandedVelocity,self._state.commandedTorque)
            else:
                #on later steps, fill parts of state that are not updated by physical parts with emulated state
                self._updateStateFromEmulator('commandedPosition')
                self._updateStateFromEmulator('commandedVelocity')
                self._updateStateFromEmulator('commandedTorque')
                self._updateStateFromEmulator('destinationPosition')
                self._updateStateFromEmulator('destinationVelocity')
                if self._state.children is not None:
                    for (k,substate) in self._state.children.items():
                        substate.cartesianState = None
                for (k,indices) in self._structure.parts.items():
                    if tuple(indices) in self._emulator.cartesianInterfaces:
                        if self._state.children is None:
                            self._state.children = dict()
                        if k not in self._state.children:
                            self._state.children[k] = _RobotInterfacePartState()
                        self._state.children[k].cartesianState = self._updateCartesianStateFromEmulator(k)
            rate = self._state.rate
            if rate is None: rate = self._structure.controlRate
            self._emulator.advanceClock(self._state.clock,rate)
            if self._state.status == 'ok':
                self._state.status = self._emulator.status
            else:
                self._emulator.status = self._state.status
            if self._state.clock is None:
                self._state.clock = self._emulator.curClock
            if self._state.rate is None:
                self._state.rate = 1.0/self._emulator.dt
            
            #distribute state across physical parts
            states = _split_state(self._state,self._physicalPartIndices)
            for (k,iface) in self._physicalParts.items():
                iface._state = states[k]
            #distribute states across child physical parts
            states = _split_state(self._state,self._childPhysicalPartIndices)
            for (k,iface) in self._childPhysicalParts.items():
                iface._state = states[k]
            #distribute state across virtual parts
            states = _split_state(self._state,self._virtualPartIndices)
            for (k,iface) in self._virtualParts.items():
                iface._state = states[k]

            #initialize _delta_settings for this and all parts
            self._delta_settings = _RobotInterfaceSettings()
            for (k,iface) in self._physicalParts.items():
                iface._delta_settings = _RobotInterfaceSettings()
            for (k,iface) in self._childPhysicalParts.items():
                iface._delta_settings = _RobotInterfaceSettings()
            for (k,iface) in self._virtualParts.items():
                iface._delta_settings = _RobotInterfaceSettings()
        except Exception:
            for (k,iface) in self._physicalParts.items():
                iface.endStep()
            raise
        self._in_step = True        
    
    def endStep(self):
        if not self._in_step:
            raise RuntimeError("endStep() called without corresponding beginStep()")
        #check settings
        indices = dict()
        indices.update(self._physicalPartIndices)
        indices.update(self._childPhysicalPartIndices)
        indices.update(self._virtualPartIndices)
        delta_settings = dict()
        for (k,iface) in self._physicalParts.items():
            delta_settings[k] = iface._delta_settings
        for (k,iface) in self._childPhysicalParts.items():
            delta_settings[k] = iface._delta_settings
        for (k,iface) in self._virtualParts.items():
            delta_settings[k] = iface._delta_settings
        _gather_settings(indices,delta_settings,self._delta_settings)

        #parse settings for virtual parts to determine whether new cartesian controllers have been created
        #update emulator settings to match all parts as well
        all_indices = list(range(self.numJoints()))
        if self._delta_settings.toolCoordinates is not None:
            self._emulator.setToolCoordinates(all_indices,self._delta_settings.toolCoordinates)
        if self._delta_settings.gravity is not None:
            self._emulator.setGravityCompensation(all_indices,self._delta_settings.gravity,self._delta_settings.load,self._delta_settings.load_com)
        if self._delta_settings.children is not None:
            for k,settings in self._delta_settings.children.items():
                if settings.toolCoordinates is not None:
                    self._emulator.setToolCoordinates(self._structure.parts[k],settings.toolCoordinates)
                if settings.gravity is not None:
                    self._emulator.setGravityCompensation(self._structure.parts[k],settings.gravity,settings.load,settings.load_com)
        #TODO: gather prev / current changes of PID 

        #distribute changed settings across physical interfaces
        delta_settings = _split_settings(self._delta_settings,self._physicalPartIndices)
        for (k,iface) in self._physicalParts.items():
            _update_from_settings(iface,iface._settings,delta_settings[k])
        delta_settings = _split_settings(self._delta_settings,self._childPhysicalPartIndices)
        for (k,iface) in self._childPhysicalParts.items():
            _update_from_settings(None,iface._settings,delta_settings[k])
        delta_settings = _split_settings(self._delta_settings,self._virtualPartIndices)
        for (k,iface) in self._virtualParts.items():
            _update_from_settings(None,iface._settings,delta_settings[k])
        self._delta_settings = None

        #gather commands
        names = [None]
        indices = [self.indices()]
        commands = [self._commands]
        for (k,iface) in self._physicalParts.items():
            names.append(k)
            indices.append(self._physicalPartIndices[k])
            commands.append([])
            for cmd in iface._commands:
                if cmd.func in ['setControlMode','functionCall']:  #custom commands
                    getattr(iface,cmd.func)(cmd.args[0],*cmd.args[1],**cmd.args[2])
                    if cmd.func == 'setControlMode':
                        self._emulator.promote(cmd.indices,None)
                else:
                    commands[-1].append(cmd)
            iface._commands = []   #erase primary commands
        for (k,iface) in self._childPhysicalParts.items():
            names.append(k)
            indices.append(self._childPhysicalPartIndices[k])
            commands.append([])
            for cmd in iface._commands:
                if cmd.func in ['setControlMode','functionCall']:  #custom commands
                    getattr(iface,cmd.func)(cmd.args[0],*cmd.args[1],**cmd.args[2])
                    if cmd.func == 'setControlMode':
                        self._emulator.promote(cmd.indices,None)
                else:
                    commands[-1].append(cmd)
            iface._commands = []   #erase primary commands
        for (k,iface) in self._virtualParts.items():
            names.append(k)
            indices.append(self._virtualPartIndices[k])
            commands.append(iface._commands)
            iface._commands = []   #erase primary commands
        self._commands = _gather_commands(commands,indices)
        #set commands to emulator, advance
        for cmd in self._commands:
            if cmd.func in ['reset']:
                getattr(self._emulator,cmd.func)()
            else:
                if cmd.indices is None:  #command to everything
                    getattr(self._emulator,cmd.func)(all_indices,*cmd.args)
                else:
                    getattr(self._emulator,cmd.func)(cmd.indices,*cmd.args)
        
        self._emulator.update(self._state.sensedPosition,self._state.sensedVelocity)

        #distribute to parts
        for (k,iface) in self._physicalParts.items():
            inds = self._physicalPartIndices[k]
            #figure out commands to actually send to the physical parts based on the desired control
            #modes of the joints
            (desiredControlMode,baseControlMode) = self._emulator.sendToInterface(iface,inds)
            #if baseControlMode is not None:
            #    print("Controller",k,iface,"desired control mode",desiredControlMode,"chosen",baseControlMode)
            #TODO: when a value is emulated, set a flag to use emulated values rather than base
            
        for (k,iface) in self._physicalParts.items():
            iface.endStep()
            iface._in_step = False
        for (k,iface) in self._childPhysicalParts.items():
            iface._commands = []
            iface._in_step = False
        for (k,iface) in self._virtualParts.items():
            iface._commands = []

        self._commands = []
        self._in_step = False

    def _updateStateFromEmulator(self,attr):
        stateval = getattr(self._state,attr)
        if stateval is None:
            value = getattr(self._emulator,attr)()
            if value is None: return
            if all(v is None for v in value): return
            setattr(self._state,attr,value)
            #assert all(v is not None for v in value),"Emulator returned a vector with some Nones for attribute {}: {}?".format(attr,value)
        elif any(v is None for v in stateval):
            value = getattr(self._emulator,attr)()
            if value is None: return
            if all(v is None for v in value): return
            #assert all(v is not None for v in value),"Emulator returned a vector with some Nones for attribute {}: {}?".format(attr,value)
            for i,v in enumerate(stateval):
                if v is None:
                    stateval[i] = value[i]

    def _updateCartesianStateFromEmulator(self,part):
        state = _RobotInterfaceCartesianState()
        indices = self._structure.parts[part]
        state.baseSensedPosition = se3.identity()
        state.baseCommandedPosition = se3.identity()
        state.baseDestinationPosition = se3.identity()
        state.baseSensedVelocity = ([0]*3,[0]*3)
        state.baseCommandedVelocity = ([0]*3,[0]*3)
        state.baseDestinationVelocity = ([0]*3,[0]*3)
        first_index = self.klamptModel().driver(indices[0]).getAffectedLink()
        base_index = self.klamptModel().link(first_index).getParent()
        if base_index >= 0:
            if self._state.sensedPosition is not None:
                state.baseSensedPosition = self._emulator.cartesianPosition(base_index,self._state.sensedPosition,'world')
                if self._state.sensedVelocity is not None:
                    state.baseSensedVelocity = self._emulator.cartesianVelocity(base_index,self._state.sensedPosition,self._state.sensedVelocity,'world')
            if self._state.commandedPosition is not None:
                state.baseCommandedPosition = self._emulator.cartesianPosition(base_index,self._state.commandedPosition,'world')
                if self._state.commandedVelocity is not None:
                    state.baseCommandedVelocity = self._emulator.cartesianVelocity(base_index,self._state.commandedPosition,self._state.commandedVelocity,'world')
            if self._state.destinationPosition is not None:
                state.baseDestinationPosition = self._emulator.cartesianPosition(base_index,self._state.destinationPosition,'world')
                if self._state.destinationVelocity is not None:
                    state.baseDestinationVelocity = self._emulator.cartesianVelocity(base_index,self._state.destinationPosition,self._state.destinationVelocity,'world')
        assert self._state.sensedPosition is not None
        assert self._state.commandedPosition is not None
        state.sensedPosition = self._emulator.cartesianPosition(indices,self._state.sensedPosition,'base')
        if self._state.sensedVelocity is not None:
            state.sensedVelocity = self._emulator.cartesianVelocity(indices,self._state.sensedPosition,self._state.sensedVelocity,'base')
        if self._state.sensedTorque is not None:
            state.sensedForce = self._emulator.cartesianForce(indices,self._state.sensedPosition,self._state.sensedTorque,'base')
        state.commandedPosition = self._emulator.cartesianPosition(indices,self._state.commandedPosition,'base')
        if self._state.commandedVelocity is not None:
            state.commandedVelocity = self._emulator.cartesianVelocity(indices,self._state.commandedPosition,self._state.commandedVelocity,'base')
        if self._state.commandedTorque is not None:
            state.commandedForce = self._emulator.cartesianForce(indices,self._state.commandedPosition,self._state.commandedTorque,'base')
        if self._state.destinationPosition is not None:
            state.destinationPosition = self._emulator.cartesianPosition(indices,self._state.destinationPosition,'base')
            if self._state.destinationVelocity is not None:
                state.destinationVelocity = self._emulator.cartesianVelocity(indices,self._state.destinationPosition,self._state.destinationVelocity,'base')
        # print("Tool coordinates",self._emulator.getToolCoordinates(indices))
        # print("Commanded base",state.baseCommandedPosition)
        # print("Commanded position w.r.t. base",state.commandedPosition)
        return state

    def partInterface(self,part,joint_idx = None):
        if part in self._physicalParts:
            iface = self._physicalParts[part]
        elif part in self._childPhysicalParts:
            iface = self._childPhysicalParts[part]
        elif part in self._virtualParts:
            iface = self._virtualParts[part]
        elif part is None:
            if joint_idx is not None:
                raise NotImplementedError("Single-joint interfaces")
            return self
        else:
            raise ValueError("Invalid part name {}".format(part))
        if joint_idx is not None:
            return iface.partInterface(None,joint_idx)
        return iface

    def reset(self):
        for part,iface in self._physicalParts.items():
            if not iface.reset():
                return False
        return True



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
        - Any sensors
        - :meth:`functionCall`
        - :meth:`setSetting` and `getSetting`
        - :meth:`stateValue`
        - :meth:`klamptModel`

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
        - E-stopping and soft stopping requiring reset.

    .. note::
        The base interface's klamptModel() method must be implemented for
        Cartesian control and acceleration-bounded control to work properly.

    Args:
        base_interface (RobotInterfaceBase): the partially-implemented
            interface.
        base_initialized (bool, optional): if True, initialize() will not
            initialize the base.

    """
    def __init__(self,base_interface : RobotInterfaceBase, base_initialized=False):
        RobotInterfaceBase.__init__(self)
        self._base = base_interface
        self._baseInitialized  = base_initialized
        self._parent = None
        self._subRobot = False
        self._parts = dict()
        self._baseParts = None
        self._has = dict()
        self._indices = None
        self._emulator = None                       #type: RobotInterfaceEmulator
        self._emulatorControlMode = None
        self._baseControlMode = None
        self._inStep = False
        self.properties = base_interface.properties.copy()
        self.properties['complete'] = True

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
        assert name not in self._parts
        self._parts[name] = indices

    def _try(self,fn,args,fallback=None):
        return _try_methods(self._base,fn,args,fallback,self._has)

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
        n = self._base.numJoints()
        for (part,indices) in self._parts.items():
            if any(i<0 or i >= n for i in indices):
                warnings.warn("RobotInterfaceCompleter(%s): part has invalid index %d (#joints=%d)"%(str(self._base),max(indices),n))
        self._baseParts = self._base.parts()
        self._parts.update(self._baseParts)
        self._indices = self.indices()
        self._try('beginStep',(),0)
        self._try('controlRate',(),0)
        curclock = self._try('clock',(),0)
        self._try('sensedPosition',(),0)
        self._try('sensedVelocity',(),0)
        self._try('sensedTorque',(),0)
        self._try('commandedPosition',(),0)
        self._try('commandedVelocity',(),0)
        self._try('commandedTorque',(),0)
        self._try('destinationPosition',(),0)
        self._try('destinationVelocity',(),0)
        self._try('destinationTime',(),0)
        self._try('queuedTrajectory',(),0)
        self._try('endStep',(),0)
        if not self._has['controlRate'] and not self._has['clock']:
            warnings.warn("RobotInterfaceCompleter(%s): Need at least one of controlRate() and clock() to be implemented"%(str(self._base),))
            return False
        if not self._has['sensedPosition'] and not self._has['commandedPosition']:
            warnings.warn("RobotInterfaceCompleter(%s): Need at least one of sensedPosition() and commandedPosition() to be implemented"%(str(self._base),))
            return False
        self._emulator = RobotInterfaceEmulator(self._base.numJoints(),self._base.klamptModel())
        self._emulator.curClock = curclock
        print("RobotInterfaceCompleter({}): Starting with clock {}".format(self._base,curclock))
        assert curclock is not None

        self._warned = False
        return True

    def close(self):
        self._base_initialized=False
        return self._base.close()

    def startStep(self):
        if not self._warned:
            warnings.warn("startStep is deprecated, use beginStep",DeprecationWarning)
        self._warned = True
        self.beginStep()

    def beginStep(self):
        assert self._indices is not None,"RobotInterface not initialized yet"
        assert not self._subRobot,"Can't do beginStep on a sub-interface"
        assert not self._inStep,"beginStep called twice in a row?"
        self._inStep = True
        self._try('beginStep',(),0)
        if self._emulator.numUpdates < 0:
            qsns = self._try('sensedPosition',(),lambda *args:None)
            vsns = self._try('sensedVelocity',(),lambda *args:None)
            tsns = self._try('sensedTorque',(),lambda *args:None)
            qcmd = self._try('commandedPosition',(),lambda *args:None)
            vcmd = self._try('commandedVelocity',(),lambda *args:None)
            tcmd = self._try('commandedTorque',(),lambda *args:None)
            self._emulator.initialize(qsns,vsns,tsns,qcmd,vcmd,tcmd)
        if not self._has['controlRate']:
            self._emulator.advanceClock(self.clock(),None)
        elif not self._has['clock']:
            self._emulator.advanceClock(None,self.controlRate())
        else:
            self._emulator.advanceClock(self.clock(),self.controlRate())

    def endStep(self):
        assert not self._subRobot,"Can't do endStep on a sub-interface"
        assert self._inStep,"endStep called outside of a step?"
        self._inStep = False

        q = self.sensedPosition()
        v = self.sensedVelocity()
        try:
            self._emulator.update(q,v)
        except Exception as e:
            self.softStop()
            raise e
        
        self._emulatorControlMode,self._baseControlMode = self._emulator.sendToInterface(self._base)
        self._try('endStep',(),0)

    def controlRate(self):
        def _controlRate_backup(self):
            if self._emulator.dt is None:
                raise RuntimeError("Base interface doesn't have controlRate, and need to have at least one time step before determining backup controlRate")
            return 1.0/self._emulator.dt
        return self._try('controlRate',(),lambda :_controlRate_backup(self))

    def clock(self):
        return self._try('clock',(),self._emulator.curClock)

    def estop(self):
        """Requires reset"""
        self._base.estop()
        self._emulator.estop(self._indices)

    def softStop(self):
        """Requires reset"""
        self._base.softStop()
        self._emulator.softStop(self._indices)

    def reset(self):
        if self._emulator.status != 'ok':
            self._emulator.reset()
        if self._base.status() != 'ok':
            return self._base.reset()
        return True

    def setSetting(self,key,value):
        self._base.setSetting(key,value)
        
    def getSetting(self,key):
        return self._base.getSetting(key)

    def partInterface(self,part,joint_idx=None):
        return _SubRobotInterfaceCompleter(self,part,joint_idx)

    def jointName(self,joint_idx):
        def modelJointName(idx):
            robot = self.klamptModel()
            if robot is None:
                return 'Joint '+str(idx)
            return robot.link(robot.driver(idx).getAffectedLink()).getName()
        return self._try('jointName',[joint_idx],modelJointName)

    def setControlMode(self,mode,*args,**kwargs):
        if self._emulator.status == 'ok':
            self._emulator.promote(self._indices,None)   #stop
            self._base.setControlMode(mode,*args,**kwargs)
    
    def functionCall(self,proc,*args,**kwargs):
        return self._base.functionCall(proc,*args,**kwargs)
    
    def setFilter(self, name : str, filter : Optional[Callable], input='auto', output='auto') -> None:
        """Enables or disables a filter. The filter is attached to the given
        name, which can be a standard filter item or sensor.

        Standard filters include:

        * 'joint_limit': adjusts position commands
        * 'velocity_limit': adjusts velocity commands
        * 'acceleration_limit': adjusts acceleration commands
        * 'torque_limit': adjusts torque commands
        * 'tracking_monitor': monitors sensed vs commanded positions
        * 'self_collision': adjusts position commands to avoid collisions
        * 'obstacle_collision': adjusts position commands to avoid collisions
        * SENSOR_NAME: some sensor for which ``hasSensor(SENSOR_NAME)=True``.

        Valid operands include:

        * 'sensedPosition', 'sensedVelocity', 'sensedAcceleration', and
          'sensedTorque': updated on every time step
        * 'commandedPosition', 'commandedVelocity', 'commandedAcceleration',
          and 'commandedTorque': updated on every time step
        * 'positionCommand', 'velocityCommand', 'torqueCommand': updated when a
          position, velocity, or torque command are scheduled for sending to the
          robot. 
        * SENSOR_NAME: some sensor for which ``hasSensor(SENSOR_NAME)=True``.
        * 'status': the input / output status

        Args:
            name (str): the filter identifier
            filter (Callable, or None): the filter operator. If None, disables
                the filter. 
                
                Otherwise, the filter is callable and takes #inputs arguments
                (usually 1) and returns #outputs arguments to be stored in the
                output (a tuple if #outputs > 1). 
                
                Note that to implement a stateful filter, you would create a
                class that updates internal state on __call__. 
                RobotInterfaceCompleter also accepts
                :class:`klampt.control.blocks.Block` objects.
            input (str or list of str): one or more input operands.  If 'auto',
                the input operands are auto-determined from the name.
            output (str or list of str): one or more output operands.  If
                'auto', the output operands are auto-determined from the name.
        """
        self._emulator.setFilter(self._indices, name, filter, input, output)

    def setJointLimits(self, qmin='auto', qmax='auto', op='clamp'):
        """Activates a joint limit filter.
        
        If qmin/qmax are 'auto', these are read from the klampt robot model
        or the properties.

        If op is...

        * 'clamp' then commands are silently clamped to their limits.
        * 'stop' a soft-stop is raised.
        * 'warn' a warning is printed and the robot silently ignores the
          command.

        """
        #need to limit to hardware values
        hw_qmin,hw_qmax = self.properties.get('joint_limits',(None,None))
        self._emulator.setJointLimits(self._indices,qmin,qmax,op,hw_qmin,hw_qmax)
    
    def setCollisionFilter(self, world : WorldModel=None, op = 'warn'):
        """Activates a collision filter.
        
        If world=None, only does self collision. Otherwise, does self and
        world collision.

        If op = 'stop', a soft stop is raised if a collision is predicted.

        If op = 'warn', a warning is printed and the robot silently ignores
        the command.
        
        """
        self._emulator.setCollisionFilter(world,op)

    def sensors(self):
        return self._try('sensors',[],[])+list(self._emulator.virtualSensorMeasurements.keys())

    def enabledSensors(self):
        return self._try('enabledSensors',[],[])+list(self._emulator.virtualSensorMeasurements.keys())

    def hasSensor(self,sensor):
        if sensor in self._emulator.virtualSensorMeasurements: return True
        return self._try('hasSensor',[sensor],lambda sensor:sensor in self.sensors())

    def enableSensor(self,sensor):
        return self._try('enableSensor',[sensor],False)

    def sensorMeasurements(self,sensor):
        return self._base.sensorMeasurements(sensor)

    def sensorUpdateTime(self,sensor):
        return self._base.sensorUpdateTime(sensor)

    def setPosition(self,q):
        assert len(q) == len(self._indices),"Position vector has incorrect length: {} != {}".format(len(q),len(self._indices))
        q = self._emulator.commandFilter(self._indices,'positionCommand',q)
        self._emulator.setPosition(self._indices,q)

    def setVelocity(self,v,ttl=None):
        assert len(v) == len(self._indices),"Velocity vector has incorrect length: {} != {}".format(len(v),len(self._indices))
        v = self._emulator.commandFilter(self._indices,'velocityCommand',v)
        self._emulator.setVelocity(self._indices,v,ttl)

    def setTorque(self,t,ttl=None):
        assert len(t) == len(self._indices),"Torque vector has incorrect length: {} != {}".format(len(t),len(self._indices))
        assert ttl is None or isinstance(ttl,(float,int)),"ttl must be a number"
        t = self._emulator.commandFilter(self._indices,'torqueCommand',t)
        self._emulator.setTorque(self._indices,t,ttl)
        
    def setPID(self,q,dq,t=None):
        assert len(q) == len(self._indices),"Position vector has incorrect length: {} != {}".format(len(q),len(self._indices))
        assert len(dq) == len(self._indices),"Velocity vector has incorrect length: {} != {}".format(len(dq),len(self._indices))
        q = self._emulator.commandFilter(self._indices,'positionCommand',q)
        dq = self._emulator.commandFilter(self._indices,'velocityCommand',dq)
        self._emulator.setPID(self._indices,q,dq,t)

    def setPIDGains(self,kP,kI,kD):
        assert len(kP) == len(self._indices),"kP vector has incorrect length: {} != {}".format(len(kP),len(self._indices))
        assert len(kI) == len(self._indices),"kI vector has incorrect length: {} != {}".format(len(kI),len(self._indices))
        assert len(kD) == len(self._indices),"kD vector has incorrect length: {} != {}".format(len(kD),len(self._indices))
        assert all(v >= 0 for v in kP),"kP must be nonnegative"
        assert all(v >= 0 for v in kI),"kI must be nonnegative"
        assert all(v >= 0 for v in kD),"kD must be nonnegative"
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

    def getPIDGains(self):
        try:
            return self._base.getPIDGains()
        except NotImplementedError:
            pass
        return self._emulator.getPIDGains(self._indices)

    def moveToPosition(self,q,speed=1):
        assert len(q) == len(self._indices),"Position vector has incorrect length: {} != {}".format(len(q),len(self._indices))
        assert isinstance(speed,(float,int))
        q = self._emulator.commandFilter(self._indices,'positionCommand',q)
        self._emulator.moveToPosition(self._indices,q,speed)

    def setPiecewiseLinear(self,ts,qs,relative=True):
        assert len(ts) == len(qs),"Piecewise linear path must have equal numbers of times and configurations"
        for i,q in enumerate(qs):
            qs[i] = self._emulator.commandFilter(self._indices,'positionCommand',q)
        self._emulator.setPiecewiseLinear(self._indices,ts,qs,relative)

    def setPiecewiseCubic(self,ts,qs,vs,relative=True):
        assert len(ts) == len(qs),"Piecewise cubic path must have equal numbers of times and configurations"
        assert len(ts) == len(vs),"Piecewise cubic path must have equal numbers of times and velocities"
        for i,q in enumerate(qs):
            qs[i] = self._emulator.commandFilter(self._indices,'positionCommand',q)
            vs[i] = self._emulator.commandFilter(self._indices,'velocityCommand',vs[i])
        self._emulator.setPiecewiseCubic(self._indices,ts,qs,vs,relative)

    def setToolCoordinates(self,xtool_local):
        self._emulator.setToolCoordinates(self._indices,xtool_local)

    def getToolCoordinates(self):
        return self._emulator.getToolCoordinates(self._indices)

    def setGravityCompensation(self,gravity=[0,0,-9.8],load=0.0,load_com=[0,0,0]):
        try:
            self._base.setGravityCompensation(gravity,load,load_com)
        except NotImplementedError:
            pass
        self._emulator.setGravityCompensation(self._indices,gravity,load,load_com)

    def getGravityCompensation(self):
        try:
            return self._base.getGravityCompensation()
        except NotImplementedError:
            pass
        self._emulator.getGravityCompensation(self._indices)

    def setCartesianPosition(self,xparams,frame='world'):
        assert len(xparams)==2 or len(xparams)==3, "Cartesian target must be a point or (R,t) transform"
        self._emulator.setCartesianPosition(self._indices,xparams,frame)

    def moveToCartesianPosition(self,xparams,speed=1.0,frame='world'):
        assert len(xparams)==2 or len(xparams)==3, "Cartesian target must be a point or (R,t) transform"
        assert isinstance(speed,(float,int)),"Speed must be a number"
        self._emulator.moveToCartesianPosition(self._indices,xparams,speed,frame)
    
    def moveToCartesianPositionLinear(self,xparams,speed=1.0,frame='world'):
        assert len(xparams)==2 or len(xparams)==3, "Cartesian target must be a point or (R,t) transform"
        assert isinstance(speed,(float,int)),"Speed must be a number"
        self._emulator.moveToCartesianPositionLinear(self._indices,xparams,speed,frame)

    def setCartesianVelocity(self,dxparams,ttl=None,frame='world'):
        assert len(dxparams)==2 or len(dxparams)==3, "Cartesian velocity must be a velocity or (angularVelocity,velocity) pair"
        assert ttl is None or isinstance(ttl,(int,float)), "ttl must be None or a number"
        self._emulator.setCartesianVelocity(self._indices,dxparams,ttl,frame)

    def setCartesianForce(self,fparams,ttl=None,frame='world'):
        assert len(fparams)==2 or len(fparams)==3, "Cartesian force must be a force or (torque,force) pair"
        assert ttl is None or isinstance(ttl,(int,float)), "ttl must be None or a number"
        self._emulator.setCartesianForce(self._indices,fparams,ttl,frame)

    def status(self,joint_idx=None):
        if self._emulator.status != 'ok': return self._emulator.status
        return self._base.status(joint_idx)

    def isMoving(self,joint_idx=None):
        if self._emulator.isMoving(self._indices):
            return True
        try:
            return self._base.isMoving(joint_idx)
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
    
    def stateValue(self,name):
        return self._base.stateValue(name)

    def cartesianPosition(self,q,frame='world'):
        if self._baseControlMode == self._emulatorControlMode:
            #using base interface
            return self._try('cartesianPosition',[q,frame],lambda q,frame: self._emulator.cartesianPosition(self._indices,q,frame))
        else:
            return self._emulator.cartesianPosition(self._indices,q,frame)

    def cartesianVelocity(self,q,dq,frame='world'):
        if self._baseControlMode == self._emulatorControlMode:
            #using base interface
            return self._try('cartesianVelocity',[q,dq,frame],lambda q,dq,frame: self._emulator.cartesianVelocity(self._indices,q,dq,frame))
        else:
            return self._emulator.cartesianVelocity(self._indices,q,dq,frame)

    def cartesianForce(self,q,t,frame='world'):
        if self._baseControlMode == self._emulatorControlMode:
            #using base interface
            return self._try('cartesianForce',[q,t,frame],lambda q,t,frame: self._emulator.cartesianForce(self._indices,q,t,frame))
        else:
            return self._emulator.cartesianForce(self._indices,q,t)

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
        return self._try('commandedCartesianForce',[frame],lambda frame: RobotInterfaceBase.commandedCartesianForce(self,frame))

    def destinationCartesianPosition(self,frame='world'):
        return self._try('destinationCartesianPosition',[frame],lambda frame: RobotInterfaceBase.destinationCartesianPosition(self,frame))

    def destinationCartesianVelocity(self,frame='world'):
        return self._try('destinationCartesianVelocity',[frame],lambda frame: RobotInterfaceBase.destinationCartesianVelocity(self,frame))

    def partToRobotConfig(self,pconfig,part,robotConfig):
        return self._base.partToRobotConfig(pconfig,part,robotConfig)

    def klamptModel(self):
        if self._klamptModel is not None: return self._klamptModel
        return self._base.klamptModel()

    def configFromKlampt(self,klamptConfig,part=None,joint_idx=None):
        #TODO: this doesn't work when this is a sub-model 
        return self._base.configFromKlampt(klamptConfig,part=None,joint_idx=None)

    def velocityFromKlampt(self,klamptVelocity,part=None,joint_idx=None):
        #TODO: this doesn't work when this is a sub-model 
        return self._base.velocityFromKlampt(klamptVelocity,part=None,joint_idx=None)

    def configToKlampt(self,config,klamptConfig=None,part=None,joint_idx=None):
        #TODO: this doesn't work when this is a sub-model 
        return self._base.configToKlampt(config,klamptConfig,part=None,joint_idx=None)

    def velocityToKlampt(self,velocity,klamptVelocity=None,part=None,joint_idx=None):
        #TODO: this doesn't work when this is a sub-model 
        return self._base.velocityToKlampt(velocity,klamptVelocity=None,part=None,joint_idx=None)

    def printStatus(self):
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
        self._emulator.printStatus()

        print("**************************************************")


class LimitFilter:
    """Checks if the argument is in the range [qmin,qmax].  Can either
    clamp to range silently, warn, or perform a stop.
    """
    def __init__(self,op,qmin,qmax):
        self.qmin = qmin
        self.qmax = qmax
        self.op = op
        if op not in ['clamp','stop','warn']:
            raise ValueError("Only supports clamp, stop, and warn")
        if op == 'clamp':
            clamper = blocks.utils.Clamp()
            self.subblock = clamper
        else:
            checker = blocks.utils.LimitExceeded()
            self.subblock = checker
    def __call__(self,q):
        if self.op == 'clamp':
            return self.subblock.advance(q,self.qmin,self.qmax)
        else:
            if self.subblock.advance(q,self.qmin,self.qmax):
                if self.op == 'stop':
                    raise blocks.utils.BlockSignal('softStop','limit reached')
                else:
                    raise blocks.utils.BlockSignal('warn','limit reached')
            return q


class BrakeLimitFilter:
    """Checks if the argument (q,dq) can brake in time to stay within in the
    range [qmin,qmax].  Can either clamp dq to range silently, warn, or
    perform a stop.
    """
    def __init__(self,op,qmin,qmax,amin,amax):
        assert len(qmin)==len(amax)
        assert len(qmax)==len(amax)
        assert len(amin)==len(amax)
        self.qmin = qmin
        self.qmax = qmax
        self.amin = amin
        self.amax = amax
        for i in range(qmin):
            assert qmin[i] <= qmax[i]
            assert amin[i] <= 0 and amax[i] >= 0
        self.op = op
        if op not in ['clamp','stop','warn']:
            raise ValueError("Only supports clamp, stop, and warn")
    def __call__(self,q,dq):
        assert len(q)==len(dq)
        assert len(q)==len(self.amax)
        qres = []
        vres = []
        for i,x in enumerate(q):
            v = dq[i]
            a = self.amax[i] if v < 0 else self.amin[i]
            t = -v/a
            xn = x - 0.5*v**2/a
            if xn < self.qmin[i] or xn > self.qmax[i]:
                if self.op == 'stop':
                    raise blocks.utils.BlockSignal('softStop','limit reached')
                elif self.op == 'warn':
                    raise blocks.utils.BlockSignal('warn','limit reached')
                else:
                    if x < self.qmin[i]:
                        x = self.qmin[i]
                        v = 0
                    elif x > self.qmax[i]:
                        x = self.qmax[i]
                        v = 0
                    else:
                        if v < 0:
                            #x - 0.5*v**2/a = qmin  =>  2a(x - qmin) = v**2
                            v = -math.sqrt((x - self.qmin[i])*2*a)
                        else:
                            #x - 0.5*v**2/a = qmax  =>  2a(x - qmax) = v**2
                            v = math.sqrt((x - self.qmax[i])*2*a)
            qres.append(x)
            vres.append(v)
        return (qres,vres)


class CartesianLimitFilter:
    """Checks if the rotation is within some range and the translation is
    in the range [xmin,xmax].  The rotation range is so3.rpy(Rref^T R) in
    [rpymin,rpymax].

    Can set Rref,rpymin,rpymax to None to turn off rotation range checking,
    and set xmin,xmax to None to turn off translation range checking.
    
    Can either clamp to range silently, warn, or perform a stop.
    """
    def __init__(self,op,Rref,rpymin,rpymax,xmin,xmax):
        self.Rref = Rref
        self.rpymin = rpymin
        self.rpymax = rpymax
        self.xmin = xmin
        self.xmax = xmax
        self.op = op
        if op not in ['clamp','stop','warn']:
            raise ValueError("Only supports clamp, stop, and warn")
    def __call__(self,T):
        R,t = T
        assert len(R)==9
        assert len(t)==3
        if self.Rref is not None:
            Rrel = so3.mul(so3.inv(self.Rref),R)
            rpy = so3.rpy(Rrel)
            rpynew = [0]*3
            for i in range(3):
                rpynew[i] = min(max(rpy[i],self.rpymin[i]),self.rpymax[i])
            if rpy != rpynew:
                if self.op == 'stop':
                    raise blocks.utils.BlockSignal('softStop','limit reached')
                elif self.op == 'warn':
                    raise blocks.utils.BlockSignal('warn','limit reached')
                R = so3.mul(self.Rref,so3.from_rpy(rpynew))
        if self.xmin is not None:
            tnew = [0]*3
            for i in range(3):
                tnew[i] = min(max(t[i],self.xmin[i]),self.xmax[i])
            if t != tnew:
                if self.op == 'stop':
                    raise blocks.utils.BlockSignal('softStop','limit reached')
                elif self.op == 'warn':
                    raise blocks.utils.BlockSignal('warn','limit reached')
        return (R,t)


class SelfCollisionFilter:
    """Soft-stops if the robot at configuration q is in collision."""
    def __init__(self, op:str, robot : 'RobotModel'):
        if op not in ['stop','warn']:
            raise ValueError("Only supports stop and warn")
        self.op = op
        self.robot = robot
    def __call__(self,q):
        self.robot.setConfig(self.robot.configFromDrivers(q))
        if self.robot.selfCollides():
            if self.op == 'stop':
                raise blocks.utils.BlockSignal('softStop','collision')
            else:
                raise blocks.utils.BlockSignal('warn','collision')
        return q

class CollisionFilter:
    """Soft-stops if the robot at configuration q is in collision."""
    def __init__(self, op:str, robot:'RobotModel', obstacles):
        if op not in ['stop','warn']:
            raise ValueError("Only supports stop and warn")
        self.op = op
        self.robot = robot
        self.robotGeoms = []
        for i in range(robot.numLinks()):
            g = robot.link(i).geometry()
            if not g.empty():
                self.robotGeoms.append(g)
        self.obstacles = obstacles
    def __call__(self,q):
        self.robot.setConfig(self.robot.configFromDrivers(q))
        for o in self.obstacles:
            for g in self.robotGeoms:
                if o.collides(g):
                    if self.op == 'stop':
                        raise blocks.utils.BlockSignal('softStop','collision')
                    else:
                        raise blocks.utils.BlockSignal('warn','collision')
        return q


class DifferenceFilter:
    """Soft-stops if the difference between the two arguments exceed a
    threshold.  Returns 0 arguments.
    """
    def __init__(self,op,threshold,norm=float('inf')):
        self.threshold = threshold
        self.op = op
        if op not in ['stop','warn']:
            raise ValueError("Only supports stop and warn")
        self.subblock = blocks.utils.Distance(norm)
    def advance(self,q1,q2):
        d = self.subblock.advance(q1,q2)
        if d > self.threshod:
            if self.op == 'stop':
                raise blocks.utils.BlockSignal('softStop','difference threshold exceeded')
            else:
                raise blocks.utils.BlockSignal('warn','difference threshold exceeded')


class LoggingFilter(blocks.Block):
    """Logs all the input items to fn, which is assumed by default to be a
    CSV file.
    """
    def __init__(self,fn,items,delim=',',header=True):
        super().__init__(items,0)
        self.items = items
        self.fn = fn
        self.file = None
        self.delim = delim
        self.header = header
    def advance(self,*args):
        assert len(args) == len(self.items)
        import numpy as np
        import os
        if self.file is None:
            if os.path.exists(self.fn):
                self.file = open(self.fn,'a')
            else:
                self.file = open(self.fn,'w')
                #write header
                if self.header:
                    argnames = []
                    for n,a in zip(self.items,args):
                        if hasattr(a,'__iter__'):
                            for i in range(len(a)):
                                argnames.append(n+'['+str(i)+']')
                    self.file.write(self.delim.join(argnames))
                    self.file.write('\n')
        channels = np.hstack(args).tolist()
        self.file.write(self.delim.join(str(v) for v in channels))
        self.file.write('\n')
        


def klamptCartesianPosition(model : RobotModel,
                            q: Vector,
                            part_indices : List[int],
                            tool_coordinates : Vector3,
                            frame :str) -> RigidTransform:
    """Helper for implementers: an implementation of cartesianPosition
    that uses the Klampt model.

    Args:
        robot (RobotModel): the Klampt model
        q: configuration for whole robot (#drivers)
        part_indices: the driver indices of the part being queried.
        tool_coordinates: the local coordinates of the tool point
        frame: either 'world' or 'base'
    """
    if frame == 'tool':
        return se3.identity()
    elif frame == 'end effector':
        return (so3.identity(),tool_coordinates)
    assert len(q) == model.numDrivers()
    for i,v in enumerate(q):
        if i > part_indices[-1]: break
        model.driver(i).setValue(v)
    model.setConfig(model.getConfig())
    ee_index = model.driver(part_indices[-1]).getAffectedLink()
    ee_xform = model.link(ee_index).getTransform()
    tool_xform = ee_xform[0],se3.apply(ee_xform,tool_coordinates)
    if frame == 'base':
        first_link_index = model.driver(part_indices[0]).getAffectedLink()
        base_link_index = model.link(first_link_index).getParent()
        if base_link_index < 0:
            return tool_xform
        base_xform = model.link(base_link_index).getTransform()
        return se3.mul(se3.inv(base_xform),tool_xform)
    elif frame == 'world':
        return tool_xform
    else:
        raise ValueError("Invalid frame specified")

def klamptCartesianVelocity(model : RobotModel,
                            q : Vector,
                            dq : Vector,
                            part_indices : Sequence[int],
                            tool_coordinates : Vector3,
                            frame : str) -> Tuple[Vector3,Vector3]:
    """Helper for implementers: an implementation of cartesianVelocity
    that uses the Klampt model.

    Args:
        q: configuration for whole robot (#drivers)
        dq: velocity for whole robot (#drivers)
        part_indices: the driver indices of the part being queried.
        tool_coordinates: the local coordinates of the tool point
        frame: either 'world' or 'base'
    """
    assert len(q) == model.numDrivers()
    assert len(dq) == model.numDrivers()
    for i,v in enumerate(q):
        if i > part_indices[-1]: break
        model.driver(i).setValue(v)
        if frame == 'base' and i < part_indices[0]:
            model.driver(i).setVelocity(0)  #get relative velocity to base
        else:
            model.driver(i).setVelocity(dq[i])
    model.setConfig(model.getConfig())
    ee_index = model.driver(part_indices[-1]).getAffectedLink()
    ee = model.link(ee_index)
    v = ee.getPointVelocity(tool_coordinates)
    w = ee.getAngularVelocity()
    if frame == 'base':
        first_link_index = model.driver(part_indices[0]).getAffectedLink()
        base_link_index = model.link(first_link_index).getParent()
        if base_link_index < 0: return (w,v)
        Tbase = model.link(base_link_index).getTransform()
        Rworld_base = so3.inv(Tbase[0])
        return (so3.apply(Rworld_base,w),so3.apply(Rworld_base,v))
    elif frame == 'end effector' or frame == 'tool':
        Tee = ee.getTransform()
        Rworld_ee = so3.inv(Tee[0])
        return (so3.apply(Rworld_ee,w),so3.apply(Rworld_ee,v))
    elif frame == 'world':
        return (w,v)
    else:
        raise ValueError("Invalid frame specified")


def klamptCartesianForce(model : RobotModel,
                         q: Vector,
                         t: Vector,
                         part_indices : List[int],
                         tool_coordinates : Vector3,
                         frame : str):
    """Helper for implementers: an implementation of cartesianForce
    that uses the Klampt model.

    Args:
        q: configuration for whole robot (#drivers)
        t: torques for whole robot (#drivers)
        part_indices: the driver indices of the part being queried.
        tool_coordinates: the local coordinates of the tool point
        frame: either 'world' or 'base'
    """
    import numpy as np
    assert len(q) == model.numDrivers()
    for i,v in enumerate(q):
        if i > part_indices[-1]: break
        model.driver(i).setValue(v)
    model.setConfig(model.getConfig())
    ee_index = model.driver(part_indices[-1]).getAffectedLink()
    ee = model.link(ee_index)
    J = ee.getJacobian(tool_coordinates)
    wrench = np.dot(J,model.velocityFromDrivers(t))
    torque,force = wrench[:3].tolist(),wrench[3:].tolist()
    if frame == 'world':
        return (torque,force)
    elif frame == 'base':
        first_link_index = model.driver(part_indices[0]).getAffectedLink()
        base_link_index = model.link(first_link_index).getParent()
        if base_link_index < 0: return (torque,force)
        Tbase = model.link(base_link_index).getTransform()
        Rworld_base = so3.inv(Tbase[0])
        return (so3.apply(Rworld_base,torque),so3.apply(Rworld_base,force))
    elif frame == 'end effector' or frame == 'tool':
        Tee = ee.getTransform()
        Rworld_ee = so3.inv(Tee[0])
        return (so3.apply(Rworld_ee,torque),so3.apply(Rworld_ee,force))
    raise ValueError("Invalid frame specified")


class ThreadedRobotInterface(_RobotInterfaceStatefulBase):
    """Adapts a synchronous robot interface to an asynchronous robot interface
    that runs in a separate thread.

    There is no need to run beginStep() and endStep() on this interface. 

    All query functions will return the results from the prior time step.

    All commands are queued, with execution delayed until the next time step.
    """
    def __init__(self, interface : RobotInterfaceBase):
        _RobotInterfaceStatefulBase.__init__(self)
        from threading import Thread, RLock
        if not isinstance(interface,_RobotInterfaceStatefulBase):
            interface = _RobotInterfaceStatefulWrapper(interface)
        self._interface = interface
        self._partInterfaces = dict()         #type: Dict[str,_RobotInterfaceStatefulBase]
        self._thread = Thread(target=self._threadFunc)
        self._thread.daemon = True
        self._lock = RLock()
        self._initialized = None
        self.properties = copy.deepcopy(self._interface.properties)
        if self.properties.get('asynchronous',False):
            warnings.warn("ThreadedRobotInterface: interface is already asynchronous?  Suggest not threading it")
        self.properties['asynchronous'] = True
        self._shortcut_update = False          #changes will only be reflected on next time step
        
        for method in _RobotInterfaceStatefulBase.COMPLEX_METHODS:
            #lock all complex method accesses
            setattr(self,method,_LockedCallable(getattr(self,method),self._lock))
    
    def __str__(self):
        return "Threaded({})".format(str(self._interface))
    
    def log(self,msg):
        """Override this to change thread warning prints"""
        print(msg)

    def initialize(self):
        if self._initialized is not None:
            raise RuntimeError("initialize() can only be called once")
        self._quit = False
        self._thread.start()
        t0 = time.time()
        while self._initialized is None:
            time.sleep(0.01)
            t1 = time.time()
            if t1 - t0 > 5:
                print("Initialization is taking a long time...")
                t0 = t1
        #lock all complex method accesses for child interfaces
        for part,iface in self._partInterfaces.items():
            for method in _RobotInterfaceStatefulBase.COMPLEX_METHODS:
                setattr(iface,method,_LockedCallable(getattr(iface,method),self._lock))
        return self._initialized

    def close(self):
        if self._quit:
            warnings.warn("close() called before initialize()?")
        elif self._thread is not None:
            self._quit = True
            self._thread.join()
            self._thread = None
        else:
            warnings.warn("close() called twice?")

    def _threadFunc(self):
        assert self._initialized is None
        from .utils import TimedLooper
        try:
            if not self._interface.initialize():
                self._initialized = False
                return
        except Exception as e:
            self._initialized = False
            raise
        with self._lock:
            #note: can't copy klampt models
            robot_model = self._interface._structure.klamptModel
            self._interface._structure.klamptModel = None
            self._structure = copy.deepcopy(self._interface._structure)
            self._interface._structure.klamptModel = robot_model
            
            self._settings = copy.deepcopy(self._interface._settings)
            self._structure.klamptModel = None   #load a new klampt model to avoid thread clashes
            self._structure.klamptModel = self.klamptModel()
            if self._structure.klamptModel is None:
                warnings.warn("Robot didn't have 'klamptModel' property -- reusing RobotModel object in thread")
                self._structure.klamptModel = robot_model
            self._init_part_interfaces()
            for part,iface in self._partInterfaces.items():
                iface.properties['asynchronous'] = True
            self._split_settings()
            self._clear_deltas()

        try:
            dt = 1.0/self._interface.controlRate()
        except Exception as e:
            print("Error calling controlRate()",e)
            self._initialized = False
            raise
        self._initialized = True   #tell calling thread that it can continue with initialize()
        
        #begin control loop
        looper = TimedLooper(dt=dt,name='RobotInterfaceServer({})'.format(str(self._interface)),warning_printer=self.log)
        while looper:
            with StepContext(self._interface,ignore=True):
                if self._interface._commands:
                    self.quit = True
                    raise RuntimeError("Interface didn't clear command queue?")
                #write state changes
                with self._lock:
                    self._state = copy.deepcopy(self._interface._state)
                    self._split_state()
                with self._lock:
                    #read settings changes, command changes and pass to interface
                    settings,cmds = self._advance_settings_and_commands()
                    self._interface._delta_settings = settings
                    self._interface._commands = cmds
            if self._quit:
                break
        self._interface.close()



class MultiprocessingRobotInterface(_RobotInterfaceStatefulBase):
    """Adapts a synchronous robot interface to an asynchronous robot interface
    that runs in a separate process.

    There is no need to run beginStep() and endStep() on this interface. 

    All query functions will return the results from the prior time step.

    All commands are queued, with execution delayed until the next time step.
    """
    def __init__(self, interface : RobotInterfaceBase):
        _RobotInterfaceStatefulBase.__init__(self)
        from threading import Thread, RLock
        from multiprocessing import Process,Queue
        self._interfaceName = str(interface)
        self._partInterfaces = dict()         #type: Dict[str,_RobotInterfaceStatefulBase]
        self._thread = Thread(target=self._threadFunc,args=[interface])
        self._thread.daemon = True
        self._lock = RLock()
        self._initialized = None
        sendQueue = Queue()
        receiveQueue = Queue()
        self._process = Process(target=self._processFunc,args=[interface,sendQueue,receiveQueue])
        self._sendQueue = sendQueue
        self._receiveQueue = receiveQueue
        self.properties = copy.deepcopy(interface.properties)
        if self.properties.get('asynchronous',False):
            warnings.warn("ThreadedRobotInterface: interface is already asynchronous?  Suggest not threading it")
        self.properties['asynchronous'] = True
        self._shortcut_update = False   #changes will only be reflected on next time step
        
        for method in _RobotInterfaceStatefulBase.COMPLEX_METHODS:
            #lock all complex method accesses
            setattr(self,method,_LockedCallable(getattr(self,method),self._lock))
    
    def __str__(self):
        return "Multiprocessing({})".format(self._interfaceName)
    
    def log(self,msg):
        """Override this to change thread warning prints"""
        print(msg)

    def initialize(self):
        if self._initialized is not None:
            raise RuntimeError("initialize() can only be called once")
        self._quit = False
        self._thread.start()
        self._process.start()
        t0 = time.time()
        while self._initialized is None:
            time.sleep(0.01)
            t1 = time.time()
            if t1 - t0 > 5:
                print("Initialization is taking a long time...")
                t0 = t1
        return self._initialized

    def close(self):
        if self._quit:
            warnings.warn("close() called before initialize()?")
        elif self._thread is not None:
            self._sendQueue.put(('quit',()))
            self._process.join()
            self._process.close()
            self._quit = True
            self._thread.join()
            self._thread = None
        else:
            warnings.warn("close() called twice?")

    def _threadFunc(self, interface):
        assert self._initialized is None
        from .utils import TimedLooper
        with self._lock:
            initial_message = self._receiveQueue.get()
            if initial_message is None:
                errorMessage = self._receiveQueue.get()
                print("  Error message:",errorMessage)
                self._initialized = False
                return
            import pickle
            data = pickle.loads(initial_message)
            self._structure,self._settings,self._state = data
            assert self._state.sensedPosition is not None,"Initial state gave empty sensedPosition?"
            #self._structure,self._settings,self._state = initial_message
            self._structure.klamptModel = None   #use the interface's klampt model -- no risk of thread clashes because the other process has a copy
            self._structure.klamptModel = interface.klamptModel()
            self._init_part_interfaces()
            #mark as asynchronous, lock complex method accesses
            for (k,iface)  in self._partInterfaces.items():
                for method in _RobotInterfaceStatefulBase.COMPLEX_METHODS:
                    setattr(iface,method,_LockedCallable(getattr(iface,method),self._lock))
                iface.properties['asynchronous'] = True
            self._split_settings()
            self._clear_deltas()

        self._initialized = True   #tell calling thread that it can continue with initialize()
        while True:
            with self._lock:
                delta_settings,commands = self._advance_settings_and_commands()
            self._sendQueue.put(('updateSettingsAndCommands',(delta_settings,commands)))
            newState = self._receiveQueue.get()
            if newState is None:
                print("Controller encountered an error, breaking out of thread.")
                errorMessage = self._receiveQueue.get()
                print("  Error message:",errorMessage)
                break
            with self._lock:
                self._state = newState
                self._split_state()
            if self._quit:
                break
    
    def _processFunc(self, interface : RobotInterfaceBase, receiveQueue, sendQueue):
        assert self._initialized is None
        from .utils import TimedLooper
        if not isinstance(interface,_RobotInterfaceStatefulBase):
            interface = _RobotInterfaceStatefulWrapper(interface)
        if not interface.initialize():
            sendQueue.put(None)  #marks an error
            sendQueue.put("Interface {} failed to initialize".format(str(interface._base)))
            return

        interface.beginStep()
        interface.endStep()
        
        dt = 1.0/interface.controlRate()
        looper = TimedLooper(dt=dt,name='RobotInterfaceServer({})'.format(self._interfaceName),warning_printer=self.log)

        #can't serialize RobotModel
        robot_model = interface._structure.klamptModel
        interface._structure.klamptModel = None
        import pickle
        data = pickle.dumps((interface._structure,interface._settings,interface._state))
        sendQueue.put(data)
        #sendQueue.put((interface._structure,interface._settings,interface._state))
        interface._structure.klamptModel = robot_model

        firstTime = True

        while looper:
            try:
                with StepContext(interface):
                    #send state after first loop
                    if firstTime:
                        firstTime = False
                    else:
                        sendQueue.put(interface._state)
                    msg = receiveQueue.get()
                    if len(msg) != 2 or msg[0] != 'updateSettingsAndCommands':
                        sendQueue.put(None) #marks an error
                        sendQueue.put("Invalid message from thread, must be a tuple")
                        interface.endStep()
                        break
                    delta_settings,commands = msg[1]
                    interface._delta_settings = delta_settings
                    interface._commands = commands
            except Exception as e:
                sendQueue.put(None) #marks an error
                sendQueue.put("Exception from interface: {}".format(str(e)))
                break
            
        print("Ending controller process")



class _JointInterfaceEmulatorData:
    CONTROL_MODE_PRECEDENCE = {'setPID':0,'setVelocity':1,'setPosition':2,'setPiecewiseLinear':3,'setPiecewiseCubic':4}

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
        self.commandSent = False

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
        if self.controlMode == 'setPID':
            self.commandedPosition = self.pidCmd[0]
            self.commandedVelocity = self.pidCmd[1]
            self.commandedTorque = self.pidCmd[2]
            self.pidIntegralError += self._positionDifference(self.commandedPosition,q)*dt
            self.commandTTL = dt*5
        elif self.controlMode == 'setPiecewiseLinear' or self.controlMode == 'setPiecewiseCubic':
            self.commandedPosition,self.commandedVelocity = self.evalTrajectory(t+dt)
            self.commandTTL = dt*5
            self.updateTrajectory(t)
            self.commandedTorque = 0
        elif self.controlMode == 'setPosition' or self.controlMode == 'moveToPosition':
            if self.lastCommandedPosition is None:
                self.commandedVelocity = 0
            else:
                self.commandedVelocity = (self.commandedPosition-self.lastCommandedPosition)/dt
            self.lastCommandedPosition = self.commandedPosition
        elif self.controlMode == 'setVelocity':
            self.commandedPosition += self.commandedVelocity*dt
            if self.commandTTL is not None and self.commandTTL <= 0:
                self.commandedVelocity = 0
                self.commandTTL = None
                self.controlMode = None
        elif self.controlMode == 'setTorque':
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
                    raise ValueError("Trying to get a command for joint {} before sensed position was entered?".format(self.name))
            self.controlMode = 'setPosition'
        if commandType == 'setPiecewiseCubic':
            if self.controlMode == 'setPosition':
                if self.commandedVelocity == 0:
                    return [0],[self.commandedPosition],[0]
                else:
                    dt = self._positionDifference(self.commandedPosition,self.lastCommandedPosition)/self.commandedVelocity
                    return [dt],[self.commandedPosition],[self.commandedVelocity]
            elif self.trajectoryTimes is not None:
                return self.trajectoryTimes,self.trajectoryMilestones,self.trajectoryVelocities
            else:
                print("TODO: convert other control modes to setPiecewiseCubic?")
        elif commandType == 'setPiecewiseLinear':
            if self.controlMode == 'setPiecewiseCubic':
                traj = HermiteTrajectory(self.trajectoryTimes,[[m] for m in self.trajectoryMilestones],[[v] for v in self.trajectoryVelocities])
                assert self.dt is not None
                configTraj = traj.discretize(self.dt)
                return configTraj.times,[m[0] for m in configTraj.milestones]
            if self.controlMode == 'setPiecewiseLinear':
                return self.trajectoryTimes,self.trajectoryMilestones
            elif self.controlMode == 'setPosition':
                #construct interpolant... should we do it in 1 time step or stretch it out?
                if self.commandedVelocity == 0:
                    return [0],[self.commandedPosition]
                dt = self._positionDifference(self.commandedPosition,self.lastCommandedPosition)/self.commandedVelocity
                return [dt],[self.commandedPosition]
            elif self.controlMode == 'setVelocity':
                #construct interpolant
                raise NotImplementedError("Shouldn't ever be in v control model")
            else:
                raise NotImplementedError("TODO: convert other control types to linear path")
        if commandType == 'setPID':
            return self.commandedPosition,self.commandedVelocity,self.commandedTorque
        elif commandType == 'setTorque':
            if self.controlMode == 'setPID':
                if self.pidGains is None:
                    raise RuntimeError("Can't emulate PID control for joint {} using torque control, no gains are set".format(self.name))
                qdes,vdes,tdes = self.pidCmd
                kp,ki,kd = self.pidGains
                t_pid = kp*self._positionDifference(qdes,self.sensedPosition) + kd*(vdes-self.sensedVelocity) + ki*self.pidIntegralError + tdes
                #if abs(self.pidIntegralError[i]*ki) > tmax:
                #cap integral error to prevent wind-up
                return t_pid,self.commandTTL
            else:
                assert self.controlMode == 'setTorque',"Can't emulate torque control with any command type except for PID and torque control"
                return self.commandedTorque,self.commandTTL
        elif commandType == 'setPosition' or commandType == 'moveToPosition':
            return self.commandedPosition,
        elif commandType == 'setVelocity':
            assert self.controlMode != 'setPID',"Can't emulate PID control mode with velocity control"
            if self.controlMode == 'setVelocity':
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
        if controlType == 'setPID':
            self.pidIntegralError = 0.0
            self.pidCmd = (self.commandedPosition,(0 if self.commandedVelocity is None else self.commandedVelocity),0.0)
        elif controlType == 'setPiecewiseCubic':
            if controlType == 'setPiecewiseLinear':
                self.trajectoryVelocities = [0]*len(self.trajectoryTimes)
            elif self.controlMode in ['setPosition','setVelocity','setTorque']:
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
            elif self.controlMode == 'setPID':
                self.trajectoryTimes = [0]
                self.trajectoryMilestones = [self.pidCmd[0]]
                self.trajectoryVelocities = [0]
        elif controlType == 'setPiecewiseLinear':
            if self.controlMode in ['setPosition','setVelocity','setTorque']:
                if self.commandedPosition is None:
                    q = self.sensedPosition
                else:
                    q = self.commandedPosition
                self.trajectoryTimes = [0]
                self.trajectoryMilestones = [q]
            elif self.controlMode == 'setPID':
                self.trajectoryTimes = [0]
                self.trajectoryMilestones = [self.pidCmd[0]]
            self.trajectoryVelocities = None
        self.controlMode = controlType

    def destinationTime(self,t):
        if self.controlMode not in ['setPiecewiseLinear','setPiecewiseCubic']:
            return t
        return self.trajectoryTimes[-1]

    def destinationPosition(self):
        if self.controlMode not in ['setPiecewiseLinear','setPiecewiseCubic']:
            return self.commandedPosition
        return self.trajectoryMilestones[-1]

    def destinationVelocity(self):
        if self.controlMode not in ['setPiecewiseLinear','setPiecewiseCubic']:
            return self.commandedVelocity
        if self.controlMode == 'setPiecewiseLinear':
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
        self.robotIndices = [robot.driver(i).getAffectedLink() for i in indices]
        self.driver = CartesianDriveSolver(robot)
        assert indices[-1] == max(indices),"Indices must be in sorted order"
        self.eeLink = robot.link(self.robotIndices[-1])
        self.toolCoordinates = [0,0,0]
        self.t = None
        self.active = False
        self.mode = None
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
            raise NotImplementedError("TODO: Can only handle world frame, for now")
        qorig = self.robot.getConfig()
        if not self.active:
            self.driver.start(qorig,self.robotIndices[-1],endEffectorPositions=self.toolCoordinates)
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
            raise NotImplementedError("TODO: Can only handle world frame, for now")
        for i,v in enumerate(qcur):
            self.robot.driver(i).setValue(v)
        qcur = self.robot.getConfig()

        if not self.active or self.mode != 'setCartesianVelocity':
            self.driver.start(qcur,self.robotIndices[-1],endEffectorPositions=self.toolCoordinates)
            self.active = True
            self.mode = 'setCartesianVelocity'

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

    def moveToCartesianPositionLinear(self,qcur,xparams,speed,frame='world'):
        assert len(qcur) == self.robot.numDrivers(),"Invalid length of current configuration: {}!={}".format(len(qcur),self.robot.numDrivers())
        assert speed is None or speed >= 0,"Invalid speed, must be positive"
        if len(xparams) == 2:
            if len(xparams[0]) != 9 or len(xparams[1]) != 3:
                raise ValueError("Invalid IK parameters, must be a 3 vector or a RigidTransform")
        else:
            if len(xparams) != 3:
                raise ValueError("Invalid IK parameters, must be a 3 vector or a RigidTransform")
        if frame!='world':
            raise NotImplementedError("TODO: Can only handle world frame, for now")
        for i,v in enumerate(qcur):
            self.robot.driver(i).setValue(v)
        qcur = self.robot.getConfig()

        if not self.active or self.mode != 'moveToCartesianPositionLinear':
            self.driver.start(qcur,self.robotIndices[-1],endEffectorPositions=self.toolCoordinates)
            self.active = True
            self.mode = 'moveToCartesianPositionLinear'
        if speed is None:
            speed = 1.0   #TODO: determine a good speed from the distance traveled
        self.driveCommand = (xparams,1.0/speed)
        self.endDriveTime = None

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
        qcur_rob = self.robot.getConfig()
        if self.mode == 'setCartesianVelocity':
            #print("Drive command",self.driveCommand,"dt",dt)
            (amt,q) = self.driver.drive(qcur_rob,self.driveCommand[0],self.driveCommand[1],dt)
            self.robot.setConfig(q)
            #print("Result",amt,q)
        else:  #moveToCartesianLinear
            xdesired,remainingTime = self.driveCommand
            if remainingTime <= dt:
                remainingTime = dt
            delta = 0
            xcur = self.cartesianPosition(qcur)
            assert len(xcur) == len(xdesired),"Invalid result from cartesianPosition or args to moveToCartesianPositionLinear"
            if len(xdesired)==3:
                v = vectorops.mul(vectorops.sub(xdesired,xcur),1.0/remainingTime)
                (amt,q) = self.driver.drive(qcur_rob,None,v,dt)
                delta = vectorops.norm(v)
            else:
                assert len(xdesired)==2
                assert len(xdesired[0])==9
                assert len(xdesired[1])==3
                wv = vectorops.mul(se3.error(self.driveCommand[0],xcur),1.0/remainingTime)
                w = wv[:3]
                v = wv[3:]
                delta = vectorops.norm(wv)
                (amt,q) = self.driver.drive(qcur_rob,w,v,dt)
            if amt*delta < 1e-8: #done
                self.active = False
            self.robot.setConfig(q)
            remainingTime = self.driveCommand[1]-dt*amt
            self.driveCommand = (self.driveCommand[0],remainingTime)
        return [self.robot.driver(i).getValue() for i in self.indices]

    def cartesianPosition(self,q,frame='world'):
        assert len(q) == self.robot.numDrivers()
        return klamptCartesianPosition(self.robot,q,self.indices,self.toolCoordinates,frame)
        
    def cartesianVelocity(self,q,dq,frame='world'):
        assert len(q) == self.robot.numDrivers()
        assert len(dq) == self.robot.numDrivers()
        return klamptCartesianVelocity(self.robot,q,dq,self.indices,self.toolCoordinates,frame)

    def cartesianForce(self,q,t,frame='world'):
        assert len(q) == self.robot.numDrivers()
        assert len(t) == self.robot.numDrivers()
        return klamptCartesianForce(self.robot,q,t,self.indices,self.toolCoordinates,frame)


class RobotInterfaceEmulator:
    """Emulates a fully-functional motion controller in software.  All motion
    commands and queries are available, but the caller must always specify
    which indices are being used.  Most prominently used in
    :class:`RobotInterfaceCompleter` and :class:`OmniRobotInterface`

    Joint control usage::

        interface = MyRobotInterfaceBase()
        emulator = RobotInterfaceEmulator(interface.numJoints(),klampt_model)
        interface.beginStep()
        qcmd = interface.commandedPosition()   #query current configuration
        interface.endStep()
        emulator.initCommand(qcmd,[0]*len(qcmd),None)  #initialize with current commanded values
        move_to_sent = False
        while True:
            t = interface.clock()
            emulator.advanceClock(t,None)  #can also advance by rate
            q,v = interface.sensedPosition(),interface.sensedVelocity()
            emulator.update(q,v)
            if not move_to_sent:
                part1_indices = [0,1,4,5]  # whatever indices are available
                part1_config = [0.4,0.3,-0.2,0.5]
                emulator.moveTo(part1_indices,part1_config)  #begins a move-to-command
                part2_indices = [2,3]      # some other indices 
                part2_velocity = [0.1,0.3]
                emulator.setVelocity(part2_indices,part2_velocity,1.5)  #begins a velocity command for 1.5s
                move_to_sent = True
            #after the first step, the emulator will generate position commands that execute the move-to
            q = emulator.getCommand('setPosition')    #argument is the desired command type
            interface.setPosition(q)
            emulator.onBaseCommand('setPosition','setPosition')  #indicates that we successfully executed that command type

    To minimize the number of instructions sent to the interface or to dispatch
    to different command types in the base interface::

        dcm = emulator.desiredControlMode()
        if dcm is not None:                           #desiredControlMode() returns None if done with sending, 
            q = emulator.getCommand('setPosition')    #argument is the desired command type
            interface.setPosition(q)                  #send to the actual controller
            emulator.onBaseCommand(dcm,'setPosition') #indicates that we successfully executed that command type
    
    Note that setPosition and setVelocity are always available as command
    types.

    To run Cartesian control::

        #setup
        six_dof_indices = [0,1,2,3,4,5]
        tool_coords = (0,0,0.1)
        emulator.setToolCoordinates(six_dof_indices,tool_coords)
        
        while True:
            ... #do standard emulator setup

            if do_cartesian_move:
                #read transform and send a move to
                target_shift = [0.2,0,0]
                Tcmd = emulator.commandedCartesianPosition(six_dof_indices)
                Ttarget = (Tcmd[0],vectorops.add(Tcmd[0],target_shift)
                emulator.moveToCartesianPosition(six_dof_indices,Ttarget)
                do_cartesian_move = False

            ... #do standard emulator command generation. The emulator will solve for IK automatically.

    """

    CONTROL_MODES = ['setPID','setVelocity','setPosition','moveToPosition','setPiecewiseLinear','setPiecewiseCubic']

    def __init__(self, nd : int, klamptModel : RobotModel):
        self.klamptModel = klamptModel
        self.curClock = None
        self.dt = None
        self.numUpdates = -1
        self.status = 'ok'
        self.jointData = [_JointInterfaceEmulatorData('Joint '+str(i)) for i in range(nd)]
        if klamptModel is not None:
            #find any continuous rotation joints
            for i in range(nd):
                d = klamptModel.driver(i)
                if len(d.getAffectedLinks())==1:
                    link = d.getAffectedLinks()[0]
                    if klamptModel.getJointType(link)=='spin':
                        #print("RobotInterfaceEmulator: Interpreting joint",i," as a spin joint")
                        self.jointData[i].continuousRotation = True
        self.cartesianInterfaces = dict()          #type: Dict[str,_CartesianEmulatorData]
        self.stateFilters = dict()                 #type: Dict[Sequence[int],Dict[str,Callable]]
        self.commandFilters = dict()               #type: Dict[Sequence[int],Dict[str,Callable]]
        self.virtualSensorMeasurements = dict()    #type: Dict[str,Tuple[float,Vector]]

    def initialize(self,qsns,vsns,tsns,qcmd,vcmd,tcmd):
        """Could be called before the emulator starts running to initialize the
        commanded joint positions before the emulator takes over.
        """
        assert qcmd is None or len(qcmd) == len(self.jointData)
        assert vcmd is None or len(vcmd) == len(self.jointData)
        assert tcmd is None or len(tcmd) == len(self.jointData)
        self.numUpdates = 0
        for i,j in enumerate(self.jointData):
            j.sensedPosition = qsns[i]
            j.sensedVelocity = 0 if vsns is None else vsns[i]
            #j.sensedTorque = None if tsns is None else tsns[i]
            if qcmd is not None:
                j.commandedPosition = qcmd[i]
            else:
                j.commandedPosition = qsns[i]
            if vcmd is not None:
                j.commandedVelocity = vcmd[i]
            else:
                j.commandedVelocity = 0
            if tcmd is not None and tcmd[i] is not None:
                j.commandedTorque = tcmd[i]
            else:
                j.commandedTorque = 0
        
    
    def advanceClock(self,newClock,rate):
        if self.numUpdates < 0:
            raise RuntimeError("Need to call initialize before advanceClock")
        lastClock = self.curClock
        if rate is None:
            self.curClock = newClock
            if lastClock is not None and lastClock != self.curClock:
                self.dt = self.curClock - lastClock
        elif newClock is None:
            self.dt = 1.0/rate
            if self.curClock is None: self.curClock = 0
            self.curClock = self.curClock + self.dt
        else:
            self.dt = 1.0/rate
            if abs(self.curClock + self.dt - newClock) > 1e-3:
                warnings.warn("Time advance doesn't seem consistent? Last clock {}, dt {}, predicted clock {}, reported clock {}".format(lastClock,self.dt,lastClock+self.dt,newClock))
                lastClock = newClock - self.dt
            self.curClock = newClock

    def update(self,q,v):
        """Advances the interface with the sensed config q and sensed velocity v.
        """
        assert q is not None,"Must pass a valid sensed config to update"
        if self.numUpdates < 0:
            raise RuntimeError("Need to call initialize before update")
        self.numUpdates += 1
        if v is None:   #can accept null velocities
            v = [None]*len(q)
        qcmd = q
        try:
            qcmd = self.getCommand('setPosition')[0]
        except Exception:
            pass
        for inds,c in self.cartesianInterfaces.items():
            qdes = c.update(qcmd,self.curClock,self.dt)
            if qdes is not None:  #cartesian controller is active -- change joints to setPosition
                assert len(qdes) == len(c.indices)
                for i,x in zip(inds,qdes):
                    self.jointData[i].promote('setPosition')
                    self.jointData[i].commandSent = False                
                    self.jointData[i].commandedVelocity = (x-self.jointData[i].commandedPosition)/self.dt
                    self.jointData[i].commandedPosition = x
                    self.jointData[i].commandTTL = 5.0*self.dt
        for i,j in enumerate(self.jointData):
            j.dt = self.dt
            j.update(self.curClock,q[i],v[i],self.dt)
    
    def sendToInterface(self,interface: RobotInterfaceBase, indices = None) -> Tuple[str,str]:
        """Sends a command to the interface using the highest level interface 
        as possible.
        
        Returns:
            tuple: (desiredControlMode,baseControlMode) indicating which mode was
            desired and which was actually achieved.
        """
        desiredControlMode = self.desiredControlMode(indices)
        baseControlMode = None
        if desiredControlMode is None:
            #no control needed
            return None,None
        if not hasattr(interface,'_control_mode_map'):  #initialize method cache
            interface._control_mode_map = dict()
        if desiredControlMode in interface._control_mode_map:
            baseControlMode = interface._control_mode_map[desiredControlMode]
            args = self.getCommand(baseControlMode,indices)
            getattr(interface,baseControlMode)(*args)
        else:
            if desiredControlMode == 'setPiecewiseCubic':
                res=_try_methods(interface,['setPiecewiseCubic','setPiecewiseLinear','setPID','setPosition','setVelocity','moveToPosition','setTorque'],
                        [lambda:self.getCommand('setPiecewiseCubic',indices),
                        lambda :self.getCommand('setPiecewiseLinear',indices),
                        lambda :self.getCommand('setPID',indices),
                        lambda :self.getCommand('setPosition',indices),
                        lambda :self.getCommand('setVelocity',indices),
                        lambda :self.getCommand('moveToPosition',indices),
                        lambda :self.getCommand('setTorque',indices)])
                baseControlMode = res[1]
            elif desiredControlMode == 'setPiecewiseLinear':
                res=_try_methods(interface,['setPiecewiseLinear','setPID','setPosition','setVelocity','moveToPosition','setTorque'],
                        [lambda:self.getCommand('setPiecewiseLinear',indices),
                        lambda :self.getCommand('setPID',indices),
                        lambda :self.getCommand('setPosition',indices),
                        lambda :self.getCommand('setVelocity',indices),
                        lambda :self.getCommand('moveToPosition',indices),
                        lambda :self.getCommand('setTorque',indices)])
                baseControlMode = res[1]
            elif desiredControlMode == 'setPID':
                res=_try_methods(interface,['setPID','setTorque','setPosition','setVelocity','moveToPosition'],
                        [lambda:self.getCommand('setPID',indices),
                        lambda :self.getCommand('setTorque',indices),
                        lambda :self.getCommand('setPosition',indices),
                        lambda :self.getCommand('setVelocity',indices),
                        lambda :self.getCommand('moveToPosition',indices)])
                baseControlMode = res[1]
            elif desiredControlMode == 'setPosition':
                res=_try_methods(interface,['setPosition','setVelocity','setPID','moveToPosition','setPiecewiseLinear','setTorque'],
                        [lambda:self.getCommand('setPosition',indices),
                        lambda :self.getCommand('setVelocity',indices),
                        lambda :self.getCommand('setPID',indices),
                        lambda :self.getCommand('moveToPosition',indices),
                        lambda :self.getCommand('setPiecewiseLinear',indices),
                        lambda :self.getCommand('setTorque',indices)])
                baseControlMode = res[1]
            elif desiredControlMode == 'setVelocity':
                res=_try_methods(interface,['setVelocity','setPosition','setPID','moveToPosition','setPiecewiseLinear','setTorque'],
                        [lambda:self.getCommand('setVelocity',indices),
                        lambda :self.getCommand('setPosition',indices),
                        lambda :self.getCommand('setPID',indices),
                        lambda :self.getCommand('moveToPosition',indices),
                        lambda :self.getCommand('setPiecewiseLinear',indices),
                        lambda :self.getCommand('setTorque',indices)])
                baseControlMode = res[1]
            elif desiredControlMode == 'setTorque':
                res=_try_methods(interface,['setTorque','setPID'],
                        [lambda:self.getCommand('setTorque',indices),
                        lambda :self.getCommand('setPID',indices)])
                baseControlMode = res[1]
            else:
                raise RuntimeError("Invalid emulator control type? "+','.join(str(j.controlMode) for j in self.jointData))
            interface._control_mode_map[desiredControlMode] = baseControlMode
        
        if isinstance(indices,list):
            indices = tuple(indices)
        stateFilters = self.stateFilters.get(indices,None)
        if stateFilters is not None:
            #process filters
            input_variables = {}
            #query the inputs as needed
            for k,(filter,inputs,outputs) in stateFilters.items():
                skip = False
                for v in inputs:
                    if v in input_variables: continue
                    try:
                        input_variables[v] = self._getFilterInput(indices,v)
                    except ValueError:
                        if interface.hasSensor(v):
                            #acts on a sensor, but the sensor is not enabled
                            skip = True
                        else:
                            raise
                if skip: continue

                args = [input_variables[v] for v in inputs]
                results = self._runFilter(k,filter,args)
                for k,r in enumerate(outputs,results):
                    if k in self.virtualSensorMeasurements:
                        self.virtualSensorMeasurements[k] = (self.curClock,r)
                    else:
                        raise RuntimeError("Invalid output item {}".format(k))

        self.onBaseCommand(desiredControlMode, baseControlMode, indices)
        return (desiredControlMode,baseControlMode)

    def desiredControlMode(self,indices=None):
        """Returns the highest level control mode requested of the joints.
        Returns None if no command is active.

        if indices is not None, then it returns the desired control mode for 
        the joints referred to by indices.
        """
        activeJoints = self.jointData
        if indices is not None:
            activeJoints = [self.jointData[j] for j in indices]
        if all(j.commandSent for j in activeJoints):
            return None
        commonMode = None
        precedence = 10000
        for j in activeJoints:
            if j.controlMode != commonMode:
                if commonMode is None:
                    commonMode = j.controlMode
                elif j.controlMode is None:
                    pass
                elif _JointInterfaceEmulatorData.CONTROL_MODE_PRECEDENCE[j.controlMode] < precedence:
                    precedence = _JointInterfaceEmulatorData.CONTROL_MODE_PRECEDENCE[j.controlMode]
                    commonMode = j.controlMode
        if commonMode is not None and any(j.controlMode is None for j in activeJoints):
            #some joint controller is off but others are on.
            #shouldn't promote to piecewise linear or piecewise cubic
            if precedence > _JointInterfaceEmulatorData.CONTROL_MODE_PRECEDENCE['setPosition']:
                commonMode = 'setPosition'
        return commonMode

    def getCommand(self,commandType,indices=None):
        """Retrieves the args for the command to be sent to the actual 
        interface on this time step. 

        if indices is not None, then it returns the command for 
        the joints referred to by indices.
        """
        activeJoints = self.jointData
        if indices is not None:
            activeJoints = [self.jointData[j] for j in indices]
        if isinstance(indices,list):
            indices = tuple(indices)
        res = [j.getCommand(commandType) for j in activeJoints]

        commandFilters = self.commandFilters.get(indices,None)
        if commandFilters and self.desiredControlMode(indices) != commandType:
            #if command items have been emulated, run the appropriate command filters here
            underlyingCommandTypes = []
            if commandType in ['setPosition','moveToPosition']: underlyingCommandTypes = ['positionCommand']
            elif commandType == 'setVelocity': underlyingCommandTypes = ['velocityCommand']
            elif commandType == 'setTorque': underlyingCommandTypes = ['torqueCommand']
            elif commandType == 'setPiecewiseLinear': underlyingCommandTypes = ['positionCommand']
            elif commandType == 'setPiecewiseCubic': underlyingCommandTypes = ['velocityCommand']
            #process filters
            input_variables = {}
            for (name,(filter,inputs,outputs)) in commandFilters.items():
                if any(c in outputs for c in underlyingCommandTypes):
                    args = []
                    for i in inputs:
                        if i not in input_variables:
                            input_variables[i] = self._getFilterInput(indices,i)
                        args.append(input_variables[i])


        #print("Extracting",commandType,":",len(res),"values for", len(indices) if indices else 'all',"joints")
        if commandType in ['setVelocity','setTorque']:
            #extract out the TTL, take the max
            ttl = None
            if any(x[1] is not None for x in res):
                ttl = max(x[1] for x in res if x[1] is not None)
            cmd = list(x[0] for x in res)
            return cmd,ttl
        if commandType == 'setPiecewiseLinear':
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
            t0 = self.curClock if self.curClock is not None else 0
            futureTimes = []
            futureMilestones = []
            for i,t in enumerate(unifiedTimes):
                if t > t0:
                    futureTimes.append(t-t0)
                    futureMilestones.append(unifiedMilestones[i])
            # if len(futureTimes)==0:
            #     warnings.warn("getCommand is returning empty command because no times are after current time???")
            return futureTimes,futureMilestones
        if commandType == 'setPiecewiseCubic':
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
            # if len(futureTimes)==0:
            #     warnings.warn("getCommand is returning empty command because no times are after current time???")
            return futureTimes,futureMilestones,futureVelocities
        return list(zip(*res))
    
    def onBaseCommand(self,desiredControlMode,sentControlMode,indices=None):
        """To be called when the completer has decided which base control mode to use."""
        if desiredControlMode is None:
            return
        activeJoints = self.jointData
        if indices is not None:
            activeJoints = [self.jointData[j] for j in indices]
        if desiredControlMode in ['setPiecewiseCubic','setPiecewiseLinear']:
            if sentControlMode in ['setPiecewiseCubic','setPiecewiseLinear']:
                for j in activeJoints: j.commandSent = True
        elif desiredControlMode == 'setPID':
            if sentControlMode in ['setPID','setPosition','moveToPosition']:
                for j in activeJoints: j.commandSent = True
        elif desiredControlMode == 'setPosition':
            if sentControlMode in ['setPosition','moveToPosition','setPiecewiseLinear']:
                for j in activeJoints: j.commandSent = True
        elif desiredControlMode == 'setVelocity':
            if sentControlMode == ['setVelocity','setPiecewiseLinear']:
                for j in activeJoints: j.commandSent = True
        elif desiredControlMode == 'setTorque':
            if sentControlMode in ['setTorque','setPID']:
                for j in activeJoints: j.commandSent = True
        if desiredControlMode not in ['setPiecewiseCubic','setPiecewiseLinear']:
            for j in self.jointData:  #continue with trajectory if any joint wants it
                if j.controlMode in ['setPiecewiseCubic','setPiecewiseLinear']:
                    j.commandSent = False

    def promote(self,indices,controlType):
        """To accept commands of the given controlMode, switches over the
        current state to the controlMode"""
        activeJoints = self.jointData
        if indices is not None:
            activeJoints = [self.jointData[j] for j in indices]
        if controlType == 'setPiecewiseLinear':
            if any(j.controlMode == 'setPiecewiseCubic' and len(j.trajectoryTimes) > 1 for j in activeJoints):
                warnings.warn("RobotInterfaceCompleter: Warning, converting from piecewise cubic to piecewise linear trajectory")
        for j in activeJoints:
            j.commandSent = False
            j.promote(controlType)
        for j in activeJoints:
            if j.externalController:
                j.externalController.active = False
                j.externalController = None

    def setFilter(self,indices, name:str, filter:Callable, input='auto', output='auto') -> None:
        if isinstance(indices,list):
            indices = tuple(indices)
        assert isinstance(indices,tuple),"Don't accept None as indices"
        commandFilters = self.commandFilters.setdefault(indices,dict())
        stateFilters = self.stateFilters.setdefault(indices,dict())
        if filter is None:
            if name in commandFilters:
                del commandFilters[name]
            if name in stateFilters:
                del stateFilters[name]
            return
        if input == 'auto':
            if name in ['joint_limit','self_collision','obstacle_collision']:
                input = 'positionCommand'
            elif name == 'velocity_limit':
                input = 'velocityCommand'
            elif name == 'acceleration_limit':
                input = 'accelerationCommand'
            elif name == 'torque_limit':
                input = 'torqueCommand'
            elif name == 'tracking_monitor':
                input = ['commandedPosition','sensedPosition']
            else:
                input = name
        if output == 'auto':
            if name in ['joint_limit','self_collision','obstacle_collision']:
                output = 'positionCommand'
            elif name == 'velocity_limit':
                output = 'velocityCommand'
            elif name == 'acceleration_limit':
                output = 'accelerationCommand'
            elif name == 'torque_limit':
                output = 'torqueCommand'
            elif name == 'tracking_monitor':
                output = []
            else:
                output = name
        if not isinstance(input,(list,tuple)):
            input = [input]
        COMMANDS = ['positionCommand','velocityCommand','accelerationCommand','torqueCommand']
        STATES = MOTION_QUERY_METHODS
        #validate inputs
        for i in input:
            if i in COMMANDS:
                continue
            try:
                meth = getattr(self,i)
            except AttributeError:
                try:
                    meth = getattr(self,'get'+i[0].upper()+i[1:])
                except AttributeError:
                    raise ValueError("Input {} isn't recognized".format(i))
        if output is None:
            output = []
        elif not isinstance(output,(list,tuple)):
            output = [output]
        for o in output:
            if o in STATES:
                raise ValueError("Can't output an existing state variable")
            if o not in COMMANDS:
                self.virtualSensorMeasurements[o] = (None,None)

        if isinstance(filter,blocks.Block):
            block = filter
            filter = lambda *args:block.advance(*args)
        if all(i in COMMANDS for i in input):
            if all(o in COMMANDS for o in output):
                commandFilters[name] = (filter,input,output)
            elif any(o in COMMANDS for o in output):
                raise NotImplementedError("TODO: mixed command / state filters")
            else:
                stateFilters[name] = (filter,input,output)
        else:
            if any(o in COMMANDS for o in output):
                raise NotImplementedError("TODO: mixed command / state filters")
            stateFilters[name] = (filter,input,output)
    
    def _validateFilters(self, interface : RobotInterfaceBase, indices):
        COMMANDS = ['positionCommand','velocityCommand','accelerationCommand','torqueCommand']
        STATES = MOTION_QUERY_METHODS
        if isinstance(indices,list):
            indices = tuple(indices)
        stateFilters = self.stateFilters.get(indices,dict())
        commandFilters = self.commandFilters.get(indices,dict())
        for (name,(filter,input,output)) in stateFilters.items():
            if not callable(filter):
                raise ValueError("Invalid filter, not callable")
            isSensor = False
            for i in input:
                if i not in COMMANDS and i not in STATES and i not in self.virtualSensorMeasurements:
                    if interface.hasSensor(i):
                        isSensor = True
                    else:
                        raise ValueError("Invalid input to filter {}, input {} isn't a sensed or commanded value".format(name,i))
            for o in output:
                if o not in self.virtualSensorMeasurements:
                    raise ValueError("Invalid output of filter {}, output {} isn't a virtual sensor".format(name,i))
        for (name,(filter,input,output)) in commandFilters.items():
            if not callable(filter):
                raise ValueError("Invalid filter, not callable")
            isSensor = False
            for i in input:
                if i not in COMMANDS and i not in STATES and i not in self.virtualSensorMeasurements:
                    if interface.hasSensor(i):
                        isSensor = True
                    else:
                        raise ValueError("Invalid input to filter {}, input {} isn't a sensed or commanded value".format(name,i))
            for o in output:
                if o not in COMMANDS:
                    raise ValueError("Invalid output of filter {}, output {} isn't a commanded value".format(name,i))

    def setJointLimits(self,indices,qmin,qmax,op,hw_qmin=None,hw_qmax=None):
        if qmin == 'auto' or qmax == 'auto':
            model = self.klamptModel
            if model is None:
                if hw_qmin is None:
                    raise ValueError("Can't set a joint limit filter with 'auto' limits when klamptModel is not defined")
                else:
                    if qmin == 'auto': qmin = hw_qmin
                    if qmax == 'auto': qmax = hw_qmax
            else:
                mqmin,mqmax = [],[]
                for i in range(model.numDrivers()):
                    a,b = model.driver(i).getLimits()
                    if model.getJointType(model.driver(i).getAffectedLink()) == 'spin':
                        a = 0
                        b = math.pi*2
                    mqmin.append(a)
                    mqmax.append(b)
                if qmin == 'auto': qmin = [mqmin[i] for i in indices]
                if qmax == 'auto': qmax = [mqmax[i] for i in indices]
        if hw_qmin is not None:
            qmin = vectorops.maximum(qmin,hw_qmin)
            qmax = vectorops.minimum(qmax,hw_qmax)
        assert len(qmin) == len(indices)
        assert len(qmax) == len(indices)
        self.setFilter(indices,"joint_limit",LimitFilter(op,qmin,qmax))
    
    def setCollisionFilter(self,world,op):
        from ..model.collide import WorldCollider
        robot = self.klamptModel
        indices = list(range(len(self.jointData)))
        if robot is None:
            raise ValueError("Can't set a collision filter when klamptModel is not defined")
        if world is None:
            self.setFilter(indices,'self_collision',SelfCollisionFilter(op,robot))
        else:
            obs = []
            for i in range(world.numTerrains()):
                obs.append(world.terrain(i).geometry())
            for i in range(world.numRigidObjects()):
                obs.append(world.rigidObject(i).geometry())
            self.setFilter(indices,'obstacle_collision',CollisionFilter(op,robot,obs))
    
    def _getFilterInput(self,indices,value):
        #TODO: calculate accelerations
        if value == 'status': return self.status
        elif value.endswith('Command'): return getattr('commanded'+value[:-7].capitalize())(indices)
        try:
            return getattr(self,value)(indices)
        except AttributeError:
            try:
                return getattr(self,'get'+value[0].upper()+value[1:])(indices)
            except AttributeError:
                raise ValueError("Invalid filter input {}".format(value))
    
    def _runFilter(self,indices,name,func,args):
        try:
            results = func(*args)
            return results
        except blocks.utils.BlockSignal as e:
            if e.signal == 'softStop':
                self.softStop(indices)
                return
            elif e.signal == 'warn':
                print("Filter",name,"produced warning",e)
            else:
                print("Filter",name,"raised an unknown signal type?",e.type,e)

    def commandFilter(self,indices,item,value):
        if isinstance(indices,list):
            indices = tuple(indices)
        commandFilters = self.commandFilters.get(indices,dict())
        for (k,f) in commandFilters.items():
            (func,inputs,outputs) = f
            if item in inputs:
                args =[]
                for i in inputs:
                    if i==item: args.append(value)
                    else:
                        args.append(self._getFilterInput(indices,i))
                res = self._runFilter(indices,k,func,args)
                if res is not None:
                    assert len(res)==len(value),"Invalid result from filter {}".format(k)
                    value = res
        return value
    
    def reset(self):
        self.status = 'ok'
    
    def softStop(self,indices):
        self.status = 'softStop'
        self.promote(range(len(self.jointData)),None)  #clear pending commands
        qcmd = self.commandedPosition()
        qcmd = [qcmd[i] for i in indices]
        self.setPosition(indices,qcmd)
    
    def estop(self,indices):
        self.status = 'estop'
        self.promote(range(len(self.jointData)),None)  #clear pending commands
        qcmd = self.commandedPosition()
        qcmd = [qcmd[i] for i in indices]
        self.setPosition(indices,qcmd)

    def setPosition(self,indices,q):
        """Backup: runs an immediate position command"""
        if self.status != 'ok': return
        self.promote(indices,'setPosition')
        for (i,v) in zip(indices,q):
            self.jointData[i].commandedPosition = v

    def moveToPosition(self,indices,q,speed):
        """Backup: runs a position command using a piecewise linear trajectory"""
        if self.status != 'ok': return
        model = self.klamptModel
        if model is None:
            #TODO: warn that move-to is unavailable?
            warnings.warn("moveToPosition is not available because base controller's klamptModel() is not implemented.")
            self.promote(indices,'setPosition')
            for (i,v) in zip(indices,q):
                self.jointData[i].commandedPosition = v
        else:
            #move to command emulation using bounded velocity curve 
            xmin, xmax = [], []
            vmax, amax = [], []
            for i in indices:
                a,b = model.driver(i).getLimits()
                xmin.append(a)
                xmax.append(b)
                a,b = model.driver(i).getVelocityLimits()
                vmax.append(min(-a,b)*speed)
                a,b = model.driver(i).getAccelerationLimits()
                amax.append(min(-a,b)*(speed**2))
            assert all(v >= 0 for v in vmax)
            assert all(v >= 0 for v in amax)
            assert all(not math.isinf(v) for v in amax),"Infinite acceleration limits not supported"
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
                ts,xs,vs = motionplanning.interpolate_nd_min_time(qcmd,dqcmd,q,[0]*len(q),xmin,xmax,vmax,amax)
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
            try:
                ts,xs,vs = motionplanning.combine_nd_cubic(ts,xs,vs)
            except Exception as e:
                print("Couldn't solve for combine?")
                print("qcmd",qcmd,"q:",q)
                print("qmin:",xmin,"qmax:",xmax)
                print("vcmd:",dqcmd,"vmax:",vmax)
                print("amax:",amax)
                print(ts)
                print(xs)
                print(vs)
                raise
            self.setPiecewiseCubic(indices,ts,xs,vs,True)

    def setVelocity(self,indices,v,ttl):
        """Backup: runs a velocity command for ttl seconds using a piecewise linear
        trajectory.  If ttl is not specified, uses ttl=1."""
        if self.status != 'ok': return
        qcmd = [self.jointData[i].commandedPosition if self.jointData[i].commandedPosition is not None else self.jointData[i].sensedPosition for i in indices]
        if ttl is None:
            ttl = 1.0
            model = self.klamptModel
            if model is not None:
                limits = [model.driver(i).getLimits() for i in indices]
                xmin,xmax = zip(*limits)
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
        if self.status != 'ok': return
        self.promote(indices,'setTorque')
        for i,v in zip(indices,t):
            self.jointData[i].commandedTorque = v
            self.jointData[i].commandTTL = ttl

    def setPID(self,indices,q,dq,t):
        if self.status != 'ok': return
        if t is None:
            t = [0.0]*len(q)
        self.promote(indices,'setPID')
        for i,qi,dqi,ti in zip(indices,q,dq,t):
            self.jointData[i].pidCmd = (qi,dqi,ti)

    def setPIDGains(self,indices,kP,kI,kD):
        if self.status != 'ok': return
        for i,P,I,D in zip(indices,kP,kI,kD):
            self.jointData[i].pidGains = (kP,kI,kD)

    def getPIDGains(self,indices):
        if self.status != 'ok': return
        kP,kI,kD = [],[],[]
        for i in indices:
            if self.jointData[i].pidGains is None:
                if kP:
                    raise RuntimeError("Some joints have PID gains and others do not?")
            else:
                P,I,D = self.jointData[i].pidGains
                kP.append(P)
                kI.append(I)
                kD.append(D)
        if not kP:
            return None,None,None
        return kP,kI,kD

    def setPiecewiseLinear(self,indices,ts,qs,relative):
        assert self.curClock is not None
        assert len(ts) == len(qs)
        if self.status != 'ok': return
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
            q0 = []
            for i in indices:
                if self.jointData[i].commandedPosition is None:
                    if self.jointData[i].sensedPosition is None:
                        raise RuntimeError("Can't set a trajectory before first controller clock cycle")
                    q0.append(self.jointData[i].sensedPosition)
                else:
                    q0.append(self.jointData[i].commandedPosition)
            qs = [q0] + qs

        self.promote(indices,'setPiecewiseLinear')
        for k,i in enumerate(indices):
            self.jointData[i].trajectoryTimes = ts
            self.jointData[i].trajectoryMilestones = [q[k] for q in qs]

    def setPiecewiseCubic(self,indices,ts,qs,vs,relative):
        assert self.curClock is not None
        assert len(ts) == len(qs)
        assert len(ts) == len(vs)
        if self.status != 'ok': return
        if len(ts)==0:
            return
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

        self.promote(indices,'setPiecewiseCubic')
        for k,i in enumerate(indices):
            self.jointData[i].trajectoryTimes = ts
            self.jointData[i].trajectoryMilestones = [q[k] for q in qs]
            self.jointData[i].trajectoryVelocities = [v[k] for v in vs]

    def isMoving(self,indices):
        for i in indices:
            j = self.jointData[i]
            if j.controlMode == 'setVelocity':
                if j.commandedVelocity != 0:
                    return True
            elif j.controlMode == 'setPosition':
                if abs(j.commandedPosition - j.sensedPosition) > 1e-3:
                    return True
            elif j.controlMode == 'setPID':
                if j.commandedVelocity != 0:
                    return True
                if abs(j.commandedPosition - j.sensedPosition) > 1e-3:
                    return True
            elif j.controlMode in ['setPiecewiseLinear','setPiecewiseCubic']:
                if self.curClock < j.destinationTime(self.curClock):
                    return True
        return False

    def commandedPosition(self):
        res = [j.commandedPosition for j in self.jointData]
        if not _valid_vector(res): raise RuntimeError("commandedPosition queried before a step could be taken?")
        return res

    def commandedVelocity(self):
        res = [j.commandedVelocity for j in self.jointData]
        if not _valid_vector(res): raise RuntimeError("commandedVelocity queried before a step could be taken?")
        return res

    def commandedTorque(self):
        res = [j.commandedTorque for j in self.jointData]
        if not _valid_vector(res): raise RuntimeError("no commanded torque sent to emulator")
        return res

    def sensedPosition(self):
        res = [j.sensedPosition for j in self.jointData]
        if not _valid_vector(res): raise RuntimeError("sensedPosition queried before a step could be taken?")
        return res

    def sensedVelocity(self):
        return [j.sensedVelocity for j in self.jointData]

    def destinationPosition(self):
        return [j.destinationPosition() for j in self.jointData]

    def destinationVelocity(self):
        return [j.destinationVelocity() for j in self.jointData]

    def destinationTime(self):
        return max(j.destinationTime(self.curClock) for j in self.jointData)

    def setToolCoordinates(self,indices,xtool_local):
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

    def setGravityCompensation(self,indices,gravity,load,load_com):
        raise NotImplementedError("TODO: implement gravity compensation?")

    def getGravityCompensation(self,indices):
        raise NotImplementedError("TODO: implement gravity compensation?")

    def setCartesianPosition(self,indices,xparams,frame):
        if self.status != 'ok': return
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

    def moveToCartesianPosition(self,indices,xparams,speed,frame):
        assert speed > 0
        if self.status != 'ok': return
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

    def moveToCartesianPositionLinear(self,indices,xparams,speed,frame):
        assert speed > 0
        if self.status != 'ok': return
        try:
            c = self.cartesianInterfaces[tuple(indices)]
        except KeyError:
            c = _CartesianEmulatorData(self.klamptModel,indices)
            self.cartesianInterfaces[tuple(indices)] = c
        for i,j in enumerate(self.jointData):
            self.klamptModel.driver(i).setValue(j.sensedPosition if j.commandedPosition is None else j.commandedPosition)
        qstart = [j.sensedPosition if j.commandedPosition is None else j.commandedPosition for j in self.jointData]
        c.moveToCartesianPositionLinear(qstart,xparams,speed,frame)
        for i in indices:
            self.jointData[i].externalController = c

    def setCartesianVelocity(self,indices,dxparams,ttl,frame):
        if self.status != 'ok': return
        try:
            c = self.cartesianInterfaces[tuple(indices)]
        except KeyError:
            c = _CartesianEmulatorData(self.klamptModel,indices)
            self.cartesianInterfaces[tuple(indices)] = c
        qstart = [j.sensedPosition if j.commandedPosition is None else j.commandedPosition for j in self.jointData]
        c.setCartesianVelocity(qstart,dxparams,ttl,frame)
        for i in indices:
            self.jointData[i].externalController = c

    def setCartesianForce(self,indices,fparams,ttl,frame):
        if self.status != 'ok': return
        try:
            c = self.cartesianInterfaces[tuple(indices)]
        except KeyError:
            c = _CartesianEmulatorData(self.klamptModel,indices)
            self.cartesianInterfaces[tuple(indices)] = c
        raise NotImplementedError("TODO: implement cartesian force?")
        qstart = [j.sensedPosition if j.commandedPosition is None else j.commandedPosition for j in self.jointData]
        c.setCartesianForce(qstart,fparams,ttl,frame)

    def cartesianPosition(self,indices,q,frame):
        """
        Emulates cartesianPosition. Can also get link transforms.

        Args:
            indices (str, int, or tuple): if str or int, retrieves a link
                transform, and ``frame`` must be "world".  If a tuple,
                retrieves the position of the cartesian controller for the
                given indices, in the given frame.
            q (Vector): the configuration of the full body
            frame (str): either "world", "base", "end effector", or "tool"
        """
        assert len(q) == len(self.jointData),"cartesianPosition: must use full-body configuration, {} != {}".format(len(q),len(self.jointData))
        if any(v is None for v in q):
            q = [0.0 if v is None else v for v in q]
        if isinstance(indices,(str,int)):  #single link 
            assert frame=='world','Can only retrieve world coordinates for single link'
            self.klamptModel.setConfig(self.klamptModel.configFromDrivers(q))
            return self.klamptModel.link(indices).getTransform()
        try:
            c = self.cartesianInterfaces[tuple(indices)]
            return c.cartesianPosition(q,frame)
        except KeyError:
            raise ValueError("Invalid Cartesian index set for emulator: no command currently set")

    def cartesianVelocity(self,indices,q,dq,frame):
        """
        Emulates cartesianVelocity. Can also get link velocities.

        Args:
            indices (str, int, or tuple): if str or int, retrieves a link
                velocity, and ``frame`` must be "world".  If a tuple,
                retrieves the velocity of the cartesian controller for the
                given indices, in the given frame.
            q (Vector): the configuration of the full body
            dq (Vector): the joint velocities of the full body
            frame (str): either "world", "base", "end effector", or "tool"
        """
        assert len(q) == len(self.jointData),"cartesianVelocity: must use full-body configuration, {} != {}".format(len(q),len(self.jointData))
        assert len(dq) == len(self.jointData),"cartesianVelocity: must use full-body configuration, {} != {}".format(len(dq),len(self.jointData))
        if any(v is None for v in q):
            q = [0.0 if v is None else v for v in q]
        if any(v is None for v in dq):
            dq = [0.0 if v is None else v for v in dq]
        if isinstance(indices,(str,int)):  #single link 
            assert frame=='world','Can only retrieve world coordinates for single link'
            self.klamptModel.setConfig(self.klamptModel.configFromDrivers(q))
            self.klamptModel.setVelocity(self.klamptModel.velocityFromDrivers(dq))
            link = self.klamptModel.link(indices)
            return link.getAngularVelocity(),link.getVelocity()
        try:
            c = self.cartesianInterfaces[tuple(indices)]
            return c.cartesianVelocity(q,dq,frame)
        except KeyError:
            raise ValueError("Invalid Cartesian index set for emulator: no command currently set")

    def cartesianForce(self,indices,q,t,frame):
        assert len(q) == len(self.jointData),"cartesianForce: must use full-body configuration, {} != {}".format(len(q),len(self.jointData))
        assert len(t) == len(self.jointData),"cartesianForce: must use full-body configuration, {} != {}".format(len(t),len(self.jointData))
        if any(v is None for v in q):
            q = [0.0 if v is None else v for v in q]
        if any(v is None for v in t):
            t = [0.0 if v is None else v for v in t]
        try:
            c = self.cartesianInterfaces[tuple(indices)]
            return c.cartesianForce(q,t,frame)
        except KeyError:
            raise ValueError("Invalid Cartesian index set for emulator: no command currently set")

    def printStatus(self):
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
        self.properties['part'] = True
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

    def status(self,joint_idx=None):
        if self._base_part_interface is not None:
            return self._base_part_interface.status(joint_idx)
        return RobotInterfaceCompleter.status(self,joint_idx)

    def estop(self):
        #if self._base_part_interface is not None:
        #    self._base_part_interface.estop()
        #Do we want the whole robot to stop, or just the part?
        return RobotInterfaceCompleter.estop(self)

    def reset(self):
        if self._base_part_interface is not None:
            return self._base_part_interface.reset()
        return RobotInterfaceCompleter.reset(self)

    def addPart(self,name,indices):
        raise RuntimeError("Can't add sub-parts to part")

    def initialize(self):
        raise RuntimeError("Can't call initialize() on a sub-robot interface.")

    def beginStep(self):
        raise RuntimeError("Can't call beginStep() on a sub-robot interface.")

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

    def sensedCartesianPosition(self,frame='world'):
        return self.cartesianPosition(self._parent.sensedPosition(),frame)

    def sensedCartesianVelocity(self,frame='world'):
        return self.cartesianVelocity(self._parent.sensedPosition(),self._parent.sensedVelocity(),frame)

    def sensedCartesianForce(self,frame='world'):
        return self.cartesianForce(self._parent.sensedPosition(),self._parent.sensedTorque(),frame)

    def commandedCartesianPosition(self,frame='world'):
        return self.cartesianPosition(self._parent.commandedPosition(),frame)

    def commandedCartesianVelocity(self,frame='world'):
        return self.cartesianVelocity(self._parent.commandedPosition(),self._parent.commandedVelocity(),frame)

    def commandedCartesianForce(self,frame='world'):
        return self.cartesianForce(self._parent.commandedPosition(),self._parent.commandedTorque(),frame)

    def destinationCartesianPosition(self,frame='world'):
        return self.cartesianPosition(self._parent.destinationPosition(),frame)

    def destinationCartesianVelocity(self,frame='world'):
        return self.cartesianVelocity(self._parent.destinationPosition(),self._parent.destinationVelocity())

    def queuedCartesianTrajectory(self,frame='world'):
        res = self._parent.queuedTrajectory()
        if len(res) == 2:
            ts,qs = res
            return ts,[self.cartesianPosition(q,frame) for q in qs]
        elif len(res) == 3:
            ts,qs,vs = res
            return ts,[self.cartesianPosition(q,frame) for q in qs],[self.cartesianVelocity(q,dq,frame) for q,dq in zip(qs,vs)]
        else:
            raise RuntimeError("Invalid result from queuedTrajectory")


class RobotInterfaceLogger:
    """A logger that will write parts of the RIL state to a CSV file.

    TODO: make this a filter.
    """

    TEST_PROPERTIES = ['controlRate','parts','sensors','enabledSensors','numJoints','indices']
    TEST_METHODS = ['clock','status','isMoving',
                'sensedPosition','sensedVelocity','sensedTorque','commandedPosition','commandedVelocity','commandedTorque',
                'destinationPosition','destinationVelocity','destinationTime']

    def __init__(self, interface : RobotInterfaceBase, file : Union[TextIO,str]):
        import csv
        self._interface = interface
        if isinstance(file,str):
            self._file = open(file,'w')
        else:
            self._file = file
        self._writer = csv.writer(self._file)
        self._first_time = True
    
    def step(self):
        if self._file is None:
            raise RuntimeError("step() called after stop()")
        if self._first_time:
            results = []
            for prop in RobotInterfaceLogger.TEST_PROPERTIES:
                try:
                    results.append(str(getattr(self._interface,prop)()))
                except Exception as e:
                    results.append('Error '+str(e))
            self._writer.writerow(RobotInterfaceLogger.TEST_PROPERTIES)
            self._writer.writerow(results)
            self._writer.writerow(RobotInterfaceLogger.TEST_METHODS)
        for func in RobotInterfaceLogger.TEST_METHODS:
            try:
                results.append(str(getattr(self._interface,func)()))
            except Exception as e:
                results.append('Error '+str(e))
        self._writer.writerow(results)
    
    def stop(self):
        self._file.close()
        self._file = None
        self._writer = None


class RobotInterfaceRecorder:
    """Records / playbacks all RIL commands to disk. 

    Record usage::

        interface = MyRobotInterface(...)
        recorder = RobotInterfaceRecorder(interface,'recording.txt')
        while True:
            ... issue commands
            recorder.step()      #this does the work
            interface.endStep()
            ... sleep, visualize, etc
        recorder.stop()

    Playback usage::
    
        interface = MyRobotInterface(...)
        recorder = RobotInterfaceRecorder(interface,'recording.txt',playback=True)
        while not recorder.done():
            recorder.step()      #this does the work
            interface.endStep()
            ... sleep, visualize, etc

    Args:
        interface (RobotInterfaceBase): NOTE: must be stateful.
        file (str or file-like): the file to save to / load from
        playback (bool): whether to use record or playback mode
        playback_delay (float, optional): when in playback mode, how long to
            wait before the first command is issued.  If None, uses the same
            timing as in the original recording (assumes clocks start at 0).

    """
    def __init__(self, interface : RobotInterfaceBase, file = Union[TextIO,str], playback=False, playback_delay=None):
        self._interface = interface
        if not hasattr(interface,'_commands'):
            raise ValueError("The interface needs to be stateful.  Try wrapping in a RobotInterfaceCompleter or _RobotInterfaceStatefulWrapper")
        if isinstance(file,str):
            if playback:
                self._file = open(file,'r')
            else:
                self._file = open(file,'w')
        else:
            self._file = file
        self._playback = playback
        if playback:
            import json
            self._commands = []
            while True:
                try:
                    jsonobj = json.load(self._file)
                except IOError:
                    break
                time = float(jsonobj['time'])
                cmd = _RobotInterfaceCommand(jsonobj['command'])
                self._commands.append((time,cmd))
            self._timewarp = 0
            self._command_index = 0
            if self._commands and playback_delay is not None:
                self._timewarp = playback_delay - self._commands[0][0]
            if isinstance(file,str):
                self._file.close()
    
    def step(self) -> None:
        """Call this after commands have been issued to ``interface`` but
        before ``interface.endStep()``.
        """
        if self._file is None:
            return
        if self._playback:
            t = self._interface.clock()
            while self._command_index < len(self._commands):
                if t >= self._commands[self._command_index][0] + self._timewarp:
                    cmd = self._commands[self._command_index][1]
                    self._interface._commands.append(cmd)
                else:
                    break
                self._command_index += 1
        else:
            import json
            t = self._interface.clock()
            cmds = self._interface._commands
            for cmd in cmds:
                cmd_json = cmd.to_json()
                json.dump([t,cmd_json],self._file)

    def stop(self) -> None:
        if self._file is None:
            return
        if not self._playback:
            self._file.close()
        self._file = None
    
    def done(self) -> bool:
        if self._file is None:
            return True
        if self._playback:
            return self._command_index >= len(self._commands)
        else:
            return False
