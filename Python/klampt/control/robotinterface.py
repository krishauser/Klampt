"""The main module for Klampt's Robot Interface Layer.
"""

from ..robotsim import WorldModel,RobotModel
from ..model.subrobot import SubRobotModel
import functools
import warnings
from typing import Union,Optional,Any,Dict,List,Tuple,Callable
from klampt.model.typing import Vector,Vector3,RigidTransform

class RobotInterfaceBase(object):
    """The main class for the Klamp't Robot Interface Layer. Defines a unifying
    API to interface with a robot's motor controller, whether it's simulated or
    a real robot. 

    .. note::
        The API may look intimidating, but a subclass implementer is free to
        set up as many or as few of the given methods as the robot's motor
        controller truly implements.  The
        :class:`~klampt.control.robotinterfaceutils.RobotInterfaceCompleter`
        class will fill in the remainder of derived methods.  See the
        Functionalities section for more details.
    
    Each of these methods should be synchronous calls, called at a single time
    step.  The calling convention is::

        from klampt.control.utils import TimedLooper

        interface = MyRobotInterface(...args...)
        if not interface.initialize():  #should be called first
            raise RuntimeError("There was some problem initializing interface "+str(interface))
        
        dt = 1.0/interface.controlRate()
        looper = TimedLooper(dt) 
        while looper:           #this will run as close to the control rate as possible
            try:
                interface.beginStep()
                if interface.status() != 'ok':  #no error handling done here... could also try interface.reset()
                    raise RuntimeError("Some error occurred: {}".format(interface.status()))
                [do any state queries or commands here comprising the control loop]
                interface.endStep()
            except Exception as e:
                print("Terminating on exception:",e)
                looper.stop()
        
        interface.close()   #cleanly shut down the interface

    To accept asynchronous commands, a :class:`RobotInterfaceBase` subclass
    can be wrapped by a
    :class:`~klampt.control.robotinterfaceutils.ThreadedRobotInterface`
    or client (e.g.,
    :class:`~klampt.control.networkrobotinterface.XMLRPCClientRobotInterface`).
    Asynchronous usage allows sending  commands in a procedural fashion, e.g.::

        interface = MyRobotInterface(...args...)
        if not interface.initialize():  #should be called first
            raise RuntimeError("There was some problem initializing interface "+str(interface))

        time.sleep(whatever)
        [any state queries or commands here]

        interface.moveToPosition(qtarget)
        poll_rate = 0.05
        while interface.isMoving()
            time.sleep(poll_rate)

        if interface.status() != 'ok':  #no error handling done here... could also try interface.reset()
            raise RuntimeError("Some error occurred: {}".format(interface.status()))

        time.sleep(whatever)
        [any state queries or commands here]
        
        interface.close()   #cleanly shut down the interface


    **DOFs and Parts**

    The number of DOFs is assumed equal to the number of joint actuators / 
    encoders. If the robot has fewer actuators than encoders, the commands for 
    unactuated joints should just be ignored. If the robot corresponds to a 
    Klampt model (typical), then the number of DOFs should be
    ``model.numDrivers()``

    A robot can have "parts", which are named groups of DOFs.  For example, a
    robot with a gripper can have parts "arm" and "gripper", which can be 
    potentially controlled separately.  You may retrieve part names using 
    :meth:`parts`, part indices using :meth:`indices`, and (potentially)
    access a RIL interface to a part using :meth:`partInterface`. 

    It is suggested that these parts correspond with parts in the robot's 
    :class:`~klampt.model.robotinfo.RobotInfo` structure.


    **Semantics of methods**

    In the strictest sense, no methods are assumed to be available before
    :meth:`initialize` is called (and returns True) .

    The following methods should be available after initialize() and do
    not need to be protected by :meth:`beginStep` / :meth:`endStep`:

    * :meth:`parts`, :meth:`indices`, :meth:`numJoints`, and :meth:`jointName`:
      should be constant. (At the very least, no parts should be destroyed)
    * :meth:`partInterface`.
    * :meth:`partToRobotConfig` and :meth:`robotToPartConfig`: utility methods.
    * :meth:`klamptModel`: if available, must be constant.
    * :meth:`configToKlampt`, :meth:`configFromKlampt`, :meth:`velocityToKlampt`,
      and :meth:`velocityFromKlampt`: utility methods.
    * :meth:`cartesianPosition`, :meth:`cartesianVelocity`, and
      :meth:`cartesianForce`: utility methods.
    * :meth:`controlRate`: if available, should be constant.
    * :meth:`sensors`, :meth:`hasSensor`: should be constant.  If a sensor can
      be hot-swapped, it should be in the list of sensors but may appear/
      disappear from :meth:`enabledSensors`.

    Most implementations will have these methods available even before
    ``initialize()`` is called, but to be safe a caller shouldn't count on it.
    For example, if a robot tests to see which parts are connected at runtime,
    then ``parts()``, ``numJoints()``, ``klamptModel()``, etc will not be
    available until afterwards.

    All other methods should be placed within a :meth:`beginStep` /
    :meth:`endStep` pair.


    **Minimal functionality (For implementers)**

    There are a few functions your subclass will need to fill out:

    * :meth:`numJoints` or :meth:`klamptModel`
    * Either :meth:`clock` or :meth:`controlRate`
    * Either :meth:`setPosition`, :meth:`moveToPosition`, :meth:`setVelocity`, 
      :meth:`setTorque`, or :meth:`setPID`
    * Either :meth:`sensedPosition` or :meth:`commandedPosition`

    Pass your RobotInterfaceBase subclass to
    :class:`~klampt.control.robotinterfaceutils.RobotInterfaceCompleter` to
    complete the implementation of as many of the remaining items as possible.

    See the :class:`.simrobotinterface.SimPositionControlInterface` class 
    for an example that passes commands to a Klamp't physics simulator.


    **Cartesian control**

    Cartesian position, velocity, and force control may be implemented. 
    The interpretation of these arguments is implementation-dependent, but 
    we support two standard modes: 6DOF position/orientation and 3DOF
    position-only mode. 
    
    The reference frame for these quantities is specified by a ``frame``
    argument, which is 'world' by default:
    
    * 'world': The world frame.  Equal to 'base' for a top-level robot, but
       differs if this interface refers to a part of a larger robot.
    * 'base': The base of this part.  Equal to 'world' for a top-level robot,
       but differs if this interface refers to a part of a larger robot.
    * 'end effector': relative to the origin of the end effector frame.
    * 'tool': relative to the current tool frame (the tool is an offset from
       the end effector frame)

    Note that an interface might only support commands to one type of frame,
    and raise ``NotImplementedError()`` for other frames.


    **Real-time filters**

    Filters perform real-time enforcement of joint limits, velocity limits,
    collision checks, and signal processing. Typically filters
    are handled in software using a
    :class:`.robotinterfaceutils.RobotInterfaceCompleter` but an implementation
    is also allowed to send filter parameters to the hardware.

    Users can activate and deactivate filters using the :func:`setFilter`
    method.  A filter has an name, operation, input arguments, and output
    arguments, and its operation will usually yield an output or raise an
    exception, e.g., if a commanded position / velocity exceeds the limits,
    it can raise a FilterSoftStop exception, or it can just alter arguments,
    e.g., clamping to valid ranges. 
    
    Common examples include the following (using filter operations available in
    :mod:`robotinterfaceutils`):

    * ``setFilter('joint_limit',LimitClampFilter(qmin,qmax))``: clamps 
      commanded position to limits.
    * ``setFilter('joint_limit',LimitStopFilter(qmin,qmax))``: stops when
      commanded position exceeds limits.
    * ``setFilter('velocity_limit',LimitClampFilter(qmin,qmax))``: clamps 
      commanded velocity to limits.
    * ``setFilter('velocity_limit',LimitStopFilter(qmin,qmax))``: stops
      when commanded velocity exceeds limits.
    * ``setFilter('collision',CollisionStopFilter(collider))``: stops when
      the robot is about to collide.
    * ``setFilter('position_tracking',DifferenceStopFilter(math.radians(15)))``:
      stops when the tracking error between the commanded and sensed position
      exceeds 15 radians.
    * ``setFilter('position_delta',DifferenceStopFilter(math.radians(5)))``:
      stops when the commanded position jumps more than 5 radians in a
      single time step.
    * ``setFilter(SENSOR_NAME,FIRFilter(b),output=OUT_NAME)``: adds a FIR
      filter to sensor SENSOR_NAME and outputs it to OUT_NAME.
    * ``setFilter('custom_filter', block, ['sensedPosition','sensedVelocity'], 'positionCommand')``:
      sets a custom filter that uses a Block of two arguments (sensed position
      and velocity) and outputs one value (a position command).

    To disable a filter, pass None as the second argument.


    **Customization**

    Certain controllers will provide custom API methods that fall outside of 
    the standard API.  Although an RIL subclass is free to implement any
    methods desired, for maximum compatibility with utility classes (e.g.,
    RobotInterfaceCompleter), the subclass should provide uniform access to
    these custom methods through the following standard methods:

    * Global settings, which overload :func:`setSetting` and
      :func:`getSetting`.  Unlike properties, settings are expected to be
      changeable through the life of the interface.
    * Sensors, which overload :func:`sensors`, :func:`enableSensor`,
      :func:`sensorMeasurements`, and :func:`sensorUpdateTime`.
    * State queries, which overload :func:`stateValue`.
    * Control modes, which overload :func:`setControlMode`.
    * Custom calls and event triggers, which overload :func:`functionCall`


    Attributes:
        properties (dict): a dict from string key to property value. Properties
            are application-dependent and constant through the life of the
            interface. Examples may include:

            * 'name' (str): the name of the robot
            * 'version' (str): a version of this interface
            * 'simulated' (bool): whether the "robot" is simulated vs physical
            * 'klamptModelFile' (str): the file name of the Klamp't model file.
            * 'asynchronous' (bool): if True, beginStep/endStep are not needed to
              communicate with the robot.  Networked controllers are often
              asynchronous.
            * 'complete' (bool): if True, all methods are implemented.
            * 'part' (bool): if True, this is not a top-level interface.
            * 'joint_limits' (pair of Vector): the hardware joint limits, not
              overridable by software limits.
            * 'velocity_limits' (pair of Vector): the hardware velocity limits, not
              overridable by software limits.
            * 'acceleration_limits' (pair of Vector): the hardware accel limits,
              not overridable by software limits.
            * 'torque_limits' (pair of Vector): the hardware torque limits,
              not overridable by software limits.

    """
    def __init__(self,**properties):
        self.properties = properties
        self._worldModel = None
        self._klamptModel = None
        self._klamptDriverIndices = None
        self._warned = False

    def __str__(self):
        inner = []
        if 'name' in self.properties:
            inner.append(self.properties['name'])
        if 'version' in self.properties:
            inner.append(self.properties['version'])
        if len(inner)==0:
            return self.__class__.__name__
        else:
            if self.properties.get('name',None) == self.__class__.__name__:
                return ','.join(inner)
            return ','.join(inner) + '({})'.format(self.__class__.__name__)

    def initialize(self) -> bool:
        """Tries to connect to the robot.  Returns true if ready to send
        commands.  This should probably be the first method called.
        """
        return True

    def close(self) -> bool:
        """Cleanly shuts down any resources acquired using initialize()."""
        return True

    def startStep(self) -> None:
        """Deprecated. use beginStep instead."""
        if not self._warned:
            warnings.warn("startStep will be deprecated, use beginStep instead",DeprecationWarning)
        self._warned = True
        self.beginStep()

    def beginStep(self) -> None:
        """This is called before the commands sent at each time step"""
        pass

    def endStep(self) -> None:
        """This is called after the commands sent at each time step"""
        pass

    def numJoints(self,part: Optional[str] = None) -> int:
        """Returns the number of joints of the given part.  By default, this
        returns the number of actuated DOFs in the Klamp't model. 
        """
        if part is None:
            m = self.klamptModel()
            if m is None:
                raise NotImplementedError()
            return m.numDrivers()
        return len(self.parts()[part])

    def jointName(self, joint_idx: int) -> str:
        """Returns a string naming the given joint"""
        raise NotImplementedError()

    @functools.lru_cache(maxsize=None)
    def parts(self) -> Dict[Any,List[int]]:
        """Returns a dictionary of (part-name,configuration index list) pairs
        defining the named parts of the robot.

        Since this will be used a lot, make sure to declare your override with
        @functools.lru_cache.
        """
        return {None:list(range(self.numJoints()))}

    def indices(self,
            part: Optional[str] = None,
            joint_idx: Optional[int] = None
        ) -> List[int]:
        """Helper: returns a list of indices for the given part / joint index"""
        plist = self.parts()[part]
        if joint_idx is None:
            return plist
        assert joint_idx >= 0 and joint_idx < len(plist),"Invalid joint index for part "+str(part)
        return [plist[joint_idx]]

    def partInterface(self,
            part: str,
            joint_idx: Optional[int] = None
        ) -> 'RobotInterfaceBase':
        """Returns a RobotInterfaceBase that allows control of the given 
        part/joint.  If no such controller exists, raises a
        NotImplementedError.

        The part/joint controller should operate on exactly the DOFs specified
        by self.indices(part,joint_idx).
        """
        raise NotImplementedError()

    def controlRate(self) -> float:
        """Returns the control rate, in Hz"""
        raise NotImplementedError()

    def clock(self) -> float:
        """Returns the current time on the robot's clock, in seconds"""
        raise NotImplementedError()

    def status(self, joint_idx: Optional[int] = None) -> str:
        """Returns a status string for the robot / given joint.  'ok' means
        everything is OK."""
        return 'ok'

    def estop(self) -> None:
        """Calls an emergency stop on the robot. Default uses the soft stop."""
        self.softStop()

    def softStop(self) -> None:
        """Calls a software E-stop on the robot (braking as quickly as
        possible).  Default implementation stops robot at current position; a 
        better solution would slow the robot down.
        """
        self.setPosition(self.commandedPosition())

    def reset(self) -> None:
        """If the robot has a non-normal status code, attempt to reset it
        to normal operation.  The caller should poll until status()=='ok'
        """
        raise NotImplementedError("Reset is not implemented")

    def getSettings(self) -> Dict[str,Any]:
        """Retrieves an implementation-dependent dict of possible settings."""
        raise NotImplementedError("Settings are not implemented")

    def getSetting(self, name : str):
        """Retrieves an implementation-dependent setting."""
        raise KeyError(name+" isn't a valid setting")
    
    def setSetting(self, name : str, value) -> None:
        """Sets an implementation-dependent setting."""
        raise KeyError(name+" isn't a valid setting")
    
    def sensors(self) -> list:
        """Returns a list of names of possible sensors on this robot."""
        raise NotImplementedError()

    def enabledSensors(self) -> list:
        """Returns a list of names of enabled sensors on this robot."""
        raise NotImplementedError()

    def hasSensor(self, sensor: str) -> bool:
        """Returns true if the given sensor can be enabled.
        """
        return sensor in self.sensors()

    def enableSensor(self,
        sensor: str,
        enabled: bool=True) -> bool:
        """Enables / disables a sensor. Returns true if successful.
        """
        raise NotImplementedError()

    def sensorMeasurements(self, name: str):
        """Returns the latest measurements from a sensor.  Interpretation of
        the result is sensor-dependent.
        """
        raise NotImplementedError()

    def sensorUpdateTime(self, name: str) -> float:
        """Returns the clock time of the last sensor update."""
        raise NotImplementedError()
    
    def setControlMode(self,mode,*args,**kwargs):
        """Enables a custom control mode."""
        raise NotImplementedError()

    def functionCall(self,proc,*args,**kwargs):
        """Enables a custom one-off function call."""
        raise NotImplementedError()

    def isMoving(self, joint_idx: Optional[int] = None) -> bool:
        """Returns true if the robot / joint are currently moving"""
        raise NotImplementedError()

    def sensedPosition(self) -> Vector:
        """Retrieves the currently sensed joint position. 
        """
        raise NotImplementedError()

    def sensedVelocity(self) -> Vector:
        """Retrieves the currently sensed joint velocity. 
        """
        raise NotImplementedError()

    def sensedTorque(self) -> Vector:
        """Retrieves the currently sensed joint torque. 
        """
        raise NotImplementedError()

    def commandedPosition(self) -> Vector:
        """Retrieves the currently commanded joint position. 
        """
        raise NotImplementedError()

    def commandedVelocity(self) -> Vector:
        """Retrieves the currently commanded joint velocity. 
        """
        raise NotImplementedError()

    def commandedTorque(self) -> Vector:
        """Retrieves the currently commanded joint torque. 
        """
        raise NotImplementedError()

    def destinationPosition(self) -> Vector:
        """Retrieves the destination of a motion queue controller.
        """
        raise NotImplementedError()

    def destinationVelocity(self) -> Vector:
        """Retrieves the final velocity of a motion queue controller.
        """
        raise NotImplementedError()
    
    def destinationTime(self) -> float:
        """Retrieves the final clock time of a motion queue controller.
        """
        raise NotImplementedError()

    def queuedTrajectory(self) -> tuple:
        """Returns a trajectory starting from the current time representing all
        commands in a motion queue controller.

        Returns:
            tuple: either (ts,qs) or (ts,qs,vs) representing a piecewise linear
            or a piecewise-cubic trajectory.
        """
        raise NotImplementedError()
    
    def stateValue(self,name) -> Union[float,Vector]:
        """Retrieves some custom state value"""
        raise NotImplementedError(name + " is not a valid state query")
    
    def cartesianPosition(self, q: Vector, frame: str = 'world') -> RigidTransform:
        """Converts from a joint position vector to a cartesian position.

        Args:
            q (vector): the (whole) robot's joint positions.
            frame (str): either 'world', 'base', 'end effector', or 'tool'.
                Note: 'end effector' and 'tool' don't make any sense here,
                since the tool frame is constant relative to these frames...

        Returns:
            Klampt se3 element: specifies end effector Cartesian transform
            relative to the given frame.
        """
        raise NotImplementedError()

    def cartesianVelocity(self,
            q: Vector,
            dq: Vector,
            frame: str = 'world'
        ) -> RigidTransform:
        """Converts from a joint position / velocity vector to a cartesian
        velocity.

        Args: 
            q (vector): the (whole) robot's joint positions.
            dq (vector): the (whole) robot's joint velocities.
            frame (str): either 'world', 'base', 'end effector', or 'tool'.
                Note: if 'base' is specified, the velocity of the base is 
                subtracted from the reported speed.

        Returns:
            (w,v): specifies end effector Cartesian angular velocity/velocity
            relative to the given frame.
        """
        raise NotImplementedError()

    def cartesianForce(self,
            q: Vector,
            t: Vector,
            frame: str = 'world'
        ) -> RigidTransform:
        """Converts from a joint position / torque vector to a cartesian force.

        Args: 
            q (vector): the (whole) robot's joint positions.
            t (vector): the (whole) robot's joint torques.
            frame (str): either 'world', 'base', 'end effector', or 'tool'.

        Returns:
            (t,f): specifies end effector Cartesian torque/force relative to
            given frame.
        """
        raise NotImplementedError()

    def sensedCartesianPosition(self, frame: str = 'world') -> RigidTransform:
        return self.cartesianPosition(self.sensedPosition(),frame)

    def sensedCartesianVelocity(self, frame: str = 'world') -> RigidTransform:
        return self.cartesianVelocity(self.sensedPosition(),self.sensedVelocity(),frame)

    def sensedCartesianForce(self, frame: str = 'world') -> RigidTransform:
        return self.cartesianForce(self.sensedPosition(),self.sensedTorque(),frame)

    def commandedCartesianPosition(self, frame: str = 'world') -> RigidTransform:
        return self.cartesianPosition(self.commandedPosition(),frame)

    def commandedCartesianVelocity(self, frame: str = 'world') -> RigidTransform:
        return self.cartesianVelocity(self.commandedPosition(),self.commandedVelocity(),frame)

    def commandedCartesianForce(self, frame: str = 'world') -> RigidTransform:
        return self.cartesianForce(self.commandedPosition(),self.commandedTorque(),frame)

    def destinationCartesianPosition(self, frame: str = 'world') -> RigidTransform:
        """Retrieves the Cartesian destination of a motion queue controller.

        Args:
            frame (str): either 'world', 'base', 'end effector', or 'tool'
        """
        return self.cartesianPosition(self.destinationPosition(),frame)

    def destinationCartesianVelocity(self, frame: str = 'world') -> RigidTransform:
        """Retrieves the final Cartesian velocity of a motion queue controller.

        Args:
            frame (str): either 'world', 'base', 'end effector', or 'tool'
        """
        return self.cartesianVelocity(self.destinationPosition(),self.desinationVelocity())

    def queuedCartesianTrajectory(self, frame: str = 'world') -> tuple:
        """Returns the Cartesian trajectory starting from the current time 
        representing all commands in a motion queue controller.

        Args:
            frame (str): either 'world', 'base', 'end effector', or 'tool'

        Returns:
            tuple: either (ts,Ts) or (ts,Ts,dTs) representing a piecewise
            linear or a piecewise-cubic trajectory.  Ts is a list of Klampt se3
            elements.  dTs is a list of (angular velocity,velocity) pairs.
        """
        res = self.queuedTrajectory()
        if len(res) == 2:
            ts,qs = res
            return ts,[self.cartesianPosition(q,frame) for q in qs]
        elif len(res) == 3:
            ts,qs,vs = res
            return ts,[self.cartesianPosition(q,frame) for q in qs],[self.cartesianVelocity(q,dq,frame) for q,dq in zip(qs,vs)]
        else:
            raise RuntimeError("Invalid result from queuedTrajectory")

    def setPosition(self, q: Vector) -> None:
        """Sets an instantaneous position command.

        Args:
            q (list of floats): A list of floats giving the desired
                configuration of the robot.
        """
        raise NotImplementedError()

    def setVelocity(self,
            v: Vector,
            ttl: Optional[float] = None
        ) -> None:
        """Sets an instantaneous velocity command. 

        Args:
            v (list of floats): A list of floats giving the desired 
                velocity of each joint.
            ttl (float, optional): A time-to-live for this command.
        """
        raise NotImplementedError()

    def setTorque(self,
            t: Vector,
            ttl: Optional[float] = None
        ) -> None:
        """Sets a instantaneous torque command.

        Args:
            t (list of floats): A list of floats giving the desired 
                torques at each joint.
            ttl (float, optional): A time-to-live for this command.
        """
        raise NotImplementedError()

    def setPID(self,
            q: Vector,
            dq: Vector,
            t: Optional[Vector] = None
        ) -> None:
        """Sets a PID command to configuration q, velocity dq, and feedforward
        torque t. 
        """
        raise NotImplementedError()

    def setPIDGains(self, kP: Vector, kI: Vector, kD: Vector) -> None:
        """Sets the PID gains.  Some controllers might not implement this even 
        if they implement setPID...
        """
        raise NotImplementedError()

    def getPIDGains(self) -> Tuple[List[float],List[float],List[float]]:
        """Gets the PID gains (kP,kI,kD) as set to a prior call to setPIDGains.
        Some controllers might not implement this even if they implement
        setPIDGains...
        """
        raise NotImplementedError()

    def moveToPosition(self,
            q: Vector,
            speed: float=1.0
        ) -> None:
        """Sets a move-to position command.  The trajectory that the robot will
        take on should be extractable through getMoveToTrajectory(q).

        Args:
            q (list of floats): A list of floats giving the desired 
                configuration of the robot.
            speed (float, optional): The speed at which the position
                should be reached.
        """
        raise NotImplementedError()

    def setPiecewiseLinear(self,
            ts: List[float],
            qs: List[List[float]],
            relative: bool=True
        ) -> None:
        """Tells the robot to start a piecewise linear trajectory
        command.  The first milestone will be interpolated from the
        current commanded configuration.

        Args:
            ts (list of floats): times of the trajectory's milestones
            qs (list of floats, or list of list of floats): list of the
                trajectory's milestones
            relative (bool): if true, the times in `ts` are assumed to start
                at current time 0.  Otherwise, they must all be greater than 
                the current time retrieved by clock().
        """
        raise NotImplementedError()

    def setPiecewiseCubic(self,
            ts: List[float],
            qs: List[List[float]],
            vs: List[List[float]]
        ) -> None:
        """Tells the robot to start a piecewise cubic trajectory
        command.   The first milestone will be interpolated from the
        current commanded configuration / velocity.

        Args:
            ts (list of floats): times of the trajectory's milestones
            qs (list of floats, or list of list of floats): list of the
                trajectory's milestones
            vs (list of floats, or list of list of floats): list of the
                trajectory's derivatives at the milestones.
            relative (bool): if true, the times in `ts` are assumed to start
                at current time 0.  Otherwise, they must all be greater than 
                the current time retrieved by clock().
        """
        raise NotImplementedError()

    def setToolCoordinates(self, xtool_local: Vector3) -> None:
        """Sets the tool coordinates of this robot relative to its end
        effector link."""
        raise NotImplementedError()

    def getToolCoordinates(self) -> Vector3:
        """Gets the tool coordinates of this robot relative to its end
        effector link."""
        raise NotImplementedError()
        
    def setGravityCompensation(self,
            gravity: Vector3=[0,0,-9.8],
            load: float=0.0,
            load_com: Vector3=[0,0,0]
        ) -> None:
        """Sets up gravity compensation with a given gravity vector and end
        effector load.

        Args:
            gravity (list of 3 floats, optional): the gravity vector in the
                base frame, in m/s^2.
            load (float, optional): a weight attached to the end effector, in
                kg.
            load_com (list of 3 floats, optional): the COM of the load,
                expressed relative to the end-effector link frame.
        """
        raise NotImplementedError()

    def getGravityCompensation(self) -> tuple:
        """Gets (gravity,load,load_com) as from a prior call to
        setGravityCompensation."""
        raise NotImplementedError()

    def setCartesianPosition(self,
            xparams: Union[Vector,RigidTransform],
            frame: str = 'world'
        ) -> None:
        """Sets a Cartesian position command.  The tool is commanded to reach
        the given coordinates relative to the given frame.  Like setPosition,
        this command is sent in immediate mode.

        Args:
            xparams: a klampt.math.se3 object for position / orientation
            commands, or a 3-vector for position-only.
            frame (str): either 'world', 'base', 'end effector', or 'tool'
        """
        raise NotImplementedError()

    def moveToCartesianPosition(self,
            xparams: Union[Vector3,RigidTransform],
            speed: float=1.0,
            frame: str = 'world'
        ) -> None:
        """Sets a Cartesian move-to-position command. The tool is commanded
        to reach the given coordinates relative to the given frame. The
        movement is accomplished via an arbitrary joint space motion, and is
        not necessarily linear in Cartesian space.

        Args:
            xparams: typically a klampt.math.se3 object for position /
                orientation commands, or a 3-vector for position-only.
            speed (float, optional): The speed at which the position
                should be reached.
            frame (str): either 'world', 'base', 'end effector', or 'tool'
        """
        raise NotImplementedError()
    
    def moveToCartesianPositionLinear(self,
            xparams: Union[Vector3,RigidTransform],
            speed: float=1.0,
            frame: str = 'world'
        ) -> None:
        """Sets a Cartesian move-to-position command. The tool is commanded
        to reach the given coordinates relative to the given frame via a
        linear Cartesian path.

        Args:
            xparams: typically a klampt.math.se3 object for position /
                orientation commands, or a 3-vector for position-only.
            speed (float, optional): The speed at which the position
                should be reached.
            frame (str): either 'world', 'base', 'end effector', or 'tool'
        """
        raise NotImplementedError()

    def setCartesianVelocity(self,
            dxparams: Union[Vector3,Tuple[Vector3,Vector3]],
            ttl: Optional[float] = None,
            frame: str = 'world'
        ) -> None:
        """Sets a Cartesian velocity command. The tool is commanded to move
        along the given velocity(s) relative to the given frame.  

        Args:
            dxparams: typically an (angular velocity, translational velocity)
                pair for translation / orientation commands.  A 3-vector for
                translation-only.
            ttl (float, optional): A time-to-live for this command.
            frame (str): either 'world', 'base', 'end effector', or 'tool'
        """
        raise NotImplementedError()

    def setCartesianForce(self,
            fparams: Union[Vector,RigidTransform],
            ttl: Optional[float] = None,
            frame: str = 'world'
        ) -> None:
        """Sets a Cartesian torque command. The tool is commanded to exert
        the given force or (torque, force) relative to the given frame.  

        Args:
            fparams: typically an (torque, force) pair for translation
                / orientation commands.  A 3-vector for translation-only.
            ttl (float, optional): A time-to-live for this command.
            frame (str): either 'world', 'base', 'end effector', or 'tool'
        """
        raise NotImplementedError()

    def partToRobotConfig(self,
            pconfig: Vector,
            part: str,
            robotConfig: Vector,
            joint_idx: Optional[int] = None
        ) -> Vector:
        """Fills a configuration vector for the whole robot given the
        configuration `pconfig` for a part"""
        pindices = self.indices(part,joint_idx)
        q = [v for v in robotConfig]
        assert len(pindices) == len(pconfig)
        for (i,x) in zip(pindices,pconfig):
            q[i] = x
        return q

    def robotToPartConfig(self,
            robotConfig: Vector,
            part: str,
            joint_idx: Optional[int] = None
        ) -> Vector:
        """Retrieves a part's configuration from a robot configuration"""
        pindices = self.indices(part,joint_idx)
        return [robotConfig[i] for i in pindices]

    def klamptModel(self) -> Optional[Union[RobotModel,SubRobotModel]]:
        """If applicable, returns the Klamp't RobotModel associated with this
        controller.  Default tries to load from properties['klamptModelFile'].

        Note that the configuration DOFs in the RIL robot correspond to the
        robot's DRIVERs, not its links.

        Note: the result of the default implementation is cached, so this can
        be called multiple times without a performance hit.
        """
        if self._klamptModel is not None:
            return self._klamptModel
        if 'klamptModelFile' in self.properties:
            if hasattr(self,'_worldModel'):
                if self._worldModel is not None:
                    raise RuntimeError("Hmm... caching doesn't seem to be working right?")
            self._worldModel = WorldModel()
            if not self._worldModel.loadFile(self.properties['klamptModelFile']):
                self._worldModel = None
            else:
                self._klamptModel = self._worldModel.robot(0)
                return self._klamptModel
        return None

    def configFromKlampt(self,
            klamptConfig: Vector,
            part: Optional[str] = None,
            joint_idx: Optional[int] = None
        ) -> Vector:
        """Extracts a RobotInterfaceBase configuration from a configuration of
        the Klampt model. 

        Note: the configuration of the model in self.klamptModel() is overridden.
        """
        model = self.klamptModel()
        if model is None:
            return klamptConfig
        if len(klamptConfig) != model.numLinks():
            raise ValueError("Length of klamptConfig is invalid for "+str(self))
        qdrivers = model.configToDrivers(klamptConfig)
        if part is None and joint_idx is None:
            return qdrivers
        return [qdrivers[i] for i in self.indices(part,joint_idx)]

    def velocityFromKlampt(self,
            klamptVelocity: Vector,
            part: Optional[str] = None,
            joint_idx: Optional[int] = None
        ) -> Vector:
        """Extracts a RobotInterfaceBase velocity from a velocity of
        the Klampt model."""
        model = self.klamptModel()
        if model is None:
            return klamptVelocity
        if len(klamptVelocity) != model.numLinks():
            raise ValueError("Length of klamptVelocity is invalid for "+str(self))
        vdrivers = model.velocityToDrivers(klamptVelocity)
        if part is None and joint_idx is None:
            return vdrivers
        return [vdrivers[i] for i in self.indices(part,joint_idx)]

    def configToKlampt(self,
            config: Vector,
            klamptConfig: Optional[Vector] = None,
            part: Optional[str] = None,
            joint_idx: Optional[int] = None
        ) -> Vector:
        """Creates a configuration vector for the Klamp't model using the 
        RobotInterfaceBase configuration.

        If klamptConfig is given, then these values are used for the non-part
        configuration values.  Otherwise, the robot's current configuration from
        self.klamptModel() is used.

        .. note::
            The configuration of the model in self.klamptModel() is overridden.

        """
        model = self.klamptModel()
        if model is None:
            return config
        dofs = self.indices(part,joint_idx)
        if len(dofs) != len(config):
            raise ValueError("Length of config is invalid for "+str(self))
        if klamptConfig is not None:
            model.setConfig(klamptConfig)
        if part is None and joint_idx is None:
            return model.configFromDrivers(config)
        else:
            for (i,x) in zip(dofs,config):
                model.driver(i).setValue(x)
            return model.getConfig()

    def velocityToKlampt(self,
            velocity: Vector,
            klamptVelocity: Optional[Vector] = None,
            part: Optional[str] = None,
            joint_idx: Optional[int] = None
        ) -> Vector:
        """Creates a velocity vector for a Klamp't model using the joint velocity.

        If klamptVelocity is given, then these values are used for the non-part
        configuration values.  Otherwise, the robot's current velocity from
        self.klamptModel() is used.

        .. note::
            The velocity of the model in self.klamptModel() is overridden.
            
        """
        model = self.klamptModel()
        if model is None:
            return velocity
        dofs = self.indices(part,joint_idx)
        if len(dofs) != len(velocity):
            raise ValueError("Length of velocity is invalid for "+str(self))
        if klamptVelocity is not None:
            model.setVelocity(klamptVelocity)
        if part is None and joint_idx is None:
            return model.velocityFromDrivers(velocity)
        else:
            for (i,x) in zip(dofs,velocity):
                model.driver(i).setVelocity(x)
            return model.getVelocity()

    def wait(self,timeout=None,condition=None,pollRate=None) -> float:
        """Sleeps the caller until ``isMoving()`` returns False.  This should 
        only be called outside of a beginStep()/endStep() pair.
        
        Args:
            timeout (float, optional): if given, will break after this amount
                of time has elapsed (in s)
            condition (callable, optional): if given, overrides the test
                ``self.isMoving()``.  Instead, will stop when ``condition()``
                returns True.
            pollRate (float, optional): time between isMoving checks, in Hz.
                If None, polls at the controller's natural rate. 
        
        .. note::

            pollRate should only be non-None if the interface is asynchronous.
            Otherwise, steps may be skipped.

        Returns:
            float: the approximate time in s that was waited.
        """
        if condition is None: condition = lambda : not self.isMoving()
        self.beginStep()
        stop = condition()
        self.endStep()
        if stop: return 0

        dt = 1.0/pollRate if pollRate is not None else 1.0/self.controlRate()
        from .utils import TimedLooper
        ttotal = 0
        looper = TimedLooper(dt)
        while looper:
            self.beginStep()
            stop = condition()
            self.endStep()
            if stop: return ttotal
            if timeout is not None and ttotal >= timeout:
                return ttotal
            ttotal += dt

