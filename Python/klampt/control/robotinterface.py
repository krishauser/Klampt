"""The main module for Klampt's Robot Interface Layer.
"""

from ..robotsim import WorldModel
import functools


class RobotInterfaceBase(object):
    """The main class for the Klamp't Robot Interface Layer. Defines a unifying
    API to interface with a robot's motor controller, whether it's simulated or
    a real robot. 

    .. note::
        The API may look intimidating, but a subclass implementer is free to
        set up as many or as few of the given methods as the robot's motor
        controller truly implements.  The :class:`RobotInterfaceCompleter` 
        class will fill in the remainder of derived methods.  See the
        Functionalities section for more details.
    
    Each of these methods should be synchronous calls, called at a single time
    step.  The calling convention is::

        interface = MyRobotInterface(...args...)
        if not interface.initialize():  #should be called first
            raise RuntimeError("There was some problem initializing interface "+str(interface))
        dt = 1.0/interface.controlRate()
        while interface.status() == 'ok':  #no error handling done here...
            t0 = time.time()
            interface.startStep()
            [any getXXX or setXXX commands here comprising the control loop]
            interface.endStep()
            t1 = time.time()
            telapsed = t1 - t0
            [wait for time max(dt - telapsed,0)]
        interface.close()   #cleanly shut down the interface

    To accept asynchronous commands, a :class:`RobotInterfaceBase` subclass
    can be passed to :class:`AsynchronousRobotInterface` or
    :class:`RobotInterfaceServer`.


    **DOFs and Parts**

    The number of DOFs is assumed equal to the number of joint actuators / 
    encoders. If the robot has fewer actuators than encoders, the commands for 
    unactuated joints should just be ignored. If the robot corresponds to a 
    Klampt model (typical), then the number of DOFs should be
    ``model.numDrivers()``

    A robot can have "parts", which are named groups of DOFs.  For example, a
    robot with a gripper can have parts "arm" and "gripper", which can be 
    controlled separately.  You may retrieve part names using :meth:`parts`, 
    part indices using :meth:`indices`, and access a RIL interface to a part
    using :meth:`partController`. 

    It is suggested that these parts correspond with parts in the robot's 
    :class:`~klampt.model.robotinfo.RobotInfo` structure.


    **Functionalities**

    There are a few functions your subclass will need to fill out:

    * :meth:`numJoints` or :meth:`klamptModel`
    * Either :meth:`clock` or :meth:`controlRate`
    * Either :meth:`setPosition`, :meth:`moveToPosition`, :meth:`setVelocity`, 
      :meth:`setTorque`, or :meth:`setPID`
    * Either :meth:`sensedPosition` or :meth:`commandedPosition`

    Pass your RobotInterfaceBase subclass to :class:`RobotInterfaceCompleter`
    to complete the implementation of as many of the remaining items as
    possible.

    See the :class:`SimPositionControlInterface` class for an example that
    passes commands to a Klamp't physics simulator.


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
    and raise NotImplementedError() for other frames.


    Attributes:
        properties (dict): a dict from string key to property value. Application
            dependent. Examples may include:

            * 'name': str
            * 'version': str
            * 'simulated': bool
            * 'klamptModelFile': str

    """
    def __init__(self,**properties):
        self.properties = properties
        self._worldModel = None
        self._klamptModel = None
        self._klamptDriverIndices = None

    def __str__(self):
        inner = []
        if 'name' in self.properties:
            inner.append(self.properties['name'])
        if 'version' in self.properties:
            inner.append(self.properties['version'])
        return "RobotInterface("+','.join(inner)+')'

    def initialize(self):
        """Tries to connect to the robot.  Returns true if ready to send
        commands.  This should probably be the first method called.
        """
        return True

    def close(self):
        """Cleanly shuts down any resources acquired using initialize()."""
        return True

    def startStep(self):
        """This is called before the commands sent at each time step"""
        pass

    def endStep(self):
        """This is called after the commands sent at each time step"""
        pass

    def numJoints(self,part=None):
        """Returns the number of joints of the given part.  By default, this
        returns the number of actuated DOFs in the Klamp't model. 
        """
        if part is None:
            m = self.klamptModel()
            if m is None:
                raise NotImplementedError()
            return m.numDrivers()
        return len(self.parts()[part])

    @functools.lru_cache(maxsize=None)
    def parts(self):
        """Returns a dictionary of (part-name,configuration index list) pairs
        defining the named parts of the robot.

        Since this will be used a lot, make sure to declare your override with
        @functools.lru_cache.
        """
        return {None:list(range(self.numJoints()))}

    def indices(self,part=None,joint_idx=None):
        """Helper: returns a list of indices for the given part / joint index"""
        plist = self.parts()[part]
        if joint_idx is None:
            return plist
        assert joint_idx >= 0 and joint_idx < len(plist),"Invalid joint index for part "+str(part)
        return [plist[joint_idx]]

    def partInterface(self,part=None,joint_idx=None):
        """Returns a RobotInterfaceBase that allows control of the given 
        part/joint.  If no such controller exists, raises a
        NotImplementedError.

        The part/joint controller should operate on exactly the DOFs specified
        by self.indices(part,joint_idx).
        """
        if part is None and joint_idx is None:
            return self
        raise NotImplementedError()

    def controlRate(self):
        """Returns the control rate, in Hz"""
        raise NotImplementedError()

    def clock(self):
        """Returns the current time on the robot's clock, in seconds"""
        raise NotImplementedError()

    def estop(self):
        """Calls an emergency stop on the robot"""
        self.softStop()

    def softStop(self):
        """Calls a software E-stop on the robot (braking as quickly as
        possible).  Default implementation stops robot at current position; a 
        better solution would slow the robot down.
        """
        self.setPosition(self.commandedPosition())

    def reset(self):
        """If the robot has a non-normal status code, attempt to reset it
        to normal operation.  Returns true on success, false on failure.
        """
        return False

    def jointName(self,joint_idx):
        """Returns a string naming the given joint"""
        raise NotImplementedError()

    def sensors(self):
        """Returns a list of names of possible sensors on this robot."""
        raise NotImplementedError()

    def enabledSensors(self):
        """Returns a list of names of enabled sensors on this robot."""
        raise NotImplementedError()

    def hasSensor(self,sensor):
        """Returns true if the given sensor can be enabled.
        """
        return sensor in self.sensors()

    def enableSensor(self,sensor,enabled=True):
        """Enables / disables a sensor. Returns true if successful.
        """
        raise NotImplementedError()

    def sensorMeasurements(self,name):
        """Returns the latest measurements from a sensor.  Interpretation of
        the result is sensor-dependent.
        """
        raise NotImplementedError()

    def sensorUpdateTime(self,name):
        """Returns the clock time of the last sensor update."""
        raise NotImplementedError()

    def status(self,part=None,joint_idx=None):
        """Returns a status string for the given part / joint.  'ok' means
        everything is OK."""
        if part is not None or joint_idx is not None:
            return self.getPartController(part,joint_idx).status()
        return 'ok'

    def isMoving(self,part=None,joint_idx=None):
        """Returns true if the part / joint are currently moving"""
        if part is not None or joint_idx is not None:
            return self.getPartController(part,joint_idx).isMoving()
        raise NotImplementedError()

    def sensedPosition(self):
        """Retrieves the currently sensed joint position. 
        """
        raise NotImplementedError()

    def sensedVelocity(self):
        """Retrieves the currently sensed joint velocity. 
        """
        raise NotImplementedError()

    def sensedTorque(self):
        """Retrieves the currently sensed joint torque. 
        """
        raise NotImplementedError()

    def commandedPosition(self):
        """Retrieves the currently commanded joint position. 
        """
        raise NotImplementedError()

    def commandedVelocity(self):
        """Retrieves the currently commanded joint velocity. 
        """
        raise NotImplementedError()

    def commandedTorque(self):
        """Retrieves the currently commanded joint torque. 
        """
        raise NotImplementedError()

    def destinationPosition(self):
        """Retrieves the destination of a motion queue controller.
        """
        raise NotImplementedError()

    def destinationVelocity(self):
        """Retrieves the final velocity of a motion queue controller.
        """
        raise NotImplementedError()
    
    def destinationTime(self):
        """Retrieves the final clock time of a motion queue controller.
        """
        raise NotImplementedError()

    def queuedTrajectory(self):
        """Returns a trajectory starting from the current time representing all
        commands in a motion queue controller.

        Returns:
            tuple: either (ts,qs) or (ts,qs,vs) representing a piecewise linear
            or a piecewise-cubic trajectory.
        """
        raise NotImplementedError()

    def cartesianPosition(self,q,frame='world'):
        """Converts from a joint position vector to a cartesian position.

        Args:
            q (vector): the robot's joint positions.
            frame (str): either 'world', 'base', 'end effector', or 'tool'.
                Note: 'end effector' and 'tool' don't make any sense here,
                since the tool frame is constant relative to these frames...

        Returns:
            Klampt se3 element: specifies end effector Cartesian transform
            relative to the given frame.
        """
        raise NotImplementedError()

    def cartesianVelocity(self,q,dq,frame='world'):
        """Converts from a joint position / velocity vector to a cartesian
        velocity.

        Args: 
            q (vector): the robot's joint positions.
            dq (vector): the robot's joint velocities.
            frame (str): either 'world', 'base', 'end effector', or 'tool'.
                Note: if 'base' is specified, the velocity of the base is 
                subtracted from the reported speed.

        Returns:
            (w,v): specifies end effector Cartesian angular velocity/velocity
            relative to the given frame.
        """
        raise NotImplementedError()

    def cartesianForce(self,q,t,frame='world'):
        """Converts from a joint position / torque vector to a cartesian force.

        Args: 
            q (vector): the robot's joint positions.
            t (vector): the robot's joint torques.
            frame (str): either 'world', 'base', 'end effector', or 'tool'.

        Returns:
            (t,f): specifies end effector Cartesian torque/force relative to
            given frame.
        """
        raise NotImplementedError()

    def sensedCartesianPosition(self,frame='world'):
        return self.cartesianPosition(self.sensedPosition(),frame)

    def sensedCartesianVelocity(self,frame='world'):
        return self.cartesianVelocity(self.sensedPosition(),self.sensedVelocity(),frame)

    def sensedCartesianForce(self,frame='world'):
        return self.cartesianForce(self.sensedPosition(),self.sensedTorque(),frame)

    def commandedCartesianPosition(self,frame='world'):
        return self.cartesianPosition(self.commandedPosition(),frame)

    def commandedCartesianVelocity(self,frame='world'):
        return self.cartesianVelocity(self.commandedPosition(),self.commandedVelocity(),frame)

    def commandedCartesianForce(self,frame='world'):
        return self.cartesianForce(self.commandedPosition(),self.commandedTorque(),frame='world')

    def destinationCartesianPosition(self,frame='world'):
        """Retrieves the Cartesian destination of a motion queue controller.

        Args:
            frame (str): either 'world', 'base', 'end effector', or 'tool'
        """
        return self.cartesianPosition(self.destinationPosition(),frame)

    def destinationCartesianVelocity(self,frame='world'):
        """Retrieves the final Cartesian velocity of a motion queue controller.

        Args:
            frame (str): either 'world', 'base', 'end effector', or 'tool'
        """
        return self.cartesianVelocity(self.destinationPosition(),self.desinationVelocity())

    def queuedCartesianTrajectory(self,frame='world'):
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

    def setPosition(self,q):
        """Sets an instantaneous position command.

        Args:
            q (list of floats): A list of floats giving the desired
                configuration of the robot.
        """
        raise NotImplementedError()

    def setVelocity(self,v,ttl=None):
        """Sets an instantaneous velocity command. 

        Args:
            v (list of floats): A list of floats giving the desired 
                velocity of each joint.
            ttl (float, optional): A time-to-live for this command.
        """
        raise NotImplementedError()

    def setTorque(self,t,ttl=None):
        """Sets a instantaneous torque command.

        Args:
            t (list of floats): A list of floats giving the desired 
                torques at each joint.
            ttl (float, optional): A time-to-live for this command.
        """
        raise NotImplementedError()

    def setPID(self,q,dq,t=None):
        """Sets a PID command to configuration q, velocity dq, and feedforward
        torque t. 
        """
        raise NotImplementedError()

    def setPIDGains(self,kP,kI,kD):
        """Sets the PID gains.  Some controllers might not implement this even 
        if they implement setPID...
        """
        raise NotImplementedError()

    def moveToPosition(self,q,speed=1.0):
        """Sets a move-to position command.  The trajectory that the robot will
        take on should be extractable through getMoveToTrajectory(q).

        Args:
            q (list of floats): A list of floats giving the desired 
                configuration of the robot.
            speed (float, optional): The speed at which the position
                should be reached.
        """
        raise NotImplementedError()

    def setPiecewiseLinear(self,ts,qs,relative=True):
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

    def setPiecewiseCubic(self,ts,qs,vs):
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

    def setToolCoordinates(self,xtool_local):
        """Sets the tool coordinates of this robot relative to its end
        effector link."""
        raise NotImplementedError()

    def getToolCoordinates(self):
        """Gets the tool coordinates of this robot relative to its end
        effector link."""
        raise NotImplementedError()
        
    def setGravityCompensation(self,gravity=[0,0,-9.8],load=0.0,load_com=[0,0,0]):
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

    def setCartesianPosition(self,xparams,frame='world'):
        """Sets a Cartesian position command.  The tool is commanded to reach
        the given coordinates relative to the given frame.  Like setPosition,
        this command is sent in immediate mode.

        Args:
            xparams: a klampt.math.se3 object for position / orientation
            commands, or a 3-vector for position-only.
            frame (str): either 'world', 'base', 'end effector', or 'tool'
        """
        raise NotImplementedError()

    def moveToCartesianPosition(self,xparams,speed=1.0,frame='world'):
        """Sets a Cartesian move-to-position command. The tool is commanded to reach
        the given coordinates relative to the given frame. 

        Args:
            xparams: typically a klampt.math.se3 object for position /
                orientation commands, or a 3-vector for position-only.
            speed (float, optional): The speed at which the position
                should be reached.
            frame (str): either 'world', 'base', 'end effector', or 'tool'
        """
        raise NotImplementedError()

    def setCartesianVelocity(self,dxparams,ttl=None,frame='world'):
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

    def setCartesianForce(self,fparams,ttl=None,frame='world'):
        """Sets a Cartesian torque command. The tool is commanded to exert
        the given force or (torque, force) relative to the given frame.  

        Args:
            fparams: typically an (torque, force) pair for translation
                / orientation commands.  A 3-vector for translation-only.
            ttl (float, optional): A time-to-live for this command.
            frame (str): either 'world', 'base', 'end effector', or 'tool'
        """
        raise NotImplementedError()


    def partToRobotConfig(self,pconfig,part,robotConfig,joint_idx=None):
        """Fills a configuration vector for the whole robot given the
        configuration `pconfig` for a part"""
        pindices = self.indices(part,joint_idx)
        q = [v for v in robotConfig]
        assert len(pindices) == len(pconfig)
        for (i,x) in zip(pindices,pconfig):
            q[i] = x
        return q

    def robotToPartConfig(self,robotConfig,part,joint_idx=None):
        """Retrieves a part's configuration from a robot configuration"""
        pindices = self.indices(part,joint_idx)
        return [robotConfig[i] for i in pindices]

    def klamptModel(self):
        """If applicable, returns the Klamp't RobotModel associated with this
        controller.  Default tries to load from properties['klamptModelFile'].

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

    def configFromKlampt(self,klamptConfig,part=None,joint_idx=None):
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

    def velocityFromKlampt(self,klamptVelocity,part=None,joint_idx=None):
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

    def configToKlampt(self,config,klamptConfig=None,part=None,joint_idx=None):
        """Creates a configuration vector for the Klamp't model using the 
        RobotInterfaceBase configuration.

        If klamptConfig is given, then these values are used for the non-part
        configuration values.  Otherwise, the robot's current configuration from
        self.klamptModel() is used.

        Note: the configuration of the model in self.klamptModel() is overridden.
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

    def velocityToKlampt(self,velocity,klamptVelocity=None,part=None,joint_idx=None):
        """Creates a velocity vector for a Klamp't model using the joint velocity.

        If klamptVelocity is given, then these values are used for the non-part
        configuration values.  Otherwise, the robot's current velocity from
        self.klamptModel() is used.

        Note: the velocity of the model in self.klamptModel() is overridden.
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

    