"""Defines a set of generic "controller blocks" that are repeatedly-updating
processes, which typically output commands to a simulated or real robot. 
However, blocks can do much more, including estimators, read from sensor
drivers, etc.

The :class:`ControllerBlock` class defines a block as accepting some
inputs and produces some outputs every time that ``advance()`` is
called.  The inputs and outputs are extremely general, and are just string-
indexed dictionaries.  The usage of a ControllerBlock is governed by
convention.


Robot controller blocks
=======================

If the block is being used for :class:`SimpleSimulator` or the Klampt Robot
Interface Layer (see :class:`RobotInterfaceBase`, there's a standard convention
defined by :class:`RobotControllerBase`. It expects to receive the following
values as input:

- t: current time, in seconds
- dt: controller time step, in seconds
- q: sensed robot configuration
- dq: sensed robot velocity
- [sensor1_name]: measurement vector for sensor 1
- ...
- [sensor1_name]: measurement vector for sensor k

The output dictionary is expected to contain one or more of the following keys:

- qcmd
- dqcmd 
- tcmd
- torquecmd

These are interpreted as follows:

- If qcmd is set, then it's a PID command. If dqcmd is also set, then it 
  describes the desired velocity. If dqcmd is not set, the desired velocity is
  0.  If torquecmd is also set, then torquecmd is a feedforward torque.
- If dqcmd and tcmd are set, they describe a fixed velocity command for
  duration tcmd.
- If torquecmd is set, this describes a torque command.

No other combinations are currently supported.

For convenience, your RobotControllerBase subclass may use the
:class:`RobotControllerIO` class for object-oriented access to the input
/ output data. Example usage is as follows::

    api = RobotControllerIO(inputs)
    print("Current time is",api.time())
    #set a position command
    api.setJointPositionCommand(5,0.5)
    return api.makeCommand()


Binding robot controllers to a robot
====================================

The easiest way to use this with a simulated / real robot is the
:class:`.interop.ControllerToInterface` class. 

Example that outputs to a simulation (and visualizes it)::

    from klampt.control.controller import RobotControllerBase
    from klampt.control.simrobotinterface import *
    from klampt.control.interop import RobotControllerToInterface
    from klampt import WorldModel, Simulator
    from klampt import vis

    world = WorldModel()
    ...
    #create world, MyControllerObject class here
    ...
    sim = Simulator(world)

    vis.add("world",world)
    vis.show()
    controller = MyControllerObject()  #subclass of RobotControllerBase
    interface = SimPositionControlInterface(sim.controller(0),sim)
    binding = RobotControllerToInterface(controller,interface)
    while vis.shown():
        binding.advance()
        sim.updateWorld()
        vis.update()

Example that outputs to a real robot::

    from klampt.control.controller import RobotControllerBase
    from klampt.control.interop import RobotControllerToInterface

    #create MyControllerObject, MyRobotInterface class here

    controller = MyControllerObject()  #subclass of RobotControllerBase
    interface = MyRobotInterface(...)
    binding = RobotControllerToInterface(controller,interface)
    while not done:
        binding.advance()

For tighter control over a simulation, such as sub-stepping, you should use the
:mod:`klampt.sim.simulation` module.  Robot controllers in ControllerBlock form
are accepted by the :meth:`SimpleSimulator.setController` method.


Blocks submodule
================

The :mod:`klampt.control.blocks` module gives several examples of controller 
blocks that can be composed.  See :mod:`klampt.control.blocks.utils` for
utilities, and :mod:`klampt.control.blocks.state_machine` for state machines
that use ControllerBlocks.

"""



class ControllerBlock(object):
    """A generic base class that outputs a named dictionary outputs of based
    on a dictionary of inputs.  This is typically used to define robot
    controllers and components of such controllers, such as state estimators.

    At a minimum, a controller should implement the :meth:`advance` method. 

    A stateful controller should also implement the :meth:`getState` and
    :meth:`setState` methods. 
    """
    def __init__(self):
        pass

    def __str__(self):
        """Optional: return a descriptive string describing this controller"""
        return self.__class__.__name__

    def inputNames(self):
        """Optional: to help debug, this is the set of input arguments that are
        expected. Returns a list, tuple, or set."""
        raise NotImplementedError()

    def inputValid(self,**inputs):
        """Optional: to help debug, tests whether the input argument dictionary
        is valid.  Returns bool."""
        try:
            return all(i in inputs for i in self.inputNames())
        except NotImplementedError:
            return True

    def outputNames(self):
        """Optional: to help debug, this is the set of output arguments that are
        expected. Returns a list, tuple, set, or None to indicate no specific
        output."""
        return None

    def advance(self,**inputs):
        """Computes the output of this controller given a dictionary of
        inputs, and advances the time step.  The output is a dictionary.
        """
        return None

    def signal(self,type,**inputs):
        """Sends some asynchronous signal to the controller. The inputs
        are application-defined, but typical signals include:

        - 'reset': return state to the initial state
        - 'enter': for state machine: repeated advance() calls will now begin.
        - 'exit': for state machine: advance() calls will now stop.

        """
        pass

    def getState(self):
        """Optional: for stateful controllers, returns some serializable
        representation of state"""
        raise NotImplementedError()

    def setState(self,state):
        """Optional: for stateful controllers, restores the internal state
        from an output of getState()"""
        raise NotImplementedError()

    def drawGL(self):
        """Optional: hook to give feedback to the visualizer"""
        pass


class RobotControllerBase(ControllerBlock):
    """A base class for a robot controller.  This doesn't do anything but
    implement the inputNames method; the subclass still has to implement
    advance, signal, getState/saveState, etc.
    """
    def __init__(self,robotModel=None):
        self.robotModel = robotModel
    def inputNames(self):
        inputs = ['t','dt','q','dq']
        if self.robotModel is not None:
            i = 0
            while True:
                s = self.robotModel.sensor(i)
                i += 1
                if s.name()=='':
                    break
                if s.name() in ['q','dq']:
                    continue
                inputs.append(s.name())
        return inputs


class RobotControllerIO:
    """A helper class that makes it a bit easier to implement a
    `RobotControllerBase` by providing an object-oriented interface to the
    dictionary-based protocol.

    Usage::

        class MyController(ControllerBlock):
            def advance(**input):
                api = RobotControllerIO(inputs)
                print("Current time is",api.time())
                #set a position command
                api.setJointPositionCommand(5,0.5)
                return api.makeCommand()

    All methods, except for those prefixed by `make` or `set`, are to get 
    values from the input dictionary.

    The `makeXCommand` methods return a propertly formatted output
    dictionary.  Alternatively, you can make several `setXCommand` calls
    and then call `makeCommand()` to retrieve the output dictionary.
    """
    def __init__(self,inputs):
        self.inputs = inputs.copy()
        self.retval = dict()
    def time(self):
        """Returns the robot clock time"""
        return self.inputs['t']
    def timeStep(self):
        """Returns the robot time step"""
        return self.inputs['dt']
    def commandedConfiguration(self):
        """Returns the commanded joint configuration or None if it is not
        sensed."""
        try: return self.inputs['qcmd']
        except KeyError: return None
    def commandedVelocity(self):
        """Returns the commanded joint velocities or None if it is not
        sensed."""
        try: return self.inputs['dqcmd']
        except KeyError: return None
    def sensedConfiguration(self):
        """Returns the sensed joint configuration or None if it is not
        sensed."""
        try: return self.inputs['q']
        except KeyError: return None
    def sensedVelocity(self):
        """Returns the sensed joint velocity or None if it is not
        sensed."""
        try: return self.inputs['dq']
        except KeyError: return None
    def sensedTorque(self):
        """Returns the sensed torque or None if it is not
        sensed."""
        try: return self.inputs['torque']
        except KeyError: return None
    def sensorNames(self):
        """Returns the list of sensor names (including clock and time step)"""
        return self.inputs.keys()
    def sensorValue(self,sensor):
        """Returns the value of the named sensor."""
        try: return self.inputs[sensor]
        except KeyError: return None
    def makePositionCommand(self,q):
        return {'qcmd':q}
    def makePIDCommand(self,q,dq):
        return {'qcmd':q,'dqcmd':dq}
    def makeFeedforwardPIDCommand(self,q,dq,torque):
        return {'qcmd':q,'dqcmd':dq,'torquecmd':torque}
    def makeVelocityCommand(self,dq,t):
        return {'dqcmd':dq,'tcmd':t}
    def makeTorqueCommand(self,torque):
        return {'torquecmd':torque}
    def setPositionCommand(self,value):
        self.retval['qcmd'] = value
    def setPIDCommand(self,q,dq):
        self.retval['qcmd'] = q
        self.retval['dqcmd'] = dq
    def setFeedforwardPIDCommand(self,q,dq,torque):
        self.retval['qcmd'] = q
        self.retval['dqcmd'] = dq
        self.retval['torquecmd'] = torque
    def setVelocityCommand(self,v,dt):
        self.retval['dqcmd'] = v
        self.retval['tcmd'] = dt
    def setTorqueCommand(self,torque):
        self.retval['torquecmd'] = torque
    def setJointPositionCommand(self,index,value):
        """Sets a single indexed joint to a position command"""
        #klampt can only do uniform position, velocity, or torque commands
        if 'qcmd' in self.retval:
            self.retval['qcmd'][index] = value
        elif 'dqcmd' in self.retval:
            self.retval['dqcmd'][index] = (value - self.inputs['qcmd'][index]) / self.inputs['dt']
            self.retval['tcmd']=self.inputs['dt']
        elif 'torquecmd' in self.retval:
            print("Cannot combine joint position commands with joint torque commands")
        else:
            #no joint commands set yet, set a position command
            self.retval['qcmd'] = self.inputs['qcmd']
            self.retval['qcmd'][index] = value
    def setJointVelocityCommand(self,index,value):
        """Sets a single indexed joint to a velocity command"""
        #klampt can only do uniform position, velocity, or torque commands
        if 'qcmd' in self.retval:
            self.retval['qcmd'][index] = self.inputs['qcmd'][index]+value*self.inputs['dt']
            if 'dqcmd' not in self.retval:
                self.retval['dqcmd'] = self.inputs['dqcmd']
            self.retval['dqcmd'][index] = value
        elif 'dqcmd' in self.retval:
            self.retval['dqcmd'][index] = value
            self.retval['tcmd']=self.inputs['dt']
        elif 'torquecmd' in self.retval:
            print("Cannot combine joint velocity commands with joint torque commands")
        else:
            #no velocity commands set yet, set a velocity command
            self.retval['dqcmd'] = self.inputs['dqcmd']
            self.retval['dqcmd'][index] = value
    def makeCommand(self):
        """Returns the command from previous setXCommand() calls."""
        return self.retval



        
