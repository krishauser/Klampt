"""Defines RobotControllerBlock, which can be used for :class:`SimpleSimulator`
or the Klampt Robot Interface Layer (see :class:`RobotInterfaceBase`). Defines
a standard convention, expecting to receive the following
values as input:

- t: current time, in seconds
- dt: controller time step, in seconds
- q: sensed robot configuration
- dq: sensed robot velocity
- [sensor1_name]: measurement vector for sensor 1
- ...
- [sensor1_name]: measurement vector for sensor k

The output is expected to contain one or more of the following keys:

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

For convenience, your RobotControllerBlock subclass may use the
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
:class:`klampt.control.interop.RobotControllerBlockToInterface` class. 

Example that outputs to a simulation (and visualizes it)::

    from klampt.control.block.robotcontroller import RobotControllerBase
    from klampt.control.simrobotinterface import *
    from klampt.control.interop import RobotControllerBlockToInterface
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
    binding = RobotControllerBlockToInterface(controller,interface)
    while vis.shown():
        binding.advance()
        sim.updateWorld()
        vis.update()

Example that outputs to a real robot::

    from klampt.control.block.controller import RobotControllerBlock
    from klampt.control.interop import RobotControllerBlockToInterface

    #create MyControllerObject, MyRobotInterface class here

    controller = MyControllerObject()  #subclass of RobotControllerBase
    interface = MyRobotInterface(...)
    binding = RobotControllerBlockToInterface(controller,interface)
    while not done:
        binding.advance()

For tighter control over a simulation, such as sub-stepping, you should use the
:mod:`klampt.sim.simulation` module.  Robot controllers in RobotControllerBlock
form are accepted by the :meth:`SimpleSimulator.setController` method.
"""

from .core import Block

class RobotControllerBlock(Block):
    """A block implementation for a robot controller.  This doesn't
    do anything but set up the block.  The subclass still has to implement
    advance, signal, __getstate__/__savestate__, etc. 
    
    Recommend using the :class:`RobotControllerIO` class to format the results.
    """
    def __init__(self,robotModel=None):
        self.robotModel = robotModel
        inputs = ['t','dt','q','dq','torque','qcmd','dqcmd']
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
        Block.__init__(self,inputs,['qcmd','dqcmd','tcmd','torquecmd'])

class RobotControllerIO:
    """A helper class that makes it a bit easier to implement a
    `RobotControllerBase` by providing an object-oriented interface to the
    dictionary-based protocol.

    Usage::

        class MyController(ControllerBlock):
            def advance(self,**args):
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
    def __init__(self,kwargs):
        self.inputs = kwargs        
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


