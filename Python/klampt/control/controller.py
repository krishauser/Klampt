"""Defines a set of generic "controllers" that are repeatedly-updating
processes, which typically output commands to a simulated or real robot. 
However, controllers can be much more, including estimators, sensor drivers, 
etc.

The :class:`ControllerBase` class defines a generic process that accepts some
inputs and produces some outputs every time that output_and_advance() is
called.  The inputs and outputs are extremely general, and are just string-
indexed dictionaries.  The usage of a ControllerBase is governed by convention.


Robot controllers
=================

If the controller is being used for a simulated robot or the Klampt Robot
Interface Layer, there's a standard convention.  It expects to receive the
following values as input:

- t = time: current time
- dt = timestep: controller time step
- sensor1 = sensor1_values: measurements for sensor 1
- ...
- sensork = sensork_values: measurements for sensor k

in which 'q', 'dq' are standard sensors, but other sensors may also be
provided.

The output dictionary is expected to contain one or more of the following keys:

- qcmd
- dqcmd 
- tcmd
- torquecmd

If qcmd is set, then it's a PID command. If dqcmd is set, then it's
the desired velocity, otherwise the desired velocity is 0.

If qcmd is set and torquecmd is set, then torquecmd is a feedforward
torque.

If dqcmd and tcmd are set, it's a fixed velocity command for duration tcmd.

Otherwise, torquecmd must be set, and it's a torque command.

For convenience, your ControllerBase subclass may use the
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

    from klampt.control.controller import ControllerBase
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
    controller = MyControllerObject()  #subclass of ControllerBase
    interface = SimPositionControlInterface(sim.controller(0),sim)
    binding = RobotControllerToInterface(controller,interface)
    while vis.shown():
        binding.advance()
        sim.updateWorld()
        vis.update()

Example that outputs to a real robot::

    from klampt.control.controller import ControllerBase
    from klampt.control.interop import RobotControllerToInterface

    #create MyControllerObject, MyRobotInterface class here

    controller = MyControllerObject()  #subclass of ControllerBase
    interface = MyRobotInterface(...)
    binding = RobotControllerToInterface(controller,interface)
    while not done:
        binding.advance()

For tighter control over a simulation, such as sub-stepping, you should use the
:mod:`klampt.sim.simulate` module.  Robot controllers in ControllerBase form
are accepted by the :meth:`SimpleSimulator.setController` method.


Blocks submodule
================

The :mod:`klampt.control.blocks` module gives several examples of controllers
that can be composed.

"""



class ControllerBase(object):
    """A generic base class that outputs a named dictionary outputs of based
    on a dictionary of inputs.  This is typically used to define robot
    controllers and components of such controllers, such as state estimators.

    At a minimum, a stateless controller should implement the output()
    method. 

    A stateful controller should also implement the :meth:`advance` method, and
    ideally the :meth:`getState` and :meth:`setState` methods.  The
    :meth:`output` method should not modify state.
    """
    def __init__(self):
        pass

    def __str__(self):
        """Optional: return a descriptive string describing this controller"""
        return self.__class__.__name__

    def inputNames(self):
        """Optional: to help debug, this is the set of input arguments that are
        expected. Returns a list, tuple, or set."""
        return []

    def inputValid(self,**inputs):
        """Optional: to help debug, tests whether the input argument dictionary
        is valid.  Returns bool."""
        return all(i in inputs for i in self.inputNames())

    def outputNames(self):
        """Optional: to help debug, this is the set of output arguments that are
        expected. Returns a list, tuple, set, or None to indicate no specific
        output."""
        return None

    def output(self,**inputs):
        """Computes the output of this controller given a dictionary of
        inputs.  The output is typically a dictionary but could also be a
        list.
        """
        return None

    def signal(self,type,**inputs):
        """Sends some asynchronous signal to the controller. The inputs
        are application-defined, but typical signals include:

        - 'reset'
        - 'enter'
        - 'exit'
        """
        pass

    def advance(self,**inputs):
        pass

    def output_and_advance(self,**inputs):
        res = self.output(**inputs)
        self.advance(**inputs)
        return res

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


class RobotControllerIO:
    """A helper class that makes it a bit easier to implement a
    `ControllerBase` robot controller by providing an object-oriented 
    interface to the dictionary-based protocol.

    Usage::

        class MyController(ControllerBase):
            def output(**input):
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
        self.retval['dqcmd'] = value
        self.retval['tcmd'] = dt
    def setTorqueCommand(self,torque):
        self.retval['torquecmd'] = torque
    def setJointPositionCommand(self,index,value):
        """Sets a single indexed joint to a position command"""
        #klampt can only do uniform position, velocity, or torque commands
        if 'qcmd' in self.retval:
            self.retval['qcmd'][index] = value
        elif 'dqcmd' in self.retval:
            self.retval['dqcmd'][index] = (value - self.inputs['qcmd'][index]) / inputs['dt']
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



        
def make(robot):
    return ControllerBase()


def launch(fn,robot):
    """Launches a controller given by the given python module for the
    given robot using the module's default make(robot) routine."""
    import os
    import importlib
    path,base = os.path.split(fn)
    mod_name,file_ext = os.path.splitext(base)
    mod = importlib.import_module(path+"."+mod_name,fn)
    try:
        maker = mod.make
    except AttributeError:
        print("Module",mod.__name__,"must have a make() method")
        raise
    return maker(robot)
