from .controller import ControllerBlock
from .robotinterface import RobotInterfaceBase
from .robotinterfaceutils import RobotInterfaceCompleter

class SimRobotControllerToInterface(object):
    """A class that provides an API like :class:`SimRobotController` so that
    simulation control code can be ported easily to a :class:`RobotInterfaceBase`
    Robot Interface Layer API.

    Missing:

    - setRate(dt): can't change a real robot's control rate.
    - getSetting()/setSetting(): no interface to configure robot.
    - setMilestone(q,dq): can't set a terminal velocity.
    - add*(): can't queue up commands.
    - getPIDGains(): can't configure PID gains.

    Arguments:
        robotInterface (RobotInterfaceBase): the robot interface to use
    """
    def __init__(self,robotInterface):
        if not isinstance(robotInterface,RobotInterfaceCompleter):
            robotInterface = RobotInterfaceCompleter(robotInterface)
        self.robotInterface = robotInterface

    def initialize(self):
        self.robotInterface.initialize()
    def startStep(self):
        self.robotInterface.startStep()
    def endStep(self):
        self.robotInterface.endStep()
    def model(self):
        return self.robotInterface.klamptModel()
    def setRate(self,dt):
        raise NotImplementedError("setRate() may not be called")
    def getRate(self):
        return 1.0/self.robotInterface.controlRate()

    def getCommandedConfig(self):
        return self.robotInterface.configToKlampt(self.robotInterface.commandedPosition())
    def getCommandedVelocity(self):
        return self.robotInterface.velocityToKlampt(self.robotInterface.commandedVelocity())
    def getCommandedTorque(self):
        return self.robotInterface.commandedTorque()
    def getSensedConfig(self):
        return self.robotInterface.configToKlampt(self.robotInterface.sensedPosition())
    def getSensedVelocity(self):
        return self.robotInterface.velocityToKlampt(self.robotInterface.sensedVelocity())
    def getSensedTorque(self):
        return self.robotInterface.sensedTorque()

    def sensor(self,index_or_name):
        if isinstance(index_or_name,int):
            sensorNames = self.robotInterface.sensors()
            if index_or_name < 0 or index_or_name >= len(sensorNames):
                #"null" sensor
                return _SimRobotSensorFromInterface(self.robotInterface,None)
            name = sensorNames[index_or_name]
        else:
            name = index_or_name
            enabled = self.robotInterface.enabledSensors()
            if name not in enabled:
                if not self.robotInterface.hasSensor(name):
                    #"null" sensor
                    return _SimRobotSensorFromInterface(self.robotInterface,None)
                self.robotInterface.enableSensor(name)
        return _SimRobotSensorFromInterface(self.robotInterface,name)
    def commands(self):
        return ['estop','softStop','reset']
    def sendCommand(self,name,args):
        getattr(self,name)()
    def getSetting(self,name):
        raise ValueError("Can't change settings")
    def setSetting(self,name,val):
        raise ValueError("Can't change settings")

    def setMilestone(self,q,dq=None):
        if dq is not None:
            raise NotImplementedError("setMilestone(q[,dq]) may not be called with dq")
        self.robotInterface.moveToPosition(self.robotInterface.configFromKlampt(q))
    def addMilestone(self,q,dq=None):
        raise NotImplementedError("addMilestone() may not be called")
    def addMilestoneLinear(self,q):
        raise NotImplementedError("addMilestoneLinear() may not be called")
    def setLinear(self,q,dt):
        self.robotInterface.setPiecewiseLinear([dt],[robotInterface.configFromKlampt(q)])
    def setCubic(self,q,v,dt):
        self.robotInterface.setPiecewiseCubic([dt],[robotInterface.configFromKlampt(q)],[robotInterface.velocityFromKlampt(v)])
    def addLinear(self,q,dt):
        raise NotImplementedError("addLinear() may not be called")
    def addCubic(self,q,dt):
        raise NotImplementedError("addCubic() may not be called")

    def remainingTime(self):
        return self.robotInterface.destinationTime()

    def setVelocity(self,dq,dt):
        self.robotInterface.setVelocity(self.robotInterface.velocityFromKlampt(dq),dt)
    def setTorque(self,t):
        self.robotInterface.setTorque(t)
    def setPIDCommand(self,qdes,dqdes,tfeedforward=None):
        self.robotInterface.setPIDCommand(robotInterface.configFromKlampt(qdes),robotInterface.velocityFromKlampt(dqdes),tfeedforward)
    def setManualMode(self,enabled):
        pass

    def getControlType(self):
        return 'unknown'

    def setPIDGains(self,kP,kI,kD):
        self.robotInterface.setPIDGains(kP,kI,kD)
    def getPIDGains(self):
        return None,None,None


class _SimRobotSensorFromInterface(object):
    def __init__(self,iface,name):
        self.iface = iface
        self._name = name
    def name(self):
        if self._name is None: return ''
        return self._name
    def type(self):
        if self._name is None: return ''
        return 'UnknownSensor'
    def robot(self):
        return self.iface.klamptModel()
    def measurementNames(self):
        if self._name is None: return []
        return self.robot().sensor(name).measurementNames()
    def getMeasurements(self):
        if self._name is None: return []
        return self.iface.sensorMeasurements(self._name)
    def getSetting(self,name):
        return self.robot().sensor(self._name).getSetting(name)
    def setSetting(self,name,val):
        raise NotImplementedError("setSetting may not be called")
    def drawGL(self,measurements=None):
        if measurements is not None:
            return self.robot().sensor(self._name).drawGL(measurements)
        return self.robot().sensor(self._name).drawGL()


class RobotControllerToInterface(object):
    """A class that connects the I/O of a standard :class:`ControllerBlock` 
    robot controller with a :class:`RobotInterfaceBase` Robot Interface Layer
    API.

    Arguments:
        controller (ControllerBlock): the controller software, should conform
            to the RobotControllerBase convention.
        robotInterface (RobotInterfaceBase): the robot interface
        controllerRateRatio (float, optional): if not 1, scales how many times
            the controller is run for each main loop of the robotInterface
    """
    def __init__(self,controller,robotInterface,controllerRateRatio=1):
        if not isinstance(robotInterface,RobotInterfaceCompleter):
            robotInterface = RobotInterfaceCompleter(robotInterface)
        self.robotInterface = robotInterface
        self.controller = controller
        self.controllerRateRatio = controllerRateRatio
        self.lastInputs = None
        self.lastOutputs = None

    def advance(self):
        if self.controllerRateRatio != 1:
            raise NotImplementedError("Can't do controller / robotInterface time step ratio != 1")
        self.robotInterface.startStep()
        inputs = dict()
        inputs['t'] = self.robotInterface.clock()
        inputs['dt'] = 1.0/self.robotInterface.controlRate()
        inputs['q'] = self.robotInterface.sensedPosition()
        inputs['dq'] = self.robotInterface.sensedVelocity()
        try:
            inputs['torque'] = self.robotInterface.sensedTorque()
        except NotImplementedError:
            pass
        for s in self.robotInterface.enabledSensors():
            inputs[s] = self.robotInterface.sensorMeasurements(s)
        res = self.controller.advance(**inputs)
        if 'qcmd' in res:
            dqcmd = res['dqcmd'] if 'dqcmd' in res else [0.0]*len(res['qcmd'])
            if 'torquecmd' in res:
                self.robotInterface.setPID(res['qcmd'],dqcmd,res['torquecmd'])
            else:
                self.robotInterface.setPID(res['qcmd'],dqcmd)
        elif 'dqcmd' in res:
            assert 'tcmd' in res
            self.robotInterface.setVelocity(res['dqcmd'],res['tcmd'])
        elif 'torquecmd' in res:
            self.robotInterface.setTorque(res['torquecmd'])
        self.robotInterface.endStep()
        self.lastInputs = inputs
        self.lastOutputs = res
    

class InterfacetoVis(object):
    """A class that manages connections between the sensor readings of a
    :class:`RobotInterfaceBase` Robot Interface Layer API to the Klamp't vis
    module."""
    def __init__(self,robotInterface):
        if not isinstance(robotInterface,RobotInterfaceCompleter):
            robotInterface = RobotInterfaceCompleter(robotInterface)
        self.robotInterface = robotInterface

    def update(self):
        pass