from .controller import ControllerBase
from .robotinterface import RobotInterfaceBase
from .robotinterfaceutils import RobotInterfaceCompleter

class RobotControllerToInterface(object):
    """A class that connects the output of a standard :class:`ControllerBase` 
    robot controller with a :class:`RobotInterfaceBase` Robot Interface Layer
    API.

    Arguments:
        controller (ControllerBase): the controller software
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
        res = self.controller.output_and_advance(**inputs)
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