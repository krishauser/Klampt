class ControllerManager(object):
    """A class that connects the output of a software ``ControllerBase`` API
    with a ``RobotControllerBase`` hardware API

    See ``klampt.control.robotcontroller``
    """
    def __init__(self,robotController,controller,controllerRateRatio=1):
        self.robotController = robotController
        self.controller = controller
        self.controllerRateRatio = controllerRateRatio
