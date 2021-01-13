from ..controller import ControllerBlock
from ..cartesian_drive import CartesianDriveSolver


class CartesianDriveController(ControllerBlock):
    """Adapts a CartesianDriveSolver to a ControllerBlock.  The robot's
    commanded position is updated by the solver.

    It is assumed that the solver is initialized with all the settings but
    start is not necessarily called yet. If start=True, then this will start 
    working by default. Otherwise, it will wait for the 'enter' or 'start'
    signal to begin using the drive commands.
    """
    def __init__(self,solver,links,baseLinks=None,endEffectorPositions=None,start=True):
        assert isinstance(solver,CartesianDriveSolver)
        self.solver = solver
        self.links = links
        self.baseLinks = baseLinks
        self.start = start
        self._qcmd = None

    def inputNames(self):
        return ['wdes','vdes','dt']

    def outputNames(self):
        return ['progress','qcmd']

    def signal(self,type,**inputs):
        if type == 'enter' or type == 'start':
            self.start = True

    def advance(self,**inputs):
        if 'qcmd' in inputs:
            self._qcmd = inputs['qcmd']
        elif self._qcmd is None:
            self._qcmd = inputs['q']

        if self.start:
            self.solver.start(self._qcmd,self.links,self.baseLinks,self.endEffectorPositions)
            self.start=False
        
        progress,qcmd = self.solver.drive(self._qcmd,inputs['wdes'],inputs['vdes'],inputs['dt'])
        self._qcmd = qcmd
        return {'progress':progress,'qcmd':qcmd}

    def getState(self):
        import copy
        return copy.deepcopy({'driveTransforms':self.solver.driveTransforms,'driveSpeedAdjustment':self.solver.driveSpeedAdjustment})

    def setState(self,state):
        self.solver.driveTransforms = state['driveTransforms']
        self.solver.driveSpeedAdjustment = state['driveSpeedAdjustment']

