from controller import ControllerBase
from klampt.model import trajectory


class TrajectoryController(ControllerBase):
    """A controller that takes in a trajectory and outputs the position along
    the trajectory.  If type is a 2-tuple, this will also output the
    derivative of the trajectory"""
    def __init__(self,traj,type=('qcmd','dqcmd')):
        self.traj = traj
        self.outputType = type

        self.startTime = None

    def inputNames(self):
        return ['t']

    def outputNames(self):
        if isinstance(self.outputType,(tuple,list)):
            return self.outputType
        else:
            return [self.outputType]
    
    def output(self,**inputs):
        t = inputs['t']
        if self.startTime == None:
            self.startTime = t
        t = t - self.startTime
        if isinstance(self.outputType,(tuple,list)):
            assert len(self.outputType)==2
            return {self.outputType[0]:self.traj.eval(t),
                    self.outputType[1]:self.traj.deriv(t)}
        else:
            return {self.outputType:self.traj.eval(t)}

    def getState(self):
        return {'startTime':self.startTime}
    def setState(self,state):
        self.startTime = state['startTime']


def make(robot,file="mypath.path"):
    if robot == None:
        l = trajectory.Trajectory()
    else:
        l = trajectory.RobotTrajectory(robot)
    l.load(file)
    return TrajectoryController(l)
