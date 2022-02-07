from .robotcontroller import RobotControllerBlock,RobotControllerIO
from klampt.model import trajectory
from klampt.io import loader

class TrajectoryPositionController(RobotControllerBlock):
    """A (robot) controller that takes in a trajectory and outputs the position
    along the trajectory.  If type is a 2-tuple, this will also output the
    derivative of the trajectory"""
    def __init__(self,traj,type=('qcmd','dqcmd')):
        self.traj = traj
        self.outputType = type
        self.startTime = None
        RobotControllerBlock.__init__(self)
        for t in type:
            self._outputs.addChannel(t)

    def advance(self,**inputs):
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

    def __getstate__(self):
        return {'startTime':self.startTime,'traj':loader.toJson(self.traj)}
    def __setstate__(self,state):
        self.startTime = state['startTime']
        self.traj = loader.fromJson(state['traj'],'Trajectory')
    def signal(self,type,*inputs):
        if type=='reset':
            self.startTime = None


class TrajectoryWithFeedforwardTorqueController(RobotControllerBlock):
    """A controller that takes in a joint trajectory and a feedforward torque
    trajectory."""
    def __init__(self,traj,torquetraj):
        self.traj = traj
        self.torquetraj = torquetraj
        self.startTime = None
        RobotControllerBlock.__init__(self)

    def advance(self,**inputs):
        api = RobotControllerIO(inputs)
        t = api.time()
        if self.startTime == None:
            self.startTime = t
        t = t - self.startTime
        return api.makeFeedforwardPIDCommand(self.traj.eval(t),self.traj.deriv(t),self.torquetraj.eval(t))

    def __getstate__(self):
        return {'startTime':self.startTime,'traj':loader.toJson(self.traj),'torquetraj':loader.toJson(self.torqueTraj)}
    def __setstate__(self,state):
        self.startTime = state['startTime']
        self.traj = loader.fromJson(state['traj'],'Trajectory')
        self.torquetraj = loader.fromJson(state['torquetraj'],'Trajectory')
    def signal(self,type,**inputs):
        if type=='reset':
            self.startTime = None


def make(robot,file="mypath.path",ff_torque_file=None):
    if robot == None:
        l = trajectory.Trajectory()
    else:
        l = trajectory.RobotTrajectory(robot)
    l.load(file)

    if ff_torque_file is not None:
        tcmd = trajectory.Trajectory()
        tcmd.load(ff_torque_file)
        return TrajectoryWithFeedforwardTorqueController(l,ff_torque_file)

    return TrajectoryPositionController(l)
