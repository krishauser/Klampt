from controller import BaseController
from klampt import trajectory
import time

class TrajectoryWithFeedforwardController(BaseController):
    """A controller that takes in a joint trajectory and a feedforward torque
    trajectory."""
    def __init__(self,traj,torquetraj):
        self.traj = traj
        self.torquetraj = torquetraj
        self.startTime = None
        self.realStartTime = time.time()
    def output(self,**inputs):
        t = inputs['t']
        if self.startTime == None:
            self.startTime = t
        t = t - self.startTime
        return {'qcmd':self.traj.eval(t),'torquecmd':self.torquetraj.eval(t)}
    def signal(self,type,**inputs):
        if type=='reset':
            self.startTime = None

def make(robot,q_file="q_cmd.txt",ff_torque_file="ff_torque_cmd.txt"):
    qcmd = trajectory.Trajectory()
    tcmd = trajectory.Trajectory()
    qcmd.load(q_file)
    tcmd.load(ff_torque_file)    
    return TrajectoryWithFeedforwardController(qcmd,tcmd)
