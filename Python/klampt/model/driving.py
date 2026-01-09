"""Utilities for working with trajectories of mobile robots.
"""
from klampt.model.trajectory import HermiteTrajectory, SE2Trajectory
from klampt.math import vectorops,so2
import math
from klampt.model.typing import Vector,Vector3
from typing import List,Optional

class DifferentiallyFlatTrajectory(HermiteTrajectory):
    """A smoothed 2D trajectory for a differentially flat system
    such as a differential drive robot.
    """
    def __init__(self, times : List[float], poses: List[Vector], velocities : Optional[List[Vector]] = None):
        """Fits a 2D HermiteTrajectory to an SE2 trajectory.
        
        interp_se2_flat on the result should yield a similar set of poses to
        the input.

        Args:
            times (list of float): the times of each pose
            poses (list of 2-vectors or 3-vectors): if velocities not given, the list of SE2 poses to interpolate. 
            velocities (list of 2-vectors, optional): if given, poses are 2D milestones and these are the outgoing derivatives.
        """
        if velocities is not None:
            for p in poses:
                if len(p) != 2:
                    raise ValueError("Poses must be 2D when velocities are given")
            HermiteTrajectory.__init__(self, times, [p[:2] for p in poses], velocities)
            return
        assert len(times) > 0
        assert len(times) == len(poses)
        milestones = [list(p[:2]) for p in poses]
        velocities = []
        for i,p in enumerate(poses):
            if i == 0:
                dt = times[i+1]-times[0]
                start = p
                end = poses[i+1]
            elif i+1 >= len(times):
                dt = times[i]-times[i-1]
                start = poses[i-1]
                end = p
            else:
                dt = 2*math.sqrt((times[i]-times[i-1])*(times[i+1]-times[i]))
                start = poses[i-1]
                end = poses[i+1]
            d = math.sqrt((end[0]-start[0])**2 + (end[1]-start[1])**2)
            d *= 1.0/dt
            vx = math.cos(p[2])*d
            vy = math.sin(p[2])*d
            velocities.append([vx,vy])
        HermiteTrajectory.__init__(self, times,milestones,velocities)

    def eval_se2(self, t: float) -> Vector3:
        """Evaluates the trajectory at time t, returning an SE2 pose.
        """
        p = self.eval(t)
        v = self.deriv(t)
        return [p[0],p[1],math.atan2(v[1],v[0])]


def discretize_differential_drive_poses_se2(times : List[float], poses: List[Vector3], dt: float) -> SE2Trajectory:
    """Given SE2 poses to interpolate for a differential drive robot,
    produces a discretized, drivable SE2Trajectory.
    """
    for p in poses:
        assert len(p)==3,"Invalid SE2 element"
    segment_types = []
    for i in range(len(poses)-1):
        if vectorops.distance(poses[i][:2],poses[i+1][:2]) < 1e-2:  #1cm difference in positions, just rotate in place
            segment_types.append("linear")
        else:
            fwd = [math.cos(poses[i][2]),math.sin(poses[i][2])]
            diff = vectorops.sub(poses[i+1][:2],poses[i][:2])
            if vectorops.dot(fwd,diff) < 0:
                #do backwards mode
                segment_types.append('backwards')
            else:
                segment_types.append('forwards')
    segment_types.append(segment_types[-1])
    trajectories = []
    segment = [0]
    for i in range(0,len(poses)):
        if i+1==len(poses) or segment_types[i+1] != segment_types[i]:
            if i+1<len(poses):
                segment.append(i+1)
            if segment_types[i] == 'linear':
                trajectories.append(SE2Trajectory([times[j] for j in segment],[poses[j] for j in segment]).discretize(dt))
            elif segment_types[i] == 'forwards':
                hermite = DifferentiallyFlatTrajectory([times[j] for j in segment],[poses[j] for j in segment]).discretize_state(dt)
                disc_poses = []
                for m in hermite.milestones:
                    disc_poses.append([m[0],m[1],math.atan2(m[3],m[2])])
                trajectories.append(SE2Trajectory(hermite.times,disc_poses))
            else:
                assert all(j < len(poses) for j in segment)
                rev_poses = [[m[0],m[1],m[2]+math.pi] for m in [poses[j] for j in segment]]
                hermite = DifferentiallyFlatTrajectory([times[j] for j in segment],rev_poses).discretize_state(dt)
                disc_poses = []
                for m in hermite.milestones:
                    disc_poses.append([m[0],m[1],(math.atan2(m[3],m[2])+math.pi)%(math.pi*2)])
                trajectories.append(SE2Trajectory(hermite.times,disc_poses))
            segment = [i+1]
        else:
            segment.append(i+1)
    #concatenate everything
    res = trajectories[0]
    for t in trajectories[1:]:
        if t.milestones[0] != trajectories[0].milestones[-1]:
            #print("Jump?",t.milestones[0],trajectories[0].milestones[-1])
            t.milestones[0] = trajectories[0].milestones[-1]
        res = res.concat(t)
    return res
    
