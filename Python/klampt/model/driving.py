"""Utilities for working with trajectories of mobile robots.
"""
from klampt.model.trajectory import HermiteTrajectory, SE2Trajectory
from klampt.math import vectorops,so2
import math
from klampt.model.typing import Vector,Vector3
from typing import List,Optional,Union

class DifferentiallyFlat2DTrajectory(HermiteTrajectory):
    """A smooth 2D trajectory that can be converted into an SE(2) trajectory
    as driven by a differentially flat SO2 system, e.g., a differential drive
    robot or forward walking humanoid. 
    
    The derivatives of the curve become headings when converted.
    """
    def __init__(self, times : List[float], poses: List[Vector], velocities : Optional[List[Vector]] = None):
        """Fits the trajectory either to 2D points and vectors or to an SE2
        trajectory.
        
        Args:
            times (list of float): the times of each pose
            poses (list of 2-vectors or 3-vectors): if velocities not given, the list of SE2 poses to interpolate. 
            velocities (list of 2-vectors, optional): if given, poses are 2D milestones and these are the outgoing derivatives.
        """
        if velocities is not None:
            for p in poses:
                if len(p) != 2:
                    raise ValueError("Poses must be 2D when velocities are given")
            HermiteTrajectory.__init__(self, times, poses, velocities)
            return
        for p in poses:
            if len(p) != 3:
                raise ValueError("Poses must be (x,y,theta) when velocities aren't given")
        
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

    def eval_se2(self, t: float, endBehavior='halt', scanIndex=None) -> Vector3:
        """Evaluates the trajectory at time t, returning an SE2 pose.
        """
        i,u = self.getSegment(t,endBehavior,scanIndex)
        p = self.evalSegment_state(i,u)
        return [p[0],p[1],math.atan2(p[3],p[2])]
    
    def discretize_se2(self, dt : float) -> SE2Trajectory:
        newtimes = []
        newmilestones = []
        t = self.times[0]
        scanIndex = None
        while t < self.times[-1]:
            scanIndex,u = self.getSegment(t,scanIndex=scanIndex)
            p = self.evalSegment_state(scanIndex,u)
            newtimes.append(t)
            newmilestones.append([p[0],p[1],math.atan2(p[3],p[2])])
            t += dt
        if t != self.times[-1]:
            newtimes.append(self.times[-1])
            p = self.evalSegment_state(len(self.times)-2,1)
            newmilestones.append([p[0],p[1],math.atan2(p[3],p[2])])
        return SE2Trajectory(newtimes,newmilestones)


def discretize_differential_drive_poses_se2(times : List[float], poses: List[Vector3], dt: float) -> SE2Trajectory:
    """Given SE2 poses to interpolate for a differential drive robot,
    produces a discretized, drivable SE2Trajectory.  Properly handles
    in-place rotations and backwards motion.
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
                hermite = DifferentiallyFlat2DTrajectory([times[j] for j in segment],[poses[j] for j in segment]).discretize_state(dt)
                disc_poses = []
                for m in hermite.milestones:
                    disc_poses.append([m[0],m[1],math.atan2(m[3],m[2])])
                trajectories.append(SE2Trajectory(hermite.times,disc_poses))
            else:
                assert all(j < len(poses) for j in segment)
                rev_poses = [[m[0],m[1],m[2]+math.pi] for m in [poses[j] for j in segment]]
                hermite = DifferentiallyFlat2DTrajectory([times[j] for j in segment],rev_poses).discretize_state(dt)
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
        if t.milestones[0] != res.milestones[-1]:
            #print("Jump?",t.milestones[0],trajectories[0].milestones[-1])
            t.milestones[0] = res.milestones[-1]
        res = res.concat(t)
    return res
    

