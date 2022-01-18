"""Classes for loading, saving, evaluating, and operating on trajectories.

* For piecewise-linear interpolation in cartesian space, use :class:`~klampt.model.trajectory.Trajectory`.
* For piecewise-linear interpolation on a robot, use :class:`~klampt.model.trajectory.RobotTrajectory`.
* For Hermite interpolation in cartesian space, use :class:`~klampt.model.trajectory.HermiteTrajectory`.

"""

import bisect

from ..math import so3,se3,vectorops
from ..math import spline
from ..math.geodesic import *
import warnings
from ..robotsim import RobotModel,RobotModelLink
from .subrobot import SubRobotModel
from typing import Optional,Union,Sequence,List,Tuple,Callable
from .typing import Vector3,Vector,Rotation,RigidTransform
MetricType = Callable[[Vector,Vector],float]

class Trajectory:
    """A basic piecewise-linear trajectory class, which can be overloaded
    to provide different functionality.  A plain Trajectory interpolates
    in Cartesian space.

    (To interpolate for a robot, use RobotTrajectory. To perform
    Hermite interpolation, use HermiteTrajectory)

    Attributes:
        times (list of floats): a list of times at which the milestones are met.
        milestones (list of Configs): a list of milestones that are interpolated.

    """
        
    def __init__(self,
            times: Optional[List[float]] = None,
            milestones: Optional[List[Vector]] = None
        ):
        """Args:
            times (list of floats, optional): if provided, initializes the
                self.times attribute.  If milestones is provided, a uniform
                timing is set.  Otherwise self.times is empty.
            milestones (list of Configs, optional): if provided, initializes
                the self.milestones attribute.  Otherwise milestones is empty.

        Does not perform error checking.  The caller must be sure that
        the lists have the same size, the times are non-decreasing, and the configs
        are equally-sized (you can call checkValid() for this).
        """
        if milestones is None:
            milestones = []
        if times is None:
            times = list(range(len(milestones)))
        self.times = times
        self.milestones = milestones

    def load(self, fn: str) -> None:
        """Reads from a whitespace-separated file in the format::

            t1 [q1]
            t2 [q2]
            ...

        where each [qi] is a Klamp't formatted length-n configuration, written
        in the form ``n qi1 ... qin``.
        """
        fin = open(fn, 'r')
        self.times = []
        self.milestones = []
        for line in fin.readlines():
            timedMilestone = [float(i) for i in line.strip().split()]
            self.times.append(timedMilestone[0])
            self.milestones.append(timedMilestone[2:])
        fin.close()

    def save(self, fn: str) -> None:
        """Writes to a whitespace-separated file"""
        fout = open(fn, 'w')
        for t,x in zip(self.times,self.milestones):
            fout.write('%f\t%d '%(t,len(x)))
            fout.write(' '.join([str(xi) for xi in x]))
            fout.write('\n')
        fout.close()

    def startTime(self) -> float:
        """Returns the initial time."""
        try: return self.times[0]
        except IndexError: return 0.0
        
    def endTime(self) -> float:
        """Returns the final time."""
        try: return self.times[-1]
        except IndexError: return 0.0

    def duration(self) -> float:
        """Returns the duration of the trajectory."""
        return self.endTime()-self.startTime()

    def checkValid(self) -> None:
        """Checks whether this is a valid trajectory, raises a
        ValueError if not."""
        if len(self.times) != len(self.milestones):
            raise ValueError("Times and milestones are not the same length")
        if len(self.times)==0:
            raise ValueError("Trajectory is empty")
        for (tprev,t) in zip(self.times[:-1],self.times[1:]):
            if tprev > t:
                raise ValueError("Timing is not sorted")
        n = len(self.milestones[0])
        for q in self.milestones:
            if len(q) != n:
                raise ValueError("Invalid milestone size")
        return

    def getSegment(self, t: float, endBehavior: str = 'halt')  -> Tuple[int,float]:
        """Returns the index and interpolation parameter for the
        segment at time t. 

        Running time is O(log n) time where n is the number of segments.

        Args:
            t (float): The time at which to evaluate the segment
            endBehavior (str): If 'loop' then the trajectory loops forever.  

        Returns:
            (index,param) giving the segment index and interpolation
            parameter.  index < 0 indicates that the time is before the first
            milestone and/or there is only 1 milestone.
        """
        if len(self.times)==0:
            raise ValueError("Empty trajectory")
        if len(self.times)==1:
            return (-1,0)
        if t > self.times[-1]:
            if endBehavior == 'loop':
                try:
                    t = t % self.times[-1]
                except ZeroDivisionError:
                    t = 0
            else:
                return (len(self.milestones)-1,0)
        if t >= self.times[-1]:
            return (len(self.milestones)-1,0)
        if t <= self.times[0]:
            return (-1,0)
        i = bisect.bisect_right(self.times,t)
        p=i-1
        assert i > 0 and i < len(self.times),"Invalid time index "+str(t)+" in "+str(self.times)
        u=(t-self.times[p])/(self.times[i]-self.times[p])
        if i==0:
            if endBehavior == 'loop':
                t = t + self.times[-1]
                p = -2
                u=(t-self.times[p])/(self.times[-1]-self.times[p])
            else:
                return (-1,0)
        assert u >= 0 and u <= 1
        return (p,u)
    
    def eval(self, t: float, endBehavior: str = 'halt') -> Vector:
        """Evaluates the trajectory using piecewise linear
        interpolation. 

        Args:
            t (float): The time at which to evaluate the segment
            endBehavior (str): If 'loop' then the trajectory loops forever.  

        Returns:
            The configuration at time t
        """
        return self.eval_state(t,endBehavior)

    def deriv(self, t: float, endBehavior: str = 'halt') -> Vector:
        """Evaluates the trajectory velocity using piecewise linear
        interpolation. 

        Args:
            t (float): The time at which to evaluate the segment
            endBehavior (str): If 'loop' then the trajectory loops forever.  

        Returns:
            The velocity (derivative) at time t
        """
        return self.deriv_state(t,endBehavior)

    def waypoint(self, state: Vector) -> Vector:
        """Returns the primary configuration corresponding to the given state.

        This is usually the same as ``state`` but for some trajectories,
        specifically Hermite curves, the state and configuration are not
        identically the same.
        """
        return state

    def eval_state(self, t: float, endBehavior: str = 'halt') -> Vector:
        """Internal eval, used on the underlying state representation"""
        i,u = self.getSegment(t,endBehavior)
        if i<0: return self.milestones[0]
        elif i+1>=len(self.milestones): return self.milestones[-1]
        #linear interpolate between milestones[i] and milestones[i+1]
        return self.interpolate_state(self.milestones[i],self.milestones[i+1],u,self.times[i+1]-self.times[i])

    def deriv_state(self, t: float, endBehavior: str = 'halt') -> Vector:
        """Internal deriv, used on the underlying state representation"""
        i,u = self.getSegment(t,endBehavior)
        if i<0: return [0.0]*len(self.milestones[0])
        elif i+1>=len(self.milestones): return [0.0]*len(self.milestones[-1])
        return self.difference_state(self.milestones[i+1],self.milestones[i],u,self.times[i+1]-self.times[i])

    def interpolate_state(self, a: Vector, b: Vector, u: float, dt: float) -> Vector:
        """Can override this to implement non-cartesian spaces.
        Interpolates along the geodesic from a to b.  dt is the 
        duration of the segment from a to b"""
        return vectorops.interpolate(a,b,u)
    
    def difference_state(self, a: Vector, b: Vector, u: float, dt: float) -> Vector:
        """Subclasses can override this to implement non-Cartesian
        spaces.  Returns the time derivative along the geodesic from b to
        a, with time domain [0,dt].  In cartesian spaces, this is (a-b)/dt.
        
        Args:
            a (vector): the end point of the segment
            b (vector): the start point of the segment.
            u (float): the evaluation point of the derivative along the
                segment, with 0 indicating b and 1 indicating a
            dt (float): the duration of the segment from b to a.
        """
        return vectorops.mul(vectorops.sub(a,b),1.0/dt)
    
    def concat(self,
            suffix: 'Trajectory',
            relative: bool = False,
            jumpPolicy: str = 'strict'
        ) -> 'Trajectory':
        """Returns a new trajectory with another trajectory
        concatenated onto self.

        Args:
            suffix (Trajectory): the suffix trajectory
            relative (bool):  If True, then the suffix's time domain is shifted
                so that self.times[-1] is added on before concatenation.
            jumpPolicy (str):  If the suffix starts exactly at the existing trajectory's
                end time, then jumpPolicy is checked.  Can be:

                - 'strict': the suffix's first milestone has to be equal to the
                  existing trajectory's last milestone. Otherwise an exception
                  is raised.
                - 'blend': the existing trajectory's last milestone is
                  discarded.
                - 'jump': a discontinuity is added to the trajectory.

        """
        if self.__class__ is not suffix.__class__:
            raise ValueError("Can only concatenate like Trajectory classes: %s != %s"%(self.__class__.__name__,suffix.__class__.__name__))
        if not relative or len(self.times)==0:
            offset = 0
        else:
            offset = self.times[-1]
        if len(self.times)!=0:
            if suffix.times[0]+offset < self.times[-1]:
                raise ValueError("Invalid concatenation, suffix startTime precedes endTime")
            if suffix.times[0]+offset == self.times[-1]:
                #keyframe exactly equal; skip the first milestone
                #check equality with last milestone
                if jumpPolicy=='strict' and suffix.milestones[0] != self.milestones[-1]:
                    print("Suffix start:",suffix.milestones[0])
                    print("Self end:",self.milestones[-1])
                    raise ValueError("Concatenation would cause a jump in configuration")
                if jumpPolicy=='strict' or (jumpPolicy=='blend' and suffix.milestones[0] != self.milestones[-1]):
                    #discard last milestone of self
                    times = self.times[:-1] + [t+offset for t in suffix.times]
                    milestones = self.milestones[:-1] + suffix.milestones
                    return self.constructor()(times,milestones)
        times = self.times + [t+offset for t in suffix.times]
        milestones = self.milestones + suffix.milestones
        return self.constructor()(times,milestones)

    def insert(self, time: float) -> int:
        """Inserts a milestone and keyframe at the given time.  Returns the index of the new
        milestone, or if a milestone already exists, then it returns that milestone index.

        If the path is empty, the milestone is set to an empty list [].
        """
        if len(self.times) == 0:
            self.times = [time]
            self.milestones = [[]]
            return 0
        if time <= self.times[0]:
            if time < self.times[0]:
                self.times.insert(0,time)
                self.milestones.insert(0,self.milestones[0][:])
            return 0
        elif time >= self.times[-1]:
            if time > self.times[-1]:
                self.times.append(time)
                self.milestones.append(self.milestones[-1][:])
            return len(self.times)-1
        else:
            i,u = self.getSegment(time)
            assert i >= 0,"getSegment returned -1? something must be wrong with the times"
            if u == 0:
                return i
            elif u == 1:
                return i+1
            else:
                q = self.interpolate_state(self.milestones[i],self.milestones[i+1],u,self.times[i+1]-self.times[i])
                self.times.insert(i,time)
                self.milestones.insert(i,q)
                return i

    def split(self, time: float) -> Tuple['Trajectory','Trajectory']:
        """Returns a pair of trajectories obtained from splitting this
        one at the given time"""
        if time <= self.times[0]:
            #split before start of trajectory
            return self.constructor()([time],[self.milestones[0]]),self.constructor()([time]+self.times,[self.milestones[0]]+self.milestones)
        elif time >= self.times[-1]:
            #split after end of trajectory
            return self.constructor()(self.times+[time],self.milestones+[self.milestones[-1]]),self.constructor()([time],[self.milestones[-1]])
        i,u = self.getSegment(time)
        assert i >= 0,"getSegment returned -1? something must be wrong with the times"
        #split in middle of trajectory
        splitpt = self.interpolate_state(self.milestones[i],self.milestones[i+1],u,self.times[i+1]-self.times[i])
        front = self.constructor()(self.times[:i+1],self.milestones[:i+1])
        back = self.constructor()(self.times[i+1:],self.milestones[i+1:])
        if u > 0:
            front.times.append(time)
            front.milestones.append(splitpt)
        if u < 1:
            back.times = [time] + back.times
            back.milestones = [splitpt] + back.milestones
        return (front,back)

    def before(self, time: float) -> 'Trajectory':
        """Returns the part of the trajectory before the given time"""
        return self.split(time)[0]

    def after(self, time: float) -> 'Trajectory':
        """Returns the part of the trajectory after the given time"""
        return self.split(time)[1]

    def splice(self,
            suffix: 'Trajectory',
            time: List[float] = None,
            relative: bool = False,
            jumpPolicy: str = 'strict'
        ) -> 'Trajectory':
        """Returns a path such that the suffix is spliced in at some time

        Args:
            suffix (Trajectory): the trajectory to splice in
            time (float, optional): determines when the splice occurs.
                The suffix is spliced in at the suffix's start time if time=None,
                or the given time if specified.
            jumpPolicy (str): if 'strict', then it is required that
                suffix(t0)=path(t0) where t0 is the absolute start time
                of the suffix.

        """
        offset = 0
        if time is None:
            time = suffix.times[0]
        if relative and len(self.times) > 0:
            offset = self.times[-1]
        time = time+offset
        before = self.before(time)
        return before.concat(suffix,relative,jumpPolicy)

    def constructor(self) -> Callable[[List,List],'Trajectory']:
        """Returns a "standard" constructor for the split / concat
        routines.  The result should be a function that takes two
        arguments: a list of times and a list of milestones."""
        return Trajectory

    def length(self, metric: Optional[MetricType] = None) -> float:
        """Returns the arc-length of the trajectory, according to the given
        metric.

        If metric = None, uses the "natural" metric for this trajectory,
        which is usually Euclidean.  Otherwise it is a function f(a,b)
        from configurations to nonnegative numbers.
        """
        if metric is None:
            metric = vectorops.distance
        return sum(metric(a,b) for a,b in zip(self.milestones[:-1],self.milestones[1:]))

    def discretize_state(self, dt: float) -> 'Trajectory':
        """Returns a copy of this but with uniformly defined milestones at
        resolution dt.  Start and goal are maintained exactly"""
        assert dt > 0,"dt must be positive"
        t = self.times[0]
        new_milestones = [self.milestones[0][:]]
        new_times = [self.times[0]]
        #TODO: (T/dt) log n time, can be done in (T/dt) time
        while t+dt < self.times[-1]:
            t += dt
            new_times.append(t)
            new_milestones.append(self.eval_state(t))
        if abs(t-self.times[-1]) > 1e-6:
            new_times.append(self.times[-1])
            new_milestones.append(self.milestones[-1][:])
        else:
            new_times[-1] = self.times[-1]
            new_milestones[-1] = self.milestones[-1][:]
        return self.constructor()(new_times,new_milestones)

    def discretize(self, dt: float) -> 'Trajectory':
        """Returns a trajectory, uniformly discretized at resolution dt, and
        with state-space the same as its configuration space. Similar to
        discretize, but if the state space is of higher dimension (e.g.,
        Hermite trajectories) this projects to a piecewise linear trajectory.
        """
        return self.discretize_state(dt)

    def remesh(self, newtimes: List[float], tol: float=1e-6) -> Tuple['Trajectory',List[int]]:
        """Returns a path that has milestones at the times given in newtimes, as well
        as the current milestone times.  Return value is (path,newtimeidx) where
        path is the remeshed path, and newtimeidx is a list of time indices for which
        path.times[newtimeidx[i]] = newtimes[i].

        newtimes is an iterable over floats.  It does not need to be sorted. 

        tol is a parameter specifying how closely the returned path must interpolate
        the original path.  Old milestones will be dropped if they are not needed to follow
        the path within this tolerance.

        The end behavior is assumed to be 'halt'.
        """
        sorter = [(t,-1-i) for (i,t) in enumerate(self.times)]  + [(t,i) for (i,t) in enumerate(newtimes)]
        sorter = sorted(sorter)
        res = self.constructor()(None,None)
        res.times.append(sorter[0][0])
        res.milestones.append(self.milestones[0])
        #maybe a constant first section
        resindices = []
        i = 0
        while sorter[i][0] < self.startTime():
            if sorter[i][1] >= 0:
                resindices.append(0)
            i += 1
        if i != 0:
            res.times.append(self.startTime())
            res.milestones.append(self.milestones[0])
        firstold = 0
        lastold = 0
        while i < len(sorter):
            #check if we should add this
            t,idx = sorter[i]
            i+=1
            if idx >= 0:  #new time
                if t == res.times[-1]:
                    resindices.append(len(res.times)-1)
                    continue
                #it's a new mesh point, add it and check whether previous old milestones should be added
                if self.times[lastold] == t:
                    #matched the last old mesh point, no need to call eval_state()
                    newx = self.milestones[lastold]
                else:
                    newx = self.eval_state(t)
                res.times.append(t)
                res.milestones.append(newx)
                for j in range(firstold,lastold):
                    if self.times[j] == t:
                        continue
                    x = res.eval_state(self.times[j])
                    if vectorops.norm(self.difference_state(x,self.milestones[j],1.0,1.0)) > tol:
                        #add it
                        res.times[-1] = self.times[j]
                        res.milestones[-1] = self.milestones[j]
                        res.times.append(t)
                        res.milestones.append(newx)
                resindices.append(len(res.times)-1)
                firstold = lastold+1
            else:
                #mark the range of old milestones to add
                lastold = -idx-1
        for j in range(firstold,lastold):
            res.times.append(self.times[j])
            res.milestones.append(self.milestones[j])
        #sanity check
        for i in range(len(res.times)-1):
            assert res.times[i] < res.times[i+1]
        for i,idx in enumerate(resindices):
            assert newtimes[i] == res.times[idx],"Resindices mismatch? {} should index {} to {}".format(resindices,newtimes,res.times)
        return (res,resindices)

    def extractDofs(self,dofs:List[int]) -> 'Trajectory':
        """Returns a trajectory just over the given DOFs.

        Args:
            dofs (list of int): the indices to extract.

        Returns:
            A copy of this trajectory but only over the given DOFs.
        """
        if len(self.times)==0:
            return self.constructor()
        n = len(self.milestones[0])
        for d in dofs:
            if abs(d) >= n:
                raise ValueError("Invalid dof")
        return self.constructor([t for t in self.times],[[m[j] for j in dofs] for m in self.milestones])

    def stackDofs(self, trajs: List['Trajectory'], strict: bool = True) -> None:
        """Stacks the degrees of freedom of multiple trajectories together.
        The result is contained in self.

        All evaluations are assumed to take place with the 'halt' endBehavior.

        Args:
            trajs (list or tuple of Trajectory): the trajectories to stack
            strict (bool, optional): if True, will warn if the classes of the
                trajectories do not match self.
        """
        if not isinstance(trajs,(list,tuple)):
            raise ValueError("Trajectory.stackDofs takes in a list of trajectories as input")
        warned = not strict
        for traj in trajs:
            if traj.__class__ != self.__class__:
                if not warned:
                    warnings.warn("Trajectory.stackDofs is merging trajectories of different classes?")
                    warned = True
        alltimes = set()
        for traj in trajs:
            for t in traj.times:
                alltimes.add(t)
        self.times = sorted(alltimes)
        stacktrajs = [traj.remesh(self.times) for traj in trajs]
        for traj in stacktrajs:
            assert len(traj.milestones) == len(self.times)
        self.milestones = []
        for i,t in enumerate(self.times):
            self.milestones.append(sum([list(traj.milestones[i]) for traj in stacktrajs],[]))


class RobotTrajectory(Trajectory):
    """A trajectory that performs interpolation according to the robot's
    interpolation scheme."""
    def __init__(self,
            robot: Union[RobotModel,SubRobotModel],
            times: Optional[List[float]] = None,
            milestones: Optional[List[Vector]] = None
        ):
        """
        Args:
            robot (RobotModel or SubRobotModel): the robot whose configuration
                should follow this trajectory.
            times (list of floats, optional): if provided, initializes the
                self.times attribute.  If milestones is provided, a uniform
                timing is set.  Otherwise self.times is empty.
            milestones (list of Configs, optional): if provided, initializes
                the self.milestones attribute.  Otherwise milestones is empty.

        """
        if not isinstance(robot,(RobotModel,SubRobotModel)):
            raise ValueError("RobotTrajectory must be provided with a RobotModel or SubRobotModel as first argument")
        Trajectory.__init__(self,times,milestones)
        self.robot = robot
    def interpolate_state(self,a,b,u,dt):
        return self.robot.interpolate(a,b,u)
    def difference_state(self,a,b,u,dt):
        assert len(a) == self.robot.numLinks(),"Invalid config "+str(a)+" should have length "+str(self.robot.numLinks())
        assert len(b) == self.robot.numLinks(),"Invalid config "+str(b)+" should have length "+str(self.robot.numLinks())
        #TODO: evaluate at u units from b to a
        return vectorops.mul(self.robot.interpolateDeriv(b,a),1.0/dt)
    def constructor(self):
        return lambda times=None,milestones=None: RobotTrajectory(self.robot,times,milestones)
    def getLinkTrajectory(self,
            link: Union[int,str,RobotModelLink],
            discretization: Optional[List[float]] = None
        ) -> 'SE3Trajectory':
        """Returns the SE3Trajectory corresponding to the link's pose along the robot's
        trajectory.  If discretization = None, only the milestones are extracted.
        Otherwise, the piecewise linear approximation at dt = discretization is used.
        """
        if discretization != None:
            return self.discretize(discretization).getLinkTrajectory(link)
        if isinstance(link,(int,str)):
            link = self.robot.link(link)
        Rmilestones = []
        for m in self.milestones:
            self.robot.setConfig(m)
            Rmilestones.append(link.getTransform())
        return SE3Trajectory(self.times[:],Rmilestones)
    def length(self,metric=None):
        if metric is None:
            return Trajectory.length(self,self.robot.distance)
        else:
            return Trajectory.length(self,metric)
    def checkValid(self):
        Trajectory.checkValid(self)
        for m in self.milestones:
            if len(m) != self.robot.numLinks():
                raise ValueError("Invalid length of milestone: {} != {}".format(len(m),self.robot.numLinks()))
    def extractDofs(self, dofs: List[Union[int,str]]) -> 'RobotTrajectory':
        """Returns a RobotTrajectory just over the given DOFs.

        Args:
            dofs (list of int or str): the indices to extract

        Returns:
            A copy of this trajectory but over a SubRobotModel.
        """
        from .subrobot import SubRobotModel
        subrob = SubRobotModel(self.robot,dofs)
        if len(self.times)==0:
            return RobotTrajectory(subrob)
        return RobotTrajectory(subrob,[t for t in self.times],[[m[j] for j in subrob._links] for m in self.milestones])
    def stackDofs(self,trajs):
        Trajectory.stackDofs(self,trajs,strict=False)
        if len(self.milestones) > 0 and len(self.milestones[0]) != self.robot.numDofs():
            warnings.warn("RobotTrajectory.stackDofs: the result doesn't match the robot's #DOF")


class GeodesicTrajectory(Trajectory):
    """A trajectory that performs interpolation on a GeodesicSpace.
    See :mod:`klampt.math.geodesic` for more information."""
    def __init__(self,
            geodesic: GeodesicSpace,
            times: Optional[List[float]] = None,
            milestones: Optional[List[Vector]] = None
        ):
        self.geodesic = geodesic
        Trajectory.__init__(self,times,milestones)
    def interpolate_state(self,a,b,u,dt):
        return self.geodesic.interpolate(a,b,u)
    def difference_state(self,a,b,u,dt):
        x = self.interpolate_state(b,a,u,dt)
        return vectorops.mul(vectorops.sub(self.geodesic.difference(a,x),self.geodesic.difference(b,x)),1.0/dt)
    def constructor(self):
        return lambda times,milestones:GeodesicTrajectory(self.geodesic,times,milestones)
    def length(self,metric=None):
        if metric is None:
            return Trajectory.length(self,self.geodesic.distance)
        else:
            return Trajectory.length(self,metric)
    def checkValid(self):
        Trajectory.checkValid(self)
        try:
            d = self.geodesic.extrinsicDimension()
            for m in self.milestones:
                if len(m) != d:
                    raise ValueError("Milestone length doesn't match geodesic space's dimension: {} != {}".format(len(m),d))
        except NotImplementedError:
            pass
    def extractDofs(self,dofs):
        """Invalid for GeodesicTrajectory."""
        raise ValueError("Cannot extract DOFs from a GeodesicTrajectory")
    def stackDofs(self,trajs):
        Trajectory.stackDofs(self,trajs,strict=False)
        try:
            self.checkValid()
        except ValueError:
            warnings.warn("GeodesicTrajectory.stackDofs: the result doesn't match the geodesic's dimension")


class SO3Trajectory(GeodesicTrajectory):
    """A trajectory that performs interpolation in SO3.  Each milestone
    is a 9-D :mod:`klampt.math.so3` element."""
    def __init__(self, times: Optional[List[float]] = None, milestones: Optional[List[Vector]] = None):
        GeodesicTrajectory.__init__(self,SO3Space(),times,milestones)
    def deriv_angvel(self, t: float,endBehavior: str = 'halt') -> Vector3:
        """Returns the derivative at t, in angular velocity form"""
        cw = GeodesicTrajectory.deriv(self,t,endBehavior)
        return so3.deskew(cw)
    def preTransform(self,R: Rotation) -> None:
        """Premultiplies every rotation in here by the so3 element
        R. In other words, if R rotates a local frame F to frame F',
        this method converts this SO3Trajectory from coordinates in F
        to coordinates in F'"""
        for i,m in enumerate(self.milestones):
            self.milestones[i] = so3.mul(R,m)
    def postTransform(self,R: Rotation) -> None:
        """Postmultiplies every rotation in here by the se3 element
        R. In other words, if R rotates a local frame F to frame F',
        this method converts this SO3Trajectory from describing how F'
        rotates to how F rotates."""
        for i,m in enumerate(self.milestones):
            self.milestones[i] = so3.mul(m,R)
    def getPointTrajectory(self, localPt: Vector3) -> Trajectory:
        """Returns a Trajectory describing the movement of the point localPt
        attached to this rotating frame. """
        return Trajectory(self.times,[so3.apply(m,localPt) for m in self.milestones])
    def checkValid(self):
        Trajectory.checkValid(self)
        for m in self.milestones:
            if len(m) != 9:
                raise ValueError("Invalid length of milestone: {} != 9".format(len(m)))
    def constructor(self):
        return SO3Trajectory


class SE3Trajectory(GeodesicTrajectory):
    """A trajectory that performs interpolation in SE3.  Each milestone (state)
    is a 12-D flattened :mod:`klampt.math.se3` element (i.e., the concatenation of
    R + t for an (R,t) pair)."""
    def __init__(self,
            times: Optional[List[float]] = None,
            milestones: Optional[Union[List[Vector],List[RigidTransform]]] = None
        ):
        """Constructor can take either a list of SE3 elements or
        12-element vectors."""
        if milestones is not None and len(milestones) > 0 and len(milestones[0])==2:
            GeodesicTrajectory.__init__(self,SE3Space(),times,[m[0]+m[1] for m in milestones])
        else:
            GeodesicTrajectory.__init__(self,SE3Space(),times,milestones)
    def to_se3(self, state: Vector) -> RigidTransform:
        """Converts a state parameter vector to a klampt.se3 element"""
        return (state[:9],state[9:])
    def waypoint(self, state: Vector) -> RigidTransform:
        return self.to_se3(state)
    def from_se3(self, T: RigidTransform) -> Vector:
        """Converts a klampt.se3 element to a state parameter vector"""
        return list(T[0]) + list(T[1])
    def eval(self, t: float, endBehavior: str = 'halt') -> RigidTransform:
        """Returns an SE3 element"""
        res = self.eval_state(t,endBehavior)
        return self.to_se3(res)
    def deriv(self, t: float, endBehavior: str = 'halt') -> RigidTransform:
        """Returns the derivative as the derivatives of an SE3
        element"""
        res = self.deriv_state(t,endBehavior)
        return self.to_se3(res)
    def deriv_screw(self, t:float, endBehavior: str = 'halt') -> Tuple[Vector3,Vector3]:
        """Returns the derivative at t, in screw form, that is, a 6D
        (angular velocity,velocity) vector."""
        dT = self.deriv(t,endBehavior)
        return so3.deskew(dT[0])+dT[1]
    def preTransform(self, T: RigidTransform) -> None:
        """Premultiplies every transform in self by the se3 element
        T. In other words, if T transforms a local frame F to frame F',
        this method converts this SE3Trajectory from coordinates in F
        to coordinates in F'"""
        for i,m in enumerate(self.milestones):
            Tm = self.to_se3(m)
            self.milestones[i] = self.from_se3(se3.mul(T,Tm))
    def postTransform(self, T: RigidTransform) -> None:
        """Postmultiplies every transform in self by the se3 element
        T. In other words, if T transforms a local frame F to frame F',
        this method converts this SE3Trajectory from describing how F'
        moves to how F moves."""
        for i,m in enumerate(self.milestones):
            Tm = self.to_se3(m)
            self.milestones[i] = self.from_se3(se3.mul(Tm,T))
    def getRotationTrajectory(self) -> SO3Trajectory:
        """Returns an SO3Trajectory describing the rotation
        trajectory."""
        return SO3Trajectory(self.times,[m[:9] for m in self.milestones])
    def getPositionTrajectory(self, localPt: Optional[Vector3] = None) -> Trajectory:
        """Returns a Trajectory describing the movement of the given
        local point localPt (or the origin, if none is provided)."""
        if localPt is None:
            return Trajectory(self.times,[m[9:] for m in self.milestones])
        else:
            return Trajectory(self.times,[se3.apply(self.to_se3(m),localPt) for m in self.milestones])
    def checkValid(self):
        Trajectory.checkValid(self)
        for m in self.milestones:
            if len(m) != 9:
                raise ValueError("Invalid length of milestone: {} != 12".format(len(m)))
    def extractDofs(self, dofs: List[int]) -> Trajectory:
        if list(dofs) == list(range(9)):
            traj = Trajectory.extractDofs(self,dofs)
            return SO3Trajectory(traj.times.traj.milestones)
        elif all(d >= 9 for d in dofs):
            return Trajectory.extractDofs(self,dofs)
        else:
            raise ValueError("Cannot extract DOFs from a SE3Trajectory")
    def constructor(self):
        return SE3Trajectory


class HermiteTrajectory(Trajectory):
    """A trajectory that performs cubic interpolation between prescribed
    segment endpoints and velocities. 

    The milestones (states) are given in phase space (x,dx).
    
    ``eval(t)`` returns the primary configuration x, and ``deriv(t)``
    returns the velocity dx.  To get acceleration, use ``accel(t)``.  To get
    the state space (x,dx), use ``eval_state(t)``.

    Args:
        times (list of float, optional): the knot points
        milestones (list of lists, optional): the milestones met at the knot
            points.
        dmilestones (list of lists, optional): the velocities (derivatives
            w.r.t time) at each knot point.  

    Possible constructor options are:

    - HermiteTrajectory(): empty trajectory
    - HermiteTrajectory(times,milestones): milestones contains 
      2N-D lists consisting of the concatenation of a point and its outgoing 
      velocity.
    - HermiteTrajectory(times,milestones,dmilestones):
      milestones and dmilestones each contain N-D lists defining the points and
      outgoing velocities.

    Note: the curve is assumed to be smooth. To make a non-smooth curve,
    duplicate the knot point and milestone, but set a different velocity
    at the copy.
    """
    def __init__(self,
            times: Optional[List[float]] = None,
            milestones: Optional[List[Vector]] = None,
            dmilestones: Optional[List[Vector]] = None
        ):
        if dmilestones is None:
            Trajectory.__init__(self,times,milestones)
        else:
            assert milestones != None
            #interpret as config/velocity
            self.times = times
            self.milestones = [q+dq for (q,dq) in zip(milestones,dmilestones)]

    def makeSpline(self,
            waypointTrajectory: Trajectory,
            preventOvershoot: bool = True,
            loop: bool = False
        ) -> None:
        """Computes natural velocities for a standard configuration-
        space Trajectory to make it smoother."""
        if loop and waypointTrajectory.milestones[-1] != waypointTrajectory.milestones[0]:
            raise ValueError("Asking for a loop trajectory but the endpoints don't match up")
        velocities = []
        t = waypointTrajectory
        d = len(t.milestones[0])
        if len(t.milestones)==1:
            velocities.append([0]*d)
        elif len(t.milestones)==2:
            if loop:
                v = [0]*d
            else:
                s = (1.0/(t.times[1]-t.times[0]) if (t.times[1]-t.times[0]) != 0 else 0)
                v = vectorops.mul(vectorops.sub(t.milestones[1],t.milestones[0]),s) 
            velocities.append(v)
            velocities.append(v)
        else:
            third = 1.0/3.0
            N = len(waypointTrajectory.milestones)
            if loop:
                timeiter = zip([-2]+list(range(N-1)),range(0,N),list(range(1,N))+[1])
            else:
                timeiter = zip(range(0,N-2),range(1,N-1),range(2,N))
            for p,i,n in timeiter:
                if p < 0:
                    dtp = t.times[-1] - t.times[-2]
                else:
                    dtp = t.times[i] - t.times[p]
                if n <= i:
                    dtn = t.times[1]-t.times[0]
                else:
                    dtn = t.times[n]-t.times[i]
                assert dtp >= 0 and dtn >= 0
                s = (1.0/(dtp+dtn) if (dtp+dtn) != 0 else 0)
                v = vectorops.mul(vectorops.sub(t.milestones[n],t.milestones[p]),s)
                if preventOvershoot:
                    for j,(x,a,b) in enumerate(zip(t.milestones[i],t.milestones[p],t.milestones[n])):
                        if x <= min(a,b):
                            v[j] = 0.0
                        elif x >= max(a,b):
                            v[j] = 0.0
                        elif v[j] < 0 and x - v[j]*third*dtp >= a:
                            v[j] = 3.0/dtp*(x-a)
                        elif v[j] > 0 and x - v[j]*third*dtp <= a:
                            v[j] = 3.0/dtp*(x-a)
                        elif v[j] < 0 and x + v[j]*third*dtn < b:
                            v[j] = 3.0/dtn*(b-x)
                        elif v[j] > 0 and x + v[j]*third*dtn > b:
                            v[j] = 3.0/dtn*(b-x)
                        
                velocities.append(v)
            if not loop:
                #start velocity as quadratic
                x2 = vectorops.madd(t.milestones[1],velocities[0],-third*(t.times[1]-t.times[0]))
                x1 = vectorops.madd(x2,vectorops.sub(t.milestones[1],t.milestones[0]),-third)
                v0 = vectorops.mul(vectorops.sub(x1,t.milestones[0]),3.0/(t.times[1]-t.times[0]))
                #terminal velocity as quadratic
                xn_2 = vectorops.madd(t.milestones[-2],velocities[-1],third*(t.times[-1]-t.times[-2]))
                xn_1 = vectorops.madd(xn_2,vectorops.sub(t.milestones[-1],t.milestones[-2]),third)
                vn = vectorops.mul(vectorops.sub(t.milestones[-1],xn_1),3.0/(t.times[-1]-t.times[-2]))
                velocities = [v0]+velocities+[vn]
        self.__init__(waypointTrajectory.times[:],waypointTrajectory.milestones,velocities)

    def makeBezier(self, times: List[float], controlPoints: List[Vector]) -> None:
        """Sets up this spline to perform Bezier interpolation of the given 
        control points, with segment 0 a Bezier curve on cps[0:3], segment 1 a
        Bezier curve on cps[3:6], etc.
        """
        nsegs = len(times)-1
        if nsegs*3+1 != len(controlPoints):
            raise ValueError("To perform Bezier interpolation, need # of controlPoints to be 3*Nsegs+1")
        newtimes = []
        milestones = []
        outgoingVelocities = []
        for i in range(0,len(times)-1):
            a,b,c,d = controlPoints[i*3:i*3+4]
            dt = times[i+1]-times[i]
            if dt <= 0: raise ValueError("Times must be strictly monotonically increasing")
            lieDeriv0 = vectorops.mul(vectorops.sub(b,a),3/dt)
            lieDeriv1 = vectorops.mul(vectorops.sub(c,d),-3/dt)
            if i > 0:
                if vectorops.distance(lieDeriv0,outgoingVelocities[-1]) > 1e-4:
                    #need to double up knot point
                    newtimes.append(newtimes[-1])
                    milestones.append(milestones[-1])
                    outgoingVelocities.append(lieDeriv0)
            else:
                newtimes.append(times[i])
                milestones.append(a)
                outgoingVelocities.append(lieDeriv0)
            newtimes.append(times[i+1])
            milestones.append(d)
            outgoingVelocities.append(lieDeriv1)
        self.__init__(newtimes,milestones,outgoingVelocities)

    def makeMinTimeSpline(self,
            milestones: List[Vector],
            velocities: Optional[List[Vector]] = None,
            xmin: Optional[Vector] = None,
            xmax: Optional[Vector] = None,
            vmax: Optional[Vector] = None,
            amax: Optional[Vector] = None
        ) -> None:
        """Creates a spline that interpolates between the given milestones with
        bounded velocities, accelerations, and positions. 

        If velocities==None, this requires the spline to move in a straight
        configuration-space path between the given milestones.  This option is 
        helpful for postprocessing the results for kinematic motion planning, 
        for example.
        """
        from ..plan import motionplanning
        if vmax is None and amax is None:
            raise ValueError("Either vmax or amax must be provided")
        if len(milestones) == 0 or len(milestones[0]) == 0:
            raise ValueError("Milestones need to be provided and at least 1-d")
        n = len(milestones[0])
        for m in milestones[1:]:
            if len(m) != n:
                raise ValueError("Invalid size of milestone")
        if velocities is not None:
            if len(velocities) != len(milestones):
                raise ValueError("Velocities need to have the same size as milestones")
            for v in velocities:
                if len(v) != n:
                    raise ValueError("Invalid size of velocity milestone")
        inf = float('inf')
        if xmin is None:
            xmin = [-inf]*n
        else:
            if len(xmin) != n:
                raise ValueError("Invalid size of lower bound")
        if xmax is None:
            xmax = [inf]*n
        else:
            if len(xmax) != n:
                raise ValueError("Invalid size of upper bound")
        if vmax is None:
            vmax = [inf]*n
        else:
            if len(vmax) != n:
                raise ValueError("Invalid size of velocity bound")
        if amax is None:
            #do a piecewise linear interpolation, ignore x bounds
            raise NotImplementedError("TODO: amax = None case")
        else:
            if len(amax) != n:
                raise ValueError("Invalid size of acceleration bound")
            zeros = [0]*n
            newtimes = [0]
            newmilestones = [milestones[0]]
            newvelocities = [velocities[0] if velocities is not None else zeros]
            for i in range(len(milestones)-1):
                m0 = milestones[i]
                m1 = milestones[i+1]
                if velocities is None:
                    ts,xs,vs = motionplanning.interpolate_nd_min_time_linear(m0,m1,vmax,amax)
                else:
                    v0 = velocities[i]
                    v1 = velocities[i+1]
                    ts,xs,vs = motionplanning.interpolate_nd_min_time(m0,v0,m1,v1,xmin,xmax,vmax,amax)
                    ts,xs,vs = motionplanning.combine_nd_cubic(ts,xs,vs)
                newtimes += [newtimes[-1] + t for t in ts[1:]]
                newmilestones += xs[1:]
                newvelocities += vs[1:]
            self.__init__(newtimes,newmilestones,newvelocities)

    def waypoint(self,state):
        return state[:len(state)//2]

    def eval_state(self,t,endBehavior='halt'):
        """Returns the (configuration,velocity) state at time t."""
        return Trajectory.eval_state(self,t,endBehavior)

    def eval(self,t,endBehavior='halt'):
        """Returns just the configuration component of the result"""
        res = Trajectory.eval_state(self,t,endBehavior)
        return res[:len(res)//2]
    
    def deriv(self,t,endBehavior='halt'):
        """Returns just the velocity component of the result"""
        res = Trajectory.eval_state(self,t,endBehavior)
        return res[len(res)//2:]

    def eval_accel(self,t,endBehavior='halt') -> Vector:
        """Returns just the acceleration component of the derivative"""
        res = Trajectory.deriv_state(self,t,endBehavior)
        return res[len(res)//2:]

    def interpolate_state(self,a,b,u,dt):
        assert len(a)==len(b)
        x1,v1 = a[:len(a)//2],vectorops.mul(a[len(a)//2:],dt)
        x2,v2 = b[:len(b)//2],vectorops.mul(b[len(b)//2:],dt)
        x = spline.hermite_eval(x1,v1,x2,v2,u)
        dx = vectorops.mul(spline.hermite_deriv(x1,v1,x2,v2,u),1.0/dt)
        return x+dx
    
    def difference_state(self,a,b,u,dt):
        assert len(a)==len(b)
        x1,v1 = a[:len(a)//2],vectorops.mul(a[len(a)//2:],dt)
        x2,v2 = b[:len(b)//2],vectorops.mul(b[len(b)//2:],dt)
        dx = vectorops.mul(spline.hermite_deriv(x1,v1,x2,v2,u,order=1),1.0/dt)
        ddx = vectorops.mul(spline.hermite_deriv(x1,v1,x2,v2,u,order=2),1.0/pow(dt,2))
        return dx+ddx

    def discretize(self,dt):
        """Creates a discretized piecewise linear Trajectory in config space
        that approximates this curve with resolution dt.
        """
        res = self.discretize_state(dt)
        n = len(res.milestones[0])//2
        return Trajectory(res.times,[m[:n] for m in res.milestones])

    def length(self) -> float:
        """Returns an upper bound on length given by the Bezier property. 
        Faster than calculating the true length.  To retrieve an approximation
        of true length, use self.discretize(dt).length().
        """
        n = len(self.milestones[0])//2
        third = 1.0/3.0
        def distance(x,y):
            cp0 = x[:n]
            cp1 = vectorops.madd(cp0,x[n:],third)
            cp3 = y[:n]
            cp2 = vectorops.madd(cp3,y[n:],-third)
            return third*vectorops.norm(x[n:]) + vectorops.distance(cp1,cp2) + third*vectorops.norm(y[n:])
        return Trajectory.length(self,distance)

    def checkValid(self):
        Trajectory.checkValid(self)
        for m in self.milestones:
            if len(m)%2 != 0:
                raise ValueError("Milestone length isn't even?: {} != {}".format(len(m)))

    def extractDofs(self,dofs) -> 'HermiteTrajectory':
        """Returns a trajectory just over the given DOFs.

        Args:
            dofs (list of int): the (primary) indices to extract. Each entry
            must be < len(milestones[0])/2.

        Returns:
            A copy of this trajectory but only over the given DOFs.
        """
        if len(self.times)==0:
            return self.constructor()
        n = len(self.milestones[0])//2
        for d in dofs:
            if abs(d) >= n:
                raise ValueError("Invalid dof")
        return self.constructor([t for t in self.times],[[m[j] for j in dofs] + [m[n+j] for j in dofs] for m in self.milestones])

    def stackDofs(self,trajs,strict=True) -> None:
        """Stacks the degrees of freedom of multiple trajectories together.
        The result is contained in self.

        All evaluations are assumed to take place with the 'halt' endBehavior.

        Args:
            trajs (list or tuple of HermiteTrajectory): the trajectories to 
                stack
            strict (bool, optional): ignored. Will always warn for invalid
                classes.
        """
        if not isinstance(trajs,(list,tuple)):
            raise ValueError("HermiteTrajectory.stackDofs takes in a list of trajectories as input")
        for traj in trajs:
            if not isinstance(traj,HermiteTrajectory):
                raise ValueError("Can't stack non-HermiteTrajectory objects into a HermiteTrajectory")
        alltimes = set()
        for traj in trajs:
            for t in traj.times:
                alltimes.add(t)
        self.times = sorted(alltimes)
        stacktrajs = [traj.remesh(self.times) for traj in trajs]
        for traj in stacktrajs:
            assert len(traj.milestones) == len(self.times)
        self.milestones = []
        for i,t in enumerate(self.times):
            q = []
            v = []
            for traj in stacktrajs:
                n = len(traj.milestones[i])//2
                q += list(traj.milestones[i][:n])
                v += list(traj.milestones[i][n:])
            self.milestones.append(q + v)

    def constructor(self):
        return HermiteTrajectory


class GeodesicHermiteTrajectory(Trajectory):
    """A trajectory that performs Hermite interpolation on a GeodesicSpace
    using the DeCastlejau algorithm.

    The milestones are a concatenation of the segment start point and the
    outgoing Lie derivatives w.r.t. t. The incoming Lie derivative at the 
    segment end point is assumed to be the negative of the outgoing Lie
    derivative.

    Args:
        geodesic (GeodesicSpace): the underlying space
        times (list of floats, optional): the knot points defining each segment
        milestones (list of lists, optional): the points at the ends of each
            segment
        outgoingLieDerivatives (list of lists, optional): the Lie derivatives
            (velocities) at the ends of each segment.

    Possible constructor options are:

    - GeodesicHermiteTrajectory(geodesic): empty trajectory
    - GeodesicHermiteTrajectory(geodesic,times,milestones): milestones contains 
      2N-D lists consisting of the concatenation of a point and its outgoing 
      Lie derivative.
    - GeodesicHermiteTrajectory(geodesic,times,milestones,lieDerivatives):
      milestones and lieDerivatives contain N-D lists defining the points and
      outgoing Lie derivatives.

    Note: the curve is assumed to be smooth. To make a non-smooth curve,
    duplicate the knot point and milestone, but set a different Lie derivative
    at the copy.
    """
    def __init__(self,
            geodesic: GeodesicSpace,
            times: Optional[List[float]] = None,
            milestones: Optional[List[Vector]] = None,
            outgoingLieDerivatives: Optional[List[Vector]] = None
        ):
        self.geodesic = geodesic
        if outgoingLieDerivatives is not None:
            assert milestones is not None
            milestones = [list(a)+list(b) for (a,b) in zip(milestones,outgoingLieDerivatives)]
        if milestones is not None:
            assert all(len(m)==geodesic.extrinsicDimension()*2 for m in milestones),"Milestones must be a concatenation of the point and outgoing milestone"
        Trajectory.__init__(self,times,milestones)
        self._skip_deriv = False

    def makeSpline(self, waypointTrajectory: Trajectory, loop: bool=False) -> None:
        """Creates a spline from a set of waypoints, with smooth interpolation
        between waypoints."""
        if loop and waypointTrajectory.milestones[-1] != waypointTrajectory.milestones[0]:
            print(waypointTrajectory.milestones[-1],"!=",waypointTrajectory.milestones[0])
            raise ValueError("Asking for a loop trajectory but the endpoints don't match up")
        velocities = []
        t = waypointTrajectory
        d = len(t.milestones[0])
        third = 1.0/3.0
        if len(t.milestones)==1:
            velocities.append([0]*d)
        elif len(t.milestones)==2:
            if loop:
                v = [0.0]*d
                velocities = [v,v]
            else:
                s = (1.0/(t.times[1]-t.times[0]) if (t.times[1]-t.times[0]) != 0 else 0)
                v = vectorops.mul(self.geodesic.difference(t.milestones[1],t.milestones[0]),s) 
                velocities.append(v)
                v2 = vectorops.mul(self.geodesic.difference(t.milestones[0],t.milestones[1]),-s) 
                velocities.append(v2)
        else:
            N = len(waypointTrajectory.milestones)
            if loop:
                timeiter = zip([-2]+list(range(N-1)),range(0,N),list(range(1,N))+[1])
            else:
                timeiter = zip(range(0,N-2),range(1,N-1),range(2,N))
            for p,i,n in timeiter:
                if p < 0: dtp = t.times[-1] - t.times[-2]
                else: dtp = t.times[i] - t.times[p]
                if n <= i: dtn = t.times[1]-t.times[0]
                else: dtn = t.times[n]-t.times[i]
                assert dtp >= 0 and dtn >= 0
                s2 = (1.0/dtn if dtn != 0 else 0)
                v2 = vectorops.mul(self.geodesic.difference(t.milestones[n],t.milestones[i]),s2)
                s1 = (1.0/dtp if dtp != 0 else 0)
                v1 = vectorops.mul(self.geodesic.difference(t.milestones[p],t.milestones[i]),-s1)
                v = vectorops.mul(vectorops.add(v1,v2),0.5)
                velocities.append(v)
            if not loop:
                #start velocity as linear
                v0 = vectorops.mul(self.geodesic.difference(t.milestones[1],t.milestones[0]),1.0/(t.times[1]-t.times[0]))
                #terminal velocity as quadratic
                vn = vectorops.mul(self.geodesic.difference(t.milestones[-2],t.milestones[-1]),-1.0/(t.times[-1]-t.times[-2]))
                velocities = [v0]+velocities+[vn]
            else:
                assert len(velocities) == N
        GeodesicHermiteTrajectory.__init__(self,self.geodesic,waypointTrajectory.times[:],waypointTrajectory.milestones,velocities)

    def makeBezier(self, times: Vector, controlPoints:List[Vector]) -> None:
        """Sets up this spline to perform Bezier interpolation of the given
        control points, with segment 0 a Bezier curve on cps[0:3], segment 1 a
        Bezier curve on cps[3:6], etc.
        """
        nsegs = len(times)-1
        if nsegs*3+1 != len(controlPoints):
            raise ValueError("To perform Bezier interpolation, need # of controlPoints to be 3*Nsegs+1")
        newtimes = []
        milestones = []
        outgoingLieDerivatives = []
        for i in range(0,len(times)-1):
            a,b,c,d = controlPoints[i*3:i*3+4]
            dt = times[i+1]-times[i]
            if dt <= 0: raise ValueError("Times must be strictly monotonically increasing")
            lieDeriv0 = vectorops.mul(self.geodesic.difference(b,a),3/dt)
            lieDeriv1 = vectorops.mul(self.geodesic.difference(c,d),-3/dt)
            if i > 0:
                if vectorops.distance(lieDeriv0,outgoingLieDerivatives[-1]) > 1e-4:
                    #need to double up knot point
                    newtimes.append(newtimes[-1])
                    milestones.append(milestones[-1])
                    outgoingLieDerivatives.append(lieDeriv0)
            else:
                newtimes.append(times[i])
                milestones.append(a)
                outgoingLieDerivatives.append(lieDeriv0)
            newtimes.append(times[i+1])
            milestones.append(d)
            outgoingLieDerivatives.append(lieDeriv1)
        GeodesicHermiteTrajectory.__init__(self,self.geodesic,newtimes,milestones,outgoingLieDerivatives)

    def waypoint(self,state):
        return state[:len(state)//2]
    def interpolate_state(self,a,b,u,dt):
        n = self.geodesic.extrinsicDimension()
        assert len(a) == n*2
        assert len(b) == n*2
        c0 = a[:n]
        v0 = a[n:]
        c3 = b[:n]
        v3 = b[n:]
        third = 1.0/3.0
        c1 = self.geodesic.integrate(c0,vectorops.mul(v0,third*dt))
        c2 = self.geodesic.integrate(c3,vectorops.mul(v3,-third*dt))
        d0 = self.geodesic.interpolate(c0,c1,u)
        d1 = self.geodesic.interpolate(c1,c2,u)
        d2 = self.geodesic.interpolate(c2,c3,u)
        e0 = self.geodesic.interpolate(d0,d1,u)
        e1 = self.geodesic.interpolate(d1,d2,u)
        f = self.geodesic.interpolate(e0,e1,u)
        if self._skip_deriv:
            v = [0.0]*n
        else:
            #since it's difficult to do the derivatives analytically, do finite differences instead
            eps = 1e-6
            u2 = u + eps
            d0 = self.geodesic.interpolate(c0,c1,u2)
            d1 = self.geodesic.interpolate(c1,c2,u2)
            d2 = self.geodesic.interpolate(c2,c3,u2)
            e0 = self.geodesic.interpolate(d0,d1,u2)
            e1 = self.geodesic.interpolate(d1,d2,u2)
            f2 = self.geodesic.interpolate(e0,e1,u2)
            v = vectorops.mul(self.geodesic.difference(f2,f),1.0/eps)
        return f + v
    def difference_state(self,a,b,u,dt):
        raise NotImplementedError("Can't do derivatives of Bezier geodesic yet")
    def eval_state(self,t,endBehavior='halt'):
        """Returns the (configuration,velocity) state at time t."""
        return Trajectory.eval_state(self,t,endBehavior)
    def eval(self,t,endBehavior='halt'):
        """Evaluates the configuration at time t"""
        self._skip_deriv = True
        res = Trajectory.eval_state(self,t,endBehavior)
        self._skip_deriv = False
        return res[:len(res)//2]    
    def deriv(self,t,endBehavior='halt'):
        """Returns the velocity at time t"""
        res = Trajectory.eval_state(self,t,endBehavior)
        return res[len(res)//2:]
    def length(self,metric=None):
        """Upper bound on the length"""
        if metric is None:
            metric = self.geodesic.distance
        n = self.geodesic.extrinsicDimension()
        l = 0
        for i,(a,b) in enumerate(zip(self.milestones[:-1],self.milestones[1:])):
            dt = self.times[i+1]-self.times[i]
            c0 = a[:n]
            v0 = vectorops.mul(a[n:],dt)
            c3 = b[:n]
            v3 = vectorops.mul(b[n:],dt)
            third = 1.0/3.0
            c1 = self.geodesic.integrate(c0,v0,third)
            c2 = self.geodesic.integrate(c3,v3,-third)
            l += metric(c0,c1)
            l += metric(c1,c2)
            l += metric(c2,c3)
        return l
    def discretize(self,dt):
        """Creates a discretized piecewise-geodesic (GeodesicTrajectory)
        approximation of this curve in config space, with resolution dt.
        """
        self._skip_deriv = True
        res = self.discretize_state(dt)
        self._skip_deriv = False
        n = self.geodesic.extrinsicDimension()
        return GeodesicTrajectory(self.geodesic,res.times,[m[:n] for m in res.milestones])
    def length(self) -> float:
        """Returns an upper bound on length given by the Bezier property. 
        Faster than calculating the true length.  To retrieve an approximation
        of true length, use self.discretize(dt).length().
        """
        n = self.geodesic.extrinsicDimension()
        third = 1.0/3.0
        def distance(x,y):
            cp0 = x[:n]
            cp1 = self.geodesic.integrate(cp0,vectorops.mul(x[n:],third))
            cp3 = y[:n]
            cp2 = self.geodesic.integrate(cp3,vectorops.mul(y[n:],-third))
            return self.geodesic.distance(cp0,cp1) + self.geodesic.distance(cp1,cp2) + self.geodesic.distance(cp2,cp3)
        return Trajectory.length(self,distance)
    def checkValid(self):
        Trajectory.checkValid(self)
        n = self.geodesic.extrinsicDimension()
        for m in self.milestones:
            if len(m) != n*2:
                raise ValueError("Invalid length of milestone: {} != {}*2".format(len(m),n))
    def extractDofs(self,dofs):
        """Invalid for GeodesicHermiteTrajectory."""
        raise ValueError("Cannot extract DOFs from a GeodesicHermiteTrajectory")
    def stackDofs(self,trajs,strict=True):
        raise ValueError("Cannot stack DOFs for a GeodesicHermiteTrajectory")
    def constructor(self):
        return lambda times,milestones:GeodesicHermiteTrajectory(self.geodesic,times,milestones)
    


class SO3HermiteTrajectory(GeodesicHermiteTrajectory):
    """A trajectory that performs Hermite interpolation in SO3.  Each milestone
    is 18-D, consisting of a 9-D :mod:`klampt.math.so3` element and its
    subsequent Lie derivative.

    Args:
        times (list of float): knot points.
        milestones (list of 9-D lists): list of waypoint orientations.
        outgoingLieDerivatives (list of 9-D lists): list of Lie derivatives,
            i.e. cross product matrix (:func:`~klampt.math.so3.cross_product`)
            for each angular velocity.
    """
    def __init__(self,
            times: Optional[List[float]] = None,
            milestones: Optional[List[Vector]] = None,
            outgoingLieDerivatives: Optional[List[Vector]] = None
        ):
        GeodesicHermiteTrajectory.__init__(self,SO3Space(),times,milestones,outgoingLieDerivatives)
    def preTransform(self, R: Rotation) -> None:
        """Premultiplies every rotation in here by the so3 element
        R. In other words, if R rotates a local frame F to frame F',
        this method converts this SO3HermiteTrajectory from coordinates in F
        to coordinates in F'"""
        for i,m in enumerate(self.milestones):
            assert len(m) == 18
            mq = m[:9]
            mv = m[9:]
            self.milestones[i] = so3.mul(R,mq) + so3.mul(R,mv)
    def deriv_angvel(self, t: float, endBehavior: str = 'halt') -> Vector3:
        """Returns the derivative at t, in angular velocity form"""
        dR = GeodesicHermiteTrajectory.eval_velocity(self,t,endBehavior)
        return so3.deskew(dR)
    def postTransform(self, R:Rotation) -> None:
        """Postmultiplies every rotation in here by the se3 element
        R. In other words, if R rotates a local frame F to frame F',
        this method converts this SO3HermiteTrajectory from describing how F'
        rotates to how F rotates."""
        for i,m in enumerate(self.milestones):
            assert len(m) == 18
            mq = m[:9]
            mv = m[9:]
            self.milestones[i] = so3.mul(mq,R) + so3.mul(mv,R)
    def discretize(self,dt):
        self._skip_deriv = True
        res = self.discretize_state(dt)
        self._skip_deriv = False
        n = 9
        return SO3Trajectory(res.times,[m[:n] for m in res.milestones])
    def constructor(self):
        return SO3HermiteTrajectory
    


class SE3HermiteTrajectory(GeodesicHermiteTrajectory):
    """A trajectory that performs Bezier interpolation in SE3.  Each milestone
    is 24-D, consisting of a 12-D flattened :mod:`klampt.math.se3` element and
    its subsequent Lie derivative.
    """
    def __init__(self,times=None,milestones=None,outgoingLieDerivatives=None):
        if milestones is not None and len(milestones) > 0 and len(milestones[0])==2:
            milestones = [R+t for (R,t) in milestones]
        if outgoingLieDerivatives is not None and len(outgoingLieDerivatives) > 0 and len(outgoingLieDerivatives[0])==2:
            outgoingLieDerivatives = [R+t for (R,t) in outgoingLieDerivatives]
        GeodesicHermiteTrajectory.__init__(self,SE3Space(),times,milestones,outgoingLieDerivatives)
    def to_se3(self, state: Vector) -> RigidTransform:
        """Converts a state parameter vector to a klampt.se3 element"""
        return (state[:9],state[9:12])
    def from_se3(self, T: RigidTransform) -> Vector:
        """Converts a klampt.se3 element to a state parameter vector"""
        return list(T[0]) + list(T[1])
    def waypoint(self,state):
        return self.to_se3(state)
    def eval(self, t: float, endBehavior: str = 'halt') -> RigidTransform:
        """Returns an SE3 element"""
        res = GeodesicHermiteTrajectory.eval(self,t,endBehavior)
        return self.to_se3(res)
    def deriv(self, t: float, endBehavior: str = 'halt') -> RigidTransform:
        """Returns the derivative as the derivatives of an SE3
        element"""
        res = GeodesicHermiteTrajectory.deriv(self,t,endBehavior)
        return self.to_se3(res[:12])
    def deriv_screw(self, t: float, endBehavior: str = 'halt') -> Vector:
        """Returns the derivative at t, in screw vector form, that is, a 6D
        vector (angular velocity, velocity)."""
        dT = self.deriv(t,endBehavior)
        return so3.deskew(dT[0])+dT[1]
    def preTransform(self, T: RigidTransform) -> None:
        """Premultiplies every transform in here by the se3 element T. In other
        words, if T transforms a local frame F to frame F', this method
        converts this SE3HermiteTrajectory from coordinates in F to coordinates
        in F'"""
        for i,m in enumerate(self.milestones):
            assert len(m) == 24
            mq = self.to_se3(m[:12])
            mv = self.to_se3(m[12:])
            self.milestones[i] = self.from_se3(se3.mul(T,mq)) + self.from_se3((so3.mul(T[0],mv[0]),so3.apply(T[0],mv[1])))
    def postTransform(self,T: RigidTransform) -> None:
        """Postmultiplies every transform in here by the se3 element
        R. In other words, if R rotates a local frame F to frame F',
        this method converts this SO3Trajectory from describing how F'
        rotates to how F rotates."""
        for i,m in enumerate(self.milestones):
            assert len(m) == 24
            mq = self.to_se3(m[:12])
            mv = self.to_se3(m[12:])
            self.milestones[i] = self.from_se3(se3.mul(mq,T)) + self.from_se3((so3.mul(mv[0],T[0]),so3.apply(so3.inv(T[0]),mv[1])))
    def discretize(self,dt):
        self._skip_deriv = True
        res = self.discretize_state(dt)
        self._skip_deriv = False
        n = 12
        return SE3Trajectory(res.times,[m[:n] for m in res.milestones])
    def constructor(self):
        return SE3HermiteTrajectory


try:
    from typing import Literal
    _VELOCITIES_OPTIONS = Literal['auto','trapezoidal','constant','triangular','parabolic','cosine','minimum-jerk','optimal']
    _TIMING_OPTIONS = Literal['limited','uniform','path','L2','Linf','robot','sqrt-L2','sqrt-Linf','sqrt-robot']
    _SMOOTHING_OPTIONS = Literal['linear','cubic','spline','ramp']
    _SMOOTHING_OPTIONS2 = Literal['spline','pause']
except ImportError:
    _VELOCITIES_OPTIONS = str
    _TIMING_OPTIONS = str
    _SMOOTHING_OPTIONS = str
    _SMOOTHING_OPTIONS2 = str

def path_to_trajectory(
        path: Union[Sequence[Vector],Trajectory,RobotTrajectory],
        velocities: _VELOCITIES_OPTIONS = 'auto',
        timing: Union[_TIMING_OPTIONS,List[float],MetricType]= 'limited',
        smoothing: str='spline',
        stoptol: Optional[float] = None,
        vmax: Union[str,float,Vector] = 'auto',
        amax: Union[str,float,Vector] = 'auto',
        speed: float = 1.0,
        dt: float = 0.01,
        startvel: float = 0.0,
        endvel: float = 0.0,
        verbose: int = 0
    ) -> Trajectory:
    """Converts an untimed path to a timed trajectory.

    The resulting trajectory passes through each of the milestones **without
    stopping**, except for "stop" milestones.  Stop milestones by default are
    only the first and last milestone, but if ``stoptol`` is given, then the
    trajectory will be stopped if the curvature of the path exceeds this value.

    The first step is to assign each segment a 'distance' d[i] suggesting how
    much time it would take to traverse that much spatial distance.  This
    distance assignment method is controlled by the ``timing`` parameter.

    The second step is to smooth the spline, if smoothing='spline' is given
    (default).

    The third step creates the trajectory, assigning the times and velocity
    profile as specified by the ``velocities`` parameter.  ``velocities``
    dictates how the overall velocity profile behaves from beginning to end,
    and basically, each profile gradually speeds up and slows down.  The
    path length L = :math:`\sum_{i=1}^n d[i]` determines the duration T of
    the trajectory, as follows:

    - For constant velocity profiles, T=L. 
    - For trapezoidal, triangular, parabolic, and cosine, T = sqrt(L). 
    - For minimum-jerk, T = L^(1/3). 

    The fourth step is to time scale the result to respect limits velocity /
    acceleration limits, if timing=='limited' or speed=='limited'.

    The fifth step is to time scale the result by speed.

    .. note::

        There are only some meaningful combinations of arguments:

        - velocities='auto'; timing='limited': a generates a timed spline using a
          heuristic and then revises it to respect velocity and acceleration limits.

          The limited timing heuristic works best when the milestones are widely
          spaced.

          Be sure to specify vmax and amax if you don't have a RobotTrajectory.

        - velocities='auto', 'trapezoidal', 'triangular', 'parabolic', 'cosine', or
          'minimum-jerk';
          timing='L2', 'Linf', 'robot', 'sqrt-L2', 'sqrt-Linf', or 'sqrt-robot':
          an entirely heuristic approach. 

          The sqrt values lead to somewhat better tangents for smoothed splines with
          nonuniform distances between milestones.

          In these cases, vmax and amax are ignored.

        - If path uses non-Euclidean interpolation, then smoothing=None should be
          provided.  Smoothing is not yet supported for non-Euclidean spaces (e.g.,
          robots with special joints, SO3, SE3).

    Args:
        path: a list of milestones, a trajectory, or a RobotTrajectory.  In the
            latter cases, if durations = 'path' then the durations are extracted
            from the trajectory's timing.

        velocities (str, optional): the manner in which velocities are assigned
            along the path. Can be:

            - 'auto' (default): if timing is 'limited', this is equivalent to
              'constant'. Otherwise, this is equivalent to 'trapezoidal'.
            - 'trapezoidal': a trapezoidal velocity profile with max
              acceleration and velocity.  If timing is 'limited', the velocity
              max is determined by vmax.  Otherwise, the ramp
              up proceeds for 1/4 of the time, stays constant 1/2 of the time,
              and then ramps down for 1/4 of the time.
            - 'constant': the path is executed at fixed constant velocity
            - 'triangular': velocities are ramped up for 1/2 of the duration
              and then ramped back down.
            - 'parabolic': a parabolic curve (output is a Hermite spline)
            - 'cosine': velocities follow (1-cosine)/2
            - 'minimum-jerk': minimum jerk velocities
            - 'optimal': uses time scaling optimization. NOT IMPLEMENTED YET

        timing (optional): affects how path timing between milestones is
            normalized. Valid options are:

            - 'limited' (default): uses the vmax, amax variables along with
              the velocity profile to dynamically determine the duration
              assigned to each segment.
            - 'uniform': base timing between milestones is uniform
              (1/(\|path\|*speed))
            - 'path': only valid if path is a Trajectory object.  Uses the
              timing in path.times as the base timing.
            - 'L2': base timing is set proportional to L2 distance between
              milestones
            - 'Linf': base timing is set proportional to L-infinity distance
              between milestones
            - 'robot': base timing is set proportional to robot's distance
              function between milestones
            - 'sqrt-L2', 'sqrt-Linf', or 'sqrt-robot': base timing is set
              proportional to the square root of the L2, Linf, or robot
              distance between milestones
            - a list or tuple: the base timing is given in this list
            - callable function f(a,b): sets the normalization to the function
              f(a,b).

        smoothing (str, optional): if 'spline', the geometric path is first
            smoothed before assigning times.  Otherwise, the geometric path
            is interpreted as a piecewise linear path.

        stoptol (float, optional): determines how start/stop segments are
            determined.  If None, the trajectory only pauses at the start and
            end of the path.  If 0, it pauses at every milestone. Otherwise,
            it pauses if the curvature at the milestone exceeds stoptol.

        vmax (optional): only meaningful if timing=='limited'. Can be:

            - 'auto' (default): either 1 or the robot's joint velocity limits
              if a RobotTrajectory is provided
            - a positive number: the L2 norm of the derivative of the result
              trajectory is limited to this value
            - a list of positive floats: the element-wise derivative of the
              result trajectory is limited to this value

        amax (optional): only meaningful if timing=='limited'. Can be:

            - 'auto' (default): either 4 or the robot's joint acceleration
              limits if a RobotTrajectory is provided
            - a positive number: the L2 norm of the acceleration of the result
              trajectory is limited to this value
            - a list of positive floats: the element-wise acceleration of the
              result trajectory is limited to this value.

        speed (float or str, optional): if a float, this is a speed multiplier
            applied to the resulting trajectory.  This can also be 'limited',
            which applies the velocity and acceleration limits.

        dt (float, optional): the resolution of the resulting trajectory. 
            Default 0.01.

        startvel (float, optional): the starting velocity of the path, given as
            a multiplier of path[1]-path[0].  Must be nonnegative. 

            Note: might not be respected for some velocity profiles.

            .. warning::
                NOT IMPLEMENTED YET

        endvel (float, optional): the ending velocity of the path, given as a
            multiplier of path[-1]-path[-2].  Must be nonnegative.

            Note: might not be respected for some velocity profiles.

            .. warning::
                NOT IMPLEMENTED YET

        verbose (int, optional): if > 0, some debug printouts will be given.

    Returns:
        A finely-discretized, timed trajectory that is C1 continuous
        and respects the limits defined in the arguments.
    """
    assert dt > 0.0,"dt has to be positive"
    if vmax == 'auto' and (timing == 'limited' or speed == 'limited'):
        if isinstance(path,RobotTrajectory):
            vmax = path.robot.getVelocityLimits()
        else:
            vmax = 1.0
    if amax == 'auto' and (timing == 'limited' or speed == 'limited'):
        if isinstance(path,RobotTrajectory):
            amax = path.robot.getAccelerationLimits()
        else:
            amax = 4.0
    if isinstance(speed,(int,float)) and speed != 1.0:
        if not (speed > 0):
            raise ValueError("Invalid value for speed, must be positive")
        dt *= speed
        startvel /= speed
        endvel /= speed
                    
    milestones = path
    if isinstance(path,Trajectory):
        milestones = path.milestones

    _durations = None
    if isinstance(timing,(list,tuple)):
        _durations = timing
    elif callable(timing):
        _durations = [timing(a,b) for a,b in zip(milestones[:-1],milestones[1:])]
    else:
        if isinstance(path,Trajectory):
            if timing == 'path':
                _durations = [(b-a) for a,b in zip(path.times[:-1],path.times[1:])]
        if _durations is None:
            if timing == 'limited':
                if hasattr(vmax,'__iter__'):
                    if not all(v >= 0 for v in vmax):
                        raise ValueError("Invalid value for vmax, must be positive")
                else:
                    if not vmax >= 0:
                        raise ValueError("Invalid value for vmax, must be positive")

                if hasattr(amax,'__iter__'):
                    if not all(v >= 0 for v in amax):
                        raise ValueError("Invalid value for amax, must be positive")
                else:
                    if not amax >= 0:
                        raise ValueError("Invalid value for amax, must be positive")
                _durations = [0.0]*(len(milestones)-1)
                for i in range(len(milestones)-1):
                    q,n = milestones[i],milestones[i+1]
                    if i == 0: p = q
                    else: p = milestones[i-1]
                    if i+2 == len(milestones): nn = n
                    else: nn = milestones[i+2]
                    if isinstance(path,Trajectory):
                        v = vectorops.mul(path.difference_state(p,n,0.5,1.0),0.5)
                        a1 = vectorops.sub(path.difference_state(q,n,0.,1.),path.difference_state(p,q,1.,1.))
                        a2 = vectorops.sub(path.difference_state(n,nn,0.,1.),path.difference_state(q,n,1.,1.))
                    else:
                        v = vectorops.mul(vectorops.sub(n,p),0.5)
                        a1 = vectorops.madd(vectorops.add(p,n),q,-2.0)
                        a2 = vectorops.madd(vectorops.add(q,nn),n,-2.0)
                    if hasattr(vmax,'__iter__'):
                        for j,(x,lim) in enumerate(zip(v,vmax)):
                            if abs(x) > lim*_durations[i]:
                                _durations[i] = abs(x)/lim
                                #print("Segment",i,"limited on axis",j,"path velocity",x,"limit",lim)
                    else:
                        _durations[i] = vectorops.norm(v)/vmax
                    if hasattr(amax,'__iter__'):
                        if i > 0:
                            for j,(x,lim) in enumerate(zip(a1,amax)):
                                if abs(x) > lim*_durations[i]**2:
                                    _durations[i] = math.sqrt(abs(x)/lim)
                                    #print("Segment",i,"limited on axis",j,"path accel",x,"limit",lim)
                        if i+2 < len(milestones):
                            for j,(x,lim) in enumerate(zip(a2,amax)):
                                if abs(x) > lim*_durations[i]**2:
                                    _durations[i] = math.sqrt(abs(x)/lim)
                                    #print("Segment",i,"limited on axis",j,"outgoing path accel",x,"limit",lim)
                    else:
                        if i > 0:
                            n = vectorops.norm(a1)
                            if n > amax*_durations[i]**2:
                                _durations[i] = math.sqrt(n/amax)
                        if i+2 < len(milestones):
                            n = vectorops.norm(a2)
                            if n > amax*_durations[i]**2:
                                _durations[i] = math.sqrt(n/amax)
            else:
                durationfuncs = dict()
                durationfuncs['L2'] = vectorops.distance
                durationfuncs['Linf'] = lambda a,b:max(abs(u-v) for (u,v) in zip(a,b))
                durationfuncs['sqrt-L2'] = lambda a,b:math.sqrt(vectorops.distance(a,b))
                durationfuncs['sqrt-Linf'] = lambda a,b:math.sqrt(max(abs(u-v) for (u,v) in zip(a,b)))
                if hasattr(path,'robot'):
                    durationfuncs['robot'] = path.robot.distance
                    durationfuncs['sqrt-robot'] = lambda a,b:math.sqrt(path.robot.distance(a,b))
                assert timing in durationfuncs,"Invalid duration function specified, valid values are: "+", ".join(list(durationfuncs.keys()))
                timing = durationfuncs[timing]
                _durations = [timing(a,b) for a,b in zip(milestones[:-1],milestones[1:])]
    assert _durations is not None,"Hmm... didn't assign durations properly?"
    if verbose >= 1:
        print("path_to_trajectory(): Segment durations are",_durations)
    #by this time we have all milestones and durations
    if stoptol is not None:
        splits = [0]
        #split the trajectory then reassemble it
        for i in range(1,len(milestones)-1):
            prev = milestones[i-1]
            q = milestones[i]
            next = milestones[i+1]
            acc = vectorops.madd(vectorops.add(prev,next),q,-2.0)
            if vectorops.norm(acc) > stoptol*(_durations[i]*_durations[i-1]):
                splits.append(i)
        splits.append(len(milestones)-1)
        if len(splits) > 2:
            if verbose >= 1:
                print("path_to_trajectory(): Splitting path into",len(splits)-1,"segments, starting and stopping between")
            res = None
            for i in range(len(splits)-1):
                a,b = splits[i],splits[i+1]
                segmentspeed = (1.0 if isinstance(speed,(int,float)) else speed)
                traj = path_to_trajectory(milestones[a:b+1],velocities,timing,smoothing,
                    None,vmax,amax,
                    segmentspeed,dt)
                if res is None:
                    res = traj
                else:
                    if res.milestones[-1] != traj.milestones[0]: #may have hermite spline interpolation problems
                        res.times.append(res.times[-1])
                        res.milestones.append(traj.milestones[0])
                    res = res.concat(traj,relative=True)
            if isinstance(speed,(int,float)) and speed != 1.0:
                res.times = vectorops.mul(res.times,1.0/speed)
            return res
    #canonical case:
    #milestones and _durations are lists
    #start and stop at beginning / end
    #speed = 1 or 'limited'
    normalizedPath = Trajectory()
    if isinstance(path,RobotTrajectory):
        normalizedPath = RobotTrajectory(path.robot)
    normalizedPath.milestones = milestones
    normalizedPath.times = [0]
    totaldistance = 0
    for d in _durations:
        totaldistance += d
        normalizedPath.times.append(totaldistance)

    if startvel != 0.0 or endvel != 0.0:
        print("path_to_trajectory(): WARNING: respecting nonzero start/end velocity not implemented yet")

    if smoothing == 'spline':
        hpath = HermiteTrajectory()
        hpath.makeSpline(normalizedPath)
        normalizedPath = hpath

    #print("path_to_trajectory(): Total distance",totaldistance)
    if totaldistance == 0.0:
        return normalizedPath
    finalduration = totaldistance
    evmax = 1
    eamax = 0
    if velocities == 'auto':
        if timing == 'limited':
            velocities = 'constant'
        else:
            velocities = 'trapezoidal'
    if velocities == 'constant':
        easing = lambda t: t
        evmax = 1.0
        eamax = 0.0
    elif velocities == 'trapezoidal' or velocities == 'triangular':
        easing = lambda t: 2*t**2 if t < 0.5 else 1.0-(2*(1.0-t)**2)
        evmax = 2.0
        eamax = 2.0
        if velocities == 'trapezoidal' and timing != 'limited':
            #ramp up c t^2 until 0.25
            #velocity 2 c t, ending velocity c/2, ending point c/16
            #continue for 0.5, ending point c/16 + c/4
            #ramp down for distance c/16, total distance c/8 + c/4 = 1 => c = 8/3
            easing = lambda t: 8.0/3.0*t**2 if t < 0.25 else (1.0-(8.0/3.0*(1.0-t)**2) if t > 0.75 else 1.0/6.0 + 4.0/3.0*(t-0.25))
        finalduration = math.sqrt(totaldistance)
    elif velocities == 'cosine':
        easing = lambda t: 0.5*(1.0-math.cos(t*math.pi))
        evmax = math.pi*0.5  #pi/2 sin (t*pi)
        eamax = math.pi**2*0.5   #pi**2/2 cos(t*pi)
        finalduration = math.sqrt(totaldistance)
    elif velocities == 'parabolic':
        easing = lambda t: -2*t**3 + 3*t**2
        evmax = 1.5  #-6t*2 + 6t
        eamax = 6    #-12t + 6
        finalduration = math.sqrt(totaldistance)
    elif velocities == 'minimum-jerk':
        easing = lambda t: 10.0*t**3 - 15.0*t**4 + 6.0*t**5 
        evmax = 15*0.25   #30t^2 - 60t*3 + 30t^4 => 1/4*(30 - 30 + 30/4)= 30/8
        t = 1.0 + math.sqrt(1.0/3.0)
        eamax = 30*t - 45*t**2 + 15*t**3         #60t - 180t*2 + 120t^3 => max at 1/6 - t + t^2 = 0 => t = (1 +/- sqrt(1 - 4/6))/2 = 1/2 +/- 1/2 sqrt(1/3)
                                                 #30(1 + sqrt(1/3)) - 45(1 + sqrt(1/3))^2 + 15(1 + sqrt(1/3))^3 
        finalduration = math.pow(totaldistance,1.0/3.0)
    else:
        raise NotImplementedError("Can't do velocity profile "+velocities+" yet")
    if timing == 'limited':
        #print("Easing velocity max",evmax,"acceleration max",eamax)
        #print("Velocity and acceleration-limited segment distances",_durations)
        #print("Total distance traveled",totaldistance)
        finalduration = totaldistance*evmax
        #y(t) = p(L*e(t/T))
        #y'(t) = p'(L*e(t)/T)*e'(t) L/T 
        #y''(t) = p''(e(t))*e'(t)^2(L/T)^2 + p'(e(t))*e''(t) (L/T)^2
        #assume |p'(u)| <= vmax, |p''(u)| <= amax
        #set T so that |p'(u)| e'(t) L/T <= |p'(u)| evmax L/T  <= vmax evmax L/T <= vmax
        #set T so that |p''(u)| evmax^2 (L/T)^2 + |p'(u)|*e''(t) (L/T)^2 <= (amax evmax^2 + vmax eamax) (L/T)^2 <= amax
        #T >= L sqrt(evmax^2 + vmax/amax eamax)
        if finalduration < totaldistance*math.sqrt(evmax**2 + eamax):
            finalduration = totaldistance*math.sqrt(evmax**2 + eamax)
        if verbose >= 1:
            print("path_to_trajectory(): Setting first guess of path duration to",finalduration)
    res = normalizedPath.constructor()()
    if finalduration == 0:
        if verbose >= 1:
            print("path_to_trajectory(): there is no movement in the path, returning a 0-duration path")
        res.times = [0.0,0.0]
        res.milestones = [normalizedPath.milestones[0],normalizedPath.milestones[0]]
        return res
    N = int(math.ceil(finalduration/dt))
    assert N > 0
    dt = finalduration / N
    res.times=[0.0]*(N+1)
    res.milestones = [None]*(N+1)
    res.milestones[0] = normalizedPath.milestones[0][:]
    dt = finalduration/float(N)
    #print(velocities,"easing:")
    for i in range(1,N+1):
        res.times[i] = float(i)/float(N)*finalduration
        u = easing(float(i)/float(N))
        #print(float(i)/float(N),"->",u)
        res.milestones[i] = normalizedPath.eval_state(u*totaldistance)
    if timing == 'limited' or speed == 'limited':
        scaling = 0.0
        vscaling = 0.0
        aLimitingTime = 0
        vLimitingTime = 0
        for i in range(N):
            q,n = res.waypoint(res.milestones[i]),res.waypoint(res.milestones[i+1])
            if i == 0: p = q
            else: p = res.waypoint(res.milestones[i-1])
            if isinstance(path,Trajectory):
                v = path.difference_state(p,n,0.5,dt*2.0)
                a = vectorops.sub(path.difference_state(q,n,0.,dt),path.difference_state(p,q,1.,dt))
                a = vectorops.div(a,dt)
            else:
                v = vectorops.div(vectorops.sub(n,p),dt*2.0)    
                a = vectorops.div(vectorops.madd(vectorops.add(p,n),q,-2.0),dt**2)
            if not hasattr(vmax,'__iter__'):
                n = vectorops.norm(v)
                if n > vmax*scaling:
                    #print("path segment",i,"exceeded scaling",scaling,"by |velocity|",n,' > ',vmax*scaling)
                    vscaling = n/vmax
                    vLimitingTime = i
            else:
                for x,lim in zip(v,vmax):
                    if abs(x) > lim*vscaling:
                        #print("path segment",i,"exceeded scaling",scaling,"by velocity",x,' > ',lim*scaling)
                        #print("Velocity",v)
                        vscaling = abs(x)/lim
                        vLimitingTime = i
            if i == 0:
                continue
            if not hasattr(amax,'__iter__'):
                n = vectorops.norm(a)
                if n > amax*scaling**2:
                    #print("path segment",i,"exceeded scaling",scaling,"by |acceleration|",n,' > ',amax*scaling**2)
                    scaling = math.sqrt(n/amax)
                    aLimitingTime = i
            else:
                for x,lim in zip(a,amax):
                    if abs(x) > lim*scaling**2:
                        #print("path segment",i,"exceeded scaling",scaling,"by acceleration",x,' > ',lim*scaling**2)
                        #print(p,q,n)
                        #print("Velocity",v)
                        #print("Previous velocity",path.difference(p,q,1.,dt))
                        scaling = math.sqrt(abs(x)/lim)
                        aLimitingTime = i
        if verbose >= 1:
            print("path_to_trajectory(): Base traj exceeded velocity limit by factor of",vscaling,"at time",res.times[vLimitingTime]*max(scaling,vscaling))
            print("path_to_trajectory(): Base traj exceeded acceleration limit by factor of",scaling,"at time",res.times[aLimitingTime]*max(scaling,vscaling))
        if velocities == 'trapezoidal':
            #speed up until vscaling is hit
            if vscaling < scaling:
                if verbose >= 1:
                    print("path_to_trajectory(): Velocity maximum not hit")
            else:
                if verbose >= 1:
                    print("path_to_trajectory(): TODO: fiddle with velocity maximum.")
                scaling = max(vscaling,scaling)
                res.times = [t*scaling for t in res.times]
        else:
            scaling = max(vscaling,scaling)
        if verbose >= 1:
            print("path_to_trajectory(): Velocity / acceleration limiting yields a time expansion of",scaling)
        res.times = vectorops.mul(res.times,scaling)
    if isinstance(speed,(int,float)) and speed != 1.0:
        res.times = vectorops.mul(res.times,1.0/speed)
    return res


def execute_path(
        path: List[Vector],
        controller: Union['SimRobotController','RobotInterfaceBase'],
        speed: float = 1.0,
        smoothing: Optional[_SMOOTHING_OPTIONS] = None,
        activeDofs: Optional[List[Union[int,str]]] = None
    ):
    """Sends an untimed trajectory to a controller.

    If smoothing = None, the path will be executed as a sequence of go-to
    commands, starting and stopping at each milestone.  Otherwise, it will
    be smoothed somehow and sent to the controller as faithfully as possible.
    
    Args:
        path (list of Configs): a list of milestones

        controller (SimRobotController or RobotInterfaceBase): the controller
            to execute the path.

        speed (float, optional): if given, changes the execution speed of the
            path.  Not valid with smoothing=None or 'ramp'.

        smoothing (str, optional): any smoothing applied to the path.  Valid
            values are:

          - None: starts / stops at each milestone, moves in linear joint-space
            paths. Trapezoidal velocity profile used.  This is most useful for
            executing paths coming from a kinematic motion planner.
          - 'linear': interpolates milestones linearly with fixed duration.  
            Constant velocity profile used.
          - 'cubic': interpolates milestones with cubic spline with fixed 
            duration.  Parabolic velocity profile used.  Starts/stops at each 
            milestone.
          - 'spline': interpolates milestones smoothly with some differenced
            velocity.
          - 'ramp': starts / stops at each milestone, moves in minimum-time / 
            minimum-acceleration paths.  Trapezoidal velocity profile used.

        activeDofs (list, optional): if not None, a list of dofs that are moved
            by the trajectory. Each entry may be an integer or a string.
    """
    if len(path)==0: return  #be tolerant of empty paths?
    if speed <= 0: raise ValueError("Speed must be positive")
    from ..control.robotinterface import RobotInterfaceBase
    from ..robotsim import SimRobotController

    if isinstance(controller,SimRobotController):
        robot_model = controller.model()
        q0 = controller.getCommandedConfig()
    elif isinstance(controller,RobotInterfaceBase):
        robot_model = controller.klamptModel()
        cq0 = controller.commandedPosition()
        if cq0[0] is None:
            cq0 = controller.sensedPosition()
        q0 = controller.configFromKlampt(cq0)
    else:
        raise ValueError("Invalid type of controller, must be SimRobotController or RobotInterfaceBase")
    if activeDofs is not None:
        indices = [robot_model.link(d).getIndex for d in activeDofs]
        liftedMilestones = []
        for m in path:
            assert(len(m)==len(indices))
            q = q0[:]
            for i,v in zip(indices,m):
                q[i] = v
            liftedMilestones.append(q)
        return execute_path(liftedMilestones,controller,speed,smoothing)

    if smoothing == None:
        if isinstance(controller,SimRobotController):
            if speed != 1.0: raise ValueError("Can't specify speed with no smoothing")
            controller.setMilestone(path[0])
            for i in range(1,len(path)):
                controller.addMilestoneLinear(path[i])
        else:
            vmax = robot_model.getVelocityLimits()
            amax = robot_model.getAccelerationLimits()
            if speed != 1.0:
                vmax = vectorops.mul(vmax,speed)
                amax = vectorops.mul(amax,speed**2)
            htraj = HermiteTrajectory()
            if q0 != path[0]:
                mpath = [q0] + path
            else:
                mpath = path
            htraj.makeMinTimeSpline(mpath,vmax=vmax,amax=amax)
            return execute_trajectory(htraj,controller)
    elif smoothing == 'linear':
        dt = 1.0/speed
        if isinstance(controller,SimRobotController):
            controller.setLinear(dt,path[0])
            for i in range(1,len(path)):
                controller.addLinear(dt,path[i])
        else:
            traj = Trajectory()
            traj.times,traj.milestones = [0],[q0]
            for i in range(len(path)):
                if i==0 and q0 == path[i]: continue  #skip first milestone
                traj.times.append(traj.times[-1]+dt)
                traj.milestones.append(path[i])
            return execute_trajectory(traj,controller)
    elif smoothing == 'cubic':
        dt = 1.0/speed
        if isinstance(controller,SimRobotController):
            zero = [0.0]*len(path[0])
            controller.setCubic(dt,path[0],zero)
            for i in range(1,len(path)):
                controller.addCubic(dt,path[i],zero)
        else:
            zero = [0.0]*controller.numJoints()
            times,milestones = [0],[q0]
            for i in range(len(path)):
                if i==0 and q0 == path[i]: continue  #skip first milestone
                times.append(times[-1]+dt)
                milestones.append(path[i])
            htraj = HermiteTrajectory(times,milestones,[zero]*len(milestones))
            return execute_trajectory(htraj,controller)
    elif smoothing == 'spline':
        dt = 1.0/speed
        times = [0]
        mpath = [q0]
        for i in range(len(path)):
            if i==0 and path[0]==q0: continue
            times.append(times[-1]+dt)
            mpath.append(path[i])
        hpath = HermiteTrajectory()
        hpath.makeSpline(Trajectory(times,mpath))
        return execute_trajectory(hpath,controller)
    elif smoothing == 'ramp':
        if isinstance(controller,SimRobotController):
            if speed != 1.0: raise ValueError("Can't specify speed with ramp smoothing")
            controller.setMilestone(path[0])
            for i in range(1,len(path)):
                controller.addMilestone(path[i])
        else:
            cv0 = controller.commandedVelocity()
            if cv0[0] == None:
                cv0 = controller.sensedVelocity()
            v0 = controller.velocityFromKlampt(cv0)
            xmin,xmax = robot_model.getJointLimits()
            vmax = robot_model.getVelocityLimits()
            amax = robot_model.getAccelerationLimits()
            if speed != 1.0:
                vmax = vectorops.mul(vmax,speed)
                amax = vectorops.mul(amax,speed**2)
            zero = [0.0]*len(q0)
            if q0 != path[0]:
                mpath = [q0] + path
                mvels = [v0] + [zero]*len(path)
            else:
                mpath = path
                mvels = [v0] + [zero]*(len(path)-1)
            zero = [0.0]*len(q0)
            htraj = HermiteTrajectory()
            htraj.makeMinTimeSpline(mpath,mvels,xmin=xmin,xmax=xmax,vmax=vmax,amax=amax)
            return execute_trajectory(htraj,controller)
    else:
        raise ValueError("Invalid smoothing method specified")


def execute_trajectory(
        trajectory: Trajectory,
        controller: Union['SimRobotController','RobotInterfaceBase'],
        speed: float = 1.0,
        smoothing: Optional[_SMOOTHING_OPTIONS2] = None,
        activeDofs: Optional[List[Union[int,str]]] = None
    ):
    """Sends a timed trajectory to a controller.

    Args:
        trajectory (Trajectory): a Trajectory, RobotTrajectory, or
            HermiteTrajectory instance.
        controller (SimRobotController or RobotInterfaceBase): the controller
            to execute the trajectory.
        speed (float, optional): modulates the speed of the path.
        smoothing (str, optional): any smoothing applied to the path.  Only
            valid for piecewise linear trajectories.  Valid values are

            * None: no smoothing, just do a piecewise linear trajectory
            * 'spline': interpolate tangents to the curve
            * 'pause': smoothly speed up and slow down

        activeDofs (list, optional): if not None, a list of dofs that are moved
            by the trajectory.  Each entry may be an integer or a string.
    """
    if len(trajectory.times)==0: return  #be tolerant of empty paths?
    if speed <= 0: raise ValueError("Speed must be positive")
    from ..control.robotinterface import RobotInterfaceBase
    from ..robotsim import SimRobotController
    if isinstance(controller,SimRobotController):
        robot_model = controller.model()
        q0 = controller.getCommandedConfig()
    elif isinstance(controller,RobotInterfaceBase):
        robot_model = controller.klamptModel()
        cq0 = controller.commandedPosition()
        if cq0[0] is None:
            cq0 = controller.sensedPosition()
        q0 = controller.configToKlampt(cq0)
    else:
        raise ValueError("Invalid type of controller, must be SimRobotController or RobotInterfaceBase")
    if activeDofs is not None:
        indices = [robot_model.link(d).getIndex for d in activeDofs]
        liftedMilestones = []
        assert not isinstance(trajectory,HermiteTrajectory),"TODO: hermite trajectory lifting"
        for m in trajectory.milestones:
            assert(len(m)==len(indices))
            q = q0[:]
            for i,v in zip(indices,m):
                q[i] = v
            liftedMilestones.append(q)
        tfull = trajectory.constructor()(trajectory.times,liftedMilestones)
        return execute_trajectory(tfull,controller,speed,smoothing)

    if isinstance(trajectory,HermiteTrajectory):
        assert smoothing == None,"Smoothing cannot be applied to hermite trajectories"
        ts = trajectory.startTime()
        n = len(q0)
        if isinstance(controller,SimRobotController):
            controller.setMilestone(trajectory.eval(ts),vectorops.mul(trajectory.deriv(ts),speed))
            n = len(trajectory.milestones[0])//2
            for i in range(1,len(trajectory.times)):
                q,v = trajectory.milestones[i][:n],trajectory.milestones[i][n:]
                controller.addCubic(q,vectorops.mul(v,speed),(trajectory.times[i]-trajectory.times[i-1])/speed)
        else:
            cv0 = controller.commandedVelocity()
            if cv0[0] is None:
                cv0 = controller.sensedVelocity()
            times,positions,velocities = [0],[controller.configFromKlampt(q0)],[cv0]
            start = 1 if trajectory.times[0]==0 else 0
            for i in range(start,len(trajectory.milestones)):
                times.append(trajectory.times[i]/speed)
                positions.append(controller.configFromKlampt(trajectory.milestones[i][:n]))
                velocities.append(controller.velocityFromKlampt(trajectory.milestones[i][n:]))
            controller.setPiecewiseCubic(times,positions,velocities)
    else:
        if smoothing == None:
            ts = trajectory.startTime()
            if isinstance(controller,SimRobotController):
                controller.setMilestone(trajectory.eval(ts))
                for i in range(1,len(trajectory.times)):
                    q = trajectory.milestones[i]
                    controller.addLinear(q,(trajectory.times[i]-trajectory.times[i-1])/speed)
            else:
                #TODO: move to start?
                times,positions = [0],[controller.configFromKlampt(q0)]
                start = 1 if 0==trajectory.times[0] else 0
                for i in range(start,len(trajectory.milestones)):
                    times.append(trajectory.times[i]/speed)
                    positions.append(controller.configFromKlampt(trajectory.milestones[i]))
                controller.setPiecewiseLinear(times,positions)
        elif smoothing == 'spline':
            t = HermiteTrajectory()
            t.makeSpline(trajectory)
            return execute_trajectory(t,controller)
        elif smoothing == 'pause':
            if isinstance(controller,SimRobotController):
                ts = trajectory.startTime()
                controller.setMilestone(trajectory.eval(ts))
                zero = [0.0]*len(trajectory.milestones[0])
                for i in range(1,len(trajectory.times)):
                    q = trajectory.milestones[i]
                    controller.addCubic(q,zero,(trajectory.times[i]-trajectory.times[i-1])/speed)
            else:
                #TODO: move to start?
                zero = [.0]*len(q0)
                t = HermiteTrajectory(trajectory.times,trajectory.milestones,[zero]*len(trajectory.milestones))
                return execute_trajectory(t,controller)
        else:
            raise ValueError("Invalid smoothing method specified")
