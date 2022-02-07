try:
    #version 0.9+
    from klampt.plan.motionplanning import interpolate_nd_min_time,brake_nd
    from klampt.plan.motionplanning import combine_nd_cubic
except ImportError:
    #version 0.8.x
    from klampt.plan.motionplanning import interpolateNDMinTime as interpolate_nd_min_time
    from klampt.plan.motionplanning import combineNDCubic as combine_nd_cubic
    brake_nd = None
from klampt.math.spline import hermite_eval,hermite_deriv
from klampt.math import vectorops
from klampt.model.trajectory import Trajectory,HermiteTrajectory
import math
import warnings

class VelocityBoundedMotionGeneration:
    """A motion generator that limits the element-wise velocities for
    continuously generated position targets.  Note that the target
    is reached simultaneously on each dimension.

    Usage::

        from klampt.control.utils import TimedLooper

        x0 = robot.getPosition()
        generator = VelocityBoundedMotionGeneration(x0,vmax)
        dt = 1.0/rate  #some control rate
        looper = TimedLooper(dt)
        while looper:
            #TODO: send commands, e.g., generator.setTarget(xtgt)
            (x,v) = generator.update(dt)
            robot.setPosition(x)

    Args:
        x0 (list of floats): the start position
        vmax (list of floats, optional): the velocity limits
    
    """
    def __init__(self,x0,vmax):
        if len(x0) != len(vmax):
            raise ValueError("Invalid length of vmax")
        n = len(x0)
        self.x = [v for v in x0]
        self.v = [0]*n
        self.vmax = vmax
        self.times = [0]
        self.milestones = [self.x]
        self.curTime = 0
        self.trajTime = 0
    
    def remainingTime(self):
        """Returns the remaining time to finish the path, in s.  Returns 0
        if the path is done executing.
        """
        return max(0,self.times[-1]-self.curTime)
    
    def reset(self,x0):
        """Resets the motion generator to the start position x0."""
        self.x = x0
        self.v = [0]*len(x0)
        self.times = [0]
        self.milestones = [self.x]
        self.curTime = 0
        self.trajTime = 0

    def update(self,dt,xtarget=None):
        """Updates the motion generator.  Optionally sets the new target.

        Returns:
            tuple: (x,v) giving the new state
        """
        if dt <= 0:
            raise ValueError("Invalid dt")
        if xtarget is not None:
            self.setTarget(xtarget)

        self.curTime += dt
        self.trajTime += dt
        xnew,vnew = self.predict(0)
        self.x = xnew
        self.v = vnew
        return xnew,vnew
    
    def setTarget(self,xtarget,append=False):
        """Sets a new target.  If append=True, appends the target
        to the motion queue.
        """
        assert len(xtarget) == len(self.x)
        self._trim(append)

        xlast = self.milestones[-1]
        tmax = 0
        for (x,xt,vmax) in zip(xlast,xtarget,self.vmax):
            dx = xt-x
            tmax = max(tmax, abs(dx) / vmax)
        self.times.append(self.times[-1]+tmax)
        self.milestones.append(xtarget)
    
    def setVelocity(self,vtarget,duration=1,append=False):
        """Sets a velocity command to vtarget.  Moves along this
        speed for duration seconds and then stops.
        """
        assert len(vtarget) == len(self.x)
        assert duration >= 0
        self._trim(append)
        xlast = self.milestones[-1]
        self.times.append(self.times[-1]+duration)
        self.milestones.append(vectorops.madd(xlast,vtarget,duration))
    
    def brake(self):
        """Stops as quickly as possible. Since acceleration is unbounded,
        this just stops immediately."""
        self.setTarget(self.x)

    def _trim(self,append):
        newtimes = [0]
        newmilestones = [self.x]
        if append:
            #cut prior trajectory to trajTime    
            for j,t in enumerate(self.times):
                if t > self.trajTime:
                    newtimes.append(t-self.trajTime)
                    newmilestones.append(self.milestones[j])
        else:
            #just reset prior trajectory
            pass
        self.times = newtimes
        self.milestones = newmilestones
        self.trajTime = 0
    
    def duration(self):
        """Returns the time remaining in the trajectory"""
        return max(self.times[-1]-self.trajTime,0)
    
    def target(self):
        """Returns the final position on the trajectory"""
        return self.milestones[-1]

    def predict(self,t):
        """Predicts the state that the system will be in at time t>=0 in the
        future, assuming no changes to the target.
        
        Returns:
            tuple: (x,v) giving the new state
        
        """
        t = t + self.trajTime
        if t < self.times[0]:
            return self.x,self.v
        j = 0
        while j+1 < len(self.times): 
            if t < self.times[j+1]:
                u = (t - self.times[j])/(self.times[j+1]-self.times[j])
                speed = 1.0/(self.times[j+1]-self.times[j])
                x = vectorops.interpolate(self.milestones[j],self.milestones[j+1],u)
                v = vectorops.mul(vectorops.sub(self.milestones[j+1],self.milestones[j],speed))
                return x,v
            j += 1
        return self.milestones[-1],[0]*len(self.x)
    
    def trajectory(self):
        """Returns the future trajectory as a Trajectory.
        """
        times = [0]
        milestones = [self.x]
        for j in range(len(self.times)):
            if self.trajTime < self.times[j]:
                times.append(self.times[j]-self.trajTime)
                milestones.append(self.milestones[j])
        return Trajectory(times,milestones)


class AccelerationBoundedMotionGeneration:
    """A motion generator similar to the Reflexxes library, which
    provides acceleration- and velocity-bounded smooth trajectories
    for arbitrary targets.

    Usage::

        from klampt.control.utils import TimedLooper

        x0 = robot.getPosition()
        generator = AccelerationBoundedMotionGeneration(x0,vmax)
        dt = 1.0/rate  #some control rate
        looper = TimedLooper(dt)
        while looper:
            #TODO: send commands, e.g., generator.setTarget(xtgt)
            (x,v) = generator.update(dt)
            robot.setPosition(x)

    Args:
        x0 (list of floats): the start position
        xmin (list of floats, optional): the lower position joint limits
        xmax (list of floats, optional): the upper position joint limits
        vmax (list of floats, optional): the velocity limits
        amax (list of floats): the acceleration limits.  Non-optional (for now.)
    
    """
    def __init__(self,x0,xmin=None,xmax=None,vmax=None,amax=None):
        if len(x0) != len(vmax):
            raise ValueError("Invalid length of vmax")
        if len(x0) != len(amax):
            raise ValueError("Invalid length of amax")
        n = len(x0)
        self.x = [v for v in x0]
        self.v = [0]*n
        self.xmin = xmin if xmin is not None else [-float('inf')]*n
        self.xmax = xmax if xmax is not None else [float('inf')]*n
        self.vmax = vmax if vmax is not None else [float('inf')]*n
        self.amax = amax if amax is not None else [float('inf')]*n
        if amax is None:
            raise ValueError("amax needs to be specified")
        self.times = [[0] for v in x0]
        self.milestones = [[v] for v in x0]
        self.dmilestones = [[0] for v in x0]
        self.trajTime = 0
        self.curTime = 0
    
    def remainingTime(self):
        """Returns the remaining time to finish the path, in s.  Returns 0
        if the path is done executing.
        """
        return max(0,max(t[-1]-self.trajTime for t in self.times))

    def reset(self,x0):
        """Resets the motion generator to the start position x0."""
        if len(x0) != len(self.amax):
            raise ValueError("Invalid length of the configuration")
        self.x = x0
        self.v = [0]*len(x0)
        self.times = [[0] for v in x0]
        self.milestones = [[v] for v in x0]
        self.dmilestones = [[0] for v in x0]
        self.trajTime = 0

    def update(self,dt,xtarget=None,vtarget=None):
        """Updates the motion generator.  Optionally sets the new target and velocity.
        If velocity is None, ends in 0 velocity.

        Returns:
            tuple: (x,v) giving the new state
        """
        if dt <= 0:
            raise ValueError("Invalid dt")
        if xtarget is not None:
            self.setTarget(xtarget,vtarget)

        self.trajTime += dt
        self.curTime += dt
        x,v = self.predict(0)
        self.x = x
        self.v = v
        return x,v

    def setTarget(self,xtarget,vtarget=None,append=False):
        """Sets a new target position xtarget and optional velocity vtarget.
        
        If append=True, appends the target to the motion queue.
        """
        assert len(xtarget) == len(self.x)
        if vtarget is None:
            vtarget = [0]*len(xtarget)
        assert len(vtarget) == len(self.x)
        self._trim(append)

        t,m,v = interpolate_nd_min_time(self.x,self.v,xtarget,vtarget,
            self.xmin,self.xmax,self.vmax,self.amax)
        if len(t)==0:
            if self.x != xtarget or self.v != vtarget:
                warnings.warn("Cannot solve for path from {}, {} to target {}, {}".format(self.x,self.v,xtarget,vtarget))
        else:
            assert len(t) == len(m)
            assert len(t) == len(v)
            assert len(t) == len(self.x)
            for i,vi in enumerate(v):
                for j,vj in enumerate(vi):
                    if abs(vj) > self.vmax[i]:
                        vj = min(max(-self.vmax[i],vj),self.vmax[i])
                        if abs(vj) > self.vmax[i]*1.001:
                            warnings.warn("Solved velocity {} is larger than vmax {}".format(vi,self.vmax[i]))
                        vi[j] = vj
            for i in range(len(self.x)):    
                self.times[i] += [x+self.trajTime for x in t[i][1:]]
                self.milestones[i] += m[i][1:]
                self.dmilestones[i] += v[i][1:]
        self._checkValid()

    def setVelocity(self,vtarget,duration=1,append=False):
        """Sets a velocity command to vtarget.  Moves along this
        speed for duration seconds and then stops.
        """
        assert len(vtarget)==len(self.x)
        assert duration >= 0
        self._trim(append)
        self._append_ramp_velocity(vtarget)
        #go straight for a duration
        for i in range(len(self.x)):
            self.times[i].append(self.times[i][-1]+duration)
            self.milestones[i].append(self.milestones[-1][i]+vtarget[i]*duration)
            self.dmilestones[i].append(vtarget)
        #ramp down
        self._append_ramp_velocity([0]*len(self.x))
        self._checkValid()
    
    def brake(self):
        """Stops as quickly as possible under the acceleration bounds.
        """
        self._trim(False)

        if brake_nd is None:
            #0.8.x Klampt
            raise NotImplementedError("brake() not available in Klampt 0.8.x")
        
        t,m,v = brake_nd(self.x,self.v,self.xmin,self.xmax,self.amax)
        for i in range(len(self.x)):
            assert t[i][0] == 0
            self.times[i] += [x+self.trajTime for x in t[i][1:]]
            self.milestones[i] += m[i][1:]
            self.dmilestones[i] += v[i][1:]
        self._checkValid()

    def _append_ramp_velocity(self,vtarget):
        vlast = self.dmilestones[-1]
        #ramp up to vtarget
        tmax = 0
        for i in range(len(self.x)):
            dv = vtarget[i] - vlast[i]
            tmax = max(tmax,abs(dv/self.amax[i]))
        if tmax > 0:
            #ramp up to tmax with parabolic curve
            for i in range(len(self.x)):
                self.times[i].append(self.times[i][-1]+tmax)
                dv = vtarget[i] - vlast[i]
                a = dv / tmax
                self.milestones[i].append(self.milestones[-1][i]+self.dmilestones[-1][i]*tmax + a*0.5*tmax**2)
                self.dmilestones[i].append(vtarget[i])
        self._checkValid()

    def _trim(self,append):
        newtimes = [[0] for v in self.x]
        newmilestones = [[v] for v in self.x]
        newdmilestones = [[v] for v in self.v]

        if append:
            #cut prior trajectory to trajTime
            for i in range(len(self.x)):
                for j,t in enumerate(self.times[i]):
                    if t > self.trajTime:
                        newtimes[i].append(t-self.trajTime)
                        newmilestones[i].append(self.milestones[i][j])
                        newdmilestones[i].append(self.dmilestones[i][j])
        else:
            #reset prior path
            pass
        self.times = newtimes
        self.milestones = newmilestones
        self.dmilestones = newdmilestones
        self.trajTime = 0
        self._checkValid()
    
    def duration(self):
        """Returns the time remaining in the trajectory"""
        return max(self.times[0][-1]-self.trajTime,0)
    
    def target(self):
        """Returns the final position on the trajectory"""
        return [mi[-1] for mi in self.milestones]

    def predict(self,t):
        """Predicts the state that the system will be in at time t>=0 in the
        future, assuming no changes to the target.

        Args:
            t (float): the time in the future. Should be >=0.

        Returns:
            tuple: (x,v) giving the predicted state

        """
        t = t + self.trajTime
        x = []
        v = []
        for i in range(len(self.times)):
            ti,mi,dmi = self.times[i],self.milestones[i],self.dmilestones[i]
            #evaluate trajectory
            j = 0
            xi = mi[-1]
            vi = dmi[-1]
            if t < ti[-1]:
                while j+1 < len(ti):
                    if t < ti[j+1]:
                        assert t >= ti[j]
                        dt = (ti[j+1]-ti[j])
                        u = (t-ti[j])/dt
                        xi = hermite_eval([mi[j]],[dmi[j]*dt],[mi[j+1]],[dmi[j+1]*dt],u)[0]
                        vi = hermite_deriv([mi[j]],[dmi[j]*dt],[mi[j+1]],[dmi[j+1]*dt],u)[0]/dt
                        if abs(vi) > self.vmax[i]:
                            if abs(vi) > self.vmax[i]*1.001:
                                warnings.warn("{} {} -> {} {} at u={}, dt={}".format(mi[j],dmi[j],mi[j+1],dmi[j+1],u,(ti[j+1]-ti[j])))
                                warnings.warn("Evaluated velocity {} is larger than vmax {}".format(vi,self.vmax[i]))
                            vi = min(max(-self.vmax[i],vi),self.vmax[i])
                        break
                    j += 1
            x.append(xi)
            v.append(vi)
        return x,v

    def trajectory(self):
        """Returns the future trajectory as a HermiteTrajectory.
        """
        self._checkValid()
        times,milestones,dmilestones = combine_nd_cubic(self.times,self.milestones,self.dmilestones)
        prefix,suffix = HermiteTrajectory(times,milestones,dmilestones).split(self.trajTime)
        suffix.times = [t-self.trajTime for t in suffix.times]
        return suffix

    def _checkValid(self):
        assert len(self.x) == len(self.v)
        assert len(self.x) == len(self.times)
        assert len(self.times) == len(self.milestones)
        assert len(self.times) == len(self.dmilestones)
        for i in range(len(self.times)):
            assert len(self.times[i]) == len(self.milestones[i])
            assert len(self.times[i]) == len(self.dmilestones[i])
            for j in range(len(self.times[i])-1):
                assert self.times[i][j+1] >= self.times[i][j]