"""Classes for loading, saving, evaluating, and operating on trajectories.

- For piecewise-linear interpolation in cartesian space, use Trajectory.
- For piecewise-linear interpolation on a robot, use RobotTrajectory.
- For Hermite interpolation in cartesian space, use HermiteTrajectory.
"""

import bisect
import vectorops

class Trajectory:
	"""A basic piecewise-linear trajectory class, which can be overloaded
	to provide different functionality.  A plain Trajectory interpolates
	in Cartesian space.

	(To interpolate for a robot, use RobotTrajectory. To perform
	Hermite interpolation, use HermiteTrajectory)

	Attributes:
		- times: a list of times at which the milestones are met.
		- milestones: a list of milestones that are interpolated.
        """
        
	def __init__(self,times=[],milestones=[]):
		self.times = times
		self.milestones = milestones

	def load(self,fn):
		"""Reads from a whitespace-separated file"""
		fin = open(fn, 'r')
		self.times = []
		self.milestones = []
		for line in fin.readlines():
			timedMilestone = [float(i) for i in line.strip().split()]
			self.times.append(timedMilestone[0])
			self.milestones.append(timedMilestone[2:])
		fin.close()

	def save(self,fn):
		"""Writes to a whitespace-separated file"""
		fout = open(fn, 'w')
		for t,x in zip(self.times,self.milestones):
			fout.write('%f\t%d '%(t,len(x)))
			fout.write(' '.join([str(xi) for xi in x]))
			fout.write('\n')
		fout.close()

	def checkValid(self):
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

	def getSegment(self,t,endBehavior='halt'):
		"""Returns the index and interpolation parameter for the
		segment at time t.  If endBehavior='loop' then the trajectory
		loops forver."""
		if len(self.times)==0:
			raise ValueError("Empty trajectory")
		if len(self.times)==1:
			return (-1,0)
		if t > self.times[-1]:
			if endBehavior == 'loop':
				t = t % self.times[-1]
			else:
				return (len(self.milestones),0)
		if t < self.times[0]:
			return (0,0)
		i = bisect.bisect_right(self.times,t)
		p=i-1
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
	
	def eval(self,t,endBehavior='halt'):
		"""Evaluates the trajectory using piecewise linear
		interpolation.  If endBehavior='loop' then the trajectory
		loops forver."""
		i,u = self.getSegment(t,endBehavior)
		if i<0: return self.milestones[0]
		elif i>=len(self.milestones): return self.milestones[-1]
		#linear interpolate between milestones[i] and milestones[i+1]
		return self.interpolate(self.milestones[i],self.milestones[i+1],u)

	def deriv(self,t,endBehavior='halt'):
		"""Evaluates the trajectory velocity using piecewise linear
		interpolation.  If endBehavior='loop' then the trajectory
		loops forver."""
		i,u = self.getSegment(t,endBehavior)
		if i<0: return [0.0]*len(self.milestones[0])
		elif i>=len(self.milestones): return [0.0]*len(self.milestones[-1])
		return vectorops.mul(self.difference(self.milestones[i+1],self.milestones[i]),1.0/(self.times[i+1]-self.times[i]))

	def interpolate(self,a,b,u):
		"""Can override this to implement non-cartesian spaces.
		Interpolates along the geodesic from a to b."""
		return vectorops.interpolate(a,b,u)
	
	def difference(self,a,b):
		"""Subclasses can override this to implement non-Cartesian
		spaces.  Returns the derivative along the geodesic from b to
		a."""
		return vectorops.sub(a,b)
	
	def concat(self,suffix,relative=False,jumpPolicy='strict'):
		"""Returns a new trajectory with another trajectory
		concatenated onto self.
		If relative=True, then the suffix's time domain is shifted
		so that self.times[-1] is added on before concatenation.

		If the suffix starts exactly at the existing trajectory's
		end time, then jumpPolicy is checked.
		- If jumpPolicy='strict', then the suffix's first milestone
		  has to be equal to the existing trajectory's last milestone.
		  Otherwise an exception is raised.
	        - If jumpPolicy='blend', then the existing trajectory's last
		  milestone is discarded.
	        - If jumpPolicy='jump', then a discontinuity is added into
		  the trajectory.
		"""
		if not relative or len(self.times)==0:
			offset = 0
		else:
			offset = self.times[-1]
		if len(self.times)!=0:
			if suffix.times[0]+offset < self.times[-1]:
				raise ValueError("Invalid concatenation")
			if suffix.times[0]+offset == self.times[-1]:
				#keyframe exactly equal; skip the first milestone
				#check equality with last milestone
				if jumpPolicy=='strict' and suffix.milestones[0] != self.milestones[-1]:
					raise ValueError("Concatenation would cause a jump in configuration")
				if jumpPolicy=='strict' or (jumpPolicy=='blend' and suffix.milestones[0] != self.milestones[-1]):
					#discard last milestone of self
					times = self.times[:-1] + [t+offset for t in suffix.times]
					milestones = self.milestones[:-1] + suffix.milestones
					return Trajectory(times,milestones)
		times = self.times + [t+offset for t in suffix.times]
		milestones = self.milestones + suffix.milestones
		return Trajectory(times,milestones)

	def split(self,time):
		"""Returns two trajectories split at the given time"""
		i,u = self.getSegment(time)
		if time < 0:
			return Trajectory(),Trajectory()
		if time <= self.times[0]:
			#split before start of trajectory
			return Trajectory([time],[self.milestones[0]]),Trajectory([time]+self.times,[self.milestones[0]]+self.milestones)
		if time >= self.times[-1]:
			#split after end of trajectory
			return Trajectory(self.times+[time],self.milestones+[self.milestones[-1]]),trajectory([time],self.milestones[-1])
		#split in middle of trajectory
		splitpt = self.interpolate(self.milestones[i],self.milestones[i+1],u)
		front = Trajectory(self.times[:i+1],self.milestones[:i+1])
		back = Trajectory(self.times[i+1:],self.milestones[i+1:])
		if u > 0:
			front.times.append(time)
			front.milestones.append(splitpt)
		if u < 1:
			back.times = [time] + back.times
			back.milestones = [splitpt] + back.milestones
		return (front,back)
	def before(self,time):
		"""Returns the part of the trajectory before the given time"""
		return self.split(time)[0]
	def after(self,time):
		"""Returns the part of the trajectory after the given time"""
		return self.split(time)[1]
	def splice(self,suffix,time=None,relative=False,jumpPolicy='strict'):
		"""Returns a path such that the suffix is spliced in
		at the suffix's start time (if time=None) or the given time
		if specified.
		
		If jumpPolicy='strict', then it is required that
		suffix(t0)=path(t0) where t0 is the absolute start time
		of the suffix."""
		offset = 0
		if time==None:
			time = suffix.times[0]
		if relative and len(self.times) > 0:
			offset = self.times[-1]
		time = time+offset
		before = self.before(time)
		return before.concat(suffix,relative,jumpPolicy)

class RobotTrajectory(Trajectory):
	"""A trajectory that performs interpolation according to the robot's
	interpolation scheme."""
	def __init__(self,robot,times=[],milestones=[]):
		Trajectory.__init__(self,times,milestones)
		self.robot = robot
	def interpolate(self,a,b,u):
		return self.robot.interpolate(a,b,u)
	def difference(self,a,b):
		return self.robot.interpolate_deriv(b,a)

class HermiteTrajectory(Trajectory):
	"""A trajectory whose milestones are given in phase space (x,dx).
	"""
	def __init__(self,times=[],milestones=[],dmilestones=None):
		"""If dmilestones is given, then milestones is interpreted
		as configurations and dmilestones is interpreted as velocities.
		
		Otherwise, the milestones are interpreted as states (x,dx)
		"""
		if dmilestones==None:
			Trajectory.__init__(self,times,milestones)
		else:
			#interpret as config/velocity
			self.times = times
			self.milestones = [q+dq for (q,dq) in zip(milestones,dmilestones)]
	def makeSpline(self,waypointTrajectory):
		"""Computes natural velocities for a standard configuration-
		space Trajectory to make it smoother."""
		velocities = []
		t = waypointTrajectory
		d = len(t.milestones[0])
		if len(t.milestones)==1:
			velocities.append([0]*d)
		elif len(t.milestones)==2:
			v = vectorops.mul(vectorops.sub(t.milestones[1],t.milestones[0]),1.0/(t.times[1]-t.times[0]))
			velocities.append(v)
			velocities.append(v)
		else :
			for i in range(1,len(waypointTrajectory.milestones)-1):
				v = vectorops.mul(vectorops.sub(t.milestones[i+1],t.milestones[i]),1.0/(t.times[i+1]-t.times[i-1]))
				velocities.append(v)
			#start velocity as quadratic
			x2 = vectorops.madd(t.milestones[1],velocities[0],-1.0/3.0)
			x1 = vectorops.madd(x2,vectorops.sub(t.milestones[1],t.milestones[0],-1.0/3.0))
			v0 = vectorops.mul(vectorops.sub(x1,t.milestones[0]),3.0)
			#terminal velocity as quadratic
			xn_2 = vectorops.madd(t.milestones[-2],velocities[-1],1.0/3.0)
			xn_1 = vectorops.madd(xn_2,vectorops.sub(t.milestones[-1],t.milestones[-2],1.0/3.0))
			vn = vectorops.mul(vectorops.sub(t.milestones[-1],xn_1),3.0)
			velocities = [v0]+velocities+[vn]
		self.__init__(waypointTrajectory.times[:],waypointTrajectory.milestones,velocities)

	def eval_config(self,t,endBehavior='halt'):
		"""Returns just the configuration component of the result"""
		res = self.eval(t,endBehavior)
		return res[:len(res)/2]
	
	def eval_velocity(self,t,endBehavior='halt'):
		"""Returns just the velocity component of the result"""
		res = self.eval(t,endBehavior)
		return res[len(res)/2:]

	def eval_accel(self,t,endBehavior='halt'):
		"""Returns just the acceleration component of the result"""
		res = self.deriv(t,endBehavior)
		return res[len(res)/2:]

	def interpolate(self,a,b,u):
		x1,v1 = a[:len(a)/2],a[len(a)/2:]
		x2,v2 = b[:len(b)/2],b[len(b)/2:]
		assert len(a)==len(b)
		assert len(x1)==len(v1)
		u2 = u*u
		u3 = u*u*u
		cx1 = 2.0*u3-3.0*u2+1.0
		cx2 = -2.0*u3+3.0*u2
		cv1 = (u3-2.0*u2+u)
		cv2 = (u3-u2)
		dcx1 = (6.0*u2-6.0*u)
		dcx2 = (-6.0*u2+6.0*u)
		dcv1 = 3.0*u2-4.0*u+1.0
		dcv2 = 3.0*u2-2.0*u
		x = [0]*len(x1)
		dx = [0]*len(x1)
		for i in xrange(len(x1)):
			x[i] = cx1*x1[i] + cx2*x2[i] + cv1*v1[i] + cv2*v2[i]
			dx[i] = dcx1*x1[i] + dcx2*x2[i] + dcv1*v1[i] + dcv2*v2[i];
		return x+dx
	
	def difference(self,a,b):
		x1,v1 = a[:len(a)/2],a[len(a)/2:]
		x2,v2 = b[:len(b)/2],b[len(b)/2:]
		assert len(a)==len(b)
		assert len(x1)==len(v1)
		u2 = u*u
		dcx1 = (6.0*u2-6.0*u)
		dcx2 = (-6.0*u2+6.0*u)
		dcv1 = 3.0*u2-4.0*u+1.0
		dcv2 = 3.0*u2-2.0*u
		ddcx1 = 12*u
		ddcx2 = -12.0*u
		ddcv1 = 6.0*u-4.0
		ddcv2 = 6.0*u-2.0
		dx = [0]*len(x1)
		ddx = [0]*len(x1)
		for i in xrange(len(x1)):
			dx[i] = dcx1*x1[i] + dcx2*x2[i] + dcv1*v1[i] + dcv2*v2[i]
			ddx[i] = ddcx1*x1[i] + ddcx2*x2[i] + ddcv1*v1[i] + ddcv2*v2[i]
		return dx+ddx

