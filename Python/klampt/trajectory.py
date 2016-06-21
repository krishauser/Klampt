"""Classes for loading, saving, evaluating, and operating on trajectories.

- For piecewise-linear interpolation in cartesian space, use Trajectory.
- For piecewise-linear interpolation on a robot, use RobotTrajectory.
- For Hermite interpolation in cartesian space, use HermiteTrajectory.
"""

import bisect
import so3,se3,vectorops
import spline
from geodesic import *

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

	def startTime(self):
		"""Returns the initial time."""
		try: return self.times[0]
		except IndexError: return 0.0
		
	def endTime(self):
		"""Returns the final time."""
		try: return self.times[-1]
		except IndexError: return 0.0

	def duration(self):
		"""Returns the duration of the trajectory."""
		return self.endTime()-self.startTime()

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
		loops forver.  O(log n) time where n is the number of
		segments."""
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
		return vectorops.mul(self.difference(self.milestones[i+1],self.milestones[i],u),1.0/(self.times[i+1]-self.times[i]))

	def interpolate(self,a,b,u):
		"""Can override this to implement non-cartesian spaces.
		Interpolates along the geodesic from a to b."""
		return vectorops.interpolate(a,b,u)
	
	def difference(self,a,b,u):
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
					return self.constructor()(times,milestones)
		times = self.times + [t+offset for t in suffix.times]
		milestones = self.milestones + suffix.milestones
		return self.constructor()(times,milestones)

	def split(self,time):
		"""Returns a pair of trajectories obtained from splitting this
		one at the given time"""
		i,u = self.getSegment(time)
		if time < 0:
			return self.constructor()(),self.constructor()()
		if time <= self.times[0]:
			#split before start of trajectory
			return self.constructor()([time],[self.milestones[0]]),self.constructor()([time]+self.times,[self.milestones[0]]+self.milestones)
		if time >= self.times[-1]:
			#split after end of trajectory
			return self.constructor()(self.times+[time],self.milestones+[self.milestones[-1]]),trajectory([time],self.milestones[-1])
		#split in middle of trajectory
		splitpt = self.interpolate(self.milestones[i],self.milestones[i+1],u)
		front = self.constructor()(self.times[:i+1],self.milestones[:i+1])
		back = self.constructor()(self.times[i+1:],self.milestones[i+1:])
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
		if time is None:
			time = suffix.times[0]
		if relative and len(self.times) > 0:
			offset = self.times[-1]
		time = time+offset
		before = self.before(time)
		return before.concat(suffix,relative,jumpPolicy)
	def constructor(self):
		"""Returns a "standard" constructor for the split / concat
		routines.  The result should be a function that takes two
		arguments: a list of times and a list of milestones."""
		return Trajectory

class RobotTrajectory(Trajectory):
	"""A trajectory that performs interpolation according to the robot's
	interpolation scheme."""
	def __init__(self,robot,times=[],milestones=[]):
		Trajectory.__init__(self,times,milestones)
		self.robot = robot
	def interpolate(self,a,b,u):
		return self.robot.interpolate(a,b,u)
	def difference(self,a,b,u):
		return self.robot.interpolate_deriv(b,a)
	def constructor(self):
		return lambda time,milestones: RobotTrajectory(self.robot,time,milestones)

class GeodesicTrajectory(Trajectory):
	"""A trajectory that performs interpolation on a GeodesicSpace.
	See klampt.geodesic for more information."""
	def __init__(self,geodesic,times=[],milestones=[]):
		self.geodesic = geodesic
		Trajectory.__init__(self,times,milestones)
	def interpolate(self,a,b,u):
		return self.geodesic.interpolate(a,b,u)
	def difference(self,a,b,u):
		if u > 0.5:
			#do this from the b side
			return vectorops.mul(self.difference(b,a,1.0-u),-1.0)
		x = self.interpolate(a,b,u)
		return vectorops.mul(self.geodesic.difference(x,b),1.0/(1.0-u))
	def constructor(self):
		return lambda times,milestones:GeodesicTrajectory(self.geodesic,times,milestones)

class SO3Trajectory(GeodesicTrajectory):
	"""A trajectory that performs interpolation in SO3.  Each milestone
	is a 9-D klampt.so3 element."""
	def __init__(self,times=[],milestones=[]):
		GeodesicTrajectory.__init__(self,SO3Space(),times,milestones)
	def preTransform(self,R):
		"""Premultiplies every rotation in here by the so3 element
		R. In other words, if R rotates a local frame F to frame F',
		this method converts this SO3Trajectory from coordinates in F
		to coordinates in F'"""
		for i,m in enumerate(self.milestones):
			self.milestones[i] = so3.mul(R,m)
	def postTransform(self,R):
		"""Postmultiplies every rotation in here by the se3 element
		R. In other words, if R rotates a local frame F to frame F',
		this method converts this SO3Trajectory from describing how F'
		rotates to how F rotates."""
		for i,m in enumerate(self.milestones):
			self.milestones[i] = se3.mul(m,T)
	def constructor(self):
		return SO3Trajectory

class SE3Trajectory(GeodesicTrajectory):
	"""A trajectory that performs interpolation in SE3.  Each milestone
	is a 12-D flattened klampt.se3 element (i.e., the concatenation of
	R + t for an (R,t) pair)."""
	def __init__(self,times=[],milestones=[]):
		"""Constructor can take either a list of SE3 elements or
		12-element vectors."""
		if len(milestones) > 0 and len(milestones[0])==2:
			GeodesicTrajectory.__init__(self,SE3Space(),times,[m[0]+m[1] for m in milestones])
		else:
			GeodesicTrajectory.__init__(self,SE3Space(),times,milestones)
	def eval_se3(self,t,endBehavior='halt'):
		"""Returns an SE3 element"""
		res = self.eval(t,endBehavior)
		return (res[:9],res[9:])
	def deriv_se3(self,t,endBehavior='halt'):
		"""Returns the derivative as the derivatives of an SE3
		element"""
		res = self.deriv(t,endBehavior)
		return (res[:9],res[9:])
	def preTransform(self,T):
		"""Premultiplies every transform in here by the se3 element
		T. In other words, if T transforms a local frame F to frame F',
		this method converts this SE3Trajectory from coordinates in F
		to coordinates in F'"""
		for i,m in enumerate(self.milestones):
			Tm = (m[:9],m[9:])
			self.milestones[i] = se3.mul(T,Tm)
	def postTransform(self,T):
		"""Postmultiplies every transform in here by the se3 element
		T. In other words, if T transforms a local frame F to frame F',
		this method converts this SE3Trajectory from describing how F'
		moves to how F moves."""
		for i,m in enumerate(self.milestones):
			Tm = (m[:9],m[9:])
			self.milestones[i] = se3.mul(Tm,T)
	def getRotationTrajectory(self):
		"""Returns an SO3Trajectory describing the rotation
		trajectory."""
		return SO3Trajectory(times,[m[:9] for m in self.milestones])
	def getPositionTrajectory(self,localPt=None):
		"""Returns a Trajectory describing the movement of the given
		local point localPt (or the origin, if none is provided)."""
		if localPt is None:
			return Trajectory(times,[m[9:] for m in self.milestones])
		else:
			return Trajectory(times,[se3.apply((m[:9],m[9:]),localPt) for m in self.milestones])
	def constructor(self):
		return SE3Trajectory

class HermiteTrajectory(Trajectory):
	"""A trajectory whose milestones are given in phase space (x,dx).
	
	eval(t) is overloaded to just get the configuration, and deriv(t) is
	overloaded to just get the velocity.  To obtain state, use
	eval_state(t).  To get acceleration, use eval_accel(t).
	"""
	def __init__(self,times=[],milestones=[],dmilestones=None):
		"""If dmilestones is given, then milestones is interpreted
		as configurations and dmilestones is interpreted as velocities.
		
		Otherwise, the milestones are interpreted as states (x,dx)
		"""
		if dmilestones is None:
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
			x1 = vectorops.madd(x2,vectorops.sub(t.milestones[1],t.milestones[0]),-1.0/3.0)
			v0 = vectorops.mul(vectorops.sub(x1,t.milestones[0]),3.0)
			#terminal velocity as quadratic
			xn_2 = vectorops.madd(t.milestones[-2],velocities[-1],1.0/3.0)
			xn_1 = vectorops.madd(xn_2,vectorops.sub(t.milestones[-1],t.milestones[-2]),1.0/3.0)
			vn = vectorops.mul(vectorops.sub(t.milestones[-1],xn_1),3.0)
			velocities = [v0]+velocities+[vn]
		self.__init__(waypointTrajectory.times[:],waypointTrajectory.milestones,velocities)

	def eval(self,t,endBehavior='halt'):
		"""Returns just the configuration component of the result"""
		res = Trajectory.eval(self,t,endBehavior)
		return res[:len(res)/2]

	def deriv(self,t,endBehavior='halt'):
		"""Returns the derivative of configuration"""
		res = Trajectory.eval(self,t,endBehavior)
		return res[len(res)/2:]

	def eval_state(self,t,endBehavior='halt'):
		"""Returns the (configuration,velocity) state at time t."""
		return Trajectory.eval(self,t,endBehavior)

	def eval_config(self,t,endBehavior='halt'):
		"""Returns just the configuration component of the result"""
		res = Trajectory.eval(self,t,endBehavior)
		return res[:len(res)/2]
	
	def eval_velocity(self,t,endBehavior='halt'):
		"""Returns just the velocity component of the result"""
		res = Trajectory.eval(self,t,endBehavior)
		return res[len(res)/2:]

	def eval_accel(self,t,endBehavior='halt'):
		"""Returns just the acceleration component of the derivative"""
		res = Trajectory.deriv(self,t,endBehavior)
		return res[len(res)/2:]

	def interpolate(self,a,b,u):
		assert len(a)==len(b)
		x1,v1 = a[:len(a)/2],a[len(a)/2:]
		x2,v2 = b[:len(b)/2],b[len(b)/2:]
		x = spline.hermite_eval(x1,v1,x2,v2,u)
		dx = spline.hermite_deriv(x1,v1,x2,v2,u)
		return x+dx
	
	def difference(self,a,b,u):
		assert len(a)==len(b)
		x1,v1 = a[:len(a)/2],a[len(a)/2:]
		x2,v2 = b[:len(b)/2],b[len(b)/2:]
		dx = spline.hermite_deriv(x1,v1,x2,v2,u,order=1)
		ddx = spline.hermite_deriv(x1,v1,x2,v2,u,order=2)
		return dx+ddx

	def constructor(self):
		return HermiteTrajectory


def execute_path(path,controller,speed=1.0,smoothing=None,activeDofs=None):
	"""Sends an untimed trajectory on a controller.
	
	Arguments:
	- path: a list of milestones
	- controller: a SimRobotController
	- smoothing: any smoothing applied to the path.  Valid values are:
	  * None: starts / stops at each milestone, moves in linear joint-space paths
	  * 'spline': interpolates milestones smoothly with some differenced velocity
	  * 'ramp': starts / stops at each milestone, moves in minimum-time / minimum-
	    acceleration paths.
	- activeDofs: if not None, a list of dofs that are moved by the trajectory.  Each
	  entry may be an integer or a string.
	"""
	if activeDofs is not None:
		indices = [controller.model().link(d).getIndex for d in activeDofs]
		q0 = controller.getCommandedConfig()
		liftedMilestones = []
		for m in path:
			assert(len(m)==len(indices))
			q = q0[:]
			for i,v in zip(indices,m):
				q[i] = v
			liftedMilestones.append(q)
		return execute_path(liftedMilestones,controller,speed,smoothing)

	if smoothing == None:
		controller.setMilestone(path[0])
		for i in range(1,len(path)):
			controller.addMilestoneLinear(path[i])
	elif smoothing == 'spline':
		raise NotImplementedError("Spline interpolation")
	elif smoothing == 'ramp':
		controller.setMilestone(path[0])
		for i in range(1,len(path)):
			controller.addMilestone(path[i])
	else:
		raise ValueError("Invalid smoothing method specified")

def execute_trajectory(trajectory,controller,speed=1.0,smoothing=None,activeDofs=None):
	"""Sends a timed trajectory to a controller.

	Arguments:
	- trajectory: a Trajectory, RobotTrajectory, or HermiteTrajectory instance
	- controller: a SimRobotController
	- smoothing: any smoothing applied to the path.  Only valid for piecewise
	  linear trajectories.  Valid values are
	  * None: no smoothing, just do a piecewise linear trajectory
	  * 'spline': interpolate tangents to the curve
	  * 'pause': smoothly speed up and slow down
	- activeDofs: if not None, a list of dofs that are moved by the trajectory.  Each
	  entry may be an integer or a string.
	"""
	if activeDofs is not None:
		indices = [controller.model().link(d).getIndex for d in activeDofs]
		q0 = controller.getCommandedConfig()
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
		controller.setMilestone(trajectory.eval(ts),trajectory.deriv(ts))
		n = len(trajectory.milestones[0])/2
		for i in range(1,len(trajectory.times)):
			q,v = trajectory.milestones[i][:n],trajectory.milestones[i][n:]
			controller.addCubic(q,v,(trajectory.times[i]-trajectory.times[i-1])/speed)
	else:
		if smoothing == None:
			ts = trajectory.startTime()
			controller.setMilestone(trajectory.eval(ts))
			for i in range(1,len(trajectory.times)):
				q = trajectory.milestones[i]
				controller.addLinear(q,(trajectory.times[i]-trajectory.times[i-1])/speed)
		elif smoothing == 'spline':
			t = HermiteTrajectory()
			t.makeSpline(trajectory)
			return execute_trajectory(t,controller)
		elif smoothing == 'pause':
			ts = trajectory.startTime()
			controller.setMilestone(trajectory.eval(ts))
			zero = [0.0]*len(trajectory.milestones[0])
			for i in range(1,len(trajectory.times)):
				q = trajectory.milestones[i]
				controller.addCubic(q,zero,(trajectory.times[i]-trajectory.times[i-1])/speed)
		else:
			raise ValueError("Invalid smoothing method specified")
