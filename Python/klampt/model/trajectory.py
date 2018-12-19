"""Classes for loading, saving, evaluating, and operating on trajectories.

- For piecewise-linear interpolation in cartesian space, use Trajectory.
- For piecewise-linear interpolation on a robot, use RobotTrajectory.
- For Hermite interpolation in cartesian space, use HermiteTrajectory.
"""

import bisect
from ..math import so3,se3,vectorops
from ..math import spline
from ..math.geodesic import *

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
        
	def __init__(self,times=None,milestones=None):
		if milestones == None:
			milestones = []
		if times == None:
			times = range(len(milestones))
		self.times = times
		self.milestones = milestones

	def load(self,fn):
		"""Reads from a whitespace-separated file in the format

		t1 [q1]
		t2 [q2]
		...

		where each [qi] is a Klamp't formatted length-n configuration, written
		in the form "n qi1 ... qin".
		"""
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
		loops forever.  O(log n) time where n is the number of
		segments."""
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
			return (0,0)
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
	
	def eval(self,t,endBehavior='halt'):
		"""Evaluates the trajectory using piecewise linear
		interpolation.  If endBehavior='loop' then the trajectory
		loops forever."""
		i,u = self.getSegment(t,endBehavior)
		if i<0: return self.milestones[0]
		elif i+1>=len(self.milestones): return self.milestones[-1]
		#linear interpolate between milestones[i] and milestones[i+1]
		return self.interpolate(self.milestones[i],self.milestones[i+1],u,self.times[i+1]-self.times[i])

	def deriv(self,t,endBehavior='halt'):
		"""Evaluates the trajectory velocity using piecewise linear
		interpolation.  If endBehavior='loop' then the trajectory
		loops forever."""
		i,u = self.getSegment(t,endBehavior)
		if i<0: return [0.0]*len(self.milestones[0])
		elif i+1>=len(self.milestones): return [0.0]*len(self.milestones[-1])
		return self.difference(self.milestones[i+1],self.milestones[i],u,self.times[i+1]-self.times[i])

	def interpolate(self,a,b,u,dt):
		"""Can override this to implement non-cartesian spaces.
		Interpolates along the geodesic from a to b.  dt is the 
		duration of the segment from a to b"""
		return vectorops.interpolate(a,b,u)
	
	def difference(self,a,b,u,dt):
		"""Subclasses can override this to implement non-Cartesian
		spaces.  Returns the time derivative along the geodesic from b to
		a.  dt is the duration of the segment form a to b"""
		return vectorops.mul(vectorops.sub(a,b),1.0/dt)
	
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
					print "Suffix start:",suffix.milestones[0]
					print "Self end:",self.milestones[-1]
					raise ValueError("Concatenation would cause a jump in configuration")
				if jumpPolicy=='strict' or (jumpPolicy=='blend' and suffix.milestones[0] != self.milestones[-1]):
					#discard last milestone of self
					times = self.times[:-1] + [t+offset for t in suffix.times]
					milestones = self.milestones[:-1] + suffix.milestones
					return self.constructor()(times,milestones)
		times = self.times + [t+offset for t in suffix.times]
		milestones = self.milestones + suffix.milestones
		return self.constructor()(times,milestones)

	def insert(self,time):
		"""Inserts a milestone and keyframe at the given time.  Returns the index of the new
		milestone, or if a milestone already exists, then it returns that milestone index.

		If the path is empty, the milestone is set to an empty list []."""
		if len(self.times) == 0:
			self.times = [time]
			self.milestones = [[]]
			return 0
		i,u = self.getSegment(time)
		if i < 0:
			self.times.insert(0,time)
			self.milestones.insert(0,self.milestones[0][:])
		elif time <= self.times[0]:
			if time < self.times[0]:
				self.times.insert(0,time)
				self.milestones.insert(0,self.milestones[0][:])
			return 0
		elif time >= self.times[-1]:
			if time > self.times[-1]:
				self.times.append(time)
				self.milestones.append(self.milestones[-1][:])
			return len(self.times)-1
		elif u == 0:
			return i
		elif u == 1:
			return i+1
		else:
			q = self.interpolate(self.milestones[i],self.milestones[i+1],u,self.times[i+1]-self.times[i])
			self.times.insert(i,time)
			self.milestones.insert(i,q)
			return i

	def split(self,time):
		"""Returns a pair of trajectories obtained from splitting this
		one at the given time"""
		i,u = self.getSegment(time)
		if i < 0:
			return self.constructor()(),self.constructor()()
		elif time <= self.times[0]:
			#split before start of trajectory
			return self.constructor()([time],[self.milestones[0]]),self.constructor()([time]+self.times,[self.milestones[0]]+self.milestones)
		elif time >= self.times[-1]:
			#split after end of trajectory
			return self.constructor()(self.times+[time],self.milestones+[self.milestones[-1]]),self.constructor()([time],self.milestones[-1])
		#split in middle of trajectory
		splitpt = self.interpolate(self.milestones[i],self.milestones[i+1],u,self.times[i+1]-self.times[i])
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

	def discretize(self,dt):
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
			new_milestones.append(self.eval(t))
		if abs(t-self.times[-1]) > 1e-6:
			new_times.append(self.times[-1])
			new_milestones.append(self.milestones[-1][:])
		else:
			new_times[-1] = self.times[-1]
			new_milestones[-1] = self.milestones[-1][:]
		return self.constructor()(new_times,new_milestones)

	def remesh(self,newtimes,tol=1e-6):
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
		i = 0
		while sorter[i][0] < self.startTime():
			i += 1
		if i != 0:
			res.times.append(self.startTime())
			res.milestones.append(self.milestones[0])
		resindices = []
		firstold = 0
		lastold = 0
		while i < len(sorter):
			#check if we should add this
			t,idx = sorter[i]
			i+=1
			if idx >= 0:
				if t == res.times[-1]:
					resindices.append(len(res.times)-1)
					continue
				#it's a new mesh point, add it and check whether previous old milestones should be added
				if self.times[lastold] == t:
					#matched the last old mesh point, no need to call eval()
					newx = self.milestones[lastold]
				else:
					newx = self.eval(t)
				res.times.append(t)
				res.milestones.append(newx)
				for j in range(firstold,lastold):
					if self.times[j] == t:
						continue
					x = res.eval(self.times[j])
					if vectorops.norm(self.difference(x,self.milestones[j],0,1.0)) > tol:
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
		for i in xrange(len(res.times)-1):
			assert res.times[i] < res.times[i+1]
		for i,idx in enumerate(resindices):
			assert newtimes[i] == res.times[idx]
		return (res,resindices)

class RobotTrajectory(Trajectory):
	"""A trajectory that performs interpolation according to the robot's
	interpolation scheme."""
	def __init__(self,robot,times=None,milestones=None):
		Trajectory.__init__(self,times,milestones)
		self.robot = robot
	def interpolate(self,a,b,u,dt):
		return self.robot.interpolate(a,b,u)
	def difference(self,a,b,u,dt):
		return vectorops.mul(self.robot.interpolateDeriv(b,a),1.0/dt)
	def constructor(self):
		return lambda times=None,milestones=None: RobotTrajectory(self.robot,times,milestones)
	def getLinkTrajectory(self,link,discretization=None):
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

class GeodesicTrajectory(Trajectory):
	"""A trajectory that performs interpolation on a GeodesicSpace.
	See klampt.geodesic for more information."""
	def __init__(self,geodesic,times=None,milestones=None):
		self.geodesic = geodesic
		Trajectory.__init__(self,times,milestones)
	def interpolate(self,a,b,u,dt):
		return self.geodesic.interpolate(a,b,u)
	def difference(self,a,b,u,dt):
		if u > 0.5:
			#do this from the b side
			return vectorops.mul(self.difference(b,a,1.0-u,dt),-1.0/dt)
		x = self.interpolate(a,b,u,dt)
		return vectorops.mul(self.geodesic.difference(x,b),1.0/(dt*(1.0-u)))
	def constructor(self):
		return lambda times,milestones:GeodesicTrajectory(self.geodesic,times,milestones)

class SO3Trajectory(GeodesicTrajectory):
	"""A trajectory that performs interpolation in SO3.  Each milestone
	is a 9-D klampt.so3 element."""
	def __init__(self,times=None,milestones=None):
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
	def __init__(self,times=None,milestones=None):
		"""Constructor can take either a list of SE3 elements or
		12-element vectors."""
		if milestones is not None and len(milestones) > 0 and len(milestones[0])==2:
			GeodesicTrajectory.__init__(self,SE3Space(),times,[m[0]+m[1] for m in milestones])
		else:
			GeodesicTrajectory.__init__(self,SE3Space(),times,milestones)
	def to_se3(self,milestone):
		"""Converts a state parameter vector to a klampt.se3 element"""
		return (milestone[:9],milestone[9:])
	def from_se3(self,T):
		"""Converts a klampt.se3 element to a state parameter vector"""
		return list(T[0]) + list(T[1])
	def eval_se3(self,t,endBehavior='halt'):
		"""Returns an SE3 element"""
		res = self.eval(t,endBehavior)
		return self.to_se3(res)
	def deriv_se3(self,t,endBehavior='halt'):
		"""Returns the derivative as the derivatives of an SE3
		element"""
		res = self.deriv(t,endBehavior)
		return self.to_se3(res)
	def preTransform(self,T):
		"""Premultiplies every transform in here by the se3 element
		T. In other words, if T transforms a local frame F to frame F',
		this method converts this SE3Trajectory from coordinates in F
		to coordinates in F'"""
		for i,m in enumerate(self.milestones):
			Tm = self.to_se3(m)
			self.milestones[i] = self.from_se3(se3.mul(T,Tm))
	def postTransform(self,T):
		"""Postmultiplies every transform in here by the se3 element
		T. In other words, if T transforms a local frame F to frame F',
		this method converts this SE3Trajectory from describing how F'
		moves to how F moves."""
		for i,m in enumerate(self.milestones):
			Tm = self.to_se3(m)
			self.milestones[i] = self.from_se3(se3.mul(Tm,T))
	def getRotationTrajectory(self):
		"""Returns an SO3Trajectory describing the rotation
		trajectory."""
		return SO3Trajectory(self.times,[m[:9] for m in self.milestones])
	def getPositionTrajectory(self,localPt=None):
		"""Returns a Trajectory describing the movement of the given
		local point localPt (or the origin, if none is provided)."""
		if localPt is None:
			return Trajectory(self.times,[m[9:] for m in self.milestones])
		else:
			return Trajectory(self.times,[se3.apply(self.to_se3(m),localPt) for m in self.milestones])
	def constructor(self):
		return SE3Trajectory

class _HermiteConfigAdaptor(Trajectory):
	def __init__(self,hermite):
		self.hermite = hermite
	def eval(self,t,endBehavior='halt'):
		return self.hermite.eval_config(t,endBehavior)
	def deriv(self,t,endBehavior='halt'):
		return self.hermite.eval_velocity(t,endBehavior)
	def accel(self,t,endBehavior='halt'):
		return self.hermite.eval_accel(t,endBehavior)
	def __getattr__(self,item):
		if item in ['eval','deriv','accel']:
			return self.__dict__[item]
		hermite = self.__dict__['hermite']
		hitem = getattr(hermite,item)
		if item == 'milestones':
			return [q[:len(q)//2] for q in hitem]
		def methodadaptor(*args,**kwargs):
			res = hitem(*args,**kwargs)
			if isinstance(res,HermiteTrajectory):
				return _HermiteConfigAdaptor(res)
			elif hasattr(res,'__iter__'):
				return [(_HermiteConfigAdaptor(v) if isinstance(v,HermiteTrajectory) else v) for v in res]
			return res
		if callable(hitem):
			return methodadaptor
		return hitem

class HermiteTrajectory(Trajectory):
	"""A trajectory whose milestones are given in phase space (x,dx).
	
	eval_config(t) returns the configuration, and eval_velocity(t) justs
	get the velocity, and to get acceleration, use eval_accel(t).

	If you want to use one of these trajectories like a normal configuration-space
	trajectory, so that eval() returns a configuration, call self.configTrajectory()
	"""
	def __init__(self,times=None,milestones=None,dmilestones=None):
		"""If dmilestones is given, then milestones is interpreted
		as configurations and dmilestones is interpreted as velocities.
		
		Otherwise, the milestones are interpreted as states (x,dx)
		"""
		if dmilestones is None:
			Trajectory.__init__(self,times,milestones)
		else:
			assert milestones != None
			#interpret as config/velocity
			self.times = times
			self.milestones = [q+dq for (q,dq) in zip(milestones,dmilestones)]

	def configTrajectory(self):
		return _HermiteConfigAdaptor(self)

	def makeSpline(self,waypointTrajectory,preventOvershoot=True):
		"""Computes natural velocities for a standard configuration-
		space Trajectory to make it smoother."""
		velocities = []
		t = waypointTrajectory
		d = len(t.milestones[0])
		if len(t.milestones)==1:
			velocities.append([0]*d)
		elif len(t.milestones)==2:
			s = (1.0/(t.times[1]-t.times[0]) if (t.times[1]-t.times[0]) != 0 else 0)
			v = vectorops.mul(vectorops.sub(t.milestones[1],t.milestones[0]),s) 
			velocities.append(v)
			velocities.append(v)
		else:
			third = 1.0/3.0
			for i in range(1,len(waypointTrajectory.milestones)-1):
				s = (1.0/(t.times[i+1]-t.times[i-1]) if (t.times[i+1]-t.times[i-1]) != 0 else 0)
				v = vectorops.mul(vectorops.sub(t.milestones[i+1],t.milestones[i-1]),s)
				if preventOvershoot:
					dtp = t.times[i]-t.times[i-1]
					dtn = t.times[i]-t.times[i-1]
					for j,(x,a,b) in enumerate(zip(t.milestones[i],t.milestones[i-1],t.milestones[i+1])):
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

	def eval_state(self,t,endBehavior='halt'):
		"""Returns the (configuration,velocity) state at time t."""
		return Trajectory.eval(self,t,endBehavior)

	def eval_config(self,t,endBehavior='halt'):
		"""Returns just the configuration component of the result"""
		res = Trajectory.eval(self,t,endBehavior)
		return res[:len(res)//2]
	
	def eval_velocity(self,t,endBehavior='halt'):
		"""Returns just the velocity component of the result"""
		res = Trajectory.eval(self,t,endBehavior)
		return res[len(res)//2:]

	def eval_accel(self,t,endBehavior='halt'):
		"""Returns just the acceleration component of the derivative"""
		res = Trajectory.deriv(self,t,endBehavior)
		return res[len(res)//2:]

	def interpolate(self,a,b,u,dt):
		assert len(a)==len(b)
		x1,v1 = a[:len(a)//2],vectorops.mul(a[len(a)//2:],dt)
		x2,v2 = b[:len(b)//2],vectorops.mul(b[len(b)//2:],dt)
		x = spline.hermite_eval(x1,v1,x2,v2,u)
		dx = vectorops.mul(spline.hermite_deriv(x1,v1,x2,v2,u),1.0/dt)
		return x+dx
	
	def difference(self,a,b,u,dt):
		assert len(a)==len(b)
		x1,v1 = a[:len(a)//2],vectorops.mul(a[len(a)//2:],dt)
		x2,v2 = b[:len(b)//2],vectorops.mul(b[len(b)//2:],dt)
		dx = vectorops.mul(spline.hermite_deriv(x1,v1,x2,v2,u,order=1),1.0/dt)
		ddx = vectorops.mul(spline.hermite_deriv(x1,v1,x2,v2,u,order=2),1.0/pow(dt,2))
		return dx+ddx

	def constructor(self):
		return HermiteTrajectory

def path_to_trajectory(path,velocities='auto',timing='limited',smoothing='spline',
	zerotol=None,vmax='auto',amax='auto',
	speed=1.0,dt=0.01,
	startvel=0.0,endvel=0.0):
	"""Converts an untimed path to a timed trajectory.

	Arguments:
	- path: a list of milestones, a trajectory, or a RobotTrajectory.  In the latter
	  cases, if durations = 'path' then the durations are extracted from the trajectory's
	  timing.
	- velocities: the manner in which velocities are assigned along the path.
	  * 'auto': if timing is limited, this is equivalent to 'constant'. Otherwise, this is equivalent
	    to trapezoidal.
	  * 'trapezoidal': a trapezoidal velocity profile with max acceleration and velocity.
	    If timing is not 'limited' then this is the same as triangular
	  * 'constant': the path is executed at fixed constant velocity
	  * 'triangular': similar to trapezoidal but without any max velocity
	  * 'parabolic': a parabolic curve (output is a Hermite spline)
	  * 'cosine': velocities follow (1-cosine)/2
	  * 'minimum-jerk': minimum jerk velocities
	  * 'optimal': uses time scaling optimization. NOT IMPLEMENTED YET
	- timing: affects how path timing between milestones is normalized.  The first step is to assign a
	  'distance' indicating how much distance is being traveled.  After assigning distances, the overall
	  length L of the path determines the duration T of the trajectory.  For constant velocity profiles, T=L. 
	  For trapezoidal, triangular, parabolic, and cosine, T = sqrt(L).  For minimum-jerk, T = L^(1/3).
	  * 'limited' (default): uses the vmax, amax variables along with the velocity profile
	    to dynamically determine the duration assigned to each segment.
	  * 'uniform': base timing between milestones is uniform (1/(|path|*speed))
	  * 'path': only valid if path is a Trajectory object.  Uses the timing in path.times as the
	    base timing.
	  * 'L2': base timing is set proportional to L2 distance between milestones
	  * 'Linf': base timing is set proportional to L-infinity distance between milestones
	  * 'robot': base timing is set proportional to robot's distance function between milestones
	  * a list or tuple: the base timing is given in this list
	  * callable function f(a,b): sets the normalization to the function f(a,b).
	- smoothing: if 'spline', the geometric path is first smoothed before assigning times.  Otherwise, the
	  geometric path is interpreted as a piecewise linear path.
	- zerotol: determines how start/stop segments are determined.  If None, the trajectory only
	  pauses at the start and end of the path.  If 0, it pauses at every milestone.  Otherwise,
	  it pauses if the curvature at the milestone exceeds zerotol.
	- vmax: only meaningful if timing=='limited'. Can be:
	  * 'auto': either 1 or the robot's joint velocity limits if a RobotTrajectory is provided
	  * a positive number: the L2 norm of the derivative of the result trajectory is limited to this value
	  * a list of positive floats: the element-wise derivative of the result trajectory is limited
	    to this value
	- amax: only meaningful if timing=='limited'. Can be:
	  * 'auto': either 4 or the robot's joint acceleration limits if a RobotTrajectory is provided
	  * a positive number: the L2 norm of the acceleration of the result trajectory is limited to this value
	  * a list of positive floats: the element-wise acceleration of the result trajectory is limited
	    to this value.
	- speed: a speed multiplier applied to the resulting path
	- dt: the resolution of the resulting trajectory.
	- startvel, endvel: the starting and ending velocities of the path. Specified as multipliers
	  of path[1]-path[0] and path[-1]-path[-2], respectively.  Must be nonnegative.  Might not be
	  respected for some velocity profiles.
	"""
	assert dt > 0.0,"dt has to be positive"
	if vmax == 'auto' and timing == 'limited':
		if isinstance(path,RobotTrajectory):
			vmax = path.robot.getVelocityLimits()
		else:
			vmax = 1.0
	if amax == 'auto' and timing == 'limited':
		if isinstance(path,RobotTrajectory):
			amax = path.robot.getAccelerationLimits()
		else:
			amax = 4.0
	if speed != 1.0:
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
				for i in xrange(len(milestones)-1):
					q,n = milestones[i],milestones[i+1]
					if i == 0: p = q
					else: p = milestones[i-1]
					if i+2 == len(milestones): nn = n
					else: nn = milestones[i+2]
					if isinstance(path,Trajectory):
						v = vectorops.mul(path.difference(p,n,0.5,1.0),0.5)
						a1 = vectorops.sub(path.difference(q,n,0.,1.),path.difference(p,q,1.,1.))
						a2 = vectorops.sub(path.difference(n,nn,0.,1.),path.difference(q,n,1.,1.))
					else:
						v = vectorops.mul(vectorops.sub(n,p),0.5)
						a1 = vectorops.madd(vectorops.add(p,n),q,-2.0)
						a2 = vectorops.madd(vectorops.add(q,nn),n,-2.0)
					if hasattr(vmax,'__iter__'):
						for j,(x,lim) in enumerate(zip(v,vmax)):
							if abs(x) > lim*_durations[i]:
								_durations[i] = abs(x)/lim
								#print "Segment",i,"limited on axis",j,"path velocity",x,"limit",lim
					else:
						_durations[i] = vectorops.norm(v)/vmax
					if hasattr(amax,'__iter__'):
						if i > 0:
							for j,(x,lim) in enumerate(zip(a1,amax)):
								if abs(x) > lim*_durations[i]**2:
									_durations[i] = math.sqrt(abs(x)/lim)
									#print "Segment",i,"limited on axis",j,"path accel",x,"limit",lim
						if i+2 < len(milestones):
							for j,(x,lim) in enumerate(zip(a2,amax)):
								if abs(x) > lim*_durations[i]**2:
									_durations[i] = math.sqrt(abs(x)/lim)
									#print "Segment",i,"limited on axis",j,"outgoing path accel",x,"limit",lim
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
				if hasattr(path,'robot'):
					durationfuncs['robot'] = path.robot.distance
				assert timing in durationfuncs,"Invalid duration function specified, valid values are: "+", ".join(durationfuncs.keys())
				timing = durationfuncs[timing]
				_durations = [timing(a,b) for a,b in zip(milestones[:-1],milestones[1:])]
	assert _durations is not None,"Hmm... didn't assign durations properly?"
	#by this time we have all milestones and durations
	if zerotol is not None:
		splits = [0]
		#split the trajectory then reassemble it
		for i in xrange(1,len(milestones)-1):
			prev = milestones[i-1]
			q = milestones[i]
			next = milestones[i+1]
			acc = vectorops.madd(vectorops.add(prev,next),q,-2.0)
			if vectorops.norm(acc) > zerotol*(_durations[i]*_durations[i-1]):
				splits.append(i)
		splits.append(len(milestones)-1)
		if len(splits) > 2:
			print "path_to_trajectory(): Splitting path into",len(splits)-1,"segments, starting and stopping between"
			res = None
			for i in xrange(len(splits)-1):
				a,b = splits[i],splits[i+1]
				traj = path_to_trajectory(milestones[a:b+1],velocities,timing,smoothing,
					None,vmax,amax,
					1.0,dt)
				if res is None:
					res = traj
				else:
					res = res.concat(traj,relative=True)
			if speed != 1.0:
				res.times = vectorops.mul(res.times,1.0/speed)
			return res
	#canonical case:
	#milestones and _durations are lists
	#start and stop at beginning / end
	#speed = 1
	normalizedPath = Trajectory()
	if isinstance(path,RobotTrajectory):
		normalizedPath = RobotTrajectory(path.robot)
	normalizedPath.milestones = milestones
	normalizedPath.times = [0]
	totaldistance = 0
	for d in _durations:
		totaldistance += d
		normalizedPath.times.append(totaldistance)

	if smoothing == 'spline':
		hpath = HermiteTrajectory()
		hpath.makeSpline(normalizedPath)
		normalizedPath = hpath.configTrajectory()

	if startvel != 0.0 or endvel != 0.0:
		print "path_to_trajectory():WARNING: respecting nonzero start/end velocity not implemented yet"

	#print "path_to_trajectory(): Total distance",totaldistance
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
		#print "Easing velocity max",evmax,"acceleration max",eamax
		#print "Velocity and acceleration-limited segment distances",_durations
		#print "Total distance traveled",totaldistance
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
		print "path_to_trajectory(): Setting first guess of path duration to",finalduration
	res = normalizedPath.constructor()()
	if finalduration == 0:
		print "path_to_trajectory(): there is no movement in the path, returning a 0-duration path"
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
	#print velocities,"easing:"
	for i in xrange(1,N+1):
		res.times[i] = float(i)/float(N)*finalduration
		u = easing(float(i)/float(N))
		#print float(i)/float(N),"->",u
		res.milestones[i] = normalizedPath.eval(u*totaldistance)
	if timing == 'limited':
		scaling = 0.0
		vscaling = 0.0
		aLimitingTime = 0
		vLimitingTime = 0
		for i in xrange(N):
			q,n = res.milestones[i],res.milestones[i+1]
			if i == 0: p = q
			else: p = res.milestones[i-1]
			if isinstance(path,Trajectory):
				v = path.difference(p,n,0.5,dt*2.0)
				a = vectorops.sub(path.difference(q,n,0.,dt),path.difference(p,q,1.,dt))
				a = vectorops.div(a,dt)
			else:
				v = vectorops.div(vectorops.sub(n,p),dt*2.0)	
				a = vectorops.div(vectorops.madd(vectorops.add(p,n),q,-2.0),dt**2)
			if not hasattr(vmax,'__iter__'):
				n = vectorops.norm(v)
				if n > vmax*scaling:
					#print "path segment",i,"exceeded scaling",scaling,"by |velocity|",n,' > ',vmax*scaling
					vscaling = n/vmax
					vLimitingTime = i
			else:
				for x,lim in zip(v,vmax):
					if abs(x) > lim*vscaling:
						#print "path segment",i,"exceeded scaling",scaling,"by velocity",x,' > ',lim*scaling
						#print "Velocity",v
						vscaling = abs(x)/lim
						vLimitingTime = i
			if i == 0:
				continue
			if not hasattr(amax,'__iter__'):
				n = vectorops.norm(a)
				if n > amax*scaling**2:
					#print "path segment",i,"exceeded scaling",scaling,"by |acceleration|",n,' > ',amax*scaling**2
					scaling = math.sqrt(n/amax)
					aLimitingTime = i
			else:
				for x,lim in zip(a,amax):
					if abs(x) > lim*scaling**2:
						#print "path segment",i,"exceeded scaling",scaling,"by acceleration",x,' > ',lim*scaling**2
						#print p,q,n
						#print "Velocity",v
						#print "Previous velocity",path.difference(p,q,1.,dt)
						scaling = math.sqrt(abs(x)/lim)
						aLimitingTime = i
		print "path_to_trajectory(): Velocity limit exceeded by factor of",vscaling,"at time",res.times[vLimitingTime]*max(scaling,vscaling)
		print "path_to_trajectory(): Acceleration limit exceeded by factor of",scaling,"at time",res.times[aLimitingTime]*max(scaling,vscaling)
		if velocities == 'trapezoidal':
			#speed up until vscaling is hit
			if vscaling < scaling:
				print "path_to_trajectory(): Velocity maximum not hit"
			else:
				print "path_to_trajectory(): TODO: fiddle with velocity maximum."
				scaling = max(vscaling,scaling)
				res.times = [t*scaling for t in res.times]
		else:
			scaling = max(vscaling,scaling)
		print "path_to_trajectory(): Velocity / acceleration limiting yields a time expansion of",scaling
		res.times = vectorops.mul(res.times,scaling)
	if speed != 1.0:
		res.times = vectorops.mul(res.times,1.0/speed)
	return res


def execute_path(path,controller,speed=1.0,smoothing=None,activeDofs=None):
	"""Sends an untimed trajectory on a controller.
	
	Arguments:
	- path: a list of milestones
	- controller: a SimRobotController
	- smoothing: any smoothing applied to the path.  Valid values are:
	  * None: starts / stops at each milestone, moves in linear joint-space paths.
	    Trapezoidal velocity profile used.
	  * 'linear': interpolates milestones linearly with fixed duration.  Constant velocity
	    profile used.
	  * 'cubic': interpolates milestones with cubic spline with fixed duration.  Parabolic
	    velocity profile used.  Starts/stops at each milestone.
	  * 'spline': interpolates milestones smoothly with some differenced velocity.
	  * 'ramp': starts / stops at each milestone, moves in minimum-time / minimum-
	    acceleration paths.  Trapezoidal velocity profile used.
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
		if speed != 1.0: raise ValueError("Can't specify speed with no smoothing")
		controller.setMilestone(path[0])
		for i in range(1,len(path)):
			controller.addMilestoneLinear(path[i])
	elif smoothing == 'linear':
		dt = 1.0/speed
		controller.setLinear(dt,path[0])
		for i in range(1,len(path)):
			controller.addLinear(dt,path[i])
	elif smoothing == 'cubic':
		dt = 1.0/speed
		zero = [0.0]*len(path[0])
		controller.addCubic(dt,path[0],zero)
		for i in range(1,len(path)):
			controller.addCubic(dt,path[i],zero)
	elif smoothing == 'spline':
		hpath = HermiteTrajectory()
		hpath.makeSpline(Trajectory(range(len(path)),path))
		qpath = hpath.configTrajectory()
		dt = controller.getRate()
		traj = qpath.discretize(dt)
		controller.setLinear(dt,traj.milestones[0])
		for i in range(1,len(traj.milestones)):
			controller.addLinear(dt,traj.milestones[i])
	elif smoothing == 'ramp':
		if speed != 1.0: raise ValueError("Can't specify speed with ramp smoothing")
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
		n = len(trajectory.milestones[0])//2
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
