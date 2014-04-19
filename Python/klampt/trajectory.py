import bisect
import vectorops

class Trajectory:
	"""A basic piecewise-linear trajectory class.  By default, interpolates
	in Cartesian space.  To interpolate for a robot, use RobotTrajectory.
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

	def getSegment(self,t,endBehavior='loop'):
		"""Returns the index and interpolation parameter for the
		segment at time t"""
		if len(self.times)==0:
			raise ValueError("Empty trajectory")
		if len(self.times)==1:
			return (-1,0)
		if t > self.times[-1]:
			if endBehavior == 'loop':
				t = t % self.times[-1]
			else:
				return (len(self.milestones),0)
		i = bisect.bisect_left(self.times,t)
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
	
	def eval(self,t,endBehavior='loop'):
		"""Evaluates the trajectory using piecewise linear
		interpolation"""
		i,u = self.getSegment(t,endBehavior)
		if i<0: return self.milestones[0]
		elif i>=len(self.milestones): return self.milestones[-1]
		#linear interpolate between milestones[i] and milestones[i+1]
		return self.interpolate(self.milestones[i],self.milestones[i+1],u)

	def deriv(self,t,endBehavior='loop'):
		"""Evaluates the trajectory velocity using piecewise linear
		interpolation"""
		i,u = self.getSegment(t,endBehavior)
		if i<0: return [0.0]*len(self.milestones[0])
		elif i>=len(self.milestones): return [0.0]*len(self.milestones[-1])
		return vectorops.mul(self.difference(self.milestones[i+1],self.milestones[i]),1.0/(self.times[i+1]-self.times[i]))

	def interpolate(self,a,b,u):
		"""Can override this to implement non-cartesian spaces"""
		return vectorops.interpolate(a,b,u)
	
	def difference(self,a,b):
		"""Can override this to implement non-Cartesian spaces"""
		return vectorops.sub(a,b)

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
