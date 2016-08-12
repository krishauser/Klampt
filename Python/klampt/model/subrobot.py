from ..robotsim import *
from collide import self_collision_iter

class SubRobotModel:
	"""A helper that lets you conveniently set/get quantities for a subset
	of moving links on a RobotModel.  This class has the same
	API as RobotModel, but everything is re-indexed so that configurations
	and link indices only modify the given subset of links.  As a reult,
	most methods applicable to a RobotModel can also be applied to
	a SubRobotModel.

	You provide the list of moving link indices or names in the constructor.

	Exceptions include:
	- The RobotLinkModel will refer to the original robot.
	"""
	def __init__(self,robot,links):
		assert isinstance(robot,(RobotModel,SubRobotModel)),"SubRobotModel constructor must be given a RobotModel or SubRobotModel as first argument"
		self.robot = robot
		self.links = links[:]
		self.index = robot.index
		self.world = robot.world
		if isinstance(robot,SubRobotModel):
			raise NotImplementedError("TODO: sub-robot of sub-robot")
		else:
			for i,l in self.links:
				if isinstance(l,str):
					self.links[i] = robot.link(l).getIndex()

	def numLinks(self):
		return len(self.links)

  	def link(self,index):
  		if isinstance(index,str):
  			return self.robot.link(index)
  		else:
  			return self.robot.link(self.links[index])

	def numDrivers(self):
		raise NotImplementedError("TODO Accessing number of drivers in sub-robot")

	def driver(self,index):
		raise NotImplementedError("TODO Accessing drivers in sub-robot")
  
	def getConfig(self):
		q = self.robot.getConfig()
		return [q[i] for i in self.links]
	def getVelocity(self):
		q = self.robot.getVelocity()
		return [q[i] for i in self.links]
	def setConfig(self,q):
		assert len(q) == len(self.links)
		qfull = self.robot.getConfig()
		for i,v in zip(self.links,q):
			qfull[i] = v
		self.robot.setConfig(qfull)
	def setVelocity(self,q):
		assert len(q) == len(self.links)
		qfull = self.robot.getVelocity()
		for i,v in zip(self.links,q):
			qfull[i] = v
		self.robot.setVelocity(qfull)
	def getJointLimits(self):
		qmin,qmax = self.robot.getJointLimits()
		return [qmin[i] for i in self.links],[qax[i] for i in self.links]
	def setJointLimits(self,qmin,qmax):
		assert len(qmin) == len(self.links)
		assert len(qmax) == len(self.links)
		qminfull,qmaxfull = self.robot.getJointLimits()
		for i,a,b in zip(self.links,qmin,qmax):
			qminfull[i] = a
			qmaxfull[i] = b
		self.robot.setJointLimits(qminfull,qmaxfull)
	def getVelocityLimits(self):
		q = self.robot.getVelocityLimits()
		return [q[i] for i in self.links]
	def setVelocityLimits(self,vmax):
		assert len(q) == len(self.links)
		qfull = self.robot.getVelocityLimits()
		for i,v in zip(self.links,vmax):
			qfull[i] = v
		self.robot.setVelocityLimits(qfull)
	def getAccelerationLimits(self):
		q = self.robot.getAccelerationLimits()
		return [q[i] for i in self.links]
	def setAccelerationLimits(self,amax):
		assert len(q) == len(self.links)
		qfull = self.robot.getAccelerationLimits()
		for i,v in zip(self.links,amax):
			qfull[i] = v
		self.robot.setAccelerationLimits(qfull)
	def getTorqueLimits(self):
		q = self.robot.getTorqueLimits()
		return [q[i] for i in self.links]
	def setTorquenLimits(self,tmax):
		assert len(q) == len(self.links)
		qfull = self.robot.getTorqueLimits()
		for i,v in zip(self.links,tmax):
			qfull[i] = v
		self.robot.setTorqueLimits(qfull)

	def setDOFPosition(self,index,qi):
		if isinstance(index,str):
			self.robot.setDOFPosition(index,qi)
		else:
			self.robot.setDOFPosition(self.links[index],qi)
	def getDOFPosition(self,index):
		if isinstance(index,str):
			return self.robot.getDOFPosition(index)
		else:
			return self.robot.getDOFPosition(self.links[index])
	def getCom(self):
		raise NotImplementedError("TODO: getCom")
	def getComJacobian(self):
		raise NotImplementedError("TODO: getComJacobian")
	def getMassMatrix(self):
		raise NotImplementedError("TODO: getMassMatrix")
	def getMassMatrixInv(self):
		raise NotImplementedError("TODO: getMassMatrix")
	def getCoriolisForceMatrix(self):
		raise NotImplementedError("TODO: getCoriolisForceMatrix")
  	def getCoriolisForces(self):
		raise NotImplementedError("TODO: getCoriolisForceMatrix")
	def getGravityForces(self,g):
		raise NotImplementedError("TODO:  getGravityForces")
	def torquesFromAccel(self,ddq):
		raise NotImplementedError("TODO: torquesFromAccel")
	def accelFromTorques(self,t):
		raise NotImplementedError("TODO: accelFromTorques")

	def interpolate(self,a,b,u):
		afull = self.robot.getConfig()
		bfull = afull[:]
		for i,ai,bi in zip(self.links,a,b):
			afull[i] =a
			bfull[i] =b
		q = self.robot.interpolate(afull,bfull)
		return [q[i] for i in self.links]
 	def distance(self,a,b):
 		afull = self.robot.getConfig()
		bfull = afull[:]
		for i,ai,bi in zip(self.links,a,b):
			afull[i] =a
			bfull[i] =b
		return self.robot.distance(afull,bfull)
	def interpolate_deriv(self,a,b):
		afull = self.robot.getConfig()
		bfull = afull[:]
		for i,ai,bi in zip(self.links,a,b):
			afull[i] =a
			bfull[i] =b
		q = self.robot.interpolate_deriv(afull,bfull)
		return [q[i] for i in self.links]
 	def selfCollisionEnabled(self,link1,link2):
 		if not isinstance(link1,str):
 			link1 = self.links[link1]
 		else:
 			link1 = self.robot.link(link1).getIndex()
 		if not isinstance(link2,str):
 			link2 = self.links[link2]
 		else:
 			link2 = self.robot.link(link2).getIndex()
 		return self.robot.selfCollisionEnabled(link1,link2)
 	def enableSelfCollision(self,link1,link2,value=True):
 		if not isinstance(link1,str):
 			link1 = self.links[link1]
 		else:
 			link1 = self.robot.link(link1).getIndex()
 		if not isinstance(link2,str):
 			link2 = self.links[link2]
 		else:
 			link2 = self.robot.link(link2).getIndex()
 		self.robot.enableSelfCollision(link1,link2,value)
  	def selfCollides(self):
  		geoms = [self.robot.link(i).geometry() for i in self.links]
  		def dotest(i,j):
  			return self.robot.selfCollisionEnabled(self.links[i],self.links[j])
  		return self_collision_iter(geoms,dotest)
	def drawGL(self,keepAppearance=True):
		for i in self.links:
			self.robot.link(i).drawGL(keepAppearance)
