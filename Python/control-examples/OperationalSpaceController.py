

###########################################
#	Operational Space Controller API
#	Updated June 17 2013
###########################################

import math
import time
from klampt.math import vectorops,so3,se3
from abc import ABCMeta, abstractmethod
from collections import defaultdict
from sparse_linalg import *
from scipy import sparse 

DEBUG_MOTION_MODEL = 1


class OpSpaceController(controller.ControllerBlock):
	"""An operational space controller that conforms to the ControllerBlock
	interface in klampt.control.controller. """
	def __init__(self,robot):
		"""Setup tasks in operational space, and reads in trajectory files.
		@param robot is an RobotModel instance
		"""
		self.robot = robot
		# Initiate an operational space controller
		self.opController = OperationalSpaceSolver(robot)
		self.reset()

	def __str__(self):
		return self.__class__.__name__ + "tasks " + ','.join(t.name for t in self.opController.taskList)

	def inputNames(self):
		return ['dt','q','dq']

	def outputNames(self):
		return ['qcmd','dqcmd']

	def setMotionModel(self,mm):
		"""Sets the motion model used by the controller"""
		self.opController.motionModel = mm

	def reset(self):
		"""By default sets all tasks to their current value in the robot"""
		self.opController.taskList = []
		self.setupTasks(self.opController)
		self.qlast = None
		self.dqlast = None
		self.dqpredlast = None

	def drawGL(self):
		from OpenGL import GL
		# Visualize tasks
		GL.glDisable(GL.GL_DEPTH_TEST)
		GL.glDisable(GL.GL_LIGHTING)
		q=self.robot.getConfig()
		for task in self.opController.taskList:
			task.drawGL(q)
		GL.glEnable(GL.GL_DEPTH_TEST)

	def signal(self,type,**inputs):
		"""Subclasses might want to override this to catch more signals, e.g., 'reset'."""
		if type == 'enter':
			self.robot.setConfig(inputs['q'])
			self.reset()
		
	def advance(self,**inputs):
		#modify tasks if necessary
		self.manageTasks(inputs,self.opController)

		try:
			dt = inputs["dt"]
			q = inputs["q"]
			dq = inputs["dq"]
		except:
			print "OpSpaceController: Input needs to have state 'q','dq', and timestep 'dt'"
			return None
		
		(qdes,dqdes) = self.opController.solve(q, dq, dt)
		self.opController.advance(q, dq, dt)
		#self.opController.printStatus(q)

		#return {'qcmd':qdes,'dqcmd':[0.0]*len(qdes)}
		return {'qcmd':qdes,'dqcmd':dqdes}

	def setupTasks(self,opSpaceController):
		"""Overload this to add tasks to the operational space controller"""
		pass

	def manageTasks(self,inputs,opSpaceController):
		"""Overload this to add, delete, or change operational space tasks
		while the controller is running"""
		pass



class OperationalSpaceSolver:
	"""A two-level velocity-based operational space controll solver.
	Operational space control tasks are converted into velocity commands
	which are then added onto the current sensed configuration.

	If only a few DOFs are to be commanded, the activeDofs member can be
	set to the list of active DOF indices.  The values of (qdes,dqdes)
	solved for by the controller corresponding to non-active dofs should
	be ignored.
	"""

	def __init__(self, robot):
		"""robot is a robot model
		"""
		self.taskList = []
		self.ulast = None
		self.qdes = None
		self.robot = robot
		self.motionModel = None
		self.activeDofs = None
		self.verbose = False

	def addTask(self, task, **args):
		"""Adds a task into operational space. Keyword args can include
		- priority
		- name
		- weight
		- gains (3-tuple)
		- value
		- velocity
		If value or velocity are not given, they are set from the
		current robot model values.
		"""
		try: task.setPriority(args['priority'])
		except KeyError: pass
		try: task.setName(args['name'])
		except KeyError: pass
		try: task.setWeight(args['weight'])
		except KeyError: pass
		try: task.setGains(*args['gains'])
		except KeyError: pass
		try:
			task.setDesiredValue(args['value'])
		except KeyError:
			self.setDesiredValuesFromConfig(self.robot.getConfig(),[task])
		try:
			task.setDesiredVelocity(args['velocity'])
		except KeyError:
			self.setDesiredVelocitiesFromDifference(self.robot.getConfig(),self.robot.getConfig(),1,[task])
		self.taskList.append(task)
		return task

	def getTaskByName(self, taskName):
		"""Finds a named task.
		Users need to assure no duplicated task names in the task list manually.
		"""
		for taski in self.taskList:
			if taski.name == taskName:
				return taski
		return None

	def setDesiredValuesFromConfig(self,qdes,tasks=None):
		"""Sets all the tasks' desired values from a given desired
		configuration (e.g., to follow a reference trajectory).
		
		If the 'tasks' variable is provided, it should be a list of
		tasks for which the desired values should be set.
		"""
		if tasks == None:
			tasks = self.taskList
		for t in tasks:
			t.setDesiredValue(t.getSensedValue(qdes))

	def setDesiredVelocitiesFromVelocity(self,q,dq,tasks=None):
		if tasks == None:
			tasks = self.taskList
		for t in tasks:
			dx = np.dot(t.getJacobian(q),dq)
			t.setDesiredVelocity(dx)

	def setDesiredVelocitiesFromDifference(self,qdes0,qdes1,dt,tasks=None):
		"""Sets all the tasks' desired velocities from a given pair
		of configurations separated by dt (e.g., to follow a reference
		trajectory).
		
		If the 'tasks' variable is provided, it should be a list of
		tasks for which the desired values should be set.
		"""
		if tasks == None:
			tasks = self.taskList
		for t in tasks:
			xdes0 = t.getSensedValue(qdes0)
			xdes1 = t.getSensedValue(qdes1)
			dx = vectorops.div(t.taskDifference(xdes1,xdes0),dt)
			t.setDesiredVelocity(dx)

	def printStatus(self,q):
		"""Prints a status printout summarizing all tasks' errors."""
		priorities = set()
		names = dict()
		errors = dict()
		totalerrors = dict()
		for t in self.taskList:
			if t.weight==0: continue
			priorities.add(t.level)
			s = t.name
			if len(s) > 8:
				s = s[0:8]
			err = t.getSensedError(q)
			names.setdefault(t.level,[]).append(s)
			errors.setdefault(t.level,[]).append("%.3f"%(vectorops.norm(err)),)
			werrsq = vectorops.normSquared(vectorops.mul(err,t.weight))
			totalerrors[t.level] = totalerrors.get(t.level,0.0) + werrsq
		cols = 5
		colwidth = 10
		for p in priorities:
			print "Priority",p,"weighted error^2",totalerrors[p]
			pnames = names[p]
			perrs = errors[p]
			start = 0
			while start < len(pnames):
				last = min(start+cols,len(pnames))
				print "  Name:  ",
				for i in range(start,last):
					print pnames[i],' '*(colwidth-len(pnames[i])),
				print
				print "  Error: ",
				for i in range(start,last):
					print perrs[i],' '*(colwidth-len(perrs[i])),
				print
				start=last
			

	def getStackedJacobian(self, q,dq,priority):
		"""Formulates J to calculate dqdes
		"""
		Jlist = []
		for taski in self.taskList:
			if taski.weight == 0 or taski.level != priority:
				continue
			Jtemp = taski.getJacobian(q)
			#convert to sparse matrix format
			if sparse.isspmatrix(Jtemp):
				Jtemp = Jtemp.tocsr()
			elif isinstance(Jtemp,np.ndarray):
				Jtemp = sparse.csr_matrix(Jtemp)
			else:
				#interpret as list-of-lists array
				Jtemp = sparse.csr_matrix(np.array(Jtemp))
			#scale by weight
			if hasattr(taski.weight,'__iter__'):
				assert(len(taski.weight)==Jtemp.shape[0])
				#treat as an elementwise weight
				for i,wi in enumerate(taski.weight):
					s,e=Jtemp.indptr[i],Jtemp.indptr[i+1]
					for v in xrange(s,e):
						Jtemp.data[v] *= wi
				"""
				for i in xrange(len(Jtemp)):
					Jtemp[i] = vectorops.mul(Jtemp[i],taski.weight[i])
				"""
			else:
				Jtemp *= taski.weight
				"""
				for i in xrange(len(Jtemp)):
					Jtemp[i] = vectorops.mul(Jtemp[i],taski.weight)
				"""
			Jlist.append(Jtemp)
		if len(Jlist)==0: return None
		J = sparse.vstack(Jlist)

		#eliminate columns corresponding to fixed links
		qmin,qmax = self.robot.getJointLimits()
		for i,(a,b) in enumerate(zip(qmin,qmax)):
			if a == b:
				#print "Fixed link",self.robot.link(i).getName()
				J = zero_col(J,i)
		return J

	def getStackedVelocity(self, q, dq, dt, priority):
		"""Formulates dx to calculate dqdes
		"""
		Vlist = []
		for taski in self.taskList:
			if taski.weight == 0 or taski.level != priority:
				continue
			#scale by weight
			Vtemp = vectorops.mul(taski.getCommandVelocity(q, dq, dt),taski.weight)
			Vlist.append(Vtemp)
		if len(Vlist)==0: return None
		V = np.hstack(Vlist)
		return V

	def getMotionModel(self,q,dq,dt):
		"""Returns a model (A,b) such that the
		next-step velocity is approximated by dq[t+dt] = Au+b"""
		#naive motion model: u = dqcmd
		if self.motionModel == None:
			return (sparse.eye(len(dq),len(dq)),np.zeros(len(dq)))
		#other motion model: assume the model takes
		#commanded velocities and outputs predicted velocities
		return self.motionModel.linearization(q,dq)

	def solve(self, q,dq,dt):
		"""Takes sensed q,dq, timestep dt and returns qdes and dqdes
		in joint space.
		"""

		for task in self.taskList:
			task.updateState(q,dq,dt)
		# priority 1
		if not hasattr(self,'timingStats'):
			self.timingStats = defaultdict(int)
		self.timingStats['count'] += 1
		t1 = time.time()
		J1 = self.getStackedJacobian(q,dq,1)
		v1 = self.getStackedVelocity(q,dq,dt,1)
		(A,b) = self.getMotionModel(q,dq,dt)
		if self.activeDofs != None:
			A = select_cols(A,self.activeDofs)
		if sparse.isspmatrix_coo(A) or sparse.isspmatrix_dia(A):
			A = A.tocsr()
		t2 = time.time()
		self.timingStats['get jac/vel p1'] += t2-t1
		
		J2 = self.getStackedJacobian(q,dq,2)
		if J2 is not None:
			V2 = self.getStackedVelocity(q,dq,dt,2)
		t3 = time.time()
		self.timingStats['get jac/vel p2'] += t3-t2

		#compute velocity limits
		vmax = self.robot.getVelocityLimits()
		vmin = vectorops.mul(vmax,-1.0)
		amax = self.robot.getAccelerationLimits()
		vref = dq if self.ulast == None else self.ulast
		for i,(v,vm,am) in enumerate(zip(vref,vmin,amax)):
			if v-dt*am > vm:
				vmin[i] = v-dt*am
			elif v < vm:
				#accelerate!
				vmin[i] = min(vm,v+dt*am)
		for i,(v,vm,am) in enumerate(zip(vref,vmax,amax)):
			if v-dt*am < vm:
				vmax[i] = v+dt*am
			elif v > vm:
				#decelerate!
				vmax[i] = max(vm,v-dt*am)
		for i,(l,u) in enumerate(zip(vmin,vmax)):
			assert l <= u
			if l > 0 or u < 0:
				print "Moving link:",self.robot.link(i).getName(),"speed",vref[i]
		#print zip(vmin,vmax)
		Aumin = np.array(vmin) - b
		Aumax = np.array(vmax) - b
		#print zip(Aumin.tolist(),Aumax.tolist())
			
		J1A = J1.dot(A)
		J1b = J1.dot(b)
		if J2 == None:
			#just solve constrained least squares
			#J1*(A*u+b) = v1
			#vmin < A*u + b < vmax
			u1 = constrained_lsqr(J1A,v1-J1b,A,Aumin,Aumax)[0]
			u2 = [0.0]*len(u1)
			t4 = time.time()
			self.timingStats['pinv jac p1'] += t4-t3
		else:			
			#solve equality constrained least squares
			#dq = A*u + b
			#J1*dq = v1
			#J1*A*u + J1*b = v1
			#least squares solve for u1:
			#J1*A*u1 = v1 - J1*b
			#vmin < A*u1 + b < vmax
			#need u to satisfy
			#Aact*u = bact
			#we know that u1 satisfies Aact*u = bact
			#let u = u1+u2
			#=> u2 = (I - Aact^+ Aact) z = N*z
			#least squares solve for z:
			#J2*A*(u1+u2+b) = v2
			#J2*A*N z = v2 - J2*(A*u1+b)
			(u1, active, activeRhs) = constrained_lsqr(J1A,v1-J1b,A,Aumin,Aumax)
			Aact = sparse.vstack([J1A]+[A[crow,:] for crow in active]).todense()
			#bact = np.hstack((v1-J1b,activeRhs))
			J1Ainv = np.linalg.pinv(Aact)
			dq1 = A.dot(u1)+b
			if self.verbose and len(active)>0:
				print "Priority 1 active constraints:"
				for a in active:
					print self.robot.link(a).getName(),vmin[a],dq1[a],vmax[a]

			r1 = J1.dot(dq1)-v1
			if self.verbose:
				print "Op space controller solve"
				print "  Residual 1",np.linalg.norm(r1)

			# priority 2
			N = np.eye(len(dq)) - np.dot(J1Ainv, Aact)
			t4 = time.time()
			self.timingStats['pinv jac p1'] += t4-t3

			u2 = [0.0]*len(u1)
			#print "  Initial priority 2 task error",np.linalg.norm(V2-J2.dot(dq1))
			J2A = J2.dot(A)
			J2AN = J2A.dot(N)
			AN = sparse.csr_matrix(np.dot(A.todense(),N))
			#Note: N destroys sparsity
			V2_m_resid = np.ravel(V2 - J2.dot(dq1))
			(z,active,activeRhs) = constrained_lsqr(J2AN,V2_m_resid,AN,vmin-dq1,vmax-dq1)
			t5 = time.time()
			self.timingStats['ls jac p2'] += t5-t4
			u2 = np.ravel(np.dot(N, z))

			#debug, should be close to zero
			#print "  Nullspace projection error:",np.linalg.norm(J1A.dot(u2))
			#this is the error in the nullspace of the first priority tasks
			dq2 = A.dot(u2) + dq1

			#debug, should be equal to residual 2 printout above
			if self.verbose:
				print "  Residual 2",np.linalg.norm(J2.dot(dq2)-V2)
			#debug should be close to zero
			#print "  Residual 2 in priority 1 frame",np.linalg.norm(J1.dot(dq2)-v1)
			if self.verbose and len(active)>0:
				print "Priority 2 active constraints:"
				for a in active:
					print self.robot.link(a).getName(),vmin[a],dq2[a],vmax[a]

		#compose the velocities together
		u = np.ravel((u1 + u2))
		dqpred = A.dot(u)+b
		if self.verbose:
			print "  Residual 1 final",np.linalg.norm(np.ravel(J1.dot(dqpred))-v1)
			if J2 != None:
				print "  Residual 2 final",np.linalg.norm(np.ravel(J2.dot(dqpred))-V2)
		
		u = u.tolist()
		#if self.activeDofs != None:
		#	print "dqdes:",[self.dqdes[v] for v in self.activeDofs]
		self.qdes = vectorops.madd(q, u, dt)
		self.ulast = u

		t6 = time.time()
		self.timingStats['total']+=t6-t1
		if self.verbose:
			if self.timingStats['count']%10==0:
				n=self.timingStats['count']
				print "OpSpace times (ms): vel/jac 1 %.2f inv 1 %.2f vel/jac 2 %.2f inv 2 %.2f total %.2f"%(self.timingStats['get jac/vel p1']/n*1000,self.timingStats['pinv jac p1']/n*1000,self.timingStats['get jac/vel p2']/n*1000,self.timingStats['ls jac p2']/n*1000,self.timingStats['total']/n*1000)
				
		return (self.qdes,u)

	def advance(self,q,dq,dt):
		"""Updates all tasks states"""
		for task in self.taskList:
			task.advance(q,dq,dt)






class Task:
	"""A base class for an operational space task. x=f(q)
	Subclasses should override getters for sensed x, sensed error
	(optional), sensed dx (optional), and appropriate calculation
	of Jacobian.
	Subclasses inherits setters for xdes, dxdes, gains, priority level, 
	weight, and task name.
	If the task space is non-cartesian, the taskDifference method should
	be overridden.
	"""
	__metaclass__ = ABCMeta

	@abstractmethod
	def __init__(self):
		"""Subclasses need to override __init__ to initialize 
		gains, priority level, task weight, task name, xdes, dxdes. 
		"""
		self.level = 1
		self.weight = 1
		self.name = 'unnamed'
		self.hP, self.hD, self.hI = -1, 0, 0
		self.qLast = None
		self.xdes = [0]
		self.dxdes = [0]
		self.eImax = 1e300
		self.vcmdmax = 1e300
		self.dvcmdmax = 1e300
		self.eI = None
		return

	def getDesiredValue(self):
		"""Returns task xdes that has been set
		"""
		return self.xdes
	def setDesiredValue(self, xdes):
		"""User calls this to set task state xdes
		"""
		self.xdes = xdes
	def setDesiredVelocity(self, dxdes):
		"""User calls this to set task state dxdes
		"""
		self.dxdes = dxdes
	def setGains(self,hP=-1,hD=-0.1,hI=-0.1):
		"""User calls this to set PID gains for feedback control in operational space
		"""
		self.hP,self.hD,self.hI = hP,hD,hI
	def setGainsTaskFrame(self,A,Ainv,hP,hD,hI):
		"""User calls this to set PID gains for feedback control
		in a task frame in operational space: as though controlling
		the variable y = Ax = A f(q) axis-wise.
		"""
		if hasattr(hP,'__iter__'):
			self.hP = np.dot(Ainv,np.dot(np.diag(hP),A))
		else:
			self.hP = hP
		if hasattr(hD,'__iter__'):
			self.hD = np.dot(Ainv,np.dot(np.diag(hD),A))
		else:
			self.hD = hD
		if hasattr(hI,'__iter__'):
			self.hI = np.dot(Ainv,np.dot(np.diag(hI),A))
		else:
			self.hI = hI
	def setPriority(self,level=1):
		"""User calls this to set priority level. A smaller value means more important
		"""
		self.level = level
	def setWeight(self, weight):
		"""User calls this to set task weight to differentiate from others on the same 
		priority level. A larger weight means more important.
		"""
		self.weight = weight
	def setName(self, name):
		"""Task name can be used to retrieve a task in an OperationalSpaceSolver instance."""
		self.name = name
	def resetITerm(self):
		self.eI = None

	def updateState(self,q,dq,dt):
		"""Called at beginning of new timestep.
		Optionally does something before computing stuff in getCommandVelocity/advance. e.g., compute cached
		values."""
		pass

	def getCommandVelocity(self, q, dq, dt):
		"""Gets the command velocity from the current state of the
		robot.
		"""
		eP = self.getSensedError(q)
		#vcmd = hP*eP + hD*eV + hI*eI
		vP = gen_mul(self.hP,eP)
		vcmd = vP
		#D term
		vcur = self.getSensedVelocity(q,dq,dt)
		eD = None
		if vcur != None:
			eD = vectorops.sub(vcur, self.dxdes)
			vD = gen_mul(self.hD,eD)
			vcmd = vectorops.add(vcmd, vD)
		#I term
		if self.eI != None:
			vI = gen_mul(self.hI,self.eI)
			vcmd = vectorops.add(vcmd, vI)
		#print "task",self.name,"error P=",eP,"D=",eD,"E=",self.eI
		#do acceleration limiting
		if vcur != None:
			dvcmd = vectorops.div(vectorops.sub(vcmd,vcur),dt)
			dvnorm = vectorops.norm(dvcmd)
			if dvnorm > self.dvcmdmax:
				vcmd = vectorops.madd(vcur,dvcmd,self.dvcmdmax/dvnorm*dt)
				print self.name,"acceleration limited by factor",self.dvcmdmax/dvnorm*dt,"to",vcmd
		#do velocity limiting
		vnorm = vectorops.norm(vcmd)
		if vnorm > self.vcmdmax:
			vcmd = vectorops.mul(vcmd,self.vcmdmax/vnorm)
			print self.name,"velocity limited by factor",self.vcmdmax/vnorm,"to",vcmd
		return vcmd

	def advance(self, q, dq, dt):
		""" Updates internal state: accumulates iterm and updates x_last
		"""
		if self.weight > 0:
			eP = self.getSensedError(q)
			# update iterm
			if self.eI == None:
				self.eI = vectorops.mul(eP,dt)
			else:
				self.eI = vectorops.madd(self.eI, eP, dt)
			einorm = vectorops.norm(self.eI)
			if einorm > self.eImax:
				self.eI = vectorops.mul(self.eI,self.eImax/einorm)

		#update qLast
		self.qLast = q
		return

	@abstractmethod
	def getSensedValue(self, q):
		"""Gets task x from sensed configuration q.
		Subclasses MUST override this.
		"""
		return
	@abstractmethod
	def getJacobian(self, q):
		"""Gets Jacobian dx/dq(q).
		Subclasses MUST override this.
		"""
		return 

	def getSensedError(self, q):
		"""Returns x(q)-xdes where - is the task-space differencing operator"""
		return self.taskDifference(self.getSensedValue(q), self.xdes)
	def taskDifference(self,a,b):
		"""Default: assumes a Cartesian space"""
		return vectorops.sub(a,b)
	def getSensedVelocity(self, q, dq, dt):
		"""Gets task velocity from sensed configuration q.
		Default implementation uses finite differencing.  Other
		implementations may use jacobian.
		"""
		#uncomment this to get a jacobian based technique
		#return np.dot(self.getJacobian(q),dq)
		if self.qLast==None:
			return None
		else:
			xlast = self.getSensedValue(self.qLast)
			xcur = self.getSensedValue(q)
			return vectorops.div(self.taskDifference(xcur,xlast),dt)
	def drawGL(self,q):
		"""Optionally can be overridden to visualize the task in OpenGL."""
		pass

class COMTask(Task):
	""" Center of Mass position task subclass
	
	If baseLinkNo is supplied, the COM task is measured relative to
	the given base link.  Otherwise, it is measured in absolute
	coordinates.

	If activeDofs are supplied, the COM task is adjusted only with
	those dofs.
	"""
	def __init__(self, robot, baseLinkNo=-1):
		Task.__init__(self)
		self.robot = robot
		q = self.robot.getConfig()
		self.name = "CoM"
		self.baseLinkNo = baseLinkNo
		self.activeDofs = None
		
	def getSensedValue(self, q):
		"""Returns CoM position
		"""
		self.robot.setConfig(q)
		com = self.robot.getCom()
		if self.baseLinkNo >= 0:
			Tb = self.robot.link(self.baseLinkNo).getTransform()
			Tbinv = se3.inv(Tb)
			com = se3.apply(Tbinv,com)
		return com
		
	def getJacobian(self, q):
		"""Returns axis-weighted CoM Jacobian by averaging
		mass-weighted Jacobian of each link.
		"""
		self.robot.setConfig(q)
		numLinks = self.robot.numLinks()
		Jcom = np.array(self.robot.getComJacobian())
		#if relative positioning task, subtract out COM jacobian w.r.t. base
		if self.baseLinkNo >= 0:
			Tb = self.robot.link(self.baseLinkNo).getTransform()
			pb = se3.apply(se3.inv(Tb),self.robot.getCom())
			Jb = np.array(self.robot.link(self.baseLinkNo).getPositionJacobian(pb))
			Jcom -= Jb
		if self.activeDofs != None:
			Jcom = select_cols(Jcom,self.activeDofs)
		return Jcom

	def drawGL(self,q):
		from OpenGL import GL
		x = self.getSensedValue(q)
		xdes = self.xdes
		#invert the transformations from task to world coordinates
		if self.baseLinkNo >= 0:
			Tb = self.robot.link(self.baseLinkNo).getTransform()
			xdes = se3.apply(Tb,self.xdes)
			x = se3.apply(Tb,x)
		GL.glPointSize(10)
		GL.glEnable(GL.GL_POINT_SMOOTH)
		GL.glBegin(GL.GL_POINTS)
		GL.glColor3f(0,1,0)	#green 
		GL.glVertex3fv(xdes)
		GL.glColor3f(0,1,1)	#cyan
		GL.glVertex3fv(x)
		GL.glEnd()


class LinkTask(Task):
	"""Link position/orientation task subclass.
	Supports both absolute and relative positioning.
	"""
	def __init__(self, robot, linkNo, taskType, baseLinkNo=-1):
		"""Supply a robot and link number to control.  taskType can be:
		-'po' for position and orientation (in concordance with se3, the task variables are a matrix (R,t))
		-'position' for position only
		-'orientation' for orientation only.

		For po and position tasks the localPosition member can be set to
		control a specified point on the link.  The origin is assumed by
		default.

		If baseLinkNo is supplied, the values are treated as relative
		values as calculated with respect to a given base link.
		"""
		Task.__init__(self)
		self.linkNo = linkNo
		self.link = robot.link(self.linkNo)
		self.baseLinkNo = baseLinkNo
		self.baseLink = (None if baseLinkNo < 0 else robot.link(baseLinkNo))
		self.activeDofs = None
		self.robot = robot
		self.hP, self.hD, self.hI = -1, 0, 0
		self.localPosition=[0.,0., 0.]
		self.taskType = taskType
		self.name = "Link"

		if self.taskType == 'po' or self.taskType == 'position' or self.taskType == 'orientation':
			pass
		else:
			raise ValueError("Invalid taskType "+self.taskType)

	def getSensedValue(self, q):
		"""Get link x, which is rotation matrix and/or translation
		"""
		self.robot.setConfig(q)
		T = self.link.getTransform()
		#check if relative transform task, modify T to local transform
		if self.baseLinkNo >= 0:
			Tb = self.baseLink.getTransform()
			Tbinv = se3.inv(Tb)
			T = se3.mul(Tbinv,T)
		if self.taskType == 'po':
			x = (T[0],se3.apply(T,self.localPosition))
		elif self.taskType == 'position':
			x = se3.apply(T,self.localPosition)
		elif self.taskType == 'orientation':
			x = T[0]
		else:
			raise ValueError("Invalid taskType "+self.taskType)
		return x

	def taskDifference(self,a,b):
		if self.taskType == 'po':
			return se3.error(a,b)
		elif self.taskType == 'position':
			return vectorops.sub(a,b)
		elif self.taskType == 'orientation':
			return so3.error(a,b)
		else:
			raise ValueError("Invalid taskType "+self.taskType)

	def getJacobian(self, q):
		self.robot.setConfig(q)
		J = None
		if self.taskType == 'po':
			J = self.link.getJacobian(self.localPosition)
		elif self.taskType == 'position':
			J = self.link.getPositionJacobian(self.localPosition)
		elif self.taskType == 'orientation':
			J = self.link.getOrientationJacobian()
		else:
			raise ValueError("Invalid taskType "+self.taskType)
		J = np.array(J)
		#check if relative transform task, modify Jacobian accordingly
		if self.baseLinkNo >= 0:
			T = self.link.getTransform()
			Tb = self.baseLink.getTransform()
			Tbinv = se3.inv(Tb)
			pb = se3.apply(Tbinv,se3.apply(T,self.localPosition))
			if self.taskType == 'po':
				Jb = self.baseLink.getJacobian(pb)
			elif self.taskType == 'position':
				Jb = self.baseLink.getPositionJacobian(pb)
			elif self.taskType == 'orientation':
				Jb = self.baseLink.getOrientationJacobian()
			Jb = np.array(Jb)
			#subtract out jacobian w.r.t. baseLink
			J -= Jb
		if self.activeDofs!=None:
			J = select_cols(J,self.activeDofs)
		return J

	def drawGL(self,q):
		from OpenGL import GL
		x = self.getSensedValue(q)
		xdes = self.xdes
		if self.baseLinkNo >= 0:
			Tb = self.baseLink.getTransform()
			if self.taskType == "position":
				x = se3.apply(Tb,x)
				xdes = se3.apply(Tb,xdes)
			elif self.taskType == "po":
				x = se3.mul(Tb,x)
				xdes = se3.mul(Tb,xdes)
		GL.glPointSize(6)
		GL.glEnable(GL.GL_POINT_SMOOTH)
		GL.glBegin(GL.GL_POINTS)
		if self.taskType == "position":
			GL.glColor3f(1,0,0)	#red 
			GL.glVertex3fv(xdes)
			GL.glColor3f(1,0.5,0)	#orange
			GL.glVertex3fv(x)
		elif self.taskType == "po":
			GL.glColor3f(1,0,0)	#red 
			GL.glVertex3fv(xdes[1])
			GL.glColor3f(1,0.5,0)	#orange
			GL.glVertex3fv(x[1])
		GL.glEnd()


class JointTask(Task):
	"""A joint angle task class 
	"""
	def __init__(self, robot, jointIndices):
		Task.__init__(self)
		self.robot = robot
		self.jointIndices = jointIndices
		self.name = "Joint"
		pass

	def getSensedValue(self, q):
		return [q[jointi] for jointi in self.jointIndices]
	
	def getJacobian(self, q):
		J = sparse.lil_matrix((len(self.jointIndices),self.robot.numLinks()))
		for i,jointi in enumerate(self.jointIndices):
			J[i,jointi] = 1
		return J
	
		J = []
		for jointi in self.jointIndices:
			Ji = [0] * self.robot.numLinks()
			Ji[jointi] = 1
			J.append(Ji)
		return J

class JointLimitTask(Task):
	def __init__(self,robot):
		Task.__init__(self)
		self.robot = robot
		self.buffersize = 2.0
		self.qmin,self.qmax = robot.getJointLimits()
		self.accelMax = robot.getAccelerationLimits()
		self.wscale = 0.1
		self.maxw = 10
		self.active = []
		self.weight = 0
		self.xdes = []
		self.dxdes = []
		self.name = "Joint limits"
		self.setGains(-0.1,-0.2,0)
	
		
	def updateState(self, q, dq, dt):
		""" check (q, dq) against joint limits
		Activates joint limit constraint, i.e., add a joint task
		to avoid reaching limit, when surpassing a threshold.

		Or, increase weight on this joint task as joint gets closer to its limit.

		"""
		self.active = []
		self.weight = []
		self.xdes = []
		self.dxdes = []
		buffersize = self.buffersize
		wscale = self.wscale
		maxw = self.maxw
		for i,(j,dj,jmin,jmax,amax) in enumerate(zip(q,dq,self.qmin,self.qmax,self.accelMax)):
			if jmax <= jmin: continue
			jstop = j
			a = amax / buffersize
			w = 0
			ades = 0
			if dj > 0.0:
				t = dj / a
			        #j + t*dj - t^2*a/2
				jstop = j + t*dj - t*t*a*0.5
				if jstop > jmax:
					#add task to slow down
				        #perfect accel solves for:
				        #j+ dj^2 / 2a  = jmax
				        #dj^2 / 2(jmax-j)   = a
					if j >= jmax:
						print "Joint",self.robot.link(i).getName(),"exceeded max",j,">=",jmax
						ades = -amax
						w = maxw
					else:
						alim = dj*dj/(jmax-j)*0.5
						if alim > amax:
							ades = -amax
							w = maxw
						else:
							ades = -alim
							w = wscale*(alim-a)/(amax-alim)
						#print "Joint",self.robot.link(i).getName(),j,dj,"near upper limit",jmax,", desired accel:",ades," weight",w
			else:
				t = -dj / a
			        #j + t*dj + t^2*a/2
				jstop = j + t*dj + t*t*a*0.5
				if jstop < jmin:
					#add task to slow down
				        #perfect accel solves for:
				        #j - dj^2 / 2a  = jmin
				        #dj^2 / 2(j-jmin)   = a
					if j <= jmin:
						print "Joint",self.robot.link(i).getName(),"exceeded min",j,"<=",jmin
						ades = amax
						w = maxw
					else:
						alim = dj*dj/(j-jmin)*0.5
						if alim > amax:
							ades = amax
							w = maxw
						else:
							ades = alim
							w = wscale*(alim-a)/(amax-alim)
						#print "Joint",self.robot.link(i).getName(),j,dj,"near lower limit",jmin,", desired accel:",ades," weight",w
			if w > maxw:
				w = maxw
			self.active.append(i)
			self.xdes.append(max(jmin,min(jmax,j)))
			self.dxdes.append(dj+dt*ades)
			self.weight.append(w)
		if len(self.weight)==0:
			self.weight = 0
		return
	
	def getSensedValue(self, q):
		return [q[jointi] for jointi in self.active]
	
	def getJacobian(self, q):
		J = sparse.lil_matrix((len(self.active),self.robot.numLinks()))
		for i,jointi in enumerate(self.active):
			J[i,jointi] = 1
		return J

		J = []
		for jointi in self.active:
			Ji = [0] * self.robot.numLinks()
			Ji[jointi] = 1
			J.append(Ji)
		return J
