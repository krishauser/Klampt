from klampt.math import vectorops
import math
import numpy as np
from scipy import sparse
from leastsq_bounds import leastsq_bounds
from controller import BaseController
from system_id import LinearSystemID
from online_leastsq import OnlineLeastSquares
from sparse_linalg import spdot


class MotionModel(object):
    """Members inputs and outputs are labels for the input type u and the
    output type v"""
    def __init__(self,inputs="torque",outputs="accel"):
        self.inputs = inputs
        self.outputs = outputs
    def eval(self,q,dq,u):
        """Returns the output v = f(q,dq,u).  For forward models, u=torque
        and v=ddq,  and for inverse models u=ddq and v=torque.
        Inputs and outputs are assumed to be Numpy 1-D arrays."""
        raise NotImplementedError()
    def linearization(self,q,dq):
        """Returns a model (A,b) such that the output is approximated by
        v = A*u+b.  A can either be a Numpy 2D array or a Scipy sparse
        matrix."""
        raise NotImplementedError()
    def getInverse(self):
        """Returns the inverted motion model for which evaluation produces
        the inputs u that would produce output v: u = f^-1(q,dq,v). """
        return DefaultInverseMotionModel(self)


class DefaultInverseMotionModel(MotionModel):
    """Basic inverse motion model that uses a linear equation solve to
    evaluate the result."""
    def __init__(self,forward):
        self.forward = forward
        MotionModel.__init__(self,forward.outputs,forward.inputs)
    def eval(self,q,dq,u):
        (A,b) = self.forward.linearization(q,dq)
        if sparse.issparse(A):
            try:
                return sparse.linalg.spsolve(A,u-b)
            except sparse.linalg.LinAlgError:
                return sparse.linalg.lsqr(A,u-b)
        else:
            try:
                return np.linalg.solve(A,u-b)
            except np.linalg.LinAlgError:
                return np.linalg.lstsq(A,u-b)[0]
    def linearization(self,q,dq):
        (A,b) = self.forward.linearization(q,dq)
        if sparse.issparse(A):
            Ainv = sparse.linalg.inv(A)
        else:
            try:
                Ainv = np.linalg.inv(A)
            except np.linalg.LinAlgError:
                Ainv = np.linalg.pinv(A)
        return (Ainv,-spdot(Ainv,b))


class NaiveMotionModel(MotionModel):
    """An identity motion model that sets v = u"""
    def __init__(self,inputs='torque',outputs='accel'):
        MotionModel.__init__(self,inputs,outputs)
    def eval(self,q,dq,u):
        return u
    def linearization(self,q,dq):
        return (sparse.eye(len(dq),len(dq)),np.zeros((len(dq),)))
    def getInverse():
        return NaiveMotionModel(self.outname,self.inname)


class CompositeMotionModel(MotionModel):
    """Returns f(q,dq,g(q,dq,u))"""
    def __init__(self,f,g):
        self.f = f
        self.g = g
        MotionModel.__init__(self,self.g.inputs,self.f.outputs)
    def eval(self,q,dq,u):
        return self.f(q,dq,self.g(q,dq,u))
    def linearization(self,q,dq):
        Af,bf = self.f.linearization(q,dq)
        Ag,bg = self.g.linearization(q,dq)
        return (spdot(Af,Ag),bf+spdot(Af,bg))


class IntegratorMotionModel(MotionModel):
    """A motion model that integrates the output of another motion model
    by dt"""
    def __init__(self,derivativeModel,dt):
        self.derivativeModel = derivativeModel
        self.dt = dt
        outname = "integrated_"+derivativeModel.outputs
        if derivativeModel.outputs=='accel':
            outname = 'velocity'
        elif derivativeModel.outputs=='velocity':
            outname = 'position'
        MotionModel.__init__(self,derivativeModel.inputs,outname)
    def eval(self,q,dq,u):
        if self.derivativeModel.outputs=='accel':
            return dq + self.derivativeModel.eval(q,dq,u)*dt
        elif self.derivativeModel.outputs=='velocity':
            return q + self.derivativeModel.eval(q,dq,u)*dt
        else:
            raise NotImplementedError()
    def linearization(self,q,dq,dt):
        (A,b) = self.derivativeModel.linearization(q,dq)
        if self.derivativeModel.outputs=='accel':
            return (A*dt,dq+b*dt)
        elif self.derivativeModel.outputs=='velocity':
            return (A*dt,q+b*dt)
        else:
            raise NotImplementedError()


class FreeBaseRobotMotionModel(MotionModel):
    """A relatively naive motion model with a free base and some links
    in contact.  The constraint equations are solved for in a least-squares
    sense.  Assumes the first 6 dofs are the virtual links of the free base.

    Set the linksInContact member to a list of indices of the links in contact.

    If the constraintWeights member is set, it is assumed to be a
    list of 6*len(linksInContact) weights where each block of 6 consecutive
    numbers weights the translation and rotation components of the
    fixed-link constraint, respectively.  (TODO: weights not done yet)
    """
    def __init__(self,inname,outname,robot,linksInContact=[]):
        self.robot = robot
        self.linksInContact = linksInContact
        self.constraintWeights = None
        MotionModel.__init__(self,inname,outname)

    def eval(self,q,dq,u):
	if len(self.linksInContact)==0:
            return np.hstack((np.zeros(6),u[6:]))
        else:
            A,b = self.linearization(q,dq)
            return spdot(A,u)+b
        
    def linearization(self,q,dq):
        self.robot.setConfig(q)
  	#free-dof constrained motion model:
	b = np.zeros(len(dq))
	Aa = sparse.eye(len(dq)-6,len(dq)-6)
	Af = sparse.csr_matrix((len(dq),6))
	if len(self.linksInContact)==0:
            Aaf = sparse.csr_matrix((6,len(dq)-6))
	else:
            #solve for first 6 rows of A by keeping foot constraints fixed
            Jfs = []
            Jas = []
            for link in self.linksInContact:
                Jl = np.array(self.robot.link(link).getJacobian((0,0,0)))
                Jfs.append(Jl[:,0:6])
                Jas.append(Jl[:,6:])
            Jf = np.vstack(Jfs)
            Ja = np.vstack(Jas)
            #solve Jf * dqf + Ja * dqa = 0 in the least squares sense
            Jfinv = np.linalg.pinv(Jf)
            Aaf = -np.dot(Jfinv,Ja)
	#return matrix
	#[   |Aaf]
        #[Af |___]
	#[   |Aa ]
	#[   |   ]
	return sparse.hstack([Af,sparse.vstack([Aaf,Aa])]),b


class RobotDynamicsMotionModel(MotionModel):
    """The basic Langrangian motion model, suitable for fully actuated robots.
    B(q)*ddq + C(q,dq) + G(q) = u + fext.
    
    By default performs gravity calculation (fext = -G(q)) but can also take
    other forces and loads."""
    def __init__(self,robotModel,gravity=(0,0,-9.8)):
        self.robotModel = robotModel
        self.gravity = gravity
        MotionModel.__init__(self)
    
    def externalForces(self,q,dq):
        """Subclasses can override this to generate other external forces besides
        gravity.  Assumes the robot model is updated"""
        return -np.array(self.robotModel.getGravityForces(self.gravity))
    
    def eval(self,q,dq,u):
        self.robotModel.setConfig(q.tolist())
        self.robotModel.setVelocity(dq.tolist())
        f = self.externalForces()
        return self.robotModel.accelFromTorques((u+f).tolist())
    def linearization(self,q,dq):
        self.robotModel.setConfig(q.tolist())
        self.robotModel.setVelocity(dq.tolist())
        f = self.externalForces()
        C = np.array(self.robotModel.getCoriolisForces())
        minv = np.array(self.robotModel.getMassMatrixInv())
        return (minv,np.dot(minv,C-f))
    def getInverse(self):
        return RobotInverseDynamicsMotionModel(self)


class RobotInverseDynamicsMotionModel(MotionModel):
    def __init__(self,forward):
        self.forward = forward
        MotionModel.__init__(self,forward.outputs,forward.inputs)
    def eval(self,q,dq,ddq):
        robotModel = self.forward.robotModel
        robotModel.setConfig(q.tolist())
        robotModel.setVelocity(dq.tolist())
        f = self.forward.externalForces()
        return np.array(robotModel.torquesFromAccel(ddq.tolist()))-f
    def linearization(self,q,dq):
        robotModel = self.forward.robotModel
        robotModel.setConfig(q.tolist())
        robotModel.setVelocity(dq.tolist())
        f = self.forward.externalForces()
        C = np.array(robotModel.getCoriolisForces())
        m = np.array(robotModel.getMassMatrix())
        return (m,C-f)


class AdaptiveMotionModel(MotionModel):
    """A motion model that performs adaptive estimation.
    By default it estimates each joint independently using a linear second
    order system: A[i]*(q[i],dq[i])+b[i]*(dqcmd[i])+c[i], i=1,...,n.
    """
    def __init__(self,inname,outname):
        MotionModel.__init__(self,inname,outname)
        #TODO: other outputs besides velocity
        assert (outname == 'velocity')
        assert (inname == 'velocity')
        self.sysids = None
    def init(self,n,dt=0):
        self.sysids = [LinearSystemID(2,1) for i in xrange(n)]
        for i,s in enumerate(self.sysids):
            dt = 0
            #default motion model: basic integrator of velocity cmds
            s.setModelPrior(np.array([[1,dt],[0,0]]),np.array([[0],[1]]),np.zeros(2),10)
    def eval(self,q,dq,u):
        if self.sysids == None:
            self.init(len(q))
        outindex = 1
        v = np.array([s.getOutput([qi,dqi],[ui])[outindex] for (s,qi,dqi,ui) in zip(self.sysids,q,dq,u)])
        return v
    def linearization(self,q,dq):
        if self.sysids == None:
            self.init(len(q))
        outindex = 1
        A = sparse.lil_matrix((len(q),len(q)))
        b = np.zeros(len(q))
        for i,s in enumerate(self.sysids):
            As,Bs,Cs = s.getModel()
            A[i,i] = Bs[outindex,0]
            b[i] = np.dot(As[outindex,:],[q[i],dq[i]])+Cs[outindex]
        return (A,b)
    def add(self,q,dq,u,qnext,dqnext):
        n= len(q)
        if self.sysids == None:
            self.init(n)
        for i in xrange(n):
            #TODO: add discount as a parameter?
            self.sysids[i].discount(0.1,'hyperbolic')
            self.sysids[i].add([q[i],dq[i]],[u[i]],[qnext[i],dqnext[i]])
        return


class GravityCompensationAdaptiveMotionModel(MotionModel):
    """A motion model that performs adaptive estimation with a gravity
    compensation term.
    It estimates each joint independently using a linear system
    dq[i] = c1[i]*dqcmd[i]+cg[i]*G[i] + c0[i], i=1,...,n.
    where dqcmd is the velocity command and G is the gravity torques
    """
    def __init__(self,robot,inname,outname):
        MotionModel.__init__(self,inname,outname)
        #TODO: other outputs besides velocity
        assert (outname == 'velocity')
        assert (inname == 'velocity')
        self.robot = robot
        self.gravity = (0,0,-9.8)
        self.estimators = None
    def init(self,n,dt=0):
        self.estimators = [OnlineLeastSquares(4) for i in xrange(n)]
        for i,e in enumerate(self.estimators):
            #default motion model: dq = 0.8*dq + 0.2*dqcmd
            e.setPrior([0.8,0.3,0,0],1)
    def eval(self,q,dq,u):
        if self.estimators == None:
            self.init(len(q))
        self.robot.setConfig(q)
        G = self.robot.getGravityForces(self.gravity)
        v = np.array([np.dot(e.x,[dqi,ui,Gi,1.0]) for (e,dqi,ui,Gi) in zip(self.esimators,dq,u,G)])
        return v
    def linearization(self,q,dq):
        if self.estimators == None:
            self.init(len(q))
        A = sparse.lil_matrix((len(q),len(q)))
        b = np.zeros(len(q))
        self.robot.setConfig(q)
        G = self.robot.getGravityForces(self.gravity)
        for i,e in enumerate(self.estimators):
            cdq,cu,cG,c0 = e.x
            sdq,su,sG,s0 = e.solutionStandardErrors()
            A[i,i] = cu
            b[i] = cdq*dq[i]+cG*G[i]+c0
            sA = su
            sb = math.sqrt((sdq*dq[i])**2+(sG*G[i])**2 + s0**2)
            if i==30:
                print "Linearization",A[i,i],b[i]
                print "Standard errors",sA,sb
        return (A,b)
    def add(self,q,dq,u,qnext,dqnext):
        n= len(q)
        if self.estimators == None:
            self.init(n)
        self.robot.setConfig(q)
        G = self.robot.getGravityForces(self.gravity)
        for i,e in enumerate(self.estimators):
            #TODO: add discount as a parameter?
            x = [dq[i],u[i],G[i],1.0]
            if i==30:
                print "Reading",x,dqnext[i]
                print "Old coeffs",i,e.x
                print "Old residual",np.dot(e.x,x)-dqnext[i]
            e.discount(0.1,'hyperbolic')
            e.add(x,dqnext[i])
            if i==30:
                print "Coeffs",i,e.x
                print "Residual",np.dot(e.x,x)-dqnext[i]
                #print e.AtA
                #print e.AtAinv
                #print e.Atb
        return


class FreeBaseAdaptiveMotionModel(AdaptiveMotionModel):
    """A motion model that performs adaptive estimation for a free-base
    robot.  It estimates all joints independently, and estimates the effect
    of all joint velocities on the base velocity.
    """
    def __init__(self,inname,outname,relevantDofs=None,robot=None):
        AdaptiveMotionModel.__init__(self,inname,outname)
        self.baseSysID = None
        self.relevantDofs = relevantDofs
        self.robot = None
    def init(self,n,dt=0):
        AdaptiveMotionModel.init(self,n,dt)
        #baseSysID takes base rotations, joint velocities, joint commands, and constant offset
        numJoints = len(self.relevantDofs) if self.relevantDofs != None else n-6
        self.baseSysID = [OnlineLeastSquares(3+numJoints+numJoints+1) for i in xrange(6)]
    def getQDofs(self,q):
        return q[3:6]
    def getDqDofs(self,dq):
        if self.relevantDofs == None:
            return dq[6:]
        else:
            return [dq[i] for i in self.relevantDofs]
    def getUDofs(self,u):
        if self.relevantDofs == None:
            return u[6:]
        else:
            return [u[i] for i in self.relevantDofs]
        
    def eval(self,q,dq,u):
        v = AdaptiveMotionModel.eval(self,q,dq,u)
        n = len(q)
        xbase = np.hstack((self.getQDofs(q),self.getDqDofs(dq),self.getUDofs(u),[1.0]))
        for i in xrange(6):
            v[i] = self.baseSysID[i].x.dot(xbase)
        return v
    def linearization(self,q,dq):
        A,b = AdaptiveMotionModel.linearization(self,q,dq)
        #now fill in top right corner of A, first 6 rows of b
        n = len(q)
        numJoints = len(self.relevantDofs) if self.relevantDofs != None else n-6
        for i in xrange(6):
            A[i,i]=0
            coeffs = self.baseSysID[i].x
            #unpack
            qCoeffs = coeffs[:3]
            dqCoeffs = coeffs[3:3+numJoints]
            uCoeffs = coeffs[3+numJoints:3+numJoints+numJoints]
            constCoeff = coeffs[3+numJoints+numJoints]
            if self.relevantDofs == None:
                A[i,6:n] = uCoeffs
            else:
                for cd,d in zip(uCoeffs,self.relevantDofs):
                    A[i,d] = cd
            b[i] = np.dot(dqCoeffs,self.getDqDofs(dq))+np.dot(qCoeffs,self.getQDofs(q))+constCoeff
            #print uCoeffs,b[i]
        return A,b
    def add(self,q,dq,u,qnext,dqnext):
        AdaptiveMotionModel.add(self,q,dq,u,qnext,dqnext)
        n = len(q)
        xbase = np.hstack((self.getQDofs(q),self.getDqDofs(dq),self.getUDofs(u),[1.0]))
        for i in xrange(6):
            if self.baseSysID[i].count > 10:
                self.baseSysID[i].discount(0.1,'hyperbolic')
            self.baseSysID[i].add(xbase,dq[i])
        return


def clamp(x,a,b):
    return a if x < a else (b if x > b else x)

class BoundedMotionModel(MotionModel):
    """A Langrangian motion model for underactuated robots. Forward dynamics
    takes into account torque limits by capping inputs to their limits."""
    def __init__(self,model,umin,umax):
        self.model = model
        self.umin,self.umax = umin,umax
        self.boundsweight = 100
        MotionModel.__init__(self,model.inputs,model.outputs)

    def clampToLimits(self,x):
        """Returns a copy of x but clamped by the torque limits."""
        res = type(x)([clamp(xi,tmini,tmaxi) for tmini,tmaxi,xi in zip(self.umin,self.umax,x)])
        return res

    def inLimits(self,x):
        return all([tmini <= xi <= tmaxi for tmini,tmaxi,xi in zip(self.umin,self.umax,x)])
    
    def eval(self,q,dq,u):
        return self.model.accel(q,dq,self.clampToLimits(u))


    """
    def inverseDynamics(self,q,dq,ddq):
        # min_u ||ddq - accel(u)||^2 s.t. umin <= u <= umax
        u0 = self.model.inverseDynamics(q,dq,ddq)
        if self.inLimits(u0): return u0
        # least squares problem
        (A,b) = self.model.accelLinearization(q,dq)
        return leastsq_bounds(lambda(u):spdot(A,u)+b-ddq,
                              u0,
                              zip(self.umin,self.umax),
                              Dfun=lambda(u):A,boundsweight=self.boundsweight)
    """



class ConstrainedMotionModel:
    """A Langrangian motion model for constrained / underactuated robots.
    Freedofs are constrained to have zero torque.  Inverse
    dynamics are given by solving an equality-constrained least squares problem. """
    def __init__(self,model,freeDofs):
        self.model = model
        self.freeDofs = freeDofs
        self.actuatedDofs = [i for i in range(model.robotModel.numLinks()) if i not in freeDofs]

    def constraintEquation(self,q,dq):
        """Returns (C,d) for constraint C*ddq = d"""
        raise NotImplementedError()
    
    def inverseDynamics(self,q,dq,ddq):
        # least squares problem min ||A*u+b-ddq||^2 s.t. u[freedofs] = 0, C*(A*u+b)=d
        (A,b) = self.model.accelLinearization(q,dq)
        (C,d) = self.constraintEquation(q,dq)
        Aactuated = np.vstack([A[:,i] for i in self.actuatedDofs])
        # now least squares problem on actuated dofs is
        # min||Aa*ua+b-ddq||^2 s.t. C*(Aa*ua+b)=d
        Cinv = np.linalg.pinv(C)
        np.dot(Cinv,d)
		#fixed-feet motion model
		#If forces f are applied such that C(q)=0, we need J(q)dq'' = 0
		#J q'' = J B^-1*(t+J^T*f-C-G) = 0
		#J B^-1*J^T*f = - J B^-1*(t-C-G)
		#f = -(J B^-1*J^T)^-1 J B^-1*(t-C-G)
		#q'' = B^-1*(t-C-G-J^T*(J B^-1*J^T)^-1 J B^-1*(t-C-G))
		#    = (B^-1 - B^-1 J^T*(J B^-1*J^T)^-1 J B^-1)*(t-C-G)


class DebugMotionModelController(BaseController):
    def __init__(self,model,robot=None,dofs=None):
        self.model = model
        self.robot = robot
        self.activeDofs = None
        assert (model.inputs=="velocity" and model.outputs=="velocity"),"Can only debug velocity models at the moment"
        self.dqpredlast = None
    
    def output_and_advance(self,**inputs):
        try:
            q = inputs['q']
            dq = inputs['dq']
            u = vectorops.div(vectorops.sub(inputs['qcmd'],q),inputs['dt'])
        except KeyError:
            print "Warning, cannot debug motion model, dq or dqcmd not in input"
            return None
        if self.dqpredlast != None:
            if self.activeDofs != None:
                dq = dq[:]
                for i in [i for i in range(len(q)) if i not in self.activeDofs]:
                    dq[i] = self.dqpredlast[i]
            #compare motion model to dq
            print "Motion model error:",np.linalg.norm(self.dqpredlast - np.array(dq))
            (v,i) = max(zip(np.abs(self.dqpredlast - np.array(dq)).tolist(),range(len(dq))))
            print "  Max error:",v,"at",i,
            if self.robot!=None: print self.robot.link(i).getName()
            else: print
            print "  Command:",self.ulast[i],"Predicted:",self.dqpredlast[i],"Actual:",dq[i]
            print "  pred:",self.Alast[i,i],"*u +",self.blast[i]
            #print "  Predicted:",self.dqpredlast
            #print "  Actual:",dq
        A,b = self.model.linearization(q,dq)
        self.dqpredlast = A.dot(u)+b
        self.ulast = u
        self.Alast,self.blast = A,b
        return None
