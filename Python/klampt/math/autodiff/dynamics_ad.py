"""Klampt dynamics AD functions:

 ====================  =============  ======================================
 Function              Derivative     Notes
 ====================  =============  ======================================
 CenterOfMass          1              com[robot](q) -> R^3
 GravityVector         1              G[robot](q,g) -> R^n
 KineticEnergy         1              KE[robot](q,dq) -> R
 MassMatrix            1              vec(B)[robot](q) -> R^(n*n)
 MassMatrixInv         N              vec(B^-1)[robot](q) -> R^(n*n)
 MomentumVector        N              B*v[robot](q,v) -> R^n
 CoriolisVector        N              C[robot](q,dq) -> R^n
 ForwardDynamics       N              ddq(q,dq,t) -> R^n
 InverseDynamics       N              t(q,dq,ddq) -> R^n
 PointForceTorques     1              J^T*f[link,localpt](q,f) -> R^n
 PointWrenchTorques    1              J^T*w[link,localpt](q,w) -> R^n
 FullDynamics          N              ddq[robot,g,links,localpoints](q,dq,t,fs) -> R^n
 ====================  =============  ======================================

"""


import numpy as np 
from .ad import ADFunctionInterface,ADFunctionCall,ADTerminal,sum_
from . import math_ad,so3_ad,se3_ad
from .. import vectorops,so3,se3
from ...robotsim import RobotModel,RobotModelLink

class CenterOfMass(ADFunctionInterface):
    """Autodiff wrapper of RobotModel.getCom()."""
    def __init__(self,robot):
        self.robot = robot
    def __str__(self):
        return "dynamics.CenterOfMass[%s]"%(self.robot.getName())
    def n_args(self):
        return 1
    def n_in(self,arg):
        return self.robot.numLinks()
    def n_out(self):
        return 3
    def eval(self,q):
        self.robot.setConfig(q.tolist())
        return np.array(self.robot.getCom())
    def derivative(self,arg,q):
        assert arg == 0
        self.robot.setConfig(q.tolist())
        return np.array(self.robot.getComJacobian())
    def jvp(self,arg,darg,q):
        assert arg == 0
        self.robot.setConfig(q.tolist())
        self.robot.setVelocity(darg.tolist())
        return np.array(self.robot.getComVelocity())


class GravityVector(ADFunctionInterface):
    """Autodiff wrapper of RobotModel.getGravityForces()."""
    def __init__(self,robot):
        self.robot = robot
    def __str__(self):
        return "dynamics.GravityVector[%s]"%(self.robot.getName())
    def n_args(self):
        return 2
    def n_in(self,arg):
        if arg == 0:
            return self.robot.numLinks()
        else:
            return 3
    def n_out(self):
        return self.robot.numLinks()
    def eval(self,q,g):
        self.robot.setConfig(q.tolist())
        return np.array(self.robot.getGravityForces(g.tolist()))
    def derivative(self,arg,q,g):
        if arg == 0:
            raise NotImplementedError()
        else:
            self.robot.setConfig(q.tolist())
            return np.array(self.robot.getComJacobian()).T
    def jvp(self,arg,darg,q,g):
        if arg == 0:
            #TODO: get all the link COM Hessians
            raise NotImplementedError()
        else:
            raise NotImplementedError()

class KineticEnergy(ADFunctionInterface):
    """Autodiff wrapper of RobotModel.getKineticEnergy()."""
    def __init__(self,robot):
        self.robot = robot
    def __str__(self):
        return "dynamics.KineticEnergy[%s]"%(self.robot.getName())
    def n_args(self):
        return 2
    def n_in(self,arg):
        return self.robot.numLinks()
    def n_out(self):
        return 1
    def eval(self,q,dq):
        self.robot.setConfig(q.tolist())
        self.robot.setVelocity(dq.tolist())
        return self.robot.getKineticEnergy()
    def derivative(self,arg,q,dq):
        if arg == 0:
            self.robot.setConfig(q.tolist())
            res = []
            for i in range(q.shape[0]):
                dB_dqi = np.array(self.robot.getMassMatrixDeriv(i))
                res.append(0.5*np.dot(dq,dB_dqi.dot(dq)))
            return np.array(res)[np.newaxis,:]
        else:
            self.robot.setConfig(q.tolist())
            return np.dot(np.array(self.robot.getMassMatrix()),dq)


class MassMatrix(ADFunctionInterface):
    """Autodiff wrapper of RobotModel.getMassMatrix(). The result is flattened
    into a 1D array of length n^2.
    """
    def __init__(self,robot):
        self.robot = robot
    def __str__(self):
        return "dynamics.MassMatrix[%s]"%(self.robot.getName())
    def n_args(self):
        return 1
    def n_in(self,arg):
        return self.robot.numLinks()
    def n_out(self):
        return self.robot.numLinks()**2
    def eval(self,q):
        self.robot.setConfig(q.tolist())
        return np.array(self.robot.getMassMatrix()).flatten()
    def derivative(self,arg,q):
        self.robot.setConfig(q.tolist())
        cols = []
        for i in range(q.shape[0]):
            cols.append(np.array(self.robot.getMassMatrixDeriv(i)).flatten())
        return np.column_stack(cols)


class MassMatrixInv(ADFunctionInterface):
    """Autodiff wrapper of RobotModel.getMassMatrixInv(). The result is 
    flattened into a 1D array of length n^2.
    """
    def __init__(self,robot):
        self.robot = robot
    def __str__(self):
        return "dynamics.MassMatrixInv[%s]"%(self.robot.getName())
    def n_args(self):
        return 1
    def n_in(self,arg):
        return self.robot.numLinks()
    def n_out(self):
        return self.robot.numLinks()**2
    def eval(self,q):
        self.robot.setConfig(q.tolist())
        return np.array(self.robot.getMassMatrixInv()).flatten()


class MomentumVector(ADFunctionInterface):
    """Autodiff function to compute B(q)*v as a function of (q,v) where B(q)
    is the mass matrix."""
    def __init__(self,robot):
        self.robot = robot
    def __str__(self):
        return "dynamics.MomentumVector[%s]"%(self.robot.getName())
    def n_args(self):
        return 2
    def n_in(self,arg):
        return self.robot.numLinks()
    def n_out(self):
        return self.robot.numLinks()
    def eval(self,q,v):
        self.robot.setConfig(q.tolist())
        return np.dot(np.array(self.robot.getMassMatrix()),v)
    def derivative(self,arg,q,v):
        if arg == 0:
            self.robot.setConfig(q.tolist())
            res = []
            for i in range(q.shape[0]):
                dB_dqi = np.array(self.robot.getMassMatrixDeriv(i))
                res.append(dB_dqi.dot(v))
            return np.column_stack(res)
        else:
            self.robot.setConfig(q.tolist())
            return np.array(self.robot.getMassMatrix())


class CoriolisVector(ADFunctionInterface):
    """Autodiff wrapper of RobotModel.getCorolisForces()."""
    def __init__(self,robot):
        self.robot = robot
    def __str__(self):
        return "dynamics.CoriolisVector[%s]"%(self.robot.getName())
    def n_args(self):
        return 2
    def n_in(self,arg):
        return self.robot.numLinks()
    def n_out(self):
        return self.robot.numLinks()
    def eval(self,q,dq):
        self.robot.setConfig(q.tolist())
        self.robot.setVelocity(dq.tolist())
        return self.robot.getCoriolisForces()


class ForwardDynamics(ADFunctionInterface):
    """Autodiff function to compute the forward dynamics ddq = f(q,dq,t).

    Note: the torque vector is the full torque vector of length
    robot.numLinks(). If you want to convert from a driver torque vector, use
    :class:`kinematics_ad.DriverDerivsToLinks`.
    """
    def __init__(self,robot):
        self.robot = robot
    def __str__(self):
        return "dynamics.ForwardDynamics[%s]"%(self.robot.getName())
    def n_args(self):
        return 3
    def n_in(self,arg):
        return self.robot.numLinks()
    def n_out(self):
        return self.robot.numLinks()
    def argname(self,arg):
        return ['q','dq','t'][arg]
    def eval(self,q,dq,t):
        self.robot.setConfig(q.tolist())
        self.robot.setVelocity(dq.tolist())
        return self.robot.torquesToAccel(t)


class InverseDynamics(ADFunctionInterface):
    """Autodiff function to compute the inverse dynamics t = f(q,dq,ddq).

    Note: the torque vector is the full torque vector of length
    robot.numLinks(). If you want to convert to a driver torque vector, use
    :class:`kinematics_ad.LinkDerivsToDrivers`.
    """
    def __init__(self,robot):
        self.robot = robot
    def __str__(self):
        return "dynamics.InverseDynamics[%s]"%(self.robot.getName())
    def n_args(self):
        return 3
    def n_in(self,arg):
        return self.robot.numLinks()
    def n_out(self):
        return self.robot.numLinks()
    def argname(self,i):
        return ['q','dq','ddq'][i]
    def eval(self,q,dq,ddq):
        self.robot.setConfig(q.tolist())
        self.robot.setVelocity(dq.tolist())
        return self.robot.accelToTorques(ddq)


class PointForceTorques(ADFunctionInterface):
    """Calculates the robot DOF torques as a function of the force f
    in R^3 on a given point ``localpt`` on link ``link``.
    """
    def __init__(self,link,localpt):
        self.robot = link.robot()
        self.link = link
        self.localpt = localpt
    def __str__(self):
        return "dynamics.PointForceTorques[%s,%s]"%(self.link.getName(),str(self.localpt))
    def n_args(self):
        return 2
    def n_in(self,arg):
        if arg==0:
            return self.robot.numLinks()
        else:
            return 3
    def n_out(self):
        return self.robot.numLinks()
    def eval(self,q,f):
        self.robot.setConfig(q.tolist())
        return np.dot(np.array(self.link.getPositionJacobian(self.localpt)).T,f)
    def jvp(self,arg,darg,q,f):
        self.robot.setConfig(q.tolist())
        if arg==0:
            H = np.array(self.link.getPositionHessian(self.localpt)) #tensor of size 3 x n x n
            return np.dot(f,np.dot(H,darg))
        else:
            return np.dot(np.array(self.link.getPositionJacobian(self.localpt)).T,darg)


class PointWrenchTorques(ADFunctionInterface):
    """Calculates the robot DOF torques as a function of the wrench w=(tau,f) 
    in R^6 on a given point ``localpt`` on link ``link``.
    """
    def __init__(self,link,localpt):
        self.robot = link.robot()
        self.link = link
        self.localpt = localpt
    def __str__(self):
        return "dynamics.PointWrenchTorques[%s,%s]"%(self.link.getName(),str(self.localpt))
    def n_args(self):
        return 2
    def n_in(self,arg):
        if arg==0:
            return self.robot.numLinks()
        else:
            return 6
    def n_out(self):
        return self.robot.numLinks()
    def eval(self,q,w):
        self.robot.setConfig(q.tolist())
        return np.dot(np.array(self.link.getJacobian(self.localpt)).T,w)
    def jvp(self,arg,darg,q,w):
        self.robot.setConfig(q.tolist())
        if arg==0:
            Ho = np.array(self.link.getOrientationHessian()) #tensor of size 3 x n x n
            Hp = np.array(self.link.getPositionHessian(self.localpt)) #tensor of size 3 x n x n
            return np.dot(w[:3],np.dot(Ho,darg[:3]))+np.dot(w[3:],np.dot(Hp,darg[3:]))
        else:
            return np.dot(np.array(self.link.getJacobian(self.localpt)).T,darg)
        

class FullDynamics(ADFunctionInterface):
    """Autodiff function that computes the standard forward dynamics equation:

    :math:`\ddot{q} = B(q)^{-1} (S \tau + sum_i J_i(q)^T f_i - C(q,\dot{q}) - G(q))`

    as a function of (q,dq,t,f).  Here, B is the mass matrix, t=:math:`\tau`
    are the actuated joint torques, S is a selection matrix, the J's are the 
    Jacobians of the contact points, C is the coriolis vector, and G is the
    gravity vector corresponding to the external force g.

    Note: the torque vector is the reduced torque vector of length
    robot.numDrivers().
    """
    def __init__(self,robot,g,links=None,localpts =None):
        self.robot = robot
        self.g = g
        self.links = links if links is not None else []
        self.localpts = localpts if localpts is not None else []
        assert len(self.links) == len(self.localpts),"must provide the same number of links and contacts"
    def __str__(self):
        return "dynamics.FullDynamics[%s,%s,%d contacts]"%(self.robot.getName(),str(self.g),len(self.links))
    def n_args(self):
        if len(self.links) == 0:
            return 3
        return 4
    def n_in(self,arg):
        if arg <= 1:
            return self.robot.numLinks()
        elif arg == 2:
            return self.robot.numDrivers()
        else:
            return 3*len(self.links)
    def n_out(self):
        return self.robot.numLinks()
    def argname(self,i):
        return ['q','dq','t','f'][i]
    def eval(self,q,dq,t,*f):
        self.robot.setConfig(q.tolist())
        self.robot.setVelocity(dq.tolist())
        G = self.robot.getGravityForces(self.g)
        tfull = np.zeros(self.robot.numLinks())
        for i in range(self.robot.numDrivers()):
            links = self.robot.driver(i).getAffectedLinks()
            if len(links) > 1:
                scale,offset = self.robot.driver(i).getAffineCoeffs()
                for j,v in zip(links,scale):
                    tfull[j] = v
            else:
                tfull[links[0]] = 1
        t - G
        if len(f) > 0:
            for i,(link,pt) in enumerate(zip(self.links,self.localpts)):
                t += np.dot(np.array(link.getPositionJacobian(pt)).T,f[i*3,i*3+3])
        return self.robot.torquesToAccel(t)

