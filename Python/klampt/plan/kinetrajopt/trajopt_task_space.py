"""Defines task space constraints for use in
:class:`~klampt.plan.kinetrajopt.KineTrajOpt`. 
"""
import numpy as np

from klampt import WorldModel
import klampt.math.so3 as so3

from .utils import CostInterface, ConstrInterface, MaskedRobot




class PoseConstraint(ConstrInterface):
    """Yet another way to impose pose constraint from the moment perspective"""
    def __init__(self, wrobot : MaskedRobot, linkid : int, target_pose):
        assert isinstance(wrobot, MaskedRobot)
        R, t = target_pose
        self.p0 = np.array(t)
        self.target_R = R
        self.robot = wrobot
        self.link = wrobot.robot.link(linkid)
        self.linkid = linkid

    def compute(self, x, grad_level=0):
        self.robot.setConfig(x)
        # p0 = self.link.getWorldPosition([0, 0, 0])
        R, p0 = self.link.getTransform()
        moment = so3.error(R, self.target_R)
        # for point x
        val = np.concatenate((np.array(p0) - self.p0, moment))
        if grad_level == 1:
            rel_R = so3.from_moment(moment)
            J0 = self.robot.positionJacobian(self.linkid, lcl_pos=[0, 0, 0])
            J1 = self.robot.orientationJacobian(self.linkid)
            m_omega = np.array([MomentDerivative(moment, rel_R, [1, 0, 0]), MomentDerivative(moment, rel_R, [0, 1, 0]), MomentDerivative(moment, rel_R, [0, 0, 1])]).T
            jac = np.r_[J0, m_omega.dot(J1)]
            return (val, jac)
        return (val,)


def uncross(R):
    return [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]


def Sinc(x):
    small=1e-7
    if np.abs(x) < small:
        c = [1.0,-1.0/6.0,1.0/120.0,-1.0/5040.0,1.0/362880.0]
        x2= x * x
        return c[0]+x2*(c[1]+x2*(c[2]+x2*(c[3]+x2*c[4])))
    else:
        return np.sin(x) / x


def Sinc_Dx(x):
    small=1e-4
    if np.abs(x) < small:
        c = [-2.0/6.0,4.0/120.0,-6.0/5040.0,8.0/362880.0]
        x2=x*x
        return x*(c[0]+x2*(c[1]+x2*(c[2]+x2*c[3])))
    else:
        return np.cos(x)/x-np.sin(x)/(x*x)


def MomentDerivative(m, R, z):
    """Compute the derivative of rotation vector given the value, rotation, and angular velocity

    Args:
        m (arr): the rotation vector
        R (arr): the rotation matrix
        z (arr): the angular velocity

    Returns:
        dm ([arr]): the dereivative
    """
    angleEps = 1e-3
    Half = 0.5
    npR = np.array(R).reshape((3, 3), order='F')  # since it is column major
    trace = np.trace(npR)
    theta = np.arccos((trace - 1) / 2)
    if np.abs(theta) < angleEps:
        return z
    # compute matrix derivative
    dR = np.zeros((3, 3), order='F')
    for i in range(3):
        dR[:, i] = np.cross(z, npR[:, i])
    vdR = uncross(dR)
    tracedR = np.trace(dR)
    if np.abs(theta - np.pi) < angleEps:
        dmx = np.pi*0.25/np.sqrt((npR[0, 0]+1)*Half)*dR[0, 0]
        dmy = np.pi*0.25/np.sqrt((npR[1, 1]+1)*Half)*dR[1, 1]
        dmz = np.pi*0.25/np.sqrt((npR[2, 2]+1)*Half)*dR[2, 2]
        return [dmx, dmy, dmz]
    sinctheta = Sinc(theta)
    dsinctheta = Sinc_Dx(theta)
    scale = Half / sinctheta
    gamma2 = dsinctheta/sinctheta/(2*np.sin(theta))
    dm = [dm_ * tracedR * gamma2 + scale * vdR_ for dm_, vdR_ in zip(m, vdR)]
    # dm.inplaceMul(tracedR*gamma2);
    # dm.madd(vdR,scale);
    return dm


class DirectionConstraint(ConstrInterface):
    """Constraints to keep one direction up"""
    def __init__(self, wrobot : MaskedRobot, linkid : int, lcl_dir=[0, 0, 1], world_dir=[0, 0, 1]):
        assert isinstance(wrobot, MaskedRobot)
        self.linkid = linkid
        self.robot = wrobot
        self.link = self.robot.link(linkid)
        self.lcl_dir = lcl_dir
        self.world_dir = np.array(world_dir)

    def compute(self, x, grad_level=0):
        self.robot.setConfig(x)
        p0 = self.link.getWorldPosition([0, 0, 0])
        p1 = self.link.getWorldPosition(self.lcl_dir)
        # val = np.array([p0[0] - p1[0], p0[1] - p1[1]])
        val = np.dot(self.world_dir, np.subtract(p1, p0))
        if grad_level == 1:
            J0 = self.robot.positionJacobian(self.linkid, lcl_pos=[0, 0, 0])
            J1 = self.robot.positionJacobian(self.linkid, lcl_pos=self.lcl_dir)
            Jac = np.dot(self.world_dir[None, :], J1 - J0)
            return (val, Jac)
        return (val,)


class OrientationConstraint(ConstrInterface):
    """Keeps a link at some orientation"""
    def __init__(self, robot : MaskedRobot, linkid : int, R):
        self.robot = robot
        self.linkid = linkid
        self.link = self.robot.link(linkid)
        self.target_R = R

    def compute(self, x, grad_level=0):
        self.robot.setConfig(x)
        R, _ = self.link.getTransform()
        moment = so3.error(R, self.target_R)
        # for point x
        val = np.array(moment)
        if grad_level == 1:
            rel_R = so3.from_moment(moment)
            J1 = self.robot.orientationJacobian(self.linkid)
            m_omega = np.array([MomentDerivative(moment, rel_R, [1, 0, 0]), MomentDerivative(moment, rel_R, [0, 1, 0]), MomentDerivative(moment, rel_R, [0, 0, 1])]).T
            jac = m_omega.dot(J1)
            return (val, jac)
        return (val,)


class PositionConstraint(ConstrInterface):
    """To constrain the position of a link local position"""
    def __init__(self, wrobot : MaskedRobot, linkid : int, lcl_pos : list, world_pos : list):
        assert isinstance(wrobot, MaskedRobot)
        self.robot = wrobot
        self.linkid = linkid
        self.lcl_pos = lcl_pos
        self.world_pos = world_pos
        self.link = wrobot.link(linkid)

    def compute(self, x, grad_level=0):
        self.robot.setConfig(x)
        wpos = self.link.getWorldPosition(self.lcl_pos)
        val = np.subtract(wpos, self.world_pos)
        if grad_level == 1:
            jac = self.robot.positionJacobian(self.linkid, lcl_pos=self.lcl_pos)
            return (val, jac)
        return (val,)

