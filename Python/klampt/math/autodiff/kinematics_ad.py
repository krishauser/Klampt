"""Klampt kinematics AD functions:

 ====================  =============  ======================================
 Function              Derivative     Notes
 ====================  =============  ======================================
 WorldPosition         1,2            wp[link,localPos](q) -> R^3
 WorldDirection        1,2            wd[link,localDir](q) -> R^3
 WorldOrientation      1,2            wo[link](q) -> SO(3)
 WorldTransform        1,2            wt[link,*localPos](q) -> SE(3)
 WorldVelocity         1              wv[link,localPos](q,dq) -> R^3
 WorldAngularVelocity  1              ww[link](q,dq) -> R^3
 #LocalPosition        1              lp[link,worldPos](q) -> R^3
 #LocalDirection       1              ld[link,worldDir](q) -> R^3
 #LocalOrientation     1              lo[link](q) -> SO(3)
 DriversToLinks        Y              d2l[robot](qdriver) -> R^numLinks()
 DriverDerivsToLinks   Y              d2l'[robot](vdriver) -> R^numLinks()
 LinksToDrivers        Y              l2d[robot](q) -> R^numDrivers()
 LinkDerivsToDrivers   Y              l2d'[robot](vdriver) -> R^numLinks()
 ConfigInterpolate     N              qinterp[robot](a,b,u) -> R^n
 ====================  =============  ======================================

Note that each call to WorldX or LocalX will recompute the forward kinematics
of the robot, which can be rather expensive if many of these will be called.

Also, see the :class:`KinematicsBuilder` class which creates the computational
graph for a robot's forward kinematics, which allows you to reuse
subexpressions for multiple forward kinematics calls.
"""

import numpy as np 
from .ad import ADFunctionInterface,ADFunctionCall,ADTerminal,sum_
from . import math_ad,so3_ad,se3_ad
from .. import vectorops,so3,se3
from ...robotsim import RobotModel,RobotModelLink


class WorldPosition(ADFunctionInterface):
    """Autodiff wrapper of the link.getWorldPosition() function as a function 
    of robot configuration q.
    """
    def __init__(self,link,localPos):
        self.robot = link.robot()
        self.link = link
        self.localPos = localPos
    def __str__(self):
        return "kinematics.WorldPosition[%s,%s]"%(self.link.getName(),str(self.localPos))
    def n_args(self):
        return 1
    def n_in(self,arg):
        return self.robot.numLinks()
    def n_out(self):
        return 3
    def eval(self,q):
        self.robot.setConfig(q.tolist())
        return np.array(self.link.getWorldPosition(self.localPos))
    def derivative(self,arg,q):
        assert arg == 0
        self.robot.setConfig(q.tolist())
        return np.array(self.link.getPositionJacobian(self.localPos))
    def gen_derivative(self,arg,q):
        if len(arg) == 1:
            return self.derivative(arg,q)
        elif len(arg) == 2:
            self.robot.setConfig(q.tolist())
            Hx,Hy,Hz = self.link.getPositionHessian(self.localPos)
            return np.array([Hx,Hy,Hz])
        else:
            raise NotImplementedError()


class WorldDirection(ADFunctionInterface):
    """Autodiff wrapper of the link.getWorldDirection() function as a function
    of robot configuration q.
    """
    def __init__(self,link,localDir):
        self.robot = link.robot()
        self.link = link
        self.localDir = localDir
    def __str__(self):
        return "kinematics.WorldDirection[%s,%s]"%(self.link.getName(),str(self.localDir))
    def n_args(self):
        return 1
    def n_in(self,arg):
        return self.robot.numLinks()
    def n_out(self):
        return 3
    def eval(self,q):
        self.robot.setConfig(q.tolist())
        return np.array(self.link.getWorldDirection(self.localDir))
    def derivative(self,arg,q):
        assert arg == 0
        self.robot.setConfig(q.tolist())
        Jo = self.link.getOrientationJacobian()
        for i in range(3):
            Jo[i] = np.array(Jo[i])
        return np.array(vectorops.cross(Jo,self.localDir))
    def gen_derivative(self,arg,q):
        if len(arg) == 1:
            return self.derivative(arg[0],q)
        elif len(arg) == 2:
            self.robot.setConfig(q.tolist())
            Hx,Hy,Hz = self.link.getOrientationHessian()
            Hx = np.array(Hx)
            Hy = np.array(Hy)
            Hz = np.array(Hz)
            return np.array(vectorops.cross([Hx,Hy,Hz],self.localDir))
        else:
            raise NotImplementedError()


class WorldOrientation(ADFunctionInterface):
    """Autodiff wrapper of the link.getTransform()[0] function as a function of
    robot configuration q.
    """
    def __init__(self,link):
        self.robot = link.robot()
        self.link = link
    def __str__(self):
        return "kinematics.WorldOrientation[%s]"%(self.link.getName(),)
    def n_args(self):
        return 1
    def n_in(self,arg):
        return self.robot.numLinks()
    def n_out(self):
        return 9
    def eval(self,q):
        self.robot.setConfig(q.tolist())
        return np.array(self.link.getTransform()[0])
    def derivative(self,arg,q):
        assert arg == 0
        self.robot.setConfig(q.tolist())
        Jo = self.link.getOrientationJacobian()
        return _cross_product_twiddle(Jo)
    def gen_derivative(self,arg,q):
        if len(arg) == 1:
            return self.derivative(arg[0],q)
        elif len(arg) == 2:
            self.robot.setConfig(q.tolist())
            Hx,Hy,Hz = self.link.getOrientationHessian()
            return _cross_product_twiddle([Hx,Hy,Hz])
        else:
            raise NotImplementedError()


class WorldTransform(ADFunctionInterface):
    """Autodiff wrapper of the link.getTransform() function as a function of robot
    configuration q.
    """
    def __init__(self,link,localPos=None):
        self.robot = link.robot()
        self.link = link
        self.localPos = localPos
    def __str__(self):
        if self.localPos is not None:
            return "kinematics.WorldTransform[%s,%s]"%(self.link.getName(),self.localPos)
        return "kinematics.WorldTransform[%s]"%(self.link.getName(),)
    def n_args(self):
        return 1
    def n_in(self,arg):
        return self.robot.numLinks()
    def n_out(self):
        return 12
    def eval(self,q):
        self.robot.setConfig(q.tolist())
        T = self.link.getTransform()
        if self.localPos is not None:
            T = (T[0],vectorops.add(so3.apply(T[0],self.localPos),T[1]))
        return np.array(T[0]+T[1])
    def derivative(self,arg,q):
        assert arg == 0
        self.robot.setConfig(q.tolist())
        J = self.link.getJacobian([0]*3 if self.localPos is None else self.localPos)
        return np.vstack([_cross_product_twiddle(J[:3])]+[J[3:]])
    def gen_derivative(self,arg,q):
        if len(arg) == 1:
            return self.derivative(arg[0],q)
        elif len(arg) == 2:
            self.robot.setConfig(q.tolist())
            Hx,Hy,Hz = self.link.getPositionHessian([0]*3 if self.localPos is None else self.localPos)
            Hox,Hoy,Hoz = self.link.getOrientationHessian()
            Hox = np.array(Hox)
            Hoy = np.array(Hoy)
            Hoz = np.array(Hoz)
            return np.vstack([_cross_product_twiddle([Hox,Hoy,Hoz])]+[[Hx,Hy,Hz]])
        else:
            raise NotImplementedError()


class WorldVelocity(ADFunctionInterface):
    """Autodiff wrapper of the link.getPointVelocity() function as a function 
    of robot configuration q and velocity dq.
    """
    def __init__(self,link,localPos):
        self.robot = link.robot()
        self.link = link
        self.localPos = localPos
    def __str__(self):
        return "kinematics.WorldVelocity[%s,%s]"%(self.link.getName(),str(self.localPos))
    def n_args(self):
        return 2
    def n_in(self,arg):
        return self.robot.numLinks()
    def n_out(self):
        return 3
    def eval(self,q,dq):
        self.robot.setConfig(q.tolist())
        self.robot.setVelocity(dq.tolist())
        return np.array(self.link.getPointVelocity(self.localPos))
    def derivative(self,arg,q,dq):
        if arg == 1:
            self.robot.setConfig(q.tolist())
            return np.array(self.link.getPositionJacobian(self.localPos))
        else:
            self.robot.setVelocity(dq.tolist())
            Hx,Hy,Hz = self.link.getPositionHessian(self.localPos)
            return np.row_stack([np.dot(Hx,dq),np.dot(Hy,dq),np.dot(Hz,dq)])


class WorldAngularVelocity(ADFunctionInterface):
    """Autodiff wrapper of the link.getAngularVelocity() function, as a
    function of robotconfiguration q and velocity dq.
    """
    def __init__(self,link):
        self.robot = link.robot()
        self.link = link
    def __str__(self):
        return "kinematics.WorldAngularVelocity[%s]"%(self.link.getName(),)
    def n_args(self):
        return 2
    def n_in(self,arg):
        return self.robot.numLinks()
    def n_out(self):
        return 3
    def eval(self,q,dq):
        self.robot.setConfig(q.tolist())
        self.robot.setVelocity(dq.tolist())
        return np.array(self.link.getAngularVelocity())
    def derivative(self,arg,q,dq):
        if arg == 1:
            self.robot.setConfig(q.tolist())
            return np.array(self.link.getOrientationJacobian())
        else:
            self.robot.setVelocity(dq.tolist())
            Hx,Hy,Hz = self.link.getOrientationHessian()
            return np.row_stack([np.dot(Hx,dq),np.dot(Hy,dq),np.dot(Hz,dq)])


class DriversToLinks(ADFunctionInterface):
    """Autodiff function to convert driver values to link values."""
    def __init__(self,robot):
        self.robot = robot
        self.drivers = [robot.driver(i) for i in robot.numDrivers()]
    def __str__(self):
        return "kinematics.DriversToLinks[%s]"%(self.robot.getName(),)
    def n_args(self):
        return 1
    def n_in(self,arg):
        return self.robot.numDrivers()
    def n_out(self):
        return self.robot.numLinks()
    def eval(self,qdriver):
        for driver,q in zip(self.drivers,qdriver):
            driver.setValue(q)
        return np.array(self.robot.getConfig())
    def jvp(self,arg,dqdriver,qdriver):
        for driver,q,v in zip(self.drivers,qdriver,dqdriver):
            driver.setValue(q)
            driver.setVelocity(v)
        return np.array(self.robot.getVelocity())


class DriverDerivsToLinks(ADFunctionInterface):
    """Autodiff function to convert driver velocities to link velocities."""
    def __init__(self,robot):
        self.robot = robot
        self.drivers = [robot.driver(i) for i in robot.numDrivers()]
    def __str__(self):
        return "kinematics.DriverDerivsToLinks[%s]"%(self.robot.getName(),)
    def n_args(self):
        return 1
    def n_in(self,arg):
        return self.robot.numDrivers()
    def n_out(self):
        return self.robot.numLinks()
    def eval(self,vdriver):
        for driver,v in zip(self.drivers,vdriver):
            driver.setVelocity(v)
        return np.array(self.robot.getConfig())
    def jvp(self,arg,dvdriver,vdriver):
        for driver,q,v in zip(self.drivers,vdriver,dvdriver):
            driver.setVelocity(v)
        return np.array(self.robot.getVelocity())


class LinksToDrivers(ADFunctionInterface):
    """Autodiff function to convert link values to driver values."""
    def __init__(self,robot):
        self.robot = robot
        self.drivers = [robot.driver(i) for i in robot.numDrivers()]
    def __str__(self):
        return "kinematics.LinksToDrivers[%s]"%(self.robot.getName(),)
    def n_args(self):
        return 1
    def n_in(self,arg):
        return self.robot.numLinks()
    def n_out(self):
        return self.robot.numDrivers()
    def eval(self,q):
        self.robot.setConfig(q)
        return np.array([driver.getValue() for driver in self.drivers])
    def jvp(self,arg,dq,q):
        self.robot.setConfig(q)
        self.robot.setVelocity(dq)
        return np.array([driver.getVelocity() for driver in self.drivers])


class LinkDerivsToDrivers(ADFunctionInterface):
    """Autodiff function to convert link velocities to driver velocities."""
    def __init__(self,robot):
        self.robot = robot
        self.drivers = [robot.driver(i) for i in robot.numDrivers()]
    def __str__(self):
        return "kinematics.LinkDerivsToDrivers[%s]"%(self.robot.getName(),)
    def n_args(self):
        return 1
    def n_in(self,arg):
        return self.robot.numLinks()
    def n_out(self):
        return self.robot.numDrivers()
    def eval(self,v):
        self.robot.setVelocity(v)
        return np.array([driver.getVelocity() for driver in self.drivers])
    def jvp(self,arg,dv,v):
        self.robot.setVelocity(dv)
        return np.array([driver.getVelocity() for driver in self.drivers])


class ConfigInterpolate(ADFunctionInterface):
    """Autodiff wrapper of the RobotModel.interpolate function"""
    def __init__(self,robot):
        self.robot = robot
    def __str__(self):
        return "kinematics.ConfigInterpolate[%s]"%(self.robot.getName(),)
    def n_args(self):
        return 3
    def n_in(self,arg):
        if arg <= 1:
            return self.robot.numLinks()
        return 1
    def n_out(self):
        return self.robot.numLinks()
    def eval(self,a,b,u):
        return np.array(self.robot.interpolate(a,b,u))
    

def _cross_product_twiddle(J):
    """Does the same thing as so3.cross_product, but with a matrix"""
    assert len(J) == 3
    n = len(J[0])
    J = [np.asarray(row) for row in J]
    res = np.empty((9,)+J[0].shape,dtype=float)
    res[0,:] = 0
    res[1,:] = J[2]
    res[2,:] = -J[1]
    res[3,:] = -J[2]
    res[4,:] = 0
    res[5,:] = J[0]
    res[6,:] = J[1]
    res[7,:] = -J[0]
    res[8,:] = 0
    return res




    



class KinematicsBuilder:
    """A class that computes the entire computation graph of forward kinematics
    and caches it so that multiple queries are auto-diffable and share the same
    intermediate computations.

    Args:
        robot (RobotModel): the robot
        configuration (array, AD expression, or list of expressions, optional):
            the robot's configuration, either as a fixed configuration or a
            variable. By default, this is fixed at the robot's configuration.
        velocity (array, AD expression, or list of expressions, optional): if
            given, the X_velocity methods are available.  This gives the 
            robot's  velocity, either as a fixed vector or a variable.  By 
            default, no velocity expression tree is created.
        relative_transform (array or list of AD so3 expressions, optional): if
            given, the relative transforms of the robot's links. By default
            these are taken from the robot model.
        axes (array or list of AD R^3 expressions, optional): if given, the
            axes relative transforms of the robot's links. By default these are
            taken from the robot model.

    Example::

        kb = KinematicsBuilder(robot,'q','dq')
        print(kb.world_position(robot.numLinks()-1))
        print(kb.world_velocity(robot.numLinks()-1))
    """
    def __init__(self,robot,configuration='fixed',velocity=None,relative_transforms='fixed',axes='fixed'):
        if configuration == 'fixed':
            configuration = robot.getConfig()
        else:
            if isinstance(configuration,str):
                configuration = ADTerminal(configuration)
            if not isinstance(configuration,ADTerminal) and not isinstance(configuration,ADFunctionCall):
                assert len(configuration) == robot.numLinks()
        if relative_transforms == 'fixed':
            relative_transforms = []
            for i in range(robot.numLinks()):
                T = robot.link(i).getParentTransform()
                relative_transforms.append(np.array(T[0] + T[1]))
        else:
            assert len(relative_transforms) == robot.numLinks()
        if axes == 'fixed':
            axes = [np.array(robot.link(i).getAxis()) for i in range(robot.numLinks())]
        else:
            assert len(axes) == robot.numLinks()

        self.robot = robot
        self.axes = axes
        self.link_transforms = []
        self.link_rotations = []
        self.link_positions = []
        self.link_inv_transforms = []
        self.link_inv_rotations = []
        for i in range(robot.numLinks()):
            link = robot.link(i)
            p = link.getParent()
            q = configuration[i]
            axis = axes[i]
            Trel = relative_transforms[i]
            if link.isPrismatic():
                link_t = Trel[9:] + q*axis
                link_R = Trel[:9]
            else:
                link_t = Trel[9:]
                Rq = so3_ad.from_axis_angle(axis,q)
                link_R = so3_ad.mul(Trel[:9],Rq)
            assert p < i
            if p < 0:
                self.link_positions.append(link_t)
                self.link_rotations.append(link_R)
                self.link_transforms.append(se3_ad.join(self.link_rotations[-1],self.link_positions[-1]))
            else:
                self.link_positions.append(se3_ad.apply(self.link_transforms[p],link_t))
                self.link_rotations.append(so3_ad.mul(self.link_rotations[p],link_R))
                self.link_transforms.append(se3_ad.mul(self.link_transforms[p],se3_ad.join(link_R,link_t)))
            self.link_inv_transforms.append(se3_ad.inv(self.link_transforms[-1]))
            self.link_inv_rotations.append(so3_ad.inv(self.link_rotations[-1]))

        if velocity is not None:
            if velocity == 'fixed':
                velocity = robot.getVelocity()
            if isinstance(velocity,str):
                velocity = ADTerminal(velocity)
            if not isinstance(velocity,ADTerminal) and not isinstance(velocity,ADFunctionCall):
                assert len(velocity) == robot.numLinks()
            self.link_world_axes = [so3_ad.apply(self.link_rotations[i],axes[i]) for i in range(robot.numLinks())]
            self.link_velocities = []
            self.link_angular_velocities = []
            for i in range(robot.numLinks()):
                link = robot.link(i)
                p = link.getParent()
                world_axis = self.link_world_axes[i]
                angvel_terms = []
                vel_terms = []
                if link.isRevolute():
                    angvel_terms.append(world_axis*velocity[i])
                else:
                    vel_terms.append(world_axis*velocity[i])
                while p >= 0:
                    link = robot.link(p)
                    world_axis = self.link_world_axes[p]
                    if link.isRevolute():
                        angvel_terms.append(world_axis*velocity[p])
                        vel_terms.append(math_ad.cross(world_axis,self.link_positions[i]-self.link_positions[p])*velocity[p])
                    else:
                        vel_terms.append(world_axis*velocity[p])
                    p = link.getParent()
                if len(vel_terms) == 0:
                    vel = np.zeros(3)
                else:
                    vel = sum_(*vel_terms)
                if len(angvel_terms) == 0:
                    angvel = np.zeros(3)
                else:
                    angvel = sum_(*angvel_terms)
                self.link_velocities.append(vel)
                self.link_angular_velocities.append(angvel)
            #done with velocities

    def _link_index(self,link):
        if isinstance(link,RobotModelLink):
            return link.index
        if isinstance(link,str):
            return self.robot.link(link).index
        return link

    def world_transform(self,link):
        """Returns an autodiff expression for the transform of the given link.
        
        Expression evaluates to a 12-D se3_ad array.
        """
        link = self._link_index(link)
        return self.link_transforms[link]

    def world_position(self,link,localPos=None):
        """Returns an autodiff expression for the world position of the point 
        localPos on the given link.  If localPos isn't given, the link origin's
        position is returned.
        """
        link = self._link_index(link)
        if localPos is None:
            return self.link_positions[link]
        return se3_ad.apply(self.link_transforms[link],localPos)

    def world_direction(self,link,localDir):
        """Returns an autodiff expression for the world direction of the  
        direction localDir on the given link.
        """
        link = self._link_index(link)
        return so3_ad.apply(self.link_rotations[link],localDir)

    def world_orientation(self,link):
        """Returns an autodiff expression for the orientation of the given
        link.
        
        Expression evaluates to a 9-D so3_ad array.
        """
        link = self._link_index(link)
        return self.link_rotations[link]

    def world_velocity(self,link,localPos=None):
        """Returns an autodiff expression for the world velocity of the point 
        localPos on the given link.  If localPos isn't given, the link origin's
        velocity is returned.
        """
        link = self._link_index(link)
        if localPos is None:
            return self.link_velocities[link]
        return self.link_velocities[link] + so3.apply(self.link_rotations[link],math_ad.cross(self.axes[link],localPos))

    def world_angular_velocity(self,link):
        """Returns an autodiff expression for the world angular velocity of the
        given link.

        Expression evaluates to a 9-D so3_ad array.
        """
        link = self._link_index(link)
        return self.link_angular_velocities[link]

    def local_position(self,link,localPos):
        link = self._link_index(link)
        return se3_ad.apply(self.link_inv_transforms[link],localPos)

    def local_direction(self,link,localDir):
        link = self._link_index(link)
        return so3_ad.apply(self.link_inv_rotations[link],localDir)

    def inv_orientation(self,link):
        link = self._link_index(link)
        return self.link_inv_rotations[link]

