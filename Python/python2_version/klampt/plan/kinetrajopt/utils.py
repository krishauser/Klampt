from __future__ import print_function,division
import numpy as np
import autograd as jax


class CostInterface(object):
    """Define the cost function class, it must support several function evaluation"""
    def __jax_cost__(self, x):
        raise NotImplementedError('Sub-class has to implement __jax_cost__ function')

    def compute(self, x, grad_level=0):
        """This function evaluates the cost and possibly (determining on grad_level) return high order derivatives

        Args:
            x (ndarray): The evaluation point
            grad_level (int, optional): Which order of derivatives are computed. Defaults to 0.
        Returns:
            Depending on grad_level, this function is expected to return different things
            if grad_level == 0, it returns the cost value itself
            if grad_level == 1, it has to additionally return gradient
            if grad_level == 2, it has to additionally return Hessian
        """
        if self.__jax_cost__.__func__ is not CostInterface.__jax_cost__:
            cost = self.__jax_cost__(x)
            if grad_level == 0:
                return (cost,)
            else:
                if not hasattr(self, '__jax_grad__'):
                    self.__jax_grad__ = jax.grad(self.__jax_cost__)
                grad = self.__jax_grad__(x)  # ._value
                if grad_level == 1:
                    return (cost, grad)
                # now hessian is necessary
                if not hasattr(self, '__jax_hess__'):
                    self.__jax_hess__ = jax.hessian(self.__jax_cost__)
                hess = self.__jax_hess__(x) # ._value
                return (cost, grad, hess)
        raise NotImplementedError("Sub-class has to implement function compute")


class ConstrInterface(object):
    """Define the constraint class, it has to return Jacobian sometimes.
    Currently sparsity is not supported.
    It has to keep track of number of constraints, too.
    """
    def __jax_constr__(self, x):
        raise NotImplementedError('Sub-class has to implement __jax_constr__ function')

    def compute(self, x, grad_level=0):
        """Similar to cost interface, by calling this function, the constraints value are returned

        Args:
            x (ndarray): the point to be evaluated
            grad_level (int, optional): which level of gradient is computed. Defaults to 0.
        Returns:
            In the highest level, it returns a tuple of information on equality and inequality constraints. 
            For each one of them, depending on grad_level, the return is different.
            If grad_level == 0, it returns an ndarray of constraint values
            If grad_level == 1, it also returns an ndarray of constraint Jacobian
        """
        if self.__jax_constr__.__func__ is not ConstrInterface.__jax_constr__:
            if grad_level == 0:
                return (self.__jax_constr__(x),) #._value,)
            else:
                if not hasattr(self, '__jax_jac__'):
                    self.__jax_jac__ = jax.jacobian(self.__jax_constr__)
                # return (self.__jax_constr__(x)._value, self.__jax_jac__(x)._value)
                return (self.__jax_constr__(x), self.__jax_jac__(x))
        raise NotImplementedError("Sub-class has to implement function compute.")


class ConstrContainer(object):
    """This one maintains a grid for constraints.
    
    :param eqs: list of equality constraint in the format of (index, constr)
    :param ineqs: list of equality constraint in the format of (index, constr)
    """
    def __init__(self, eqs=None, ineqs=None):
        self.eqs = eqs if eqs is not None else []  # a list of (pos, constr) tuple
        self.ineqs = ineqs if ineqs is not None else []

    def add_eq(self, index, constr):
        """Add an equality constraint to the container.

        :param index: int, at which point of the trajectory is the constraint imposed. Can be negative indexed. If None, it applies to all points.
        :param constr: ConstrInterface, the constraint which has to be zero at the solution.
        """
        assert isinstance(constr, ConstrInterface), "constraint has to be subclass of ConstrInterface"
        self.eqs.append((index, constr))

    def add_ineq(self, index, constr):
        """Add an inequality constraint to the container.

        :param index: int, at which point of the trajectory is the constraint imposed. Can be negative indexed. If None, it applies to all points.
        :param constr: ConstrInterface, the constraint which has to be negative at the solution.
        """
        assert isinstance(constr, ConstrInterface), "constraint has to be subclass of ConstrInterface"
        self.ineqs.append((index, constr))

    def merge(self, container):
        """Merge current container with another container

        :param container: ConstrContainer, jsut another instance of constraint container
        """
        self.eqs += container.eqs
        self.ineqs += container.ineqs


class JointLimitsConstr(ConstrInterface):
    """The class for joint limits"""
    def __init__(self, qmin, qmax):
        self.qmin = np.array(qmin)
        self.qmax = np.array(qmax)

    def compute(self, x, grad_level=0):
        val = np.concatenate((self.qmin - x, x - self.qmax))
        if grad_level == 1:
            dimq = len(self.qmin)
            jac = np.r_[-np.eye(dimq), np.eye(dimq)]
            return (val, jac)
        else:
            return (val,)


class MaskedRobot(object):
    """This object creates a mask to robot so only certain links can be changed

    Args:
        object (robot): klampt robot
        index (arr-like): list of active links
    """
    def __init__(self, robot, index):
        self.robot = robot
        self.index = index
        self._config = np.array(robot.getConfig())

    def __len__(self):
        return len(self.index)

    def setConfig(self, config):
        self._config[self.index] = config
        self.robot.setConfig(self._config)

    def positionJacobian(self, linkid, config=None, lcl_pos=[0, 0, 0]):
        if config is not None:
            self.setConfig(config)
        jac = np.array(self.robot.link(linkid).getPositionJacobian(lcl_pos))
        return jac[:, self.index]

    def orientationJacobian(self, linkid, config=None):
        if config is not None:
            self.setConfig(config)
        jac = np.array(self.robot.link(linkid).getOrientationJacobian())
        return jac[:, self.index]

    def numLink(self):
        return len(self.index)

    def link_masked(self, num):
        return self.robot.link(int(self.index[num]))

    def link(self, num):
        return self.robot.link(num)


class MaskedTerrain(object):
    """This object manages the obstacles we have to consider
    """
    def __init__(self, world, index):
        self.world = world
        self.index = index

    def __len__(self):
        return len(self.index)

    def terrain(self, num):
        return self.world.terrain(int(self.index[num]))

    def terrain_unmasked(self, num):
        return self.world.terrain(num)

