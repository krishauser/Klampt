import numpy as np

class CostInterface(object):
    """An abstract base class for a cost function.  The subclass must implement
    `compute` to evaluate f(x), d/dx f(x), and/or d^2/dx^2 f(x).
    """
    def compute(self, x, grad_level=0):
        """Evaluates the cost and possibly (determining on grad_level) the
        derivative and/or Hessian.

        Args:
            x (ndarray): The evaluation point
            grad_level (int, optional): Which order of derivatives are
                computed. Defaults to 0, which only computes cost.

        Returns:
            tuple: the cost and optional derivatives, structured as follows:

            - if grad_level == 0, it returns (cost,)
            - if grad_level == 1, it returns (cost,gradient)
            - if grad_level == 2, it returns (cost,gradient,hessian)
        """
        raise NotImplementedError("Sub-class has to implement function compute")


class ConstrInterface(object):
    """An abstract base class for a constraint function.  The subclass must
    implement `compute` to evaluate g(x) and/or d/dx g(x).

    The interpretation of whether this gives an equality or equality is not
    provided in this class.  See :class:`ConstrContainer`.

    Currently sparsity is not supported.
    """
    def compute(self, x, grad_level=0):
        """Evaluates the constraint function and possibly (determining on
        grad_level) the Jacobian.

        Args:
            x (ndarray): the point to be evaluated
            grad_level (int, optional): which level of gradient is computed.
                Defaults to 0.

        Returns:
            tuple: The constraint value and optional derivatives, structured as
            follows:

            - If grad_level == 0, it returns a 1-D ndarray of g(x)
            - If grad_level == 1, it also returns a 2-D ndarray giving the
              constraint Jacobian d/dx g(x).
        """
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
    """Creates a mask to robot so only certain links can be changed.
    
    Slightly faster than SubRobotModel since it uses numpy to do the indexing.

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

