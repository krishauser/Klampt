"""
Implementation of TrajOpt algorithm in klampt.
"""
from collections import Iterable
import numpy as np
import cvxpy as cp

from klampt import Geometry3D, DistanceQuerySettings
from klampt.math import se3

from .utils import CostInterface, ConstrInterface, ConstrContainer, JointLimitsConstr, MaskedRobot, MaskedTerrain
from .trajopt_task_space import PoseConstraint, DirectionConstraint, PositionConstraint, OrientationConstraint


class TrajOptSettings(object):
    """Defines settings of the KineTrajOpt solver."""
    def __init__(self, **kw):
        self.dsafe = kw.get('dsafe', 0.05)  # safe distance one has to keep
        self.dcheck = kw.get('dcheck', 0.1)  # distance that trigers constraint consideration
        self.ctol = kw.get('ctol', 1e-3)  # tolerance for constraint satisfaction
        self.max_iter = kw.get('max_iter', 40)
        self.min_approx_improve_frac = kw.get('min_approx_improve_frac', 0.001)
        self.improve_ratio_threshold = kw.get('improve_ratio_threshold', 0.2)
        self.merit_error_coeff = kw.get('merit_error_coeff', 20)
        self.min_trust_box_size = kw.get('min_trust_box_size', 1e-4)
        self.min_approx_improve = kw.get('min_approx_improve', 1e-4)
        self.taus = kw.get('trust_region_expand_decrease', (1.5, 0.2))
        self.cnt_tolerance = kw.get('cnt_tolerance', 1e-4)
        self.max_merit_coeff_increase = kw.get('max_merit_coeff_increase', 5)
        self.merit_coeff_increase_ratio = kw.get('merit_coeff_increase_ratio', 10)
        self.trust_box_size = kw.get('trust_box_size', 0.1)
        self.cvxpy_args = kw.get('cvxpy_args', {})  # let cvxpy select solver
        self.verbose = kw.get('verbose', False)
        self.joint_limits_by_bound = kw.get('joint_limits_by_bound', True)
        self.limit_joint = kw.get('limit_joint', True)


class JointObjectInfo(object):
    """Information contraining joint-object collision.

    :param linkid: int, the id of link with collision, just for bookkeeping
    :param objid: int, the id of world object, just for bookkeeping
    :param theta: ndarray, configuration, the configuration of robot at this collision
    :param qid: int, this is used to locate which index theta is at
    :param jacobian: ndarray, the jacobian matrix of the interesting point
    :param normal: ndarray, the normal direction of intervention
    :param distance: float, the distance for this collision pair
    """
    def __init__(self, linkid, objid, theta, qid, jacobian, normal, distance):
        self.linkid = linkid
        self.objid = objid
        self.theta = theta
        self.qid = qid
        self.jacobian = jacobian
        self.normal = normal
        self.distance = distance


class SweepJointObjectInfo(object):
    """
    Information for sweeping joint object collision

    :param linkid: int, the id of link with collision
    :param objid: int, the id of object
    :param theta1: ndarray, configuration at first step
    :param theta2: ndarray, configuration at second step
    :param qid: int, index of theta1, the next one is qid + 1
    :param alpha: float, the split between two points
    :param jacobian1: ndarray, the jacobian matrix of p0 at theta1
    :param jacobian2: ndarray, the jacobian matrix of p1 at theta2
    :param normal: ndarray, the normal direction from object to link hull
    :param distance: float, the distance between object and convex hull
    """
    def __init__(self, linkid, objid, theta1, theta2, qid1, alpha, jacobian1, jacobian2, normal, distance):
        self.linkid = linkid
        self.objid = objid
        self.theta1 = theta1
        self.theta2 = theta2
        self.qid1 = qid1
        self.alpha = alpha
        self.jacobian1 = jacobian1
        self.jacobian2 = jacobian2
        self.normal = normal
        self.distance = distance
        self.point1 = None
        self.point2 = None
        self.pswept = None
        self.pobs = None


class QPException(Exception):
    pass


class KineTrajOpt:
    r"""
    An implementation of the trajopt library by Josh Schulman, authored by Gao Tang.

    :param world: WorldModel, the world which contains obstacle information.
    :param robot: RobotModel, the robot whose trajectory has to be optimized.
    :param q0: arr-like, if not None, it gives the initial configuration of the robot. Its length equals number of joints being optimized. See link_index for details
    :param qf: arr-like, if not None, it gives the final configuration of the robot. Its length equals q0
    :param config: TrajOptSettings, it sets some hyperparameters of the solver.
    :param link_index: arr-like, if not None, it is the links whose configurations are optimized. It can have smaller length than robot.numLinks()
    :param geom_index: arr-like, if not None, it is the links whose geometries are considered for collision.
    :param obs_index: arr-like, if not None, it is the index of terrains considered as obstacles
    :param mounted: list of (int, se3, convexhull), this means other geometries rigidly mounted on link with given index
    :param constrs: ConstrContainer, if not None, it contains the constraints the trajectory has to satisfy.
    :param losses: list of CostInterface, if not None, it contains all the loss functions. If None, we optimize \sum (q_i - q_{i-1}) ** 2
    """
    def __init__(self, world, robot, q0=None, qf=None, config=None, link_index=None, geom_index=None, obs_index=None, mounted=None, constrs=None, losses=None):
        n_robot_link = robot.numLinks()
        link_index = np.arange(n_robot_link) if link_index is None else link_index
        self.dimq = len(link_index)
        self.constrs = ConstrContainer()
        if q0 is not None:
            # self.constrs.eqs.append((0, ConfigurationTargetConstr(q0)))  # fix q0
            self.fixedq0 = True
            self.q0 = np.array(q0)
        else:
            self.fixedq0 = False
        if qf is not None:
            # self.constrs.eqs.append((-1, ConfigurationTargetConstr(qf)))  # fix qf
            self.fixedqf = True
            self.qf = np.array(qf)
        else:
            self.fixedqf = False
        if constrs is not None and isinstance(constrs, ConstrContainer):
            self.constrs.merge(constrs)
        self.losses = losses
        if self.losses is not None and not isinstance(self.losses, Iterable):
            self.losses = [self.losses]
        self.config = config if config is not None else TrajOptSettings()
        # compute and set joint limits
        qmin, qmax = robot.getJointLimits()
        self.qmin = np.array(qmin)[link_index]
        self.qmax = np.array(qmax)[link_index]
        if self.config.limit_joint and not self.config.joint_limits_by_bound:
            self.constrs.add_ineq(None, JointLimitsConstr(self.qmin, self.qmax))
        obs_index = np.arange(world.numTerrains()) if obs_index is None else obs_index
        geom_index = np.arange(n_robot_link) if geom_index is None else geom_index
        self.mounted = mounted if mounted is not None else []
        self.world = MaskedTerrain(world, obs_index)
        self.robot = MaskedRobot(robot, link_index)
        self.compute_point_collision = True
        self.compute_sweep_collision = True
        self.logs = []  # caching the results...
        self.create_geometry_cache(robot, link_index, geom_index)
        # compute joint limits...
        qmin, qmax = robot.getJointLimits()
        self.qmin = np.array(qmin)[link_index]
        self.qmax = np.array(qmax)[link_index]

    def set_q0(self, q0):
        """Sets a fixed initial configuration."""
        if not self.fixedq0:
            self.fixedq0 = True
            self.q0 = np.copy(q0)
        else:
            self.q0[:] = q0

    def set_qf(self, qf):
        """Sets a fixed final configuration."""
        if not self.fixedqf:
            self.fixedqf = True
            self.qf = np.copy(qf)
        else:
            self.qf[:] = qf

    def add_pose_constraint(self, index, linkid, pose):
        """Add constraint such that at index of the trajectory, the robot link linkid is at pose"""
        self.constrs.add_eq(index, PoseConstraint(self.robot, linkid, pose))

    def add_position_constraint(self, index, linkid, lcl_pos, world_pos):
        """Add constraint such that at index of the trajectory, the robot link linkid's local position at world is world_pos"""
        self.constrs.add_eq(index, PositionConstraint(self.robot, linkid, lcl_pos, world_pos))

    def add_orientation_constraint(self, index, linkid, R):
        """Add constraint such that the link is at some fixed orientation"""
        self.constrs.add_eq(index, OrientationConstraint(self.robot, linkid, R))

    def add_direction_constraint(self, index, linkid, lcl_dir, world_dir):
        """Add constraint such that one link's local direction aligns with some world direction"""
        self.constrs.add_eq(index, DirectionConstraint(self.robot, linkid, lcl_dir, world_dir))

    def optimize(self, theta0):
        """Given an initial trajectory, use trajopt algorithm to update it.
        
        Args:
            theta0 (ndarray). The initial guess provided to the solver. This 
                should have shape (N,dof) where N is grid size and dof is the
                number of links being optimized.  It can also have size N*dof,
                in which case it will be internally reshaped to the proper
                dimension.
        
        Returns:
            dict: The result of optimization. Contains keys:

            - 'success': True if successful, False otherwise
            - 'sol': the solution trajectory, in the same form as theta0.
            - 'cost': the cost of the solution trajectory.
            
        """
        if theta0.ndim == 1:
            theta0 = theta0.reshape((-1, self.dimq))
        self.N = theta0.shape[0]
        d_safe = self.config.dsafe
        cur_sol = theta0.copy()
        mu = self.config.merit_error_coeff
        tr_size = self.config.trust_box_size
        taup, taum = self.config.taus
        ctol = self.config.cnt_tolerance
        N = theta0.shape[0]
        cost_cache = None
        collision_cache = None
        is_feasible = False
        for i in range(self.config.max_merit_coeff_increase):  # loop to adjust penalty
            for j in range(self.config.max_iter):  # trajopt is endless loop here
                if collision_cache is not None:
                    point_collisions, sweep_collisions = collision_cache
                else:
                    point_collisions, sweep_collisions = self.find_collision_pair(cur_sol, self.config.dcheck)
                    collision_cache = (point_collisions, sweep_collisions)
                # convexify the problem at current solution, which means we find collision
                if cost_cache is None:
                    cost0_t, cost0_c = self.compute_costs(cur_sol, point_collisions, sweep_collisions, d_safe)
                    cost_cache = (cost0_t, cost0_c)
                else:
                    cost0_t, cost0_c = cost_cache
                cost0 = cost0_t + mu * cost0_c
                self.logs.append((point_collisions, sweep_collisions))
                if self.config.verbose:
                    print('qp has %d point and %d sweep'%(len(point_collisions),len(sweep_collisions)))
                self.build_qp(point_collisions, sweep_collisions, N, cur_sol, mu, d_safe)
                goto15 = False
                trk = 0
                while tr_size > self.config.min_trust_box_size:
                    try:
                        obj, new_theta = self.solve_qp_with_tr_size(tr_size)
                    except QPException:
                        return {'success': False, 'sol': cur_sol, 'cost': np.inf}
                    # print(f'~~~constraint residual {self.cp_cache[-1].value}')
                    update = new_theta - cur_sol
                    if self.config.verbose:
                        print('!Update region size %d'%(np.amax(np.abs(update)),))
                    # compute cost at new solution
                    pcs, scs = self.find_collision_pair(new_theta, self.config.dcheck)
                    new_cost_t, new_cost_c = self.compute_costs(new_theta, pcs, scs, d_safe)
                    new_cost = new_cost_t + mu * new_cost_c
                    if self.config.verbose:
                        print('i=%d mu=%f j=%d k=%d cost0=%.3f obj=%.3f newcost=%.3f tr_size=%.2g'%(i,mu,j,trk,cost0,obj,new_cost,tr_size))
                    trk += 1
                    # compute true and model improve
                    approx_merit_improve = cost0 - obj
                    exact_merit_improve = cost0 - new_cost
                    merit_improve_ratio = exact_merit_improve / approx_merit_improve

                    if approx_merit_improve < -1e-5:
                        if self.config.verbose:
                            print('approximate merit got worse %.3g'%(approx_merit_improve,))
                    if approx_merit_improve < self.config.min_approx_improve:
                        if self.config.verbose:
                            print('approxi merit improve ABSOLUTE small')
                        goto15 = True
                        break
                    if approx_merit_improve / cost0 < self.config.min_approx_improve_frac:
                        if self.config.verbose:
                            print('approxi merit improve RELATIVE small')
                        goto15 = True
                        break
                    elif exact_merit_improve < 0 or merit_improve_ratio < self.config.improve_ratio_threshold:
                        tr_size *= taum
                        if self.config.verbose:
                            print('shrink trust region size to %.5f' % tr_size)
                    else:
                        cur_sol = new_theta
                        cost_cache = (new_cost_t, new_cost_c)
                        collision_cache = (pcs, scs)
                        tr_size *= taup
                        if self.config.verbose:
                            print('expand trust region size to %.5f' % tr_size)
                        break
                # check how convergence is obtained
                if tr_size < self.config.min_trust_box_size:
                    if self.config.verbose:
                        print('trust region too small %.5f / %.5f'%(tr_size, self.config.min_trust_box_size))
                    goto15 = True
                elif j == self.config.max_iter - 1:
                    if self.config.verbose:
                        print('Iteration limit reached, return')
                    return cur_sol
                if goto15:
                    break
            # Here comes step15, check collision and see if update of mu is needed
            point_collisions, sweep_collisions = collision_cache
            if self.collision_satisfy(point_collisions, sweep_collisions, ctol, d_safe) and self.constrs_satisfy(cur_sol, ctol):
                if self.config.verbose:
                    print('collision satisfy and constraint satisfy')
                is_feasible = True
                break
            else:
                if self.config.verbose:
                    print('mu update from %f to %f'%(mu,self.config.merit_coeff_increase_ratio * mu))
                mu = self.config.merit_coeff_increase_ratio * mu
                tr_size = max(tr_size, self.config.min_trust_box_size / taum * 1.5)
        # that's it, easy, but maybe we need more...
        return {'success': is_feasible, 'sol': cur_sol, 'cost': cost_cache[0]}

    def _indexes(self, idx):
        if idx is None:
            return range(self.N)
        else:
            return [idx]

    def compute_costs(self, thetas, point_collisions, sweep_collisions, d_safe):
        """Just compute the cost function at current solution"""
        N = thetas.shape[0]
        if self.losses is None:
            loss = np.sum((thetas[1:] - thetas[:-1]) ** 2)
        else:
            loss = 0
            for lossi in self.losses:
                obj = lossi.compute(thetas.flatten(), 0)[0]
                loss += obj
        closs = 0
        dists = []
        for pc in point_collisions + sweep_collisions:
            closs += max(0, d_safe - pc.distance)
            if d_safe > pc.distance:
                dists.append(pc.distance)
        # also consider other constraints
        for idx_, con in self.constrs.eqs:
            for idx in self._indexes(idx_):
                if (idx == 0 and self.fixedq0) or (idx == N - 1 and self.fixedqf):
                    continue
                val = con.compute(thetas[idx], grad_level=0)[0]
                # print(f'~~~~~~~~constr val = {val}')
                closs += np.sum(np.abs(val))
        for idx_, con in self.constrs.ineqs:
            for idx in self._indexes(idx_):
                if (idx == 0 and self.fixedq0) or (idx == N - 1 and self.fixedqf):
                    continue
                val = con.compute(thetas[idx], grad_level=0)[0]
                closs += np.sum(np.maximum(0, val))
        if self.config.verbose:
            print('traj loss %.4f collision loss %.4f bad dists %s'%(loss,closs,', '.join(["%.4f" % (i,) for i in dists])))
        return loss, closs

    def collision_satisfy(self, point_collisions, sweep_collisions, ctol, dsafe):
        """Check if at current solution, all collision avoidance constraints are satisfied"""
        dist_threshold = dsafe - ctol
        for pc in point_collisions + sweep_collisions:
            if pc.distance < dist_threshold:
                if self.config.verbose:
                    print('Distance %f not satisfied'%(pc.distance,))
                return False
        return True

    def constrs_satisfy(self, thetas, ctol):
        """Check if the constraints are satisfied"""
        N = thetas.shape[0]
        for idx_, con in self.constrs.eqs:
            for idx in self._indexes(idx_):
                if (idx == 0 and self.fixedq0) or (idx == N - 1 and self.fixedqf):
                    continue
                val = con.compute(thetas[idx], grad_level=0)[0]
                if np.any(np.abs(val) > ctol):
                    return False
        for idx_, con in self.constrs.ineqs:
            for idx in self._indexes(idx_):
                if (idx == 0 and self.fixedq0) or (idx == N - 1 and self.fixedqf):
                    continue
                val = con.compute(thetas[idx], grad_level=0)[0]
                if np.any(val > ctol):
                    return False
        return True

    def solve_qp_with_tr_size(self, tr_size):
        """Given trust region size, we solve the qp"""
        prob_cache, trs_cache, x_cache = self.cp_cache
        trs_cache.value = tr_size
        try:
            prob_cache.solve(**self.config.cvxpy_args)
        except Exception as e:
            print("QP failure, exception is", e)
            raise QPException
        if x_cache.value is None:
            raise QPException
        return prob_cache.value, x_cache.value

    def build_qp(self, point_collisions, sweep_collisions, N, theta0, mu, d_safe):
        """Create the qp problem to be solved.
        
        point_collisions is an Iterable of JointObjectInfo storing all point collision
        sweep_collisions is an Iterable of SweepJointObjectInfo storing all sweeping collisions
        theta0 is the trajectory at the last step
        mu is the penalty parameter to constraints
        warm means only trust region size is updated and we can reuse
        """
        N, dim_x = theta0.shape
        thetas = cp.Variable((N, dim_x), name="Configurations")
        trsize = cp.Parameter(name='Trust Region Size')
        # the sos loss is easy
        loss = 0
        if self.losses is None:
            loss += cp.sum_squares(thetas[1:] - thetas[:-1])
        else:
            for lossi in self.losses:
                obj, grad, hess = lossi.compute(theta0.flatten(), 2)
                delta = thetas.flatten() - theta0.flatten()
                loss = obj + grad * delta
                loss += 0.5 * cp.quad_form(delta, hess)
        # constraints on initial and final configuration
        # cons = [thetas[0] == self.q0, thetas[-1] == self.qf]
        cons = []
        if self.fixedq0:
            cons.append(thetas[0] == self.q0)
        if self.fixedqf:
            cons.append(thetas[-1] == self.qf)
        for idx_, con in self.constrs.eqs:
            for idx in self._indexes(idx_):
                if (idx == 0 and self.fixedq0) or (idx == N - 1 and self.fixedqf):
                    continue
                val, jac = con.compute(theta0[idx], grad_level=1)
                pt = cp.Variable(val.size)
                loss += mu * cp.sum(pt)
                cons.append(pt >= val + jac * (thetas[idx] - theta0[idx]))
                cons.append(pt >= -(val + jac * (thetas[idx] - theta0[idx])))
        for idx_, con in self.constrs.ineqs:
            for idx in self._indexes(idx_):
                if (idx == 0 and self.fixedq0) or (idx == N - 1 and self.fixedqf):
                    continue
                val, jac = con.compute(theta0[idx], grad_level=1)
                ineqpt = cp.Variable(val.size)
                loss += mu * cp.sum(ineqpt)
                cons.append(ineqpt >= 0)
                cons.append(ineqpt >= val + jac * (thetas[idx] - theta0[idx]))
        if self.config.limit_joint and self.config.joint_limits_by_bound:
            cons.append(thetas >= np.tile(self.qmin[None, :], (N, 1)))  # hope broadcasting works here
            cons.append(thetas <= np.tile(self.qmax[None, :], (N, 1)))
        # Now let's handle point_collisions
        n_point_collide = len(point_collisions)
        n_collide = n_point_collide + len(sweep_collisions)
        if n_collide > 0:
            pt = cp.Variable(n_collide, name='Auxiliary Variable')
            # all pt are added to the cost function and are positive
            loss += cp.sum(pt) * mu
            cons.append(pt >= 0)
        for i, pc in enumerate(point_collisions):
            # constraint that t > g_i(x) = d_safe - d0 - n^T J (theta - theta0)
            lin_coef = pc.normal.dot(pc.jacobian[:3])
            qid = pc.qid
            cons.append(pt[i] >= d_safe - pc.distance - cp.sum(cp.multiply(lin_coef, thetas[qid] - theta0[qid])))
        for i, pc in enumerate(sweep_collisions):
            lin_coef1 = pc.normal.dot(pc.jacobian1[:3])
            lin_coef2 = pc.normal.dot(pc.jacobian2[:3])
            alpha, beta = pc.alpha, 1 - pc.alpha
            qid1, qid2 = pc.qid1, pc.qid1 + 1
            cons.append(
                pt[i + n_point_collide] >= d_safe - pc.distance - cp.sum(
                alpha * (cp.multiply(lin_coef1, thetas[qid1] - theta0[qid1])) + beta * (cp.multiply(lin_coef2, thetas[qid2] - theta0[qid2]))
                )
            )
        # add trust region constraints, here I use infinite norm
        cons.append(cp.pnorm(thetas - theta0, 'inf') <= trsize)
        # ready to build the problem and cache them
        prob = cp.Problem(cp.Minimize(loss), cons)
        self.cp_cache = (prob, trsize, thetas)

    def create_geometry_cache(self, robot, link_index, geom_index):
        """Populates the internal geometry cache with ConvexHull versions of
        the link and terrain geometries.
        """
        self.geom_cache = {}
        link_geom_info = []  # stores the (hull, geom_idx, link_idx) tuple
        for i, geom_idx in enumerate(geom_index):
            link_idx = geom_idx
            while link_idx not in link_index:  # Here it's important to find the minimum link that controls this geometry
                link_idx = robot.link(link_idx).getParent()  # find its parent... makes sense
                assert link_idx != -1  # error may occur
            link = self.robot.link(geom_idx)
            geom = link.geometry().convert('ConvexHull', 0)
            geom_copy = link.geometry().convert('ConvexHull', 0)
            link_geom_info.append((geom, geom_copy, geom_idx, link_idx))
        assert self.compute_point_collision or self.compute_sweep_collision, "You have to at least consider one link collision"
        self.geom_cache['link'] = link_geom_info
        # consider the rigidly mounted part
        mount_cache = []
        for i in range(len(self.mounted)):
            link_idx, relT, geom = self.mounted[i]
            geom = geom.convert('ConvexHull', 0)
            geom_copy = geom.convert('ConvexHull', 0)
            mount_cache.append((link_idx, relT, geom, geom_copy))
        self.geom_cache['mount'] = mount_cache
        # for obstacles, there is not much we have to do
        n_obs = len(self.world)
        obs_geoms = []
        for i in range(n_obs):
            terrain = self.world.terrain(i)
            geom = terrain.geometry().convert('ConvexHull', 0)
            obs_geoms.append(geom)
        self.geom_cache['obs'] = obs_geoms

    def distance_with_one_link(self, q, geom_order):
        self.robot.setConfig(q)
        lk_geom, geom_idx, _ = self.geom_cache['link'][geom_order]
        tran = self.robot.link(geom_idx).getTransform()
        assert lk_geom is not None
        lk_geom.setCurrentTransform(*tran)
        return [lk_geom.distance(obs).d for obs in self.geom_cache['obs']]

    def all_linkgeom_transforms(self, thetas):
        """Compute transform matrix for all active links for all configurations.
        Return a list (for each configuration) of list (for each active link) of (R, t) tuples
        """
        trans = []
        mtrans = []
        for q in thetas:
            self.robot.setConfig(q)
            tmp = []
            for _, _, geom_idx, _ in self.geom_cache['link']:
                tmp.append(self.robot.link(geom_idx).getTransform())
            trans.append(tmp)
            # for mounted
            tmp = []
            for link_idx, relT, _, _ in self.geom_cache['mount']:
                tmp.append(se3.mul(self.robot.link(link_idx).getTransform(), relT))
            mtrans.append(tmp)
        return trans, mtrans

    def find_collision_pair(self, thetas, dcheck):
        """
        Given current solution, find all possible collision pairs.
        """
        link_trans, mount_trans = self.all_linkgeom_transforms(thetas)
        point_collisions = []
        sweep_collisions = []
        # Just iterate through all stuff and see what happens... We can safely ignore first and last
        n_theta = thetas.shape[0]
        setting = DistanceQuerySettings()
        setting.upperBound = dcheck  # value greater than this is not considered

        def add_point(link_idx, geom_idx, rst, i, oj):
            # This function add point collision information...
            p1, p2 = np.array(rst.cp1), np.array(rst.cp2)
            if rst.d > 0:
                normal = (p1 - p2)
            else:
                normal = p2 - p1
            normal /= np.linalg.norm(normal)
            # find p1 in local coordinate
            self.robot.setConfig(thetas[i])
            p1_local = self.robot.link(link_idx).getLocalPosition(list(p1))
            jacobian = self.robot.positionJacobian(link_idx, lcl_pos=p1_local)
            point_collisions.append(JointObjectInfo(geom_idx, oj, thetas[i].copy(), i, jacobian, normal, rst.d))

        def add_sweep(link_idx, geom_idx, sgl_lv, ltran1, ltran2, rst, i, oj):
            # This function add sweep collision information
            p2, pswept = np.array(rst.cp2), np.array(rst.cp1)
            if rst.d > 0:
                normal = pswept - p2
            else:
                normal = p2 - pswept
            normal /= np.linalg.norm(normal)
            # I have to find the two points on two seperate geometry 
            sgl_lv.setCurrentTransform(*ltran1)
            p0 = np.array(sgl_lv.support(-normal))
            sgl_lv.setCurrentTransform(*ltran2)
            p1 = np.array(sgl_lv.support(-normal))
            dist1 = np.linalg.norm(p1 - pswept)
            dist0 = np.linalg.norm(p0 - pswept)
            alpha = dist1 / (dist0 + dist1)
            # print(f'Sweep link {li} obj {oj} step {i} dist {dist:.3f} dist0 {dist0:.3f} dist1 {dist1:.3f}')
            # compute the two jacobians
            self.robot.setConfig(thetas[i])
            the_link = self.robot.link(link_idx)
            p0_local = the_link.getLocalPosition(p0)
            # jac0 = np.array(self.robot.link(li_).getPositionJacobian(p0_local))
            jac0 = self.robot.positionJacobian(link_idx, None, p0_local)
            self.robot.setConfig(thetas[i + 1])
            p1_local = the_link.getLocalPosition(p1)
            # jac1 = np.array(self.robot.link(li_).getPositionJacobian(p1_local))
            jac1 = self.robot.positionJacobian(link_idx, None, p1_local)
            sweep_collisions.append(SweepJointObjectInfo(geom_idx, oj, thetas[i].copy(), thetas[i + 1].copy(), i, alpha, jac0, jac1, normal, rst.d))
            sweep_collisions[-1].point1 = p0
            sweep_collisions[-1].point2 = p1
            sweep_collisions[-1].pswept = pswept
            sweep_collisions[-1].pobs = p2

        # outer loop is for obstacles
        for oj_, oj in enumerate(self.world.index):
            ov = self.geom_cache['obs'][oj_]  # transformation information is already contained in Geometry3D
            for li_, (lv, lv_copy, geom_idx, link_idx) in enumerate(self.geom_cache['link']):
                # first step is to compute point violation...
                if self.compute_point_collision:
                    for i in range(1, n_theta - 1):
                        ltran = link_trans[i][li_]
                        lv.setCurrentTransform(*ltran)
                        rst = lv.distance_ext(ov, setting)
                        # append if dist is smaller than dcheck
                        if rst.d < dcheck:
                            add_point(link_idx, geom_idx, rst, i, oj)
                # the next step is to compute sweep information
                if self.compute_sweep_collision:
                    for i in range(n_theta - 1):
                        ltran1, ltran2 = link_trans[i][li_], link_trans[i + 1][li_]
                        lv.setCurrentTransform(*ltran1)  # set transformation of link start
                        lv_copy.setCurrentTransform(*ltran2)  # set transform of link end
                        cvxhull = Geometry3D()
                        cvxhull.setConvexHullGroup(lv, lv_copy)
                        rst = cvxhull.distance_ext(ov, setting)
                        if rst.d < dcheck:
                            add_sweep(link_idx, geom_idx, lv, ltran1, ltran2, rst, i, oj)
            # now I consider attached geometries
            for li_, (link_idx, relT, geom, geom_copy) in enumerate(self.geom_cache['mount']):
                # start with point ocllision
                if self.compute_point_collision:
                    for i in range(1, n_theta - 1):
                        ltran = mount_trans[i][li_]
                        geom.setCurrentTransform(*ltran)
                        rst = geom.distance_ext(ov, setting)
                        if rst.d < dcheck:
                            add_point(link_idx, -1 - li_, rst, i, oj)
                # then sweep one
                if self.compute_sweep_collision:
                    for i in range(n_theta - 1):
                        ltran1, ltran2 = mount_trans[i][li_], mount_trans[i + 1][li_]
                        geom.setCurrentTransform(*ltran1)
                        geom_copy.setCurrentTransform(*ltran2)
                        cvxhull = Geometry3D()
                        cvxhull.setConvexHullGroup(geom, geom_copy)
                        rst = cvxhull.distance_ext(ov, setting)
                        if rst.d < dcheck:
                            add_sweep(link_idx, -1 - li_, geom, ltran1, ltran2, rst, i, oj)
        return point_collisions, sweep_collisions
