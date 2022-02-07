"""Functions for computing robot workspaces, with lots of options for setting
feasibility conditions.

.. versionadded:: 0.9
"""

from klampt import RobotModel,RobotModelLink,RigidObjectModel,TerrainModel,VolumeGrid,IKObjective
from klampt import vis
from klampt.math import vectorops,so3,se3
from klampt.model import ik
from typing import Union,Sequence,Dict,Callable
from klampt.model.typing import Vector3,Config
import math
import numpy as np


def compute_occupancy_grid(points : Sequence[Vector3],
                           resolution=0.05,
                           dimensions=None,
                           bounds=None,
                           value='occupancy') -> VolumeGrid:
    """
    Helper to compute an occupancy grid given a set of points.

    Args:
        points (list of Vector3 or nx3 array): the points
        resolution (float, 3-vector, or None): the resolution of the resulting
            grid.
        dimensions (int or 3-vector optional): if resolution=None and dimensions
            is given, the number of dimensions. If a single int, the cell size
            is determined by dividing the longest side by dimensions.
        bounds (pair of 3-vectors, optional): specifies the minimum and maximum
            range of the grid. If not given, calculated by the 
        value (str): either 'occupancy', 'count', or 'probability'.

    """
    points = np.asarray(points)
    auto_bounds = False
    expand = False
    if bounds is not None:
        lower_corner,upper_corner = bounds
        if len(lower_corner) != 3 or len(upper_corner) != 3:
            raise ValueError("Invalid bounds")
    else:
        if len(points) == 0:
            raise ValueError("Cannot compute occupancy grid of empty set of points")
        lower_corner,upper_corner = np.min(points,axis=0),np.max(points,axis=0)
        assert len(lower_corner) == 3
        assert len(upper_corner) == 3
        auto_bounds = True
        expand = True

    if dimensions is not None:
        if hasattr(dimensions,'__iter__'):
            cellsize = vectorops.div(vectorops.sub(upper_corner,lower_corner),dimensions)
            invcellsize = vectorops.div(dimensions,vectorops.sub(upper_corner,lower_corner))
            auto_bounds = False
        else:
            w = max(*vectorops.sub(upper_corner,lower_corner))
            cellsize = [w / dimensions]*3
            invcellsize = [dimensions/w]*3
            dimensions = [int(math.floor(d/c)+1) for d,c in zip(vectorops.sub(upper_corner,lower_corner),cellsize)]
    else:
        if resolution is None:
            raise ValueError("One of resolution or dimensions must be given")
        cellsize = resolution
        if not hasattr(resolution,'__iter__'):
            cellsize = [resolution]*3
        invcellsize = [1.0/c for c in cellsize]
        dimensions = [int(math.floor(d/c)+1) for d,c in zip(vectorops.sub(upper_corner,lower_corner),cellsize)]

    if auto_bounds:    
        #adjust limits to reduce artifacts
        bmax = vectorops.add(lower_corner,[d*c for (d,c) in zip(dimensions,cellsize)])
        shift = vectorops.mul(vectorops.sub(upper_corner,bmax),0.5)
        lower_corner = vectorops.add(lower_corner,shift)
        upper_corner = vectorops.add(upper_corner,shift)
    if expand:
        lower_corner = vectorops.sub(lower_corner,cellsize)
        upper_corner = vectorops.add(upper_corner,cellsize)
        dimensions = [d+2 for d in dimensions]

    #compact way to compute all valid indices of points
    if len(points)==0:
        valid_indices = []
    else:
        indices = np.multiply(points - np.asarray(lower_corner),np.asarray(invcellsize))
        indices = np.floor(indices).astype(int)
        valid_points = np.all((0 <= indices) & (indices < np.array(dimensions)),axis=1)
        valid_indices = indices[valid_points,:]
    #output
    reachable = np.zeros(dimensions)
    if value=='occupancy':
        for ind in valid_indices:
            ind=tuple(ind)
            reachable[ind] = 1.0
    elif value in ['count','probability']:
        for ind in valid_indices:
            ind=tuple(ind)
            reachable[ind] += 1
        if value == 'probability' and len(points)>0:
            reachable *= 1.0/len(points)
    vg = VolumeGrid()
    vg.setBounds(lower_corner,upper_corner)
    vg.setValues(reachable)
    return vg


def compute_field_grid(points : Sequence[Vector3],
                       values : Sequence[float],
                       resolution=0.05,
                       dimensions=None,
                       bounds=None,
                       aggregator='max',
                       initial_value='auto') -> VolumeGrid:
    """
    Helper to compute a gridded value field over a set of scattered points.

    Args:
        points (list of Vector3 or nx3 array): the points
        values (list of float): the values at each point
        resolution (float, 3-vector, or None): the resolution of the resulting
            grid.
        dimensions (int or 3-vector optional): if resolution=None and dimensions
            is given, the number of dimensions. If a single int, the cell size
            is determined by dividing the longest side by dimensions.
        bounds (pair of 3-vectors, optional): specifies the minimum and maximum
            range of the grid. If not given, calculated by the 
        aggregator (str or pair): either 'max', 'min', 'sum', 'average', or a 
            pair of functions (f,g) giving an arbitrary aggregator
            f(x,value) -> x', g(x) -> float. x is an arbitrary object, which
            for each cell is initialized to the value specified by
            initial_value (default None).
        initial_value (float or 'auto'): the initial value of the cell before
            aggregation.  If aggregator is a pair of functions, 'auto' sets x
            to None by default.

    """
    points = np.asarray(points)
    auto_bounds = False
    expand = False
    if bounds is not None:
        lower_corner,upper_corner = bounds
        if len(lower_corner) != 3 or len(upper_corner) != 3:
            raise ValueError("Invalid bounds")
    else:
        if len(points) == 0:
            raise ValueError("Cannot compute occupancy grid of empty set of points")
        lower_corner,upper_corner = np.min(points,axis=0),np.max(points,axis=0)
        assert len(lower_corner) == 3
        assert len(upper_corner) == 3
        auto_bounds = True
        expand = True

    if dimensions is not None:
        if hasattr(dimensions,'__iter__'):
            cellsize = vectorops.div(vectorops.sub(upper_corner,lower_corner),dimensions)
            invcellsize = vectorops.div(dimensions,vectorops.sub(upper_corner,lower_corner))
            auto_bounds = False
        else:
            w = max(*vectorops.sub(upper_corner,lower_corner))
            cellsize = [w / dimensions]*3
            invcellsize = [dimensions/w]*3
            dimensions = [int(math.floor(d/c)+1) for d,c in zip(vectorops.sub(upper_corner,lower_corner),cellsize)]
    else:
        if resolution is None:
            raise ValueError("One of resolution or dimensions must be given")
        cellsize = resolution
        if not hasattr(resolution,'__iter__'):
            cellsize = [resolution]*3
        invcellsize = [1.0/c for c in cellsize]
        dimensions = [int(math.floor(d/c)+1) for d,c in zip(vectorops.sub(upper_corner,lower_corner),cellsize)]

    if auto_bounds:    
        #adjust limits to reduce artifacts
        bmax = vectorops.add(lower_corner,[d*c for (d,c) in zip(dimensions,cellsize)])
        shift = vectorops.mul(vectorops.sub(upper_corner,bmax),0.5)
        lower_corner = vectorops.add(lower_corner,shift)
        upper_corner = vectorops.add(upper_corner,shift)
    if expand:
        lower_corner = vectorops.sub(lower_corner,cellsize)
        upper_corner = vectorops.add(upper_corner,cellsize)
        dimensions = [d+2 for d in dimensions]
    
    #compact way to compute all valid indices of points
    if len(points)==0:
        valid_indices = []
    else:
        indices = np.multiply(points - np.asarray(lower_corner),np.asarray(invcellsize))
        indices = np.floor(indices).astype(int)
        valid_points = np.all((0 <= indices) & (indices < np.array(dimensions)),axis=1)
        valid_indices = indices[valid_points,:]
    
    result = None
    if aggregator == 'max':
        if initial_value == 'auto':
            initial_value = -float('inf')
        value_grid = np.full(dimensions,fill_value=initial_value)
        f=max
        g=None
    elif aggregator == 'min':
        if initial_value == 'auto':
            initial_value = float('inf')
        value_grid = np.full(dimensions,fill_value=float('inf'))
        f=min
        g=None
    elif aggregator == 'sum':
        if initial_value == 'auto':
            initial_value = 0.0
        value_grid = np.full(dimensions,fill_value=initial_value)
    elif aggregator == 'average':
        if initial_value == 'auto':
            prior,strength = 0.0,0
        else:
            if not isinstance(initial_value,(tuple,list)) or len(initial_value) != 2:
                raise ValueError("Initial value for average must be of the form (prior,strength)")
            prior,strength = initial_value
        vsum = np.full(dimensions,fill_value=prior*strength)
        count = np.full(dimensions,fill_value=strength)
        for ind,v in zip(valid_indices,values):
            vsum[ind] += v
            count[ind] += 1
        if strength > 0:
            result = np.divide(vsum,count)
        else:
            result = np.zeros(dimensions)
            nz = (count > 0)
            result[nz] = np.divide(vsum[nz],count[nz])
    elif isinstance(aggregator,tuple):
        f,g = aggregator
        value_grid = np.full(dimensions,fill_value=initial_value)
    else:
        raise ValueError("Invalid value for aggregator, must be min, max, sum, average, or a pair of callables")
    if result is None:
        for ind,v in zip(valid_indices,values):
            ind=tuple(ind)
            value_grid[ind] = f(value_grid[ind],v)
        if g is not None:
            result = np.empty(dimensions)
            for i in range(dimensions[0]):
                for j in range(dimensions[1]):
                    for k in range(dimensions[2]):
                        result[i,j,k] = g(value_grid[i,j,k])
        else:
            result = value_grid
    vg = VolumeGrid()
    vg.setBounds(lower_corner,upper_corner)
    vg.setValues(result)
    return vg


def compute_workspace(link : RobotModelLink,
                      constraint_or_point : Union[Vector3,IKObjective],
                      Nsamples=100000,
                      resolution=0.05,
                      dimensions=None,
                      fixed_links : Sequence[Union[int,str]] = None,
                      qmin : Config = None,
                      qmax : Config = None,
                      obstacles : Sequence[Union[RigidObjectModel,TerrainModel]] = None,
                      load : Union[float,Vector3] = None,
                      load_type : str = None,
                      gravity : Vector3 = (0,0,-9.8),
                      self_collision : bool = True,
                      feasibility_test : Callable = None,
                      value='occupancy',
                      all_tests=True) -> Union[VolumeGrid,Dict[str,VolumeGrid]]:
    """Compute the reachable workspace of a point on a robot's end effector.
    Has several options to ensure various feasibility conditions.  Ensures that
    the robot does not collide with itself, optionally with obstacles.
    
    Arguments:
        link (RobotModelLink): the link on the robot for which the workspace
            should be computed
        constraint_or_point (3-vector or IKObjective): the point on the link.
            For fixed orientations, use an IKObjective.
        Nsamples (int): the number of configuration samples to use
        resolution (float, 3-vector, or None): the resolution of the resulting
            grid.
        dimensions (int or 3-vector optional): if resolution=None and
            dimensions is given, the number of dimensions. If a single int, the
            cell size is determined by dividing the longest side by dimensions.
        fixed_links (list of int or str, optional): list of fixed link indices
        qmin (Config, optional): overrides the robots lower joint limits
        qmax (Config, optional): overrides the robots upper joint limits
        obstacles (list of RigidObjectModel or TerrainModel): Ensures that the
            robot is collision free with environment obstacles
        load (float or vector, optional): a required load capacity, either an
            isotropic load or a specific vector
        load_type (str, optional): either 'auto' (default), 'isotropic',
            'force', 'torque', 'wrench', 'force_local', 'torque_local', or
            'wrench_local'.
        gravity (3-vector, optional): if load testing is used, the gravity
            vector used for gravity compensation.
        self_collision (bool): whether self collision should be tested.
        feasibility_test (callable, list, or dict): a function
            feasible(q)->bool that tests whether the configuration is feasible.
            If a list, a list of callables. If a dict, a map of str->callable.
        value (str): either 'occupancy', 'count', or 'probability'.
        all_tests (bool): whether to return a dict of results.

    Returns:
        The grid of reached points. If ``all_tests=True``, gives a set of
        grids of reached points indexed by strings.  Key
        'workspace' gives the reachable workspace of points meeting every
        feasibility condition.  Keys 'self_collision_free',
        'obstacle_X_collision_free' (X in range 0,...,len(obstacles)-1),
        'load', 'feasibility_test' give the set of points passing each
        feasibility condition.
    """
    robot = link.robot()   # type: RobotModel
    qmin_orig,qmax_orig = robot.getJointLimits()
    
    if qmin is None:
        qmin = qmin_orig
    else:
        if len(qmin)!=len(qmin_orig):
            raise ValueError("Invalid length of qmin")
    if qmax is None:
        qmax = qmax_orig
    else:
        if len(qmax)!=len(qmax_orig):
            raise ValueError("Invalid length of qmin")
    if fixed_links is not None:
        import copy
        q = robot.getConfig()
        qmin = copy.copy(qmin)
        qmax = copy.copy(qmax)
        for l in fixed_links:
            i = robot.link(l).getIndex()
            qmin[i] = q[i]
            qmax[i] = q[i]
        robot.setJointLimits(qmin,qmax)
    else:
        robot.setJointLimits(qmin,qmax)
    
    if isinstance(constraint_or_point,IKObjective):
        point_local,_ = constraint_or_point.getPosition()
        constraint = constraint_or_point
    else:
        point_local = constraint_or_point
        constraint = None

    #setup feasibility test
    feasibleTests = _setup_feasible_tests(robot,link,point_local,fixed_links,
        obstacles,load,load_type,gravity,self_collision,feasibility_test)
    def overall_feasibility_test():
        return all(test() for k,test in feasibleTests.items())

    #run tests
    if all_tests:
        points = {}
        points['workspace'] = []
        for name in feasibleTests:
            points[name] = []
        if isinstance(constraint_or_point,IKObjective):
            #first do a run to figure out the possible cells 
            vg_temp,_,_ = _naive_workspace_occupancy(robot,link,point_local,Nsamples,
                resolution,dimensions)
            vg_temp,_,_ = _expand_workspace_occupancy(robot,link,point_local,vg_temp)
            bmin,bmax = [vg_temp.bbox[0],vg_temp.bbox[1],vg_temp.bbox[2]],[vg_temp.bbox[3],vg_temp.bbox[4],vg_temp.bbox[5]]
            cellsize = vectorops.div(vectorops.sub(bmax,bmin),vg_temp.dims)
            vals = vg_temp.getValues()

            #now solve the IK constraint
            i,j,k = vals.nonzero()
            num_restarts = max(5,Nsamples//len(i))
            print("Using",num_restarts,"restarts to solve IK constraint")
            for n in range(len(i)):
                ind = (i[n],j[n],k[n])
                target = vectorops.add(bmin,vectorops.mul(vectorops.add(ind,[0.5]*3),cellsize))  #target at center point
                #try reaching point described by ind. TODO: allow up to cellsize/2 slop
                constraint.setFixedPosConstraint(point_local,target)
                res = ik.solve_global(constraint,iters=20,numRestarts=num_restarts)
                if res:
                    feasible = True
                    for (name,test) in feasibleTests.items():
                        if test():
                            points[name].append(target)
                        else:
                            feasible = False
                    if feasible:
                        points['workspace'].append(target)
            print("With IK constraint, reached",len(points[name]),"/",len(i),"cells")
            res = {}
            for name in points:
                res[name] = compute_occupancy_grid(points[name],resolution=None,dimensions=vg_temp.dims,bounds=(bmin,bmax),value=value)
        else:
            vg_temp,temppoints,configs = _naive_workspace_occupancy(robot,link,point_local,Nsamples,
                resolution,dimensions)
            bmin,bmax = [vg_temp.bbox[0],vg_temp.bbox[1],vg_temp.bbox[2]],[vg_temp.bbox[3],vg_temp.bbox[4],vg_temp.bbox[5]]
            for target,q in zip(temppoints,configs):
                robot.setConfig(q)
                feasible = True
                for (name,test) in feasibleTests.items():
                    if test():
                        points[name].append(target)
                    else:
                        feasible = False
                if feasible:
                    points['workspace'].append(target)
            res = {}
            for name in points:
                res[name] = compute_occupancy_grid(points[name],resolution=None,dimensions=vg_temp.dims,bounds=(bmin,bmax),value=value)
                if name=='workspace':
                    res[name],_,_ = _expand_workspace_occupancy(robot,link,point_local,res[name],overall_feasibility_test)
                else:
                    res[name],_,_ = _expand_workspace_occupancy(robot,link,point_local,res[name],feasibleTests[name])
    else:
        points = []
        if isinstance(constraint_or_point,IKObjective):
            #first do a run to figure out the possible cells        
            vg_temp,_,_ = _naive_workspace_occupancy(robot,link,point_local,Nsamples,
                resolution,dimensions)
            vg_temp,_,_ = _expand_workspace_occupancy(robot,link,point_local,vg_temp)
            bmin,bmax = [vg_temp.bbox[0],vg_temp.bbox[1],vg_temp.bbox[2]],[vg_temp.bbox[3],vg_temp.bbox[4],vg_temp.bbox[5]]
            cellsize = vectorops.div(vectorops.sub(bmax,bmin),vg_temp.dims)
            vals = vg_temp.getValues()

            #now solve the IK constraint
            i,j,k = vals.nonzero()
            for n in range(len(i)):
                ind = (i[n],j[n],k[n])
                target = vectorops.add(bmin,vectorops.mul(vectorops.add(ind,[0.5]*3),cellsize))  #target at center point
                #try reaching point described by ind. TODO: allow up to cellsize/2 slop
                constraint.setFixedPosConstraint(point_local,target)
                res = ik.solve_global(constraint,iters=20,numRestarts=5,feasibilityCheck=overall_feasibility_test)
                if res:
                    points.append(target)
                    
            res = VolumeGrid(points,resolution=None,dimensions=vg_temp.dims,bounds=(bmin,bmax),value=value)
        else:
            vg_temp,points1,_ = _naive_workspace_occupancy(robot,link,point_local,Nsamples,
                resolution,dimensions,overall_feasibility_test)
            vg_temp,points2,_ = _expand_workspace_occupancy(robot,link,point_local,vg_temp)
            bmin,bmax = [v for v in vg_temp.bbox[0:3]],[v for v in vg_temp.bbox[3:6]]
            res = compute_occupancy_grid(points1+points2,dimensions=vg_temp.dims,bounds=(bmin,bmax),value=value)

    #restore original limits
    robot.setJointLimits(qmin_orig,qmax_orig)

    return res



def compute_workspace_field(link : RobotModelLink,
                            constraint_or_point : Union[Vector3,IKObjective],
                            value : Union[str,Callable],
                            Nsamples=100000,
                            resolution=0.05,
                            dimensions=None,
                            fixed_links : Sequence[Union[int,str]] = None,
                            qmin : Config = None,
                            qmax : Config = None,
                            obstacles : Sequence[Union[RigidObjectModel,TerrainModel]] = None,
                            load : Union[float,Vector3] = None,
                            load_type : str = None,
                            gravity : Vector3 = (0,0,-9.8),
                            self_collision : bool = True,
                            feasibility_test : Callable = None) -> VolumeGrid:
    """Compute a scalar field over the reachable workspace of a point on a 
    robot's end effector. Has several options to ensure various feasibility 
    conditions. Ensures that the robot does not collide with itself, optionally
    with obstacles.
    
    Arguments:
        link (RobotModelLink): the link on the robot for which the workspace
            should be computed
        constraint_or_point (3-vector or IKObjective): the point on the link.
            For fixed orientations, use an IKObjective.
        value (str or callable): 'occupancy', 'manipulability',
            'max manipulability', 'min manipulability', 'load', or an arbitrary
            function f(q) -> float.
        Nsamples (int): the number of configuration samples to use
        resolution (float, 3-vector, or None): the resolution of the resulting
            grid.
        dimensions (int or 3-vector optional): if resolution=None and
            dimensions is given, the number of dimensions. If a single int, the
            cell size is determined by dividing the longest side by dimensions.
        fixed_links (list of int or str, optional): list of fixed link indices
        qmin (Config, optional): overrides the robots lower joint limits
        qmax (Config, optional): overrides the robots upper joint limits
        obstacles (list of RigidObjectModel or TerrainModel): Ensures that the
            robot is collision free with environment obstacles
        load (float or vector, optional): a required load capacity, either an
            isotropic load or a specific vector
        load_type (str, optional): either 'auto' (default), 'isotropic',
            'force', 'torque', 'wrench', 'force_local', 'torque_local', or
            'wrench_local'.
        gravity (3-vector, optional): if load testing is used, the gravity
            vector used for gravity compensation.
        self_collision (bool): whether self collision should be tested.
        feasibility_test (callable, list, or dict): a function
            feasible(q)->bool that tests whether the configuration is feasible.
            If a list, a list of callables. If a dict, a map of str->callable.

    Returns:
        The grid of reached points.
        If ``all_tests=True``, gives a set of grids of reached points.  Key
        'workspace' gives the reachable workspace of points meeting every
        feasibility condition.  Keys 'self_collision_free',
        'obstacle_X_collision_free' (X in range 0,...,len(obstacles)-1),
        'load', 'feasibility_test' give the set of points passing each
        feasibility condition.
    """
    robot = link.robot()
    qmin_orig,qmax_orig = robot.getJointLimits()
    
    if qmin is None:
        qmin = qmin_orig
    else:
        if len(qmin)!=len(qmin_orig):
            raise ValueError("Invalid length of qmin")
    if qmax is None:
        qmax = qmax_orig
    else:
        if len(qmax)!=len(qmax_orig):
            raise ValueError("Invalid length of qmin")
    if fixed_links is not None:
        import copy
        q = robot.getConfig()
        qmin = copy.copy(qmin)
        qmax = copy.copy(qmax)
        for l in fixed_links:
            i = robot.link(l).getIndex()
            qmin[i] = q[i]
            qmax[i] = q[i]
        robot.setJointLimits(qmin,qmax)
    else:
        robot.setJointLimits(qmin,qmax)

    if isinstance(constraint_or_point,IKObjective):
        point_local,_ = constraint_or_point.getPosition()
        constraint = constraint_or_point
    else:
        point_local = constraint_or_point
        constraint = None

    #setup feasibility test
    feasibleTests = _setup_feasible_tests(robot,link,point_local,fixed_links,
        obstacles,load,load_type,gravity,self_collision,feasibility_test)
    def overall_feasibility_test():
        return all(test() for k,test in feasibleTests.items())

    if isinstance(value,callable):
        value_fn = lambda : value(robot.getConfig())
    elif value == 'occupancy':
        value_fn = lambda : 1
    elif value == 'load':
        assert 'load' in feasibleTests
        raise NotImplementedError("TODO: do load calculation")
    elif value.endswith('manipulability'):
        raise NotImplementedError("TODO: do manipulability calculation")
    else:
        raise ValueError("Invalid value to output")

    points = []
    values = []
    if isinstance(constraint_or_point,IKObjective):
        #first do a run to figure out the possible cells        
        vg_temp,_,_ = _naive_workspace_occupancy(robot,link,point_local,Nsamples,
                resolution,dimensions)
        bmin,bmax = [vg_temp.bbox[0],vg_temp.bbox[1],vg_temp.bbox[2]],[vg_temp.bbox[3],vg_temp.bbox[4],vg_temp.bbox[5]]
        cellsize = vectorops.div(vectorops.sub(bmax,bmin),vg_temp.dims)
        vals = vg_temp.getValues()

        #now solve the IK constraint
        i,j,k = vals.nonzero()
        for n in range(len(i)):
            ind = (i[n],j[n],k[n])
            target = vectorops.add(bmin,vectorops.mul(vectorops.add(ind,[0.5]*3),cellsize))  #target at center point
            #try reaching point described by ind. TODO: allow up to cellsize/2 slop
            constraint.setFixedPosConstraint(point_local,target)
            res = ik.solve_global(constraint,iters=20,numRestarts=5,feasibilityCheck=overall_feasibility_test)
            if res:
                points.append(target)
                values.append(value_fn())

        res = VolumeGrid(points,dimensions=vg_temp.dims,bounds=(bmin,bmax),value=value)
    else:
        vg_temp,points1,configs1 = _naive_workspace_occupancy(robot,link,point_local,Nsamples,
                resolution,dimensions,overall_feasibility_test)
        vg_temp,points2,configs2 = _expand_workspace_occupancy(robot,link,point_local,vg_temp,overall_feasibility_test)
        for (p,q) in zip(points1+points2,configs1+configs2):
            robot.setConfig(q)
            if overall_feasibility_test():
                points.append(p)
                values.append(value_fn())
        res = compute_field_grid(points,values,resolution,dimensions,value=value)

    #restore original limits
    robot.setJointLimits(qmin_orig,qmax_orig)

    return res


def _setup_feasible_tests(robot,link,point_local,fixed_links,
    obstacles,load,load_type,gravity,
    self_collision,feasibility_test):
    feasibleTests = {}
    if self_collision:
        feasibleTests['self_collision_free'] = lambda: not robot.selfCollides()
    if obstacles is not None:
        free_links = []
        for i in range(robot.numLinks()):
            if i not in fixed_links:
                free_links.append(i)
        def collision_free_obstacle(obstacle):
            for j in free_links:
                if robot.link(j).geometry().collides(obstacle.geometry()):
                    return False
            return True
        for i,o in enumerate(obstacles):
            feasibleTests['obstacle_{}_collision_free'.format(i)] = lambda : collision_free_obstacle(o)
    
    if load is not None:
        tmax = np.asarray(robot.getTorqueLimits())
        if fixed_links is not None:
            for l in fixed_links:
                i = robot.link(l).getIndex()
                tmax[i] = float('inf')
    
        #|J^T f| <= tmax
        if load_type == 'isotropic':
            #for isotropic, test if |G(q)-J^T f| <= tmax for all |f|<=load
            #test whether tmax_i <= max |gi - Ai f|  s.t. |f| <= load
            fixed = set(fixed_links)
            def satisfies_load():
                G = robot.getGravityForces(gravity)
                Jp = link.getPositionJacobian(point_local)
                for i in range(robot.numLinks()):
                    if i in fixed: continue
                    d = Jp[:,i]
                    g = G[i]
                    max_torque_i = abs(g) + load*vectorops.norm(d)
                    if max_torque_i > tmax[i]:
                        return False
                return True
            feasibleTests['load'] = satisfies_load
        elif load_type == 'force':
            if len(load) != 3:
                raise ValueError("Need load to be a 3-vector")
            def satisfies_load():
                G = np.asarray(robot.getGravityForces(gravity))
                Jp = link.getPositionJacobian(point_local)
                return np.all(np.abs(G-np.dot(Jp.T,load)) <= tmax)
            feasibleTests['load'] = satisfies_load
        elif load_type == 'torque':
            if len(load) != 3:
                raise ValueError("Need load to be a 3-vector")
            def satisfies_load():
                G = np.asarray(robot.getGravityForces(gravity))
                Jo = link.getOrientationJacobian()
                return np.all(np.abs(G-np.dot(Jo.T,load)) <= tmax)
            feasibleTests['load'] = satisfies_load
        elif load_type == 'wrench':
            if len(load) != 6:
                raise ValueError("Need load to be a 6-vector")
            def satisfies_load():
                G = np.asarray(robot.getGravityForces(gravity))
                Jo = link.getJacobian(point_local)
                return np.all(np.abs(G-np.dot(Jo.T,load)) <= tmax)
            feasibleTests['load'] = satisfies_load
        else:
            raise NotImplementedError("TODO: local loads")
    if feasibility_test is not None:
        if isinstance(feasibility_test,(list,tuple)):
            for (name,test) in enumerate(feasibility_test):
                if not callable(test):
                    raise ValueError("Feasibility test {} must be callable".format(name))
                feasibleTests['feasibility_test_{}'.format(name)] = lambda: test(robot.getConfig())
        elif isinstance(feasibility_test,dict):
            for (name,test) in feasibility_test.items():
                if not callable(test):
                    raise ValueError("Feasibility test {} must be callable".format(name))
                feasibleTests[name] = lambda: test(robot.getConfig())
        else:
            if not callable(feasibility_test):
                raise ValueError("Feasibility test must be callable")
            feasibleTests['feasibility_test'] = lambda: feasibility_test(robot.getConfig())
    return feasibleTests

def _naive_workspace_occupancy(robot,link,point_local,
    Nsamples,resolution,dimensions,feasibility_test=None):
    if feasibility_test is None:
        feasibility_test = lambda : True
    points = []
    configs = []
    for i in range(Nsamples):
        robot.randomizeConfig()
        if feasibility_test():
            points.append(link.getWorldPosition(point_local))
            configs.append(robot.getConfig())
    vg_temp = compute_occupancy_grid(points,resolution,dimensions)
    return vg_temp,points,configs

def _expand_workspace_occupancy(robot,link,point_local,vg_temp,
    feasibility_test=None):
    bmin,bmax = [vg_temp.bbox[0],vg_temp.bbox[1],vg_temp.bbox[2]],[vg_temp.bbox[3],vg_temp.bbox[4],vg_temp.bbox[5]]
    cellsize = vectorops.div(vectorops.sub(bmax,bmin),vg_temp.dims)
    vals = vg_temp.getValues()
    
    fringe = set()
    def add_neighbors(x,y,z):
        if x-1 >= 0 and vals[x-1,y,z]==0:
            fringe.add((x-1,y,z))
        if x+1 < vals.shape[0] and vals[x+1,y,z]==0:
            fringe.add((x+1,y,z))
        if y-1 >= 0 and vals[x,y-1,z]==0:
            fringe.add((x,y-1,z))
        if y+1 < vals.shape[1] and vals[x,y+1,z]==0:
            fringe.add((x,y+1,z))
        if z-1 >= 0 and vals[x,y,z-1]==0:
            fringe.add((x,y,z-1))
        if z+1 < vals.shape[2] and vals[x,y,z+1]==0:
            fringe.add((x,y,z+1))

    #initialize neighbors  of visited cells
    i,j,k = vals.nonzero()
    for n in range(len(i)):
        add_neighbors(i[n],j[n],k[n])

    points = []
    configs = []
    numTests = 0
    numSuccesses = 0
    while len(fringe) > 0:
        numTests += 1
        cell = next(iter(fringe))
        center = vectorops.add(bmin,vectorops.mul(vectorops.add(cell,[0.5]*3),cellsize))
        obj = ik.objective(link,local=point_local,world=center)
        if ik.solve_global(obj,iters=20,numRestarts=5,feasibilityCheck=feasibility_test):
            numSuccesses += 1
            vals[cell] = 1.0
            add_neighbors(cell[0],cell[1],cell[2])
            points.append(center)
            configs.append(robot.getConfig())
        fringe.remove(cell)
    print("Workspace expansion succeeded in {}/{} tests".format(numSuccesses,numTests))
    vg_temp.setValues(vals)
    return vg_temp,points,configs