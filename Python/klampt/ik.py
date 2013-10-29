"""Helpers to make inverse kinematics more convenient (and Pythonic)"""

from robotsim import *

def objective(body,ref=None,local=None,world=None,R=None,t=None):
    """Returns an IKObjective or GeneralizedIKObjective for a given body.
    If ref is provided, there is a target body.

    If local and world are provided (either as single 3D points, or a list of
    3D points), then this objective will match the local point(s) to the
    world point(s).

    If R and t are provided, then the objective is set to a relative
    transformation between body and ref.
    """
    generalized = False
    if not hasattr(body,'getRobot'):
        generalized = True
    else: 
        if ref and (not hasattr(ref,'getRobot') or ref.getRobot() != body.getRobot()):
            generalized=True

    if generalized:
        obj = None
        if ref:
            obj = GeneralizedIKObjective(body,ref)
        else:
            obj = GeneralizedIKObjective(body)
        if local and world:
            assert(len(local)==len(world))
            if hasattr(local[0],'__iter__'):
                #sanity check
                for pt in local:
                    assert(len(pt)==3)
                for pt in world:
                    assert(len(pt)==3)
                obj.setPoints(local,world)
            else:
                obj.setPoint(local,world)
        elif R and t:
            obj.setTransform(R,t)
        else:
            raise RuntimeError("Need to specify either local and world or R and t")
        return obj
    else:
        obj = IKObjective()
        obj.robot = body.getRobot()
        if local and world:
            assert(len(local)==len(world))
            if hasattr(local[0],'__iter__'):
                #sanity check
                for pt in local:
                    assert(len(pt)==3)
                for pt in world:
                    assert(len(pt)==3)
                if ref:
                    obj.setRelativePoints(body.index,ref.index,local,world)
                else:
                    obj.setFixedPoints(body.index,local,world)
            else:
                if ref:
                    obj.setRelativePoint(body.index,ref.index,local,world)
                else:
                    obj.setFixedPoint(body.index,local,world)
        elif R and t:
            if ref:
                obj.setRelativeTransform(body.index,ref.index,R,t)
            else:
                obj.setFixedTransform(body.index,R,t)
        else:
            raise RuntimeError("Need to specify either local and world or R and t")
        return obj

def objects(objectives):
    """Returns a list of all objects touched by the given objective(s).
    
    If using any regular IKObjectives, the .robot member must be set (see the
    objectives() function)."""
    raise NotImplementedError()
    pass

def solver(objectives):
    """Returns either an IKSolver, list of IKSolver's, or a
    GeneralizedIKSolver corresponding to the given objective(s).
    
    If using any regular IKObjectives, the .robot member must be set (see the
    objectives() function).
    """
    if hasattr(objectives,'__iter__'):
        generalized = []
        robs = dict()
        for obj in objectives:
            if isinstance(obj,IKObjective):
                robs.getdefault(obj.robot,[]).append(obj)
            elif isinstance(obj,GeneralizedIKObjective):
                generalized.append(obj)
            else:
                raise TypeError("Objective is of wrong type")
        if len(generalized) != 0:
            #need a generalized solver
            world = None
            if generalized[0].isObj1:
                world = WorldModel(generalized[0].obj1.world)
            else:
                world = WorldModel(generalized[0].link1.world)
            s = GeneralizedIKSolver(world)
            for obj in generalized:
                s.add(obj)
            for (r,objs) in robs.iteritems():
                for obj in objs:
                    s.add(GeneralizedIKObjective(r,obj))
            return s
        else:
            res = []
            for (r,objs) in robs:
                s = IKSolver(r)
                for obj in objs:
                    s.add(obj)
                res.append(s)
            if len(res)==1:
                return res[0]
            return res
    else:
        if isinstance(objectives,IKObjective):
            s = IKSolver(objectives.robot)
            s.add(objectives)
            return s
        elif isinstance(objectives,GeneralizedIKObjective):
            world = None
            if objectives.isObj1:
                world = WorldModel(objectives.obj1.world)
            else:
                world = WorldModel(objectives.link1.world)
            s = GeneralizedIKSolver(world)
            s.add(objectives)
            return s
        else:
            raise TypeError("Objective is of wrong type")

def solve(objectives,iters=1000,tol=1e-3):
    """Attempts to solve the given objectives.  Returns true if successful.
    
    If using any regular IKObjectives, the .robot member must be set (see the
    objectives() function).
    """
    s = solver(objectives)
    if hasattr(s,'__iter__'):
        res = [si.solve(iters,tol)[0] for si in s]
        return all(res)
    else:
        return s.solve(iters,tol)[0]

def residual(objectives):
    """Returns the residual of the given objectives."""
    return solver(objectives).getResidual()

def jacobian(objectives):
    """Returns the jacobian of the given objectives."""
    return solver(objectives).getJacobian()
