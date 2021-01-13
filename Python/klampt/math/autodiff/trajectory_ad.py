"""Klampt trajectory AD functions:

 =====================  =============  ========================================
 Function               Derivative     Notes
 =====================  =============  ========================================
 hermite                Y              hermite(x1,v1,x2,v2,u) -> R^n
 GeodesicInterpolate    1(u)           ginterp[geodesic](a,b,u) -> R^n
 GeodesicDistance       1              gdist[geodesic](a,b) -> R
 TrajectoryEval         1              eval[Trajectory,endBehavior](t) -> R^n
 Trajectory             1              trajectory[n,end](ts,xs,t) -> R^n
 =====================  =============  ========================================

Module summary
~~~~~~~~~~~~~~

.. autosummary::
    hermite
    GeodesicInterpolate
    GeodesicDistance
    TrajectoryEval
    Trajectory

"""


import numpy as np 
from .ad import ADFunctionInterface,ADFunctionCall,ADTerminal,function,_scalar
from ...model import trajectory
from .. import spline
from ..geodesic import GeodesicSpace
from klampt import RobotModel

def _hermite(x1,v1,x2,v2,u):
    assert len(x1)==len(v1)
    assert len(x1)==len(x2)
    assert len(x1)==len(v2)
    u2 = u*u
    u3 = u*u*u
    cx1 = 2.0*u3-3.0*u2+1.0
    cx2 = -2.0*u3+3.0*u2
    cv1 = (u3-2.0*u2+u)
    cv2 = (u3-u2)
    return cx1*x1 + cx2*x2 + cv1*v1 + cv2*v2

def _hermite_jvp(arg,darg,x1,v1,x2,v2,u):
    if arg==4:
        return np.array(spline.hermite_deriv(x1,v1,x2,v2,u))*darg[0]
    u2 = u*u
    u3 = u*u*u
    if arg==0:
        cx1 = 2.0*u3-3.0*u2+1.0
        return cx1*darg
    elif arg==1:
        cv1 = (u3-2.0*u2+u)
        return cv1*darg
    elif arg==2:
        cx2 = -2.0*u3+3.0*u2
        return cx2*darg
    else:
        cv2 = (u3-u2)
        return cv2*darg

def _hermite_gen_deriv(args,x1,v1,x2,v2,u):
    assert len(args) >= 2
    numu = len([a for a in args if a==4])
    if numu==len(args):
        #only deriv w.r.t. u
        res = np.array(spline.hermite_deriv(x1,v1,x2,v2,u,order=len(args)))
        #resize to the right tensor dimension
        for a in args:
            res=res[:,np.newaxis]
        return res
    if len(args)-numu > 1:
        #bilinear-ish
        return 0
    if numu == 1:
        u2 = u*u
        dcx1 = (6.0*u2-6.0*u)
        dcx2 = (-6.0*u2+6.0*u)
        dcv1 = 3.0*u2-4.0*u+1.0
        dcv2 = 3.0*u2-2.0*u
        coeffs = [dcx1,dcv1,dcx2,dcv2]
        assert len(args)==2
        if args[0] != 4:
            res = np.diag([coeffs[args[0]]]*len(x1))
            return res[:,:,np.newaxis]
        elif args[1] != 4:
            res = np.diag([coeffs[args[1]]]*len(x1))
            return res[:,np.newaxis,:]
        assert False,"This code should not be reached"
    elif numu == 2:
        ddcx1 = 12*u
        ddcx2 = -12.0*u
        ddcv1 = 6.0*u-4.0
        ddcv2 = 6.0*u-2.0
        coeffs = [ddcx1,ddcv1,ddcx2,ddcv2]
        res = None
        for a in args:
            if a!=4:
                res = np.diag([coeffs[a]]*len(x1))
        assert res is not None,"This code should not be reached"
        uhit = False
        for a in args:
            if a==4:
                #TODO: figure how to extend axis properly
                if uhit:
                    res = res[:,np.newaxis]
                else:
                    res = res[:,np.newaxis,:]
        return res
    elif numu == 3:
        cx1 = 12
        cx2 = -12.0
        cv1 = 6.0
        cv2 = 6.0
        coeffs = [cx1,cv1,cx2,cv2]
        res = None
        for a in args:
            if a!=4:
                res = np.diag([coeffs[a]]*len(x1))
        assert res is not None,"This code should not be reached"
        uhit = False
        for a in args:
            if a==4:
                #TODO: figure how to extend axis properly
                if uhit:
                    res = res[:,np.newaxis]
                else:
                    res = res[:,np.newaxis,:]
        return res
    else:
        return 0


hermite = function(_hermite,'trajectory.hermite',[-1,-1,-1,-1,1],-1,['x1','v1','x2','v2','u'],jvp=_hermite_jvp,gen_derivative=_hermite_gen_deriv)
"""Autodiff'ed version of the :func:`klampt.math.spline.hermite` function. Takes
Arguments (x1,v1,x2,v2,u) defining a Hermite curve with endpoints x1,x2 and
velocities v1,v2, and returns the interpolant at parameter u in [0,1]."""

class GeodesicInterpolate(ADFunctionInterface):
    """Autodiff wrapper of geodesic.interpolate(a,b,u)."""
    def __init__(self,geodesic):
        self.geodesic = geodesic
    def __str__(self):
        return "trajectory.GeodesicInterpolate[%s]"%(self.geodesic.__class__.__name__)
    def n_args(self):
        return 3
    def n_in(self,arg):
        if arg==0 or arg==1:
            return self.geodesic.extrinsicDimension()
        return 1
    def n_out(self):
        return self.geodesic.extrinsicDimension()
    def eval(self,a,b,u):
        x = self.geodesic.interpolate(a.tolist(),b.tolist(),u)
        return np.array(x)
    def jvp(self,arg,darg,a,b,u):
        if arg==2:
            x = self.geodesic.interpolate(a.tolist(),b.tolist(),u)
            return (np.array(self.geodesic.difference(b,x))-np.array(self.geodesic.difference(a,x)))*darg[0]
        raise NotImplementedError()


class GeodesicDistance(ADFunctionInterface):
    """Autodiff wrapper of geodesic.distance(a,b)."""
    def __init__(self,geodesic):
        self.geodesic = geodesic
    def __str__(self):
        return "trajectory.GeodesicDistance[%s]"%(self.geodesic.__class__.__name__)
    def n_args(self):
        return 2
    def n_in(self,arg):
        return self.geodesic.extrinsicDimension()
    def n_out(self):
        return 1
    def eval(self,a,b):
        return self.geodesic.distance(a.tolist(),b.tolist())
    def jvp(self,arg,darg,a,b):
        d = self.eval(a,b)
        if arg==0:
            return np.array(self.geodesic.difference(b,a)).dot(darg)/d
        else:
            return np.array(self.geodesic.difference(a,b)).dot(darg)/d


class TrajectoryEval(ADFunctionInterface):
    """Autodiff wrapper of traj.eval(t). 

    Args:
        traj (Trajectory): the trajectory to evaluate. Can be any Trajectory
            subclass.
        endBehavior (string): how to evaluate out-of-domain times. Can be
            'halt' or 'loop'.
    """
    def __init__(self,traj,endBehavior='halt'):
        self.traj = traj
        if isinstance(self.traj,trajectory.SE3Trajectory) or isinstance(self.traj,trajectory.SE3HermiteTrajectory):
            self.flatten=lambda x:x[0]+x[1]
        else:
            self.flatten=lambda x:x
        self.endBehavior = endBehavior
        assert len(self.traj.milestones) > 0
    def __str__(self):
        if self.endBehavior == 'halt':
            return "trajectory.TrajectoryEval[%s,%d segments,%f,%f]"%(self.traj.__class__.__name__,len(self.traj.times)-1,self.traj.startTime(),self.traj.endTime())
        else:
            return "trajectory.TrajectoryEval[%s,%d segments,%s]"%(self.traj.__class__.__name__,len(self.traj.times)-1,self.endBehavior)
    def n_args(self):
        return 1
    def n_in(self,arg):
        return 1
    def n_out(self):
        return len(self.flatten(self.traj.waypoint(self.traj.milestones[0])))
    def eval(self,t):
        x = self.traj.eval(t,self.endBehavior)
        return np.array(self.flatten(x))
    def derivative(self,arg,t):
        assert arg == 0
        return np.array(self.flatten(self.traj.deriv(t,self.endBehavior)))[:,np.newaxis]
    def jvp(self,arg,darg,t):
        assert arg == 0
        return np.array(self.flatten(self.traj.deriv(t,self.endBehavior)))*darg


class Trajectory(ADFunctionInterface):
    """Autodiff piecewise linear interpolation traj(times,milestone_stack,t).  
    The dimensionality or a robot must be provided on initialization.

    ``milestone_stack`` is a flattened array of the trajectory's milestones.

    Also, ``t`` can be either a scalar or an array of times.  Providing an 
    array of times is faster than calling this repeatedly for multiple times.

    Args:
        n_or_robot (int or RobotModel): the dimension of the state, or the
            robot. In this case the milestones must be robot configurations.
        endBehavior (string): how to evaluate out-of-domain times. Can be
            'halt' or 'loop'.
    """
    def __init__(self,n_or_robot,endBehavior='halt'):
        if isinstance(n_or_robot,RobotModel):
            self.robot = n_or_robot
            self.n = self.robot.numLinks()
        else:
            self.n = n_or_robot
            self.robot = None
        self.endBehavior = endBehavior
    def __str__(self):
        if self.robot is None:
            return "trajectory.Trajectory[%d]"%(self.n,)
        else:
            return "trajectory.Trajectory[%s]"%(self.robot.getName(),)
    def n_args(self):
        return 3
    def n_in(self,arg):
        return -1
    def n_out(self):
        return -1
    def eval(self,ts,xs,t):
        if xs.shape != (len(ts)*self.n,):
            raise ValueError("Invalid size of the xs array, must be a stacked vector of milestones of size %d x %d"%(len(ts),self.n))
        if self.robot is None:
            traj = trajectory.Trajectory(ts,xs.reshape(len(ts),self.n))
        else:
            traj = trajectory.RobotTrajectory(self.robot,ts,xs.reshape(len(ts),self.n))
        if _scalar(t):
            return np.array(traj.eval(t),self.endBehavior)
        else:
            return np.array([traj.eval(v,self.endBehavior) for v in t])
    def jvp(self,arg,darg,ts,xs,t):
        if xs.shape != (len(ts)*self.n,):
            raise ValueError("Invalid size of the xs array, must be a stacked vector of milestones of size %d x %d"%(len(ts),self.n))
        if self.robot is None:
            traj = trajectory.Trajectory(ts,xs.reshape(len(ts),self.n))
        else:
            traj = trajectory.RobotTrajectory(self.robot,ts,xs.reshape(len(ts),self.n))
        if arg==2:
            if _scalar(t):
                return np.array(traj.deriv(t,self.endBehavior))
            else:
                return np.array([traj.deriv(v,self.endBehavior) for v in t])
        else:
            if darg.shape != (self.n,):
                raise ValueError("Invalid size of the darg array, must have size %d"%(self.n,))
            if _scalar(t):
                segs = [traj.getSegment(t,self.endBehavior)]
            else:
                segs = [traj.getSegment(v,self.endBehavior) for v in t]
            derivs = []
            for index,(i,u) in enumerate(segs):
                if i<0 or i+1 >= len(ts): derivs.append(np.zeros(self.n))
                ta = ts[i]
                tb = ts[i+1]
                a = xs[i*self.n:(i+1)*self.n]
                b = xs[(i+1)*self.n:(i+2)*self.n]
                if arg==0:
                    #u = (t-ta)/(tb-ta)
                    #x = a+u*(b-a)
                    #du = [-dta*(tb-ta)  - (t-ta)*(dtb-dta)]/(tb-ta)^2 = [dta*(t-tb) + (ta-t)*dtb]/(tb-ta)^2
                    if ta==tb:
                        derivs.append(np.zeros(self.n))
                    else:
                        dta = darg[i]
                        dtb = darg[i+1]
                        du = -(dta*(1-u) + dtb*u) /(tb-ta)
                        derivs.append(du*(b-a))
                else:
                    #x = (1-u)a+ub
                    da = darg[i*self.n:(i+1)*self.n]
                    db = darg[(i+1)*self.n:(i+2)*self.n]
                    derivs.append((1-u)*da + u*db)
            if _scalar(t):
                return derivs[0]
            return np.array(derivs)

