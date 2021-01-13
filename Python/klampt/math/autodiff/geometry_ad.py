"""Klampt geometry AD functions:

 ======================  =============  ======================================
 Function                Derivative     Notes
 ======================  =============  ======================================
 BoxPointMargin          Y              margin[bmin,bmax](x) -> R
 BoxPointDistance        1              D[bmin,bmax](x) -> R
 BoxPointClosest         Y              CP[bmin,bmax](x) -> R^n
 sphere_point_distance   Y              D(c,r,x) -> R
 sphere_point_closest    Y              CP(c,r,x) -> R^n
 sphere_sphere_distance  Y              D(c1,r1,c2,r2) -> R
 sphere_sphere_closest   Y              CP(c1,r1,c2,r2) -> R^n
 GeomPointDistance       1              D[geom](Tgeom,pt) -> R
 GeomSphereDistance      1              D[geom](Tgeom,c,r) -> R
 GeomPointClosest        1              CP[geom](Tgeom,pt) -> R^3
 GeomGeomDistance        1              D[geom1,geom2](Tgeom1,Tgeom2) -> R
 GeomGeomClosest         1              CPs[geom1,geom2](Tgeom1,Tgeom2) -> R^6
 GeomRayCast             1              RC[geom](Tgeom,s,d) -> R
 MinDistance             1              Faster than min XYDistance() 
 ======================  =============  ======================================

Note that all the GeomX functions' derivatives should depend on the local shape
of the geometry, but we don't actually take that into account, so their
analytical derivatives may not be accurate.  For example, if the closest point
to x on a triangle mesh lies on a vertex, then its derivative w.r.t. x is 0.
If it lies on an edge, then it depends on the edge direction. If it lies on
a face, then it depends on the face's normal.  Future versions may support such
reasoning.
"""

from inspect import indentsize
import numpy as np 
from .ad import ADFunctionInterface,function,minimum,maximum,stack
from . import math_ad,so3_ad,se3_ad
from .. import vectorops,so3,se3
from ...robotsim import Geometry3D,DistanceQuerySettings

def _array_list_equal(list1,list2):
    assert len(list1) == len(list2)
    return all(np.array_equal(a,b) for (a,b) in zip(list1,list2))

def _array_list_copy(list1):
    return [np.copy(A) for A in list1]

class BoxPointMargin(ADFunctionInterface):
    """Returns the inner margin of a point x inside a bounding box [bmin,bmax].
    I.e., the minimum distance to the box boundaries, which is positive inside
    and negative outside.
    """
    def __init__(self,bmin,bmax):
        self.bmin = np.asarray(bmin)
        self.bmax = np.asarray(bmax)
        assert self.bmin.shape == self.bmax.shape
        assert len(self.bmin.shape) == 1
    def __str__(self):
        return "geometry.BoxPointMargin[%s,%s]"%(str(self.bmin),str(self.bmax))
    def n_args(self):
        return 1
    def n_in(self,arg):
        return len(self.bmin)
    def n_out(self):
        return 1
    def eval(self,x):
        return min((x - self.bmin).min(),(self.bmax - x).min())
    def jvp(self,arg,dx,x):
        assert arg == 0
        amin_bmin = np.argmin(x - self.bmin)
        amin_bmax = np.argmin(self.bmax - x)
        if x[amin_bmin] - self.bmin[amin_bmin] < self.bmax[amin_bmax] - x[amin_bmax]:
            return dx[amin_bmin]
        else:
            return -dx[amin_bmax]
    def gen_derivative(self,arg,x):
        if len(arg) == 1:
            return self.derivative(arg,x)
        else:
            return 0


class BoxPointDistance(ADFunctionInterface):
    """Returns the (signed) distance of a point x to a bounding box
    [bmin,bmax].  The result is positive outside, and negative inside.
    """
    def __init__(self,bmin,bmax):
        self.bmin = np.asarray(bmin)
        self.bmax = np.asarray(bmax)
        assert self.bmin.shape == self.bmax.shape
        assert len(self.bmin.shape) == 1
    def __str__(self):
        return "geometry.BoxPointDistance[%s,%s]"%(str(self.bmin),str(self.bmax))
    def n_args(self):
        return 1
    def n_in(self,arg):
        return len(self.bmin)
    def n_out(self):
        return 1
    def eval(self,x):
        xclose = np.minimum(np.maximum(x,self.bmin),self.bmax)
        if np.array_equal(xclose,x):
            #return negative margin
            return -min((x - self.bmin).min(),(self.bmax - x).min())
        return np.linalg.norm(x-xclose)
    def jvp(self,arg,dx,x):
        assert arg==0
        xclose = np.minimum(np.maximum(x,self.bmin),self.bmax)
        if np.array_equal(xclose,x):
            amin_bmin = np.argmin(x - self.bmin)
            amin_bmax = np.argmin(self.bmax - x)
            if x[amin_bmin] - self.bmin[amin_bmin] < self.bmax[amin_bmax] - x[amin_bmax]:
                return -dx[amin_bmin]
            else:
                return dx[amin_bmax]
        else:
            d = x-xclose
            return np.dot(d,dx)/np.linalg.norm(d)


class BoxPointClosest(ADFunctionInterface):
    """Returns the closest point to x within the bounding box [bmin,bmax].
    """
    def __init__(self,bmin,bmax):
        self.bmin = np.asarray(bmin)
        self.bmax = np.asarray(bmax)
        assert self.bmin.shape == self.bmax.shape
        assert len(self.bmin.shape) == 1
    def __str__(self):
        return "geometry.BoxPointClosest[%s,%s]"%(str(self.bmin),str(self.bmax))
    def n_args(self):
        return 1
    def n_in(self,arg):
        return len(self.bmin)
    def n_out(self):
        return len(self.bmin)
    def eval(self,x):
        return np.minimum(np.maximum(x,self.bmin),self.bmax)
    def jvp(self,arg,dx,x):
        assert arg==0
        xclose = np.minimum(np.maximum(x,self.bmin),self.bmax)
        res = np.copy(dx)
        res[x!=xclose] = 0
        return res
    def gen_derivative(self,arg,x):
        if len(arg) == 1:
            return self.derivative(arg,x)
        else:
            return 0

def sphere_point_distance(c,r,x):
    """Autodiff function D(c,r,x) giving the distance from a sphere with center
    c and radius r to a point x"""
    return math_ad.distance(c,x)-r

def sphere_point_closest(c,r,x):
    """Autodiff function CP(c,r,x) giving the closest point from a sphere with
    center c and radius r to a point x"""
    diff = c - x
    d = math_ad.norm(diff)
    return c + minimum(d,r)/d * diff

def sphere_sphere_distance(c1,r1,c2,r2):
    """Autodiff function D(c1,r1,c2,r2) giving the distance from a sphere with
    center c1, radius r1 to a sphere with center c2, radius r2."""
    return math_ad.distance(c1,c2)-r1-r2

def sphere_sphere_closest(c1,r1,c2,r2):
    """Autodiff function CP(c1,r1,c2,r2)->R^2n giving the closest points 
    between a sphere with center c1, radius r1 to a sphere with center c2, 
    radius r2. The two points are stacked into a length 2n vector. """
    diff = c1 - c2
    d = math_ad.norm(diff)
    return stack(c1 + minimum(d,r1)/d * diff,c2 - minimum(d,r2)/d * diff)


def _geom_str(geom):
    gtype = geom.type()
    if gtype == 'GeometricPrimitive':
        gprim = geom.getGeometricPrimitive()
        return gprim.type
    else:
        if gtype == 'TriangleMesh':
            return '%s with %d tris'%(gtype,geom.numElements())
        elif gtype == 'PointCloud':
            return '%s with %d points'%(gtype,geom.numElements())
        elif gtype == 'Group':
            return '%s with %d sub-elements'%(gtype,geom.numElements())
        else:
            return gtype

def _shape_dim(geom):
    """Returns 0 if the object should be treated as points, 1 if it's line segments, or 2 if it's smooth"""
    gtype = geom.type()
    if gtype == 'GeometricPrimitive':
        gprim = geom.getGeometricPrimitive()
        if gprim.type == 'Point':
            return 0
        if gprim.type == 'Segment':
            return 1
        return 2
    elif gtype == 'PointCloud':
        return 0
    elif gtype == 'Group':
        return max(_shape_dim(geom.getElement(i) for i in range(geom.numElements())))
    else:
        return 2


class GeomPointDistance(ADFunctionInterface):
    """Autodiff wrapper of Geometry3D.distance_point.  This is a function
    D(Tgeom,x) where Tgeom is the transform of the geometry.

    Note that this will make various assumptions about the local geometry
    shape, and how the closest point varies w.r.t. changes in Tgeom or x.

    .. note::
        This caches the result of eval().  If you change the geometry's
        transform between eval(T,x) and eval(T,y), then incorrect results will
        be returned.
    """
    def __init__(self,geom,name=None,relErr=None,absErr=None,upperBound=None):
        if not isinstance(geom,Geometry3D):
            raise ValueError("GeomPointDistance expects a Geometry3D")
        self.geom = geom
        self.eval_args = (None,None)
        self.eval_res = None
        if name is not None:
            self.name = name
        else:
            self.name = _geom_str(geom)
        self.shape_dims = _shape_dim(self.geom)
        self.settings = None
        if relErr is not None or absErr is not None or upperBound is not None:
            self.settings = DistanceQuerySettings()
            if relErr is not None:
                self.settings.relErr = relErr
            if absErr is not None:
                self.settings.absErr = absErr
            if upperBound is not None:
                self.settings.upperBound = upperBound
    def __str__(self):
        return "geometry.GeomPointDistance[%s]"%(self.name,)
    def n_args(self):
        return 2
    def n_in(self,arg):
        if arg == 0: return se3_ad.SIZE
        else: return 3
    def n_out(self):
        return 1
    def argname(self,arg):
        return ['T','x'][arg]
    def eval(self,T,x):
        if not np.array_equal(self.eval_args[0],T):
            self.geom.setCurrentTransform(*se3_ad.to_klampt(T))
        self.eval_args = _array_list_copy((T,x))
        if self.settings is None:
            self.eval_res = self.geom.distance_point(x.tolist())
        else:
            self.eval_res = self.geom.distance_point_ext(x.tolist(),self.settings)
        return self.eval_res.d
    def jvp(self,arg,darg,T,x):
        if not _array_list_equal(self.eval_args,(T,x)):
            self.eval(T,x)
        if self.settings is not None and self.eval_res.d == self.settings.upperBound:
            return 0
        if self.shape_dims == 0 or not self.eval_res.hasGradients:
            #assume closest point is constant
            if self.eval_res.hasClosestPoints:
                if arg == 0:
                    #d = ||x - p|| with p = T*p_l
                    Tnative = se3_ad.to_klampt(T)
                    Tinv = se3.inv(Tnative)
                    dR,dt = se3_ad.to_klampt(darg)
                    pgeom_loc = se3.apply(Tinv,self.eval_res.cp1)
                    return math_ad._distance_jvp_a(se3.apply((dR,dt),pgeom_loc),self.eval_res.cp1,x)
                else:
                    return math_ad._distance_jvp_a(darg,x,self.eval_res.cp1)
        elif self.shape_dims == 1:
            pass
        elif self.shape_dims == 2:
            if self.eval_res.hasClosestPoints and self.eval_res.hasGradients:
                #approximate geometry at closest point as a plane n^T (p - x) = d 
                if arg == 0:
                    Tnative = se3_ad.to_klampt(T)
                    Tinv = se3.inv(Tnative)
                    dR,dt = se3_ad.to_klampt(darg)
                    #since n = R*n_l and p = R*p_l + t, we have
                    #dd/ds = (p - x)^T dn/ds + n^T dp/ds = (p-x)^T dR/ds n_l + n^T (dR/ds*p_l + dt/ds)
                    disp = vectorops.sub(x,self.eval_res.cp1) 
                    pgeom_loc = se3.apply(Tinv,self.eval_res.cp1)
                    ngeom_loc = se3.apply_rotation(Tinv,self.eval_res.grad1)
                    return vectorops.dot(disp,so3.apply(dR,ngeom_loc)) + vectorops.dot(self.eval_res.grad1,vectorops.add(so3.apply(dR,pgeom_loc),dt))
                else:
                    return -np.dot(self.eval_res.grad1,darg)
        raise NotImplementedError() 


class GeomSphereDistance(ADFunctionInterface):
    """Autodiff wrapper of Geometry3D.distance_point(c)-r.  This is a function
    D(Tgeom,c,r) where Tgeom is the transform of the geometry.

    See :class:`GeomPointClosest` for a description of the arguments.
    """
    def __init__(self,geom,name=None,relErr=None,absErr=None,upperBound=None):
        self.geom_point = GeomPointDistance(geom,name,relErr,absErr,upperBound)
    def __str__(self):
        return "geometry.GeomSphereDistance[%s]"%(self.geom_point.name,)
    def n_args(self):
        return 3
    def n_in(self,arg):
        if arg == 0: return se3_ad.SIZE
        elif arg == 1: return 3
        else: return 1
    def n_out(self):
        return 1
    def argname(self,arg):
        return ['T','c','r'][arg]
    def eval(self,T,c,r):
        return self.geom_point.eval(T,c)-r
    def jvp(self,arg,darg,T,c,r):
        if arg==2:
            return -darg
        else:
            return self.geom_point.jvp(arg,darg,T,c)


class GeomPointClosest(ADFunctionInterface):
    """Autodiff wrapper of Geometry3D.distance_point(x).cp1.  This is a 
    function CP(Tgeom,x) where Tgeom is the transform of the geometry

    .. note::
        This caches the result of eval().  If you change the geometry's
        transform between eval(T,x) and eval(T,y), then incorrect results will
        be returned.
    """
    def __init__(self,geom,name=None):
        if not isinstance(geom,Geometry3D):
            raise ValueError("GeomPointClosest expects a Geometry3D")
        self.geom = geom
        self.eval_args = (None,None)
        self.eval_res = None
        if name is not None:
            self.name = name
        else:
            self.name = _geom_str(geom)
        self.shape_dims = _shape_dim(self.geom)
    def __str__(self):
        return "geometry.GeomPointClosest[%s]"%(self.name,)
    def n_args(self):
        return 2
    def n_in(self,arg):
        if arg == 0: return se3_ad.SIZE
        else: return 3
    def n_out(self):
        return 3
    def argname(self,arg):
        return ['T','x'][arg]
    def eval(self,T,x):
        if not np.array_equal(self.eval_args[0],T):
            self.geom.setCurrentTransform(*se3_ad.to_klampt(T))
        self.eval_args = _array_list_copy((T,x))
        self.eval_res = self.geom.distance_point(x.tolist())
        if not self.eval_res.hasClosestPoints:
            raise RuntimeError("Klampt geom type %s doesn't support closest point query?"%(self.geom.type(),))
        return self.eval_res.cp1
    def jvp(self,arg,darg,T,x):
        if not _array_list_equal(self.eval_args,(T,x)):
            self.eval(T,x)
        if self.shape_dims == 0:
            #approximate geometry as a point @ p
            if arg == 0:
                Tnative = se3_ad.to_klampt(T)
                Tinv = se3.inv(Tnative)
                dR,dt = se3_ad.to_klampt(darg)
                pgeom_loc = se3.apply(Tinv,self.eval_res.cp1)
                return np.array(se3.apply((dR,dt),pgeom_loc))
            else:
                return darg*0.0
        elif self.shape_dims == 1:
            pass
        elif self.shape_dims == 2:
            if self.eval_res.hasClosestPoints and self.eval_res.hasGradients:
                #approximate geometry at closest point as a plane n^T (p0 - x) = d 
                #p ~= x + d*n = x + n n^T (p0 - x) 
                if arg == 0:
                    Tnative = se3_ad.to_klampt(T)
                    Tinv = se3.inv(Tnative)
                    dR,dt = se3_ad.to_klampt(darg)
                    #since n = R*n_l and p0 = R*p_l + t, we have
                    #dp/ds = dn/ds n^T (p0 - x)  + n dn/ds^T (p0 - x)  + n n^T dp0/ds
                    #dn/ds = dR/ds*n_l, dp0/ds = dR/ds*p_l + dt/ds
                    p0 = self.eval_res.cp1
                    n = self.eval_res.grad1
                    disp = vectorops.sub(x,self.eval_res.cp1) 
                    pgeom_loc = se3.apply(Tinv,self.eval_res.cp1)
                    ngeom_loc = se3.apply_rotation(Tinv,self.eval_res.grad1)
                    dn_ds = so3.apply(dR,ngeom_loc)
                    dp_ds = vectorops.add(so3.apply(dR,pgeom_loc),dt)
                    return np.asarray(dn_ds)*self.eval_res.d + np.asarray(n)*(np.dot(dn_ds,disp) + np.dot(n,dp_ds))
                else:
                    #dp/ds = dx/ds - n n^T dx/ds
                    n = self.eval_res.cp1
                    return darg - np.dot(n,darg)*np.asarray(n)
        raise NotImplementedError()


class GeomGeomDistance(ADFunctionInterface):
    """Autodiff wrapper of Geometry3D.distance.  This is a function
    D(Tgeom1,Tgeom2) where Tgeom1 and Tgeom2 are the transforms of each
    geometry.

    .. note::
        This caches the result of eval().  If you change the geometry's
        transform between eval(T1,T2) and eval(T1,T2_prime), then incorrect
        results will be returned.
    """
    def __init__(self,geom1,geom2,name1=None,name2=None,relErr=None,absErr=None,upperBound=None):
        if not isinstance(geom1,Geometry3D) or not isinstance(geom2,Geometry3D):
            raise ValueError("GeomGeomDistance expects its args to be Geometry3D")
        self.geom1 = geom1
        self.geom2 = geom2
        self.eval_args = (None,None)
        self.eval_res = None
        if name1 is not None:
            self.name1 = name1
        else:
            self.name1 = _geom_str(geom1)
        if name2 is not None:
            self.name2 = name2
        else:
            self.name2 = _geom_str(geom2)
        self.settings = None
        if relErr is not None or absErr is not None or upperBound is not None:
            self.settings = DistanceQuerySettings()
            if relErr is not None:
                self.settings.relErr = relErr
            if absErr is not None:
                self.settings.absErr = absErr
            if upperBound is not None:
                self.settings.upperBound = upperBound
        self.shape_dims1 = _shape_dim(geom1)
        self.shape_dims2 = _shape_dim(geom2)
        self.eval_args = (None,None)
        self.eval_res = None
    def __str__(self):
        return "geometry.GeomGeomDistance[%s,%s]"%(self.name1,self.name2)
    def n_args(self):
        return 2
    def n_in(self,arg):
        return se3_ad.SIZE
    def n_out(self):
        return 1
    def argname(self,arg):
        return ['T1','T2'][arg]
    def eval(self,T1,T2):
        if not np.array_equal(self.eval_args[0],T1):
            self.geom1.setCurrentTransform(*se3_ad.to_klampt(T1))
        if not np.array_equal(self.eval_args[1],T2):
            self.geom2.setCurrentTransform(*se3_ad.to_klampt(T2))
        self.eval_args = _array_list_copy((T1,T2))
        if self.settings is not None:
            self.eval_res = self.geom1.distance_ext(self.geom2,self.settings)
        else:
            self.eval_res = self.geom1.distance(self.geom2)
        #print("Eval result",self.eval_res.d,"for transforms",T1,T2)
        return self.eval_res.d
    def jvp(self,arg,darg,T1,T2):
        if not _array_list_equal(self.eval_args,(T1,T2)):
            self.eval(T1,T2)
        if self.settings is not None and self.eval_res.d == self.settings.upperBound:
            return 0
        if not self.eval_res.hasClosestPoints:
            raise NotImplementedError()
        if self.shape_dims1 == 1 or self.shape_dims2 == 1:
            raise NotImplementedError()
        T1inv = se3.inv(se3_ad.to_klampt(T1))
        T2inv = se3.inv(se3_ad.to_klampt(T2))
        if self.shape_dims1 == 0 and self.shape_dims2 == 0:
            #approximate local geometry as points
            if arg == 0:
                p1loc = se3.apply(T1inv,self.eval_res.cp1)
                dp1 = se3.apply(se3_ad.to_klampt(darg),p1loc)
                return math_ad._distance_jvp_a(dp1,self.eval_res.cp1,self.eval_res.cp2)
            else:
                p2loc = se3.apply(T2inv,self.eval_res.cp2)
                dp2 = se3.apply(se3_ad.to_klampt(darg),p2loc)
                return math_ad._distance_jvp_a(dp2,self.eval_res.cp2,self.eval_res.cp1)
        elif self.shape_dims1 == 2 and self.shape_dims2 == 2:
            #hmm... how to approximate local geometry?
            if arg == 0:
                p1loc = se3.apply(T1inv,self.eval_res.cp1)
                dp1 = se3.apply(se3_ad.to_klampt(darg),p1loc)
                return math_ad._distance_jvp_a(dp1,self.eval_res.cp1,self.eval_res.cp2)
            else:
                p2loc = se3.apply(T2inv,self.eval_res.cp2)
                dp2 = se3.apply(se3_ad.to_klampt(darg),p2loc)
                return math_ad._distance_jvp_a(dp2,self.eval_res.cp2,self.eval_res.cp1)
        elif self.shape_dims2 == 0:
            #approximate shape 2 as a point, shape 1 as a plane
            if not self.eval_res.hasGradients:
                raise NotImplementedError()
            #approximate geometry at closest point as a plane n1^T (p1 - p2) = d or n2^T (p1 - p2) = d
            if arg == 0:
                dR,dt = se3_ad.to_klampt(darg)
                
                #dd/ds = (p - x)^T dn/ds + n^T dp/ds = (p-x)^T dR/ds n_l + n^T (dR/ds*p_l + dt/ds)
                disp = vectorops.sub(self.eval_res.cp1,self.eval_res.cp2) 
                pgeom_loc = se3.apply(T1inv,self.eval_res.cp1)
                ngeom_loc = se3.apply_rotation(T1inv,self.eval_res.grad1)
                return vectorops.dot(disp,so3.apply(dR,ngeom_loc)) + vectorops.dot(self.eval_res.grad1,vectorops.add(so3.apply(dR,pgeom_loc),dt))
            else:
                return -np.dot(self.eval_res.grad1,darg)
        else:
            #approximate shape 1 as a point, shape 2 as a plane
            if not self.eval_res.hasGradients:
                raise NotImplementedError()
            #approximate geometry at closest point as a plane n1^T (p1 - p2) = d or n2^T (p1 - p2) = d
            if arg == 0:
                dR,dt = se3_ad.to_klampt(darg)
                
                #dd/ds = (p - x)^T dn/ds + n^T dp/ds = (p-x)^T dR/ds n_l + n^T (dR/ds*p_l + dt/ds)
                disp = vectorops.sub(self.eval_res.cp1,self.eval_res.cp2) 
                pgeom_loc = se3.apply(T2inv,self.eval_res.cp2)
                ngeom_loc = se3.apply_rotation(T2inv,self.eval_res.grad1)
                return vectorops.dot(disp,so3.apply(dR,ngeom_loc)) + vectorops.dot(self.eval_res.grad1,vectorops.add(so3.apply(dR,pgeom_loc),dt))
            else:
                return np.dot(self.eval_res.grad1,darg)
        raise NotImplementedError() 


class GeomRayCast(ADFunctionInterface):
    """Autodiff wrapper of Geometry3D.rayCast.  This is a function
    RC(Tgeom,s,d) where Tgeom is the transform of the geometry, s is the ray
    source, and d is the ray direction.

    .. note::
        This caches the result of eval().  If you change the geometry's
        transform between eval(T,s,d) and eval(T,sprime,dprime), then incorrect
        results will be returned.
    """
    def __init__(self,geom,name=None):
        if not isinstance(geom,Geometry3D):
            raise ValueError("GeomRayCast expects a Geometry3D")
        self.geom = geom
        self.eval_args = (None,None,None)
        self.eval_res = None
        if name is not None:
            self.name = name
        else:
            self.name = _geom_str(geom)
        self.supportsClosestPoint = True
    def __str__(self):
        return "geometry.GeomRayCast[%s]"%(self.name,)
    def n_args(self):
        return 3
    def n_in(self,arg):
        if arg == 0: return se3_ad.SIZE
        else: return 3
    def n_out(self):
        return 1
    def argname(self,arg):
        return ['T','s','d'][arg]
    def eval(self,T,s,d):
        if not np.array_equal(self.eval_args[0],T):
            self.geom.setCurrentTransform(*se3_ad.to_klampt(T))
        self.eval_args = _array_list_copy((T,s,d))
        self.eval_res = self.geom.rayCast(s.tolist(),d.tolist())
        if not self.eval_res[0]:
            return float('inf')
        else:
            return np.dot(d,np.array(self.eval_res[1])-s)
    def jvp(self,arg,darg,T,s,d):
        if not _array_list_equal(self.eval_args,[T,s,d]):
            self.eval(T,s,d)
        if not self.eval_res[0]:
            #not hit
            return 0.0*darg
        if not self.supportsClosestPoint:
            return NotImplementedError()
        cp = self.eval_res[1]
        #t = d^T(cp - s)
        cp_res = self.geom.distance_point(cp.tolist())
        if cp_res.hasGradients and cp_res.hasClosestPoints:
            #approximate geometry as t = -n^T x, or 0 = n^T(x-c) 
            Tnative = se3_ad.to_klampt(T)
            Tinv = se3.inv(Tnative)
            if arg == 0:
                dR,dt = se3_ad.to_klampt(darg)
                #given 0=n^T(s + t*d - c) and n=R*n_l and c = R*c_l
                #t = n^T(s-c)/n^T d
                #dt/du = (dn/du^T (s-c) - n^T dc/du)/n^T d - n^T(s-c)/(n^T d)^2 dn/du^T d 
                n = self.eval_res.cp1
                disp = vectorops.sub(s,cp) 
                pgeom_loc = se3.apply(Tinv,n)
                ngeom_loc = se3.apply_rotation(Tinv,n)
                dn_du = so3.apply(dR,ngeom_loc)
                dp_du = vectorops.add(so3.apply(dR,pgeom_loc),dt)
                dn = np.dot(d,n)
                return (np.dot(dn_du,disp) - np.dot(n,dp_du))/dn - np.dot(n,disp)/dn**2 * np.dot(dn_du,d)
            elif arg == 1:
                #n^T (s + t*d - c) = 0 => t = n^T(s-c)/n^T d
                #dt/du = n^T ds/du/ n^T d
                numer = np.dot(self.eval_res.grad1,darg)
                denom = np.dot(self.eval_res.grad1,d)
                return numer / denom
            else:
                #n^T (s + t*d - c) = 0 => t = n^T(s-c)/n^T d
                #dt/du = -n^T (s-c)/ (n^T d)^2 * n^T dd/du
                numer = np.dot(self.eval_res.grad1,s-np.array(cp))
                denom = np.dot(self.eval_res.grad1,d)
                return -numer/denom**2 * np.dot(self.eval_res.grad1,darg)
        self.supportsClosestPoint = False
        return NotImplementedError()


class MinDistance(ADFunctionInterface):
    """Autodiff function that is equivalent to minimum(D1(...),...,DM(...)) but
    is faster due to the use of upper bounding and order caching. Upper
    bounding does early-stopping for distance calculation if it is found that

    Di are functions of the form XYDistance.  They are defined by a pair of
    geometries and are auto-determined by the geometry types.

    Each argument to this function is a configuration of the corresponding
    geometry in the ``geometries`` list.

    .. note::
        This caches the result of eval().  If you change a geometry's
        transform between eval(...) and eval(...), then incorrect
        results will be returned.

    Args:
        geometries (list): a list of Geometry3D's, points given as 3-vectors, 
            or spheres given as (3-vector,scalar) pairs.
        pairs (list, optional): if 'all', all pairs of geometries are tested.
            otherwise, this is a list of pairs of ints giving the indices of
            geometries to be tested against one another.
        names (list of str, optional): if given, the geometries will be named
            for more informative print statements.
        relErr (float, optional): passed to the DistanceQuerySettings objects
            used in distance queries.
        absErr (float, optional): passed to the DistanceQuerySettings objects
            used in distance queries. 
        upperBound (float, optional): passed to the DistanceQuerySettings 
            objects used in distance queries.  If the minimum distance is
            greater than this value, this value will be returned.
    """
    def __init__(self,geometries,pairs='all',names=None,
                 relErr=None,absErr=None,upperBound=None):
        self.geometries = []
        for g in geometries:
            if isinstance(g,Geometry3D):
                self.geometries.append(g)
            elif hasattr(g,'__iter__'):
                if len(g)==3:
                    self.geometries.append('point')
                elif len(g)==2:
                    if len(g[0])==3 and not hasattr(g[1],'__iter__'):
                        self.geometries.append('sphere')
                    else:
                        raise ValueError("Geometry list item %s is not a Geometry3D, point, or sphere"%(str(g),))
                else:
                    raise ValueError("Geometry list item %s is not a Geometry3D, point, or sphere"%(str(g),))
            else:
                raise ValueError("Geometry list item %s is not a Geometry3D, point, or sphere"%(g.__class__.__name__,))
        if names is not None:
            if len(names) != len(geometries):
                raise ValueError("Invalid number of names provided")
            self.names = names
        else:
            self.names = [None]*len(self.geometries)
        if pairs == 'all':
            pairs = []
            for i in range(len(self.geometries)):
                for j in range(i+1,len(self.geometries)):
                    pairs.append((i,j))
        self.queries = []
        self.queryFlipped = []
        for p in pairs:
            i,j = p
            if i < 0 or i >= len(self.geometries):
                raise ValueError("Invalid geometry index %d"%(i,))
            if j < 0 or j >= len(self.geometries):
                raise ValueError("Invalid geometry index %d"%(j,))
            if i == j:
                raise ValueError("Pairs need to be distinct indices, (%d,%d) given"%(i,j))
            g1,g2 = self.geometries[i],self.geometries[j]
            if isinstance(g1,Geometry3D):
                if isinstance(g2,Geometry3D):
                    self.queries.append(GeomGeomDistance(g1,g2,self.names[i],self.names[j],relErr,absErr,float('inf')))
                    self.queryFlipped.append(False)
                elif g2 == 'point':
                    self.queries.append(GeomPointDistance(g1,self.names[i],relErr,absErr,float('inf')))
                    self.queryFlipped.append(False)
                else:
                    #sphere
                    self.queries.append(GeomSphereDistance(g1,self.names[i],relErr,absErr,float('inf')))
                    self.queryFlipped.append(False)
            elif g1 == 'point':
                if isinstance(g2,Geometry3D):
                    self.queries.append(GeomPointDistance(g2,self.names[j],relErr,absErr,float('inf')))
                    self.queryFlipped.append(True)
                elif g2 == 'point':
                    self.queries.append(math_ad.distance)
                    self.queryFlipped.append(False)
                else:
                    #sphere
                    self.queries.append(sphere_point_distance)
                    self.queryFlipped.append(True)
            else:
                #sphere
                if isinstance(g2,Geometry3D):
                    self.queries.append(GeomSphereDistance(g2,self.names[j],relErr,absErr,float('inf')))
                    self.queryFlipped.append(True)
                elif g2 == 'point':
                    self.queries.append(sphere_point_distance)
                    self.queryFlipped.append(False)
                else:
                    #sphere
                    self.queries.append(sphere_sphere_distance)
                    self.queryFlipped.append(False)

        self.pairs = pairs
        self.testOrder = [(0,i) for i in range(len(pairs))]
        self.upperBound = upperBound
        self.eval_args = None
        self.closest = None

    def __str__(self):
        if self.names[0] is None:
            label = '%d geometries, %d queries'%(len(self.geometries),len(self.pairs))
        else:
            label = ','.join(self.names)
        return "geometry.MinDistance[%s]"%(label,)
    def n_args(self):
        return len(self.geometries)
    def n_in(self,arg):
        if isinstance(self.geometries[arg],'Geometry3D'):
            return se3_ad.SIZE
        elif self.geometries[arg] == 'point':
            return 3
        else:
            #sphere
            return 4
    def n_out(self):
        return 1
    def argname(self,arg):
        if isinstance(self.geometries[arg],'Geometry3D'):
            base='T'
        elif self.geometries[arg] == 'point':
            base='x'
        else:
            base='cr'
        if self.names[arg] is not None:
            return base+'_'+self.names[arg]
        else:
            return base+'_'+str(arg)
    def eval(self,*args):
        assert len(args) == len(self.geometries)
        self.eval_args = _array_list_copy(args)
        self.closest = None
        dmin = self.upperBound
        for orderind,(dlast,ind) in enumerate(self.order):
            i,j = self.pairs[ind]
            g1,g2 = self.geometries[i],self.geometries[j]
            arg1 = [args[i]]
            arg2 = [args[j]]
            if g1 == 'sphere':
                arg1 = [args[i][:3],args[i][3]]
            if g2 == 'sphere':
                arg2 = [args[j][:3],args[j][3]]
            try:
                self.queries[ind].settings.upperBound = dmin
            except AttributeError:
                pass
            if self.queryFlipped[ind]:
                dij = self.queries[ind].eval(*(arg2+arg1))
            else:
                dij = self.queries[ind].eval(*(arg1+arg2))
            if dij != dmin: #upperbound not hit
                self.order[orderind] = (dij,ind)
            if dij < dmin:
                self.closest = ind
                dmin = dij
        #now re-determine order
        self.order = sorted(self.order)
        return dmin
    def jvp(self,arg,darg,*args):
        assert len(args) == len(self.geometries)
        if self.eval_args is None or not _array_list_equal(self.eval_args,args):
            self.eval(*args)

        if self.closest is None:
            #upper bound is hit
            return 0
        ind = self.closest
        i,j = self.pairs[ind]
        if arg != i and arg != j:
            #irrelevant argument
            return 0
        arg1 = [args[i]]
        arg2 = [args[j]]
        g1,g2 = self.geometries[i],self.geometries[j]
        if g1 == 'sphere':
            arg1 = [args[i][:3],args[i][3]]
        if g2 == 'sphere':
            arg2 = [args[j][:3],args[j][3]]
        if self.queryFlipped[ind]:
            if i == arg:
                return self.queries[ind].jvp(1,darg,*(arg2+arg1))
            else:
                return self.queries[ind].jvp(0,darg,*(arg2+arg1))
        else:
            if i == arg:
                return self.queries[ind].jvp(0,darg,*(arg1+arg2))
            else:
                return self.queries[ind].jvp(1,darg,*(arg1+arg2))
