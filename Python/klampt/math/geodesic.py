"""Common interface for geodesic interpolation and distances on manifolds.
"""

from . import vectorops, so2, so3, se3
import math

class GeodesicSpace:
    """Base class for geodesic spaces.  A geodesic is equipped with a
    a geodesic (interpolation via the interpolate(a,b,u) method), a natural
    arc length distance metric (distance(a,b) method), an intrinsic dimension
    (intrinsicDimension() method), an extrinsic dimension (extrinsicDimension()
    method), and natural tangents (the difference and  integrate  methods)."""
    def intrinsicDimension(self):
        raise NotImplementedError()
    def extrinsicDimension(self):
        raise NotImplementedError()
    def distance(self,a,b):
        return vectorops.distance(a,b)
    def interpolate(self,a,b,u):
        return vectorops.interpolate(a,b,u)
    def difference(self,a,b):
        """For Lie groups, returns a difference vector that, when integrated
        would get to a from b.  In Cartesian spaces it is a-b.  In other spaces,
        it should be related to d/du interpolate(b,a,u) at u=0."""
        return vectorops.sub(a,b)
    def integrate(self,x,d):
        """For Lie groups, returns the point that would be arrived at via
        integrating the difference vector d starting from x.  Must satisfy
        the relationship a = integrate(b,difference(a,b)). In Cartesian
        spaces it is x+d"""
        return vectorops.add(x,d)


class CartesianSpace(GeodesicSpace):
    """The standard geodesic on R^d"""
    def __init__(self,d):
        self.d = d
    def intrinsicDimension(self):
        return self.d
    def extrinsicDimension(self):
        return self.d


class MultiGeodesicSpace(GeodesicSpace):
    """This forms the cartesian product of one or more GeodesicSpace's.
    Distances are simply added together."""
    def __init__(self,*components):
        self.components = components
        self.componentWeights = [1]*len(self.components)
    def intrinsicDimension(self):
        return sum(c.intrinsicDimension() for c in self.components)
    def extrinsicDimension(self):
        return sum(c.extrinsicDimension() for c in self.components)
    def split(self,x):
        i = 0
        res = []
        for c in self.components:
            d = c.extrinsicDimension()
            res.append(x[i:i+d])
            i += d
        return res
    def join(self,xs):
        return sum(xs,[])
    def distance(self,a,b):
        i = 0
        res = 0.0
        for c,w in zip(self.components,self.componentWeights):
            d = c.extrinsicDimension()
            res += (c.distance(a[i:i+d],b[i:i+d])**2)*w
            i += d
        return math.sqrt(res)
    def interpolate(self,a,b,u):
        i = 0
        res = [0]*len(a)
        for c in self.components:
            d = c.extrinsicDimension()
            res[i:i+d] = c.interpolate(a[i:i+d],b[i:i+d],u)
            i += d
        return res

    def difference(self,a,b):
        i = 0
        res = [0]*len(a)
        for c in self.components:
            d = c.extrinsicDimension()
            res[i:i+d] = c.difference(a[i:i+d],b[i:i+d])
            i += d
        return res
    def integrate(self,x,diff):
        i = 0
        res = [0]*len(x)
        for c in self.components:
            d = c.extrinsicDimension()
            res[i:i+d] = c.integrate(x[i:i+d],diff[i:i+d])
            i += d
        return res


class SO2Space(GeodesicSpace):
    """The space of 2D rotations SO(2)."""
    def intrinsicDimension(self):
        return 1
    def extrinsicDimension(self):
        return 1
    def distance(self,a,b):
        return abs(so2.diff(a[0],b[0]))
    def interpolate(self,a,b,u):
        return [so2.interp(a[0],b[0],u)]
    def difference(self,a,b):
        return [so2.diff(a[0],b[0])]
    def integrate(self,x,d):
        return [so2.normalize(x[0]+d[0])]


class SO3Space(GeodesicSpace):
    """The space of 3D rotations SO(3).  The representation is 9 entries of the
    rotation matrix, laid out in column-major form, like the math.so3 module.

    The difference is the cross product matrix of the angular velocity vector.
    Note that this is the extrinsic angular velocity, i.e., angular velocity
    expressed in the world frame of the reference rotation.  The integrate
    method integrates the angular velocity in the local frame to produce a new
    rotation matrix.
    """
    def intrinsicDimension(self):
        return 3
    def extrinsicDimension(self):
        return 9
    def distance(self,a,b):
        return vectorops.norm(so3.error(a,b))
    def interpolate(self,a,b,u):
        return so3.interpolate(a,b,u)
    def difference(self,a,b):
        w = so3.error(a,b)
        return so3.cross_product(so3.apply(b,w))
        #for matrix derivative representation
        #return so3.mul(so3.cross_product(w),b)
    def integrate(self,x,d):
        wcross = d
        w = so3.apply(so3.inv(x),so3.deskew(wcross))
        return so3.mul(so3.from_moment(w),x)
        #for matrix derivative representation
        #wcross = so3.mul(d,so3.inv(x))


class SE3Space(GeodesicSpace):
    """The space of 3D rigid transforms SE(3).  The representation is 9 entries
    of SO(3) + 3 entries of translation.

    The difference is a 12 element vector, the first 9 being the intrinsic
    angular velocity (see :class:`SO3Space`), and the last 3 being the linear
    velocity.
    """
    def intrinsicDimension(self):
        return 6
    def extrinsicDimension(self):
        return 12
    def to_se3(self,x):
        return (x[:9],x[9:])
    def from_se3(self,T):
        return list(T[0])+list(T[1])
    def distance(self,a,b):
        return vectorops.norm(se3.error(self.to_se3(a),self.to_se3(b)))
    def interpolate(self,a,b,u):
        r = se3.interpolate(self.to_se3(a),self.to_se3(b),u)
        return r[0]+r[1]
    def difference(self,a,b):
        Rb,tb = self.to_se3(b)
        w = se3.error(self.to_se3(a),self.to_se3(b))
        return so3.cross_product(so3.apply(Rb,w[:3]))+w[3:]
        #for matrix derivative representation
        #return so3.mul(Rb,so3.cross_product(w[:3]))+w[3:]
    def integrate(self,x,d):
        assert len(x) == 12
        #for matrix derivative representation
        Rx,tx = self.to_se3(x)
        #w = so3.deskew(so3.mul(so3.inv(Rx),d[:9]))
        w = so3.apply(so3.inv(Rx),so3.deskew(d[:9]))
        v = d[9:]
        wR = so3.from_moment(w)
        assert len(wR) == 9
        R = so3.mul(Rx,wR)
        t = vectorops.add(tx,v)
        return R + t
