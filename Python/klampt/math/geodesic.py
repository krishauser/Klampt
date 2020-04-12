from . import vectorops, so2, so3, se3
import math

class GeodesicSpace:
    """A class representing a geodesic space.  A geodesic is equipped with a
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
        would get to a from b.  In Cartesian spaces it is a-b."""
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

class MultiGeodesicSpace:
    """This forms the cartesian product of one or more GeodesicSpace's.
    Distances are simply added together."""
    def __init__(self,*components):
        self.components = components
        self.componentWeights = [1]*len(self.components)
    def intrinsicDimension(self):
        return sum(c.intrinsicDimension() for c in components)
    def extrinsicDimension(self):
        return sum(c.extrinsicDimension() for c in components)
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
            d = c.dimension()
            res[i:i+d] = c.difference(x[i:i+d],diff[i:i+d])
            i += d
        return res

class SO2Space(GeodesicSpace):
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
        return so3.cross_product(w)
    def integrate(self,x,d):
        w = [d[0],d[4],d[8]]
        return so3.mul(x,so3.from_moment(w))


class SE3Space(GeodesicSpace):
    def intrinsicDimension(self):
        return 6
    def extrinsicDimension(self):
        return 12
    def distance(self,a,b):
        return vectorops.norm(se3.error((a[:9],a[9:]),(b[:9],b[9:])))
    def interpolate(self,a,b,u):
        r = se3.interpolate((a[:9],a[9:]),(b[:9],b[9:]),u)
        return r[0]+r[1]
    def difference(self,a,b):
        w = se3.error(a,b)
        return so3.cross_product(w[:3])+w[3:]
    def integrate(self,x,d):
        w = [d[0],d[4],d[8]]
        v = d[9:]
        return se3.mul(x,(so3.from_moment(w),v))
