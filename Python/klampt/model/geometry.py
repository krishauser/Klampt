"""Utility functions for operating on  geometry.  See the :class:`Geometry3D`
documentation for the core geometry class. 

.. versionadded:: 0.8.6

[functions moved here from :mod:`klampt.model.sensing`]

Working with geometric primitives
=================================

:func:`box` and :func:`sphere` are aliases for the functions in
:mod:`klampt.model.create.primitives`.

Working with point clouds
=========================

:func:`point_cloud_normals` estimates normals from a normal-free
:class:`PointCloud`.

The :func:`fit_plane`, :func:`fit_plane3`, and :class:`PlaneFitter` class help
with plane estimation.

:func:`point_cloud_simplify` simplifies a PointCloud.

:func:`point_cloud_colors` and :func:`point_cloud_set_colors` sets / gets 
colors from a PointCloud.

"""

from ..robotsim import Geometry3D,PointCloud,TriangleMesh
import math
from .create import primitives
from ..math import vectorops,so3,se3

_has_numpy = False
_tried_numpy_import = False
np = None

_has_scipy = False
_tried_scipy_import = False
sp = None

box = primitives.box
"""Alias for :func:`klampt.model.create.primitives.box`"""

sphere = primitives.sphere
"""Alias for :func:`klampt.model.create.primitives.sphere`"""

def _try_numpy_import():
    global _has_numpy,_tried_numpy_import
    global np
    if _tried_numpy_import:
        return _has_numpy
    _tried_numpy_import = True
    try:
        import numpy as np
        _has_numpy = True
        #sys.modules['numpy'] = numpy
    except ImportError:
        import warnings
        warnings.warn("klampt.model.geometry.py: numpy not available.",ImportWarning)
        _has_numpy = False
    return _has_numpy

def _try_scipy_import():
    global _has_scipy,_tried_scipy_import
    global sp
    if _tried_scipy_import:
        return _has_scipy
    _tried_scipy_import = True
    try:
        import scipy as sp
        _has_scipy = True
        #sys.modules['scipy'] = scipy
    except ImportError:
        import warnings
        warnings.warn("klampt.model.geometry.py: scipy not available.",ImportWarning)
        _has_scipy = False
    return _has_scipy

class PlaneFitter:
    """
    Online fitting of planes through 3D point clouds
   
    Attributes:
        normal (3-vector): best-fit normal
        centroid (3-vector): centroid of points
        count (int): # of points
        sse (float): fitting sum of squared errors
        cov (3x3 array): covariance of points
    """
    def __init__(self,points=None):
        _try_numpy_import()
        if points is None:
            self.count = 0
            self.centroid = np.zeros(3)
            self.cov = np.zeros((3,3))
            self.normal = np.array([0,0,1])
            self.sse = 0
        else:
            self.count = len(points)
            self.centroid = np.average(points,axis=0)
            pprime = points - [self.centroid]*len(points)
            self.cov = np.dot(pprime.T,pprime)/self.count
            self._update_plane()
   
    def plane_equation(self):
        """Returns (a,b,c,d) with ax+by+cz+d=0 the plane equation"""
        offset = np.dot(self.centroid,self.normal)
        return (self.normal[0],self.normal[1],self.normal[2],-offset)
   
    def goodness_of_fit(self):
        """Returns corrected RMSE"""
        if self.count <= 3:
            return float('inf')
        return math.sqrt(self.sse*self.count / (self.count-3))
   
    def add_point(self,pt):
        """Online estimation of best fit plane"""
        new_count = self.count + 1
        new_centroid = self.centroid + (pt-self.centroid)/new_count
        old_sse = (self.cov + np.outer(self.centroid,self.centroid))*self.count
        new_sse = old_sse + np.outer(pt,pt)
        new_cov = new_sse/new_count - np.outer(new_centroid,new_centroid)
        self.count = new_count
        self.centroid = new_centroid
        self.cov = new_cov
        self._update_plane()
   
    def merge(self,fitter,inplace = False):
        """Online merging of two plane fitters.
        
        If inplace = False, returns a new PlaneFitter.
        If inplace = True, self is updated with the result.
        """
        if not inplace:
            res = PlaneFitter()
        else:
            res = self
        new_count = self.count + fitter.count
        old_sum = self.centroid*self.count
        new_sum = old_sum + fitter.centroid*fitter.count
        new_centroid = new_sum/new_count
        old_sse = (self.cov + np.outer(self.centroid,self.centroid))*self.count
        fitter_sse = (fitter.cov + np.outer(fitter.centroid,fitter.centroid))*fitter.count
        new_sse = old_sse + fitter_sse
        new_cov = new_sse/new_count - np.outer(new_centroid,new_centroid)
        res.count = new_count
        res.centroid = new_centroid
        res.cov = new_cov
        res._update_plane()
        return res
   
    def distance(self,pt):
        """Returns the signed distance to this plane"""
        return np.dot(self.normal,pt)-np.dot(self.normal,self.centroid)
 
    def _update_plane(self):
        w,v = np.linalg.eig(self.cov)
        index = np.argmin(w)
        self.normal = v[:,index]
        self.sse = self.count * np.dot(self.normal,np.dot(self.cov,self.normal))


def point_cloud_simplify(pc,radius):
    """Simplifies a point cloud by averaging points within neighborhoods. Uses 
    a fast hash grid data structure.

    Args:
        pc (Geometry3D or PointCloud): the point cloud
        radius (float): the neighborhood radius.
    """
    if radius <= 0:
        raise ValueError("radius must be > 0")
    if isinstance(pc,Geometry3D):
        assert pc.type() == 'PointCloud',"Must provide a point cloud to point_cloud_simplify"
        return pc.convert('PointCloud',radius)
    else:
        return Geometry3D(pc).convert('PointCloud',radius).getPointCloud()
 

def point_cloud_normals(pc,estimation_radius=None,estimation_knn=None,estimation_viewpoint=None,add=True):
    """Returns the normals of the point cloud.  If pc has the standard
    ``normal_x, normal_y, normal_z`` properties, these will be returned. 
    Otherwise, they will be estimated using plane fitting.

    The plane fitting method uses scipy nearest neighbor detection if
    scipy is available. Otherwise it uses a spatial grid.  The process is as
    follows:

    - If ``estimation_radius`` is provided, then it will use neighbors within
      this range.  For a spatial grid, this is the grid size.
    - If ``estimation_knn`` is provided, then planes will be fit to these 
      number of neighbors. 
    - If neither is provided, then estimation_radius is set to 3 * max
      dimension of the point cloud / sqrt(N). 
    - If not enough points are within a neighborhood (either 4 or
      ``estimation_knn``, whichever is larger), then the normal is set to 0.
    - If ``estimation_viewpoint`` is provided, this must be a 3-list.  The
      normals are oriented such that they point toward the viewpoint.

    Returns:
        A list of N 3-lists, or an N x 3 numpy array if numpy is available.

        If ``add=True``, estimated normals will be added to the point cloud 
        under the ``normal_x, normal_y, normal_z`` properties.
    """
    geom = None
    if isinstance(pc,Geometry3D):
        assert pc.type() == 'PointCloud',"Must provide a point cloud to point_cloud_normals"
        geom = pc
        pc = pc.getPointCloud()
    assert isinstance(pc,PointCloud)
    inds = [-1,-1,-1]
    props = ['normal_x','normal_y','normal_z']
    for i in range(pc.numProperties()):
        try:
            ind = props.index(pc.propertyNames[i])
            inds[ind] = i
        except ValueError:
            pass
    if all(i>=0 for i in inds):
        #has the properties!
        normal_x = pc.getProperties(inds[0])
        normal_y = pc.getProperties(inds[1])
        normal_z = pc.getProperties(inds[2])
        if _has_numpy:
            return np.array([normal_x,normal_y,normal_z]).T
        else:
            return list(zip(normal_x,normal_y,normal_z))

    if not all(i < 0 for i in inds):
        raise ValueError("Point cloud has some normal components but not all of them?")
    #need to estimate normals
    _try_numpy_import()
    _try_scipy_import()
    N = len(pc.vertices)//3
    if not _has_numpy:
        raise RuntimeError("Need numpy to perform plane fitting")
    positions = np.array(pc.vertices)
    positions = positions.reshape((N,3))
    if estimation_radius is None and estimation_knn is None:
        R = max(positions.max(axis=0)-positions.min(axis=0))
        estimation_radius = 3*R/math.sqrt(N)
    if estimation_knn is None or estimation_knn < 4:
        estimation_knn = 4
    normals = []
    if _has_scipy:
        import scipy.spatial
        tree = scipy.spatial.cKDTree(positions)
        if estimation_radius is not None:
            neighbors = tree.query_ball_point(positions,estimation_radius)
            for n in neighbors:
                if len(n) < estimation_knn:
                    normal = [0,0,0]
                else:
                    try:
                        #fit a plane to neighbors
                        normal = fit_plane([positions[i] for i in n])[:3]
                    except ValueError:
                        normal = [0,0,0]
                normals.append(normal)
        else:
            d,neighbors = tree.query(positions,estimation_knn)
            for n in neighbors:
                try:
                    normal = fit_plane([positions[i] for i in n])[:3]
                except ValueError:
                    normal = [0,0,0]
                normals.append(normal)
    else:
        if estimation_radius is None:
            raise ValueError("Without scipy, can't do a k-NN plane estimation")
        #do a spatial hash
        normals = np.zeros((N,3))
        indices = (positions * (1.0/estimation_radius)).astype(int)
        from collections import defaultdict
        pt_hash = defaultdict(list)
        for i,(ind,p) in enumerate(zip(indices,positions)):
            pt_hash[ind].append((i,p))
        successful = 0
        for (ind,iplist) in pt_hash.items():
            if len(iplist) < estimation_knn:
                pass
            else:
                pindices = [ip[0] for ip in iplist]
                pts = [ip[1] for ip in iplist]
                try:
                    n = fit_plane(pts)[:3]
                except ValueError:
                    n = [0,0,0]
                normals[pindices,:] = n
                successful += len(pindices)
    normals = np.asarray(normals)

    if estimation_viewpoint is not None:
        #flip back-facing normals
        disp = positions - estimation_viewpoint
        for i,(n,d) in enumerate(zip(normals,disp)):
            if np.dot(n,d) < 0:
                normals[i,:] = -n
    else:
        #flip back-facing normals assuming centroid is interior
        centroid = np.average(positions,axis=0)
        for i,(n,p) in enumerate(zip(normals,positions)):
            if np.dot(n,p-centroid) < 0:
                normals[i,:] = -n

    if add:
        normal_x = normals[:,0].tolist()
        normal_y = normals[:,1].tolist()
        normal_z = normals[:,2].tolist()
        pc.addProperty('normal_x',normal_x)
        pc.addProperty('normal_y',normal_y)
        pc.addProperty('normal_z',normal_z)
        if geom is not None:
            geom.setPointCloud(pc)
    return normals


def fit_plane3(point1,point2,point3):
    """Returns a 3D plane equation fitting the 3 points.
  
    The result is (a,b,c,d) with the plane equation ax+by+cz+d=0
    """
    _try_numpy_import()
    normal = np.cross(point2-point1,point3-point1)
    nlen = np.linalg.norm(normal)
    if nlen < 1e-4:
        #degenerate
        raise ValueError("Points are degenerate")
    normal = normal / nlen
    offset = -np.dot(normal,point1)
    return (normal[0],normal[1],normal[2],offset)


def fit_plane(points):
    """Returns a 3D plane equation that is a least squares fit
    through the points (len(points) >= 3)."""
    centroid,normal = fit_plane_centroid(points)
    return normal[0],normal[1],normal[2],-vectorops.dot(centroid,normal)


def fit_plane_centroid(points):
    """Similar to :func:`fit_plane`, but returns a (centroid,normal) pair."""
    if len(points)<3:
        raise ValueError("Need to have at least 3 points to fit a plane")
    #if len(points)==3:
    #    return fit_plane3(points[0],points[1],points[2])
    _try_numpy_import()
    points = np.asarray(points)
    centroid = np.average(points,axis=0)
    U,W,Vt = np.linalg.svd(points-[centroid]*len(points),full_matrices=False)
    if np.sum(W<1e-6) > 1:
        raise ValueError("Point set is degenerate")
    normal = Vt[2,:]
    return centroid.tolist(),normal.tolist()


def _color_format_from_uint8_channels(format,r,g,b,a=None):
    import numpy as np
    if a is None:
        a = 0xff
    if format == 'rgb':
        return np.bitwise_or.reduce((np.left_shift(r,16),np.left_shift(g,8),b)).tolist()
    elif format == 'bgr':
        return np.bitwise_or.reduce((np.left_shift(b,16),np.left_shift(g,8),r)).tolist()
    elif format=='rgba':
        return np.bitwise_or.reduce((np.left_shift(r,24),np.left_shift(g,16),np.left_shift(b,8),a)).tolist()
    elif format=='bgra':
        return np.bitwise_or.reduce((np.left_shift(g,24),np.left_shift(g,16),np.left_shift(r,8),a)).tolist()
    elif format=='argb':
        return np.bitwise_or.reduce((np.left_shift(a,24),np.left_shift(r,16),np.left_shift(g,8),b)).tolist()
    elif format=='abgr':
        return np.bitwise_or.reduce((np.left_shift(a,24),np.left_shift(b,16),np.left_shift(g,8),r)).tolist()
    elif format=='channels':
        one_255 = 1.0/255.0
        r = np.asarray(r)
        g = np.asarray(g)
        b = np.asarray(b)
        if not hasattr(a,'__iter__'):
            return (r*one_255).tolist(),(g*one_255).tolist(),(b*one_255).tolist()
        else:
            a = np.asarray(a)
            return (r*one_255).tolist(),(g*one_255).tolist(),(b*one_255).tolist(),(a*one_255).tolist()
    elif format=='opacity':
        one_255 = 1.0/255.0
        if not hasattr(a,'__iter__'):
            return np.ones(len(r))
        a = np.asarray(a)
        return (a*one_255).tolist()
    elif tuple(format)==('r','g','b'):
        one_255 = 1.0/255.0
        r = np.asarray(r)
        g = np.asarray(g)
        b = np.asarray(b)
        return np.column_stack((r*one_255,g*one_255,b*one_255)).tolist()
    elif tuple(format)==('r','g','b','a'):
        one_255 = 1.0/255.0
        if not hasattr(a,'__iter__'):
            a = np.full(len(r),a)
        else:
            a = np.asarray(a)
        r = np.asarray(r)
        g = np.asarray(g)
        b = np.asarray(b)
        return np.column_stack((r*one_255,g*one_255,b*one_255,a*one_255)).tolist()
    else:
        raise ValueError("Invalid format specifier "+str(format))


def _color_format_to_uint8_channels(format,colors):
    import numpy as np
    if format=='channels':
        return tuple((np.asarray(c)*255).astype(np.uint8).tolist() for c in colors)
    colors = np.asarray(colors)
    if format == 'rgb':
        r,g,b = np.right_shift(np.bitwise_and(colors,0xff0000),16),np.right_shift(np.bitwise_and(colors,0xff00),8),np.bitwise_and(colors,0xff)
        return r.tolist(),g.tolist(),b.tolist()
    elif format == 'bgr':
        b,g,r = np.right_shift(np.bitwise_and(colors,0xff0000),16),np.right_shift(np.bitwise_and(colors,0xff00),8),np.bitwise_and(colors,0xff)
        return r.tolist(),g.tolist(),b.tolist()
    elif format=='rgba':
        r,g,b,a = np.right_shift(np.bitwise_and(colors,0xff000000),24),np.right_shift(np.bitwise_and(colors,0xff0000),16),np.right_shift(np.bitwise_and(colors,0xff00),8),np.bitwise_and(colors,0xff)
        return r.tolist(),g.tolist(),b.tolist(),a.tolist()
    elif format=='bgra':
        b,g,r,a = np.right_shift(np.bitwise_and(colors,0xff000000),24),np.right_shift(np.bitwise_and(colors,0xff0000),16),np.right_shift(np.bitwise_and(colors,0xff00),8),np.bitwise_and(colors,0xff)
        return r.tolist(),g.tolist(),b.tolist(),a.tolist()
    elif format=='argb':
        a,r,g,b = np.right_shift(np.bitwise_and(colors,0xff000000),24),np.right_shift(np.bitwise_and(colors,0xff0000),16),np.right_shift(np.bitwise_and(colors,0xff00),8),np.bitwise_and(colors,0xff)
        return r.tolist(),g.tolist(),b.tolist(),a.tolist()
    elif format=='abgr':
        a,b,g,r = np.right_shift(np.bitwise_and(colors,0xff000000),24),np.right_shift(np.bitwise_and(colors,0xff0000),16),np.right_shift(np.bitwise_and(colors,0xff00),8),np.bitwise_and(colors,0xff)
        return r.tolist(),g.tolist(),b.tolist(),a.tolist()
    elif format=='opacity':
        r = [0xff]*len(colors)
        return r,r,r,(colors*255).astype(np.uint8).tolist()
    elif tuple(format)==('r','g','b'):
        r = (np.asarray(colors[0])*255).astype(np.uint8)
        b = (np.asarray(colors[1])*255).astype(np.uint8)
        g = (np.asarray(colors[2])*255).astype(np.uint8)
        return r.tolist(),g.tolist(),b.tolist()
    elif tuple(format)==('r','g','b','a'):
        r = (np.asarray(colors[0])*255).astype(np.uint8)
        b = (np.asarray(colors[1])*255).astype(np.uint8)
        g = (np.asarray(colors[2])*255).astype(np.uint8)
        a = (np.asarray(colors[3])*255).astype(np.uint8)
        return r.tolist(),g.tolist(),b.tolist(),a.tolist()
    else:
        raise ValueError("Invalid format specifier "+str(format))


def point_cloud_colors(pc,format='rgb'):
    """Returns the colors of the point cloud in the given format.  If the
    point cloud has no colors, this returns None.  If the point cloud has no
    colors but has opacity, this returns white colors.

    Args:
        pc (PointCloud): the point cloud
        format: describes the output color format, either:

            - 'rgb': packed 24bit int, with the hex format 0xrrggbb,
            - 'bgr': packed 24bit int, with the hex format 0xbbggrr,
            - 'rgba': packed 32bit int, with the hex format 0xrrggbbaa,
            - 'bgra': packed 32bit int, with the hex format 0xbbggrraa,
            - 'argb': packed 32bit int, with the hex format 0xaarrggbb,
            - 'abgr': packed 32bit int, with the hex format 0xaabbggrr,
            - ('r','g','b'): triple with each channel in range [0,1]
            - ('r','g','b','a'): tuple with each channel in range [0,1]
            - 'channels': returns a list of channels, in the form (r,g,b) or 
                (r,g,b,a), where each value in the channel has range [0,1].
            - 'opacity': returns opacity only, in the range [0,1].

    Returns:
        list: A list of pc.numPoints() colors corresponding to the points
            in the point cloud.  If format='channels', the return value is
            a tuple (r,g,b) or (r,g,b,a).
    """
    rgbchannels = []
    alphachannel = None
    for i,prop in enumerate(pc.propertyNames):
        if prop in ['r','g','b','rgb']:
            rgbchannels.append((prop,i))
        elif prop == 'rgba':
            rgbchannels.append((prop,i))
            if alphachannel is not None:
                alphachannel = (prop,i)
        elif prop in ['opacity','a','c']:
            if alphachannel is not None:
                alphachannel = (prop,i)
    if len(rgbchannels)==0 and alphachannel is None:
        return
    if len(rgbchannels)==1:
        rgb = pc.getProperties(rgbchannels[0][1])
        if format == rgbchannels[0][0]:
            return rgb
        import numpy as np
        rgb = np.array(rgb,dtype=np.uint32)
        r = np.right_shift(np.bitwise_and(rgb,0xff0000),16)
        g = np.right_shift(np.bitwise_and(rgb,0xff00),8)
        b = np.bitwise_and(rgb,0xff)
        if alphachannel is not None:  #rgba
            if alphachannel[0] == 'rgba':
                a = np.right_shift(np.bitwise_and(rgb,0xff000000),24)
            elif alphachannel[0] == 'opacity':
                a = pc.getProperties(alphachannel[0][1])
                a = (np.array(a)*255).astype(np.uint32)
            elif alphachannel[0] == 'c':
                a = pc.getProperties(alphachannel[0][1])
            else:
                raise ValueError("Weird type of alpha channel? "+alphachannel[0])
            return _color_format_from_uint8_channels(format,r,g,b,a)
        else:
            return _color_format_from_uint8_channels(format,r,g,b)
    elif len(rgbchannels) == 3:
        r=None
        g=None
        b=None
        for (name,index) in rgbchannels:
            if name=='r':
                r = pc.getProperties(index)
            elif name=='g':
                g = pc.getProperties(index)
            elif name=='b':
                b = pc.getProperties(index)
            else:
                raise ValueError("Strange, have some subset of r,g,b and other channels in point cloud? "+name)
        if r is None or g is None or b is None:
            raise ValueError("Strange, point cloud has some weird subset of r,g,b channels? "+','.join(v[0] for v in rgbchannels))
        if alphachannel is None:
            a = 1.0
        elif alphachannel[0] == 'opacity':
            a = pc.getProperties(alphachannel[0][1])
        elif alphachannel[0] == 'c':
            import numpy as np
            one_255 = 1.0/255.0
            a = (np.array(pc.getProperties(alphachannel[0][1]))*one_255).tolist()
        else:
            raise ValueError("Weird type of alpha channel? "+alphachannel[0])
        if format=='channels':
            if alphachannel is None:
                return r,g,b
            else:
                return r,g,b,a
        elif isinstance(format,(list,tuple)) and tuple(format)==('r','g','b'):
            return list(zip(r,g,b))
        elif isinstance(format,(list,tuple)) and tuple(format)==('r','g','b','a'):
            if alphachannel is None:
                a = [1.0]*pc.numPoints()
            return list(zip(r,g,b,a))
        
        import numpy as np
        r = (np.array(r)*255.0).astype(np.uint32)
        g = (np.array(g)*255.0).astype(np.uint32)
        b = (np.array(b)*255.0).astype(np.uint32)
        if alphachannel is not None:
            a = (np.array(a)*255.0).astype(np.uint32)
            return _color_format_from_uint8_channels(format,r,g,b,a)
        else:
            return _color_format_from_uint8_channels(format,r,g,b)
    elif len(rgbchannels)==0 and alphachannel is not None:
        if alphachannel[0] == 'opacity':
            import numpy as np
            a = pc.getProperties(alphachannel[0][1])
            a = (np.array(a)*255).astype(np.uint32)
        elif alphachannel[0] == 'c':
            import numpy as np
            a = pc.getProperties(alphachannel[0][1])
        else:
            raise ValueError("Weird type of alpha channel? "+alphachannel[0])
        r = [0xff]*pc.numPoints()
        return _color_format_from_uint8_channels(format,r,r,r,a)
    else:
        raise ValueError("Invalid colors in point cloud? found "+str(len(rgbchannels))+" color channels")


def point_cloud_set_colors(pc,colors,color_format='rgb',pc_property='auto'):
    """Sets the colors of a point cloud.

    Args:
        pc (PointCloud): the point cloud
        colors (list): the list of colors, which can be either ints, tuples, or
            channels, depending on color_format.
        color_format: describes the format of each element of ``colors``, and
            can be:

            - 'rgb': packed 24bit int, with the hex format 0xrrggbb,
            - 'bgr': packed 24bit int, with the hex format 0xbbggrr,
            - 'rgba': packed 32bit int, with the hex format 0xrrggbbaa,
            - 'bgra': packed 32bit int, with the hex format 0xbbggrraa,
            - 'argb': packed 32bit int, with the hex format 0xaarrggbb,
            - 'abgr': packed 32bit int, with the hex format 0xaabbggrr,
            - ('r','g','b'): triple with each channel in range [0,1]
            - ('r','g','b','a'): tuple with each channel in range [0,1]
            - 'channels': ``colors`` is a list of channels, in the form (r,g,b)
               or (r,g,b,a), where each value in the channel has range [0,1].
            - 'opacity': opacity only, in the range [0,1].

        pc_property (str): describes to which property the colors should be
            set.  'auto' determines chooses the property from the point cloud
            if it's already colored, or color_format if not.  'channels' sets
            the 'r', 'g', 'b', and optionally 'a' properties.

    Returns:
        None
    """
    rgbchannels = []
    alphachannel = None
    for i,prop in enumerate(pc.propertyNames):
        if prop in ['r','g','b','rgb']:
            rgbchannels.append((prop,i))
        elif prop == 'rgba':
            rgbchannels.append((prop,i))
            if alphachannel is not None:
                alphachannel = (prop,i)
        elif prop in ['opacity','a','c']:
            if alphachannel is not None:
                alphachannel = (prop,i)
    rgbdict = dict(rgbchannels)
    if pc_property == 'auto':
        if len(rgbchannels) == 0 and alphachannel is None:
            if color_format=='channels' or isinstance(color_format,(list,tuple)):
                pc_property = 'channels' 
            else:
                if 'a' in color_format:
                    pc_property = 'rgba'
                else:
                    pc_property = 'rgb'
        elif len(rgbchannels) == 3:
            pc_property = 'channels'
        elif len(rgbchannels) == 1:
            if alphachannel is not None:
                pc_property = 'rgba'
            else:
                pc_property = rgbchannels[0][0]
    if color_format == pc_property:
        if color_format == 'channels':
            assert len(colors)==3 or len(colors)==4,'Channels must give a 3-tuple or 4-tuple'
            for c,values in zip('rgb',colors):
                if c in rgbdict:
                    pc.setProperties(rgbdict[c],values)
                else:
                    pc.addProperty(c,values)
            if len(colors)==4:
                if alphachannel[0] == 'a':
                    pc.setProperties(alphachannel[1],colors[3])
                else:
                    pc.addProperty('a',colors[3])
        else:
            if color_format in rgbdict:
                pc.setProperties(rgbdict[color_format],colors)
            else:
                pc.addProperty(color_format,colors)
    else:
        channels = _color_format_to_uint8_channels(color_format,colors)
        packed = _color_format_from_uint8_channels(pc_property,*channels)
        if pc_property in rgbdict:
            pc.setProperties(rgbdict[pc_property],packed)
        elif alphachannel is not None and pc_property == alphachannel[0]:
            pc.setProperties(alphachannel[1],packed)
        elif pc_property == 'channels':
            pc.addProperty('r',packed[0])
            pc.addProperty('g',packed[1])
            pc.addProperty('b',packed[2])
            if len(packed)==4:
                pc.addProperty('a',packed[3])
        else:
            pc.addProperty(pc_property,packed)


def triangle_normals(trimesh):
    """
    Returns a list or numpy array of (outward) triangle normals for the
    triangle mesh defined by vertices verts and triangles tris.
    
    Args:
        trimesh (TriangleMesh or Geometry3D)

    Returns:
        list of lists or numpy array: an n x 3 matrix of triangle normals.
    """
    if isinstance(trimesh,Geometry3D):
        assert trimesh.type() == 'TriangleMesh',"Must provide a TriangleMesh to triangle_normals"
        trimesh = trimesh.getTriangleMesh()
    assert isinstance(trimesh,TriangleMesh)

    import numpy as np
    from ..io import numpy_convert

    verts,tris = numpy_convert.to_numpy(trimesh)
    normals = np.zeros(tris.shape)
    dba = verts[tris[:,1]]-verts[tris[:,0]]
    dca = verts[tris[:,2]]-verts[tris[:,0]]
    n = np.cross(dba,dca)
    norms = np.linalg.norm(n,axis=1)[:, np.newaxis]
    n = np.divide(n,norms,where=norms!=0)
    return n
