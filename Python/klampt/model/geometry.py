"""Utility functions for operating on  geometry.  See the
:class:`~klampt.Geometry3D` documentation for the core geometry class. 

.. versionadded:: 0.8.6

[functions moved here from :mod:`klampt.model.sensing`]

Working with geometric primitives
=================================

:func:`box` and :func:`sphere` are aliases for the functions in
:mod:`klampt.model.create.primitives`.

Working with point clouds
=========================

:func:`point_cloud_normals` estimates normals from a normal-free
:class:`~klampt.PointCloud`.

The :func:`fit_plane`, :func:`fit_plane3`, and :class:`PlaneFitter` class help
with plane estimation.

The :func:`align_points` and :func:`align_points_rotation` functions
solve for point cloud alignment.
.. versionadded:: 0.9.2

:func:`point_cloud_simplify` simplifies a PointCloud.

:func:`point_cloud_colors` and :func:`point_cloud_set_colors` sets / gets 
colors from a PointCloud.

Other utilities
===============

:func:`merge` can put together TriangleMesh and PointCloud geometries into
unified geometries.

:func:`triangle_normals` computes triangle normals for a TriangleMesh.

:func:`vertex_normals` computes vertex normals for a TriangleMesh.
.. versionadded:: 0.9.2

:func:`sample_surface` samples points on the surface of a Geometry3D.
.. versionadded:: 0.9.2

"""

from ..robotsim import Geometry3D,PointCloud,TriangleMesh
import math
from .create import primitives
from ..math import vectorops,so3,se3
from typing import Union, Tuple, Sequence, List
from .typing import Vector3, Rotation, RigidTransform
import numpy as np

_has_scipy = False
_tried_scipy_import = False
sp = None

box = primitives.box
"""Alias for :func:`klampt.model.create.primitives.box`"""

sphere = primitives.sphere
"""Alias for :func:`klampt.model.create.primitives.sphere`"""

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


def point_cloud_simplify(pc : Union[Geometry3D,PointCloud], radius : float) -> Union[Geometry3D,PointCloud]:
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
 

def point_cloud_normals(pc : Union[Geometry3D,PointCloud], estimation_radius=None,estimation_knn=None,estimation_viewpoint=None,add=True) -> np.ndarray:
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

    If ``add=True``, estimated normals will be added to the point cloud 
    under the ``normal_x, normal_y, normal_z`` properties.

    Returns:
        A N x 3 numpy array of normals.

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
        return np.column_stack([normal_x,normal_y,normal_z])

    if not all(i < 0 for i in inds):
        raise ValueError("Point cloud has some normal components but not all of them?")
    #need to estimate normals
    _try_scipy_import()
    positions = pc.getPoints()
    N = positions.shape[0]
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
            pt_hash[tuple(ind)].append((i,p))
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
        normal_x = normals[:,0]
        normal_y = normals[:,1]
        normal_z = normals[:,2]
        pc.addProperty('normal_x',normal_x)
        pc.addProperty('normal_y',normal_y)
        pc.addProperty('normal_z',normal_z)
        if geom is not None:
            geom.setPointCloud(pc)
    return normals


def fit_plane3(point1 : Vector3, point2 : Vector3, point3 : Vector3) -> Tuple[float,float,float,float]:
    """Returns a 3D plane equation fitting the 3 points.
  
    The result is (a,b,c,d) with the plane equation ax+by+cz+d=0
    """
    normal = np.cross(point2-point1,point3-point1)
    nlen = np.linalg.norm(normal)
    if nlen < 1e-4:
        #degenerate
        raise ValueError("Points are degenerate")
    normal = normal / nlen
    offset = -np.dot(normal,point1)
    return (normal[0],normal[1],normal[2],offset)


def fit_plane(points : Sequence[Vector3]) -> Tuple[float,float,float,float]:
    """Returns a 3D plane equation that is a least squares fit
    through the points (len(points) >= 3)."""
    centroid,normal = fit_plane_centroid(points)
    return normal[0],normal[1],normal[2],-vectorops.dot(centroid,normal)


def fit_plane_centroid(points : Sequence[Vector3]) -> Tuple[Vector3,Vector3]:
    """Similar to :func:`fit_plane`, but returns a (centroid,normal) pair."""
    if len(points)<3:
        raise ValueError("Need to have at least 3 points to fit a plane")
    #if len(points)==3:
    #    return fit_plane3(points[0],points[1],points[2])
    points = np.asarray(points)
    centroid = np.average(points,axis=0)
    U,W,Vt = np.linalg.svd(points-[centroid]*len(points),full_matrices=False)
    if np.sum(W<1e-6) > 1:
        raise ValueError("Point set is degenerate")
    normal = Vt[2,:]
    return centroid.tolist(),normal.tolist()




def align_points_rotation(apts,bpts) -> Rotation:
    """Computes a 3x3 rotation matrix that rotates the points apts to
    minimize the distance to bpts.

    apts and bpts can either be a list of 3-vectors, nx3 numpy array,
    or a PointCloud.

    Returns:
        Rotation: the klampt.so3 element that minimizes the sum of
        squared errors ||R*ai-bi||^2.
    """
    if isinstance(apts, PointCloud):
        apts = apts.getPoints()
    if isinstance(bpts, PointCloud):
        bpts = bpts.getPoints()
    assert len(apts)==len(bpts)

    C = np.dot(np.asarray(apts).T,np.asarray(bpts))
    #let A=[a1 ... an]^t, B=[b1 ... bn]^t
    #solve for min sum of squares of E=ARt-B
    #let C=AtB
    #solution is given by CCt = RtCtCR

    #Solve C^tR = R^tC with SVD CC^t = R^tC^tCR
    #CtRX = RtCX
    #C = RCtR
    #Ct = RtCRt
    #=> CCt = RCtCRt
    #solve SVD of C and Ct (giving eigenvectors of CCt and CtC
    #C = UWVt => Ct=VWUt
    #=> UWUt = RVWVtRt
    #=> U=RV => R=UVt
    (U,W,Vt) = np.linalg.svd(C)

    R = np.dot(U,Vt)
    if np.linalg.det(R) < 0:
        #it's a mirror. flip the zero 
        #svd.sortSVs();
        if abs(W[2]) > 1e-2:
            raise RuntimeError("point_fit_rotation_3d: Uhh... what do we do?  SVD of rotation doesn't have a zero singular value")
        #negate the last column of V
        Vt[2,:] *= -1
        R = np.dot(U,Vt)
        assert np.linalg.det(R) > 0
    return R.flatten().tolist()


def align_points(apts,bpts) -> RigidTransform:
    """Finds a 3D rigid transform that maps the list of points apts to the
    list of points bpts. 
    
    apts and bpts can either be a list of 3-vectors, nx3 numpy array,
    or a PointCloud.

    Returns:
        RigidTransform: the klampt.se3 element that minimizes the sum of
        squared errors ||T*ai-bi||^2.
    """
    if isinstance(apts, PointCloud):
        apts = apts.getPoints()
    if isinstance(bpts, PointCloud):
        bpts = bpts.getPoints()
    assert len(apts)==len(bpts)
    apts = np.asarray(apts)
    bpts = np.asarray(bpts)
    ca = np.average(apts,axis=0)
    cb = np.average(bpts,axis=0)
    arel = apts - ca
    brel = bpts - cb
    R = align_points_rotation(arel,brel)
    #R minimizes sum_{i=1,...,n} ||R(ai-ca) - (bi-cb)||^2
    t = cb - so3.apply(R,ca)
    return (R,t.tolist())



def _color_format_from_uint8_channels(format,r,g,b,a=None):
    if a is None:
        a = 0xff
    if format == 'rgb':
        r = np.asarray(r).astype(np.uint32)
        g = np.asarray(g).astype(np.uint32)
        b = np.asarray(b).astype(np.uint32)
        return np.bitwise_or.reduce((np.left_shift(r,16),np.left_shift(g,8),b))
    elif format == 'bgr':
        r = np.asarray(r).astype(np.uint32)
        g = np.asarray(g).astype(np.uint32)
        b = np.asarray(b).astype(np.uint32)
        return np.bitwise_or.reduce((np.left_shift(b,16),np.left_shift(g,8),r))
    elif format=='rgba':
        r = np.asarray(r).astype(np.uint32)
        g = np.asarray(g).astype(np.uint32)
        b = np.asarray(b).astype(np.uint32)
        a = np.asarray(a).astype(np.uint32)
        return np.bitwise_or.reduce((np.left_shift(r,24),np.left_shift(g,16),np.left_shift(b,8),a))
    elif format=='bgra':
        r = np.asarray(r).astype(np.uint32)
        g = np.asarray(g).astype(np.uint32)
        b = np.asarray(b).astype(np.uint32)
        a = np.asarray(a).astype(np.uint32)
        return np.bitwise_or.reduce((np.left_shift(g,24),np.left_shift(g,16),np.left_shift(r,8),a))
    elif format=='argb':
        r = np.asarray(r).astype(np.uint32)
        g = np.asarray(g).astype(np.uint32)
        b = np.asarray(b).astype(np.uint32)
        a = np.asarray(a).astype(np.uint32)
        return np.bitwise_or.reduce((np.left_shift(a,24),np.left_shift(r,16),np.left_shift(g,8),b))
    elif format=='abgr':
        r = np.asarray(r).astype(np.uint32)
        g = np.asarray(g).astype(np.uint32)
        b = np.asarray(b).astype(np.uint32)
        a = np.asarray(a).astype(np.uint32)
        return np.bitwise_or.reduce((np.left_shift(a,24),np.left_shift(b,16),np.left_shift(g,8),r))
    elif format=='channels':
        one_255 = 1.0/255.0
        r = np.asarray(r)
        g = np.asarray(g)
        b = np.asarray(b)
        if not hasattr(a,'__iter__'):
            return (r*one_255),(g*one_255),(b*one_255)
        else:
            a = np.asarray(a)
            return (r*one_255),(g*one_255),(b*one_255),(a*one_255)
    elif format=='opacity':
        one_255 = 1.0/255.0
        if not hasattr(a,'__iter__'):
            return np.ones(len(r))
        a = np.asarray(a)
        return (a*one_255)
    elif tuple(format)==('r','g','b'):
        one_255 = 1.0/255.0
        r = np.asarray(r)
        g = np.asarray(g)
        b = np.asarray(b)
        return np.column_stack((r*one_255,g*one_255,b*one_255))
    elif tuple(format)==('r','g','b','a'):
        one_255 = 1.0/255.0
        if not hasattr(a,'__iter__'):
            a = np.full(len(r),a)
        else:
            a = np.asarray(a)
        r = np.asarray(r)
        g = np.asarray(g)
        b = np.asarray(b)
        return np.column_stack((r*one_255,g*one_255,b*one_255,a*one_255))
    else:
        raise ValueError("Invalid format specifier "+str(format))


def _color_format_to_uint8_channels(format,colors):
    if format=='channels':
        return tuple((np.asarray(c)*255).astype(np.uint8) for c in colors)
    colors = np.asarray(colors)
    if format == 'rgb':
        r,g,b = np.right_shift(np.bitwise_and(colors,0xff0000),16),np.right_shift(np.bitwise_and(colors,0xff00),8),np.bitwise_and(colors,0xff)
        return r,g,b
    elif format == 'bgr':
        b,g,r = np.right_shift(np.bitwise_and(colors,0xff0000),16),np.right_shift(np.bitwise_and(colors,0xff00),8),np.bitwise_and(colors,0xff)
        return r,g,b
    elif format=='rgba':
        r,g,b,a = np.right_shift(np.bitwise_and(colors,0xff000000),24),np.right_shift(np.bitwise_and(colors,0xff0000),16),np.right_shift(np.bitwise_and(colors,0xff00),8),np.bitwise_and(colors,0xff)
        return r,g,b,a
    elif format=='bgra':
        b,g,r,a = np.right_shift(np.bitwise_and(colors,0xff000000),24),np.right_shift(np.bitwise_and(colors,0xff0000),16),np.right_shift(np.bitwise_and(colors,0xff00),8),np.bitwise_and(colors,0xff)
        return r,g,b,a
    elif format=='argb':
        a,r,g,b = np.right_shift(np.bitwise_and(colors,0xff000000),24),np.right_shift(np.bitwise_and(colors,0xff0000),16),np.right_shift(np.bitwise_and(colors,0xff00),8),np.bitwise_and(colors,0xff)
        return r,g,b,a
    elif format=='abgr':
        a,b,g,r = np.right_shift(np.bitwise_and(colors,0xff000000),24),np.right_shift(np.bitwise_and(colors,0xff0000),16),np.right_shift(np.bitwise_and(colors,0xff00),8),np.bitwise_and(colors,0xff)
        return r,g,b,a
    elif format=='opacity':
        r = [0xff]*len(colors)
        return r,r,r,(colors*255).astype(np.uint8)
    elif tuple(format)==('r','g','b'):
        colors = np.asarray(colors)
        r = np.rint(colors[:,0]*255).astype(np.uint8)
        g = np.rint(colors[:,1]*255).astype(np.uint8)
        b = np.rint(colors[:,2]*255).astype(np.uint8)
        return r,g,b
    elif tuple(format)==('r','g','b','a'):
        colors = np.asarray(colors)
        r = np.rint(colors[:,0]*255).astype(np.uint8)
        g = np.rint(colors[:,1]*255).astype(np.uint8)
        b = np.rint(colors[:,2]*255).astype(np.uint8)
        a = np.rint(colors[:,3]*255).astype(np.uint8)
        return r,g,b,a
    else:
        raise ValueError("Invalid format specifier "+str(format))


def point_cloud_colors(pc : PointCloud, format='rgb') -> np.ndarray:
    """Returns the colors of the point cloud in the given format.  If the
    point cloud has no colors, this returns None.  If the point cloud has no
    colors but has opacity, this returns white colors.

    Args:
        pc (PointCloud): the point cloud
        format: describes the output color format, either:

            - 'rgb': packed 32bit int, with the hex format 0xrrggbb (only 24
               bits used),
            - 'bgr': packed 32bit int, with the hex format 0xbbggrr (only 24
               bits used),
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
        A an array of pc.numPoints() colors corresponding to 
        the points in the point cloud.  If format='channels', the return
        value is a tuple (r,g,b) or (r,g,b,a).
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
        if format == 'rgb' and rgbchannels[0][0] == 'rgb':
            return rgb
        if format == 'argb' and rgbchannels[0][0] == 'rgba':
            return rgb
        rgb = np.asarray(rgb).astype(dtype=np.uint32)
        r = np.right_shift(np.bitwise_and(rgb,0xff0000),16)
        g = np.right_shift(np.bitwise_and(rgb,0xff00),8)
        b = np.bitwise_and(rgb,0xff)
        if alphachannel is not None:  #rgba
            if alphachannel[0] == 'rgba':
                a = np.right_shift(np.bitwise_and(rgb,0xff000000),24)
            elif alphachannel[0] == 'opacity':
                a = pc.getProperties(alphachannel[0][1])
                a = np.rint(np.asarray(a)*255).astype(np.uint32)
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
            one_255 = 1.0/255.0
            a = (np.asarray(pc.getProperties(alphachannel[0][1]))*one_255)
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
                a = np.full(pc.numPoints(),1.0)
            return np.column_stack((r,g,b,a))
        
        r = np.rint(np.asarray(r)*255.0).astype(np.uint32)
        g = np.rint(np.asarray(g)*255.0).astype(np.uint32)
        b = np.rint(np.asarray(b)*255.0).astype(np.uint32)
        if alphachannel is not None:
            a = np.rint(np.asarray(a)*255.0).astype(np.uint32)
            return _color_format_from_uint8_channels(format,r,g,b,a)
        else:
            return _color_format_from_uint8_channels(format,r,g,b)
    elif len(rgbchannels)==0 and alphachannel is not None:
        if alphachannel[0] == 'opacity':
            a = pc.getProperties(alphachannel[0][1])
            a = (np.asarray(a)*255).astype(np.uint32)
        elif alphachannel[0] == 'c':
            a = pc.getProperties(alphachannel[0][1])
        else:
            raise ValueError("Weird type of alpha channel? "+alphachannel[0])
        r = [0xff]*pc.numPoints()
        return _color_format_from_uint8_channels(format,r,r,r,a)
    else:
        raise ValueError("Invalid colors in point cloud? found "+str(len(rgbchannels))+" color channels")


def point_cloud_set_colors(pc : PointCloud, colors, color_format='rgb',pc_property='auto') -> None:
    """Sets the colors of a point cloud.

    Args:
        pc (PointCloud): the point cloud
        colors (list or numpy.ndarray): the array of colors, and each color 
            can be either ints, tuples, or channels, depending on color_format.
        color_format: describes the format of each element of ``colors``, and
            can be:

            - 'rgb': packed 32bit int, with the hex format 0xrrggbb (only 24
               bits used),
            - 'bgr': packed 32bit int, with the hex format 0xbbggrr (only 24
               bits used),
            - 'rgba': packed 32bit int, with the hex format 0xrrggbbaa,
            - 'bgra': packed 32bit int, with the hex format 0xbbggrraa,
            - 'argb': packed 32bit int, with the hex format 0xaarrggbb,
            - 'abgr': packed 32bit int, with the hex format 0xaabbggrr,
            - ('r','g','b'): triple with each channel in range [0,1]. Also use
              this if colors is an n x 3 numpy array.
            - ('r','g','b','a'): tuple with each channel in range [0,1]. Also 
              use this if colors is an n x 4 numpy array.
            - 'channels': ``colors`` is a list of 3 or 4 channels, in the form
               (r,g,b) or (r,g,b,a), where each element in a channel has range
               [0,1].
            - 'opacity': opacity only, in the range [0,1].

        pc_property (str): describes to which property the colors should be
            set.  'auto' determines chooses the property from the point cloud
            if it's already colored, or color_format if not.  'channels' sets
            the 'r', 'g', 'b', and optionally 'a' properties.

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
    pc_color_format = pc_property
    if pc_property == 'rgba':
        pc_color_format = 'argb'

    if color_format == pc_color_format:
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
        packed = _color_format_from_uint8_channels(pc_color_format,*channels)
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


def triangle_normals(trimesh : Union[TriangleMesh,Geometry3D]) -> np.ndarray:
    """
    Returns a list or numpy array of (outward) triangle normals for a
    triangle mesh.
    
    Args:
        trimesh (TriangleMesh or Geometry3D)

    Returns:
        An N x 3 matrix of triangle normals with N the number of triangles.
    """
    if isinstance(trimesh,Geometry3D):
        assert trimesh.type() == 'TriangleMesh',"Must provide a TriangleMesh to triangle_normals"
        trimesh = trimesh.getTriangleMesh()
    assert isinstance(trimesh,TriangleMesh),"Must provide a TriangleMesh to triangle_normals"
    verts=trimesh.getVertices()
    tris=trimesh.getIndices()
    #normals = np.zeros(tris.shape)
    dba = verts[tris[:,1]]-verts[tris[:,0]]
    dca = verts[tris[:,2]]-verts[tris[:,0]]
    n = np.cross(dba,dca)
    norms = np.linalg.norm(n,axis=1)[:, np.newaxis]
    n = np.divide(n,norms,where=norms!=0)
    return n

def vertex_normals(trimesh : Union[TriangleMesh,Geometry3D], area_weighted=True) -> np.ndarray:
    """
    Returns a list or numpy array of (outward) vertex normals for a
    triangle mesh.
    
    Args:
        trimesh (TriangleMesh or Geometry3D)
        area_weighted (bool): whether to compute area-weighted average or
            simple average.

    Returns:
        An N x 3 matrix of triangle normals with N the number of vertices.
    """
    if isinstance(trimesh,Geometry3D):
        assert trimesh.type() == 'TriangleMesh',"Must provide a TriangleMesh to vertex_normals"
        trimesh = trimesh.getTriangleMesh()
    assert isinstance(trimesh,TriangleMesh),"Must provide a TriangleMesh to vertex_normals"
    verts=trimesh.getVertices()
    tris=trimesh.getIndices()
    dba = verts[tris[:,1]]-verts[tris[:,0]]
    dca = verts[tris[:,2]]-verts[tris[:,0]]
    n = np.cross(dba,dca)
    normals = [np.zeros(3) for i in range(len(verts))]
    if area_weighted:
        for i,t in enumerate(tris):
            for j in range(3):
                normals[t[j]] += n[i]
    else:
        norms = np.linalg.norm(n,axis=1)[:, np.newaxis]
        n = np.divide(n,norms,where=norms!=0)
        for i,t in enumerate(tris):
            for j in range(3):
                normals[t[j]] += n[i]
    normals = np.array(normals)
    norms = np.linalg.norm(normals,axis=1)[:, np.newaxis]
    return np.divide(normals,norms,where=norms!=0)


def merge(*items) -> Geometry3D:
    """Merges one or more geometries into a single geometry.
    
    Args:
        items: Each arg must be a Geometry3D, TriangleMesh, PointCloud,
            RobotLinkModel, RigidObjectModel, or TerrainModel.
    
    Returns:
        A merged geometry. If all underlying geometries have PointCloud type,
        then the result will have PointCloud type.  Otherwise, they must all
        be convertable to TriangleMesh, and the result will be of TriangleMesh
        type.
    """
    from klampt.io import numpy_convert

    xforms = []
    tri_meshes = []
    point_clouds = []
    for item in items:
        if isinstance(item,TriangleMesh):
            xforms.append(se3.identity())
            tri_meshes.append(item)
            continue
        if isinstance(item,PointCloud):
            xforms.append(se3.identity())
            point_clouds.append(item)
            continue
        geom = item
        if hasattr(item,'geometry') and callable(item.geometry):
            geom = item.geometry()
        if len(items) == 1:
            return geom
        if isinstance(geom,Geometry3D):
            xforms.append(geom.getCurrentTransform())
            if item.type() == 'TriangleMesh':
                tri_meshes.append(geom.getTriangleMesh())
            elif item.type() == 'PointCloud':
                point_clouds.append(geom.getPointCloud())
            else:
                tri_meshes.append(geom.convert('TriangleMesh'))
        else:
            raise ValueError("Can't merge item of type "+str(type(item)))
    #print("Merging",len(point_clouds),"point clouds and",len(tri_meshes),"triangle meshes")
    if len(point_clouds) != 0:
        if len(tri_meshes) != 0:
            raise ValueError("Can't pass mixed PointCloud and TriangleMesh types")
        all_points = []
        for xform,pc in point_clouds:
            iverts = numpy_convert.to_numpy(pc,'PointCloud')
            all_points.append(np.dot(np.hstack(iverts,np.ones((len(iverts),1))),se3.ndarray(xform).T))
        if not all(pc.shape[1]==all_points[0].shape[1] for pc in all_points):
            raise ValueError("Mismatch in PointCloud # of properties, can't merge")
        points = np.vstack(all_points)
        pc = numpy_convert.from_numpy(points,'PointCloud')
        return Geometry3D(pc)
    elif len(tri_meshes) != 0:
        verts = []
        tris = []
        nverts = 0
        for xform,tm in zip(xforms,tri_meshes):
            (iverts,itris) = numpy_convert.to_numpy(tm,'TriangleMesh')
            verts.append(np.dot(np.hstack((iverts,np.ones((len(iverts),1)))),se3.ndarray(xform).T)[:,:3])
            tris.append(itris+nverts)
            nverts += len(iverts)
        verts = np.vstack(verts)
        tris = np.vstack(tris)
        for t in tris:
            assert all(v >= 0 and v < len(verts) for v in t)
        mesh = numpy_convert.from_numpy((verts,tris),'TriangleMesh')
        merged_geom = Geometry3D()
        merged_geom.setTriangleMesh(mesh)
        return merged_geom
    else:
        return Geometry3D()


def sample_surface(geom : Geometry3D,
                   num_samples=None,
                   local=True,
                   want_elements=False,
                   want_normals=False) -> np.ndarray:
    """Samples the surface of a geometry uniformly at random.

    If local=True, points are returned in the local coordinate frame.
    Otherwise, the object's current transform is applied.

    If want_elements=True, then the indices of the elements (i.e., triangles
    or point indices) are returned the result column 3.  This only makes sense
    for TriangleMesh and PointCloud types.

    If want_normals=True, then the normals at each point are returned in the
    result columns [3:6] (if want_elements=False) or [4:7] if
    (want_elements=True).
    
    If num_samples = None, then all vertices of the representation will be
    returned.  This cannot be None for spheres.
    """
    points = None
    elements = None
    normals = None
    if geom.type() == 'TriangleMesh':
        tm = geom.getTriangleMesh()
        verts = tm.getVertices()
        tris = tm.getIndices()
        if num_samples is None:
            points = verts
            if want_elements:
                vindices = [-1] * len(verts)
                for i,t in enumerate(tris):
                    for j in t:
                        vindices[j] = i
                elements = vindices
            if want_normals:
                normals = vertex_normals(geom)
        else:
            #need to sample the surface
            dba = verts[tris[:,1]]-verts[tris[:,0]]
            dca = verts[tris[:,2]]-verts[tris[:,0]]
            trinormals = np.cross(dba,dca)
            norms = np.linalg.norm(trinormals,axis=1)
            sample = np.random.choice(len(tris),num_samples,p=norms/np.sum(norms))
            uv = np.random.rand(num_samples,2)
            points = []
            normals = []
            for i,elem in enumerate(sample):
                a,b,c = tris[elem]
                va = verts[a]
                vb = verts[b]
                vc = verts[c]
                u,v = uv[i]
                if u+v > 1:
                    v2 = (1-u)
                    u2 = (1-v)
                    u,v = u2,v2
                pt = va*(1-u-v) + vb*u + vc*v
                n = trinormals[elem]/norms[elem]
                points.append(pt)
                normals.append(n)
            elements = sample
    elif geom.type() == 'PointCloud':
        allPoints = geom.getPointCloud().getPoints()
        if num_samples is None:
            points = allPoints
            if want_elements:
                elements = list(range(len(allPoints)))
            if want_normals:
                normals = point_cloud_normals(geom)
        else:
            sample = np.random.choice(len(allPoints),num_samples)
            points = [allPoints[i] for i in sample]
            elements = sample
            if want_normals:
                allNormals = point_cloud_normals(geom,sample)
                normals = [allNormals[i] for i in sample]
    elif geom.type() in ['VolumeGrid','ConvexHull']:
        return sample_surface(geom.convert('TriangleMesh'),num_samples,local,want_elements,want_normals)
    elif geom.type() == 'GeometricPrimitive':
        prim = geom.getGeometricPrimitive()
        if prim.type() == 'Sphere':
            c = prim.properties[0],prim.properties[1],prim.properties[2]
            r = prim.properties[3]
            if num_samples is None:
                raise ValueError("Cannot sample a sphere without providing a number of samples")
            points = []
            pts = np.random.normal(size=(num_samples,3))
            pts /= np.linalg.norm(pts,axis=1)[:,np.newaxis]
            normals = pts
            points = pts * r + c
        else:
            return sample_surface(geom.convert('TriangleMesh'),num_samples,local,want_elements,want_normals)
        if want_elements:
            elements = [-1]*len(points)
    else:
        raise ValueError("Unable to sample surface of geometry type "+geom.type())
    if not local:
        R,t = geom.getCurrentTransform()
        points = np.asarray(points).dot(so3.ndarray(R).T) + np.asarray(t)
        if want_normals:
            normals = np.asarray(normals).dot(so3.ndarray(R).T)
    items = [points]
    if want_elements:
        items.append(np.asarray(elements)[:,np.newaxis])
    if want_normals:
        items.append(normals)
    if len(items)==1:
        return items[0]
    return np.hstack(items)
        