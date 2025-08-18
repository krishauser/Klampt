"""Utility functions for operating on  geometry.  See the
:class:`~Geometry3D` documentation for the core geometry class. 

.. versionadded:: 0.8.6

[functions moved here from :mod:`model.sensing`]

Working with geometric primitives
=================================

:func:`box` and :func:`sphere` are aliases for the functions in
:mod:`model.create.primitives`.

Working with point clouds
=========================

:func:`point_cloud_normals` estimates normals from a normal-free
:class:`~PointCloud`.

The :func:`fit_plane`, :func:`fit_plane3`, and :class:`PlaneFitter` class help
with plane estimation.

The :func:`align_points` and :func:`align_points_rotation` functions
solve for point cloud alignment.
.. versionadded:: 0.9.2

:func:`point_cloud_simplify` simplifies a PointCloud.

:func:`point_cloud_colors` and :func:`point_cloud_set_colors` sets / gets 
colors from a PointCloud.

Working with heightmaps
========================

:func:`upper_heightmap` and :func:`lower_heightmap` create a heightmap
from the top down or bottom up, respectively, of a :class:`~Geometry3D` object
that matches the array of an existing heightmap.

:func:`heightmap_normals` computes normals for a heightmap.

Other utilities
===============

:func:`merge` can put together TriangleMesh and PointCloud geometries into
unified geometries.

:func:`split` can break apart a Geometry3D into segmented parts.
.. versionadded:: 0.10.2

:func:`triangle_normals` computes triangle normals for a TriangleMesh.
Note: should be replaced with TriangleMesh.triangle_normals() for future
code.

:func:`vertex_normals` computes vertex normals for a TriangleMesh.
Note: should be replaced with TriangleMesh.triangle_normals() for future
code.
.. versionadded:: 0.9.2

:func:`sample_surface` samples points on the surface of a Geometry3D.
.. versionadded:: 0.9.2

:func:`convert` converts all geometries in a world, robot, etc to another
type.
.. versionadded:: 0.10.1


"""

from ..robotsim import WorldModel,RobotModel,RigidObjectModel,RobotModelLink,TerrainModel,Geometry3D,PointCloud,ImplicitSurface,OccupancyGrid,Heightmap,TriangleMesh,Appearance
import math
from .create import primitives
from ..math import vectorops,so3,se3
from typing import Union, Tuple, Sequence, List, Callable
from .typing import Vector3, Rotation, RigidTransform
import numpy as np

_has_scipy = False
_tried_scipy_import = False
sp = None

box = primitives.box
"""Alias for :func:`model.create.primitives.box`"""

sphere = primitives.sphere
"""Alias for :func:`model.create.primitives.sphere`"""

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
        warnings.warn("model.geometry.py: scipy not available.",ImportWarning)
        _has_scipy = False
    return _has_scipy

class PlaneFitter:
    """
    Online fitting of planes to 3D point clouds.  Updating via
    :meth:`add_point` and retrieving via :meth:`plane_equation`
    is O(1).
   
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
        A N x 3 numpy array of normals in the local frame of the point cloud.

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
            ind = props.index(pc.getPropertyName(i))
            inds[ind] = i
        except ValueError:
            pass
    if all(i>=0 for i in inds):
        #has the properties!
        normal_x = pc.properties[:,inds[0]]
        normal_y = pc.properties[:,inds[1]]
        normal_z = pc.properties[:,inds[2]]
        return np.column_stack([normal_x,normal_y,normal_z])

    if not all(i < 0 for i in inds):
        raise ValueError("Point cloud has some normal components but not all of them?")
    #need to estimate normals
    _try_scipy_import()
    positions = pc.points
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
        Rotation: the so3 element that minimizes the sum of
        squared errors ||R*ai-bi||^2.
    """
    if isinstance(apts, PointCloud):
        apts = apts.points
    if isinstance(bpts, PointCloud):
        bpts = bpts.points
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
        RigidTransform: the se3 element that minimizes the sum of
        squared errors ||T*ai-bi||^2.
    """
    if isinstance(apts, PointCloud):
        apts = apts.points
    if isinstance(bpts, PointCloud):
        bpts = bpts.points
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
        A an array of len(pc.points) colors corresponding to 
        the points in the point cloud.  If format='channels', the return
        value is a tuple (r,g,b) or (r,g,b,a).
    """
    rgbchannels = []
    alphachannel = None
    for i in range(pc.numProperties()):
        prop = pc.getPropertyName(i)
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
        rgb = pc.properties[:,rgbchannels[0][1]]
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
                a = pc.properties[:,alphachannel[0][1]]
                a = np.rint(np.asarray(a)*255).astype(np.uint32)
            elif alphachannel[0] == 'c':
                a = pc.properties[:,alphachannel[0][1]]
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
                r = pc.properties[:,index]
            elif name=='g':
                g = pc.properties[:,index]
            elif name=='b':
                b = pc.properties[:,index]
            else:
                raise ValueError("Strange, have some subset of r,g,b and other channels in point cloud? "+name)
        if r is None or g is None or b is None:
            raise ValueError("Strange, point cloud has some weird subset of r,g,b channels? "+','.join(v[0] for v in rgbchannels))
        if alphachannel is None:
            a = 1.0
        elif alphachannel[0] == 'opacity':
            a = pc.properties[:,alphachannel[0][1]]
        elif alphachannel[0] == 'c':
            one_255 = 1.0/255.0
            a = pc.properties[:,alphachannel[0][1]]*one_255
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
                a = np.full(len(pc.points),1.0)
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
            a = pc.properties[:,alphachannel[0][1]]
            a = (np.asarray(a)*255).astype(np.uint32)
        elif alphachannel[0] == 'c':
            a = pc.properties[:,alphachannel[0][1]]
        else:
            raise ValueError("Weird type of alpha channel? "+alphachannel[0])
        r = [0xff]*len(pc.points)
        return _color_format_from_uint8_channels(format,r,r,r,a)
    else:
        raise ValueError("Invalid colors in point cloud? found "+str(len(rgbchannels))+" color channels")


def point_cloud_set_colors(pc : PointCloud, colors : Union[list,np.ndarray], color_format='rgb',pc_property='auto') -> None:
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
    for i in range(pc.numProperties()):
        prop = pc.getPropertyName(i)
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
                    pc.properties[:,rgbdict[c]] = values
                else:
                    pc.addProperty(c,values)
            if len(colors)==4:
                if alphachannel[0] == 'a':
                    pc.properties[:,alphachannel[1]] = colors[3]
                else:
                    pc.addProperty('a',colors[3])
        else:
            if color_format in rgbdict:
                pc.properties[:,rgbdict[color_format]] = colors
            else:
                pc.addProperty(color_format,colors)
    else:
        channels = _color_format_to_uint8_channels(color_format,colors)
        packed = _color_format_from_uint8_channels(pc_color_format,*channels)
        if pc_property in rgbdict:
            pc.properties[:,rgbdict[pc_property]] = packed
        elif alphachannel is not None and pc_property == alphachannel[0]:
            pc.properties[:,alphachannel[1]] = packed
        elif pc_property == 'channels':
            pc.addProperty('r',packed[0])
            pc.addProperty('g',packed[1])
            pc.addProperty('b',packed[2])
            if len(packed)==4:
                pc.addProperty('a',packed[3])
        else:
            propind = pc.addProperty(pc_property,packed)
            assert isinstance(propind,int)
            pc.properties[:,propind] = packed


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
    return trimesh.triangleNormals()


def vertex_normals(trimesh : Union[TriangleMesh,Geometry3D], area_weighted=True) -> np.ndarray:
    """
    Returns a list or numpy array of (outward) vertex normals for a
    triangle mesh.  The result is in the local frame of the mesh.
    
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
    return trimesh.vertexNormals()


def merge(*items) -> Geometry3D:
    """Merges one or more geometries into a single geometry.

    If the geometries have different colors, vertex/face colors, or texture
    maps, they are converted to vertex colors before merging.
    
    Args:
        items: Each arg must be a Geometry3D, TriangleMesh, PointCloud,
            RobotLinkModel, RigidObjectModel, or TerrainModel.
    
    Returns:
        A merged geometry. If all underlying geometries have PointCloud type,
        then the result will have PointCloud type.  Otherwise, they must all
        be convertable to TriangleMesh, and the result will be of TriangleMesh
        type.
    """
    from ..io import numpy_convert

    xforms = []
    tri_meshes = []
    point_clouds = []
    appearances = []
    for i,item in enumerate(items):
        if isinstance(item,TriangleMesh):
            xforms.append(se3.identity())
            tri_meshes.append(item)
            continue
        if isinstance(item,PointCloud):
            xforms.append(se3.identity())
            point_clouds.append(item)
            continue
        geom = item
        app = None
        if hasattr(item,'geometry') and callable(item.geometry):
            geom = item.geometry()
            app = item.appearance() if hasattr(item,'appearance') and callable(item.appearance) else None
        if len(items) == 1:
            return geom
        if isinstance(geom,Geometry3D):
            xforms.append(geom.getCurrentTransform())
            if item.type() == 'TriangleMesh':
                tri_meshes.append(geom.getTriangleMesh())
                if app is None:
                    app = geom.getAppearance()
                appearances.append(app)
            elif item.type() == 'PointCloud':
                point_clouds.append(geom.getPointCloud())
            elif item.type() == 'Group':
                g = merge(*[geom.getElement(i) for i in range(geom.numElements())])
                tri_meshes.append(g.getTriangleMesh())
                if app is None:
                    app = g.getAppearance()
                appearances.append(app)
            else:
                g = geom.convert('TriangleMesh')
                tri_meshes.append(g.getTriangleMesh())
                if app is None:
                    app = g.getAppearance()
                appearances.append(app)
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
        
        #merge appearances.  Check if there are any textures or vertex colors
        has_appearance = False
        has_vertex_coloration = False
        has_face_coloration = False
        for app in appearances:
            if app.getTexture2D_format() != '' or len(app.getColors(Appearance.VERTICES)) > 1:
                has_appearance = True
                has_vertex_coloration = True
            if len(app.getColors(Appearance.FACES)) > 1:
                has_appearance = True
                has_face_coloration = True
            fcol = app.getColor()
            if fcol != [0.5,0.5,0.5,1.0]:
                has_appearance = True
        if has_appearance and not has_face_coloration and not has_vertex_coloration:
            #check whether all items have the same color; this looks better
            for app in appearances:
                fcol = app.getColor()
                if fcol != appearances[0].getColor():
                    has_face_coloration = True
                    break
        if has_appearance:
            if has_vertex_coloration:
                fcols = []
                for app in appearances:
                    fcols.append(app.getColor())
                default_color = np.mean(fcols,axis=0) if len(fcols) > 0 else np.array([0.5,0.5,0.5,1.0])
                vcols = []
                for tm,app in zip(tri_meshes,appearances):
                    if app.getColor() == [0.5,0.5,0.5,1.0]:  #default color
                        vcols.append(np.full((tm.vertices.shape[0],4),default_color))
                    else:
                        vcols.append(vertex_colors(tm,app))
                vcols = np.concatenate(vcols,axis=0).astype(np.float32)
                app = Appearance()
                app.setColors(Appearance.VERTICES,vcols)
                merged_geom.setAppearance(app)
            elif has_face_coloration:
                fcols = []
                for tm,app in zip(tri_meshes,appearances):
                    fcol = app.getColors(Appearance.FACES)
                    if len(fcol) == 1:
                        fcol = fcol.repeat(tm.indices.shape[0],axis=0)
                    fcols.append(fcol)
                fcols = np.concatenate(fcols,axis=0).astype(np.float32)
                app = Appearance()
                app.setColors(Appearance.FACES,fcols)
                merged_geom.setAppearance(app)
            else:
                merged_geom.setAppearance(appearances[0])
        return merged_geom
    else:
        return Geometry3D()


def split(item : Union[Geometry3D,TriangleMesh,PointCloud,ImplicitSurface,OccupancyGrid,Heightmap],
          by : Union[List[int],Callable]) -> List[Union[Geometry3D,TriangleMesh,PointCloud,ImplicitSurface,OccupancyGrid,Heightmap]]:
    """Splits a geometry into multiple geometries based on a segmentation
    criterion.  The criterion can be a list of segment indices, or a function
    that segments the elements of the geometry.

    Args:
        item: The geometry to split.  Does not work for primitives.
        by: A list of segment indices, or a function that takes an element
            of the geometry in local coordinates and returns a segment index or
            boolean mask. 
    
    For `by` a callable, the element type is a vertex for PointCloud, ImplicitSurface,
    Heightmap, and OccupancyGrid.  For TriangleMesh, the element type is a triangle
    of the form (v0,v1,v2) where v0,v1,v2 are 3-vectors.  All elements are in local
    coordinates of the geometry.

    Returns:
        A list of geometries.  The type of the geometries will match that of
        the input geometry.
    """
    elements = None
    indices = None
    if isinstance(item,Geometry3D):
        gtype = item.type()
        if gtype == 'PointCloud':
            res = [Geometry3D(pc) for pc in split(item.getPointCloud(),by)]
            for g in res:
                g.setCurrentTransform(*item.getCurrentTransform())
            return res
        elif gtype == 'TriangleMesh':
            res = [Geometry3D(tm) for tm in split(item.getTriangleMesh(),by)]
            for g in res:
                g.setCurrentTransform(*item.getCurrentTransform())
            return res
        elif gtype == 'ImplicitSurface':
            res = [Geometry3D(isurf) for isurf in split(item.getImplicitSurface(),by)]
            for g in res:
                g.setCurrentTransform(*item.getCurrentTransform())
            return res
        elif gtype == 'OccupancyGrid':
            res = [Geometry3D(og) for og in split(item.getOccupancyGrid(),by)]
            for g in res:
                g.setCurrentTransform(*item.getCurrentTransform())
            return res
        elif gtype == 'Heightmap':
            res = [Geometry3D(hm) for hm in split(item.getHeightmap(),by)]
            for g in res:
                g.setCurrentTransform(*item.getCurrentTransform())
            return res
        elif gtype == 'Group':
            n = item.numElements()
            elements = [item.getElement(i) for i in range(n)]
        else:
            raise ValueError("Geometry type {} not supported for splitting".format(gtype))
    elif isinstance(item,PointCloud):
        elements = item.points
    elif isinstance(item,TriangleMesh):
        elements = [(item.vertices[a],item.vertices[b],item.vertices[c]) for (a,b,c) in item.indices]
        indices = item.indices
    elif isinstance(item,ImplicitSurface):
        #elements are points at the centers of each voxel
        elements = []
        indices = []
        dims = item.bmax - item.bmin
        resolution = [d/s for (d,s) in zip(dims,item.values.shape)]
        for i in range(item.values.shape[0]):
            for j in range(item.values.shape[1]):
                for k in range(item.values.shape[2]):
                    elements.append((item.bmin[0] + (i+0.5)*resolution[0],
                                        item.bmin[1] + (j+0.5)*resolution[1],
                                        item.bmin[2] + (k+0.5)*resolution[2]))
                    indices.append((i,j,k))
    elif isinstance(item,OccupancyGrid):
        #elements are points at the centers of each voxel
        elements = []
        indices = []
        dims = item.bmax - item.bmin
        resolution = [d/s for (d,s) in zip(dims,item.values.shape)]
        for i in range(item.values.shape[0]):
            for j in range(item.values.shape[1]):
                for k in range(item.values.shape[2]):
                    elements.append((item.bmin[0] + (i+0.5)*resolution[0],
                                        item.bmin[1] + (j+0.5)*resolution[1],
                                        item.bmin[2] + (k+0.5)*resolution[2]))
                    indices.append((i,j,k))
    elif isinstance(item,Heightmap):
        #elements are points at the centers of each voxel
        elements = []
        indices = []
        dims = item.getSize()
        lower = [-d/2 for d in dims]
        resolution = [d/(s-1) for (d,s) in zip(dims,item.heights.shape)]
        for i in range(item.heights.shape[0]):
            for j in range(item.heights.shape[1]):
                elements.append((lower[0] + i*resolution[0],
                                    lower[1] + j*resolution[1],
                                    item.heights[i,j]))
                indices.append((i,j))
    else:
        raise ValueError("Geometry type {} not supported for splitting".format(type(item)))
    if callable(by):
        by = [by(e) for e in elements]
    elif not hasattr(by,'__iter__'):
        raise ValueError("by must be a list of segment indices or a callable function")
    if len(by) != len(elements):
        raise ValueError("by must have the same length as the number of elements in the geometry")
    segments = {}
    for i,seg in enumerate(by):
        if seg not in segments:
            segments[seg] = []
        segments[seg].append(i)
    if len(segments) == 1:
        return [item]  #no need to split, only one segment
    keys = sorted(segments.keys())
    result = []
    for key in keys:
        inds = segments[key]
        if isinstance(item,PointCloud):
            new_pc = PointCloud()
            new_pc.points=item.points[inds]
            if len(item.properties) > 0:
                new_pc.properties = item.properties[inds,:]
            result.append(new_pc)
        elif isinstance(item,TriangleMesh):
            new_inds = indices[inds]
            vinds = set()
            for (a,b,c) in new_inds:
                vinds.add(a)
                vinds.add(b)
                vinds.add(c)
            vinds = sorted(vinds)
            vindmap = {v:i for i,v in enumerate(vinds)}
            new_inds = np.array([[vindmap[a],vindmap[b],vindmap[c]] for (a,b,c) in new_inds],dtype=np.int32)
            new_verts = item.vertices[vinds]
            new_tm = TriangleMesh()
            new_tm.vertices = new_verts
            new_tm.indices = new_inds
            result.append(new_tm)
        elif isinstance(item,ImplicitSurface):
            new_vals = np.full_like(item.values,np.inf)
            for (i,j,k) in inds:
                new_vals[i,j,k] = item.values[i,j,k]
            new_isurf = ImplicitSurface()
            new_isurf.values = new_vals
            new_isurf.bmin = item.bmin
            new_isurf.bmax = item.bmax
            result.append(new_isurf)
        elif isinstance(item,OccupancyGrid):
            new_vals = np.full_like(item.values,0)
            for (i,j,k) in inds:
                new_vals[i,j,k] = item.values[i,j,k]
            new_og = OccupancyGrid()
            new_og.values = new_vals
            new_og.bmin = item.bmin
            new_og.bmax = item.bmax
            result.append(new_og)
        elif isinstance(item,Heightmap):
            new_heights = np.full_like(item.heights,-np.inf)
            for (i,j) in inds:
                new_heights[i,j] = item.heights[i,j]
            new_hm = Heightmap()
            new_hm.heights = new_heights
            new_hm.viewport = item.viewport
            result.append(new_hm)
        elif isinstance(item,Geometry3D):
            assert item.type() == 'Group'
            new_group = Geometry3D()
            new_group.setGroup()
            for i,j in enumerate(inds):
                new_group.setElement(i,item.getElement(j))
            result.append(new_group)
        else:
            raise ValueError("Geometry type {} not supported for splitting".format(gtype))
    return result


def vertex_colors(geom : Geometry3D, app : Appearance = None) -> np.ndarray:
    """Returns the vertex colors of a geometry as an N x 4 numpy array,
    where N is the number of vertices. 
    
    The colors are in the range [0,1] and the channels are in the order (r,g,b,a).
    
    This will act smartly for other forms of coloration.  If the geometry
    has a texture, it will extract the texture colors.  If it has face colors,
    it will average the face colors at each vertex.  If the geometry is flat
    shaded, the vertex colors will be uniform.

    If the appearance is tinted, the tint will be applied to the vertex colors.
    """
    if isinstance(geom,Geometry3D):
        gtype = geom.type()
        if geom.type() == 'PointCloud':
            return vertex_colors(geom.getPointCloud(),app)
        elif geom.type() == 'TriangleMesh':
            app = geom.getAppearance() if app is None else app
            return vertex_colors(geom.getTriangleMesh(),app)
        elif app is not None:
            vcols = app.getColors(Appearance.VERTICES)
            if len(vcols) <= 1:
                raise ValueError("Geometry type {} not supported for vertex colors".format(geom.type()))
        else:
            raise ValueError("Geometry type {} not supported for vertex colors".format(geom.type()))
    elif isinstance(geom,PointCloud):
        gtype = 'PointCloud'
        vcols = point_cloud_colors(geom,format=('r','g','b','a'))
    elif isinstance(geom,TriangleMesh):
        gtype = 'TriangleMesh'
        if app is None:
            return np.array([0.5,0.5,0.5,1.0],dtype=np.float32).reshape((1,4)).repeat(geom.vertices.shape[0],axis=0)
        if app.getTexture2D_format():
            texfmt = app.getTexture2D_format() 
            tex = app.getTexture2D_channels()
            texcoords = app.getTexcoords2D()
            #lookup texture at each vertex
            assert len(geom.vertices) == len(texcoords), \
                "Geometry has {} vertices, but {} texture coordinates".format(len(geom.vertices),len(texcoords))
            x = texcoords[:,0]* tex.shape[1]
            y = (1-texcoords[:,1])* tex.shape[0]
            x = np.clip(x,0,tex.shape[1]-1).astype(np.int32)
            y = np.clip(y,0,tex.shape[0]-1).astype(np.int32)
            vcols = np.array([tex[y[i],x[i],:] for i in range(len(geom.vertices))])
            if texfmt == 'rgb8':
                #add alpha channel
                vcols = np.hstack((vcols/255.0,np.ones((len(vcols),1))).astype(np.float32))
            else:
                raise NotImplementedError("Any colors but rgb8 not implemented yet")
        else:
            vcols = app.getColors(Appearance.VERTICES)
            fcols = app.getColors(Appearance.FACES)
            if len(vcols) == 1:
                if len(fcols) == 1:
                    vcols = app.getColors(Appearance.FACES).repeat(geom.vertices.shape[0],axis=0)
                else:
                    #extrapolate face colors to vertices
                    if len(fcols) != geom.indices.shape[0]:
                        raise ValueError("Geometry has {} faces, but {} face colors".format(geom.indices.shape[0],len(fcols)))
                    vcols = np.zeros((geom.vertices.shape[0],4),dtype=np.float32)
                    vcounts = np.zeros(geom.vertices.shape[0],dtype=np.int32)
                    for i,f in enumerate(geom.indices):
                        vcols[f[0]] += fcols[i]
                        vcols[f[1]] += fcols[i]
                        vcols[f[2]] += fcols[i]
                        vcounts[f[0]] += 1
                        vcounts[f[1]] += 1 
                        vcounts[f[2]] += 1
                    vcols /= vcounts[:,np.newaxis]
            else:
                assert len(vcols) == geom.vertices.shape[0], \
                    "Geometry has {} vertices, but {} vertex colors".format(geom.vertices.shape[0],len(vcols))
                pass
    elif app is not None:
        vcols = app.getColors(Appearance.VERTICES)
        if len(vcols) <= 1:
            raise ValueError("Geometry type {} not supported for vertex colors".format(gtype))
    else:
        raise ValueError("Geometry type {} not supported for vertex colors".format(gtype))
    if app is not None:
        tintStrength = app.getTintStrength()
        if tintStrength > 0:
            tint = app.getTintColor()
            vcols = vcols * tint + (1-tintStrength)
    return vcols



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
        verts = tm.vertices
        tris = tm.indices
        if num_samples is None:
            points = verts
            if want_elements:
                vindices = [-1] * len(verts)
                for i,t in enumerate(tris):
                    for j in t:
                        vindices[j] = i
                elements = vindices
            if want_normals:
                normals = geom.vertexNormals(geom)
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
        allPoints = geom.getPointCloud().points
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


def upper_heightmap(geom : Geometry3D, like : Heightmap, mask_value = -np.inf) -> Geometry3D:
    """Gets the upper heightmap of a geometry, aligned to the grid of an
    existing heightmap.
    
    Cells that do not contain any proejcted geometry get `mask_value` as their
    heights."""
    resgeom = Geometry3D(like)
    res = resgeom.getHeightmap()
    res.heights[:,:] = mask_value
    resgeom.merge(geom)
    return resgeom


def lower_heightmap(geom : Geometry3D, like : Heightmap, mask_value = -np.inf) -> Geometry3D:
    """Gets the lower heightmap of a geometry, aligned to the grid of an
    existing heightmap.
    
    Cells that do not contain any proejcted geometry get `mask_value` as their
    heights."""
    resgeom = Geometry3D(like)
    res = resgeom.getHeightmap()
    res.heights[:,:] = mask_value
    vp = res.viewport
    #flip the viewport z direction
    vpPose = vp.getPose()
    vp.setPose(so3.mul(vpPose[0],so3.rotation((0,1,0),-np.pi)),vpPose[1])
    res.viewport = vp
    resgeom.merge(geom)
    #flip x-dim and values of heights
    res.heights = np.flip(res.heights, axis=0)
    mask = np.isfinite(res.heights) & (res.heights != mask_value)
    res.heights[mask] = -res.heights[mask]
    #flip viewport back
    res.viewport = like.viewport
    return resgeom

def heightmap_normals(hm : Union[Heightmap,Geometry3D]) -> np.ndarray:
    """Returns the normals of a heightmap, either from a Heightmap object or
    from a Geometry3D object that has a Heightmap representation.
    
    Args:
        hm (Heightmap or Geometry3D): the heightmap

    Returns:
        np.ndarray: An M x N x 3 matrix of outward normals in the heightmap's
        local frame, but transformed by any rotation in hm.pose.  If a heightmap
        cell is non-finite, its normal is set to (0,0,0).
    """
    if isinstance(hm,Geometry3D):
        assert hm.type() == 'Heightmap',"Must provide a Heightmap to heightmap_normals"
        hm = hm.getHeightmap()
    assert isinstance(hm,Heightmap),"Must provide a Heightmap to heightmap_normals"
    valid = np.isfinite(hm.heights)
    #forward / reverse differences at borders, centered differences in the middle
    with np.errstate(invalid='ignore'):
        dx = np.empty(hm.heights.shape)
        dx[0,:] = hm.heights[1,:]-hm.heights[0,:]
        dx[1:-1,:] = (hm.heights[2:,:] - hm.heights[:-2,:])*0.5
        dx[-1,:] = hm.heights[-1,:]-hm.heights[-2,:]
        dy = np.empty(hm.heights.shape)
        dy[:,0] = hm.heights[:,1]-hm.heights[:,0]
        dy[:,1:-1] = (hm.heights[:,2:] - hm.heights[:,:-2])*0.5
        dy[:,-1] = hm.heights[:,-1]-hm.heights[:,-2]
    if not np.all(valid):
        #switch to forward/backward differences at validity borders
        valid_left = np.zeros(hm.heights.shape,dtype=bool)
        valid_right = np.zeros(hm.heights.shape,dtype=bool)
        valid_up = np.zeros(hm.heights.shape,dtype=bool)
        valid_down = np.zeros(hm.heights.shape,dtype=bool)
        valid_left[1:,:] = valid[:-1,:]
        valid_right[:-1,:] = valid[1:,:]
        valid_up[:,1:] = valid[:,:-1]
        valid_down[:,:-1] = valid[:,1:]
        dx[~valid_left & ~valid_right] = 0.0
        dy[~valid_up & ~valid_down] = 0.0
        with np.errstate(invalid='ignore'):
            right_diffs = np.concatenate((hm.heights[1:,:]-hm.heights[:-1,:],np.zeros((1,hm.heights.shape[1]))),axis=0)
            down_diffs = np.concatenate((hm.heights[:,1:]-hm.heights[:,:-1],np.zeros((hm.heights.shape[0],1))),axis=1)
        dx[~valid_left & valid_right] = right_diffs[~valid_left & valid_right]
        dx[valid_left & ~valid_right] = right_diffs[np.roll(valid_left & ~valid_right,-1,axis=0)]
        dy[~valid_up & valid_down] = down_diffs[~valid_up & valid_down]
        dy[valid_up & ~valid_down] = down_diffs[np.roll(valid_up & ~valid_down,-1,axis=1)]
    if hm.isOrthographic():
        xres,yres = hm.getResolution()
        dx *= (1.0/xres)
        dy *= (1.0/yres)
        normals = np.empty((hm.heights.shape[0],hm.heights.shape[1],3))
        normals[:,:,0] = -dx
        normals[:,:,1] = dy
        normals[:,:,2] = 1.0
        norms = np.linalg.norm(normals,axis=2)
        normals[~valid] = 0.0
        normals[valid] /= norms[valid][:,np.newaxis]
        R,t = hm.viewport.getPose()
        if R != so3.identity():
            normals = np.dot(normals,so3.ndarray(R).T)
        return normals
    else:
        raise NotImplementedError("Heightmap normals only implemented for orthographic heightmaps, not perspective heightmaps")


def convert(obj : Union[WorldModel,RobotModel,RigidObjectModel,RobotModelLink,TerrainModel,Geometry3D],
            type : str, param :float = 0):
    """Converts an object or collection's geometry to another type. 
    The object is modified in-place.
    
    You can also convert to a bounding box by setting type='aabb'
    """
    if isinstance(obj,WorldModel):
        for r in obj.robots:
            convert(r,type,param)
        for o in obj.rigidObjects:
            convert(o,type,param)
        for t in obj.terrains:
            convert(o,type,param)
    elif isinstance(obj,RobotModel):
        for l in obj.links:
            convert(l,type,param)
    elif hasattr(obj,'geometry'):
        convert(obj.geometry(),type,param)
    elif isinstance(obj,Geometry3D):
        if obj.empty(): return
        if type == 'aabb':
            from klampt import GeometricPrimitive
            obj.setCurrentTransform(*se3.identity())
            BB = obj.getBBTight()
            BBgeom = GeometricPrimitive()
            BBgeom.setAABB(BB[0],BB[1])
            obj.setGeometricPrimitive(BBgeom)
        else:
            obj2 = obj.convert(type,param)
            obj.set(obj2)
