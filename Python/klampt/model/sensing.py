"""A collection of utility functions for dealing with sensors and sensor data.

More sophisticated operations call for the use of a full-fledged sensor
package, such as Open3D or PCL.

Sensor transforms
=================

The :func:`set_sensor_xform` function can conveniently set the frame of a
sensor.

Getting images and point clouds
===============================

The :func:`camera_to_images`, :func:`camera_to_points`, and
:func:`camera_to_points_world` functions convert raw CameraSensor outputs to
Python objects that are more easily operated upon, e.g., images and point
clouds.  Use these to retrieve images as Numpy arrays.

The :func:`image_to_points` function converts a depth / color image to a
point cloud, given camera intrinsic information.

Working with cameras
====================

The :func:`camera_to_viewport` and :func:`viewport_to_camera` functions help 
with converting to and from the :class:`klampt.vis.glprogram.GLViewport` class
used in :mod:`klampt.vis`.

The :func:`camera_to_intrinsics` and :func:`intrinsics_to_camera` functions
convert between intrinsics definitions.

:func:`camera_ray`, and :func:`camera_project` convert to/from image points.

:func:`visible` determines whether a point or object is visible from a camera.
:func:`visible_fraction` determines the fraction of a point or object visible
from a camera.

:func:`projection_map_texture` maps a texture from a camera into an OpenGL
appearance.

"""

from ..robotsim import *
from ..robotsim import Geometry3D
from ..io import loader
from ..vis.glviewport import GLViewport
from ..model.typing import RigidTransform,Vector3
from ..model.collide import bb_create,bb_intersect
from . import coordinates
from typing import Union,Tuple,List,Any,Optional,Literal
import math
import sys
from ..math import vectorops,so3,se3
import time
import numpy as np

_has_scipy = False
_tried_scipy_import = False
sp = None

ImageLike = Union[np.ndarray,List[List]]

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
        warnings.warn("klampt.model.sensing.py: scipy not available.",ImportWarning)
        _has_scipy = False
    return _has_scipy


def set_sensor_xform(sensor : SensorModel, T : RigidTransform, link : Optional[RobotModelLink] = None):
    """Given a link-mounted sensor (e.g., CameraSensor or ForceSensor), sets 
    its link-local transform to T.

    Args:
        sensor (SensorModel)
        T (se3 element or coordinates.Frame): desired local coordinates of the
            sensor on its link.
        link (int or RobotModelLink, optional): if provided, the link of the
            sensor is modified. 

    Another way to set a sensor is to give a coordinates.Frame object.  This
    frame must either be associated with a RobotModelLink or its parent should
    be associated with  one.
    """
    if isinstance(T,coordinates.Frame):
        if isinstance(T._data,RobotModelLink):
            #attach it directly to the link
            return set_sensor_xform(sensor,se3.identity(),T._data)
        parent = None
        if T.parent() is None:
            parent = -1
        else:
            #assume its parent is a link?
            parent = T.parent()._data
        return set_sensor_xform(sensor,T.relativeCoordinates(),parent)
    sensor.setTransform(*T)
    if link != None:
        sensor.setLink(link)


def camera_to_images(camera : SensorModel, image_format='numpy',color_format='channels') -> Union[np.ndarray,Tuple[np.ndarray,np.ndarray]]:
    """Given a SensorModel that is a CameraSensor, returns either the RGB
    image, the depth image, or both.

    Args:
        camera (SensorModel): a sensor that is of 'CameraSensor' type
        image_format (str): governs the return type.  Can be:

            * 'numpy' (default): returns numpy arrays.  Depending on the
              value of color_format, the RGB image either has shape (h,w,3)
              and dtype uint8 or (h,w) and dtype uint32. Depth images as
              numpy arrays with shape (h,w).

        color_format (str): governs how pixels in the RGB result are packed. 
        Can be:

            * 'channels' (default): returns a 3D array with 3 channels
               corresponding to R, G, B values in the range [0,255]. 
            * 'rgb' returns a 2D array with a 32-bit integer channel, with
              R,G,B channels packed in hex format 0xrrggbb.
            * 'bgr': similar to 'rgb' but with hex order 0xbbggrr.

    Returns:
        (rgb, depth), which are either numpy arrays or another image
        format, as specified by image_format.

            * rgb: the RGB result (packed as specified by color_format)
            * depth: the depth result (floats)

    """
    assert isinstance(camera,SensorModel),"Must provide a SensorModel instance"
    assert camera.type == 'CameraSensor',"Must provide a camera sensor instance"
    #import time
    #t_1 = time.time()
    w = int(camera.getSetting('xres'))
    h = int(camera.getSetting('yres'))
    has_rgb = int(camera.getSetting('rgb'))
    has_depth = int(camera.getSetting('depth'))
    #t0 = time.time()
    #print("camera.getSettings() time",t0-t_1)
    measurements = camera.getMeasurements()
    #t1 = time.time()
    #print("camera.getMeasurements() time",t1-t0)
    rgb = None
    depth = None
    if has_rgb and len(measurements) > 0:
        if image_format == 'numpy':
            #t0 = time.time()
            argb = np.asarray(measurements[0:w*h]).reshape(h,w).astype(np.uint32)
            #t1 = time.time()
            #print("Numpy array creation time",t1-t0)
            if color_format == 'rgb':
                rgb = argb
            elif color_format == 'bgr':
                rgb = np.bitwise_or.reduce((np.left_shift(np.bitwise_and(argb,0x00000ff),16),
                                        np.bitwise_and(argb,0x000ff00)),
                                        np.right_shift(np.bitwise_and(argb,0x0ff0000), 16))
            else:
                rgb = np.zeros((h,w,3),dtype=np.uint8)
                rgb[:,:,0] = np.right_shift(np.bitwise_and(argb,0x0ff0000), 16)
                rgb[:,:,1] = np.right_shift(np.bitwise_and(argb,0x00ff00), 8)
                rgb[:,:,2] =                np.bitwise_and(argb,0x00000ff)
            #t2 = time.time()
            #print("  Conversion time",t2-t1)
        else:
            raise NotImplementedError("No other image formats besides numpy supported")
    if has_depth and len(measurements) > 0:
        start = (w*h if has_rgb else 0)
        if image_format == 'numpy':
            #t0 = time.time()
            depth = np.asarray(measurements[start:start+w*h]).reshape(h,w)
            #t1 = time.time()
            #print("Numpy array creation time",t1-t0)
        else:
            raise NotImplementedError("No other image formats besides numpy supported")
    if has_rgb and has_depth:
        return rgb,depth
    elif has_rgb:
        return rgb
    elif has_depth:
        return depth
    return None


def image_to_points(depth : ImageLike, color : ImageLike,
                    intrinsics : Optional[dict]=None, xfov : Optional[float]=None, yfov: Optional[float]=None,
                    depth_scale : Optional[float] = None, depth_range : Optional[Tuple[float,float]]=None,
                    color_format='auto',points_format='numpy',all_points=False):
    r"""Given a depth and optionally color image, returns a point cloud
    representing the depth or RGB-D scene.

    Optimal performance is obtained with ``points_format='PointCloud'`` or
    ``'Geometry3D'``, with ``all_points=True``.

    Args:
        depth (list of lists or numpy array): the w x h depth image (rectified)
            given as a numpy array of shape (h,w).
        color (list of lists or numpy array, optional): the w x h color image
            given as a numpy uint8 array of shape (h,w,3) or a numpy uint32
            array of shape (h,w) encoding RGB pixels in the format 0xrrggbb.
            It is assumed that color maps directly onto depth pixels. 
            If color = None, an uncolored point cloud will be produced. 
        intrinsics (dict, optional): Contains fields 'fx', 'fy', 'cx', 'cy' giving
            the intrisincs parameters of the camera.
        xfov (float, optional): horizontal field of view, in radians.  Related to the
            intrinsics fx via :math:`fx = w/(2 \tan(xfov/2))`, i.e.,
            :math:`xfov = 2*\arctan(w/(2*fx))`.
        yfov (float, optional): vertical field of view, in radians.  If not
            given, square pixels are assumed.  Related to the intrinsics
            :math:`fy = h/(2 \tan(yfov/2))`, i.e.,
            :math:`yfov = 2*\arctan(h/(2*fy))`.
        depth_scale (float, optional): a scaling from depth image values to
            absolute depth values.
        depth_range (pair of floats, optional): if given, only points within 
            this depth range (non-inclusive) will be extracted.  If
            all_points=False, points that fail the range test will be stripped
            from the output.  E.g., (0.5,8.0) only extracts points with
            z > 0.5 and z < 8 units.
        color_format (str): governs how pixels in the RGB result are packed. 
            Can be:

            * 'auto' (default): if it's a 3D array, it assumes elements are in
              'channels' format, otherwise it assumes 'rgb'.
            * 'channels': a 3D array with 3 channels corresponding to R, G,
              B values in the range [0,255] if uint8 type, otherwise in the
              range [0,1]. 
            * 'rgb' a 2D array with a 32-bit integer channel, with
              R,G,B channels packed in hex format 0xrrggbb.

        points_format (str, optional): configures the format of the return
            value. Can be:

            * 'numpy' (default): either an Nx3, Nx4, or Nx6 numpy array,
              depending on whether color is requested (and its format).  
            * 'Geometry3D': a Klampt Geometry3D point cloud object
            * 'PointCloud': a Klampt PointCloud object
            * 'TriangleMesh': a Klampt TriangleMesh object showing a regular 
              grid encoded with the depth image.
            * 'Heightmap': a Klampt Heightmap object

        all_points (bool, optional): configures whether bad points should be
            stripped out.  If False (default), this strips out all pixels that
            don't have a good depth reading (i.e., the camera sensor's maximum
            reading.)  If True, these pixels are all set to (0,0,0).

    Returns:
        numpy ndarray or Geometry3D: the point cloud.  Represented as being 
        local to the standard camera frame with +x to the right, +y down, +z
        forward.

    """
    depth = np.asarray(depth)
    assert len(depth.shape)==2
    h,w = depth.shape
    if color is not None:
        color = np.asarray(color)
        if h != color.shape[0] or w != color.shape[1]:
            raise ValueError("color and depth need to have same dimensions")
        if color_format == 'auto':
            if len(color.shape)==3:
                color_format = 'channels'
            else:
                assert len(color.shape)==2
                color_format = 'rgb'
    else:
        color_format = None

    if points_format == 'Heightmap':
        res = Heightmap()
        vp = Viewport()
        vp.resize(w,h)
        if intrinsics is None:
            vp.setFOV(xfov,yfov)
        else:
            vp.fx = intrinsics['fx']
            vp.fy = intrinsics['fy']
            vp.cx = intrinsics['cx']
            vp.cy = intrinsics['cy']
        res.setViewport(vp)
        res.setHeightImage(depth,depth_scale)
        if color is not None:
            if color_format == 'channels':
                res.setColorImage(color)
            else:
                res.setColorImage(color)
        return res

    if (points_format == 'PointCloud' or points_format == 'Geometry3D') and all_points:
        #shortcut, about 2x faster than going through Numpy
        res = PointCloud()
        if intrinsics is None:
            if xfov is None:
                raise ValueError("Either intrinsics or xfov must be provided")
            fx = 0.5*w/math.tan(xfov*0.5)
            if yfov is None:
                fy = fx
            else:
                fy = 0.5*h/math.tan(yfov*0.5)
            cx = 0.5*w
            cy = 0.5*h
        else:
            fx,fy,cx,cy = intrinsics['fx'],intrinsics['fy'],intrinsics['cx'],intrinsics['cy']
        if depth_scale is None:
            depth_scale = 1.0
        if color_format is None:
            res.setDepthImage([fx,fy,cx,cy],depth,depth_scale)
        else:
            res.setRGBDImages([fx,fy,cx,cy],color,depth,depth_scale)

        if points_format == 'PointCloud':
            return res
        else:
            g = Geometry3D()
            g.setPointCloud(res)
            return g

    if depth_scale is not None:
        depth *= depth_scale

    if points_format == 'TriangleMesh':
        if not all_points:
            raise NotImplementedError("TODO: TriangleMesh result but with missing data")

    if intrinsics is None:
        xshift = -w*0.5
        yshift = -h*0.5
        xscale = math.tan(xfov*0.5)/(w*0.5)
        if yfov is not None:
            yscale = math.tan(yfov*0.5)/(h*0.5)
        else:
            yscale = xscale #square pixels are assumed
    else:
        xshift = -intrinsics['cx']
        yshift = -intrinsics['cy']
        xscale = 1.0/fx
        yscale = 1.0/fy
    xs = [(j+xshift)*xscale for j in range(w)]
    ys = [(i+yshift)*yscale for i in range(h)]
    if color_format == 'channels' and color.dtype == np.uint8:
        #scale to range [0,1]
        color = color*(1.0/255.0)

    xgrid = np.repeat(np.array(xs).reshape((1,w)),h,0)
    ygrid = np.repeat(np.array(ys).reshape((h,1)),w,1)
    assert xgrid.shape == (h,w)
    assert ygrid.shape == (h,w)
    pts = np.dstack((np.multiply(xgrid,depth),np.multiply(ygrid,depth),depth))
    assert pts.shape == (h,w,3)
    if color_format is not None:
        if len(color.shape) == 2:
            color = color.reshape(color.shape[0],color.shape[1],1)

    #now have a nice array containing all points, shaped h x w x (3+c)
    #extract out the valid points from this array
    if all_points:
        pts = pts.reshape(w*h,pts.shape[2])
        if color_format is not None:
            color = color.reshape(w*h,color.shape[2])
    else:
        if depth_range is not None:
            valid = np.logical_and((depth > depth_range[0]),(depth < depth_range[1]))
            if all_points and points_format != 'TriangleMesh':
                depth[~valid] = 0
            valid = (depth > 0)
        else:
            valid = (depth > 0)
        pts = pts[valid]
        if color is not None:
            color = color[valid]

    if points_format == 'numpy':
        if color_format is not None:
            pts = np.concatenate((pts,color),1)
        return pts
    elif points_format == 'PointCloud' or points_format == 'Geometry3D':
        res = PointCloud()
        if all_points:
            res.setSetting('width',str(w))
            res.setSetting('height',str(h))
        res.setPoints(pts)
        if color_format == 'rgb':
            res.addProperty('rgb')
            assert color.shape == (len(pts),1)
            res.properties = color
        elif color_format == 'channels':
            res.addProperty('r')
            res.addProperty('g')
            res.addProperty('b')
            assert color.shape == (len(pts),3)
            res.properties = color
        if points_format == 'PointCloud':
            return res
        else:
            g = Geometry3D()
            g.setPointCloud(res)
            return g
    elif points_format == 'TriangleMesh':
        res = TriangleMesh()
        res.setVertices(pts)
        indices = np.empty(((w-1)*(h-1)*2,3),dtype=np.int32)
        template = np.array([[0,1,w+1],[w+1,w,0]],dtype=np.int32)
        rowtemplate = np.vstack([template+j for j in range(w-1)])
        k = 0
        for i in range(h-1):
            indices[k:k+(w-1)*2,:] = rowtemplate + (i*w)
            k += (w-1)*2
        res.setIndices(indices)
        if color is not None:
            app = Appearance()
            if color_format == 'channels':
                app.setColors(Appearance.VERTICES,np.asarray(color).T)
            else:
                raise NotImplementedError("TODO: convert colors to per-vertex colors")
            return (res,app)
        return res
    else:
        raise ValueError("Invalid points_format, must be either numpy, PointCloud, or Geometry3D")


def camera_to_points(camera : SensorModel,
                     points_format='numpy',
                     all_points=False,
                     color_format='channels') -> Union[np.ndarray,PointCloud,TriangleMesh,Geometry3D]:
    """Given a SensorModel that is a CameraSensor, returns a point cloud
    associated with the current measurements.

    Points are triangulated with respect to the camera's intrinsic coordinates,
    and are returned in the camera local frame (+z backward, +x toward the
    right, +y toward up). 

    The arguments 

    Args:
        points_format (str, optional): configures the format of the return
            value. Can be:

            * 'numpy' (default): either an Nx3, Nx4, or Nx6 numpy array,
              depending on whether color is requested (and its format).  
            * 'PointCloud': a Klampt PointCloud object
            * 'Geometry3D': a Klampt Geometry3D point cloud object
            * 'TriangleMesh': a Klampt TriangleMesh object showing a regular 
              grid encoded with the depth image.

        all_points (bool, optional): configures whether bad points should be
            stripped out.  If False (default), this strips out all pixels that
            don't have a good depth reading (i.e., the camera sensor's maximum
            reading.)  If True, these pixels are all set to (0,0,0).

        color_format (str):  If the sensor has an RGB component, then color
            channels may be produced.  This value configures the output format,
            and can take on the values:

            * 'channels': produces individual R,G,B channels in the range
              [0,1]. (note this is different from the interpretation of
              camera_to_images)
            * 'rgb': produces a single 32-bit integer channel packing the 8-bit
              color channels together in the format 0xrrggbb.
            * None: no color is produced.

    Returns:
        The point cloud in the requested format.
    """
    assert isinstance(camera,SensorModel),"Must provide a SensorModel instance"
    assert camera.type == 'CameraSensor',"Must provide a camera sensor instance"
    assert int(camera.getSetting('depth'))==1,"Camera sensor must have a depth channel"

    images = camera_to_images(camera,'numpy',color_format)
    assert images is not None

    rgb,depth = None,None
    if int(camera.getSetting('rgb'))==0:
        depth = images
        color_format = None
    else:
        rgb,depth = images

    w = int(camera.getSetting('xres'))
    h = int(camera.getSetting('yres'))
    fx = float(camera.getSetting('fx'))
    fy = float(camera.getSetting('fy'))
    cx = float(camera.getSetting('cx'))
    cy = float(camera.getSetting('cy'))

    if points_format == 'Heightmap':
        res = Heightmap()
        vp = Viewport()
        vp.resize(w,h)
        vp.fx = fx
        vp.fy = fy
        vp.cx = cx
        vp.cy = cy
        res.setViewport(vp)
        res.setHeightImage(depth)
        if rgb is not None:
            res.setColorImage(rgb)
        return res

    if (points_format == 'PointCloud' or points_format == 'Geometry3D') and (color_format is None or color_format == 'rgb') and all_points:
        #shortcut, about 2x faster than going through Numpy
        res = PointCloud()
        if color_format is None:
            res.setDepthImage([fx,fy,cx,cy],depth)
        else:
            res.setRGBDImages([fx,fy,cx,cy],rgb,depth)
        if points_format == 'PointCloud':
            return res
        else:
            g = Geometry3D()
            g.setPointCloud(res)
            return g

    if points_format == 'TriangleMesh':
        if not all_points:
            raise NotImplementedError("TODO: TriangleMesh result but with missing data")

    zmin = float(camera.getSetting('zmin'))
    zmax = float(camera.getSetting('zmax'))
    xshift = -cx
    yshift = -cy
    xscale = 1.0/fx
    yscale = 1.0/fy
    xs = [(j+xshift)*xscale for j in range(w)]
    ys = [(i+yshift)*yscale for i in range(h)]

    if all_points and points_format != 'TriangleMesh':
        depth[depth >= zmax] = 0
    if color_format == 'channels':
        #scale to range [0,1]
        rgb = rgb*(1.0/255.0)
    xgrid = np.repeat(np.array(xs).reshape((1,w)),h,0)
    ygrid = np.repeat(np.array(ys).reshape((h,1)),w,1)
    assert xgrid.shape == (h,w)
    assert ygrid.shape == (h,w)
    pts = np.dstack((np.multiply(xgrid,depth),np.multiply(ygrid,depth),depth))
    assert pts.shape == (h,w,3)
    if color_format is not None:
        if len(rgb.shape) == 2:
            rgb = rgb.reshape(rgb.shape[0],rgb.shape[1],1)
        pts = np.concatenate((pts,rgb),2)
    #now have a nice array containing all points, shaped h x w x (3+c)
    #extract out the valid points from this array
    if all_points:
        pts = pts.reshape(w*h,pts.shape[2])
    else:
        pts = pts[depth < zmax]

    if points_format == 'numpy':
        return pts
    elif points_format == 'PointCloud' or points_format == 'Geometry3D':
        res = PointCloud()
        if all_points:
            res.setSetting('width',str(w))
            res.setSetting('height',str(h))
        res.setPoints(pts[:,0:3])
        if color_format == 'rgb':
            res.addProperty('rgb')
            res.properties = pts[:,3:4]
        elif color_format == 'channels':
            res.addProperty('r')
            res.addProperty('g')
            res.addProperty('b')
            res.properties = pts[:,3:6]
        elif color_format == 'bgr':
            raise ValueError("bgr color format not supported with PointCloud output")
        if points_format == 'PointCloud':
            return res
        else:
            g = Geometry3D()
            g.setPointCloud(res)
            return g
    elif points_format == 'TriangleMesh':
        res = TriangleMesh()
        res.setVertices(pts[:,:3])
        indices = np.empty(((w-1)*(h-1)*2,3),dtype=np.int32)
        template = np.array([[0,w+1,1],[w+1,0,w]],dtype=np.int32)
        rowtemplate = np.vstack([template+j for j in range(w-1)])
        k = 0
        for i in range(h-1):
            indices[k:k+(w-1)*2,:] = rowtemplate + (i*w)
            k += (w-1)*2
        res.setIndices(indices)
        if color_format is not None:
            app = Appearance()
            if color_format == 'channels':
                app.setColors(Appearance.VERTICES,pts[:,3:].astype(np.float32))
            else:
                raise NotImplementedError("TODO: convert colors to per-vertex colors")
            return (res,app)
        return res
    else:
        raise ValueError("Invalid points_format "+points_format)


def camera_to_points_world(camera : SensorModel,
                           points_format='numpy',
                           color_format='channels') -> Union[np.ndarray,PointCloud,Geometry3D]:
    """Same as :meth:`camera_to_points`, but converts to the world coordinate
    system.
    
    If points_format='Geometry3D' then the Geometry3D data will be in local
    coordinates but the object will have its current transform set to the
    camera's world transform.  

    Points that have no reading are stripped out.
    """
    assert isinstance(camera,SensorModel),"Must provide a SensorModel instance"
    assert camera.type == 'CameraSensor',"Must provide a camera sensor instance"
    Tworld = camera.getTransformWorld()
    #now get the points
    pts = camera_to_points(camera,points_format,all_points=False,color_format=color_format)
    if points_format == 'numpy':
        Rw = np.array(so3.matrix(Tworld[0]))
        tw = np.array(Tworld[1])
        pts[:,0:3] = np.dot(pts[:,0:3],Rw.T) + tw
        return pts
    elif points_format == 'PointCloud' or points_format == 'TriangleMesh':
        pts.transform(*Tworld)
    elif points_format == 'Geometry3D':
        pts.setCurrentTransform(*Tworld)
    else:
        raise ValueError("Invalid format "+str(points_format))
    return pts


def camera_to_viewport(camera : SensorModel, frame : Literal['world','link','sensor'] = 'world') -> GLViewport:
    """Returns a Viewport / GLViewport instance corresponding to the camera's
    view. 

    See :mod:`klampt.vis.glprogram` and :mod:`klampt.vis.visualization` for
    information about how to use the object with the visualization, e.g.
    ``vis.setViewport(vp)``.

    Args:
        camera (SensorModel): the camera instance.
        frame (str): whether to return the viewport in the world frame
            ('world'), the camera's link frame ('link'), or the camera's
            local frame with the viewport at the origin ('sensor').

    Returns:
        A Viewport / GLViewport matching the camera's viewport.
    """
    assert isinstance(camera,SensorModel),"Must provide a SensorModel instance"
    assert camera.type == 'CameraSensor',"Must provide a camera sensor instance"
    if frame == 'world':
        xform = camera.getTransformWorld()
    elif frame == 'link':
        xform = camera.getTransform()
    else:
        xform = se3.identity()
    w = int(camera.getSetting('xres'))
    h = int(camera.getSetting('yres'))
    fx = float(camera.getSetting('fx'))
    fy = float(camera.getSetting('fy'))
    cx = float(camera.getSetting('cx'))
    cy = float(camera.getSetting('cy'))
    zmin = float(camera.getSetting('zmin'))
    zmax = float(camera.getSetting('zmax'))
    view = GLViewport()
    view.w, view.h = w,h
    view.fx = fx
    view.fy = fy
    view.cx = cx
    view.cy = cy
    view.controller.dist = 1.0
    view.controller.tgt = se3.apply(xform,[0,0,view.controller.dist])
    #axes corresponding to right, down, fwd in camera view
    view.controller.set_orientation(xform[0],['x','y','z'])
    view.n = zmin
    view.f = zmax
    return view


def viewport_to_camera(viewport : Viewport, camera : SensorModel, frame : Literal['world','link',None] = 'world'):
    """Fills in a simulated camera's settings to match a GLViewport specifying
    the camera's view. 

    Args:
        viewport (GLViewport): the viewport to match
        camera (SensorModel): the viewport will be output to this sensor
        frame (str or None): which frame the viewport is located in.  If 'world',
            the camera pose will be set to match the xform of the viewport.  If
            'link', the camera pose will be set to match the xform of the viewport
            in the camera's link frame. If None, the camera pose will not be set.

    """
    assert isinstance(viewport,Viewport)
    assert isinstance(camera,SensorModel),"Must provide a SensorModel instance"
    assert camera.type == 'CameraSensor',"Must provide a camera sensor instance"
    xform = viewport.get_transform()
    link = int(camera.getSetting('link'))
    if frame == 'world':
        if link < 0:
            camera.setTransform(*xform)
        else:
            rlink = camera.robot().link(link)
            camera.setTransform(*se3.mul(se3.inv(rlink.getTransform()),xform))
    elif frame == 'link':
        camera.setTransform(*xform)
    camera.setSetting('xres',str(viewport.w))
    camera.setSetting('yres',str(viewport.h))
    camera.setSetting('fx',str(viewport.fx))
    camera.setSetting('fy',str(viewport.fy))
    camera.setSetting('cx',str(viewport.cx))
    camera.setSetting('cy',str(viewport.cy))
    camera.setSetting('zmin',str(viewport.n))
    camera.setSetting('zmax',str(viewport.f))
    return camera


def camera_to_intrinsics(camera : SensorModel, format='opencv', fn=None):
    """Returns the camera's intrinsics and/or saves them to a file under the
    given format.

    Args:
        camera (SensorModel): the camera instance.
        format (str): either 'opencv', 'numpy', 'ros', or 'json' describing the 
            desired type
        fn (str, optional): the file to save to (must be .json, .xml, or .yml).
        
    Returns:
        If format='opencv', the (projection, distortion) matrix is
        returned.
        
        If format='numpy', just the projection matrix is returned. 

        If format=='json', a dict of the fx, fy, cx, cy values is returned
        
    """
    assert isinstance(camera,SensorModel),"Must provide a SensorModel instance"
    assert camera.type == 'CameraSensor',"Must provide a camera sensor instance"
    w = int(camera.getSetting('xres'))
    h = int(camera.getSetting('yres'))
    fx = float(camera.getSetting('fx'))
    fy = float(camera.getSetting('fy'))
    cx = float(camera.getSetting('cx'))
    cy = float(camera.getSetting('cy'))
    if format == 'json':
        jsonobj = {'fx':fx,'fy':fy,'cx':cy,'model':None,'coeffs':[]}
        if fn is not None:
            import json
            with open(fn,'w') as f:
                json.dump(jsonobj,f)
        return jsonobj
    elif format == 'numpy':
        res = np.zeros((3,3))
        res[0,0] = fx
        res[1,1] = fy
        res[0,2] = cx
        res[1,2] = cy
        res[2,2] = 1
        if fn is not None:
            np.save(fn,res)
        return res
    elif format == 'ros':
        from ..io import ros
        return ros.to_CameraInfo(camera)
    elif format == 'opencv':
        res = np.zeros((3,3))
        dist = np.zeros(5)
        res[0,0] = fx
        res[1,1] = fy
        res[0,2] = cx
        res[1,2] = cy
        res[2,2] = 1
        if fn is not None:
            if fn.endswith('yml'):
                #write as YAML
                with open(fn,'w') as f:
                    w.write("""%YAML:1.0
image_width: {}
image_height: {}
camera_matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ {}, 0., {}, 0.,
       {}, {}, 0., 0., 1. ]
distortion_coefficients: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [ 0 0 0 0 0 ]""".format(w,h,fx,cx,fy,cy))
            else:
                #write as XML
                with open(fn,'w') as f:
                    w.write("""<opencv_storage>
<cameraResolution>
{} {}</cameraResolution>
<cameraMatrix type_id="opencv-matrix">
  <rows>3</rows>
  <cols>3</cols>
  <dt>d</dt>
  <data>
    {} 0 {} 0 {} {} 0 0 1</data></cameraMatrix>
<dist_coeffs type_id="opencv-matrix">
  <rows>1</rows>
  <cols>5</cols>
  <dt>d</dt>
  <data>
    0 0 0. 0. 0</data></dist_coeffs>""".format(w,h,fx,cx,fy,cy))
            
        return res,dist
    else:
        raise ValueError("Invalid format, only opencv, numpy, ros, and json are supported")


def intrinsics_to_camera(data, camera : SensorModel, format='opencv'):
    """Fills in a simulated camera's settings to match given intrinsics.  Note:
    all distortions are dropped.

    Args:
        data: the filename or data to set. Interpretation varies depending on
            the format.
        camera (SensorModel): the viewport will be output to this sensor
        format (str): either 'opencv', 'numpy', 'ros', or 'json'

    """
    assert isinstance(camera,SensorModel),"Must provide a SensorModel instance"
    assert camera.type == 'CameraSensor',"Must provide a camera sensor instance"
    if isinstance(data,str):
        #it's a filename
        with open(data,'r') as f:
            if format == 'opencv':
                raise NotImplementedError("TODO: read from OpenCV calibrations")
            elif format == 'numpy':
                return intrinsics_to_camera(np.load(data),camera,format)
            elif format == 'json':
                import json
                with open(data,'r') as f:
                    jsonobj = json.load(f)
                    return intrinsics_to_camera(jsonobj,camera,format)
            else:
                raise ValueError("Invalid format, only opencv, numpy, and json are supported to load from disk")
    if format == 'ros':
        from ..io import ros
        return ros.from_CameraInfo(data,camera)
    elif format == 'numpy':
        if data.shape != (3,3):
            raise ValueError("data must be a 3x3 numpy matrix")
        fx = data[0,0]
        fy = data[1,1]
        cx = data[0,2]
        cy = data[1,2]
    elif format == 'opencv':
        proj,dist = data
        if proj.shape != (3,3):
            raise ValueError("projection matrix must be a 3x3 numpy matrix")
        fx = proj[0,0]
        fy = proj[1,1]
        cx = proj[0,2]
        cy = proj[1,2]
    elif format == 'json':
        fx = data['fx']
        fy = data['fy']
        cx = data['cx']
        cy = data['cy']
    else:
        raise ValueError("Invalid format, only opencv, numpy, ros, and json are supported")
    w = int(cx*2)
    h = int(cy*2)
    camera.setSetting('xres',str(w))
    camera.setSetting('yres',str(h))
    camera.setSetting('fx',str(fx))
    camera.setSetting('fy',str(fy))
    camera.setSetting('cx',str(cx))
    camera.setSetting('cy',str(cy))
    return camera



def camera_ray(camera : SensorModel, x : float, y : float) -> Tuple[Vector3,Vector3]:
    """Returns the (source,direction) of a ray emanating from the
    SensorModel at pixel coordinates (x,y).

    If you are doing this multiple times, it's faster to convert the camera
    to GLViewport and use GLViewport.click_ray.

    Arguments:
        camera (SensorModel): the camera
        x (int/float): x pixel coordinates
        y (int/float): y pixel coordinates

    Returns:
        A pair (source,direction) giving the world-space ray source/direction.
    """
    return camera_to_viewport(camera).click_ray(x,y)


def camera_project(camera : SensorModel, pt : Vector3,clip=True) -> Vector3:
    """Given a point in world space, returns the (x,y,z) coordinates of the
    projected pixel.  z is given in absolute coordinates, while x,y are given
    in pixel values.

    If clip=True and the point is out of the viewing volume, then None is
    returned. Otherwise, if the point is exactly at the focal plane then the
    middle of the viewport is returned.

    If you are doing this multiple times, it's faster to convert the camera
    to GLViewport and use GLViewport.project.

    Arguments:
        camera (SensorModel): the camera
        pt (3-vector): world coordinates of point
        clip (bool, optional): if true, None will be returned if the point is
            outside of the viewing volume.

    Returns:
        (x,y,z), where x,y are pixel value of image, z is depth.
    """
    return camera_to_viewport(camera).project(pt,clip)


def visible(camera : Union[SensorModel,GLViewport], object, full=True) -> bool:
    """Tests whether the given object is visible in a SensorModel or a
    GLViewport. 

    If you are doing this multiple times, it's marginally faster to first 
    convert camera to GLViewport.

    Args:
        camera (SensorModel or GLViewport): the camera.
        object: a 3-vector, a (center,radius) pair indicating a sphere, an
            axis-aligned bounding box (bmin,bmax), a Geometry3D, or an object
            that has a geometry() method, e.g., RigidObjectModel, RobotModelLink.
        full (bool, optional): if True, the entire object must be in the
            viewing frustum for it to be considered visible.  If False, any
            part of the object can be in the viewing frustum.
    """
    if isinstance(camera,SensorModel):
        camera = camera_to_viewport(camera)
    if hasattr(object,'geometry'):
        return visible(camera,object.geometry(),full)
    if hasattr(object,'__iter__'):
        if not hasattr(object[0],'__iter__'):
            #vector
            if len(object) != 3:
                raise ValueError("Object must be a 3-vector")
            return camera.project(object) != None
        elif hasattr(object[1],'__iter__'):
            if len(object[0]) != 3 or len(object[1]) != 3:
                raise ValueError("Object must be a bounding box")
            bmin,bmax = object
            if not full:
                #test whether center is in bmin,bmax
                center = vectorops.interpolate(bmin,bmax,0.5)
                cproj = camera.project(center)
                if cproj is not None:
                    return True
                if all(a <= v <= b for (a,b,v) in zip(bmin,bmax,camera.getTransform()[1])):
                    return True

            points = [camera.project(bmin,full),camera.project(bmax,full)]
            pt = [bmin[0],bmin[1],bmax[2]]
            points.append(camera.project(pt,full))
            pt = [bmin[0],bmax[1],bmax[2]]
            points.append(camera.project(pt,full))
            pt = [bmin[0],bmax[1],bmin[2]]
            points.append(camera.project(pt,full))
            pt = [bmax[0],bmin[1],bmin[2]]
            points.append(camera.project(pt,full))
            pt = [bmax[0],bmin[1],bmax[2]]
            points.append(camera.project(pt,full))
            pt = [bmax[0],bmax[1],bmin[2]]
            points.append(camera.project(pt,full))
            if any(p is None for p in points):
                return False
            if full:
                return True
            if min(p[2] for p in points) > camera.f:
                return False
            if max(p[2] for p in points) < camera.n:
                return False
            points = [p for p in points if p[2] > 0]
            for p in points:
                if 0 <= p[0] <= camera.w and 0 <= p[1] <= camera.h:
                    return True
            #TODO: intersection of projected polygon
            return False
        else:
            #sphere
            if len(object[0]) != 3:
                raise ValueError("Object must be a sphere")
            c,r = object
            if full:
                cproj = camera.project(c,True)
                if cproj is None: return False
                rproj = camera.w/cproj[2]*r
                if cproj[2] - r < camera.n or cproj[2] + r > camera.f: return False
                return 0 <= cproj[0] - rproj and cproj[0] + rproj <= camera.w and 0 <= cproj[1] - rproj and cproj[1] + rproj <= camera.h
            else:
                cproj = camera.project(c,False)
                if cproj is None:
                    dist = r - vectorops.distance(camera.getTransform()[1],c) 
                    if dist >= camera.n:
                        return True
                    return False
                if 0 <= cproj[0] <= camera.w and 0 <= cproj[1] <= camera.h:
                    if cproj[2] + r > camera.n and cproj[2] - r < camera.f:
                        return True
                    return False
                rproj = camera.w/cproj[2]*r
                xclosest = max(min(cproj[0],camera.w),0)
                yclosest = max(min(cproj[1],camera.h),0)
                zclosest = max(min(cproj[2],camera.f),camera.n)
                return vectorops.distance((xclosest,yclosest),cproj[0:2]) <= rproj
    if not isinstance(object,Geometry3D):
        raise ValueError("Object must be a point, sphere, bounding box, or Geometry3D")
    return visible(camera,object.getBB(),full)


def occluded(origin : Vector3, point : Vector3, occluders : Union[WorldModel,List], tol=1e-3) -> bool:
    """Returns whether the ray from origin to point is occluded by any of the
    objects in occluders.
    
    .. versionadded:: 0.9.2
    """
    if not occluders:
        return False
    if isinstance(occluders, WorldModel):
        entities = []
        for i in range(occluders.numRobots()):
            for j in range(occluders.robot(i).numLinks()):
                entities.append(occluders.robot(i).link(j))
        for i in range(occluders.numRigidObjects()):
            entities.append(occluders.rigidObject(i))
        if i in range(occluders.numTerrains()):
            entities.append(occluders.terrain(i))
        return occluded(origin,point,entities)
    d = vectorops.sub(point,origin)
    dp = vectorops.dot(d,d)
    sbmin,sbmax = bb_create(point,origin)
    for o in occluders:
        if hasattr(o,'geometry'):
            o = o.geometry()
        if not isinstance(o,Geometry3D):
            raise ValueError("Occluders must be a list of Geometry3D objects or entities")
        #quick BB rejection test
        bmin,bmax = o.getBB()
        if not bb_intersect((sbmin,sbmax),(bmin,bmax)):
            continue
        hit,pt = o.rayCast(origin,point)
        if hit and vectorops.dot(vectorops.sub(pt,origin),d) < dp-tol:
            return True
    return False


def sample_visible_surface(origin : Vector3, object : Geometry3D, num_samples : int) -> List[Vector3]:
    """Samples the visible surface of an object.

    .. versionadded:: 0.9.2
    """
    from . import geometry
    pn = geometry.sample_surface(object,num_samples*2.5, want_normals=True)
    pts = pn[:,:3]
    normals = pn[:,:6]
    forward_pts = []
    for p,n in zip(pts,normals):
        if vectorops.dot(vectorops.sub(p,origin),n) < 0:
            #back facing
            pass
        else:
            forward_pts.append(p)
    if num_samples < len(forward_pts):
        return forward_pts[:num_samples]
    return forward_pts


def visible_fraction(camera : Union[SensorModel,GLViewport], object, num_samples=100,
                     occluders : Optional[Union[WorldModel,List]] = None,
                     self_occlusion = False) -> float:
    """Estimates how much of the given object would be visible in a
    SensorModel or a GLViewport, if the sensor were to have infinite field
    of view.

    Returns a fraction from 0 to 1.

    If you are doing this multiple times, it's marginally faster to first 
    convert camera to GLViewport.

    .. versionadded:: 0.9.2

    Args:
        camera (SensorModel or GLViewport): the camera.
        object: a 3-vector, a (center,radius) pair indicating a sphere, an
            axis-aligned bounding box (bmin,bmax), a Geometry3D, or an object
            that has a geometry() method, e.g., RigidObjectModel, RobotModelLink.
        full (bool, optional): if True, the entire object must be in the
            viewing frustum for it to be considered visible.  If False, any
            part of the object can be in the viewing frustum.
        occluders (list of world objects or Geometry3D, optional): if given,
            these objects will be ray-casted to determine if they occlude the
            object.
        self_occlusion (bool, optional): if True, the object will be tested for
            self-occlusion.  This is only meaningful if the object is non-convex.
    """
    if isinstance(camera,SensorModel):
        camera = camera_to_viewport(camera)
    R,origin = camera.get_transform()
    if isinstance(occluders,WorldModel):
        entities = []
        for i in range(occluders.numRobots()):
            for j in range(occluders.robot(i).numLinks()):
                entities.append(occluders.robot(i).link(j))
        for i in range(occluders.numRigidObjects()):
            entities.append(occluders.rigidObject(i))
        if i in range(occluders.numTerrains()):
            entities.append(occluders.terrain(i))
        occluders = entities

    if hasattr(object,'geometry'):
        return visible_fraction(camera,object.geometry(),num_samples,occluders,self_occlusion)
    if hasattr(object,'__iter__'):
        if not hasattr(object[0],'__iter__'):
            #vector
            if len(object) != 3:
                raise ValueError("Object must be a 3-vector")
            pts = [object]
        elif hasattr(object[1],'__iter__'):
            if len(object[0]) != 3 or len(object[1]) != 3:
                raise ValueError("Object must be a bounding box")
            bmin,bmax = object
            prim = GeometricPrimitive()
            prim.setAABB(bmin,bmax)
            pts = sample_visible_surface(origin, Geometry3D(prim), num_samples)
        else:
            #sphere
            if len(object[0]) != 3:
                raise ValueError("Object must be a sphere")
            prim = GeometricPrimitive()
            prim.setSphere(object[0],object[1])
            pts = sample_visible_surface(origin, Geometry3D(prim), num_samples)
    elif isinstance(object,Geometry3D):
        if self_occlusion and object.type() not in ['TriangleMesh','VolumeGrid']:
            self_occlusion = False
        pts = sample_visible_surface(origin, object, num_samples)
    else:
        raise ValueError("Object must be a point, sphere, bounding box, or Geometry3D")
    #test whether these are out of frame or occluded rays
    nocc = 0
    for pt in pts:
        if camera.project(pt) == None or occluded(origin,pt,occluders):
            nocc += 1
        if self_occlusion and occluded(origin,pt,[object]):  #do we want a different tolerance for self-occlusion?
            nocc += 1
    return 1.0 - nocc/len(pts)




def laser_to_points(sensor : SensorModel, frame : Literal['world','link','sensor'] = 'world', scan=None):
    """Converts laser readings to a PointCloud.

    Args:
        sensor (SensorModel): the sensor
        frame (str): whether to return the points in the world frame
            ('world'), the camera's link frame ('link'), or the camera's
            local frame with the laser at the origin ('sensor').
        scan (list of float, optional): the results from
            sensor.getMeasurements(). Otherwise, sensor.getMeasurements()
            is called
    
    Returns:
        PointCloud: the point cloud.  Given in world coordinates if robot!=None
        or in link-local coordinates if robot=None.
    
    Note: only works with planar laser scanners, for now.
    """
    if frame == 'world':
        xform = sensor.getTransformWorld()
    elif frame == 'link':
        xform = sensor.getTransform()
    else:
        xform = se3.identity()

    if scan is None:
        scan = sensor.getMeasurements()

    angle_range = float(sensor.getSetting("xSweepMagnitude"))

    from klampt.math import so2
    points = []
    for i in range(len(scan)):
        u = float(i)/float(len(scan)+1)
        # If period = 0, measurement sweeps over range of [-magnitude,magnitude]
        angle = so2.interpolate(-angle_range, angle_range, u)
        pt = [math.cos(angle)*scan[i], math.sin(angle)*scan[i],0]
        points += pt

    rv = PointCloud()
    rv.setPoints(len(scan),points)
    rv.transform(*xform)
    return rv


def projection_map_texture(vp : Union[SensorModel,GLViewport],
                           app : Appearance):
    """Calculates texture coordinate generator for a given appearance to
    project an image texture onto some geometry.

    The appearance should already have been set up for the given object.

    To complete the projection mapping, call app.setTexture2D(format,image).
    """
    if isinstance(vp,SensorModel):
        vp = camera_to_viewport(vp)
    texgen = np.zeros((4,4))
    T = vp.get_transform('openGL')
    xdir = so3.apply(T[0],[1,0,0])
    ydir = so3.apply(T[0],[0,1,0])
    zdir = so3.apply(T[0],[0,0,1])
    raise NotImplementedError("TODO: update this for new Viewport data structure")
    if not vp.perspective:
        vscale = vp.h/vp.w
        texgen[0,0:3] = vectorops.mul(xdir,0.5/vp.fov)
        texgen[1,0:3] = vectorops.mul(ydir,0.5/(vp.fov*vscale))
        texgen[0,3] = 0.5 - vectorops.dot(xdir,T[1])*0.5/vp.fov
        texgen[1,3] = 0.5 - vectorops.dot(ydir,T[1])*0.5/(vp.fov*vscale)
        texgen[3,3] = 1 
    else:
        fov = math.radians(vp.fov)
        vfov = math.atan(math.tan(fov*0.5)*vp.w/vp.h)*2.0
        #u = vx^T(p-o)/(s vz^T(p-o)) + 0.5 = S/Q
        #  = [vx^T(p-o) + 0.5 (s vz^T(p-o))] / (s vz^T(p-o))
        #S = (vx+0.5 s vz)^T p - (vx + 0.5 s vz)^T o)
        #Q = s vz^T p - s vz^T o
        #v = 1/scale * vy^T(p-o)/(s vz^T(p-o)) + 0.5 = S/Q
        #  = [vy^T(p-o)/scale + 0.5 (s vz^T(p-o))] / (s vz^T(p-o))
        #T = (vy/scale+0.5 s vz)^T p - (vy/scale + 0.5 s vz)^T o)
        scale = 2.0*math.tan(fov*0.5)
        vscale = vp.h/vp.w
        xdir = vectorops.mul(xdir,-1)
        ydir = vectorops.mul(ydir,-1)
        xnum = vectorops.madd(xdir,zdir,0.5*scale)
        ynum = vectorops.madd(vectorops.div(ydir,vscale),zdir,0.5*scale)
        texgen[0,0:3] = xnum
        texgen[1,0:3] = ynum
        texgen[0,3] = -vectorops.dot(xnum,T[1])
        texgen[1,3] = -vectorops.dot(ynum,T[1])
        texgen[3,0:3] = vectorops.mul(zdir,scale)
        texgen[3,3] = -scale*vectorops.dot(zdir,T[1])
    app.setTexgen(texgen,True)
