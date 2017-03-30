"""sensing.py: A collection of utility functions for dealing with sensors and sensor data.

The get/set_sensor_xform functions are used to interface cleanly with the klampt se3 functions.

The camera_to_X functions convert raw CameraSensor outputs to Python objects that are more easily
operated upon, e.g., images and point clouds.
"""

from ..robotsim import *
from ..io import loader
import coordinates
import math
import sys
from ..math import vectorops,so3,se3
import time

_has_numpy = False
_tried_numpy_import = False
np = None

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
		print "klampt.model.sensing.py: Warning, numpy not available."
		_has_numpy = False
	return _has_numpy

def get_sensor_xform(sensor,robot=None):
	"""Returns the sensor transform in klampt.se3 format.  If robot!=None, returns the world coordinates
	of the sensor.
	"""
	s = sensor.getSetting("Tsensor")
	Tsensor = loader.readSe3(s)
	if robot is not None:
		link = int(sensor.getSetting("link"))
		if link >= 0:
			return se3.mul(robot.link(link).getTransform(),Tsensor)
	return Tsensor


def set_sensor_xform(sensor,T,link=None):
	"""Given a SimRobotSensor that has a certain pose on a robot, such as a CameraSensor or 
	ForceSensor, sets the transform to the se3 element T, as local to the given link.

	If link is given, then the link of the sensor is modified.  link can either be an integer
	or a RobotModelLink.

	Another way to set a sensor is to give a coordinates.Frame object.  This frame must
	either be associated with a RobotModelLink or its parent should be associated with 
	one.

	(the reason why you should use this is that the Tsensor attribute )
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
	try:
		s = sensor.getSetting("Tsensor")
	except Exception:
		raise ValueError("Sensor does not have a Tsensor attribute")
	sensor.setSetting("Tsensor",loader.writeSe3(T))
	if link != None:
		if isinstance(link,RobotModelLink):
			sensor.setSetting("link",str(link.index))
		else:
			assert isinstance(link,int),"Can only set a sensor transform to a RobotModelLink or an integer link index"
			sensor.setSetting("link",str(link))


def camera_to_images(camera,image_format='numpy',color_format='channels'):
	"""Given a SimRobotSensor that is a CameraSensor, returns either the RGB image, the depth image, or both.

	If image_format='numpy' (default), returns numpy arrays.  Depending on the value of color_format, the RGB image
	either has shape (h,w,3) and dtype uint8 or (h,w) and dtype uint32.  Depth images as numpy arrays with
	shape (h,w).  Will fall back to 'native' if numpy is not available.

	If image_format='native', returns list-of-lists arrays in the same format as above

	If color_format='channels' (default), the RGB result is a 3D array with 3 channels corresponding to R, G, B
	values in the range [0,255].  If color_format='rgb' the result is a 2D array with a 32-bit integer channel
	with R,G,B channels packed in order XRGB.  If color_format='bgr' the result is similar but with order XBGR.

	(Note that image_format='native' takes up a lot of extra memory, especially with color_format='channels')
	"""
	assert isinstance(camera,SimRobotSensor),"Must provide a SimRobotSensor instance"
	assert camera.type() == 'CameraSensor',"Must provide a camera sensor instance"
	w = int(camera.getSetting('xres'))
	h = int(camera.getSetting('yres'))
	has_rgb = int(camera.getSetting('rgb'))
	has_depth = int(camera.getSetting('depth'))
	measurements = camera.getMeasurements()
	if image_format == 'numpy':
		if not _try_numpy_import():
			image_format = 'native'
	rgb = None
	depth = None
	if has_rgb:
		if image_format == 'numpy':
			abgr = np.array(measurements[0:w*h]).reshape(h,w).astype(np.uint32)
			if color_format == 'bgr':
				rgb = abgr
			elif color_format == 'rgb':
				rgb = np.bitwise_or(np.bitwise_or(np.left_shift(np.bitwise_and(abgr,0x00000ff),16),
										np.bitwise_and(abgr,0x000ff00)),
										np.right_shift(np.bitwise_and(abgr,0x0ff0000), 16))
			else:
				rgb = np.zeros((h,w,3),dtype=np.uint8)
				rgb[:,:,0] =                np.bitwise_and(abgr,0x00000ff)
				rgb[:,:,1] = np.right_shift(np.bitwise_and(abgr,0x00ff00), 8)
				rgb[:,:,2] = np.right_shift(np.bitwise_and(abgr,0x0ff0000), 16)
		else:
			if color_format == 'bgr':
				rgb = []
				for i in xrange(h):
					rgb.append([int(v) for v in measurements[i*w:(i+1)*w]])
			elif color_format == 'rgb':
				def bgr_to_rgb(pixel):
					return ((pixel & 0x0000ff) << 16) | (pixel & 0x00ff00) | ((pixel & 0xff0000) >> 16)
				rgb = []
				for i in xrange(h):
					rgb.append([bgr_to_rgb(int(v)) for v in measurements[i*w:(i+1)*w]])
			else:
				rgb = []
				for i in xrange(h):
					start = i*w
					row = []
					for j in xrange(w):
						pixel = int(measurements[start+j])
						row.append([pixel&0xff,(pixel>>8)&0xff,(pixel>>16)&0xff])
					rgb.append(row)
	if has_depth:
		start = (w*h if has_rgb else 0)
		if image_format == 'numpy':
			depth = np.array(measurements[start:start+w*h]).reshape(h,w)
		else:
			depth = []
			for i in xrange(h):
				depth.append(measurements[start+i*w:start+(i+1)*w])
	if has_rgb and has_depth:
		return rgb,depth
	elif has_rgb:
		return rgb
	elif has_depth:
		return depth
	return None

def camera_to_points(camera,points_format='numpy',all_points=False,color_format='channels'):
	"""Given a SimRobotSensor that is a CameraSensor, returns a point cloud associated with the current measurements.
	Points are triangulated with respect to the camera's intrinsic coordinates, and are returned in the camera local frame
	(+z backward, +x toward the right, +y toward up).
	The point cloud can be returned in several formats.  

	The points_format argument can take on several values:
	- 'numpy' (default): either an Nx3, Nx4, or Nx6 numpy array, depending on whether color is requested
	  (and its format).  Will fall back to 'native' if numpy is not available.
	- 'native': same as numpy, but in list-of-lists format rather than numpy arrays.
	- 'PointCloud': a Klampt PointCloud object
	- 'Geometry3D': a Klampt Geometry3D point cloud object

	If all_points=False (default), this strips out all pixels that don't have a good depth reading (i.e., the camera sensor's
	maximum reading.)  Otherwise these pixels are all set to (0,0,0).

	If the sensor has an RGB component, then color channels are produced.  These are interpreted differently
	depending on the color_format argument, which can take the following values:
	- 'channels': produces individual R,G,B channels in the range [0,1]. (note this is different from the
	  interpretation of camera_to_images)
	- 'rgb': produces a single 32-bit integer channel packing the 8-bit color channels together (actually BGR)
	- None: no color is produced.
	"""
	assert isinstance(camera,SimRobotSensor),"Must provide a SimRobotSensor instance"
	assert camera.type() == 'CameraSensor',"Must provide a camera sensor instance"
	assert int(camera.getSetting('depth'))==1,"Camera sensor must have a depth channel"
	has_numpy = _try_numpy_import()
	if points_format == 'numpy' and not has_numpy:
		points_format = 'native'

	images = camera_to_images(camera,'numpy',color_format)
	assert images != None

	rgb,depth = None,None
	if int(camera.getSetting('rgb'))==0:
		depth = images
		color_format = None
	else:
		rgb,depth = images

	w = int(camera.getSetting('xres'))
	h = int(camera.getSetting('yres'))
	xfov = float(camera.getSetting('xfov'))
	yfov = float(camera.getSetting('yfov'))
	zmin = float(camera.getSetting('zmin'))
	zmax = float(camera.getSetting('zmax'))
	xshift = -w*0.5
	yshift = -h*0.5
	xscale = math.tan(xfov*0.5)/(w*0.5)
	#yscale = -1.0/(math.tan(yfov*0.5)*h/2)
	yscale = xscale #square pixels are assumed
	xs = [(j+xshift)*xscale for j in range(w)]
	ys = [(i+yshift)*yscale for i in range(h)]
	if has_numpy:
		if all_points:
			depth[depth >= zmax] = 0
		if color_format is 'channels':
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

		if points_format == 'native':
			return pts.tolist()
		elif points_format == 'numpy':
			return pts
		elif points_format == 'PointCloud' or points_format == 'Geometry3D':
			res = PointCloud()
			if all_points:
				res.setSetting('width',str(w))
				res.setSetting('height',str(h))
			res.setPoints(pts.shape[0],pts[:,0:3].flatten().tolist())
			if color_format == 'rgb':
				res.addProperty('rgb')
				res.setProperties(pts[:,3].flatten().tolist())
			elif color_format == 'channels':
				res.addProperty('r')
				res.addProperty('g')
				res.addProperty('b')
				res.setProperties(pts[:,3:6].flatten().tolist())
			elif color_format == 'bgr':
				raise ValueError("bgr color format not supported with PointCloud output")
			if points_format == 'PointCloud':
				return res
			else:
				g = Geometry3D()
				g.setPointCloud(res)
				return g
		else:
			raise ValueError("Invalid points_format "+points_format)
		return Nnoe
	else:
		raise NotImplementedError("Native format depth image processing not done yet")

def camera_to_points_world(camera,robot,points_format='numpy',color_format='channels'):
	"""Same as camera_to_points, but converts to the world coordinate system given the robot
	to which the camera is attached.  Points that have no reading are stripped out. """
	assert isinstance(camera,SimRobotSensor),"Must provide a SimRobotSensor instance"
	assert camera.type() == 'CameraSensor',"Must provide a camera sensor instance"
	link = int(camera.getSetting('link'))
	Tsensor = camera.getSetting('Tsensor')
	#first 9: row major rotation matrix, last 3: translation
	entries = [float(v) for v in Tsensor.split()]
	Tworld = get_sensor_xform(camera,robot)
	#now get the points
	pts = camera_to_points(camera,points_format,all_points=False,color_format=color_format)
	if points_format == 'numpy':
		Rw = np.array(so3.matrix(Tworld[0]))
		tw = np.array(Tworld[1])
		pts[:,0:3] = np.dot(pts[:,0:3],Rw.T) + tw
		return pts
	elif points_format == 'native':
		for p in pts:
			p[0:3] = se3.apply(Tworld,p[0:3])
		return pts
	elif points_format == 'PointCloud' or points_format == 'Geometry3D':
		pts.transform(*Tworld)
	else:
		raise ValueError("Invalid format "+str(points_format))
	return pts

def camera_to_viewport(camera,robot):
	"""Returns a GLViewport instance corresponding to the camera's view.  See klampt.vis.glprogram
	and klampt.vis.visualization for information about how to use the object with the visualization."""
	assert isinstance(camera,SimRobotSensor),"Must provide a SimRobotSensor instance"
	assert camera.type() == 'CameraSensor',"Must provide a camera sensor instance"
	from ..vis.glprogram import GLViewport
	xform = get_sensor_xform(camera,robot)
	w = int(camera.getSetting('xres'))
	h = int(camera.getSetting('yres'))
	xfov = float(camera.getSetting('xfov'))
	yfov = float(camera.getSetting('yfov'))
	zmin = float(camera.getSetting('zmin'))
	zmax = float(camera.getSetting('zmax'))
	view = GLViewport()
	view.w, view.h = w,h
	view.fov = math.degrees(xfov)
	view.camera.dist = 1.0
	view.camera.tgt = se3.apply(xform,[0,0,view.camera.dist])
	#axes corresponding to right, down, fwd in camera view
	view.camera.set_orientation(xform[0],['x','y','z'])
	view.clippingplanes = (zmin,zmax)
	return view
