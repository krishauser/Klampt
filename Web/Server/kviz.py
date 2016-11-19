import json
from klampt import *
global _world,_sceneCurrent,_sceneChanges,_namedItems
_world = None
_sceneCurrent = {}
_RPC = []
_ghosts = []
#_namedItems = {}

def clear_extras():
	"""Erases all ghosts, lines, points, text, etc from the visualization, but keeps the original world."""
	_RPC.append({'type':'clear_extras'})

def remove(name):
	"""Removes a certain named target"""
	_RPC.append({'type':'remove','name':name})	

def set_visible(name,value=True):
	"""Changes the visibility status of a certain named target"""
	_RPC.append({'type':'set_visible','name':name,'value':value})	

def set_color(target,rgba_color):
	"""Sets the given RobotModel, RobotModelLink, named link, indexed link,
	or object name to some color (given as an (r,g,b) or (r,g,b,a) tuple)."""
	global _world,_ghosts
	recursive=False
	if isinstance(target, (int, long, float, complex)):
		robot = _world.robot(0)
		target_as_link = robot.link(target)
		target_name=target_as_link.getName()

	elif isinstance(target,RobotModelLink): 
		target_name=target.getName()

	elif isinstance(target,RobotModel): 
		target_name=target.getName()
		recursive = True

	elif isinstance(target, basestring):
		target_name=target
		if target in _ghosts:
			recursive = True
		else:
			#see if it's the name of a robot
			try:
				_world.robot(target).index
				recursive = True
			except Exception:
				pass
	else:
		print "ERROR: kviz.set_color requires target of either robot, link, index, or string name of object!"
		return;

	if len(rgba_color) == 3:
		rgba_color.append(1.0)

	if recursive:
		_RPC.append({'type':'set_color','object':target_name,'rgba':rgba_color,'recursive':True})
	else:
		_RPC.append({'type':'set_color','object':target_name,'rgba':rgba_color})
	#print "Setting link color!",('object',target_name,'rgba'),rgba_color

def set_transform(name,R=so3.identity(),t=[0]*3,matrix=None):
	"""Sets the transform of the target object.  If matrix is given, it's a 16-element 
	array giving the 4x4 homogeneous transform matrix, in row-major format.  Otherwise,
	R and t are the 9-element klampt.so3 rotation and 3-element translation."""
	if matrix != None:
		_RPC.append({'type':'set_transform','object':name,'matrix':matrix})
	else:
		_RPC.append({'type':'set_transform','object':name,'matrix':[R[0],R[3],R[6],t[0],R[1],R[4],R[7],t[1],R[2],R[5],R[8],t[2],0,0,0,1]})

def add_ghost(prefixname="ghost",robot=0):
	"""Adds a ghost configuration of the robot that can be posed independently.
	prefixname can be set to identify multiple ghosts. 

	Returns the identifier of the ghost for use in set_color.  The identifier is
	just prefixname + robot.getName()."""
	global _world
	target_name=_world.robot(robot).getName()	
	_RPC.append({'type':'add_ghost','object':target_name,'prefix_name':prefixname})
	_ghosts.append(prefixname+target_name)
	return prefixname+target_name

def get_robot_config(robot=0):
	"""A convenience function.  Gets the robot's configuration in the visualization
	world."""
	global _world,_namedItems
	robot = _world.robot(robot)	
	q = robot.getConfig()
	return q

def set_ghost_config(q,prefixname="ghost",robot=0):
	"""Sets the configuration of the ghost to q.  If the ghost is named, place its name
	in prefixname."""
	global _world,_namedItems
	robot = _world.robot(robot)	

	q_original = robot.getConfig()

	assert len(q) == robot.numLinks(),"Config must be correct size"
	robot.setConfig(q)

	for i in range(robot.numLinks()):
		T = robot.link(i).getTransform()
		p = robot.link(i).getParent()

		if p>=0:
			Tp = robot.link(p).getTransform()
  			T = se3.mul(se3.inv(Tp),T)

		mat = se3.homogeneous(T)
  		#mat is now a 4x4 homogeneous matrix 

  		name = prefixname+robot.link(i).getName()
  		#send to the ghost link with name "name"...
		_RPC.append({'type':'set_transform','object':name,'matrix':[mat[0][0],mat[0][1],mat[0][2],mat[0][3],mat[1][0],mat[1][1],mat[1][2],mat[1][3],mat[2][0],mat[2][1],mat[2][2],mat[2][3],mat[3][0],mat[3][1],mat[3][2],mat[3][3]]})

	robot.setConfig(q_original) #restore original config

def add_text(name="HUD_Text1",x=0,y=0,text=""):
	"""Adds a new piece of text displayed on the screen.  name is a unique identifier of
	the text, and x,y are the coordinates of upper left corner of the the text, in percent """
	_RPC.append({'type':'add_text','name':name,'x':x,'y':y,'text':text})	

def update_text(name="HUD_Text1",text=""):
	"""Changes string used in the the named piece of text"""
	_RPC.append({'type':'update_text','name':name,'text':text})	

def add_sphere(name="KVIZ_Sphere1",x=0,y=0,z=0,r=1):
	"""Adds a new sphere to the world with the given x,y,z position and radius r."""
	_RPC.append({'type':'add_sphere','name':name,'x':x,'y':y,'z':z,'r':r})	

def update_sphere(name="KVIZ_Sphere1",x=0,y=0,z=0,r=-1):
	"""Changes the position and or radius of the sphere.  If r < 0 then the radius is
	kept the same."""
	_RPC.append({'type':'update_sphere','name':name,'x':x,'y':y,'z':z,'r':r})	

def add_line(name="KVIZ_Line1",x1=0,y1=0,z1=0,x2=1,y2=1,z2=1):
	"""Adds a new line segment to the world connecting point (x1,y1,z1) to (x2,y2,z2)"""
	_RPC.append({'type':'add_line','name':name,'x1':x1,'y1':y1,'z1':z1,'x2':x2,'y2':y2,'z2':z2})	

def update_line(name="KVIZ_Line1",x1=0,y1=0,z1=0,x2=1,y2=1,z2=1):
	"""Updates the endpoints of a line segment to be (x1,y1,z1) and (x2,y2,z2)"""
	_RPC.append({'type':'update_line','name':name,'x1':x1,'y1':y1,'z1':z1,'x2':x2,'y2':y2,'z2':z2})	

def add_triangle(name="KVIZ_Tri1",a=(0,0,0),b=(1,0,0),c=(0,1,0)):
	"""Adds a new triangle with vertices a,b,c.  a,b, and c are 3-lists or 3-tuples."""
	_RPC.append({'type':'add_trilist','name':name,'verts':a+b+c})	

def update_triangle(name="KVIZ_Tri1",a=(0,0,0),b=(1,0,0),c=(0,1,0)):
	"""Updates the vertices of a triangle with vertices a,b,c.  a,b, and c are 3-lists or 3-tuples."""
	_RPC.append({'type':'update_trilist','name':name,'verts':a+b+c})

def add_quad(name="KVIZ_Quad1",a=(0,0,0),b=(1,0,0),c=(1,1,0),d=(0,1,0)):
	"""Adds a new quad (in CCW order) with vertices a,b,c,d.  a,b,c and d are 3-lists or 3-tuples."""
	_RPC.append({'type':'add_trilist','name':name,'verts':a+b+c+a+c+d})	

def update_quad(name="KVIZ_Quad1",a=(0,0,0),b=(1,0,0),c=(1,1,0),d=(0,1,0)):
	"""Updates the vertices of a quad (in CCW order) with vertices a,b,c,d.  a,b,c and d are 3-lists or 3-tuples."""
	_RPC.append({'type':'update_trilist','name':name,'verts':a+b+c+a+c+d})	

def add_billboard(name="KVIZ_Billboard",image=[[]],format='auto',crange=[0,1],colormap='auto',filter='linear',size=(1,1)):
	"""Adds a 2D billboard to the world.  The image is a 2D array of values, which is
	texure-mapped to a quad of size (w,h).  The format argument determines the format of the image
	data and the colormap argument specifies how the image affects the billboard appearance.

	By default the billboard is centered at (0,0,0) and faces up.  To modify it, call set_transform.

	- image: a 2D array of single-channel values, (r,g,b) tuples, or (r,g,b,a) tuples.  Rows are listed
	  top to bottom, rows from left to right.  Or, can also be a URL.
	- format:
	  - 'auto': autodetect the type from the image.  If the image contains values, the format is 'value'.
	  - 'value': the values are mapped through either 'opacity', 'rainbow', or gradient
	    color mapping.
	  - 'rgb': if the image contains values, they are interpreted as RGB values packed in 24 bit integers.
	    Otherwise, the first 3 channels of the tuple are used
	  - 'rgba': if the image contains values, they are interpreted as RGB values packed in 32 bit integers.
	    Otherwise, they are assumed (r,g,b,a) tuples
	- crange: the range of the given values / channels. By default [0,1], but if you are using uint8 encoding
	  this should be set to [0,255].
	- colormap: how the color of the billboard should be set based on the image.  Valid values are:
	  - 'auto': if the image contains values, the gradient ((0,0,0),(1,1,1)) is used.  Otherwise 'replace'
	    is used.
	  - (color1,color2): interpolates between the two given (r,g,b) or (r,g,b,a) tuples.
	  - 'opacity': sets the alpha channel only.
	  - 'modulate': the value / rgb / rgba texture modulates the billboard color as set by set_color
	- filter: how values between pixels are interpolated.  Either 'nearest' or 'linear'.
	- size: the (width,height) pair of the billboard, in world units.
	"""
	if not isinstance(image,str):
		import struct
		import base64
		bytes = []
		w,h = None,None
		h = len(image)
		for row in image:
			if w == None:
				w = len(row)
			else:
				assert w == len(row),"Image is not a 2D array"
		pixel = image[0][0]
		if format == 'auto':
			if hasattr(pixel,'__iter__'):
				if len(pixel) == 4:
					format = 'rgba'
				else:
					format = 'rgb'
			else:
				format = 'value'
		else:
			if not hasattr(pixel,'__iter__'):
				format = 'p'+format
		gradient = (type(colormap) != str)
		for row in image:
			for pixel in row:
				if format == 'value':
					u = min(1,max(0,(pixel - crange[0]) / (crange[1]-crange[0])))
					if gradient:
						color = vectorops.interpolate(gradient[0],gradient[1],u)
						r = 0xff * min(1,max(0,color[0]))
						g = 0xff * min(1,max(0,color[1]))
						b = 0xff * min(1,max(0,color[2]))
						packed = (0xff << 24) | (int(b) << 16) | (int(g) << 8) | int(r)
						bytes.append(struct.pack('<I',packed))
					else:
						val = 0xff * u
						bytes.append(struct.pack('B',val))
				elif format == 'prgb' or format == 'prgba':
					bytes.append(struct.pack('<I', pixel))
				elif format == 'rgb':
					r = 0xff * min(1,max(0,(pixel[0] - crange[0]) / (crange[1]-crange[0])))
					g = 0xff * min(1,max(0,(pixel[1] - crange[0]) / (crange[1]-crange[0])))
					b = 0xff * min(1,max(0,(pixel[2] - crange[0]) / (crange[1]-crange[0])))
					packed = (0xff << 24) | (int(b) << 16) | (int(g) << 8) | int(r)
					bytes.append(struct.pack('<I', packed))
				elif format == 'rgba':
					r = 0xff * min(1,max(0,(pixel[0] - crange[0]) / (crange[1]-crange[0])))
					g = 0xff * min(1,max(0,(pixel[1] - crange[0]) / (crange[1]-crange[0])))
					b = 0xff * min(1,max(0,(pixel[2] - crange[0]) / (crange[1]-crange[0])))
					a = 0xff * min(1,max(0,(pixel[3] - crange[0]) / (crange[1]-crange[0])))
					packed = (int(a) << 24) | (int(b) << 16) | (int(g) << 8) | int(r)
					bytes.append(struct.pack('<I', packed))
				else:
					raise ValueError("Invalid format "+format)
		image = base64.b64encode(''.join(bytes))
		_RPC.append({'type':'add_billboard','name':name,'imagedata':image,'width':w,'height':h,'size':size,'filter':filter,'colormap':colormap})
	else:
		_RPC.append({'type':'add_billboard','name':name,'image':image,'size':size,'filter':filter,'colormap':colormap})

def _init(world):
	global _world,_RPC,_ghosts
	_world = world
	_RPC = []
	_ghosts = []

def _reset():
	global _world,_RPC,_ghosts
	_world = WorldModel()
	_RPC = []
	_ghosts = []


def make_fixed_precision(obj,digits):
	if isinstance(obj,float):
		return round(obj,digits)
	elif isinstance(obj,list):
		for i in xrange(len(obj)):
			obj[i] = make_fixed_precision(obj[i],digits)
	elif isinstance(obj,tuple):
		return [make_fixed_precision(val,digits) for val in obj]
	elif isinstance(obj,dict):
		for i,v in obj.iteritems():
			obj[i] = make_fixed_precision(v,digits)
	return obj

def json_fixed_precision_dump(obj,digits):
	return json.dumps(make_fixed_precision(obj,digits))

def _getInitialJSON():
	global _world,_sceneCurrent,_RPC
	msg = json.loads(_world.getSceneJSON())
	msg['RPC'] = _RPC
	_RPC = []
	return json_fixed_precision_dump(msg,3)


def _getUpdateJSON():
	global _world,_sceneCurrent,_RPC
	msg = json.loads(_world.getTransformsJSON())
	msg['RPC'] = _RPC
	_RPC = []
	return json_fixed_precision_dump(msg,3)
