import json
from klampt import *
global _world,_sceneCurrent,_sceneChanges,_namedItems
_world = None
_sceneCurrent = {}
_RPC = []
_ghosts = []
#_namedItems = {}

def set_color(target,rgba_color):
	"""Sets the given RobotModel, RobotModelLink, named robot, named ghost, named link, or indexed link to
	some color."""
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

def add_ghost(prefixname="ghost",robot=0):
	global _world
	target_name=_world.robot(robot).getName()	
	_RPC.append({'type':'add_ghost','object':target_name,'prefix_name':prefixname})
	_ghosts.append(prefixname+target_name)
	return prefixname+target_name

def get_robot_config(robot=0):
	global _world,_namedItems
	robot = _world.robot(robot)	
	q = robot.getConfig()
	return q

def set_ghost_config(q,prefixname="ghost",robot=0):
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
		_RPC.append({'type':'set_position','object':name,'matrix':[mat[0][0],mat[0][1],mat[0][2],mat[0][3],mat[1][0],mat[1][1],mat[1][2],mat[1][3],mat[2][0],mat[2][1],mat[2][2],mat[2][3],mat[3][0],mat[3][1],mat[3][2],mat[3][3]]})

	robot.setConfig(q_original) #restore original config

def add_text(name="HUD_Text1",x=0,y=0,text=""):
	_RPC.append({'type':'add_text','name':name,'x':x,'y':y,'text':text})	

def update_text(name="HUD_Text1",text=""):
	_RPC.append({'type':'update_text','name':name,'text':text})	

def add_sphere(name="KVIZ_Sphere1",x=0,y=0,z=0,r=1):
	_RPC.append({'type':'add_sphere','name':name,'x':x,'y':y,'z':z,'r':r})	

def update_sphere(name="KVIZ_Sphere1",x=0,y=0,z=0,r=-1):
	_RPC.append({'type':'update_sphere','name':name,'x':x,'y':y,'z':z,'r':r})	

def add_line(name="KVIZ_Line1",x1=0,y1=0,z1=0,x2=1,y2=1,z2=1):
	_RPC.append({'type':'add_line','name':name,'x1':x1,'y1':y1,'z1':z1,'x2':x2,'y2':y2,'z2':z2})	

def update_line(name="KVIZ_Line1",x1=0,y1=0,z1=0,x2=1,y2=1,z2=1):
	_RPC.append({'type':'update_line','name':name,'x1':x1,'y1':y1,'z1':z1,'x2':x2,'y2':y2,'z2':z2})	

def set_visible(name="KVIZ_target",value=True):
	_RPC.append({'type':'set_visible','name':name,'value':value})	


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
	elif isinstance(obj,(list,tuple)):
		for i in xrange(len(obj)):
			obj[i] = make_fixed_precision(obj[i],digits)
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
