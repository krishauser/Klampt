import json
from klampt import *
global _world,_sceneCurrent,_sceneChanges,_namedItems
_world = None
_sceneCurrent = {}
_RPC = []
#_namedItems = {}


#def _get_path(path,struct=None):
#	if struct==None:
#		struct = _sceneCurrent
#	if len(path)==0:
#		return struct
#	else:
#		if isinstance(struct,(list,tuple)):
#			assert isinstance(path[0],int) and path[0] >= 0 and path[0] < len(struct),"Invalid list index "+str(path[0])
#		elif isinstance(struct,dict):
#			if path[0] not in struct:
#				return None
#		else:
#			print struct,path
#			raise ValueError("Cannot retrieve sub-element of basic data type")
#		return _get_path(path[1:],struct[path[0]])
#

#def _add_rpc(path,value):
#	global _sceneCurrent,_sceneChanges
#	v = _get_path(path[:-1])
#	if value == None:
#		del v[path[-1]]
#		_sceneChanges.append({'command':'delete','path':path})
#	else:
#		v[path[-1]] = value
#		_sceneChanges.append({'type':'change','path':path,'value':value})


#def _to_threejs_color(color):
#	return int(color[0]*0xff) << 16 | int(color[1]*0xff)<<8 | int(color[2]*0xff)

def set_color(target,rgba_color,recursive=False):
	"""Sets the given RobotModelLink or named link or indexed link to
	some color."""

	if isinstance(target, (int, long, float, complex)):
		global _world
		robot = _world.robot(robot)
		target_as_link = robot.link(target)
		target_name=target_as_link.getName()

	elif isinstance(target,RobotModelLink): 
		target_name=target.getName()

	elif isinstance(target, basestring):
		target_name=target
	else:
		print "ERROR: kviz.set_color requires target of either link, index, or string name of object!"
		return;

	if len(rgba_color) == 3:
		rgba_color.append(1.0)

	_RPC.append({'type':'set_color','object':target_name,'rgba':rgba_color,'recursive':recursive})
	#print "Setting link color!",('object',target_name,'rgba'),rgba_color

def add_ghost(prefixname="ghost",robot=0):
	global _world
	target_name=_world.robot(robot).getName()	
	_RPC.append({'type':'add_ghost','object':target_name,'prefix_name':prefixname})
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


#def add_config(q,color=(0,1,0,0.5),name="ghost",robot=0):
#	"""Draws the configuration q using a ghosted robot.  Multiple ghosted
#	configurations can be added using the name keyword.  The configuration
#	is persistent and can be changed using multiple calls with the same name.
#	To remove the configuration, call kviz.remove_config."""
#	global _world,_namedItems
#	robot = _world.robot(robot)
#	assert len(q) == robot.numLinks(),"Config must be correct size"
#	iname = ('config',robot.getName(),name)
#	path = ('object',robot.getName()+"_"+name)
#	if iname in _namedItems:
#		#just change the config / color 
#		pass
#	else:
#		#TODO: fill out a three.js structure representing the robot
#		robotJson = {}
#		_set_path(path,robotJson)
#		_namedItems[iname] = path
#
#def remove_config(name="ghost",robot=0):
#	"""Deletes the drawing of the ghosted robot"""
#	global _world,_namedItems
#	iname = ('config',_world.robot(robot).getName(),name)
#	_set_path(_namedItems[iname],None)
#	del _namedItems[iname]
#
#def add_text(text,x,y,color=(0,0,0),size=12,name="_"):
#	global _world,_namedItems
#	assert len(q) == _world.robot(robot).numLinks(),"Config must be correct size"
#	path = ('text',name)
#	_set_path(path,{'text':text,'position':[x,y],'color':color,'size':size})
#	_namedItems[('text',name)] = path
#

def add_text(name="HUD_Text1",x=0,y=0,text=""):
	_RPC.append({'type':'add_text','name':name,'x':x,'y':y,'text':text})	

def update_text(name="HUD_Text1",text=""):
	_RPC.append({'type':'update_text','name':name,'text':text})	

def add_sphere(name="KVIZ_Sphere1",x=0,y=0,z=0,r=1):
	_RPC.append({'type':'add_sphere','name':name,'x':x,'y':y,'z':z,'r':r})	

def update_sphere(name="KVIZ_Sphere1",x=0,y=0,z=0,r=-1):
	_RPC.append({'type':'update_sphere','name':name,'x':x,'y':y,'z':z,'r':r})	

#def remove_text(name="_"):
#	"""Deletes the drawing of the given text"""
#	global _world,_namedItems
#	_set_path(_namedItems[('config',name)],None)
#	del _namedItems[('config',name)]
#
def _init(world):
	global _world
	_world = world


def _getInitialJSON():
	global _world,_sceneCurrent,_RPC
	msg = json.loads(_world.getSceneJSON())
	msg['RPC'] = _RPC
	_RPC = []
	return json.dumps(msg)


def _getUpdateJSON():
	global _world,_sceneCurrent,_RPC
	msg = json.loads(_world.getTransformsJSON())
	msg['RPC'] = _RPC
	_RPC = []
	return json.dumps(msg)
