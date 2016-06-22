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

def set_color(target,rgba_color,robot=0):
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
		rgba_color.append(1.0);

	_RPC.append({'type':'set_color','object':target_name,'rgba':rgba_color})
	#print "Setting link color!",('object',target_name,'rgba'),rgba_color

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
