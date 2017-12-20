from klampt import *
from klampt.model import trajectory
from klampt import robotsim
import json

_title_id = '__TITLE__'
_scene_id = '__SCENE_JSON__'
_path_id = '__PATH_JSON__'
_rpc_id = '__RPC_JSON__'
_compressed_id = '__COMPRESSED__'
_dt_id = '__TIMESTEP__'

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

class HTMLSharePath:
	"""An exporter that converts animations to shareable HTML files.

	Example code:
	sharer = HTMLSharePath("mypath.html",name="My spiffy path")
	sharer.start(sim)  #or world
	while [simulation is running]:
		#do whatever control you wish to do here
		sim.simulate(...)
		sharer.animate()
	sharer.end() #this saves to the given filename
	"""
	def __init__(self,filename="path.html",name="Klamp't Three.js app",boilerplate='auto'):
		"""Initializes this to save to the given filename, with 'name' as the title of the HTML page.
		boilerplate, if not 'auto', is the boilerplate HTML file.  Otherwise it it automatically found
		in the klampt folder
		"""
		self.name = name
		if boilerplate == 'auto':
			import pkg_resources
			boilerplate = pkg_resources.resource_filename('klampt','data/share_path_boilerplate.html')
		f = open(boilerplate,'r')
		self.boilerplate_file = ''.join(f.readlines())
		f.close()
		if any(v not in self.boilerplate_file for v in [_title_id,_scene_id,_path_id,_rpc_id,_compressed_id,_dt_id]):
			raise RuntimeError("Boilerplate file does not contain the right tags")
		self.fn = filename
		self.scene = []
		self.transforms = {}
		self.rpc = []
		self.dt = 0
		self.last_t = 0
	def start(self,world):
		"""Begins the path saving with the given WorldModel or Simulator"""
		if isinstance(world,Simulator):
			self.sim = world
			self.world = world.world
			self.last_t = world.getTime()
		else:
			self.sim = None
			self.world = world
		self.scene = robotsim.ThreeJSGetScene(self.world)
	def animate(self,time=None):
		"""Updates the path from the world.  If the world wasn't a simulator, the time
		argument needs to be provided"""
		if self.sim != None and time == None:
			time = self.sim.getTime()
			self.sim.updateWorld()
		dt = time - self.last_t
		if self.dt == 0:
			self.dt = dt
		if self.dt == 0:
			return
		if abs(dt - self.dt) <= 1e-6:
			dt = self.dt
		numadd = 0
		while dt >= self.dt:
			numadd += 1
			transforms = json.loads(robotsim.ThreeJSGetTransforms(self.world))
			for update in transforms['object']:
				n = update['name']
				mat = make_fixed_precision(update['matrix'],4)
				matpath = self.transforms.setdefault(n,[])
				assert len(matpath) == len(self.rpc)
				lastmat = None
				for m in matpath[::-1]:
					if m != None:
						lastmat = m
						break
				if lastmat != mat:
					matpath.append(mat)
				else:
					matpath.append(None)
			self.rpc.append('null')
			dt -= self.dt
			self.last_t += self.dt
		if numadd > 1:
			print "Uneven time spacing, duplicating frame",numadd,"times"
	def end(self):
		data = self.boilerplate_file.replace(_title_id,self.name)
		data = data.replace(_scene_id,self.scene)
		data = data.replace(_path_id,json.dumps(self.transforms))
		data = data.replace(_rpc_id,'['+','.join(self.rpc)+']')
		data = data.replace(_compressed_id,'true')
		data = data.replace(_dt_id,str(self.dt))
		print "Path with",len(self.rpc),"frames saved to",self.fn
		f = open(self.fn,'w')
		f.write(data)
		f.close()

if __name__ == '__main__':
	import sys
	import os
	from klampt import trajectory
	world = WorldModel()
	if len(sys.argv) == 1:
		world.readFile("../../data/athlete_plane.xml")
		q = world.robot(0).getConfig()
		q[2] = 2
		world.robot(0).setConfig(q)
		sim = Simulator(world)
		share = HTMLSharePath(name="Klamp't simulation path")
		share.start(sim)
		for i in range(100):
			sim.simulate(0.02)
			share.animate()
		share.end()
	else:
		assert len(sys.argv) == 3,"Usage: sharepath.py world.xml robot_path"
		world.readFile(sys.argv[1])
		traj = trajectory.RobotTrajectory(world.robot(0))
		traj.load(sys.argv[2])
		world.robot(0).setConfig(traj.milestones[0])

		dt = 0.02
		excess = 1.0

		share = HTMLSharePath(name="Klamp't path "+os.path.split(sys.argv[2])[1])
		share.start(world)
		share.dt = dt
		t = traj.times[0]
		while t < traj.times[-1] + excess:
			world.robot(0).setConfig(traj.eval(t))
			share.animate(t)
			t += dt
		share.end()
