#!/usr/bin/python
import time
import sys
from klampt import *
sys.path.append("Web/Server")
#sys.path.append(".")
from webrobotprogram import *
import kviz
from klampt import *
from klampt import vectorops,so2,so3,se3
import random

#from kviz import *

#Stub code module will have these functions defined:
#def init()
#   pass
#
#def depthImageToPointCloud(scan):
#	pass
#
#def localToWorld(localPC,robotState,cameraExtrinsics):
#	pass

world = None
robot = None
t = 0

def rayCast(world,s,d):
	mindist = float('inf')
	closestpt = None
	for i in xrange(world.numTerrains()):
		hit,pt = world.terrain(i).geometry().rayCast(s,d)
		if hit:
			dist = vectorops.dot(d,vectorops.sub(pt,s))
			if dist < mindist:
				closestpt = pt
				mindist = dist
	return closestpt

class RangeSensor1D:
	def __init__(self):
		self.halffov = math.radians(120)/2
		self.scale = math.tan(self.halffov)
		self.range = (0.5,4)
		self.resolution = 100
		self.quanta = 256
		self.noisy = False
		self.perturbationNoise = 0.02
		self.shotNoiseProbability = 0.02
	
	def generate(self,T,world):
		"""Simulates the sensor, generating a quantized 1-D depth "image".

		Input: T is the transform of the sensor (forward in the y direction)
		world is the klamp't WorldModel.
		"""
		src = T[1]
		R = T[0]
		x = [R[0],R[1],R[2]]
		y = [R[3],R[4],R[5]]
		z = [R[6],R[7],R[8]]
		res = []
		for i in xrange(self.resolution):
			#map i to range [-1,1]
			xcoord = float(i)*2/(self.resolution-1)-1
			#add y + xscale*mapped i
			dir = vectorops.madd(y,x,xcoord*self.scale)
			dir = vectorops.unit(dir)
			#do ray casting
			pt = rayCast(world,src,dir)
			if pt == None:
				res.append(self.simulateNoise(0))
			else:
				d = vectorops.dot(vectorops.sub(pt,src),y)
				if d < self.range[0] or d > self.range[1]:
					res.append(self.simulateNoise(0))
				else:
					d = self.simulateNoise(d)
					res.append(int(self.quanta*d/self.range[1]))		   
		return res
	
	def simulateNoise(self,d):
		if not self.noisy: return d
		if d == 0:
			if random.random() < self.shotNoiseProbability:
				d = random.random()*self.range[1]
				if d < self.range[0]: return 0
			return 0
		if random.random() < self.shotNoiseProbability: return d*random.random()
		else: return d*(1.0+random.gauss(0,self.perturbationNoise))



class Robot:
	def __init__(self,world):
		self.world = world
		self.robot = world.rigidObject(0)
		self.state = (0,0,0)
		self.cameraPos = (0.05,0.02,0.25)
		self.sensor = RangeSensor1D()
		self.sensor.noisy = stub.noisy_sensor

		#current time
		self.time = 0
		
		#rate of sensor readings, in Hz
		self.senseRate = 10
		#next time the sensor should be read
		self.nextSenseTime = 0
		#last scan
		self.lastScan = None
		
		#TODO: In problem C you will uncomment this line
		#self.sensor.noisy = True

		#choose the mapper
		if stub.method == 'accumulator':
			self.mapper = stub.AccumulatorMapper()
		elif stub.method == 'occupancy_grid':
			self.mapper = stub.OccupancyGridMapper(10,10,128,128)
		elif stub.method == 'robust_occupancy_grid':
			self.mapper = stub.RobustOccupancyGridMapper(10,10,128,128)
		else:
			self.mapper = stub.ProbabilisticOccupancyGridMapper(10,10,128,128)
		self.map = None

		self.last_viz_point = 0

	def initVis(self):
		kviz.add_sphere("camera",0,0,0,0.03)
		kviz.add_line("camera_dir")
		kviz.set_color("camera",(1,1,0,1))
		kviz.set_color("camera_dir",(1,1,0,1))
		self.last_viz_point = 0
		self.updateVis()

	def updateVis(self):
		Trobot = (so3.rotation((0,0,1),self.state[2]),(self.state[0],self.state[1],0))
		self.robot.setTransform(Trobot[0],Trobot[1])
		kviz.update_sphere("camera",*se3.apply(Trobot,self.cameraPos),r=0.03)
		kviz.update_line("camera_dir",*(se3.apply(Trobot,self.cameraPos)+se3.apply(Trobot,vectorops.add(self.cameraPos,(0.15,0,0)))))

		#draw map, if it exists
		if self.map:
			height = self.cameraPos[2]
			if isinstance(self.map,stub.PointCloudMap):
				for i,p in enumerate(self.map.points):
					if i >= self.last_viz_point:
						kviz.add_sphere("p"+str(i),p[0],p[1],height,r=0.01)
				self.last_viz_point = len(self.map.points)
			else:
				kviz.remove("map")
				w = self.map.bounds[1][0]-self.map.bounds[0][0]
				h = self.map.bounds[1][1]-self.map.bounds[0][1]
				cx = 0.5*(self.map.bounds[1][0]+self.map.bounds[0][0])
				cy = 0.5*(self.map.bounds[1][1]+self.map.bounds[0][1])
				kviz.add_billboard("map",self.map.array,'auto',(self.map.vmin,self.map.vmax),filter='nearest',size=(w,h),colormap='opacity')
				kviz.set_color("map",(0,1,1,1))
				kviz.set_transform("map",R=so3.rotation((0,0,1),math.pi/2),t=(cx,cy,height))

	def doSensing(self):
		#get simulated sensor transform (note: y is forward)
		y = [math.cos(self.state[2]),math.sin(self.state[2]),0]
		z = [0,0,1]
		x = [y[1],-y[0],0]
		R = x + y + z
		Trobot = (so3.rotation((0,0,1),self.state[2]),(self.state[0],self.state[1],0))
		t = se3.apply(Trobot,self.cameraPos)
		Tsensor = R,t
		img = self.sensor.generate(Tsensor,self.world)
		self.lastScan = img[:]
		#data processing pipeline
		localpc = stub.depthImageToPointCloud(img)
		worldpc = stub.localToWorld(localpc,self.state,self.cameraPos)
		if self.mapper:
			self.mapper.addScan(Tsensor,worldpc)
			self.map = self.mapper.getMap()
	
	def drive(self,control,dt):
		self.time += dt
		#integrate
		wcontrol = so2.apply(self.state[2],(control[0],control[1]))
		self.state = vectorops.madd(self.state,(wcontrol[0],wcontrol[1],control[2]),dt)
		self.state[2] = self.state[2] % (math.pi*2.0)
		#determine if we should do sensing
		if self.time >= self.nextSenseTime:
			self.doSensing()
			self.nextSenseTime += 1.0/self.senseRate



def boilerplate_start():
	global world,robot,t
	t = 0
	world = WorldModel()
	fn = "Web/Client/Scenarios/lab5/lab6world.xml"
	if stub.environment == 'fancy':
		fn = "Web/Client/Scenarios/lab5/luxuryhouseworld.xml"
	if not world.readFile(fn):
		raise IOError("Unable to load world XML file")
	robot = Robot(world)
	stub.init()
	kviz._init(world)
	robot.initVis()


def boilerplate_advance():
	global world,robot,t
	u = stub.get_control(t)
	dt = 0.05
	robot.drive(u,dt)

	#robot.drive((1,0,0),dt)
	robot.updateVis()
	t += dt
	

def boilerplate_keypress(key):
	print "boiler plate received key: " + str(key)
	#TODO call student code here? via stub? -DJZ


