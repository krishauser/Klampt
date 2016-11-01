#!/usr/bin/python
import time
import sys
from klampt import *
sys.path.append("Web/Server")
#sys.path.append(".")
from webrobotprogram import *
import kviz
#from kviz import *

#Stub code module will have these functions defined:
#def init(world):
#	pass
#
#def advance(t,world):
#	pass

world = None
sim = None 
dt = 1.0/100.0
ee_link = 3
t = 0
in_trace = False
trace = []
trace_viz_status = {}

def update_trace():
	global trace,trace_viz_status
	if len(trace) == 0:
		#clear, the visualization world is cleared
		trace_viz_status = {}
	else:
		#update selectively
		for i in xrange(len(trace)):
			for j in xrange(len(trace[i])-1):
				if (i,j) in trace_viz_status:
					name,status = trace_viz_status[i,j]
					if status == 'hidden':
						a = trace[i][j]
						b = trace[i][j+1]
						kviz.set_visible(name,True)
						kviz.update_line(name,a[0],a[1],a[2],b[0],b[1],b[2])
						kviz.set_color(name,[1,0.5,0,1])
						trace_viz_status[i,j] = (name,'shown')
					else:
						pass
				else:
					name = 'l[%d,%d]'%(i,j)
					a = trace[i][j]
					b = trace[i][j+1]
					kviz.add_line(name,a[0],a[1],a[2],b[0],b[1],b[2])
					kviz.set_color(name,[1,0.5,0,1])
					trace_viz_status[i,j] = (name,'shown')


def boilerplate_start():
	global world,sim,t,desired,in_trace,trace
	t = 0
	world = WorldModel()
	world.loadElement("Web/Client/Scenarios/lab4/scara.rob")
	sim = Simulator(world)
	kviz._init(world)
	stub.init(world)
	curves = stub.curves()
	cnt = 0
	for curve in curves:
		for i in xrange(len(curve)-1):
			a,b = curve[i],curve[i+1]
			kviz.add_line("c"+str(cnt),a[0],a[1],0,b[0],b[1],0)
	in_trace = False
	trace = []
	update_trace()

def boilerplate_advance():
	global world,sim,t,in_trace,trace
	sim.updateWorld()
	torque = stub.getTorque(t,sim.controller(0).getSensedConfig(),sim.controller(0).getSensedVelocity())
	assert len(torque)==4,"Torque must be of length 4"
	tmax = world.robot(0).getTorqueLimits()
	for i in xrange(len(torque)):
		if abs(torque[i]) > tmax[i]:
			print "Exceeding torque limit joint %d, |%f| > %f"%(i,torque[i],tmax[i])
			torque[i] = torque[i]/abs(torque[i])*tmax[i]
	sim.controller(0).setTorque(torque)
	sim.simulate(dt)
	sim.updateWorld()
	t += 0.02
	ee = world.robot(0).link(ee_link)
	ee_pos = ee.getTransform()[1]
	if ee_pos[2] < 0:
		#add to trace
		if not in_trace:
			trace.append([ee_pos])
			in_trace = True
		else:
			if vectorops.distance(trace[-1][-1],ee_pos) > 0.005:
				trace[-1].append(ee_pos)
				update_trace()

def boilerplate_keypress(key):
	print "boiler plate received key: " + str(key)
	#TODO call student code here? via stub? -DJZ


