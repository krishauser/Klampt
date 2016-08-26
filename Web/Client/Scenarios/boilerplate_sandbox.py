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
#def control_loop(t,controller):
#	pass

viewer = None

def boilerplate_start():
	global viewer
	viewer = WebSimulationProgram([])
	kviz._init(viewer.world)
	stub.init(viewer.world)
	viewer.sim = Simulator(viewer.world)

def boilerplate_advance():
	global viewer
	if viewer.world.numRobots() > 0:
		stub.control_loop(viewer.sim.getTime(),viewer.sim.controller(0))  #call student code
	print "Simulating...",viewer.dt
	viewer.sim.simulate(viewer.dt)
	viewer.sim.updateWorld()
	print "Done."

def boilerplate_keypress(key):
	print "boiler plate received key: " + str(key)
	#TODO call student code here? via stub? -DJZ


