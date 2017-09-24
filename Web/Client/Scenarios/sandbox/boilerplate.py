#!/usr/bin/python
import time
import sys
from klampt import *
sys.path.append("Web/Server")
#sys.path.append(".")
import kviz
#from kviz import *

#Stub code module will have these functions defined:
#def init(world):
#	pass
#
#def control_loop(t,controller):
#	pass

world = None
sim = None
dt = 0.02

def boilerplate_start():
	global world,sim
	world = WorldModel()
	kviz._init(world)
	stub.init(world)
	sim = Simulator(world)

def boilerplate_advance():
	global world,sim
	if world.numRobots() > 0:
		stub.control_loop(sim.getTime(),sim.controller(0))  #call student code
	#print "Simulating...",dt
	sim.simulate(dt)
	sim.updateWorld()
	#print "Done."

def boilerplate_keypress(key):
	stub.keypress(key)

def boilerplate_setitem(name,value):
	if name == "move_delay":
		print "Setting move_delay to",value
		stub.move_delay = float(value)