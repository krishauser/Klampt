#!/usr/bin/python
import time
import sys
from klampt import *
sys.path.append("Web/Server")
#sys.path.append(".")
import kviz

#Stub code module will have these functions defined:
#def init(robot_model):
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
	world.readFile("./data/athlete_fractal_1.xml")
	kviz._init(world)
	sim = Simulator(world)
	stub.init(world.robot(0))

def boilerplate_advance():
	global world,sim
	stub.control_loop(sim.getTime(),sim.controller(0))  #call student code
	sim.simulate(dt)
	sim.updateWorld()



