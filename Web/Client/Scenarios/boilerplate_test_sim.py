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
#def init(robot_model):
#	pass
#
#def control_loop(t,controller):
#	pass

viewer = None

def boilerplate_start():
	global viewer
	viewer = WebSimulationProgram(["./data/athlete_fractal_1.xml"])
	kviz._init(viewer.world)
	stub.init(viewer.world.robot(0))

def boilerplate_advance():
	global viewer	
	stub.control_loop(viewer.sim.getTime(),viewer.sim.controller(0))  #call student code
	viewer.sim.simulate(viewer.dt)
	viewer.sim.updateWorld()



