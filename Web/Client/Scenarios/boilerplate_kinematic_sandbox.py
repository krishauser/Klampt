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
#def advance(t,world):
#	pass

world = None
t = 0

def boilerplate_start():
	global world,t
	t = 0
	world = WorldModel()
	kviz._init(world)
	stub.init(world)

def boilerplate_advance():
	global world,t
	stub.advance(t,world)
	t += 0.02

def boilerplate_keypress(key):
	print "boiler plate received key: " + str(key)
	#TODO call student code here? via stub? -DJZ


