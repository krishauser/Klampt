#!/usr/bin/python
import sys
sys.path.append("Web/Server")
import time
from klampt import *
from webrobotprogram import *
import kviz

#To be defined in stub module
def lab1a(point1,point2):
    pass

#To be defined in stub module
def source_motion(t):
    pass

#To be defined in stub module
def target_motion(t):
    pass


pos1 = (0,0)
pos2 = (2,0)
t = 0

def boilerplate_start():
    global pos1,pos2,t
    kviz.add_sphere("source",0,0,0,0.1)
    kviz.add_sphere("target",2,0,0,0.1)
    kviz.set_color("source",[1,0,0,1])
    kviz.set_color("target",[0,0,1,1])
    kviz.add_text("HUD1",20,30)

    pos1 = (0,0)
    pos2 = (2,0)
    t = 0

def boilerplate_advance():
    global pos1,pos2,t
    t += 0.02
    pos1 = stub.source_motion(t)
    pos2 = stub.target_motion(t)
    kviz.update_sphere("source",pos1[0],pos1[1],0)
    kviz.update_sphere("source",pos2[0],pos2[1],0)
    
    #draw text
    (length,angle) = stub.lab1a(pos1,pos2)
    kviz.update_text("HUD1","Length %f, angle %f"%(length,angle))
    return
