#!/usr/bin/python
import sys
sys.path.append("Web/Server")
import time
from klampt import *
import kviz
import math
import random

robot = None
target = None
target_axis = None
configuration = None

ghosts = None
ee_link = 7
ee_localpos = (0.17,0,0)
ee_localdir = (0,0,1)
t = 0

#stub provided
#def lab2c(robot,qseed,ee_link,ee_local_position,ee_local_axis,target,target_axis):
#   pass
#def pick_ik_seed(robot):
#    pass
#def target_motion(t):
#   pass
#def target_axis_motion(t):
#   pass


def boilerplate_start():
    global robot,target,target_axis,configurations,ghosts,t
    fn = "Web/Client/Scenarios/lab2/tx90pr2.rob"
    plane = "data/terrains/plane.env"
    cylinder = "data/objects/cylinder.obj"
    kviz._world.loadElement(plane)
    kviz._world.loadElement(cylinder)
    res = kviz._world.loadElement(fn)
    robot = kviz._world.robot(0)
    target = (0,0,0)
    target_axis = (0,0,1)
    configuration = None
    kviz.add_text("HUD1",1,1)
    kviz.add_text("HUD2",5,1)
    kviz.add_text("HUD3",9,1)
    kviz.add_text("HUD4",13,1)
    kviz.add_text("HUD5",17,1)
    kviz.add_text("HUD6",21,1)
    kviz.add_text("HUD7",25,1)
    kviz.set_color("cylinder",[1,0,0,1])

def boilerplate_advance():
    global robot,target,target_axis,configurations,ghosts,t,ee_link,ee_localpos,ee_localdir
    target = stub.target_motion(t)
    target_axis = stub.target_axis_motion(t)
    #aligns x to target_axis... we want z
    R = so3.canonical(list(target_axis))
    Rzx = so3.rotation([0,1,0],math.pi/2)
    kviz._world.rigidObject(0).setTransform(so3.mul(R,Rzx),target)

    qseed = stub.pick_ik_seed(robot)
    q,dpos,daxis = stub.lab2c(robot,qseed,ee_link,ee_localpos,ee_localdir,
                             target,target_axis)
    if not isinstance(q,list) or len(q) != robot.numLinks():
        print q
        raise ValueError("Improperly formatted return value")
    robot.setConfig(q)
    messages = []
    messages.append("Position error: %f, axis error %f"%(dpos,daxis))
    qmin,qmax = robot.getJointLimits()
    for i in range(min(len(q),7)):
        if q[i] < qmin[i] or q[i] > qmax[i]:
            messages.append("Joint %d position %f out of joint limits [%f,%f]"%(i,q[i],qmin[i],qmax[i]))
    for (i,msg) in enumerate(messages):
        kviz.update_text("HUD"+str(i+1),msg)
    for i in range(len(messages),6):
        kviz.update_text("HUD"+str(i+1),"")
    t += 0.02
    

       
