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
configurations = None
ghosts = None
ee_link = 7
ee_localpos = (0.17,0,0)
t = 0

#stub provided
#def lab2a(robot,q,ee_link,ee_localpos,target):
#   pass
#stub provided
#def target_motion(t):
#   pass

def arg_min(items):
    return min((v,i) for (i,v) in enumerate(items))[1]

def boilerplate_start():
    global robot,target,configurations,ghosts,t
    fn = "Web/Client/Scenarios/lab2/tx90pr2.rob"
    plane = "data/terrains/plane.env"
    kviz._world.loadElement(plane)
    res = kviz._world.loadElement(fn)
    robot = kviz._world.robot(0)
    target = (0,0,0)
    configurations = []
    ghosts = []
    for i in range(5):
        qmin,qmax = robot.getJointLimits()
        q = [random.uniform(a,b) for a,b in zip(qmin,qmax)]
        #stretch it out a little bit
        q[3] = random.uniform(-0.5,0.5)
        q[5] = random.uniform(-0.5,0.5)
        configurations.append(q)
        ghost = kviz.add_ghost("config"+str(i))
        ghosts.append(ghost)
        kviz.set_color(ghosts[i],0,1,0,1)
    kviz.add_sphere("target",target[0],target[1],target[2],0.1)
    kviz.add_text("HUD1",1,1)
    #hide the robot
    robot.setConfig([0.0]*robot.numLinks())
    for i in range(robot.numLinks()):
        robot.link(i).appearance().setColor(0,0,1,0)
    t = 0

def boilerplate_advance():
    global robot,target,configurations,ghosts,t,ee_link,ee_localpos
    target = stub.target_motion(t)
    kviz.update_sphere("target",*target)

    #draw the configurations with opacity proportional to end-effector
    #distance
    distances = []
    for q in configurations:
        robot.setConfig([0.0]*robot.numLinks())
        distance = stub.lab2a(robot,q,ee_link,ee_localpos,target)
        distances.append(distance)
    closest = arg_min(distances)
    #drange = (min(distances),max(distances))
    drange = (0,2)
    for i,(d,q) in enumerate(zip(distances,configurations)):
        opacity = 1
        if drange[1]!=drange[0]:
            opacity = (drange[1]-d)/(drange[1]-drange[0])
            opacity = max(0,min(opacity,1))
        kviz.set_ghost_config(q,"config"+str(i))
        if i == closest:
            kviz.set_color(ghosts[i],1,0,0,opacity)
        else:
            kviz.set_color(ghosts[i],0.5,0.5,0.5,opacity)

    kviz.update_text("HUD1","Closest: "+str(closest)+" at distance "+str(distances[closest]))
    t += 0.02

