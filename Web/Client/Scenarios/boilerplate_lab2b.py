#!/usr/bin/python
import sys
sys.path.append("Web/Server")
import time
from klampt import *
import kviz
import math

#stub code
#def lab2b(L1,L2,L3,point,angle):
#    pass
#def ik_goal_motion(t):
#    pass

solutions = (1,[(0,0,0)])
robot = None
ghost = ""
t = 0

def boilerplate_start():
    global robot,ghost,solutions,t
    solutions = (1,[(0,0,0)])
    world = kviz._world
    fn = "Web/Client/Scenarios/lab2/planar3R.rob"
    res = world.loadElement(fn)
    assert res >= 0
    kviz._init(world)

    robot = world.robot(0)
    ghost = kviz.add_ghost("second_solution")
    #hide it
    kviz.set_color(ghost,[0,0,0,0])
    kviz.add_sphere("target_point",0,0,0,0.15)
    kviz.set_color("target_point",[1,0,0,1])
    kviz.add_sphere("target_direction",0,0,0,0.1)
    kviz.set_color("target_direction",[1,0.5,0,1])
    t = 0

def boilerplate_advance():
    global robot,ghost,solutions,t

    point,angle = stub.ik_goal_motion(t)
    kviz.update_sphere("target_point",point[0],point[1],0)
    offset = 0.25
    kviz.update_sphere("target_direction",point[0]+offset*math.cos(angle),point[1]+offset*math.sin(angle),0)

    #solve
    solutions=stub.lab2b(1,1,1,(point[0],point[1]),angle)

    #update visualization of solutions
    if solutions[0] == 0:
        #no solutions, draw robot in transparent red
        for i in range(robot.numLinks()):
            kviz.set_color(robot.link(i),[1,0,0,0.25])
        #hide ghost
        kviz.set_color(ghost,[0,0,0,0])
    elif solutions[0] == 1:
        #hide ghost
        kviz.set_color(ghost,[0,0,0,0])
        for i in range(robot.numLinks()):
            kviz.set_color(robot.link(i),1,0,1,1)
        robot.setConfig(solutions[1][0])
    else:
        kviz.set_color(ghost,[1,0.5,1,1])
        for i in range(robot.numLinks()):
            kviz.set_color(robot.link(i),[1,0,1,1])
        robot.setConfig(solutions[1][0])
        kviz.set_ghost_config(solutions[1][1],"second_solution")
    t += 0.02

    

