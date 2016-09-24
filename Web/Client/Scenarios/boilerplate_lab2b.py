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
ghosts = []
t = 0

def boilerplate_start():
    global robot,ghosts,solutions,t
    solutions = (1,[(0,0,0)])
    world = kviz._world
    fn = "Web/Client/Scenarios/lab2/3R_zyy.rob"
    res = world.loadElement(fn)
    assert res >= 0
    kviz._init(world)

    robot = world.robot(0)
    ghosts = []
    ghosts.append(kviz.add_ghost("solution2"))
    ghosts.append(kviz.add_ghost("solution3"))
    ghosts.append(kviz.add_ghost("solution4"))
    kviz.add_sphere("target_point",0,0,0,0.15)
    kviz.set_color("target_point",[1,0,0,1])
    t = 0

def boilerplate_advance():
    global robot,ghosts,solutions,t

    point = stub.ik_goal_motion(t)
    kviz.update_sphere("target_point",point[0],point[1],point[2])

    #solve
    solutions=stub.lab2b(0.25,1,1,point)
    print solutions

    #update visualization of solutions
    if solutions[0] == 0 or solutions[0] == float('inf'):
        #no solutions, draw robot in transparent red
        for i in range(robot.numLinks()):
            kviz.set_color(robot.link(i),[1,0,0,0.25])
        #hide ghosts
        for ghost in ghosts:
            kviz.set_color(ghost,[0,0,0,0])
    else:
        #show/hide ghosts
        for i,ghost in enumerate(ghosts):
            if i+1 >= solutions[0]:
                print "Hiding ghost",i
                kviz.set_color(ghost,[0,0,0,0])
            else:
                print "Setting ghost",i,"solution",solutions[1][i+1]
                kviz.set_color(ghost,[1,1.0/(i+2),1,1])
                kviz.set_ghost_config(solutions[1][i+1],"solution"+str(i+2))
        for i in range(robot.numLinks()):
            kviz.set_color(robot.link(i),[1,1,1,1])
        robot.setConfig(solutions[1][0])
    t += 0.02

    

