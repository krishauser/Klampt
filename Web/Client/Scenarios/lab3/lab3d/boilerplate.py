from klampt import *
from klampt.math import vectorops,so3,se3
from klampt.plan.cspace import MotionPlan
import math
import sys
sys.path.append("Web/Server")
import kviz
import random

world = WorldModel()
space = None
start = None
target = None
path = None
roadmap = None
planner = None
optimizing = False
existing_path_lines = []
existing_ghosts = []
existing_roadmap_lines = set()
path_height = 0.02
accumulated_time = 0
    
def boilerplate_start():
    global world,space,start,target,planner,optimizing,path,existing_path_lines,existing_ghosts,roadmap,existing_roadmap_lines
    global accumulated_time
    accumulated_time = 0

    #global options that don't get forgotten from instance to instance
    MotionPlan.setOptions(restart=False,shortcut=False)

    start=stub.start()
    target=stub.target()

    world = WorldModel()
    world.readFile("Web/Client/Scenarios/lab3/plan_world.xml")
    space = stub.SE2ObstacleCSpace(world)

    #TODO: in Problem 4 you should switch the commented-out lines to
    #decide whether you want a feasible planner or an optimizing planner
    planner,optimizing = stub.makePlanner(space,start,target)
    try:
        planner.setEndpoints(start,target)
    except:
        pass
    
    path = []
    roadmap = [],[]
    kviz._init(world)
    radius = 0.025
    
    print world.robot(0).numLinks()
    start_ghost = kviz.add_ghost("start")
    target_ghost = kviz.add_ghost("target")
    kviz.set_ghost_config(start,"start")
    kviz.set_ghost_config(target,"target")
    kviz.set_color(start_ghost,0,1,0)
    kviz.set_color(target_ghost,1,0,0)
    existing_roadmap_lines = set()
    existing_path_lines = []
    existing_ghosts = []
    refresh_viz()

def refresh_viz():
    global start,target,path,existing_path_lines,existing_ghosts,roadmap,existing_roadmap_lines
    kviz.update_sphere("robot",start[0],start[1],0)
    kviz.update_sphere("target",target[0],target[1],0)
    for i,name in enumerate(existing_path_lines):
        if i >= len(path):
            kviz.set_visible(name,False)
            kviz.set_visible(existing_ghosts[i],False)
    for i in xrange(len(path)-1):
        if i >= len(existing_path_lines):
            name = "path"+str(i)
            existing_path_lines.append(name)
            kviz.add_line(name,path[i][0],path[i][1],path_height,path[i+1][0],path[i+1][1],path_height)
            kviz.set_visible(name,True)
            kviz.set_color(name,0,0,1)
            existing_ghosts.append(kviz.add_ghost("ghost"+str(i)))
            kviz.set_ghost_config(path[i],"ghost"+str(i))
            kviz.set_color(existing_ghosts[-1],1,1,1,0.5)
        else:
            name = existing_path_lines[i]
            kviz.set_visible(name,True)
            kviz.update_line(name,path[i][0],path[i][1],path_height,path[i+1][0],path[i+1][1],path_height)
            kviz.set_ghost_config(path[i],"ghost"+str(i))
    if stub.draw_roadmap:
        V,E = roadmap
        for i,e in enumerate(E):
            name = "edge"+str(i)
            a,b = e
            if name not in existing_roadmap_lines:
                existing_roadmap_lines.add(name)
                kviz.add_line(name,V[a][0],V[a][1],0.0,V[b][0],V[b][1],0.0)
                kviz.set_color(name,1,1,0)
            else:
                kviz.update_line(name,V[a][0],V[a][1],0.0,V[b][0],V[b][1],0.0)
                kviz.set_visible(name,True)

def boilerplate_advance():
    global path,optimizing,planner,roadmap,accumulated_time
    if optimizing or not path:
        t0 = time.time()
        iters = 0
        while time.time() - t0 < stub.max_plan_time:
            planner.planMore(1)
            iters += 1
            if iters >= stub.max_plan_iters:
                break
        t1 = time.time()
        accumulated_time += t1-t0
        V,E = planner.getRoadmap()
        roadmap = V,E
        print len(V),"feasible milestones sampled,",len(E),"edges connected",t1-t0,"s","total time",accumulated_time
        path = planner.getPath()
        if path == None:
            path = []
        else:
            print "Found a path containing",len(path),"milestones"
        refresh_viz()

def boilerplate_keypress(c):
    global field,start,path
    if c=='randomize':
        for i in xrange(10000):
            q = (random.random(),random.random())
            if not any(o.contains(q) for o in field.obstacles):
                break
        start = q
        path = [q]
        refresh_viz()

    
