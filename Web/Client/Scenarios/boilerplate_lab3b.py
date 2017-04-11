import pkg_resources
pkg_resources.require("klampt==0.6.2")
from klampt import *
from klampt.math import vectorops,so3,se3
from klampt.cspace import MotionPlan
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
path_height = 0.002
accumulated_time = 0

class Circle:
    def __init__(self,x=0,y=0,radius=1):
        self.center = (x,y)
        self.radius = radius
        
    def contains(self,point):
        return (vectorops.distance(point,self.center) <= self.radius)

    def distance(self,point):
        return (vectorops.distance(point,self.center) - self.radius)


    
def boilerplate_start():
    global world,space,start,target,planner,optimizing,path,existing_path_lines,existing_ghosts,roadmap,existing_roadmap_lines
    global accumulated_time
    accumulated_time = 0

    #global options that don't get forgotten from instance to instance
    MotionPlan.setOptions(restart=False,shortcut=False)

    start=stub.start()
    target=stub.target()

    space = stub.CircleObstacleCSpace()
    for o in stub.obstacles():
        space.addObstacle(o)

    #TODO: in Problem 4 you should switch the commented-out lines to
    #decide whether you want a feasible planner or an optimizing planner
    planner,optimizing = stub.makePlanner(space,start,target)
    planner.setEndpoints(start,target)
    
    path = []
    roadmap = [],[]
    world = WorldModel()
    for o in space.obstacles:
        obst = world.loadElement("Web/Client/Scenarios/lab3/cylinder.tri")
        if obst < 0:
            raise IOError("Unable to load cylinder, check the path")
        obst = world.terrain(world.numTerrains()-1)
        obst.geometry().transform([o.radius,0,0,0,o.radius,0,0,0,0.05],[o.center[0],o.center[1],0])
        obst.appearance().setColor(0.3,0.3,0.3,1)
    #make a box around everything
    world.loadElement("Web/Client/Scenarios/lab3/cube.tri")
    wall = world.terrain(world.numTerrains()-1)
    wall.geometry().transform([0.01,0,0,0,1,0,0,0,0.05],[-0.01,0,0])
    world.loadElement("Web/Client/Scenarios/lab3/cube.tri")
    wall = world.terrain(world.numTerrains()-1)
    wall.geometry().transform([0.01,0,0,0,1,0,0,0,0.05],[1,0,0])
    world.loadElement("Web/Client/Scenarios/lab3/cube.tri")
    wall = world.terrain(world.numTerrains()-1)
    wall.geometry().transform([1,0,0,0,0.01,0,0,0,0.05],[0,-0.01,0])
    world.loadElement("Web/Client/Scenarios/lab3/cube.tri")
    wall = world.terrain(world.numTerrains()-1)
    wall.geometry().transform([1,0,0,0,0.11,0,0,0,0.05],[0,1,0])
    
    radius = space.robot.radius
    kviz._init(world)
    kviz.add_sphere("robot",start[0],start[1],0,radius)
    kviz.add_sphere("target",target[0],target[1],0,radius)
    kviz.set_color("robot",0,0,1)
    kviz.set_color("target",1,0,0)
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
            existing_ghosts.append("ghost"+str(i))
            kviz.add_sphere("ghost"+str(i),path[i][0],path[i][1],0,space.robot.radius)
            kviz.set_color("ghost"+str(i),1,1,1,0.5)
        else:
            name = existing_path_lines[i]
            kviz.set_visible(name,True)
            kviz.update_line(name,path[i][0],path[i][1],path_height,path[i+1][0],path[i+1][1],path_height)
            kviz.update_sphere("ghost"+str(i),path[i][0],path[i][1],0,space.robot.radius)
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

    
