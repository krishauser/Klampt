from klampt import *
from klampt.math import vectorops,so3,se3
import math
import sys
sys.path.append("Web/Server")
import kviz
import random

world = WorldModel()
field = None
start = None
path = None
existing_path_lines = []

class Circle:
    def __init__(self,x=0,y=0,radius=1):
        self.center = (x,y)
        self.radius = radius
        
    def contains(self,point):
        return (vectorops.distance(point,self.center) <= self.radius)

    def distance(self,point):
        return (vectorops.distance(point,self.center) - self.radius)

class ObstacleAvoidingPotentialField:
    def __init__(self):
        #set target here
        self.target = (0.5,0.5)
        #set obstacles using addObstacle
        self.obstacles = []
        #time step for applying force
        self.timeStep = 0.01

    def addObstacle(self,circle):
        self.obstacles.append(circle)
    
    def nextStep(self,q):
        """Returns the next configuration in the path when stepped along the
        potential field"""
        f = self.force(q)
        return vectorops.madd(q,f,self.timeStep)

    def force(self,q):
        return stub.force(q,self.target,self.obstacles)


def boilerplate_start():
    global world,field,start,path,existing_path_lines
    world = WorldModel()
    field = ObstacleAvoidingPotentialField()
    for o in stub.obstacles():
	    field.addObstacle(o)
    start=stub.start()
    field.target=stub.target()
    path = [start]
    existing_path_lines = []
    for o in field.obstacles:
        obst = world.loadElement(__DIR__+"../cylinder.tri")
        if obst < 0:
            raise IOError("Unable to load cylinder, check the path")
        obst = world.terrain(world.numTerrains()-1)
        obst.geometry().transform([o.radius,0,0,0,o.radius,0,0,0,0.05],[o.center[0],o.center[1],0])
        obst.appearance().setColor(0.3,0.3,0.3,1)
    radius = 0.02
    kviz._init(world)
    kviz.add_sphere("robot",start[0],start[1],0,radius)
    kviz.add_sphere("target",field.target[0],field.target[1],0,radius)
    kviz.set_color("robot",0,0,1)
    kviz.set_color("target",1,0,0)
    refresh_viz()

def refresh_viz():
    global start,field,path,existing_path_lines
    kviz.update_sphere("robot",start[0],start[1],0)
    kviz.update_sphere("target",field.target[0],field.target[1],0)
    for i in xrange(len(path)-1):
        if i >= len(existing_path_lines):
            name = "path"+str(i)
            existing_path_lines.append(name)
            kviz.add_line(name,path[i][0],path[i][1],0.0,path[i+1][0],path[i+1][1],0.0)
            kviz.set_visible(name,True)
            kviz.set_color(name,1,1,0)
        else:
            name = existing_path_lines[i]
            #kviz.update_line(name,path[i][0],path[i][1],0.0,path[i+1][0],path[i+1][1],0.0)

def boilerplate_advance():
    global start,field,path
    path.append(field.nextStep(start))
    start = path[-1]
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

    
