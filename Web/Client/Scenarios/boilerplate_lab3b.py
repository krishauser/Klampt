import pkg_resources
pkg_resources.require("klampt==0.6.2")
from klampt import *
import math
import sys
sys.path.append("Web/Server")
import kviz
import random
from klampt.cspace import CSpace,MotionPlan
from klampt import vectorops

world = WorldModel()
space = None
start = None
target = None
path = None
roadmap = None
planner = None
optimizing = False
existing_path_lines = []
existing_roadmap_lines = set()

class Circle:
    def __init__(self,x=0,y=0,radius=1):
        self.center = (x,y)
        self.radius = radius
        
    def contains(self,point):
        return (vectorops.distance(point,self.center) <= self.radius)

    def distance(self,point):
        return (vectorops.distance(point,self.center) - self.radius)

class CircleObstacleCSpace(CSpace):
    def __init__(self):
        CSpace.__init__(self)
        #set bounds
        self.bound = [(0.0,1.0),(0.0,1.0)]
        #set collision checking resolution
        self.eps = 1e-3
        #setup a robot with radius 0.05
        self.robot = Circle(0,0,0.05)
        #set obstacles here
        self.obstacles = []

    def addObstacle(self,circle):
        self.obstacles.append(circle)
    
    def feasible(self,q):
        #bounds test
        if not CSpace.feasible(self,q): return False
        r = self.robot.radius
        #bounds can be checked by bounds-testing lower left and upper left
        #point in a square around robot
        if not CSpace.feasible(self,vectorops.add(q,(-r,-r))): return False
        if not CSpace.feasible(self,vectorops.add(q,(r,r))): return False
        #an alternative would just check if q[i]-r >= 0 and q[i]+1 <= 1
        #for i in [0,1]
        #make sure center point at least distance r from obstacles
        for o in self.obstacles:
            if o.distance(q) <= r: return False
        return True


def makePlanner(space,start,goal):
    """Creates a MotionPlan object for the given space, start, and goal"""
    #TODO: In lab3c, you should tune these parameters
    #
    #This sets a Probabilistic Road Map (PRM) planner that connects
    #a random point to its 10 nearest neighbors. If knn is set to 0,
    #the points are connected as long as they lie
    #within distance 0.1 of one another
    #MotionPlan.setOptions(type="prm",knn=10,connectionThreshold=0.01)
    #This line sets a Rapidly-exploring Random Tree (RRT) planner that
    #repeatedly extends the tree toward a random point at maximum
    #distance 0.25.  It uses the bidirectional=True option, which grows
    #trees from both the start and the goal
    MotionPlan.setOptions(type="rrt",connectionThreshold=0.1,perturbationRadius=0.25,bidirectional=True)
    planner = MotionPlan(space)
    planner.setEndpoints(start,goal)
    return planner

def makeOptimizingPlanner(space,start,goal):
    """Creates an incrementally-optimizing MotionPlan object for the
    given space, start, and goal"""
    #TODO: In Problem 4, you should play with these parameters
    #
    #This sets the PRM algorithm with shortcutting
    MotionPlan.setOptions(type="prm",knn=10,connectionThreshold=0.1,shortcut=True)
    #This sets the RRT* algorithm
    #MotionPlan.setOptions(type="rrt*")
    #This sets a fast-marching method algorithm (NOTE: use [space] rather than
    #p to plan, each iteration runs an entire grid search)
    #MotionPlan.setOptions(type="fmm*")
    planner = MotionPlan(space)
    planner.setEndpoints(start,goal)
    return planner

    
def boilerplate_start():
    global world,space,start,target,planner,path,existing_path_lines,roadmap,existing_roadmap_lines

    start=stub.start()
    target=stub.target()

    space = CircleObstacleCSpace()
    for o in stub.obstacles():
        space.addObstacle(o)

    #TODO: in Problem 4 you should switch the commented-out lines to
    #decide whether you want a feasible planner or an optimizing planner
    planner = makePlanner(space,start,target)
    optimizing = False
    #planner = makeOptimizingPlanner(space,start,target)
    #optimizing = True
    
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
    radius = space.robot.radius
    kviz._init(world)
    kviz.add_sphere("robot",start[0],start[1],0,radius)
    kviz.add_sphere("target",target[0],target[1],0,radius)
    kviz.set_color("robot",[0,0,1,1])
    kviz.set_color("target",[1,0,0,1])
    existing_roadmap_lines = set()
    existing_path_lines = []
    refresh_viz()

def refresh_viz():
    global start,target,path,existing_path_lines,roadmap,existing_roadmap_lines
    kviz.update_sphere("robot",start[0],start[1],0)
    kviz.update_sphere("target",target[0],target[1],0)
    for i,name in enumerate(existing_path_lines):
        if i >= len(path):
            kviz.set_visible(name,False)
    for i in xrange(len(path)-1):
        if i >= len(existing_path_lines):
            name = "path"+str(i)
            existing_path_lines.append(name)
            kviz.add_line(name,path[i][0],path[i][1],0.0,path[i+1][0],path[i+1][1],0.0)
            kviz.set_visible(name,True)
            kviz.set_color(name,[0,0,1,1])
        else:
            name = existing_path_lines[i]
            #kviz.update_line(name,path[i][0],path[i][1],0.0,path[i+1][0],path[i+1][1],0.0)
    V,E = roadmap
    for i,e in enumerate(E):
        name = "edge"+str(i)
        a,b = e
        if name not in existing_roadmap_lines:
            existing_roadmap_lines.add(name)
            kviz.add_line(name,V[a][0],V[a][1],0.0,V[b][0],V[b][1],0.0)
            kviz.set_color(name,[1,1,0,1])
        else:
            kviz.update_line(name,V[a][0],V[a][1],0.0,V[b][0],V[b][1],0.0)
            kviz.set_visible(name,True)

def boilerplate_advance():
    global path,optimizing,planner,roadmap
    if optimizing or not path:
        planner.planMore(100)
        V,E = planner.getRoadmap()
        roadmap = V,E
        print len(V),"feasible milestones sampled,",len(E),"edges connected"
        path = planner.getPath()
        if path == None:
            path = []
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

    
