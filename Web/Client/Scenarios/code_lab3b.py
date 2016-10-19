from klampt import *
from klampt import vectorops
from klampt.cspace import CSpace,MotionPlan

#turn this to False if redrawing becomes too slow
draw_roadmap = True
#how much planning time to take every frame, in seconds
max_plan_time = 0.5
#how many iterations to take per frame
max_plan_iters = 100

class Circle:
    def __init__(self,x=0,y=0,radius=1):
        self.center = (x,y)
        self.radius = radius
        
    def contains(self,point):
        return (vectorops.distance(point,self.center) <= self.radius)

    def distance(self,point):
        return (vectorops.distance(point,self.center) - self.radius)

class CircleObstacleCSpace(CSpace):
    """The configuration space being used in Lab3B, C, and D.
    Consists of a circular robot and circular obstacles.
    """
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
        """TODO: Implement this feasibility test.  It is used by the motion planner to
        determine whether the robot at configuration q is feasible."""
        #bounds test
        if not CSpace.feasible(self,q): return False
        #make sure center point at least distance r from obstacles
        for o in self.obstacles:
            if o.contains(q): return False
        return True


def makePlanner(space, start, goal):
    """Creates a MotionPlan object for the given space, start, and goal.
    Returns (planner,optimizing) where optimizing is True if the planner should
    continue be run after the first solution path has been found"""
    #TODO: In lab3c, you should tune these parameters
    #
    #This sets a Probabilistic Road Map (PRM) planner that connects
    #a random point to its 10 nearest neighbors. If knn is set to 0,
    #the points are connected as long as they lie
    #within distance 0.1 of one another
    MotionPlan.setOptions(type="prm",knn=10,connectionThreshold=0.1)
    #This line sets a Rapidly-exploring Random Tree (RRT) planner that
    #repeatedly extends the tree toward a random point at maximum
    #distance 0.25.  It uses the bidirectional=True option, which grows
    #trees from both the start and the goal
    #MotionPlan.setOptions(type="rrt",connectionThreshold=0.1,perturbationRadius=0.25,bidirectional=True)
    optimizing = False

    #Optimizing planners, for use in Lab3C, part 2.  Make sure to uncomment optimizing = True below.
    #This sets the PRM algorithm with shortcutting
    #MotionPlan.setOptions(type="prm",knn=10,connectionThreshold=0.1,shortcut=True)
    #This sets the RRT* algorithm
    #MotionPlan.setOptions(type="rrt*",connectionThreshold=0.1,perturbationRadius=0.25)
    #This sets a fast-marching method algorithm
    #MotionPlan.setOptions(type="fmm*")
    #This sets a random-restart + shortcutting RRT
    #MotionPlan.setOptions(type="rrt",connectionThreshold=0.1,perturbationRadius=0.25,bidirectional=True,restart=True,shortcut=True)
    #optimizing = True

    #create the planner and return it along with the termination criterion
    planner = MotionPlan(space)
    return planner,optimizing

def start():
    return (0.06,0.6)

def target():
    return (0.94,0.5)

def obstacles():
    return [Circle(0.5,0.5,0.36)]
