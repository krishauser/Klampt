from klampt import *
from klampt import vectorops

#attractive force constant
attractiveConstant = 100
#repulsive distance
repulsiveDistance = 0.1
#time step to limit the distance traveled
timeStep = 0.01

class Circle:
    def __init__(self,x=0,y=0,radius=1):
        self.center = (x,y)
        self.radius = radius
        
    def contains(self,point):
        return (vectorops.distance(point,self.center) <= self.radius)

    def distance(self,point):
        return (vectorops.distance(point,self.center) - self.radius)

def force(q,target,obstacles):
    """Returns the potential field force for a robot at configuration q,
    trying to reach the given target, with the specified obstacles.

    Input:
    - q: a 2D point giving the robot's configuration
    - robotRadius: the radius of the robot
    - target: a 2D point giving the robot's target
    - obstacles: a list of Circle's giving the obstacles
    """
    #basic target-tracking potential field implemented here
    #TODO: implement your own potential field
    f = vectorops.mul(vectorops.sub(target,q),attractiveConstant)
    #limit the norm of f
    if vectorops.norm(f) > 1:
        f = vectorops.unit(f)
    return f

def start():
    return (0.06,0.6)

def target():
    return (0.94,0.5)

def obstacles():
    return [Circle(0.5,0.25,0.2),
        Circle(0.5,0.75,0.2)]
