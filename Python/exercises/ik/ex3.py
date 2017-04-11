from klampt import *
from klampt import vis
from klampt.model import coordinates
import time
import math
#import numpy as np


def solve_ik(robotlink,localpos,worldpos):
    """IMPLEMENT ME: solve inverse kinematics to place the 3D point
    localpos on robotlink (a RobotLink instance) at the 3D position
    worldpos in the world coordinate frame.
        
    Returns the robot configuration that solves the IK problem.
    """
    linkindex = robotlink.index
    robot = robotlink.robot()
    #hint: your code should look like this
    #obj = ik.objective(robotlink,...)
    # # In the ... you should do something to set up the objective so
    # # that the point localpos on the link is matched to worldpos.
    # # See klampt/ik.py for more details.
    #s = ik.solver(obj)
    # # Set up some parameters for the numerical solver
    #maxIters = 100
    #tol = 1e-3
    # # Optionally you can set an initial configuration like so:
    # # robot.setConfig([0]*robot.numLinks())
    # # or set a random initial configuration like so:
    # # s.sampleInitial()
    #(res,iters) = s.solve(maxIters,tol);
    #return robot.getConfig()
    
    #right now this just sets the zero configuration
    q = [0]*robot.numLinks()

    #Implementation
    obj = coordinates.ik_objective(coordinates.getPoint("ik-constraint-local"),coordinates.getPoint("ik-constraint-world"))
    s = IKSolver(robot)
    s.add(obj)

    maxIters = 100
    tol = 1e-3
    # # Optionally you can set an initial configuration like so:
    # # robot.setConfig([0]*robot.numLinks())
    # # or set a random initial configuration like so:
    #s.sampleInitial()
    (res,iters) = s.solve(maxIters,tol);
    return robot.getConfig()
    return q

      
if __name__ == "__main__":
    world = WorldModel()
    res = world.readFile("ex2_file.xml")
    if not res: raise RuntimeError("Unable to load world file")

    linkindex = 7
    localpos = (0.17,0,0)
    robot = world.robot(0)
    link = robot.link(linkindex)

    coordinates.setWorldModel(world)
    goalpoint = [0,0,0]
    ptlocal = coordinates.addPoint("ik-constraint-local",localpos,robot.getName()+":"+link.getName())
    ptworld = coordinates.addPoint("ik-constraint-world",goalpoint,"world")
    print coordinates.manager().frames.keys()
    
    vis.add("robot",robot)
    vis.add("coordinates",coordinates.manager())
    vis.show()
    iteration = 0
    while vis.shown():
        vis.lock()
        #set the desired position goalpoint to move in a circle
        r = 0.4
        t = vis.animationTime()
        goalpoint[0],goalpoint[1],goalpoint[2] = 0.8,r*math.cos(t),0.7+r*math.sin(t)
        q = solve_ik(link,localpos,goalpoint)
        robot.setConfig(q)
        #this updates the coordinates module
        coordinates.updateFromWorld()
        
        vis.unlock()
        time.sleep(0.05)
        iteration += 1
    #terminate smoothly
    vis.kill()
