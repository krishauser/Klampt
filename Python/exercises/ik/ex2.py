from klampt import *
from klampt.model import ik
from klampt import vis 
from klampt.math import vectorops,so3
import math
import time


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
    #maxIters = 100
    #tol = 1e-3
    #s = ik.solver(obj,maxIters,tol)
    # # Set up some parameters for the numerical solver
    # # Optionally you can set an initial configuration like so:
    # # robot.setConfig([0]*robot.numLinks())
    # # or set a random initial configuration like so:
    # # s.sampleInitial()
    #res = s.solve();
    #if not res: print "Couldn't solve IK problem"
    #return robot.getConfig()

    #right now this just sets the zero configuration
    q = [0]*robot.numLinks()
    return q
    
    

if __name__ == "__main__":
    world = WorldModel()
    res = world.readFile("ex2_file.xml")
    if not res: raise RuntimeError("Unable to load world file")
    

    robot = world.robot(0)
    link = robot.link(7)
    #coordinates of the constrained IK point on the robot, in the link's local frame
    localpos = [0.17,0,0]
    #coordinates of the IK goal in world coordinates
    position = [0,0,0]
    t0 = time.time()

    #draw the world, local point in yellow, and target point in white
    vis.add("world",world)
    vis.add("local_point",position)
    vis.setColor("local_point",1,1,0)
    vis.add("target",position)
    vis.setColor("target",1,1,1)
    vis.edit("target")
    vis.show()
    while vis.shown():
        vis.lock()
        #move the target and solve
        t = time.time()-t0
        r = 0.4
        position = vis.getItemConfig("target")
        position[1] = r*math.cos(t)
        position[2] = 0.7+r*math.sin(t)
        q = solve_ik(link,localpos,position)
        robot.setConfig(q)
        vis.setItemConfig("local_point",link.getWorldPosition(localpos))
        vis.setItemConfig("target",position)
        vis.unlock()
        time.sleep(0.01)
    vis.kill()