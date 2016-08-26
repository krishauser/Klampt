from klampt import *
import math

def lab2b(L1,L2,L3,point,angle):
    """
    Compute all IK solutions of a 3R planar manipulator
    In:
    - L1, L2, L3: the link lengths
    - point: the target position (x,y)
    - angle: the target angle of the third link (in radians)
    Out:
    - A pair (n,solutionList) where n is the number of solutions (either 0,
      1, 2, or float('inf')) and solutionList is a list of all solutions.
      In the n=0 case, it should be an empty list [], and in the n=inf case
      it should give one example solution.

      Each solution is a 3-tuple giving the joint angles, in radians.
    """
    #this returns no solution
    return (0,[])
    #this line would return one (incorrect) solution (0,0,0)
    return (1,[(0,0,0)])
    #this line would return two (incorrect) solutions (0,0,0)
    return (2,[(0,0,0),(0,0,0)])
    #this line would return infinite solutions, and one example (incorrect) solution (0,0,0)
    return (float('inf'),[(0,0,0)])

def ik_goal_motion(t):
    """Returns a point,angle pair describing where the goal should be at time t"""
    return ((math.sin(t)*2.5+0.3,3.0*math.cos(t/2+0.5)),t / 3)
