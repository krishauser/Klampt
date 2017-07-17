from klampt import *
from klampt.math import vectorops,so3,se3
import math

def lab2b(L1,L2,L3,point):
    """
    Compute all IK solutions of a 3R manipulator.  The joint axes in the reference configuration
    are ZYY, with each link's origin displaced by a given amount on the X axis.
    In:
    - L1, L2, L3: the link lengths
    - point: the target position (x,y,z)
    Out:
    - A pair (n,solutionList) where n is the number of solutions (either 0,
      1, 2, 4, or float('inf')) and solutionList is a list of all solutions.
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
    #this line would return four (incorrect) solutions (0,0,0)
    return (4,[(0,0,0),(0,0,0),(0,0,0),(0,0,0)])
    #this line would return infinite solutions, and one example (incorrect) solution (0,0,0)
    return (float('inf'),[(0,0,0)])

def ik_goal_motion(t):
    """Returns a point describing where the goal should be at time t"""
    return (math.sin(t)*1.5+0.3,1.0*math.cos(t/2+0.5), abs((t % 3)*0.2-0.5 ) )

