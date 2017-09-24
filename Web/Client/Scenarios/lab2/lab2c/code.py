from klampt import *
from klampt.math import vectorops,so3,se3
import math

def lab2c(robot,qseed,ee_link,ee_local_position,ee_local_axis,target,target_axis):
    """In:
    - robot: a RobotModel instance
    - qseed: a seed configuration of the robot.
    - ee_link: the index of the end effector's link
    - ee_local_position: the position of the end effector, expressed in the frame
      of link ee_link.
    - ee_local_axis: the rotational axis of the end effector, expressed in
      the frame of link ee_link.
    - target: a 3d target position (x,y,z)
    - target_axis: a 3d target direction for the end effector's axis (x,y,z)
    Out: a triple (q,errpos,erraxis) giving
    - q: the inverse kinematics optimized configuration of the robot, which (locally)
      minimizes the distance between the end effector position/axis and the target position/axis
    - errpos: The final distance between the end effector's world position
      and the target position. (should be as close as possible to 0 given a
      successful solution)
    - erraxis: The final distance between the end effector's world axis and
      the target axis.

    You will need to examine the documentation of the klampt.ik module, specifically
    the ik.objective and ik.solve functions.
    """
    #TODO: put your code here
    ##HINT: set up an IK objective as follows to strictly constrain the link's
    ##position and orientation
    #goal = ik.objective(robot.link(ee_link),R=desired_rotation_of_link,t=desired_translation_of_link)
    ##OR if you want to match some points lp1,lp2,..., given in the local
    ##coordinate frame to points wp1,wp2,..., given in the global coordinate
    ##frame
    #goal = ik.objective(robot.link(ee_link),local=[lp1,lp2,...],world=[wp1,wp2,...,])
    ##now seed the ik solver with the correct configuration (READ THE DOCS)...
    #???
    ##now solve
    #if ik.solve(goal):
    #    print "IK success!"
    ##   now the robot model's configuration is changed, and you need to
    ##   extract it. Your errpos and erraxis values should be very small
    #else:
    #    print "IK failure... returning best solution found"
    ##   the robot model's configuration is changed, but your errpos and
    ##   erraxis will not be close to 0

    ee_world_position = ee_local_position
    ee_world_direction = ee_local_axis
    return (qseed,vectorops.distance(ee_world_position,target),vectorops.distance(ee_world_direction,target_axis))

def pick_ik_seed(robot):
    """Returns an IK seed for use in the local IK solver"""
    q = [0.0]*robot.numLinks()
    qmin,qmax = robot.getJointLimits()
    #TODO: see the difference when a random configuration is picked as seed
    #for i in range(ee.link):
    #    q[i] = random.uniform(qmin[i],qmax[i])
    return q

def target_motion(t):
    return (0.5*math.sin(t),0.7*math.cos(t*0.5),0.7+0.4*math.sin(t*0.7+0.5))

def target_axis_motion(t):
    """Returns the axis that should be met at time t"""
    return (0,0,1)