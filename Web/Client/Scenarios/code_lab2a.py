from klampt import *
from klampt.math import vectorops,so3,se3
import math

def lab2a(robot_model,q,ee_link,ee_local_position,target):
    """In:
    - robot_model: a RobotModel instance.
    - q: a configuration of the robot (an n-element list of floats where n=robot_model.numLinks()).
    - ee_link: the index of the end effector
    - ee_local_position: the position of the end effector (ex,ey,ez), expressed
      in the frame of link ee_link.
    - target: a 3d target position (tx,ty,tz)
    Out:
    - The distance between the end effector's *world* position and target

    You will need to study the documentation for the RobotModel and RobotModelLink
    classes.

    Don't forget that the robot_model's current configuration and the layout
    of its frames are considered temporary variables.  Before lab2a is called,
    you have no control over the model's current configuration, and anything after this
    will disregard where the robot is placed.  In other words, q is the authoritative
    representation of the robot's configuration, and robot_model is a "scratch pad".
    """
    #TODO: put your code here
    ee_world_position = ee_local_position
    return vectorops.distance(ee_world_position,target)

def target_motion(t):
    return (math.sin(t),math.cos(t*0.5),1.0+math.sin(t*0.7+0.5))
