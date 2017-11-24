#!/usr/bin/python
import sys
sys.path.append("Web/Server")
import time
from klampt import *
from klampt.math import *
import math
import kviz

#stub code
def interpolate_linear(a,b,u):
    """Interpolates linearly in cartesian space between a and b."""
    pass

#stub code
def interpolate_euler_angles(ea,eb,u,convention='zyx'):
    """Interpolates between the two euler angles.
    TODO: The default implementation interpolates linearly.  Can you
    do better?
    """
    pass

#stub code
def euler_angle_to_rotation(ea,convention='zyx'):
    """Converts an euler angle representation to a rotation matrix.
    Can use arbitrary axes specified by the convention
    arguments (default is 'zyx', or roll-pitch-yaw euler angles).  Any
    3-letter combination of 'x', 'y', and 'z' are accepted.
    """
    pass

#TODO: play around with these euler angles
Ta,Tb = None,None
ttotal = 0
period = 3.0
manual = False

def make_frame_widget(name,size,opacity=1.0):
    world = kviz._world
    x = world.makeRigidObject(name+"_x")
    y = world.makeRigidObject(name+"_y")
    z = world.makeRigidObject(name+"_z")
    x.geometry().loadFile(__KLAMPT_DIR__+"data/objects/cube.off")
    y.geometry().loadFile(__KLAMPT_DIR__+"data/objects/cube.off")
    z.geometry().loadFile(__KLAMPT_DIR__+"data/objects/cube.off")
    w = size*0.1
    x.geometry().transform([size,0,0,0,w,0,0,0,w],[0,-w*0.5,-w*0.5])
    y.geometry().transform([w,0,0,0,size,0,0,0,w],[-w*0.5,0,-w*0.5])
    z.geometry().transform([w,0,0,0,w,0,0,0,size],[-w*0.5,-w*0.5,0])
    x.appearance().setColor(1,0,0,1)
    y.appearance().setColor(0,1,0,1)
    z.appearance().setColor(0,0,1,1)

def set_frame_widget_xform(name,T):
    world = kviz._world
    world.rigidObject(name+"_x").setTransform(*T)
    world.rigidObject(name+"_y").setTransform(*T)
    world.rigidObject(name+"_z").setTransform(*T)

def update_interpolation(u):
    global Ta,Tb
    R = stub.do_interpolate(u)
    t = stub.interpolate_linear(Ta[1],Tb[1],u)
    return (R,t)

def boilerplate_start():
    global Ta,Tb,ttotal
    Ta = (stub.euler_angle_to_rotation(stub.ea),(-1,0,0.5))
    Tb = (stub.euler_angle_to_rotation(stub.eb),(1,0,0.5))
    ttotal = 0
    make_frame_widget("start_frame",0.3,opacity=0.5)
    make_frame_widget("goal_frame",0.3,opacity=0.5)
    make_frame_widget("current_frame",0.6)
    set_frame_widget_xform("start_frame",Ta)
    set_frame_widget_xform("goal_frame",Tb)
    set_frame_widget_xform("current_frame",Ta)

def boilerplate_advance():
    #interpolate with a period of 3 seconds
    global ttotal
    if not manual:
        ttotal += 0.02
        u = (ttotal%period)/period
        T = update_interpolation(u)
        set_frame_widget_xform("current_frame",T)


def boilerplate_setitem(name,value):
    global ttotal,manual
    if name == 'manual':
        manual = value
    if manual:
        #allow control
        if name == 'u':
            ttotal = float(value)/100*period
        elif name == 'psi1' and manual:
            stub.ea[0] = math.radians(float(value))
            print "ea is now",stub.ea
        elif name == 'theta1':
            stub.ea[1] = math.radians(float(value))
            print "ea is now",stub.ea
        elif name == 'phi1':
            stub.ea[2] = math.radians(float(value))
            print "ea is now",stub.ea
        elif name == 'psi2':
            stub.eb[0] = math.radians(float(value))
            print "eb is now",stub.eb
        elif name == 'theta2':
            stub.eb[1] = math.radians(float(value))
            print "eb is now",stub.eb
        elif name == 'phi2':
            stub.eb[2] = math.radians(float(value))
            print "eb is now",stub.eb
        global Ta,Tb
        Ta = (stub.euler_angle_to_rotation(stub.ea),(-1,0,0.5))
        Tb = (stub.euler_angle_to_rotation(stub.eb),(1,0,0.5))
        set_frame_widget_xform("start_frame",Ta)
        set_frame_widget_xform("goal_frame",Tb)
        ttotal += 0.02
        u = (ttotal%period)/period
        T = update_interpolation(u)
        set_frame_widget_xform("current_frame",T)