from klampt.math import vectorops,so3,se3
from klampt import vis
import math
import time

def interpolate_linear(a,b,u):
    """Interpolates linearly in cartesian space between a and b."""
    return vectorops.madd(a,vectorops.sub(b,a),u)

def interpolate_euler_angles(ea,eb,u,convention='zyx'):
    """Interpolates between the two euler angles.
    TODO: The default implementation interpolates linearly.  Can you
    do better?
    """
    return interpolate_linear(ea,eb,u)

def euler_angle_to_rotation(ea,convention='zyx'):
    """Converts an euler angle representation to a rotation matrix.
    Can use arbitrary axes specified by the convention
    arguments (default is 'zyx', or roll-pitch-yaw euler angles).  Any
    3-letter combination of 'x', 'y', and 'z' are accepted.
    """
    axis_names_to_vectors = dict([('x',(1,0,0)),('y',(0,1,0)),('z',(0,0,1))])
    axis0,axis1,axis2=convention
    R0 = so3.rotation(axis_names_to_vectors[axis0],ea[0])
    R1 = so3.rotation(axis_names_to_vectors[axis1],ea[1])
    R2 = so3.rotation(axis_names_to_vectors[axis2],ea[2])
    return so3.mul(R0,so3.mul(R1,R2))

#TODO: play around with these euler angles
ea = (math.pi/4,0,0)
eb = (math.pi*7/4,0,0)
Ta = [euler_angle_to_rotation(ea),[-1,0,1]]
Tb = [euler_angle_to_rotation(eb),[1,0,1]]


def update_interpolation(u):
    #linear interpolation with euler angles
    e = interpolate_euler_angles(ea,eb,u)
    t = interpolate_linear(Ta[1],Tb[1],u)
    return (euler_angle_to_rotation(e),t)


if __name__ == "__main__":
    #draw the world reference frame, the start and the goal, and the interpolated frame
    vis.add("world",se3.identity())
    vis.add("start",Ta)
    vis.add("end",Tb)
    vis.add("interpolated",Ta)
    vis.setAttribute("world","length",0.25)
    vis.setAttribute("interpolated","fancy",True)
    vis.setAttribute("interpolated","width",0.03)
    vis.setAttribute("interpolated","length",1.0)
    period = 3.0
    t0 = time.time()
    vis.show()
    while vis.shown():
        #repeat the interpolation every 3 seconds
        u = ((time.time()-t0)%period)/period
        T = update_interpolation(u)
        vis.setItemConfig("interpolated",T)
        time.sleep(0.01)
    vis.kill()
