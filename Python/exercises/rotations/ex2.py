from klampt.math import vectorops,so3,se3
from klampt import vis
import math
import time

def interpolate_linear(a,b,u):
    """Interpolates linearly in cartesian space between a and b."""
    return vectorops.madd(a,vectorops.sub(b,a),u)

def interpolate_rotation(R1,R2,u):
    """Interpolate linearly between the two rotations R1 and R2.
    TODO: the current implementation doesn't work properly.  Why? """
    m1 = so3.moment(R1)
    m2 = so3.moment(R2)
    mu = interpolate_linear(m1,m2,u)
    angle = vectorops.norm(mu)
    axis = vectorops.div(mu,angle)
    return so3.rotation(axis,angle)

def interpolate_transform(T1,T2,u):
    """Interpolate linearly between the two transformations T1 and T2."""
    return (interpolate_rotation(T1[0],T2[0],u),interpolate_linear(T1[1],T2[1],u))


Ra = so3.rotation((0,1,0),math.pi*-0.9)
Rb = so3.rotation((0,1,0),math.pi*0.9)
print "Angle between"
print so3.matrix(Ra)
print "and"
print so3.matrix(Rb)
print "is",so3.distance(Ra,Rb)
Ta = [Ra,[-1,0,1]]
Tb = [Rb,[1,0,1]]


if __name__ == "__main__":
    vis.add("world",se3.identity())
    vis.add("start",Ta)
    vis.add("end",Tb)
    vis.add("interpolated",Ta)
    vis.edit("start")
    vis.edit("end")
    vis.setAttribute("world","length",0.25)
    vis.setAttribute("interpolated","fancy",True)
    vis.setAttribute("interpolated","width",0.03)
    vis.setAttribute("interpolated","length",1.0)

    vis.addText("angle_display","")
    vis.show()
    t0 = time.time()
    while vis.shown():
        #interpolate with a period of 3 seconds
        period = 3.0
        u = ((time.time()-t0)%period)/period
        T = interpolate_transform(Ta,Tb,u)
        #uncomment for question 3 
        #vis.addText("angle_display","Angle to a: %f, b: %f"%(so3.distance(Ta[0],T[0]),so3.distance(Tb[0],T[0])))
        vis.setItemConfig("interpolated",T)
        time.sleep(0.01)

