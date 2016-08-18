from klampt import *
import math
import sys
sys.path.append("Web/Server")
import kviz
import random

world = WorldModel()
car_body = None
#fl, fr, bl, br
wheels = {'fl':None,'fr':None,'bl':None,'br':None}
velocity = 0
steer = 0
vinc = 2
vmin = -100
vmax = 100
steermax = 30
steerinc = 2
xform = [0,0,0]

def boilerplate_start():
    global world
    car_body = world.loadElement("Web/Client/Scenarios/lab1/body.obj")
    wheels['fl'] = world.loadElement("Web/Client/Scenarios/lab1/tire.obj")
    wheels['fr'] = world.loadElement("Web/Client/Scenarios/lab1/tire.obj")
    wheels['bl'] = world.loadElement("Web/Client/Scenarios/lab1/tire.obj")
    wheels['br'] = world.loadElement("Web/Client/Scenarios/lab1/tire.obj")
    kviz._init(world)

def update_2d_xform(rigidObject,xform2d):
    """Given an RigidObjectModel and a 2D transform (x,y,theta), sets the object's 3D transform
    while keeping height constant"""
    xform3d = rigidObject.getTransform()
    R,t = xform3d
    x,y,theta = xform2d
    t = [x,y,t[2]]
    R = so3.rotation([0,0,1],theta)
    rigidObject.setTransform(R,t)

def update_car():
    global xform,steer
    steeringAngle = math.radians(steer)
    frontTirePos1 = (0.30,0.09)
    frontTirePos2 = (0.30,-0.09)
    rearTirePos1 = (-0.30,0.09)
    rearTirePos2 = (-0.30,0-.09)
    rearPoint1 = stub.apply_xform(xform,rearTirePos1)
    rearPoint2 = stub.apply_xform(xform,rearTirePos2)
    update_2d_xform(wheels['bl'],(rearPoint1[0],rearPoint1[1],xform[2]))
    update_2d_xform(wheels['br'],(rearPoint2[0],rearPoint2[1],xform[2]))
    tire1xform = stub.lab1c(xform,(frontTirePos1[0],frontTirePos1[1],steeringAngle))
    tire2xform = stub.lab1c(xform,(frontTirePos2[0],frontTirePos2[1],steeringAngle))
    update_2d_xform(wheels['fl'],tire1xform)
    update_2d_xform(wheels['fr'],tire2xform)
    update_2d_xform(car_body,xform)
    return

def update_xform(dt=0.02):
    global xform,velocity,steer
    rangle = math.radians(xform[2])
    v = [velocity*math.cos(rangle),velocity*math.sin(rangle)]
    dtheta = velocity*steer*0.05
    xform = (xform[0]+v[0]*dt,xform[1]+v[1]*dt,xform[2]+dtheta*dt)

def boilerplate_advance():
    update_xform()
    update_car()
    boilerplate_keypress(random.choice(['up','down','left','right']))

def boilerplate_keypress(c):
    global velocity,steer,xform
    if c=='up':
        velocity += vinc
        if velocity > vmax:
            velocity = vmax
    elif c=='down':
        velocity -= vinc
        if velocity < vmin:
            velocity = vmin
        print velocity
    elif c=='right':
        steer -= steerinc
        if steer < -steermax:
            steer = -steermax
    elif c=='left':
        steer += steerinc
        if steer > steermax:
            steer = steermax
    
