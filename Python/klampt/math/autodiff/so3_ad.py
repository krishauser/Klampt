"""so3 module AD functions:

 ====================  ============= 
 Function              Derivative    
 ====================  ============= 
 identity              N/A
 apply                 Y
 mul                   Y
 inv                   Y
 trace                 Y
 rpy                   N
 from_rpy              1
 rotation_vector       1
 from_rotation_vector  1
 axis                  N
 angle                 1
 from_axis_angle       1
 quaternion            N
 from_quaternion       N
 distance              N
 error                 1
 cross_product         N
 canonical             N
 interpolate           N
 det                   N
 ====================  ============= 

"""

import numpy as np 
from .ad import ADFunctionInterface,function
from . import math_ad
from .. import vectorops,so3
import math

identity = so3.identity
inv = function(so3.inv,'so3.inv',(9,),9,
        jvp=[lambda dR,R:so3.inv(dR)],order=1)
mul = function(so3.mul,'so3.mul',(9,9),9,['R1','R1'],
        jvp=[lambda dR1,R1,R2:so3.mul(dR1,R2),lambda dR2,R1,R2:so3.mul(R1,dR2)],order=2)
apply = function(so3.apply,'so3.apply',(9,3),3,['R','x'],
        jvp=[lambda dR,R,x:so3.apply(dR,x),lambda dx,R,x:so3.apply(R,dx)],order=2)
trace = function(so3.trace,'so3.trace',(9,),1,
        jvp=[lambda dR,R:so3.trace(dR)],order=1)

def from_rpy_jvp(drpy,rpy):
    roll,pitch,yaw = rpy
    droll,dpitch,dyaw = drpy
    Rx,Ry,Rz = from_axis_angle(((1,0,0),roll)),from_axis_angle(((0,1,0),pitch)),from_axis_angle(((0,0,1),yaw))
    wx = so3.cross_product([droll,0,0])
    wy = so3.cross_product([0,dpitch,0])
    wz = so3.cross_product([0,0,dyaw])
    Ryx = so3.mul(Ry,Rx)
    return vectorops.add(so3.mul(so3.mul(wz,Rz),Ryx), so3.mul(Rz,so3.mul(so3.mul(wy,Ryx))), so3.mul(so3.mul(Rz,Ry),so3.mul(wx,Rx)))
from_rpy = function(so3.from_rpy,'so3.from_rpy',(3,),9,
    jvp=[from_rpy_jvp])
rpy = function(so3.rpy,'so3.rpy',(9,),3)

def from_rotation_vector_jvp(dw,w):
    length = np.linalg.norm(w)
    dlength = math_ad.norm_jvp(dw,w)
    if length < 1e-7: return so3.cross_product(dw)
    axis = w/length
    daxis = math_ad.unit_jvp(dw,w)
    return from_axis_angle_jvp_axis(daxis,axis,length) + from_axis_angle_jvp_angle(dlength,length)
from_rotation_vector = function(so3.from_rotation_vector,'so3.from_rotation_vector',(3,),9,
    jvp = [from_rotation_vector_jvp])

def rotation_vector_jvp(dR,R):
    theta = so3.angle(R)
    dtheta = angle_jvp(dR,R)
    #normal
    scale = 0.5
    dscale = -0.5*dtheta
    if abs(theta) > 1e-5:
        s = math.sin(theta)
        ds = math.cos(theta)*dtheta
        scale = 0.5*theta/s
        dscale = 0.5*(dtheta/s - theta/s**2*ds)
    x = (dR[3+2]-dR[6+1]) * scale + (R[3+2]-R[6+1]) * dscale
    y = (dR[6+0]-dR[0+2]) * scale + (R[6+0]-R[0+2]) * dscale
    z = (dR[0+1]-dR[3+0]) * scale + (R[0+1]-R[3+0]) * dscale
    return [x,y,z]
rotation_vector = function(so3.rotation_vector,'so3.rotation_vector',(9,),3,
    jvp=[rotation_vector_jvp])

from_quaternion = function(so3.from_quaternion,'so3.from_quaternion',(4,),9,order=2)
quaternion = function(so3.quaternion,'so3.quaternion',(9,),4)

def from_axis_angle_derivative_axis(axis,angle):
    raise NotImplementedError()
def from_axis_angle_derivative_angle(axis,angle):
    #m = cos(angle)*I + (1-cos(angle))axis*axis^T + sin(angle)[axis]
    R = so3.from_axis_angle((axis,angle))
    return np.array(so3.mul(R,so3.cross_product(axis)))[:,np.newaxis]
def from_axis_angle_jvp_axis(daxis,axis,angle):
    #m = cos(angle)*I + (1-cos(angle))axis*axis^T + sin(angle)[axis]
    #dm/daxis*delta = (1-cos(angle))d/daxis(axis*axis^T) + sin(angle)d/daxis([axis]))*delta
    #so result is (1-cos(angle)) (delta x^T + x delta^T) + sin(angle)[delta]
    c = math.cos(angle)
    s = math.sin(angle)
    R = vectorops.mul(so3.cross_product(daxis),s)
    for i in range(3):
        for j in range(3):
            R[i*3+j] += (daxis[i]*axis[j] + axis[i]*daxis[j])*(1-c)
    return np.array(R)
def from_axis_angle_jvp_angle(dangle,axis,angle):
    R = so3.from_axis_angle((axis,angle))
    return dangle*np.array(so3.mul(R,so3.cross_product(axis)))
from_axis_angle = function(lambda axis,angle:so3.from_axis_angle((axis,angle)),'so3.from_axis_angle',(3,1),9,['axis','angle'],
                    derivative=[from_axis_angle_derivative_axis,from_axis_angle_derivative_angle],
                    jvp=[from_axis_angle_jvp_axis,from_axis_angle_jvp_angle])

def axis_jvp(dR,R):
    w = np.array(so3.rotation_vector(R))
    dw = rotation_vector_jvp(dR,R)
    return math_ad.unit_jvp(dw,w)
axis = function(lambda R:vectorops.unit(so3.rotation_vector(R)),'axis',(9,),3,
    jvp=[axis_jvp])

def angle_jvp(dR,R):
    cosangle = (so3.trace(R) - 1)*0.5
    cosangle = max(min(cosangle,1.0),-1.0)
    if cosangle == 1:
        return vectorops.norm([dR[1],dr[2],dr[5]])
    #dangle / dR[0] = -1.0/sqrt(1-cosangle**2) * dcosangle/dR[0]
    dacos = -1.0/math.sqrt(1-cosangle**2)
    return so3.trace(dR)*0.5*dacos
angle = function(so3.angle,'so3.angle',(9,),1,jvp=[angle_jvp])

def error_jvp_Ra(dRa,Ra,Rb):
    #error = so3.rotation_vector(so3.mul(Ra,so3.inv(Rb))
    #derror/dRa * delta = drotation_vector/dR(...)*dR/dRa* delta
    #d(Ra*Rb^-1 / dRa)*delta = delta*Rb^-1
    Rbinv = so3.inv(Rb)
    Rrel = so3.mul(Ra,Rbinv)
    dRrel = so3.mul(dRa,Rbinv)
    return rotation_vector_jvp(dRrel,Rrel)
def error_jvp_Rb(dRb,Ra,Rb):
    return -error_jvp_Ra(dRb,Rb,Ra)
error = function(so3.error,'so3.error',(9,9),3,
    jvp=[error_jvp_Ra,error_jvp_Rb])
distance = function(so3.distance,'so3.distance',(9,9),1,['Ra','Rb'])
canonical = function(so3.canonical,'so3.canonical',(3,),9)
cross_product = function(so3.cross_product,'so3.cross_product',(3,),9,['x'],
        jvp=[lambda dx,x:so3.cross_product(dx)],order=1)
interpolate = function(so3.interpolate,'so3.interpolate',(9,9,1),9,['Ra','Rb','u'])
det = function(so3.det,'so3.det',(9,),1)

