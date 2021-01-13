"""Defines symbolic linear algebra functions.

Function list: 

- norm(x): returns the L-2 norm of a vector
- norm2(x): returns the squared L-2 norm of a vector
- norm_L1(x): returns the L-1 norm of a vector
- norm_Linf(x): returns the L-infinity norm of a vector
- norm_fro(A): returns the Frobeneus norm of a matrix
- distance(x,y): returns the L-2 distance between two vectors
- distance2(x,y): returns the squared L-2 distance between two vectors
- distance_L1(x,y): returns the L-1 distance between two vectors
- distance_Linf(x,y): returns the L-infinity distance between two vectors
- mahalanobis_distance(x,y,A): returns the mahalanobis distance between two vectors weighted by A, i.e. :math:`\sqrt((x-y)^T A (x-y))`
- mahalanobis_distance2(x,y,A): returns the squared mahalanobis distance between two vectors weighted by A
- unit(x): returns the unit vector in the direction x
- inv(A): returns the inverse of a matrix
- pinv(A): returns the pseudoinverse of a matrix
- linear(x,A): the linear function :math:`A x`
- quadratic(x,A): the quadratic function :math:`x^T A x`
- bilinear(x,y,A): the bilinear function :math:`x^T A y`
- bound_contains(xmin,xmax,x): returns True if xmin <= x <= xmax element-wise
- bound_margin(xmin,xmax,x): returns the distance from x to boundaries of the bounding box [xmin,xmax], if inside (positive is inside)
- bound_overlaps(xmin,xmax,ymin,ymax): returns True if xmin <= x <= xmax element-wise

Also defines a context LinAlgContext that can be included into a Context under "linalg" 

Completeness table

=================  =============  ==============
Function           Derivative     Simplification
=================  =============  ==============
  norm             Y                              
  norm2            Y                              
  norm_L1          Y                              
  norm_Linf        Y                              
  norm_fro         Y                              
  distance         Y                              
  distance2        Y                              
  distance_L1      Y                              
  distance_Linf    Y                              
  mahalanobis_dis  Y,Y,N                          
  mahalanobis...2  Y,Y,N                          
  unit             Y                              
  inv              Y                              
  pinv             Y                              
  linear           Y                              
  quadratic        Y,Y,N                          
  bilinear         Y,Y,N                          
  bound_contains   N/A                            
  bound_margin                                    
  bound_overlaps   N/A                            
=================  =============  ==============

Module contents
~~~~~~~~~~~~~~~

.. autosummary::
    norm
    norm2
    norm_L1
    norm_Linf
    norm_fro
    distance
    distance2
    distance_L1
    distance_Linf
    mahalanobis_distance
    mahalanobis_distance2
    unit
    inv
    pinv
    linear
    quadratic
    bilinear
    bound_contains
    bound_margin
    bound_overlaps
    LinAlgContext
  
"""

from .symbolic import *
from . import vectorops
import numpy as np
import weakref

_x = Variable("x","V")
_y = Variable("y","V")
_A = Variable("A","M")
norm = Function('norm',np.linalg.norm,['x'],returnType='N')
norm.description = "Returns the L-2 norm of a vector"
norm2 = Function('norm2',dot(_x,_x),['x'],returnType='N')
norm_L1 = Function('norm_L1',lambda x:np.linalg.norm(x,ord=1),returnType='N')
_inf = float('inf')
norm_Linf = Function('norm_Linf',lambda x:np.linalg.norm(x,ord=_inf),returnType='N')
norm_fro = Function('norm_fro',np.linalg.norm,['A'],returnType='N')
norm2_fro = Function('norm2_fro',lambda x:np.linalg.norm(x)**2,['A'],returnType='N')
distance = Function('distance',norm(_x-_y),['x','y'],returnType='N')
distance2 = Function('distance2',norm2(_x-_y),['x','y'],returnType='N')
distance_L1 = Function('distance_L1',norm_L1(_x-_y),['x','y'],returnType='N')
distance_Linf = Function('distance_Linf',norm_Linf(_x-_y),['x','y'],returnType='N')
unit = Function('unit',vectorops.unit,['x'])
norm.argTypes = [Vector]
norm2.argTypes = [Vector]
norm_L1.argTypes = [Vector]
norm_Linf.argTypes = [Vector]
norm_fro.argTypes = [Matrix]
norm.properties['nonnegative'] = True
norm2.properties['nonnegative'] = True
norm_L1.properties['nonnegative'] = True
norm_Linf.properties['nonnegative'] = True
norm_fro.properties['nonnegative'] = True
norm2_fro.properties['nonnegative'] = True
norm.addSimplifier(['zero'],lambda x:0)
norm.addSimplifier(['unit'],lambda x:1)
norm2.addSimplifier(['zero'],lambda x:0)
norm2.addSimplifier(['unit'],lambda x:1)
norm_L1.addSimplifier(['zero'],lambda x:0)
norm_L1.addSimplifier(['basis'],lambda x:1)
norm_Linf.addSimplifier(['zero'],lambda x:0)
norm_Linf.addSimplifier(['basis'],lambda x:1)
norm.setDeriv(0,(lambda x,dx:dot(x,dx)/norm(x)),asExpr=True,stackable=True)
norm2.setDeriv(0,(lambda x,dx:dot(x,dx)*2),asExpr=True,stackable=True)
norm_L1.setDeriv(0,lambda x,dx:dot(sign(x),dx),asExpr=True,stackable=True)
def _norm_Linf_deriv(x,dx):
    imax = argmax(abs_(x))
    return sign(x[imax])*dx[imax]
norm_Linf.setDeriv(0,_norm_Linf_deriv,asExpr=True,stackable=True)
norm_fro.setDeriv(0,(lambda A,dA:mul(A,dA)/norm_fro(A)),asExpr=True,stackable=True)
norm2_fro.setDeriv(0,(lambda A,dA:mul(A,dA)*2),asExpr=True,stackable=True)
distance.argTypes = [Vector,Vector]
distance2.argTypes = [Vector,Vector]
distance_L1.argTypes = [Vector,Vector]
distance_Linf.argTypes = [Vector,Vector]
distance.properties['nonnegative'] = True
distance2.properties['nonnegative'] = True
distance_L1.properties['nonnegative'] = True
distance_Linf.properties['nonnegative'] = True
distance.addSimplifier(['zero',None],lambda x,y:norm(y))
distance.addSimplifier([None,'zero'],lambda x,y:norm(x))
distance2.addSimplifier(['zero',None],lambda x,y:norm2(y))
distance2.addSimplifier([None,'zero'],lambda x,y:norm2(x))
distance_L1.addSimplifier(['zero',None],lambda x,y:norm_L1(y))
distance_L1.addSimplifier([None,'zero'],lambda x,y:norm_L1(x))
distance_Linf.addSimplifier(['zero',None],lambda x,y:norm_Linf(y))
distance_Linf.addSimplifier([None,'zero'],lambda x,y:norm_Linf(x))
distance.setDeriv(0,(lambda x,y,dx:dot(x-y,dx)/distance(x,y)),asExpr=True,stackable=True)
distance.setDeriv(1,(lambda x,y,dy:dot(x-y,dy)/distance(x,y)),asExpr=True,stackable=True)
distance2.setDeriv(0,(lambda x,y,dx:dot(x-y,dx)*2),asExpr=True,stackable=True)
distance2.setDeriv(1,(lambda x,y,dy:dot(y-x,dy)*2),asExpr=True,stackable=True)
distance_L1.autoSetJacobians()
distance_Linf.autoSetJacobians()
linear = Function('linear',dot(_A,_x),["x","A"])
linear.argTypes = [Vector,Matrix]
linear.autoSetJacobians()
bilinear = Function('bilinear',dot(_x,dot(_A,_y)),["x","A","y"])
bilinear.argTypes = [Vector,Matrix,Vector]
bilinear.setJacobian(0,lambda x,A,y:dot(A,y),asExpr=True)
bilinear.setJacobian(1,lambda x,A,y:outer(x,y),asExpr=True)
bilinear.setJacobian(2,lambda x,A,y:dot(A,x),asExpr=True)
quadratic = Function('quadratic',dot(_x,dot(_A,_x)),["x","A"],returnType='N')
quadratic.argTypes = [Vector,Matrix]
quadratic.setJacobian('x',lambda x,A:2*dot(x,A),asExpr=True)
quadratic.setJacobian('A',lambda x,A:outer(x,x),asExpr=True)
mahalanobis_distance2 = Function('mahalanobis_distance2',quadratic(_x-_y,_A),['x','y','A'])
mahalanobis_distance2.autoSetJacobians()
mahalanobis_distance = Function('mahalanobis_distance',sqrt(mahalanobis_distance2(_x,_y,_A)),['x','y','A'])
mahalanobis_distance.autoSetJacobians()
unit.setDeriv(0,lambda x,dx:if_(x==0,zero(shape(x)),dx/norm(x)-x*(dot(x,dx)/norm(x))**3),stackable=True)
inv = Function('inv',np.linalg.inv,['A'],returnType='M')
inv.argTypes = [Matrix]
def _inv_deriv(A,dA):
    Ainv = inv(A)
    return -dot(Ainv,dot(dA,Ainv))
inv.properties['inverse'] = weakref.proxy(inv)
#patch the dot function
dot.addSimplifier(['inv','inv'],lambda Ainv,Binv:inv(dot(Binv.args[0],Ainv.args[0])))
dot.addSimplifier(['linalg.inv','linalg.inv'],lambda Ainv,Binv:inv(dot(Binv.args[0],Ainv.args[0])))
inv.addSimplifier(['neg'],lambda x:-inv(x.args[0]))
inv.setDeriv(0,_inv_deriv,asExpr=True)
inv.printers['str'] = lambda expr,astr:astr[0]+'^-1'
pinv = Function('pinv',np.linalg.pinv,['A'],returnType='M')
pinv.argTypes = [Matrix]
pinv.addSimplifier(['neg'],lambda x:-pinv(x.args[0]))
pinv.printers['str'] = lambda expr,astr:astr[0]+'^+'
def _pinv_deriv(A,dA):
    Ainv = pinv(A)
    return -dot(Ainv,dot(dA,Ainv))
pinv.setDeriv(0,_pinv_deriv,asExpr=True)
def _bound_contains(xmin,xmax,x):
    return all(a <= v and v <= b for v,a,b in zip(x,xmin,xmax))
def _bound_overlaps(xmin,xmax,ymin,ymax):
    for a,b,c,d in zip(xmin,xmax,ymin,ymax):
        if d < a or c > b: return False
        if b < c or a > d: return False
    return True
def _bound_margin(xmin,xmax,x):
    return min(min(v-a,b-v) for v,a,b in zip(x,xmin,xmax))
bound_contains = Function('bound_contains',_bound_contains,returnType = 'B')
bound_overlaps = Function('bound_overlaps',_bound_overlaps,returnType = 'B')
bound_margin = Function('bound_margin',_bound_margin,returnType = 'N')
bound_contains.argTypes = [Vector]*3
bound_overlaps.argTypes = [Vector]*4
bound_margin.argTypes = [Vector]*3
#bound_margin = Function('bound_margin',min_(min_(Variable('x','V',None)-Variable('xmin','V',None)),min_(Variable('xmax','V',None)-Variable('x','V',None))),['xmin','xmax','x'],returnType = 'N')

class LinAlgContext(Context):
    def __init__(self):
        Context.__init__(self)
        self.norm = self.declare(norm)
        self.norm2 = self.declare(norm2)
        self.norm_L1 = self.declare(norm_L1)
        self.norm_Linf = self.declare(norm_Linf)
        self.distance = self.declare(distance)
        self.distance2 = self.declare(distance2)
        self.distance_L1 = self.declare(distance_L1)
        self.distance_Linf = self.declare(distance_Linf)
        self.unit = self.declare(unit)
        self.linear = self.declare(linear)
        self.bilinear = self.declare(bilinear)
        self.quadratic = self.declare(quadratic)
        self.mahalanobis_distance = self.declare(mahalanobis_distance)
        self.mahalanobis_distance2 = self.declare(mahalanobis_distance2)
        self.inv = self.declare(inv)
        self.pinv = self.declare(pinv)
        self.bound_contains = self.declare(bound_contains)
        self.bound_overlaps = self.declare(bound_overlaps)
        self.bound_margin = self.declare(bound_margin)
