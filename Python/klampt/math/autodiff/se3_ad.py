"""se3 module AD functions:

Note that each function takes a 12 element vector which is the concatenation
of the R and t terms in an se3 element.  In other words, to convert from an
se3 element T to an entry that can be properly eval'ed, use the code
``np.array(T[0] + T[1])``.  Alternatively, you can use the ``join`` function
``se3_ad.join(T[0],T[1])`` which is differentiable.

To extract out the rotation / translation from an autodiff-ed function, use
the ``se3_ad.rotation`` and ``se3_ad.translation`` functions.

 ====================  =============  ====================================
 Function              Derivative     Notes
 ====================  =============  ====================================
 join                  Y              Creates a 12-element vector from R,t
 identity              N/A            A constant function with no args
 apply                 Y
 apply_rotation        Y
 mul                   Y
 inv                   Y
 rotation              Y
 from_rotation         Y
 translation           Y
 from_translation      Y
 distance              N
 interpolate           N
 error                 N
=====================  =============  ====================================
"""

import numpy as np 
from .ad import ADFunctionInterface,function
from .. import se3

def _rotation(T):
    return T[:9]
def _translation(T):
    return T[9:]
def _split(T):
    return (T[:9],T[9:])
def _join(T):
    return np.hstack(T)

join = function(lambda R,t:np.hstack((R,t)),'se3.join',[9,3],12,
    derivative=[lambda R,t:np.vstack((np.eye(9),np.zeros((3,9)))),lambda R,t:np.vstack((np.zeros((9,3)),np.eye(3)))],
    jvp=[lambda dR,R,t:np.hstack((dR,np.zeros(3))),lambda dt,R,t:np.hstack((np.zeros(9),dt))],order=1)
identity = se3.identity
apply = function(lambda T,x:se3.apply(_split(T),x),'se3.apply',(12,3),3,['T','x'],
        jvp=[lambda dT,T,x:np.array(so3.apply(_rotation(dT),x))+_translation(dT),lambda dx,T,x:np.array(so3.apply(_rotation(T),dx))],order=2)
apply_rotation = function(lambda T,x:so3.apply(_rotation(T),x),'se3.apply_rotation',(12,3),3,['T','x'],
        jvp=[lambda dT,T,x:np.array(so3.apply(_rotation(dT),x)),lambda dx,T,x:np.array(so3.apply(_rotation(T),dx))],order=2)
mul = function(lambda T1,T2:_join(se3.mul(_split(T1),_split(T2))),'se3.mul',(12,12),12,['T1','T1'])
inv = function(lambda T:_join(se3.inv(_split(T))),'se3.inv',(12,),12)

rotation = function(_rotation,'se3.rotation',(12,),9,
    #derivative=[lambda T:np.hstack((np.eye(9),np.zeros((9,3))))],
    jvp=[lambda dT,T:_rotation(dT)],order=1)
translation = function(_translation,'se3.translation',(12,),3,
    #derivative=[lambda T:np.hstack((np.zeros((3,9)),np.eye(3)))],
    jvp=[lambda dT,T:_translation(dT)],order=1)

from_rotation = function(lambda R:_join((R,np.zeros(3))),'se3.from_rotation',(9,),12,
    jvp=[lambda dR,R:_join((R,np.zeros(3)))],order=1)
from_translation = function(lambda t:_join((np.eye(3).flatten(),t)),'se3.from_translation',(3,),12,
    jvp=[lambda dt,t:_join((np.eye(3).flatten(),dt))],order=1)


error = function(lambda T1,T2:se3.error(_split(T1),_split(T2)),'se3.error',(12,12),6,['T1','T2'])
distance = function(lambda T1,T2:se3.distance(_split(T1),_split(T2)),'se3.distance',(12,12),1,['T1','T2'])
interpolate = function(lambda T1,T2,u:_join(se3.interpolate(_split(T1),_split(T2),u)),'se3.interpolate',(12,12,1),12,['Ta','Tb','u'])

