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
 apply                 1
 apply_rotation        1
 mul                   N
 inv                   N
 rotation              Y
 from_rotation         Y
 translation           Y
 from_translation      Y
 distance              N
 interpolate           N
 error                 N
 ====================  =============  ====================================
 
.. note::
    To do gradient descent on se3 elements, you will need to either project to
    the se3 submanifold or use a non-redundant representation.  The rotation 
    vector representation (see so3_ad.rotation_vector and
    so3_ad.from_rotation_vector) is recommended. 

    The code ``se3_ad.join(so3_ad.from_rotation_vector(x[0:3]),x[3:6]))`` will
    convert from a 6-element vector x (first 3 elements encoding the rotation
    vector, final 3 elements encoding the translation) to a se3 element.

Module contents
~~~~~~~~~~~~~~~

.. autosummary::
    join
    identity
    apply
    apply_rotation
    mul
    inv
    rotation
    from_rotation
    translation
    from_translation
    distance
    interpolate
    error

"""

import numpy as np 
from .ad import ADFunctionInterface,function
from .. import so3,se3

SIZE = 12
"""Constant giving the dimension of an se3_ad element"""

def _rotation(T):
    return T[:9]
def _translation(T):
    return T[9:]
def _split(T):
    return (T[:9],T[9:])
def _join(T):
    return np.hstack(T)

def to_klampt(T):
    """Converts an autodiff se3 representation as a length-12 numpy array to
    the native Klampt representation (R,t)."""
    return (T[:9].tolist(),T[9:].tolist())

def from_klampt(T):
    """Converts a native Klampt se3 representation (R,t) to the length-12 numpy
    array representation used in autodiff."""
    return _join(T)

join = function(lambda R,t:np.hstack((R,t)),'se3.join',[9,3],12,
    derivative=[lambda R,t:np.vstack((np.eye(9),np.zeros((3,9)))),lambda R,t:np.vstack((np.zeros((9,3)),np.eye(3)))],
    jvp=[lambda dR,R,t:np.hstack((dR,np.zeros(3))),lambda dt,R,t:np.hstack((np.zeros(9),dt))],order=1)
"""Autodiff function to join SO(3) rotation matrix and 3D translation vector
into an SE(3) 12-element vector. All derivatives are implemented."""

rotation = function(_rotation,'se3.rotation',(12,),9,
    #derivative=[lambda T:np.hstack((np.eye(9),np.zeros((9,3))))],
    jvp=[lambda dT,T:_rotation(dT)],order=1)
"""Autodiff function to extract the SO(3) rotation component of an SE(3)
12-element vector. All derivatives are implemented."""

translation = function(_translation,'se3.translation',(12,),3,
    #derivative=[lambda T:np.hstack((np.zeros((3,9)),np.eye(3)))],
    jvp=[lambda dT,T:_translation(dT)],order=1)
"""Autodiff function to extract the 3-D translation component of an SE(3)
12-element vector. All derivatives are implemented."""

identity = lambda:_join(se3.identity())
"""A 0 argument function that returns the SO(3) identity (just an alias to
so3.identity)"""

apply = function(lambda T,x:se3.apply(_split(T),x),'se3.apply',(12,3),3,['T','x'],
        jvp=[lambda dT,T,x:np.array(so3.apply(_rotation(dT),x))+_translation(dT),lambda dx,T,x:np.array(so3.apply(_rotation(T),dx))],order=2)
"""Autodiff wrapper of se3.apply.  First derivatives are implemented."""

apply_rotation = function(lambda T,x:so3.apply(_rotation(T),x),'se3.apply_rotation',(12,3),3,['T','x'],
        jvp=[lambda dT,T,x:np.array(so3.apply(_rotation(dT),x)),lambda dx,T,x:np.array(so3.apply(_rotation(T),dx))],order=2)
"""Autodiff wrapper of se3.apply_rotation.  First derivatives are implemented."""

mul = function(lambda T1,T2:_join(se3.mul(_split(T1),_split(T2))),'se3.mul',(12,12),12,['T1','T1'])
"""Autodiff wrapper of se3.mul."""

inv = function(lambda T:_join(se3.inv(_split(T))),'se3.inv',(12,),12)
"""Autodiff wrapper of se3.inv."""

from_rotation = function(lambda R:_join((R,np.zeros(3))),'se3.from_rotation',(9,),12,
    jvp=[lambda dR,R:_join((R,np.zeros(3)))],order=1)
"""Autodiff wrapper of se3.from_rotation.  All derivatives are implemented."""

from_translation = function(lambda t:_join((np.eye(3).flatten(),t)),'se3.from_translation',(3,),12,
    jvp=[lambda dt,t:_join((np.eye(3).flatten(),dt))],order=1)
"""Autodiff wrapper of se3.from_translation.  All derivatives are implemented."""

error = function(lambda T1,T2:se3.error(_split(T1),_split(T2)),'se3.error',(12,12),6,['T1','T2'])
"""Autodiff wrapper of se3.error."""

distance = function(lambda T1,T2:se3.distance(_split(T1),_split(T2)),'se3.distance',(12,12),1,['T1','T2'])
"""Autodiff wrapper of se3.distance."""

interpolate = function(lambda T1,T2,u:_join(se3.interpolate(_split(T1),_split(T2),u)),'se3.interpolate',(12,12,1),12,['Ta','Tb','u'])
"""Autodiff wrapper of se3.interpolate."""

