"""Operations for rigid transformations in Klamp't.  All rigid
transformations are specified in the form ``T=(R,t)``,
where R is a 9-list specifying the entries of the rotation matrix in column
major form, and t is a 3-list specifying the translation vector.

Primarily, this form was chosen for interfacing with C++ code.  C++ methods
on objects of the form ``T = X.getTransform()`` will return se3 element proper.
Conversely, ``X.setTransform(*T)`` will set the rotation and translation of
some transform.

Extracting the so3 portion of the transform is simply T[0].  Extracting
the translation vector is simply T[1].

Applying an se3 element to a point p is ``apply(T,p)``.  Applying it to a
directional quantity ``d`` is either ``apply_rotation(T,d)`` or
``so3.apply(T[0],d)``.

To flatten into a single array, use ``klampt.model.getConfig(T)``, which is
equivalent to ``T[0] + T[1]``.  To read and write to disk in a way that is
compatible with  other Klamp't IO routines, use ``klampt.io.loader.write_se3()``
and ``klampt.io.loader.read_se3()``.
"""

from . import vectorops
from . import so3
from typing import List,Tuple,Callable
from ..model.typing import Vector3,Rotation,RigidTransform

def identity() -> RigidTransform:
    """Returns the identity transformation."""
    return ([1.,0.,0.,0.,1.,0.,0.,0.,1.],[0.,0.,0.])

def inv(T : RigidTransform) -> RigidTransform:
    """Returns the inverse of the transformation."""
    (R,t) = T
    Rinv = so3.inv(R)
    tinv = [-Rinv[0]*t[0]-Rinv[3]*t[1]-Rinv[6]*t[2],
            -Rinv[1]*t[0]-Rinv[4]*t[1]-Rinv[7]*t[2],
            -Rinv[2]*t[0]-Rinv[5]*t[1]-Rinv[8]*t[2]]
    return (Rinv,tinv)

def apply(T : RigidTransform, point: Vector3) -> Vector3:
    """Applies the transform T to the given point"""
    return vectorops.add(apply_rotation(T,point),T[1])

def apply_rotation(T : RigidTransform, point : Vector3) -> Vector3:
    """Applies only the rotation part of T"""
    R = T[0]
    return so3.apply(R,point)

def rotation(T : RigidTransform) -> Rotation:
    """Returns the 3x3 rotation matrix corresponding to T's rotation"""
    (R,t) = T
    return so3.matrix(R)

def from_rotation(mat : List[List[float]]) -> RigidTransform:
    """Returns a transformation T corresponding to the 3x3 rotation matrix mat"""
    R = so3.from_matrix(mat)
    return (R,[0.,0.,0.])

def translation(T : RigidTransform) -> Vector3:
    """Returns the translation vector corresponding to T's translation"""
    return T[1]

def from_translation(t : Vector3) -> RigidTransform:
    """Returns a transformation T that translates points by t"""
    return (so3.identity(),t[:])

def homogeneous(T : RigidTransform) -> List[List[float]]:
    """Returns the 4x4 homogeneous transform corresponding to T."""
    (R,t) = T
    return [[R[0],R[3],R[6],t[0]],
            [R[1],R[4],R[7],t[1]],
            [R[2],R[5],R[8],t[2]],
            [0.,0.,0.,1.]]

def from_homogeneous(mat : List[List[float]]) -> RigidTransform:
    """Returns an se3 transform corresponding to the 4x4 homogeneous
    transform matrix mat."""
    t = [mat[0][3],mat[1][3],mat[2][3]]
    R = [mat[0][0],mat[1][0],mat[2][0],mat[0][1],mat[1][1],mat[2][1],mat[0][2],mat[1][2],mat[2][2]]
    return (R,t)

def ndarray(T : RigidTransform) -> "ndarray":
    """Returns the 4x4 homogeneous transform corresponding to T."""
    import numpy
    return numpy.array(homogeneous(T))

def from_ndarray(mat : "ndarray") -> "RigidTransform":
    """Returns an se3 transform from a 4x4 Numpy array representing the
    homogeneous transform."""
    return from_homogeneous(mat)

def mul(T1 : RigidTransform, T2 : RigidTransform) -> RigidTransform:
    """Composes two transformations."""
    if len(T1) != 2: raise ValueError("T1 is not a transform")
    if len(T2) != 2: raise ValueError("T2 is not a transform (did you mean to use apply())?")
    (R1,t1) = T1
    (R2,t2) = T2
    R = so3.mul(R1,R2)
    t = vectorops.add(so3.apply(R1,t2),t1)
    return (R,t)

def distance(T1 : RigidTransform, T2 : RigidTransform, Rweight=1.0,tweight=1.0) -> float:
    """Returns a distance metric between the two transformations. The
    rotation distance is weighted by Rweight and the translation distance
    is weighted by tweight"""
    (R1,t1)=T1
    (R2,t2)=T2
    return Rweight*so3.distance(R1,R2) + tweight*vectorops.distance(t1,t2)

def error(T1 : RigidTransform, T2 : RigidTransform) -> float:
    """Returns a 6D "difference vector" that describes how far T1 is from T2.
    More precisely, this is the (stacked) Lie derivative (w,v).
    
    Fun fact: the error is related to the derivative of interpolate(T2,T1,u)
    at u=0 by d/du interpolate(T2,T1,0) = (mul(cross_product(w),R2),v).
    
    You can also recover T1 from (w,v) via T1 = (mul(from_moment(w),T2[0]),
    vectorops.add(v,T2[1])).
    """
    (R1,t1)=T1
    (R2,t2)=T2
    #concatenate lists
    return so3.error(R1,R2) + vectorops.sub(t1,t2)

def interpolate(T1 : RigidTransform, T2 : RigidTransform, u : float) -> RigidTransform:
    """Interpolate linearly between the two transformations T1 and T2."""
    return (so3.interpolate(T1[0],T2[0],u),vectorops.interpolate(T1[1],T2[1],u))

def interpolator(T1 : RigidTransform, T2 : RigidTransform) -> Callable:
    """Returns a function of one parameter u that interpolates linearly
    between the two transformations T1 and T2. After f(u) is constructed, calling
    f(u) is about 2x faster than calling interpolate(T1,T2,u)."""
    R1,t1 = T1
    R2,t2 = T2
    dt = vectorops.sub(t2,t1)
    def f(u,so3_interp=so3.interpolator(R1,R2),t1=t1,dt=dt):
        return (so3_interp(u),vectorops.madd(t1,dt,u))
    return f
