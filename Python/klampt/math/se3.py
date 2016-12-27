"""Operations for rigid transformations in Klamp't.  All rigid
transformations are specified in the form T=(R,t),
where R is a 9-list specifying the entries of the rotation matrix in column
major form, and t is a 3-list specifying the translation vector.

Primarily, this form was chosen for interfacing with C++ code.  C++ methods
on objects of the form T = X.getTransform() will return se3 element proper.
Conversely, X.setTransform(*T) will set the rotation and translation of some
transform.

Extracting the so3 portion of the transform is simply T[0].  Extracting
the translation vector is simply T[1].

Applying an se3 element to a point p is apply(T,p).  Applying it to a
directional quantity d is either apply_rotation(T,d) or so3.apply(T[0],d).

To flatten into a single array, use model.getConfig(T), which is equivalent
to T[0] + T[1].  To read and write to disk in a way that is compatible with 
other Klamp't IO routines, use io.loader.writeSe3() and io.loader.readSe3().
"""

import vectorops
import so3

def identity():
    """Returns the identity transformation."""
    return ([1.,0.,0.,0.,1.,0.,0.,0.,1.],[0.,0.,0.])

def inv(T):
    """Returns the inverse of the transformation."""
    (R,t) = T
    Rinv = so3.inv(R)
    tinv = [-Rinv[0]*t[0]-Rinv[3]*t[1]-Rinv[6]*t[2],
            -Rinv[1]*t[0]-Rinv[4]*t[1]-Rinv[7]*t[2],
            -Rinv[2]*t[0]-Rinv[5]*t[1]-Rinv[8]*t[2]]
    return (Rinv,tinv)

def apply(T,point):
    """Applies the transform T to the given point"""
    return vectorops.add(apply_rotation(T,point),T[1])

def apply_rotation(T,point):
    """Applies only the rotation part of T"""
    R = T[0]
    return so3.apply(R,point)

def rotation(T):
    """Returns the 3x3 rotation matrix corresponding to T's rotation"""
    (R,t) = T
    return so3.matrix(R)

def from_rotation(mat):
    """Returns a transformation T corresponding to the 3x3 rotation matrix mat"""
    R = so3.from_matrix(R)
    return (R,[0.,0.,0.])

def translation(T):
    """Returns the translation vector corresponding to T's translation"""
    return T[1]

def from_translation(t):
    """Returns a transformation T that translates points by t"""
    return (so3.identity(),t[:])

def homogeneous(T):
    """Returns the 4x4 homogeneous transform corresponding to T"""
    (R,t) = T
    return [[R[0],R[3],R[6],t[0]],
            [R[1],R[4],R[7],t[1]],
            [R[2],R[5],R[8],t[2]],
            [0.,0.,0.,1.]]

def from_homogeneous(mat):
    """Returns a T corresponding to the 4x4 homogeneous transform mat"""
    t = [mat[0][3],mat[1][3],mat[2][3]]
    R = [mat[0][0],mat[1][0],mat[2][0],mat[0][1],mat[1][1],mat[2][1],mat[0][2],mat[1][2],mat[2][2]]
    return (R,t)

def mul(T1,T2):
    """Composes two transformations."""
    (R1,t1) = T1
    (R2,t2) = T2
    R = so3.mul(R1,R2)
    t = apply(T1,t2)
    return (R,t)

def distance(T1,T2,Rweight=1.0,tweight=1.0):
    """Returns a distance metric between the two transformations. The
    rotation distance is weighted by Rweight and the translation distance
    is weighted by tweight"""
    (R1,t1)=T1
    (R2,t2)=T2
    return Rweight*so3.distance(R1,R2) + tweight*vectorops.distance(t1,t2)

def error(T1,T2):
    """Returns a 6D "difference vector" that describes how far T1 is from T2.
    More precisely, this is the Lie derivative (w,v)."""
    (R1,t1)=T1
    (R2,t2)=T2
    #concatenate lists
    return so3.error(R1,R2) + vectorops.sub(t1,t2)

def interpolate(T1,T2,u):
    """Interpolate linearly between the two transformations T1 and T2."""
    return (so3.interpolate(T1[0],T2[0],u),vectorops.interpolate(T1[1],T2[1],u))
