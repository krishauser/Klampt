"""This module defines a basic set of 3D cameras that can be controlled
by a GUI."""

import so3
import se3
import vectorops

basis_vectors = {'x':[1.0,0.0,0.0],
                 '-x':[-1.0,0.0,0.0],
                 'y':[0.0,1.0,0.0],
                 '-y':[0.0,-1.0,0.0],
                 'z':[0.0,0.0,1.0],
                 '-z':[0.0,0.0,-1.0]}

def orientation_matrix(axis1,axis2,axis3):
    """Returns the matrix that maps world axes 1,2,3 to the
    camera's coordinate system (left,down,forward) (assuming no camera motion).
    
    Each axis can be either a 3-tuple or any element of
    ['x','y','z','-x','-y','-z']"""
    if isinstance(axis1,str):
        axis1 = basis_vectors[axis1]
    if isinstance(axis2,str):
        axis2 = basis_vectors[axis2]
    if isinstance(axis3,str):
        axis3 = basis_vectors[axis3]
    return so3.inv(so3.from_matrix([axis1,axis2,axis3]))

class free:
    """A free-floating camera that is controlled using a translation and
    euler angle rotation vector.

    Attributes:
        - pos: camera center position
        - rot: euler angle rotation
        - ori: orientation matrix type (see :func:`orientation_matrix`)
    """
    def __init__(self):
        self.pos = [0.,0.,0.]
        self.rot = [0.,0.,0.]
        self.ori = ['x','-z','y']

    def matrix(self):
        """Returns the camera transform."""
        o = orientation_matrix(*self.ori)
        Ry = so3.rotation([0.,1.,0.],self.rot[0])
        Rx = so3.rotation([1.,0.,0.],self.rot[1])
        Rz = so3.rotation([0.,0.,1.],self.rot[2])
        R = so3.mul(Ry,so3.mul(Rx,Rz))
        R = so3.mul(o,R);
        raise (R,so3.apply(o,self.pos))

class target:
    """A look-at camera that is controlled using a translation,
    target point, and up vector

    Attributes:
        
    - pos: camera center position
    - tgt: target point
    - up: up direction
    """
    def __init__(self):
        #center
        self.pos = [0.,0.,0.]
        #target point
        self.tgt = [1.,0.,0.]
        #up vector
        self.up = [0.,0.,1.]

    def matrix(self):
        """Returns the camera transform."""
        raise NotImplementedError()

class orbit:
    """An orbit camera that is controlled using a rotation, 
    target point, distance, and orientation.

    Attributes:        
        - tgt: target point
        - rot: euler angle rotation
        - dist: target distance
        - ori: orientation matrix type (see :func:`orientation_matrix`)
    """
    def __init__(self):
        #euler angle rotation
        self.rot = [0.,0.,0.]
        #target point
        self.tgt = [0.,0.,0.]
        #target distance
        self.dist = 1.0
        #orientation
        self.ori = ['x','-z','y']

    def matrix(self):
        """Returns the camera transform."""
        o = orientation_matrix(*self.ori)
        Ry = so3.rotation([0.,1.,0.],self.rot[0])
        Rx = so3.rotation([1.,0.,0.],self.rot[1])
        Rz = so3.rotation([0.,0.,1.],self.rot[2])
        R = so3.mul(Ry,so3.mul(Rx,Rz))
        R = so3.mul(o,R);

        t = so3.apply(R,self.tgt)
        return (R,vectorops.madd(t,[0.,0.,1.],-self.dist))
