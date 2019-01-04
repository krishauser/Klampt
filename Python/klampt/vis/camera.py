"""This module defines a basic set of 3D cameras that can be controlled
by a GUI."""

from ..math import so3,se3,vectorops

basis_vectors = {'x':[1.0,0.0,0.0],
                 '-x':[-1.0,0.0,0.0],
                 'y':[0.0,1.0,0.0],
                 '-y':[0.0,-1.0,0.0],
                 'z':[0.0,0.0,1.0],
                 '-z':[0.0,0.0,-1.0]}

def orientation_matrix(axis1,axis2,axis3):
    """Returns the matrix that maps the camera's identity coordinate system (right,down,forward)
    to world axes 1,2,3 (assuming no camera translation).
    
    Each axis can be either a 3-tuple or any element of
    ['x','y','z','-x','-y','-z']"""
    if isinstance(axis1,str):
        axis1 = basis_vectors[axis1]
    if isinstance(axis2,str):
        axis2 = basis_vectors[axis2]
    if isinstance(axis3,str):
        axis3 = basis_vectors[axis3]
    return so3.from_matrix([axis1,axis2,axis3])

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
        """Returns the camera transform.  Applying this transform converts points in OpenGL camera
        coordinates (x right, y up, -z forward), to points in world coordinates."""
        o = orientation_matrix(*self.ori)
        Rz = so3.rotation([0.,0.,1.],self.rot[2])
        Rx = so3.rotation([1.,0.,0.],self.rot[1])
        Ry = so3.rotation([0.,1.,0.],self.rot[0])
        R = so3.mul(Rz,so3.mul(Rx,Ry))
        R = so3.mul(R,o);
        raise (R,self.pos)

    def set_matrix(self,T):
        raise NotImplementedError("Can't set free camera matrix yet")

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
        """Returns the camera transform.  Applying this transform converts points in OpenGL camera
        coordinates (x right, y up, -z forward), to points in world coordinates."""
        raise NotImplementedError()

    def set_matrix(self,T):
        raise NotImplementedError("Can't set target camera matrix yet")

class orbit:
    """An orbit camera that is controlled using a rotation, 
    target point, distance, and orientation.

    Attributes:        
        - tgt: target point (in world coordinates)
        - rot: euler angle rotation (roll-pitch-yaw entries relative to default view with fwd = +y, right = +x, up = +z)
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
        """Returns the camera transform.  Applying this transform converts points in OpenGL camera
        coordinates (x right, y up, -z forward), to points in world coordinates."""
        o = orientation_matrix(*self.ori)
        Rz = so3.rotation([0.,0.,1.],self.rot[2])
        Rx = so3.rotation([1.,0.,0.],self.rot[1])
        Ry = so3.rotation([0.,1.,0.],self.rot[0])
        R = so3.mul(Rz,so3.mul(Rx,Ry))
        R = so3.mul(R,o)

        return (R,vectorops.add(self.tgt,so3.apply(R,[0.,0.,self.dist])))

    def set_orientation(self,R,ori=None):
        """Sets the orientation of the camera to the so3 element R. 

        If ori is provided, it is an orientation list (e.g., ['x','y','z'])
        that tells the function how to interpret the columns of R in terms of
        the right, down, and fwd axes of the camera.  Its default value is 
        None.
        """
        import math
        #Rdes*oR*[right,down,fwd] = R_euler(rot)*o*[right,down,fwd]
        if ori is not None:
            o = orientation_matrix(*self.ori)
            oR = orientation_matrix(*ori)
            R = so3.mul(R,so3.mul(so3.inv(oR),o))
        #Ry = so3.rotation([0,1,0],self.rot[0])
        #Rx = so3.rotation([1,0,0],self.rot[1])
        #Rz = so3.rotation([0,0,1],self.rot[2])
        #set self.rot to fulfill constraint R = Rz*Rx*Ry
        #     [cz -sz 0][1  0   0][cy  0 sy]   [cz -szcx  szsx][cy  0 sy]
        # R = [sz  cz 0][0 cx -sx][0   1  0] = [sz  czcx -czsx][0   1 0 ]
        #     [0   0  1][0 sx  cx][-sy 0 cy]   [0    sx     cx][-sy 0 cy]
        #    [czcy-szsxsy   -szcx  czsy+szsxcy ]
        #  = [szcy+czsxsy    czcx  szsy-czsxcy ]
        #    [-cxsy           sx   cxcy        ]
        m = so3.matrix(R)
        cx = math.sqrt(m[0][1]**2 + m[1][1]**2)
        #assume cx is positive (pitch in range [-pi/2,pi/2])
        sx = m[2][1]
        self.rot[1] = math.atan2(sx,cx)
        if abs(cx) > 1e-5:
            sz = -m[0][1]
            cz = m[1][1]
            sy = -m[2][0]
            cy = m[2][2]
            self.rot[2] = math.atan2(sz,cz)
            self.rot[0] = math.atan2(sy,cy)
        else:
            #near vertical, have redundancy, have cx=0,sx = +/-1, set Ry=0 (so cy=1, sy=0)
            self.rot[0] = 0
            cz = m[0][0]
            sz = m[1][0]
            self.rot[2] = math.atan2(sz,cz)

    def set_matrix(self,T):
        """Restores from a matrix retrieved using matrix()"""
        R,t = T
        #self.rot fulfills constraint R = Rz*Rx*Ry*ori
        o = orientation_matrix(*self.ori)
        R2 = so3.mul(R,so3.inv(o))
        #      [czcy-szsxsy   -szcx  czsy+szsxcy ]
        #R2  = [szcy+czsxsy    czcx  szsy-czsxcy ]
        #      [-cxsy           sx   cxcy        ]
        self.set_orientation(R2,None)
        #tgt + R*[0,0,dist] = t
        self.tgt = vectorops.sub(t,so3.apply(R,[0,0,self.dist]))
        