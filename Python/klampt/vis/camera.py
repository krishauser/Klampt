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
    """Returns the matrix that maps world axes 1,2,3 to the
    camera's coordinate system (right,down,forward) (assuming no camera motion).
    
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
        raise (R,vectorops.mul(so3.apply(R,self.pos),-1.0))

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
        """Returns the camera transform."""
        o = orientation_matrix(*self.ori)
        Ry = so3.rotation([0.,1.,0.],self.rot[0])
        Rx = so3.rotation([1.,0.,0.],self.rot[1])
        Rz = so3.rotation([0.,0.,1.],self.rot[2])
        R = so3.mul(Ry,so3.mul(Rx,Rz))
        R = so3.mul(o,R);

        t = vectorops.mul(so3.apply(R,self.tgt),-1.0)
        return (R,vectorops.add(t,[0.,0.,-self.dist]))

    def set_orientation(self,R,ori=None):
        """Sets the orientation of the camera to the so3 element R. 

        If ori is provided, it is an orientation list (e.g., ['x','y','z'])
        that tells the function how to interpret the columns of R in terms of
        the right, down, and fwd axes of the camera.  Its default value is 
        ['x','y','z']."""
        if ori is not None:
            oR = orientation_matrix(*ori)
            R = so3.mul(R,oR)
        #Ry = so3.rotation([0,1,0],self.rot[0])
        #Rx = so3.rotation([1,0,0],self.rot[1])
        #Rz = so3.rotation([0,0,1],self.rot[2])
        #set self.rot to fulfill constraint R = Ry*Rx*Rz
        #     [cy  0 sy][1  0   0][cz -sz 0]   [cy  0 sy][cz   -sz    0]
        # R = [0   1  0][0 cx -sx][sz  cz 0] = [0   1 0 ][cxsz cxcz -sx]
        #     [-sy 0 cy][0 sx  cx][0   0  1]   [-sy 0 cy][sxsz sxcz  cx]
        #    [cycz+sysxsz  -cysz+sysxcz  sycx]
        #  = [cxsz          cxcz         -sx ]
        #    [-sycz+cysxsz  sysz+cysxcz cycx ]
        m = so3.matrix(R)
        cx = m[1][0]**2 + m[1][1]**2
        sx = -m[1][2]
        self.rot[1] = math.atan2(sx,cx)
        if abs(cx) > 1e-5:
            sz = m[1][0]
            cz = m[1][1]
            sy = m[0][2]
            cy = m[2][2]
            self.rot[2] = math.atan2(sz,cz)
            self.rot[0] = math.atan2(sy,cy)
        else:
            #near vertical, have redundancy, set Ry=0 (so cy=1, sy=0)
            self.rot[0] = 0
            cz = m[0][0]
            sz= -m[0][1]
            self.rot[2] = math.atan2(sz,cz)


        