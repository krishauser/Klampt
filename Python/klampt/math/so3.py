"""Operations for rigid rotations in Klampt.  All rotations are
represented by a 9-list specifying the entries of the rotation matrix
in column major form.

In other words, given a 3x3 matrix::

    [a11,a12,a13]
    [a21,a22,a23]
    [a31,a32,a33],

Klamp't represents the matrix as a list [a11,a21,a31,a12,a22,a32,a13,a23,a33].

The reasons for this representation are 1) simplicity, and 2) a more
convenient interface with C code.

"""

import math
from . import vectorops
from typing import Tuple,Callable
from ..model.typing import Rotation,Matrix3,Vector3

def __str__(R : Rotation) -> str:
    """Converts a rotation to a string."""
    return '\n'.join([' '.join([str(ri) for ri in r]) for r in matrix(R)])

def identity() -> Rotation:
    """Returns the identity rotation."""
    return [1.,0.,0.,0.,1.,0.,0.,0.,1.]

def inv(R : Rotation) -> Rotation:
    """Inverts the rotation."""
    Rinv = [R[0],R[3],R[6],R[1],R[4],R[7],R[2],R[5],R[8]]
    return Rinv

def apply(R : Rotation, point : Vector3) -> Vector3:
    """Applies the rotation to a point."""
    return (R[0]*point[0]+R[3]*point[1]+R[6]*point[2],
            R[1]*point[0]+R[4]*point[1]+R[7]*point[2],
            R[2]*point[0]+R[5]*point[1]+R[8]*point[2])

def matrix(R : Rotation) -> Matrix3:
    """Returns the 3x3 rotation matrix corresponding to R."""
    return [[R[0],R[3],R[6]],
            [R[1],R[4],R[7]],
            [R[2],R[5],R[8]]]

def from_matrix(mat : Matrix3) -> Rotation:
    """Returns a rotation R corresponding to the 3x3 rotation matrix mat."""
    R = [mat[0][0],mat[1][0],mat[2][0],mat[0][1],mat[1][1],mat[2][1],mat[0][2],mat[1][2],mat[2][2]]
    return R

def ndarray(R : Rotation) -> "ndarray":
    """Returns the 3x3 numpy rotation matrix corresponding to R."""
    import numpy
    return numpy.array(matrix(R))

def from_ndarray(mat : "ndarray") -> Rotation:
    """Returns a rotation R corresponding to the 3x3 rotation matrix mat."""
    return mat.T.flatten().tolist()

def mul(R1 : Rotation, R2 : Rotation) -> Rotation:
    """Multiplies two rotations."""
    if len(R1) != 9: raise ValueError("R1 is not a rotation matrix")
    if len(R2) != 9: raise ValueError("R2 is not a rotation matrix (did you mean to use apply())?")
    m1=matrix(R1)
    m2T=matrix(inv(R2))
    mres = matrix(identity())
    for i in range(3):
        for j in range(3):
            mres[i][j] = vectorops.dot(m1[i],m2T[j])
    R = from_matrix(mres)
    return R

def trace(R : Rotation) -> float:
    """Computes the trace of the rotation matrix."""
    return R[0]+R[4]+R[8]

def angle(R : Rotation) -> float:
    """Returns absolute deviation of R from identity"""
    ctheta = (trace(R) - 1.0)*0.5
    return math.acos(max(min(ctheta,1.0),-1.0))

def rpy(R : Rotation) -> Vector3:
    """Converts a rotation matrix to a roll,pitch,yaw angle triple.
    The result is given in radians."""
    sign = lambda x: 1 if x > 0 else (-1 if x < 0 else 0)

    m = matrix(R)
    _sb = min(1.0, max(m[2][0],-1.0))
    b = -math.asin(_sb) # m(2,0)=-sb
    cb = math.cos(b)
    if abs(cb) > 1e-7:
        ca = m[0][0]/cb   #m(0,0)=ca*cb
        ca = min(1.0,max(ca,-1.0))
        if sign(m[1][0]) == sign(cb): #m(1,0)=sa*cb
            a = math.acos(ca);
        else:
            a = 2*math.pi - math.acos(ca)

        cc = m[2][2] / cb  #m(2,2)=cb*cc
        cc = min(1.0,max(cc,-1.0))
        if sign(m[2][1]) == sign(cb): #m(2,1)=cb*sc
            c = math.acos(cc)
        else:
            c = math.pi*2 - math.acos(cc)
    else: 
        #b is close to 90 degrees, i.e. cb=0
        #this reduces the degrees of freedom, so we can set c=0
        c = 0
        #m(0,1)=-sa
        _sa = min(1.0, max(m[0][1],-1.0))
        a = -math.asin(_sa);
        if sign(math.cos(a)) != sign(m[1][1]): #m(1,1)=ca
            a = math.pi - a;
    return c,b,a
    
def from_rpy(rollpitchyaw : Vector3) -> Rotation:
    """Converts from roll,pitch,yaw angle triple to a rotation
    matrix.  The triple is given in radians.  The x axis is "roll",
    y is "pitch", and z is "yaw".
    """
    roll,pitch,yaw = rollpitchyaw
    Rx,Ry,Rz = from_axis_angle(((1,0,0),roll)),from_axis_angle(((0,1,0),pitch)),from_axis_angle(((0,0,1),yaw))
    return mul(Rz,mul(Ry,Rx))

def rotation_vector(R : Rotation) -> Vector3:
    """Returns the rotation vector w (exponential map) representation of R such
    that e^[w] = R.  Equivalent to axis-angle representation with
    w/||w||=axis, ||w||=angle."""
    theta = angle(R)
    if abs(theta-math.pi)<0.5:
        #for values close to pi this alternate technique has better numerical
        #performance
        c = math.cos(theta)
        x2=(R[0]-c)/(1.0 - c)
        y2=(R[4]-c)/(1.0 - c)
        z2=(R[8]-c)/(1.0 - c)
        if x2 < 0:
            assert(x2>-1e-5)
            x2=0
        if y2 < 0:
            assert(y2>-1e-5)
            y2=0
        if z2 < 0:
            assert(z2>-1e-5)
            z2=0
        x = theta*math.sqrt(x2)
        y = theta*math.sqrt(y2)
        z = theta*math.sqrt(z2)
        if abs(theta-math.pi) < 1e-5:
            #determined up to sign changes, we know r12=2xy,r13=2xz,r23=2yz
            xy=R[3]
            xz=R[6]
            yz=R[7]
            if(x > y):
                if(x > z):
                    #x is largest
                    if(xy < 0): y=-y
                    if(xz < 0): z=-z
                else:
                    #z is largest
                    if(yz < 0): y=-y
                    if(xz < 0): x=-x
            else:
                if(y > z):
                    #y is largest
                    if(xy < 0): x=-x
                    if(yz < 0): z=-z
                else:
                    #z is largest
                    if(yz < 0): y=-y
                    if(xz < 0): x=-x
        else:
            #alternate technique: use sign of anti-cross product
            eps = theta-math.pi
            if eps*(R[3+2]-R[6+1]) > 0:
                x = -x
            if eps*(R[6+0]-R[0+2]) > 0:
                y = -y
            if eps*(R[0+1]-R[3+0]) > 0:
                z = -z
        return [x,y,z]
    #normal
    scale = 1
    if abs(theta) > 1e-5:
        scale = theta/math.sin(theta)
    return vectorops.mul(deskew(R),scale)

def axis_angle(R : Rotation) -> Tuple:
    """Returns the (axis,angle) pair representing R"""
    m = rotation_vector(R)
    return (vectorops.unit(m),vectorops.norm(m))

def from_axis_angle(aa : Tuple) -> Rotation:
    """Converts an axis-angle representation (axis,angle) to a 3D rotation
    matrix."""
    return rotation(aa[0],aa[1])

def from_rotation_vector(w : Vector3) -> Rotation:
    """Converts a rotation vector representation w to a 3D rotation matrix."""
    length = vectorops.norm(w)
    if length < 1e-7: return identity()
    return rotation(vectorops.mul(w,1.0/length),length)

#aliases for rotation_vector and from_rotation_vector
moment = rotation_vector
from_moment = from_rotation_vector

def from_quaternion(q : Tuple) -> Rotation:
    """Given a unit quaternion (w,x,y,z), produce the corresponding rotation
    matrix."""
    w,x,y,z = q
    x2 = x + x; y2 = y + y; z2 = z + z;
    xx = x * x2;   xy = x * y2;   xz = x * z2;
    yy = y * y2;   yz = y * z2;   zz = z * z2;
    wx = w * x2;   wy = w * y2;   wz = w * z2;

    a11 = 1.0 - (yy + zz)
    a12 = xy - wz
    a13 = xz + wy
    a21 = xy + wz
    a22 = 1.0 - (xx + zz)
    a23 = yz - wx
    a31 = xz - wy
    a32 = yz + wx
    a33 = 1.0 - (xx + yy)
    return [a11,a21,a31,a12,a22,a32,a13,a23,a33]

def quaternion(R : Rotation) -> Tuple:
    """Given a Klamp't rotation representation, produces the corresponding
    unit quaternion (w,x,y,z)."""
    tr = trace(R) + 1.0;
    a11,a21,a31,a12,a22,a32,a13,a23,a33 = R

    #If the trace is nonzero, it's a nondegenerate rotation
    if tr > 1e-5:
        s = math.sqrt(tr)
        w = s * 0.5
        s = 0.5 / s
        x = (a32 - a23) * s
        y = (a13 - a31) * s
        z = (a21 - a12) * s
        return vectorops.unit((w,x,y,z))
    else:
        #degenerate it's a rotation of 180 degrees
        nxt = [1, 2, 0]
        #check for largest diagonal entry
        i = 0
        if a22 > a11: i = 1
        if a33 > max(a11,a22): i = 2
        j = nxt[i]
        k = nxt[j]
        M = matrix(R)

        q = [0.0]*4
        s = math.sqrt((M[i][i] - (M[j][j] + M[k][k])) + 1.0);
        q[i] = s * 0.5
    
        if abs(s)<1e-7:
            raise ValueError("Could not solve for quaternion... Invalid rotation matrix?")
        else:
            s = 0.5 / s;
            q[3] = (M[k][j] - M[j][k]) * s;
            q[j] = (M[i][j] + M[j][i]) * s;
            q[k] = (M[i][k] + M[k][i]) * s;
        w,x,y,z = q[3],q[0],q[1],q[2]
        return vectorops.unit([w,x,y,z])
    
def distance(R1 : Rotation, R2 : Rotation) -> float:
    """Returns the absolute angle one would need to rotate in order to get
    from R1 to R2"""
    R = mul(R1,inv(R2))
    return angle(R)

def error(R1 : Rotation, R2 : Rotation) -> float:
    """Returns a 3D "difference vector" that describes how far R1 is from R2.
    More precisely, this is the (local) Lie derivative, which is the rotation 
    vector representation of R1*R2^T.

    Fun fact: the error w=error(R1,R2) is related to the derivative of
    interpolate(R2,R1,u) at u=0 by
    d/du interpolate(R2,R1,0) = mul(cross_product(w),R2).
    
    You can also recover R1 from w via R1 = mul(from_moment(w),R2).
    """
    R = mul(R1,inv(R2))
    return moment(R)

def cross_product(w : Vector3) -> Rotation:
    """Returns the cross product matrix associated with w.

    The matrix [w]R is the derivative of the matrix R as it rotates about
    the axis w/||w|| with angular velocity ||w||.
    """
    return [0.,w[2],-w[1],  -w[2],0.,w[0],  w[1],-w[0],0.]

def diag(R : Rotation) -> Vector3:
    """Returns the diagonal of the 3x3 matrix reprsenting the so3 element R."""
    return [R[0],R[4],R[8]]

def deskew(R : Rotation) -> Vector3:
    """If R is a (flattened) cross-product matrix of the 3-vector w, this will
    return w.  Otherwise, it will return a representation w of (R-R^T)/2 (off
    diagonals of R) such that (R-R^T)/2 = cross_product(w). """
    return [0.5*(R[5]-R[7]),0.5*(R[6]-R[2]),0.5*(R[1]-R[3])]

def rotation(axis : Vector3, angle: float) -> Rotation:
    """Given a unit axis and an angle in radians, returns the rotation
    matrix."""
    cm = math.cos(angle)
    sm = math.sin(angle)

    #m = s[r]-c[r][r]+rrt = s[r]-c(rrt-I)+rrt = cI + rrt(1-c) + s[r]
    R = vectorops.mul(cross_product(axis),sm)
    for i in range(3):
        for j in range(3):
            R[i*3+j] += axis[i]*axis[j]*(1.-cm)
    R[0] += cm
    R[4] += cm
    R[8] += cm
    return R

def canonical(v : Vector3) -> Rotation:
    """Given a unit vector v, finds R that defines a basis [x,y,z] such that
    x = v and y and z are orthogonal"""
    if abs(vectorops.normSquared(v) - 1.0) > 1e-4:
        raise RuntimeError("Nonunit vector supplied to canonical()")
    assert(len(v)==3)
    if abs(v[0]-1.0) < 1e-5:
        return identity()
    elif abs(v[0]+1.0) < 1e-5:
        #flip of basis
        R = identity()
        R[0] = -1.0
        R[4] = -1.0
        return R
    R = list(v) + [0.]*6
    x,y,z = v
    scale = 1.0/(1.0+x);
    R[3]= -y;
    R[4]= x + scale*z*z;
    R[5]= -scale*y*z;
    R[6]= -z;
    R[7]= -scale*y*z;
    R[8]= x + scale*y*y;
    return R

def align(a : Vector3, b : Vector3) -> Rotation:
    """Returns a minimal-angle rotation that aligns the vector a to align with
    the vector b. Both a and b must be nonzero."""
    an = vectorops.norm(a)
    bn = vectorops.norm(b)
    if abs(an) < 1e-5 or abs(bn) < 1e-5:
        return identity()
    a = vectorops.mul(a,1.0/an)
    b = vectorops.mul(b,1.0/bn)
    v = vectorops.cross(a,b)
    c = vectorops.dot(a,b)
    if abs(c+1)<1e-5: #rotation of pi
        v = vectorops.cross(a,[0,0,1])
        vn = vectorops.norm(v)
        if vn < 1e-5:
            v = vectorops.cross(a,[0,1,0])
            vn = vectorops.norm(v)
        return rotation(vectorops.mul(v,1.0/vn),math.pi)
    vhat = cross_product(v)
    vhat2 = mul(vhat,vhat)
    return vectorops.madd(vectorops.add(identity(),vhat),vhat2,1.0/(1.0+c))

def interpolate(R1 : Rotation, R2 : Rotation, u : float) -> Rotation:
    """Interpolate linearly between the two rotations R1 and R2. """
    R = mul(inv(R1),R2)
    m = moment(R)
    angle = vectorops.norm(m)
    if angle==0: return R1
    axis = vectorops.div(m,angle)
    return mul(R1,rotation(axis,angle*u))

def interpolator(R1 : Rotation, R2 : Rotation) -> Callable:
    """Returns a function of one parameter u that interpolates linearly
    between the two rotations R1 and R2. After f(u) is constructed, calling
    f(u) is about 2x faster than calling interpolate(R1,R2,u)."""
    R = mul(inv(R1),R2)
    m = moment(R)
    angle = vectorops.norm(m)
    if angle==0:
        axis = [1,0,0]
    else:
        axis = vectorops.div(m,angle)
    def f(u,R1=R1,axis=axis,angle=angle):
        return mul(R1,rotation(axis,angle*u))
    return f

def det(R : Rotation) -> float:
    """Returns the determinant of the 3x3 matrix R"""
    m = matrix(R)
    return m[0][0]*m[1][1]*m[2][2]+m[0][1]*m[1][2]*m[2][0]+m[0][2]*m[1][0]*m[2][1]-m[0][0]*m[1][2]*m[2][1]-m[0][1]*m[1][0]*m[2][2]-m[0][2]*m[1][1]*m[2][0]

def is_rotation(R : Rotation, tol=1e-5) -> bool:
    """Returns true if R is a rotation matrix, i.e. is orthogonal to the given tolerance and has + determinant"""
    RRt = mul(R,inv(R))
    err = vectorops.sub(RRt,identity())
    if any(abs(v) > tol for v in err):
        return False
    if det(R) < 0: 
        return False
    return True

def sample() -> Rotation:
    """Returns a uniformly distributed rotation matrix."""
    import random
    q = [random.gauss(0,1),random.gauss(0,1),random.gauss(0,1),random.gauss(0,1)]
    q = vectorops.unit(q)
    theta = math.acos(q[3])*2.0
    if abs(theta) < 1e-8:
        m = [0,0,0]
    else:
        m = vectorops.mul(vectorops.unit(q[0:3]),theta)
    return from_moment(m)
