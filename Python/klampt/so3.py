"""Operations for rigid rotations in Klampt.  All rotations are
represented by a 9-list specifying the entries of the rotation matrix
in column major form.

These are useful for interfacing with C code.
"""

import math
import vectorops

def __str__(R):
    """Converts a rotation to a string."""
    return '\n'.join([' '.join([str(ri) for ri in r]) for r in matrix(R)])

def identity():
    """Returns the identity rotation"""
    return [1.,0.,0.,0.,1.,0.,0.,0.,1.]

def inv(R):
    """Inverts the rotation"""
    Rinv = [R[0],R[3],R[6],R[1],R[4],R[7],R[2],R[5],R[8]]
    return Rinv

def apply(R,point):
    """Applies the rotation to a point"""
    return (R[0]*point[0]+R[3]*point[1]+R[6]*point[2],
            R[1]*point[0]+R[4]*point[1]+R[7]*point[2],
            R[2]*point[0]+R[5]*point[1]+R[8]*point[2])

def matrix(R):
    """Returns the 3x3 rotation matrix corresponding to R"""
    return [[R[0],R[3],R[6]],
            [R[1],R[4],R[7]],
            [R[2],R[5],R[8]]]

def from_matrix(mat):
    """Returns an R corresponding to the 3x3 rotation matrix mat"""
    R = [mat[0][0],mat[1][0],mat[2][0],mat[0][1],mat[1][1],mat[2][1],mat[0][2],mat[1][2],mat[2][2]]
    return R

def mul(R1,R2):
    """Multiplies two rotations."""
    m1=matrix(R1)
    m2T=matrix(inv(R2))
    mres = matrix(identity())
    for i in xrange(3):
        for j in xrange(3):
            mres[i][j] = vectorops.dot(m1[i],m2T[j])
    #print "Argument 1"
    #print __str__(R1)
    #print "Argument 2"
    #print __str__(R2)
    #print "Result"
    R = from_matrix(mres)
    #print __str__(R)
    return R

def trace(R):
    """Computes the trace of the rotation matrix."""
    return R[0]+R[4]+R[8]

def angle(R):
    """Returns absolute deviation of R from identity"""
    ctheta = (trace(R) - 1.0)*0.5
    return math.acos(max(min(ctheta,1.0),-1.0))

def moment(R):
    """Returns the moment w (exponential map) representation of R such
    that e^[w] = R.  Equivalent to axis-angle representation with
    w/||w||=axis, ||w||=angle."""
    theta = angle(R)
    if abs(theta-math.pi)<1e-5:
        #can't do normal version because the scale factor reaches a singularity
        x2=(R[0]+1.)*0.5
        y2=(R[4]+1.)*0.5
        z2=(R[8]+1.)*0.5
        if x2 < 0:
            assert(x2>-1e-5)
            x2=0
        if y2 < 0:
            assert(y2>-1e-5)
            y2=0
        if z2 < 0:
            assert(z2>-1e-5)
            z2=0
        x = math.pi*math.sqrt(x2)
        y = math.pi*math.sqrt(y2)
        z = math.pi*math.sqrt(z2)
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
        return [x,y,z]
    #normal
    scale = 0.5
    if abs(theta) > 1e-5:
        scale = 0.5*theta/math.sin(theta)
    x = (R[3+2]-R[6+1]) * scale;
    y = (R[6+0]-R[0+2]) * scale;
    z = (R[0+1]-R[3+0]) * scale;
    return [x,y,z]

def axis_angle(R):
    """Returns the (axis,angle) pair representing R"""
    m = moment(R)
    return (vectorops.unit(m),vectorops.norm(m))

def from_axis_angle(aa):
    """Converts an axis-angle representation (axis,angle) to a 3D rotation
    matrix."""
    return rotation(aa[0],aa[1])

def from_moment(w):
    """Converts a moment representation w to a 3D rotation matrix."""
    length = vectorops.norm(w)
    if length < 1e-7: return identity()
    return rotation(vectorops.mul(w,1.0/length),length)
    
def distance(R1,R2):
    """Returns the absolute angle one would need to rotate in order to get
    from R1 to R2"""
    R = mul(R1,inv(R2))
    return angle(R)

def error(R1,R2):
    """Returns a 3D "difference vector" that describes how far R1 is from R2.
    More precisely, this is the Lie derivative."""
    R = mul(R1,inv(R2))
    return moment(R)

def cross_product(w):
    """Returns the cross product matrix associated with w.

    The matrix [w]R is the derivative of the matrix R as it rotates about
    the axis w/||w|| with angular velocity ||w||.
    """
    return [0.,w[2],-w[1],  -w[2],0.,w[0],  w[1],-w[0],0.]

def rotation(axis,angle):
    """Given a unit axis and an angle in radians, returns the rotation
    matrix."""
    cm = math.cos(angle)
    sm = math.sin(angle)

    #m = s[r]-c[r][r]+rrt = s[r]-c(rrt-I)+rrt = cI + rrt(1-c) + s[r]
    R = vectorops.mul(cross_product(axis),sm)
    for i in xrange(3):
        for j in xrange(3):
            R[i*3+j] += axis[i]*axis[j]*(1.-cm)
    R[0] += cm
    R[4] += cm
    R[8] += cm
    return R

def canonical(v):
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
    R = v + [0.]*6
    (x,y,z) = tuple(v)
    scale = (1.0-x)/(1.0-x*x);
    R[3]= -y;
    R[4]= x + scale*z*z;
    R[5]= -scale*y*z;
    R[6]= -z;
    R[7]= -scale*y*z;
    R[8]= x + scale*y*y;
    return R

def vector_rotation(v1,v2):
    """Finds the minimal-angle matrix that rotates v1 to v2.  v1 and v2
    are assumed to be nonzero"""
    a1 = vectorops.unit(v1)
    a2 = vectorops.unit(v2)
    cp = vectorops.cross(a1,a2)
    dp = vectorops.dot(a1,a2)
    if abs(vectorops.norm(cp)) < 1e-4:
        if dp < 0:
            R0 = canonical(a1)
            #return a rotation 180 degrees about the canonical y axis
            return rotation(R0[3:6],math.pi)
        else:
            return identity()
    else:
        angle = math.acos(max(min(dp,1.0),-1.0))
        axis = vectorops.mul(cp,1.0/vectorops.norm(cp))
        return rotation(axis,angle)

def interpolate(R1,R2,u):
    """Interpolate linearly between the two rotations R1 and R2. """
    R = mul(inv(R1),R2)
    m = moment(R)
    angle = vectorops.norm(m)
    if angle==0: return R1
    axis = vectorops.div(m,angle)
    return mul(R1,rotation(axis,angle*u))
