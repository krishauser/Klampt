"""Allows treating tuples/lists as vectors"""

import math

def add(*items):
    """Adds one or more vectors."""
    if len(items) == 0: return 0
    n = len(items[0])
    for v in items:
        if n!=len(v):
            raise RuntimeError('Vector dimensions not equal')
    return [sum([v[i] for v in items]) for i in range(n)]

def madd(a,b,c):
    """Return a+c*b where a and b are vectors."""
    if len(a)!=len(b):
        raise RuntimeError('Vector dimensions not equal')
    return [ai+c*bi for ai,bi in zip(a,b)]

def sub(a,b):
    """Subtract a vector b from a, or subtract a scalar"""
    if hasattr(b,'__iter__'):
        if len(a)!=len(b):
            raise RuntimeError('Vector dimensions not equal')
        return [ai-bi for ai,bi in zip(a,b)]
    else:
        return [ai-b for ai in a]

def mul(a,b):
    """Multiply a vector either elementwise with another vector, or with a
    scalar."""
    if hasattr(b,'__iter__'):
        if len(a)!=len(b):
            raise RuntimeError('Vector dimensions not equal')
        return [ai*bi for ai,bi in zip(a,b)]
    else:
        return [ai*b for ai in a]

def div(a,b):
    """Elementwise division with another vector, or with a scalar."""
    if hasattr(b,'__iter__'):
        if len(a)!=len(b):
            raise RuntimeError('Vector dimensions not equal')
        return [ai/bi for ai,bi in zip(a,b)]
    else:
        return [ai/b for ai in a]

def maximum(a,b):
    """Elementwise max"""
    if hasattr(b,'__iter__'):
        return [max(ai,bi) for ai,bi in zip(a,b)]
    else:
        return [max(ai,b) for ai in a]

def minimum(a,b):
    """Elementwise min"""
    if hasattr(b,'__iter__'):
        return [min(ai,bi) for ai,bi in zip(a,b)]
    else:
        return [min(ai,b) for ai in a]

def dot(a,b):
    """Dot product."""
    if len(a)!=len(b):
        raise RuntimeError('Vector dimensions not equal')
    return sum([a[i]*b[i] for i in range(len(a))])

def normSquared(a):
    """Returns the norm of a, squared."""
    return sum(ai*ai for ai in a)

def norm(a):
    """L2 norm"""
    return math.sqrt(normSquared(a))

def unit(a,epsilon=1e-5):
    """Returns the unit vector in the direction a.  If the norm of
    a is less than epsilon, a is left unchanged."""
    n = norm(a)
    if n > epsilon:
        return mul(a,1.0/n)
    return a[:]

norm_L2 = norm

def norm_L1(a):
    """L1 norm"""
    return sum(abs(ai) for ai in a)

def norm_Linf(a):
    """L-infinity norm"""
    return max(abs(ai) for ai in a)

def distanceSquared(a,b):
    if len(a)!=len(b): raise RuntimeError('Vector dimensions not equal')
    sum=0
    for i in range(len(a)):
        sum = sum + (a[i]-b[i])*(a[i]-b[i])
    return sum

def distance(a,b):
    return math.sqrt(distanceSquared(a,b));

def cross(a,b):
    """Cross product between a 3-vector or a 2-vector"""
    if len(a)!=len(b):
        raise RuntimeError('Vector dimensions not equal')
    if len(a)==3:
        return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
    elif len(a)==2:
        return a[0]*b[1]-a[1]*b[0]
    else:
        raise RuntimeError('Vectors must be 2D or 3D')

def interpolate(a,b,u):
    """Linear interpolation between a and b"""
    return madd(a,sub(b,a),u)
