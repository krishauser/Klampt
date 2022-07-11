"""Operations for 2D rotations.  Rotations are represented by a single number
giving the angle in radians.
"""

import math
from typing import List
from ..model.typing import Vector2

twopi = math.pi*2

def identity() -> float:
    """The identity rotation. Here for compatibility with so3.py"""
    return 0

def inv(a : float) -> float:
    """The inverse rotation."""
    return -a

def apply(a : float, pt : Vector2) -> Vector2:
    """Applies a rotation of the angle a about the origin to the point pt."""
    c = math.cos(a)
    s = math.sin(a)
    return [c*pt[0]-s*pt[1],s*pt[0]+c*pt[1]]

def normalize(a : float) -> float:
    """Returns an angle a normalized to the range [0,2pi]."""
    res = a%twopi
    if res < 0:
        return res + twopi
    return res

def diff(a : float, b : float) -> float:
    """returns the CCW difference between angles a and b, i.e. the amount
    that you'd neet to rotate from b to get to a.  The result is in the
    range [-pi,pi]."""
    d = normalize(a)-normalize(b)
    if d < -math.pi: return d+twopi
    if d > math.pi: return d-twopi
    return d

def interp(a : float, b : float, u : float) -> float:
    """interpolates on the geodesic between angles a and b, with interpolation
    parameter u in the range [0,1] (can also be used for extrapolation)."""
    return a + diff(b,a)*u

def compose(a : float, b : float) -> float:
    """Returns the composition of rotations a and b."""
    return a+b

def matrix(a : float) -> List[List[float]]:
    """Returns the 2x2 rotation matrix representing a rotation about the
    angle a."""
    c = math.cos(a)
    s = math.sin(a)
    return [[c,-s],[s,c]]

def from_matrix(R : List[List[float]]) -> float:
    """Returns the rotation angle of a rotation matrix."""
    return math.atan2(R[1][0],R[0][0])

def ndarray(a : float) -> "ndarray":
    """Returns the Numpy array representing a 2D rotation about
    the angle a."""
    import numpy
    return numpy.array(matrix(a))

def from_ndarray(R : "ndarray") -> float:
    """Returns the rotation angle of a rotation matrix."""
    return from_matrix(R)
