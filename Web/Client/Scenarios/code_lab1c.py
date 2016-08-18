from klampt import *
import math

def get_rotation_matrix(xform):
    """TODO: return the 2x2 rotation matrix corresponding to the given xform.
    In:
    - xform: a tuple (tx,ty,angle) indicating the translation amount (tx,ty)
      and the rotation amount angle (in degrees)
    Out:
    - R: a list of lists, indicating the rotation matrix corresponding to the
      given angle. The entry R[i][j] is the i'th row and j'th column
      (zero based). 
    """
    #TODO: fill in your entries here
    a,b,c,d=1,0,0,1
    return [[a,b],
            [c,d]]

def get_translation_vector(xform):
    """Return the 2d translation vector corresponding to the given xform.
    In:
    - xform: a tuple (tx,ty,angle) indicating the translation amount (tx,ty)
      and the rotation amount angle (in degrees)
    Out:
    - the translation vector (tx,ty)
    """
    return [xform[0],xform[1]]

def apply_xform(xform,point):
    """Apply a 2D rigid transform to a 2D point P.
    In:
    - xform: a tuple (tx,ty,angle) indicating the translation amount (tx,ty)
      and the rotation amount angle (in degrees)
    - point: a pair (px,py) indicating the coordinates of P in the original
      frame.
    Out:
    - a pair (qx,qy) indicating the coordinates of P after rotating by
      angle and then translating by (tx,ty). 
    """
    (tx,ty,angle) = xform
    t = get_translation_vector(xform)
    R = get_rotation_matrix(xform)
    Rp = [vectorops.dot(R[0],point),vectorops.dot(R[1],point)]
    return vectorops.add(Rp,t)

def lab1c(xform1,xform2):
    """TODO: compose two 2D transforms.  In other words, performs the operation
    xform1*xform2, where xform = lab1c(xform1,xform2) = xform1*xform2 ,
    is the transform that satisfiesapply_xform(xform,p) = apply_xform(xfo
    rm1,apply_xform(xform2,p)) for all points p
    In:
     - xform1: a tuple (tx1,ty1,angle1) (angles in degrees)
     - xform2: a tuple (tx2,ty2,angle2) (angles in degrees)
    Out:
     - xform: a tuple (tx,ty,angle) representing the output transform
     """
    return xform2

def fuzzy_eq(a,b,eps=1e-8):
    """Returns true if a is within +/- eps of b."""
    return abs(a-b)<=eps

def fuzzy_veq(a,b,eps=1e-8):
    return all(fuzzy_eq(ai,bi,eps) for ai,bi in zip(a,b)) 

def selfTest():
    """You may use this function to make sure your values are correct"""
    R=get_rotation_matrix((0,0,0))
    assert fuzzy_veq(R[0],[1,0])
    assert fuzzy_veq(R[1],[0,1])
    R=get_rotation_matrix((0,0,90))
    assert fuzzy_veq(R[0],[0,-1])
    assert fuzzy_veq(R[1],[1,0])
    R=get_rotation_matrix((0,0,180))
    assert fuzzy_veq(R[0],[-1,0])
    assert fuzzy_veq(R[1],[0,-1])
    assert fuzzy_veq(lab1c((0,0,0),(20,30,70)),(20,30,70))
    assert fuzzy_veq(lab1c((-40,-5,0),(20,30,70)),(-20,25,70))
    assert fuzzy_veq(lab1c((0,0,180),(20,30,70)),(-20,-30,250))
    return

