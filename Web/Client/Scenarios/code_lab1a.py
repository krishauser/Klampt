#Lab 1a stub code
import math

def lab1a(point1,point2):
    #TODO: Return a tuple (length,angle), where length is the
    #length of the segment with endpoints point1 and point2,
    #and angle is the angle of the vector from point 1 to point 2 in
    #degrees, in the range [0,360).
    #    
    #As usual, angle 0 points along the +x axis, 90 points along +y,
    #180 points along -x, and 270 points along -y
    length = 0
    angle = 0
    displacement = [b-a for (a,b) in zip(point1,point2)]
    return (length,angle)

def fuzzy_eq(a,b,eps=1e-8):
    """Returns true if a is within +/- eps of b."""
    return abs(a-b)<=eps

def selfTest():
    """You may use this function to make sure your values are correct"""
    assert lab1a((0,0),(1,0)) == (1,0)
    assert lab1a((0,0),(10,0)) == (10,0)
    assert lab1a((0,0),(0,10)) == (10,90)
    assert lab1a((0,0),(0,-10)) == (10,270)
    assert lab1a((55,55),(55,45)) == (10,270)
    assert lab1a((55,55),(55,55)) == (0,0)
    #test whether length is symmetric, and angle has a symmetry +/- 180
    val1 = lab1a((-53,74),(39.3,93.5))
    val2 = lab1a((39.3,93.5),(-53,74))
    assert val1[0] == val2[0]
    assert fuzzy_eq(180+val1[1],val2[1])
    return

def source_motion(t):
    return 0,0
    #comment out the above line to make sure your lab1a routine works when the source is modified
    return math.sin(t),math.cos(t)

def target_motion(t):
    return 1+math.sin(t+math.pi),math.cos(t*2)
