import math
from klampt.math import vectorops,so3
#import numpy as np

def solve_2R_inverse_kinematics(x,y,L1=1,L2=1):
    """For a 2R arm centered at the origin, solves for the joint angles
    (q1,q2) that places the end effector at (x,y).

    The result is a list of up to 2 solutions, e.g. [(q1,q2),(q1',q2')].
    """
    D = vectorops.norm((x,y))
    thetades = math.atan2(y,x)
    if D == 0:
        raise ValueError("(x,y) at origin, infinite # of solutions")
    c2 = (D**2-L1**2-L2**2)/(2.0*L1*L2)
    q2s = []
    if c2 < -1:
        print "solve_2R_inverse_kinematics: (x,y) inside inner circle"
        return []
    elif c2 > 1:
        print "solve_2R_inverse_kinematics: (x,y) out of reach"
        return []
    else:
        if c2 == 1:
            q2s = [math.acos(c2)]
        else:
            q2s = [math.acos(c2),-math.acos(c2)]
    res = []
    for q2 in q2s:
        thetaactual = math.atan2(math.sin(q2),L1+L2*math.cos(q2))
        q1 = thetades - thetaactual
        res.append((q1,q2))
    return res


def solve_3R_forward_kinematics(q1,q2,q3,L1=1,L2=1,L3=1):
    """Returns a list of (x,y,theta) triples for each link
    for a planar, 3R manipulator with link lengths L1, L2, L3.
    It also returns the end effector transform."""
    T1 = (0,0,q1)
    dx1 = (L1*math.cos(T1[2]),L1*math.sin(T1[2]))
    T2 = vectorops.add((T1[0],T1[1]),dx1)+[T1[2]+q2]
    dx2 = (L2*math.cos(T2[2]),L2*math.sin(T2[2]))
    T3 = vectorops.add((T2[0],T2[1]),dx2)+[T2[2]+q3]
    dx3 = (L3*math.cos(T3[2]),L2*math.sin(T3[2]))
    T4 = vectorops.add((T3[0],T3[1]),dx3)+[T3[2]]
    return [T1,T2,T3,T4]

def solve_3R_inverse_kinematics(x,y,theta,L1=1,L2=1,L3=1):
    """IMPLEMENT ME: for a planar, 3R manipulator with link lengths L1, L2, L3,
    solve for the joint angles (q1,q2,q3) such that the end effector
    is placed at x,y and is oriented along the angle theta.

    The current implementation is incorrect, and only tries to reach x,y
    with the second and third joint angle, using 2R manipulator routine
    presented in class.  Your implementation should correctly place the end
    effector to be oriented at the angle theta (which will require using all
    three joint angles).

    In general there will be up to two solutions.  The result is a list of
    solutions.  Your implementation should correctly return 0 solutions
    if the request is infeasible.
    """
    q1,q2,q3 = 0,0,0
    (x1,y1) = (L1*math.cos(q1),L1*math.sin(q1))
    q23s = solve_2R_inverse_kinematics(x-x1,y-y1,L2,L3)
    res = []
    for q2,q3 in q23s:
        res.append((q1,q2,q3))
    return res

def run_ex1():
    print solve_3R_forward_kinematics(0,0,0)

    #test 1
    xdes = (3,0,0)
    qs = solve_3R_inverse_kinematics(*xdes)
    print "xdes =",xdes,
    if len(qs)==0:
        print "failed"
    else:
        print len(qs),"solutions:"
    for q in qs:
        print "    q =",q,"fk =",solve_3R_forward_kinematics(*q)[3]
        
    #note the orientation error
    xdes = (2.0,0.5,0)
    qs = solve_3R_inverse_kinematics(*xdes)
    print "xdes =",xdes,
    if len(qs)==0:
        print "failed"
    else:
        print len(qs),"solutions:"
    for q in qs:
        print "    q =",q,"fk =",solve_3R_forward_kinematics(*q)[3]

    #note the orientation error
    xdes = (2.0,0.0,math.pi*0.5)
    qs = solve_3R_inverse_kinematics(*xdes)
    print "xdes =",xdes,
    if len(qs)==0:
        print "failed"
    else:
        print len(qs),"solutions:"
    for q in qs:
        print "    q =",q,"fk =",solve_3R_forward_kinematics(*q)[3]

    #this one is feasible, but the current implementation fails
    xdes = (-3.0,0.0,0)
    qs = solve_3R_inverse_kinematics(*xdes)
    print "xdes =",xdes,
    if len(qs)==0:
        print "failed"
    else:
        print len(qs),"solutions:"
    for q in qs:
        print "    q =",q,"fk =",solve_3R_forward_kinematics(*q)[3]

    #this one is infeasible
    xdes = (-2.0,0.0,-math.pi*0.5)
    qs = solve_3R_inverse_kinematics(*xdes)
    print "xdes =",xdes,
    if len(qs)==0:
        print "failed"
    else:
        print len(qs),"solutions:"
    for q in qs:
        print "    q =",q,"fk =",solve_3R_forward_kinematics(*q)[3]

if __name__ == "__main__":
    run_ex1()

