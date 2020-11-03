"""Spline utilities."""
from . import vectorops

def hermite_eval(x1,v1,x2,v2,u):
    """Returns the position a hermite curve with control points
    x1, v1, x2, v2 at the parameter u in [0,1]."""
    assert len(x1)==len(v1)
    assert len(x1)==len(x2)
    assert len(x1)==len(v2)
    u2 = u*u
    u3 = u*u*u
    cx1 = 2.0*u3-3.0*u2+1.0
    cx2 = -2.0*u3+3.0*u2
    cv1 = (u3-2.0*u2+u)
    cv2 = (u3-u2)
    x = [0]*len(x1)
    for i in range(len(x1)):
        x[i] = cx1*x1[i] + cx2*x2[i] + cv1*v1[i] + cv2*v2[i]
    return x


def hermite_deriv(x1,v1,x2,v2,u,order=1):
    """Returns the derivative of a hermite curve with control points
    x1, v1, x2, v2 at the parameter u in [0,1].  If order > 1, higher
    order derivatives are returned."""
    assert len(x1)==len(v1)
    assert len(x1)==len(x2)
    assert len(x1)==len(v2)
    if order == 1:
        u2 = u*u
        dcx1 = (6.0*u2-6.0*u)
        dcx2 = (-6.0*u2+6.0*u)
        dcv1 = 3.0*u2-4.0*u+1.0
        dcv2 = 3.0*u2-2.0*u
        dx = [0]*len(x1)
        for i in range(len(x1)):
            dx[i] = dcx1*x1[i] + dcx2*x2[i] + dcv1*v1[i] + dcv2*v2[i];
        return dx
    elif order == 2:
        ddcx1 = 12*u-6.0
        ddcx2 = -12.0*u+6.0
        ddcv1 = 6.0*u-4.0
        ddcv2 = 6.0*u-2.0
        ddx = [0]*len(x1)
        for i in range(len(x1)):
            ddx[i] = ddcx1*x1[i] + ddcx2*x2[i] + ddcv1*v1[i] + ddcv2*v2[i]
        return ddx
    elif order == 3:
        cx1 = 12
        cx2 = -12.0
        cv1 = 6.0
        cv2 = 6.0
        dddx = [0]*len(x1)
        for i in range(len(x1)):
            dddx[i] = cx1*x1[i] + cx2*x2[i] + cv1*v1[i] + cv2*v2[i]
        return dddx
    elif order == 0:
        return hermite_eval(x1,v1,x2,v2,u)
    else:
        return [0]*len(x1)


def hermite_subdivide(x1,v1,x2,v2,u=0.5):
    """Subdivides a hermite curve into two hermite curves."""
    xm = hermite_eval(x1,v1,x2,v2,u)
    vm = hermite_deriv(x1,v1,x2,v2,u)
    return [(x1,vectorops.mul(v1,u),xm,vectorops.mul(vm,u)),(xm,vectorops.mul(vm,1.0-u),x2,vectorops.mul(v2,1.0-u))]

def hermite_length_bound(x1,v1,x2,v2):
    """Returns an upper bound on the arc length of the hermite curve"""
    return bezier_length_bound(*hermite_to_bezier(x1,v1,x2,v2))

def hermite_to_bezier(x1,v1,x2,v2):
    """Returns the cubic bezier representation of a hermite curve"""
    c1 = vectorops.madd(x1,v1,1.0/3.0)
    c2 = vectorops.madd(x2,v2,-1.0/3.0)
    return x1,c1,c2,x2

def bezier_subdivide(x1,x2,x3,x4,u=0.5):
    """Subdivides a Bezier curve at the parameter u"""
    p1 = vectorops.interpolate(x1,x2,u)
    p2 = vectorops.interpolate(x2,x3,u)
    p3 = vectorops.interpolate(x3,x4,u)
    q1 = vectorops.interpolate(p1,p2,u)
    q2 = vectorops.interpolate(p2,p3,u)
    r1 = vectorops.interpolate(q1,q2,u)
    return [(x1,p1,q1,r1),(r1,q2,p3,x4)]

def bezier_length_bound(x1,x2,x3,x4):
    """Returns an upper bound on the arc length of the bezier curve"""
    return vectorops.distance(x1,x2)+ vectorops.distance(x2,x3) + vectorops.distance(x3,x4)

def bezier_discretize(x1,x2,x3,x4,res,return_params = False):
    """Discretizes a bezier curve into a list of points.  If return_params
    is True, a pair (points,params) is returned where params are the exact
    parameter values for which each point was generated.  Otherwise, just
    the points are returned."""
    stack = [(x1,x2,x3,x4)]
    path = [x1]
    if return_params:
        params = [0.0]
        param_stack = [(0.0,1.0)]
    while len(stack) > 0:
        c = stack[-1]
        stack.pop(-1)
        if return_params:
            a,b = param_stack[-1]
            param_stack.pop(-1)
            if bezier_length_bound(*c) > res:
                half1,half2 = bezier_subdivide(*c)
                stack.append(half2)
                stack.append(half1)
                m=(a+b)*0.5
                param_stack.append((m,b))
                param_stack.append((a,m))
            else:
                assert path[-1] == c[0]
                assert params[-1] == a
                path += c[1:]
                params.append(b)
        else:
            if bezier_length_bound(*c) > res:
                half1,half2 = bezier_subdivide(*c)
                stack.append(half2)
                stack.append(half1)
            else:
                assert path[-1] == c[0]
                path += c[1:]
    if return_params:
        return path,params
    return path

def bezier_to_hermite(x1,x2,x3,x4):
    """Returns the cubic bezier representation of a hermite curve"""
    v1 = vectorops.mul(vectorops.sub(x2,x1),3.0)
    v2 = vectorops.mul(vectorops.sub(x4,x3),3.0)
    return x1,v1,x4,v2
