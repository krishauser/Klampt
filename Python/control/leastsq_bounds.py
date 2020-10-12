# leastsq_bounds.py

from __future__ import division
import numpy as np
from scipy.optimize import leastsq

__date__ = "2012-03-26 mar"

def leastsq_bounds( func, x0, bounds, boundsweight=10, Dfun = None, **kwargs ):
    """ leastsq with bound conatraints lo <= p <= hi
    run leastsq with additional constraints to minimize the sum of squares of
        [func(p) ...]
        + boundsweight * [max( lo_i - p_i, 0, p_i - hi_i ) ...]

    Parameters
    ----------
    func() : a function of parameters `p`
    bounds : an n x 2 list or array `[[lo_0,hi_0], [lo_1, hi_1] ...]`.
        Use e.g. [0, inf]; do not use NaNs.
        A bound e.g. [2,2] pins that x_j == 2.
    boundsweight : weights the bounds constraints
    kwargs : keyword args passed on to leastsq

    Returns
    -------
    exactly as for leastsq,
http://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.leastsq.html

    Notes
    -----
    The bounds may not be met if boundsweight is too small;
    check that with e.g. check_bounds( p, bounds ) below.

    To access `x` in `func(p)`, `def func( p, x=xouter )`
    or make it global, or `self.x` in a class.

    There are quite a few methods for box constraints;
    you'll maybe sing a longer song ...
    Comments are welcome, test cases most welcome.

"""
    # Example: test_leastsq_bounds.py

    myDfun = Dfun
    if bounds is not None  and  boundsweight > 0:
        funcbox = lambda p: \
            np.hstack(( func(p), _inbox( p, bounds, boundsweight )))
        if Dfun != None:
            myDfun = lambda p: np.hstack((Dfun(p),_inbox_jacobian(p,bounds,boundsweight)))
    else:
        funcbox = func
    return leastsq( funcbox, x0, Dfun = myDfun, **kwargs )

def leastsq_bounds2( func, x0, Cfunc, bounds, boundsweight=10, Dfun = None, CDfun = None,**kwargs ):
    """ leastsq with bound conatraints lo <= C(p) <= hi
    run leastsq with additional constraints to minimize the sum of squares of
        [func(p) ...]
        + boundsweight * [max( lo_i - c_i(p), 0, c_i(p) - hi_i ) ...]

    Parameters
    ----------
    func() : a function of parameters `p`
    Cfunc(): a Rn->Rm function of parameters `p`
    bounds : an m x 2 list or array `[[lo_0,hi_0], [lo_1, hi_1] ...]`.
        Use e.g. [0, inf]; do not use NaNs.
        A bound e.g. [2,2] pins that x_j == 2.
    boundsweight : weights the bounds constraints
    kwargs : keyword args passed on to leastsq

    Returns
    -------
    exactly as for leastsq,
http://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.leastsq.html

    Notes
    -----
    The bounds may not be met if boundsweight is too small;
    check that with e.g. check_bounds( p, bounds ) below.

    To access `x` in `func(p)`, `def func( p, x=xouter )`
    or make it global, or `self.x` in a class.

    There are quite a few methods for box constraints;
    you'll maybe sing a longer song ...
    Comments are welcome, test cases most welcome.

"""
    # Example: test_leastsq_bounds.py

    myDfun = Dfun
    if bounds is not None  and  boundsweight > 0:
        funcbox = lambda p: \
            np.hstack(( func(p), _inbox( Cfunc(p), bounds, boundsweight )))
        if Dfun != None:
            myDfun = lambda p: np.hstack((Dfun(p),np.dot(CDfun(p),_inbox_jacobian(Cfunc(p),bounds,boundsweight))))
    else:
        funcbox = func
    return leastsq( funcbox, x0, Dfun = myDfun, **kwargs )


def _inbox( X, box, weight=1 ):
    """ -> [tub( Xj, loj, hij ) ... ]
        all 0  <=>  X in box, lo <= X <= hi
    """
    assert len(X) == len(box), \
        "len X %d != len box %d" % (len(X), len(box))
    return weight * np.array([
        np.fmax( lo - x, 0 ) + np.fmax( 0, x - hi )
            for x, (lo,hi) in zip( X, box )])

def _inbox_jacobian( X, box, weight=1 ):
    """ -> [tub( Xj, loj, hij ) ... ]
        all 0  <=>  X in box, lo <= X <= hi
    """
    assert len(X) == len(box), \
        "len X %d != len box %d" % (len(X), len(box))
    J = np.zeros((len(X),len(X)))
    for i,x,(lo,hi) in enumerate(zip(X,box)):
        if x < lo:
            J[i,i] = -weight
        elif x > hi:
            J[i,i] = weight
    return J

# def tub( x, lo, hi ):
#     """ \___/  down to lo, 0 lo .. hi, up from hi """
#     return np.fmax( lo - x, 0 ) + np.fmax( 0, x - hi )

def check_bounds( X, box ):
    """ print Xj not in box, loj <= Xj <= hij
        return nr not in
    """
    nX, nbox = len(X), len(box)
    assert nX == nbox, \
        "len X %d != len box %d" % (nX, nbox)
    nnotin = 0
    for j, x, (lo,hi) in zip( range(nX), X, box ):
        if not (lo <= x <= hi):
            print "check_bounds: x[%d] %g is not in box %g .. %g" % (j, x, lo, hi)
            nnotin += 1
    return nnotin
