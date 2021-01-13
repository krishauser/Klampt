from klampt.math import vectorops
from ..controller import ControllerBlock
from .utils import LambdaBlock
from collections import deque

class DerivativeEstimator(ControllerBlock):
    """An estimator computes the derivative of some input (typically 'q') using
    finite differences.  Outputs to 'dq' or ('d'+name in general).
    """
    def __init__(self,name='q',robot=None):
        self.name = name
        self.robot = robot
        self.qlast = None
    def inputNames(self):
        return ['dt',self.name]
    def outputNames(self):
        return ['d'+self.name]
    def getState(self):
        return {'last':self.qlast}
    def setState(self,state):
        self.qlast = state['last']
    def advance(self,**inputs):
        try:
            dt = inputs["dt"]
            q = inputs[self.name]
        except KeyError:
            raise ValueError("Input needs to have value '%s' and timestep 'dt'"%(self.name,))
        if len(q)==0: return None
        if self.qlast==None:
            dq = [0]*len(q)
        else:
            if self.robot==None:
                dq = vectorops.div(self.robot.sub(q,self.qlast),dt)
            else:
                assert(len(self.qlast)==len(q))
                dq = vectorops.div(self.robot.interpolate_deriv(self.qlast,q),dt)
        self.qlast = q
        return {'d'+self.name:dq}

    def signal(self,type,**inputs):
        if type=='reset':
            self.qlast=None


class IntegralEstimator(ControllerBlock):
    """An estimator computes the integral of some input using the 
    trapezoidal rule.
    """
    def __init__(self,name):
        self.name = name
        self.integral = None
    def inputNames(self):
        return ['dt',self.name]
    def outputNames(self):
        return ['I'+self.name]
    def getState(self):
        return self.integral
    def setState(self,state):
        self.integral = state
    def advance(self,**inputs):
        try:
            dt = inputs["dt"]
            v = inputs[self.name]
        except KeyError:
            raise ValueError("Input needs to have value %s and timestep 'dt'"%(self.name,))
        if len(v)==0: return None
        if self.integral is None:
            self.integral = vectorops.mul(v,dt)
        else:
            self.integral = vectorops.madd(self.integral,v,dt)
        result = vectorops.madd(self.integral,v,-0.5*dt)
        return {'I'+self.name:result}

    def signal(self,type,**inputs):
        if type=='reset':
            self.integral=None


class FIRFilter(ControllerBlock):
    """An estimator that filters some other signal using a Finite Impulse Response
    filter.  `b` is the vector of coefficients.

    For example, a k-moving average filter would set the b vector to
    [1/k,...,1/k]
    """
    def __init__(self,argname,b,outname=None):
        self.argname = argname
        self.outname = argname + " filtered" if outname is None else outname
        self.b = b
        assert hasattr(b,'__iter__')
        assert len(b) > 0
        from collections import deque
        self.history = deque()

    def inputNames(self):
        return [self.argname]

    def outputNames(self):
        return [self.outname]

    def advance(self,**inputs):
        val = inputs[self.argname]
        if hasattr(val,'__iter__'):
            res = vectorops.mul(val,self.b[0])
            assert len(self.history)+1 <= len(self.b)
            for i,v in enumerate(self.history):
                res = vectorops.madd(res,v,self.b[i+1])
            if len(self.history) + 1 < len(self.b):
                res = vectorops.madd(res,self.history[-1],sum(self.b[len(self.history)+1:]))
        else:
            res = val*self.b[0]
            assert len(self.history)+1 <= len(self.b)
            for i,v in enumerate(self.history):
                res += v*self.b[i+1]
            if len(self.history) + 1 < len(self.b):
                res += self.history[-1]*sum(self.b[len(self.history)+1:])
        #advance history
        self.history.appendleft(val)
        while len(self.history) >= len(self.b):
            self.history.pop()
        return res

    def getState(self):
        return {'history',self.history}

    def setState(self,state):
        self.history = state['history']

    def signal(self,type,**inputs):
        if type=='reset':
            self.history = deque()
