from klampt.math import vectorops
from controller import ControllerBase,LambdaController
from MotionModel import AdaptiveMotionModel

class VelocityEstimator(ControllerBase):
    """An estimator that runs by computing the velocity using finite
    differences."""
    def __init__(self,robot=None):
        self.robot = robot
        self.qlast = None
    def inputNames(self):
        return ['dt','q']
    def outputNames(self):
        return ['dq']
    def getState(self):
        return {'qlast':self.qlast}
    def setState(self,state):
        self.qlast = state['qlast']
    def output_and_advance(self,**inputs):
        try:
            dt = inputs["dt"]
            q = inputs["q"]
        except KeyError:
            raise ValueError("Input needs to have configuration 'q' and timestep 'dt'")
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
        return {'dq':dq}

    def signal(self,type,**inputs):
        if type=='reset':
            self.qlast=None


class DifferenceEstimator(LambdaController):
    """An estimator that produces an output "A - B" for two arguments "A" and "B"."""
    def __init__(self,arg1,arg2):
        def diff(x,y):
            if hasattr(x,'__iter__'):
                return vectorops.sub(x,y)
            else:
                return x-y
        LambdaController.__init__(self,diff,[arg1,arg2],arg1+" - "+arg2)


class FIRFilter(ControllerBase):
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

    def output(self,**inputs):
        val = inputs[self.argname]
        if hasattr(val,'__iter__'):
            res = vectorops.mul(val,self.b[0])
            assert len(self.history)+1 <= len(self.b)
            for i,v in enumerate(self.history):
                res = vectorops.madd(res,v,self.b[i+1])
            if len(self.history) + 1 < len(self.b):
                res = vectorops.madd(res,self.history[-1],sum(self.b[len(self.history)+1:]))
            return res
        else:
            res = val*self.b[0]
            assert len(self.history)+1 <= len(self.b)
            for i,v in enumerate(self.history):
                res += v*self.b[i+1]
            if len(self.history) + 1 < len(self.b):
                res += self.history[-1]*sum(self.b[len(self.history)+1:])
            return res

    def advance(self,**inputs):
        val = inputs[self.argname]
        self.history.appendleft(val)
        while len(self.history) >= len(self.b):
            self.history.pop()

    def getState(self):
        return {'history',self.history}

    def setState(self,state):
        self.history = state['history']

    def signal(self,type,**inputs):
        if type=='reset':
            self.history = deque()
