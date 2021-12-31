from .core import Block
from .utils import BlockSignal
from klampt.math import vectorops
from collections import deque

class FIRFilter(Block):
    """An estimator that filters some other signal using a Finite Impulse Response
    filter.  `b` is the vector of coefficients:
    
        y[t] = x[t]*b[0] + ... + x[t-k+1]*b[k-1].

    For example, a k-moving average filter would set the b vector to
    [1/k,...,1/k]
    """
    def __init__(self,b):
        Block.__init__(self,1,1)
        self.b = b
        assert hasattr(b,'__iter__')
        assert len(b) > 0
        self.history = deque()

    def advance(self,x):
        if hasattr(x,'__iter__'):
            res = vectorops.mul(x,self.b[0])
            assert len(self.history)+1 <= len(self.b)
            for i,v in enumerate(self.history):
                res = vectorops.madd(res,v,self.b[i+1])
        else:
            res = x*self.b[0]
            assert len(self.history)+1 <= len(self.b)
            for i,v in enumerate(self.history):
                res += v*self.b[i+1]
        #advance history
        self.history.appendleft(x)
        while len(self.history) >= len(self.b):
            self.history.pop()
        return res

    def __getstate__(self):
        return {'history':self.history,'b':self.b}

    def __setstate__(self,state):
        self.history = state['history']
        self.b = state['b']

    def signal(self,type,**inputs):
        if type=='reset':
            self.history = deque()

