from .core import Block
from ..system_id import LinearSystemID

class LinearSystemIDBlock(Block):
    """Outputs the result from linear system ID to matrices A, B  and vector C.

    Inputs:
        - y: the next state
        - x: the input state
        - u: the control vector
    
    Outputs:
        - A, B, C: y ~= A*x + B*u + C
    """
    def __init__(self,m,n):
        self.estimator = LinearSystemID(m,n)
        Block.__init__(self,['y','x','u'],['A','B','C'])
    def advance(self,y,x,u):
        self.estimator.add(x,u,y)
        A,B,C = self.estimator.getOutput()
        return A,B,C


class LinearSystemIDPredictionBlock(Block):
    """Outputs the prediction from linear system ID to the output,
    uses the actual observation to update the model from the last time step.
    """
    def __init__(self,m,n):
        Block.__init__(self,['x','u','ylast'],['y'])
        self.xlast = None
        self.ulast = None
        self.estimator = LinearSystemID(m,n)
    def advance(self,x,u,ylast):
        if self.xlast is not None:
            self.estimator.add(self.xlast,self.ulast,ylast)
        ypred = self.estimator.getOutput(x,u)
        self.xlast = x
        self.ulast = u
        return ypred

