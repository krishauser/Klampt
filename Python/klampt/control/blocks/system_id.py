from ..controller import ControllerBlock
from ..system_id import LinearSystemID

class LinearSystemIDBlock(ControllerBlock):
    """Outputs the result from linear system ID to matrics A, B  and vector C.
    """
    def __init__(self,m,n,xname,uname,yname,Aname='A',bname='B',cname='C'):
        self.xname = xname
        self.uname = uname
        self.yname = yname
        self.Aname = Aname
        self.bname = bname
        self.cname = cname
        self.estimator = LinearSystemID(m,n)
    def advance(self,**inputs):
        x = inputs[self.xname]
        y = inputs[self.yname]
        u = inputs[self.uname]
        self.estimator.add(x,u,y)
        A,B,C = self.estimator.getOutput()
        outputs = {}
        outputs[self.Aname] = A
        outputs[self.Bname] = B
        outputs[self.Cname] = C
        return outputs


class LinearSystemIDPredictionBlock(ControllerBlock):
    """Outputs the prediction from linear system ID to the output predname,
    uses the actual observation to update the model from the last time step.
    """
    def __init__(self,m,n,xname,uname,yname,predname=None):
        self.xname = xname
        self.uname = uname
        self.yname = yname
        self.predname = yname+" predicted" if predname is None else predname
        self.xlast = None
        self.ulast = None
        self.estimator = LinearSystemID(m,n)
    def advance(self,**inputs):
        x = inputs[self.xname]
        y = inputs[self.yname]
        u = inputs[self.uname]
        if self.xlast is not None:
            self.estimator.add(self.xlast,self.ulast,y)
        ypred = self.estimator.getOutput(x,u)
        self.xlast = x
        self.ulast = u
        return {self.predname:ypred}

