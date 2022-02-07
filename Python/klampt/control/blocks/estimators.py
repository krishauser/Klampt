from klampt.math import vectorops
from .core import Block

class Differentiator(Block):
    """Computes the derivative of some input using finite differences.
    """
    def __init__(self,robot=None):
        self.robot = robot
        self.xlast = None
        Block.__init__(self,['dt','x'],['dx'])
    def __getstate__(self):
        return {'last':self.xlast}
    def __setstate__(self,state):
        self.xlast = state['last']
    def advance(self,dt,x):
        if len(x)==0: return None
        if self.xlast==None:
            dx = [0]*len(x)
        else:
            if self.robot==None:
                dx = vectorops.div(self.robot.sub(x,self.xlast),dt)
            else:
                assert(len(self.xlast)==len(x))
                dx = vectorops.div(self.robot.interpolate_deriv(self.xlast,x),dt)
        self.xlast = x
        return dx

    def signal(self,type,*args):
        if type=='reset':
            self.xlast=None


class Integrator(Block):
    """Computes the integral of some input using the 
    trapezoidal rule.
    """
    def __init__(self):
        self.integral = None
        Block.__init__(self,['dt','x'],'Ix')
    def __getstate__(self):
        return self.integral
    def __setstate__(self,state):
        self.integral = state
    def advance(self,dt,x):
        if len(x)==0: return None
        if self.integral is None:
            self.integral = vectorops.mul(x,dt)
        else:
            self.integral = vectorops.madd(self.integral,x,dt)
        result = vectorops.madd(self.integral,x,-0.5*dt)
        return result
    def signal(self,type,*inputs):
        if type=='reset':
            self.integral=None


