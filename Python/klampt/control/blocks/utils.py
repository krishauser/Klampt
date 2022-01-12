"""
Contains the following helper blocks:

- :class:`BlockSignal`: an exception that allows a block to signal
  something (e.g., an error condition) to its parent.
- :class:`SignalBlock`: a block that raises a signal if an error
  is raised.
- :class:`LambdaBlock`: a stateless block that simply runs a fixed 
  function on each time step.
- :class:`LinearBlock`: a stateless block that simply performs a 
  linear or affine function.
- :class:`Concatenate`: concatenates sevearl items together.
- :class:`Clamp`: restricts an item to a range.
- :class:`LimitExceeded`: returns 1 if the limits are exceeded
- :class:`Distance`: calculates the distance.
"""
from .core import Block
from klampt.math import vectorops
import inspect


class BlockSignal(RuntimeError):
    """An exception raised by a block if it wishes to raise a signal to a
    parent.

    Attributes:
        signal (str): the identifier of the signal
    """
    def __init__(self,signal,text):
        self.signal = signal
        RuntimeError.__init__(self,text)


class SignalBlock(Block):
    """A block that raises a signal if its input is nonzero"""
    def __init__(self,type,text):
        self.type = type
        self.text = text
        Block.__init__(self,['signal'],0)
    def advance(self,signal):
        if signal:
            raise BlockSignal(self.type,self.text)
        return


class LambdaBlock(Block):
    """A fixed-function controller that simply evaluates a function.  The
    function arguments and return values are mapped from/to the input/output
    dictionaries.
    """
    def __init__(self,f,inputs='auto',outputs='auto'):
        self.f = f
        if inputs == 'auto':
            inputs = inspect.getargspec(f).args
        if outputs == 'auto':
            outputs = 1
        Block.__init__(self,inputs,outputs)
    def advance(self,*args):
        return self.f(*args)



class LinearBlock(Block):
    """Implements a linear function
    output = A*input + b

    The user must fill out the self.gains member using the addGain()
    method.

    To use this, Numpy must be available on your system.
    """
    def __init__(self,A,b=None):
        Block.__init__(self,1,1)
        import numpy as np
        self.A = A
        self.b = b
    def advance(self,x):
        import numpy as np
        if self.b is not None:
            return np.dot(self.A,x)+self.b
        return np.dot(self.A,x)


class Concatenate(Block):
    """Concatenates vectors from multiple items into a single vector.
    Useful for when you have one controller for each arm, one for a lower body,
    etc.
    
    Arguments:
        n (int): the number of items
    """
    def __init__(self,inputs):
        Block.__init__(self,inputs,1)
    
    def advance(self,*args):
        import numpy as np
        return np.hstack(args)



class Clamp(Block):
    """Restricts a value to some range"""
    def __init__(self):
        Block.__init__(self,["x","minimum","maximum"],1)
    def advance(self,x,minimum,maximum):
        if hasattr(x,'__iter__'):
            if hasattr(maximum,'__iter__'):
                assert len(x) == len(maximum)
            return vectorops.minimum(vectorops.maximum(x,minimum),maximum)
        else:
            return min(max(x,minimum),maximum)


class LimitExceeded(Block):
    """Returns 1 if a value exceeds some range"""
    def __init__(self):
        Block.__init__(self,['x','minimum','maximum'],0)
    def advance(self,x,minimum,maximum):
        assert len(x) == len(maximum)
        for (v,a,b) in zip(x,minimum,maximum):
            if v < a or v > b:
                return 1
        return 0


class Distance(Block):
    """Returns the L-p distance between two values"""
    def __init__(self,metric=float('inf')):
        if metric not in [1,2,float('inf')]:
            raise ValueError("Only supports L1, L2, or Linf distances")
        if metric == 1:
            self.metric = vectorops.norm_L1
        elif metric == 2:
            self.metric = vectorops.norm
        else:
            self.metric = vectorops.norm_Linf
        Block.__init__(self,2,1)
    def advance(self,x1,x2):
        assert len(x1) == len(x2)
        return self.metric(vectorops.sub(x1,x2))


class WorldCollision(Block):
    """Returns True if a collision occurrs in the world (or a collider)"""
    def __init__(self, world_or_collider):
        from klampt.model.collide import WorldCollider
        if not isinstance(world_or_collider,WorldCollider):
            collider = WorldCollider(world_or_collider)
        else:
            collider = world_or_collider
        self.collider = collider
        Block.__init__(self,'q',0)
    def advance(self,q) -> bool:
        robot = self.collider.world.robot(0)
        qrobot = robot.configFromDrivers(q)
        qrobot.setConfig(q)
        for a,b in self.collider.collisions():
            return True
        return False


class If(Block):
    def __init__(self):
        Block.__init__(self,['cond','truebranch','falsebranch'],1)
    def advance(self,cond,truebranch,falsebranch):
        if cond: return truebranch
        else: return falsebranch


class Mux(Block):
    """Function (index, case0, case1, ..., casek) returning case[index]
    """
    def __init__(self,k):
        Block.__init__(self,k+1,1)
    def advance(self,*args):
        index = int(args[0])
        if index < 0 or index >= len(args)-1:
            raise RuntimeError("Mux index is invalid")
        return args[index+1]

