"""
Defines a set of generic "system blocks" that are repeatedly-updating
processes. These can implement filters, estimators, read from sensor drivers,
or output commands to a simulated or real robot. 

The :class:`Block` class defines a block as accepting some
inputs and produces some outputs every time that ``advance()`` is
called.  The inputs and outputs are extremely general, and consist
of arguments referred to by an int or str index.  Outputs can also be
added, subtracted, and compared to produce new outputs.

The core module contains several examples of controller 
blocks that can be composed.  Connect the inputs and outputs of blocks,
pass any subset to a :class:`SuperBlock`, and the inputs and outputs of
the SuperBlock will be automatically determined.

See :mod:`klampt.control.blocks.utils` for
utilities, and :mod:`klampt.control.blocks.state_machine` for state machines
that use Blocks.

.. versionadded:: 0.9
"""

from klampt.math import vectorops
from typing import Union,Any,Tuple,Sequence,Dict,List,Iterator
import weakref
import warnings
import operator


class Block(object):
    """A generic base class for some streaming computational block.
    This is typically used to define robot controllers and components
    of such controllers, such as state estimators and filters.

    Users will create blocks, connect their inputs and outputs, and
    then set up a SuperBlock.  Then, SuperBlock.advance() will be called
    repeatedly.

    At a minimum, a block should specify its # of inputs and outputs
    and implement the :meth:`advance` method.  A block can also specify
    named inputs and outputs, which will let the inputs to advance() be
    specified as keyword, and the return value can also be a dict.

    A stateful block should also implement the :meth:`__getstate__` and
    :meth:`__getstate__` methods to help serialization / deserialization,
    state machine resetting, etc.

    Arguments:
        inputs (int or list of str): gives the # of inputs or a list of
            input names.
        outputs (int or list of str): gives the # of outputs or a list
            of output names.
    """
    def __init__(self,inputs=0,outputs=0) -> None:
        self._name = None
        self._inputs = _BlockInputs(weakref.proxy(self),inputs)
        self._outputs = _BlockOutputs(weakref.proxy(self),outputs)

    def advance(self,*args) -> Union[None,Any,Tuple,Dict[str,Any]]:
        """Computes the output of this controller given an argument list,
        and advances the time step. 
        
        Returns:
            None (0 outputs), a value (1 output), a tuple, or a dict
        """
        return None
    
    def signal(self,name,*args):
        """Reacts to some asynchronous signal. Most blocks won't use this.
        Common signals include:
        
        * 'reset': reset a state machine
        * 'enter': enter activity in a state machine
        * 'exit': exit activity in a state machine
        * 'change_state' index: trigger a state change in a state machine.

        """
        return

    def __str__(self):
        """Optional: return a descriptive string describing this block"""
        if self._name is None:
            return self.__class__.__name__
        else:
            return "{}({})"%(self._name,self.__class__.__name__)

    def _process(self):
        """Called internally"""
        for c in self._inputs._channels:
            c.advance()
        if len(self._inputs._namesToIndices) > 0:  #map to keyword args
            args = {}
            for k,i in self._inputs._namesToIndices.items():
                args[k] = self._inputs._channels[i]._currentValue
            res = self.advance(**args)
        else:
            args = [c._currentValue for c in self._inputs._channels]
            res = self.advance(*args)
        if res is None:
            if len(self._outputs)!=0:
                raise ValueError("Block {} produces {} outputs but advance returned None".format(str(self),len(self._outputs)))
        else:
            if len(self._outputs)==1:
                self._outputs[0].setValue(res)
            else:
                if len(res) != len(self._outputs):
                    raise ValueError("Block {} produces {} outputs but advance returned {}".format(str(self),len(self._outputs),len(res)))
                if isinstance(res,dict):
                    #map to outputs using keywords
                    for k,r in res.items():
                        if k not in self._outputs._namesToIndices:
                            raise ValueError("Block {} produces output {} which is not a named output".format(str(self),str(k)))
                        i = self._outputs._namesToIndices[k]
                        self._outputs._channels[i].setValue(r)
                else:
                    for r,o in zip(res,self._outputs._channels):
                        o.setValue(r)

    def __call__(self,*args) -> 'Block':
        """If each of args is a one-output function, this connects all of their
        outputs to each of this function's inputs.
        
        Usage::

            #result is a SomeBlock taking args block1 output 0, block2 output 1
            result = SomeBlock()(block1,block2.output[1])  
        
        Returns:
            Block
        """
        if len(args) != len(self._inputs):
            raise ValueError("Invalid number of args {} != {}".format(len(args),len(self._inputs)))
        if len(self._outputs) != 1:
            raise ValueError("Can only use call notation for functions with 1 channel output")
        self._storage = []  #for shortcut evaluation, only weak references are stored in each channel. This prevents block deletion.
        for i,arg in enumerate(args):
            if isinstance(arg,Block):
                if len(arg._output) != 1:
                    raise ValueError("Invalid arg {}, doesn't have single output".format(str(arg)))
                self._inputs[i].connect(arg._outputs[0])
                self._storage.append(arg)
            elif isinstance(arg,(_BlockOutputs,_BlockOutputChannel)):
                self._inputs[i].connect(arg)
            else:
                raise ValueError("Invalid arg {}, must be a block or an output channel".format(str(arg)))
        return self
    
    def __getattr__(self,name):
        if name == 'input':
            return self._inputs
        elif name == 'output':
            return self._outputs
        else:
            raise AttributeError()
    
    def __setattr__(self,name,value):
        if name == 'input':
            if len(self._inputs._channels)!=1:
                raise ValueError("For input=X to work, need exactly 1 channel")
            self._inputs[0].connect(value)
        elif name == 'output':
            if len(self._outputs._channels)!=1:
                raise ValueError("For output=X to work, need exactly 1 channel")
            self._outputs[0].connect(value)
        else:
            super().__setattr__(name,value)

    def debug(self,type):
        """Optional: hook to give feedback to the visualizer"""
        if type=='print':
            print(str(self))
        else:
            pass


class SuperBlock(Block):
    """A collection of Blocks with connected inputs and outputs.

    Initialized with one or more seed Blocks. All nodes path-connected
    to these seeds will be included in the group.
    """
    def __init__(self,seeds:Union['Block',Sequence['Block']]):
        self.blocks = []
        self.edges = []
        self.free_inputs = []
        self.free_outputs = []
        if isinstance(seeds,Block):
            seeds = [seeds]
        if len(seeds)==0:
            return
        #do an undirected DFS to extract all blocks
        q = [s for s in seeds]
        visited = {}
        for s in seeds: visited[id(s.__repr__.__self__)] = True
        while len(q)>0:
            b = q.pop()
            self.blocks.append(b)
            for c in b._outputs._channels:
                for target in c._downstream:
                    hash = id(target._block.__repr__.__self__)
                    if hash not in visited:
                        q.append(target._block)
                        visited[hash] = True
            for c in b._inputs._channels:
                target = c._upstream
                if target is None: continue
                hash = hash = id(target._block.__repr__.__self__)
                if hash not in visited:
                    q.append(target._block)
                    visited[hash] = True
        #do a topological sort
        inputBlocks = []
        for b in self.blocks:
            inputBlocks.append((len(b._inputs._channels),b))
        inputBlocks = sorted(inputBlocks,key=lambda x:x[0])
        if inputBlocks[0][0] != 0:
            warnings.warn("Diagram doesn't have any source blocks")
            seeds = [inputBlocks[0][1]]
        else:
            seeds = []
            for (c,b) in inputBlocks:
                if c!=0: break
                seeds.append(b)
        #BFS to perform topological sort
        self.blocks = []
        visited = {}
        q = [s for s in seeds]
        for s in seeds: visited[id(s)] = True
        while len(q)>0:
            b = q.pop(0)
            self.blocks.append(b)
            for c in b._outputs._channels:
                for target in c._downstream:
                    if id(target._block) not in visited:
                        q.append(target._block)
                        visited[id(target._block)] = True

        for b in self.blocks:
            self.edges.append([])
            for c in b._outputs._channels:
                if len(c._downstream)==0:
                    self.free_outputs.append(c)
                for target in c._downstream:
                    self.edges[-1].append((c,target))
            for c in b._inputs._channels:
                if c._upstream is None:
                    self.free_inputs.append(c)
        free_input_names = []
        free_output_names = []
        for c in self.free_inputs:
            free_input_names.append(str(c))
        for c in self.free_outputs:
            free_output_names.append(str(c))
        Block.__init__(self,free_input_names,free_output_names)
    
    def advance(self,*args):
        """Triggers all of the nodes in the block diagram."""
        if len(args) != len(self.free_inputs):
            raise RuntimeError("Not enough arguments (%d) for free inputs (%d)"%(len(args),len(self.free_inputs)))

        for a,c in self.free_inputs:
            c._currentValue = a

        for b in self.blocks:
            b._process()

        for c in self.free_inputs:
            c._currentValue = None
        
        return [c._currentValue for c in self.free_outputs]

    def signal(self,name,*args):
        for b in self.blocks:
            b.signal(name,*args)

    def debug(self,type):
        for b in self.blocks:
            b.debug(type)


class Source(Block):
    def __init__(self):
        Block.__init__(self,0,1)
        
class Sink(Block):
    def __init__(self):
        Block.__init__(self,1,0)

class ValueSource(Source):
    """A simple Source whose value is controlled by an external process
    via ``Source.write(value)``.
    """
    def __init__(self,value):
        Source.__init__(self)
        self.value = value
    def advance(self):
        return self.value
    def write(self,value):
        self.value = value

class ValueSink(Sink):
    """A simple Sink whose value is extracted by an external process
    via ``Sink.read()``.
    """
    def advance(self,value):
        self.value = value
    def read(self):
        return self.value

class Counter(Source):
    """Maintains a value that increases linearly each time advance() is
    called.
    """
    def __init__(self,initVal=0,increment=1):
        Source.__init__(self)
        self.value = initVal
        self.increment = increment
    def advance(self):
        res = self.value
        self.value += self.increment
        return res
    def __getstate__(self):
        return self.value,self.increment
    def __setstate__(self,state):
        self.value,self.increment = state

class Add(Block):
    """Adds two inputs (scalar or vector)."""
    def __init__(self):
        Block.__init__(self,2,1)
    def advance(self,a,b):
        if hasattr(a,'__iter__') or hasattr(b,'__iter__'):
            return vectorops.add(a,b)
        return a+b

class Sub(Block):
    """Subtracts the second input from the first (scalar or vector)."""
    def __init__(self):
        Block.__init__(self,2,1)
    def advance(self,a,b):
        if hasattr(a,'__iter__') or hasattr(b,'__iter__'):
            return vectorops.sub(a,b)
        return a-b

class BinaryOp(Block):
    """Generic lambda for a binary operation."""
    def __init__(self,func):
        self.func = func
        Block.__init__(self,2,1)
    def advance(self,a,b):
        return self.func(a,b)


class Min(Block):
    """Takes the minimum of two inputs (scalar or vector)."""
    def __init__(self):
        Block.__init__(self,2,1)
    def advance(self,a,b):
        if hasattr(a,'__iter__') or hasattr(b,'__iter__'):
            return vectorops.minimum(a,b)
        return min(a,b)

class Max(Block):
    """Takes the maximum of two inputs (scalar or vector)."""
    def __init__(self):
        Block.__init__(self,2,1)
    def advance(self,a,b):
        if hasattr(a,'__iter__') or hasattr(b,'__iter__'):
            return vectorops.maximum(a,b)
        return max(a,b)


class _BlockInputChannel(object):
    def __init__(self,block,index,name=None):
        self._block = block
        self._index = index
        self._global_name = None
        self._name = name
        self._type = None
        self._upstream = None      #type: _BlockOutputChannel
        self._currentValue = None
    def __str__(self):
        if self._global_name is not None: return self._global_name
        elif self._name is None: return str(self._block)+'.'+"inputs[{}]".format(self._index)
        else: return str(self._block)+'.'+self._name
    def setName(self,name):
        self._global_name = name
    def setType(self,type):
        self._type = type
    def connect(self,channel):
        if isinstance(channel,Block):
            self.connect(channel._outputs)
        elif isinstance(channel,_BlockOutputs):
            if len(channel._channels) != 1:
                raise ValueError("Cannot connect input to outputs with more than one channel")
            self.connect(channel[0])
        elif isinstance(channel,_BlockOutputChannel):
            if channel._type is not None and self._type is not None:
                if self._type != channel._type:
                    raise ValueError("Can't connect input of type {} to output of type {}".format(self._type,channel._type))
            channel._downstream.append(weakref.proxy(self))
            self._upstream = channel
        elif isinstance(channel,(_BlockInputChannel,_BlockInputs)):
            raise ValueError("Can only connect outputs to inputs")
        else:
            self._upstream = channel
    def read(self):
        return self._currentValue
    def reset(self):
        self._currentValue = None
    def advance(self):
        if self._upstream is not None:
            if isinstance(self._upstream,_BlockOutputChannel):
                self._currentValue = self._upstream._currentValue
            else:
                self._currentValue = self._upstream
        else:
            self._currentValue = None

class _BlockOutputChannel(object):
    def __init__(self,block,index,name=None):
        self._block = block
        self._index = index
        self._global_name = None
        self._name = name
        self._type = None
        self._currentValue = None
        self._downstream = []    #type: List[_BlockInputChannel]
    def __str__(self):
        if self._global_name is not None: return self._global_name
        elif self._name is None: return str(self._block)+".outputs[{}]".format(self._index)
        else: return str(self._block)+'.'+self._name
    def setName(self,name):
        self._global_name = name
    def setType(self,type):
        self._type = type
    def connect(self,channel):
        if isinstance(channel,_BlockInputs):
            self.connect(channel[0])
        elif isinstance(channel,_BlockInputChannel):
            if channel._type is not None and self._type is not None:
                if self._type != channel._type:
                    raise ValueError("Can't connect output of type {} to input of type {}".format(self._type,channel._type))
            self._downstream.append(weakref.proxy(channel))
            channel._upstream = self
        elif isinstance(channel,(_BlockOutputChannel,_BlockOutputs)):
            raise ValueError("Can only connect inputs to outputs")
        else:
            raise ValueError("Can't connect outputs to constants")
    def reset(self):
        self._currentValue = None
    def setValue(self,val):
        self._currentValue = val
    def read(self):
        return self._currentValue
    def __add__(self,rhs):
        res = Add()
        res.input[0].connect(self)
        res.input[1].connect(rhs)
        return res
    def __sub__(self,rhs):
        res = Sub()
        res.input[0].connect(self)
        res.input[1].connect(rhs)
        return res
    def __gt__(self,rhs):
        res = BinaryOp(operator.gt)
        res.input[0].connect(self)
        res.input[1].connect(rhs)
        return res
    def __ge__(self,rhs):
        res = BinaryOp(operator.ge)
        res.input[0].connect(self)
        res.input[1].connect(rhs)
        return res
    def __lt__(self,rhs):
        res = BinaryOp(operator.lt)
        res.input[0].connect(self)
        res.input[1].connect(rhs)
        return res
    def __le__(self,rhs):
        res = BinaryOp(operator.le)
        res.input[0].connect(self)
        res.input[1].connect(rhs)
        return res
    def __eq__(self,rhs):
        res = BinaryOp(operator.eq)
        res.input[0].connect(self)
        res.input[1].connect(rhs)
        return res
    def __ne__(self,rhs):
        res = BinaryOp(operator.ne)
        res.input[0].connect(self)
        res.input[1].connect(rhs)
        return res

class _BlockInputs(object):
    def __init__(self,block,inputs) -> None:
        self._block = block
        if isinstance(inputs,int):
            self._channels = [_BlockInputChannel(block,i) for i in range(inputs)]
            self._namesToIndices = {}
        else:
            self._channels = [_BlockInputChannel(block,i,n) for i,n in enumerate(inputs)]
            self._namesToIndices = {}
            for i,n in enumerate(inputs):
                self._namesToIndices[n] = i
    
    def addChannel(self,name=None) -> None:
        if name is None:
            if len(self._namesToIndices) == len(self._channels) and len(self._channels) > 0:
                raise ValueError("Can't add unnamed channel when prior channels are named")
            self._channels.append(_BlockInputChannel(self._block),len(self._channels))
        else:
            if len(self._namesToIndices) != len(self._channels):
                raise ValueError("Can't add named channel when prior channels are unnamed")
            if name in self._namesToIndices:
                raise ValueError("Already have a channel named {}".format(name))
            i = len(self._channels)
            self._channels.append(_BlockInputChannel(self._block,name),i)
            self._namesToIndices[name] = i

    def __len__(self) -> int:
        return len(self._channels)

    def __contains__(self, key : str) -> bool:
        return key in self._namesToIndices
    
    def items(self) -> Iterator:
        if len(self._namesToIndices):
            for k,i in self._namesToIndices.items():
                yield (k,self._channels[i])
        else:
            return enumerate(self._channels)

    def setName(self, name : str) -> None:
        if len(self._channels)!=1:
            raise ValueError("For input.setName() to work, need exactly 1 channel")
        self._channels[0].setName(name)
    
    def setType(self,type) -> None:
        if len(self._channels)!=1:
            raise ValueError("For input.setType() to work, need exactly 1 channel")
        self._channels[0].setType(type)
    
    def __getitem__(self, key : Union[int,str]):
        if isinstance(key,int):
            return self._channels[key]
        else:
            return self._channels[self._namesToIndices[key]]
    
    def __setitem__(self, key : Union[int,str], value) -> None:
        if isinstance(key,int):
            self._channels[key].connect(value)
        else:
            self._channels[self._namesToIndices[key]].connect(value)

    def read(self):
        if len(self._channels)==0:
            return None
        elif len(self._channels) > 1:
            if len(self._namesToIndices):
                return dict((k,self._channels[i].read()) for (k,i) in self._namesToIndices.items())
            else:
                return [c.read() for c in self._channels]
        return self._channels[0].read()

    
class _BlockOutputs(object):
    def __init__(self,block,outputs) -> None:
        self._block = block
        if isinstance(outputs,int):
            self._channels = [_BlockOutputChannel(block,i) for i in range(outputs)]
            self._namesToIndices = {}
        else:
            self._channels = [_BlockOutputChannel(block,i,n) for i,n in enumerate(outputs)]
            self._namesToIndices = {}
            for i,n in enumerate(outputs):
                self._namesToIndices[n] = i
    
    def addChannel(self,name=None) -> None:
        if name is None:
            if len(self._namesToIndices) == len(self._channels) and len(self._channels) > 0:
                raise ValueError("Can't add unnamed channel when prior channels are named")
            i = len(self._channels)
            self._channels.append(_BlockOutputChannel(self._block),i)
            return i
        else:
            if len(self._namesToIndices) != len(self._channels):
                raise ValueError("Can't add named channel when prior channels are unnamed")
            if name in self._namesToIndices:
                raise ValueError("Already have a channel named {}".format(name))
            i = len(self._channels)
            self._channels.append(_BlockOutputChannel(self._block,name),i)
            self._namesToIndices[name] = i
            return name
        
    def __len__(self) -> int:
        return len(self._channels)

    def __contains__(self, key : str) -> bool:
        return key in self._namesToIndices
    
    def items(self) -> Iterator:
        if len(self._namesToIndices):
            for k,i in self._namesToIndices.items():
                yield (k,self._channels[i])
        else:
            return enumerate(self._channels)

    def setName(self, name : str) -> None:
        if len(self._channels)!=1:
            raise ValueError("For output.setName() to work, need exactly 1 channel")
        self._channels[0].setName(name)
    
    def setType(self,type) -> None:
        if len(self._channels)!=1:
            raise ValueError("For output.setType() to work, need exactly 1 channel")
        self._channels[0].setType(type)
    
    def __getitem__(self, key : Union[int,str]):
        if isinstance(key,int):
            return self._channels[key]
        else:
            return self._channels[self._namesToIndices[key]]
    
    def __setitem__(self, key : Union[int,str], value) -> None:
        if isinstance(key,int):
            self._channels[key].connect(value)
        else:
            self._channels[self._namesToIndices[key]].connect(value)

    def read(self):
        if len(self._channels)==0:
            return None
        elif len(self._channels) > 1:
            if len(self._namesToIndices):
                return dict((k,self._channels[i].read()) for (k,i) in self._namesToIndices.items())
            else:
                return [c.read() for c in self._channels]
        return self._channels[0].read()
    
    def reset(self) -> None:
        if len(self._channels)!=1:
            raise ValueError("For output.reset() to work, need exactly 1 channel")
        self._channels[0].reset()

    def setValue(self,val) -> None:
        if len(self._channels)!=1:
            raise ValueError("For output.reset() to work, need exactly 1 channel")
        self._channels[0].setValue(val)

    def __add__(self,rhs):
        if len(self._channels)!=1:
            raise ValueError("For output + to work, need exactly 1 channel")
        res = Add()
        res.input[0].connect(self)
        res.input[1].connect(rhs)
        return res
    def __sub__(self,rhs):
        if len(self._channels)!=1:
            raise ValueError("For output - to work, need exactly 1 channel")
        res = Sub()
        res.input[0].connect(self)
        res.input[1].connect(rhs)
        return res
    def __gt__(self,rhs):
        if len(self._channels)!=1:
            raise ValueError("For output > to work, need exactly 1 channel")
        res = BinaryOp(operator.gt)
        res.input[0].connect(self)
        res.input[1].connect(rhs)
        return res
    def __ge__(self,rhs):
        if len(self._channels)!=1:
            raise ValueError("For output >= to work, need exactly 1 channel")
        res = BinaryOp(operator.ge)
        res.input[0].connect(self)
        res.input[1].connect(rhs)
        return res
    def __lt__(self,rhs):
        if len(self._channels)!=1:
            raise ValueError("For output < to work, need exactly 1 channel")
        res = BinaryOp(operator.lt)
        res.input[0].connect(self)
        res.input[1].connect(rhs)
        return res
    def __le__(self,rhs):
        if len(self._channels)!=1:
            raise ValueError("For output <= to work, need exactly 1 channel")
        res = BinaryOp(operator.le)
        res.input[0].connect(self)
        res.input[1].connect(rhs)
        return res
    def __eq__(self,rhs):
        if len(self._channels)!=1:
            raise ValueError("For output == to work, need exactly 1 channel")
        res = BinaryOp(operator.eq)
        res.input[0].connect(self)
        res.input[1].connect(rhs)
        return res
    def __ne__(self,rhs):
        if len(self._channels)!=1:
            raise ValueError("For output != to work, need exactly 1 channel")
        res = BinaryOp(operator.ne)
        res.input[0].connect(self)
        res.input[1].connect(rhs)
        return res


def self_test():
    from . import filters
    import math
    sensor = ValueSource(0.0) 
    sensor.output.setName("some sensor")
    filter = filters.FIRFilter([0.25]*4)  #moving average
    filter.input = sensor.output[0]   #for single-output blocks, output is the same as output[0]
    filter.output.setName("filtered sensor")
    #sink = Sink()
    proc = Min()(filter.output,sensor.output)
    #sink.input = proc.output
    diagram = SuperBlock([sensor])
    print("Blocks in super-block:",[str(b) for b in diagram.blocks])
    print("Free inputs: ",[str(c) for c in diagram.free_inputs])
    print("Free outputs: ",[str(c) for c in diagram.free_outputs])

    for t in range(10):
        t = t*0.001
        sval = math.sin(t*10)
        sensor.write(sval)
        res = diagram.advance()
        fval = proc.output.read()
        print("f({}) = {}".format(sval,fval))

if __name__ == '__main__':
    self_test()