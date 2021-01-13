"""
Contains the following helper controllers:

- :class:`RemappedBlock`: a remapping of inputs/outputs into another block's 
  inputs and outputs.
- :class:`MultiBlock`: a block that runs many other sub-blocks on 
  each step.  The I/O dictionaries are basically used as a blackboard
  architecture.
- :class:`LambdaBlock`: a stateless block that simply runs a fixed 
  function on each time step.
- :class:`SumBlock`: computes a sum operation
- :class:`DifferenceBlock`: computes a minus operation
- :class:`ProductBlock`: computes a product operation
- :class:`LinearBlock`: computes a linear function of its inputs.
- :class:`TimedSequenceBlock`: a sequence of controllers, switched by
  time.
- :class:`ComposeController`: a set of controllers for parts of a robot,
  for which q, dq, qcmd, dqcmd, torquecmd, are concatenated.
- :class:`SourceBlock`: a block that writes to the ControllerBlock
  architecture.
- :class:`SinkBlock`: a block that reads from the ControllerBlock
  architecture.
"""
from .state_machine import TransitionStateMachine
from ..controller import ControllerBlock
from klampt.math import vectorops

class RemappedBlock(ControllerBlock):
    """A remapping of a block's inputs and outputs"""
    def __init__(self,block,inmap=None,outmap=None):
        self.block = block
        self.inmap = inmap
        self.outmap = outmap
    def __str__(self):
        instr = str(self.inmap) if self.inmap is not None else ''
        outstr = str(self.outmap) if self.outmap is not None else ''
        return str(self.block)+'{'+instr+';'+outstr+'}'
    def inputValid(self,**inputs):
        if self.inmap is None:
            return self.block.inputValid(**inputs)
        else:
            return ControllerBlock.inputValid(self,**inputs)
    def inputNames(self):
        if self.inmap is None:
            return self.block.inputNames()
        else:
            return list(self.inmap.keys())
    def outputNames(self):
        if self.inmap is None:
            return self.block.inputNames()
        else:
            return list(self.inmap.keys())
    def advance(self,**inputs):
        blockInputs = inputs if self.inmap is None else dict((v,inputs[k]) for v,k in self.inmap.items())
        blockOutputs = self.block.advance(**blockInputs)
        if self.outmap is None:
            return blockOutputs
        else:
            return dict((v,blockOutputs[k]) for (v,k) in self.outmap.items())
    def signal(self,type,**inputs):
        blockInputs = inputs if self.inmap is None else dict((v,inputs[k]) for v,k in self.inmap.items())
        self.block.signal(type,**blockInputs)
    def getState(self):
        return self.block.getState()
    def setState(self,state):
        return self.block.setState(state)
    def drawGL(self):
        return self.block.drawGL()


class MultiBlock(ControllerBlock):
    """A controller that runs several other subcontrollers each time step
    and emulates a sort of blackboard architecture.  Basically used to
    emulate a multiprocessor on a robot.
    
    For example, a state estimator can be run before a controller to process
    the robot's state into more meaningful information.
    
    The controller stores a register (a dict) that is sent to the
    the input of each subcontroller, and the output of each subcontroller
    is added to the register.

    Selective input/output or renaming of registers can be accomplished via
    register-to-input and register-to-output mappings.  This makes it easier
    to reuse individual subcontrollers when other subcontrollers change their
    output.
    """
    def __init__(self,controllers=[]):
        """Given a list of controllers, will execute all of them (in sequence)
        on each time step.
        """
        self.controllers = controllers[:]
        self.register = {}
        self.outregister = {}
        self.inmap = [None for c in self.controllers]
        self.outmap = [None for c in self.controllers]
        self.myoutmap = None

    def inputNames(self):
        res = set()
        for i in self.inmap:
            if i is not None:
                res |= set(i.keys())
        return res

    def outputNames(self):
        return self.myoutmap.values()

    def launch(self,c):
        """Start running a new controller and returns its id.  By default,
        all items in the registry are sent to the controllers.  To remap,
        call the `map_input` method."""
        self.controllers.append(c)

        self.inmap.append(None)
        self.outmap.append(None)
        return len(self.controllers)-1
    
    def map_input(self,c,regitem,citem=None):
        """Sends register `regitem` to the input of controller `c`.

        Args:
            c (int): the index of a sub-controller
            regitem (str): the name of an input arg.
            citem (str, optional): if specified, the input arg name is 
                mapped to c's argument `citem`.
        
        If this is not called for a given controller, then all items
        in the register are automatically sent to the controller.
        """
        if self.inmap[c]==None:
            self.inmap[c] = {}
        if citem == None:
            self.inmap[c][regitem]=regitem
        else:
            self.inmap[c][citem]=regitem
        return

    def map_output(self,c,citem,regitem=None):
        """Sends output citem of controller c to the register.

        Args:
            c (int): the index of a sub-controller
            citem (str): the name of an output item of c.
            regitem (str, optional): if specified, c's output item cname is 
                mapped to name regitem.
        
        If this is not called for a given controller, then all items in
        the controller's output are automatically sent to the register
        """
        if self.outmap[c]==None:
            self.outmap[c] = {}
        if regitem == None:
            self.outmap[c][citem]=citem
        else:
            self.outmap[c][citem]=regitem
        return

    def map_my_output(self,regitem,outitem=None):
        """Sends register item regitem to the output of the controller.

        Args:
            regitem (str): the name of an item in the register
            outitem (str, optional): if specified, maps the data to the output
                name outitem.
        
        If this is not called, then the entire register is sent to the
        output.
        """
        if self.myoutmap == None:
            self.myoutmap = {}
        if outitem == None:
            self.myoutmap[regitem] = regitem
        else:
            self.myoutmap[regitem] = outitem
        return

    def signal(self,type,**inputs):
        self.register.update(inputs)
        for i,c in enumerate(self.controllers):
            c.signal(type,**self.controller_inputs(i))
        return
    
    def controller_inputs(self,c):
        if self.inmap[c] == None: return self.register
        return dict((k,self.register[v]) for k,v in self.inmap[c].iteritems())
    
    def advance(self,**inputs):
        assert len(self.inmap)==len(self.controllers),"%d inmaps != %d controlers"%(len(self.inmap),len(self.controllers))
        assert len(self.outmap)==len(self.controllers),"%d outmaps != %d controlers"%(len(self.outmap),len(self.controllers))
        self.register.update(inputs)
        self.outregister = {}
        for i,c in enumerate(self.controllers):
            cout = c.advance(**self.controller_inputs(i))
            if not cout: continue
            if self.outmap[i] == None:
                self.register.update(cout)
                self.outregister.update(cout)
            else:
                for (k,v) in self.outmap[i].iteritems():
                    self.register[v] = cout[k]
                    self.outregister[v] = cout[k]
        if self.myoutmap == None:
            return self.outregister
        else:
            res = {}
            for k,v in self.myoutmap.iteritems():
                try:
                    res[v] = self.outregister[k]
                except KeyError:
                    print("Warning, output item",k,"not present in register")
            return res
    def getState(self):
        res = []
        for c in self.controllers:
            try:
                s = c.getState()
            except NotImplementedError:
                s = None
            res.append(s)
        return res
    def setState(self,state):
        assert len(state) == len(self.controllers)
        for c,s in zip(self.controllers,state):
            if s is not None:
                c.setState(s)
    def drawGL(self):
        for c in self.controllers:
            c.drawGL()
        return


class LambdaBlock(ControllerBlock):
    """A fixed-function controller that simply evaluates a function.  The
    function arguments and return values are mapped from/to the input/output
    dictionaries.
    """
    def __init__(self,f,argnames,outnames=None):
        self.f = f
        self.argnames = argnames
        self.outnames = outnames
    def inputNames(self):
        return self.argnames
    def outputNames(self):
        return self.outnames
    def advance(self,**inputs):
        try:
            args = [inputs[a] for a in self.argnames]
        except KeyError:
            print("LambdaBlock: Warning, argument does not exist in inputs")
            return None
        res = self.f(*args)
        if isinstance(self.outnames,(list,tuple)):
            return dict(zip(self.outnames,res))
        elif self.outnames != None:
            return {self.outnames:res}
        else:
            if isinstance(res,dict):
                return res
            return None


class SumBlock(LambdaBlock):
    """An estimator that produces an output "A + B [ + ...]" for two or more arguments"""
    def __init__(self,*args):
        def add(*args):
            if hasattr(args[0],'__iter__'):
                return vectorops.add(*args)
            else:
                return sum(args)
        LambdaBlock.__init__(self,add,args,' + '.join(args))


class DifferenceBlock(LambdaBlock):
    """An estimator that produces an output "A - B" for two arguments "A" and "B"."""
    def __init__(self,arg1,arg2):
        def diff(x,y):
            if hasattr(x,'__iter__'):
                return vectorops.sub(x,y)
            else:
                return x-y
        LambdaBlock.__init__(self,diff,[arg1,arg2],arg1+" - "+arg2)    


class ProductBlock(LambdaBlock):
    """An estimator that produces an output "A*B" for two arguments "A" and "B"."""
    def __init__(self,arg1,arg2):
        def prod(x,y):
            if hasattr(x,'__iter__') or hasattr(y,'__iter__'):
                return vectorops.mul(x,y)
            else:
                return x*y
        LambdaBlock.__init__(self,prod,[arg1,arg2],arg1+" * "+arg2)    


class LinearBlock(ControllerBlock):
    """Implements a linear function
    u = K1*input[x1] + ... + Kn*input[xn] + offset

    The user must fill out the self.gains member using the addGain()
    method.

    To use this, Numpy must be available on your system.
    """
    def __init__(self,type='torquecmd'):
        import numpy as np
        self.outputType = type
        self.gains = dict()
        self.offset = None
    def addGain(self,inputTerm,K):
        self.gains[inputTerm] = K
    def inputNames(self):
        return self.gains.keys()
    def outputNames(self):
        return [self.outputType]
    def setConstant(self,offset):
        self.offset = offset
    def advance(self,**inputs):
        import numpy as np
        res = self.offset
        for (x,K) in self.gains.iteritems():
            if x not in inputs:
                print("LinearController: warning, input",x,"doesn't exist, ignoring")
                continue
            if res==None:
                res = np.dot(K,inputs[x])
            else:
                res += np.dot(K,inputs[x])
        return {self.outputType:res}


class CounterBlock(ControllerBlock):
    """A block that just counts the number of times that it's run"""
    def __init__(self,initial=0):
        self.counter = initial
    def inputNames(self):
        return []
    def outputNames(self):
        return ['counter']
    def advance(self,**inputs):
        res = {'counter':self.counter}
        self.counter = self.counter+1
        return res
    def getState(self):
        return self.counter
    def setState(self,state):
        self.counter = state
    def signal(self,type,**inputs):
        if type=='reset':
            self.counter = 0


class ComposeController(ControllerBlock):
    """Concatenates vectors from multiple items into a single vector.
    Useful for when you have one controller for each arm, one for a lower body,
    etc.
    
    Arguments:
        itemindices (dict): a map from items to indices
        outitem (str): the name of the output
        flatten (bool, optional): true if you want a list rather than a dict,
            in which case the indices are assumed to be all integers. Empty
            indices are filled in with 'self.fill'.  By default this produces
            a vector.
    """
    def __init__(self,itemindices,outitem,flatten=True):
        self.itemindices = itemindices
        self.outitem = outitem
        self.flatten = flatten
        self.fill = 0.0
    
    def advance(self,**inputs):
        res = {}
        for (k,v) in self.itemindices.iteritems():
            try:
                for index in v:
                    res[index] = inputs[k][index]
            except KeyError:
                print("ComposeController: Warning, item",k,"does not exist in index")
                pass
        if self.flatten:
            inds = sorted(res.keys())
            vres = [self.fill]*(max(inds)+1)
            for (k,v) in res.iteritems():
                vres[k] = v
            return {self.outitem:vres}
        else:
            return {self.outitem:res}


class TimedSequenceBlock(TransitionStateMachine):
    """A state-machine controller that goes through each sub-controller
    in sequence.
    """
    def __init__(self,controllers,times):
        assert len(times)==len(controllers)
        trans = [{} for c in controllers]
        for i in range(len(controllers)-1):
            trans[i][i+1] = lambda input:input['t'] >= times[i]
        trans[-1][-1] = lambda input:input['t'] >= times[-1]
        TransitionStateMachine.__init__(self,controllers,trans)


class SourceBlock(ControllerBlock):
    """Outputs the dictionary self.values, which may be constant or written to
    by an external process (that is, code outside of the ControllerBlock
    architecture).
    """
    def __init__(self,values=None):
        if values is None:
            values = dict()
        self.values = values
    def outputNames(self):
        return list(self.values.keys())
    def inputNames(self):
        return []
    def advance(self,**inputs):
        return self.values
    def getState(self):
        return self.values
    def setState(self,state):
        self.values = state


class SinkBlock(ControllerBlock):
    """Inputs to the dictionary self.values, which may be read by an external
    process (that is, code outside of the ControllerBlock architecture).
    """
    def __init__(self,values=None):
        self.values = None
    def outputNames(self):
        return None
    def advance(self,**inputs):
        self.values = inputs
        return
    def getState(self):
        return self.values
    def setState(self,state):
        self.values = state
    def signal(self,type,**inputs):
        if type=='reset':
            self.values = None
