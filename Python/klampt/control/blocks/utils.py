"""
Contains the following helper controllers:

- :class:`MultiController`: a process that runs many other sub-processes on 
  each step.  The I/O dictionaries are basically used as a blackboard
  architecture.
- :class:`LambdaController`: a stateless process that simply runs a fixed 
  function on each time step.
- :class:`StateMachineController`: a state machine that switches between
  multiple sub-controllers, with one sub-controller running at once and the
  active controller triggered by some signal.
- :class:`TransitionStateMachineController`: a state machine with explicit
  transition conditions.
- :class:`TimedControllerSequence`: a sequence of controllers, switched by
  time.
- :class:`ComposeController`: a set of controllers for parts of a robot,
  for which q, dq, qcmd, dqcmd, torquecmd, are concatenated.
- :class:`LinearController`: computes a linear function of its inputs.

"""

class MultiController(ControllerBase):
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
    
    def output_and_advance(self,**inputs):
        assert len(self.inmap)==len(self.controllers),"%d inmaps != %d controlers"%(len(self.inmap),len(self.controllers))
        assert len(self.outmap)==len(self.controllers),"%d outmaps != %d controlers"%(len(self.outmap),len(self.controllers))
        self.register.update(inputs)
        self.outregister = {}
        for i,c in enumerate(self.controllers):
            cout = c.output_and_advance(**self.controller_inputs(i))
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


class LambdaController(ControllerBase):
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
    def output_and_advance(self,**inputs):
        try:
            args = [inputs[a] for a in self.argnames]
        except KeyError:
            print("LambdaController: Warning, argument does not exist in inputs")
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


class StateMachineController(ControllerBase):
    """A base class for a finite state machine controller. :meth:`next_state`
    needs to be filled out by the subclass."""
    def __init__(self,controllers,start=0):
        self.controllers = controllers
        self.current = start
    def __str__(self):
        strs = [str(c) for c in self.controllers]
        strs[self.current] = '* '+strs[self.current]
        return self.__class__.__name__+'\n'+'\n  '.join(strs)
    def signal(self,type,**inputs):
        if self.current >= 0:
            self.controllers[self.current].signal(type,**inputs)
    def output(self,**inputs):
        if self.current<0: return None
        return self.controllers[self.current].output(**inputs)
    def advance(self,**inputs):
        if self.current<0: return None
        self.controllers[self.current].advance(**inputs)
        self.current = self.next_state(self.current,**inputs)
    def output_and_advance(self,**inputs):
        if self.current<0: return None
        res = self.controllers[self.current].output_and_advance(**inputs)
        n = self.next_state(self.current,**inputs)
        if n != self.current:
            self.controllers[self.current].signal('exit',**inputs)
            if n >= 0:
                self.controllers[n].signal('enter',**inputs)
            self.current = n
        return res
    def drawGL(self):
        if self.current>=0:
            self.controllers[self.current].drawGL()
    def next_state(self,state,**inputs):
        """Subclasses should override this to implement the transitions"""
        return state
    def getState(self):
        controllerState = []
        for c in self.controllers:
            try:
                s = c.getState()
            except NotImplementedError:
                s = None
            controllerState.append(s)
        return {'current':self.current,'controllers':controllerState}
    def setState(self,state):
        if 'current' not in state or 'controllers' not in state or len(state['controllers']) != len(self.controllers):
            raise ValueError("Invalid state dict")
        self.current = state.current
        for (c,s) in zip(self.controllers,state['controllers']):
            if s is not None:
                c.setState(s)


class TransitionStateMachineController(StateMachineController):
    """A state machine controller with a transition matrix that determines
    when to move to the next state.
    """
    def __init__(self,controllers,transitions,start=0):
        """transitions is a list of dictionaries,
        [{x1:cond1,...,xk:condk},...,{}]
        so that if j is in transitions[i], then transitions[i][j](inputs)
        is a condition that tells you whether to change states from i to j.
        """
        StateMachineController.__init__(self,controllers,start)
        self.transitions = transitions

    def next_state(self,state,**inputs):
        if state < 0: return state
        trans = self.transitions[state]
        for (k,v) in trans.iteritems():
            if v(inputs):
                print("TransitionStateMachineController: Transition to controller",k)
                return k
        return state


class ComposeController(ControllerBase):
    """Integrates vectors from multiple items into a single vector.
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
    
    def output(self,**inputs):
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


class TimedControllerSequence(TransitionStateMachineController):
    """A state-machine controller that goes through each sub-controller
    in sequence.
    """
    def __init__(self,controllers,times):
        assert len(times)==len(controllers)
        trans = [{} for c in controllers]
        for i in xrange(len(controllers)-1):
            trans[i][i+1] = lambda input:input['t'] >= times[i]
        trans[-1][-1] = lambda input:input['t'] >= times[-1]
        TransitionStateMachineController.__init__(self,controllers,trans)


class LinearController(ControllerBase):
    """Implements a linear controller
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
    def output(self,**inputs):
        res = self.offset
        for (x,K) in self.gains.iteritems():
            if x not in inputs:
                print("LinearController: warning, input",x,"doesn't exist, ignoring")
                continue
            if res==None:
                res = np.dot(K,inputs[x])
            else:
                res += np.dot(K,inputs[x])
        return {outputType:res}


