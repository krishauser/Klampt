class BaseController(object):
    """A generic class that outputs a control based on a dictionary of
    inputs"""
    def __init__(self):
        pass
    def output(self,**inputs):
        """Computes the output of this controller given a dictionary of
        inputs.  The output is typically a dictionary but could also be a
        list.

        A top-level controller that communicates with SerialController and
        simtest.py will get the following values as input:
        - t = time: simulation time
        - dt = timestep: controller time step
        - sensor1 = sensor1_values: measurements for sensor 1
        - ...
        - sensork = sensork_values: measurements for sensor k

        A top-level controller that communicates with SerialController and
        simtest.py should produce a dictionary containing one or more of
        the following keys:
        - qcmd
        - dqcmd 
        - tcmd
        - torquecmd
        
        If qcmd is set, then it's a PID command. If dqcmd is set, then it's
        the desired velocity, otherwise the desired velocity is 0.
        
        If qcmd is set and torquecmd is set, then torquecmd is a feedforward
        torque.
        
        If dqcmd and tcmd are set, it's a fixed velocity command.

        Otherwise, torquecmd must be set, and it's a torque command.
        """
        return None
    def signal(self,type,**inputs):
        """Sends some asynchronous signal to the controller. Application
        defined, but typical signals include:
        - 'reset'
        - 'enter'
        - 'exit'
        """
        pass
    def advance(self,**inputs):
        pass
    def output_and_advance(self,**inputs):
        res = self.output(**inputs)
        self.advance(**inputs)
        return res
    def drawGL(self):
        """Optional: hook to give feedback to the visualizer"""
        pass

class MultiController(BaseController):
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

    def launch(self,c):
        """Start running a new controller and returns its id"""
        self.controllers.append(c)
        self.inmap.append(None)
        self.outmap.append(None)
        return len(self.controllers)-1
    
    def map_input(self,c,regitem,citem=None):
        """Sends register regitem to the input of controller c.
        If citem is specified, the data is is mapped to name citem.
        If this is not called for a given controller, then all items
        in the register are automatically sent to the controller."""
        if self.inmap[c]==None:
            self.inmap[c] = {}
        if citem == None:
            self.inmap[c][regitem]=regitem
        else:
            self.inmap[c][citem]=regitem
        return

    def map_output(self,c,citem,regitem=None):
        """Sends output citem of controller c to the register.
        If regitem is specified, the data is mapped to name regitem.
        If this is not called for a given controller, then all items in
        the controller's output are automatically sent to the register"""
        if self.outmap[c]==None:
            self.outmap[c] = {}
        if regitem == None:
            self.outmap[c][citem]=citem
        else:
            self.outmap[c][citem]=regitem
        return

    def map_my_output(self,regitem,outitem=None):
        """Sends register item regitem to the output of the controller.
        If outitem is specified, maps the data to the name outitem.
        If this is not called, then the entire register is sent to the
        output
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
                    print "Warning, output item",k,"not present in register"
            return res
    def drawGL(self):
        for c in self.controllers:
            c.drawGL()
        return

class LambdaController(BaseController):
    def __init__(self,f,argnames,outnames=None):
        self.f = f
        self.argnames = argnames
        self.outnames = outnames
    def output_and_advance(self,**inputs):
        try:
            args = [inputs[a] for a in self.argnames]
        except KeyError:
            print "LambdaController: Warning, argument does not exist in inputs"
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

class StateMachineController(BaseController):
    """A base class for a finite state machine controller. next_state needs
    to be filled out by the subclass."""
    def __init__(self,controllers,start=0):
        self.controllers = controllers
        self.current = start
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
                print "Transition to controller",k
                return k
        return state


class ComposeController(BaseController):
    """Integrates vectors from multiple items into a single vector.
    Useful for when you have one controller for each arm, one for a lower
    body, etc.
    
    Initializers:
        - itemindices: a dict from items to indices
        - outitem: the name of the output
        - flatten: true if you want a list rather than a dict, in which case
          the indices are assumed to be all integers. Empty indices are
          filled in with 'self.fill'.  By default this produces a vector.
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
                print "Warning, item",k,"does not exist in index"
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
            trans[i][i+1] = lambda(input):input['t'] >= times[i]
        trans[-1][-1] = lambda(input):input['t'] >= times[-1]
        TransitionStateMachineController.__init__(self,controllers,trans)

class LinearController(BaseController):
    """Implements a linear controller
    u = K1*input[x1] + ... + Kn*input[xn] + offset
    """
    def __init__(self,type='torque'):
        import numpy as np
        self.outputType = type
        self.gains = dict()
        self.offset = None
    def output(self,**inputs):
        res = self.offset
        for (x,K) in self.gains.iteritems():
            if x not in inputs:
                print "LinearController: warning, input",x,"doesn't exist, ignoring"
                continue
            if res==None:
                res = np.dot(K,inputs[x])
            else:
                res += np.dot(K,inputs[x])
        return {outputType:res}
        
def make(robot):
    return BaseController()

def launch(fn,robot):
    """Launches a controller given by the given python module for the
    given robot using the module's default make(robot) routine."""
    import os
    import importlib
    path,base = os.path.split(fn)
    mod_name,file_ext = os.path.splitext(base)
    mod = importlib.import_module(path+"."+mod_name,fn)
    try:
        maker = mod.make
    except AttributeError:
        print "Module",mod.__name__,"must have a make() method"
        raise
    return maker(robot)
