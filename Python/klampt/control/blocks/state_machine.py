"""
Implements state machines as :class:`ControllerBlock` structures.

- :class:`StateMachineController`: a state machine that switches between
  multiple sub-controllers, with one sub-controller running at once and the
  active controller triggered by some signal.
- :class:`TransitionStateMachineController`: a state machine with explicit
  transition conditions.

"""

from ..controller import ControllerBlock

class StateMachineBase(ControllerBlock):
    """A base class for a finite state machine controller. One sub-controller
    may be active at once. 

    To implement transitions, :meth:`next_state` needs to be filled out by the
    subclass.

    The state machine calls signals 'enter' and 'exit' for each sub-controller.
    It also responds to the 'reset' signal, which goes back to the start state.

    Attributes:
        controllers (list of ControllerBlock): the list of sub-controllers
        current (int): the index of the currently active sub-controller. -1
            indicates no current sub-controller.
        start (int): the index of the initial sub-controller.
    """
    def __init__(self,controllers,start=0):
        self.controllers = controllers
        self.current = start
        self.start = start
    def next_state(self,state,**inputs):
        """Subclasses should override this to implement the transitions"""
        return state

    def __str__(self):
        strs = [str(c) for c in self.controllers]
        strs[self.current] = '* '+strs[self.current]
        return self.__class__.__name__+'\n'+'\n  '.join(strs)
    def inputNames(self):
        if self.current < 0: return []
        return self.controllers[self.current].inputNames()
    def inputValid(self,**inputs):
        if self.current < 0: return True
        return self.controllers[self.current].inputValid(**inputs)
    def outputNames(self):
        if self.current < 0: return []
        return self.controllers[self.current].outputNames()
    def signal(self,type,**inputs):
        if type=='reset':
            if self.current >= 0:
                self.controllers[self.current].signal('exit',**inputs)
            self.current = self.start
            if self.current >= 0:
                self.controllers[self.current].signal('enter',**inputs)
        else:
            if self.current >= 0:
                self.controllers[self.current].signal(type,**inputs)
    def advance(self,**inputs):
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


class TransitionStateMachine(StateMachineBase):
    """A state machine controller with a transition matrix that determines
    when to move to the next state.

    Attributes:
        controllers: same as StateMachineBase
        current: same as StateMachineBase
        transitions (list of dicts): a list of transition conditions
            [{x1:cond1,...,xk:condk},...,{}] so that if j is in transitions[i],
            then transitions[i][j](inputs) is a condition that tells the state
            machine whether to change states from i to j.
    """
    def __init__(self,controllers,transitions=None,start=0):

        StateMachineBase.__init__(self,controllers,start)
        if transitions is None:
            self.transitions = [dict() for c in self.controllers]
        else:
            if len(transitions) != len(controllers):
                raise ValueError("Transitions is in invalid format")
            self.transitions = transitions

    def set_transition(self,fromState,toState,func):
        assert fromState >= 0 and fromState < len(self.transitions)
        assert toState >= 0 and toState < len(self.transitions)
        self.transitions[fromState][toState] = func

    def next_state(self,state,**inputs):
        if state < 0: return state
        trans = self.transitions[state]
        for (k,v) in trans.iteritems():
            if v(inputs):
                print("TransitionStateMachine: Transition to controller",k,":",str(self.controllers[k]))
                return k
        return state

