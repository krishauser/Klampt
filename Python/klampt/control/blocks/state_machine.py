"""
Implements state machines as :class:`Block` structures.

- :class:`StateMachineBase`: a state machine that switches between
  multiple sub-controllers, with one sub-controller running at once and the
  active controller triggered by some signal.
- :class:`TransitionStateMachine`: a state machine with explicit
  transition conditions.
- :class:`TimedSequenceBlock`: a sequence of controllers, switched by
  time.

"""

from .core import Block
from .utils import BlockSignal

class StateMachineBase(Block):
    """A base class for a finite state machine controller. One sub-controller
    may be active at once. 

    To implement transitions, :meth:`next_state` needs to be filled out by the
    subclass.

    If a sub-block has inputs 'enter' or 'exit', they are raised when the
    sub-controller enters or exits.

    Signals:
        - 'reset' (bool): when 1, the state machine goes back to the start
          state and resets all blocks.

    Outputs:
        - 'state' (int): the currently active state.

    Attributes:
        blocks (list of Block): the list of sub-controllers
        current (int): the index of the currently active sub-controller. -1
            indicates no current sub-controller.
        start (int): the index of the initial sub-controller.
    """
    def __init__(self,blocks,start=0,reset_on_enter=True):
        Block.__init__(self,[],['state'])
        self.blocks = blocks
        self.current = start
        self.start = start
        self.reset_on_enter = reset_on_enter

    def next_state(self,state,*args,**kwargs):
        """Subclasses should override this to implement the transitions"""
        return state

    def __str__(self):
        strs = [str(c) for c in self.blocks]
        strs[self.current] = '* '+strs[self.current]
        return self.__class__.__name__+'\n'+'\n  '.join(strs)

    def signal(self,type,*args):
        if type=='reset' or (type == 'enter' and self.reset_on_enter):
            if self.current >= 0:
                self.blocks[self.current].signal('exit')
            self.current = self.start
            if self.current >= 0:
                self.blocks[self.current].signal('enter')
        else:
            if self.current >= 0:
                self.blocks[self.current].signal(type,*args)

    def advance(self,*args,**kwargs):
        if self.current<0: return None
        n = None
        try:
            res = self.blocks[self.current]._process()
        except BlockSignal as s:
            if s.type == 'change_state':
                n = int(s.text)
            else:
                self.signal(s.type)

        if n is None:
            n = self.next_state(self.current,*args,**kwargs)
        if n != self.current:
            self.blocks[self.current].signal('exit')
            if n >= 0:
                self.blocks[n].signal('enter')
            self.current = n
        return self.current
        
    def func(self,name):
        if self.current>=0:
            self.blocks[self.current].func(name)

    def __getstate__(self):
        blockState = []
        for c in self.blocks:
            try:
                s = c.__getstate__()
            except NotImplementedError:
                s = None
            blockState.append(s)
        return {'current':self.current,'blocks':blockState}
    
    def __setstate__(self,state):
        if 'current' not in state or 'blocks' not in state or len(state['blocks']) != len(self.blocks):
            raise ValueError("Invalid state dict")
        self.current = state.current
        for (c,s) in zip(self.blocks,state['blocks']):
            if s is not None:
                c.__setstate__(s)


class TransitionStateMachine(StateMachineBase):
    """A state machine controller with a transition matrix that determines
    when to move to the next state.

    Attributes:
        blocks: same as StateMachineBase
        current: same as StateMachineBase
        transitions (list of dicts): a list of transition conditions
            [{x1:cond1,...,xk:condk},...,{}] so that if j is in transitions[i],
            then transitions[i][j](*inputs) is a condition that tells the state
            machine whether to change states from i to j.
    """
    def __init__(self,blocks,transitions=None,inputs=0,start=0,reset_on_enter=True):
        StateMachineBase.__init__(self,blocks,start,reset_on_enter)
        Block.__init__(self,inputs,['state'])
        if transitions is None:
            self.transitions = [dict() for c in self.blocks]
        else:
            if len(transitions) != len(blocks):
                raise ValueError("Transitions is in invalid format")
            self.transitions = transitions

    def set_transition(self,fromState,toState,func):
        assert fromState >= 0 and fromState < len(self.transitions)
        assert toState >= 0 and toState < len(self.transitions)
        self.transitions[fromState][toState] = func

    def next_state(self,state,*args,**kwargs):
        if state < 0: return state
        trans = self.transitions[state]
        for (k,v) in trans.items():
            if v(*args,**kwargs):
                print("TransitionStateMachine: Transition to controller",k,":",str(self.blocks[k]))
                return k
        return state


class TimedSequenceBlock(TransitionStateMachine):
    """A state-machine controller that goes through each sub-controller
    in sequence.

    Input:
        t (float): the current time
    """
    def __init__(self,blocks,times):
        assert len(times)==len(blocks)
        trans = [{} for c in blocks]
        for i in range(len(blocks)-1):
            trans[i][i+1] = lambda t:t >= times[i]
        trans[-1][-1] = lambda t:t >= times[-1]
        TransitionStateMachine.__init__(self,blocks,trans,['t'])

