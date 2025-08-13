"""A generalizable framework for multi-step planning.  Notably used in
manipulation planning.

The problem that this package solves is that choices made in early stages
of a multi-step plan may not yield a feasible solution later on.  A naive
approach would be to repeatedly attempt the sequence of steps, but this is
not as efficient as it could be.  The multi-step planner framework allows
flexible sequences of reasoning to be employed by the planner, i.e.,
prioritizing low-likelihood bottlenecks first, and then solving for the
remaining items later. 

Unlike a procedural multi-step plan, in which your code solves step 1, then
step 2, etc., this package implements a *declarative* framework in which your
code declares the sequence of interrelated steps in a *planning graph*.  We
implement a Pythonic approach to setting up the planning graph which is as
close as possible to writing native procedural code, while providing the
flexibility required for more efficient planning.

Usage:

1. Declare elements of the planning graph via :func:`constant` and
    :func:`node`.  Several nodes are provided to perform standard tasks,
    such as kinematic motion planning and IK solving.  You can pass
    Python functions to :func:`node` as well. 
2. Declare any validity checks for items in the planning graph via
    `item.add_constraint(func)`.
3. To assist in debugging, set the names of intermediate items
    via `item.set_name(name)`.
4. Create a :class:`MultiStepPlanner` subclass, such as
    :class:`IntrospectiveMultiStepPlanner`.  Set one or more target items
    to solve for using `planner.add_target()`.  The multi-step planner
    will automatically compute the planning graph from the target items.
5. The planning graph will be checked for validity; correct any errors
    that are reported. 
6. Set any priors for runtime statistics in the planner, if you are using
    :class:`IntrospectiveMultiStepPlanner`. This is essential for optimal
    performance, as the planner will prioritize nodes that are cheap and
    difficult (bottlenecks).
7. Call `planner.solve(max_iters, max_time)` to run the planner until it
    finds a solution, or runs out of iterations or time.
8. To debug failures, you can check `planner.failed_plans()` to see which
    nodes failed and `planner.stalled_plans()` to see what plans were
    attempted.


Planning Graph
--------------

The multi-step planner framework uses a planning graph approach that is
designed to be flexible and Pythonic.  The key ingredients of a planning
graph are *items* and *nodes*. 

- An item is any object that is provided as a constant or produced by a node,
    such as a configuration, path, or transform.  
- A node performs some element of planning when the multi-step planner
    triggers it.  It returns one or more results.  You declare a relationship
    between a node's inputs and outputs by calling it, e.g.,
    `out_item = some_node(in_item1,in_item12)` (TODO: support multiple outputs)

To declare a planning graph, you will first create some constant items, then
add nodes to the planner.  You will declare the computation graph using syntax
following this example::

    a = constant('a',value1)  #declares a constant item named 'a'
    b = constant('b',value2)  #declares a constant item named 'b'
    c = node('node_name',node)(a,b,3.0)   #note: the value 3.0 will be converted automatically to a constant PlanItem
    d = node('another_node',another_node)(c)
    c.set_name('c')  #sets the name of item 'c' to 'c'
    d.set_name('final_output')  #names don't have to match the python variable name
    planner = IntrospectiveMultiStepPlanner(context)
    planner.add_target(d)  #declares that the planner will solve for item 'd' and checks the planning graph
  

Validity checking
-----------------

There are two types of validity checking performed in planning:

- At planning graph construction time, we ensure that arguments to 
    nodes match the signature of each node, and that there are no
    floating items that are neither constants nor outputs of nodes.
- At planning time, we ensure that the values of items are valid.
    An item contains a set of validity checks, and the multi-step
    planner will ensure that item values obey all specified constraints.

    
Nodes
------

Several nodes are provided to perform standard tasks, such as kinematic
motion planning and IK solving.  You can pass simple functions to `node`,
which are assumed to produce None upon failure.  Or, you can subclass
:class:`NodeBase` to implement a custom node that performs some
planning functionality.

When creating your own non-trivial nodes, it is very important to remember
that the node is essentially a "factory" that gets instantiated for each set
of arguments.  If you return CONTINUE, the multi-step planner will potentially
call the node again with the same arguments, OR it might call the node with
some other set of arguments in another partial plan that it is exploring. 
Hence, it needs a mechanism to save and restore internal state between steps.
So, you will need to implement the `get_state()` and `set_state()` methods.

Nodes have a `properties` dictionary that holds any annotations that can
help the multi-step planner optimize the planning process.  Common properties
include:
- `deterministic` (bool): the node implements a deterministic function.
- `procedural` (bool): the node is a procedural function that terminates
    with a single iteration of `plan()`.
- `time_prior` (float): a prior for how long the node takes to complete.
- `valid_prior` (float): a prior for how likely the node is to produce
    a valid result.
- `prior_strength` (float): a prior for how strong the node's priors are.


Context
-------

Every node's `plan()` method will take in a first argument that is the
*context* of the planner. This is any object that you would like, and using it
is best practice for passing in a world, robot, objects, algorithm parameters,
etc.  It is recommended to refrain from using global variables, as this will
make your planning graph more modular and reusable.

Many built-in nodes assume the following context attributes:
- robot: the robot model to plan for, which is a :class:`RobotModel`.  

Current world state
-------------------

There is no notion of the "current" world state in the multi-step planner,
i.e., the current configuration of the robot and objects, because its reasoning
must have the flexibility to jump around in time.  This is a potential pitfall
because planning steps treat the configurations of models as temporary
variables, so you can get into trouble when you are implementing a node if you
assume that a model keeps the same configuration as it was in during
construction time.  For correctness of the plan, it is very important to ensure
that movable objects are in the correct pose before invoking a motion planner
or collision checker.

To do so, you can use the `with multistep.set_config(...)` construction, which
has the following syntax::

    with multistep.set_config((context.object, object_config)):
        path = multistep.node('plan_path', multistep.KinematicMotionPlannerNode(sspace, context.planner_settings))(startConfig, endConfig)

This will set the configuration of `context.object` to `object_config` whenever
the 'plan_path' node is called.  You can set multiple objects at once, e.g.,::

    with multistep.set_config((context.robot, robot_config), (context.object, object_config)):
        ...

The first argument in each pair is a Klampt object and the second argument can
either be a constant or a PlanItem that is generated from another step.


Performance Notes
-----------------

These planning algorithms are designed to operate in least-committment
fashion, in which multiple candidate plans are considered, and then a node
will be chosen to solve one or more items that are not yet solved for in
a candidate plan.

The strategy for choosing the candidate plan is to consider the remaining
items left in the plan (:class:`MultiStepPlanner`), possibly weighted by
the complexity of finding the items in it
(:class:`IntrospectiveMultiStepPlanner`).

The strategy for choosing the next node to run is to consider any applicable
node to the current plan (:class:`MultiStepPlanner`), or to select the
applicable node that has the lowest expected cost to prove infeasibility
(:class:`IntrospectiveMultiStepPlanner`). The reason why we want to
choose the node that is least likely to find a solution is that all nodes
must be solved for in order for the solution to be complete, and if we
expend too much effort with the current candidate plan, the planner should
deviate to another candidate plan that has not yet been explored.

Note that current implemented methods are forward-reasoning methods, meaning
that plans are incrementally constructed in steps as long as the precursor 
items for a step have been solved for.  The heuristics are reasonable for
selecting amongst those steps, and this is a convenient mental model of multi-
step planning compared to alternative formulations like constraint formulation.
However, there are some problems in which this is not the best strategy.

Consider a problem in which there are 100 target objects, of which 5 are
reachable, and the goal is to swap two of them.  A forward-reasoning
formulation would first select the two target objects, then solve for reaching
configurations and a put-aside location, then solve for paths.  The problem
is that the likelihood of finding object indices that are reachable is
0.05*0.05 = 0.0025, which is quite low, meaning that approximately 400
attempts would be needed (assuming random sampling) to find successful options
for the downstream configuration / path solving steps.  A backward-reasoning
approach might solve for configurations first, then select two objects that
are reachable, and then find the paths.  This would be several times more
efficient (and its comparative efficiency grows as the fraction of reachable
targets shrinks).   

Single-step motion planning nodes currently do not share information amongst
active plans.  This means that if the multi-step planner is running several
queries for the same C-space, it will not share roadmaps between them.  Future
work might explore improving running times through information sharing.


Optimality
----------
There are no optimality guarantees for the multi-step planner, as it is
currently set up to operate in feasible planning mode.  You can continue
to run the planner with the `plan()` method and it will continue to
generate new plans, but it is currently up to the user to maintain
the best plan found so far. 

TODO: consider cost functions that get included in the choice of candidate
plans.


Limitations / TODOs
--------------------

Current implementation does not support multiple outputs from nodes.

Current forward planning implementation does not support or-branching or
otherwise dynamic determination of which targets need to be solved for.  To
support this kind of reasoning, you will need to set up an early stage node
that determines which targets to solve for. 

We don't yet have a mechanism for prioritizing based on deterministic,
dependencies which may lead to priority inversion.  I.e, A and B are both
applicable to a plan, and C is dependent on the output of B.  A has a moderate
chance of success and is more expensive, while B is cheap and has a 100% chance
of success.  However, C is cheap and has a low chance of success.  The optimal
strategy would be to finish B first then attempt C, but at the moment the
planner only considers A vs B, and it will prioritize A because it has a lower
chance of success.  

Input-output type checking is exact.  You can pip install the `typeguard` module
for better type checking than the defaults in Python, which do not support
subscripted generics (e.g., List[List[float]]).

The cost of validity checking is not considered in prioritization, so checks
will be run every time a node produces an output. A smarter thing to do would
be to include checking as a node in the planning graph.

Independent subtrees are solved in forward search manner, which does not
lend itself to sharing information amongst candidate plans.  For example, if
A and B are independent and C depends on both, then the planner will proceed
along two branches: 1) solve for A, then B "conditional on" A, and finally C,
and 2) solve for B, then A "conditional on" B, and finally C.  A more efficient
approach would enumerate combinations of solutions for A and B solved
independently.


"""

from __future__ import annotations
from klampt.plan import cspace,robotplanning
from klampt import IKObjective
from klampt.model import ik
from klampt.model.typing import Config,Configs,RigidTransform
from klampt.model import trajectory,cartesian_trajectory
from klampt.model import config
from klampt.math import se3
from enum import Enum
import heapq
import copy
import time
import warnings
import inspect
from contextlib import contextmanager
from collections.abc import Iterator,Iterable
from typing import Dict,List,Tuple,Union,Any,Optional,Callable,Generator


class PlanItem:
    """A placeholder that declares metadata for items in a multi-step plan.

    Can include name, type, validity checks, and generating node.
    """
    def __init__(self,
                 name : Optional[str] = None,
                 value_type : Optional[type] = None,
                 constant : Any = None,
                 parent : Optional[NodeBase] = None,
                 parent_inputs : Optional[List[PlanItem]] = None):
        self.name = name
        self.value_type = value_type
        self.constant = constant
        self.parent = parent
        self.parent_inputs = parent_inputs
        self.validity_checks = []
    
    def __str__(self):
        if self.name is None:
            if self.value_type is not None:
                return "unknown : {}".format(self.value_type.__name__)
            return "unknown"
        if self.value_type is not None:
            return "{} : {}".format(self.name, self.value_type.__name__)
        return self.name

    def set_name(self, name : str) -> PlanItem:
        """Sets the name of this item.  This is used to identify the item in
        the planning graph."""
        if not isinstance(name, str):
            raise ValueError("Name must be a string")
        self.name = name
        return self

    def set_type(self, value_type : type) -> PlanItem:
        """Sets the type of this item.  This is used to check the validity
        of the planning graph and generated values."""
        if self.value_type is not None:
            raise ValueError("Cannot set type of PlanItem that already has a type")
        if not isinstance(value_type, type):
            raise ValueError("Type must be a type object")
        self.value_type = value_type

    def add_constraint(self, check : Callable[[Any],bool]) -> PlanItem:
        """Adds a validity check for this item.  The check should be a function
        that takes a value and returns True if the value is valid, or False
        otherwise."""
        if not callable(check):
            raise ValueError("Validity check must be a callable function")
        self.validity_checks.append(check)
        return self

    def valid(self, value : Any) -> bool:
        """Returns true if the value is a valid setting for this item. """
        if self.value_type is not None:
            try:
                if not isinstance(value, self.value_type):
                    return False
            except TypeError:
                #must be a subscripted generic type, try using typeguard
                try:
                    from typeguard import check_type
                    if not check_type(value, self.value_type):
                        return False
                except ImportError:
                    pass
        try:
            for check in self.validity_checks:
                if not check(value):
                    return False
        except Exception as e:
            #some validity check failed, so raise an exception with more information
            raise RuntimeError("Validity checking failed on {}, value {}: {}".format(self,value,e))
        return True

    def __getitem__(self, item : Any) -> Any:
        """Allows indexed access to the results of this node."""
        def index(arr,idx):
            return arr[idx]
        return LambdaProcedureNode(index,include_context=False)(self,item)
    

class NodeBase:
    """A planner node base class.  This is used to implement a step in a
    multi-step planning algorithm. 

    A planner node can continue (indefinitely) to generate possible options
    for its outputs.  Or, it can be a simple process that terminates with a
    result.
    
    The plan() method must be a generator function, returning a sequence of
    iterates using `yield`.  Use `yield None` to indicate that planning is
    still proceeding but the multi-step planner can give time to consider
    other plans. The multi-step planner will keep track of progress by storing
    the iterator with a candidate plan, and may resume an iterator on a future
    steps.

    `properties` stores annotations about the node's behavior, and typical
    properties of a node include:
    - deterministic (bool): whether the node is a deterministic function of
        its inputs.
    - procedural (bool): whether the node is a procedural function that
        terminates with a single iteration of `plan()`
    """
    def __init__(self):
        self.name = None
        self.properties = {}
        self._results = []
    
    def __str__(self):
        if self.name is None:
            return self.__class__.__name__
        else:
            return self.name
    
    def signature(self) -> Tuple[Tuple[PlanItem],PlanItem]:
        """Returns a tuple of the inputs and outputs of this node.
        
        Inputs are a tuple of PlanItem objects, which can include metadata
        such as type checks and validity checks.  Outputs are a single
        PlanItem object that indicates the result of the step.
        """
        raise NotImplementedError
    
    def __call__(self, *inputs : PlanItem) -> PlanItem:
        converted_inputs = []
        for i in range(len(inputs)):
            assert not isinstance(inputs[i], NodeBase), \
                "Node {} input {} is a NodeBase, needs to be a PlanItem or constant".format(self, i)
            if isinstance(inputs[i], PlanItem):
                converted_inputs.append(inputs[i])
            else:
                converted_inputs.append(PlanItem(constant=inputs[i]))
        inputs = converted_inputs
        try:
            argsig,retsig = self.signature()
        except NotImplementedError:
            return PlanItem(parent=self, parent_inputs=inputs)  #if the signature is not implemented, just return a PlanItem with the inputs
        if len(inputs) != len(argsig):
            raise ValueError("Node {} expected {} inputs, got {}".format(self, len(argsig), len(inputs)))
        for i in range(len(inputs)):
            if inputs[i].value_type is not None and argsig[i].value_type is not None and not (inputs[i].value_type is argsig[i].value_type):
                raise ValueError("Node {} input {} type {} mismatch, expected type {}".format(self, inputs[i], inputs[i].value_type, argsig[i].value_type))
        return PlanItem(None,retsig.value_type,parent=self,parent_inputs=inputs)  #return a PlanItem that has the same type as the output signature
    
    def solve(self, context : Any, max_iters : int, max_time : float = None) -> Any:
        """Runs the node until it finds a solution, completes, fails,
        runs out of iterations, or runs out of time.
        
        Returns solution if found, or None if a failure is encountered,
        the iteration or time limits are met.
        """
        import math
        if max_time is None:
            max_time = math.inf
        t0 = time.time()
        iters = 0
        gen = self.plan(context)
        for res in gen:
            if res is not None:
                return res
            iters += 1
            if iters >= max_iters:
                print("{} reached max iterations of {}".format(self.__class__.__name__,max_iters))
                return None
            if time.time() - t0 > max_time:
                print("{} reached max time of {}".format(self.__class__.__name__,max_time))
                return None        
            
    def plan(self, context : Any) -> Generator:
        """Returns a generator that performs increments of planning."""
        raise NotImplementedError



def constant(name : str, value : Any, value_type : type = None) -> PlanItem:
    """Returns a PlanItem that is a constant value.  This is used to
    declare constants in the plan."""
    if not isinstance(value, PlanItem):
        value_type = value_type or type(value)
        return PlanItem(name, value_type, constant=value)
    else:
        if value.constant is not None:
            raise ValueError("Cannot declare a constant PlanItem with a constant value")
        value.name = name
        return value


_with_configs = []

def node(name : str,
         node : Union[Callable,NodeBase],
         can_fail : bool=True) -> NodeBase:
    """Creates a node and assigns its name.  The node argument can be
    a function or a NodeBase object. 
    
    If a function is given, it will be wrapped in a LambdaNode or
    LambdaProceduralNode that includes the context as the first argument. It is
    helpful to set can_fail to False if the function is guaranteed to
    succeed.
    """
    if not isinstance(node,NodeBase):
        assert callable(node), "Node must be a callable or NodeBase"
        if inspect.isgeneratorfunction(node):
            node = LambdaGeneratorNode(node, include_context=True)
        else:
            if not can_fail:
                node = LambdaProcedureNode(node, include_context=True)
            else:
                node = LambdaNode(node, include_context=True)
    node.name = name
    global _with_configs
    if len(_with_configs) > 0:
        objects = [tup[0] for tup in _with_configs]
        configs = tuple(tup[1] for tup in _with_configs)
        return lambda *inputs: WithConfigNode(node, objects)(*(configs+inputs))
    return node


@contextmanager
def set_config(*items : Tuple):
    """Takes one or more (object,config) pairs as arguments.  Each object
    will have its configuration set to the configuration for any future
    :func:`node` calls. 
    
    This is useful for ensuring that contextual objects (e.g., rigid objects,
    the robot) are in the correct configuration for collision checking, motion
    planning, etc.

    Configurations can be PlanItems or raw Python objects.
    """
    global _with_configs
    for tup in items:
        if len(tup) != 2:
            raise ValueError("set_config arguments must be (object,config) pairs")
        obj,config = tup
        if not hasattr(obj,'setConfig') and not hasattr(obj,'setTransform') and not hasattr(obj,'setCurrentTransform'):
            raise ValueError("Object {} does not have setConfig, setTransform, or setCurrentTransform methods".format(obj))

    _with_configs.extend(items)
    try:
        yield
    finally:
        _with_configs = _with_configs[:-len(items)]



class MultiStepPlannerBase(NodeBase):
    """Base class for multi step planners.  Users will want to use
    MultiStepPlanner or IntrospectiveMultiStepPlanner. 

    Usage is to first set up a planning graph, including constants,
    nodes, and targets.  Then, you call solve() to run the planner
    (or call `for res in plan()` multiple times to run the planner
    in a loop).

    Any intermediate plans are stored as a dictionary.  Items in the
    graph will have their values stored under the names of the PlanItem
    objects that they correspond to.  It is usually good practice to
    set the `PlanItem.name` variables to meaningful names, otherwise
    they are given names like _item0_X, _item1_Y, etc.
    
    Users will use the methods:

        - constant(name,value): declare a constant value
        - node(name,node): declare a node
        - set_target(item): declares that the planner will solve for the
            given item.
        - add_target(item): declares that the planner will solve for the
            given item (as well as any other previously added targets).
        - solve(max_iters, max_time=None): run the planner until it finds a
            solution, or time runs out.
        - reset(): reset the planner to its initial state.
    
    Subclass implementers need to override the plan() method, which should
    perform some unit of planning.
    """
    def __init__(self, context: Any):
        super().__init__()
        self.context = context            #type: Any #the context for the planner, containing any non-variable items
        self.targets = []                 #type: List[PlanItem] # list of items that the planner will solve for
        self.initial_plan = {}            #dict: the plan to be modified by the planner
        self.nodes = {}                   #type: Dict[str,NodeBase] # list of nodes defining planning steps
        self.items = {}                   #type: Dict[str,PlanItem] # list of items in the planning graph.  Will be recalculated after add_target()
        self.node_inputs = {}             #type: Dict[str,List[PlanItem]]
        self.node_outputs = {}            #type: Dict[str,PlanItem]
        self.node_stats = {}  #type: Dict[str,Dict[str,int]]
        self.item_stats = {}  #type: Dict[str,Dict[str,int]]
        self._failed_plans = {}  #type: Dict[str,List[Dict[str,Any]]] #a dictionary of failed plans, indexed by failed node name
        self._solution_plans = []  #type: List[Dict[str,Any]] #a list of solution plans, each is a dictionary of PlanItem names to values

    def set_target(self, item : PlanItem):
        """Specifies that the given item is the only output of the planner.
        
        The computation graph will be built from this target.
        """
        self.targets = [item]
        self.compute_planning_graph()

    def add_target(self, item : PlanItem):
        """Specifies that the given item is an output of the planner.
        
        The computation graph will be built from the list of targets.
        """
        for t in self.targets:
            if item == t:
                raise ValueError("Target item {} already exists in planner".format(item))
        self.targets.append(item)
        self.compute_planning_graph()
    
    def compute_planning_graph(self):
        """Computes the planning graph from the targets. The user does not usually
        need to call this."""
        self.items = {}
        self.nodes = {}
        self.node_inputs = {}
        self.node_outputs = {}
        #discover all items in the planning graph, assign names if names are not set
        item_set = dict()
        from collections import deque
        queue = deque()
        for item in self.targets:
            if item.name is None:
                item.name = "_target{}".format(len(item_set))
                assert item.name not in self.items, \
                    "Target item {} has no name, but name {} already exists in planner".format(item, item.name)
            item_set[id(item)] = item
            queue.append(item)
        while len(queue) > 0:
            item = queue.popleft()
            assert item.name not in self.items, \
                "Item named {} duplicated in planner".format(item.name)
            self.items[item.name] = item
            if item.parent is not None:
                #add all parent inputs to the queue
                if item.parent.name is None:
                    item.parent.name = "_node{}_{}".format(len(self.nodes),item.parent)
                    assert item.parent.name not in self.nodes, \
                        "Node {} has no name, but name {} already exists in planner".format(item.parent, item.parent.name)
                node_name = item.parent.name
                assert node_name not in self.nodes, \
                    "Node named {} duplicated in planner".format(node_name)
                self.nodes[node_name] = item.parent
                self.node_inputs[node_name] = []
                self.node_outputs[node_name] = item
                for parent_input in item.parent_inputs:
                    if id(parent_input) not in item_set:
                        if parent_input.name is None:
                            parent_input.name = "_item{}_{}".format(len(item_set),item.parent)
                            assert parent_input.name not in self.items, \
                                "Item {} has no name, but name {} already exists in planner".format(item, item.name)
                        item_set[id(parent_input)] = parent_input
                        queue.append(parent_input)
                    self.node_inputs[node_name].append(parent_input)
            else:
                assert item.constant is not None, \
                    "Item {} has no parent or constant value, cannot be part of planning graph".format
                if not item.valid(item.constant):
                    raise ValueError("Item {} with constant value {} is not valid".format(item, item.constant))

    def reset(self):
        """Resets the planner to its initial state."""
        super().reset()
        for node in self.nodes.values():
            node.reset()
        self.node_stats = {}
        self.item_stats = {}
    
    def solve(self, max_iters : int, max_time : float = None) -> Any:
        """Runs the planner until it finds a solution, exhausts all plans,
        runs out of iterations, or runs out of time.
        
        Returns the target(s), if a solution is found, or None if no solution
        is found within the iteration or time limits.
        """
        assert self.is_valid(), "Planning graph is not valid, cannot solve"
        result = super().solve(self.context, max_iters, max_time)
        if result is not None:
            self._solution_plans.append(result)
            if len(self.targets) == 1:
                return result[self.targets[0].name]
            return [result[t.name] for t in self.targets]
        return None
            
    def applicable_nodes(self, plan: dict) -> Iterator[str]:
        """Returns an iterator over all nodes that can be run given
        the current plan."""
        for node_name in self.nodes.keys():
            outputs = self.node_outputs[node_name]
            if outputs.name not in plan:
                inputs = self.node_inputs[node_name]
                if all((i.constant is not None or i.name in plan) for i in inputs):
                    yield node_name
        return
    
    def is_valid(self) -> bool:
        """Returns true if the planner could possibly solve the problem."""
        for item_name,item in self.items.items():
            if item.parent is None:  #root node
                if item.constant is None:
                    print("Planner is not valid, root item {} has no constant value".format(item))
                    return False
            if item.constant is not None:
                if not item.valid(item.constant):
                    print("Planner is not valid, constant item {}={} is not valid".format(item,item.constant))
                    return False
        #check whether at least one node can be run at the initial state
        if len(list(self.applicable_nodes(self.initial_plan))) == 0:
            print("Planner is not valid, no step can be run at the initial state")
            return False
        return True

    def run_node(self, node : str, plan : dict, node_state : Iterator = None) -> Tuple[Any,Iterator]:
        """Runs a node on the plan, returning the result of the step and any internal state."""
        if node not in self.node_stats:
            self.node_stats[node] = { 'count': 0, 'total_time': 0.0, 'successes': 0, 'failures': 0, 'results': 0 }
        stats = self.node_stats[node]
        stats['count'] += 1

        node_obj = self.nodes[node]
        t0 = time.time()
        if node_state is not None:
            try:
                res = next(node_state)
            except StopIteration:
                res = None
                node_state = None
        else:
            inputs = self.node_inputs[node]
            plan_inputs = [(i.constant if i.constant is not None else plan[i.name]) for i in inputs]
            assert not any(i is None for i in plan_inputs), \
                "Node {} inputs {} are not all set in plan {}".format(node,inputs,plan)
            node_state = node_obj.plan(self.context, *plan_inputs)
            try:
                res = next(node_state)
            except StopIteration:
                res = None
                node_state = None
        t1 = time.time()
        stats['total_time'] += (t1 - t0)
        if res != None:
            if node_obj.properties.get('deterministic', False):
                #deterministic node, so we can assume it will not continue
                node_state = None
            if node_state is None:
                stats['successes'] += 1
            valid = True
            outputs = [self.node_outputs[node]]
            #check validity of all outputs
            assert len(outputs) == len([res]), \
                "Node {} outputs {} do not match length of result {}".format(node,outputs,res)
            for oitem,val in zip(outputs,[res]):
                if oitem.name not in self.item_stats:
                    self.item_stats[oitem.name] = { 'count': 0, 'total_time':0.0, 'valid': 0 }
                istats = self.item_stats[oitem.name]
                istats['count'] += 1
                istats['total_time'] += (t1 - t0)
                try:
                    if oitem.valid(val):
                        istats['valid'] += 1
                    else:
                        valid = False
                except Exception:
                    raise ValueError("Possible type error when checking validity of item {} with value {}".format(oitem, val))
            if valid:
                stats['results'] += 1
            else:
                res = None
        else:
            if node_state is None:
                stats['failures'] += 1
                if node not in self._failed_plans:
                    self._failed_plans[node] = []
                self._failed_plans[node].append(copy.copy(plan))  #store the failed plan for debugging
        return res,node_state

    def solution_plans(self) -> List[Dict[str,Any]]:
        """Returns a list of solution plans found by the planner.
        
        Each plan is a dictionary mapping PlanItem names to values."""
        return self._solution_plans

    def failed_plans(self) -> Dict[str,List[Dict[str,Any]]]:
        """Returns a dictionary of failed plans, indexed by node name."""
        return self._failed_plans

    def stalled_plans(self) -> Dict[str,List[Dict[str,Any]]]:
        """Returns a dictionary of stalled plans, indexed by node name.
        
        A stalled plan is a plan that has not been completed or failed, but
        has not produced any new results in the last step."""
        raise NotImplementedError("Stalled plans are not implemented in MultiStepPlannerBase, please implement in subclass")



class _PartialPlan:
    def __init__(self, items_solved : dict):
        """A data structure that holds the solved items in a plan and information
        about solve attempts."""
        self.items_solved = items_solved  #type: Dict[str,Any] #dictionary of items solved in this plan
        self.node_count = {}          #type: Dict[str,int] #number of times each node has been run on this plan
        self.node_states = {}         #type: Dict[str,Any] #state of each node on this plan
        self.applicable_nodes = None  #type: List[str] #list of nodes that can be run on this plan
        self.last_node = None

    def copy(self) -> _PartialPlan:
        """Returns a copy of this plan."""
        new_plan = _PartialPlan(self.items_solved.copy())
        new_plan.node_count = self.node_count.copy()
        new_plan.node_states = self.node_states.copy()
        new_plan.applicable_nodes = self.applicable_nodes.copy() if self.applicable_nodes is not None else None
        new_plan.last_node = self.last_node
        return new_plan


class _PQNode:
    def __init__(self, item, priority):
        self.item = item
        self.priority = priority

    def __lt__(self, other : _PQNode):
        return self.priority < other.priority

    def __str__(self):
        return str("{} : {}".format(self.item, self.priority))



class MultiStepPlanner(MultiStepPlannerBase):
    """
    A solver for a multi-step planning problem that operates by least committment
    search.

    `context` is an argument that is passed to all NodeBase.plan() methods.
    It can include global worlds, robots, logging, etc.
    """
    def __init__(self, context):
        super().__init__(context)

    def plan(self, context: Any, *inputs) -> Generator:
        """A generator implementation of the multi-step planner.  Performs
        a unit of planning on each next() call."""
        self.plans = [] #type: List[_PQNode]
        self.push_plan(_PartialPlan(self.initial_plan)) 

        while len(self.plans) > 0:        
            priority,plan = self.pop_plan()
            node = self.select_node(plan)
            if node is None:
                #no node applicable, check whether it's a solution
                if all(i.name in plan.items_solved for i in self.targets):
                    #all items are solved for, so this is a solution
                    yield plan.items_solved
                else:
                    #some target is not solved for, so it is not a solution
                    print("  No applicable node, but plan is not a solution?")
                    for node in self.nodes:
                        print("  Node {}".format(node))
                        if self.node_outputs[node].name not in plan.items_solved:
                            print("   output {} not solved".format(self.node_outputs[node]))
                        else:
                            print("   output {} solved".format(self.node_outputs[node]))
                            for i in self.node_inputs[node]:
                                if i.name not in plan:
                                    print("   input {} is not solved".format(i.name))
                                else:
                                    print("   input {} is solved".format(i.name))
                    yield None
            print("Selected node {} for plan with {} items solved, priority {}".format(node,len(plan.items_solved),priority))
            plan.last_node = node
            node_state = plan.node_states.get(node, None)
            res,node_state = self.run_node(node, plan.items_solved, node_state)
            plan.node_count[node] = plan.node_count.get(node, 0) + 1
            plan.node_states[node] = node_state
            if res is None and node_state is None:  #let's not continue to explore this branch
                print("   Failed")
                yield None
            elif node_state is not None:
                #keep attempting this same plan
                if res is None:
                    print("   Requested to continue")
                else:
                    print("   Found solution and requested to continue")
                self.push_plan(plan)
            else:
                print("   Found solution")
            #look through results, create new possible plans for each output
            if res is not None:
                new_plan = plan.copy()
                output_items = [self.node_outputs[node]]
                res = [res]
                assert hasattr(res, '__len__') and len(res) == len(output_items), \
                    "Node {} returned {} results, but expected {}".format(node, len(res), len(output_items))
                #assume this is a tuple of outputs
                for i, o in enumerate(output_items):
                    new_plan.items_solved[o.name] = res[i]
                self.push_plan(new_plan)
            yield None

    def score_plan(self, plan : _PartialPlan) -> float:
        """Scores a plan, with lower numbers being preferred.  The default
        heuristic counts how many items aren't set in the plan."""
        score = float(len(self.items) - len(plan.items_solved))
        penalty = sum(self.node_continue_penalty(node)*count for node,count in plan.node_count.items())
        return score + penalty

    def select_node(self, plan : _PartialPlan) -> Optional[str]:
        """Selects a node to run that will solve some element of plan.

        Default just looks through items that are None in the plan,
        and returns the first node that can run and has an output 
        that matches that item.
        """
        for node in self.applicable_nodes(plan.items_solved):
            return node
        return None
    
    def node_continue_penalty(self, node : str) -> float:
        """Nodes that don't solve anything during plan() incur a penalty for
        the next round."""
        return 0.25

    def pop_plan(self) -> Tuple[float,_PartialPlan]:
        """Pops the best plan from the heap of plans."""
        node = heapq.heappop(self.plans)
        return node.priority, node.item
    
    def push_plan(self, plan: _PartialPlan, score = None):
        """Pushes a plan onto the heap of plans, with a score."""
        if score is None:
            score = self.score_plan(plan)
        assert isinstance(score, (int, float)), "Score must be a number"
        heapq.heappush(self.plans, _PQNode(plan,score))

    def stalled_plans(self) -> Dict[str,List[Dict[str,Any]]]:
        """Returns a dictionary of stalled plans, indexed by node name.
        
        A stalled plan is a plan that has not been completed or failed, but
        has not produced any new results in the last step."""
        res = {}
        for pqnode in self.plans:
            if pqnode.item.last_node is not None:
                if pqnode.item.last_node not in res:
                    res[pqnode.item.last_node] = []
                res[pqnode.item.last_node].append(pqnode.item.items_solved)
        return res


class IntrospectiveMultiStepPlanner(MultiStepPlanner):
    """A multi-step planner that uses least-commitment search
    but with good heuristics to prioritize low-likelihood and
    cheap steps in the plan."""
    def __init__(self, context: Any):
        super().__init__(context)
    
    def set_node_prior(self, node : str,
                       average_time : float,
                       success_likelihood : float,
                       feasibility_likelihood : float = 1.0,
                       strength : float = 10.0):
        """Sets the prior for all nodes in the planner.  This is used to
        bias the selection of nodes towards those that are more likely to
        succeed, and those that are cheaper to run.
        
        average_time is the average time it takes to run a node.
        success_likelihood is the likelihood of a node returning a valid result.
        feasibility_likelihood is the likelihood that the valid results of a node
            are feasible according to item validity checks.
        strength is a multiplier for the prior, which can be used to adjust the
            relative importance of the prior.
        """
        assert strength > 0, "Strength must be positive"
        assert success_likelihood >= 0 and success_likelihood <= 1, \
            "Success likelihood must be between 0 and 1"
        assert feasibility_likelihood >= 0 and feasibility_likelihood <= 1, \
            "Feasibility likelihood must be between 0 and 1"
        self.node_stats[node] = { 'total_time' : average_time*strength,
                                 'count':strength,
                                 'results':strength*success_likelihood*feasibility_likelihood,
                                 'failures':(1-success_likelihood)*strength,
                                 'successes':success_likelihood*strength }

    def set_item_prior(self, item : str,
                       average_time : float,
                       feasibility_likelihood : float,
                       strength : float = 10.0):
        """Sets the prior for all items in the planner.  This is used to
        bias the selection of items towards those that are more likely to
        be feasible.
        
        feasibility_likelihood is the likelihood that an item is feasible.
        strength is a multiplier for the prior, which can be used to adjust the
            relative importance of the prior.
        """
        assert strength > 0, "Strength must be positive"
        self.item_stats[item] = { 'total_time' : average_time*strength,
                                  'count':strength,
                                  'valid':strength*feasibility_likelihood }

    def score_plan(self, plan : _PartialPlan) -> float:
        """Scores a plan, with lower numbers being preferred.  The
        introspective heuristic examines the difficulty of solving for
        the remaining items in the plan."""
        remaining_cost_est = 0.0
        for name,item in self.items.items():
            if item.name not in plan.items_solved and item.constant is None:
                #item is not set in the plan, so we need to consider it
                if item.name not in self.item_stats or self.item_stats[item.name]['count'] == 0:
                    #no statistics for this item, assume default values
                    prior_strength = 1
                    if item.parent.properties.get('procedural', False):
                        avg_time = 0.001
                        valid_frac = 0.999
                    elif item.parent.properties.get('deterministic', False):
                        avg_time = 0.001
                        valid_frac = 0.5
                    else:
                        avg_time = 0.005
                        valid_frac = 0.1
                    if 'time_prior' in item.parent.properties:
                        avg_time = item.parent.properties['time_prior']
                    if 'valid_prior' in item.parent.properties:
                        valid_frac = item.parent.properties['valid_prior']
                    if 'prior_strength' in item.parent.properties:
                        avg_time *= item.parent.properties['prior_strength']
                        valid_frac *= item.parent.properties['prior_strength']
                        prior_strength = item.parent.properties['prior_strength']
                    stats = {'total_time': avg_time, 'count': prior_strength, 'valid': valid_frac}                    
                else:
                    stats = self.item_stats[item.name]
                avg_time = stats['total_time'] / stats['count']
                valid_frac = (stats['valid'] + 1e-5) / stats['count']  
                remaining_cost_est += avg_time / valid_frac
        return remaining_cost_est

    def select_node(self, plan : _PartialPlan) -> Optional[str]:
        """Selects a node by minimizing average_time / (1-run with result probability).
        """
        nodes_by_score = []
        for node in self.applicable_nodes(plan.items_solved):
            if node not in self.node_stats or self.node_stats[node]['count'] == 0:
                #no statistics for this node, assume default values
                prior_strength = 1
                props = self.nodes.node[node].properties
                if props.get('procedural', False):
                    avg_time = 0.001
                    result_probability = 0.999
                elif props.get('deterministic', False):
                    avg_time = 0.001
                    result_probability = 0.5
                else:
                    avg_time = 0.005
                    result_probability = 0.1
                if 'time_prior' in props:
                    avg_time = props['time_prior']
                if 'valid_prior' in props:
                    result_probability = props['valid_prior']
                if 'prior_strength' in props:
                    avg_time *= props['prior_strength']
                    result_probability *= props['prior_strength']
                    prior_strength = props['prior_strength']
                stats = {'total_time': avg_time, 'count': prior_strength, 'results': result_probability, 'failures': 1 - result_probability, 'successes': result_probability}
            else:
                stats = self.node_stats[node]
            avg_time = stats['total_time'] / stats['count'] 
            plan_count = plan.node_count.get(node,0)
            result_probability = stats['results'] / (plan_count + stats['count'] + 1e-5) 
            score = avg_time / (1.0 - result_probability)
            nodes_by_score.append((score,node))
        plan.applicable_nodes = [n[1] for n in nodes_by_score]
        if len(nodes_by_score) == 0:
            return None
        return min(nodes_by_score, key=lambda x: x[0])[1]
    


class SequentialMultiStepPlanner(MultiStepPlannerBase):
    """
    A solver for a multi-step planning problem that operates with the
    naive method of sequential search.  This is not a great planner,
    but is provided for comparison purposes.
    """
    def __init__(self, context: Any):
        super().__init__(context)

    def plan(self, context: Any, *inputs) -> Generator:
        """Performs one step of the multi-step planner.  Returns a generator
        indicating the result of the step."""
        self.current_plan = copy.copy(self.initial_plan)
        while True:
            next_node = None
            for n in self.applicable_nodes(self.current_plan):
                next_node = n
                break
            if next_node is None:
                print("SequentialMultiStepPlanner: No applicable nodes")
                return

            print("SequentialMultiStepPlanner: Attempting node",next_node)
            node_state = None
            res,node_state = self.run_node(next_node, self.current_plan, node_state)
            iters = 1
            while res is None:
                if node_state is None:
                    print("  Result is FAILURE")        
                    return
                print("  Result is CONTINUE; no results are returned")
                res,node_state = self.run_node(next_node, self.current_plan, node_state)
                iters += 1

            print("  Result is SOLVED after {} iters".format(iters))
            outputs = [self.node_outputs[next_node]]
            assert len(res) == len(outputs), "Invalid number of items returned by node {}".format(n)
            for i, o in enumerate(outputs):
                self.current_plan[o.name] = res[i]
            #check whether the plan is complete
            if all(i.name in self.current_plan for i in self.targets):
                yield self.current_plan
                return
            yield None


def _signature(func : callable):
    from typing import get_type_hints
    sig = inspect.signature(func)
    annotations = get_type_hints(func)
    arg_items = []
    for p in sig.parameters.values():
        if p.kind == inspect.Parameter.VAR_POSITIONAL or p.kind == inspect.Parameter.VAR_KEYWORD:
            raise NotImplementedError("Signature cannot be extracted for positional or var keyword arguments")        
        arg_items.append(PlanItem(p.name, annotations.get(p.name,None)))
    ret_item = PlanItem(None, annotations.get('return',None))
    return tuple(arg_items), ret_item


class LambdaNode(NodeBase):
    """A node that just runs a single function.  The function should
    return None if it fails, or a result if it succeeds."""
    def __init__(self, func : Callable, include_context=False):
        super().__init__()
        self.properties['deterministic'] = True
        self.func = func
        self.include_context = include_context
    def __str__(self):
        if self.name is None:
            return self.func.__name__
        else:
            return self.name
    def signature(self) -> Tuple[Tuple[PlanItem], PlanItem]:
        args,ret = _signature(self.func)
        if self.include_context:
            return args[1:],ret
        else:
            return args,ret
    def plan(self, context : Any, *inputs) -> Generator:
        if self.include_context:
            res = self.func(context, *inputs)
        else:
            res = self.func(*inputs)
        if res is not None:
            yield res
        return
    

class LambdaProcedureNode(NodeBase):
    """A node that just runs a single function that always succeeds.  The
    function should return None if it fails, or a result if it succeeds."""
    def __init__(self, func : Callable, include_context=False):
        super().__init__()
        self.properties['deterministic'] = True
        self.properties['procedural'] = True
        self.func = func
        self.include_context = include_context
    def __str__(self):
        if self.name is None:
            return self.func.__name__
        else:
            return self.name
    def signature(self) -> Tuple[Tuple[PlanItem], PlanItem]:
        args,ret = _signature(self.func)
        if self.include_context:
            return args[1:],ret
        else:
            return args,ret
    def plan(self, context : Any, *inputs) -> Generator:
        if self.include_context:
            res = self.func(context, *inputs)
        else:
            res = self.func(*inputs)
        if res is None:
            raise ValueError('Function {} was marked as procedural but returned None'.format(self.func.__name__))
        yield res
        return


class LambdaGeneratorNode(NodeBase):
    """A node that repeatedly runs a single function
    without termination.  The function should return None
    if it fails, or a result if it succeeds."""
    def __init__(self, func : Callable, include_context=False):
        super().__init__()
        self.func = func
        self.include_context = include_context
        assert inspect.isgeneratorfunction(func), \
            "Function {} is not a generator function".format(func.__name__)
    def __str__(self):
        if self.name is None:
            return self.func.__name__
        else:
            return self.name
    def signature(self) -> Tuple[Tuple[PlanItem], PlanItem]:
        args,ret = _signature(self.func)
        if self.include_context:
            return args[1:],ret
        else:
            return args,ret
    def plan(self, context : Any, *inputs) -> Generator:
        if self.include_context:
            for item in self.func(context, *inputs):
                yield item
        else:
            for item in self.func(*inputs):
                yield item


class WithConfigNode(NodeBase):
    def __init__(self, base : NodeBase, objects : List[Any]):
        """A node that runs a base node with a set of objects configured
        to the given configurations before running the base node."""
        super().__init__()
        self.base = base
        self.properties = base.properties
        self.objects = objects
    def __str__(self):
        return str(self.base)
    def signature(self) -> Tuple[Tuple[PlanItem], PlanItem]:
        args, ret = self.base.signature()
        return tuple(PlanItem() for obj in self.objects) + args, ret
    def plan(self, context : Any, *inputs) -> Generator:
        """Runs the base planner node with the objects configured to the given
        configurations."""
        configs = inputs[:len(self.objects)]
        base_inputs = inputs[len(self.objects):]
        gen = self.base.plan(context, *base_inputs)
        while True:
            try:
                #set the configurations of the objects
                for obj,conf in zip(self.objects,configs):
                    if hasattr(obj, 'setConfig'):
                        obj.setConfig(conf)
                    elif hasattr(obj, 'setTransform'):
                        obj.setTransform(*conf)
                    elif hasattr(obj, 'setCurrentTransform'):
                        obj.setCurrentTransform(*conf)
                    else:
                        raise ValueError("Object {} does not have setConfig, setTransform, or setCurrentTransform methods".format(obj))
                #run the base node
                yield next(gen)
            except StopIteration:
                return

class IteratedNode(NodeBase):
    """A helper node that runs a base planner node for a number of
    iterations for each next() call."""
    def __init__(self, base : NodeBase, num_iterations : int = 1):
        if base.properties.get('procedural', False):
            raise ValueError("IteratedNode cannot be used with procedural nodes")
        super().__init__()
        self.base = base
        self.properties = base.properties
        self.num_iterations = num_iterations
        
    def signature(self) -> Tuple[Tuple[PlanItem], PlanItem]:
        return self.base.signature()

    def plan(self, context : Any, *inputs) -> Generator:
        """Runs the base planner node for a number of iterations."""
        gen = self.base.plan(context, *inputs)
        while True:
            for i in range(self.num_iterations):
                try:
                    res = next(gen)
                except StopIteration:
                    return
                if res is not None:
                    yield res
            yield None


class OptionsNode(NodeBase):
    """A node that runs a set of node options, returning the first one that
    succeeds.
    
    The options are given as a list of NodeBase objects.  The node will run each
    option in order, and return the first one that returns ResultEnum.COMPLETE.
    If no option succeeds, it returns ResultEnum.FAILURE.

    The iters list contains a number of iterations to run each option.  You can
    set an iteration number to None to indicate that the option should run
    forever.
    """
    def __init__(self, options : List[NodeBase], iters : List[Optional[int]] = None):
        super().__init__()
        self.properties['deterministic'] = all(o.properties.get('deterministic', False) for o in options)
        self.options = options
        if iters is None:
            iters = [1] * len(options)
        assert len(options) == len(iters), "Options and iterations must have the same length"
        self.iters = iters
        #check that all options have the same signature
        sig = None
        for option in options:
            try:
                option_sig = option.signature()
            except NotImplementedError:
                pass
            if sig is None:
                sig = option_sig
            else:
                in1 = sig[0]
                in2 = option_sig[0]
                out1 = sig[1]
                out2 = option_sig[1]
                if len(in1) != len(in2):
                    raise ValueError("Options have different input arguments: {} vs {}".format(len(in1), len(in2)))
                for i,(a1,a2) in enumerate(zip(in1, in2)):
                    if a1.value_type is not None and a2.value_type is not None and a1.value_type is not a2.value_type:
                        raise ValueError("Options argument {} have different input types: {} vs {}".format(i, a1.value_type, a2.value_type))
                if out1.value_type is not None and out2.value_type is not None and out1.value_type is not out2.value_type:
                    raise ValueError("Options have different output types: {} vs {}".format(out1.value_type, out2.value_type))
        self._signature = sig

    def signature(self) -> Tuple[Tuple[PlanItem], PlanItem]:
        if self._signature is not None:
            return self._signature
        raise NotImplementedError()

    def __str__(self):
        return "OptionsNode({})".format(", ".join(str(o) for o in self.options))

    def plan(self, context : Any, *inputs) -> Generator:
        for i,option in enumerate(self.options):
            iters = 0
            for res in option.plan(context, *inputs):
                yield res
                iters += 1
                if iters[i] is not None and iters >= self.iters[i]:
                    break
        return


class OptimizingNode(NodeBase):
    """A node that only produces better results than the previous ones,
    as measured by an objective function.
    
    Lower values are considered better.
    """
    def __init__(self, base : NodeBase, objective : Callable[[Any], float]):
        super().__init__()
        if base.properties.get('deterministic', False):
            raise ValueError("OptimizingNode cannot be used with deterministic nodes")
        self.properties = base.properties
        self.base = base
        self.objective = objective

    def __str__(self):
        return "OptimizingNode({})".format(self.base)

    def signature(self) -> Tuple[Tuple[PlanItem], PlanItem]:
        return self.base.signature()
    
    def plan(self, context : Any, *inputs) -> Generator:
        best_value = float('inf')  #initial value is infinity
        for res in self.base.plan(context, *inputs):
            if res is None:
                yield None
            else:
                value = self.objective(res)
                if value < best_value:
                    best_value = value
                    yield res
                else:
                    yield None


class IKSolverNode(NodeBase):
    """An IK solver node that solves an IK problem with a given number of
    restarts. Call signature is (IKObjective) -> Config.

    If numRestarts is None, it will run forever.  If generate = True, it will
    continue to generate solutions, otherwise it will return the first
    solution it finds.

    Assumes context has a member `robot` of type RobotModel.  Argument is the
    IKObjective to solve for.  Returns the robot's Config if the IK solver
    finds a solution.
    """
    def __init__(self, seedConfig = None, numRestarts=None, generate=True):
        super().__init__()
        self.seedConfig = seedConfig
        self.numRestarts = numRestarts
        self.generate = generate
        if numRestarts == 0:
            self.properties['deterministic'] = True
            self.properties['procedural'] = True
    
    def signature(self) -> Tuple[Tuple[PlanItem], PlanItem]:
        """Returns the signature of the inputs and outputs of this node."""
        return ( (PlanItem("ik_goal", IKObjective),), PlanItem("config", Config) )

    def plan(self, context : Any, ik_goal : IKObjective) -> Generator:
        """Runs the IK solver on the given objective.  If it finds a solution,
        it yields it.  If it fails, it yields None."""
        count = 0
        solver = ik.solver(ik_goal)
        while True:
            count += 1
            if self.numRestarts is not None and count > self.numRestarts:
                return
            if count == 0 and self.seedConfig is not None:
                context.robot.setConfig(self.seedConfig)
            else:
                solver.sampleInitial()
            if solver.solve():
                yield context.robot.config
                if not self.generate:
                    return
            else:
                yield None


class CartesianMotionNode(NodeBase):
    """Translates/rotates an end effector via a cartesian space motion
    and returns a path. Call signature is (Config,RigidTransform) -> Configs.

    `type` can be "absolute", "extrinsic", or "intrinsic".  In "absolute" mode,
    the transform is interpreted as the absolute position of the end effector.
    In "extrinsic" mode, the transform is interpreted as a change in the
    world coordinates of the end effector.  In "intrinsic" mode, the transform
    is interpreted as a change in the end effector's local coordinates.
    
    Assumes context has a member `robot` of type RobotModel.
    """
    def __init__(self, end_effector_link : Union[int,str], type : str= 'extrinsic'):
        super().__init__()
        self.end_effector_link = end_effector_link
        self.transform_type = type
        self.properties['deterministic'] = True

    def signature(self) -> Tuple[Tuple[PlanItem], PlanItem]:
        """Returns the signature of the inputs and outputs of this node."""
        return ( (PlanItem("start_config", Config), PlanItem("goal_xform", RigidTransform)), 
                 PlanItem("path", Configs) )

    def plan(self, context, startConfig : Config, goalXform : RigidTransform) -> Generator:
        end_effector_link = context.robot.link(self.end_effector_link)
        context.robot.setConfig(startConfig)
        Tstart = end_effector_link.getTransform()
        if self.transform_type == 'absolute':
            Tgoal = goalXform
        elif self.transform_type == 'extrinsic':
            Tgoal = se3.mul(goalXform, Tstart)
        elif self.transform_type == 'intrinsic':
            Tgoal = se3.mul(Tstart, goalXform)
        else:
            raise ValueError("Unknown cartesian motion type {}".format(self.transform_type))
        path = cartesian_trajectory.cartesian_move_to(end_effector_link, Tgoal)
        if path is not None:
            yield path.milestones
        return


class CartesianOffsetNode(NodeBase):
    """Translates/rotates an end effector in cartesian space and returns the end
    configuration. Call signature is (Config,RigidTransform) -> Config.

    `type` can be "absolute", "extrinsic", or "intrinsic".  In "absolute" mode,
    the transform is interpreted as the absolute position of the end effector.
    In "extrinsic" mode, the transform is interpreted as a change in the
    world coordinates of the end effector.  In "intrinsic" mode, the transform
    is interpreted as a change in the end effector's local coordinates.
    
    Assumes context has a member `robot` of type RobotModel.

    Note: a Cartesian motion can be accomplished via a single CartesianMotionNode
    or a combination of a CartesianOffsetNode and a CartesianInterpolateNode. 
    The results will be slightly different, as the CartesianMotionNode will
    incrementally move toward the goal transform, while the combined approach
    will first solve for a goal configuration and the interpolate between the
    configurations.
    """
    def __init__(self, end_effector_link : Union[int,str], type : str= 'extrinsic'):
        super().__init__()
        self.end_effector_link = end_effector_link
        self.transform_type = type
        self.properties['deterministic'] = True

    def signature(self) -> Tuple[Tuple[PlanItem], PlanItem]:
        """Returns the signature of the inputs and outputs of this node."""
        return ( (PlanItem("start_config", Config), PlanItem("goal_xform", RigidTransform)), 
                 PlanItem("end_config", Config) )

    def plan(self, context, startConfig : Config, goalXform : RigidTransform) -> Generator:
        end_effector_link = context.robot.link(self.end_effector_link)
        context.robot.setConfig(startConfig)
        Tstart = end_effector_link.getTransform()
        if self.transform_type == 'absolute':
            Tgoal = goalXform
        elif self.transform_type == 'extrinsic':
            Tgoal = se3.mul(goalXform, Tstart)
        elif self.transform_type == 'intrinsic':
            Tgoal = se3.mul(Tstart, goalXform)
        else:
            raise ValueError("Unknown cartesian motion type {}".format(self.transform_type))
        obj = ik.objective(end_effector_link, R=Tgoal[0], t=Tgoal[1])
        if ik.solve(obj):
            yield context.robot.config
        else:
            # print("CartesianOffsetNode: Failed to solve for end effector transform {}".format(Tgoal))
            # print("Residual",ik.residual(obj))
            # from klampt import vis
            # vis.debug(Tgoal, Tstart, world=context.world)
            pass
        return


class CartesianInterpolateNode(NodeBase):
    """Interpolates an end effector between two configurations via a cartesian
    space motion and returns a path. Call signature is (Config,Config) ->
    Configs.
 
    Assumes context has a member `robot` of type RobotModel.
    """
    def __init__(self, end_effector_link : Union[int,str]):
        super().__init__()
        self.end_effector_link = end_effector_link
        self.properties['deterministic'] = True

    def signature(self) -> Tuple[Tuple[PlanItem], PlanItem]:
        """Returns the signature of the inputs and outputs of this node."""
        return ( (PlanItem("start_config", Config), PlanItem("goal_config", Config)), 
                 PlanItem("path", Configs) )

    def plan(self, context, startConfig : Config, goalConfig : Config) -> Generator:
        from ..model import config
        end_effector_link = context.robot.link(self.end_effector_link)
        context.robot.setConfig(startConfig)
        Tstart = end_effector_link.getTransform()
        context.robot.setConfig(goalConfig)
        Tgoal = end_effector_link.getTransform()
        path = cartesian_trajectory.cartesian_interpolate_bisect(context.robot, config.get_config(Tstart), config.get_config(Tgoal),
                                                                 end_effector_link, startConfig, goalConfig)
        if path is not None:
            yield path.milestones
        return



class KinematicMotionPlannerNode(NodeBase):
    """Runs a motion planner between two endpoint configurations.
    Call signature is (Config,Config) -> Configs (i.e., a feasible path
    of milestones connecting them).
    
    A motion planner is stored for each pair of endpoints given.
    When repeatedly called with the same endpoints, it will continue
    to run a motion planner until it finds a solution (for a feasible
    planner) or exhausts the number of optimization iterations.
    It will then restart the planner with the same endpoints so it
    can generate more solutions.
    """
    def __init__(self, space : cspace.CSpace, planner_settings : dict, increment : int = 10):
        super().__init__()
        self.space = space
        self.planner_settings = planner_settings
        self.increment = increment
    
    def signature(self) -> Tuple[Tuple[PlanItem], PlanItem]:
        """Returns the signature of the inputs and outputs of this node."""
        return ( (PlanItem("start_config", Config), PlanItem("goal_config", Config)), 
                 PlanItem("path", Configs) )

    def plan(self, context : Any, startConfig : Config, goalConfig : Config) -> Generator:
        """Runs the motion planner on the given start and goal configurations.
        If it finds a solution, it returns it.  If it fails, it returns None."""
        if hasattr(self.space,'lift'):  # if this is an embedded space, ensure that the non-moving DOFs are set to startConfig
            self.space.xinit = startConfig
        # debugging
        # from klampt import vis
        # vis.debug(startConfig,{'color':(0,1,0,0.5)},goalConfig,{'color':(1,0,0,0.5)},world=context.world)
        if hasattr(self.space,'lift'):  #the planning takes place in a space of lower dimension than #links
            planner = robotplanning.EmbeddedKinematicPlanner(self.space, startConfig, **self.planner_settings)
        else:
            planner = cspace.KinematicPlanner(self.space, **self.planner_settings)
        try:
            planner.setEndpoints(startConfig, goalConfig)
        except RuntimeError:
            #one of the endpoints is infeasible, print it out
            if self.space.cspace==None:
                self.space.setup()
            q0_raw = startConfig if not hasattr(planner.space,'project') else planner.space.project(startConfig)
            target_raw = goalConfig if not hasattr(planner.space,'project') else planner.space.project(goalConfig)
            sfailures = self.space.cspace.feasibilityFailures(q0_raw)
            gfailures = self.space.cspace.feasibilityFailures(target_raw)
            if sfailures:
                print("Start configuration fails {}".format(sfailures))
            if gfailures:
                print("Goal configuration fails {}".format(gfailures))
            return

        while True:
            planner.planMore(self.increment)
            path = planner.getPath()
            if path is not None and len(path) > 0:
                yield path
                if not planner.isOptimizing():
                    return
            else:
                yield None


class PathToTrajectoryNode(NodeBase):
    """Converts a path of Configs to a RobotTrajectory.
    Signature is (Configs) -> RobotTrajectory.
    
    Assumes context has a member `robot` of type RobotModel.
    """
    def __init__(self, smoothing = None):
        super().__init__()
        self.smoothing = smoothing
        self.properties['deterministic'] = True
        self.properties['procedural'] = True
    
    def signature(self) -> Tuple[Tuple[PlanItem], PlanItem]:
        """Returns the signature of the inputs and outputs of this node."""
        return ( (PlanItem("path", Configs),), PlanItem("trajectory", trajectory.RobotTrajectory) )

    def plan(self, context : Any, path : Configs) -> Generator:
        """Converts the path to a RobotTrajectory."""
        if len(path) == 0:
            return self.failure("Path is empty")
        if hasattr(context, 'robot'):
            path = trajectory.RobotTrajectory(context.robot, times=None, milestones=path)
        traj = trajectory.path_to_trajectory(path, smoothing=self.smoothing)
        if traj is not None:
            yield traj
        return


class ConcatPathsNode(NodeBase):
    """Concatenates two paths of Configs into a single path.
    Signature is (Configs,Configs) -> Configs.
    
    Assumes context has a member `robot` of type RobotModel.
    """
    def __init__(self):
        super().__init__()
        self.properties['deterministic'] = True
        self.properties['procedural'] = True

    def signature(self) -> Tuple[Tuple[PlanItem], PlanItem]:
        """Returns the signature of the inputs and outputs of this node."""
        return ( (PlanItem("path1", Configs), PlanItem("path2", Configs)), 
                 PlanItem("concatenated_path", Configs) )

    def plan(self, context : Any, path1 : Configs, path2 : Configs) -> Generator:
        """Concatenates the two paths."""
        if len(path1) == 0 or len(path2) == 0:
            return self.failure("One of the paths is empty")
        if path1[-1] != path2[0]:
            #ensure that the end of path1 matches the start of path2
            raise ValueError("End of path1 does not match start of path2")
        concatenated_path = path1[:-1] + path2
        yield concatenated_path
        return


class ConcatTrajectoriesNode(NodeBase):
    """Concatenates two RobotTrajectories into a single trajectory.
    Signature is (RobotTrajectory,RobotTrajectory) -> RobotTrajectory.
    """
    def __init__(self):
        super().__init__()  
        self.properties['deterministic'] = True
        self.properties['procedural'] = True

    def signature(self) -> Tuple[Tuple[PlanItem], PlanItem]:
        """Returns the signature of the inputs and outputs of this node."""
        return ( (PlanItem("trajectory1", trajectory.RobotTrajectory), 
                   PlanItem("trajectory2", trajectory.RobotTrajectory)), 
                 PlanItem("concatenated_trajectory", trajectory.RobotTrajectory) )

    def plan(self, context : Any, traj1 : trajectory.RobotTrajectory, traj2 : trajectory.RobotTrajectory) -> Generator:
        """Concatenates the two trajectories."""
        if len(traj1.milestones) == 0 or len(traj2.milestones) == 0:
            raise ValueError("One of the trajectories is empty")
        concatenated_traj = traj1.concat(traj2,relative=True)
        yield concatenated_traj
        return
