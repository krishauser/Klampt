from ..robotsim import *
from ..model import map
from simulation import SimpleSimulator
import time


def getWorldSimState(world):
    """Returns a dict containing a copy of all variables that are
    simulated in the world.  Can be used with setWorldSimState to save/
    restore state.

    NOTE: this does not perfectly save the state of a Simulator!  To do that,
    you must use the Simulator().getState()/saveState() methods.
    """
    res = dict()
    for i in range(world.numRigidObjects()):
        res['rigidObjects['+str(i)+'].transform']=world.rigidObject(i).getTransform()
    for i in range(world.numRobots()):
        res['robots['+str(i)+'].config']=world.robot(i).getConfig()
        res['robots['+str(i)+'].velocity']=world.robot(i).getVelocity()
    return res
    
def setWorldSimState(world,state):
    """Sets the world state to the prior saved state (a dict from
    getWorldSimState())
    
    NOTE: this does not perfectly save simulation state!  To do that,
    you must use the Simulator().getState()/saveState() methods.
    """
    for (k,v) in state.iteritems():
        map.set_item(world,k,v)
    return


def doSim(world,duration,initialCondition,
          returnItems=None,trace=False,
          simDt=0.01,simInit=None,simStep=None,simTerm=None):
    """Runs a simulation for a given initial condition of a world.

    Arguments:
    - world: the world
    - duration: the maximum duration of simulation, in seconds
    - initialCondition: a dictionary mapping named items to values.
      Each named item is specified by a path as used by the map module, e.g.
      'robot[0].config[4]'.  See the documentation for map.get_item()/
      map.set_item() for details.

      Special items include 'args' which is a tuple provided to each simInit,
      simStep, and simTerm call.
    - returnItems (optional): a list of named items to return in the final
      state of the simulation.  By default returns everything that is
      variable in the simulator (simulation time, robot and rigid object
      configuration / velocity, robot commands, robot sensors).
    - trace (optional, default False): if True, returns the entire trace of
      the items specified in returnItems rather than just the final state.
    - simDt (optional, default 0.01): the outer simulation loop (usually
      corresponds to the control rate).
    - simInit (optional): a function f(sim) called on the simulator after its
      initial conditions are set but before simulating. You may configure the
      simulator with this function.
    - simStep (optional): a function f(sim) that is called on every outer
      simulation loop (usually a controller function).
    - simTerm (optional): a function f(sim) that returns True if the simulation
      should terminate early.  Called on every outer simulation loop.

    Return value is the final state of each returned item upon termination. 
    This takes the form of a dictionary mapping named items (specified by
    the returnItems argument) to their values. 
    Additional returned items are:
    - 'status', which gives the status string of the simulation
    - 'time', which gives the time of the simulation, in s
    - 'wall_clock_time', which gives the time elapsed while computing the simulation, in s
    """
    if returnItems == None:
        #set up default return items
        returnItems = []
        for i in range(world.numRigidObjects()):
            returnItems.append('rigidObjects['+str(i)+'].transform')
            returnItems.append('rigidObjects['+str(i)+'].velocity')
        for i in range(world.numRobots()):
            returnItems.append('time')
            returnItems.append('controllers['+str(i)+'].commandedConfig')
            returnItems.append('controllers['+str(i)+'].commandedVelocity')
            returnItems.append('controllers['+str(i)+'].sensedConfig')
            returnItems.append('controllers['+str(i)+'].sensedVelocity')
            returnItems.append('controllers['+str(i)+'].sensors')
            returnItems.append('robots['+str(i)+'].actualConfig')
            returnItems.append('robots['+str(i)+'].actualVelocity')
            returnItems.append('robots['+str(i)+'].actualTorques')
    initCond = getWorldSimState(world)
    args = ()
    for k,v in initialCondition.iteritems():
        if k is not 'args':
            map.set_item(world,k,v)
        else:
            args = v
    sim = SimpleSimulator(world)
    if simInit: simInit(sim,*args)
    assert simDt > 0,"Time step must be positive"
    res = dict()
    if trace:
        for k in returnItems:
            res[k] = [map.get_item(sim,k)]
        res['status'] = [sim.getStatusString()]
    print "klampt.batch.doSim(): Running simulation for",duration,"s"
    t0 = time.time()
    t = 0
    worst_status = 0
    while t < duration:
        if simTerm and simTerm(sim,*args)==True:
            if not trace:
                for k in returnItems:
                    res[k] = map.get_item(sim,k)
                res['status']=sim.getStatusString(worst_status)
                res['time']=t
                res['wall_clock_time']=time.time()-t0
            #restore initial world state
            setWorldSimState(world,initCond)
            print "  Termination condition reached at",t,"s"
            print "  Computation time:",time.time()-t0
            return res
        if simStep: simStep(sim,*args)
        sim.simulate(simDt)
        worst_status = max(worst_status,sim.getStatus())
        if trace:
            for k in returnItems:
                res[k].append(map.get_item(sim,k))
            res['status'].append(sim.getStatusString())
            res['time']=t
            res['wall_clock_time']=time.time()-t0
        t += simDt
    if not trace:
        #just get the terminal stats
        for k in returnItems:
            res[k] = map.get_item(sim,k)
        res['status']=sim.getStatusString(worst_status)
        res['time']=t
        res['wall_clock_time']=time.time()-t0

    print "  Done."
    print "  Computation time:",time.time()-t0
    #restore initial world state
    setWorldSimState(world,initCond)
    return res

def batchSim(world,duration,initialConditions,returnItems,
          simDt=0.01,simInit=None,simStep=None,simTerm=None):
    """Given a world, a simulation duration, and a list of initial conditions,
    runs simulations for all initial conditions.

    Arguments:
        - everything but initialConditions is the same as in doSim()
        - initialConditions: either a dict mapping named items to lists
          of initial values, or a list of initial state dictionaries.
          In the former case, all entries must be of the same length.
    
    Return a list of return values from doSim().  See the doSim()
    documentation for more information on the arguments.
    """
    res = []
    if isinstance(initialConditions,dict):
        #assume it is a dict-of-lists type
        v0 = dict.itervalues().next()
        for (k,v) in initialConditions.iteritems():
            assert len(v)==len(v0),"initialConditions entries must all be of same length"
        print "klampt.batch.batchSim(): Running",len(v0),"simulations..."
        for i in xrange(len(v0)):
            initCond = dict((k,v[i]) for (k,v) in initialConditions.iteritems())
            try:
                simRes = doSim(world,duration,initCond,returnItems,trace=False,
                               simDt=simDt,simInit=simInit,simStep=simStep,simTerm=simTerm)
            except Exception:
                print "  Exception thrown on trial",i
                simRes = 'error'
            res.append(simRes)
    else:
        print "klampt.batch.batchSim(): Running",len(initialConditions),"simulations..."
        for i,initCond in enumerate(initialConditions):
            try:
                simRes = doSim(world,duration,initCond,returnItems,trace=False,
                               simDt=simDt,simInit=simInit,simStep=simStep,simTerm=simTerm)
            except Exception:
                print "  Exception thrown on trial",i
                simRes = 'error'
            res.append(simRes)
    return res

def monteCarloSim(world,duration,initialConditionSamplers,N,returnItems,
          simDt=0.01,simInit=None,simStep=None,simTerm=None):
    """Given a world, a simulation duration, and dict of sampling functions
    for world items, runs N monte-carlo simulations.

    Arguments:
        - world, duration, returnItems, simDt, simInit, simStep, simTerm:
            same as for doSim()
        - initialConditionSamplers: a dict mapping named world items to
          sampling functions that take no arguments (i.e., sample()).
        - N: the number of Monte Carlo samples
    
    The return value is a list of N pairs (initCond,returnVal)
    where initCond is the sampled initial condition and returnVal is the
    return value from doSim().
    """
    print "klampt.batch.monteCarloSim(): Running",N,"simulations..."
    res = []
    for sample in xrange(N):
        initCond = dict((k,v()) for k,v in initialConditionSamplers.iteritems())
        try:
            simRes = doSim(world,duration,initCond,returnItems,trace=False,
                           simDt=simDt,simInit=simInit,simStep=simStep,simTerm=simTerm)
        except Exception as e:
            print "  Exception thrown on trial",sample
            print "    what:",e
            import traceback
            traceback.print_exc()
            simRes = 'error'
        res.append((initCond,simRes))
    return res



def saveStateHeaderCSV(state,f):
    """Given a state dictionary, saves the header CSV format to the given
    output stream f"""
    vflat = [map.flatten(state[k]) for k in state]
    #write header
    itemNames = []
    for k,v in zip(state.keys(),vflat):
        if len(v)==1:
            itemNames.append(k)
        else:
            for i in range(len(v)):
                itemNames.append(k+'['+str(i)+']')
    f.write(','.join(items))
    f.write('\n')


def saveStateCSV(state,f):
    """Given a state dictionary, saves it to CSV format to the given
    output stream f"""
    saveStateHeaderCSV(state,f)
    f.write(','.join(str(v) for v in map.flatten(state)))
    f.write('\n')

def saveStatesCSV(states,f):
    """Given list of state dictionaries, saves them to CSV format to the
    given output stream f"""
    saveStateHeaderCSV(states[0],f)
    for state in states:
        f.write(','.join(str(v) for v in map.flatten(state)))
        f.write('\n')
    return

def saveStateTrajectoryCSV(stateTraj,f):
    """Given a state trajectory (dict mapping keys to lists), saves it
    to CSV format to the given output stream f."""
    state0 = dict((k,v[0]) for (k,v) in stateTraj.iteritems())
    state0['iter'] = 0
    saveStateHeaderCSV(state0,f)
    if len(stateTraj.items())==0:
        return
    length = len(stateTraj.values()[0])
    for i in xrange(length):
        state0['iter'] = i
        for k in stateTraj.iterkeys():
            state0[k] = stateTraj[k][i]
        f.write(','.join(str(v) for v in map.flatten(state0)))
        f.write('\n')
    return
