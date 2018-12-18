from ..robotsim import *
from . import simlog
import weakref
import collections

class SensorEmulator:
    """A generic sensor emulator.  Translates from the physics simulation -> inputs to a Python controller.

    The Python controller is assumed to have the structure of BaseController, where it is given as input
    a dictionary of named items reflecting the most up-to-date readings on each control time-step.
    """
    def __init__(self):
        pass
    def update(self):
        """Returns a dictionary mapping named sensors to their outputs."""
        return {}
    def drawGL(self):
        """Optional: for debugging"""
        return

class DefaultSensorEmulator(SensorEmulator):
    """A sensor emulator that by default provides the robot's commanded position, velocity, 
    and other sensors defined in the robot or world XML file.
    """
    def __init__(self,sim,controller):
        self.sim = sim
        self.controller = controller
    def update(self):
        measurements = {}
        mode = self.controller.getControlType()
        if mode == "PID":
            measurements['qcmd'] = self.controller.getCommandedConfig()
            measurements['dqcmd'] = self.controller.getCommandedVelocity()
        k = 0
        while True:
            s = self.controller.sensor(k)
            if s.type()=='':
                break;
            measurements[s.name()] = s.getMeasurements()
            k+=1
        return measurements
    def drawGL(self):
        if self.controller.getControlType() == "PID":
            q = self.controller.getCommandedConfig()
            r = self.controller.model()
            r.setConfig(q)
            colors = []
            for j in range(r.numLinks()):
                colors.append(r.link(j).appearance().getColor())
                r.link(j).appearance().setColor(0,1,0,0.5)
            r.drawGL()
            for j in range(r.numLinks()):
                r.link(j).appearance().setColor(*colors[j])

class ActuatorEmulator:
    """A generic actuator emulator.  Translates outputs from the Python controller -> the physics simulation.
    A variety of non-traditional actuators can be simulated here.

    The Python controller is assumed to have the structure of BaseController, where outputs a dictionary
    of commands every control time step.  The emulator will read these with the process() methods
    """
    def __init__(self):
        pass
    def process(self,commands,dt):
        """Processes the dictionary of commands, which are outputted by the controller.
        This may involve applying commands to the low-level motor emulator, 
        or applying forces to the simulator.

        Arguments:
        - commands: a dictionary of commands produced by the controller
        - dt: the control time step (not the underlying simulation time step)

        To play nicely with other actuators in a nested steup, once a command is processed, the class should
        remove it from the commands dictionary.
        """
        pass
    def substep(self,dt):
        """This is called every simulation substep, which occurs at a higher rate than
        process() is called.  dt is the simulation substep.
        """
        pass
    def drawGL(self):
        """Optional: for debugging"""
        return

class DefaultActuatorEmulator(ActuatorEmulator):
    """This default emulator can take the commands
    - torquecmd: torque comand
    - qcmd: position command
    - dqcmd: velocity command
    - tcmd: time for a dqcmd
    And will also pass any remaining commands to the low-level C controller.

    It can also simulate forces, etc. at a higher rate than the control loop rate.
    """
    def __init__(self,sim,controller):
        self.sim = sim
        self.controller = controller
    def process(self,commands,dt):
        """Commands: a dictionary of values outputted from the controller module,
        or None if no command was issued. """
        if commands == None: return
        c = self.controller
        defaultVals = set(['torquecmd','qcmd','dqcmd','tcmd'])
        if 'qcmd' in commands:
            dqcmd = commands['dqcmd'] if 'dqcmd' in commands else [0.0]*len(commands['qcmd'])
            if 'torquecmd' in commands:
                c.setPIDCommand(commands['qcmd'],dqcmd,commands['torquecmd'])
            else:
                c.setPIDCommand(commands['qcmd'],dqcmd)
        elif 'dqcmd' in commands:
            assert 'tcmd' in commands
            c.setVelocityCommand(commands['dqcmd'],commands['tcmd'])
        elif 'torquecmd' in commands:
            c.setTorque(commands['torquecmd'])
        for (k,v) in commands.items():
            if k not in defaultVals:
                print("Sending command",k,v,"to low level controller")
                c.sendCommand(k,v)
        return


class SimpleSimulator (Simulator):
    """A convenience class that enables easy logging, definition of simulation hooks, emulators
    of sensors / actuators, and definition of robot controllers.

    Note that for greatest compatibility you should NOT manually apply forces to the simulation
    except for inside of hooks and emulators.  This is because several simulation sub-steps will be
    taken, and you will have no control over the forces applied except for the first time step.
    """
    def __init__(self,world):
        """Arguments:
        - world: a RobotWorld instance.
        """
        Simulator.__init__(self,world)
        #these are functions automatically called at each time step
        self.robotControllers = [None]*world.numRobots()
        self.sensorEmulators = [[DefaultSensorEmulator(weakref.proxy(self),self.controller(i))] for i in range(world.numRobots())]
        self.actuatorEmulators = [[DefaultActuatorEmulator(weakref.proxy(self),self.controller(i))] for i in range(world.numRobots())]
        self.hooks = []
        self.hook_args = []
        #the rate of applying simulation substeps.  Hooks and actuator emulators are
        #called at this rate.  Note: this should be set at least as large as the simulation time step
        self.substep_dt = 0.001
        self.worst_status = Simulator.STATUS_NORMAL

        #turn this on to save log to disk
        self.logging = False
        self.logger = None
        self.log_state_fn="simulation_state.csv"
        self.log_contact_fn="simulation_contact.csv"

        #save state so controllers don't rely on world state
        self.robotStates = []
        self.objectStates = []

    def getStatus(self):
        return self.worst_status

    def getStatusString(self, status = -1):
        if status > 0:
            return Simulator.getStatusString(self, status)
        return Simulator.getStatusString(self, self.getStatus())

    def beginLogging(self):
        self.logging = True
        self.logger = simlog.SimLogger(weakref.proxy(self),self.log_state_fn,self.log_contact_fn)
    def endLogging(self):
        self.logging = False
        self.logger = None
    def pauseLogging(self,paused=True):
        self.logging=not paused
    def toggleLogging(self):
        if self.logging:
            self.pauseLogging()
        else:
            if self.logger==None:
                self.beginLogging()
            else:
                self.pauseLogging(False)

    def setController(self,robot,function):
        """Sets a robot's controller function.

        Arguments:
        - robot: either an index, string, or RobotModel.
        - function: either be 1) a one-argument function that takes the
          robot's SimRobotController instance, or 2) an instance of a
          BaseController class (see Python/control/controller.py)
        """
        if isinstance(robot,int):
            index = robot
        elif isinstance(robot,str):
            index = self.world.robot(robot).index
        elif isinstance(robot,RobotModel):
            index = robot.index
        else:
            raise ValueError("Invalid robot specified")
        if not isinstance(function, collections.Callable):
            assert hasattr(function,'output_and_advance'),"setController takes either a 1-argument function or a BaseController instance"
        self.robotControllers += [None]*(self.world.numRobots()-len(self.robotControllers))
        self.robotControllers[index] = function

    def addEmulator(self,robot,e):
        """Adds an emulator to the given robot.  e must be of SensorEmulator or ActuatorEmulator type.
        """
        if isinstance(e,SensorEmulator):
            self.sensorEmulators[robot].append(e)
        elif isinstance(e,ActuatorEmulator):
            self.actuatorEmulators[robot] = [e] + self.actuatorEmulators[robot]
        else:
            raise ValueError("Invalid emulator type")

    def addHook(self,objects,function):
        """For the world object or objects 'objects', applies a hook that gets called every
        simulation loop.  The objects may be certain identifiers, WorldModel items or SimBodies. 
        - Accepted names are: 'time', or any items in the world
        - If they are individual bodies, the corresponding SimBody objects are passed to function. 
        - If they are RobotModel's, the corresponding SimRobotController objects are passed to function.
        - Otherwise they are passed directly to function.
        """
        if not hasattr(objects,'__iter__'):
            objects = [objects]
        args = []
        for o in objects:
            if isinstance(o,(RobotModelLink,RigidObjectModel,TerrainModel)):
                args.append(self.body(o))
            elif isinstance(o,RobotModel):
                args.append(self.controller(o))
            elif isinstance(o,str):
                if o == 'time':
                    args.append(o)
                elif self.world.robot(o).world >= 0:
                    args.append(self.world.robot(o))
                elif self.world.terrain(o).world >= 0:
                    args.append(self.world.terrain(o))
                elif self.world.rigidObject(o).world >= 0:
                    args.append(self.world.rigidObject(o))
                else:
                    raise ValueError("String value "+o+" is unknown")
            else:
                args.append(o)
        self.hooks.append(function)
        self.hook_args.append(args)

    def drawGL(self):
        self.updateWorld()
        self.world.drawGL()
        self.drawEmulatorsGL()
        self.drawControllersGL()

    def drawEmulatorsGL(self):
        #draw emulators
        for elist in self.sensorEmulators:
            for e in elist:
                e.drawGL()
        for elist in self.actuatorEmulators:
            for e in elist:
                e.drawGL()

    def drawControllersGL(self):
        #draw controllers
        for i in range(self.world.numRobots()):
            if self.robotControllers[i] == None: 
                continue
            if not hasattr(self.robotControllers[i],'drawGL'):
                continue
            self.robotControllers[i].drawGL()

    def simulate(self,dt):
        """Runs the simulation.  Note that this should be called at the
        rate of the controller.  Simulation hooks and emulator substeps
        will be called at the rate of substep_dt.

        Arguments:
        - dt: control timestep
        """
        #Handle logging
        if self.logger: self.logger.saveStep()

        self.worst_status = Simulator.STATUS_NORMAL

        #Advance controller, emulators
        #restore state from previous call -- this is done so that simulation data doesn't leak into controllers
        for i,(q,dq) in enumerate(self.robotStates):
            self.world.robot(i).setConfig(q)
            self.world.robot(i).setVelocity(dq)
        for i,T in enumerate(self.objectStates):
            self.world.rigidObject(i).setTransform(*T)
        #advance controller
        self.control_loop(dt)
        #save post-controller state
        self.robotStates = [(self.world.robot(i).getConfig(),self.world.robot(i).getVelocity()) for i in range(self.world.numRobots())]
        self.objectStates = [self.world.rigidObject(i).getTransform() for i in range(self.world.numRigidObjects())]


        #advance hooks and the physics simulation at the high rate
        assert self.substep_dt > 0
        t = 0
        while True:
            substep = min(self.substep_dt,dt-t)
            for i in range(self.world.numRobots()):
                for e in self.actuatorEmulators[i]:
                    e.substep(substep)
            for (hook,args) in zip(self.hooks,self.hook_args):
                resolvedArgs = []
                for a in args:
                    if isinstance(a,str):
                        if a=='time':
                            resolvedArgs.append(sim.getTime())
                        elif a=='dt':
                            resolvedArgs.append(substep)
                        else:
                            raise ValueError("Invalid unresolved argument",a)
                    else:
                        resolvedArgs.append(a)
                try:
                    hook(*resolvedArgs)
                except Exception as e:
                    import traceback
                    print("Hook encountered error with arguments",resolvedArgs)
                    traceback.print_exc()
                    raise
            #Finally advance the physics simulation
            Simulator.simulate(self,substep)
            s = Simulator.getStatus(self)
            if s > self.worst_status:
                self.worst_status = s
            t += self.substep_dt
            if t >= dt:
                break
        #done
        return

    def control_loop(self,dt):
        for i in range(self.world.numRobots()):
            c = self.robotControllers[i]
            if isinstance(c, collections.Callable):
                c(self.controller(i))
            else:
                #build measurement dict
                measurements = {'t':self.getTime(),'dt':dt}
                for e in self.sensorEmulators[i]:
                    measurements.update(e.update())
                
                """
                #debug: print measurements
                for (k,v) in measurements.iteritems():
                    print k,":",
                    if hasattr(v,'__iter__'):
                        print ' '.join("%.2f"%(vi,) for vi in v)
                    else:
                        print v
                """
                if c:
                    #assume it's a BaseController instance
                    #compute controller output, advance controller
                    output = c.output_and_advance(**measurements)
                else:
                    output = None

                #process output => sim using actuator emulators
                for e in self.actuatorEmulators[i]:
                    e.process(output,dt)
