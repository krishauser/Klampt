"""Used for testing code that works with the Klamp't Robot Interface Layer on a
simualted robot.  Defines a variety of RobotInterfaceBase interfaces that work
with Klamp't simulations.

For each of the classes in this module, if you provide the simulator argument
then this will automatically update your simulation upon each beginStep() /
endStep() pair.  Otherwise, you will have to step the simulation manually.
"""

from .robotinterface import RobotInterfaceBase
from klampt.model.robotinfo import RobotInfo
from klampt.model import robotinfo
from klampt import RobotModel,Simulator,SimRobotController
import functools
import warnings

class _SimControlInterface(RobotInterfaceBase):
    def __init__(self,sim_controller,simulator=None,robotInfo=None):
        RobotInterfaceBase.__init__(self,name=self.__class__.__name__)
        assert isinstance(sim_controller,SimRobotController)
        self.sim_controller = sim_controller
        self.robot = sim_controller.model()
        if simulator is not None:
            assert isinstance(simulator,Simulator),"If you want to simulate, pass a klampt.Simulator object"
            self.simulator = simulator
        else:
            self.simulator = None
        self._status = 'disconnected'
        self.robotInfo = robotInfo
        if robotInfo is not None:
            assert isinstance(robotInfo,RobotInfo)
            if robotInfo.modelFile is not None:
                self.properties['klamptModelFile'] = robotInfo.modelFile
    
    def klamptModel(self):
        return self.robot

    def initialize(self):
        self._status = 'ok'
        return True

    def klamptModel(self):
        return self.robot

    @functools.lru_cache(maxsize=None)
    def parts(self):
        if self.robotInfo is None:
            return RobotInterfaceBase.parts(self)
        res = {None:list(range(self.numJoints()))}
        for k in self.robotInfo.parts:
            res[k] = self.robotInfo.partDriverIndices(k)
        return res

    def controlRate(self):
        return 1.0/self.sim_controller.getRate()

    def sensors(self):
        sensorNames = []
        index = 0
        while True:
            s = self.sim_controller.sensor(index)
            sname = s.name()
            if len(sname) > 0:
                sensorNames.append(sname)
            else:
                break
            index += 1
        return sensorNames

    def enabledSensors(self):
        return self.sensors()

    def hasSensor(self,sensor):
        return len(self.sim_controller.sensor(sensor).type()) > 0

    def enableSensor(self,sensor,enabled=True):
        if not enabled:
            raise NotImplementedError("Can't disable a simulation sensor")
        return True

    def sensorMeasurements(self,name):
        return self.sim_controller.sensor(name).getMeasurements()

    def endStep(self):
        if self.simulator is not None:
            self.simulator.simulate(self.sim_controller.getRate())
            if self.simulator.getStatus() >= Simulator.STATUS_UNSTABLE:
                self._status = self.simulator.getStatusString()

    def status(self,joint_idx=None):
        return self._status


class SimPositionControlInterface(_SimControlInterface):
    """Adapts a SimRobotController to the RobotInterfaceBase class in position
    control mode. 

    Only implements setPosition, sensedPosition, and commandedPosition; you
    should use :class:`RobotInterfaceCompleter` to fill in move-to control,
    cartesian control, velocity control, etc.
    """
    def __init__(self,sim_controller,simulator=None,robotInfo=None):
        _SimControlInterface.__init__(self,sim_controller,simulator,robotInfo)

    def setPosition(self,q):
        self.sim_controller.setPIDCommand(q,[0]*len(q))
        
    def sensedPosition(self):
        return self.configFromKlampt(self.sim_controller.getSensedConfig())
    
    def commandedPosition(self):
        return self.configFromKlampt(self.sim_controller.getCommandedConfig())


class SimVelocityControlInterface(_SimControlInterface):
    """Adapts a SimRobotController to the RobotInterfaceBase class in velocity
    control mode. 

    Only implements setVelocity, sensedPosition, and commandedPosition; you
    should use :class:`RobotInterfaceCompleter` to fill in move-to control,
    cartesian control, position control, etc.
    """
    def __init__(self,sim_controller,simulator=None,robotInfo=None):
        _SimControlInterface.__init__(self,sim_controller,simulator,robotInfo)

    def setVelocity(self,v,ttl=None):
        if ttl is None:
            ttl = 1.0
        self.sim_controller.setVelocity(self.velocityToKlampt(v),ttl)
        
    def sensedPosition(self):
        return self.configFromKlampt(self.sim_controller.getSensedConfig())
    
    def commandedPosition(self):
        return self.configFromKlampt(self.sim_controller.getCommandedConfig())


class SimMoveToControlInterface(_SimControlInterface):
    """Adapts a SimRobotController to the RobotInterfaceBase class in move-to
    control mode. 

    Only implements moveToPosition, sensedPosition, and commandedPosition; you
    should use :class:`RobotInterfaceCompleter` to fill in position control,
    cartesian control, velocity control, etc.
    """
    def __init__(self,sim_controller,simulator=None,robotInfo=None):
        _SimControlInterface.__init__(self,sim_controller,simulator,robotInfo)

    def moveToPosition(self,q,speed=1.0):
        assert speed == 1.0,"Can't accept non-max speed commands yet"
        self.sim_controller.setMilestone(self.configToKlampt(q))
        
    def sensedPosition(self):
        return self.configFromKlampt(self.sim_controller.getSensedConfig())
    
    def commandedPosition(self):
        return self.configFromKlampt(self.sim_controller.getCommandedConfig())

    def isMoving(self,joint_idx=None):
        return self.sim_controller.remainingTime() > 0


class SimFullControlInterface(_SimControlInterface):
    """Adapts a SimRobotController to the RobotInterfaceBase class, accepting
    position control, move to control, velocity control, and torque control
    modes. 

    You should use :class:`RobotInterfaceCompleter` to fill in move-to control,
    cartesian control, velocity control, etc.
    """
    def __init__(self,sim_controller,simulator=None,robotInfo=None):
        _SimControlInterface.__init__(self,sim_controller,simulator,robotInfo)

    def setPosition(self,q):
        self.sim_controller.setPIDCommand(q,[0]*len(q))

    def setVelocity(self,v,ttl=None):
        if ttl is None:
            ttl = 0.1
        v_config = self.velocityToKlampt(v)  #only accepts velocities of #links
        self.sim_controller.setVelocity(v_config,ttl)

    def setTorque(self,t,ttl=None):
        if ttl is not None:
            raise NotImplementedError("Can't set TTL on torque commands yet")
        self.sim_controller.setTorque(t)

    def setPID(self,q,dq,t=None):
        if t is not None:
            self.sim_controller.setPIDCommand(q,dq,t)
        else:
            self.sim_controller.setPIDCommand(q,dq)

    def setPIDGains(self,kP,kD,kI):
        self.sim_controller.setPIDGains(kP,kD,kI)

    def setPiecewiseLinear(self,ts,qs,relative=True):
        if not relative:
            raise NotImplementedError("Can't accept absolute-time piecewise linear commands")
        if len(ts)==0: return
        assert ts[0] >= 0,"First timing needs to be nonnegative"
        self.sim_controller.setLinear(self.configToKlampt(qs[0]),ts[0])
        tlast = ts[0]
        for (t,q) in zip(ts[1:],qs[1:]):
            if t < tlast: raise ValueError("Invalid timing, not monotonic")
            self.sim_controller.addLinear(self.configToKlampt(q),t-tlast)
            tlast = t

    def setPiecewiseCubic(self,ts,qs,vs,relative=True):
        if not relative:
            raise NotImplementedError("Can't accept absolute-time piecewise cubic commands")
        if len(ts)==0: return
        assert ts[0] >= 0,"First timing needs to be nonnegative"
        self.sim_controller.setCubic(self.configToKlampt(qs[0]),self.velocityToKlampt(vs[0]),ts[0])
        tlast = ts[0]
        for (t,q,v) in zip(ts[1:],qs[1:],vs[1:]):
            if t < tlast: raise ValueError("Invalid timing, not monotonic")
            self.sim_controller.addCubic(self.configToKlampt(q),self.velocityToKlampt(v),t-tlast)
            tlast = t

    def moveToPosition(self,q,speed=1.0):
        assert speed == 1.0,"Can't accept non-max speed commands yet"
        self.sim_controller.setMilestone(self.configToKlampt(q))

    def isMoving(self,joint_idx=None):
        return self.sim_controller.remainingTime() > 0
        
    def sensedPosition(self):
        return self.configFromKlampt(self.sim_controller.getSensedConfig())
    
    def sensedVelocity(self):
        return self.velocityFromKlampt(self.sim_controller.getSensedVelocity())

    def sensedTorque(self):
        try:
            return self.sim_controller.getSensedTorque()
        except Exception:
            raise NotImplementedError()

    def commandedPosition(self):
        return self.configFromKlampt(self.sim_controller.getCommandedConfig())

    def commandedVelocity(self):
        return self.velocityFromKlampt(self.sim_controller.getCommandedVelocity())

    def commandedTorque(self):
        return self.velocityFromKlampt(self.sim_controller.getCommandedVelocity())

    def commandedPosition(self):
        return self.configFromKlampt(self.sim_controller.getCommandedConfig())




class KinematicSimControlInterface(RobotInterfaceBase):
    """A very basic control interface that just sets the robot's config to the
    last setPosition command.  Can also perform kinematic simulation of
    simulators.

    Also performs joint limit testing and self collision checking. These change
    the status of the interface to non-'ok' error codes.
    """
    def __init__(self,robot,robotInfo=None):
        RobotInterfaceBase.__init__(self,name=self.__class__.__name__)
        assert isinstance(robot,RobotModel)
        self.robot = robot
        self._status = 'ok'
        self.robotInfo = robotInfo
        if robotInfo is not None:
            assert isinstance(robotInfo,RobotInfo)
            if robotInfo.modelFile is not None:
                self.properties['klamptModelFile'] = robotInfo.modelFile
        q0 = robot.getConfig()
        self.q = self.configFromKlampt(robot.getConfig())
        qmin,qmax = robot.getJointLimits()
        for i in range(robot.numDrivers()):
            if robot.driver(i).getType() == 'affine':
                links = robot.driver(i).getAffectedLinks()
                scale,offset = robot.driver(i).getAffineCoeffs()
                for l,s in zip(links,scale):
                    if s < 0:
                        qmin[l],qmax[l] = qmax[l],qmin[l]
        self.qmin,self.qmax = self.configFromKlampt(qmin),self.configFromKlampt(qmax)
        robot.setConfig(q0)

    def klamptModel(self):
        return self.robot

    @functools.lru_cache(maxsize=None)
    def parts(self):
        if self.robotInfo is None:
            return RobotInterfaceBase.parts(self)
        res = {None:list(range(self.numJoints()))}
        for k in self.robotInfo.parts:
            res[k] = self.robotInfo.partDriverIndices(k)
        return res

    def controlRate(self):
        return 200.0

    def sensors(self):
        sensorNames = []
        index = 0
        while True:
            s = self.robot.sensor(index)
            sname = s.name()
            if len(sname) > 0:
                sensorNames.append(sname)
            else:
                break
            index += 1
        return sensorNames

    def enabledSensors(self):
        return self.sensors()

    def sensorMeasurements(self,name):
        self.robot.setConfig(self.configToKlampt(self.q))
        return self.robot.sensor(name).getMeasurements()

    def endStep(self):
        pass

    def status(self,joint_idx=None):
        return self._status

    def setPosition(self,q):
        if self._status != 'ok':
            return
        if len(q) != len(self.q):
            raise ValueError("Invalid position command")
        if any(v < a or v > b for (v,a,b) in zip(q,self.qmin,self.qmax)):
            for i,(v,a,b) in enumerate(zip(q,self.qmin,self.qmax)):
                if v < a or v > b:
                    self._status = 'joint %d limit violation: %f <= %f <= %f'%(i,a,v,b)
                    return
        self.robot.setConfig(self.configToKlampt(q))
        if self.robot.selfCollides():
            self._status = 'self collision'
            return
        self.q = q
        
    def reset(self):
        self._status = 'ok'
        return True

    def sensedPosition(self):
        return self.q
    
    def commandedPosition(self):
        return self.q


def make(robotModel : RobotModel):
    """Makes a default KinematicSimControlInterface for the robot. This module
    can be referenced using 'klampt.control.simrobotinterface', e.g. as an 
    argument to the ``klampt_control`` app.
    """
    try:
        ri = robotinfo.RobotInfo.get(robotModel.getName())
    except KeyError:
        ri = None
    if ri is not None and ri.parts:
        from .robotinterfaceutils import OmniRobotInterface
        res = OmniRobotInterface(robotModel)
        sim_interface = KinematicSimControlInterface(robotModel)
        res.addPhysicalPart('robot',sim_interface,sim_interface.indices())
        #set up parts
        for (name,indices) in ri.parts.items():
            driver_indices = ri.toDriverIndices(indices)
            res.addVirtualPart(name,driver_indices)
            print('klampt.control.simrobotinterface: adding virtual part "{}", indices {}'.format(name,driver_indices))
        return res
    else:
        from .robotinterfaceutils import RobotInterfaceCompleter
        return RobotInterfaceCompleter(KinematicSimControlInterface(robotModel,ri))
