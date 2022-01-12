"""Defines utilities for connecting simulation controllers to the Robot 
Interface Layer, and connect ControllerBlocks to RobotInterfaceBase receivers.
"""

from .blocks.robotcontroller import RobotControllerBlock
from .robotinterface import RobotInterfaceBase
from .robotinterfaceutils import RobotInterfaceCompleter

class SimRobotControllerToInterface(object):
    """A class that provides an API like :class:`SimRobotController` so that
    simulation control code can be ported easily to a 
    :class:`RobotInterfaceBase` Robot Interface Layer API.

    Missing:

    - setRate(dt): can't change a real robot's control rate.
    - getSetting()/setSetting(): no interface to configure robot.
    - setMilestone(q,dq): can't set a terminal velocity.
    - add*(): can't queue up commands.
    - getPIDGains(): can't configure PID gains.

    Arguments:
        robotInterface (RobotInterfaceBase): the robot interface to use
    """
    def __init__(self, robotInterface : RobotInterfaceBase) -> None:
        if not robotInterface.properties.get('complete',False):
            robotInterface = RobotInterfaceCompleter(robotInterface)
        self.robotInterface = robotInterface

    def initialize(self):
        self.robotInterface.initialize()
    def beginStep(self):
        self.robotInterface.beginStep()
    def endStep(self):
        self.robotInterface.endStep()
    def model(self):
        return self.robotInterface.klamptModel()
    def setRate(self,dt):
        raise NotImplementedError("setRate() may not be called")
    def getRate(self):
        return 1.0/self.robotInterface.controlRate()

    def getCommandedConfig(self):
        return self.robotInterface.configToKlampt(self.robotInterface.commandedPosition())
    def getCommandedVelocity(self):
        return self.robotInterface.velocityToKlampt(self.robotInterface.commandedVelocity())
    def getCommandedTorque(self):
        return self.robotInterface.commandedTorque()
    def getSensedConfig(self):
        return self.robotInterface.configToKlampt(self.robotInterface.sensedPosition())
    def getSensedVelocity(self):
        return self.robotInterface.velocityToKlampt(self.robotInterface.sensedVelocity())
    def getSensedTorque(self):
        return self.robotInterface.sensedTorque()

    def sensor(self,index_or_name):
        if isinstance(index_or_name,int):
            sensorNames = self.robotInterface.sensors()
            if index_or_name < 0 or index_or_name >= len(sensorNames):
                #"null" sensor
                return _SimRobotSensorFromInterface(self.robotInterface,None)
            name = sensorNames[index_or_name]
        else:
            name = index_or_name
            enabled = self.robotInterface.enabledSensors()
            if name not in enabled:
                if not self.robotInterface.hasSensor(name):
                    #"null" sensor
                    return _SimRobotSensorFromInterface(self.robotInterface,None)
                self.robotInterface.enableSensor(name)
        return _SimRobotSensorFromInterface(self.robotInterface,name)
    def commands(self):
        return ['estop','softStop','reset']
    def sendCommand(self,name,args):
        getattr(self,name)()
    def getSetting(self,name):
        raise ValueError("Can't change settings")
    def setSetting(self,name,val):
        raise ValueError("Can't change settings")

    def setMilestone(self,q,dq=None):
        if dq is not None:
            raise NotImplementedError("setMilestone(q[,dq]) may not be called with dq")
        self.robotInterface.moveToPosition(self.robotInterface.configFromKlampt(q))
    def addMilestone(self,q,dq=None):
        raise NotImplementedError("addMilestone() may not be called")
    def addMilestoneLinear(self,q):
        raise NotImplementedError("addMilestoneLinear() may not be called")
    def setLinear(self,q,dt):
        self.robotInterface.setPiecewiseLinear([dt],[self.robotInterface.configFromKlampt(q)])
    def setCubic(self,q,v,dt):
        self.robotInterface.setPiecewiseCubic([dt],[self.robotInterface.configFromKlampt(q)],[self.robotInterface.velocityFromKlampt(v)])
    def addLinear(self,q,dt):
        raise NotImplementedError("addLinear() may not be called")
    def addCubic(self,q,dt):
        raise NotImplementedError("addCubic() may not be called")

    def remainingTime(self):
        return self.robotInterface.destinationTime()

    def setVelocity(self,dq,dt):
        self.robotInterface.setVelocity(self.robotInterface.velocityFromKlampt(dq),dt)
    def setTorque(self,t):
        self.robotInterface.setTorque(t)
    def setPIDCommand(self,qdes,dqdes,tfeedforward=None):
        self.robotInterface.setPID(self.robotInterface.configFromKlampt(qdes),self.robotInterface.velocityFromKlampt(dqdes),tfeedforward)
    def setManualMode(self,enabled):
        pass

    def getControlType(self):
        return 'unknown'

    def setPIDGains(self,kP,kI,kD):
        self.robotInterface.setPIDGains(kP,kI,kD)
    def getPIDGains(self):
        return None,None,None


class _SimRobotSensorFromInterface(object):
    def __init__(self, iface : RobotInterfaceBase, name : str):
        self.iface = iface
        self._name = name
    def name(self):
        if self._name is None: return ''
        return self._name
    def type(self):
        if self._name is None: return ''
        return 'UnknownSensor'
    def robot(self):
        return self.iface.klamptModel()
    def measurementNames(self):
        if self._name is None: return []
        return self.robot().sensor(self._name).measurementNames()
    def getMeasurements(self):
        if self._name is None: return []
        return self.iface.sensorMeasurements(self._name)
    def getSetting(self,name):
        return self.robot().sensor(self._name).getSetting(name)
    def setSetting(self,name,val):
        raise NotImplementedError("setSetting may not be called")
    def drawGL(self,measurements=None):
        if measurements is not None:
            return self.robot().sensor(self._name).drawGL(measurements)
        return self.robot().sensor(self._name).drawGL()


class RobotControllerBlockToInterface(object):
    """A class that connects the I/O of a :class:`RobotControllerBlock` robot
    controller with a :class:`RobotInterfaceBase` Robot Interface Layer API.

    Arguments:
        controller (RobotControllerBlock): the controller block.
        robotInterface (RobotInterfaceBase): the robot interface
        controllerRateRatio (float, optional): if not 1, scales how many times
            the block is run for each main loop of the robotInterface
    """
    def __init__(self, controller : RobotControllerBlock, robotInterface : RobotInterfaceBase, controllerRateRatio=1):
        assert isinstance(robotInterface,RobotInterfaceBase),"robotInterface must be a RobotInterfaceBase"
        if not robotInterface.properties.get('complete',False):
            robotInterface = RobotInterfaceCompleter(robotInterface,base_initialized=True)
            robotInterface.initialize()
        assert isinstance(controller,RobotControllerBlock),"controller must be a ControllerBlock"
        self.robotInterface = robotInterface
        self.controller = controller
        self.controllerRateRatio = controllerRateRatio
        self.lastInputs = None
        self.lastOutputs = None

    def advance(self,step_interface=False):
        """Moves forward the controller and interface one step.

        If step_interface = True the robotInterface will also be
        stepped.  Otherwise, make sure you call startStep()/endStep() like::
        
            c2i = RobotControllerBlockToInterface(...)
            c2i.robotInterface.startStep()
            c2i.advance()
            c2i.robotInterface.endStep()
        """
        if self.controllerRateRatio != 1:
            raise NotImplementedError("Can't do controller / robotInterface time step ratio != 1")
        if step_interface:
            self.robotInterface.startStep()
        #sensing
        inputs = dict()
        inputs['t'] = self.robotInterface.clock()
        inputs['dt'] = 1.0/self.robotInterface.controlRate()
        inputs['q'] = self.robotInterface.configToKlampt(self.robotInterface.sensedPosition())
        inputs['dq'] = self.robotInterface.velocityToKlampt(self.robotInterface.sensedVelocity())
        try:
            inputs['torque'] = self.robotInterface.velocityToKlampt(self.robotInterface.sensedTorque())
        except NotImplementedError:
            pass
        for s in self.robotInterface.enabledSensors():
            try:
                inputs[s] = self.robotInterface.sensorMeasurements(s)
            except Exception as e:
                import traceback
                print("Uh... can't read sensor",s,"from",self.robotInterface)
                traceback.print_exc()
        #plan
        res = self.controller.advance(**inputs)
        #act
        if 'qcmd' in res:
            qcmd = self.robotInterface.configFromKlampt(res['qcmd'])
            if 'dqcmd' in res:
                dqcmd = self.robotInterface.velocityFromKlampt(res['dqcmd'])
                if 'torquecmd' in res:
                    tcmd = self.robotInterface.velocityFromKlampt(res['torquecmd'])
                    self.robotInterface.setPID(qcmd,dqcmd,tcmd)
                else:
                    self.robotInterface.setPID(qcmd,dqcmd)
            else:
                self.robotInterface.setPosition(qcmd)
        elif 'dqcmd' in res:
            assert 'tcmd' in res
            dqcmd = self.robotInterface.velocityFromKlampt(res['dqcmd'])
            self.robotInterface.setVelocity(dqcmd,res['tcmd'])
        elif 'torquecmd' in res:
            tcmd = self.robotInterface.velocityFromKlampt(res['torquecmd'])
            self.robotInterface.setTorque(tcmd)
        if step_interface:
            self.robotInterface.endStep()
        self.lastInputs = inputs
        self.lastOutputs = res
    

class RobotInterfacetoVis(object):
    """A class that produces a standard klampt.vis display to monitor a 
    :class:`RobotInterfaceBase` Robot Interface Layer API.

    Note: this assumes that ``vis`` has been set up with an appropriate world.
    """
    def __init__(self, robotInterface : RobotInterfaceBase, visRobotIndex=0):
        self.interface = robotInterface
        self.visRobotIndex = visRobotIndex
        self.text_x = 10
        self.text_y = 10
        self.tag = '_'+str(visRobotIndex)+'_'

    def update(self):
        from klampt import vis
        t = 0
        try:
            t = self.interface.clock()
            vis.addText(self.tag+"clock",'%.3f'%(t,),(self.text_x,self.text_y))
        except NotImplementedError:
            vis.addText(self.tag+"clock",str(self.interface)+" provides no clock",(self.text_x,self.text_y))
        try:
            stat = self.interface.status()
            if stat != 'ok':
                vis.addText(self.tag+"status",'Status: '+str(stat),(self.text_x,self.text_y+15),color=(1,0,0))
            else:
                try:
                    vis.remove(self.tag+"status")
                except Exception:
                    pass
        except NotImplementedError:
            vis.addText(self.tag+"status",str(self.interface)+" provides no status",(self.text_x,self.text_y+15))
        moving = False
        endTime = 0
        try:
            moving = self.interface.isMoving()
            if moving:
                endTime = self.interface.destinationTime()
            if moving:
                if endTime > t:
                    vis.addText(self.tag+"moving","Moving, %.3fs left"%(endTime-t,),(self.text_x,self.text_y+30))
                else:
                    vis.addText(self.tag+"moving","Moving",(self.text_x,self.text_y+30))
            else:
                try:
                    vis.remove(self.tag+"moving")
                except Exception:
                    pass
        except NotImplementedError:
            pass

        try:
            qsns = self.interface.configToKlampt(self.interface.sensedPosition())
            vis.add(self.tag+"q_sns",qsns,robot=self.visRobotIndex,color=(0,1,0,0.5))
        except NotImplementedError:
            qsns = None
        try:
            jcmd = self.interface.commandedPosition()
            if all(j is not None for j in jcmd):
                qcmd = self.interface.configToKlampt(jcmd)
                if qcmd != qsns:
                    vis.add(self.tag+"q_cmd",qcmd,robot=self.visRobotIndex,color=(1,1,0,0.5))
            else:
                try:
                    vis.remove(self.tag+"q_cmd")
                except Exception:
                    pass
        except NotImplementedError:
            pass
        try:
            if moving and endTime > t:
                jdes = self.interface.destinationPosition()
                assert all(j is not None for j in jdes)
                qdes = self.interface.configToKlampt(jdes)
                vis.add(self.tag+"q_dest",qdes,robot=self.visRobotIndex,color=(1,0,0,0.5))
            else:
                try:
                    vis.remove(self.tag+"q_dest")
                except Exception:
                    pass
        except NotImplementedError:
            pass
