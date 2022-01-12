from .robotcontroller import RobotControllerIO,RobotControllerBlock
import math

class BigWiggleController(RobotControllerBlock):
    """A controller that wiggles each of the robot's joints between their
    extrema.
    """
    def __init__(self, robot, period=10):
        self.robot = robot
        self.qmin,self.qmax = robot.getJointLimits()
        for i in range(len(self.qmin)):
            if self.qmax[i]-self.qmin[i] > math.pi*2:
                self.qmax[i] = min(self.qmax[i],math.pi)
                self.qmin[i] = max(self.qmin[i],-math.pi)
        self.q = robot.getConfig()
        self.index = 0
        self.startTime = None
        self.period = period
        RobotControllerBlock.__init__(self,robot)

    def __getstate__(self):
        return {'q':self.q,'index':self.index,'startTime':self.startTime,'period':self.period}

    def __setstate__(self,state):
        self.q = state['q']
        self.index=state['index']
        self.startTime=state['startTime']
        self.period=state['period']
        
    def advance(self,**inputs):
        api = RobotControllerIO(inputs)
        t = api.time()
        if self.startTime == None:
            self.startTime = t
        u = (t - self.startTime)/self.period
        #pick a good index
        while self.qmin[self.index]==self.qmax[self.index]:
            self.index += 1
            if self.index >= self.robot.numLinks():
                self.index = 0
        
        qdes = self.q[:]
        #wiggle between joint extrema
        if u < 0.25:
            s = math.sin(u*4*math.pi*0.5)
            qdes[self.index] += s*(self.qmax[self.index]-qdes[self.index])
            pass
        elif u < 0.75:
            s = (-math.sin(u*4*math.pi*0.5)+1.0)*0.5
            qdes[self.index] = self.qmax[self.index]+s*(self.qmin[self.index]-self.qmax[self.index])
        elif u < 1.0:
            s = math.sin(u*4*math.pi*0.5)+1.0
            qdes[self.index] = self.qmin[self.index]+s*(qdes[self.index]-self.qmin[self.index])
        else:
            #go to next index
            self.startTime = t
            self.index += 1
            if self.index >= self.robot.numLinks():
                self.index = 0
        return api.makePositionCommand(qdes)

    def signal(self,type,**inputs):
        if type=='reset':
            self.index = 0
            self.startTime = None


class OneJointWiggleController(RobotControllerBlock):
    """A controller that wiggles one of the robot's joints by some magnitude"""
    def __init__(self,robot,index,magnitude,period=10):
        self.robot = robot
        self.qmin,self.qmax = robot.getJointLimits()
        self.q = robot.getConfig()
        self.index = index
        self.startTime = None
        self.magnitude = magnitude
        self.period = period
        RobotControllerBlock.__init__(self,robot)

    def __getstate__(self):
        return {'startTime',self.startTime}

    def __setstate__(self,state):
        self.startTime=state['startTime']
        
    def advance(self,**inputs):
        api = RobotControllerIO(inputs)
        t = api.time()
        if self.startTime == None:
            self.startTime = t
        u = (t - self.startTime)/self.period
        
        qdes = self.q[:]
        s = math.sin(u*4*math.pi*0.5)
        qdes[self.index] += s*self.magnitude
        return api.makePositionCommand(qdes)

    def signal(self,type,**inputs):
        if type=='reset':
            self.startTime = None

