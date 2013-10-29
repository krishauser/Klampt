from OperationalSpaceController import *
from MotionModel import *
from estimators import *
from controller import *
import robotinfo

class BalanceController(OpSpaceController):
    def __init__(self,robot,jointStiffness=0,jointRecovery=1):
        self.jointStiffness = jointStiffness
        self.jointRecovery = jointRecovery
        OpSpaceController.__init__(self,robot)

        # set the motion model to assume that the feet are fixed in place
        assert(robotinfo.freeBase(robot)==True)
        feet = robotinfo.feet(robot)
        assert len(feet)==2
        self.setMotionModel(FreeBaseRobotMotionModel('velocity','velocity',robot,feet.values()))

    def setupTasks(self,opSpaceController):
        robot = self.robot

        feet = robotinfo.feet(robot)
                
        # priority 1
        # right foot task -- stay still relative to left foot
        RFTask = opSpaceController.addTask(LinkTask(robot, feet['r'], "po", feet['l']),
                                           priority=1,
                                           name="RF",
                                           gains=(0.0,0,0))
        
        # priority 2
        # CoM task
        self.comTask = opSpaceController.addTask(COMTask(robot),
                                                 priority=2,
                                                 gains=([-60.0,-30.0,-10.0], [-0.4,-0.4,-0.2], 0.0),
                                                 weight=[1.0,1.0,1.0])

        #correct for com errors only using legs and hip
        self.comTask.activeDofs = robotinfo.legsAndBase(robot)
        #self.comTask.vcmdmax = 0.25
        #approximation to ZMP criterion
        #self.comTask.dvcmdmax = 100.0

        #store local position of COM relative to feet
        self.comLocalHomePos = {}
        for k,v in feet.iteritems():
            self.comLocalHomePos[v] = se3.apply(se3.inv(robot.getLink(v).getTransform()),self.comTask.getDesiredValue())

        """
        # joint task
        jtask = opSpaceController.addTask(JointTask(robot,range(6,robot.numLinks())),
                                          gains=(-self.jointStiffness, 0, -self.jointRecovery),
                                          weight = 0.001,
                                          priority=2)
                                          """

    def manageTasks(self,inputs,opController):
        return
        if 'contactLinks' in inputs:
            opController.motionModel.linksInContact = inputs['contactLinks']
           
            comDes = self.comTask.getDesiredValue()
            if len(inputs['contactLinks'])==0:
                self.comTask.setWeight(0)
            else:
                comDes = (0,0,0)
                for c in opController.motionModel.linksInContact:
                    #rotate the x-y components of the com position,
                    #adjust vertically by the z component
                    (Rl,tl) = self.robot.getLink(c).getTransform()
                    comLocal = self.comLocalHomePos[c]
                    comDesC = [0]*3
                    so3.apply(Rl,(comLocal[0],comLocal[1],0))
                    comDesC[2] += tl[2]+comLocal[2]
                    comDes = vectorops.add(comDes,comDesC)

                comDes = vectorops.div(comDes,len(opController.motionModel.linksInContact))
            self.comTask.setDesiredValue(comDes)
        return
    

class FootContactEstimator(BaseController):
    def __init__(self,robot):
        self.robot = robot
        self.lfscale = -1.0
        self.lfthresh = 100
        self.rfscale = -1.0
        self.rfthresh = 100
    def output_and_advance(self,**inputs):
        lf = inputs['LF_ForceSensor']
        rf = inputs['RF_ForceSensor']
        #print lf,rf
        out = {}
        contactLinks = []
        if self.lfscale*lf[2] > self.lfthresh:
            out['lfcontact']=1
            contactLinks.append(robotinfo.feet(self.robot)['l'])
        else:
            out['lfcontact']=0
        if self.rfscale*rf[2] > self.rfthresh:
            out['rfcontact']=1
            contactLinks.append(robotinfo.feet(self.robot)['r'])
        else:
            out['rfcontact']=0
        print "Estimated links in contact",contactLinks
        out['contactLinks'] = contactLinks
        return out


class HuboStateEstimator(BaseController):
    def __init__(self,robot):
        self.robot = robot
        self.qbase = robot.getConfig()[:6]
    def output_and_advance(self,**inputs):
        enc = inputs['encoders']
        return {'q':qbase + enc}

def makeBasic(robot):
    return TimedControllerSequence([BaseController(),BalanceController(robot)],
                                   [0.1,1e30])

def makeAdaptive(robot):
    balanceController = BalanceController(robot)
    motionModel = FreeBaseAdaptiveMotionModel('velocity','velocity',robotinfo.leg(robot,'l')+robotinfo.leg(robot,'r'),robot)
    differ = DifferenceEstimator("q","qcmd")
    balanceController.setMotionModel(motionModel)
    sysidEstimator = SystemIDEstimator(robot,motionModel)
    c = MultiController()
    c.launch(VelocityEstimator(robot))
    c.launch(sysidEstimator)
    c.launch(TimedControllerSequence([BaseController(),balanceController],[0.1,1e30]))
    c.map_my_output('qcmd')
    c.map_my_output('dqcmd')
    return c

def makeFootEstimator(robot):
    balanceController = BalanceController(robot)
    footEstimator = FootContactEstimator(robot)
    c = MultiController()
    c.launch(VelocityEstimator(robot))
    c.launch(footEstimator)
    c.launch(TimedControllerSequence([BaseController(),balanceController],[0.1,1e30]))
    c.map_my_output('qcmd')
    c.map_my_output('dqcmd')
    return c

def make(robot):
    #return makeAdaptive(robot)
    return makeFootEstimator(robot)

