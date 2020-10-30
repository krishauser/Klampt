from OperationalSpaceController import *
from controller import *
from estimators import *
import robotinfo

class ArmController(OpSpaceController):
    def __init__(self,robot,arm):
        self.arm = arm
        self.dofs = robotinfo.arm(robot,self.arm)
        OpSpaceController.__init__(self,robot)

    def setupTasks(self,opSpaceController):
        robot = self.robot

        opSpaceController.activeDofs = self.dofs
        opSpaceController.regularizationFactor = 1e-2
        
        # joint task
        self.jtask = opSpaceController.addTask(JointTask(robot,self.dofs),
                                          gains=(-0.01, 0, 0.0),
                                          weight = 0.001,
                                          priority=2)
        self.eeTask = opSpaceController.addTask(LinkTask(robot,self.dofs[-1],'position'),
                                                gains=(-10.0,-0.25,-20.0),
                                                weight = 1.0,
                                                priority=1)
        self.eeTask.eImax = 1.0
        self.eeTask.vcmdmax = 1.0

    def manageTasks(self,inputs,opController):
        return


def makeTwoArm(robot):
    rarm_dofs = robotinfo.arm(robot,'r')
    larm_dofs = robotinfo.arm(robot,'l')
    other_dofs = [i for i in range(robot.numLinks()) if i not in rarm_dofs and i not in larm_dofs]
    
    master = MultiController()
    #need to get dq from q
    master.launch(VelocityEstimator(robot))

    c = MultiController()
    #set the right arm controller to qcmd_r,dqcmd_r
    rc = ArmController(robot,'r')
    rc = c.launch(rc)
    c.map_output(rc,'qcmd','qcmd_r')
    c.map_output(rc,'dqcmd','dqcmd_r')
    lc = ArmController(robot,'l')
    lc = c.launch(lc)
    #set the left arm controller to qcmd_r,dqcmd_r
    c.map_output(lc,'qcmd','qcmd_l')
    c.map_output(lc,'dqcmd','dqcmd_l')
    #map the single arm controller outputs to qcmd and dqcmd
    c.launch(ComposeController({'qcmd_r':rarm_dofs,'qcmd_l':larm_dofs,'qcmd':other_dofs},'qcmd'))
    c.launch(ComposeController({'dqcmd_r':rarm_dofs,'dqcmd_l':larm_dofs,'dqcmd':other_dofs},'dqcmd'))
    
    #switch to the operational space controller after 0.1 second
    tc = TimedControllerSequence([BaseController(),c],[0.01,1e30])
    master.launch(tc)
    
    #send qcmd and dqcmd to the controllers
    master.map_my_output('qcmd')
    master.map_my_output('dqcmd')
    return master

def make(robot):
    return makeTwoArm(robot)

