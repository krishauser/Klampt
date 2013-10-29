from OperationalSpaceController import *
from controller import *
from estimators import *
from MotionModel import *
import robotinfo

class ArmController(OpSpaceController):
    def __init__(self,robot,arm):
        self.arm = arm
        self.dofs = robotinfo.arm(robot,self.arm)
        OpSpaceController.__init__(self,robot)

    def setupTasks(self,opSpaceController):
        robot = self.robot

        opSpaceController.activeDofs = self.dofs
        
        # joint task
        self.jtask = opSpaceController.addTask(JointTask(robot,self.dofs),
                                          gains=(-1.0, 0, 0.0),
                                          weight = 1,
                                          priority=1)

def makeRArmEstimator(robot):
    rarm_dofs = robotinfo.arm(robot,'r')
    other_dofs = [i for i in range(robot.numLinks()) if i not in rarm_dofs]
    
    master = MultiController()
    #need to get dq from q
    master.launch(VelocityEstimator(robot))
    #estimate the gravity compensation terms
    sysid = SystemIDEstimator(robot,GravityCompensationAdaptiveMotionModel(robot,'velocity','velocity'))
    master.launch(sysid)
    #debug the motion model
    master.launch(DebugMotionModelController(sysid.getMotionModel(),robot,rarm_dofs))

    c = MultiController()
    #set the right arm controller to output qcmd_r,dqcmd_r
    rc = ArmController(robot,'r')
    rc.setMotionModel(sysid.getMotionModel())
    rc = c.launch(rc)
    c.map_output(rc,'qcmd','qcmd_r')
    c.map_output(rc,'dqcmd','dqcmd_r')
    #compose the rarm controller with the other dofs controller
    c.launch(ComposeController({'qcmd_r':rarm_dofs,'qcmd':other_dofs},'qcmd'))
    c.launch(ComposeController({'dqcmd_r':rarm_dofs,'dqcmd':other_dofs},'dqcmd'))
    
    #switch to the operational space controller after 1 second
    tc = TimedControllerSequence([BaseController(),c],[1,1e30])
    master.launch(tc)
    
    #send qcmd and dqcmd to the controllers
    master.map_my_output('qcmd')
    master.map_my_output('dqcmd')
    return master

def make(robot):
    return makeRArmEstimator(robot)

