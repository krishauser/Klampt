"""Relays messages between ROS controller and Baxter controller in Klamp't
"""

import controller
import rospy
from baxter_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

POSITION_MODE=1
VELOCITY_MODE=2
TORQUE_MODE=3

class RosBaxterController(controller.BaseController):
    """A controller that reads JointCommand from the ROS topics
    
    '/[robot_name]/limb/right/joint_command',
    '/[robot_name]/limb/left/joint_command'
    
    and writes JointState to the ROS topic
    
    '/[robot_name]/joint_states'.
    """
    def __init__(self,robot_name,link_list):
        self.state = JointState()
        n = len(link_list)
        self.state.name = link_list[:]
        self.state.position     = []
        self.state.velocity     = []
        self.state.effort       = []

        # fast indexing structure for partial commands
        self.nameToIndex = dict(zip(self.state.name,range(i)))

        # Setup publisher of robot states
        self.pub = rospy.Publisher("/%s/joint_states"%(robot_name,), JointState)
        
        # set up the command subscriber
        self.currentLeftArmMsg = None
        self.currentRightArmMsg = None
        rospy.Subscriber('/%s/limb/left/joint_command'%(robot_name), JointTrajectory,self.leftArmCallback)
        rospy.Subscriber('/%s/limb/right/joint_command'%(robot_name), JointTrajectory,self.rightArmCallback)
        return

    def leftArmCallback(self,msg):
        self.currentLeftArmMsg = msg
        return

    def rightArmCallback(self,msg):
        self.currentRightArmMsg = msg
        return

    def output(self,**inputs):       
        res = {}
        if self.currentRightArmMsg != None and self.currentLeftArmMsg != None:
            if self.currentRightArmMsg.mode != self.currentLeftArmMsg.mode:
                print "RosBaxterController: Needs to take arm message of the same type"
                return res
            
            #map JointCommands to Klamp't style commands
            value = [0.0]*len(self.state.name)
            for n,cmd in zip(self.currentRightArmMsg.names,self.currentRightArmMsg.command):
                value[self.nameToIndex[n]] = cmd
            for n,cmd in zip(self.currentLeftArmMsg.names,self.currentLeftArmMsg.command):
                value[self.nameToIndex[n]] = cmd
                
            if self.currentRightArmMsg.mode == POSITION_MODE:
                res['qcmd'] = value
            elif self.currentRightArmMsg.mode == VELOCITY_MODE:
                res['dqcmd'] = value
                res['tcmd'] = 1
            elif self.currentRightArmMsg.mode == TORQUE_MODE:
                res['torquecmd'] = value
            else:
                raise RuntimeError("Invalid mode "+str(self.currentRightArmMsg.mode))

        #sense the configuration and velocity, possibly the effort
        self.state.header.stamp = rospy.get_rostime()
        if 'q' in inputs:
            self.state.position = inputs['q']
        if 'dq' in inputs:
            self.state.velocity = inputs['dq']
        if 'torque' in inputs:
            self.state.effort = inputs['torque']
        self.pub.publish(self.state)
        return res

    def map_output(self,vector,names,output_map,output_name):
        """Maps a partial vector to output_map[output_name].
        If output_name exists in output_map, then only the named values
        are overwritten.  Otherwise, the missing values are set to zero.
        """
        val = []
        if output_name in output_map:
            val = output_map[output_name]
        else:
            val = [0.0]*len(self.state.name)
        for n,v in zip(names,vector):
            val[self.nameToIndex[n]] = v
        output_map[output_name] = val
        return
