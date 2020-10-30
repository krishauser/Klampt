"""This controller accepts ROS JointCommands messages (from DRCSim)
from the ROS topic '/[robot_name]/joint_commands' and writes out
JointState messages to the ROS topic '/[robot_name]/joint_state'.

It also acts as a ROS clock server.
"""

import controller
import rospy
from osrf_msgs.msg import JointCommands
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock


#test
"""
class Time:
    def __init__(self):
        self.secs = 0
        self.nsecs = 0

class Clock:
    def __init__(self):
        self.clock = Time()

class JointCommands:
    def __init__(self):
        pass

class JointState:
    def __init__(self):
        pass

class PublisherProxy:
    def __init__(self,name,message_type):
        self.name = name
        self.message_type = message_type
        return
    def publish(self,object):
        print "Publishing on topic "+self.name+":"
        for k,v in object.__dict__.iteritems():
            if k[0]=='_': continue
            print "  ",k,":",v
        return

class SubscriberProxy:
    def __init__(self,name,message_type,callback):
        self.name = name
        self.message_type = message_type
        self.callback = callback

class RospyProxy:
    def init_node(self,name):
        print "init_node("+name+") called"
        return
    def Publisher(self,topic,message_type):
        print "Making publisher on topic "+topic
        return PublisherProxy(topic,message_type)
    def Subscriber(self,topic,message_type,callback):
        print "Making subscriber on topic "+topic
        return SubscriberProxy(topic,message_type,callback)
rospy = RospyProxy()
"""

class RosRobotController(controller.BaseController):
    """A controller that reads JointCommands from the ROS topic
    '/[robot_name]/joint_commands' and writes JointState to the ROS topic
    '/[robot_name]/joint_state'.

    Uses PID control with feedforward torques.

    Supports partial commands as well, e.g., only controlling part of the
    robot, ignoring feedforward torques, etc.  Note that if you start sending
    some command and then stop (like velocities or feedforward torques) this
    controller will NOT zero them out.

    Currently does not support:
        * Setting PID gain constants,
        * Setting PID integral term bounds.
    """
    def __init__(self,klampt_robot_model):
        self.robot = klampt_robot_model
        
        self.state = JointState()
        n = self.robot.numLinks()
        self.state.name = [self.robot.getLink(i).getName() for i in range(n)]
        self.state.position     = []
        self.state.velocity     = []
        self.state.effort       = []

        # fast indexing structure for partial commands
        self.nameToIndex = dict(zip(self.state.name,range(i)))

        # Setup publisher of robot states
        robot_name = self.robot.getName()
        self.pub = rospy.Publisher("/%s/joint_states"%(robot_name,), JointState)
        
        # set up the command subscriber
        self.firstJointCommand = False
        self.newJointCommand = False
        self.currentJointCommand = JointCommands()
        self.currentJointCommand.name = self.state.name
        rospy.Subscriber('/%s/joint_commands'%(robot_name), JointCommands,self.jointCommandCallback)
        return

    def jointCommandCallback(self,msg):
        if self.firstJointCommand:
            print "Warning, got ROS joint command without reading from simulator yet, ignoring."
            return
        
        #copy elements of msg to the current joint command
        elements = ['position','velocity','effort','kp_position','ki_position','kd_position','kp_velocity','i_effort_min','i_effort_max']
        for i,n in enumerate(msg.name):
            index = self.nameToIndex[n]
            for e in elements:
                emsg = getattr(msg,e)
                if len(emsg) > 0:
                    ecmd = getattr(self.currentJointCommand,e)
                    ecmd[index] = emsg[i]
        self.newJointCommand = True
        return

    def output(self,**inputs):
        if self.firstJointCommand:
            self.firstJointCommand = False
            #fill out currentJointCommand from the simulator properties
            n = self.robot.numLinks()
            self.currentJointCommand.position = inputs['qcmd']
            self.currentJointCommand.velocity = inputs['dqcmd']
            self.currentJointCommand.effort = [0.0]*n
            self.currentJointCommand.kp_position = [0.0]*n
            self.currentJointCommand.ki_position = [0.0]*n
            self.currentJointCommand.kd_position = [0.0]*n
            self.currentJointCommand.kp_velocity = [0.0]*n
            self.currentJointCommand.i_effort_min = [0.0]*n
            self.currentJointCommand.i_effort_max = [0.0]*n
        
        res = {}
        #read in the joint command, and send it to output
        if self.newJointCommand:
            self.newJointCommand = False
            res['qcmd'] = self.currentJointCommand.position
            res['dqcmd'] = self.currentJointCommand.velocity
            res['torquecmd'] = self.currentJointCommand.effort

        #sense the configuration and velocity, possibly the effort
        if 'q' in inputs:
            self.state.position = inputs['q']
        if 'dq' in inputs:
            self.state.velocity = inputs['dq']
        if 'torque' in inputs:
            self.state.effort = inputs['torque']
        self.state.header.stamp = rospy.get_rostime()
        self.pub.publish(self.state)
        return res

class RosTimeController(controller.BaseController):
    """A controller that simply publishes the simulation time to ROS.
    Doesn't output anything.
    """
    def __init__(self):
        self.clockpub = rospy.Publisher("/clock", Clock)
        
    def output(self,**inputs):
        t = inputs['t']
        time = Clock()
        time.clock.secs = int(t)
        time.clock.nsecs = int(float(t-int(t))*1000000000)
        self.clockpub.publish(time)
        return {}



ros_initialized = False

def make(klampt_robot_model):
    global ros_initialized
    if not ros_initialized:
        ros_initialized = True
        rospy.init_node('klampt_sim')
        #launch a controller to publish the simulation time to ROS, PLUS
        #the robot's controller
        c = controller.MultiController()
        c.launch(RosTimeController())
        c.launch(RosRobotController(klampt_robot_model))
        return c
    #just launch the robot's controller, some other RosTimeController has been
    #launched before
    return RosRobotController(klampt_robot_model)
