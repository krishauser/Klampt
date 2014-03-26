"""This controller accepts ROS JointTrajectory messages
from the ROS topic '/[robot_name]/joint_trajectory' and writes out
JointState messages to the ROS topic '/[robot_name]/joint_state'.

It also acts as a ROS clock server.
"""

import controller
import rospy
from trajectory_msgs.msg import JointTrajectory
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

class JointTrajectory:
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
    """A controller that reads JointTrajectory from the ROS topic
    '/[robot_name]/joint_trajectory' and writes JointState to the ROS topic
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
        self.firstJointTrajectory = False
        self.currentJointTrajectoryMsg = None
        rospy.Subscriber('/%s/joint_trajectory'%(robot_name), JointTrajectory,self.jointTrajectoryCallback)

        # these are parsed in from the trajectory message
        self.currentTrajectoryStart = 0
        self.currentTrajectoryNames = []
        self.currentPositionTrajectory = None
        self.currentVelocityTrajectory = None
        self.currentEffortTrajectory = None
        return

    def jointTrajectoryCallback(self,msg):
        self.currentJointTrajectoryMsg = msg
        return

    def set_index(self,name,index):
        self.nameToIndex[name] = index

    def output(self,**inputs):       
        res = {}
        if self.currentJointTrajectoryMsg != None:
            #parse in the message -- are positions, velocities, efforts specified?
            self.currentTrajectoryStart = inputs['t']
            self.currentTrajectoryNames = self.currentJointTrajectoryMsg.joint_names
            times = [p.time_from_start for p in self.currentJointTrajectoryMsg.points]
            milestones = [p.positions for p in self.currentJointTrajectoryMsg.points]
            velocities = [p.velocities for p in self.currentJointTrajectoryMsg.points]
            efforts = [p.velocities for p in self.currentJointTrajectoryMsg.points]
            if all(len(x) != 0 for x in milestones):
                self.currentPositionTrajectory = Trajectory(times,milestones)
            else:
                self.currentPositionTrajectory = None
            if all(len(x) != 0 for x in velocities):
                self.currentVelocityTrajectory = Trajectory(times,velocities)
            else:
                self.currentVelocityTrajectory = None
            if all(len(x) != 0 for x in efforts):
                self.currentEffortTrajectory = Trajectory(times,efforts)
            else:
                self.currentEffortTrajectory = None
            self.currentJointTrajectoryMsg = None
            
        #evaluate the trajectory and send it to controller's output
        t = inputs['t']-self.currentTrajectoryStart
        if self.currentPositionTrajectory != None:
            qcmd = self.currentPositionTrajectory.eval(t,'halt')
            self.map_output(qcmd,self.currentTrajectoryNames,res,'qcmd')
        if self.currentVelocityTrajectory != None:
            dqcmd = self.currentVelocityTrajectory.deriv(t,'halt')
            self.map_output(dqcmd,self.currentTrajectoryNames,res,'dqcmd')
        elif self.currentPositionTrajectory != None:
            #automatic differentiation
            dqcmd = self.currentPositionTrajectory.deriv(t,'halt')
            self.map_output(dqcmd,self.currentTrajectoryNames,res,'dqcmd')
        if self.currentEffortTrajectory != None:
            torquecmd = self.currentEffortTrajectory.eval(t,'halt')
            self.map_output(torquecmd,self.currentTrajectoryNames,res,'torquecmd')

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
    """Creates a ROS controller for the given model.
    klampt_robot_model is a RobotModel instance."""
    global ros_initialized
    robotName = klampt_robot_model.getName()
    linkNames = [klampt_robot_model.getLink(i).getName() for i in range(klampt_robotmodel.numLinks())]
    if not ros_initialized:
        ros_initialized = True
        rospy.init_node('klampt_sim')
        #launch a controller to publish the simulation time to ROS, PLUS
        #the robot's controller
        c = controller.MultiController()
        c.launch(RosTimeController())
        c.launch(RosRobotController(robotName,linkNames))
        return c
    #just launch the robot's controller, some other RosTimeController has been
    #launched before
    return RosRobotController(robotName,linkNames)
