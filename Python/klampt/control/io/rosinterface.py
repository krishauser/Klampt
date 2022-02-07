"""Defines interface between a Klamp't application and ROS robot.

- :class:`RosRobotInterface` is a Klampt Robot Interface Layer
  (:class:~klampt.control.robotinterface.RobotInterfaceBase`) implementation 
  that outputs commands to a ROS controller.

This also implements the make() protocol for use in klampt_control.  The returned
controller accepts ROS JointTrajectory messages from the ROS topic
'/[robot_name]/joint_trajectory' and writes out JointState messages to the ROS
topic '/[robot_name]/joint_state'.  It also acts as a ROS clock server.
"""

from .. import robotinterface
import rospy
from klampt.model.trajectory import Trajectory,HermiteTrajectory
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from sensor_msgs.msg import JointState


class RosRobotInterface(robotinterface.RobotInterfaceBase):
    """Implements a Klampt Robot Interface Layer for a ROS controlled

    robot that outputs JointState messages and accepts JointTrajectory
    commands.

    initialize() will create and start a ROS node.

    initialize(False) will not start ROS. Instead, you'll have to do this 
    manually before calling initialize.

    close() is optional to call before quitting. It must be called if
    you'd like to re-initialize.
    """
    def __init__(self,robot,joint_state_sub_topic,joint_trajectory_pub_topic):
        robotinterface.RobotInterfaceBase.__init__(self)
        self.properties['asynchronous'] = True
        self.robot = robot
        self.link_list = [robot.link(i).getName() for i in range(robot.numLinks())]
        self.driver_list = [robot.link(robot.driver(i).getAffectedLink()).getName() for i in range(robot.numDrivers())]
        self.link_dict = dict((n,i) for i,n in enumerate(self.link_list))
        self.joint_state_sub_topic = joint_state_sub_topic
        self.joint_trajectory_pub_topic = joint_trajectory_pub_topic
        self.commandedPosition = None
        self.last_joint_state = None
        self.joint_state_sub = None
        self.joint_trajectory_sub = None
        self.pub_seq = 0
    def initialize(self,init_ros=True):
        if init_ros:
            global ros_initialized
            ros_initialized = True
            rospy.init_node('klampt_RosRobotInterface')
        self.joint_state_sub = rospy.Subscriber(self.joint_state_sub_topic, JointState,self.jointStateCallback)
        self.joint_trajectory_pub = rospy.Publisher(self.joint_trajectory_pub_topic, JointTrajectory)
    def close(self):
        self.joint_state_sub.unregister()
        self.joint_state_sub = None
        self.joint_trajectory_pub.unregister()
        self.joint_trajectory_pub = None
    def jointStateCallback(self,jointState):
        if len(jointState.names) > self.robot.numLinks():
            raise RuntimeError("Invalid number of links")
        for n in jointState.names:
            if n not in self.link_dict:
                raise RuntimeError("Invalid link "+n+", must match Klamp't model")
        self.last_joint_state = jointState
    def klamptModel(self):
        return self.robot
    def clock(self):
        return rospy.get_rostime().to_sec()
    def status(self):
        return 'ok'
    def commandedPosition(self):
        return self.commandedPosition
    def sensedPosition(self):
        js = self.last_joint_state
        if js is None: return None
        q = self.robot.getConfig()
        for (v,n) in zip(js.position,js.names):
            q[self.link_dict[n]] = v
        return self.configFromKlampt(q)
    def sensedVelocity(self):
        js = self.last_joint_state
        if js is None: return None
        if len(js.velocity) == 0: return None
        dq = self.robot.getVelocity()
        for (v,n) in zip(js.velocity,js.names):
            dq[self.link_dict[n]] = v
        return self.velocityFromKlampt(dq)
    def sensedTorque(self):
        js = self.last_joint_state
        if js is None: return None
        if len(js.effort) == 0: return None
        t = [0.0]*self.robot.numLinks()
        for (v,n) in zip(js.effort,js.names):
            t[self.link_dict[n]] = v
        return self.velocityFromKlampt(t)
    def setPiecewiseLinear(self,ts,qs,relative=True):
        if not relative:
            raise NotImplementedError("Can't do non-relative piecewise-linear commands")
        traj = JointTrajectory()
        traj.header.seq = self.pub_seq
        self.pub_seq += 1
        traj.header.stamp = rospy.get_rostime()
        traj.joint_names = self.driver_list
        points = []
        for t,q in zip(ts,qs):
            pt = JointTrajectoryPoint()
            pt.time_from_start = rospy.Duration.from_sec(t)
            pt.positions = q
            points.append(pt)
        traj.points = points
        self.joint_trajectory_pub.publish(traj)

ros_initialized = False

def make(klampt_robot_model):
    """Creates a RosRobotInterface for the given model.
    klampt_robot_model is a RobotModel instance.

    Publishes to '/[robot_name]/joint_trajectory'.
    Subscribes to '/[robot_name]/joint_state'
    """
    global ros_initialized
    robotName = klampt_robot_model.getName()
    joint_trajectory_topic = "/%s/joint_trajectory"%(robotName,)
    joint_states_topic = "/%s/joint_states"%(robotName,)
    interface = RosRobotInterface(klampt_robot_model,joint_states_topic,joint_trajectory_topic)
    return interface
