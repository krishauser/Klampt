"""Launches a relay from ROS messages to Klamp't serial controller messages.
"""
import controller
from roscontroller import *
from serialcontroller import ControllerClient

rospy.init_node('klampt_sim')
#launch a controller to publish the simulation time to ROS, PLUS
#the ros controller
c = controller.MultiController()
c.launch(RosTimeController())
c.launch(RosRobotController(robotName,linkNames))

#launch the serial client to connect to a given host and relay messages from the socket to/from ROS
host = 'localhost'
port = 3456
s = ControllerClient((host,port),c)
asyncore.loop()
