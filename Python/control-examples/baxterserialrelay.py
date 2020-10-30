#!/usr/bin/env python
"""Relays messages between ROS controller and simulated Baxter robot in Klamp't OR
between Klamp't serial controller and ROS Baxter robot.
"""

import rosbaxtercontroller
from klampt.control.io.serialcontroller import ControllerClient
from klampt import *
import asyncore
import rospy
import argparse

def mainKlamptControllerToRosRobot(klampt_robot_model_fn,klampt_serial_port):
    """Relays Klampt controller messages to and from a ROS Baxter robot"""
    #load robot file
    rospy.init_node('klampt_controller')
    world = WorldModel()
    world.enableGeometryLoading(False)
    res = world.readFile(klampt_robot_model_fn)
    if not res: 
        print 'Error, could not load klampt model from',klampt_robot_model_fn
        exit(1)
    if world.numRobots()==0:
        print 'Error, klampt model',klampt_robot_model_fn,'did not contain a robot'
        exit(1)
    klampt_robot_model = world.robot(0)
    print "Load successful"

    print "Running Klamp't controller -> Baxter robot relay..."
    controller = rosbaxtercontroller.KlamptSerialBaxterController(('localhost',klampt_serial_port),klampt_robot_model)
    controller.run()

def mainRosControllerToKlamptRobot(klampt_robot_model_fn,klampt_serial_port):
    """Relays ROS Baxter controller messages to and from Klamp't simulated robot"""
    rospy.init_node('klampt_sim')

    #load robot file
    world = WorldModel()
    world.enableGeometryLoading(False)
    res = world.readFile(klampt_robot_model_fn)
    if not res: 
        print 'Error, could not load klampt model from',klampt_robot_model_fn
        exit(1)
    if world.numRobots()==0:
        print 'Error, klampt model',klampt_robot_model_fn,'did not contain a robot'
        exit(1)
    klampt_robot_model = world.robot(0)
    print "Load successful"

    #print some info
    robotName = klampt_robot_model.getName()
    linkNames = [klampt_robot_model.link(i).getName() for i in range(klampt_robot_model.numLinks())]
    print "Running controller listening on topic /%s/limb/right/joint_command and"%(robotName,)
    print "and /%s/limb/left/joint_command andd publishing on topic"%(robotName,)
    print "/%s/joint_states"%(robotName,)
    print "Klamp't link names are:",linkNames

    #advertise version 1.0.0 of the Baxter software
    print "Emulating ROS Baxter API version 1.0.0"
    rospy.set_param('/rethink/software_version', '1.0.0')

    #create the ROS controller
    c = rosbaxtercontroller.make(klampt_robot_model)

    #launch the serial client to connect to a given host and relay messages from the socket to/from ROS
    host = 'localhost'
    port = klampt_serial_port
    s = ControllerClient((host,port),c)

    print "Running Baxter controller -> Klamp't robot relay..."
    try:
        asyncore.loop()
    except KeyboardInterrupt:
        print "Ctrl+C pressed, exiting..."


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run a Baxter controller <-> Klampt simulator or Baxter robot <-> Klampt controller.')
    parser.add_argument('-m','--mode',default='b2k',nargs='?',help='For Baxter controller to klampt simulator use b2k, otherwise use k2b')
    parser.add_argument('-p','--port',default=None,nargs='?',help="Set the Klamp't port (default 3456)")
    parser.add_argument('-r','--robot',default=None,nargs='?',help="Set the Klamp't robot model")
    args = parser.parse_args()

    #read klampt_robot_file and optionally klampt_serial_port from parameter server
    klampt_serial_port = args.port
    klampt_robot_model_fn = args.robot
    if klampt_robot_model_fn == None:
        try:
            klampt_robot_model_fn = rospy.get_param('/klampt_robot_file')
        except KeyError:
            print 'Error, ROS parameter "/klampt_model_name" doesn\'t exist.'
            print 'Set this using rosparam set klampt_model_name [KLAMPT .rob FILE]'
            exit(1)
    if klampt_serial_port == None:
        try:
            klampt_serial_port = rospy.get_param('/klampt_serial_port')
            print "Using serial port",klampt_serial_port,"from parameter /klampt_serial_port"
        except KeyError:
            klampt_serial_port = 3456
            print "Using serial port 3456 by default, use rosparam set"
            print "klampt_serial_port [PORT] if you want to change this."


    if args.mode == 'b2k':
        mainRosControllerToKlamptRobot(klampt_robot_model_fn,klampt_serial_port)
    elif args.mode == 'k2b':
        mainKlamptControllerToRosRobot(klampt_robot_model_fn,klampt_serial_port)
    else:
        raise ValueError("Invalid mode, must be either b2k or k2b")
