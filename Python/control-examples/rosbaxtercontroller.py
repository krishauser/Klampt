"""Relays messages between ROS controller and Baxter controller in Klamp't
"""

import controller
import rospy
import math
import time
import socket
from roscontroller import ros_initialized,RosTimeController
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from baxter_core_msgs.msg import JointCommand,HeadState,HeadPanCommand,AssemblyState
from std_msgs.msg import Bool
from serialcontroller import SerialController
import json

POSITION_MODE=1
VELOCITY_MODE=2
TORQUE_MODE=3
RAW_POSITION_MODE=4

class RosBaxterController(controller.ControllerBlock):
    """A controller that reads from the ROS topics
    
    '/[robot_name]/limb/right/joint_command' (JointCommand)
    '/[robot_name]/limb/left/joint_command' (JointCommand)
    '/[robot_name]/head/command_head_pan' (HeadPanCommand)
    '/[robot_name]/head/command_head_nod' (Bool)
    
    and writes the Baxter-specific ROS topics

    '/[robot_name]/state' (AssemblyState)
    '/[robot_name]/joint_states' (JointState)
    '/[robot_name]/head/head_state' (HeadState)    

    As used by baxter's ROS package, '[robot_name]' should be set to 'robot'.
    """
    def __init__(self,robot_model,robot_name='robot'):
        self.robot_model = robot_model

        #emulate the robot state
        self.state = AssemblyState()
        self.state.enabled = True
        self.state.stopped = False
        self.state.error = False
        self.state.estop_button = AssemblyState.ESTOP_BUTTON_UNPRESSED
        self.state.estop_source = AssemblyState.ESTOP_SOURCE_NONE

        self.names_klampt_to_baxter = {'left_upper_shoulder':'left_s0', 'left_lower_shoulder':'left_s1', 
                            'left_upper_elbow':'left_e0', 'left_lower_elbow':'left_e1',
                            'left_upper_forearm':'left_w0', 'left_lower_forearm':'left_w1', 'left_wrist':'left_w2',
                            'right_upper_shoulder':'right_s0', 'right_lower_shoulder':'right_s1', 
                            'right_upper_elbow':'right_e0', 'right_lower_elbow':'right_e1',
                            'right_upper_forearm':'right_w0', 'right_lower_forearm':'right_w1', 'right_wrist':'right_w2',
                            'head':'head_pan','screen':'head_nod'}
        for l in range(self.robot_model.numLinks()):
            klamptName = self.robot_model.link(l).getName()
            if klamptName not in self.names_klampt_to_baxter:
                self.names_klampt_to_baxter[klamptName] = klamptName
        self.joint_state = JointState()
        self.joint_state.name = [self.names_klampt_to_baxter[self.robot_model.getDriver(d).getName()] for d in range(self.robot_model.numDrivers())]
        self.joint_state.position     = []
        self.joint_state.velocity     = []
        self.joint_state.effort       = []

        self.head_state = HeadState()
        self.head_state.pan = None
        self.head_state.isPanning = False
        self.head_state.isNodding = False
        self.head_pan_link_index = self.robot_model.link("head").index
        self.head_nod_link_index = self.robot_model.link("screen").index
        self.head_pan_driver_index = self.robot_model.getDriver("head").index
        self.head_nod_driver_index = self.robot_model.getDriver("screen").index
        self.head_nod_start_time = None

        # fast indexing structure for partial commands
        self.nameToDriverIndex = dict(zip(self.joint_state.name,range(len(self.joint_state.name))))
        self.nameToLinkIndex = dict((self.names_klampt_to_baxter[self.robot_model.link(l).getName()],l) for l in range(self.robot_model.numLinks()))

        # Setup publisher of robot states
        self.pub_s = rospy.Publisher("/%s/state"%(robot_name,), AssemblyState)
        self.pub_js = rospy.Publisher("/%s/joint_states"%(robot_name,), JointState)
        self.pub_hs = rospy.Publisher("/%s/head/head_state"%(robot_name,), HeadState)
        
        # set up the command subscriber
        self.currentLeftArmMsg = None
        self.currentRightArmMsg = None
        self.currentHeadPanMsg = None
        self.currentHeadNodMsg = None
        rospy.Subscriber('/%s/limb/left/joint_command'%(robot_name), JointCommand,self.leftArmCallback)
        rospy.Subscriber('/%s/limb/right/joint_command'%(robot_name), JointCommand,self.rightArmCallback)
        rospy.Subscriber('/%s/head/command_head_pan'%(robot_name), HeadPanCommand,self.headPanCallback)
        rospy.Subscriber('/%s/head/command_head_nod'%(robot_name), Bool,self.headNodCallback)
        return

    def leftArmCallback(self,msg):
        self.currentLeftArmMsg = msg
        return

    def rightArmCallback(self,msg):
        self.currentRightArmMsg = msg
        return

    def headPanCallback(self,msg):
        self.currentHeadPanMsg = msg
        return

    def headNodCallback(self,msg):
        print "Got head nod message",msg.data
        self.currentHeadNodMsg = msg
        return

    def advance(self,**inputs):       
        res = {}
        if self.currentRightArmMsg != None or self.currentLeftArmMsg != None:
            if self.currentRightArmMsg != None and self.currentLeftArmMsg != None:
                if (self.currentRightArmMsg.mode == TORQUE_MODE) != (self.currentLeftArmMsg.mode == TORQUE_MODE):
                    print "RosBaxterController: Two-arm messages may not mix torque and velocity/position control"
                    return res
            mode = None
            if self.currentRightArmMsg != None:
                mode = self.currentRightArmMsg.mode
            if self.currentLeftArmMsg != None:
                mode = self.currentLeftArmMsg.mode

            if mode == TORQUE_MODE:
                value = [0.0]*self.robot_model.numLinks()
                if self.currentRightArmMsg:
                    for n,cmd in zip(self.currentRightArmMsg.names,self.currentRightArmMsg.command):
                        value[self.nameToLinkIndex[n]] = cmd
                if self.currentLeftArmMsg:
                    for n,cmd in zip(self.currentLeftArmMsg.names,self.currentLeftArmMsg.command):
                        value[self.nameToLinkIndex[n]] = cmd
            else:
                if self.currentRightArmMsg:
                    if self.currentRightArmMsg.mode == POSITION_MODE:
                        self.setPositionCommands(inputs,res,self.currentRightArmMsg.names,self.currentRightArmMsg.command)
                    else:
                        self.setVelocityCommands(inputs,res,self.currentRightArmMsg.names,self.currentRightArmMsg.command)
                if self.currentLeftArmMsg:
                    if self.currentLeftArmMsg.mode == POSITION_MODE:
                        self.setPositionCommands(inputs,res,self.currentLeftArmMsg.names,self.currentLeftArmMsg.command)
                    else:
                        self.setVelocityCommands(inputs,res,self.currentLeftArmMsg.names,self.currentLeftArmMsg.command)

        #sense the configuration and velocity, possibly the effort
        self.joint_state.header.stamp = rospy.get_rostime()
        if 'q' in inputs:
            self.robot_model.setConfig(inputs['q'])
            self.joint_state.position = [self.robot_model.getDriver(d).getValue() for d in range(self.robot_model.numDrivers())]
        if 'dq' in inputs:
            self.robot_model.setVelocity(inputs['dq'])
            self.joint_state.velocity = [self.robot_model.getDriver(d).getVelocity() for d in range(self.robot_model.numDrivers())]
        if 'torque' in inputs:
            self.joint_state.effort = inputs['torque']
            if(len(self.joint_state.effort) != len(self.joint_state.name)):
                print "Uh... input[torque] has length",len(inputs['torque']),"while names has length",len(self.joint_state.name)

        #head actuator simulation
        if self.head_state.pan == None:
            self.head_state.pan = inputs['q'][self.head_pan_link_index]
        if self.currentHeadPanMsg != None:
            #simulate a linear ramp to desired
            velmax = self.robot_model.getVelocityLimits()[self.head_pan_link_index]
            dt = inputs['dt']
            speed = 0
            if self.currentHeadPanMsg.target < self.head_state.pan:
                speed = -self.currentHeadPanMsg.speed*0.01*velmax
                if self.head_state.pan + speed*dt <= self.currentHeadPanMsg.target:
                    #reached destination
                    self.head_state.pan = self.currentHeadPanMsg.target
                    self.head_state.isPanning = False
                else:
                    self.head_state.pan += speed*dt
                    self.head_state.isPanning = True
            elif self.currentHeadPanMsg.target > self.head_state.pan:
                speed = self.currentHeadPanMsg.speed*0.01*velmax
                if self.head_state.pan + speed*dt >= self.currentHeadPanMsg.target:
                    #reached destination
                    self.head_state.pan = self.currentHeadPanMsg.target
                    self.head_state.isPanning = False
                else:
                    self.head_state.pan += speed*dt
                    self.head_state.isPanning = True
            else:
                self.head_state.isPanning = False
            print "Pan target:",self.head_state.pan
            self.setPositionCommand(inputs,res,self.head_pan_link_index,self.head_state.pan)
        donod = False
        if self.currentHeadNodMsg != None:
           donod = self.currentHeadNodMsg.data 
        if donod or self.head_state.isNodding:
            #how long the whole cycle takes
            nodDuration = 1.0
            #negative to start going down first
            nodAmplitude = -0.1
            if not self.head_state.isNodding:
                self.head_nod_start_time = inputs['t']
                self.head_state.isNodding = True
            #make a sinusoid
            nodJointAngle = math.sin((inputs['t']-self.head_nod_start_time)/nodDuration*math.pi*2.0)*nodAmplitude
            if inputs['t'] - self.head_nod_start_time > nodDuration:
                nodJointAngle = 0
                self.head_state.isNodding = False
            print "Nod target:",self.head_state.pan
            self.setPositionCommand(inputs,res,self.head_nod_link_index,nodJointAngle)

        print "Output to Klampt:",res
        #publish the ros topics
        self.pub_s.publish(self.state)
        self.pub_js.publish(self.joint_state)
        self.pub_hs.publish(self.head_state)
        return res

    def setPositionCommand(self,inputs,res,index,value):
        """Given a return dictionary res for the output() function, sets a joint command for a single
        link, qdes[index]=value.  If a velocity command is already given, this will figure out how to
        go there.  Index can also be a joint name."""
        if isinstance(index,str):
            #perform the mapping automatically
            index = self.nameToLinkIndex[index]
        #klampt can only do uniform position, velocity, or torque commands
        if 'qcmd' in res:
            res['qcmd'][index] = value
        elif 'dqcmd' in res:
            res['dqcmd'][index] = (value - inputs['qcmd'][index]) / inputs['dt']
            res['tcmd']=inputs['dt']
        elif 'torquecmd' in res:
            print "Cannot combine joint position commands with joint torque commands"
        else:
            #no joint commands set yet, set a position command
            res['qcmd'] = inputs['qcmd']
            res['qcmd'][index] = value

    def setVelocityCommand(self,inputs,res,index,value):
        """Given a return dictionary res for the output() function, sets a joint command for a single
        link, dqdes[index]=value.  If a position command is already given, this will figure out how to
        go along the desired velocity for the next cycle.  Index can also be a joint name."""
        if isinstance(index,str):
            #perform the mapping automatically
            index = self.nameToLinkIndex[index]
        #klampt can only do uniform position, velocity, or torque commands
        if 'qcmd' in res:
            res['qcmd'][index] = inputs['qcmd'][index] + value*inputs['dt']
        elif 'dqcmd' in res:
            res['dqcmd'][index] = value
            res['tcmd']=inputs['dt']
        elif 'torquecmd' in res:
            print "Cannot combine joint velocity commands with joint torque commands"
        else:
            #no joint commands set yet, set a position command
            res['dqcmd'] = inputs['dqcmd']
            res['dqcmd'][index] = value

    def setPositionCommands(self,inputs,res,indices,values):
        """Given a return dictionary res for the output() function, sets a joint command for a multiple
        links, qdes[index]=value for (index,value) in zip(indices,values). 
        If a velocity command is already given, this will figure out how to go there.
        Indices can also be joint names."""
        #klampt can only do uniform position, velocity, or torque commands
        if 'qcmd' in res:
            for index,value in zip(indices,values):
                if isinstance(index,str):
                    #perform the mapping automatically
                    index = self.nameToLinkIndex[index]
                res['qcmd'][index] = value
        elif 'dqcmd' in res:
            for index,value in zip(indices,values):
                if isinstance(index,str):
                    #perform the mapping automatically
                    index = self.nameToLinkIndex[index]
                res['dqcmd'][index] = (value - inputs['qcmd'][index]) / inputs['dt']
            res['tcmd']=inputs['dt']
        elif 'torquecmd' in res:
            print "Cannot combine joint position commands with joint torque commands"
        else:
            #no joint commands set yet, set a position command
            res['qcmd'] = inputs['qcmd']
            for index,value in zip(indices,values):
                if isinstance(index,str):
                    #perform the mapping automatically
                    index = self.nameToLinkIndex[index]
                res['qcmd'][index] = value
    def setVelocityCommands(self,inputs,res,indices,values):
        """Given a return dictionary res for the output() function, sets a joint command for a multiple
        links, dqdes[index]=value for (index,value) in zip(indices,values). 
        If a position command is already given, this will figure out how to move along the desired velocity
        over the next cycle.
        Indices can also be joint names."""
        #klampt can only do uniform position, velocity, or torque commands
        if 'qcmd' in res:
            for index,value in zip(indices,values):
                if isinstance(index,str):
                    #perform the mapping automatically
                    index = self.nameToLinkIndex[index]
                res['qcmd'][index] = inputs['qcmd'][index]+value*inputs['dt']
        elif 'dqcmd' in res:
            for index,value in zip(indices,values):
                if isinstance(index,str):
                    #perform the mapping automatically
                    index = self.nameToLinkIndex[index]
                res['dqcmd'][index] = value
            res['tcmd']=inputs['dt']
        elif 'torquecmd' in res:
            print "Cannot combine joint velocity commands with joint torque commands"
        else:
            #no joint commands set yet, set a position command
            res['dqcmd'] = inputs['dqcmd']
            for index,value in zip(indices,values):
                if isinstance(index,str):
                    #perform the mapping automatically
                    index = self.nameToLinkIndex[index]
                res['dqcmd'][index] = value


class KlamptSerialBaxterController(SerialController):
    """A controller that reads from the ROS topics

    '/[robot_name]/state' (AssemblyState)
    '/[robot_name]/joint_states' (JointState)
    '/[robot_name]/head/head_state' (HeadState)    

    into a Klamp't input message, and writes it to the given socket.
    It then reads the replied Klamp't control messages from the socket and
    translates the messages to the Baxter-specific ROS topics:

    '/[robot_name]/limb/right/joint_command' (JointCommand)
    '/[robot_name]/limb/left/joint_command' (JointCommand)
    '/[robot_name]/head/command_head_pan' (HeadPanCommand)
    '/[robot_name]/head/command_head_nod' (Bool)

    As used by baxter's ROS package, '[robot_name]' should be set to 'robot'.
    """
    def __init__(self,addr,robot_model,robot_name='robot'):
        SerialController.__init__(self,addr)
        #defined in SerialController       
        self.robot_model = robot_model

        self.baxter_larm_names = ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2']
        self.baxter_rarm_names = ['right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2']
        self.names_klampt_to_baxter = {'left_upper_shoulder':'left_s0', 'left_lower_shoulder':'left_s1', 
                            'left_upper_elbow':'left_e0', 'left_lower_elbow':'left_e1',
                            'left_upper_forearm':'left_w0', 'left_lower_forearm':'left_w1', 'left_wrist':'left_w2',
                            'right_upper_shoulder':'right_s0', 'right_lower_shoulder':'right_s1', 
                            'right_upper_elbow':'right_e0', 'right_lower_elbow':'right_e1',
                            'right_upper_forearm':'right_w0', 'right_lower_forearm':'right_w1', 'right_wrist':'right_w2',
                            'head':'head_pan','screen':'head_nod'}
        for l in range(self.robot_model.numLinks()):
            klamptName = self.robot_model.link(l).getName()
            if klamptName not in self.names_klampt_to_baxter:
                self.names_klampt_to_baxter[klamptName] = klamptName
        self.baxterDriverNames = [self.names_klampt_to_baxter[self.robot_model.getDriver(d).getName()] for d in range(self.robot_model.numDrivers())]
        self.head_pan_link_index = self.robot_model.link("head").index
        self.head_nod_link_index = self.robot_model.link("screen").index

        self.larm_command = JointCommand()
        self.rarm_command = JointCommand()
        self.hpan_command = HeadPanCommand()
        self.hnod_command = Bool()

        # fast indexing structure for partial commands
        self.nameToDriverIndex = dict(zip(self.baxterDriverNames,range(len(self.baxterDriverNames))))
        self.nameToLinkIndex = dict((self.names_klampt_to_baxter[self.robot_model.link(l).getName()],l) for l in range(self.robot_model.numLinks()))

        # Setup publisher of robot states
        self.currentTime = None
        self.lastUpdateTime = None
        self.currentAssemblyState = None
        self.currentJointStates = None
        self.currentHeadState = None
        rospy.Subscriber("/%s/state"%(robot_name,), AssemblyState, self.assemblyStateCallback)
        rospy.Subscriber("/%s/joint_states"%(robot_name,), JointState, self.jointStatesCallback)
        rospy.Subscriber("/%s/head/head_state"%(robot_name,), HeadState, self.headStateCallback)
        
        self.pub_larm = rospy.Publisher('/%s/limb/left/joint_command'%(robot_name), JointCommand)
        self.pub_rarm = rospy.Publisher('/%s/limb/right/joint_command'%(robot_name), JointCommand)
        self.pub_hpan = rospy.Publisher('/%s/head/command_head_pan'%(robot_name), HeadPanCommand)
        self.pub_hnod = rospy.Publisher('/%s/head/command_head_nod'%(robot_name), Bool)
        return

    def assemblyStateCallback(self,msg):
        self.currentAssemblyState = msg
        return

    def jointStatesCallback(self,msg):
        self.currentJointStates = msg
        return

    def headStateCallback(self,msg):
        self.currentHeadState = msg
        return

    def process(self):
        if self.currentAssemblyState==None:
            #not enabled, return
            print "Waiting for /robot/state..."
            time.sleep(1.0)
            return
        if self.currentAssemblyState.stopped:
            print "Robot is stopped"
            time.sleep(1.0)
            return
        if self.currentJointStates==None:
            print "Waiting for joint states to be published..."
            time.sleep(1.0)
            return
        if self.lastUpdateTime == None:
            self.lastUpdateTime = self.currentJointStates.header.stamp
            return
        self.currentTime = self.currentJointStates.header.stamp

        #convert to Klamp't message
        klamptInput = dict()
        klamptInput['t'] = self.currentTime.to_sec()
        klamptInput['dt'] = self.currentTime.to_sec() - self.lastUpdateTime.to_sec()
        if klamptInput['dt'] == 0.0:
            #no new clock messages read
            return
        klamptInput['q'] = [0.0]*self.robot_model.numLinks()
        klamptInput['dq'] = [0.0]*self.robot_model.numLinks()
        klamptInput['torque'] = [0.0]*self.robot_model.numDrivers()
        for i,n in enumerate(self.currentJointStates.name):
            if n not in self.nameToLinkIndex:
                if n != 'torso_t0':
                    print "Ignoring state of link",n
                continue
            klamptIndex = self.nameToLinkIndex[n]
            klamptInput['q'][klamptIndex] = self.currentJointStates.position[i]
            if len(self.currentJointStates.velocity) > 0:
                klamptInput['dq'][klamptIndex] = self.currentJointStates.velocity[i]
            if len(self.currentJointStates.effort) > 0:
                driverIndex = self.nameToDriverIndex[n]
                klamptInput['torque'][driverIndex] = self.currentJointStates.effort[i]
        #if self.currentHeadState != None:
        #    klamptInput['q'][self.head_pan_link_index] = self.currentHeadState.pan

        #output is defined in SerialController
        klamptReply = self.output(**klamptInput)
        if klamptReply == None:
            return False

        #now conver the Klamp't reply to a Baxter command
        lcmd = JointCommand()
        rcmd = JointCommand()
        lcmd.names = self.baxter_larm_names
        rcmd.names = self.baxter_rarm_names
        hpcmd = HeadPanCommand()
        hncmd = Bool()
        if 'qcmd' in klamptReply:
            #use position mode
            lcmd.mode = rcmd.mode = POSITION_MODE
            q = klamptReply['qcmd']
            lcmd.command = [q[self.nameToLinkIndex[n]] for n in lcmd.names]
            rcmd.command = [q[self.nameToLinkIndex[n]] for n in rcmd.names]
            hpcmd.target = q[self.head_pan_link_index]
            hpcmd.speed = q[self.head_pan_link_index]
        elif 'dqcmd' in klamptReply:
            lcmd.mode = rcmd.mode = VELOCITY_MODE
            dq = klamptReply['dqcmd']
            lcmd.command = [dq[self.nameToLinkIndex[n]] for n in lcmd.names]
            rcmd.command = [dq[self.nameToLinkIndex[n]] for n in rcmd.names]

            hpcmd = None
        elif 'torquecmd' in klamptReply:
            lcmd.mode = rcmd.mode = TORQUE_MODE
            torque = klamptReply['torquecmd']
            lcmd.command = [torque[self.nameToDriverIndex[n]] for n in lcmd.names]
            rcmd.command = [torque[self.nameToDriverIndex[n]] for n in rcmd.names]

            hpcmd = None
        else:
            lcmd = rcmd = None
            hpcmd = None
        hncmd = None

        if self.currentAssemblyState.estop_button != AssemblyState.ESTOP_BUTTON_UNPRESSED:
            print "Estop is on..."
            time.sleep(1.0)
        elif not self.currentAssemblyState.enabled:
            print "Waiting for robot to turn on..."
            time.sleep(1.0)
        else:
            #publish the messages
            if lcmd:
                self.pub_larm.publish(lcmd)
            if rcmd:
                self.pub_rarm.publish(rcmd)
            if hpcmd:
                self.pub_hpan.publish(hpcmd) 
            if hncmd:
                self.pub_hnod.publish(hncmd)
        self.lastUpdateTime = self.currentTime
        return True

    def run(self,rate=100):
        self.accept()
        delay = 1.0/rate
        try:
            while True:
                res = self.process()
                if res == False:
                    print "Connection closed... stopping robot and going back to waiting state"
                    self.stopMoving()
                    self.accept()
                time.sleep(delay)
        except socket.error:
            print "Klampt controller disconnected"
            self.stopMoving()
            self.close()
            raise
        except Exception as e:
            self.stopMoving()
            self.close()
            raise
        except KeyboardInterrupt:
            print "Closing... stopping robot"
            self.stopMoving()
            self.close()
            
    def stopMoving(self):
        lcmd = JointCommand()
        rcmd = JointCommand()
        lcmd.names = self.baxter_larm_names
        rcmd.names = self.baxter_rarm_names
        lcmd.mode = rcmd.mode = POSITION_MODE
        lcmd.command = []
        for n in lcmd.names:
            index = self.currentJointStates.name.index(n)
            lcmd.command.append(self.currentJointStates.position[index])
        rcmd.command = []
        for n in rcmd.names:
            index = self.currentJointStates.name.index(n)
            rcmd.command.append(self.currentJointStates.position[index])
        self.pub_larm.publish(lcmd)
        self.pub_rarm.publish(rcmd)
        self.pub_hnod.publish(False)
            
            
def make(klampt_robot_model):
    global ros_initialized
    if not ros_initialized:
        ros_initialized = True
        rospy.init_node('klampt_sim')
        #launch a controller to publish the simulation time to ROS, PLUS
        #the robot's controller
        c = controller.MultiController()
        c.launch(RosTimeController())
        c.launch(RosBaxterController(klampt_robot_model))
        return c
    #just launch the robot's controller, some other RosTimeController has been
    #launched before
    return RosBaxterController(klampt_robot_model)
