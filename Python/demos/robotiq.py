import math

def mymodel(th1,th2, th3, g, gob, in_contact):
    #g = int(raw_input("introdurre il valore di g: "))
    #gob = int(raw_input("introdurre il valore di gob: "))

    m1 = 70.00/140.00
    m2 = 90.00/100.00
    PI = 3.1415926535

    #case without obstacle
    if not in_contact:
        if 0 <= g <= 120:
            g1 = g
            g2 = 0
            g3 = 0
            th1 = m1*g1
            th2 = m2*g2
            th3 = -th1
        elif 120 < g <= 140:
            g1 = g
            g2 = 0
            g3 = 0
            th1 = m1*g1
            th2 = m2*g2
            th3 = -55.00 - ((140.00-g1)/2.00) 
        elif 140 < g <= 240:
            g1 = 140
            g2 = g-g1
            g3 = 0
            th1 = m1*g1
            th2 = m2*g2
            th3 = -55 
        elif 240 < g <= 255:
            g1 = 140
            g2 = 100
            g3 = g-g2-g1
            th1 = m1*g1
            th2 = m2*g2
            th3 = -55 
        else: 
            print 'valore di g non ammesso, g =',g
    #case with object
    else:
        if 0 <= gob <= 140:
            if g-gob <= 0:
                g1 = g
                g2 = 0
                g3 = 0
                th1 = m1*g1
                th2 = m2*g2
                th3 = -th1
            elif 0 < g-gob <= 100:
                g1 = gob
                g2 = g-gob
                g3 = 0
                th1 = m1*g1
                th2 = m2*g2
                th3 = -(th1+th2) 
                if th3 <= -55:
                    th3 = -55 
            elif 100 < g-gob <= 255:
                g1 = gob
                g2 = 100
                g3 = g-gob-g2
                g3ob = 255-g1-g2-g3
                th1 = m1*g1
                th2 = m2*g2
                th3 = -55+(98/g3ob)*g3
            else: 
                print 'valore di g o gob non ammesso, g-gob =',g-gob
        elif 140 <= gob <= 240:
            if g <= 120:
                g1 = g
                g2 = 0
                g3 = 0
                th1 = m1*g1
                th2 = m2*g2
                th3 = -th1
                if th3 <= -55:
                    th3 = -55
            elif 120 < g <= 140:
                g1 = g
                g2 = 0
                g3 = 0
                th1 = m1*g1
                th2 = m2*g2
                th3 = -55.00 - ((140-g1)/2) 
            elif  g <= gob:
                g1 = 140
                g2 = g-g1
                g3 = 0
                th1 = m1*g1
                th2 = m2*g2
                th3 = -55 
            elif  g > gob:
                g1 = 140
                g2 = gob-g1
                g3 = g-gob-g2
                g3ob = 255-g1-g2-g3
                th1 = m1*g1
                th2 = m2*g2
                th3 = -55+(98/g3ob)*g3 
                if th3 <= -55:
                    th3 = -55
                if th3 >= 43:
                    th3 = 43
        else: 
            print 'valore di g o gob non ammesso, g =',g

    return th1, th2, th3

class FingerEmulator:
    def __init__(self):
        self.g = 0
        self.gobs = None
        self.gstop = None
        self.forceThreshold = 255
        #TODO: calibrate the map from link errors to force
        #deadband should correspond to an angular error in degrees that replicates
        #the force of a "0" force command value
        self.errorDeadband = 0.25
        #first 3 values determine the error in the drive link proportionally
        #to an error in each of these links.  4th value scales from error to
        #force threshold units
        self.kP = [1,1,1,200]
        #TODO: calibrate the speed
        self.dt = 0.01
        self.speed = 255

    def set_speed(self,speed):
        #speed is a value from 0-255
        self.speed = speed

    def set_force(self,force):
        #force is a value from 0-255
        self.forceThreshold = force

    def next_q(self,q,qactual,gdes,in_contact):
        """Given a commanded configuration q, an actual configuration qactual,
        a desired gdes, and a list in_contact denoting whether each link is
        colliding, returns a q for the next time step"""
        dg = gdes - self.g
        #clamp the speed
        dg = max(-self.speed*self.dt,min(dg,self.speed*self.dt))
        self.g += dg
        trigger_gobs = (in_contact[0] or in_contact[1])
        print "ramp interpolator g:",self.g,"gobs",self.gobs,"gstop",self.gstop,"trigger",trigger_gobs
        if self.gstop and self.g < self.gstop:
            #opening motion after full stop
            self.gstop = None
            if self.gobs == None and trigger_gobs:
                self.gobs = self.g
            elif self.gobs != None and not trigger_gobs:
                self.gobs = None
            #TODO: this doesn't work when link 1 and link 2 are in collision
            #and link 2 separates
            return mymodel(q[0],q[1],q[2],self.g,self.gobs,trigger_gobs)
        else:
            #closing motion
            if self.gstop != None:
                return q
            if self.gobs == None and trigger_gobs:
                self.gobs = self.g
            elif self.gobs != None and not trigger_gobs:
                self.gobs = None
            #hack -- full stop if link 2 is hit
            #do_stop = in_contact[2]
            kP,errorDeadband,forceThreshold = self.kP,self.errorDeadband,self.forceThreshold
            print "qactual:",qactual
            print "q:",q
            do_stop = ((kP[0]*(q[0]-qactual[0])+kP[1]*(q[1]-qactual[1])+kP[2]*(q[2]-qactual[2])-errorDeadband)*kP[3] > forceThreshold)
            if do_stop:
                print "stopping finger"
                self.gstop = self.g
                return q
            return mymodel(q[0],q[1],q[2],self.g,self.gobs,trigger_gobs)


class Emulator:
    def __init__(self,sim,robotIndex = 0, handLinkIndex = 0):
        self.sim = sim
        # if you don't enable contact feedback, the contact feedback will not be
        # generated
        sim.enableContactFeedbackAll()
        self.finger_sims = [FingerEmulator() for i in range(3)]

        #get link IDs
        self.scissorlinks = [handLinkIndex+1,handLinkIndex+2]
        self.fingerlinks = [range(handLinkIndex+4,handLinkIndex+7),
                            range(handLinkIndex+7,handLinkIndex+10),
                            range(handLinkIndex+10,handLinkIndex+13)]

        world = sim.getWorld()
        #get the IDs of the gripper links 
        self.fingerids = [[world.robotLink(robotIndex,i).getID() for i in self.fingerlinks[f]] for f in range(3)]
        self.robotIndex = robotIndex

    def get_command(self,g,scissor=None):
        """Emulates the Robotiq given the command g (a 3-tuple of values from
        [0-255]) and the scissor value ([0-255], or None to keep it
        unchanged).
        """
        finger_links_touching = [[self.sim.hadContact(linkj,-1) for linkj in self.fingerids[f]] for f in range(3)]

        controller = self.sim.getController(self.robotIndex)
        desired_joint_angles = controller.getCommandedConfig()
        actual_joint_angles = self.sim.getActualConfig(self.robotIndex)
        #commanded finger configuration
        fingerq = [[desired_joint_angles[i]*180.0/math.pi for i in self.fingerlinks[f]] for f in range(3)]
        #actual finger configurations
        fingerqa = [[actual_joint_angles[i]*180.0/math.pi for i in self.fingerlinks[f]] for f in range(3)]
        #simulate the fingers
        for f in range(3):
            fingerq[f] = self.finger_sims[f].next_q(fingerq[f],fingerqa[f],g[f],finger_links_touching[f])
        
        #read this into the desired joint angles
        for f in range(3):
            for i,link in enumerate(self.fingerlinks[f]):
                desired_joint_angles[link] = fingerq[f][i]/180.0*math.pi
        if scissor != None:
            u = float(scissor)/255.0
            desired_joint_angles[self.scissorlinks[0]] = (-17 + u*34)*math.pi/180.0
            desired_joint_angles[self.scissorlinks[1]] = (17.0 - u*34)*math.pi/180.0*u
        return desired_joint_angles
        
    def send_command(self,g,scissor=None):
        """Sends the command given by g and scissor to the robot controller"""
        qdes = self.get_command(g,scissor)
        controller = self.sim.getController(self.robotIndex)
        controller.setPIDCommand(qdes,[0.0]*len(qdes))
