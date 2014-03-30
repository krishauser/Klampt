import math

xmin = (0,0,-55,0,0)
xmax = (70,90,43,255,255)
g1max = 140
g2max = 100
m1 = float(xmax[0])/g1max
m2 = float(xmax[1])/g2max

def get_finger_angles(x,m):
    """Gets the finger angles (th1,th2,th3) from a hybrid state"""
    return x[0:3]

def get_finger_proximal_angle(x,m):
    """Gets the finger angle th1 from a hybrid state"""
    return x[0]

def get_finger_medial_angle(x,m):
    """Gets the finger angle th2 from a hybrid state"""
    return x[1]

def get_finger_distal_angle(x,m):
    """Gets the finger angle th3 from a hybrid state"""
    return x[2]

def get_finger_motor_position(x,m):
    """Gets the drive motor position from a hybrid state"""
    return x[3]

def get_finger_g(x,m):
    """Gets the drive command from a hybrid state"""
    return x[4]

def get_finger_contact_state(x,m):
    """Gets the contact state of the finger from a hybrid state"""
    return m[0:3]

def get_finger_joint_limit_state(x,m):
    """Gets the joint limit state of the finger from a hybrid state"""
    return m[3:6]

def finger_hybrid_initial_state():
    """Returns the initial hybrid state and mode (x0,m0) of a finger.
    state is x=(theta1,theta2,theta3,thetamotor,g) and m=(c1,c2,c3,l1,l2,l3)
    where c1,c2,c3 indicate contact states 0 or 1, and l1,l2,l3 indicate upper
    joint limit hit +1, lower joint limit hit -1, or neither 0."""
    return ([0,0,0,0,0],[0,0,0,0,0,0])

def finger_hybrid_model(x,m,gincrement,madeContact):
    """Input: a hybrid state x, mode m, increment of the g variable (-1, 0,
    or 1), and signals madeContact for having made contact on each finger
    at the last time step (a triple).
    Returns the new hybrid state and mode (x',m').
    """
    global xmin,xmax,m1,m2
    assert abs(gincrement) <= 1,"Increment must be between -1 and 1"
    xnext = x[:]
    mnext = m[:]
    for i in range(3):
        if madeContact[i]:
            mnext[i] = 1
    th1,th2,th3,gmotor,g = x
    if g + gincrement > 255:
        return xnext,mnext
    if g + gincrement < 0:
        return xnext,mnext
    #un-break third link if the desired position is lower than the current
    if (m[5]==1 or m[2]==1) and g+gincrement < gmotor:
        #print "un-breaking third link at g =",g
        mnext[2] = mnext[5]=0
    if m[5]==-1 and gincrement < 0 and x[0]+x[1] < -xmin[2]:
        #print "un-breaking third link lower limit at g =",g
        #opening gripper...
        mnext[5] = 0
    if m[5]==-1 and gincrement > 0 and x[1] >= xmax[1] or madeContact[1]:
        #print "un-breaking third link at g =",g
        #closing gripper
        mnext[5] = 0
    stop1 = (mnext[0]!=0 or mnext[3]!=0)
    stop2 = (mnext[1]!=0 or mnext[4]!=0)
    stop3 = (mnext[2]!=0 or mnext[5]==1)
    dx = [0,0,0,0,gincrement]
    if stop3:
        dx = [0,0,0,0,gincrement]
    elif stop2:
        theta3inc = (xmax[2]-xmin[2])/(256.0-gmotor-gincrement)
        dx = [0,0,theta3inc*gincrement,gincrement,gincrement]
    elif stop1:
        dx = [0,m2*gincrement,-m2*gincrement,gincrement,gincrement]
    else:
        dx = [m1*gincrement,0,-m1*gincrement,gincrement,gincrement]
    if mnext[5]==-1:
        dx[2] = 0

    if mnext[5]==1:
        assert(dx[3]==0)

    #figure out mode changes
    if gincrement < 0:
        #test for breaking contact or leaving maximum
        for i in range(2):
            if m[i]==1 and not madeContact[i]:
                if xnext[i+1] <= xmin[i+1]:
                    mnext[i]=0
            if m[i+3]==1 and not madeContact[i]:
                if xnext[i+1] <= xmin[i+1]:
                    mnext[i+3]=0
    if mnext[5]==1:
        assert(dx[3]==0)
        assert xnext[3]==x[3]
    for i in range(3):
        #hit upper joint limit
        if x[i] + dx[i] > xmax[i]:
            #print "hit upper joint limit on link",i+1,"at g =",g
            mnext[i+3] = 1
        #releasing from upper joint limit
        if mnext[i+3]==1 and x[i] + dx[i] < xmax[i]:
            mnext[i+3]=0
    if mnext[5]==0:
        #distal link hits lower joint limit
        if x[2] + dx[2] < xmin[2]:
            #print "hit lower joint limit on link",3,"at g =",g
            mnext[5] = -1
    for i in range(len(x)):
        xnext[i] = max(xmin[i],min(x[i] + dx[i],xmax[i]))
    return (xnext,mnext)

def theta3func(g,ofs):
    c = xmax[2]-xmin[2]
    return min(xmax[2],xmin[2]+c*(g-ofs)/(255-g))

def quasistatic_finger_model(gob,g):
    """Given input:
    - gob: values of g where the link is pressed against an obstacle
      (None if no obstacle is hit)
    - and a desired g value,
    Returns the triple of joint angles (th1,th2,th3).
    """
    global xmin,xmax,m1,m2
    free1 = (gob[0] == None or g < gob[1])
    free2 = (gob[1] == None or g < gob[1])
    free3 = (gob[2] == None or g < gob[1])
    th3max = (xmax[2] if free3 else gob[2])
    #if link 3 hit, then the finger is stopped at gob[2]
    if not free3: g = gob[2]
    if free1 and free2:
        if g <= 110:
            return (m1*g,0,-m1*g)
        elif g <= 140:
            return (m1*g,0,xmin[2])
        elif g <= 240:
            return (xmax[0],m2*(g-140),xmin[2])
        else:
            return (xmax[0],xmax[1],xmin[2])
    elif free2:
        assert gob[0] <= 140
        th1 = m1*gob[0]
        if g-gob[0] <= 100:
            th2 = m2*(g-gob[0])
            return (th1,th2,max(-th1-th2,xmin[2]))
        else:
            return (th1,xmax[1],min(th3max,theta3func(g,gob[0]+100)))
    elif free1:
        assert gob[1] <= g
        if gob[1]<=140:
            return (m1*gob[1],0,min(th3max,theta3func(g,gob[1])))
        else:
            return (xmax[1],m2*(gob[1]-140),min(th3max,theta3func(g,gob[1])))
    else:
        #link 1 and 2 in contact
        return (m1*gob[0],m2*(gob[1]-gob[0]),min(theta3func(g,gob[1]),th3max))

class FingerEmulator:
    def __init__(self):
        self.g = 0
        self.gobs = [None,None,None]
        self.gstop = None
        self.forceThreshold = 255
        #TODO: calibrate the map from link errors to force
        #deadband should correspond to an angular error in degrees that replicates
        #the force of a "0" force command value
        self.errorDeadband = 0.5
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
        for i in range(3):
            if self.gobs[i] != None and gdes < self.gobs[i]:
                self.gobs[i] = None
        for i in range(3):
            if self.gobs[i] == None and in_contact[i]:
                self.gobs[i] = self.g
        if self.gstop and self.g < self.gstop:
            #opening motion after full stop
            self.gstop = None
            res = quasistatic_finger_model(self.gobs,self.g)
            print "RobotiQ finger model g:",self.g,"gobs",self.gobs,"gstop",self.gstop,"res",res
            return res
        else:
            #closing motion
            if self.gstop != None:
                print "RobotiQ finger model g:",self.g,"gobs",self.gobs,"gstop",self.gstop,"stopped."
                return q
            #hack -- full stop if link 2 is hit
            #do_stop = in_contact[2]
            kP,errorDeadband,forceThreshold = self.kP,self.errorDeadband,self.forceThreshold
            #print "qactual:",qactual
            #print "q:",q
            do_stop = ((kP[0]*(q[0]-qactual[0])+kP[1]*(q[1]-qactual[1])+kP[2]*(q[2]-qactual[2])-errorDeadband)*kP[3] > forceThreshold)
            if do_stop:
                #print "stopping finger"
                self.gstop = self.g
                print "RobotiQ finger model g:",self.g,"gobs",self.gobs,"gstop",self.gstop,"stopped."
                return q
            res = quasistatic_finger_model(self.gobs,self.g)
            print "RobotiQ finger model g:",self.g,"gobs",self.gobs,"gstop",self.gstop,"res",res
            return res


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

def self_test(fn="robotiq.csv",gob1=None,gob2=None,gob3=None):
    x,m = finger_hybrid_initial_state()
    print "Writing self test to",fn
    f = open(fn,'w')
    f.write("step,th1,th2,th3,gmotor,g,th3ref,c1,c2,c3,l1,l2,l3\n")
    numclose = 260
    for step in range(numclose):
        f.write(str(step)+',')
        """
        #quasistatic model
        c1 = gob1 if (gob1!=None and step >= gob1) else None
        c2 = gob1 if (gob2!=None and step >= gob2) else None
        c3 = gob1 if (gob3!=None and step >= gob3) else None
        q = quasistatic_finger_model((c1,c2,c3),step)
        f.write(','.join(str(s) for s in q))
        f.write('\n')
        """
        #hybrid dynamic model
        f.write(','.join(str(s) for s in x+m))
        f.write('\n')
        c1 = (gob1!=None and step == gob1)
        c2 = (gob2!=None and step == gob2)
        c3 = (gob3!=None and step == gob3)
        (x,m) = finger_hybrid_model(x,m,1,(c1,c2,c3))
        
    for step in range(numclose):
        f.write(str(step+numclose)+',')
        f.write(','.join(str(s) for s in x+m))
        f.write('\n')
        (x,m) = finger_hybrid_model(x,m,-1,(0,0,0))
    f.close()

if __name__=='__main__':
    print quasistatic_finger_model((81,104,None),104)
    exit(1)
    self_test('robotiq_free.csv')
    self_test('robotiq_free_gob30.csv',30)
    self_test('robotiq_free_gob70.csv',70)
    self_test('robotiq_free_gob110.csv',110)
    self_test('robotiq_free_gob200.csv',None,200)
    self_test('robotiq_free_gob2_110.csv',None,110,None)
    self_test('robotiq_free_gob3_110.csv',None,None,110)
