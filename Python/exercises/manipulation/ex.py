import math
import random
from klampt import *
from klampt import robotcollide
from klampt.glprogram import *
from klampt import vectorops
from klampt import so3,se3
from klampt.cspace import CSpace,MotionPlan
from openhands import openhand

class Hand:
    """Defines basic elements of a hand and its arm.
    Members include hand (either 'l' or 'r'), link (the hand link index),
    localPosition1 and localPosition2 (two points along the middle axis of the hand)
    armIndices (the hand's arm link indices), and fingerIndices (the hand's
    finger link indices).
    """
    def __init__(self,hand='l'):
        self.hand = hand
        if hand[0] == 'l':
            self.link = 13
        else:
            self.link = 34 
        if hand[0] == 'l':
            self.localPosition1 = (-0.03,-0.02,-0.1)
            self.localPosition2 = (0.03,-0.02,-0.1)
        else:
            self.localPosition1 = (-0.03,0.02,-0.1)
            self.localPosition2 = (0.03,0.02,-0.1)
        self.armIndices = range(self.link-5,self.link+1)
        self.fingerIndices = range(self.link+1,self.link+16)

    def open(self,q,amount=1.0):
        """Computes a configuration identical to q but with hand open
        if amount = 1 or closed if amount = 1."""
        return openhand(q,self.hand,amount)

    def ikSolver(self,robot,obj_pt,obj_axis):
        """Returns an IK solver that places the hand center at obj_pt with
        the palm oriented along obj_axis"""
        q = robot.getConfig()
        obj = IKObjective()
        obj.setFixedPoints(self.link,[self.localPosition1,self.localPosition2],[vectorops.madd(obj_pt,obj_axis,-0.03),vectorops.madd(obj_pt,obj_axis,0.03)])
        solver = IKSolver(robot)
        solver.add(obj)
        solver.setActiveDofs(self.armIndices)
        return solver

class Globals:
    def __init__(self,world):
        self.world = world
        self.robot = world.robot(0)
        self.collider = robotcollide.WorldCollider(world)

############################# Problem 1 ##############################

class TransitCSpace(CSpace):
    """A CSpace defining the feasibility constraints of the robot"""
    def __init__(self,globals,hand):
        CSpace.__init__(self)
        self.globals = globals
        self.robot = globals.robot
        self.hand = hand
        #initial whole-body configuratoin
        self.q0 = self.robot.getConfig()
        #setup CSpace sampling range
        qlimits = zip(*self.robot.getJointLimits())
        self.bound = [qlimits[i] for i in self.hand.armIndices]
        #setup CSpace edge checking epsilon
        self.eps = 1e-2

    def feasible(self,x):
        #check arm joint limits
        for (xi,bi) in zip(x,self.bound):
            if xi < bi[0] or xi > bi[1]:
                return False
        #set up whole body configuration to test environment and self collisions
        q = self.q0[:]
        for i,xi in zip(self.hand.armIndices,x):
            q[i] = xi
        self.robot.setConfig(q)
        world = self.globals.world
        collider = self.globals.collider
        #test robot-object collisions
        for o in xrange(world.numRigidObjects()):
            if any(collider.robotObjectCollisions(self.robot.index,o)):
                return False;
        #test robot-terrain collisions
        for o in xrange(world.numTerrains()):
            if any(collider.robotTerrainCollisions(self.robot.index,o)):
                return False;
        #test robot self-collisions
        if any(collider.robotSelfCollisions(self.robot.index)):
            return False
        return True

def planTransit(world,objectIndex,hand):
    globals = Globals(world)
    cspace = TransitCSpace(globals,hand)
    obj = world.rigidObject(objectIndex)
    robot = world.robot(0)
    qmin,qmax = robot.getJointLimits()
    
    #get the start config
    q0 = robot.getConfig()
    q0arm = [q0[i] for i in hand.armIndices]
    if not cspace.feasible(q0arm):
        print "Warning, arm start configuration is infeasible"
    #get the pregrasp config -- TODO: what if the ik solver doesn't work?
    qpregrasp = None
    qpregrasparm = None
    solver = hand.ikSolver(robot,obj.getTransform()[1],[0,0,1])
    print "Trying to find pregrasp config..."
    (res,iters) = solver.solve(100,1e-3);
    if res:
        qpregrasp =  robot.getConfig()
        qpregrasparm = [qpregrasp[i] for i in hand.armIndices]
        if not cspace.feasible(qpregrasparm):
            print "Pregrasp config infeasible"
            return None
    if qpregrasp == None:
        print "Pregrasp solve failed"
        return None

    print "Planning transit motion to pregrasp config..."
    MotionPlan.setOptions(connectionThreshold=5.0,perturbationRadius=0.5)
    planner = MotionPlan(cspace,'sbl')
    planner.setEndpoints(q0arm,qpregrasparm)
    iters = 0
    step = 10
    while planner.getPath()==None and iters < 1000:
        planner.planMore(step)
        iters += step
    if planner.getPath() == None:
        print "Failed finding transit path"
        return None
    print "Success"

    #lift arm path to whole configuration space path
    path = []
    for qarm in planner.getPath():
        path.append(q0[:])
        for qi,i in zip(qarm,hand.armIndices):
            path[-1][i] = qi
    
    #add a path to the grasp configuration
    return path + [hand.open(path[-1],0)]

class GLTransitPlanProgram(GLRealtimeProgram):
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"GLTransitPlanProgram")
        self.world = world
        self.robot = world.robot(0)
        self.qstart = self.robot.getConfig()
        #solution to planning problem
        self.path = None
        self.animationTime = None

    def display(self):
        #draw the world
        self.world.drawGL()
        lh = Hand('l')
        rh = Hand('r')
        glDisable(GL_LIGHTING)
        glPointSize(5.0)
        glDisable(GL_DEPTH_TEST)
        glBegin(GL_POINTS)
        glColor3f(0,1,0)
        glVertex3fv(se3.apply(self.world.robot(0).link(lh.link).getTransform(),lh.localPosition1))
        glVertex3fv(se3.apply(self.world.robot(0).link(rh.link).getTransform(),rh.localPosition1))
        glColor3f(0,0,1)
        glVertex3fv(se3.apply(self.world.robot(0).link(lh.link).getTransform(),lh.localPosition2))
        glVertex3fv(se3.apply(self.world.robot(0).link(rh.link).getTransform(),rh.localPosition2))
        glColor3f(1,0,0)
        glVertex3fv(self.world.rigidObject(0).getTransform()[1])
        glEnd()
        glEnable(GL_DEPTH_TEST)

    def keyboardfunc(self,key,x,y):
        if key == 'l':
            self.robot.setConfig(self.qstart)
            self.path = planTransit(self.world,0,Hand('l'))
            #reset the animation
            self.animationTime = self.ttotal
            glutPostRedisplay()
        elif key == 'r':
            self.robot.setConfig(self.qstart)
            self.path = planTransit(self.world,0,Hand('r'))
            #reset the animation
            self.animationTime = self.ttotal
            glutPostRedisplay()

    def idle(self):
        if self.path:
            #loop the path animation
            u = (self.ttotal - self.animationTime)
            i = int(math.floor(u))
            s = u - i
            i = i%(len(self.path)-1)
            #set the robot configuration for display
            q = vectorops.interpolate(self.path[i],self.path[i+1],s)
            self.robot.setConfig(q)
            glutPostRedisplay()

def run_ex1():
    world = WorldModel()
    res = world.readFile("ex1_file.xml")
    if not res: raise RuntimeError("Unable to load world file")
    GLTransitPlanProgram(world).run()



############################# Problem 2 ##############################


def graspedObjectTransform(robot,hand,qrobot0,Tobj0,qrobot):
    """Given initial robot configuration qrobot0 and object transform Tobj0,
    returns the object transformation corresponding to new configuration
    qrobot assuming the object is rigidly attached to the hand"""
    robot.setConfig(qrobot0)
    Thand0 = robot.link(hand.link).getTransform()
    Tgrasp = se3.mul(se3.inv(Thand0),Tobj0)
    robot.setConfig(qrobot)
    Thand = robot.link(hand.link).getTransform()
    return se3.mul(Thand,Tgrasp)


class TransferCSpace(CSpace):
    def __init__(self,globals,hand,object):
        CSpace.__init__(self)
        self.globals = globals
        self.robot = globals.robot
        self.hand = hand
        self.object = object
        #setup initial object-robot transform
        Thand0 = self.robot.link(hand.link).getTransform()
        Tobj0 = object.getTransform()
        self.Tgrasp = se3.mul(se3.inv(Thand0),Tobj0)
        #initial whole-body configuratoin
        self.q0 = self.robot.getConfig()
        #setup CSpace sampling range
        qlimits = zip(*self.robot.getJointLimits())
        self.bound = [qlimits[i] for i in self.hand.armIndices]
        #setup CSpace edge checking epsilon
        self.eps = 1e-2

    def objectTransform(self,x):
        """Given an arm configuration x, returns the object transformation"""
        q = self.q0[:]
        for i,xi in zip(self.hand.armIndices,x):
            q[i] = xi
        self.robot.setConfig(q)
        Thand = self.robot.link(self.hand.link).getTransform()
        return se3.mul(Thand,self.Tgrasp)

    def feasible(self,x):
        #TODO: fix the transfer constraint testing
        #check arm joint limits
        for (xi,bi) in zip(x,self.bound):
            if xi < bi[0] or xi > bi[1]:
                #print "Out of joint limits"
                return False
        #set up whole body configuration to test environment and self collisions
        q = self.q0[:]
        for i,xi in zip(self.hand.armIndices,x):
            q[i] = xi
        self.robot.setConfig(q)
        world = self.globals.world        
        collider = self.globals.collider
        #test robot-object collisions
        for o in xrange(world.numRigidObjects()):
            if any(collider.robotObjectCollisions(self.robot.index,o)):
                #print "Collided with object",o
                return False;
        #test robot-terrain collisions
        for o in xrange(world.numTerrains()):
            if any(collider.robotTerrainCollisions(self.robot.index,o)):
                #print "Collided with terrain",o
                return False;
        #test robot self-collisions
        if any(collider.robotSelfCollisions(self.robot.index)):
            #print "Robot self collision"
            return False
        return True

def planTransfer(world,objectIndex,hand,shift):
    """Plan a transfer path for the robot given in world, which is currently
    holding the object indexed by objectIndex in the hand hand.
    
    The desired motion should translate the object by shift without rotating
    the object.
    """
    globals = Globals(world)
    obj = world.rigidObject(objectIndex)
    cspace = TransferCSpace(globals,hand,obj)
    robot = world.robot(0)
    qmin,qmax = robot.getJointLimits()

    #get the start config
    q0 = robot.getConfig()
    q0arm = [q0[i] for i in hand.armIndices]
    if not cspace.feasible(q0arm):
        print "Warning, arm start configuration is infeasible"
        print "TODO: Complete 2.a to bypass this error"
        raw_input()
        return None
                
    #TODO: get the ungrasp config using an IK solver
    qungrasp = None
    qungrasparm = None
    print "TODO: Complete 2.b to find a feasible ungrasp config"
    raw_input()
    solver = hand.ikSolver(robot,vectorops.add(obj.getTransform()[1],shift),(0,0,1))

    #plan the transfer path between q0arm and qungrasparm
    print "Planning transfer motion to ungrasp config..."
    MotionPlan.setOptions(connectionThreshold=5.0,perturbationRadius=0.5)
    planner = MotionPlan(cspace,'sbl')
    planner.setEndpoints(q0arm,qungrasparm)
    #TODO: do the planning
    print "TODO: Complete 2.c to find a feasible transfer path"
    raw_input()
    
    #lift arm path to whole configuration space path
    path = []
    for qarm in planner.getPath():
        path.append(q0[:])
        for qi,i in zip(qarm,hand.armIndices):
            path[-1][i] = qi

    qpostungrasp = hand.open(qungrasp,1.0)
    return path + [qpostungrasp]

class GLTransferPlanProgram(GLRealtimeProgram):
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"GLTransferPlanProgram")
        self.world = world
        self.robot = world.robot(0)
        self.object = world.rigidObject(0)
        #start robot config
        self.qstart = self.robot.getConfig()
        #start object transform
        self.Tstart = self.object.getTransform()
        #solution to planning problem
        self.path = None
        self.animationTime = None

    def display(self):
        #draw the world
        self.world.drawGL()

    def keyboardfunc(self,key,x,y):
        if key == 'r':
            self.robot.setConfig(self.qstart)
            self.object.setTransform(*self.Tstart)
            self.path = planTransfer(self.world,0,Hand('l'),(0,-0.15,0))
            #reset the animation
            self.animationTime = self.ttotal
            glutPostRedisplay()
        elif key == 'f':
            self.robot.setConfig(self.qstart)
            self.object.setTransform(*self.Tstart)
            self.path = planTransfer(self.world,0,Hand('l'),(0.15,0,0))
            #reset the animation
            self.animationTime = self.ttotal
            glutPostRedisplay()

    def idle(self):
        if self.path:
            #loop the path animation
            u = (self.ttotal - self.animationTime)
            i = int(math.floor(u))
            s = u - i
            i = i%(len(self.path)-1)
            q = vectorops.interpolate(self.path[i],self.path[i+1],s)
            #set the robot configuration for display
            self.robot.setConfig(q)
            #compute and set object transform for display
            Tobj = graspedObjectTransform(self.robot,Hand('l'),self.qstart,self.Tstart,q)
            self.object.setTransform(*Tobj)
            glutPostRedisplay()

def run_ex2():
    world = WorldModel()
    res = world.readFile("ex2_file.xml")
    if not res: raise RuntimeError("Unable to load world file")
    GLTransferPlanProgram(world).run()

############################# Problem 3 ##############################


def planFree(world,hand,qtarget):
    """Plans a free-space motion for the robot's arm from the current
    configuration to the destination qtarget"""
    globals = Globals(world)
    cspace = TransitCSpace(globals,hand)
    robot = world.robot(0)
    qmin,qmax = robot.getJointLimits()
    
    #get the start/goal config
    q0 = robot.getConfig()
    q0arm = [q0[i] for i in hand.armIndices]
    qtargetarm = [qtarget[i] for i in hand.armIndices]

    if not cspace.feasible(q0arm):
        print "Warning, arm start configuration is infeasible"
    if not cspace.feasible(qtargetarm):
        print "Warning, arm goal configuration is infeasible"

    print "Planning transit motion to target config..."
    MotionPlan.setOptions(connectionThreshold=5.0,perturbationRadius=0.5)
    planner = MotionPlan(cspace,'sbl')
    planner.setEndpoints(q0arm,qtargetarm)
    iters = 0
    step = 10
    while planner.getPath()==None and iters < 1000:
        planner.planMore(step)
        iters += step
    if planner.getPath() == None:
        print "Failed finding transit path"
        return None
    print "Success"
    
    #lift arm path to whole configuration space path
    path = []
    for qarm in planner.getPath():
        path.append(q0[:])
        for qi,i in zip(qarm,hand.armIndices):
            path[-1][i] = qi
    return path

class GLPickAndPlaceProgram(GLRealtimeProgram):
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"GLPickAndPlaceProgram")
        self.world = world
        self.robot = world.robot(0)
        self.object = world.rigidObject(0)
        #start robot config
        self.qstart = self.robot.getConfig()
        #start object transform
        self.Tstart = self.object.getTransform()
        #solution to planning problem
        self.transitPath = None
        self.transferPath = None
        self.retractPath = None
        self.animationTime = None

    def display(self):
        #draw the world
        self.world.drawGL()


    def keyboardfunc(self,key,x,y):
        h = 0.932
        targets = {'a':(0.35,0.25,h),'b':(0.35,0.05,h),'c':(0.35,-0.1,h),'d':(0.35,-0.1,h)}
        if key in targets:
            dest = targets[key]
            shift = vectorops.sub(dest,self.Tstart[1])
            self.robot.setConfig(self.qstart)
            self.object.setTransform(*self.Tstart)
            self.hand = Hand('l')
            #plan transit path to grasp object
            self.transitPath = planTransit(self.world,0,self.hand)
            if self.transitPath:
                #plan transfer path
                self.robot.setConfig(self.transitPath[-1])
                self.transferPath = planTransfer(self.world,0,self.hand,shift)
                if self.transferPath:
                    self.Tgoal = graspedObjectTransform(self.robot,self.hand,self.transferPath[0],self.Tstart,self.transferPath[-1])
                    #plan free path to retract arm
                    self.robot.setConfig(self.transferPath[-1])
                    self.object.setTransform(*self.Tgoal)
                    self.retractPath = planFree(self.world,self.hand,self.qstart)
            
            #reset the animation
            self.animationTime = self.ttotal
            glutPostRedisplay()
        if key=='n':
            print "Moving to next action"
            if self.transitPath and self.transferPath and self.retractPath:
                #update start state
                self.qstart = self.retractPath[-1]
                self.Tstart = self.Tgoal
                self.robot.setConfig(self.qstart)
                self.object.setTransform(*self.Tstart)
                self.transitPath = None
                self.transferPath = None
                self.hand = None
                glutPostRedisplay()

    def idle(self):
        if self.transitPath and self.transferPath and self.retractPath:
            #loop the path animation
            n = len(self.transitPath)+len(self.transferPath)+len(self.retractPath)-2
            u = (self.ttotal - self.animationTime)*3.0
            i = int(math.floor(u))
            s = u - i
            i = i%(n-1)
            if i+1 < len(self.transitPath):
                q = vectorops.interpolate(self.transitPath[i],self.transitPath[i+1],s)
                #set the robot configuration for display
                self.robot.setConfig(q)
                self.object.setTransform(*self.Tstart)
            elif i+1 < len(self.transitPath)+len(self.transferPath)-1:
                #index into the transfer path
                i = i - (len(self.transitPath)-1)
                q = vectorops.interpolate(self.transferPath[i],self.transferPath[i+1],s)
                Tobj = graspedObjectTransform(self.robot,self.hand,self.transferPath[0],self.Tstart,q)
                self.robot.setConfig(q)
                self.object.setTransform(*Tobj)
            else:
                #index into the transfer path
                i = i - (len(self.transitPath)-1) - (len(self.transferPath)-1)
                q = vectorops.interpolate(self.retractPath[i],self.retractPath[i+1],s)
                self.robot.setConfig(q)
                self.object.setTransform(*self.Tgoal)
            glutPostRedisplay()

def run_ex3():
    world = WorldModel()
    res = world.readFile("ex3_file.xml")
    if not res: raise RuntimeError("Unable to load world file")
    GLPickAndPlaceProgram(world).run()    

if __name__ == "__main__":
    #runs exercise 1
    run_ex1()

    #runs exercise 2
    #run_ex2()

    #runs exercise 3
    #run_ex3()
