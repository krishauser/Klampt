from klampt import *
from klampt.model.collide import WorldCollider
from klampt.model.trajectory import RobotTrajectory
from klampt.vis.glcommon import *
from klampt import vis
from klampt.math import vectorops,so3,se3
from klampt.plan.cspace import CSpace,MotionPlan
from openhands import openhand
from OpenGL.GL import *
import math
import random
import time

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
        self.collider = WorldCollider(world)

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
        #TODO: this could be much more efficient if you only tested the robot's moving joints
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
    solver.setMaxIters(100)
    solver.setTolerance(1e-3)
    res = solver.solve();
    if res:
        qpregrasp =  robot.getConfig()
        qpregrasparm = [qpregrasp[i] for i in hand.armIndices]
        if not cspace.feasible(qpregrasparm):
            print "Pregrasp config infeasible"
            cspace.close()
            return None
    if qpregrasp == None:
        print "Pregrasp solve failed"
        cspace.close()
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
    cspace.close()
    if planner.getPath() == None:
        print "Failed finding transit path"
        return None
    print "Success, found path with",len(planner.getPath()),"milestones"

    #lift arm path to whole configuration space path
    path = []
    for qarm in planner.getPath():
        path.append(q0[:])
        for qi,i in zip(qarm,hand.armIndices):
            path[-1][i] = qi
    
    #add a path to the grasp configuration
    return path + [hand.open(path[-1],0)]

class GLTransitPlanPlugin(GLPluginInterface):
    def __init__(self,world):
        GLPluginInterface.__init__(self)
        self.world = world
        self.robot = world.robot(0)
        self.qstart = self.robot.getConfig()
        #solution to planning problem
        self.path = None

    def display(self):
        #draw points on the robot
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
            if self.path:
                #convert to a timed path for animation's sake
                self.path = RobotTrajectory(self.robot,range(len(self.path)),self.path)
            #reset the animation
            vis.animate(("world",self.robot.getName()),self.path)
            return True
        elif key == 'r':
            self.robot.setConfig(self.qstart)
            self.path = planTransit(self.world,0,Hand('r'))
            if self.path:
                #convert to a timed path for animation's sake
                self.path = RobotTrajectory(self.robot,range(len(self.path)),self.path)
            #reset the animation
            vis.animate(("world",self.robot.getName()),self.path)
            return True
        return False

def run_ex1():
    world = WorldModel()
    res = world.readFile("ex1_file.xml")
    if not res: raise RuntimeError("Unable to load world file")
    vis.add("world",world)
    vis.setWindowTitle("Transit plan test, press l/r to plan with left/right arm")
    vis.pushPlugin(GLTransitPlanPlugin(world))
    vis.show()
    while vis.shown():
        time.sleep(0.1)
    vis.setPlugin(None)
    vis.kill()



############################# Problem 2 ##############################

def graspTransform(robot,hand,qrobot0,Tobj0):
    """Given initial robot configuration qrobot0 and object transform Tobj0,
    returns the grasp transformation Tgrasp, which produces the object transform
    via the composition  Tobj = Thand * Tgrasp"""
    robot.setConfig(qrobot0)
    Thand0 = robot.link(hand.link).getTransform()
    Tgrasp = se3.mul(se3.inv(Thand0),Tobj0)
    return Tgrasp


def graspedObjectTransform(robot,hand,qrobot0,Tobj0,qrobot):
    """Given initial robot configuration qrobot0 and object transform Tobj0,
    returns the object transformation corresponding to new configuration
    qrobot assuming the object is rigidly attached to the hand"""
    Tgrasp = graspedObjectTransform(robot,hand,qrobot0,Tobj0)
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
        #initial whole-body configuratoin
        self.q0 = self.robot.getConfig()
        #setup initial grasp transform
        Tobj0 = object.getTransform()
        self.Tgrasp = graspTransform(self.robot,hand,self.q0,Tobj0)
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
        cspace.close()
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
    
    cspace.close()
    #lift arm path to whole configuration space path
    path = []
    for qarm in planner.getPath():
        path.append(q0[:])
        for qi,i in zip(qarm,hand.armIndices):
            path[-1][i] = qi

    qpostungrasp = hand.open(qungrasp,1.0)
    return path + [qpostungrasp]

class GLTransferPlanPlugin(GLPluginInterface):
    def __init__(self,world):
        GLPluginInterface.__init__(self)
        self.world = world
        self.robot = world.robot(0)
        self.object = world.rigidObject(0)
        #start robot config
        self.qstart = self.robot.getConfig()
        #start object transform
        self.Tstart = self.object.getTransform()
        #grasp transform
        self.Tgrasp = Tgrasp = graspTransform(self.robot,Hand('l'),self.qstart,self.Tstart)
        #solution to planning problem
        self.path = None

    def keyboardfunc(self,key,x,y):
        if key == 'r':
            self.robot.setConfig(self.qstart)
            self.object.setTransform(*self.Tstart)
            self.path = planTransfer(self.world,0,Hand('l'),(0,-0.15,0))
            if self.path:
                #convert to a timed path for animation's sake
                self.path = RobotTrajectory(self.robot,range(len(self.path)),self.path)
                #compute object trajectory
                resolution = 0.05
                self.objectPath = self.path.getLinkTrajectory(Hand('l').link,resolution)
                self.objectPath.postTransform(self.Tgrasp)
            else:
                self.path = None
            #reset the animation
            vis.animate(("world",self.robot.getName()),self.path)
            vis.animate(("world",self.object.getName()),self.objectPath)
            return True
        elif key == 'f':
            self.robot.setConfig(self.qstart)
            self.object.setTransform(*self.Tstart)
            self.path = planTransfer(self.world,0,Hand('l'),(0.15,0,0))
            if self.path:
                #convert to a timed path for animation's sake
                self.path = RobotTrajectory(self.robot,range(len(self.path)),self.path)
                #compute object trajectory
                resolution = 0.05
                self.objectPath = self.path.getLinkTrajectory(Hand('l').link,resolution)
                self.objectPath.postTransform(self.Tgrasp)
            else:
                self.path = None
            #reset the animation
            vis.animate(("world",self.robot.getName()),self.path)
            vis.animate(("world",self.object.getName()),self.objectPath)
            return True
        return False


def run_ex2():
    world = WorldModel()
    res = world.readFile("ex2_file.xml")
    if not res: raise RuntimeError("Unable to load world file")
    vis.add("world",world)
    vis.pushPlugin(GLTransferPlanPlugin(world))
    vis.setWindowTitle("Transfer plan test, press r/f to plan with right/forward target")
    vis.show()
    while vis.shown():
        time.sleep(0.1)
    vis.setPlugin(None)
    vis.kill()

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
    cspace.close()
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

class GLPickAndPlacePlugin(GLPluginInterface):
    def __init__(self,world):
        GLPluginInterface.__init__(self)
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
                self.Tgrasp = graspTransform(self.robot,self.hand,self.transitPath[-1],self.Tstart)
                self.robot.setConfig(self.transitPath[-1])
                self.transferPath = planTransfer(self.world,0,self.hand,shift)
                if self.transferPath:
                    self.Tgoal = graspedObjectTransform(self.robot,self.hand,self.transferPath[0],self.Tstart,self.transferPath[-1])
                    #plan free path to retract arm
                    self.robot.setConfig(self.transferPath[-1])
                    self.object.setTransform(*self.Tgoal)
                    self.retractPath = planFree(self.world,self.hand,self.qstart)
            
            #reset the animation
            if self.transitPath and self.transferPath and self.retractPath:
                milestones = self.transitPath+self.transferPath+self.retractPath
                self.path = RobotTrajectory(self.robot,range(len(milestones)),milestones)

                resolution = 0.05
                xfertraj = RobotTrajectory(self.robot,range(len(self.transferPath)),self.transferPath)
                xferobj = xfertraj.getLinkTrajectory(self.hand.link,resolution)
                xferobj.postTransform(self.Tgrasp)
                #offset times to be just during the transfer stage
                for i in xrange(len(xferobj.times)):
                    xferobj.times[i] += len(self.transitPath)
                self.objectPath = xferobj
                vis.animate(("world",self.robot.getName()),self.path)
                vis.animate(("world",self.object.getName()),self.objectPath)
            else:
                vis.animate(("world",self.robot.getName()),None)
                vis.animate(("world",self.object.getName()),None)
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
                self.Tgrasp = None
                self.refresh()


def run_ex3():
    world = WorldModel()
    res = world.readFile("ex3_file.xml")
    if not res: raise RuntimeError("Unable to load world file")
    vis.add("world",world)
    vis.setWindowTitle("Pick and place test, use a/b/c/d to select target")
    vis.pushPlugin(GLPickAndPlacePlugin(world))    
    vis.show()
    while vis.shown():
         time.sleep(0.1)
    vis.setPlugin(None)
    vis.kill()

if __name__ == "__main__":
    #runs exercise 1
    #run_ex1()

    #runs exercise 2
    #run_ex2()

    #runs exercise 3
    run_ex3()
