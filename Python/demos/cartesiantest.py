#!/usr/bin/python

import sys
from klampt import *
from klampt import vis
from klampt.vis import editors
from klampt.io import resource
from klampt.model import ik,coordinates,config,cartesian_trajectory
from klampt.math import vectorops,so3,se3
from klampt.model import trajectory
import random
import time
import math

def random_rotation():
    """Returns a uniformly distributed rotation matrix."""
    q = [random.gauss(0,1),random.gauss(0,1),random.gauss(0,1),random.gauss(0,1)]
    q = vectorops.unit(q)
    theta = math.acos(q[3])*2.0
    if abs(theta) < 1e-8:
        m = [0,0,0]
    else:
        m = vectorops.mul(vectorops.unit(q[0:3]),theta)
    return so3.from_moment(m)

class InterpKeyCapture(vis.GLPluginInterface):
    def __init__(self,endeffectors,constraints):
        vis.GLPluginInterface.__init__(self)
        self.endeffectors = endeffectors
        self.constraints = constraints
        self.robot = self.constraints[0].robot
        self.goalConfig = self.robot.getConfig()
        vis.add("ghost1",self.robot.getConfig())
        vis.setColor("ghost1",0,1,0,0.5)
        vis.add("ghost2",self.robot.getConfig())
        vis.setColor("ghost2",1,0,0,0.5)
        #vis.hide("ghost1",False)
        #vis.hide("ghost2",False)
    
        def goToWidget():
            #first, set all constraints so they are fit at the robot's last solved configuration, and get the
            #current workspace coordinates
            vis.setItemConfig("ghost1",self.goalConfig)
            self.robot.setConfig(self.goalConfig)
            for e,d in zip(self.endeffectors,self.constraints):
                d.matchDestination(*self.robot.link(e).getTransform())
            wcur = config.getConfig(self.constraints)
            wdest = []
            for e in self.endeffectors:
                xform = vis.getItemConfig("ee_"+robot.link(e).getName())
                wdest += xform
            print "Current workspace coords",wcur
            print "Dest workspace coords",wdest
            #Now interpolate
            self.robot.setConfig(self.goalConfig)
            traj1 = cartesian_trajectory.cartesian_interpolate_linear(robot,wcur,wdest,self.constraints,delta=1e-2,maximize=False)
            self.robot.setConfig(self.goalConfig)
            traj2 = cartesian_trajectory.cartesian_interpolate_bisect(robot,wcur,wdest,self.constraints,delta=1e-2)
            self.robot.setConfig(self.goalConfig)
            #traj3 = cartesian_trajectory.cartesian_path_interpolate(robot,[wcur,wdest],self.constraints,delta=1e-2,method='any',maximize=False)
            traj3 = None
            print "Method1 Method2 Method3:"
            print "  ",(traj1 != None), (traj2 != None), (traj3 != None)
            if traj1: traj = traj1
            elif traj2: traj = traj2
            elif traj3: traj = traj3
            else: traj = cartesian_trajectory.cartesian_interpolate_linear(robot,wcur,wdest,self.constraints,delta=1e-2,maximize=True)
            if traj:
                print "Result has",len(traj.milestones),"milestones"
                self.goalConfig = traj.milestones[-1]
                vis.setItemConfig(("world",world.robot(0).getName()),self.goalConfig)
                vis.animate(("world",world.robot(0).getName()),traj,speed=0.2,endBehavior='loop')
                vis.setItemConfig("ghost2",traj.milestones[-1])
                vis.add("ee_trajectory",traj)
            self.refresh()
        self.add_action(goToWidget,"Go to widget",' ')

class BumpKeyCapture(vis.GLPluginInterface):
    def __init__(self,endeffectors,constraints,traj):
        vis.GLPluginInterface.__init__(self)
        self.endeffectors = endeffectors
        self.constraints = constraints
        self.robot = self.constraints[0].robot
        self.traj = traj
        self.refConfig = self.robot.getConfig()
        vis.add("ghost1",self.refConfig)
        vis.setColor("ghost1",0,1,0,0.5)
        #vis.hide("ghost1",False)
    
        def bumpTrajectory():
            relative_xforms = []
            robot.setConfig(self.refConfig)
            for e in self.endeffectors:
                xform = vis.getItemConfig("ee_"+robot.link(e).getName())
                T = (xform[:9],xform[9:])
                T0 = robot.link(e).getTransform()
                Trel = se3.mul(se3.inv(T0),T)
                print "Relative transform of",e,"is",Trel
                relative_xforms.append(Trel)
            bumpTraj = cartesian_trajectory.cartesian_bump(self.robot,self.traj,self.constraints,relative_xforms,ee_relative=True,closest=True)
            assert bumpTraj != None
            vis.animate(("world",world.robot(0).getName()),bumpTraj)
            self.refresh()
        self.add_action(bumpTrajectory,"Bump trajectory",'b')

if __name__ == "__main__":
    print "cartesiantest.py: This example demonstrates cartesian trajectory interpolation"
    if len(sys.argv)<=1:
        print "USAGE: cartesiantest.py [robot or world file]"
        exit(0)
            
    #creates a world and loads all the items on the command line
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)

    robot = world.robot(0)
    endeffectors = [robot.numLinks()-1]
    #(res,val) = editors.run(editors.SelectionEditor("endeffectors",endeffectors,description="End effector links",world=world,robot=robot))
    #if res:
    #    print "Return value",val
    #    endeffectors = val

    vis.add("world",world)
    vis.listItems()
    vis.setColor(("world",world.robot(0).getName()),1,1,0,1)
    coordinates.setRobotModel(robot)
    eenames = [robot.link(e).getName() for e in endeffectors]
    eeobjectives = []
    for e in endeffectors:
        f = coordinates.frame(robot.link(e).getName())
        fw = coordinates.addFrame(robot.link(e).getName()+"_tgt",robot.link(e).getTransform())
        assert f != None
        vis.add("ee_"+robot.link(e).getName(),fw)
        vis.edit("ee_"+robot.link(e).getName())
        obj = coordinates.ik_fixed_objective(f)
        eeobjectives.append(obj)

    #this tests the cartesian interpolation stuff
    print "***** BEGINNING CARTESIAN INTERPOLATION TEST *****"
    vis.setWindowTitle("Klamp't Cartesian interpolation test")
    vis.pushPlugin(InterpKeyCapture(endeffectors,eeobjectives))
    vis.show()
    while vis.shown():
        coordinates.updateFromWorld()
        time.sleep(0.1)
    vis.popPlugin()
    vis.hide("ghost1")
    vis.hide("ghost2")
    vis.animate(("world",world.robot(0).getName()),None)

    print
    print 
    #this tests the "bump" function stuff
    print "***** BEGINNING BUMP FUNCTION TEST *****"
    configs = resource.get("cartesian_test"+world.robot(0).getName()+".configs",world=world)
    print "Found trajectory with",len(configs),"configurations"
    traj = trajectory.RobotTrajectory(robot,range(len(configs)),configs)
    vis.setWindowTitle("Klamp't Trajectory bump test")
    vis.pushPlugin(BumpKeyCapture(endeffectors,eeobjectives,traj))
    vis.show()
    while vis.shown():
        coordinates.updateFromWorld()
        time.sleep(0.1)
    vis.popPlugin()
    
    print "Ending vis."
    vis.kill()
