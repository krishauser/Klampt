#!/usr/bin/python

import sys
from klampt import *
from klampt import visualization
from klampt import resource
from klampt import coordinates
from klampt import so3,se3
from klampt import trajectory
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

if __name__ == "__main__":
    print "trajectorytest.py: This example demonstrates several types of trajectory"
    if len(sys.argv)<=1:
        print "USAGE: trajectorytest.py [robot or world file]"
        print "With no arguments, shows some 3D trajectories"
        
        #add a point to the visualizer and animate it
        point = coordinates.addPoint("point")
        visualization.add("point",point)
        traj = trajectory.Trajectory()
        for i in range(10):
            traj.times.append(i)
            traj.milestones.append([random.uniform(-1,1),random.uniform(-1,1),random.uniform(-1,1)])

        traj2 = trajectory.HermiteTrajectory()
        traj2.makeSpline(traj)
        visualization.animate("point",traj2)

        #add a transform to the visualizer and animate it
        xform = visualization.add("xform",se3.identity())
        traj3 = trajectory.SE3Trajectory()
        for i in range(10):
            traj3.times.append(i)
            rrot = random_rotation()
            rpoint = [random.uniform(-1,1),random.uniform(-1,1),random.uniform(-1,1)]
            traj3.milestones.append(rrot+rpoint)
        
        visualization.animate("xform",traj3)
    else:
        #creates a world and loads all the items on the command line
        world = WorldModel()
        for fn in sys.argv[1:]:
            res = world.readFile(fn)
            if not res:
                raise RuntimeError("Unable to load model "+fn)

        #add the world to the visualizer
        robot = world.robot(0)
        visualization.add("robot",robot)
        traj = trajectory.RobotTrajectory(robot)
        qmin,qmax = robot.getJointLimits()
        q0 = robot.getConfig()
        for i in range(robot.numLinks()):
            if math.isinf(qmin[i]) or math.isinf(qmax[i]):
                #don't animate floating base
                continue
            traj.times.append(len(traj.times)*0.5)
            q = q0[:]
            q[i] = qmin[i]
            traj.milestones.append(q)
            
            traj.times.append(len(traj.times)*0.5)
            q[i] = qmax[i]
            traj.milestones.append(q)

        save,traj.milestones = resource.edit("trajectory",traj.milestones,world=world)
        visualization.animate("robot",traj)
    
    visualization.show()
    
    iteration = 0
    while visualization.shown():
        #visualization.lock()
        #can modify the visualization here
        #visualization.unlock()
        #time.sleep(0.01)
        #animationTime = visualization.animationTime()
        iteration += 1

    if len(sys.argv)>1:
        visualization.animate("robot",None)
        sim = Simulator(world)
        sim.simulate(0)
        trajectory.execute_path(traj.milestones,sim.controller(0))
        #for some tricky Qt reason, need to sleep before showing a window again
        #Perhaps the event loop must complete some extra cycles?
        time.sleep(0.01)
        visualization.show()
        t0 = time.time()
        while visualization.shown():
            #print "Time",sim.getTime()
            sim.simulate(0.01)
            if sim.controller(0).remainingTime() <= 0:
                print "Executing timed trajectory"
                trajectory.execute_trajectory(traj,sim.controller(0),smoothing='pause')
            visualization.setItemConfig("config",sim.controller(0).getCommandedConfig())
            t1 = time.time()
            time.sleep(max(0.01-(t1-t0),0.0))
            t0 = t1
    
    print "Ending visualization."
    visualization.kill()
