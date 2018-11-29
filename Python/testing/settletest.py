#!/usr/bin/python

import sys
from klampt import *
from klampt import vis
from klampt.math import vectorops,so3
from klampt.sim import settle
import time
import math
import random

def rand_rotation():
    q = [random.gauss(0,1),random.gauss(0,1),random.gauss(0,1),random.gauss(0,1)]
    q = vectorops.unit(q)
    return so3.from_quaternion(q)

if __name__ == "__main__":
    print "settletest.py: This example demonstrates how to run the settling process"
    
    #creates a world and loads all the items on the command line
    world = WorldModel()
    res = world.readFile("../../data/tx90cups.xml")
    if not res:
        raise RuntimeError("Unable to load model "+fn)

    t0 = time.time()
    T = settle.settle(world,world.robot(0).link(7),forcedir=(0,0,-2),perturb=0.0,settletol=1e-3,debug=False)
    t1 = time.time()
    print "Settling robot link took time",t1-t0
    raw_input("Press enter to continue")
    for i in xrange(world.numRigidObjects()):
        obj = world.rigidObject(i)
        obj.setTransform(rand_rotation(),obj.getTransform()[1])
    for i in xrange(world.numRigidObjects()):
        obj = world.rigidObject(i)
        t0 = time.time()
        T,touched = settle.settle(world,obj,perturb=0.0,orientationDamping=0,settletol=1e-3,debug=False)
        obj.setTransform(*T)
        t1 = time.time()
        print "Settling object",i,"took time",t1-t0
        raw_input("Press enter to continue")
