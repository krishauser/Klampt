#!/usr/bin/python

import sys
import time
from klampt import *


if __name__ == "__main__":
    print "manyworldtest.py: This example demonstrates loading of many worlds"
    if len(sys.argv)<=1:
        print "USAGE: manyworldtest.py world_file N"
        exit()

    N = int(sys.argv[2])

    w0 = WorldModel()
    w0.readFile(sys.argv[1])
    w0.saveFile("temp_world.xml")
    #this line takes all the geometries out of the cache
    #w0 = None

    t0 = time.time()
    for i in xrange(N):
        world = WorldModel()
        world.readFile(sys.argv[1])
    t1 = time.time()
    test1time = t1-t0

    w1 = WorldModel()
    w1.readFile("temp_world.xml")
    #this line takes all the geometries out of the cache
    #w1 = None
    t0 = time.time()
    for i in xrange(N):
        world = WorldModel()
        world.readFile("temp_world.xml")
    t1 = time.time()
    test2time = t1-t0
    print "Time to load",N,"worlds, raw:",test1time
    print "Time to load",N,"worlds, after saving:",test2time