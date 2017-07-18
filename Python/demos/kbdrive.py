#!/usr/bin/python

import sys
from klampt import *
from klampt.vis.glrobotprogram import *
from klampt import vis

#FOR DEFAULT JOINT-BY-JOINT KEYMAP: set keymap=None
keymap = None

#FOR CUSTOM KEYMAPS: set up keymap to define how keys map to velocities.
#keymap is a map from key name to (robot index,velocity vector) pairs.
#Key names can either be single keys or names of special keys
#'left','up','down','right', 'home', 'insert', 'end', and the function keys 'f1',...,'f12'.
#keymap = {'up':(0,[0,1]),'down':(0,[0,-1]),'left':(0,[-1,0]),'right':(0,[1,0])}

def build_default_keymap(world):
    """builds a default keymape: 1234567890 increases values of DOFs 1-10
    of robot 0.  qwertyuiop decreases values."""
    if world.numRobots() == 0:
        return {}
    robot = world.robot(0)
    up = '1234567890'
    down = 'qwertyuiop'
    res = {}
    for i in range(min(robot.numDrivers(),10)):
        #up velocity
        vel = [0]*robot.numLinks()
        if robot.driver(i).getType() == 'normal':
            vel[robot.driver(i).getAffectedLink()] = 1
        else:
            #skip it
            #links = robot.driver(i).getAffectedLinks();
            continue
        res[up[i]] = (0,vel)
        #down velocity
        vel = vectorops.mul(vel,-1)
        res[down[i]] = (0,vel)
    return res



class MyGLViewer(GLSimulationPlugin):
    def __init__(self,world):
        global keymap
        GLSimulationPlugin.__init__(self,world)
        self.world = world
        if keymap == None:
            keymap = build_default_keymap(world)
        self.keymap = keymap
        self.current_velocities = {}
        #Put your initialization code here

    def control_loop(self):
        #Calculate the desired velocity for each robot by adding up all
        #commands
        rvels = [[0]*self.world.robot(r).numLinks() for r in range(self.world.numRobots())]
        for (c,(r,v)) in self.current_velocities.iteritems():
            rvels[r] = vectorops.add(rvels[r],v)
        #print rvels
        #send to the robot(s)
        for r in range(self.world.numRobots()):
            robotController = self.sim.controller(r)
            qdes = robotController.getCommandedConfig()
            qdes = vectorops.madd(qdes,rvels[r],self.dt)
            #clamp to joint limits
            (qmin,qmax) = self.world.robot(r).getJointLimits()
            for i in xrange(len(qdes)):
                qdes[i] = min(qmax[i],max(qdes[i],qmin[i]))
            robotController.setPIDCommand(qdes,rvels[r])
        return

    def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        #the current example prints out the list of objects clicked whenever
        #you right click
        if button==2:
            if state==0:
                print [o.getName() for o in self.click_world(x,y)]
                return
        GLSimulationPlugin.mousefunc(self,button,state,x,y)

    def print_help(self):
        self.window.program.print_help()
        print 'Drive keys:',sorted(self.keymap.keys())

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        if c in self.keymap:
            self.current_velocities[c]=self.keymap[c]
            return True
        elif c == '?':
            self.print_help()
            return True
        else:
            GLSimulationPlugin.keyboardfunc(self,c,x,y)
            return True
        self.refresh()

    def keyboardupfunc(self,c,x,y):
        if c in self.current_velocities:
            del self.current_velocities[c]
        return


if __name__ == "__main__":
    print "kbdrive.py: This example demonstrates how to drive a robot using keyboard input"
    if len(sys.argv)<=1:
        print "USAGE: kbdrive.py [world_file]"
        exit()
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)

    viewer = MyGLViewer(world)

    print 
    print "**********************"
    print "       HELP"
    print "Press 's' to start simulating."
    print "Use 1,2,..,0 to increase the robot's joints and q,w,...,p to reduce them."
    print "**********************"
    print 
    vis.run(viewer)
