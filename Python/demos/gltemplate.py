#!/usr/bin/python

import sys
from klampt import *
from klampt.glprogram import *

class MyGLViewer(GLRealtimeProgram):
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"My GL program")
        self.world = world
        #Put your initialization code here
        #the current example creates a collision class, simulator, 
        #simulation flag, and screenshot flags
        self.collider = robotcollide.WorldCollider(world)
        self.sim = Simulator(world)
        self.simulate = False

        self.saveScreenshots = False
        self.nextScreenshotTime = 0
        self.screenshotCount = 0

    def display(self):
        #Put your display handler here
        #the current example draws the simulated world in grey and the
        #commanded configurations in transparent green
        self.sim.updateWorld()
        self.world.drawGL()

        #draw commanded configurations
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0,1,0,0.5])
        for i in xrange(self.world.numRobots()):
            r = self.world.robot(i)
            q = self.sim.getController(i).getCommandedConfig()
            r.setConfig(q)
            r.drawGL(False)
        glDisable(GL_BLEND)

    def control_loop(self):
        #Put your control handler here
        pass

    def idle(self):
        #Put your idle loop handler here
        #the current example simulates with the current time step self.dt
        if self.simulate and self.saveScreenshots:
            #The following line saves movies on simulation time
            if self.sim.getTime() >= self.nextScreenshotTime:
            #The following line saves movies on wall clock time
            #if self.ttotal >= self.nextScreenshotTime:
                self.save_screen("image%04d.ppm"%(self.screenshotCount,))
            self.screenshotCount += 1
            self.nextScreenshotTime += 1.0/30.0;

        if self.simulate:
            self.control_loop()
            self.sim.simulate(self.dt)
            glutPostRedisplay()

    def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        #the current example prints out the list of objects clicked whenever
        #you right click
        print "mouse",button,state,x,y
        if button==2:
            if state==0:
                print [o.getName() for o in self.click_world(x,y)]
            return
        GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y):
        GLRealtimeProgram.motionfunc(self,x,y)

    def specialfunc(self,c,x,y):
        #Put your keyboard special character handler here
        print c,"pressed"

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        print c,"pressed"
        if c == 's':
            self.simulate = not self.simulate
            print "Simulating:",self.simulate
        elif c == 'm':
            self.saveScreenshots = not self.saveScreenshots
            print "Movie mode:",self.saveScreenshots
        glutPostRedisplay()

    def click_world(self,x,y):
        """Helper: returns a list of world objects sorted in order of
        increasing distance."""
        #get the viewport ray
        (s,d) = self.click_ray(x,y)
        
        #run the collision tests
        #self.collider.updateFrames()
        collided = []
        for g in self.collider.geomList:
            (hit,pt) = g[1].rayCast(s,d)
            if hit:
                dist = vectorops.dot(vectorops.sub(pt,s),d)
                collided.append((dist,g[0]))
        return [g[1] for g in sorted(collided)]


if __name__ == "__main__":
    print "gltemplate.py: This example demonstrates how to simulate a world and read user input"
    if len(sys.argv)<=1:
        print "USAGE: gltemplate.py [world_file]"
        exit()
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)
    viewer = MyGLViewer(world)
    viewer.run()
