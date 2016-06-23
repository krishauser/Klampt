#!/usr/bin/python
import os
import sys
from klampt import *
from klampt.glprogram import *
import importlib
from klampt.simlog import *
from klampt.simulation import *


class MyGLViewer(GLRealtimeProgram):
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"SimTest")
        self.world = world
        #Put your initialization code here
        #the current example creates a collision class, simulator, 
        #simulation flag, and screenshot flags
        self.collider = robotcollide.WorldCollider(world)
        self.sim = SimpleSimulator(world)
        #contact feedback is enabled, needed to detect penetration situations
        self.sim.enableContactFeedbackAll()
        self.simulate = False
        self.forceApplicationMode = False
        
        #this is the controller time step.  The internal simulation time step
        #can be modified in the world's XML file
        self.dt = 0.01

        #press 'c' to toggle display contact points / forces
        self.drawContacts = False

        #press 'm' to toggle screenshot saving
        self.saveScreenshots = False
        self.nextScreenshotTime = 0
        self.screenshotCount = 0

        self.logger = None
        #self.logger = SimLogger(self.sim,"simtest_state.csv","simtest_contact.csv")

    def display(self):
        #Put your display handler here
        #the current example draws the simulated world in grey and the
        #commanded configurations in transparent green
        self.sim.drawGL()
            
        #draw force springs if using
        if self.forceApplicationMode:
            glDisable(GL_LIGHTING)
            glDisable(GL_DEPTH_TEST)
            glColor3f(1,0.5,0)
            glLineWidth(3.0)
            glBegin(GL_LINES)
            glVertex3fv(self.forceAnchorPoint)
            glVertex3fv(se3.apply(self.forceObject.getTransform(),self.forceLocalPoint))
            glEnd()
            glLineWidth(1.0)
            glEnable(GL_DEPTH_TEST)

        #draw contacts, if enabled
        if self.drawContacts:
            glDisable(GL_LIGHTING)
            glDisable(GL_DEPTH_TEST)
            glEnable(GL_POINT_SMOOTH)
            glColor3f(1,1,0)
            glLineWidth(1.0)
            glPointSize(5.0)
            forceLen = 0.1  #scale of forces
            maxid = self.world.numIDs()
            for i in xrange(maxid):
                for j in xrange(i+1,maxid):
                    points = self.sim.getContacts(i,j)
                    if len(points) > 0:
                        forces = self.sim.getContactForces(i,j)
                        glBegin(GL_POINTS)
                        for p in points:
                            glVertex3f(*p[0:3])
                        glEnd()
                        glBegin(GL_LINES)
                        for p,f in zip(points,forces):
                            glVertex3f(*p[0:3])
                            glVertex3f(*vectorops.madd(p[0:3],f,forceLen))
                        glEnd()                        
            glEnable(GL_DEPTH_TEST)

    def display_screen(self):
        glDisable(GL_LIGHTING)
        self.draw_text(20,20,str(self.sim.getTime()))
        if self.sim.hadPenetration(-1,-1):
            self.draw_text(20,40,"Meshes penetrating, simulation may be unstable",color=[1,0,0])

    def control_loop(self):
        pass

    def idle(self):
        #Put your idle loop handler here
        #the current example simulates with the current time step self.dt
        if self.simulate and self.saveScreenshots:
            if self.sim.getTime() >= self.nextScreenshotTime: 
                self.save_screen("image%04d.ppm"%(self.screenshotCount,))
                self.screenshotCount += 1
                self.nextScreenshotTime += 1.0/30.0;

        if self.simulate:
            if self.logger: self.logger.saveStep()
            self.control_loop()
            self.simulateForceSpring()
            self.sim.simulate(self.dt)
            glutPostRedisplay()

    def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        #the current example prints out the list of objects clicked whenever
        #you right click, and applies forces
        if button==2:
            if state==0:
                self.sim.updateWorld()
                objs = self.click_world(x,y)
                if len(objs) > 0:
                    print "Clicked:",[o[0].getName() for o in objs]
                    (s,d) = self.click_ray(x,y)
                    if isinstance(objs[0][0],(RobotModelLink,RigidObjectModel)):
                        print "Clicked, turning on force application mode",objs[0][0].getName()
                        self.forceApplicationMode = True
                        self.addForceSpring(objs[0][0],objs[0][1])
                        return
            elif self.forceApplicationMode:
                print "Turning off force application mode"
                self.forceApplicationMode = False
                return
        GLRealtimeProgram.mousefunc(self,button,state,x,y)
        
    def motionfunc(self,x,y,dx,dy):
        if self.forceApplicationMode:
            self.moveForceSpring(x,y)
            glutPostRedisplay()
        else:
            GLRealtimeProgram.motionfunc(self,x,y,dx,dy)

    def moveForceSpring(self,x,y):
        self.sim.updateWorld()
        (s,d) = self.click_ray(x,y)
        u = vectorops.dot(vectorops.sub(self.forceAnchorPoint,s),d)
        self.forceAnchorPoint = vectorops.madd(s,d,u)

    def addForceSpring(self,obj,worldpt):
        self.sim.updateWorld()
        self.forceObject = obj
        Ro,to = obj.getTransform()
        self.forceAnchorPoint = worldpt
        self.forceLocalPoint = se3.apply(se3.inv((Ro,to)),self.forceAnchorPoint)

    def simulateForceSpring(self,kP = 10.0):
        if not self.forceApplicationMode: return
        self.sim.updateWorld()
        body = self.sim.body(self.forceObject)
        T = self.forceObject.getTransform()
        wp = se3.apply(T,self.forceLocalPoint)
        f = vectorops.mul(vectorops.sub(self.forceAnchorPoint,wp),kP)
        #get wrench about com
        momentArmLocal = vectorops.sub(self.forceLocalPoint,self.forceObject.getMass().getCom())
        momentArmWorld = so3.apply(T[0],momentArmLocal)
        w = vectorops.cross(momentArmWorld,f)
        print "Applying wrench",(f,w)
        body.applyWrench(f,w)

    def specialfunc(self,c,x,y):
        #Put your keyboard special character handler here
        print c,"pressed"

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        print c,"pressed"
        if c == 'h':
            print "************** Help **************"
            print "s: toggle simulation"
            print "m: toggle movie mode"
            print "l: toggle logging"
            print "c: toggle contact drawing"
            print "**********************************"
        elif c == 's':
            self.simulate = not self.simulate
            print "Simulating:",self.simulate
        elif c == 'm':
            self.saveScreenshots = not self.saveScreenshots
            print "Movie mode:",self.saveScreenshots
            if self.nextScreenshotTime < self.sim.getTime():
                self.nextScreenshotTime = self.sim.getTime()
        elif c == 'l':
            self.sim.toggleLogging()
            if not self.sim.logging:
                print "Turned off logging"
        elif c == 'c':
            self.drawContacts = not self.drawContacts
            #this is being done automatically now to detect penetration situations
            #if self.drawContacts:
            #    self.sim.enableContactFeedbackAll()
        glutPostRedisplay()

    def click_world(self,x,y):
        """Helper: returns a list of (world object,clicked point) pairs,
        sorted in order of increasing distance."""
        #get the viewport ray
        (s,d) = self.click_ray(x,y)
        self.sim.updateWorld()        
        #run the collision tests
        collided = []
        for g in self.collider.geomList:
            (hit,pt) = g[1].rayCast(s,d)
            if hit:
                dist = vectorops.dot(vectorops.sub(pt,s),d)
                collided.append((dist,g[0],pt))
        return [(g[1],g[2]) for g in sorted(collided)]


if __name__ == "__main__":
    print "simtest.py: Simulates a robot file and Python controller"
    if len(sys.argv)<=1:
        print "USAGE: simtest.py [world_file] [controller files (.py)]"
        exit()
    world = WorldModel()
    control_modules = []
    for fn in sys.argv[1:]:
        path,base = os.path.split(fn)
        mod_name,file_ext = os.path.splitext(base)
        if file_ext=='.py' or file_ext=='.pyc':
            sys.path.append(os.path.abspath(path))
            mod = importlib.import_module(mod_name,base)
            control_modules.append(mod)
        elif file_ext=='.path':
            control_modules.append(fn)
        else:
            res = world.readFile(fn)
            if not res:
                raise RuntimeError("Unable to load model "+fn)
    viewer = MyGLViewer(world)

    for i,c in enumerate(control_modules):
        if isinstance(c,str):
            sys.path.append(os.path.abspath("../control"))
            import trajectory_controller
            #it's a path file, try to load it
            controller = trajectory_controller.make(world.robot(i),c)
        else:
            try:
                maker = c.make
            except AttributeError:
                print "Module",c.__name__,"must have a make() method"
                raise
            controller = maker(world.robot(i))
        viewer.sim.setController(i,controller)
    
    viewer.run()
