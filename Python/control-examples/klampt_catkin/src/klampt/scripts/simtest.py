#!/usr/bin/python
import os
import sys
from klampt import *
from klampt.glprogram import *
from klampt import collide
import importlib

class MyGLViewer(GLRealtimeProgram):
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"SimTest")
        self.world = world
        #Put your initialization code here
        #the current example creates a collision class, simulator, 
        #simulation flag, and screenshot flags
        self.collider = robotcollide.WorldCollider(world)
        self.sim = Simulator(world)
        self.simulate = False
        self.controllers = []
        self.forceApplicationMode = False
        
        #this is the controller time step.  The internal simulation time step
        #can be modified in the world's XML file
        self.dt = 0.01

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

        #draw controller
        self.sim.updateWorld()
        for i in xrange(self.world.numRobots()):
            if i >= len(self.controllers): break
            self.controllers[i].drawGL()
            
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

    def control_loop(self):
        for i in xrange(self.world.numRobots()):
            if i >= len(self.controllers): break
            c = self.sim.getController(i)
            #build measurement dict
            measurements = {'t':self.sim.getTime(),'dt':self.dt,'qcmd':c.getCommandedConfig(),'dqcmd':c.getCommandedVelocity()}
            k = 0
            while True:
                s = c.getSensor(k)
                if s.type()=='':
                    break;
                measurements[s.name()] = s.getMeasurements()
                k+=1
            """
            #debug: print measurements
            for (k,v) in measurements.iteritems():
                print k,":",
                if hasattr(v,'__iter__'):
                    print ' '.join("%.2f"%(vi,) for vi in v)
                else:
                    print v
            """
            #compute controller output, advance
            output = self.controllers[i].output_and_advance(**measurements)
            #process output depending on type
            if output==None: continue
            defaultVals = set(['torquecmd','qcmd','dqcmd','tcmd'])
            if 'qcmd' in output:
                dqcmd = output['dqcmd'] if 'dqcmd' in output else [0.0]*len(output['qcmd'])
                if 'torquecmd' in output:
                    c.setPIDCommand(output['qcmd'],dqcmd,output['torquecmd'])
                else:
                    c.setPIDCommand(output['qcmd'],dqcmd)
            elif 'dqcmd' in output:
                assert 'tcmd' in output
                c.setVelocityCommand(output['dqcmd'],output['tcmd'])
            elif 'torquecmd' in output:
                c.setTorque(output['torquecmd'])
            for (k,v) in output.iteritems():
                if k not in defaultVals:
                    print "Sending command",k,v,"to low level controller"
                    c.sendCommand(k,v)

    def idle(self):
        #Put your idle loop handler here
        #the current example simulates with the current time step self.dt
        if self.simulate and self.saveScreenshots:
            if self.sim.getTime() >= self.nextScreenshotTime: 
                self.save_screen("image%04d.ppm"%(self.screenshotCount,))
                self.screenshotCount += 1
                self.nextScreenshotTime += 1.0/30.0;

        if self.simulate:
            self.control_loop()
            self.simulateForceSpring()
            self.sim.simulate(self.dt)
            glutPostRedisplay()

    def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        #the current example prints out the list of objects clicked whenever
        #you right click
        if button==2:
            if state==0:
                self.sim.updateWorld()
                objs = self.click_world(x,y)
                if len(objs) > 0:
                    print "Clicked:",[o[0].getName() for o in objs]
                    (s,d) = self.click_ray(x,y)
                    if isinstance(objs[0][0],RobotModelLink):
                        print "Clicked, turning on force application mode",objs[0][0].getName()
                        self.forceApplicationMode = True
                        self.addForceSpring(objs[0][0],objs[0][1])
                        return
            elif self.forceApplicationMode:
                print "Turning off force application mode"
                self.forceApplicationMode = False
                return
        GLRealtimeProgram.mousefunc(self,button,state,x,y)
        
    def motionfunc(self,x,y):
        if self.forceApplicationMode:
            self.moveForceSpring(x,y)
            glutPostRedisplay()
        else:
            GLRealtimeProgram.motionfunc(self,x,y)

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

    def simulateForceSpring(self,kP = 100.0):
        if not self.forceApplicationMode: return
        self.sim.updateWorld()
        body = self.sim.getBody(self.forceObject)
        T = self.forceObject.getTransform()
        wp = se3.apply(T,self.forceLocalPoint)
        #since we're not applying this force over sub-steps, we have to make it big!
        kP *= self.dt/0.001
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
        if c == 's':
            self.simulate = not self.simulate
            print "Simulating:",self.simulate
        elif c == 'm':
            self.saveScreenshots = not self.saveScreenshots
            print "Movie mode:",self.saveScreenshots
            if self.nextScreenshotTime < self.sim.getTime():
                self.nextScreenshotTime = self.sim.getTime()
        glutPostRedisplay()

    def click_world(self,x,y):
        """Helper: returns a list of (world object,clicked point) pairs,
        sorted in order of increasing distance."""
        #get the viewport ray
        (s,d) = self.click_ray(x,y)
        
        #run the collision tests
        self.collider.updateFrames()
        collided = []
        for g in self.collider.geomList:
            (hit,pt) = collide.rayCast(g[1],s,d)
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
        viewer.controllers.append(controller)
    
    viewer.run()
