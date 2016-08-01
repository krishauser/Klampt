from OpenGL.GL import *
from ..robotsim import WidgetSet,RobotPoser
from glprogram import *
from ..model import robotcollide
from ..sim.simulation import SimpleSimulator
import sys

class GLSimulationProgram(GLRealtimeProgram):
    """A program that runs a simulation given a world.
    Attributes:
    - world: the RobotWorld instance provided on startup.  All elements
      are assumed to be instantiated already.
    - sim: a Simulator for the given world.
    - simulate: set this to True to start simulating.
    - saveScreenshots: set this to True if frames should be saved to disk.
    - commanded_config_color: an RGBA tuple defining the color of the
      commanded configuration, or None if it should not be drawn.
    - verbose: set to 1 if you wish to get printouts of the event loop
    - logging, logger: set to True and the logger if you wish to save a
      CSV log file to disk. Easier to use beginLogging(), pauseLogging(),
      and endLogging().

    Subclasses should overload self.control_loop() and put whatever control
    loop you desire inside.  Note: in this loop you should interact with
    self.sim.controller(0), not self.world.robot(0).  self.world is simply
    a model and does not have a direct relation to the simulation.
    """
    def __init__(self,world,name="My GL simulation program"):
        """Arguments:
        - world: a RobotWorld instance.
        """
        GLRealtimeProgram.__init__(self,name)
        self.world = world
        #Put your initialization code here
        #the current example creates a collision class, simulator, 
        #simulation flag, and screenshot flags
        self.collider = robotcollide.WorldCollider(world)
        self.sim = SimpleSimulator(world)
        self.simulate = False
        self.commanded_config_color = [0,1,0,0.5]

        #turn this on to draw contact points
        self.drawContacts = False

        #turn this on to draw sensors
        self.drawSensors = False

        #turn this on to save screenshots
        self.saveScreenshots = False
        self.nextScreenshotTime = 0
        self.screenshotCount = 0
        self.verbose = 0


    def display(self):
        #Put your display handler here
        #the current example draws the simulated world in grey and the
        #commanded configurations in transparent green
        self.sim.drawGL()

        """
        #draw commanded configurations
        if self.commanded_config_color != None:
            for i in xrange(self.world.numRobots()):
                r = self.world.robot(i)
                mode = self.sim.controller(i).getControlType()
                if mode == "PID":
                    q = self.sim.controller(i).getCommandedConfig()
                    #save old appearance
                    oldapps = [r.link(j).appearance().clone() for j in xrange(r.numLinks())]
                    #set new appearance
                    for j in xrange(r.numLinks()):
                        r.link(j).appearance().setColor(*self.commanded_config_color)
                    r.setConfig(q)
                    r.drawGL()
                    #restore old appearance
                    for j in xrange(r.numLinks()):
                        r.link(j).appearance().set(oldapps[j])
            glDisable(GL_BLEND)
        """

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

        #draw sensors, if enabled
        if self.drawSensors:
            for i in xrange(self.world.numRobots()):
                c = self.sim.controller(i)
                j = 0
                while j >= 0:
                    s = c.sensor(j)
                    if s.name() == '':
                        j = -1
                        break
                    if self.drawSensors == 'full':
                        s.drawGL(s.getMeasurements())
                    else:
                        s.drawGL()
                    j += 1

    def control_loop(self):
        """Overload this to perform custom control handling."""
        pass

    def idle(self):
        #Put your idle loop handler here
        #the current example simulates with the current time step self.dt
        if self.simulate:
            #Handle screenshots
            if self.saveScreenshots:
                #The following line saves movies on simulation time
                if self.sim.getTime() >= self.nextScreenshotTime:
                #The following line saves movies on wall clock time
                #if self.ttotal >= self.nextScreenshotTime:
                    self.save_screen("image%04d.ppm"%(self.screenshotCount,))
                self.screenshotCount += 1
                self.nextScreenshotTime += 1.0/30.0;

            if self.sim.getTime() == 0:
                self.sim.simulate(0)
            self.control_loop()
            self.sim.simulate(self.dt)
            self.refresh()

    def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        #the current example prints out the list of objects clicked whenever
        #you right click
        if self.verbose: print "mouse",button,state,x,y
        GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y,dx,dy):
        GLRealtimeProgram.motionfunc(self,x,y,dx,dy)

    def specialfunc(self,c,x,y):
        #Put your keyboard special character handler here
        if self.verbose: print c,"pressed"
        pass

    def print_help(self):
        #Put your help printouts here
        print "************** Help **************"
        print "?: print this help message"
        print "s: toggle simulation"
        print "m: toggle movie mode"
        print "l: toggle logging"
        print "c: toggle contact drawing"
        print "v: toggle sensor drawing"
        print "**********************************"

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        if self.verbose: print c,"pressed"
        if c=='?':
            self.print_help()
        elif c == 's':
            self.simulate = not self.simulate
            print "Simulating:",self.simulate
        elif c == 'm':
            self.saveScreenshots = not self.saveScreenshots
            print "Movie mode:",self.saveScreenshots
        elif c == 'l':
            self.sim.toggleLogging()
        elif c == 'c':
            self.drawContacts = not self.drawContacts
            if self.drawContacts:
                self.sim.enableContactFeedbackAll()
        elif c == 'v':
            if self.drawSensors == False:
                self.drawSensors = True
            elif self.drawSensors == True:
                self.drawSensors = 'full'
            else:
                self.drawSensors = False
        self.refresh()

    def click_world(self,x,y,want_points=False):
        """Helper: returns a list of (world object, point) pairs sorted in order of
        increasing distance."""
        #get the viewport ray
        (s,d) = self.click_ray(x,y)

        #run the collision tests
        collided = []
        for g in self.collider.geomList:
            (hit,pt) = g[1].rayCast(s,d)
            if hit:
                dist = vectorops.dot(vectorops.sub(pt,s),d)
                collided.append((dist,g[0],pt))
        if want_points:
            return [(g[1],g[2]) for g in sorted(collided)]
        else:
            return [g[1] for g in sorted(collided)]


