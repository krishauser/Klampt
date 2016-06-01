from robotsim import Simulator,WidgetSet,RobotPoser
import robotcollide
import simlog
import sys
import camera
import se3
import so3
import vectorops
import math
import time

class WebSimulationProgram:
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
    def __init__(self,world,name="My Web simulation program"):
        """Arguments:
        - world: a RobotWorld instance.
        """
        self.world = world
        #Put your initialization code here
        #the current example creates a collision class, simulator, 
        #simulation flag, and screenshot flags
        self.collider = robotcollide.WorldCollider(world)
        self.sim = Simulator(world)
        self.simulate = False
        self.commanded_config_color = [0,1,0,0.5]

        #turn this on to draw contact points
        self.drawContacts = False

        #turn this on to save screenshots
        self.saveScreenshots = False
        self.nextScreenshotTime = 0
        self.screenshotCount = 0
        self.verbose = 0

        self.ttotal = 0.0
        self.fps = 50
        self.dt = 1.0/self.fps
        self.running = True
        self.counter = 0
        self.lasttime = time.time()

        #turn this on to save log to disk
        self.logging = False
        self.logger = None
        
    def run(self):
	print "in run"
	while True:
		self.idlefunc()
       
    def display_screen(self):
        """Do drawing of objects on screen"""
        pass

    def save_screen(self,fn):
        """Saves a screenshot"""
        try:
            import Image
        except ImportError:
            print "Cannot save screens to disk, the Python Imaging Library is not installed"
            return
        screenshot = glReadPixels( 0,0, self.width, self.height, GL_RGBA, GL_UNSIGNED_BYTE)
        im = Image.frombuffer("RGBA", (self.width, self.height), screenshot, "raw", "RGBA", 0, 0)
        print "Saving screen to",fn
        im.save(fn)

    def beginLogging(self,state_fn="simulation_state.csv",contact_fn="simulation_contact.csv"):
        self.logging = True
        self.logger = SimLogger(self.sim,state_fn,contact_fn)
    def endLogging(self):
        self.logging = False
        self.logger = None
    def pauseLogging(self,paused=True):
        self.logging=not paused
        
    def display(self):
        #Put your display handler here
        #the current example draws the simulated world in grey and the
        #commanded configurations in transparent green
        self.sim.updateWorld()

        #jString=self.world.getJSON();
        #todo: ship out json here
        #print "here's the huge string!" + jString;
    def control_loop(self):
        #Put your control handler here
        pass

    # idle callback
    def idlefunc (self):
        t = self.dt - (time.time() - self.lasttime)
        if (t > 0):
            time.sleep(t)
        
        self.ttotal += self.dt
        self.counter += 1

        #do something random
        self.idle()
        
        self.lasttime = time.time()

        #glutPostRedisplay()
        self.display()

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

            #Handle logging
            if self.logger: self.logger.saveStep()

            #Advance simulation
	    self.starttime=time.time(); 
	
	    self.control_loop()
            self.sim.simulate(self.dt)

	    self.secs = time.time()- self.starttime
            self.msecs = self.secs * 1000  # millisecs
	    print "computing simulation took: " + str(self.msecs) + " ms"


    def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        #the current example prints out the list of objects clicked whenever
        #you right click
        if self.verbose: print "mouse",button,state,x,y
        #GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y):
        pass

    def specialfunc(self,c,x,y):
        #Put your keyboard special character handler here
        if self.verbose: print c,"pressed"
        pass

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        if self.verbose: print c,"pressed"
        if c == 's':
            self.simulate = not self.simulate
            print "Simulating:",self.simulate
        elif c == 'm':
            self.saveScreenshots = not self.saveScreenshots
            print "Movie mode:",self.saveScreenshots
        elif c == 'l':
            if self.logging:
                self.pauseLogging()
            else:
                if self.logger==None:
                    self.beginLogging()
                else:
                    self.pauseLogging(False)
        elif c == 'c':
            self.drawContacts = not self.drawContacts
            if self.drawContacts:
                self.sim.enableContactFeedbackAll()
        self.refresh()


