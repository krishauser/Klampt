from OpenGL.GL import *
from robotsim import Simulator,WidgetSet,RobotPoser
from glprogram import *
import robotcollide
import simlog
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

        #turn this on to save log to disk
        self.logging = False
        self.logger = None

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
        self.world.drawGL()

        #draw commanded configurations
        if self.commanded_config_color != None:
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
            glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,self.commanded_config_color)
            for i in xrange(self.world.numRobots()):
                r = self.world.robot(i)
                mode = self.sim.controller(i).getControlType()
                if mode == "PID":
                    q = self.sim.controller(i).getCommandedConfig()
                    r.setConfig(q)
                    r.drawGL(False)
            glDisable(GL_BLEND)

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

    def control_loop(self):
        #Put your control handler here
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

            #Handle logging
            if self.logger: self.logger.saveStep()

            #Advance simulation
            self.control_loop()
            self.sim.simulate(self.dt)
            self.refresh()

    def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        #the current example prints out the list of objects clicked whenever
        #you right click
        if self.verbose: print "mouse",button,state,x,y
        GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y):
        GLRealtimeProgram.motionfunc(self,x,y)

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

    def click_world(self,x,y):
        """Helper: returns a list of world objects sorted in order of
        increasing distance."""
        #get the viewport ray
        (s,d) = self.click_ray(x,y)

        #run the collision tests
        collided = []
        for g in self.collider.geomList:
            (hit,pt) = g[1].rayCast(s,d)
            if hit:
                dist = vectorops.dot(vectorops.sub(pt,s),d)
                collided.append((dist,g[0]))
        return [g[1] for g in sorted(collided)]



class GLWidgetProgram(GLRealtimeProgram):
    """A base class for using widgets.  Right-clicks are passed onto widgets.
    
    Subclasses should call self.widgetMaster.add() upon initializiation to
    add widgets to the program.
    """
    def __init__(self,world,name="My GL Widget Program"):
        GLRealtimeProgram.__init__(self,name)
        self.world = world
        self.widgetMaster = WidgetSet()
        self.widgetButton = 2  #right-clicks are sent to widget
        self.draggingWidget = False
        self.verbose = 0

    def display(self):
        #Put your display handler here
        """
        #the next few lines draw everything but the robot
        for i in xrange(self.world.numTerrains()):
            self.world.terrain(i).drawGL()
        for i in xrange(self.world.numRigidObjects()):
            self.world.rigidObject(i).drawGL()
        for i in xrange(1,self.world.numRobots()):
            self.world.robot(i).drawGL()
        #TEMP: the widget master will draw the robot if a RobotPoseWidget
        #is added... but what if there is no such widget?
        """
        self.world.drawGL()
        self.widgetMaster.drawGL(self.viewport())

    def idle(self):
        self.widgetMaster.idle()
        if self.widgetMaster.wantsRedraw():
            self.refresh()

    def mousefunc(self,button,state,x,y):
        if self.verbose: print "mouse",button,state,x,y
        if button==self.widgetButton:
            if state==0:
                if self.widgetMaster.beginDrag(x,self.height-y,self.viewport()):
                    self.draggingWidget = True
            else:
                if self.draggingWidget:
                    self.widgetMaster.endDrag()
                    self.draggingWidget = False
            if self.widgetMaster.wantsRedraw():
                self.refresh()
            self.lastx,self.lasty = x,y
            return
        GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y):
        if self.draggingWidget:
            self.widgetMaster.drag(x-self.lastx,self.lasty-y,self.viewport())
            if self.widgetMaster.wantsRedraw():
                self.refresh()
            self.lastx,self.lasty = x,y
        else:
            res = self.widgetMaster.hover(x,self.height-y,self.viewport())
            if self.widgetMaster.wantsRedraw():
                self.refresh()
            GLRealtimeProgram.motionfunc(self,x,y)

    def specialfunc(self,c,x,y):
        #Put your keyboard special character handler here
        if self.verbose: print c,"pressed"
        self.widgetMaster.keypress(c)
        self.refresh()

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        if self.verbose: print c,"pressed"
        self.widgetMaster.keypress(c)
        self.refresh()
