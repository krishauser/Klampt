"""Defines GLWorldPlugin and GLSimulationPlugin, which may be useful as base
classes for your own plugins.
"""

from . import glinit
from OpenGL.GL import *
from ..robotsim import WidgetSet,RobotPoser
from .glinterface import GLPluginInterface
from ..model import collide
from ..math import vectorops
from ..sim.simulation import SimpleSimulator
import sys

class GLWorldPlugin(GLPluginInterface):
    """A program that draws a WorldModel.  This is a pretty bare plugin class,
    and users will want to override the GLPluginInterface methods to provide
    any reasonable functionality.

    Attributes:
        world (WorldModel): a world instance provided on construction.  All elements
            in the world must be instantiated already.
        collider (WorldCollider): used for the click_world helper.
    """
    def __init__(self,world):
        """Args:
            world (WorldModel): the world to draw
        """
        GLPluginInterface.__init__(self)
        self.world = world
        #Put your initialization code here
        #the current example creates a collision class, simulator, 
        #simulation flag, and screenshot flags
        self.collider = collide.WorldCollider(world)

    def display(self):
        self.world.drawGL()

    def click_world(self,x,y,want_points=False):
        """Helper: returns a list of objects sorted in order of
        increasing distance.  This will be useful for handling mouse_click events.

        Args:
            x,y (int): the screen coordinates of a point being clicked
            want_points (bool, optional): if true, the return list contains the
                point of contact.

        Returns:
            (list): If want_points=False, a list of world objects.
                If want_points=True, a list of (world object, point) pairs
        """
        #get the viewport ray
        (s,d) = self.view.click_ray(x,y)

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


class GLSimulationPlugin(GLWorldPlugin):
    """A program that runs a simulation given a world.

    Attributes:
        sim (SimpleSimulator): the simulator that this plugin will be using
        simulate (bool): set this to True to start simulating.
        dt (float): the timestep for each idle call.
        drawContacts (bool): set this to True to draw contact points
        drawSensors (bool): set this to True to draw sensors
        saveScreenshots (bool): set this to True if frames should be saved to
            disk.
        verbose (int, optional): set to 1 if you wish to get printouts of the
            event loop

    Subclasses should overload self.control_loop() and put whatever control
    loop you desire inside.  Note: in this loop you should interact with
    self.sim.controller(0), not self.world.robot(0).  self.world is simply
    a model and does not have a direct relation to the simulation.
    """
    def __init__(self,world):
        """Args:
            world (WorldModel): the world to simulate
        """
        GLWorldPlugin.__init__(self,world)
        self.sim = SimpleSimulator(world)
        self.simulate = False
        self.dt = 0.02

        #turn this on to draw contact points
        self.drawContacts = False

        #turn this on to draw sensors
        self.drawSensors = False

        #turn this on to save screenshots
        self.saveScreenshots = False
        self.nextScreenshotTime = 0
        self.screenshotCount = 0
        self.verbose = 0

        self.htmlSharePath = None
        
        def toggle_simulate():
            self.simulate = not self.simulate
            print("Simulating:",self.simulate)
        def toggle_movie_mode():
            self.saveScreenshots = not self.saveScreenshots
            print("Movie mode:",self.saveScreenshots)
        def toggle_draw_contacts():
            self.drawContacts = not self.drawContacts
            if self.drawContacts:
                self.sim.enableContactFeedbackAll()
        def toggle_draw_sensors():
            if self.drawSensors == False:
                self.drawSensors = True
            elif self.drawSensors == True:
                self.drawSensors = 'full'
            else:
                self.drawSensors = False
        def single_step():
            print("Advancing by 0.01s")
            self.sim_step(0.01)
        self.add_action(toggle_simulate,'Toggle simulation','s')
        self.add_action(single_step,'Step simulation',' ',)
        #self.add_action(toggle_movie_mode,'Toggle movie mode','m')
        self.add_action(self.sim.toggleLogging,'Toggle simulation logging','l')
        self.add_action(toggle_draw_contacts,'Toggle draw contacts','c')
        self.add_action(toggle_draw_sensors,'Toggle draw sensors','v')

    def initialize(self):
        #match window refresh rate
        self.dt = self.window.program.dt
        return GLWorldPlugin.initialize(self)

    def display(self):
        """Override this to do custom drawing"""
        #This draws the simulated world in grey and the
        #commanded configurations in transparent green
        self.sim.drawGL()

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
            for i in range(maxid):
                for j in range(i+1,maxid):
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
            for i in range(self.world.numRobots()):
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
        return True

    def control_loop(self):
        """Overload this to perform custom control handling."""
        pass

    def sim_step(self,dt=None):
        """Advance the simulation and update the GUI"""
        if dt is None:
            dt = self.dt
        if self.sim.getTime() == 0:
            self.sim.simulate(0)

        #Handle screenshots
        if self.saveScreenshots:
            #The following line saves movies on simulation time
            if self.sim.getTime() >= self.nextScreenshotTime:
            #The following line saves movies on wall clock time
            #if self.ttotal >= self.nextScreenshotTime:
                self.save_screen("image%04d.ppm"%(self.screenshotCount,))
            self.screenshotCount += 1
            self.nextScreenshotTime += 1.0/30.0;

        if self.htmlSharePath:
            self.htmlSharePath.animate()

        self.control_loop()
        self.sim.simulate(dt)
        self.refresh()

    def idle(self):
        #Put your idle loop handler here
        #the current example simulates with the current time step self.dt
        if self.simulate:
            self.sim_step()
        return True

    def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        #the current example prints out the list of objects clicked whenever
        #you right click
        if self.verbose: print("mouse",button,state,x,y)
        return GLPluginInterface.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y,dx,dy):
        return GLPluginInterface.motionfunc(self,x,y,dx,dy)



