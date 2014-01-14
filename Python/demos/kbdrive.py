#!/usr/bin/python

import sys
from klampt import *
from klampt.glprogram import *

#FOR DEFAULT JOINT-BY-JOINT KEYMAP: set keymap=None
keymap = None

#FOR CUSTOM KEYMAPS: set up keymap to define how keys map to velocities.
#keymap is a map from key name to (robot index,velocity vector) pairs.
#Key names can either be single keys or names of special keys
#'left','up','down','right', 'home', 'insert', 'end', and the function keys.
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
    for i in range(min(robot.numLinks(),10)):
        #up velocity
        vel = [0]*robot.numLinks()
        vel[i] = 1
        res[up[i]] = (0,vel)
        #down velocity
        vel = vel[:]
        vel[i] = -1
        res[down[i]] = (0,vel)
    return res

glutspecialmap = {
    GLUT_KEY_F1:'f1',
    GLUT_KEY_F2:'f2',
    GLUT_KEY_F3:'f3',
    GLUT_KEY_F4:'f4',
    GLUT_KEY_F5:'f5',
    GLUT_KEY_F6:'f6',
    GLUT_KEY_F7:'f7',
    GLUT_KEY_F8:'f8',
    GLUT_KEY_F9:'f9',
    GLUT_KEY_F10:'f10',
    GLUT_KEY_F11:'f11',
    GLUT_KEY_F12:'f12',
    GLUT_KEY_LEFT:'left',
    GLUT_KEY_UP:'up',
    GLUT_KEY_RIGHT:'right',
    GLUT_KEY_DOWN:'down',
    GLUT_KEY_PAGE_UP:'pageup',
    GLUT_KEY_PAGE_DOWN:'pagedown',
    GLUT_KEY_HOME:'home',
    GLUT_KEY_END:'end',
    GLUT_KEY_INSERT:'insert'
    }

glutspecialinvmap = dict((v,k) for (k,v) in glutspecialmap.items())


class MyGLViewer(GLRealtimeProgram):
    def __init__(self,world):
        global keymap
        GLRealtimeProgram.__init__(self,"My GL program")
        self.world = world
        if keymap == None:
            keymap = build_default_keymap(world)
        self.keymap = keymap
        self.current_velocities = {}
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
        #Calculate the desired velocity for each robot by adding up all
        #commands
        rvels = [[0]*self.world.robot(r).numLinks() for r in range(self.world.numRobots())]
        for (c,(r,v)) in self.current_velocities.iteritems():
            rvels[r] = vectorops.add(rvels[r],v)
        #send to the robot(s)
        for r in range(self.world.numRobots()):
            robotController = self.sim.getController(r)
            qdes = robotController.getCommandedConfig()
            qdes = vectorops.madd(qdes,rvels[r],self.dt)
            #clamp to joint limits
            (qmin,qmax) = self.world.robot(r).getJointLimits()
            for i in xrange(len(qdes)):
                qdes[i] = min(qmax[i],max(qdes[i],qmin[i]))
            robotController.setPIDCommand(qdes,rvels[r])
        return

    def idle(self):
        #Put your idle loop handler here
        #the current example simulates with the current time step self.dt
        if self.simulate and self.saveScreenshots:
            if self.ttotal >= self.nextScreenshotTime:
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
        if button==2:
            if state==0:
                print [o.getName() for o in self.click_world(x,y)]
                return
        GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def specialfunc(self,c,x,y):
        #Put your keyboard special character handler here
        if c in glutspecialinvmap:
            name = glutspecialinvmap[c]
            if name in self.keymap:
                self.current_velocities[name]=self.keymap[name]
        pass

    def specialupfunc(self,c,x,y):
        #Put your keyboard special character handler here
        print c,"unpressed"
        if c in glutspecialinvmap:
            name = glutspecialinvmap[c]
            if name in self.current_velocities:
                del self.current_velocities[name]
        pass

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        if c == 's':
            self.simulate = not self.simulate
            print "Simulating:",self.simulate
        elif c == 'm':
            self.saveScreenshots = not self.saveScreenshots
            print "Movie mode:",self.saveScreenshots
        elif c == 'h':
            print 'Available keys:',sorted(self.keymap.keys())
        elif c in self.keymap:
            self.current_velocities[c]=self.keymap[c]
        glutPostRedisplay()

    def keyboardupfunc(self,c,x,y):
        if c in self.current_velocities:
            del self.current_velocities[c]
        return

    def click_world(self,x,y):
        """Helper: returns a list of world objects sorted in order of
        increasing distance."""
        #get the viewport ray
        (s,d) = self.click_ray(x,y)
        
        #run the collision tests
        self.collider.updateFrames()
        collided = []
        for g in self.collider.geomList:
            (hit,pt) = collide.rayCast(g[1],s,d)
            if hit:
                dist = vectorops.dot(vectorops.sub(pt,s),d)
                collided.append((dist,g[0]))
        return [g[1] for g in sorted(collided)]


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
    viewer.run()
