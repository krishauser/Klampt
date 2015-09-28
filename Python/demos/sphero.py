#!/usr/bin/python

import sys
from klampt import *
from klampt.glprogram import *
from OpenGL.GLUT import *
import math

#Sphero local axes: [0,0,1] is "turn", [1,0,0] is "drive"
#can be 'left','up','down','right', 'home', 'insert', 'end', and the function keys.
keymap = {'up':(0,[-1,0,0]),'down':(0,[1,0,0]),'left':(0,[0,0,-1]),'right':(0,[0,0,1])}


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


def euler_zyx_moments(theta):
    """For the zyx euler angles theta=(rz,ry,rx), produces a matrix A such that
    A*dtheta is the angular velocities when dtheta is the rate of change of the
    euler angles"""
    eu = [0,0,1]
    ev = [0,1,0]
    ew = [1,0,0]
    Ru = so3.rotation([0,0,1],theta[0])
    Rv = so3.rotation([0,1,0],theta[1])
    col1 = eu
    col2 = so3.apply(Ru,ev)
    col3 = so3.apply(Ru,so3.apply(Rv,ew))
    #col1 = [0,0,1]
    #col2 = [c0 -s0 0] [0] = [-s0]
    #       [s0 c0  0]*[1]   [c0 ]
    #       [0  0   1] [0]   [0  ]
    #col3 = Ru*[c1  0 s1] [1] = Ru*[c1 ] = [c1c0]
    #          [0   1 0 ]*[0]      [0  ]   [c1s0]
    #          [-s1 0 c1] [0]      [-s1]   [-s1 ]
    #A = [ 0 -s0 c1c0]
    #    [ 0  c0 c1s0]
    #    [ 1  0  -s1 ]    
    return zip(col1,col2,col3)

def euler_zyx_moments_inv(theta):
    """Returns the inverse of the matrix returned by the above procedure"""
    c0 = math.cos(theta[0])
    s0 = math.sin(theta[0])
    c1 = math.cos(theta[1])
    s1 = math.sin(theta[1])
    #A = [ 0 -s0 c1c0]
    #    [ 0  c0 c1s0]
    #    [ 1  0  -s1 ]
    #det(A) = -c1
    #A^-1 = 1/c1*[ s1c0 s0s1 c1  ]
    #            [-c1s0 c1c0 0   ]
    #            [ c0   s0   0   ]
    #A^-1*A = 1/c1*[c1 -s0s1c0+c0s0s1 s1c1c0^2+s1c1s0^2-c1s1 ] = [1 0 0]
    #              [0   c1s0^2+c1c0^2 -c0c1^2s0+s0c1^2c0     ]   [0 1 0]
    #              [0   -s0c0+s0c0    c1c0^2+c1s0^2          ]   [0 0 1]
    sec1 = 1.0/c1
    return [[c0*s1/c1,s0*s1/c1,1],
            [-s0,c0,0],
            [c0/c1,s0/c1,0]]


class Emulator:
    def __init__(self,sim,robotIndex = 0):
        self.sim = sim
        self.robotIndex = robotIndex
        self.controller = sim.controller(robotIndex)
        self.robot = sim.getWorld().robot(robotIndex)
        #indices: turn and drive, respectively
        self.velocityLimits = [180*math.pi/180,1080*math.pi/180]
        self.accelLimits = [360*math.pi/180,2080*math.pi/180]
        self.motorSpeeds = [0,0]
        #velocity locking gain
        self.velocityLockGain = 0.1
        #rolling friction
        self.rollingFrictionCoeff = 0.1
        #timestep
        self.dt = 0.01
    
    def send_command(self,twist):
        assert twist[1] == 0
        #compute the angular velocity of the shell in the motor frame
        motorBody = self.sim.body(self.robot.link(5))
        shellBody = self.sim.body(self.robot.link(8))
        motorTwist = motorBody.getVelocity()
        shellTwist = shellBody.getVelocity()
        motorXform = motorBody.getTransform()
        shellXform = shellBody.getTransform()
        shellRelativeAvel = so3.apply(so3.inv(motorXform[0]),vectorops.sub(shellTwist[0],motorTwist[0]))
        #print "Relative angular vel",shellRelativeAvel

        desiredTurnSpeed = twist[2]*self.velocityLimits[0]
        desiredDriveSpeed = 0
        if twist[0] == 0 or twist[0]*self.motorSpeeds[1] < 0: #stop
            desiredDriveSpeed = 0
        else:
            desiredDriveSpeed =  self.motorSpeeds[1]+twist[0]*self.accelLimits[1]*self.dt
        #print "Turn des",desiredTurnSpeed, "drive des",desiredDriveSpeed
        #clamp speeds to limits
        desiredTurnSpeed = max(-self.velocityLimits[0],min(desiredTurnSpeed,self.velocityLimits[0]))
        desiredDriveSpeed = max(-self.velocityLimits[1],min(desiredDriveSpeed,self.velocityLimits[1]))
        terr = desiredTurnSpeed - self.motorSpeeds[0]
        derr = desiredDriveSpeed - self.motorSpeeds[1]
        #clamp desired accelerations to limits
        terr = max(-self.accelLimits[0]*self.dt,min(terr,self.accelLimits[0]*self.dt))
        derr = max(-self.accelLimits[1]*self.dt,min(derr,self.accelLimits[1]*self.dt))
        self.motorSpeeds[0] += terr
        self.motorSpeeds[1] += derr

        #apply locked velocity control to bring shell up to relative speed
        #this is the desired angular velocity of the shell in the motor
        #coordinates
        desiredShellAvel = [self.motorSpeeds[1],0,self.motorSpeeds[0]]
        #print "Desired angular vel",desiredShellAvel
        relativeVelError = vectorops.sub(desiredShellAvel,shellRelativeAvel)
        wrenchlocal = vectorops.mul(relativeVelError,self.velocityLockGain)
        #local wrench is k*(wdes-wrel)
        wrench = so3.apply(motorXform[0],wrenchlocal)
        #print "Wrench to shell",wrench
        motorBody.applyWrench([0,0,0],vectorops.mul(wrench,-1.0))
        shellBody.applyWrench([0,0,0],wrench)
        #disable PID controllers
        self.controller.setTorque([0,0,0])
        
        #apply rolling friction forces
        shellBody.applyWrench([0,0,0],vectorops.mul(shellTwist[0],-self.rollingFrictionCoeff))
        return


class MyGLViewer(GLRealtimeProgram):
    def __init__(self,world):
        global keymap
        GLRealtimeProgram.__init__(self,"My GL program")
        self.world = world
        self.keymap = keymap
        self.current_velocities = {}
        #Put your initialization code here
        #the current example creates a collision class, simulator, 
        #simulation flag, and screenshot flags
        self.collider = robotcollide.WorldCollider(world)
        self.sim = Simulator(world)
        self.simulate = False
        #initialize emulators
        self.spheros = [Emulator(self.sim,r) for r in range(world.numRobots())]

        self.saveScreenshots = False
        self.nextScreenshotTime = 0
        self.screenshotCount = 0

    def display(self):
        #Put your display handler here
        #the current example draws the simulated world in grey and the
        #commanded configurations in transparent green
        self.sim.updateWorld()
        self.world.drawGL()
        return

    def control_loop(self):
        #Calculate the desired velocity for each robot by adding up all
        #commands
        rvels = [[0]*3 for r in range(self.world.numRobots())]
        for (c,(r,v)) in self.current_velocities.iteritems():
            rvels[r] = vectorops.add(rvels[r],v)
        #send to the robot(s)
        for r in range(self.world.numRobots()):           
            self.spheros[r].send_command(rvels[r])
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
        if c in glutspecialmap:
            name = glutspecialmap[c]
            print name,"pressed"
            if name in self.keymap:
                self.current_velocities[name]=self.keymap[name]
        pass

    def specialupfunc(self,c,x,y):
        #Put your keyboard special character handler here
        if c in glutspecialmap:
            name = glutspecialmap[c]
            #print name,"unpressed"
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
        sys.argv = [sys.argv[0],'sphero_data/sphero.xml']
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)
    viewer = MyGLViewer(world)
    viewer.run()
