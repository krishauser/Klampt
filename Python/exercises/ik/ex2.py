import math
from klampt import *
from klampt.glprogram import *
from klampt import vectorops
from klampt import so3
#import numpy as np


class GLIKTest(GLRealtimeProgram):
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"GLIKTest")
        self.world = world
        self.position = (0,0,0)

    def display(self):
        #draw the world
        self.world.drawGL()
        #draw the end-effector position and the desired position
        link = 7
        localpos = (0.17,0,0)
        robot = self.world.robot(0)
        eepos = robot.getLink(link).getWorldPosition(localpos)
        glDisable(GL_LIGHTING)
        glDisable(GL_DEPTH_TEST)
        glPointSize(5.0)
        glEnable(GL_POINT_SMOOTH)
        glBegin(GL_POINTS)
        glColor3f(1,1,0)
        glVertex3fv(eepos)
        glColor3f(1,1,1)
        glVertex3fv(self.position)
        glEnd()
        glEnable(GL_DEPTH_TEST)

    def solve_ik(self,robotlink,localpos,worldpos):
        """TODO: solve inverse kinematics to place the 3D point localpos
        on robotlink (a RobotLink instance) at the 3D position worldpos
        in the world coordinate frame.
        
        Returns the robot configuration that solves the IK problem.
        """
        linkindex = robotlink.index
        #hint: your code should look like this
        #obj = IKObjective()
        # ... do something to set up the objective, see robot/src/robotik.h ...
        #s = IKSolver(robotlink.getRobot())
        #s.add(obj)
        #maxIters = 100
        #tol = 1e-3
        # ... optionally you can set an initial configuration like so:
        #robotlink.getRobot().setConfig([0]*robotlink.getRobot().numLinks())
        #(res,iters) = s.solve(maxIters,tol);
        #return robotlink.getRobot().getConfig()

        #right now this just sets the zero configuration
        q = [0]*robotlink.getRobot().numLinks()
        return q
    

    def idle(self):
        #set the desired position self.position to move in a circle
        r = 0.4
        self.position = (0.8,r*math.cos(self.ttotal),0.7+r*math.sin(self.ttotal))
        link = 7
        localpos = (0.17,0,0)
        robot = self.world.robot(0)
        q = self.solve_ik(robot.getLink(link),localpos,self.position)
        robot.setConfig(q)
    

if __name__ == "__main__"
    world = WorldModel()
    res = world.readFile("ex2_file.xml")
    if not res: raise RuntimeError("Unable to load world file")
    GLIKTest(world).run()
