from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import math
from klampt.cspace import CSpace,MotionPlan
from klampt.glprogram import GLProgram
from klampt import vectorops

problem = "1"
#problem = "2"

def interpolate(a,b,u):
    """Interpolates linearly between a and b"""
    return vectorops.madd(a,vectorops.sub(b,a),u)

class Circle:
    def __init__(self,x=0,y=0,radius=1):
        self.center = (x,y)
        self.radius = radius
        
    def contains(self,point):
        return (vectorops.distance(point,self.center) <= self.radius)

    def drawGL(self,res=0.01):
        numdivs = int(math.ceil(self.radius*math.pi*2/res))
        glBegin(GL_TRIANGLE_FAN)
        glVertex2f(*self.center)
        for i in xrange(numdivs+1):
            u = float(i)/float(numdivs)*math.pi*2
            glVertex2f(self.center[0]+self.radius*math.cos(u),self.center[1]+self.radius*math.sin(u))
        glEnd()

class CircleObstacleCSpace(CSpace):
    def __init__(self):
        CSpace.__init__(self)
        #set bounds
        self.bound = [(0.0,1.0),(0.0,1.0)]
        #set collision checking resolution
        self.eps = 1e-3
        #setup a robot with radius 0.05
        self.robot = Circle(0,0,0.05)
        #set obstacles here
        self.obstacles = []

    def addObstacle(self,circle):
        self.obstacles.append(circle)
    
    def feasible(self,q):
        #bounds test
        if not CSpace.feasible(self,q): return False
        #TODO: Problem 1: implement your feasibility tests here
        #currently, only the center is checked, so the robot collides
        #with boundary and obstacles
        for o in self.obstacles:
            if o.contains(q): return False
        return True

    def drawObstaclesGL(self):
        glColor3f(0.2,0.2,0.2)
        for o in self.obstacles:
            o.drawGL()

    def drawRobotGL(self,q):
        glColor3f(0,0,1)
        newc = vectorops.add(self.robot.center,q)
        c = Circle(newc[0],newc[1],self.robot.radius)
        c.drawGL()

class RigidBarCSpace(CSpace):
    """A 3D configuration space for a bar of length L"""
    def __init__(self):
        CSpace.__init__(self)
        #set bounds
        self.bound = [(0.0,1.0),(0.0,1.0),(0.0,math.pi*2)]
        #set collision checking resolution
        self.eps = 1e-3
        #setup a bar with length 0.1
        self.L = 0.1
        #set obstacles here
        self.obstacles = []

    def addObstacle(self,circle):
        self.obstacles.append(circle)

    def distance(self,a,b):
        #TODO: return a distance metric between a and b that correctly
        #takes rotation into account.  Current implementation assumes
        #a cartesian space
        return vectorops.distance(a,b) 

    def interpolate(self,a,b,u):
        #TODO: return a configuration that interpolates between a and b
        #at parameter u.  Current implementation is linear.
        return interpolate(a,b,u)

    def endpoints(self,q):
        """Returns the bar's endpoints for the configuration q"""
        dx,dy = 0.5*self.L*math.cos(q[2]), 0.5*self.L*math.sin(q[2])
        p1 = (q[0]-dx,q[1]-dy)
        p2 = (q[0]+dx,q[1]+dy)
        return (p1,p2)
    
    def feasible(self,q):
        #bounds test
        if not CSpace.feasible(self,(q[0],q[1],0)): return False
        
        #get the endpoints of the bar
        p1,p2 = self.endpoints(q)
        #TODO: implement your feasibility test here: the current implementation
        #checks endpoints, but doesnt consider collisions with the
        #intermediate segment between the endpoints
        for o in self.obstacles:
            if o.contains(p1) or o.contains(p2): return False
        return True

    def drawObstaclesGL(self):
        glColor3f(0.2,0.2,0.2)
        for o in self.obstacles:
            o.drawGL()

    def drawRobotGL(self,q):
        #get the endpoints of the bar
        p1,p2 = self.endpoints(q)
        
        glColor3f(0,0,1)
        glLineWidth(5.0)
        glPointSize(8.0)
        glEnable(GL_POINT_SMOOTH)
        glBegin(GL_POINTS)
        glVertex2f(p1[0],p1[1])
        glVertex2f(p2[0],p2[1])
        glEnd()
        glBegin(GL_LINES)
        glVertex2f(p1[0],p1[1])
        glVertex2f(p2[0],p2[1])
        glEnd()
        glLineWidth(1.0)
        glPointSize(1.0)

class CSpaceObstacleProgram(GLProgram):
    def __init__(self,space,start=(0.1,0.5),goal=(0.9,0.5)):
        GLProgram.__init__(self)
        self.space = space
        #PRM planner
        MotionPlan.setOptions(type="prm",knn=10,connectionThreshold=0.1)
        self.optimizingPlanner = False
        
        #FMM* planner
        #MotionPlan.setOptions(type="fmm*")
        #self.optimizingPlanner = True
        
        #RRT planner
        #MotionPlan.setOptions(type="rrt",perturbationRadius=0.25,bidirectional=True)
        #self.optimizingPlanner = False

        #RRT* planner
        #MotionPlan.setOptions(type="rrt*")
        #self.optimizingPlanner = True
        
        #random-restart RRT planner
        #MotionPlan.setOptions(type="rrt",perturbationRadius=0.25,bidirectional=True,shortcut=True,restart=True,restartTermCond="{foundSolution:1,maxIters:1000}")
        #self.optimizingPlanner = True

        #OMPL planners:
        #Tested to work fine with OMPL's prm, lazyprm, prm*, lazyprm*, rrt, rrt*, rrtconnect, lazyrrt, lbtrrt, sbl, bitstar.
        #Note that lbtrrt doesn't seem to continue after first iteration.
        #Note that stride, pdst, and fmt do not work properly...
        #MotionPlan.setOptions(type="ompl:rrt",suboptimalityFactor=0.1,knn=10,connectionThreshold=0.1)
        #self.optimizingPlanner = True
        
        self.planner = MotionPlan(space)
        self.start=start
        self.goal=goal
        self.planner.setEndpoints(start,goal)
        self.path = []
        self.G = None
        
    def keyboardfunc(self,key,x,y):
        if key==' ':
            if self.optimizingPlanner or not self.path:
                print "Planning 1..."
                self.planner.planMore(1)
                self.path = self.planner.getPath()
                self.G = self.planner.getRoadmap()
                glutPostRedisplay()
        elif key=='p':
            if self.optimizingPlanner or not self.path:
                print "Planning 100..."
                self.planner.planMore(100)
                self.path = self.planner.getPath()
                self.G = self.planner.getRoadmap()
                glutPostRedisplay()
       
    def display(self):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0,1,1,0,-1,1);
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        glDisable(GL_LIGHTING)
        self.space.drawObstaclesGL()
        if self.path:
            #draw path
            glColor3f(0,1,0)
            glBegin(GL_LINE_STRIP)
            for q in self.path:
                print q
                glVertex2f(q[0],q[1])
            glEnd()
            for q in self.path:
                self.space.drawRobotGL(q)
        else:
            self.space.drawRobotGL(self.start)
            self.space.drawRobotGL(self.goal)

        if self.G:
            #draw graph
            V,E = self.G
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
            glColor4f(0,0,0,0.5)
            glPointSize(3.0)
            glBegin(GL_POINTS)
            for v in V:
                glVertex2f(v[0],v[1])
            glEnd()
            glColor4f(0.5,0.5,0.5,0.5)
            glBegin(GL_LINES)
            for (i,j) in E:
                glVertex2f(V[i][0],V[i][1])
                glVertex2f(V[j][0],V[j][1])
            glEnd()
            glDisable(GL_BLEND)
    
if __name__=='__main__':
    space = None
    start = None
    goal = None
    if problem == "1":
        space = CircleObstacleCSpace()
        space.addObstacle(Circle(0.5,0.5,0.36))
        start=(0.06,0.5)
        goal=(0.94,0.5)
    elif problem == "2":
        space = RigidBarCSpace()
        space.addObstacle(Circle(0.5,0.5,0.4))
        start=(0.1,0.1,0.0)
        goal=(0.9,0.9,6.20)
    program = CSpaceObstacleProgram(space,start,goal)
    program.name = "Lab 5"
    program.width = program.height = 640
    program.run()
    
