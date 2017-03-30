from klampt import *
from klampt.vis.glprogram import *
from klampt import vis
from klampt.math import vectorops

#uncomment the problem that you're testing
problem = "1a"
#problem = "1b"
#problem = "2a"
#problem = "2b"
#problem = "3"

class GLTest(GLRealtimeProgram):
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"GLTest")
        self.world = world
        self.sim = Simulator(world)
        controller = self.sim.controller(0)
        if problem == "1a" or problem == "1b":
            controller.setPIDCommand([0],[0])
        #set this to true if you want to step through the program manually
        self.step = False

    def display(self):
        self.sim.updateWorld()
        self.world.drawGL()
        if problem == "3":
            #draw target point
            glDisable(GL_LIGHTING)
            glPointSize(5.0)
            glEnable(GL_POINT_SMOOTH)
            glColor3f(1,1,0)
            glBegin(GL_POINTS)
            glVertex3f(1,0,0);
            glEnd()

    def control_loop(self):
        sim = self.sim
        world = self.world
        controller = sim.controller(0)
        robotModel = world.robot(0)
        t = self.ttotal
        #Problem 1: tune the values in this line
        if problem == "1a" or problem == "1b":
            kP = 1
            kI = 1
            kD = 1
            controller.setPIDGains([kP],[kI],[kD])
        
        #Problem 2: use setTorque to implement a control loop with feedforward
        #torques
        if problem == "2a" or problem == "2b":
            qcur = controller.getSensedConfig()
            dqcur = controller.getSensedVelocity()
            qdes = [0]
            dqdes = [0]
            robotModel.setConfig(qcur)
            robotModel.setVelocity(dqcur)
            while qcur[0]-qdes[0] > math.pi:
                qcur[0] -= 2*math.pi
            while qcur[0]-qdes[0] < -math.pi:
                qcur[0] += 2*math.pi
            ddq = vectorops.mul(vectorops.sub(qdes,qcur),100.0)
            ddq = vectorops.madd(ddq,vectorops.sub(dqdes,dqcur),20.0)
            #TODO: solve the dynamics equation to fill this in
            T = [3]
            controller.setTorque(T)

        #Problem 3: drive the robot so its end effector goes
        #smoothly toward the target point
        if problem == "3":
            target = [1.0,0.0,0.0]
            qdes = [0.7,-2.3]
            dqdes = [0,0]
            controller.setPIDCommand(qdes,dqdes)

    def keyboardfunc(self,c,x,y):
        if self.step:
            self.control_loop()
            self.sim.simulate(self.dt)

    def idle(self):
        if not self.step:
            self.control_loop()
            self.sim.simulate(self.dt)

if __name__ == "__main__":
    world = WorldModel()
    fn = ""
    if problem == "1a":
        fn = "world1.xml"
    elif problem == "1b":
        fn = "world2.xml"
    elif problem == "2a" or problem == "2b":
        fn = "world3.xml"
    elif problem == "3":
        fn = "world4.xml"
    res = world.readFile(fn)
    if not res:
        raise RuntimeError("Unable to load world "+fn)

    program = GLTest(world)

    if problem == "2b":
        #mass estimated to be 50% lighter
        m = world.robot(0).link(0).getMass()
        scale = 0.5
        m.mass *= scale
        for i in xrange(9):
            m.inertia[i] *= scale
        world.robot(0).link(0).setMass(m)
    program.run()

