import os
import sys
from klampt import *
from klampt import vis
from klampt.math import vectorops,so3,se3
from klampt.model.robotinfo import RobotInfo
from klampt.vis.glrobotprogram import *
import importlib
import time

SPLIT_SCREEN_TEST = False

class MyGLViewer(GLSimulationPlugin):
    """Simulates some functionality of the SimTest program.
    Shows how to subclass GLSimulationPlugin and apply hooks to SimpleSimulation for applying a
    spring-like force to a simulation body.
    """
    def __init__(self,world):
        GLSimulationPlugin.__init__(self,world)
        self.world = world
        self.sim.enableContactFeedbackAll()
        self.forceApplicationMode = False
        self.statusLog = []
        
    def display(self):
        GLSimulationPlugin.display(self)
            
        #draw force springs if using
        if self.forceApplicationMode:
            self.sim.updateWorld()
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

    def display_screen(self):
        glDisable(GL_LIGHTING)
        h = 20
        self.draw_text((20,h),"%.3f"%(self.sim.getTime(),),color=[1,1,1])
        h += 20
        for (t,s) in self.statusLog:
            self.draw_text((20,h),"Sim status "+"%.3f"%(t,)+": "+self.sim.getStatusString(s),color=[1,0,0])
            h += 20
        if self.sim.hadPenetration(-1,-1):
            self.draw_text((20,h),"Meshes penetrating, simulation may be unstable",color=[1,0,0])
            h += 20

    def control_loop(self):
        #you can put more control code here
        pass

    def idle(self):
        #Put your idle loop handler here
        GLSimulationPlugin.idle(self)
        t = self.sim.getTime()
        if self.sim.getStatus() != 0:
            self.statusLog.append((t,self.sim.getStatus()))
        while len(self.statusLog) > 0 and self.statusLog[0][0] < t-1:
            self.statusLog.pop(0)
        return True

    def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        #the current example prints out the list of objects clicked whenever
        #you right click, and applies forces
        if button==2:
            if state==0:
                self.sim.updateWorld()
                objs = self.click_world(x,y,want_points=True)
                if len(objs) > 0:
                    print("Clicked:",[o[0].getName() for o in objs])
                    obj,pt = objs[0]
                    if isinstance(obj,(RobotModelLink,RigidObjectModel)):
                        print("Clicked, turning on force application mode with object",obj.getName())
                        self.forceApplicationMode = True
                        self.addForceSpring(obj,pt)
                        return True
            elif self.forceApplicationMode:
                print("Turning off force application mode")
                self.forceApplicationMode = False
                self.sim.hooks.pop(-1)
                return True
        return GLSimulationPlugin.mousefunc(self,button,state,x,y)
        
    def motionfunc(self,x,y,dx,dy):
        if self.forceApplicationMode:
            self.moveForceSpring(x,y)
            self.refresh()
            return True
        else:
            return GLSimulationPlugin.motionfunc(self,x,y,dx,dy)

    def moveForceSpring(self,x,y):
        self.sim.updateWorld()
        (s,d) = self.view.click_ray(x,y)
        u = vectorops.dot(vectorops.sub(self.forceAnchorPoint,s),d)
        self.forceAnchorPoint = vectorops.madd(s,d,u)

    def addForceSpring(self,obj,worldpt):
        self.sim.updateWorld()
        self.forceObject = obj
        Ro,to = obj.getTransform()
        self.forceAnchorPoint = worldpt
        self.forceLocalPoint = se3.apply(se3.inv((Ro,to)),self.forceAnchorPoint)
        self.sim.addHook(obj,lambda obj:self.simulateForceSpring())

    def simulateForceSpring(self,kP = 10.0):
        if not self.forceApplicationMode: return
        self.sim.updateWorld()
        body = self.sim.body(self.forceObject)
        T = self.forceObject.getTransform()
        wp = se3.apply(T,self.forceLocalPoint)
        f = vectorops.mul(vectorops.sub(self.forceAnchorPoint,wp),kP)
        body.applyForceAtLocalPoint(f,self.forceLocalPoint)
        #alternate approach: get wrench about com, use applyWrench
        #momentArmLocal = vectorops.sub(self.forceLocalPoint,self.forceObject.getMass().getCom())
        #momentArmWorld = so3.apply(T[0],momentArmLocal)
        #w = vectorops.cross(momentArmWorld,f)
        #print "Applying wrench",(f,w)
        #body.applyWrench(f,w)

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the GLSimulationPlugin class uses h,s,m,l,c
        return GLSimulationPlugin.keyboardfunc(self,c,x,y)


def main():
    print("================================================================================")
    print(sys.argv[0]+": Simulates a robot file and Python controller")
    if len(sys.argv)<=1:
        print("USAGE: klampt_sim [world_file] [trajectory (.traj/.path) or controller (.py)]")
        print("         [robotinfo file]")
        print()
        print("  Try: klampt_sim athlete_plane.xml motions/athlete_flex_opt.path ")
        print("       [run from Klampt-examples/data/]")
        print("  Try: klampt_sim ur5/ur5_with_robotiq85_sim.json ")
        print("       [run from Klampt-examples/robotinfo/]")
    print("================================================================================")
    if len(sys.argv)<=1:
        exit()

    world = WorldModel()
    #load up any world items, control modules, or paths
    robotinfos = dict()
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
        elif file_ext=='.json':
            robotinfo = RobotInfo.load(fn)
            if robotinfo.modelFile is not None:
                robotinfos[world.numRobots()] = robotinfo
                model = robotinfo.klamptModel()
                print("Loaded robot from",robotinfo.modelFile)
                world.add(robotinfo.name,model)
            else:
                print("RobotInfo {} has no robotmodel?".format(fn))
        else:
            res = world.readFile(fn)
            if not res:
                print("Unable to load model "+fn)
                print("Quitting...")
                sys.exit(1)
    viewer = MyGLViewer(world)
    for index,robotinfo in robotinfos.items():
        if robotinfo.simulatorFile is not None:
            robotinfo.configureSimulator(viewer.sim,index)

    for i,c in enumerate(control_modules):
        if isinstance(c,str):
            from klampt.control.blocks import trajectory_tracking
            from klampt.model.trajectory import RobotTrajectory
            from klampt.io import loader
            traj = loader.load('Trajectory',c)
            rtraj = RobotTrajectory(world.robot(i),traj.times,traj.milestones)
            #it's a path file, try to load it
            controller = trajectory_tracking.TrajectoryPositionController(rtraj)
        else:
            try:
                maker = c.make
            except AttributeError:
                print("Module",c.__name__,"must have a make() method")
                print("Quitting...")
                sys.exit(1)
            controller = maker(world.robot(i))
        viewer.sim.setController(world.robot(i),controller)
    
    vis.setWindowTitle("Klamp't Simulation Tester")
    if SPLIT_SCREEN_TEST:
        viewer2 = MyGLViewer(world)
        vis.setPlugin(viewer)
        vis.addPlugin(viewer2)
        viewer2.window.broadcast = True
        vis.show()
        while vis.shown():
            time.sleep(0.01)
        vis.kill()
    else:
        vis.run(viewer)

if __name__ == '__main__':
    main()