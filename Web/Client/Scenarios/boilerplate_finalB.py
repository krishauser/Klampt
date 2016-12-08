import math
import sys
sys.path.append("Web/Server")
sys.path.append('Web/Client/Scenarios/final')
from klampt import *
from common import *
import time
import random
import traceback
import sensor
import kviz

class EventB:
    """This class does the event logic"""
    def __init__(self,sim):
        self.difficulty = stub.difficulty
        self.score = 0
        self.forfeit = False
        
        self.lastpenaltytime = 0
        self.tcontroller = 0

        numballs = 10
        self.ball = 0
        if self.difficulty == 'easy':
            self.balltimes = [2+i*2 for i in range(numballs)]
        elif self.difficulty == 'medium':
            self.balltimes = [1+i*1.5 for i in range(numballs)]
        elif self.difficulty == 'hard':
            self.balltimes = [1+i*1 for i in range(numballs)]
        else:
            raise ValueError("Invalid difficulty value "+str(difficulty))
        self.activeBalls = [False]*numballs
        self.endtime = self.balltimes[-1] + 3.0

        #activate collision feedback
        robot = sim.world.robot(0)
        for i in range(robot.numLinks()):
            for j in range(sim.world.numTerrains()):
                sim.enableContactFeedback(robot.link(i).getID(),sim.world.terrain(j).getID())
        
    def mark_controller_time(self,tcontroller):
        self.tcontroller += tcontroller
        
    def update(self,dt,sim):
        if self.forfeit: return
        t = sim.getTime()
        if t > self.lastpenaltytime + 1.0:
            if self.tcontroller > 5:
                print "Event supervisor: Took too long to compute controls"
                print "  Result: forfeit"
                self.score -= 5
                self.forfeit = True
            elif self.tcontroller > 1:
                print "Event supervisor: Took too long to compute controls"
                print "  Time",self.tcontroller,"over the last 1s"
                print "  Penalty: 1"
                self.score -= 1
            self.tcontroller = 0
            self.lastpenaltytime = t
        #check joint limits, velocity limits, and torque limits
        qrobot = sim.getActualConfig(0)
        vrobot = sim.getActualVelocity(0)
        trobot = sim.getActualTorques(0)
        qmin,qmax = sim.world.robot(0).getJointLimits()
        vmax = sim.world.robot(0).getVelocityLimits()
        tmax = sim.world.robot(0).getTorqueLimits()
        for i in range(7):
            if qrobot[i] < qmin[i] or qrobot[i] > qmax[i]:
                if not hasattr(stub,'verbose') or stub.verbose:
                    print "Event supervisor: Joint %d value %f out of joint limits [%f,%f]"%(i,qrobot[i],qmin[i],qmax[i])
                self.score -= dt*10
                break
        for i in range(1,7):
            if abs(vrobot[i]) > vmax[i]:
                if not hasattr(stub,'verbose') or stub.verbose:
                    print "Event supervisor: Joint %d value %f out of velocity limits [%f,%f]"%(i,vrobot[i],-vmax[i],vmax[i])
                    #print vrobot,vmax
                self.score -= dt*10
                break
        for i in range(6):
            if abs(trobot[i]) > tmax[i+1]:
                if not hasattr(stub,'verbose') or stub.verbose:
                    print "Event supervisor: Out of torque limits"
                    print trobot,tmax
                self.score -= dt*10
                break
        #check collisions between robot and terrain
        if self.inContact(sim):
            if not hasattr(stub,'verbose') or stub.verbose:
                print "Event supervisor: in contact with terrain"
            self.score -= dt*30
            
        #do ball kicking logic
        self.doGameLogic(sim)
        return
    
    def doGameLogic(self,sim):
        t = sim.getTime()
        goalcenter = (-1,0,0.5)
        goaldims = (1.5,1.5,0)
        goalmin = vectorops.madd(goalcenter,goaldims,-0.5)
        goalmax = vectorops.madd(goalcenter,goaldims,0.5)
        if self.ball < len(self.balltimes) and t > self.balltimes[self.ball]:
            obj = sim.world.rigidObject(self.ball)
            ballbody = sim.body(obj)
            Tb = ballbody.getTransform()
            ballrad = 0.1
            tgtx = random.uniform(goalmin[0],goalmax[0])
            tgty = random.uniform(goalmin[1],goalmax[1])
            target = (tgtx,
                      tgty,
                      goalcenter[2])
            vel = vectorops.sub(target,Tb[1])
            h = 1
            if self.difficulty == "easy":
                h=random.uniform(4,5)
            elif self.difficulty == "medium":
                h=random.uniform(3,6)
            elif self.difficulty == "hard":
                h=random.uniform(2,7)
            #h = g*t^2 - 1/2*t*2*g
            g = 9.8
            t = math.sqrt(2*h/g)
            vel = vectorops.mul(vel,0.5/t)
            vel[2] = t*g
            print "Event supervisor: Lobbing ball",self.ball,"target",target,"velocity",vel
            self.activeBalls[self.ball] = True
            ballbody.setTransform(Tb[0],vectorops.add(Tb[1],[0,0,0.05]))
            ballbody.setVelocity([0,0,0],vel)
            ballbody.enable(True)
            self.ball += 1
        
        #determine if any scored
        rob = sim.world.robot(0)
        ee = sim.body(rob.link(6))
        eeTrans = ee.getTransform()
        scoopmin = [-0.16,-0.16,0.12]
        scoopmax = [0.16,0.16,0.32]
        for i in range(self.ball):
            if self.activeBalls[i]:
                obj = sim.world.rigidObject(i)
                Tb = sim.body(obj).getTransform()
                Rb,tb = Tb
                Rloc,tloc = se3.mul(se3.inv(eeTrans),Tb)
                if all(tloc[i] > scoopmin[i] and tloc[i] < scoopmax[i] for i in range(3)):
                    print "Event supervisor: Ball",i,"scored, adding 10 points"
                    self.score += 10
                    sim.body(obj).setTransform(so3.identity(),[0,0,-10])
                    sim.body(obj).enable(False)
                    self.activeBalls[i] = False
                if tb[2] < 0.06:
                    #fallen
                    print "Event supervisor: Ball",i,"fell, disabling"
                    sim.body(obj).enable(False)
                    self.activeBalls[i] = False
        return
    
    def inContact(self,sim):
        """Returns true if the robot touches the environment"""
        robot = sim.world.robot(0)
        for i in range(robot.numLinks()):
            for j in range(sim.world.numTerrains()):
                if sim.hadContact(robot.link(i).getID(),sim.world.terrain(j).getID()):
                    return True
        return False

class GLTest:
    def __init__(self,simWorld,planningWorld):
        self.simWorld = simWorld
        self.planningWorld = planningWorld
        self.sim = Simulator(self.simWorld)
        self.event = EventB(self.sim)
        #set up sensors
        self.sensors = dict()
        cameraRot = [0,-1,0,0,0,-1,1,0,0]
        #on ground near robot, pointing up and slightly to the left
        Tsensor = (so3.mul(so3.rotation([1,0,0],-0.10),so3.mul(so3.rotation([0,-1,0],math.radians(90)),cameraRot)),[-1.5,-0.5,0.25])
        self.controller = stub.MyController(self.planningWorld,self.sim.controller(0))
        if stub.omniscient_sensor:
            self.sensors['omniscient'] = sensor.OmniscientObjectSensor()
        else:
            self.sensors['blobdetector'] = sensor.CameraColorDetectorSensor()
            self.sensors['blobdetector'].Tsensor = Tsensor        

        #set up camera to get a better vantage point
        #self.camera.dist = 12
        #self.camera.tgt[2] = -1
        #self.clippingplanes = (0.2,50)
        

        self.mode = 'automatic'
        self.quser = simWorld.robot(0).getConfig()
        
        self.dt = 0.02
        self.sim.simulate(0)
        self.simulate = True
        self.finalScore = None
        self.readings = dict()
        self.initVis()

    def initVis(self):
        kviz._init(self.simWorld)
        kviz.add_text("time",5,5)
        kviz.add_text("score",5,10)
        kviz.add_text("final",5,15)
        if 'blobdetector' in self.sensors:
            Tsensor = self.sensors['blobdetector'].Tsensor
            x,y,z = Tsensor[1]
            kviz.add_sphere("cam_center",x,y,z,0.03)
            kviz.set_color("cam_center",(1,1,0,1))
            kviz.add_polyline("cam_fwd",[Tsensor[1],se3.apply(Tsensor,[0,0,0.2])])
            kviz.set_color("cam_fwd",(0,0,1,1))
            kviz.add_polyline("cam_up",[Tsensor[1],se3.apply(Tsensor,[0,0.1,0])])
            kviz.set_color("cam_up",(0,1,0,1))
        self.ghost = kviz.add_ghost()
        kviz.set_color(self.ghost,(1,1,0,0.5))
        self.numBlobs = 0
        self.updateVis()

    def updateVis(self):
        kviz.update_text("time","Time: "+str(self.sim.getTime()))
        kviz.update_text("score","Score: "+str(self.event.score))
        if self.finalScore != None:
            kviz.update_text("final","Final score: "+str(self.finalScore))
        if self.mode == 'user':
            kviz.set_visible(self.ghost,True)
            kviz.set_ghost_config(self.quser)
        else:
            kviz.set_visible(self.ghost,False)
        if 'blobdetector' in self.sensors:
            sensor = self.sensors['blobdetector']
            Tsensor = sensor.Tsensor
            for n,r in self.readings.iteritems():
                assert isinstance(r,CameraColorDetectorOutput)
                for i,blob in enumerate(r.blobs):
                    xmin = blob.x-blob.w*0.5
                    xmax = blob.x+blob.w*0.5
                    ymin = blob.y-blob.h*0.5
                    ymax = blob.y+blob.h*0.5
                    umin = (xmin - sensor.w/2)/math.tan(math.radians(sensor.fov*0.5))/(sensor.w/2)
                    umax = (xmax - sensor.w/2)/math.tan(math.radians(sensor.fov*0.5))/(sensor.w/2)
                    vmin = (ymin - sensor.h/2)/math.tan(math.radians(sensor.fov*0.5))/(sensor.w/2)
                    vmax = (ymax - sensor.h/2)/math.tan(math.radians(sensor.fov*0.5))/(sensor.w/2)
                    depth = 0.2
                    a = se3.apply(Tsensor,(umin*depth,vmin*depth,depth))
                    b = se3.apply(Tsensor,(umax*depth,vmin*depth,depth))
                    c = se3.apply(Tsensor,(umax*depth,vmax*depth,depth))
                    d = se3.apply(Tsensor,(umin*depth,vmax*depth,depth))
                    kviz.update_quad("blob"+str(i),a,d,c,b)
                    kviz.update_quad("blob_back"+str(i),a,b,c,d)
                    #kviz.update_quad("blob"+str(i),(0,0,2),b=(1,0,2),c=(1,1,2),d=(0,1,2))
                    kviz.set_color("blob"+str(i),blob.color+(1,))
                    kviz.set_color("blob_back"+str(i),blob.color+(1,))
                for i in xrange(len(r.blobs),self.numBlobs):
                    print "Removing blob",i
                    kviz.remove("blob"+str(i))
                    kviz.remove("blob_back"+str(i))
                self.numBlobs = len(r.blobs)
            if len(self.readings) == 0:
                for i in xrange(self.numBlobs):
                    print "Removing blob",i
                    kviz.remove("blob"+str(i))
                    kviz.remove("blob_back"+str(i))
                self.numBlobs = 0

    def control_loop(self):
        self.readings = dict()
        for n,s in self.sensors.iteritems():
            self.readings[n] = s.emulate(self.sim)
        if self.mode == 'user':
            self.sim.controller(0).setMilestone(self.quser)
        else:
            try:
                self.controller.loop(self.dt,self.sim.controller(0),self.readings)
            except Exception as e:
                print "Exception called during controller.loop:"
                traceback.print_exc()

    def step(self):
        t0 = time.time()
        self.control_loop()
        tcontroller = time.time()-t0
        self.event.mark_controller_time(tcontroller)
        
        self.sim.simulate(self.dt)
        self.event.update(self.dt,self.sim)
        if self.finalScore == None and self.sim.getTime() >= self.event.endtime:
            self.finalScore = self.event.score
        self.updateVis()

program = None

def boilerplate_start():
    global program,world,world2
    world = WorldModel()
    world2 = WorldModel()
    fn = "Web/Client/Scenarios/final/finalB.xml"
    res = world.readFile(fn)
    if not res:
        raise RuntimeError("Unable to load world "+fn)
    res = world2.readFile(fn)
    for i in range(world.numRigidObjects()):
        world.rigidObject(i).appearance().setColor(*sensor.objectColors[i%len(sensor.objectColors)])
        world2.rigidObject(i).appearance().setColor(*sensor.objectColors[i%len(sensor.objectColors)])
    program = GLTest(world,world2)
    random.seed(stub.random_seed)

def boilerplate_advance():
    global program
    program.step()
    program.sim.updateWorld()
    
def boilerplate_event(name):
    global program
    if name=="print":
        print program.quser

def boilerplate_setitem(name,value):
    global program
    if name=="mode":
        program.mode = value
        program.updateVis()
    elif name=="q1":
        program.quser[1] = math.radians(float(value))
        program.updateVis()
    elif name=="q2":
        program.quser[2] = math.radians(float(value))
        program.updateVis()
    elif name=="q3":
        program.quser[3] = math.radians(float(value))
        program.updateVis()
    elif name=="q4":
        program.quser[4] = math.radians(float(value))
        program.updateVis()
    elif name=="q5":
        program.quser[5] = math.radians(float(value))
        program.updateVis()
    elif name=="q6":
        program.quser[6] = math.radians(float(value))
        program.updateVis()
