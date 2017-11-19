import sys
sys.path.append("Web/Server")
sys.path.append('Web/Client/Scenarios/lab_state_estimation')
from klampt import *
from klampt.math import vectorops,so3,se3
from common import *
import time
import random
import traceback
import sensor
import kviz
import math

class Supervisor:
    def __init__(self,simWorld):
        self.simWorld = simWorld
        self.sim = Simulator(self.simWorld)
        #set up sensors
        self.sensors = dict()
        cameraRot = [0,-1,0,0,0,-1,1,0,0]
        #at goal post, pointing a bit up and to the left
        Tsensor = (so3.mul(so3.rotation([0,0,1],0.20),so3.mul(so3.rotation([0,-1,0],0.25),cameraRot)),[-2.55,-1.1,0.25])
        self.sensors['omniscient'] = sensor.OmniscientObjectSensor()
        self.sensors['position'] = sensor.ObjectPositionSensor()
        self.sensors['blobdetector'] = sensor.CameraColorDetectorSensor()
        self.sensors['blobdetector'].Tsensor = Tsensor
        self.sensor = 'omniscient'

        #set up camera to get a better vantage point
        #self.camera.dist = 12
        #self.camera.tgt[2] = -1

        #times for ball launching
        self.balltimes = [0.5+i*2 for i in range(14)]
        self.currentBall = 0

        self.dt = 0.02
        self.sim.simulate(0)
        self.readings = dict()
        self.initVis()

    def initVis(self):
        kviz._init(self.simWorld)
        if 'blobdetector' in self.sensors:
            Tsensor = self.sensors['blobdetector'].Tsensor
            x,y,z = Tsensor[1]
            kviz.add_sphere("cam_center",x,y,z,0.03)
            kviz.set_color("cam_center",1,1,0)
            kviz.add_polyline("cam_fwd",[Tsensor[1],se3.apply(Tsensor,[0,0,0.2])])
            kviz.set_color("cam_fwd",0,0,1)
            kviz.add_polyline("cam_up",[Tsensor[1],se3.apply(Tsensor,[0,0.1,0])])
            kviz.set_color("cam_up",0,1,0)
        self.numBlobs = 0
        self.updateVis()

    def updateVis(self):
        if 'blobdetector' in self.sensors:
            sensor = self.sensors['blobdetector']
            Tsensor = sensor.Tsensor
            for n,r in self.readings.iteritems():
                if n != 'blobdetector':
                    continue
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
                    kviz.set_color("blob"+str(i),*blob.color)
                    kviz.set_color("blob_back"+str(i),*blob.color)
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

    def kickBall(self,ball):
        goalcenter = (-2.7,0,0.5)
        goaldims = (0.2,2.2,1.0)
        goalmin = vectorops.madd(goalcenter,goaldims,-0.5)
        goalmax = vectorops.madd(goalcenter,goaldims,0.5)
        sim = self.sim
        obj = sim.world.rigidObject(ball)
        ballbody = sim.body(obj)
        Tb = ballbody.getTransform()
        ballrad = 0.1
        cmin= vectorops.add(goalmin,[ballrad]*3)
        cmax= vectorops.add(goalmax,[-ballrad]*3)
        tgty = random.uniform(cmin[1],cmax[1])
        #don't shoot in middle
        while abs(tgty) < 0.2:
            tgty = random.uniform(cmin[1],cmax[1])
        tgtz = random.uniform(cmin[2],cmax[2])
        target = (goalcenter[0],
                  tgty,
                  tgtz)
        vel = vectorops.sub(target,Tb[1])
        t = random.uniform(0.8,1.2)
        vel = vectorops.mul(vel,t)
        vel[2] = random.uniform(3.0/t,5.4/t)
        ballbody.setVelocity([0,0,0],vel)
        ballbody.enable(True)

    def step(self):
        self.sim.simulate(self.dt)
        if self.currentBall < len(self.balltimes) and self.sim.getTime() >= self.balltimes[self.currentBall]:
            self.kickBall(self.currentBall)
            self.currentBall += 1
        self.readings = dict()
        for n,s in self.sensors.iteritems():
            self.readings[n] = s.emulate(self.sim)
        stub.update(self.sensor,self.readings[self.sensor])
        self.updateVis()
        

program = None

def boilerplate_start():
    global program
    world = WorldModel()
    fn = "Web/Client/Scenarios/lab_state_estimation/world.xml"
    #fn = "Web/Client/Scenarios/final/finalA.xml"
    res = world.readFile(fn)
    if not res:
        raise RuntimeError("Unable to load world "+fn)
    for i in range(world.numRigidObjects()):
        world.rigidObject(i).appearance().setColor(*sensor.objectColors[i%len(sensor.objectColors)])
    program = Supervisor(world)
    random.seed(stub.random_seed)
    print "Done initializing"

def boilerplate_advance():
    global program
    program.step()
    program.sim.updateWorld()

def boilerplate_event(name):
    global program
    pass

def boilerplate_setitem(name,value):
    global program
    if name == 'sensor':
        program.sensor = value
    pass
