from klampt import vectorops,so3,se3
import math
import random
from common import CameraBlob,CameraColorDetectorOutput,OmniscientObjectOutput

objectColors = [(1,0,0,1),
                (1,0.5,0,1),
                (1,1,0,1),
                (0.5,1,0,1),
                (0,1,0,1),
                (0,1,0.5,1),
                (0,1,1,1),
                (0,0.5,1,1),
                (0,0,1,1),
                (0.5,0,1,1),
                (1,0,1,1),
                (0,0,0.5,1),
                (0.5,0,0.5,1),
                (0.5,0,0,1)]

class CameraColorDetectorSensor:
    """Attributes:
    - Tsensor: camera transform (z forward, x right, y down, origin focal point)
    - fov: horizontal field of view
    - w,h: width and height of frame
    - dmin,dmax: minimum / maximum depth
    - pixelError: standard deviation of pixel-wise errors
    """
    def __init__(self):
        self.Tsensor = se3.identity()
        self.fov = 90
        self.w,self.h = 320,240
        self.dmin,self.dmax = 0.01,5
        self.pixelError = 0.5
    def emulate(self,sim):
        """Given a Simulator instance, emulates the sensor.
        Result is a CameraColorDetectorOutput structure."""
        global objectColors
        xscale = math.tan(math.radians(self.fov*0.5))*self.w/2
        blobs = []
        for i in range(sim.world.numRigidObjects()):
            o = sim.world.rigidObject(i)
            body = sim.body(o)
            Tb = body.getTransform()
            plocal = se3.apply(se3.inv(self.Tsensor),Tb[1])
            x,y,z = plocal
            if z < self.dmin or z > self.dmax:
                continue
            c = objectColors[i%len(objectColors)]
            o.geometry().setCurrentTransform(so3.mul(so3.inv(self.Tsensor[0]),Tb[0]),[0]*3)
            bmin,bmax = o.geometry().getBB()
            err = self.pixelError
            dscale = xscale/z
            xim = dscale * x + self.w/2 
            yim = dscale * y + self.h/2
            xmin = int(xim - dscale*(bmax[0]-bmin[0])*0.5 + random.uniform(-err,err))
            ymin = int(yim - dscale*(bmax[1]-bmin[1])*0.5 + random.uniform(-err,err))
            xmax = int(xim + dscale*(bmax[0]-bmin[0])*0.5 + random.uniform(-err,err))
            ymax = int(yim + dscale*(bmax[1]-bmin[1])*0.5 + random.uniform(-err,err))
            if xmin < 0: xmin=0
            if ymin < 0: ymin=0
            if xmax >= self.w: xmax=self.w-1
            if ymax >= self.h: ymax=self.h-1
            if ymax <= ymin: continue
            if xmax <= xmin: continue
            #print "Actual x,y,z",x,y,z
            #print "blob color",c[0:3],"dimensions",xmin,xmax,"x",ymin,ymax
            blob = CameraBlob(c[0:3],0.5*(xmin+xmax),0.5*(ymin+ymax),xmax-xmin,ymax-ymin)
            blobs.append(blob)
        return CameraColorDetectorOutput(blobs)


class OmniscientObjectSensor:
    def __init__(self):
        pass
    def emulate(self,sim):
        """Given a Simulator instance, emulates the sensor.
        Result is a OmniscientObjectOutput structure."""
        global objectColors
        names = []
        positions = []
        velocities = []
        for i in range(sim.world.numRigidObjects()):
            o = sim.body(sim.world.rigidObject(i))
            name = objectColors[i%(len(objectColors))]
            names.append(name)
            positions.append(o.getTransform()[1])
            velocities.append(o.getVelocity()[1])
        return OmniscientObjectOutput(names,positions,velocities)


