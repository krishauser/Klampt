from klampt import *
from klampt.math import vectorops,so3,se3
from common import *
import random
#this may be useful...
#import numpy as np

##################### SETTINGS ########################

random_seed = 12345
#random_seed = random.seed()

verbose = True

################ STATE ESTIMATION #####################

class OmniscientStateEstimator:
    """A hack state estimator that gives perfect state information from
    OmniscientObjectOutput readings."""
    def __init__(self):
        self.reset()
        return
    def reset(self):
        pass
    def update(self,o):
        """Produces an updated MultiObjectStateEstimate given an OmniscientObjectOutput
        sensor reading."""
        assert isinstance(o,OmniscientObjectOutput),"OmniscientStateEstimator only works with an omniscientObjectOutput object"
        estimates = [ObjectStateEstimate(n,p+v) for n,p,v in zip(o.names,o.positions,o.velocities)]
        return MultiObjectStateEstimate(estimates)

class PositionStateEstimator:
    def __init__(self):
        self.dt = 0.02
        self.reset()
        return
    def reset(self):
        pass
    def update(self,o):
        """Produces an updated MultiObjectStateEstimate given an ObjectPositionOutput
        sensor reading."""
        assert isinstance(o,ObjectPositionOutput),"PositionStateEstimator only works with an ObjectPositoinOutput object"
        #TODO: Fill me in for Problem 1
        estimates = []
        for n,p in zip(o.names,o.positions):
            #currently this just takes the sensed position and assumes 0 velocity
            estimates.append(ObjectStateEstimate(n,p+[0,0,0]))
        return MultiObjectStateEstimate(estimates)

class BlobStateEstimator:
    def __init__(self):
        self.Tsensor = None
        cameraRot = [0,-1,0,0,0,-1,1,0,0]
        #at goal post, pointing a bit up and to the left
        self.Tsensor = (so3.mul(so3.rotation([0,0,1],0.20),so3.mul(so3.rotation([0,-1,0],0.25),cameraRot)),[-2.55,-1.1,0.25])
        self.fov = 90
        self.w,self.h = 320,240
        self.dmax = 5
        self.dt = 0.02
        self.reset()
        return
    def reset(self):
        pass
    def update(self,o):
        """Produces an updated MultiObjectStateEstimate given an CameraColorDetectorOutput
        sensor reading."""
        assert isinstance(o,CameraColorDetectorOutput),"BlobStateEstimator only works with an CameraColorDetectorOutput object"
        #TODO: Fill me in for Problem 2
        return MultiObjectStateEstimate([])


positionStateEstimator = PositionStateEstimator()
blobStateEstimator = BlobStateEstimator()

def reset():
    initVis()
    positionStateEstimator.reset()
    blobStateEstimator.reset()

def update(sensorType,observation):
    """Produces an updated MultiObjectStateEstimate given sensor input.

    Input:
    - sensorType: "omniscient", "position", or "blobdetector".
    - observation: one of the types of sensor input.

    Return value: MultiObjectStateEstimate giving the estimated state of all
    observed objects.
    """
    if sensorType == 'omniscient':
        omniscientObjectState = OmniscientStateEstimator().update(observation)
        updateVis(omniscientObjectState)
        return omniscientObjectState
    elif sensorType == 'position':
        positionObjectState = positionStateEstimator.update(observation)
        updateVis(positionObjectState)
        return positionObjectState            
    elif sensorType == 'blobdetector':
        blobObjectState = blobStateEstimator.update(observation)
        updateVis(blobObjectState)
        return blobObjectState
            
    return MultiObjectStateEstimate([])
        

def initVis():
    """If you want to do some visualization, initialize it here.
        TODO: You may consider visually debugging some of your code here, along with updateVis().
    """
    pass
    
def updateVis(objectEstimates):
    """This gets called by update()
    TODO: You may consider visually debugging some of your code here, along with initVis().

    The current code draws gravity-inflenced arcs leading from all the
    object position / velocity estimates from your state estimator. 
    """
    gravity = 9.8
    if objectEstimates:
        for o in objectEstimates.objects:
            #draw a point
            kviz.update_sphere("object_est"+str(o.name),o.x[0],o.x[1],o.x[2],0.03)
            kviz.set_color("object_est"+str(o.name),o.name[0],o.name[1],o.name[2])
            #draw an arc
            trace = []
            x = [o.x[0],o.x[1],o.x[2]]
            v = [o.x[3],o.x[4],o.x[5]]
            for i in range(20):
                t = i*0.05
                trace.append(vectorops.sub(vectorops.madd(x,v,t),[0,0,0.5*gravity*t*t]))
                if trace[-1][2] < 0: break
            kviz.remove("object_trace"+str(o.name))
            kviz.add_polyline("object_trace"+str(o.name),trace);
            kviz.set_color("object_trace"+str(o.name),o.name[0],o.name[1],o.name[2])

