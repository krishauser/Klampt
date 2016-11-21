import numpy as np

#Raw sensing output

class OmniscientObjectOutput:
    """Stores the output of an OmniscientObjectSensor
    Attributes:
    names: a list of object names (actually, these will be the colors
      of the objects)
    positions: a list of object positions
    velocities: a list of object positions
    """
    def __init__(self,names,positions,velocities):
        self.names = names[:]
        self.positions = positions[:]
        self.velocities = velocities[:]

class CameraBlob:
    """A blob on the camera screen, in image coordinates.
    Attributes:
    - color: an (r,g,b) tuple indicating the blob's color.
    - x,y: the coordinates of the blob center
    - w,h: the width and height of the blob
    """
    def __init__(self,color,x,y,w,h):
        self.color = color
        self.x = x
        self.y = y
        self.w = w
        self.h = h

class CameraColorDetectorOutput:
    """Stores the output of a CameraColorDetectorSensor
    Attributes:
    blobs: a list of CameraBlobs specifying the regions of the detected
      blobs.
    """
    def __init__(self,blobs):
        self.blobs = blobs[:]

#State estimation code 

class ObjectStateEstimate:
    """Attributes:
    - name: an identifier of the object
    - x: mean state (position / velocity) estimate, a 6-D vector
    - cov: state (position / velocity) covariance, a 6x6 numpy array
    """
    def __init__(self,name,x,cov=0):
        self.name = name
        self.x = x
        self.cov = cov
        if isinstance(cov,(int,float)):
            self.cov = np.eye(6)*cov
    def meanPosition(self):
        return self.x[0:3]
    def meanVelocity(self):
        return self.x[3:6]

class MultiObjectStateEstimate:
    """A list of ObjectStateEstimates.
    
    Attributes:
        - objects: a list of ObjectStateEstimates, corresponding to all
          of the objects currently tracked by the state estimator.
    """
    def __init__(self,objectEstimates):
        self.objects = objectEstimates[:]
        
    def get(self,name):
        """Retrieves an object's state estimate by name"""
        for o in self.objects:
            if o.name==name:
                return o
        return None

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
        assert isinstance(o,OmniscientObjectOutput),"OmniscientStateEstimator only works with an omniscient sensor"
        estimates = [ObjectStateEstimate(n,p+v) for n,p,v in zip(o.names,o.positions,o.velocities)]
        return MultiObjectStateEstimate(estimates)

#Kalman filtering code

def gaussian_linear_transform(mean,cov,A,b):
    """Given a prior x~N(mean,cov), returns the
    mean and covariance of the variate y=A*x+b.
    """
    ymean = np.dot(A,mean)+b
    ycov = np.dot(A,np.dot(cov,A.T))
    return (ymean,ycov)

def kalman_filter_predict(prior_mean,prior_cov,F,g,SigmaX):
    """For the Kalman filter model with transition model:
      x[t] = F*x[t-1] + g + eps_x
    with eps_x ~ N(0,SigmaX) 
    and given prior estimate x[t-1]~N(prior_mean,prior_cov),
    computes the predicted mean and covariance matrix at x[t]

    Output:
    - A pair (mu,cov) with mu the updated mean and cov the updated covariance
      matrix.

    Note: all elements are numpy arrays.
    """
    if isinstance(SigmaX,(int,float)):
        SigmaX = np.eye(len(prior_mean))*SigmaX
    muprime = np.dot(F,prior_mean)+g
    covprime = np.dot(F,np.dot(prior_cov,F.T))+SigmaX
    return (muprime,covprime)

def kalman_filter_update(prior_mean,prior_cov,F,g,SigmaX,H,j,SigmaZ,z):
    """For the Kalman filter model with transition model:
      x[t] = F*x[t-1] + g + eps_x
    and observation model
      z[t] = H*x[t] + j + eps_z
    with eps_x ~ N(0,SigmaX) and eps_z ~ N(0,SigmaZ),
    and given prior estimate x[t-1]~N(prior_mean,prior_cov),
    computes the updated mean and covariance matrix after observing z[t]=z.

    Output:
    - A pair (mu,cov) with mu the updated mean and cov the updated covariance
      matrix.

    Note: all elements are numpy arrays.

    Note: can be applied as an approximate extended Kalman filter by setting
    F*x+g and H*x+j to be the linearized models about the current estimate
    prior_mean.  (The true EKF would propagate the mean update and linearize
    the observation term around the mean update)
    """
    if isinstance(SigmaX,(int,float)):
        SigmaX = np.eye(len(prior_mean))*SigmaX
    if isinstance(SigmaZ,(int,float)):
        SigmaZ = np.eye(len(z))*SigmaZ
    muprime = np.dot(F,prior_mean)+g
    covprime = np.dot(F,np.dot(prior_cov,F.T))+SigmaX
    C = np.dot(H,np.dot(covprime,H.T))+SigmaZ
    zpred = np.dot(H,muprime)+j
    K = np.dot(covprime,np.dot(H.T,np.linalg.pinv(C)))
    mu = muprime + np.dot(K,z-zpred)
    cov = np.dot(covprime,np.eye(covprime.shape[0])-np.dot(K,H))
    return (mu,cov)
