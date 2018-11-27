from cspace import CSpace
from .. import robotsim
from ..model import collide
from ..model import trajectory
from ..math import vectorops,so3,se3
import math
import random

class RigidObjectCSpace(CSpace):
    """A basic free-flying rigid object cspace that allows collision free motion.

    The C-space is 6D, with parameters [tx,ty,tz,mx,my,mz].  [tx,ty,tz] is the translation
    and [mx,my,mz] are the exponential map parameters of the rotation.

    Attributes:
    - rigidObject
    - rotationDomain
    - collider
    - rotationWeight: the relative weight used for measuring rotation vs translation distance
    """
    def __init__(self,rigidObject,collider=None,translationDomain=None,rotationDomain=None):
        """Arguments:
        - rigidObject: the RigidObjectModel that should move.
        - collider (optional): a collide.WorldCollider instance containing
          the world in which the robot lives.  Any ignored collisions will be
          respected in the collision checker.
        - translationDomain: None, or a bounding box in which the translation should be sampled. 
          If None (default), the Jeffrey's prior is used to sample translations.
        - rotationDomain: None, or a (rotation0,rdomain) pair specifying a range in which the
          rotation should be sampled. If rdomain is a number, the rotation is sampled with absolute
          angular error from rotation0 in the range [0,rdomain].  If rdomain is a triple, then the
          rotation is sampled with euler angles with roll in the range [-rdomain[0],rdomain[0]], pitch
          in the range [-rdomain[1],rdomain[1]], and yaw in the range [-rdomain[2],rdomain[2]].  The 
          sampled rotation is then multiplied by rotation0.
        """
        CSpace.__init__(self)
        self.rigidObject = rigidObject
        if translationDomain is None:
            translationDomain = [(-float('inf'),float('inf'))]*3
        self.bound = translationDomain + [(-math.pi,math.pi)]*3
        self.rotationDomain = rotationDomain
        self.collider = collider
        self.rotationWeight = 1.0/math.pi

        if collider:
            def robCollide(r):
                return any(True for _ in self.collider.robotObjectCollisions(r,self.rigidObject.index))
            def objCollide(o):
                return any(True for _ in self.collider.objectObjectCollisions(self.rigidObject.index,o))
            def terrCollide(o):
                return any(True for _ in self.collider.objectTerrainCollisions(self.rigidObject.index,o))
            self.addFeasibilityTest(self.setConfig,"setconfig")
            self.addFeasibilityTest((lambda x: not self.selfCollision()),"self collision",dependencies="setconfig")
            #self.addFeasibilityTest((lambda x: not self.envCollision()),"env collision")
            for o in range(self.collider.world.numRobots()):
                self.addFeasibilityTest((lambda x,o=o: not robCollide(o)),"robot collision "+str(o)+" "+self.collider.world.robot(o).getName(),dependencies="setconfig")
            for o in range(self.collider.world.numRigidObjects()):
                if o != self.rigidObject:
                    self.addFeasibilityTest((lambda x,o=o: not objCollide(o)),"obj collision "+str(o)+" "+self.collider.world.rigidObject(o).getName(),dependencies="setconfig")
            for o in range(self.collider.world.numTerrains()):
                self.addFeasibilityTest((lambda x,o=o: not terrCollide(o)),"terrain collision "+str(o)+" "+self.collider.world.terrain(o).getName(),dependencies="setconfig")
        else:
            self.addFeasibilityTest(self.setConfig,"setconfig")

        self.properties['geodesic'] = 1

    def configToTransform(self,x):
        return (so3.from_moment(x[3:6]),x[:3])

    def transformToConfig(self,T):
        return T[1] + so3.moment(T[0])

    def setConfig(self,x):
        self.rigidObject.setTransform(*self.configToTransform(x))
        return True

    def addConstraint(self,checker,name=None):
        self.addFeasiblilityTest(checker,name)

    def sample(self):
        """Overload this to implement custom sampling strategies."""
        t = [0.0]*3
        for i,bnd in enumerate(self.bound[:3]):
            if math.isnan(bnd[0]):
                t[i] = math.log(random.random())
                if random.random() < 0.5:
                    t[i] *= -1.0
            else:
                t[i] = random.uniform(bnd[0],bnd[1])
        if self.rotationDomain is None:
            R = so3.sample()
        else:
            R0,arange = self.rotationDomain
            if hasattr(arange,'__iter__'):
                #euler angles
                r,p,y = random.uniform(arange[0]),random.uniform(arange[1]),random.uniform(arange[2])
                Rrand = so3.from_rpy((r,p,y))
            else:
                axis = random.gauss(0,1),random.gauss(0,1),random.gauss(0,1)
                axis = vectorops.uniform(axis)
                angle = random.uniform(0,arange)
                Rrand = so3.rotation(axis,angle)
            R = so3.mul(R0,Rrand)
        return self.transformToConfig((R,t))

    def envCollision(self,x=None):
        """Checks whether the robot at its current configuration is in
        collision with the environment."""
        if not self.collider: return False
        if x is not None: self.setConfig(x)
        for o in xrange(self.collider.world.numRobots()):
            if any(self.collider.robotObjectCollisions(o.index,self.rigidObject.index)):
                return True;
        for o in xrange(self.collider.world.numRigidObjects()):
            if any(self.collider.objectObjectCollisions(self.rigidObject.index,o)):
                return True;
        for o in xrange(self.collider.world.numTerrains()):
            if any(self.collider.objectTerrainCollisions(self.rigidObject.index,o)):
                return True;
        return False

    def interpolate(self,a,b,u):
        return self.transformToConfig(se3.interpolate(self.configToTransform(a),self.configToTransform(b),u))

    def distance(self,a,b):
        td = vectorops.distanceSquared(a[:3],b[:3])
        Ra = so3.from_moment(a[3:6])
        Rb = so3.from_moment(b[3:6])
        return math.sqrt(td + (self.rotationWeight*so3.distance(Ra,Rb))**2)

    def pathToTransforms(self,path):
        return [self.configToTransform(x) for x in path]

    def pathToTrajectory(self,path):
        times = range(len(path))
        Ts = [self.configToTransform(x) for x in path]
        return trajectory.SE3Trajectory(times,Ts)
