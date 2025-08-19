from .grasp_space import GraspParameterSpace
from .grasp_sampler import GraspSamplerBase
from ..model.gripperinfo import GripperInfo
from ..model.typing import Config
from ..math import vectorops,so3,se3
from .. import Geometry3D, RigidObjectModel
from .grasp import Grasp
import random
from typing import Optional,Union,List,Callable

class GraspSpaceSampler(GraspSamplerBase):
    """A grasp sampler that samples grasps uniformly from a GraspParameterSpace
    and filters and ranks them using a user-provided function.

    The scoring function should return a value in [0,1] for each grasp, where
    0 is the worst and 1 is the best. 

    If num_initial_samples > 0, then that many samples will be drawn from the
    space, scored, and filtered by the threshold function.  The remaining
    samples will be sorted by score, and returned in that order.  If
    num_initial_samples=0, then samples will be drawn from the space on-the-fly
    and scored, and no filtering will be done.

    If distance_to_object is given, then the sampled grasps will be sampled
    with the contact point a range of at most that distance to the object.
    """
    def __init__(self,
                 space : GraspParameterSpace,
                 score_function : Callable[[Grasp], float],
                 threshold : Union[Callable[[Grasp], bool],float] = 0.0,
                 distance_to_object : Optional[float] = None,
                 num_initial_samples : int = 100,
                 num_retries : int = 0,
                 object_relative : bool = False):
        self.space = space
        self.featureBounds = space.featureBounds()
        self.score_function = score_function
        if not callable(threshold):
            self.threshold = lambda s: s.score >= threshold
        else:
            self.threshold = threshold
        self.distance_to_object = distance_to_object
        self.num_initial_samples = num_initial_samples
        self.num_retries = num_retries
        self.samples = []  # type: List[Grasp]
        self.index = 0
        self.retry_count = 0
        self.object_relative = object_relative
        self.transform = None

    def gripper(self) -> GripperInfo:
        return self.space.gripper
    
    def init(self, scene, object, hints=None) -> bool:
        if self.object_relative:
            self.transform = object.getTransform()
        else:
            self.transform = None
        if self.num_initial_samples == 0:
            return True #generate on the fly, no filtering
        if self.distance_to_object is not None:
            from ..model.geometry import sample_surface
            g = object if isinstance(object,Geometry3D) else object.geometry()
            object_points = sample_surface(g, self.num_initial_samples, local=False)
            #fuzz by distance_to_object
            if self.distance_to_object > 0:
                for i,p in enumerate(object_points):
                    e = (random.gauss(0,1),random.gauss(0,1),random.gauss(0,1))
                    object_points[i] = vectorops.madd(object_points[i],vectorops.unit(e),self.distance_to_object*random.uniform(0,1)**(1.0/3.0))
            self.object_points = object_points
        self.samples = []
        for _ in range(self.num_initial_samples):
            g = self.sampleGraspSpace()
            if self.threshold(g):
                self.samples.append(g)
        self.samples.sort(key=lambda g: g.score, reverse=True)
        self.index = 0
        self.retry_count = 0
        return True
    
    def next(self) -> Optional[Grasp]:
        if self.num_initial_samples == 0:
            #generate on the fly
            g = self.sampleGraspSpace()
            return g
        else:
            if self.index >= len(self.samples):
                if self.retry_count < self.num_retries:  #re-sample a batch of grasps
                    self.retry_count += 1
                    self.init(None,None)
                    return self.next()
                else:
                    return None
            g = self.samples[self.index]
            self.index += 1
            return g

    def score(self) -> float:
        """Returns the score of the best remaining grasp."""
        return self.score_function(self.samples[self.index]) if self.index < len(self.samples) else 0.0

    def visDebug(self):
        for i in range(self.index, len(self.samples)):
            g = self.samples[i]
            g.addToVis("grasp_%d" % i)

    def sampleGraspSpace(self) -> Grasp:
        """Returns a scored grasp, sampled uniformly at random from the grasp space."""
        f = [random.uniform(b[0], b[1]) for b in zip(*self.featureBounds)]
        g = self.space.toGrasp(f)

        if self.transform is not None:
            g = g.getTransformed(self.transform)

        if self.distance_to_object is not None:
            #move the contact centroid to a random point sampled near the object surface
            old_center = se3.apply(g.asFixedGrasp().ikConstraint.getTransform(),self.space.gripper.estimatedContactCentroid())
            g = g.getTransformed((so3.identity(),vectorops.sub(random.choice(self.object_points),old_center)))

        g.score = self.score_function(g)
        return g
