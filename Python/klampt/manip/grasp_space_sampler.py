from .grasp_space import GraspParameterSpace
from .grasp_sampler import GraspSamplerBase
from klampt.model.gripperinfo import GripperInfo
from klampt.model.typing import Config
from .grasp import Grasp
import random
from typing import Optional,Union,List,Callable

class GraspSpaceSampler(GraspSamplerBase):
    """A grasp sampler that samples grasps uniformly from a GraspParameterSpace
    and filters and ranks them using a user-provided function.

    If num_initial_samples > 0, then that many samples will be drawn from the
    space, scored, and filtered by the threshold function.  The remaining
    samples will be sorted by score, and returned in that order.  If
    num_initial_samples=0, then samples will be drawn from the space on-the-fly
    and scored, and no filtering will be done.
    """
    def __init__(self,
                 space : GraspParameterSpace,
                 score_function : Callable[[Grasp], float],
                 threshold : Union[Callable[[Grasp], bool],float] = 0.0,
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
        g.score = self.score_function(g)
        return g
