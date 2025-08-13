from __future__ import annotations
from ..model.gripperinfo import GripperInfo
from ..model.contact import ContactPoint
from ..model import ik
from .. import IKObjective, RobotModel, WorldModel, RigidObjectModel
from .grasp import Grasp
from ..model.typing import Config
from typing import List, Optional, Union, Generator



class GraspSamplerBase:
    """An abstract base class that will sample grasps for a given scene.
    """
    def gripper(self) -> GripperInfo:
        """Returns a GripperInfo describing which gripper this works for."""
        raise NotImplementedError()

    def init(self, scene, object, hints=None) -> bool:
        """Does some initialization of the grasp generator.  Return True if
        this generator can be applied.

        Args:
            scene: the scene to sample grasps for, which may contain the
                robot, the object to grasp, and any other objects to avoid.
                Typically, a WorldModel object.
            object: the object to be grasped.  Typically a RigidObjectModel
                or geometry. The interpretation may depend on the type of
                grasp sampler (see below)
            hints (any): additional hints or context for the grasp generation.
                The interpretation of this argument depends on the type of
                grasp sampler.
            
        For object-centric grasp samplers, object should be a RigidObjectModel
        containing the object model in its estimated pose.

        For image-based grasp samplers, object is typically a RigidObjectModel
        containing the point cloud of the scene / object to be grasped. 
        hints can include a point in image space, a point in 3D space,
        a ray, or even a description of the object.
        """
        return True

    def next(self) -> Optional[Union[Grasp,GraspWithConfig]]:
        """Returns the next candidate Grasp or GraspWithConfig for the scene
        and object provided in init().  Can return None if the grasp generator
        fails."""
        raise NotImplementedError()
    
    def generator(self, scene, object, hints=None) -> Generator[Optional[Union[Grasp,GraspWithConfig]], None, None]:
        """A generator that yields grasps for the scene and object provided in
        init().  This is a convenience method that calls init() and then
        repeatedly calls next() until it returns None."""
        if not self.init(scene, object, hints):
            raise RuntimeError("GraspSamplerBase: init() failed, cannot generate grasps")
        while True:
            g = self.next()
            if g is None:
                return
            yield g

    def score(self) -> float:
        """Return a heuristic score in the range [0,1] that estimates the
        likelihood of a grasp being generated for the scene and object
        provided in init()."""
        raise NotImplementedError()

    def visDebug(self):
        """Does some visual debugging of the sampler, if possible."""
        return


class GraspSamplerWithIK(GraspSamplerBase):
    """A grasp sampler that uses another grasp sampler to generate grasps,
    but will check for feasibility of the IK solutions defined by those grasps.

    A random restart approach will be used to find a feasible IK solution.
    The number of restarts is set by the `restartsPerGrasp` parameter.  If the 
    seedConfig is provided, then the robot will be set to that configuration
    first before any restarts are attempted.

    Collisions can be tested in the scene if checkCollisions=True is provided
    to the constructor (True by default).

    GraspWithConfig objects will be returned.
    """
    def __init__(self,
                 sampler: GraspSamplerBase,
                 robotIndex:Union[int,str]=0,
                 restartsPerGrasp:int=1,
                 seedConfig:Optional[Config]=None,
                 checkCollisions:bool=True):
        self.sampler = sampler
        self._gripper = sampler.gripper()
        self.robotIndex = robotIndex
        self.seedConfig = seedConfig
        self.restartsPerGrasp = restartsPerGrasp
        self.checkCollisions = checkCollisions
        self.world = None  # type: Optional[WorldModel]
        self.object = None  # type: Optional[RigidObjectModel]
        self.robot = None  # type: Optional[RobotModel]
        self.gripperLinks = None
        self.nonGripperLinks = None

    def gripper(self) -> GripperInfo:
        return self._gripper

    def init(self, scene : WorldModel, object : RigidObjectModel, hints=None) -> bool:
        if not self.sampler.init(scene, object, hints):
            return False
        self.world = scene
        self.object = object
        self.robot = scene.robot(self.robotIndex)
        self.gripperLinks = self._gripper.descendantLinks(self.robot)
        self.nonGripperLinks = [i for i in range(self.robot.numLinks()) if i not in self.gripperLinks]
        return True
    
    def feasibilityTest(self) -> bool:
        """Returns True if the robot is in a valid configuration."""
        if self.robot is None:
            return False
        if self.robot.selfCollides(): 
            return False
        if self.checkCollisions:
            for i in range(self.world.numRigidObjects()):
                if i == self.object.index:
                    #check the non-gripper links
                    for j in self.nonGripperLinks:
                        if self.robot.link(j).geometry().collides(self.world.rigidObject(i).geometry()):
                            return False
                else:
                    #check all links
                    for j in range(self.robot.numLinks()):
                        if self.robot.link(j).geometry().collides(self.world.rigidObject(i).geometry()):
                            return False
        return True 
    
    def next(self) -> Optional[GraspWithConfig]:
        while True:
            g = self.sampler.next()
            if g is None:
                return None
            if not isinstance(g,Grasp):
                raise TypeError("GraspSamplerWithIK: expected a Grasp, got %s" % type(g))
            
            #check feasibility
            ikConstraint = g.ikConstraint
            if ikConstraint is None:
                raise ValueError("GraspSamplerWithIK: grasp does not have an IK constraint")
            
            if self.seedConfig is not None:
                self.robot.setConfig(self.seedConfig)

            #set the robot configuration to the grasp's finger configuration
            self.robot.setConfig(self._gripper.setFingerConfig(self.robot.getConfig(),g.fingerConfig))
            ikConstraint.robot = self.robot
            if self.solve_ik(ikConstraint):
                #if feasible, create a GraspWithConfig object
                robotConfig = self._gripper.setFingerConfig(self.robot.getConfig(),g.fingerConfig)  #double check that the finger config is set correctlyd
                return GraspWithConfig(ikConstraint,g.fingerLinks,g.fingerConfig,g.contacts,robotConfig,g.score)
            else:
                #if not feasible, get a new grasp
                pass

    def solve_ik(self, ikConstraint: IKObjective) -> bool:
        """Attempts to solve the IK constraint.  If seedConfig is given, the
        current robot configuration is set to it.

        Subclass can override this to use a different IK solver or add additional
        parameters.
        
        Returns True if a feasible solution is found, False otherwise.
        """
        return ik.solve_global(ikConstraint, numRestarts=self.restartsPerGrasp,feasibilityCheck=self.feasibilityTest,startRandom=(self.seedConfig is None))


class GraspWithConfig(Grasp):
    """Result from a GraspSamplerBase that may return a robot configuration
    as well as the Grasp.  Notably, used in GraspSamplerWithIK.
    """
    def __init__(self, ikConstraint:Optional[IKObjective]=None,
                 fingerLinks:Optional[List[int]]=None,
                 fingerConfig:Optional[List[float]]=None,
                 contacts:Optional[List[ContactPoint]]=None,
                 robotConfig:Optional[Config]=None,
                 score:Optional[float]=None):
        Grasp.__init__(self,ikConstraint,fingerLinks,fingerConfig,contacts,score)
        self.robotConfig = robotConfig

    def toJson(self):
        """Returns a JSON-compatible object storing all of the grasp/config
        data.
        """
        res = Grasp.toJson(self)
        res['robotConfig'] = self.robotConfig
        return res

    @staticmethod
    def fromJson(jsonObj : dict) -> GraspWithConfig:
        """Creates the GraspWithConfig from a JSON-compatible object previously
        saved by toJson.
        """
        g = Grasp.fromJson(jsonObj)
        robotConfig = jsonObj.get('robotConfig',None)
        return GraspWithConfig(g.ikConstraint,g.fingerLinks,g.fingerConfig,robotConfig,g.score)

    def addToVis(self,prefix:str):
        from klampt import vis
        Grasp.addToVis(self,prefix)
        vis.add(prefix+"_config",self.robotConfig,color=(0,0,1,0.5))

    def removeFromVis(self,prefix:str):
        from klampt import vis
        Grasp.removeFromVis(self,prefix)
        vis.remove(prefix+"_config")
