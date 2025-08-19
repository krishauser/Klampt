from ..model.robotinfo import GripperInfo
from .grasp import Grasp
from .grasp_sampler import GraspSamplerBase
from ..math import vectorops,so3,se3
from .. import RobotModel,RigidObjectModel
import json
import os
from ..model.typing import RigidTransform
from typing import Dict,List,Union,Optional

def _object_name(obj):
    if isinstance(obj,str):
        return obj
    elif hasattr(obj,'name'):
        return obj.name
    elif hasattr(obj,'getName'):
        return obj.getName()
    else:
        raise ValueError("Can't determine name of object {}".format(obj))


class GraspDatabase:
    """A database of grasps, loadable from disk"""
    def __init__(self, gripper : GripperInfo, fn : str = None):
        if not isinstance(gripper, GripperInfo):
            raise ValueError("gripper needs to be a GripperInfo")
        self.gripper = gripper
        self.objects = []                # type   : List[str]
        self.object_to_grasps = dict()   # type   : Dict[str, List[Grasp]]
        if fn is not None:
            self.load(fn)

    def print_info(self):
        print("Grasp database statistics:")
        print("Gripper:",self.gripper.name)
        for (o,gs) in self.object_to_grasps:
            print("Object",o,":",len(gs),"grasps")

    def load(self,fn : str) -> bool:
        with open(fn,'r') as f:
            jsonobj = json.load(f)
        self.objects = jsonobj['objects']
        self.object_to_grasps = dict()
        for o,gs in jsonobj['object_to_grasps'].items():
            grasps = []
            for g in gs:
                gparsed = Grasp(None)
                gparsed.fromJson(g)
                grasps.append(gparsed)
            self.object_to_grasps[o] = grasps
        return True

    def save(self, fn : str) -> bool:
        jsonobj = dict()
        jsonobj['objects'] = self.objects
        grasp_dict = dict()
        for o,gs in self.object_to_grasps.items():
            gsjson = [g.toJson() for g in gs]
            grasp_dict[o] = gsjson
        jsonobj['object_to_grasps'] = grasp_dict
        with open(fn,'w') as f:
            json.dump(jsonobj,f)
        return True

    def loadFolder(self,fn : str) -> bool:
        """Reads from a folder containing object folders, each of which contains
        some set of grasp json files."""
        for obj in os.listdir(fn):
            print(obj)
            if os.path.isdir(os.path.join(fn,obj)):
                inobjs = False
                for gfn in os.listdir(os.path.join(fn,obj)):
                    if gfn.endswith('.json') and self.gripper.name in gfn:
                        if not inobjs:
                            if obj not in self.objects:
                                self.objects.append(obj)
                                self.object_to_grasps[obj] = []  
                            inobjs = True
                            print("Loading grasps for object",obj,"...")
                        with open(os.path.join(fn,obj,gfn),'r') as f:
                            jsonobj = json.load(f)
                        try:
                            gparsed = Grasp(None)
                            gparsed.fromJson(jsonobj)
                            self.object_to_grasps[obj].append(gparsed)
                        except Exception as e:
                            print("Unable to load",os.path.join(fn,obj,gfn),"as a Grasp")
                            raise

    def addObject(self, name : str):
        if name in self.objects:
            raise ValueError("Object {} already exists".format(name))
        self.objects.append(name)
        self.object_to_grasps[name] = []

    def addGrasp(self, object : Union[str, RigidObjectModel], grasp : Grasp):
        """Adds a new Grasp to the database for the given object.

        If the object is not at the origin, the grasp will be transformed
        to the object's local frame.
        """
        if not isinstance(grasp,Grasp):
            raise ValueError("grasp needs to be a Grasp")
        oname = _object_name(object)
        if oname not in self.object_to_grasps:
            self.addObject(oname)
        if isinstance(object, RigidObjectModel):
            Tinv = se3.inv(object.getTransform())
            grasp = grasp.getTransformed(Tinv)
        self.object_to_grasps[oname].append(grasp)
    
    def addGraspSamples(self, object : Union[str, RigidObjectModel], sampler : GraspSamplerBase, max_samples : int = 100):
        """Adds samples from a GraspSamplerBase to the database for the given object.

        Assumes that the grasp sampler is initialized for the object and gripper.

        If the object is not at the origin, grasps will be transformed
        to the object's local frame.
        """
        if not isinstance(sampler,GraspSamplerBase):
            raise ValueError("sampler needs to be a GraspSamplerBase")
        if max_samples <= 0:
            return
        oname = _object_name(object)
        if oname not in self.object_to_grasps:
            self.addObject(oname)
        for i in range(max_samples):
            g = sampler.next()
            if g is None:
                break
            self.addGrasp(oname,g)

    def sampler(self, robot : RobotModel) -> GraspSamplerBase:
        """Returns a GraspDatabaseSampler for this robot."""
        return GraspDatabaseSampler(robot,self.gripper,self.object_to_grasps)


class GraspDatabaseSampler(GraspSamplerBase):
    """A GraspSamplerBase subclass that will read from a dict of
    object-centric Grasp's.

    Args:
        robot (RobotModel): the robot.
        gripper (GripperInfo): the gripper
        object_to_grasps (dict of str -> list): for each object, gives a list
            of Grasp templates.
    """
    def __init__(self,robot : RobotModel, gripper : GripperInfo, object_to_grasps : Dict[str, List[Grasp]]):
        self._robot = robot
        self._gripper = gripper
        self._object_to_grasps = object_to_grasps  # type: Dict[str, List[Grasp]]
        self._target_object = None
        self._matching_object = None
        self._matching_xform = None
        self._grasp_index = 0

    def object_match(self, object_source: Union[str, RigidObjectModel], object_target: Union[str, RigidObjectModel]) -> Optional[RigidTransform]:
        """Determine whether object_source is a match to object_target.
        If they match, return a transform from the reference frame of
        object_source to object_target.  Otherwise, return None.

        Default implementation: determines whether the name of
        object_source matches object_target.name or object_target.getName()
        exactly.

        Subclasses can override this to provide more sophisticated
        matching, such as checking for similarity between object types
        within an object class.
        """
        if _object_name(object_source) == _object_name(object_target):
            if isinstance(object_source, RigidObjectModel) and isinstance(object_target, RigidObjectModel):
                return se3.mul(se3.inv(object_source.getTransform()), object_target.getTransform())
            elif isinstance(object_source, str) and isinstance(object_target, RigidObjectModel):
                return object_target.getTransform()
            return se3.identity()
        return None

    def gripper(self):
        return self._gripper

    def init(self, scene, object : RigidObjectModel, hints=None):
        """Checks for either an exact match or if object_match(o,object)
        exists"""
        if _object_name(object) in self._object_to_grasps:
            self._target_object = object
            self._matching_object = _object_name(object)
            self._matching_xform = se3.identity()
            self._grasp_index = 0
            return True
        for o,g in self._object_to_grasps.items():
            xform = self.object_match(o,object)
            if xform is not None:
                self._target_object = object
                self._matching_object = o
                self._matching_xform = xform
                self._grasp_index = 0
                return True
        return False

    def next(self):
        """Returns the next Grasp from the database."""
        if self._matching_object is None:
            return None
        grasps = self._object_to_grasps[self._matching_object]  # type: List[Grasp]
        if self._grasp_index >= len(grasps):
            self._matching_object = None
            return None
        grasp = grasps[self._grasp_index]
        self._grasp_index += 1
        return grasp.getTransformed(se3.mul(self._target_object.getTransform(),self._matching_xform))
    
    def score(self):
        """Returns a score going from 1 to 0 as the number of grasps
        gets exhausted.
        """
        if self._matching_object is None: return 0
        grasps = self._object_to_grasps[self._matching_object]
        if self._grasp_index >= len(grasps):
            return 0
        if grasps[self._grasp_index].score is not None:
            return grasps[self._grasp_index].score
        #if no score, return a linear score
        return 1.0 - self._grasp_index/float(len(grasps))


