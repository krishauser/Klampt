from __future__ import annotations
from ..model.gripperinfo import GripperInfo
from .grasp import Grasp
from .grasp_sampler import GraspSamplerBase
from .. import Geometry3D,GeometricPrimitive,RigidObjectModel,IKObjective
from ..math import vectorops,so3,se3
import math
import numpy as np
from ..model.typing import Vector3, RigidTransform
from typing import Union,Optional,Callable,Tuple

def _angle(axis1: Vector3, axis2: Vector3) -> float:
    """Returns the angle between two unit vectors."""
    dp = vectorops.dot(axis1, axis2)
    if dp >= 1.0:
        return 0.0
    elif dp <= -1.0:
        return math.pi
    else:
        return np.arccos(dp)


class PrimitiveGraspSampler(GraspSamplerBase):
    """A GraspSamplerBase subclass that will find appropriate grasps for
    a geometric primitive.  Vacuum grippers will pick from the top down,
    and parallel jaw grippers will pick from the sides.  This cannot be used
    for multi-fingered grippers.

    If the object is a box, then it will try to find grasps on each side, with
    orientations that fit within the gripper's maximum span. Grasps are sampled and
    then scored by their verticality, centrality, and finger clearance.

    - verticality = angle(down, gripper primary axis).
    - centrality = distance from the center of the object along the chosen axis.
    - finger clearance = how much the gripper's maximum span exceeds the width of
        the object along the chosen axis.
    
    The default penalty is
        vertical_penalty * verticality + centrality_penalty * centrality + clearance_penalty * max(0, finger width - finger clearance).

    If the object is a cylinder, then it will try to find grasps along top and
    bottom faces and along the sides. 

    If the object is a sphere, then it will try to find grasps all over the
    surface.  All grasps are "central" so the centrality penalty is not applied.

    If the centrality weight is set to infinity, then only the center of the
    top of the object will be sampled (default).  There still may be multiple
    grasps sampled, depending on the symmetry of the object.

    All the calculation is done in the init() method, so next() just
    iterates through the grasps found.
    """
    def __init__(self, gripper : GripperInfo,
                 vertical_penalty : Union[float,Callable] = 1.0,
                 centrality_penalty : Union[float,Callable] = math.inf,
                 clearance_penalty : Union[float,Callable] = 1.0,
                 sampling_resolution : Optional[float] = None):
        self._gripper = gripper
        assert gripper.primaryAxis is not None,"Gripper needs a primary axis"
        assert gripper.maximumSpan is not None,"Gripper needs an opening span"
        if gripper.type != 'parallel' and gripper.type != 'vacuum':
            raise ValueError("Gripper type must be 'parallel' or 'vacuum'")
        self.vertical_penalty = vertical_penalty
        self.centrality_penalty = centrality_penalty
        self.clearance_penalty = clearance_penalty
        self.sampling_resolution = sampling_resolution if sampling_resolution is not None else gripper.maximumSpan / 4.0
        if not callable(vertical_penalty):
            self.vertical_penalty = lambda angle:angle*vertical_penalty
        if not callable(centrality_penalty) and not math.isinf(centrality_penalty):
            self.centrality_penalty = lambda dist:dist*centrality_penalty
        if not callable(clearance_penalty):
            self.clearance_penalty = lambda width:max(gripper.fingerWidth-width,0)*clearance_penalty
        self.primitive = None  # type: Optional[GeometricPrimitive]
        self.primitive_xform = None  # type: Optional[RigidTransform]
        self.options = None    # type: Optional[list[tuple[Grasp,float]]]
        self.index = None

    def gripper(self):
        return self._gripper

    def init(self,scene,object:Union[RigidObjectModel,Geometry3D,GeometricPrimitive],hints:Optional[Vector3]=None) -> bool:
        """Needs object to contain a GeometricPrimitive."""
        if not isinstance(object,(RigidObjectModel,Geometry3D,GeometricPrimitive)):
            print("Need to pass an object as a RigidObjectModel, Geometry3D, or GeometricPrimitive")
            return False
        if isinstance(object,RigidObjectModel):
            return self.init(scene,object.geometry(),hints)
        prim = None
        xform = None
        if isinstance(object,Geometry3D):
            if object.type() != 'GeometricPrimitive':
                #just calculate the bounding box?
                prim = GeometricPrimitive()
                T = object.getCurrentTransform()
                object.setCurrentTransform(*se3.identity())
                prim.setAABB(*object.getBBTight())
                object.setCurrentTransform(*T)
            else:
                prim = object.getGeometricPrimitive()
            xform = object.getCurrentTransform()
        else:
            prim = object
            xform = se3.identity()
        if hints is None:
            up = [0,0,1]  #prefer vertical axis
        else:
            up = hints[:]
            if len(up) != 3:
                raise ValueError("Hints must be a 3D up vector")
        self.primitive = prim
        self.primitive_xform = xform
        local_vertical = so3.apply(so3.inv(xform[0]),up)  # vertical axis in primitive's local frame

        options = []   # type: list[tuple[Grasp,float]]
        if prim.type == 'aabb':
            bmin, bmax = prim.properties[:3], prim.properties[3:6]
            center = vectorops.mul(vectorops.add(bmin, bmax), 0.5)
            width = vectorops.sub(bmax, bmin)
            for i in range(3):
                j = (i + 1) % 3
                k = (i + 2) % 3
                palm_center = center[:]
                for sign in [-1, 1]:
                    palm_center[i] = bmax[i] if sign > 0 else bmin[i]
                    axis_up = [0,0,0]
                    axis_up[i] = sign
                    axis_1 = [0,0,0]
                    axis_1[j] = sign
                    axis_2 = [0,0,0]
                    axis_2[k] = sign
                    vertical_penalty = self.vertical_penalty(_angle(axis_up, local_vertical))

                    if not callable(self.centrality_penalty):
                        if self._gripper.type == 'parallel':
                            if width[j] < self._gripper.maximumSpan and (self._gripper.minimumSpan is None or width[j] > self._gripper.minimumSpan):
                                g,s = self.make_grasp(palm_center, axis_up, axis_1, width[j])
                                options.append((g,s + vertical_penalty))
                            if width[k] < self._gripper.maximumSpan and (self._gripper.minimumSpan is None or width[k] > self._gripper.minimumSpan):
                                g,s = self.make_grasp(palm_center, axis_up, axis_2, width[k])
                                options.append((g,s + vertical_penalty))
                        else:
                            #just add one at the center
                            g,s = self.make_grasp(palm_center, axis_up, axis_1, 0.0)
                            options.append((g,s + vertical_penalty))
                    else:
                        #sample along the adjacent edges
                        if self._gripper.type == 'parallel':
                            if width[j] < self._gripper.maximumSpan and (self._gripper.minimumSpan is None or width[j] > self._gripper.minimumSpan):
                                for s in np.arange(bmin[k] + self._gripper.fingerWidth*0.5, bmax[k] - self._gripper.fingerWidth*0.5, self.sampling_resolution):
                                    palm_center[k] = s
                                    g,s = self.make_grasp(palm_center, axis_up, axis_1, width[j])
                                    centrality_penalty = self.centrality_penalty(abs(s - center[k]))
                                    options.append((g,s + centrality_penalty + vertical_penalty))
                            if width[k] < self._gripper.maximumSpan and (self._gripper.minimumSpan is None or width[k] > self._gripper.minimumSpan):
                                for s in np.arange(bmin[j] + self._gripper.fingerWidth*0.5, bmax[j] - self._gripper.fingerWidth*0.5, self.sampling_resolution):
                                    palm_center[j] = s
                                    g,s = self.make_grasp(palm_center, axis_up, axis_2, width[k])
                                    centrality_penalty = self.centrality_penalty(abs(s - center[j]))
                                    options.append((g,s + centrality_penalty + vertical_penalty))
                        else:
                            #vacuum grasp, sample along the face
                            for s in np.arange(bmin[j] + self._gripper.maximumSpan*0.5, bmax[j] - self._gripper.maximumSpan*0.5, self.sampling_resolution):
                                palm_center[j] = s
                                for t in np.arange(bmin[k] + self._gripper.maximumSpan*0.5, bmax[k] - self._gripper.maximumSpan*0.5, self.sampling_resolution):
                                    palm_center[k] = t
                                    g,s = self.make_grasp(palm_center, axis_up, axis_1, width[j])
                                    centrality_penalty = self.centrality_penalty(vectorops.distance((s,t),(center[j],center[k])))
                                    options.append((g,s + centrality_penalty + vertical_penalty))
        elif prim.type == 'cylinder':
            raise NotImplementedError("Cylinder grasp sampling not implemented yet")
        elif prim.type == 'sphere':
            raise NotImplementedError("Sphere grasp sampling not implemented yet")
        else:
            print("PrimitiveGraspSampler only supports AABBs, cylinders, and spheres")
            return False
        
        if len(options) == 0:
            print("PrimitiveGraspSampler found no grasps, maybe object is too large?  Gripper max span is",self._gripper.maximumSpan)
            return False
        self.options = sorted(options,key=lambda x:x[1])
        self.index = 0
        return True

    def make_grasp(self, palm_center : Vector3, axis_up : Vector3, axis_side : Vector3, width : float) -> Tuple[Grasp,float]:
        center = se3.apply(self.primitive_xform, palm_center)
        up = so3.apply(self.primitive_xform[0], axis_up)
        side = so3.apply(self.primitive_xform[0], axis_side)
        if self._gripper.type == 'vacuum':
            score = 0
            objective = IKObjective()
            objective.setFixedPosConstraint(self._gripper.center,center)
            objective.setAxialRotConstraint(self._gripper.primaryAxis,vectorops.mul(up,-1))
            return Grasp(objective), score
        elif self._gripper.type == 'parallel':
            # control how much the gripper overlaps the object, right now set to finger width
            finger_overlap = self._gripper.fingerWidth
            center_target = vectorops.madd(center, up, self._gripper.fingerLength - finger_overlap)
            clearance_penalty = self.clearance_penalty(self._gripper.maximumSpan - width)
            score = clearance_penalty
            objective = IKObjective()
            objective.setLinks(self._gripper.baseLink)
            objective.setFixedPosConstraint(self._gripper.center,center_target)  
            #need to align the gripper's primary axis with the negative up axis and the secondary axis to the side axis
            R = so3.align(self._gripper.primaryAxis, vectorops.mul(up,-1))
            swivel_angle = _angle(so3.apply(R,self._gripper.secondaryAxis),side)
            R_swivel = so3.rotation(self._gripper.primaryAxis,swivel_angle)
            objective.setFixedRotConstraint(so3.mul(R,R_swivel))
            opening = self._gripper.widthToOpening(width)
            finger_config = self._gripper.partwayOpenConfig(opening)
            return Grasp(objective, fingerLinks=self._gripper.fingerLinks, fingerConfig=finger_config), score
        else:
            raise ValueError("Gripper type must be 'parallel' or 'vacuum'")

    def next(self) -> Optional[Grasp]:
        """Returns the next Grasp from the database."""
        if self.options is None:
            return None
        
        if self.index >= len(self.options):
            self.options = None
            return None

        grasp,score = self.options[self.index]
        self.index += 1
        grasp.score = math.exp(-score)
        return grasp

    def score(self):
        """Returns the top score.
        """
        if self.options is None: return 0
        if self.index >= len(self.options):
            return 0
        return math.exp(-self.options[self.index][1])

    def visDebug(self):
        """Visualizes the flat areas found."""
        from klampt import vis
        if self.options is None: return
        if self.index >= len(self.options):
            return
        for i,(grasp,score) in enumerate(self.options[self.index:]):
            score = math.exp(-score)
            opacity = score
            #TODO: colorize with score
            grasp.addToVis(prefix=f"grasp_{i}", gripper=self._gripper, hide_label=True)
