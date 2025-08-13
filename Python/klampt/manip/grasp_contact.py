from .grasp_sampler import GraspSamplerBase
from klampt.model.gripperinfo import GripperInfo
from klampt.model.typing import Config
from ..model.contact import ContactPoint
from ..math import se3,so3,vectorops
from .. import Geometry3D, RigidObjectModel, RobotModel, WorldModel, DistanceQueryResult, DistanceQuerySettings
from .grasp import Grasp
import math
from ..model.typing import RigidTransform,Vector3
from typing import Optional,Union,List,Tuple,Callable

def _object_shift(geom1 : Geometry3D, geom2 : Geometry3D, desired_penetration:float=0.0) -> Tuple[Vector3,Vector3,Vector3]:
    """Returns the shift vector to apply to geom1 so that it is shifted to
    have the desired penetration depth with respect to geom2.
    
    Returns:
        A tuple (shift, contact, normal) where shift is the vector to apply to
        geom1, and contact and normal are the closest point / normal vector on
        the surface of geom2.
    """
    res = geom1.distance(geom2)
    if not res.hasClosestPoints and not res.hasGradients:
        print("contact_cleanup: Warning, base link does not have closest points to object geometry, cannot determine shift")
        return None
    if res.hasGradients:
        assert res.hasClosestPoints, "contact_cleanup: distance query result has gradients but not closest points"
        shift = vectorops.mul(res.grad1,-res.d - desired_penetration)
        normal = vectorops.unit(res.grad1)
        return shift, res.cp2, normal
    else:
        shift = vectorops.sub(res.cp2,res.cp1)
        normal = vectorops.mul(vectorops.unit(shift),-1.0)
        shift = vectorops.madd(shift, normal, -desired_penetration)
        return shift, res.cp2, normal


def contact_cleanup(gripper: GripperInfo, robot:RobotModel, object: RigidObjectModel,
                    grasp: Grasp,
                    translate_base:bool=True,
                    rotate_base:bool=True,
                    desired_penetration:float=0.0) -> Optional[Grasp]:
    """Given a gripper, an object, and a grasp, returns a new grasp
    where the gripper's fingers are moved to make contact with the object.

    If translate_base and/or rotate_base are True, the gripper's base will
    be shifted (a small amount) so that contact is made more precisely.
    One of these must be True for cleanup of a vacuum gripper, but can be False
    for a parallel or multi-finger gripper.
    
    If desired_penetration is provided, the fingers will be moved
    so that the contact points are approximately at that penetration depth.
    If not provided, the fingers will be moved to just touch the object.
    
    Note that the object's geometry should be an ImplicitSurface, Heightmap,
    or GeometricPrimitive type so that penetration depth information is
    available.

    Returns:
        A new Grasp object with gripper's fingers in contact with the object,
        if such a shift is possible.  The contact points set to the
        ContactPoint information representing the estimated contact points.
        If the shift is not possible, returns None.

    Note: the robot's configuration may be changed by this function.
    """
    base_link_geom = robot.link(gripper.baseLink).geometry()
    finger_link_geoms = [robot.link(l).geometry() for l in gripper.fingerLinks] 
    ogeom = object.geometry()
    if desired_penetration > 0.0 and ogeom.type() not in ['ImplicitSurface', 'Heightmap', 'GeometricPrimitive']:
        print("contact_cleanup: Warning, object geometry should be an ImplicitSurface, Heightmap, or GeometricPrimitive for best results")
    if gripper.type == 'vacuum':
        if not translate_base and not rotate_base:
            return None  #not possible to make contact without shifting the base
        if len(finger_link_geoms) > 0:
            print("contact_cleanup: Warning, vacuum gripper has finger links?")
        #TODO: maybe use ogeom.distance_point() to the gripper center instead?
        shift,cp,normal = _object_shift(base_link_geom, ogeom, desired_penetration)
        T = (so3.identity(), shift)
        newgrasp = grasp.getTransformed(T)
        if rotate_base:
            #align primary axis to the normal
            local_axis, world_axis = grasp.ikConstraint.getRotationAxis()
            if vectorops.distance(local_axis, gripper.primaryAxis) > 1e-3:
                print("contact_cleanup: Warning, gripper's primary axis does not match the ikConstraint's rotation axis, rotation alignment may be incorrect")
            newgrasp.ikConstraint.setAxialRotConstraint(local_axis,vectorops.mul(normal,-1))
        newgrasp.contacts = [ContactPoint(cp, normal, 1.0)]
        newgrasp.contacts[0].object1 = robot.link(gripper.baseLink)
        newgrasp.contacts[0].object2 = object
        return newgrasp
    elif gripper.type == 'parallel':
        #need to search for an opening width that makes contact with both sides of the object
        fingers = gripper.eachFingerLinks()
        assert len(fingers) == 2, "contact_cleanup: parallel gripper should have exactly two finger links"
        left_finger_geoms = [robot.link(f).geometry() for f in fingers[0]]
        right_finger_geoms = [robot.link(f).geometry() for f in fingers[1]]
        if translate_base:
            #make sure the base does not penetrate the object
            shift,cp,normal = _object_shift(base_link_geom, ogeom, 0.0)
            T = (so3.identity(), shift)
            newgrasp = grasp.getTransformed(T)
        else:
            newgrasp = grasp.getTransformed(se3.identity())
        #look for a finger configuration that makes contact
        robot.setConfig(gripper.setFingerConfig(robot.getConfig(),gripper.openConfig))
        left_open_dist = math.inf
        right_open_dist = math.inf
        left_close_dist = math.inf
        right_close_dist = math.inf
        for g in left_finger_geoms:
            res = g.distance(ogeom)
            if res.d < -desired_penetration:
                print("contact_cleanup: warning, left finger penetrates object beyond desired penetration")
            left_open_dist = min(left_open_dist,res.d + desired_penetration)
        for g in right_finger_geoms:
            res = g.distance(ogeom)
            if res.d < -desired_penetration:
                print("contact_cleanup: warning, right finger penetrates object beyond desired penetration")
                right_open_dist = min(right_open_dist,res.d + desired_penetration)
        robot.setConfig(gripper.setFingerConfig(robot.getConfig(),gripper.closedConfig))
        for g in left_finger_geoms:
            res = g.distance(ogeom)
            if res.d < -desired_penetration:
                print("contact_cleanup: warning, left finger penetrates object beyond desired penetration")
            left_close_dist = min(left_close_dist,res.d + desired_penetration)
        for g in right_finger_geoms:
            res = g.distance(ogeom)
            if res.d < -desired_penetration:
                print("contact_cleanup: warning, right finger penetrates object beyond desired penetration")
            right_close_dist = min(right_close_dist,res.d + desired_penetration)
        if not math.isfinite(left_open_dist) or not math.isfinite(right_open_dist):
            print("contact_cleanup: warning, left or right finger does not have a valid open distance to the object")
            return None
        if not math.isfinite(left_close_dist) or not math.isfinite(right_close_dist):
            print("contact_cleanup: warning, left or right finger does not have a valid close distance to the object")
            return None
        #assume distance is linearly varying in openness, d = (open_dist-close_dist)*f + close_dist, and we wish
        #to set d = 0
        left_open_fraction = -left_close_dist / (left_open_dist - left_close_dist) if left_open_dist - left_close_dist != 0 else 0
        right_open_fraction = -right_close_dist / (right_open_dist - right_close_dist) if right_open_dist - right_close_dist != 0 else 0
        left_open_fraction = max(0.0,min(1.0,left_open_fraction))
        right_open_fraction = max(0.0,min(1.0,right_open_fraction))
        if not translate_base:
            #just take the average of the two fractions
            open_fraction = 0.5*(left_open_fraction + right_open_fraction)
            newgrasp.fingerConfig = gripper.partwayOpenConfig(open_fraction)
        else:
            #translate according to the secondary axis
            open_fraction = 0.5*(left_open_fraction + right_open_fraction)
            newgrasp.fingerConfig = gripper.partwayOpenConfig(open_fraction)
            shift_right_amount = 0.5*(left_open_dist - right_open_dist)
            shift = vectorops.mul(gripper.secondaryAxis, shift_right_amount)
            T = (so3.identity(), shift)
            newgrasp = newgrasp.getTransformed(T)
        #TODO: any rotation?
        return newgrasp
    else:
        raise NotImplementedError("contact_cleanup: only vacuum and parallel grippers are supported so far")


class GraspSamplerWithContactCleanup(GraspSamplerBase):
    """A grasp sampler that cleans up a base sampler's grasps by moving
    the gripper's fingers so that they make contact with the object.

    The grasps also include ContactPoint information
    """
    def __init__(self, base_sampler: GraspSamplerBase,
                 translate_base:bool=True,
                 rotate_base:bool=True,
                 desired_penetration:float=0.0,
                 robotIndex:Union[int,str]=0):
        """Initializes the grasp sampler.

        Args:
            base_sampler: the underlying grasp sampler
            translate_base: if True, the gripper's base will be translated
                slightly to make contact.  This should be True for vacuum
                grippers and can be False for parallel grippers.
            rotate_base: if True, the gripper's base will be rotated slightly
                to make contact.  This should be True for vacuum grippers and
                can be False for parallel grippers.
            desired_penetration: if provided, the fingers will be moved so that
                the contact points are approximately at that penetration depth.
                If not provided, the fingers will be moved to just touch the object.
        """
        self.base_sampler = base_sampler
        self.translate_base = translate_base
        self.rotate_base = rotate_base
        self.desired_penetration = desired_penetration
        self.robotIndex = robotIndex
        self.robot = None
        self.object = None

    def gripper(self) -> GripperInfo:
        return self.base_sampler.gripper()

    def init(self, scene: WorldModel, object: RigidObjectModel, hints = None) -> bool:
        if not self.base_sampler.init(scene, object, hints):
            return False
        self.robot = scene.robot(self.robotIndex)
        self.object = object
        return True

    def next(self) -> Optional[Grasp]:
        while True:
            g = self.base_sampler.next()
            if g is None:
                return None
            newg = contact_cleanup(self.gripper(), self.robot, self.object, g,
                                   translate_base=self.translate_base,
                                   rotate_base=self.rotate_base,
                                   desired_penetration=self.desired_penetration)
            if newg is not None:
                return newg