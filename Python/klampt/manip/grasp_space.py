from .. import WorldModel, IKObjective
from ..model import ik
from ..math import so3,se3
from ..model.gripperinfo import GripperInfo
from .grasp import Grasp
from ..model.typing import Vector
from ..io import features
import copy
import math
from typing import List, Union, Optional, Tuple

class GraspParameterSpace:
    """Allows for a vectorized representation of a Grasp, which is useful for
    optimization and learning tasks.

    The space is based on a template, which is a Grasp object that is used to
    define the attributes of the grasp outside of the parameter space.  The
    `items` attribute is a list of strings that refer to the attributes of the
    grasp that are to be parameterized.  

    Attributes:
        gripper (GripperInfo, optional): the info of the gripper being used.
        template (Grasp): the grasp template.
        template_json (dict): the JSON object belonging to template
        items (list of str or list of list of str): the grasp items referred
            to by the parameter space.  Can be:
            
            - 'transform' (6D rotation_vector + translation concatenated)
            - 'fingerDriverConfig' (len(gripper.fingerDrivers))
            - 'fingerConfig' (len(gripper.fingerLinks))
            - ['ikConstraint','endPosition'] (3D),
            - ['ikConstraint','endRotation'] (3D),
            - 'robotConfig'

            Item strings can also refer to sub-indices, like
            ['ikConstraint','endPosition',0] or ['transform',3].  See the
            :mod:`klampt.io.features` module for more information on how to
            specify items.
    
    
    If the first argument is a GripperInfo, then the template is set to a
    Grasp with the gripper's closed configuration as the finger configuration
    and the constraint set to a fixed transform on the gripper's base link
    (for parallel and multi-finger grippers) or an axial rotation constraint
    (for suction grippers).
    """
    def __init__(self, template : Union[Grasp,GripperInfo], items : Union[List[str],List[List[str]]], gripper : Optional[GripperInfo] = None):
        self.gripper = gripper
        if isinstance(template,GripperInfo):
            self.gripper = template    
        else:
            self.gripper = None
        if self.gripper is not None:
            w = WorldModel()
            res = w.readFile(self.gripper.klamptModel)
            if not res:
                raise RuntimeError("GraspParameterSpace: Error, could not load gripper model")
            self.world = w
            self.robot = w.robot(0)
        if isinstance(template,GripperInfo):
            q = self.gripper.closedConfig if self.gripper.closedConfig is not None else [0]*len(self.gripper.fingerLinks)
            if template.type == 'vacuum':
                # use an axis constraint rather than fixed transform
                fixed_constraint = IKObjective()
                fixed_constraint.setLinks(self.gripper.baseLink)
                fixed_constraint.setFixedPoint(self.gripper.center,[0,0,0])
                fixed_constraint.setAxialRotConstraint(self.gripper.primaryAxis,[0,0,-1])
            else:
                fixed_constraint = ik.fixed_objective(self.robot.link(self.gripper.baseLink))
            template = Grasp(fixed_constraint,self.gripper.fingerLinks,q)
        self.template = template
        self.template_json = template.toJson()
        self.items = items
        self.use_transform = ('transform' in items or any(i[0] == 'transform' for i in items))
        self.use_drivers = ('fingerDriverConfig' in items or any(i[0] == 'fingerDriverConfig' for i in items))
        if self.use_transform:
            T = self.template.ikConstraint.closestMatch(*se3.identity())
            self.template_json['transform'] = so3.rotation_vector(T[0]) + T[1]
        if self.use_drivers:
            if self.gripper is None:
                raise ValueError("Cannot use fingerDriverConfig unless gripper is provided")
            #self.template_json['fingerDriverConfig'] = [0]*len(self.gripper.fingerDrivers)
            self.robot.setConfig(self.gripper.setFingerConfig(self.robot.getConfig(),template.fingerConfig))
            qdriver = [self.robot.driver(i).getValue() for i in self.gripper.fingerDrivers]
            self.template_json['fingerDriverConfig'] = qdriver

        #check
        try:
            nd = self.numDims()
        except Exception as e:
            import traceback
            traceback.print_exc()
            raise ValueError("Invalid items specified")

        self.min_template_json = copy.deepcopy(self.template_json)
        self.max_template_json = copy.deepcopy(self.template_json)
        inf = float('inf')
        self.min_template_json['ikConstraint']['endRotation'] = [-math.pi,-math.pi,-math.pi]
        self.max_template_json['ikConstraint']['endRotation'] = [math.pi,math.pi,math.pi]
        self.min_template_json['ikConstraint']['endPosition'] = [-inf,-inf,-inf]
        self.max_template_json['ikConstraint']['endPosition'] = [inf,inf,inf]
        self.min_template_json['fingerConfig'] = [-inf]*len(self.template_json['fingerConfig'])
        self.max_template_json['fingerConfig'] = [inf]*len(self.template_json['fingerConfig'])
        if self.use_transform:
            self.min_template_json['transform'] = [-math.pi,-math.pi,-math.pi,-inf,-inf,-inf]
            self.max_template_json['transform'] = [math.pi,math.pi,math.pi,inf,inf,inf]
        
            if len(self.template.fingerLinks) > 0:
                qmin,qmax = self.robot.getJointLimits()
                self.min_template_json['fingerConfig'] = [qmin[i] for i in self.template.fingerLinks]
                self.max_template_json['fingerConfig'] = [qmax[i] for i in self.template.fingerLinks]

        if self.use_drivers and len(self.gripper.fingerDrivers) > 0:
            qmin,qmax = self.robot.getJointLimits()
            drivers = [self.robot.driver(i) for i in self.gripper.fingerDrivers]
            dmin,dmax = [],[]
            for d in drivers:
                dlinks = d.getAffectedLinks()
                if d.getType() == 'affine':
                    scale,offset = d.getAffineCoeffs()
                    ddmin = float('inf')
                    ddmax = -ddmin
                    for i,s,o in zip(dlinks,scale,offset):
                        if (qmin[i]-o)/s < ddmin:
                            ddmin = (qmin[i]-o)/s
                        if (qmax[i]-o)/s > ddmax:
                            ddmax = (qmax[i]-o)/s
                    dmin.append(ddmin)
                    dmax.append(ddmax)
                else:
                    dmin.append(qmin[dlinks[0]])
                    dmax.append(qmax[dlinks[0]])
            self.min_template_json['fingerDriverConfig'] = dmin
            self.max_template_json['fingerDriverConfig'] = dmax

    def numDims(self) -> int:
        """Returns the number of dimensions in the parameter space."""
        return len(self.toFeatures(self.template))
    
    def toFeatures(self,grasp : Grasp) -> Vector:
        """Converts a grasp into a feature vector.  ``grasp`` must be 
        compatible with ``template``.
        """
        jsonobj = grasp.toJson()
        #set up special items `transform` and `fingerDriverConfig`
        if self.use_transform:
            T = grasp.ikConstraint.closestMatch(*se3.identity())
            jsonobj['transform'] = so3.rotation_vector(T[0]) + T[1]
        if self.use_drivers:
            self.robot.setConfig(self.gripper.setFingerConfig(self.robot.getConfig(),grasp.fingerConfig))
            qdriver = [self.robot.driver(i).getValue() for i in self.gripper.fingerDrivers]
            jsonobj['fingerDriverConfig'] = qdriver
        return features.extract(jsonobj,self.items)
    
    def toGrasp(self,feature_vec : Vector) -> Grasp:
        """Converts a feature vector to a Grasp.  ``feature_vec`` must
        be of the right length.
        """
        res = copy.deepcopy(self.template_json)
        features.inject(res,self.items,feature_vec)
        g = self.template.__class__.fromJson(res)
        if self.use_transform:
            Tvec = res['transform']
            rv,t = Tvec[:3],Tvec[3:]
            T = so3.from_rotation_vector(rv),t
            g.ikConstraint.matchDestination(*T)
        if self.use_drivers:
            for i,v in zip(self.gripper.fingerDrivers,res['fingerDriverConfig']):
                self.robot.driver(i).setValue(v)
            q = self.robot.getConfig()
            g.fingerConfig = [q[i] for i in self.gripper.fingerLinks]
        return g

    def featureBounds(self) -> Tuple[Vector,Vector]:
        """Returns bounds on the feature space.  Some of these may be
        infinite.
        """
        return (features.extract(self.min_template_json,self.items),
                features.extract(self.max_template_json,self.items))

    def setPositionBounds(self, pmin:Vector, pmax:Vector):
        """Restricts the position bounds to the bounding box (pmin,pmax)."""
        if 'transform' in self.min_template_json:
            self.min_template_json['transform'][3:6] = pmin
            self.max_template_json['transform'][3:6] = pmax
        self.min_template_json['ikConstraint']['endPosition'] = pmin
        self.min_template_json['ikConstraint']['endPosition'] = pmax


