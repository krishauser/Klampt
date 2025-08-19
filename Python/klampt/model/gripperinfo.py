"""Registry of semantic properties for grippers to build universal
(non-gripper specific) code.

Grippers come with a lot of semantic information, such as typical
approach directions, opening widths, synergies, etc. These should be stored in
a :class:`GripperInfo` structure for compatibility with future grasp planning
algorithms.

"""
from __future__ import annotations
from klampt import WorldModel,RobotModel,Geometry3D
from klampt.model.subrobot import SubRobotModel
import copy
from klampt.math import vectorops,so3,se3
import json
import os
from .typing import Vector,Vector3,RigidTransform
from typing import Optional,Union,Sequence,List,Tuple,Dict,Any,TextIO

class GripperInfo:
    """Stores basic information describing a gripper and its mounting on
    a robot.

    For a vacuum-type gripper,

    - center should be set to middle of the vacuum at a "tight" seal.
    - primaryAxis should be set to the outward direction from the vacuum.
    - maximumSpan should be set to the outer diameter of the vacuum seal
    - minimumSpan should be set to the minimum diameter of an object that
      a seal can form around
    - fingerLength should be set to the amount the vacuum should lower from 
      an offset away from the object (the length of the vacuum seal)

    For a parallel-jaw gripper,

    - center should be set to the deepest point within the gripper.
    - primaryAxis should be set to the "down" direction for a top-down grasp
      (away from the wrist, usually). 
    - secondaryAxis should be set to the axis along which the fingers close
      / open (either sign is OK).
    - fingerLength should be set to the distance along primaryAxis from center to
      the tips of the gripper's fingers.
    - fingerWidth should be set to the width of the fingers.
    - fingerDepth should be set to the thickness of the fingers along
      secondaryAxis
    - maximumSpan should be set to the fingers' maximum separation.
    - minimumSpan should be set to the fingers' minimum separation.

    For a multi-finger gripper, these elements are less important, but can
    help with general heuristics.

    - center should be set to a point on the "palm"
    - primaryAxis should be set to a open direction away from the wrist.
    - secondaryAxis should be set to the axis along which the fingers
      close / open in a power grasp (either sign is OK).
    - fingerLength should be set to approximately the length of each finger.
    - fingerWidth should be set to approximately the width of each finger.
    - fingerDepth should be set to approximately the thickness of each finger.
    - maximumSpan should be set to the width of the largest object grippable.
    - minimumSpan should be set to the width of the smallest object grippable.

    Attributes:
        name (str): the gripper name
        baseLink (int or str): the index of the gripper's base
        fingerLinks (list of int): the moving indices of the gripper's fingers.
        fingerDrivers (list of int): the driver indices of the gripper's
            fingers. Can also be a list of list of ints if each finger joint
            can be individually actuated.
        type (str, optional): Specifies the type of gripper. Can be 'vacuum',
            'parallel', 'wrapping', or None (unknown)
        center (list of 3 floats, optional): A central point on the "palm" of
            the gripper.
        primaryAxis (list of 3 floats, optional): The local axis of the
            gripper that opposes the "typical" load.  (Unit vector in the
            opposite direction of the load)
        secondaryAxis (list of 3 floats, optional): The local axis of the
            gripper perpendicular to the primary that dictates the direction
            of the fingers opening and closing
        fingerLength,fingerWidth,fingerDepth (float, optional): dimensions
            of the fingers.
        maximumSpan (float, optional): the maximum opening span of the gripper.
        minimumSpan (float, optional): the minimum opening span of the gripper.
        closedConfig (list of floats, optional): the "logical closed" gripper
            finger config.
        openConfig (list of floats, optional): the "logical open" gripper
            finger config.
        synergies (dict str -> list of Vectors): for multifingered hands, a set
            of "synergies" that are able to generate a full gripper
            configuration.  If a synergy is a list of k Vectors, the synergy
            accepts k-1 parameters ``p`` in the range [0,1]^(k-1) and outputs 
            an affine combination A[0]*p[0] + ... + A[k-2]*p[k-2] + A[k-1].
        gripperLinks (list of int, optional): the list of all links attached 
            to the gripper, including non-actuated ones.  If not provided,
            assumed to be the base link plus all descendant links on the
            klamptModel.
        klamptModel (str, optional): the Klamp't .rob or .urdf model to which
            this gripper is attached.  Note: this is usually not a model of
            just the gripper.  To get just the gripper, use ``self.getSubrobot()``.
    """

    all_grippers = dict()

    @staticmethod
    def register(gripper : 'GripperInfo'):
        GripperInfo.all_grippers[gripper.name] = gripper

    @staticmethod
    def get(name) -> 'GripperInfo':
        return GripperInfo.all_grippers.get(name,None)
    
    @staticmethod
    def load(fn: str) -> 'GripperInfo':
        """Loads / registers a GripperInfo from a JSON file previously saved to disk."""
        path,basename = os.path.split(fn)
        res = GripperInfo(os.path.splitext(basename)[0],-1)
        with open(fn,'r') as f:
            jsonobj = json.load(f)
            if jsonobj.get('klamptModel',None) is not None:
                if not jsonobj['klamptModel'].startswith('/'):
                    jsonobj['klamptModel'] = os.path.normpath(os.path.join(path,jsonobj['klamptModel']))
            res.fromJson(jsonobj)
        GripperInfo.register(res)
        return res

    @staticmethod
    def mounted(gripper : 'GripperInfo', klamptModel : str, baseLink : Union[int,str],
        name : str = None, register=True):
        """From a standalone gripper, return a GripperInfo such that the link
        indices are shifted onto a new robot model.
        
        klamptModel should contain the arm as well as the gripper links, and
        baseLink should be the name or index of the gripper base on the model.
        """
        if name is None:
            name = gripper.name + "_mounted"
        w = WorldModel()
        w.enableGeometryLoading(False)
        res = w.readFile(klamptModel)
        if not res:
            raise IOError("Unable to load file "+str(klamptModel))
        robot = w.robot(0)
        w.enableGeometryLoading(True)
        if isinstance(baseLink,str):
            baseLink = robot.link(baseLink).index
        shifted_fingerLinks = [l+baseLink for l in gripper.fingerLinks]
        mount_driver = -1
        for i in range(robot.numDrivers()):
            ls = robot.driver(i).getAffectedLinks()
            if any(l in ls for l in shifted_fingerLinks):
                mount_driver = i
                break
        if mount_driver < 0:
            raise RuntimeError("Can't find the base driver for the mounted gripper?")
        shifted_fingerDrivers = [l+mount_driver for l in gripper.fingerDrivers]
        res = copy.copy(gripper)
        res.name = name
        res.baseLink = baseLink
        res.klamptModel = klamptModel
        res.fingerLinks = shifted_fingerLinks
        res.fingerDrivers = shifted_fingerDrivers
        if register:
            GripperInfo.register(res)
        return res
        
    def __init__(self, name : str, baseLink : Union[int,str],
                fingerLinks : List[int]=None, fingerDrivers : List[int]=None,
                type : str=None, center : Vector3=None, primaryAxis : Vector3=None, secondaryAxis : Vector3=None,
                fingerLength : float=None, fingerWidth : float=None, fingerDepth : float=None,
                maximumSpan : float=None, minimumSpan : float=None,
                closedConfig : Vector=None, openConfig : Vector=None,
                gripperLinks : List[int]=None,
                klamptModel : str=None,
                register=True):
        self.name = name
        self.baseLink = baseLink
        self.fingerLinks = fingerLinks if fingerLinks is not None else []
        self.fingerDrivers = fingerDrivers if fingerDrivers is not None else []
        self.type=type
        self.center = center
        self.primaryAxis = primaryAxis
        self.secondaryAxis = secondaryAxis
        self.fingerLength = fingerLength
        self.fingerWidth = fingerWidth
        self.fingerDepth = fingerDepth
        self.maximumSpan = maximumSpan
        self.minimumSpan = minimumSpan
        self.closedConfig = closedConfig 
        self.openConfig = openConfig
        self.gripperLinks = gripperLinks
        self.synergies = dict()                   # type: Dict[str,Sequence[Vector]]
        self.klamptModel = klamptModel
        if register:
            GripperInfo.register(self)

    def partwayOpenConfig(self, amount : float) -> Vector:
        """Returns a finger configuration partway open, with amount in the
        range [0 (closed),1 (fully open)].
        """
        if self.closedConfig is None or self.openConfig is None:
            raise ValueError("Can't get an opening configuration on a robot that does not define it")
        return vectorops.interpolate(self.closedConfig,self.openConfig,amount)
    
    def configToOpening(self, qfinger : Vector) -> float:
        """Estimates how far qfinger is from closedConfig to openConfig.
        Only meaningful if qfinger is close to being along the straight
        C-space line between the two.
        """
        if self.closedConfig is None or self.openConfig is None:
            raise ValueError("Can't estimate opening width to")
        assert len(qfinger) == len(self.closedConfig)
        b = vectorops.sub(qfinger,self.closedConfig)
        a = vectorops.sub(self.openConfig,self.closedConfig)
        return min(1,max(0,vectorops.dot(a,b)/vectorops.normSquared(a)))

    def estimatedContactCentroid(self) -> Vector3:
        """Estimates the centroid of points of contact are normally made with
        an object. This is just a heuristic, which works best with vacuum and
        parallel-jaw grippers.
        """
        if self.center is None:
            raise ValueError("Gripper does not have a center defined")
        if self.type == 'parallel':
            if self.primaryAxis is not None:
                return vectorops.madd(self.center,self.primaryAxis,self.fingerLength-self.fingerWidth*0.5)
        elif self.primaryAxis is not None:
            return vectorops.madd(self.center, self.primaryAxis * 2.0/3.0)  #assume fingers partway closed
        return self.center

    def evalSynergy(self, synergy_name : str, parameters : Vector) -> Vector:
        """Maps from a set of parameters to a full gripper configuration
        according to the given synergy."""
        synergy = self.synergies[synergy_name]
        if callable(synergy):
            return synergy(parameters)
        if len(parameters)+1 != len(synergy):
            raise ValueError("Synergy {} accepts {} parameters".format(synergy_name,len(synergy)-1))
        res = synergy[-1]
        for i,p in enumerate(parameters):
            res = vectorops.madd(res,synergy[i],p)
        return res
    
    def configToSynergy(self, qfinger : Vector, synergy_name : str = None) -> Tuple[str,Vector]:
        """Maps a finger config to the closest synergy parameters.  If
        synergy_name is given, then this restricts the search to a single
        synergy.
        
        Only works for affine synergies.
        """
        if synergy_name is None:
            sclosest = None
            pclosest = None
            dclosest = float('inf')
            for name in self.synergies:
                try:
                    (_,p) = self.configToSynergy(qfinger, name)
                    q = self.evalSynergy(name,p)
                    d = vectorops.distanceSquared(qfinger,q)
                    if d < dclosest:
                        sclosest,pclosest = name,p
                except Exception:
                    pass
            return (sclosest,pclosest)
        synergy = self.synergies[synergy_name]
        if callable(synergy):
            raise ValueError("Can't map to synergy for a callable")
        import numpy as np
        b = np.asarray(qfinger) - synergy[-1]
        A = np.asarray(synergy[:-1]).T
        return (synergy_name,np.linalg.lstsq(A,b)[0].tolist())

    def widthToOpening(self, width : float) -> float:
        """Returns an opening amount in the range 0 (closed) to 1 (open)
        such that the fingers have a given width between them.
        """
        if self.maximumSpan is None:
            raise ValueError("Can't convert from width to opening without maximumSpan")
        minspan = self.minimumSpan if self.minimumSpan is not None else 0
        return (width-minspan)/(self.maximumSpan-minspan)
    
    def openingToWidth(self, opening : float) -> float:
        """For a given opening amount in the range 0 (closed) to 1 (open)
        returns an approximation to the width between the fingers.
        """
        if self.maximumSpan is None:
            raise ValueError("Can't convert from width to opening without maximumSpan")
        minspan = self.minimumSpan if self.minimumSpan is not None else 0
        return minspan + opening*(self.maximumSpan-minspan)

    def setFingerConfig(self, qrobot : Vector, qfinger : Vector) -> Vector:
        """Given a full robot config qrobot, returns a config but with the finger
        degrees of freedom fixed to qfinger.
        """
        assert len(qfinger) == len(self.fingerLinks)
        qf = [v for v in qrobot]
        for (i,v) in zip(self.fingerLinks,qfinger):
            qf[i] = v
        return qf

    def getFingerConfig(self, qrobot : Vector) -> Vector:
        """Given a full robot config qrobot, returns a finger config."""
        return [qrobot[i] for i in self.fingerLinks]

    def descendantLinks(self, robot : RobotModel) -> List[int]:
        """Returns all links under the base link.  This may be different
        from fingerLinks if there are some frozen DOFs and you prefer
        to treat a finger configuration as only those DOFS for the active
        links.
        """
        descendants = [False]*robot.numLinks()
        baseLink = robot.link(self.baseLink).index
        descendants[baseLink] = True
        for i in range(robot.numLinks()):
            if descendants[robot.link(i).getParent()]:
                descendants[i] = True
        return [i for (i,d) in enumerate(descendants) if d]

    def eachFingerLinks(self, robot : RobotModel) -> List[List[int]]:
        """Returns a list of lists, where each sublist contains the
        indices of the links that are part of each finger.

        E.g., if the gripper is a parallel gripper, this will return a
        list of two lists of links, one for each finger.
        """
        if self.fingerLinks is None or len(self.fingerLinks) == 0:
            return []
        #determine topology of fingerLinks
        fingers = []
        for i in self.fingerLinks:
            p = robot.link(i).getParent()
            finger_idx = -1
            for idx,finger in enumerate(fingers):
                if p in finger:
                    finger_idx = idx
                    break
            if finger_idx == -1:
                #new finger
                fingers.append([i])
            else:
                fingers[finger_idx].append(i)
        return fingers

    def getSubrobot(self, robot : RobotModel, all_descendants=True) -> SubRobotModel: 
        """Returns the SubRobotModel of the gripper given a RobotModel.

        If some of the links belonging to the gripper are frozen and not
        part of the DOFs (i.e., part of fingerLinks), then they will 
        be included in the SubRobotModel if all_descendants=True.  This
        means there may be a discrepancy between the finger configuration
        and the sub-robot configuration.

        Otherwise, they will be excluded and finger configurations will
        map one-to-one to the sub-robot.
        """
        baseLink = robot.link(self.baseLink).index
        if all_descendants:
            return SubRobotModel(robot,[baseLink] + self.descendantLinks(robot))
        return SubRobotModel(robot,[baseLink]+list(self.fingerLinks))

    def getGeometry(self, robot : RobotModel, qfinger:Optional[Vector]=None,type='Group') -> Geometry3D:
        """Returns a Geometry of the gripper frozen at its configuration.
        If qfinger = None, the current configuration is used.  Otherwise,
        qfinger is a finger configuration.
        
        type can be 'Group' (most general and fastest) or 'TriangleMesh'
        (compatible with Jupyter notebook visualizer.)
        """
        if qfinger is not None:
            q0 = robot.getConfig()
            robot.setConfig(self.setFingerConfig(q0,qfinger))
        res = Geometry3D()
        baseLink = robot.link(self.baseLink).index
        gripperLinks = self.gripperLinks if self.gripperLinks is not None else [baseLink] + self.descendantLinks(robot)
        if type == 'Group':
            res.setGroup()
            Tbase = robot.link(self.baseLink).getTransform()
            for i,link in enumerate(gripperLinks):
                Trel = se3.mul(se3.inv(Tbase),robot.link(link).getTransform())
                g = robot.link(link).geometry().copy()
                if not g.empty():
                    g.setCurrentTransform(*se3.identity())
                    g.transform(*Trel)
                else:
                    print("Uh... link",robot.link(link).getName(),"has empty geometry?")
                res.setElement(i,g)
            if qfinger is not None:
                robot.setConfig(q0)
            return res
        else:
            from . import geometry
            res = geometry.merge(*[robot.link(link) for link in gripperLinks])
            if qfinger is not None:
                robot.setConfig(q0)
            return res
    
    def fromJson(self,jsonobj) -> None:
        """Loads the info from a JSON object."""
        REQUIRED = ['name','baseLink']
        OPTIONAL = ['fingerLinks','fingerDrivers','type','center','primaryAxis','secondaryAxis','fingerLength','fingerWidth','fingerDepth',
            'maximumSpan','minimumSpan','closedConfig','openConfig','synergies','gripperLinks','klamptModel']
        for attr in REQUIRED+OPTIONAL:
            if attr not in jsonobj:
                if attr in OPTIONAL: 
                    continue
                else:
                    raise IOError("Loaded JSON object doesn't contain '"+attr+"' key")
            setattr(self,attr,jsonobj[attr])

    def toJson(self) -> dict:
        """Saves the info to a JSON file. f is a file object."""
        jsonobj = dict()
        REQUIRED = ['name','baseLink']
        OPTIONAL = ['fingerLinks','fingerDrivers','type','center','primaryAxis','secondaryAxis','fingerLength','fingerWidth','fingerDepth',
            'maximumSpan','minimumSpan','closedConfig','openConfig','synergies','gripperLinks','klamptModel']
        for attr in REQUIRED+OPTIONAL:
            jsonobj[attr] = getattr(self,attr)
        for (k,val) in self.synergies.items():
            if callable(val):
                raise RuntimeError("Can't convert callable synergy to JSON... need to find alternative methods")
        return jsonobj
    
    def save(self, f : Union[str,TextIO]) -> None:
        """Saves to disk in JSON format."""
        if isinstance(f,str):
            with open(f,'w') as file:
                self.save(file)
        else:
            jsonobj = self.toJson()
            json.dump(jsonobj,f)

    def visualize(self) -> None:
        """Visually debugs the gripper"""
        from klampt import vis
        vis.loop(lambda: self.addToVis())

    def addToVis(self,
                 robot : RobotModel=None,
                 animate : bool=True,
                 base_xform : Optional[RigidTransform]=None,
                 prefix : Optional[str] = None,
                 hide_label: bool = True) -> None:
        """Adds the gripper to the klampt.vis scene.
        
        Args:
            robot (RobotModel, optional): if given, this gripper is a sub-robot
                of the given robot.
            animate (bool): if True, animates the opening and closing of the
                gripper.
            base_xform (se3 element, optional): if given and robot=False, poses 
                the gripper base at this transform.
            prefix (str, optional): if given, this is the prefix for the
                visualization names.  If None, defaults to "gripper_" + self.name.
            hide_label (bool): if True, the visualization objects' labels will
                not be shown.
        """
        from klampt import vis
        from klampt import WorldModel,Geometry3D,GeometricPrimitive
        from klampt.model.trajectory import Trajectory
        if prefix is None:
            prefix = "gripper_"+self.name
        if robot is None and self.klamptModel is not None:
            w = WorldModel()
            if w.readFile(self.klamptModel):
                robot = w.robot(0)
                vis.add(prefix+"_gripper",w,hide_label=hide_label)
                robotPath = (prefix+"_gripper",robot.getName())
        elif robot is not None:
            vis.add(prefix+"_gripper",robot,hide_label=hide_label)
            robotPath = prefix+"_gripper"
        if robot is not None:
            baseLink = robot.link(self.baseLink)
            assert baseLink.index >= 0 and baseLink.index < robot.numLinks()
            baseLink.appearance().setColor(1,0.75,0.5)
            if base_xform is None:
                base_xform = baseLink.getTransform()
            else:
                if baseLink.getParent() >= 0:
                    #print("GripperInfo.addToVis(): Warning, setting base link transform for an attached gripper base")
                    #robot.link(self.baseLink).setParent(-1)
                    parent_xform = baseLink.getParentLink().getTransform()
                    base_rel_xform = se3.mul(se3.inv(parent_xform),base_xform)
                    baseLink.setParentTransform(*base_rel_xform)
                else:
                    baseLink.setParentTransform(*base_xform)
                robot.setConfig(robot.getConfig())  #update forward kinematics
            for l in self.fingerLinks:
                assert l >= 0 and l < robot.numLinks()
                robot.link(l).appearance().setColor(1,1,0.5)
        else:
            if base_xform is None:
                base_xform = se3.identity()
        if robot is not None and animate:
            q0 = robot.getConfig()
            for i in self.fingerDrivers:
                if isinstance(i,(list,tuple)):
                    for j in i:
                        robot.driver(j).setValue(1)
                else:
                    robot.driver(i).setValue(1)
            traj = Trajectory([0],[robot.getConfig()])
            for i in self.fingerDrivers:
                if isinstance(i,(list,tuple)):
                    for j in i:
                        robot.driver(j).setValue(0)
                        traj.times.append(traj.endTime()+0.5)
                        traj.milestones.append(robot.getConfig())
                else:
                    robot.driver(i).setValue(0)
                    traj.times.append(traj.endTime()+1)
                    traj.milestones.append(robot.getConfig())
            traj.times.append(traj.endTime()+1)
            traj.milestones.append(traj.milestones[0])
            traj.times.append(traj.endTime()+1)
            traj.milestones.append(traj.milestones[0])
            assert len(traj.times) == len(traj.milestones)
            traj.checkValid()
            vis.animate(robotPath,traj)
            robot.setConfig(q0)
        if self.center is not None:
            vis.add(prefix+"_center",se3.apply(base_xform,self.center),hide_label=hide_label)
        center_point = (0,0,0) if self.center is None else self.center
        outer_point = (0,0,0)
        if self.primaryAxis is not None:
            length = 0.1 if self.fingerLength is None else self.fingerLength
            outer_point = vectorops.madd(self.center,self.primaryAxis,length)
            line = Trajectory([0,1],[self.center,outer_point])
            line.milestones = [se3.apply(base_xform,m) for m in line.milestones]
            vis.add(prefix+"_primary",line,color=(1,0,0,1),hide_label=hide_label)
        if self.secondaryAxis is not None:
            width = 0.1 if self.maximumSpan is None else self.maximumSpan
            line = Trajectory([0,1],[vectorops.madd(outer_point,self.secondaryAxis,-0.5*width),vectorops.madd(outer_point,self.secondaryAxis,0.5*width)])
            line.milestones = [se3.apply(base_xform,m) for m in line.milestones]
            vis.add(prefix+"_secondary",line,color=(0,1,0,1),hide_label=hide_label)
        elif self.maximumSpan is not None:
            #assume vacuum gripper?
            p = GeometricPrimitive()
            p.setSphere(outer_point,self.maximumSpan)
            g = Geometry3D()
            g.setGeometricPrimitive(p)
            if base_xform is not None:
                g.setCurrentTransform(*base_xform)
            vis.add(prefix+"_opening",g,color=(0,1,0,0.25),hide_label=hide_label)
        #TODO: add finger box

    def removeFromVis(self, prefix: Optional[str] = None) -> None:
        """Removes a previously-added gripper from the klampt.vis scene."""
        if prefix is None:
            prefix = "gripper_"+self.name
        from klampt import vis
        try:
            vis.remove(prefix+"_gripper")
        except Exception:
            pass
        if self.center is not None:
            vis.remove(prefix+"_center")
        if self.primaryAxis is not None:
            vis.remove(prefix+"_primary")
        if self.secondaryAxis is not None:
            vis.remove(prefix+"_secondary")
        elif self.maximumSpan is not None:
            vis.remove(prefix+"_opening")
        
