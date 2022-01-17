"""Registry of semantic properties for robots to help build generalized
(non-robot-specific) code. 

A robot typically consists of a set of parts, a base (either fixed or
moving), end effectors, a controller, and other assorted properties.  The
:class:`RobotInfo` data structure will let you define these and store them
in a human-editable file format.  This is also used for ``klampt_control`` and
some Robot Interface Layer controllers (see
:class:`klampt.control.robotinterfaceutils.OmniRobotInterface`).

Grippers also come with a lot of semantic information, such as typical
approach directions, opening widths, synergies, etc. These should be stored in
a :class:`GripperInfo` structure.  (So far, Klamp't doesn't have built-in
integration with grasp planning algorithms, but we hope to add more in the
future.)

"""
from klampt.control.robotinterface import RobotInterfaceBase
import os
import sys
import importlib
from klampt import WorldModel,RobotModel,Geometry3D,IKSolver,IKObjective,SimRobotSensor
from klampt.model.subrobot import SubRobotModel
import copy
from klampt.math import vectorops,se3
import json
import warnings
from .typing import Vector,Vector3,RigidTransform
from typing import Optional,Union,Sequence,List,Tuple,Dict,Any,TextIO

class RobotInfo:
    """Stores common semantic properties for a robot.

    Attributes:
        name (str): a string identifying this robot by name.
        modelFile (str, optional): a file pointing to the Klamp't model (.urdf 
            or .rob.)
        baseType (str): specifies whether the robot has a mobile or floating
            base. Possible values are:

            - 'fixed': rigidly attached
            - 'floating': floating in space (e.g., drone, legged robot)
            - 'differential': differential drive base
              (properties['differential'] should hold more info)
            - 'dubins': Dubins car base (properties['dubins'] should hold
              more info)

        parts (dict str->list): a set of named parts.  Each part consists
            of a list of link names or indices.
        endEffectors (dict str->EndEffectorInfo): a set of named end
            effectors.
        grippers (dict str->GripperInfo): a set of named grippers. 
        properties (dict str->anything): a dict of named properties. Values
            must be JSON-loadable/savable.
        controllerFile (str, optional): the file containing code for the Klampt
            RIL controller. 

            Should be a Python file (``/path/to/file.py``) or module
            (`package.module`) containing a function ``make(robotModel)`` that
            returns a :class:`RobotInterfaceBase`.
        controllerArgs (list, optional): extra arguments that will be passed to
            the make() function.
        simulatorFile (str, optional): the file containing code for simulation
            emulators for the robot.  If not provided, the default robot
            simulator is used.

            Should be a Python file (``/path/to/file.py``) or module
            (`package.module`) containing a function
            ``make(sim,robotIndex)`` which returns a pair
            ``(controller, emulators)``.

            Here, ``controller`` is a :class:`RobotInterfaceBase` configured
            for the simulator (see :mod:`klampt.control.simrobotinterface`).
            ``emulators`` is a list of :class:`SensorEmulator` or
            :class:`ActuatorEmulator` objects configured for use in a
            :class:`SimpleSimulator`.
        calibrationFiles (dict str->str): a set of calibration files that will
            be loaded and imposed on the klamptModel when it is loaded.  Using
            separate calibration files is more efficient than overwriting a
            .rob or .urdf file when the calibration changes.

            Valid keys include:

            - [sensor name]: a JSON file giving any of the named sensor's
              settings (typically, the transform T)
            - kinematics: a JSON file giving link parent transforms and
              joint axes. The file should have format:
              ``{"link1":{"axis":VALUE,"Tparent":VALUE} ...}``
            
        resourceDir (str, optional): the directory containing resources for
            this robot.
        filePaths (list of str): a list of paths where files will be searched.
        robotModel (RobotModel): a cached robot model, loaded upon calling
            :func:`klamptModel`.  If a robot is loaded externally (e.g., via
            :class:`~klampt.WorldModel`) you may set this member to avoid re-
            loading a model.
    """

    all_robots = dict()

    @staticmethod
    def register(robotInfo: 'RobotInfo'):
        """Registers a RobotInfo to the global registry."""
        RobotInfo.all_robots[robotInfo.name] = robotInfo

    @staticmethod
    def get(name: str) -> 'RobotInfo':
        """Retrieves a registered RobotInfo from the global registry."""
        return RobotInfo.all_robots[name]

    @staticmethod
    def load(fn: str) -> 'RobotInfo':
        """Loads / registers a RobotInfo from a JSON file previously saved to disk."""
        res = RobotInfo(None)
        res.load(fn)
        if res.modelFile is not None:  #make sure the model file is an absolute path
            if not res.modelFile.startswith('/') and not res.modelFile.startswith('http://')  and not res.modelFile.startswith('https://'):
                res.modelFile = os.path.abspath(os.path.join(os.path.split(fn)[0],res.modelFile))
        path,file = os.path.split(fn)
        if path:
            res.filePaths.append(path)
        RobotInfo.register(res)
        return res

    def __init__(self,name,modelFile=None,
                baseType='fixed',
                parts=None,endEffectors=None,grippers=None,
                properties=None):
        self.name = name                # type: str
        self.modelFile = modelFile      # type: str
        self.baseType = baseType        # type: str
        self.controllerFile = None      # type: Optional[str]
        self.controllerArgs = None      # type: Optional[list]
        self.simulatorFile = None       # type: Optional[str]
        self.calibrationFiles = dict()  # type: Dict[str,str]
        self.resourceDir = None         # type: Optional[str]
        if parts is None:
            parts = dict()
        else:
            if not isinstance(parts,dict):
                raise ValueError("`parts` must be a dict from names to link lists")
            for (k,v) in parts:
                if not hasattr(v,'__iter__'):
                    raise ValueError("`parts` must be a dict from names to link lists")
        if endEffectors is None:
            endEffectors = dict()
        else:
            if not isinstance(endEffectors,dict):
                raise ValueError("`endEffectors` must be a dict from names to EndEffectorInfos")
        if grippers is None:
            grippers = dict()
        else:
            if not isinstance(grippers,dict):
                raise ValueError("`grippers` must be a dict from names to GripperInfos")
        if properties is None:
            properties = dict()
        else:
            if not isinstance(properties,dict):
                raise ValueError("`properties` must be a dict")
        self.parts = parts                # type: Dict[str,Sequence[Union[int,str]]]
        self.endEffectors = endEffectors  # type: Dict[str,EndEffectorInfo]
        self.grippers = grippers          # type: Dict[str,GripperInfo]
        self.properties = properties      # type: Dict[str,Any]
        self.filePaths = []               # type: List[str]
        self.robotModel = None            # type: RobotModel
        self.load = self._instance_load
        self._driverIndices = None
        self._worldTemp = None

    def klamptModel(self) -> RobotModel:
        """Returns the Klamp't RobotModel associated with this robot, either by
        the ``robotModel`` object or loading from the given filename
        ``self.modelFile``.

        The results are cached so this can be called many times without a
        performance hit.
        """
        if self.robotModel is not None:
            return self.robotModel
        if self.modelFile is None:
            raise RuntimeError("Can't load robot model for {}, no file given".format(self.name))
        self._worldTemp = WorldModel()
        def doload(fn):
            self.robotModel = self._worldTemp.loadRobot(fn)
            return self.robotModel.index >= 0
        if not self._tryload(self.modelFile,doload):
            raise IOError("Unable to load robot from file {}".format(self.modelFile))
        self.robotModel.setName(self.name)
        #apply calibration
        for (k,file) in self.calibrationFiles.items():
            if k == 'kinematics':
                def docalib(fn):
                    try:
                        with open(fn,'r') as f:
                            jsonobj = json.load(f)
                    except IOError:
                        return False
                    for k,items in jsonobj.items():
                        link = self.robotModel.link(k) 
                        if link.index < 0:
                            raise ValueError("Calibration file refers to invalid link {}".format(k))
                        for key,value in items.items():
                            if key == 'axis':
                                link.setAxis(value)
                            elif key == 'Tparent':
                                link.setParentTransform(value)
                            else:
                                raise KeyError("Invalid calibration item {}".format(key))
                    return True
                if not self._tryload(file,docalib):
                    raise IOError("Unable to load kinematics calibration from file "+file)
            else:
                s = self.robotModel.sensor(k)
                if s.getName():
                    self.configureSensor(s)
                else:
                    warnings.warn("Calibration item {} doesn't refer to a sensor or kinematics".format(k))
        return self.robotModel
    
    def configureSensor(self,sensor : Union[int,str,SimRobotSensor]) -> SimRobotSensor:
        """Configures a sensor with a calibration, if present."""
        if not isinstance(sensor,SimRobotSensor):
            return self.configureSensor(self.klamptModel().sensor(sensor))
        if sensor.name() in self.calibrationFiles:
            calib = self.calibrationFiles[sensor.name()]
            for k,value in calib:
                if hasattr(value,'__iter__'):
                    import klampt.io.loader
                    cast_value = klampt.io.loader.write(value,'auto')
                else:
                    cast_value = str(value)
                sensor.setSetting(k,cast_value)
        return sensor

    def controller(self) -> RobotInterfaceBase:
        """Returns a Robot Interface Layer object configured for use on the
        real robot.  Requires ``controllerFile`` to be defined.
        """
        if self.controllerFile is None:
            raise RuntimeError("Can't create controller for "+self.name+", no file given")
        mod = _dynamic_load_module(self.controllerFile,self.filePaths)
        try:
            maker = mod.make
        except AttributeError:
            raise RuntimeError("Module {} must have a make(robotModel) method".format(mod.__name__))
        if self.controllerArgs is not None:
            args = self.controllerArgs
        else:
            args = ()
        try:
            res = mod.make(self.klamptModel(),*args)
        except Exception as e:
            raise RuntimeError("Error running make(robotModel) for module {}: {}".format(mod.__name__,str(e)))
        res.properties['klamptModelFile'] = self.modelFile
        return res
        
    def configureSimulator(self,sim,robotIndex=0):
        """Configures a :class:`~klampt.sim.simulation.SimpleSimulator` to
        emulate the robot's behavior.

        Args:
            sim (SimpleSimulator): the simulator to configure.  It should have
                a world model set up to already have a matching robot's model.
            robotIndex (int or str): the index of the robot model in the sim's
                world.
        """
        if self.simulatorFile is None:
            return
        robot = sim.world.robot(robotIndex)
        robotModel = self.klamptModel()
        if robotModel.numLinks() != robot.numLinks():
            raise ValueError("The robot in the simulator doesn't match this robot")
        mod = _dynamic_load_module(self.simulatorFile,self.filePaths)
        try:
            maker = mod.make
        except AttributeError:
            raise RuntimeError("Module {} must have a make(sim,robotIndex) method".format(mod.__name__))
        try:
            res = mod.make(sim,robotIndex)
        except Exception as e:
            raise RuntimeError("Error running make(sim,robotIndex) for module {}: {}"%(mod.__name__,str(e)))
        try:
            controller,emulators = res
            for e in emulators:
                pass
        except Exception:
            raise RuntimeError("Result of make(sim,robotIndex) for module {} is not a pair (controller,emulators)"%(mod.__name__,))
        sim.setController(robotIndex,controller)
        for e in emulators:
            sim.addEmulator(robotIndex,e)

        #configure simulated sensor
        simcontroller = sim.controller(robot)
        sindex = 0
        while True:
            s = simcontroller.sensor(sindex)
            if s.type() == '':
                break
            sindex += 1
            self.configureSensor(s)
        

    def partLinks(self, part: str) -> List[Union[int,str]]:
        return self.parts[part]

    def partLinkIndices(self, part: str) -> List[int]:
        res = self.parts[part]
        return self.toIndices(res)

    def partLinkNames(self, part: str) -> List[str]:
        res = self.parts[part]
        return self.toNames(res)
    
    def partDriverIndices(self, part: str) -> List[int]:
        res = self.parts[part]
        return self.toDriverIndices(res)

    def partAsSubrobot(self, part: str) -> SubRobotModel:
        partLinks = self.partLinkIndices(part)
        model = self.klamptModel()
        return SubRobotModel(model,partLinks)

    def eeSolver(self, endEffector: str, target: Union[Vector,RigidTransform]) -> IKSolver:
        """Given a named end effector and a target point, transform, or set of
        parameters from config.setConfig(ikgoal) / config.getConfig(ikgoal),
        returns the IKSolver for the end effector and  that target.
        """
        ee = self.endEffectors[endEffector]
        from klampt import IKObjective
        from ..model import config,ik
        robot = self.klamptModel()
        link = robot.link(ee.link)
        s = IKSolver(robot)
        q = config.getConfig(target)
        obj = ee.ikObjective
        if ee.ikObjective is None:
            if len(q) == 3:
                obj = ik.objective(link,local=[0,0,0],world=q)
            elif len(q) == 12:
                obj = ik.objective(link,R=q[:9],t=q[9:])
            else:
                raise ValueError("Invalid # of elements given in target, must be either a 3-vector or SE3 element")
        s.add(obj)
        s.setActiveDofs(self.toIndices(ee.activeLinks))
        return s

    def toIndices(self, items: Sequence[Union[int,str]]) -> List[int]:
        """Returns link identifiers as link indices"""
        if all(isinstance(v,int) for v in items):
            return items
        else:
            robot = self.klamptModel()
            res = [i for i in items]
            for i,v in enumerate(items):
                if not isinstance(v,int):
                    assert isinstance(v,str),"Link identifiers must be int or str"
                    res[i] = robot.link(v).getIndex()
                    assert res[i] >= 0,"Link %s doesn't exist in robot %s"%(v,self.name)
            return res
    
    def toDriverIndices(self, items: Sequence[Union[int,str]]) -> List[int]:
        """Converts link names or indices to robot driver indices"""
        robot = self.klamptModel()
        if self._driverIndices is None:
            self._driverIndices = dict()
            for i in range(robot.numDrivers()):
                d = robot.driver(i)
                self._driverIndices[d.getAffectedLink()] = i
        res = []
        for i,v in enumerate(items):
            if not isinstance(v,int):
                assert isinstance(v,str),"Link identifiers must be int or str"
                v= robot.link(v).getIndex()
                assert v >= 0,"Link %s doesn't exist in robot %s"%(v,self.name)
            if v in self._driverIndices:
                res.append(self._driverIndices[v])
        return res

    def toNames(self, items: Sequence[Union[int,str]]) -> List[str]:
        """Returns link identifiers as link names"""
        if all(isinstance(v,str) for v in items):
            return items
        else:
            robot = self.klamptModel()
            res = [i for i in items]
            for i,v in enumerate(items):
                if not isinstance(v,str):
                    assert isinstance(v,int),"Link identifiers must be int or str"
                    assert v >= 0 and v < robot.numLinks(),"Link %d is invalid for robot %s"%(v,self.name)
                    res[i] = robot.link(v).getName()
            return res

    def listResources(self) -> List[str]:
        """Retrieves a list of all named resources, if resourceDir is set"""
        if self.resourceDir is None:
            return []
        import os
        resourceDir = _resolve_file(self.resourceDir,self.filePaths)
        return [item for item in os.listdir(resourceDir)]

    def getResource(self, name : str, doedit='auto'):
        """Loads a named resource."""
        from klampt.io import resource
        resourceDir = _resolve_file(self.resourceDir,self.filePaths)
        return resource.get(name, directory=resourceDir, doedit=doedit, world=self._worldTemp)
    
    def setResource(self, name :str, object) -> None:
        """Saves a new named resource.  object can be any supported Klampt type."""
        from klampt.io import resource
        resourceDir = _resolve_file(self.resourceDir,self.filePaths)
        resource.set(name, object, directory=resourceDir)

    def _instance_load(self, f : Union[str,TextIO]) -> None:
        """Loads the info from a JSON file. f is a file name or file object."""
        from ..io import loader
        if isinstance(f,str):
            with open(f,'r') as file:
                jsonobj = json.load(file)
        else:
            jsonobj = json.load(f)
        self.endEffectors = dict()
        self.grippers = dict()
        REQUIRED = ['name','modelFile']
        OPTIONAL = ['parts','baseType','endEffectors','grippers','properties','controllerFile','controllerArgs','simulatorFile','calibrationFiles','resourceDir','filePaths']
        for attr in REQUIRED+OPTIONAL:
            if attr not in jsonobj:
                if attr in OPTIONAL: #optional
                    continue
                else:
                    raise IOError("Loaded JSON object doesn't contain '"+attr+"' key")
            setattr(self,attr,jsonobj[attr])
        ees = dict()
        for (k,v) in self.endEffectors.items():
            obj = None if v['ikObjective'] is None else loader.fromJson(v['ikObjective'],'IKObjective')
            ees[k] = EndEffectorInfo(v['link'],v['activeLinks'],obj)
        self.endEffectors = ees
        grippers = dict()
        for (k,v) in self.grippers.items():
            gripper = GripperInfo('',0)
            gripper.fromJson(v)
            grippers[k] = gripper
        self.grippers = grippers

    def save(self, f : Union[str,TextIO]) -> None:
        """Saves the info to a JSON file. f is a file object."""
        from ..io import loader
        jsonobj = dict()
        for attr in ['name','modelFile','parts','baseType','properties','controllerFile','controllerArgs','simulatorFile','calibrationFiles','resourceDir','filePaths']:
            jsonobj[attr] = getattr(self,attr)
        ees = dict()
        for k,v in self.endEffectors.items():
            obj = None if v.ikObjective is None else loader.toJson(v.ikObjective)
            gripper = None if v.gripperInfo is None else v.gripperInfo.toJson()
            ees[k] = {'link':v.link,'activeLinks':v.activeLinks,'ikObjective':obj,'gripperInfo':gripper}
        if len(ees) > 0:
            jsonobj['endEffectors'] = ees
        grippers = dict()
        for k,v in self.grippers.items():
            grippers[k] = v.toJson()
        if len(grippers) > 0:
            jsonobj['grippers'] = grippers
        if isinstance(f,str):
            with open(f,'w') as file:
                json.dump(jsonobj,file)
        else:
            json.dump(jsonobj,f)

    def _tryload(self,fn,loadcallback):
        if loadcallback(fn):
            return True
        if not fn.startswith('/'):
            for path in self.filePaths:
                if loadcallback(fn):
                    return True
        return False


class EndEffectorInfo:
    """Stores info about default end effectors and the method for Cartesian
    solving.

    Attributes:
        link (int or str): the link on which the end effector "lives".  
        activeLinks (list of int or str): the links that are driven by
            cartesian commands to this end effector.
        ikObjective (IKObjective, optional): the template IK objective giving
            the tool center point and objective type. If None, this will be
            left unspecified.
    """
    def __init__(self,link,activeLinks,ikObjective=None):
        self.link = link                 # type: Union[int,str]
        self.activeLinks = activeLinks   # type: List[Union[int,str]]
        self.ikObjective = ikObjective   # type: Optional[IKObjective]


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
            this refers to.  Note: this is not necessarily a model of just the
            gripper.  To get just the gripper, use ``self.getSubrobot()``.
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
        res = GripperInfo(fn,-1)
        with open(fn,'r') as f:
            jsonobj = json.load(f)
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

    def getGeometry(self, robot : RobotModel, qfinger=None,type='Group') -> Geometry3D:
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
                g = robot.link(link).geometry().clone()
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

    def addToVis(self, robot : RobotModel=None, animate=True,base_xform=None) -> None:
        """Adds the gripper to the klampt.vis scene.
        
        Args:
            robot (RobotModel, optional): if given, this gripper is a sub-robot
                of the given robot.
            animate (bool): if True, animates the opening and closing of the
                gripper.
            base_xform (se3 element, optional): if given and robot=False, poses 
                the gripper base at this transform.
        """
        from klampt import vis
        from klampt import WorldModel,Geometry3D,GeometricPrimitive
        from klampt.model.trajectory import Trajectory
        prefix = "gripper_"+self.name
        if robot is None and self.klamptModel is not None:
            w = WorldModel()
            if w.readFile(self.klamptModel):
                robot = w.robot(0)
                vis.add(prefix+"_gripper",w)
                robotPath = (prefix+"_gripper",robot.getName())
        elif robot is not None:
            vis.add(prefix+"_gripper",robot)
            robotPath = prefix+"_gripper"
        if robot is not None:
            baseLink = robot.link(self.baseLink)
            assert baseLink.index >= 0 and baseLink.index < robot.numLinks()
            baseLink.appearance().setColor(1,0.75,0.5)
            if base_xform is None:
                base_xform = baseLink.getTransform()
            else:
                if baseLink.getParent() >= 0:
                    print("Warning, setting base link transform for an attached gripper base")
                #robot.link(self.baseLink).setParent(-1)
                baseLink.setParentTransform(*base_xform)
                robot.setConfig(robot.getConfig())
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
            vis.add(prefix+"_center",se3.apply(base_xform,self.center))
        center_point = (0,0,0) if self.center is None else self.center
        outer_point = (0,0,0)
        if self.primaryAxis is not None:
            length = 0.1 if self.fingerLength is None else self.fingerLength
            outer_point = vectorops.madd(self.center,self.primaryAxis,length)
            line = Trajectory([0,1],[self.center,outer_point])
            line.milestones = [se3.apply(base_xform,m) for m in line.milestones]
            vis.add(prefix+"_primary",line,color=(1,0,0,1))
        if self.secondaryAxis is not None:
            width = 0.1 if self.maximumSpan is None else self.maximumSpan
            line = Trajectory([0,1],[vectorops.madd(outer_point,self.secondaryAxis,-0.5*width),vectorops.madd(outer_point,self.secondaryAxis,0.5*width)])
            line.milestones = [se3.apply(base_xform,m) for m in line.milestones]
            vis.add(prefix+"_secondary",line,color=(0,1,0,1))
        elif self.maximumSpan is not None:
            #assume vacuum gripper?
            p = GeometricPrimitive()
            p.setSphere(outer_point,self.maximumSpan)
            g = Geometry3D()
            g.set(p)
            vis.add(prefix+"_opening",g,color=(0,1,0,0.25))
        #TODO: add finger box

    def removeFromVis(self) -> None:
        """Removes a previously-added gripper from the klampt.vis scene."""
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
        


def _resolve_file(file,search_paths=[]):
    if '~' in file:
        return _resolve_file(os.path.expanduser(file),search_paths)
    if os.path.exists(file):
        return os.path.abspath(file)
    for path in search_paths:
        if os.path.exists(os.path.join(path,file)):
            return os.path.abspath(os.path.join(path,file))
    raise RuntimeError("Unable to resolve {} in paths {}".format(file,search_paths))

def _dynamic_load_module(fn,search_paths=[]):
    if fn.endswith('py') or fn.endswith('pyc'):
        path,base = os.path.split(fn)
        loaded = False
        exc = None
        for fullpath in [None] + search_paths:
            if fullpath is None:
                fullpath = fn
            else:
                fullpath = os.path.join(fullpath,fn)
            if os.path.exists(fullpath):
                path,base = os.path.split(fullpath)
                mod_name,file_ext = os.path.splitext(base)
                try:
                    sys.path.append(os.path.abspath(path))
                    print("Try path",sys.path[-1])
                    mod = importlib.import_module(mod_name,base)
                    sys.path.pop(-1)
                    loaded = True
                    break
                except ImportError as e:
                    import traceback
                    traceback.print_exc()
                    exc = e
                finally:
                    sys.path.pop(-1)
        if not loaded:
            print("Unable to load module",base,"in any of",path,"+s",search_paths)
            raise exc
    else:
        try:
            mod = importlib.import_module(fn)
        except ImportError as e:
            if not fn.startswith('/'):
                loaded = False
                for search_path in search_paths:
                    try:
                        sys.path.append(search_path)
                        mod = importlib.import_module(fn)
                        loaded = True
                        break
                    except ImportError as e:
                        pass
                    finally:
                        sys.path.pop(-1)
            if not loaded:
                raise e
        finally:
            sys.path.pop(-1)
    return mod
