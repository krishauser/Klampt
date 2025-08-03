"""Registry of semantic properties for robots to help build universal
(non-robot-specific) code. 

A robot typically consists of a set of parts, a base (either fixed or
moving), end effectors, a controller, and other assorted properties.  The
:class:`RobotInfo` data structure will let you define these and store them
in a human-editable file format.  This is also used for ``klampt_control`` and
some Robot Interface Layer controllers (see
:class:`klampt.control.robotinterfaceutils.OmniRobotInterface`).

"""
from __future__ import annotations
from klampt.control.robotinterface import RobotInterfaceBase
import os
import sys
import importlib
from klampt import WorldModel,RobotModel,Geometry3D,IKSolver,IKObjective,SimRobotController,SensorModel
from klampt.model.subrobot import SubRobotModel
from .gripperinfo import GripperInfo
from klampt.math import vectorops,so3,se3
import json
import warnings
from .typing import Vector,Vector3,RigidTransform
from typing import Optional,Union,Sequence,List,Tuple,Dict,Any,TextIO

class RobotInfo:
    """Stores common semantic properties for a robot.  Universal algorithms
    should use this structure to get information about the robot, rather
    than hardcoding robot-specific information.

    Attributes:
        name (str): a string identifying this robot by name.
        version (str, optional): a version string for this robot. 
        serial (str, optional): a serial number for this robot.
        description (str, optional): a human-readable description of the robot
            giving further details.
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
        preferences (dict of str -> list[str]): a dictionary of preferences
            mapping tasks to either parts, grippers, or sensors.  The
            interpretation of the tasks is up to the user, but you can, for
            example, indicate that the left arm and left gripper are preferred
            for grasping with the entry
            ``preferences["grasping"] = ["left_arm","left_gripper"]``.
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
        path,file = os.path.split(fn)
        if path:
            res.filePaths.append(path)
        res.load(fn)
        if res.modelFile is not None:  #make sure the model file is an absolute path
            if not res.modelFile.startswith('/') and not res.modelFile.startswith('http://')  and not res.modelFile.startswith('https://'):
                res.modelFile = os.path.abspath(os.path.join(os.path.split(fn)[0],res.modelFile))
        RobotInfo.register(res)
        return res

    def __init__(self,name,modelFile=None,
                baseType='fixed',
                parts=None,endEffectors=None,grippers=None,
                properties=None):
        self.name = name                # type: str
        self.version = None             # type: Optional[str]
        self.serial = None              # type: Optional[str]
        self.description = None         # type: Optional[str]
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
        self.preferences = {}             # type: Dict[str,List[str]]
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
                self.configureKinematics(file)
            else:
                try:
                    s = self.robotModel.sensor(k)
                except KeyError:
                    warnings.warn("Calibration item {} doesn't refer to a sensor or kinematics".format(k))
                    continue
                if s.getName():
                    self.configureSensor(s)
                else:
                    warnings.warn("Calibration item {} doesn't refer to a sensor or kinematics".format(k))
        return self.robotModel
    
    def configureKinematics(self, file : str):
        """Loads a json calibration file from the given file name and
        configures the kinematic model."""
        def docalib(fn):
            try:
                jsonobj = _load_config(fn,'KinematicCalibration')
            except IOError:
                return False
            if jsonobj.get('type',None) != 'KinematicCalibration':
                print("Warning, kinematic calibration file does not have 'type':'KinematicCalibration' tag")
            for k,items in jsonobj.items():
                link = self.robotModel.link(k) 
                if link.index < 0:
                    raise ValueError("Calibration file refers to invalid link {}".format(k))
                for key,value in items.items():
                    if key == 'axis':
                        link.setAxis(value)
                    elif key == 'Tparent':
                        link.setParentTransform(*value)
                    else:
                        raise KeyError("Invalid calibration item {}".format(key))
            return True
        if not self._tryload(file,docalib):
            raise IOError("Unable to load kinematics calibration from file "+file)
    
    def saveKinematicCalibration(self, file : str, update_calibration_attr = True):
        """Saves the kinematic calibration to a file"""
        if self.robotModel is None:
            raise RuntimeError("No robot model available")
        jsonobj = {'type':'KinematicCalibration'}
        for l in self.robotModel.links:
            jsonobj[l.name] = {'axis':l.getAxis(),'Tparent':l.getParentTransform()}
        if file.endswith('yaml'):
            import yaml
            with open(file,'w') as f:
                yaml.dump(jsonobj,f)
        else:
            if not file.endswith('json'):
                warnings.warn("Calibration file should be .json or .yaml, using json")
            with open(file,'w') as f:
                json.dump(jsonobj,f)
        if update_calibration_attr:
            self.calibrationFiles['kinematics'] = file

    def configureSensor(self,sensor : Union[int,str,SensorModel]) -> SensorModel:
        """Configures a sensor with a calibration, if present.

        The calibration file can be a JSON, YAML, or numpy file.  If the sensor
        is a camera, the calibration file can be an intrinsics calibration.
        """
        if not isinstance(sensor,SensorModel):
            return self.configureSensor(self.klamptModel().sensor(sensor))
        if sensor.name in self.calibrationFiles:
            calib_file = self.calibrationFiles[sensor.name]
            if calib_file.endswith('.json'):
                with open(calib_file,'r') as f:
                    calib = json.load(f)
            elif calib_file.endswith('.yaml'):
                import yaml
                with open(calib_file,'r') as f:
                    calib = yaml.load(f,Loader=yaml.SafeLoader)
            elif calib_file.endswith('.npy') or calib_file.endswith('.npz'):
                import numpy as np
                calib = np.load(calib_file)
            else:
                raise ValueError("Invalid calibration file format, must be .json or .yaml")
            if sensor.type == 'CameraSensor':
                #might be an intrinsics calibration
                from klampt.model import sensing
                if not isinstance(calib,dict):
                    sensing.intrinsics_to_camera(calib,sensor,'numpy')
                    return sensor
                elif 'fx' in calib:
                    assert 'fy' in calib and 'cx' in calib and 'cy' in calib, \
                        "Camera intrinsics calibration must have fx, fy, cx, cy"
                    sensing.intrinsics_to_camera(calib,sensor,'json')
                    del calib['fx']
                    del calib['fy']
                    del calib['cx']
                    del calib['cy']
            for k,value in calib.items():
                if hasattr(value,'__iter__'):
                    import klampt.io.loader
                    cast_value = klampt.io.loader.write(value,'auto')
                else:
                    cast_value = str(value)
                sensor.setSetting(k,cast_value)
        return sensor

    def saveSensorCalibration(self,sensor : Union[int,str,SensorModel],file : str, update_calibration_attr = True):
        """Configures a sensor with a calibration, if present.

        The calibration file can be a JSON, YAML, or numpy file.  If the sensor
        is a camera, the calibration file can be an intrinsics calibration.
        """
        if not isinstance(sensor,SensorModel):
            return self.saveSensorCalibration(self.klamptModel().sensor(sensor),file,update_calibration_attr)
        jsonobj = {'type':'SensorCalibration'}
        for s in sensor.settings():
            val = sensor.getSetting(s)
            #parse the string
            if ' ' in val:
                import klampt.io.loader
                jsonobj[s] = klampt.io.loader.read('Config',val)
        if file.endswith('yaml'):
            import yaml
            with open(file,'w') as f:
                yaml.dump(jsonobj,f)
        else:
            if not file.endswith('json'):
                warnings.warn("Calibration file should be .json or .yaml, using json")
            with open(file,'w') as f:
                json.dump(jsonobj,f)
        if update_calibration_attr:
            self.calibrationFiles[sensor.name] = file

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
    
    def configureControllerEndEffectors(self,controller: RobotInterfaceBase) -> Tuple[Dict[str,str],List[str]]:
        """Configures a Robot Interface Layer object with the appropriate
        end effector configuration.

        Assumes the controller is initialized.
        
        Returns a dictionary mapping end effectors to parts and a list of
        end effectors that are not associated with any part.
        """
        eematches = dict()
        unmatchedEEs = set(eename for eename in self.endEffectors)
        controllerParts = [None]
        try:
            parts = controller.parts()
            for k in parts:
                if k is not None:
                    controllerParts.append(k)
        except NotImplementedError:
            pass

        for part in controllerParts:
            partIndices = controller.indices(part)
            for eename,ee in self.endEffectors.items():
                activeDrivers = self.toDriverIndices(ee.activeLinks)
                if partIndices == activeDrivers:
                    if part is None:
                        print("Controller is a match for end effector",eename)
                        controller.properties['klamptModelCartesianLink'] = ee.link
                    else:
                        print("Controller for part",part,"is a match for end effector",eename)
                        controller.partInterface(part).properties['klamptModelCartesianLink'] = ee.link
                    #use the robotinfo end effector's link as the cartesian link
                    eematches[eename] = part
                    unmatchedEEs.remove(eename)
        robot = self.klamptModel()
        controller.beginStep()
        for eename,part in eematches.items():
            ee = self.endEffectors[eename]
            partInterface = controller.partInterface(part)
            obj = ee.ikObjective   # type: IKObjective
            if obj is None:
                local = [0,0,0]
            else:
                local,world = obj.getPosition()
            partInterface.setToolCoordinates(local)
        controller.endStep()
        controller.beginStep()
        for eename,part in eematches.items():
            ee = self.endEffectors[eename]
            partInterface = controller.partInterface(part)
            obj = ee.ikObjective   # type: IKObjective
            if obj is None:
                local = [0,0,0]
            else:
                local,world = obj.getPosition()
            #check that the tool coordinates are set correctly
            Tcmd = partInterface.commandedCartesianPosition()
            qcmd = partInterface.commandedPosition()
            subrobot = SubRobotModel(robot, ee.activeLinks)
            subrobot.setConfig(subrobot.configFromDrivers(qcmd))
            t = robot.link(ee.link).getWorldPosition(local)
            R = robot.link(ee.link).getTransform()[0]
            if vectorops.norm(vectorops.sub(t,Tcmd[1])) > 1e-3 or vectorops.norm(so3.error(R,Tcmd[0])) > 1e-3:
                warnings.warn("Tool coordinates or link for end effector {} are not set correctly".format(eename))
        controller.endStep()
        return eematches,list(unmatchedEEs)
            
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
            raise RuntimeError("Error running make(sim,robotIndex) for module {}: {}".format(mod.__name__,str(e)))
        try:
            controller,emulators = res
            for e in emulators:
                pass
        except Exception:
            raise RuntimeError("Result of make(sim,robotIndex) for module {} is not a pair (controller,emulators)".format(mod.__name__,))
        if not isinstance(controller,SimRobotController):
            print("Setting the simulator controller block to",controller.__class__.__name__,"for robot",robotIndex)
            assert callable(controller) or hasattr(controller,'advance')
            sim.setController(robotIndex,controller)
        for e in emulators:
            print("Adding emulator",e.__class__.__name__,"to robot",robotIndex)
            sim.addEmulator(robotIndex,e)

        #configure simulated sensor
        simcontroller = sim.controller(robot)
        for s in simcontroller.sensors:
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
        """Loads the info from a JSON or YAML file. f is a file name or file object."""
        from ..io import loader
        if isinstance(f,str):
            jsonobj = _load_config(f,'RobotInfo')
        else:
            jsonobj = json.load(f)
        self.endEffectors = dict()
        self.grippers = dict()
        REQUIRED = ['name','modelFile']
        OPTIONAL = ['version','serial','description','parts','baseType','endEffectors','grippers','properties','controllerFile','controllerArgs','simulatorFile','calibrationFiles','resourceDir','filePaths','preferences']
        for attr in REQUIRED+OPTIONAL:
            if attr not in jsonobj:
                if attr in OPTIONAL: #optional
                    continue
                else:
                    raise IOError("Loaded JSON object doesn't contain '"+attr+"' key")
            setattr(self,attr,jsonobj[attr])
        self.modelFile = _resolve_file(self.modelFile,self.filePaths)  #get absolute path of model file
        ees = dict()
        for (k,v) in self.endEffectors.items():
            obj = None if v['ikObjective'] is None else loader.from_json(v['ikObjective'],'IKObjective')
            ees[k] = EndEffectorInfo(v['link'],v['activeLinks'],obj)
            ees[k].gripper = v.get('gripper',None)
        self.endEffectors = ees
        grippers = dict()
        for (k,v) in self.grippers.items():
            gripper = GripperInfo('',0)
            if isinstance(v,str):
                #directive format?
                if v.startswith('!'):
                    components = v.split()
                    if components[0] == '!mounted':
                        if len(components) != 3:
                            raise ValueError("Invalid gripper directive %s, should be !mounted <filename> <target>"%v)
                        fn = _resolve_file(components[1],self.filePaths)
                        target_link_or_ee = components[2]
                        v = _load_config(fn,'GripperInfo')
                        gripper.fromJson(v)
                        target_ee = None
                        if target_link_or_ee in self.endEffectors:
                            target_ee = self.endEffectors[target_link_or_ee]
                            #find a link that is mounted to this end effector
                            target_link = -1
                            for link in self.klamptModel().links:
                                if link.getParent() == target_ee.link:
                                    target_link = link.getIndex()
                                    break
                            if target_link < 0:
                                raise ValueError("End effector %s has no links mounted to it, cannot mount gripper %s"%(target_link_or_ee,k))
                        else:
                            try:
                                target_link = int(target_link_or_ee)
                            except ValueError:
                                target_link = self.klamptModel().link(target_link_or_ee).getIndex()
                            if target_link < 0:
                                raise ValueError("Invalid target link %s for gripper %s, must be a valid link index or name"%(target_link_or_ee,k))
                        print('Mounting gripper "%s" on link %s'%(k,target_link))
                        grippers[k] = GripperInfo.mounted(gripper, self.modelFile, target_link , k)
                        if target_ee is not None:
                            target_ee.gripper = k
                    elif components[0] == '!include':
                        if len(components) != 2:
                            raise ValueError("Invalid gripper directive %s, should be !include <filename>"%v)
                        fn = _resolve_file(components[1],self.filePaths)
                        v = _load_config(fn,'GripperInfo')
                        gripper.fromJson(v)
                        grippers[k] = gripper
                    else:
                        raise ValueError("Invalid gripper directive %s, can only handle !mount and !include"%v)
                else:
                    raise ValueError("Invalid gripper string %s, should start with ! to issue directives" % v)
            else:
                gripper.fromJson(v)
                grippers[k] = gripper
        self.grippers = grippers

    def save(self, f : Union[str,TextIO]) -> None:
        """Saves the info to a JSON file. f is a file object."""
        from ..io import loader
        jsonobj = dict()
        for attr in ['name','version','serial','description','modelFile','baseType','controllerFile','simulatorFile','resourceDir','filePaths']:
            val = getattr(self,attr)
            if val is not None:
                jsonobj[attr] = val
        for attr in ['calibrationFiles','preferences','controllerArgs','parts','properties']:
            val = getattr(self,attr)
            if len(val) > 0:
                jsonobj[attr] = val
        ees = dict()
        for k,v in self.endEffectors.items():
            ees[k] = v.toJson()
        if len(ees) > 0:
            jsonobj['endEffectors'] = ees
        grippers = dict()
        for k,v in self.grippers.items():
            grippers[k] = v.toJson()
        if len(grippers) > 0:
            jsonobj['grippers'] = grippers
        if isinstance(f,str):
            if f.endswith('.yaml'):
                import yaml
                with open(f,'w') as file:
                    yaml.dump(jsonobj,file)
            else:
                if not f.endswith('.json'):
                    warnings.warn("RobotInfo file should be .json or .yaml, using json")
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
            the tool center point and objective type. The target position and
            rotation are ignored.  If None, the TCP will be left unspecified
            and a 6+-link constraint will be assumed to be a fixed transform
            objective.
        gripper (str, optional): if provided, the name of the gripper attached
            to this end effector.  This may be used for grasp planning and
            other tasks.
    """
    def __init__(self,link,activeLinks,ikObjective=None):
        self.link = link                 # type: Union[int,str]
        self.activeLinks = activeLinks   # type: List[Union[int,str]]
        self.ikObjective = ikObjective   # type: Optional[IKObjective]
        self.gripper = None              # type: Optional[str]
    
    def setTCP(self, tcp: Vector3):
        """Sets the tool center point of this end effector to the given
        coordinates in the link's local frame.
        """
        if self.ikObjective is None:
            self.ikObjective = IKObjective()
            self.ikObjective.setFixedPoint(self.link,tcp,[0,0,0])
            self.ikObjective.setFixedRotConstraint(so3.identity())
        else:
            self.ikObjective.setFixedPosConstraint(tcp,[0,0,0])
        
    def getTCP(self) -> Vector3:
        """Returns the tool center point of this end effector in the link's
        local frame.
        """
        if self.ikObjective is None:
            return [0,0,0]
        return self.ikObjective.getPosition()[0]

    def toJson(self) -> Dict[str,Any]:
        from ..io import loader
        obj = None if self.ikObjective is None else loader.to_json(self.ikObjective)
        return {'link':self.link,'activeLinks':self.activeLinks,'ikObjective':obj,'gripper':self.gripper}

    def fromJson(self,jsonobj: Dict[str,Any]) -> None:
        from ..io import loader
        self.link = jsonobj['link']
        self.activeLinks = jsonobj['activeLinks']
        if 'ikObjective' in jsonobj:
            obj = jsonobj['ikObjective']
            if obj is not None:
                self.ikObjective = loader.from_json(obj,'IKObjective')
            else:
                self.ikObjective = None
        if 'gripper' in jsonobj:
            self.gripper = jsonobj['gripper']


def _load_config(fn : str, config_type : str) -> dict:
    if fn.endswith('.yaml'):
        import yaml
        with open(fn,'r') as file:
            jsonobj = yaml.load(file,Loader=yaml.SafeLoader)
    else:
        if not fn.endswith('.json'):
            warnings.warn(config_type+" file should be .json or .yaml, trying json")
        with open(fn,'r') as file:
            jsonobj = json.load(file)
    return jsonobj

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
