"""Registry of semantic properties for robots to help build generalized
(non-robot-specific) code. 

A robot typically consists of a set of parts, a base (either fixed or
moving), end effectors, a controller, and other assorted properties. 
"""
import os
import sys
import importlib

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
        self.link = link
        self.activeLinks = activeLinks
        self.ikObjective = ikObjective


class RobotInfo:
    """Stores common semantic properties for a robot.

    Attributes:
        name (str): a string identifying this robot by name.
        modelFile (str, optional): a file pointing to the Klamp't model (.urdf 
            or .rob.)
        freeBase (bool): whether the robot has a mobile or floating base.
        parts (dict str->list): a set of named parts.  Each part consists
            of a list of link names or indices.
        endEffectors (dict str->EndEffectorInfo): a set of named end
            effectors.
        properties (dict str->anything): a dict of named properties. Values
            must be JSON-loadable/savable.
        controllerFile (str, optional): the file containing code for the Klampt
            RIL controller. 

            Should be a Python file (``/path/to/file.py``) or module
            (`package.module`) containing a function ``make(robotModel)`` which
            returns a :class:`RobotInterfaceBase`.
        simulatorFile (str, optional): the file containing code for simulation
            emulators for the robot.  If not provided, the default robot
            simulator is used.

            Should be a Python file (``/path/to/file.py``) or module
            (`package.module`) containing a function ``make(robotModel,sim)``
            which returns a pair ``(controller, emulators)``.

            Here, ``controller`` is a :class:`RobotInterfaceBase` configured
            for the simlator (see :mod:`klampt.control.simrobotinterface`).
            ``emulators`` is a list of :class:`SensorEmulator` or
            :class:`ActuatorEmulator` objects configured for use in a
            :class:`SimpleSimulator`.
        filePaths (list of str): a list of paths where files will be searched.
        robotModel (RobotModel): a cached robot model, loaded upon calling
            :func:`klamptModel`.
    """
    def __init__(self,name,modelFile=None,
                freeBase=False,
                parts=None,endEffectors=None,
                properties=None):
        self.name = name
        self.modelFile = modelFile
        self.controllerFile = None
        self.simulatorFile = None
        self.freeBase = freeBase
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
                raise ValueError("`endEffectors` must be a dict from names to links")
        if properties is None:
            properties = dict()
        else:
            if not isinstance(properties,dict):
                raise ValueError("`properties` must be a dict")
        self.parts = parts
        self.endEffectors = endEffectors
        self.properties = properties
        self.filePaths = []
        self.robotModel = None
        self._worldTemp = None

    def klamptModel(self):
        """Returns the Klamp't RobotModel associated with this robot, either by
        the ``robotModel`` object or loading from the given filename
        ``self.modelFile``.

        The results are cached so this can be called many times without a
        performance hit.
        """
        if self.robotModel is not None:
            return self.robotModel
        if self.modelFile is None:
            raise RuntimeError("Can't load robot model for "+self.name+", no file given")
        from klampt import WorldModel,RobotModel
        self._worldTemp = WorldModel()
        self.robotModel = self._worldTemp.loadRobot(self.modelFile)
        if self.robotModel.index < 0:
            if not self.modelFile.startswith('/'):
                for path in self.filePaths:
                    self.robotModel = self._worldTemp.loadRobot(os.path.join(path,self.modelFile))
                    if self.robotModel.index >= 0:
                        break
            if self.robotModel.index < 0:
                raise IOError("Unable to load robot from file "+self.modelFile)
        self.robotModel.setName(self.name)
        return self.robotModel

    def controller(self):
        """Returns a :class:`~klampt.control.robotinterface.RobotInterfaceBase`
        configured for use on the real robot.  Requires ``controllerFile`` to 
        be defined.
        """
        if self.controllerFile is None:
            raise RuntimeError("Can't create controller for "+self.name+", no file given")
        mod = _dynamic_load_module(self.controllerFile)
        try:
            maker = mod.make
        except AttributeError:
            raise RuntimeError("Module {} must have a make(robotModel) method".format(mod.__name__))
        try:
            return mod.make(self.klamptModel())
        except Exception as e:
            raise RuntimeError("Error running make(robotModel) for module {}: {}"%(mod.__name__,str(e)))

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
            raise RuntimeError("Module {} must have a make(sim,) method".format(mod.__name__))
        try:
            res = mod.make(robotModel,sim,robotIndex)
        except Exception as e:
            raise RuntimeError("Error running make(robotModel,sim,robotIndex) for module {}: {}"%(mod.__name__,str(e)))
        try:
            controller,emulators = res
            for e in emulators:
                pass
        except Exception:
            raise RuntimeError("Result of make(robotModel,sim,robotIndex) for module {} is not a pair (controller,emulators)"%(mod.__name__,))
        sim.setController(robotIndex,controller)
        for e in emulators:
            sim.addEmulator(robotIndex,e)

    def partLinks(self,part):
        return self.parts[part]

    def partLinkIndices(self,part):
        res = self.parts[part]
        return self.toIndices(res)

    def partLinkNames(self,part):
        res = self.parts[part]
        return self.toNames(res)

    def partAsSubrobot(self,part):
        from klampt.model.subrobot import SubRobotModel
        partLinks = self.partLinkIndices(part)
        model = self.klamptModel()
        return SubRobotModel(model,partLinks)

    def eeSolver(self,endEffector,target):
        """Given a named end effector and a target point, transform, or set of
        parameters from config.setConfig(ikgoal) / config.getConfig(ikgoal),
        returns the :class:`~klampt.robotsim.IKSolver` for the end effector and
        that target.
        """
        ee = self.endEffectors[endEffector]
        from klampt import IKSolver,IKObjective
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

    def toIndices(self,items):
        """Returns link identifiers as link indices"""
        if all(isinstance(v,int) for v in items):
            return items
        else:
            robot = self.klamptModel()
            for i,v in enumerate(items):
                if not isinstance(v,int):
                    assert isinstance(v,str),"Link identifiers must be int or str"
                    items[i] = robot.link(v).getIndex()
                    assert items[i] >= 0,"Link %s doesn't exist in robot %s"%(v,self.name)
            return items

    def toNames(self,items):
        """Returns link identifiers as link names"""
        if all(isinstance(v,str) for v in items):
            return items
        else:
            robot = self.klamptModel()
            for i,v in enumerate(items):
                if not isinstance(v,str):
                    assert isinstance(v,int),"Link identifiers must be int or str"
                    assert v >= 0 and v < robot.numLinks(),"Link %d is invalid for robot %s"%(v,self.name)
                    items[i] = robot.link(v).getName()
            return items

    def load(self,f):
        """Loads the info from a JSON file. f is a file object."""
        import json
        from ..io import loader
        jsonobj = json.load(f)
        for attr in ['name','modelFile','parts','freeBase','endEffectors','properties','controllerFile','simulatorFile','filePaths']:
            if attr not in jsonobj:
                if attr in ['parts','endEffectors','properties','freeBase','parts','controllerFile','simulatorFile','filePaths']: #optional
                    continue
                else:
                    raise IOError("Loaded JSON object doesn't contain '"+attr+"' key")
            setattr(self,attr,jsonobj[attr])
        ees = dict()
        for (k,v) in self.endEffectors.items():
            obj = None if v['ikObjective'] is None else loader.fromJson(v['ikObjective'],'IKObjective')
            ees[k] = EndEffectorInfo(v['link'],v['activeLinks'],obj)
        self.endEffectors = ees

    def save(self,f):
        """Saves the info to a JSON file. f is a file object."""
        import json
        from ..io import loader
        jsonobj = dict()
        for attr in ['name','modelFile','parts','freeBase','properties','controllerFile','simulatorFile','filePaths']:
            jsonobj[attr] = getattr(self,attr)
        ees = dict()
        for k,v in self.endEffectors.items():
            obj = None if v.ikObjective is None else loader.toJson(v.ikObjective)
            ees[k] = {'link':v.link,'activeLinks':v.activeLinks,'ikObjective':obj}
        jsonobj['endEffectors'] = ees
        json.dump(jsonobj,f)


def _dynamic_load_module(fn,search_paths=[]):
    if fn.endswith('py') or fn.endswith('pyc'):
        path,base = os.path.split(fn)
        mod_name,file_ext = os.path.splitext(base)
        try:
            sys.path.append(os.path.abspath(path))
            mod = importlib.import_module(mod_name,base)
        except ImportError as e:
            if not fn.startswith('/'):
                loaded = False
                for search_path in search_paths:
                    try:
                        sys.path.append(os.path.abspath(os.path.join(search_path,path)))
                        mod = importlib.import_module(mod_name,base)
                        sys.path.pop(-1)
                        loaded = True
                        break
                    except ImportError:
                        pass
                    finally:
                        sys.path.pop(-1)
            if not loaded:
                raise e
        finally:
            sys.path.pop(-1)
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
                    except ImportError:
                        pass
                    finally:
                        sys.path.pop(-1)
            if not loaded:
                raise e
        finally:
            sys.path.pop(-1)
    return mod

allRobots = dict()

def register(robotInfo):
    """Registers a RobotInfo to the global registry."""
    global allRobots
    allRobots[robotInfo.name] = robotInfo

def load(fn):
    """Loads / registers a RobotInfo from a JSON file previously saved to disk."""
    res = RobotInfo()
    with open(fn,'r') as f:
        res.load(f)
        register(res)
    return res

def get(name):
    """Retrieves a registered RobotInfo from the global registry."""
    global allRobots
    return allRobots[name]
