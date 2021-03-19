"""Registry of semantic properties for robots.  A robot typically consists of a
set of parts, a base (either fixed or moving), end effectors, and other
assorted properties.  This module provides utilities to help build generalized
code.
"""

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
        name (str): a string identifying this robot
        fn (fn, optional): a file pointing to the Klamp't model.
        freeBase (bool): whether the robot has a mobile or floating base.
        endEffectors (dict str->EndEffectorInfo): a set of named end
            effectors.
        parts (dict str->list): a set of named parts.  Each part consists
            of a list of link names or indices.
        robotModel (RobotModel): a robot model, loaded upon calling
            klamptModel().
    """
    def __init__(self,name,fn=None,
                freeBase=False,
                parts=None,endEffectors=None,
                properties=None):
        self.name = name
        self.fn = fn
        self.robotModel = None
        self._worldTemp = None
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

    def klamptModel(self):
        """Returns the Klamp't RobotModel associated with this robot, either by
        the robotModel object or loading from the given filename self.fn.

        The results are cached so this can be called many times.
        """
        if self.robotModel is not None:
            return self.robotModel
        if self.fn is None:
            raise RuntimeError("Can't load robot model for "+self.name+", no file given")
        from klampt import WorldModel,RobotModel
        self._worldTemp = WorldModel()
        self.robotModel = self._worldTemp.loadRobot(self.fn)
        if self.robotModel.index < 0:
            raise IOError("Unable to load robot from file "+self.fn)
        return self.robotModel

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
        for attr in ['name','fn','parts','freeBase','endEffectors','properties']:
            if attr not in jsonobj:
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
        for attr in ['name','fn','parts','freeBase','properties']:
            jsonobj[attr] = getattr(self,attr)
        ees = dict()
        for k,v in self.endEffectors.items():
            obj = None if v.ikObjective is None else loader.toJson(v.ikObjective)
            ees[k] = {'link':v.link,'activeLinks':v.activeLinks,'ikObjective':obj}
        jsonobj['endEffectors'] = ees
        json.dump(jsonobj,f)


allRobots = dict()

def register(robotInfo):
    """Registers a RobotInfo to the global reigstry."""
    global allRobots
    allRobots[robotInfo.name()] = robotInfo

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
