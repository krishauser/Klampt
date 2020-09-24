class RobotInfo:
    """Stores common semantic properties for robots."""
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

    def ensureLoaded(self):
        if self.fn is None:
            raise RuntimeError("Can't load robot model for "+self.name+", no file given")
        if self.robotModel is not None:
            return self.robotModel
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
        partLinks = self.partLinks(part)
        self.ensureLoaded()
        return SubRobotModel(self.robotModel,partLinks)

    def toIndices(self,items):
        if all(isinstance(v,int) for v in items):
            return items
        else:
            self.ensureLoaded()
            for i,v in enumerate(items):
                if not isinstance(v,int):
                    assert isinstance(v,str),"Link identifiers must be int or str"
                    items[i] = self.robotModel.link(v).getIndex()
                    assert items[i] >= 0,"Link %s doesn't exist in robot %s"%(v,self.name)
            return items

    def toNames(self,items):
        if all(isinstance(v,str) for v in items):
            return items
        else:
            self.ensureLoaded()
            for i,v in enumerate(items):
                if not isinstance(v,str):
                    assert isinstance(v,int),"Link identifiers must be int or str"
                    assert v >= 0 and v < self.robotModel.numLinks(),"Link %d is invalid for robot %s"%(v,self.name)
                    items[i] = self.robotModel.link(v).getName()
            return items

    def load(self,f):
        import json
        jsonobj = json.load(f)
        for attr in ['name','fn','parts','freeBase','endEffectors','properties']:
            if attr not in jsonobj:
                raise IOError("Loaded JSON object doesn't contain '"+name+"' key")
            setattr(self,attr,jsonobj[attr])

    def save(self,f):
        import json
        jsonobj = dict()
        for attr in ['name','fn','parts','freeBase','endEffectors','properties']:
            jsonobj[attr] = getattr(self,attr)
        json.dump(jsonobj,f)


allRobots = dict()

def register(robotInfo):
    allRobotInfo[robotInfo.name()] = robotInfo

def load(fn):
    res = RobotInfo()
    with open(fn,'r') as f:
        res.load(f)
        register(res)
    return res

def get(name):
  return allRobots[name]
