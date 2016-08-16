from ..robotsim import WorldModel,RobotModel,RobotModelLink,RigidObjectModel,IKObjective
import coordinates

def getConfig(item):
    """Returns a flattened version of the configuration of the given item.
    Nearly all Klamp't objects are recognized, including RobotModel's,
    RigidObjectModel's, WorldModel's, and all variable types in the
    coordinates module.

    TODO: IKObjective,ContactPoint
    """
    if hasattr(item,'getConfig'):
        return item.getConfig()
    elif isinstance(item,RigidObjectModel):
        R,t = item.getTransform()
        return R+t
    elif isinstance(item,WorldModel):
        res = []
        for i in range(item.numRobots()):
            res += getConfig(item.robot(i))
        for i in range(item.numRigidObjects()):
            res += getConfig(item.rigidObject(i))
        return res
    elif isinstance(item,coordinates.Point):
        return item.localCoordinates()
    elif isinstance(item,coordinates.Direction):
        return item.localCoordinates()
    elif isinstance(item,coordinates.Frame):
        R,t = item.relativeCoordinates()
        return R+t
    elif isinstance(item,coordinates.Group):
        res = []
        for n,f in item.frames.iteritems():
            res += getConfig(f)
        for n,p in item.points.iteritems():
            res += getConfig(p)
        for n,d in item.directions.iteritems():
            res += getConfig(d)
        for n,g in item.subgroups.iteritems():
            res += getConfig(g)
        return res
    elif hasattr(item,'__iter__'):
        if not hasattr(item[0],'__iter__'):
            return item[:]
        else:
            return sum(item,[])
    else:
        return []

def setConfig(item,vector):
    """Sets the configuration of the given item to the given vector.
    Nearly all Klamp't objects are recognized, including RobotModel's,
    RigidObjectModel's, WorldModel's, and all variable types in the
    coordinates module.

    TODO: IKObjective,ContactPoint
    """
    if hasattr(item,'setConfig'):
        assert len(vector)==item.numLinks(),"Robot model config has %d DOFs"%(item.numLinks(),)
        item.setConfig(vector)
    elif isinstance(item,RigidObjectModel):
        assert len(vector)==12,"Rigid object model config has 12 DOFs"
        item.setTransform(vector[:9],vector[9:])
    elif isinstance(item,WorldModel):
        k=0
        for i in range(item.numRobots()):
            n = item.robot(i).numLinks()
            setConfig(item.robot(i),vector[k:k+n])
            k += n
        for i in range(item.numRigidObjects()):
            n = 12
            setConfig(item.robot(i),vector[k:k+n])
            k += n
        assert k == len(vector),"World model has %d DOFs"%(k,)
    elif isinstance(item,coordinates.Point):
        assert len(vector)==3,"Point config has 3 DOFs"
        item._localCoordinates = vector[:]
    elif isinstance(item,coordinates.Direction):
        assert len(vector)==3,"Direction config has 3 DOFs"
        item._localCoordinates = vector[:]
    elif isinstance(item,coordinates.Frame):
        assert len(vector)==12,"Frame config has 12 DOFs"
        item._relativeCoordinates = (vector[:9],vector[9:])
    elif isinstance(item,coordinates.Group):
        k = 0
        for n,f in item.frames.iteritems():
            setConfig(f,vector[k:k+12])
            k += 12
        for n,p in item.points.iteritems():
            setConfig(p,vector[k:k+3])
            k += 3
        for n,d in item.directions.iteritems():
            setConfig(d,vector[k:k+3])
            k += 3
        for n,g in item.subgroups.iteritems():
            raise NotImplementedError("TODO: set configuration of Group's subgroups")
    elif hasattr(item,'__iter__'):
        if not hasattr(item[0],'__iter__'):
            return vector[:]
        else:
            #split vector according to the sizes of the items
            res = []
            k = 0
            for v in item:
                n = len(v)
                res.append(vector[k:k+n])
                k += n
            return res
    return item
