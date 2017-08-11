"""Helpers for accessing world/simulation variables using a class-style
interface.

For example, map(world).robots[0].config retrieves the name of robot 0.
This saves a little writing compared to world.robot(0).getConfig().

You can also write map(world).robots[0].config = [q1,...,qn] to set the
configuration of robot 0, rather than world.robot(0).setConfig([q1,...,qn]).

Conveniently, you can write expressions like:
    print len(map(world).robots)
    map(world).robots[0].config[4] = 3.5

Which is a shortcut to:
    print world.numRobots()
    q = world.robot(0).getConfig()
    q[4] = 3.5
    world.robot(0).setConfig(q)
"""

from ..robotsim import *
import string

class map:
    """A class-style interface for accessing all elements of a
    WorldModel or Simulator.

    The following class hierarchy is supported:
        * indicates read-only
        # indicates sub-item set access is supported (otherwise, to set
          you have to set the entire object)
    WorldModel:
        *robots (list/dict of RobotModels)
        *rigidObjects (list/dict of RigidObjectModels)
        *terrains (list/dict of TerrainModels)
        *elements (list/dict of all elements by id)
        *[string]: accesses objects by name
    RobotModel:
        *name (string)
        *id (int)
        *links (list/dict of RobotModelLinks)
        #config (list of floats)
        #velocity (list of floats)
        #jointLimits (pair of lists of floats)
        velocityLimits (list of floats)
        accelerationLimits (list of floats)
        torqueLimits (list of floats)
        *[string]: accesses links by name
    RobotModelLink:
        *name (string)
        *id (int)
        *robot (RobotModel)
        parent (int)
        geometry (Geometry3D)
        appearance (Appearance)
        mass (Mass)
        #parentTransform (se3 object (R,t))
        #transform (se3 object (R,t))
        #axis (3-list of floats)
    RigidObjectModel:
        *name (string)
        *id (int)
        geometry (Geometry3D)
        appearance (Appearance)
        mass (Mass)
        contactParameters (ContactParameters)
        # transform (se3 object (R,t))
        # velocity (pair of 3D vectors w,v giving angular and translational velocity)
    TerrainModel:
        *name (string)
        *id (int)
        geometry (Geometry3D)
        appearance (Appearance)
    Simulator:
        *world (WorldModel)
        *bodies (list of SimBodys)
        *controllers (list/dict of SimRobotControllers)
        *time (float)
        *robots (list/dict of SimRobots)
        *rigidObjects (list/dict of SimBodys)
        *terrains (list of SimBodys)
        gravity (3-list of floats)
        [string]: accesses bodies by name
    SimRobot:
        *actualConfig (list of floats) 
        *actualVelocity (list of floats)
        *actualTorques (list of floats)
        *config (an alias of actualConfig)
        *velocity (an alias of actualVelocity)
        *torques (an alias of actualTorques)
        *links (list/dict of SimLinks)
    SimLink:
        # transform (se3 object (R,t))
        # velocity (pair of 3D vectors w,v giving angular and translational velocity)
    SimBody:
        enabled (bool)
        collisionPadding (float)
        surface (ContactParameters)
        # transform (se3 object (R,t))
        # objectTransform (se3 object (R,t))
        # velocity (pair of 3D vectors w,v giving angular and translational velocity)
        # objectVelocity (pair of 3D vectors w,v giving angular and translational velocity)
    SimRobotController:
        *commandedConfig (list of floats)
        *commandedVelocity (list of floats)
        *sensedConfig (list of floats)
        *sensedVelocity (list of floats)
        *sensors (list/dict of SimRobotSensors)
    
    dicts, lists, tuples: behave like normal
    
    Other objects (e.g., ContactParameters, Mass, Geometry3D): must
    be operated with using the standard Klamp't API.
    """
    
    def __init__(self,obj,setter=None):
        self.setter = setter
        if isinstance(obj,map):
            self.obj = obj.obj
        else:
            self.obj = obj

    def __getattr__(self,name):        
        if name in['obj','setter']: return self.__dict__[name]
        #any internal Python operations return the operation on the actual object
        if name.startswith('__'): return getattr(self.obj,name)
        #handle non-standard getters
        if isinstance(self.obj,WorldModel):
            if name == 'robots':
                return _index_name_map([self.obj.robot(i) for i in xrange(self.obj.numRobots())])
            elif name == 'rigidObjects':
                return _index_name_map([self.obj.rigidObject(i) for r in xrange(self.obj.numRigidObjects())])
            elif name == 'terrains':
                return _index_name_map([self.obj.terrain(i) for r in xrange(self.obj.numTerrains())])
            elif name == 'elements':
                elements = [self.obj.terrain(i) for r in xrange(self.obj.numTerrains())]+[self.obj.rigidObjects(i) for r in xrange(self.obj.numRigidObjects())]
                for i in xrange(self.obj.numRobots()):
                    elements.append(self.obj.robot(i))
                    for j in self.obj.robot(i).numLinks():
                        elements.append(self.obj.robotModelLink(i,j))
                return _index_name_map(elements)
            else:
                for i in xrange(self.obj.numRobots()):
                    if name == self.obj.robot(i).getName():
                        return map(self.obj.robot(i))
                for i in xrange(self.obj.numRigidObjects()):
                    if name == self.obj.rigidObject(i).getName():
                        return map(self.obj.rigidObject(i))
                for i in xrange(self.obj.numTerrains()):
                    if name == self.obj.terrain(i).getName():
                        return map(self.obj.terrain(i))
        elif isinstance(self.obj,RobotModel):
            if name == 'id':
                return self.obj.getID()
            elif name == 'links':
                return _index_name_map([self.obj.link(i) for i in xrange(self.obj.numLinks())])
            elif name == 'config':
                return map(self.obj.getConfig(),self.obj.setConfig)
            elif name == 'velocity':
                return map(self.obj.getVelocity(),self.obj.setVelocity)
            else:
                for i in xrange(self.obj.numLinks()):
                    if self.obj.link(i).getName() == name:
                        return map(self.obj.link(i))
        elif isinstance(self.obj,RobotModelLink):
            if name == 'id':
                return self.obj.getID()
            elif name == 'robot':
                return map(self.obj.robot())
            elif name == 'geometry':
                return self.obj.geometry()
            elif name == 'appearance':
                return self.obj.appearance()
            elif name == 'axis':
                return map(self.obj.getAxis(),self.obj.setAxis)
            elif name == 'transform':
                return map(self.obj.getTransform(),lambda T:self.obj.setTransform(*T))
            elif name == 'parentTransform':
                return map(self.obj.getParentTransform(),lambda T:self.obj.setParentTransform(*T))
        elif isinstance(self.obj,RigidObjectModel):
            if name == 'id':
                return self.obj.getID()
            elif name == 'geometry':
                return self.obj.geometry()
            elif name == 'appearance':
                return self.obj.appearance()
            elif name == 'transform':
                return map(self.obj.getTransform(),lambda T:self.obj.setTransform(*T))
            elif name == 'velocity':
                return map(self.obj.getVelocity(),lambda twist:self.obj.setVelocity(*twist))
        elif isinstance(self.obj,TerrainModel):
            if name == 'id':
                return self.obj.getID()
            elif name == 'geometry':
                return self.obj.geometry()
            elif name == 'appearance':
                return self.obj.appearance()
        elif isinstance(self.obj,Simulator):
            w = self.obj.world
            if name == 'world':
                return w
            elif name == 'bodies':
                return self.obj.getID()
            elif name == 'controllers':
                nr = w.numRobots()
                return _index_name_map([self.obj.controller(i) for i in range(nr)],[w.robot(i).getName() for i in range(nr)])
            elif name == 'robots':
                nr = w.numRobots()
                return _index_name_map([_SimRobot(self.obj,i) for i in range(w.numRobots())],[w.robot(i).getName() for i in range(nr)])
            elif name == 'rigidObjects':
                nr = w.numRigidObjects()
                return _index_name_map([_SimObjectCentricBody(self.obj.body(w.rigidObject(i))) for i in range(w.numRigidObjects())],[w.rigidObject(i).getName() for i in range(nr)])
            elif name == 'terrains':
                nr = w.numTerrains()
                return _index_name_map([self.obj.body(w.terrain(i)) for i in range(w.numTerrains())],[w.terrain(i).getName() for i in range(nr)])
            elif name == 'gravity':
                return map(self.obj.getGravity(),self.obj.setGravity)
            else:
                try:
                    obj = getattr(map(w),name)
                    if isinstance(obj.obj,RobotModel):
                        return map(_SimRobot(self.obj,obj.obj.index))
                    return map(self.obj.body(obj.obj))
                except Exception as e:
                    print "Exception raised on Simulator.",name,":",e
                    print "Simulator has no object",name
                    pass
        elif isinstance(self.obj,_SimRobot):
            if name=='links':
                return _index_name_map(self.obj.getLinks(),self.obj.getBodyNames())
            elif name=='config':
                return self.obj.getActualConfig()
            elif name=='velocity':
                return self.obj.getActualVelocity()
            elif name=='torques':
                return self.obj.getActualTorques()
            elif name == 'bodies':
                raise AttributeError("Object of type "+self.obj.__class__.__name__+" does not have attribute "+name)
        elif isinstance(self.obj,_SimObjectCentricBody):
            if name == 'transform':
                return map(self.obj.obj.getObjectTransform(),lambda T:self.obj.obj.setObjectTransform(*T))
            elif name == 'velocity':
                return map(self.obj.obj.getObjectVelocity(),lambda twist:self.obj.obj.setObjectVelocity(*twist))
        elif isinstance(self.obj,SimBody):
            if name=='enabled':
                return self.obj.isEnabled()
            elif name == 'transform':
                return map(self.obj.getTransform(),lambda T:self.obj.setTransform(*T))
            elif name == 'objectTransform':
                return map(self.obj.getObjectTransform(),lambda T:self.obj.setObjectTransform(*T))
            elif name == 'velocity':
                return map(self.obj.getVelocity(),lambda twist:self.obj.setVelocity(*twist))
            elif name == 'objectVelocity':
                return map(self.obj.getObjectVelocity(),lambda twist:self.obj.setObjectVelocity(*twist))
        elif isinstance(self.obj,SimRobotController):
            if name=='sensors':
                sensors = []
                index = 0
                while True:
                    s = self.obj.getSensor(index)
                    if s.type() == '': break
                    sensors.append(s)
                    index += 1
                return _index_name_map(sensors,[s.name() for s in sensors])
        elif isinstance(self.obj,(list,tuple,dict)):
            print "Accessing",name,"from",self.obj
            return self.obj[name]

        #now do the default
        getname = 'get'+name[0].upper()+name[1:]
        if hasattr(self.obj,name):
            return getattr(self.obj,name)
        elif hasattr(self.obj,getname):
            return getattr(self.obj,getname)()
        raise AttributeError("Object of type "+self.obj.__class__.__name__+" does not have attribute "+name)

    def __setattr__(self,name,value):
        if name in ['obj','setter']:
            self.__dict__[name] = value
            return
        if self.setter != None:
            self.obj[name] = value
            self.setter(self.obj)
            return
        if isinstance(self.obj,WorldModel):
            raise ValueError("Element "+name+" is read only")
        elif isinstance(self.obj,RobotModel):
            if name == 'name' or name == 'id' or name == 'links':
                raise ValueError("Element "+name+" is read only")
            elif name == 'config':
                self.obj.setConfig(value)
                return
            elif name == 'velocity':
                self.obj.setVelocity(value)
                return
            elif name == 'jointLimits':
                self.obj.setJointLimits(*value)
                return
        elif isinstance(self.obj,RobotModelLink):
            if name == 'name' or name == 'id' or name == 'robot':
                raise ValueError("Element "+name+" is read only")
            elif name == 'geometry':
                self.obj.geometry().set(value)
                return
            elif name == 'appearance':
                self.obj.appearance().set(value)
                return
            elif name == 'parentTransform':
                self.obj.setParentTransform(*value)
                return
            elif name == 'transform':
                self.obj.setTransform(*value)
                return
        elif isinstance(self.obj,RigidObjectModel):
            if name == 'name' or name == 'id':
                raise ValueError("Element "+name+" is read only")
            elif name == 'geometry':
                self.obj.geometry().set(value)
                return
            elif name == 'transform':
                self.obj.setTransform(*value)
                return
            elif name == 'velocity':
                self.obj.setVelocity(*value)
                return
        elif isinstance(self.obj,TerrainModel):
            if name == 'name' or name == 'id':
                raise ValueError("Element "+name+" is read only")
            elif name == 'geometry':
                self.obj.geometry().set(value)
                return
            elif name == 'appearance':
                self.obj.appearance().set(value)
                return
        setname ='set'+name[0].upper()+name[1:]
        if hasattr(self.obj,name):
            setattr(self.obj,name,value)
            return
        elif hasattr(self.obj,setname):
            getattr(self.obj,name,setname)(value)
            return
        #duck-typing: add it to this item's dict
        self.__dict__[name]=value

    def __getitem__(self,index):
        #for nested lists / dicts with a setter
        if self.setter != None:
            #this is a sub-object of an object that needs custom setting
            def subsetter(x):
                self.obj[index] = x
                self.setter(self.obj)
            return map(self.obj[index],subsetter)
        return self.obj[index]

    def __setitem__(self,index,value):
        #for lists / dicts with a setter
        if self.setter != None:
            self.obj[index] = value
            self.setter(self.obj)
            return
        else:
            raise AttributeError("Array access invalid on object of type "+self.obj.__class__.__name__)

class _index_name_map:
    #helper: allows accessing items of lists via named string access
    #if the object supports object.getName()
    def __init__(self,objects,names=None):
        self.objects = objects
        if names != None:
            self.nameMap = dict((n,i) for i,n in enumerate(names))
        else:
            self.nameMap = dict()
            for i,o in enumerate(self.objects):
                self.nameMap[o.getName()]=i
    def __getitem__(self,index):
        if isinstance(index,str):
            return map(self.objects[self.nameMap[index]])
        return map(self.objects[index])
    def __len__(self):
        return len(self.objects)
    def __iter__(self):
        for i in self.objects:
            yield i
    def __setitem__(self,index):
        raise AttributeError("Cannot set in a map()'ed named list")

class _SimObjectCentricBody:
    #helper for accessing robot links and so on from simulation, but returning object-centric transform and velocity
    def __init__(self,obj):
        self.obj = obj
    
class _SimRobot:
    #helper for accessing robot
    def __init__(self,sim,index):
        self.sim = sim
        self.index = index
    def getActualConfig(self):
        return self.sim.getActualConfig(self.index)
    def getActualVelocity(self):
        return self.sim.getActualVelocity(self.index)
    def getActualTorques(self):
        return self.sim.getActualTorques(self.index)
    def getBodies(self):
        w = self.sim.world
        r = w.robot(self.index)
        return [self.sim.body(r.link(j) ) for j in range(r.numLinks())]
    def getBodyNames(self):
        w = self.sim.world
        r = w.robot(self.index)
        return [r.link(j).getName() for j in range(r.numLinks())]
    def getLinks(self):
        w = self.sim.world
        r = w.robot(self.index)
        return [_SimObjectCentricBody(self.sim.body(r.link(j) )) for j in range(r.numLinks())]
    

def get_item(obj,name):
    """Given a attribute item like 'robots[2].links[4].name', evaluates
    the value of the item in the object.

    Note: not secure! Uses eval()
    """
    loc = {'_w':map(obj)}
    result = {}
    return eval('_w.'+name,globals(),loc)

def set_item(obj,name,value):
    """Given a config dict, sets the object's configuration accordingly
    
    Note: not secure! Uses exec
    """
    loc = {'_w':map(obj)}
    result = {}
    exec '_w.'+name+'='+str(value) in globals(),loc

def get_dict(world,items):
    """Retrieves a dictionary of elements referred to by the given
    list of items.
    
    Note: not secure! Uses eval()
    """
    return dict((i,get_item(world,i)) for i in items)

def set_dict(world,config):
    """Sets the values in the dictionary config, which maps element names
    to values which should be set in the world.
        
    Note: not secure! Uses exec """
    for (k,v) in config.iteritems():
        set_item(world,k,v)






def flatten(value):
    if isinstance(value, (list,tuple)):
        return sum([flatten(vi) for vi in value],[])
    else:
        return [value]

def _match_hierarchy_iter(flattened,ref):
    if isinstance(ref,(list,tuple)):
        res = []
        for v in ref:
            resv,partial = _match_hierarchy_iter(flattened,v)
            res.append(resv)
            flattened = partial
        return res,flattened
    else:
        return flattened[0],flattened[1:]


def _match_hierarchy(flattened,ref):
    result,flattened = _match_hierarchy_iter(flattened,ref)
    if len(flattened) != 0:
        raise ValueError("Vector too large for hierarchy")
    return result


class Vectorizer:
    """A class that retrieves named items in a world and places them into
    a flattened vector.  Useful for planning.
    """
    def __init__(self,world,items):
        self.world = world
        self.ref = get_dict(world,items)
        self.keys = self.ref.keys()
        self.lengths = [len(flatten(self.ref[k])) for k in self.keys]

    def getVector(self):
        """Flattens the selected items in the world into a vector"""
        v = [get_item(self.world,k) for k in self.keys]
        return flatten(v)

    def setVector(self,v):
        """Un-flattens the selected elements in the world from a vector"""
        suml = 0
        for l,k in zip(self.lengths,self.keys):
            vk = v[suml:suml+l]
            vk_val = _match_hierarchy(vk,self.ref[k])
            set_item(self.world,k,vk_val)
            suml += l

if __name__ == '__main__':
    w = WorldModel()
    res = w.readFile('tx90blocks.xml')
    assert(res)
    print get_item(w,'tx90pr2.config')
    print get_item(w,'robots[0].config')
    print get_item(w,'robots[0].links[6].transform')
    print get_item(w,'cube1.transform')

    v = Vectorizer(w,['tx90pr2.config','cube1.transform'])
    q = v.getVector()
    q[4] = 1.0
    q[14] = 1.0
    v.setVector(q)
    assert(abs(v.getVector()[4]-1.0) < 1e-5)
    assert(abs(v.getVector()[14]-1.0) < 1e-5)
