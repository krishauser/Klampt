"""Helpers for accessing world variables using a class-style interface"""

from robotsim import *

class map:
    """A class-style interface for accessing all elements of the
    world model"""
    
    def __init__(self,obj):
        if isinstance(obj,map):
            self.obj = obj.obj
        else:
            self.obj = obj

    def __getattr__(self,name):        
        if hasattr(self.obj,name):
            return self.obj.name
        
        if isinstance(self.obj,WorldModel):
            if name == 'robots':
                return [map(self.obj.robot(i)) for i in xrange(self.obj.numRobots())]
            elif name == 'rigidObjects':
                return [map(self.obj.rigidObject(i)) for r in xrange(self.obj.numRigidObjects())]
            elif name == 'terrains':
                return [map(self.obj.terrain(i)) for r in xrange(self.obj.numTerrains())]
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
            if name == 'name':
                return self.obj.getName()
            elif name == 'id':
                return self.obj.getID()
            elif name == 'links':
                return [map(self.obj.getLink(i)) for i in xrange(self.obj.numLinks())]
            elif name == 'config':
                return self.obj.getConfig()
            elif name == 'velocity':
                return self.obj.getVelocity()
            elif name == 'jointLimits':
                return self.obj.getJointLimits()
            elif name == 'velocityLimits':
                return self.obj.getVelocityLimits()
            elif name == 'accelerationLimits':
                return self.obj.getAccelerationLimits()
            elif name == 'torqueLimits':
                return self.obj.getTorqueLimits()
            else:
                for i in xrange(self.obj.numLinks()):
                    if self.obj.getLink(i).getName() == name:
                        return map(self.obj.getLink(i))
        elif isinstance(self.obj,RobotModelLink):
            if name == 'name':
                return self.obj.getName()
            elif name == 'id':
                return self.obj.getID()
            elif name == 'robot':
                return map(self.obj.getRobot())
            elif name == 'parent':
                return self.obj.getParent()
            elif name == 'mesh':
                return self.obj.getMesh()
            elif name == 'mass':
                return self.obj.getMass()
            elif name == 'parentTransform':
                return self.obj.getParentTransform()
            elif name == 'transform':
                return self.obj.getTransform()
            elif name == 'axis':
                return self.obj.getAxis()
        elif isinstance(self.obj,RigidObjectModel):
            if name == 'name':
                return self.obj.getName()
            elif name == 'id':
                return self.obj.getID()
            elif name == 'mesh':
                return self.obj.getMesh()
            elif name == 'mass':
                return self.obj.getMass()
            elif name == 'contactParameters':
                return self.obj.getContactParameters()
            elif name == 'transform':
                return self.obj.getTransform()
        elif isinstance(self.obj,TerrainModel):
            if name == 'name':
                return self.obj.getName()
            elif name == 'id':
                return self.obj.getID()
            elif name == 'mesh':
                return self.obj.getMesh()            
        raise AttributeError(name)

    def __setattr__(self,name,value):
        if name == 'obj':
            self.__dict__['obj'] = value
            return
        elif hasattr(self.obj,name):
            setattr(self.obj,name,value)
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
            elif name == 'velocityLimits':
                self.obj.setVelocityLimits(value)
                return
            elif name == 'accelerationLimits':
                self.obj.setAccelerationLimits(value)
                return
            elif name == 'torqueLimits':
                self.obj.getTorqueLimits(value)
                return
        elif isinstance(self.obj,RobotModelLink):
            if name == 'name' or name == 'id' or name == 'robot':
                raise ValueError("Element "+name+" is read only")
            elif name == 'parent':
                self.obj.setParent(value)
                return
            elif name == 'mesh':
                self.obj.setMesh(value)
                return
            elif name == 'mass':
                self.obj.setMass(value)
                return
            elif name == 'parentTransform':
                self.obj.setParentTransform(*value)
                return
            elif name == 'transform':
                self.obj.setTransform(*value)
                return
            elif name == 'axis':
                self.obj.setAxis(value)
                return
        elif isinstance(self.obj,RigidObjectModel):
            if name == 'name' or name == 'id':
                raise ValueError("Element "+name+" is read only")
            elif name == 'mesh':
                self.obj.setMesh(value)
                return
            elif name == 'mass':
                self.obj.setMass(value)
                return
            elif name == 'contactParameters':
                self.obj.setContactParameters(value)
                return
            elif name == 'transform':
                self.obj.setTransform(*value)
                return
        elif isinstance(self.obj,TerrainModel):
            if name == 'name' or name == 'id':
                raise ValueError("Element "+name+" is read only")
            elif name == 'mesh':
                self.obj.setMesh(value)
        self.__dict__[name]=value

def get_item(obj,name):
    """Given a attribute item like 'robots[2].links[4].name', evaluates
    the value of the item in the object.
    """
    loc = {'_w':map(obj)}
    result = {}
    return eval('_w.'+name,globals(),loc)

def set_item(obj,name,value):
    """Given a config dict, sets the object's configuration accordingly"""
    loc = {'_w':map(obj)}
    result = {}
    exec '_w.'+name+'='+str(value) in globals(),loc

def get_dict(world,items):
    return dict((i,get_item(world,i)) for i in items)

def set_dict(world,config):
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


def match_hierarchy(flattened,ref):
    result,flattened = _match_hierarchy_iter(flattened,ref)
    if len(flattened) != 0:
        raise ValueError("Vector too large for hierarchy")
    return result


class Vectorizer:
    def __init__(self,world,items):
        self.world = world
        self.ref = get_dict(world,items)
        self.keys = self.ref.keys()
        self.lengths = [len(flatten(self.ref[k])) for k in self.keys]

    def getVector(self):
        v = [get_item(self.world,k) for k in self.keys]
        return flatten(v)

    def setVector(self,v):
        suml = 0
        for l,k in zip(self.lengths,self.keys):
            vk = v[suml:suml+l]
            vk_val = match_hierarchy(vk,self.ref[k])
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
