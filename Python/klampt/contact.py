"""Definitions of frictional point contacts, contact maps, and basic wrench
subroutines."""

import ik
from robotsim import *
try:
	import numpy
except ImportError:
	pass

def idToObject(world,ID):
    """Helper: takes a WorldModel ID and converts it into an object."""
    assert(ID >= 0 and ID < world.numIDs())
    if ID < world.numTerrains():
        return world.terrain(ID)
    ID -= world.numTerrains()
    if ID < world.numRigidObjects():
        return world.rigidObject(ID)
    ID -= world.numRigidObjects()
    for i in xrange(world.numRobots()):
        if ID==0:
            return world.robot(i)
        ID -= 1
        if ID < world.numRobotLinks(i):
            return world.robotLink(i,ID)
        ID -= world.numRobotLinks(i)
    raise RuntimeError("Internal error in idToObject")


class ContactPoint:
    """A point contact between two rigid bodies, object1 and object2.

    Attributes:
        - x is the contact point in world coordinates.
        - n is the normal pointing from object1 into object 2.
        - kFriction is the friction coefficient.
        - object1 and object2 are the objects in contact (optional).
    """
    def __init__(self,x=[0.,0.,0.],n=[0.,0.,1.],kFriction=0.):
        self.x = x
        self.n = n
        self.kFriction = kFriction
        self.object1 = None
        self.object2 = None

    def reflect(self):
        p = ContactPoint(self.x,[-ni for ni in self.n],self.kFriction)
        p.object2,p.object1 = self.object1,self.object2
        return p

def contactMap(contacts,fixed=None):
    """Given an unordered list of ContactPoints, computes a canonical dict
    from (obj1,obj2) pairs to a list of contacts on those objects.
    The resulting dict is also regularized so that objects are sorted in
    increasing getID(), so that (obj1,obj2) is not duplicated as (obj2,obj1).
    
    If fixed is provided, all objects for which fixed(x) returns true will be
    set to None.  The most common example, which fixes terrains, is
       lambda(x): x==None or isinstance(x,TerrainModel)
    """
    worlds = set()
    robots = set()
    objects = set()
    #check which worlds, robots, and objects are used
    for c in contacts:
        assert(c.object1.world == c.object2.world),"Contacts need to be in the same world"
        worlds.insert(c.object1.world)
    assert(len(worlds)<=1),"Only one world is supported"
    if len(worlds)==0: return []

    for c in contacts:
        if hasattr(c.object1,'robot'):
            assert(hasattr(c.object1,'robotIndex')),"Contact pairs must be RobotModelLink's"
            robots.insert(c.object1.robot)
        elif hasattr(c.object1,'object'):
            objects.insert(c.object1.object)
        if hasattr(c.object2,'robot'):
            assert(hasattr(c.object2,'robotIndex')),"Contact pairs must be RobotModelLink's"
            robots.insert(c.object2.robot)
        elif hasattr(c.object2,'object'):
            objects.insert(c.object2.object)

    #first sort out all the collision pairs
    paircontacts = dict()
    for c in contacts:
        reverse = False
        #treat all non RigidObjectModel or RobotModelLink objects as fixed
        o1 = c.object1 if not fixed(c.object1) else None
        o2 = c.object2 if not fixed(c.object2) else None
        if hasattr(o1,'getID'):
            if hasattr(o2,'getID'):
                if o2.getID() < o1.getID():
                    reverse=True
                elif o2.getID() == o1.getID():
                    raise RuntimeError("Contacts specified on object to itself")
        elif hasattr(o2,'getID'):
                reflect = True

        if o1 == o2:
            raise RuntimeError("Contact specified between an object and itself")

        if reflect:
            paircontacts.getdefault((o2,o1),[]).append(c.reflect())
        else:
            paircontacts.getdefault((o1,o2),[]).append(c)
    return paircontacts

def simContactMap(sim):
    """Given a robotsim simulation, returns a contact map representing all
    current contacts (among bodies with collision feedback enabled)."""
    cmap = dict()
    w = sim.getWorld()
    for a in xrange(w.numIDs()):
        for b in xrange(a):
            c = sim.getContacts(a,b)
            if len(c) > 0:
                #figure out the objects corresponding to a and b
                oa = idToObject(w,a)
                ob = idToObject(w,b)
                clist = [ContactPoint(ci[0:3],ci[3:6],ci[6]) for ci in c]
                cmap[(oa,ob)] = clist
    return cmap

def contactIKObjectives(contactMap):
    """Given a contact map, computes a set of non-conflicting
    IKObjective's or GeneralizedIKObjective's that enforce all simultaneous
    contact constraints.  Usually called in conjunction with contactMap
    with the following sequence:

    objectives = contactIKObjectives(contactMap(contacts,lambda(x):x==None or isinstance(x,TerrainModel)))
    """
    for (o1,o2) in contactMap.iterkeys():
        objectives = []
        for ((o1,o2),clist) in contactMap:
            assert o1 != None
            
            x1loc = [o1.getLocalPosition(c.x) for c in clist]
            if o2 != None:
                x2loc = [o2.getLocalPosition(c.x) for c in clist]
                objectives.append(ik.objective(o1,o2,local=x1loc,world=x2loc))
            else:
                x2 = [c.x for c in clist]
                objectives.append(ik.objective(o1,local=x1loc,world=x2))
        return objectives

def skew(x):
    """Returns the skew-symmetric cross-product matrix corresponding to the
    matrix x"""
    assert(len(x) == 3)
    xhat = numpy.zeros((3,3))
    xhat[0,1] = -x[2]
    xhat[1,0] = x[2]
    xhat[0,2] = x[1]
    xhat[2,0] = -x[1]
    xhat[1,2] = -x[0]
    xhat[2,1] = x[0]
    return xhat

def invMassMatrix(obj):
    """Returns the inverse of obj's generalized mass matrix
      [H 0 ]-1
      [0 mI]
    about the origin."""
    Hinv = numpy.zeros((6,6))
    if obj == None or isinstance(obj,TerrainModel):
        #infinite inertia
        return Hinv
    if isinstance(obj,RobotModel):
        return obj.getMassMatrixInv()
    m = obj.getMass()
    minv = 1.0/m.mass
    Hinv[3,3]=Hinv[4,4]=Hinv[5,5]=minv
    #offset the inertia matrix about the COM
    H = numpy.array((3,3))
    H[0,:] = numpy.array(m.inertia[0:3])
    H[1,:] = numpy.array(m.inertia[3:6])
    H[2,:] = numpy.array(m.inertia[6:9])
    H -= skew(m.com)*skew(m.com)*m.mass
    Hinv[0:3,0:3] = numpy.inv(H)
    return Hinv

def wrenchMatrices(contactMap):
    """Returns a map from contact pairs (o1,o2) to pairs of rigid-body wrench
    matrices (W1,W2) corresponding to each pair of objects in the contact map.

    Let the pair (o1,o2) have n contacts. The matrix W1 is a 6 x 3n mapping
    from contact force vectors applied on o2 to torque and force at the
    origin of o1.
    The columns [3i,...,3i+2] correspond to the force at the i'th point in
    contactMap[(o1,o2)].  Rows 0-2 correspond to torque about the origin, 
    while 3-5 correspond to force.

    W2 is similar, but is the jacobian regarding the force on o1.
    """
    res = dict()
    for ((o1,o2),clist) in contactMap:
        w1 = numpy.zeros((6,3*len(clist)))
        w2 = numpy.zeros((6,3*len(clist)))
        for (i,c) in enumerate(clist):
            arm = numpy.array(c.x)
            if o1 != None:
                arm -= numpy.array(o1.getTransform()[1])
            #skew symmetric product matrix
            w1[0:3,3*i:3*i+3] = skew(arm)            
            w1[3:6,3*i:3*i+3] = -numpy.eye(3)
            
        for (i,c) in enumerate(clist):
            arm = numpy.array(c.x)
            if o1 != None:
                arm -= numpy.array(o1.getTransform()[1])
            #negative skew symmetric product matrix
            w2[0:3,3*i:3*i+3] = -skew(arm)
            w2[3:6,3*i:3*i+3] = numpy.eye(3)
            
        res[(o1,o2)]=(w1,w2)
