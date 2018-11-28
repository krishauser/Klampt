"""Definitions of frictional point contacts, contact maps, and basic wrench
subroutines."""

import ik
from ..math import so3,se3
from .. import robotsim
from ..robotsim import RobotModel,RobotModelLink,RigidObjectModel,TerrainModel

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
    raise RuntimeError("Internal error in idToObject, invalid ID?")


class ContactPoint:
    """A point contact between two rigid bodies, object1 and object2.

    Attributes:
        - x is the contact point in world coordinates.
        - n is the normal pointing from object1 into object 2.
        - kFriction is the friction coefficient.
        - object1 and object2 are the objects in contact (optional).
    """
    def __init__(self,x=None,n=None,kFriction=0.):
        if x is None:
            x = [0.,0.,0.]
        if n is None:
            n = [0.,0.,1.]
        self.x = x
        self.n = n
        self.kFriction = kFriction
        self.object1 = None
        self.object2 = None

    def reflect(self):
        """Flips the contact point to switch the base object from object1
        to object2"""
        p = ContactPoint(self.x,[-ni for ni in self.n],self.kFriction)
        p.object2,p.object1 = self.object1,self.object2
        return p

    def transform(self,xform):
        """Given a rigid transform xform given as an se3 element, transforms
        the contact point."""
        self.x = se3.apply(xform,self.x)
        self.n = so3.apply(xform[0],self.n)

    def tolist(self):
        """Returns a 7-list representing this contact point, for use in
        the stability testing routines."""
        return list(self.x) + list(self.n) + [self.kFriction]

    def fromlist(self,v):
        """Reads the values x,n, and kFriction from the 7-list v."""
        if len(v) != 7: raise ValueError("ContactPoint can only be converted from a 7-element list")
        self.x = v[0:3]
        self.n = v[3:6]
        self.kFriction = v[6]



class Hold:
    """A Hold, contains both contact points and an IK constraint.
    Similar to the Hold class in the C++ RobotSim library.

    Attributes:
        - link: the link index
        - ikConstraint: an IKObjective object
          (see klampt.robotsim.IKObjective or klampt.ik.objective)
        - contacts: a list of ContactPoint objects
          (see klampt.contact.ContactPoint)
    """
    def __init__(self):
        self.link = None
        self.ikConstraint = None
        self.contacts = []

    def setFixed(self,link,contacts):
        """Creates this hold such that it fixes a robot link to match a list of contacts
        (in world space) at its current transform.

        Arguments:
        - link: a robot link or rigid object, currently contacting the environment / object at contacts
        - contacts: a list of ContactPoint objects, given in world coordinates.
        """
        assert isinstance(link,(RobotModelLink,RigidObjectModel)),"Argument must be a robot link or rigid object"
        self.link = link.index
        T = link.getTransform()
        Tinv = se3.inv(T)
        self.ikConstraint = ik.objective(link,local=[se3.apply(Tinv,c.x) for c in contacts],world=[c.x for c in contacts])
        self.contacts = contacts[:]

    def transform(self,xform):
        """Given a rigid transform xform given as an se3 element, transforms the hold's
        contacts and ik constraint in-place."""
        for c in self.contacts:
            c.transform(xform)
        self.ikConstraint.transform(*xform)




def _flatten(contactOrHoldList):
    if isinstance(contactOrHoldList,ContactPoint):
        return [contactOrHoldList.tolist()]
    elif isinstance(contactOrHoldList,Hold):
        return [c.tolist() for c in contactOrHoldList.contacts]
    else:
        return sum([_flatten(c) for c in contactOrHoldList],[])

def forceClosure(contactOrHoldList):
    """Given a list of contacts or Holds, tests for force closure.
    Return value is True or False"""
    return robotsim.forceClosure(_flatten(contactOrHoldList))
    
def comEquilibrium(contactOrHoldList,fext=(0,0,-1),com=None):
    """Given a list of contacts or Holds, an external gravity force,
    and a COM, tests for the existence of an equilibrium solution.

    If com == None, this tests whether there exists any equilibrium
    com, and returns True/False.

    If com != None, this returns either None if there is no solution,
    or otherwise returns a list of contact forces"""
    return robotsim.comEquilibrium(_flatten(contactOrHoldList),fext,com)

def supportPolygon(contactOrHoldList):
    """Given a list of contacts or Holds, returns the support polygon.
    The support polygon is given by list of tuples (ax,ay,b) such
    that the contraint ax*x+ay*y <= c holds for all (x,y) in the support
    polygon.

    An empty support polygon is given by the result [(0,0,-1)].
    A complete support polygon is given by the result [].
    """
    return robotsim.supportPolygon(_flatten(contactOrHoldList))

def equilibriumTorques(robot,holdList,fext=(0,0,-9.8),internalTorques=None,norm=0):
    """ Solves for the torques / forces that keep the robot balanced against gravity.
 
    Arguments
    - robot: the robot model, posed in its current configuration
    - holdList: a list of Holds.
    - fext: the external force (e.g., gravity)
    - internalTorques: if given, a list of length robot.numDofs giving internal torques.
      For example, can incorporate dynamics into the solver.
    - norm: the torque norm to minimize.  If 0, minimizes the l-infinity norm (default)
         If 1, minimizes the l-1 norm.  If 2, minimizes the l-2 norm (experimental,
         may not get good results)
    Return value is a pair (t,f) giving the joint torques and a list of frictional
    contact forces, if a solution exists. The return value is None if no solution exists.
    """
    links = sum([[h.link]*len(h.contacts) for h in holdList],[])
    if internalTorques is None:
        res = robotsim.equilibriumTorques(robot,_flatten(holdList),links,fext,norm)
    else:
        res = robotsim.equilibriumTorques(robot,_flatten(holdList),links,fext,internalTorques,norm)
    if res is None: return res
    f = res[1]
    return (res[0],[f[i*3:i*3+3] for i in xrange(len(f)/3)])

def contactMap(contacts,fixed=None):
    """Given an unordered list of ContactPoints, computes a canonical dict
    from (obj1,obj2) pairs to a list of contacts on those objects.
    The resulting dict is also regularized so that objects are sorted in
    increasing getID(), so that (obj1,obj2) is not duplicated as (obj2,obj1).
    
    If fixed is provided, all objects for which fixed(x) returns true will be
    set to None.  The most common example, which fixes terrains, is
       lambda x: x is None or isinstance(x,TerrainModel)
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
    w = sim.world
    for a in xrange(w.numIDs()):
        for b in xrange(a):
            c = sim.getContacts(a,b)
            if len(c) > 0:
                for ci in c:
                    assert len(ci) == 7,"Internal error in Simulation.getContacts()?"
                #figure out the objects corresponding to a and b
                oa = idToObject(w,a)
                ob = idToObject(w,b)
                clist = [ContactPoint(ci[0:3],ci[3:6],ci[6]) for ci in c]
                cmap[(oa,ob)] = clist
    return cmap

def contactMapIKObjectives(contactmap):
    """Given a contact map, computes a set of non-conflicting
    IKObjective's or GeneralizedIKObjective's that enforce all simultaneous
    contact constraints.  Usually called in conjunction with contactMap
    with the following sequence:

    objectives = contactMapIKObjectives(contactMap(contacts,lambda x:x==None or isinstance(x,TerrainModel)))
    """
    objectives = []
    for ((o1,o2),clist) in contactmap.iteritems():
        assert o1 != None
        
        x1loc = [o1.getLocalPosition(c.x) for c in clist]
        if o2 is not None and not isinstance(o2,TerrainModel):
            x2loc = [o2.getLocalPosition(c.x) for c in clist]
            objectives.append(ik.objective(o1,o2,local=x1loc,world=x2loc))
        else:
            x2 = [c.x for c in clist]
            objectives.append(ik.objective(o1,local=x1loc,world=x2))
    return objectives

def contactMapHolds(contactmap):
    """Given a contact map, computes a set of non-conflicting
    Holds that enforce all simultaneous contact constraints.  Usually called in conjunction with contactMap
    with the following sequence:

    objectives = contactMapHolds(contactMap(contacts,lambda x:x==None or isinstance(x,TerrainModel)))
    """
    holds = []
    for ((o1,o2),clist) in contactmap.iteritems():
        assert o1 != None
        
        if not isinstance(o1,RobotModelLink):
            raise ValueError("Cannot retrieve Holds for contact map not containing robot links")
        h = Hold()
        h.link = o1.index
        h.contacts = clist
        x1loc = [o1.getLocalPosition(c.x) for c in clist]
        if o2 is not None and not isinstance(o2,TerrainModel):
            x2loc = [o2.getLocalPosition(c.x) for c in clist]
            h.ikConstraint = ik.objective(o1,o2,local=x1loc,world=x2loc)
        else:
            x2 = [c.x for c in clist]
            h.ikConstraint = ik.objective(o1,local=x1loc,world=x2)
        holds.append(h)
    return holds

def skew(x):
    """Returns the skew-symmetric cross-product matrix corresponding to the
    matrix x"""
    try:
        import numpy
    except ImportError:
        raise RuntimeError("skew(x) needs numpy")
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
    try:
        import numpy
    except ImportError:
        raise RuntimeError("invMassMatrix(obj) needs numpy")
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
    try:
        import numpy
    except ImportError:
        raise RuntimeError("wrenchMatrices(contactMap) needs numpy")
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
