"""Definitions of frictional point contacts, contact maps, basic wrench space
calculation subroutines, and performing equilibrium testing.
"""

from . import ik
from ..math import vectorops,so3,se3
from .. import robotsim
from ..robotsim import RobotModel,RobotModelLink,RigidObjectModel,TerrainModel
import numpy as np
import warnings

def id_to_object(world,ID):
    """Helper: takes a WorldModel ID and converts it into an object."""
    assert(ID >= 0 and ID < world.numIDs())
    if ID < world.numTerrains():
        return world.terrain(ID)
    ID -= world.numTerrains()
    if ID < world.numRigidObjects():
        return world.rigidObject(ID)
    ID -= world.numRigidObjects()
    for i in range(world.numRobots()):
        if ID==0:
            return world.robot(i)
        ID -= 1
        if ID < world.numRobotLinks(i):
            return world.robotLink(i,ID)
        ID -= world.numRobotLinks(i)
    raise RuntimeError("Internal error in id_to_object, invalid ID?")


class ContactPoint:
    """A point contact between two rigid bodies, object1 and object2.

    Attributes:
        x (list of 3 floats): the contact point in world coordinates.
        n (list of 3 floats): the normal pointing from object1 into object 2.
        kFriction (float): the friction coefficient.
        object1, object2 (optional): the objects in contact.
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
    """A container for both contact points and an IK constraint on a robot's
    link.  Can represent face-face, point-point, or edge contact. 

    Similar to the Hold class in the C++ RobotSim library.  Hypothetically,
    can also represent sliding contact.

    Attributes:
        link (int): the link index
        ikConstraint (IKObjective): the constraint
            (see klampt.robotsim.IKObjective or klampt.ik.objective)
        contacts (list of ContactPoint): the contacts used in the hold.
          (see klampt.contact.ContactPoint)
    """
    def __init__(self):
        self.link = None
        self.ikConstraint = None
        self.contacts = []

    def setFixed(self,link,contacts):
        """Creates this hold such that it fixes a robot link to match a list of
        contacts (in world space) at its current transform.

        Args:
            link: a robot link or rigid object, currently contacting the
                environment / object at contacts
            contacts (list of :class:`ContactPoint`): the fixed contact points,
                given in world coordinates.
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

def _toarray(contactOrHoldList):
    return np.array(_flatten(contactOrHoldList))

def force_closure(contactOrHoldList):
    """Given a list of ContactPoints or Holds, tests for force closure.
    Return value is True or False.

    Formulates the wrench matrix W of all contacts, including edges of the
    friction cones.  If Hull(W) contains the zero vector in its interior,
    then the contacts are said to be in force closure.
    """
    return robotsim.force_closure(_toarray(contactOrHoldList))
    
def com_equilibrium(contactOrHoldList,fext=(0,0,-1),com=None):
    """Given a list of ContactPoints or Holds, an external gravity force,
    and a COM, tests for the existence of an equilibrium solution.

    Specifically, an equilibrium exists when the following equations are
    met:

    .. math::

        \\begin{eqnarray}
          fext    & = \\sum_i f_i \\\\
          0 &= \\sum_i f_i \\times (p_i - com) \\\\
          f_i & \\in FC_i
       \end{eqnarray}
 
    where :math:`f_i`, :math:`p_i`, and :math:`FC_i` are the force, position,
    and the friction cone of the i'th contact point.

    If com == None, this tests whether there exists any equilibrium
    com, and returns True/False.

    If com != None, this returns either None if there is no solution,
    or otherwise returns a list of contact forces.
    """
    return robotsim.com_equilibrium(_toarray(contactOrHoldList),fext,com)

def support_polygon(contactOrHoldList):
    """Given a list of ContactPoints or Holds, returns the support polygon.
    The support polygon is given by list of tuples (ax,ay,b) such
    that the contraint ax*x+ay*y <= c holds for all (x,y) in the support
    polygon.

    An empty support polygon is given by the result [(0,0,-1)].
    A complete support polygon is given by the result [].
    """
    return robotsim.support_polygon(_toarray(contactOrHoldList))

def equilibrium_torques(robot,holdList,fext=(0,0,-9.8),internalTorques=None,norm=0):
    """ Solves for the torques / forces that keep the robot balanced against
    gravity.

    Specifically, solves for :math:`(\\tau,f)` in the equation:

    .. math::

        G(q;fext) + \\tau_{int} = \\tau + \\sum_i J_i(q)^T f_i
 
    where :math:`G(q;fext)` is the generalized gravity, :math:`\\tau_{int}` are
    the internal torques, and :math:`J_i(q)` is the Jacobian of the i'th
    contact point.  All forces are required to be in their friction cones.

    Args:
        robot (RobotModel): the robot, posed in its current configuration
        holdList (list of Hold): a list of Holds.
        fext (list of 3 floats, optional): the external force (e.g., gravity)
        internalTorques (list, optional): if given, a list of length
            ``robot.numDofs`` giving internal torques. For example, setting
            this to ``robot.accelToTorques(ddq)`` can incorporate dynamics
            into the solver.
        norm (float, optional): the torque norm to minimize.  If 0, minimizes
            the l-infinity norm (default).  If 1, minimizes the l-1 norm.  If
            2, minimizes the l-2 norm (experimental, may not get good results)

    Returns:
        tuple or None: A pair (t,f) giving the joint torques and a list of
        frictional contact forces, if a solution exists. The return value may
        be None if no solution exists.
    """
    links = sum([[h.link]*len(h.contacts) for h in holdList],[])
    if internalTorques is None:
        res = robotsim.equilibrium_torques(robot,_toarray(holdList),links,fext,norm)
    else:
        res = robotsim.equilibrium_torques(robot,_toarray(holdList),links,fext,internalTorques,norm)
    if res is None: return res
    f = res[1]
    return (res[0],[f[i*3:i*3+3] for i in range(len(f)//3)])

def contact_map(contacts,fixed=None):
    """Given an unordered list of ContactPoints, computes a canonical dict
    from (obj1,obj2) pairs to a list of contacts on those objects.
    The resulting dict is also regularized so that objects are sorted in
    increasing getID(), so that (obj1,obj2) is not duplicated as (obj2,obj1).
    
    If fixed is provided, all objects for which fixed(x) returns true will be
    set to None.  The most common example, which fixes terrains, is::
    
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
    if len(worlds)==0: return dict()

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
        reflect = False
        #treat all non RigidObjectModel or RobotModelLink objects as fixed
        o1 = c.object1 if not fixed(c.object1) else None
        o2 = c.object2 if not fixed(c.object2) else None
        if hasattr(o1,'getID'):
            if hasattr(o2,'getID'):
                if o2.getID() < o1.getID():
                    reflect=True
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

def geometry_contacts(geom1,geom2,padding1,padding2=0,maxcontacts=0,kFriction=1):
    """Similar to ``geom1.contacts(geom2,padding1,padding2,maxcontacts)``, but
    returns a list of :class:`ContactPoint`, where the point (x) of each 
    contact is placed in the center of the overlap region.

    The friction coefficient of each contact (kFriction) is set to kFriction.

    Note:

        Contact normals may be set to (0,0,0) if they cannot be computed
        properly, such as when two meshes intersect.

    """
    res = geom1.contacts(geom2,padding1,padding2,maxcontacts)
    cps = []
    for i,(p1,p2,n) in enumerate(zip(res.points1,res.points2,res.normals)):
        x = vectorops.interpolate(p1,p2,0.5)
        n = [v for v in n]
        cps.append(ContactPoint(x,n,kFriction))
        cps[-1].object1 = geom1
        cps[-1].object2 = geom2
    return cps

def world_contact_map(world,padding,kFriction=1,collider=None):
    """Given a WorldModel, returns a contact map representing all current
    contacts (distance >= 0 and <= padding).

    Args:
        world (WorldModel): the world
        padding (float or function): if float, a constant padding, otherwise
            a function f(object) that returns the padding for an object.
        kFriction (float or function, optional): if float, a constant
            friction.  Otherwise, a function f(object1,object2) that returns
            the friction for a pair of objects.
        collider (WorldCollider, optional): if given, only the pairs of
            objects whose collisions are enabled will be checked.

    Note:

        Contact normals may be set to (0,0,0) if they cannot be computed
        properly, such as when two meshes intersect.

    """
    fpadding = padding
    ffriction = kFriction
    if not callable(padding):
        fpadding = lambda obj:padding
    if not callable(kFriction):
        ffriction = lambda obj1,obj2:kFriction
    from .collide import WorldCollider
    if collider is None:
        collider = WorldCollider(world)
    cmap = dict()
    for (i,j) in collider.collisionTests(bb_reject=False):
        obj1,geom1 = i
        obj2,geom2 = j
        pad1 = fpadding(obj1)
        pad2 = fpadding(obj2)
        clist = geometry_contacts(geom1,geom2,pad1,pad2)
        if len(clist) > 0:
            kf = ffriction(obj1,obj2)
            for c in clist:
                c.kFriction = kf
            cmap[(obj1,obj2)] = clist
    return cmap

def sim_contact_map(sim):
    """Given a :class:`Simulation`, returns a contact map representing all
    current contacts (among bodies with collision feedback enabled).
    """
    cmap = dict()
    w = sim.world
    for a in range(w.numIDs()):
        for b in range(a):
            c = sim.getContacts(a,b)
            if len(c) > 0:
                for ci in c:
                    assert len(ci) == 7,"Internal error in Simulation.getContacts()?"
                #figure out the objects corresponding to a and b
                oa = id_to_object(w,a)
                ob = id_to_object(w,b)
                clist = [ContactPoint(ci[0:3],ci[3:6],ci[6]) for ci in c]
                cmap[(oa,ob)] = clist
    return cmap

def contact_map_ik_objectives(contactmap):
    """Given a contact map, computes a set of non-conflicting
    IKObjective's or GeneralizedIKObjective's that enforce all simultaneous
    contact constraints. 
    
    Usually called in conjunction with  :func:`contact_map` with the following
    sequence::

        objectives = contact_map_ik_objectives(contact_map(contacts,lambda x:x==None or isinstance(x,TerrainModel)))

    """
    objectives = []
    for ((o1,o2),clist) in contactmap.items():
        assert o1 != None
        
        x1loc = [o1.getLocalPosition(c.x) for c in clist]
        if o2 is not None and not isinstance(o2,TerrainModel):
            x2loc = [o2.getLocalPosition(c.x) for c in clist]
            objectives.append(ik.objective(o1,o2,local=x1loc,world=x2loc))
        else:
            x2 = [c.x for c in clist]
            objectives.append(ik.objective(o1,local=x1loc,world=x2))
    return objectives

def contact_map_holds(contactmap):
    """Given a contact map, computes a set of non-conflicting
    Holds that enforce all simultaneous contact constraints.
    
    Usually called in conjunction with :func:`contact_map` with the following
    sequence::

        objectives = contact_map_holds(contact_map(contacts,lambda x:x==None or isinstance(x,TerrainModel)))
        
    """
    holds = []
    for ((o1,o2),clist) in contactmap.items():
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

def inv_mass_matrix(obj):
    """Returns the inverse of obj's generalized mass matrix::

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

def wrench_matrices(contactMap):
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


def _deprecated_func(oldName,newName):
    import sys
    mod = sys.modules[__name__]
    f = getattr(mod,newName)
    def depf(*args,**kwargs):
        warnings.warn("{} will be deprecated in favor of {} in a future version of Klampt".format(oldName,newName),DeprecationWarning)
        return f(*args,**kwargs)
    depf.__doc__ = 'Deprecated in a future version of Klampt. Use {} instead'.format(newName)
    setattr(mod,oldName,depf)

_deprecated_func('idToObject','id_to_object')
_deprecated_func('forceClosure','force_closure')
_deprecated_func('comEquilibrium','com_equilibrium')
_deprecated_func('supportPolygon','support_polygon')
_deprecated_func('equilibriumTorques','equilibrium_torques')
_deprecated_func('contactMap','contact_map')
_deprecated_func('geometryContacts','geometry_contacts')
_deprecated_func('worldContactMap','world_contact_map')
_deprecated_func('simContactMap','sim_contact_map')
_deprecated_func('contactMapIKObjectives','contact_map_ik_objectives')
_deprecated_func('contactMapHolds','contact_map_holds')
_deprecated_func('invMassMatrix','inv_mass_matrix')
_deprecated_func('wrenchMatrices','wrench_matrices')
