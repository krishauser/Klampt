from .. import TerrainModel,RobotModel
from ..model import contact,collide
from ..model.contact import ContactPoint,Hold
from ..math import vectorops
from .cspaceutils import CompositeCSpace
from .robotcspace import ClosedLoopRobotCSpace
from .rigidobjectcspace import RigidObjectCSpace

class StanceCSpace(ClosedLoopRobotCSpace):
    """A cspace with contacts that impose closed-chain constraints between a
    robot and zero or more objects.  Stability of the robot is also enforced.

    Collisions are not checked between the world and fixed robot links, or 
    contacting links that are initially already colliding with the world.

    Attributes:
        robot, collider, solver, tol, maxIters: inherited from
            :class:`ClosedLoopRobotCSpace`
        gravity: the gravity vector (default [0,0,-9.8] -- currently cannot
            accept anything else)
        holds (list of :class:`Hold`): a list of Holds associated with the
            contacts.
        sp (list): the support polygon planes resulting from
            ``contact.supportPolygon(holds)``
        equilibriumMargin (float): Can enforce that the COM needs to be within
            the support polygon shrunk by some safety margin.  default value
            is 0.

    """
    def __init__(self,robot,holds,collider=None,world=None,checkTorqueLimits=False):
        if collider is None and world is not None:
            ignoreCollisions = [robot.link(h.link) for h in holds]
            collider = collide.WorldCollider(world,ignore=ignoreCollisions)
        ClosedLoopRobotCSpace.__init__(self,robot,[h.ikConstraint for h in holds],collider)
        self.holds = holds
        self.gravity = (0,0,-9.8)

        sp = contact.supportPolygon(holds)
        self.sp = sp
        self.equilibriumMargin = 0.0
        self.addFeasibilityTest(self.testSupportPolygon,"suppPoly")
        if checkTorqueLimits:
            raise NotImplementedError("Torque limit testing")

    def testSupportPolygon(self,q):
        """Returns True if the robot's COM is in the support polygon at
        configuration q.
        """
        self.robot.setConfig(q)
        x = self.robot.getCom()
        for plane in self.sp:
            if vectorops.dot(plane[:2],(x[0],x[1])) > plane[2] - self.equilibriumMargin:
                #print("COM",x[:2],"out of support polygon size",len(sp))
                #for plane in sp:
                #   print("  ",vectorops.dot(plane[:2],(x[0],x[1])) - plane[2])
                return False
        return True


class TransitionCSpace(ClosedLoopRobotCSpace):
    """A configuration space for a transition between stances.  For this to be
    meaningful, the holds of one stance must be a superset of the holds of the other.

    Args:
        space1 (StanceCSpace): the first stance.
        space2 (StanceCSpace): the second stance.
    """
    def __init__(self,space1,space2):
        assert isinstance(space1,StanceCSpace)
        assert isinstance(space2,StanceCSpace)
        if len(space1.holds) > len(space2.holds):
            space1,space2 = space2,space1
        self.space1,self.space2 = space1,space2
        ClosedLoopRobotCSpace.__init__(self,space1.robot,[h.ikConstraint for h in space2.holds],space2.collider)
        self.addFeasibilityTest(space1.testSupportPolygon,"suppPoly")


class MultiContactCSpace(CompositeCSpace):
    """A cspace with contacts that impose closed-chain constraints between a
    robot and zero or more objects. 

    .. warning::
        NOT IMPLEMENTED YET

    Collisions are not checked between the world and fixed robot links, or 
    contacting links that are initially already colliding with the world.

    Attributes:
        gravity: the gravity vector (default [0,0,-9.8])
        contactMap (dict): a dictionary mapping (obj1,obj2) pairs to lists of
            contacts. Same structures that results from :func:`contactMap`.
        holds (list of :class:`Hold`): a list of Holds associated with the
            contacts.
        robotCSpace (ClosedLoopRobotCSpace): a ClosedLoopRobotCSpace for the
            robot
        objectCSpaces (list of :class:`RigidObjectCSpace`): a list of
            RigidObjectCSpace's for each moving object.
        movingObjects (list): a list of moving robots and rigid objects.
        stabilityTestObjects (list): a list of moving robots and rigid objects
            that will be stability tested.
        supportPolygon (list): a list of supportPolygon planes (result from
            :func:`contact.supportPolygon`).
        supportPolygonVerts (list): a list of support polygon vertices.
    """
    
    def __init__(self,robot,contacts,stabilityTestObjects=None,collider=None):
        """Initializes the ContactCSpace.  The members contactMap,
        and objectives are constructed from the contacts argument. 

        Args:
            robot (RobotModel): the moving robot
            contacts: a list of :class:`Hold`, or :class:`ContactPoint` objects
                with the ``object1`` and/or ``object2`` elements filled out.
            stabilityTestObjects (list, optional): if provided, specifies which
                objects are stability tested (either type :class:`RobotModel`
                or :class:`RigidObjectModel`).  Otherwise, all objects and
                free-floating robots in contact are stability tested.
        """
        self.gravity = (0,0,-9.8)
        
        #construct contact structures
        if len(contacts)==0:
            self.contactMap = dict()
            self.holds = []
        elif isinstance(contacts[0],ContactPoint):
            self.contactMap = contact.contactMap(contacts,lambda x:x is None or isinstance(x,TerrainModel))
            self.holds = contact.contactMapHolds(self.contactMap)
        else:
            self.contactMap = dict()
            self.holds = contacts
        ikobjectives = [h.ikObjective for h in self.holds]
        self.robotCSpace = ClosedLoopRobotCSpace(robot,ikobjectives,collider)
        self.objectCSpaces = []
        numRobots = 0
        objs = set()
        for (obj1,obj2) in self.contactMap():
            if obj1 is not None and not isinstance(obj1,TerrainModel):
                objs.add(obj1)
            if obj2 is not None and not isinstance(obj2,TerrainModel):
                objs.add(obj2)
        for o in objs:
            if isinstance(o,RobotModel):
                numRobots+=1
            else:
                self.objectCSpaces.append(RigidObjectCSpace(o,collider))
        assert numRobots == 1,"ContactCSpace: Can only handle one robot in contact"

        #Check and create vector indexing list
        objs = set()
        for (o1,o2) in self.contactMap.iterKeys():
            if hasattr(o1,'robot'): #it's a RobotModelLink
                objs.insert(o1.robot())
            elif o1 != None:
                objs.insert(o1)
            if hasattr(o2,'robot'): #it's a RobotModelLink
                objs.insert(o2.robot())
            elif o2 != None:
                objs.insert(o2)
        self.movingObjects = list(objs)
        if stabilityTestObjects == None:
            #construct from the contact list
            stabilityTestObjects = []
            for o in objs:
                if isinstance(o,RobotModel):
                    #HACK: need a better test for floating bases
                    if o.numDrivers() + 6 <= o.numLinks():
                        print("ContactCSpace: Robot",o.getName(),"is being treated as a floating-base robot")
                        stabilityTestObjects.append(o)
                    else:
                        print("ContactCSpace: Robot",o.getName(),"is being treated as a fixed-base robot")
                else:
                    stabilityTestObjects.append(o)
        else:
            for o in stabilityTestObjects:
                assert o in objs
                
        self.stabilityTestObjects = stabilityTestObjects
        self.supportPolygon = None
        self.supportPolygonVerts = None

        CompositeCSpace.__init__(self,[self.robotCSpace] + self.objectCSpaces)
        #TODO: add stability tests


