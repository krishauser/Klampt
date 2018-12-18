from .. import robotsim
from ..model import contact,collide
from ..math import vectorops
from .cspaceutils import CompositeCSpace
from .robotcspace import ClosedLoopRobotCSpace
from .rigidobjectcspace import RigidObjectCSpace

class StanceCSpace(ClosedLoopRobotCSpace):
    """A cspace with contacts that impose closed-chain constraints between a robot and
    zero or more objects.

    Collisions are not checked between the world and fixed robot links, or 
    contacting links that are initially already colliding with the world.

    Members:
    - (inherited from ClosedLoopRobotCSpace) robot, collider, solver, tol, maxIters: 
    - gravity: the gravity vector (default 0,0,-9.8 -- currently cannot accept anything else)
    - holds: a list of Holds associated with the contacts.
    - supportPolygon: the support polygon planes resulting from contact.supportPolygon(holds)
    """
    def __init__(self,robot,holds,collider=None,world=None,checkTorqueLimits=False):
        if collider is None and world is not None:
            ignoreCollisions = [robot.link(h.link) for h in holds]
            collider = collide.WorldCollider(world,ignore=ignoreCollisions)
        ClosedLoopRobotCSpace.__init__(self,robot,[h.ikConstraint for h in holds],collider)
        self.holds = holds
        self.gravity = (0,0,-9.8)

        sp = contact.supportPolygon(holds)
        #print "Support polygon",sp
        self.sp = sp
        self.equilibriumMargin = 0.0
        self.addFeasibilityTest(self.testSupportPolygon,"suppPoly")
        if checkTorqueLimits:
            raise NotImplementedError("Torque limit testing")

    def testSupportPolygon(self,q):
        self.robot.setConfig(q)
        x = self.robot.getCom()
        for plane in self.sp:
            if vectorops.dot(plane[:2],(x[0],x[1])) > plane[2] - self.equilibriumMargin:
                #print "COM",x[:2],"out of support polygon size",len(sp)
                #for plane in sp:
                #   print "  ",vectorops.dot(plane[:2],(x[0],x[1])) - plane[2]
                return False
        return True

class TransitionCSpace(ClosedLoopRobotCSpace):
    """A configuration space for a transition between stances."""
    def __init__(self,space1,space2):
        assert isinstance(space1,StanceCSpace)
        assert isinstance(space2,StanceCSpace)
        if len(space1.holds) > len(space2.holds):
            space1,space2 = space2,space1
        self.space1,self.space2 = space1,space2
        ClosedLoopRobotCSpace.__init__(self,space1.robot,[h.ikConstraint for h in space2.holds],space2.collider)
        self.addFeasibilityTest(space1.testSupportPolygon,"suppPoly")

class MultiContactCSpace(CompositeCSpace):
    """A cspace with contacts that impose closed-chain constraints between a robot and
    zero or more objects.  NOT IMPLEMENTED YET

    Collisions are not checked between the world and fixed robot links, or 
    contacting links that are initially already colliding with the world.

    Members:
    - gravity: the gravity vector (default 0,0,-9.8)
    - contactMap: a dictionary mapping (obj1,obj2) pairs to lists of contacts. Same structures
      that results from contactMap.
    - holds: a list of Holds associated with the contacts.
    - robotCSpace: a ClosedLoopRobotCSpace for the robot
    - objectCSpaces: a list of RigidObjectCSpace's for each moving object
    - movingObjects: a list of moving robots and rigid objects.
    - stabilityTestObjects: a list of moving robots and rigid objects that will be stability tested
    - supportPolygon: a list of supportPolygon planes (result from contact.supportPolygon).
    - supportPolygonVerts: a list of support polygon vertices.
    """
    
    def __init__(self,robot,contacts,stabilityTestObjects=None,collider=None):
        """Initializes the ContactCSpace.  The members contactMap,
        and objectives are constructed from the contacts argument. 

        Arguments:
        - robot: the moving robot
        - contacts: a list of Holds, or ContactPoint objects with the object1 and/or object2 elements
          filled out.
        - stabilityTestObjects argument, if provided, specifies which objects are
          stability tested (either type RobotModel or RigidObjectModel).  Otherwise,
          all objects and free-floating robots in contact are stability tested.
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
            self.holds = contacts
        ikobjectives = [h.ikObjective for h in self.holds]
        self.robotCSpace = ClosedLoopRobotCSpace(robot,ikobjectives,collider)
        self.objectCSpaces = []
        numRobots = 0
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


