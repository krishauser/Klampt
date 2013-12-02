import robotsim
from contact import *
from cspaceutils import AdaptiveCSpace

class ContactCSpace(AdaptiveCSpace):
    """A cspace with contacts that impose closed-chain constraints."""
    
    def __init__(self,world,contacts,movingObjects=None,collider=None):
        """Initializes the ContactCSpace.  The members solver, contactMap,
        and objectives are constructed from the contacts argument.  The
        movingObjects argument, if provided, specifies which objects are
        allowed to move.  Otherwise, all objects and robots in contact are
        movable."""

        self.collider = collider
        self.world = world

        #construct contact structures
        self.contactMap = contactMap(contacts,lambda(x):x==None or isinstance(x,TerrainModel))
        self.objectives = contactIKObjectives(self.contactMap)
        self.solver = ik.solver(self.objectives)

        #Check and create vector indexing list
        if movingObjects == None:
            #construct from the contact list
            objs = set()
            for (o1,o2) in self.contactMap.iterKeys():
                if hasattr(o1,'getRobot'): #it's a RobotModelLink
                    objs.insert(o1.getRobot())
                elif o1 != None:
                    objs.insert(o1)
                if hasattr(o2,'getRobot'): #it's a RobotModelLink
                    objs.insert(o2.getRobot())
                elif o2 != None:
                    objs.insert(o2)
            movingObjects = list(objs)
        else:
            for (o1,o2) in self.contactMap.iterKeys():
                assert(o1 in movingObjects)
                assert(o2 in movingObjects)
        self.movingObjects = movingObjects
        self.indices = []
        index = 0
        for o in movingObjects:
            n = self.dofs(o)
            self.indices.append((index,index+n))
            index += n

        #IK solve iterations
        self.maxIters = 100
        self.tol = 1e-3

        #adaptive checker
        self.addFeasibleTest(lambda(x): self.closedLoop())

        raise NotImplementedError("NOT DONE YET -- need collision testing and slice indexing")

    def sample():
        self.solver.sampleInitial()
        (res,iters) = self.solver.solve(self.maxIters,self.tol)

    def feasible(self,x):
        if self.adaptive:
            #Use the adaptive tester
            robot.setConfig(x)
            return AdaptiveCSpace.feasible(self,x)
            
        if not self.inJointLimits(x): return False
        robot.setConfig(x)
        if not self.closedLoop(): return False;
        if self.selfCollision(): return False
        if self.envCollision(): return False
        return True

    def closedLoop(self,tol=None):
        e = self.solver.getError()
        if tol==None: tol = self.tol
        return max(abs(ei) for ei in e) <= tol


