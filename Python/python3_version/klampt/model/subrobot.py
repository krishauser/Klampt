from ..robotsim import *
from .collide import self_collision_iter
from .trajectory import Trajectory,HermiteTrajectory
import weakref

class SubRobotModel:
    """A helper that lets you conveniently set/get quantities for a subset
    of moving links on a RobotModel.  This class has the same
    API as RobotModel, but everything is re-indexed so that configurations
    and link indices only modify the given subset of links.  As a reult,
    most methods applicable to a RobotModel can also be applied to
    a SubRobotModel.

    You provide the list of moving link indices or names in the constructor.

    The methods tofull and fromfull convert objects to and from the full robot.
    """
    def __init__(self,robot,links):
        assert isinstance(robot,(RobotModel,SubRobotModel)),"SubRobotModel constructor must be given a RobotModel or SubRobotModel as first argument"
        self._robot = robot
        self._links = links[:]
        self.index = robot.index
        self.world = robot.world
        if isinstance(robot,SubRobotModel):
            print("Warning, taking sub-robot of sub-robot... not tested yet")
            input("Press enter to continue...")         
        for i,l in enumerate(self._links):
            if isinstance(l,str):
                self._links[i] = robot.link(l).getIndex()

    def tofull(self,object,type=None):
        """Converts the given index, link, configuration, velocity, or trajectory of a sub robot
        to the corresponding object of the full robot.  Returns the object for the full robot.

        Arguments:
        - object: an integer index, configuration, velocity, matrix, list of configurations,
          or Trajectory.
        - type: if not None, the type of the object.  Used in the case of velocity objects
          where type = 'Vector' or 'velocity' will treat the item like a velocity, or in
          matrices, where type = 'jacobian' will treat the object like a jacobian matrix
          (otherwise it will be treated as a list of configurations).
        """
        if isinstance(object,int):
            return self._links[object]
        elif isinstance(object,SubRobotModelLink):
            return object._link
        elif isinstance(object,(list,tuple)):
            if hasattr(object[0],'__iter__'):
                if type == 'jacobian':
                    #treat this as a jacobian matrix
                    res = []
                    for row in object:
                        assert len(row) == len(self._links)
                        res.append(self.tofull(row,'velocity'))
                    return res
                else:
                    #treat this like a list of configurations
                    res = []
                    for row in object:
                        assert len(row) == len(self._links)
                        res.append(self.tofull(row))
                    return res
            else:
                assert len(object) == len(self._links)
                if type == 'velocity' or type == 'Vector':
                    #treat as a velocity
                    res = [0.0]*self._robot.numLinks()
                    for l,v in zip(self._links,object):
                        res[l] = v
                    return res
                else:
                    #treat as a configuration 
                    res = self._robot.getConfig()
                    for l,v in zip(self._links,object):
                        res[l] = v
                    return res
        elif isinstance(object,Trajectory):
            if isinstance(object,HermiteTrajectory):
                raise NotImplementedError("Can't lift hermite trajectories to full robots yet")
            newmilestones = [self.tofull(v) for v in object.milestones]
            return object.constructor(object.times,newmilestones)
        else:
            raise ValueError("Invalid object type, not an integer, configuration, or Trajectory")

    def fromfull(self,object):
        """Converts the given index, configuration, velocity, or trajectory of a full robot
        to the corresponding object of the sub-robot.  Returns the object for the sub-robot.

        Arguments:
        - object: an integer index, configuration, velocity, matrix, list of configurations,
          or Trajectory.

        Note: for indices, this is an O(n) operation where n is the size of the sub-robot.
        If the index doesn't belong to the sub-robot then None is returned.
        """
        if isinstance(object,int):
            for i,l in enumerate(self._links):
                if l == object:
                    return i
            return None
        elif isinstance(object,RobotModelLink):
            return SubRobotModelLink(object,weakref.byref(self))
        elif isinstance(object,(list,tuple)):
            if hasattr(object[0],'__iter__'):
                #treat this like a list of configurations
                res = []
                for row in object:
                    assert len(row) == self._robot.numLinks()
                    res.append(self.fromfull(row))
                return res
            else:
                #treat as a configuration 
                assert len(object) == self._robot.numLinks()
                return [object[i] for i in self._links]
        elif isinstance(object,Trajectory):
            if isinstance(object,HermiteTrajectory):
                raise NotImplementedError("Can't project hermite trajectories to sub-robots yet")
            newmilestones = [self.fromfull(v) for v in object.milestones]
            return object.constructor(object.times,newmilestones)
        else:
            raise ValueError("Invalid object type, not an integer, configuration, or Trajectory")

    def numLinks(self):
        return len(self._links)

    def link(self,index):
        if isinstance(index,str):
            return SubRobotModelLink(self._robot.link(index),weakref.byref(self))
        else:
            return SubRobotModelLink(self._robot.link(self._links[index]),weakref.byref(self))

    def numDrivers(self):
        raise NotImplementedError("TODO Accessing number of drivers in sub-robot")

    def driver(self,index):
        raise NotImplementedError("TODO Accessing drivers in sub-robot")
  
    def getConfig(self):
        q = self._robot.getConfig()
        return [q[i] for i in self._links]
    def getVelocity(self):
        q = self._robot.getVelocity()
        return [q[i] for i in self._links]
    def setConfig(self,q):
        assert len(q) == len(self._links)
        qfull = self._robot.getConfig()
        for i,v in zip(self._links,q):
            qfull[i] = v
        self._robot.setConfig(qfull)
    def setVelocity(self,q):
        assert len(q) == len(self._links)
        qfull = self._robot.getVelocity()
        for i,v in zip(self._links,q):
            qfull[i] = v
        self._robot.setVelocity(qfull)
    def getJointLimits(self):
        qmin,qmax = self._robot.getJointLimits()
        return [qmin[i] for i in self._links],[qmax[i] for i in self._links]
    def setJointLimits(self,qmin,qmax):
        assert len(qmin) == len(self._links)
        assert len(qmax) == len(self._links)
        qminfull,qmaxfull = self._robot.getJointLimits()
        for i,a,b in zip(self._links,qmin,qmax):
            qminfull[i] = a
            qmaxfull[i] = b
        self._robot.setJointLimits(qminfull,qmaxfull)
    def getVelocityLimits(self):
        q = self._robot.getVelocityLimits()
        return [q[i] for i in self._links]
    def setVelocityLimits(self,vmax):
        assert len(q) == len(self._links)
        qfull = self._robot.getVelocityLimits()
        for i,v in zip(self._links,vmax):
            qfull[i] = v
        self._robot.setVelocityLimits(qfull)
    def getAccelerationLimits(self):
        q = self._robot.getAccelerationLimits()
        return [q[i] for i in self._links]
    def setAccelerationLimits(self,amax):
        assert len(q) == len(self._links)
        qfull = self._robot.getAccelerationLimits()
        for i,v in zip(self._links,amax):
            qfull[i] = v
        self._robot.setAccelerationLimits(qfull)
    def getTorqueLimits(self):
        q = self._robot.getTorqueLimits()
        return [q[i] for i in self._links]
    def setTorqueLimits(self,tmax):
        assert len(q) == len(self._links)
        qfull = self._robot.getTorqueLimits()
        for i,v in zip(self._links,tmax):
            qfull[i] = v
        self._robot.setTorqueLimits(qfull)

    def setDOFPosition(self,index,qi):
        if isinstance(index,str):
            self._robot.setDOFPosition(index,qi)
        else:
            self._robot.setDOFPosition(self._links[index],qi)
    def getDOFPosition(self,index):
        if isinstance(index,str):
            return self._robot.getDOFPosition(index)
        else:
            return self._robot.getDOFPosition(self._links[index])
    def getCom(self):
        raise NotImplementedError("TODO: getCom")
    def getComJacobian(self):
        raise NotImplementedError("TODO: getComJacobian")
    def getMassMatrix(self):
        raise NotImplementedError("TODO: getMassMatrix")
    def getMassMatrixInv(self):
        raise NotImplementedError("TODO: getMassMatrix")
    def getCoriolisForceMatrix(self):
        raise NotImplementedError("TODO: getCoriolisForceMatrix")
    def getCoriolisForces(self):
        raise NotImplementedError("TODO: getCoriolisForceMatrix")
    def getGravityForces(self,g):
        raise NotImplementedError("TODO:  getGravityForces")
    def torquesFromAccel(self,ddq):
        raise NotImplementedError("TODO: torquesFromAccel")
    def accelFromTorques(self,t):
        raise NotImplementedError("TODO: accelFromTorques")

    def interpolate(self,a,b,u):
        afull = self._robot.getConfig()
        bfull = afull[:]
        for i,ai,bi in zip(self._links,a,b):
            afull[i] =ai
            bfull[i] =bi
        q = self._robot.interpolate(afull,bfull,u)
        return [q[i] for i in self._links]
    def distance(self,a,b):
        afull = self._robot.getConfig()
        bfull = afull[:]
        for i,ai,bi in zip(self._links,a,b):
            afull[i] =ai
            bfull[i] =bi
        return self._robot.distance(afull,bfull)
    def interpolate_deriv(self,a,b):
        afull = self._robot.getConfig()
        bfull = afull[:]
        for i,ai,bi in zip(self._links,a,b):
            afull[i] =ai
            bfull[i] =bi
        q = self._robot.interpolate_deriv(afull,bfull)
        return [q[i] for i in self._links]
    def randomizeConfig(self,unboundedScale=1.0):
        oldfull = self._robot.getConfig()
        self._robot.randomizeConfig(unboundedScale)
        qfull = self._robot.getConfig()
        for i,l in enumerate(self._links):
            oldfull[l] = qfull[l]
        self._robot.setConfig(oldfull)
    def selfCollisionEnabled(self,link1,link2):
        if not isinstance(link1,str):
            link1 = self._links[link1]
        else:
            link1 = self._robot.link(link1).getIndex()
        if not isinstance(link2,str):
            link2 = self._links[link2]
        else:
            link2 = self._robot.link(link2).getIndex()
        return self._robot.selfCollisionEnabled(link1,link2)
    def enableSelfCollision(self,link1,link2,value=True):
        if not isinstance(link1,str):
            link1 = self._links[link1]
        else:
            link1 = self._robot.link(link1).getIndex()
        if not isinstance(link2,str):
            link2 = self._links[link2]
        else:
            link2 = self._robot.link(link2).getIndex()
        self._robot.enableSelfCollision(link1,link2,value)
    def selfCollides(self):
        geoms = [self._robot.link(i).geometry() for i in self._links]
        def dotest(i,j):
            return self._robot.selfCollisionEnabled(self._links[i],self._links[j])
        return any(self_collision_iter(geoms,dotest))
    def drawGL(self,keepAppearance=True):
        for i in self._links:
            self._robot.link(i).drawGL(keepAppearance)

class SubRobotModelLink:
    """A helper that lets you treat links of a subrobot just like a normal RobotModelLink.
    Correctly implements jacobians and indices with respect to the sub-robot.
    """
    def __init__(self,link,robot):
        self._link = link
        self._robot = robot
        self.robotIndex = link.robotIndex
        self.index = link.index
        self.getID = link.getID
        self.getName = link.getName
        self.setName = link.setName
        self.geometry = link.geometry
        self.appearance = link.appearance
        self.getMass = link.getMass
        self.setMass = link.setMass
        self.getParentTransform = link.getParentTransform
        self.setParentTransform = link.setParentTransform
        self.getAxis = link.getAxis
        self.setAxis = link.setAxis
        self.getWorldPosition = link.getWorldPosition
        self.getWorldDirection = link.getWorldDirection
        self.getLocalPosition = link.getLocalPosition
        self.getLocalDirection = link.getLocalDirection
        self.getTransform = link.getTransform
        self.setTransform = link.setTransform
        self.getVelocity = link.getVelocity
        self.getAngularVelocity = link.getAngularVelocity
        self.getPointVelocity = link.getPointVelocity
        self.drawLocalGL = link.drawLocalGL
        self.drawWorldGL = link.drawWorldGL
    def robot(self):
        return self._robot
    def getIndex(self):
        return self._robot.fromfull(self._link.getIndex())
    def getParent(self):
        p = self._robot.fromfull(self._link.getParent())
        if p == None: return -1
        return p
    def parent(self):
        p = self.getParent()
        return self._robot.link(p)
    def setParent(self,p):
        self._link.setParent(self._robot.tofull(p))
    def getJacobian(self,p):
        self._robot.tofull(self._link.getJacobian(p),'jacobian')
    def getPositionJacobian(self,p):
        self._robot.tofull(self._link.getPositionJacobian(p),'jacobian')
    def getOrientationJacobian(self):
        self._robot.tofull(self._link.getOrientationJacobian(),'jacobian')

