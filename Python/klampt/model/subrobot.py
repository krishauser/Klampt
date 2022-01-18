"""Defines the :class:`SubRobotModel` class, which is :class:`RobotModel`-like 
but only modifies selected degrees of freedom of the robot (e.g., an arm, a
leg).

Many, but not all, ``klampt`` module functions accept SubRobotModel in
the place of RobotModel.
"""

from ..robotsim import *
from .collide import self_collision_iter
import warnings
from typing import Union,List,Sequence
from .typing import Config,Vector,Vector3,RigidTransform

class SubRobotModel:
    """A helper that lets you conveniently set/get quantities for a subset
    of moving links on a RobotModel.  This class has the same
    API as RobotModel, but everything is re-indexed so that configurations
    and link indices only modify the given subset of links.  As a reult,
    most methods applicable to a RobotModel can also be applied to
    a SubRobotModel.

    You provide the list of moving link indices or names in the constructor.

    The methods ``tofull`` and ``fromfull`` convert objects to and from the
    full robot.
    """
    def __init__(self, robot : Union[RobotModel,'SubRobotModel'], links : Sequence[Union[int,str]]):
        """
        Args:
            robot (RobotModel or SubRobotModel): the robot to base this on.
            links (list of ints or strs): the links to use in this sub-robot.
        """
        assert isinstance(robot,(RobotModel,SubRobotModel)),"SubRobotModel constructor must be given a RobotModel or SubRobotModel as first argument"
        self._robot = robot
        self._links = links[:]     #type : List[int]
        self._drivers = None       #type : List[RobotModelDriver]
        self.index = robot.index
        self.world = robot.world
        if isinstance(robot,SubRobotModel):
            warnings.warn("Taking sub-robot of sub-robot... not tested yet")
            self._robot = robot._robot
            for i,l in enumerate(self._links):
                if isinstance(l,int):
                    self._links[i] = robot._links[l]
        for i,l in enumerate(self._links):
            if isinstance(l,str):
                self._links[i] = robot.link(l).getIndex()
        self._inv_links = dict((l,i) for (i,l) in enumerate(self._links))

    def tofull(self,object,reference=None):
        """Converts the given index, link, configuration, velocity, or
        trajectory of a sub robot to the corresponding object of the full
        robot. 

        Args:
            object: an integer index, configuration, velocity, matrix, list of 
                configurations, or Trajectory.
            reference (list, optional): describes the reference 
                object that this should fill in for the indices not in this
                sub-robot. By default, uses the robot's current configuration.

        Returns:
            The corresponding object mapped to the full robot.
        """
        if isinstance(object,int):
            return self._links[object]
        elif isinstance(object,SubRobotModelLink):
            return object._link
        elif isinstance(object,(list,tuple)):
            if hasattr(object[0],'__iter__'):
                #treat this as a list of configuration-like objects
                res = []
                if reference is not None:
                    if len(reference) != len(object):
                        if not hasattr(reference[0],'__iter__'):
                            reference = [reference]*len(object)
                        else:
                            raise ValueError("Invalid size of reference object")
                else:
                    reference = [None]*len(object)
                for i,row in enumerate(object):
                    assert len(row) == len(self._links)
                    res.append(self.tofull(row,reference=reference[i]))
                return res
            else:
                assert len(object) == len(self._links)
                if reference is None:
                    res = self._robot.getConfig()
                else:
                    res = [v for v in reference]
                for l,v in zip(self._links,object):
                    res[l] = v
                return res
        else:
            from .trajectory import Trajectory,HermiteTrajectory
            if isinstance(object,Trajectory):
                if isinstance(object,HermiteTrajectory):
                    raise NotImplementedError("Can't lift hermite trajectories to full robots yet")
                newmilestones = self.tofull(object.milestones,reference=reference)
                return object.constructor(object.times,newmilestones)
            else:
                raise ValueError("Invalid object type, not an integer, configuration, or Trajectory")

    def fromfull(self,object):
        """Converts the given index, configuration, velocity, or trajectory of
        a full robot to the corresponding object of the sub-robot. 

        Args:
            object: an integer index, configuration, velocity, matrix, list of
                configurations, or Trajectory.

        Returns:
            The corresponding object mapped to the sub-robot.

        .. note::
            For indices, if the index doesn't belong to the sub-robot then None
            is returned.
        """
        if isinstance(object,int):
            return self._inv_links.get(object,None)
        elif isinstance(object,RobotModelLink):
            return SubRobotModelLink(object,self)
        elif isinstance(object,(list,tuple)):
            if hasattr(object[0],'__iter__'):
                #treat this like a list of configurations
                res = []
                for i,row in enumerate(object):
                    assert len(row) == self._robot.numLinks(),'Object {} needs to be a configuration of length {}'.format(i,self._robot.numLinks())
                    res.append(self.fromfull(row))
                return res
            else:
                #treat as a configuration 
                assert len(object) == self._robot.numLinks(),'Object needs to be a configuration of length {}'.format(self._robot.numLinks())
                return [object[i] for i in self._links]
        else:
            from .trajectory import Trajectory,HermiteTrajectory
            if isinstance(object,Trajectory):
                if isinstance(object,HermiteTrajectory):
                    raise NotImplementedError("Can't project hermite trajectories to sub-robots yet")
                newmilestones = [self.fromfull(v) for v in object.milestones]
                return object.constructor(object.times,newmilestones)
            else:
                raise ValueError("Invalid object type, not an integer, configuration, or Trajectory")

    def numLinks(self) -> int:
        return len(self._links)

    def link(self,index) -> 'SubRobotModelLink':
        if isinstance(index,str):
            return SubRobotModelLink(self._robot.link(index),self)
        else:
            return SubRobotModelLink(self._robot.link(self._links[index]),self)

    def _computeDrivers(self):
        self._drivers = []
        for i in range(self._robot.numDrivers()):
            d = self._robot.driver(i)
            for l in d.getAffectedLinks():
                if l in self._links:
                    self._drivers.append(d)

    def numDrivers(self) -> int:
        if self._drivers is None:
            self._computeDrivers()
        return len(self._drivers)

    def driver(self, index : int) -> 'SubRobotModelDriver':
        if self._drivers is None:
            self._computeDrivers()
        if index < 0 or index >= len(self._drivers):
            raise ValueError("Invalid driver index, must be between {} and {}".format(0,len(self._drivers)-1))
        dindex = self._drivers[index]
        return SubRobotModelDriver(self._robot.driver(index),self)
  
    def getConfig(self) -> Config:
        q = self._robot.getConfig()
        return [q[i] for i in self._links]
    def getVelocity(self) -> Vector:
        q = self._robot.getVelocity()
        return [q[i] for i in self._links]
    def setConfig(self, q : Config) -> None:
        assert len(q) == len(self._links)
        qfull = self._robot.getConfig()
        for i,v in zip(self._links,q):
            qfull[i] = v
        self._robot.setConfig(qfull)
    def setVelocity(self, q:Vector) -> None:
        assert len(q) == len(self._links)
        qfull = self._robot.getVelocity()
        for i,v in zip(self._links,q):
            qfull[i] = v
        self._robot.setVelocity(qfull)
    def getJointLimits(self) -> Tuple[Config,Config]:
        qmin,qmax = self._robot.getJointLimits()
        return [qmin[i] for i in self._links],[qmax[i] for i in self._links]
    def setJointLimits(self, qmin : Config, qmax : Config) -> None:
        assert len(qmin) == len(self._links)
        assert len(qmax) == len(self._links)
        qminfull,qmaxfull = self._robot.getJointLimits()
        for i,a,b in zip(self._links,qmin,qmax):
            qminfull[i] = a
            qmaxfull[i] = b
        self._robot.setJointLimits(qminfull,qmaxfull)
    def getVelocityLimits(self) -> Vector:
        q = self._robot.getVelocityLimits()
        return [q[i] for i in self._links]
    def setVelocityLimits(self, vmax : Vector) -> None:
        assert len(vmax) == len(self._links)
        qfull = self._robot.getVelocityLimits()
        for i,v in zip(self._links,vmax):
            qfull[i] = v
        self._robot.setVelocityLimits(qfull)
    def getAccelerationLimits(self) -> Vector:
        q = self._robot.getAccelerationLimits()
        return [q[i] for i in self._links]
    def setAccelerationLimits(self, amax : Vector) -> None:
        assert len(amax) == len(self._links)
        qfull = self._robot.getAccelerationLimits()
        for i,v in zip(self._links,amax):
            qfull[i] = v
        self._robot.setAccelerationLimits(qfull)
    def getTorqueLimits(self) -> Vector:
        q = self._robot.getTorqueLimits()
        return [q[i] for i in self._links]
    def setTorqueLimits(self, tmax : Vector) -> None:
        assert len(tmax) == len(self._links)
        qfull = self._robot.getTorqueLimits()
        for i,v in zip(self._links,tmax):
            qfull[i] = v
        self._robot.setTorqueLimits(qfull)

    def setDOFPosition(self, index : Union[int,str], qi : float) -> None:
        if isinstance(index,str):
            self._robot.setDOFPosition(index,qi)
        else:
            self._robot.setDOFPosition(self._links[index],qi)
    def getDOFPosition(self, index : Union[int,str]) -> float:
        if isinstance(index,str):
            return self._robot.getDOFPosition(index)
        else:
            return self._robot.getDOFPosition(self._links[index])
    def getCom(self):
        raise NotImplementedError("TODO: getCom")
    def getComJacobian(self):
        raise NotImplementedError("TODO: getComJacobian")
    def getLinearMomentum(self) -> Vector3:
        vinit = self._robot.getVelocity()
        vtemp = self.tofull(self.getVelocity,[0]*self._robot.numLinks())
        self._robot.setVelocity(vtemp)
        res = self._robot.getLinearMomentum()
        self._robot.setVelocity(vinit)
        return res
    def getAngularMomentum(self) -> Vector3:
        vinit = self._robot.getVelocity()
        vtemp = self.tofull(self.getVelocity,[0]*self._robot.numLinks())
        self._robot.setVelocity(vtemp)
        res = self._robot.getAngularMomentum()
        self._robot.setVelocity(vinit)
        return res
    def getKineticEnergy(self) -> float:
        vinit = self._robot.getVelocity()
        vtemp = self.tofull(self.getVelocity,[0]*self._robot.numLinks())
        self._robot.setVelocity(vtemp)
        res = self._robot.getKineticEnergy()
        self._robot.setVelocity(vinit)
        return res
    def getTotalInertia(self):
        raise NotImplementedError("TODO: getTotalInertia")
    def getMassMatrix(self):
        raise NotImplementedError("TODO: getMassMatrix")
    def getMassMatrixInv(self):
        raise NotImplementedError("TODO: getMassMatrix")
    def getCoriolisForceMatrix(self):
        raise NotImplementedError("TODO: getCoriolisForceMatrix")
    def getCoriolisForces(self):
        raise NotImplementedError("TODO: getCoriolisForceMatrix")
    def getGravityForces(self,g):
        raise NotImplementedError("TODO: getGravityForces")
    def torquesFromAccel(self,ddq):
        raise NotImplementedError("TODO: torquesFromAccel")
    def accelFromTorques(self,t):
        raise NotImplementedError("TODO: accelFromTorques")

    def interpolate(self, a : Config, b : Config, u : float) -> Config:
        afull = self.tofull(a)
        bfull = self.tofull(b)
        q = self._robot.interpolate(afull,bfull,u)
        return [q[i] for i in self._links]
    def distance(self, a : Config, b : Config) -> float:
        afull = self.tofull(a)
        bfull = self.tofull(b)
        return self._robot.distance(afull,bfull)
    def interpolateDeriv(self, a : Config, b : Config) -> Vector:
        afull = self.tofull(a)
        bfull = self.tofull(b)
        q = self._robot.interpolateDeriv(afull,bfull)
        return [q[i] for i in self._links]
    def randomizeConfig(self,unboundedScale=1.0) -> None:
        oldfull = self._robot.getConfig()
        self._robot.randomizeConfig(unboundedScale)
        qrand = self._robot.getConfig()
        for i,l in enumerate(self._links):
            oldfull[l] = qrand[l]
        self._robot.setConfig(oldfull)
    def configToDrivers(self, q : Config) -> Config:
        qfull = self.tofull(q)
        dfull = self._robot.configToDrivers(qfull)
        self._computeDrivers()
        dsub = [dfull[d.index] for d in self._drivers]
        return dsub
    def velocityToDrivers(self, dq : Vector) -> Vector:
        dqfull = self.tofull(dq)
        ddfull = self._robot.velocityToDrivers(dqfull)
        self._computeDrivers()
        dsub = [ddfull[d.index] for d in self._drivers]
        return dsub
    def configFromDrivers(self, driverValues : Vector) -> Config:
        self._computeDrivers()
        if len(driverValues) != len(self._drivers): raise ValueError("Invalid # of drivers")
        for (driver,d) in zip(self._drivers,driverValues):
            driver.setValue(d)
        qfull = self._robot.getConfig()
        return self.fromfull(qfull)
    def velocityFromDrivers(self,driverVelocities : Vector) -> Vector:
        self._computeDrivers()
        if len(driverVelocities) != len(self._drivers): raise ValueError("Invalid # of drivers")
        for (driver,dd) in zip(self._drivers,driverVelocities):
            driver.setVelocity(dd)
        dqfull = self._robot.getVelocity()
        return self.fromfull(dqfull)
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
            self._robot.link(i).drawWorldGL(keepAppearance)
    def reduce(self):
        raise NotImplementedError("Can't reduce a sub-robot")
    def mount(self,link,subRobot,R,t):
        self._robot.mount(self.tofull(link),subRobot,R,t)
    def sensor(self,index):
        """Returns the SimSensorModel corresponding to index. Note however that
        you can't set the "link" setting according to this SubRobotModel.

        Args:
            index (int or str)
        """
        if isinstance(index,str):
            return self._robot.sensor(index)
        else:
            return self._robot.sensor(self.tofull(index))


class SubRobotModelLink:
    """A helper that lets you treat links of a subrobot just like a normal
    RobotModelLink. Correctly implements jacobians and indices with respect
    to the sub-robot.
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
        self._robot.fromfull(self._link.getJacobian(p))
    def getPositionJacobian(self,p):
        self._robot.fromfull(self._link.getPositionJacobian(p))
    def getOrientationJacobian(self):
        self._robot.fromfull(self._link.getOrientationJacobian())


class SubRobotModelDriver:
    """A helper that lets you treat drivers of a subrobot just like a normal
    RobotModelDriver.
    """
    def __init__(self,driver,robot):
        self._driver = driver
        self._robot = robot
        self.getName = driver.getName
        self.getType = driver.getType
        self.getAffineCoeffs = driver.getAffineCoeffs
        self.setValue = driver.setValue
        self.getValue = driver.getValue
        self.setVelocity = driver.setVelocity
        self.getVelocity = driver.getVelocity
    
    def robot(self):
        return self._robot
  
    def getAffectedLink(self):
        origLink = self._driver.getAffectedLink()
        return self._robot._inv_links[origLink]

    def getAffectedLinks(self):
        origLinks = self._driver.getAffectedLinks()
        return [self._robot._inv_links[origLink] for origLink in origLinks]
